/*
 * i.MX93 ChipIdea USB controller driver for osdyne's gadget, resident in BL31
 * (EL3).  The protocol, the console rings and the debug monitor are in
 * plat/imx/common/imx_gadget.c; this moves their bytes over USB OTG1, the
 * port the boot ROM serves serial download on.
 *
 * The controller is the ChipIdea (Synopsys) high-speed OTG core every i.MX
 * since the i.MX27 has had, run as a device.  Its PHY is integrated, and needs
 * nothing beyond its clock and leaving low-power mode.  It interrupts on GIC
 * INTID 219 (SPI 187), which imx_gadget.c makes a Group-0 FIQ to EL3.
 *
 * The controller works from a list of endpoint queue heads (dQH) and the
 * transfer descriptors (dTD) hung off them, both in memory it reads and
 * writes.  They live in a DRAM window kept out of osdyne's way, mapped
 * non-cacheable, so neither side needs cache maintenance.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/cassert.h>
#include <lib/mmio.h>

#include <imx_gadget_controller.h>
#include <platform_def.h>
#include <pwr_ctrl.h>

#include "imx93_chipidea.h"

/* ------------------------------------------------------------------ bases */

#define USB_BASE	U(0x4c100000)	/* USB OTG1 */
#define DMA_MEM_BASE	U(0xfff00000)

/* The controller's clock gate, CCGR_USBC. */
#define LPCG_USBC	104U

/* ------------------------------------------------------- controller regs */

#define USBCMD		0x140
#define USBSTS		0x144
#define USBINTR		0x148
#define DEVICEADDR	0x154
#define ENDPTLISTADDR	0x158
#define PORTSC1		0x184
#define USBMODE		0x1a8
#define ENDPTSETUPSTAT	0x1ac
#define ENDPTPRIME	0x1b0
#define ENDPTFLUSH	0x1b4
#define ENDPTSTAT	0x1b8
#define ENDPTCOMPLETE	0x1bc
#define ENDPTCTRL(n)	(0x1c0 + ((n) * 4))

#define USBCMD_RS		(1u << 0)
#define USBCMD_RST		(1u << 1)
#define USBCMD_SUTW		(1u << 13)
#define USBCMD_ITC_MASK		(0xffu << 16)

/* USBSTS and USBINTR share their layout. */
#define USBSTS_UI		(1u << 0)	/* a transfer completed, or SETUP */
#define USBSTS_UEI		(1u << 1)	/* a transfer ended in error */
#define USBSTS_PCI		(1u << 2)	/* port change: attach, speed */
#define USBSTS_URI		(1u << 6)	/* bus reset */

#define DEVICEADDR_USBADR_SHIFT	25
/* Take the new address once the next IN transaction, the status stage, is done. */
#define DEVICEADDR_USBADRA	(1u << 24)

#define PORTSC1_CCS		(1u << 0)
#define PORTSC1_PHCD		(1u << 23)
#define PORTSC1_PSPD_SHIFT	26
#define PORTSC1_PSPD_MASK	0x3u
#define PORTSC1_PSPD_HIGH	0x2u

#define USBMODE_CM_DEVICE	0x2u
/* Setup lockout off: a SETUP is read under the tripwire instead. */
#define USBMODE_SLOM		(1u << 3)

#define ENDPTCTRL_RXS		(1u << 0)
#define ENDPTCTRL_RXT_BULK	(2u << 2)
#define ENDPTCTRL_RXR		(1u << 6)
#define ENDPTCTRL_RXE		(1u << 7)
#define ENDPTCTRL_TXS		(1u << 16)
#define ENDPTCTRL_TXT_BULK	(2u << 18)
#define ENDPTCTRL_TXR		(1u << 22)
#define ENDPTCTRL_TXE		(1u << 23)

/*
 * ENDPTPRIME, ENDPTFLUSH, ENDPTSTAT and ENDPTCOMPLETE: one bit per endpoint
 * and direction, OUT in the low half and IN in the high one.
 */
#define EP_OUT_BIT(n)	(1u << (n))
#define EP_IN_BIT(n)	(1u << (16 + (n)))

/* ------------------------------------------------------- queue structures */

/* Endpoint queue head.  64 bytes, 64-byte aligned, in a 2 KiB-aligned list. */
struct ci_dqh {
	uint32_t cap;		/* DQH_CAP_* */
	uint32_t current;	/* the dTD being worked on, written by the controller */
	/* The overlay: the controller's working copy of the current dTD. */
	uint32_t next;
	uint32_t token;
	uint32_t buf[5];
	uint32_t reserved;
	uint32_t setup[2];	/* the last SETUP packet, on a control OUT queue */
	uint32_t pad[4];
};

/* Interrupt on SETUP: only on EP0 OUT. */
#define DQH_CAP_IOS		(1u << 15)
#define DQH_CAP_MPL_SHIFT	16
/* Zero-length termination off: a transfer that needs one is sent explicitly. */
#define DQH_CAP_ZLT		(1u << 29)

/* Device transfer descriptor.  32 bytes, 32-byte aligned. */
struct ci_dtd {
	uint32_t next;		/* DTD_TERMINATE when there is none */
	uint32_t token;		/* DTD_* */
	uint32_t buf[5];	/* the buffer's 4 KiB pages; [0] has the offset */
	uint32_t pad;
};

#define DTD_TERMINATE		1u
#define DTD_ACTIVE		(1u << 7)
#define DTD_IOC			(1u << 15)
#define DTD_BYTES_SHIFT		16
#define DTD_BYTES_MASK		0x7fffu

CASSERT(sizeof(struct ci_dqh) == 64U, assert_ci_dqh_size);
CASSERT(sizeof(struct ci_dtd) == 32U, assert_ci_dtd_size);

/*
 * Every queue head the controller may look at: two for each of the up to
 * eight endpoints it has.  Only EP0 and EP1 are ever enabled.
 */
#define QH_COUNT	16U
#define QH_EP0_OUT	0U
#define QH_EP0_IN	1U
#define QH_EP1_OUT	2U
#define QH_EP1_IN	3U

/* EP0's packets are 64 bytes at high and full speed alike. */
#define EP0_MPS		64U
#define EP1_BUF		512U

struct ci_mem {
	struct ci_dqh qh[QH_COUNT];
	struct ci_dtd td[QH_COUNT];	/* one per queue head, as indexed */
	uint8_t ep0_data[IMX_GADGET_EP0_DATA_MAX];
	uint8_t rx_buf[EP1_BUF];
	uint8_t tx_buf[EP1_BUF];
};

CASSERT(sizeof(struct ci_mem) <= 0x10000U, assert_ci_mem_fits_window);

#define MEM ((volatile struct ci_mem *)DMA_MEM_BASE)

/* ------------------------------------------------------ low-level accessors */

static inline uint32_t rd(uint32_t off)
{
	return mmio_read_32(USB_BASE + off);
}

static inline void wr(uint32_t off, uint32_t val)
{
	mmio_write_32(USB_BASE + off, val);
}

/* Wait, briefly, for the bits @mask of register @off to clear. */
static bool wait_clear(uint32_t off, uint32_t mask)
{
	for (unsigned int i = 0U; i < 100000U; i++) {
		if ((rd(off) & mask) == 0U) {
			return true;
		}
	}
	ERROR("chipidea: register 0x%x stuck at 0x%x\n", off, rd(off));
	return false;
}

static uint32_t qh_bit(unsigned int qh)
{
	return ((qh & 1U) != 0U) ? EP_IN_BIT(qh / 2U) : EP_OUT_BIT(qh / 2U);
}

/*
 * Queue one transfer of @len bytes at @buf on queue head @qh, and prime the
 * endpoint.  The endpoint must be idle.
 */
static void prime(unsigned int qh, const volatile uint8_t *buf, uint32_t len)
{
	volatile struct ci_dtd *td = &MEM->td[qh];
	volatile struct ci_dqh *q = &MEM->qh[qh];
	uint32_t addr = (uint32_t)(uintptr_t)buf;

	td->next = DTD_TERMINATE;
	td->token = (len << DTD_BYTES_SHIFT) | DTD_IOC | DTD_ACTIVE;
	td->buf[0] = addr;
	for (unsigned int i = 1U; i < 5U; i++) {
		td->buf[i] = (addr & ~0xfffU) + (i * 0x1000U);
	}

	q->next = (uint32_t)(uintptr_t)td;
	q->token = 0U;

	/* The controller reads both as soon as it is primed. */
	dsb();
	wr(ENDPTPRIME, qh_bit(qh));
}

/* How many bytes of the transfer last primed on @qh were not moved. */
static uint32_t remaining(unsigned int qh)
{
	return (MEM->td[qh].token >> DTD_BYTES_SHIFT) & DTD_BYTES_MASK;
}

/* Abandon whatever is primed on the endpoints in @bits. */
static void flush(uint32_t bits)
{
	do {
		wr(ENDPTFLUSH, bits);
		(void)wait_clear(ENDPTFLUSH, bits);
	} while ((rd(ENDPTSTAT) & bits) != 0U);
	wr(ENDPTCOMPLETE, bits);
}

/* ----------------------------------------------------------------- console */

static bool ep1_in_busy;
static uint32_t ep1_mps = 512U;

void imx_gadget_ctrl_tx_pump(void)
{
	uint32_t n;

	if (ep1_in_busy) {
		return;
	}
	n = imx_gadget_console_take((uint8_t *)MEM->tx_buf, sizeof(MEM->tx_buf));
	if (n == 0U) {
		return;
	}
	prime(QH_EP1_IN, MEM->tx_buf, n);
	ep1_in_busy = true;
}

/* --------------------------------------------------------- control transfers */

enum ep0_state {
	EP0_IDLE,
	EP0_DATA_IN,		/* the reply is on its way */
	EP0_DATA_IN_ZLP,	/* the zero-length packet that ends a short reply */
	EP0_DATA_OUT,		/* waiting for the host's data */
	EP0_STATUS_IN,		/* acknowledging the host's request or data */
	EP0_STATUS_OUT,		/* waiting for the host to acknowledge the reply */
};

static enum ep0_state ep0_state;
static bool ep0_zlp;
/* How much of the host's data the OUT data stage was primed for. */
static uint32_t ep0_out_len;

void imx_gadget_ctrl_set_address(uint8_t addr)
{
	wr(DEVICEADDR, ((uint32_t)addr << DEVICEADDR_USBADR_SHIFT) |
		       DEVICEADDR_USBADRA);
}

void imx_gadget_ctrl_configure(void)
{
	uint32_t cap = (ep1_mps << DQH_CAP_MPL_SHIFT) | DQH_CAP_ZLT;

	MEM->qh[QH_EP1_OUT].cap = cap;
	MEM->qh[QH_EP1_OUT].next = DTD_TERMINATE;
	MEM->qh[QH_EP1_IN].cap = cap;
	MEM->qh[QH_EP1_IN].next = DTD_TERMINATE;
	dsb();

	wr(ENDPTCTRL(1), ENDPTCTRL_RXE | ENDPTCTRL_RXR | ENDPTCTRL_RXT_BULK |
			 ENDPTCTRL_TXE | ENDPTCTRL_TXR | ENDPTCTRL_TXT_BULK);

	prime(QH_EP1_OUT, MEM->rx_buf, sizeof(MEM->rx_buf));
	ep1_in_busy = false;
}

static void handle_setup(void)
{
	uint32_t setup[2];
	uint8_t s[8];
	uint16_t wlen;
	int n;

	/*
	 * Read the packet under the tripwire: a SETUP that lands meanwhile
	 * overwrites the buffer and clears SUTW, and is then read again.
	 */
	wr(ENDPTSETUPSTAT, EP_OUT_BIT(0));
	do {
		wr(USBCMD, rd(USBCMD) | USBCMD_SUTW);
		setup[0] = MEM->qh[QH_EP0_OUT].setup[0];
		setup[1] = MEM->qh[QH_EP0_OUT].setup[1];
	} while ((rd(USBCMD) & USBCMD_SUTW) == 0U);
	wr(USBCMD, rd(USBCMD) & ~USBCMD_SUTW);
	memcpy(s, setup, sizeof(s));

	/* The endpoint cannot be primed for the reply until this clears. */
	(void)wait_clear(ENDPTSETUPSTAT, EP_OUT_BIT(0));

	/* A new SETUP ends whatever the last control transfer left primed. */
	flush(EP_OUT_BIT(0) | EP_IN_BIT(0));

	wlen = s[6] | (s[7] << 8);
	n = imx_gadget_setup(s, (uint8_t *)MEM->ep0_data);

	if (n < 0) {
		/* Both halves; the controller clears them at the next SETUP. */
		wr(ENDPTCTRL(0), rd(ENDPTCTRL(0)) | ENDPTCTRL_RXS | ENDPTCTRL_TXS);
		ep0_state = EP0_IDLE;
	} else if (wlen == 0U) {
		prime(QH_EP0_IN, MEM->ep0_data, 0U);
		ep0_state = EP0_STATUS_IN;
	} else if ((s[0] & 0x80U) != 0U) {
		/*
		 * A reply shorter than asked for, that ends on a packet
		 * boundary, has to be ended with a zero-length packet.  An
		 * empty reply is that packet already.
		 */
		ep0_zlp = (n > 0) && ((uint32_t)n < wlen) &&
			  (((uint32_t)n % EP0_MPS) == 0U);
		prime(QH_EP0_IN, MEM->ep0_data, (uint32_t)n);
		ep0_state = EP0_DATA_IN;
	} else {
		ep0_out_len = (uint32_t)n;
		prime(QH_EP0_OUT, MEM->ep0_data, ep0_out_len);
		ep0_state = EP0_DATA_OUT;
	}
}

static void ep0_in_done(void)
{
	switch (ep0_state) {
	case EP0_DATA_IN:
		if (ep0_zlp) {
			ep0_zlp = false;
			prime(QH_EP0_IN, MEM->ep0_data, 0U);
			ep0_state = EP0_DATA_IN_ZLP;
			break;
		}
		/* Fall through - the reply is all sent. */
	case EP0_DATA_IN_ZLP:
		prime(QH_EP0_OUT, MEM->ep0_data, 0U);
		ep0_state = EP0_STATUS_OUT;
		break;
	case EP0_STATUS_IN:
		ep0_state = EP0_IDLE;
		imx_gadget_ep0_status_done();
		break;
	default:
		break;
	}
}

static void ep0_out_done(void)
{
	switch (ep0_state) {
	case EP0_DATA_OUT:
		imx_gadget_ep0_data_out((const uint8_t *)MEM->ep0_data,
					ep0_out_len - remaining(QH_EP0_OUT));
		prime(QH_EP0_IN, MEM->ep0_data, 0U);
		ep0_state = EP0_STATUS_IN;
		break;
	case EP0_STATUS_OUT:
		ep0_state = EP0_IDLE;
		break;
	default:
		break;
	}
}

/* ------------------------------------------------------------------ events */

static void bus_reset(void)
{
	wr(ENDPTSETUPSTAT, rd(ENDPTSETUPSTAT));
	wr(ENDPTCOMPLETE, rd(ENDPTCOMPLETE));
	(void)wait_clear(ENDPTPRIME, ~0U);
	flush(~0U);

	wr(DEVICEADDR, 0U);
	wr(ENDPTCTRL(1), 0U);

	ep0_state = EP0_IDLE;
	ep0_zlp = false;
	ep1_in_busy = false;

	imx_gadget_bus_reset();
}

static void port_change(void)
{
	uint32_t portsc = rd(PORTSC1);

	if ((portsc & PORTSC1_CCS) == 0U) {
		imx_gadget_bus_disconnected();
		return;
	}

	ep1_mps = (((portsc >> PORTSC1_PSPD_SHIFT) & PORTSC1_PSPD_MASK) ==
		   PORTSC1_PSPD_HIGH) ? 512U : 64U;
}

static void transfers_done(void)
{
	uint32_t done = rd(ENDPTCOMPLETE);

	wr(ENDPTCOMPLETE, done);

	if ((done & EP_OUT_BIT(1)) != 0U) {
		uint32_t got = EP1_BUF - remaining(QH_EP1_OUT);

		if (got > 0U) {
			imx_gadget_console_give((const uint8_t *)MEM->rx_buf, got);
		}
		prime(QH_EP1_OUT, MEM->rx_buf, sizeof(MEM->rx_buf));
	}
	if ((done & EP_IN_BIT(1)) != 0U) {
		ep1_in_busy = false;
		imx_gadget_ctrl_tx_pump();
	}

	if ((done & EP_OUT_BIT(0)) != 0U) {
		ep0_out_done();
	}
	if ((done & EP_IN_BIT(0)) != 0U) {
		ep0_in_done();
	}
}

void imx_gadget_ctrl_service(void)
{
	uint32_t sts = rd(USBSTS) & rd(USBINTR);

	if (sts == 0U) {
		return;
	}
	wr(USBSTS, sts);

	if ((sts & USBSTS_URI) != 0U) {
		bus_reset();
	}
	if ((sts & USBSTS_PCI) != 0U) {
		port_change();
	}
	if ((sts & (USBSTS_UI | USBSTS_UEI)) != 0U) {
		transfers_done();
		if ((rd(ENDPTSETUPSTAT) & EP_OUT_BIT(0)) != 0U) {
			handle_setup();
		}
	}
}

/* ----------------------------------------------------------------- bring-up */

void imx_gadget_ctrl_disconnect(void)
{
	/*
	 * Clearing RS drops the D+ pull-up, so the host sees the device go and
	 * the next stage's serial-download gadget can enumerate after a reset.
	 * Give the host a moment to notice before anything else comes up.
	 * Harmless if the controller was never started.
	 */
	wr(USBCMD, rd(USBCMD) & ~USBCMD_RS);
	mdelay(20);
}

void imx_gadget_ctrl_start(void)
{
	/*
	 * The boot ROM leaves the clock on when it boots over USB; another
	 * boot may not have.  The PHY only needs to leave low-power mode.
	 */
	mmio_write_32(LPCG(LPCG_USBC), 1U);
	wr(PORTSC1, rd(PORTSC1) & ~PORTSC1_PHCD);

	/*
	 * Take the controller over from the ROM's serial-download gadget,
	 * which is still enumerated: drop off the bus so the host forgets it,
	 * then reset the controller to a clean slate.
	 */
	imx_gadget_ctrl_disconnect();
	mdelay(30);
	wr(USBCMD, rd(USBCMD) | USBCMD_RST);
	(void)wait_clear(USBCMD, USBCMD_RST);

	wr(USBMODE, USBMODE_CM_DEVICE | USBMODE_SLOM);

	memset((void *)(uintptr_t)MEM, 0, sizeof(struct ci_mem));
	for (unsigned int i = 0U; i < QH_COUNT; i++) {
		MEM->qh[i].next = DTD_TERMINATE;
	}
	MEM->qh[QH_EP0_OUT].cap = DQH_CAP_IOS | (EP0_MPS << DQH_CAP_MPL_SHIFT) |
				  DQH_CAP_ZLT;
	MEM->qh[QH_EP0_IN].cap = (EP0_MPS << DQH_CAP_MPL_SHIFT) | DQH_CAP_ZLT;
	dsb();
	wr(ENDPTLISTADDR, (uint32_t)(uintptr_t)MEM->qh);

	ep0_state = EP0_IDLE;
	ep0_zlp = false;
	ep1_in_busy = false;

	wr(USBSTS, ~0U);
	wr(USBINTR, USBSTS_UI | USBSTS_UEI | USBSTS_PCI | USBSTS_URI);

	/* Interrupt at once rather than after a few microframes. */
	wr(USBCMD, (rd(USBCMD) & ~USBCMD_ITC_MASK) | USBCMD_RS);

	NOTICE("chipidea(el3): gadget up\n");
}
