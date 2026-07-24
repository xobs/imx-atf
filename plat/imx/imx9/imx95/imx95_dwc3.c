/*
 * i.MX95 DWC3 USB console gadget, resident in BL31 (EL3).
 *
 * Ported from osdyne's EL1 gadget (os/kernel/src/platform/hal/imx95/dwc3.rs).
 * The controller is serviced from a Group-0 secure FIQ (GIC INTID 207 = SPI
 * 175), so it keeps running even while EL1/EL2 (osdyne) is CPU-bound, in an
 * interrupt-disabled section, or hung. This makes the USB console immune to
 * osdyne's scheduling, hides it from osdyne's tests, and lets the board be
 * rebooted over USB even when frozen.
 *
 * DMA memory lives at 0x93300000 (the only mid-DRAM window the DWC3 DMA master
 * can reach; OCRAM and the DRAM tail are unreachable). Mapped cacheable, with
 * explicit clean/invalidate (flush_dcache_range) around every controller-shared
 * access, mirroring the EL1 driver.
 *
 * Milestone M0: enumeration + SET_CONFIGURATION + vendor reboot. The console
 * data path (EL1<->EL3 rings + SMC kick) is added in M1.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <arch.h>
#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>

#include "imx95_dwc3.h"

/* ------------------------------------------------------------------ bases */

#define DWC3_BASE	0x4c100000UL
#define USBPHY_BASE	0x4c1f0040UL
#define DMA_MEM_BASE	0x93300000UL

/* --------------------------------------------------------------- PHY regs */

#define PHY_CTRL0	0x0
#define PHY_CTRL1	0x4
#define PHY_CTRL2	0x8
#define PHY_CTRL6	0x18

#define PHY_CTRL0_REF_SSP_EN		(1u << 2)
#define PHY_CTRL0_FSEL_MASK		(0x3fu << 5)
#define PHY_CTRL0_FSEL_24M		(0x2au << 5)
#define PHY_CTRL1_RESET			(1u << 0)
#define PHY_CTRL1_ATERESET		(1u << 3)
#define PHY_CTRL1_VDATSRCENB0		(1u << 19)
#define PHY_CTRL1_VDATDETENB0		(1u << 20)
#define PHY_CTRL2_TXENABLEN0		(1u << 8)
#define PHY_CTRL2_OTG_DISABLE		(1u << 9)
#define PHY_CTRL6_ALT_CLK_EN		(1u << 1)
#define PHY_CTRL6_ALT_CLK_SEL		(1u << 0)
#define PHY_CTRL6_RXTERM_OVERRIDE_SEL	(1u << 29)

/* -------------------------------------------------------------- DWC3 regs */

#define GSBUSCFG0	0xc100
#define GSBUSCFG1	0xc104
#define GTXFIFOSIZ0	0xc300
#define GRXFIFOSIZ0	0xc380
#define GSNPSID		0xc120
#define GEVNTADRLO	0xc400
#define GEVNTADRHI	0xc404
#define GEVNTSIZ	0xc408
#define GEVNTCOUNT	0xc40c
#define DCFG		0xc700
#define DCTL		0xc704
#define DEVTEN		0xc708
#define DSTS		0xc70c
#define DALEPENA	0xc720
#define GCTL		0xc110
#define GFLADJ		0xc630
#define GUSB2PHYCFG	0xc200
#define GUSB3PIPECTL	0xc2c0

#define DCTL_RUN_STOP	(1u << 31)
#define DCTL_CSFTRST	(1u << 30)
#define GUSB2PHYCFG_SUSPHY	(1u << 6)
#define GUSB2PHYCFG_ENBLSLPM	(1u << 8)
#define DCFG_DEVADDR_MASK	(0x7fu << 3)
#define DSTS_CONNECTSPD		0x7u
#define GEVNTSIZ_INTMASK	(1u << 31)
#define DEVTEN_CONN	((1u << 0) | (1u << 1) | (1u << 2) | (1u << 3) | (1u << 6))

#define DEVT_DISCONNECT	0
#define DEVT_USBRST	1
#define DEVT_CONNECTDONE	2

#define DEPCMD_CMDACT		(1u << 10)
#define DEPCMD_SETEPCONFIG	0x1
#define DEPCMD_SETTRANSFRESOURCE	0x2
#define DEPCMD_SETSTALL		0x4
#define DEPCMD_STARTTRANSFER	0x6
#define DEPCMD_ENDTRANSFER	0x8
#define DEPCMD_STARTCFG		0x9
#define DEPCMD_HIPRI_FORCERM	(1u << 11)
#define DEPCFG_XFER_COMPLETE_EN	(1u << 8)
#define DEPCFG_XFER_NOT_READY_EN	(1u << 10)
#define DEPCFG_IGN_SEQ_NUM	(1u << 31)
#define DEPEVT_XFERCOMPLETE	1
#define DEPEVT_XFERNOTREADY	3
#define DEPEVT_STATUS_CONTROL_DATA	1
#define DEPEVT_STATUS_CONTROL_STATUS	2

#define EP0_OUT	0
#define EP0_IN	1
#define EP1_OUT	2
#define EP1_IN	3
#define EP_TYPE_CONTROL	0
#define EP_TYPE_BULK	2

#define TRB_HWO		(1u << 0)
#define TRB_LST		(1u << 1)
#define TRB_ISP_IMI	(1u << 10)
#define TRB_IOC		(1u << 11)
#define TRBCTL_NORMAL		(1u << 4)
#define TRBCTL_CONTROL_SETUP	(2u << 4)
#define TRBCTL_CONTROL_STATUS_2	(3u << 4)
#define TRBCTL_CONTROL_STATUS_3	(4u << 4)
#define TRBCTL_CONTROL_DATA	(5u << 4)

#define REQ_GET_STATUS		0x00
#define REQ_CLEAR_FEATURE	0x01
#define REQ_SET_FEATURE		0x03
#define REQ_SET_ADDRESS		0x05
#define REQ_GET_DESCRIPTOR	0x06
#define REQ_GET_CONFIGURATION	0x08
#define REQ_SET_CONFIGURATION	0x09
#define DESC_DEVICE		1
#define DESC_CONFIG		2
#define DESC_STRING		3
#define DESC_DEVICE_QUALIFIER	6
#define DESC_OTHER_SPEED_CONFIG	7

#define MS_VENDOR_CODE		0x1a
#define MS_OS_STRING_INDEX	0xee
#define VENDOR_REQ_REBOOT	0x01
#define REBOOT_MAGIC		0x5253

#define EVENT_COUNT	64
#define EVENT_BYTES	(EVENT_COUNT * 4)

/* -------------------------------------------------------------- descriptors */

static const uint8_t DEVICE_DESC[18] = {
	0x12, DESC_DEVICE, 0x00, 0x02, 0xff, 0x00, 0x00, 0x40,
	0xc9, 0x1f, 0x5d, 0x01, 0x00, 0x01, 0x01, 0x02, 0x03, 0x01,
};

static const uint8_t CONFIG_DESC[32] = {
	0x09, DESC_CONFIG, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
	0x09, 0x04, 0x00, 0x00, 0x02, 0xff, 0xff, 0xff, 0x00,
	0x07, 0x05, 0x01, 0x02, 0x00, 0x02, 0x00,
	0x07, 0x05, 0x81, 0x02, 0x00, 0x02, 0x00,
};

static const uint8_t OTHER_SPEED_CONFIG_DESC[32] = {
	0x09, DESC_OTHER_SPEED_CONFIG, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
	0x09, 0x04, 0x00, 0x00, 0x02, 0xff, 0xff, 0xff, 0x00,
	0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00,
	0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,
};

static const uint8_t DEVICE_QUALIFIER_DESC[10] = {
	0x0a, DESC_DEVICE_QUALIFIER, 0x00, 0x02, 0xff, 0x00, 0x00, 0x40, 0x01, 0x00,
};

static const uint8_t STR_LANGID[4] = { 0x04, DESC_STRING, 0x09, 0x04 };
static const uint8_t STR_MANUFACTURER[14] = {
	0x0e, DESC_STRING, 'o', 0, 's', 0, 'd', 0, 'y', 0, 'n', 0, 'e', 0,
};
static const uint8_t STR_PRODUCT[30] = {
	0x1e, DESC_STRING, 'o', 0, 's', 0, 'd', 0, 'y', 0, 'n', 0, 'e', 0,
	' ', 0, 'c', 0, 'o', 0, 'n', 0, 's', 0, 'o', 0, 'l', 0, 'e', 0,
};
static const uint8_t STR_SERIAL[10] = {
	0x0a, DESC_STRING, '0', 0, '0', 0, '0', 0, '1', 0,
};
static const uint8_t MS_OS_STRING[18] = {
	0x12, DESC_STRING, 'M', 0, 'S', 0, 'F', 0, 'T', 0,
	'1', 0, '0', 0, '0', 0, MS_VENDOR_CODE, 0x00,
};
static const uint8_t MS_COMPAT_ID[40] = {
	0x28, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x01,
	0, 0, 0, 0, 0, 0, 0, 0x00, 0x01,
	'W', 'I', 'N', 'U', 'S', 'B', 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

/* ------------------------------------------------------------------- memory */

struct dwc3_mem {
	uint32_t event_buf[EVENT_COUNT];
	uint32_t ep0_setup_trb[4];
	uint32_t ep0_data_trb[4];
	uint32_t ep0_status_trb[4];
	uint32_t ep1out_trb[4];
	uint32_t ep1in_trb[4];
	uint8_t setup_buf[8];
	uint8_t ep0_data[512];
	uint8_t rx_buf[512];
	uint8_t tx_buf[512];
};

#define MEM ((volatile struct dwc3_mem *)DMA_MEM_BASE)

/*
 * EL1<->EL3 console channel: a pair of SPSC byte rings in a fixed non-cacheable
 * DRAM window shared with osdyne (EL1). osdyne writes console output into tx_buf
 * and reads host input from rx_buf; BL31 drains tx_buf to EP1 IN (driven by the
 * host's IN polling via XferNotReady) and fills rx_buf from EP1 OUT. Non-cacheable
 * so no cross-EL cache maintenance is needed. Ring sizes are powers of two.
 */
#define CONSOLE_CHAN_BASE	0x93380000UL
#define CONSOLE_CHAN_MAGIC	0x6f64636eu /* "ncdo" */
#define TX_RING_SZ	4096
#define RX_RING_SZ	512

struct console_chan {
	volatile uint32_t magic;
	volatile uint32_t configured;
	volatile uint32_t tx_head;	/* producer: EL1 */
	volatile uint32_t tx_tail;	/* consumer: EL3 */
	volatile uint32_t rx_head;	/* producer: EL3 */
	volatile uint32_t rx_tail;	/* consumer: EL1 */
	volatile uint8_t tx_buf[TX_RING_SZ];
	volatile uint8_t rx_buf[RX_RING_SZ];
};

#define CHAN ((volatile struct console_chan *)CONSOLE_CHAN_BASE)

/* ------------------------------------------------------ low-level accessors */

static inline uint32_t rd(uint32_t off)
{
	return mmio_read_32(DWC3_BASE + off);
}
static inline void wr(uint32_t off, uint32_t val)
{
	mmio_write_32(DWC3_BASE + off, val);
}
static inline uint32_t phy_rd(uint32_t off)
{
	return mmio_read_32(USBPHY_BASE + off);
}
static inline void phy_wr(uint32_t off, uint32_t val)
{
	mmio_write_32(USBPHY_BASE + off, val);
}

/* Clean+invalidate a DMA range to the point of coherency (== EL1 dc civac). */
static void dcache_flush(uintptr_t addr, size_t len)
{
	flush_dcache_range(addr, len);
}

static void write_trb(volatile uint32_t *trb, uint32_t w0, uint32_t w1,
		      uint32_t w2, uint32_t w3)
{
	trb[0] = w0;
	trb[1] = w1;
	trb[2] = w2;
	dsb();
	trb[3] = w3; /* HWO last */
	dsb();
	dcache_flush((uintptr_t)trb, 16);
}

/* Issue a DEPCMD on physical endpoint `ep`, wait for completion. */
static uint32_t depcmd(unsigned int ep, uint32_t cmd, uint32_t par0, uint32_t par1)
{
	uint32_t base = 0xc800 + ep * 0x10;
	uint32_t spins = 0;

	wr(base, 0);		/* PAR2 */
	wr(base + 0x4, par1);	/* PAR1 */
	wr(base + 0x8, par0);	/* PAR0 */
	dsb();
	wr(base + 0xc, cmd | DEPCMD_CMDACT);
	while (rd(base + 0xc) & DEPCMD_CMDACT) {
		if (++spins > 100000) {
			ERROR("dwc3: DEPCMD timeout ep%u cmd=%#x\n", ep, cmd);
			break;
		}
	}
	return rd(base + 0xc);
}

static uint32_t xfer_rsc(uint32_t depcmd_result)
{
	return (depcmd_result >> 16) & 0x7f;
}

static uint32_t start_xfer(unsigned int ep, uint32_t trb_addr)
{
	return depcmd(ep, DEPCMD_STARTTRANSFER, 0, trb_addr);
}

static void configure_ep(unsigned int phys_ep, uint32_t ep_type, uint32_t mps)
{
	bool dir_in = (phys_ep & 1) == 1;
	uint32_t fifo = dir_in ? (phys_ep >> 1) : 0;
	uint32_t par0 = (ep_type << 1) | (mps << 3) | (fifo << 17);
	uint32_t par1 = DEPCFG_XFER_COMPLETE_EN | DEPCFG_XFER_NOT_READY_EN |
			(phys_ep << 25);

	depcmd(phys_ep, DEPCMD_SETEPCONFIG, par0, par1);
	depcmd(phys_ep, DEPCMD_SETTRANSFRESOURCE, 1, 0);
	wr(DALEPENA, rd(DALEPENA) | (1u << phys_ep));
}

static void reconfigure_ep0(uint32_t mps)
{
	unsigned int eps[2] = { EP0_OUT, EP0_IN };
	for (int i = 0; i < 2; i++) {
		unsigned int phys_ep = eps[i];
		uint32_t par0 = (EP_TYPE_CONTROL << 1) | (mps << 3) | DEPCFG_IGN_SEQ_NUM;
		uint32_t par1 = DEPCFG_XFER_COMPLETE_EN | DEPCFG_XFER_NOT_READY_EN |
				(phys_ep << 25);
		depcmd(phys_ep, DEPCMD_SETEPCONFIG, par0, par1);
	}
}

static void end_xfer(unsigned int ep, uint32_t rsc)
{
	depcmd(ep, DEPCMD_ENDTRANSFER | DEPCMD_HIPRI_FORCERM | ((rsc & 0x7f) << 16),
	       0, 0);
}

static uint32_t post_ep0_setup(void)
{
	uint32_t addr = (uint32_t)(uintptr_t)MEM->ep0_setup_trb;
	write_trb(MEM->ep0_setup_trb, (uint32_t)(uintptr_t)MEM->setup_buf, 0,
		  sizeof(MEM->setup_buf),
		  TRB_HWO | TRB_LST | TRBCTL_CONTROL_SETUP | TRB_ISP_IMI | TRB_IOC);
	return start_xfer(EP0_OUT, addr);
}

static uint32_t post_ep0_data(bool dir_in, uint32_t len)
{
	uint32_t addr = (uint32_t)(uintptr_t)MEM->ep0_data_trb;
	if (dir_in && len > 0)
		dcache_flush((uintptr_t)MEM->ep0_data, len);
	write_trb(MEM->ep0_data_trb, (uint32_t)(uintptr_t)MEM->ep0_data, 0, len,
		  TRB_HWO | TRB_LST | TRBCTL_CONTROL_DATA | TRB_ISP_IMI | TRB_IOC);
	return start_xfer(dir_in ? EP0_IN : EP0_OUT, addr);
}

static uint32_t post_ep0_status(bool dir_in, bool had_data)
{
	uint32_t addr = (uint32_t)(uintptr_t)MEM->ep0_status_trb;
	uint32_t ctl = had_data ? TRBCTL_CONTROL_STATUS_3 : TRBCTL_CONTROL_STATUS_2;
	write_trb(MEM->ep0_status_trb, (uint32_t)(uintptr_t)MEM->ep0_data, 0, 0,
		  TRB_HWO | TRB_LST | ctl | TRB_ISP_IMI | TRB_IOC);
	return start_xfer(dir_in ? EP0_IN : EP0_OUT, addr);
}

static void stall_ep0(void)
{
	depcmd(EP0_OUT, DEPCMD_SETSTALL, 0, 0);
}

static uint32_t post_ep1_out(void)
{
	uint32_t addr = (uint32_t)(uintptr_t)MEM->ep1out_trb;
	write_trb(MEM->ep1out_trb, (uint32_t)(uintptr_t)MEM->rx_buf, 0,
		  sizeof(MEM->rx_buf), TRB_HWO | TRB_LST | TRBCTL_NORMAL | TRB_IOC);
	return start_xfer(EP1_OUT, addr);
}

static uint32_t post_ep1_in(uint32_t len)
{
	uint32_t addr = (uint32_t)(uintptr_t)MEM->ep1in_trb;
	write_trb(MEM->ep1in_trb, (uint32_t)(uintptr_t)MEM->tx_buf, 0, len,
		  TRB_HWO | TRB_LST | TRBCTL_NORMAL | TRB_IOC);
	return start_xfer(EP1_IN, addr);
}

static bool ep1_in_busy;

/*
 * Drain the EL1->EL3 console TX ring into tx_buf and kick an EP1 IN transfer.
 * Called on EP1 IN XferComplete/XferNotReady (the host's polling drives it).
 */
static void dwc3_tx_pump(void)
{
	uint32_t n = 0;

	if (!CHAN->configured || ep1_in_busy)
		return;
	while (n < sizeof(MEM->tx_buf) && CHAN->tx_tail != CHAN->tx_head) {
		MEM->tx_buf[n++] = CHAN->tx_buf[CHAN->tx_tail & (TX_RING_SZ - 1)];
		CHAN->tx_tail = CHAN->tx_tail + 1;
	}
	if (n == 0)
		return;
	dcache_flush((uintptr_t)MEM->tx_buf, n);
	post_ep1_in(n);
	ep1_in_busy = true;
}

/* ------------------------------------------------------------ PHY + bring-up */

static void phy_init(void)
{
	uint32_t v;

	v = phy_rd(PHY_CTRL0);
	v &= ~PHY_CTRL0_FSEL_MASK;
	v |= PHY_CTRL0_FSEL_24M;
	phy_wr(PHY_CTRL0, v);

	v = phy_rd(PHY_CTRL6);
	v &= ~(PHY_CTRL6_ALT_CLK_SEL | PHY_CTRL6_ALT_CLK_EN);
	phy_wr(PHY_CTRL6, v);

	v = phy_rd(PHY_CTRL1);
	v &= ~(PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0);
	v |= PHY_CTRL1_RESET | PHY_CTRL1_ATERESET;
	phy_wr(PHY_CTRL1, v);

	v = phy_rd(PHY_CTRL0);
	v |= PHY_CTRL0_REF_SSP_EN;
	phy_wr(PHY_CTRL0, v);

	v = phy_rd(PHY_CTRL2);
	v |= PHY_CTRL2_TXENABLEN0 | PHY_CTRL2_OTG_DISABLE;
	phy_wr(PHY_CTRL2, v);

	mdelay(1);

	v = phy_rd(PHY_CTRL1);
	v &= ~(PHY_CTRL1_RESET | PHY_CTRL1_ATERESET);
	phy_wr(PHY_CTRL1, v);

	v = phy_rd(PHY_CTRL6);
	v &= ~PHY_CTRL6_RXTERM_OVERRIDE_SEL;
	phy_wr(PHY_CTRL6, v);
}

static void global_init(void)
{
	wr(GCTL, 0x00112004);
	wr(GSBUSCFG0, 0x00000001);
	wr(GSBUSCFG1, 0x00000300);
	wr(GFLADJ, 0x00800000);
	wr(GUSB3PIPECTL, 0x010e0002);
}

/* State machine, persistent across FIQ invocations. */
enum ep0_state {
	EP0_STATE_SETUP,
	EP0_STATE_DATA,
	EP0_STATE_STATUS_PENDING,
	EP0_STATE_STATUS,
};

static enum ep0_state ep0_state;
static bool ep0_dir_in;
static bool ep0_three_stage;
static uint32_t ep0_resp_len;
static bool ep_busy[2];
static uint32_t ep_rsc[2];
static bool config_pending;
static bool reboot_pending;
static bool configured;
static unsigned int evt_idx;

static void set_address(uint16_t addr)
{
	wr(DCFG, (rd(DCFG) & ~DCFG_DEVADDR_MASK) | (((uint32_t)addr & 0x7f) << 3));
}

static void set_configuration(void)
{
	depcmd(EP0_OUT, DEPCMD_STARTCFG | (2 << 16), 0, 0);
	configure_ep(EP1_OUT, EP_TYPE_BULK, 512);
	configure_ep(EP1_IN, EP_TYPE_BULK, 512);
	post_ep1_out();
	configured = true;
	ep1_in_busy = false;
	CHAN->configured = 1;
}

static void gadget_start(void)
{
	uint32_t residual;

	global_init();

	/* Force a clean disconnect edge before presenting the gadget. */
	wr(DCTL, rd(DCTL) & ~DCTL_RUN_STOP);
	mdelay(300);

	phy_init();

	/* Device-controller soft reset. */
	wr(DCTL, DCTL_CSFTRST);
	for (int i = 0; i < 5000000; i++) {
		if ((rd(DCTL) & DCTL_CSFTRST) == 0)
			break;
	}
	mdelay(50);

	/* Keep the PHY awake so pre-enumeration EP0 arming lands. */
	wr(GUSB2PHYCFG,
	   rd(GUSB2PHYCFG) & ~GUSB2PHYCFG_SUSPHY & ~GUSB2PHYCFG_ENBLSLPM);

	wr(GEVNTADRLO, (uint32_t)DMA_MEM_BASE);
	wr(GEVNTADRHI, 0);
	wr(GEVNTSIZ, GEVNTSIZ_INTMASK | EVENT_BYTES);
	residual = rd(GEVNTCOUNT) & 0xffff;
	if (residual != 0)
		wr(GEVNTCOUNT, residual);
	wr(DEVTEN, DEVTEN_CONN);

	wr(DCFG, rd(DCFG) & ~DCFG_DEVADDR_MASK & ~0x7u);

	wr(DALEPENA, 0);
	depcmd(EP0_OUT, DEPCMD_STARTCFG, 0, 0);
	configure_ep(EP0_OUT, EP_TYPE_CONTROL, 64);
	configure_ep(EP0_IN, EP_TYPE_CONTROL, 64);
	ep_rsc[EP0_OUT] = xfer_rsc(post_ep0_setup());
	ep_busy[EP0_OUT] = true;

	wr(DCTL, rd(DCTL) | DCTL_RUN_STOP);

	/*
	 * Unmask interrupter-0 now that the EL3 FIQ handler is registered (it is
	 * set up before imx95_dwc3_gadget_init() runs). Clears GEVNTSIZ_INTMASK
	 * so event-ring writes assert the GIC line -> FIQ 207.
	 */
	wr(GEVNTSIZ, EVENT_BYTES);

	NOTICE("dwc3(el3): gadget up, gsnpsid=%#x dcfg=%#x dalepena=%#x\n",
	       rd(GSNPSID), rd(DCFG), rd(DALEPENA));
}

/* --------------------------------------------------------------- handle_setup */

/* Stage a device->host reply into ep0_data; return bytes to send. */
static uint32_t stage_reply(const uint8_t *src, size_t src_len, uint16_t wlen)
{
	size_t n = src_len;
	if (n > wlen)
		n = wlen;
	if (n > sizeof(MEM->ep0_data))
		n = sizeof(MEM->ep0_data);
	for (size_t i = 0; i < n; i++)
		MEM->ep0_data[i] = src[i];
	dcache_flush((uintptr_t)MEM->ep0_data, n ? n : 1);
	return n;
}

/*
 * Decode a SETUP packet. Returns >= 0 for the data/no-data length (staged in
 * ep0_data), or -1 to STALL.
 */
static int handle_setup(uint8_t bmreq, uint8_t brequest, uint16_t wvalue,
			uint16_t windex, uint16_t wlen)
{
	uint32_t req_type = bmreq & 0x60;

	if (req_type == 0x00) {
		switch (brequest) {
		case REQ_GET_DESCRIPTOR: {
			uint8_t dtype = wvalue >> 8;
			uint8_t didx = wvalue & 0xff;
			switch (dtype) {
			case DESC_DEVICE:
				return stage_reply(DEVICE_DESC, sizeof(DEVICE_DESC), wlen);
			case DESC_DEVICE_QUALIFIER:
				return stage_reply(DEVICE_QUALIFIER_DESC,
						   sizeof(DEVICE_QUALIFIER_DESC), wlen);
			case DESC_OTHER_SPEED_CONFIG:
				return stage_reply(OTHER_SPEED_CONFIG_DESC,
						   sizeof(OTHER_SPEED_CONFIG_DESC), wlen);
			case DESC_CONFIG:
				return stage_reply(CONFIG_DESC, sizeof(CONFIG_DESC), wlen);
			case DESC_STRING:
				switch (didx) {
				case 0:
					return stage_reply(STR_LANGID, sizeof(STR_LANGID), wlen);
				case 1:
					return stage_reply(STR_MANUFACTURER,
							   sizeof(STR_MANUFACTURER), wlen);
				case 2:
					return stage_reply(STR_PRODUCT, sizeof(STR_PRODUCT), wlen);
				case 3:
					return stage_reply(STR_SERIAL, sizeof(STR_SERIAL), wlen);
				case MS_OS_STRING_INDEX:
					return stage_reply(MS_OS_STRING, sizeof(MS_OS_STRING), wlen);
				default:
					return -1;
				}
			default:
				return -1;
			}
		}
		case REQ_SET_ADDRESS:
			set_address(wvalue);
			return 0;
		case REQ_SET_CONFIGURATION:
			if (wvalue == 0)
				configured = false;
			return 0;
		case REQ_GET_CONFIGURATION:
			MEM->ep0_data[0] = configured ? 1 : 0;
			dcache_flush((uintptr_t)MEM->ep0_data, 1);
			return wlen < 1 ? wlen : 1;
		case REQ_GET_STATUS:
			MEM->ep0_data[0] = 0;
			MEM->ep0_data[1] = 0;
			dcache_flush((uintptr_t)MEM->ep0_data, 2);
			return wlen < 2 ? wlen : 2;
		case REQ_SET_FEATURE:
		case REQ_CLEAR_FEATURE:
			return 0;
		default:
			return -1;
		}
	} else if (req_type == 0x40) {
		if (brequest == MS_VENDOR_CODE && windex == 0x0004)
			return stage_reply(MS_COMPAT_ID, sizeof(MS_COMPAT_ID), wlen);
		if (brequest == VENDOR_REQ_REBOOT && wvalue == REBOOT_MAGIC)
			return 0;
		return -1;
	}
	return -1;
}

/* --------------------------------------------------------- event processing */

static void handle_ep0_xfercomplete(unsigned int ep)
{
	unsigned int epi = ep;

	ep_busy[epi] = false;
	ep_rsc[epi] = 0;

	if (ep0_state == EP0_STATE_SETUP && ep == EP0_OUT) {
		uint8_t s[8];
		uint8_t bmreq, brequest;
		uint16_t wvalue, windex, wlen;
		int n;

		dcache_flush((uintptr_t)MEM->setup_buf, 8);
		for (int i = 0; i < 8; i++)
			s[i] = MEM->setup_buf[i];
		bmreq = s[0];
		brequest = s[1];
		wvalue = s[2] | (s[3] << 8);
		windex = s[4] | (s[5] << 8);
		wlen = s[6] | (s[7] << 8);
		ep0_dir_in = (bmreq & 0x80) != 0;
		ep0_three_stage = wlen != 0;
		/* New SETUP supersedes any stale EP0_IN in-flight state. */
		ep_busy[EP0_IN] = false;
		ep_rsc[EP0_IN] = 0;

		n = handle_setup(bmreq, brequest, wvalue, windex, wlen);
		if (n < 0) {
			stall_ep0();
			ep0_resp_len = 0;
			ep0_three_stage = false;
			if (!ep_busy[EP0_OUT]) {
				ep_rsc[EP0_OUT] = xfer_rsc(post_ep0_setup());
				ep_busy[EP0_OUT] = true;
			}
			ep0_state = EP0_STATE_SETUP;
		} else if (ep0_three_stage) {
			unsigned int data_ep = ep0_dir_in ? EP0_IN : EP0_OUT;
			uint32_t dn = ep0_dir_in ? (uint32_t)n : 512;
			ep0_resp_len = n;
			if (!ep_busy[data_ep]) {
				ep_rsc[data_ep] = xfer_rsc(post_ep0_data(ep0_dir_in, dn));
				ep_busy[data_ep] = true;
			}
			ep0_state = EP0_STATE_DATA;
		} else {
			if (brequest == REQ_SET_CONFIGURATION && wvalue != 0)
				config_pending = true;
			if (bmreq == 0x40 && brequest == VENDOR_REQ_REBOOT &&
			    wvalue == REBOOT_MAGIC)
				reboot_pending = true;
			ep0_state = EP0_STATE_STATUS_PENDING;
		}
	} else if (ep0_state == EP0_STATE_DATA) {
		ep0_state = EP0_STATE_STATUS_PENDING;
	} else if (ep0_state == EP0_STATE_STATUS) {
		if (reboot_pending)
			imx95_dwc3_do_reboot(); /* never returns */
		if (config_pending) {
			set_configuration();
			config_pending = false;
		}
		if (!ep_busy[EP0_OUT]) {
			ep_rsc[EP0_OUT] = xfer_rsc(post_ep0_setup());
			ep_busy[EP0_OUT] = true;
		}
		ep0_state = EP0_STATE_SETUP;
	}
}

static void handle_ep0_xfernotready(unsigned int ep, uint32_t estatus)
{
	uint32_t phase = estatus & 0x3;

	if (phase == DEPEVT_STATUS_CONTROL_DATA && ep0_state == EP0_STATE_SETUP) {
		unsigned int data_ep = ep0_dir_in ? EP0_IN : EP0_OUT;
		uint32_t n = ep0_dir_in ? ep0_resp_len : 512;
		if (!ep_busy[data_ep]) {
			ep_rsc[data_ep] = xfer_rsc(post_ep0_data(ep0_dir_in, n));
			ep_busy[data_ep] = true;
		}
		ep0_state = EP0_STATE_DATA;
	} else if (phase == DEPEVT_STATUS_CONTROL_STATUS &&
		   ep0_state == EP0_STATE_STATUS_PENDING) {
		bool status_in = (ep == EP0_IN);
		if (!ep_busy[ep]) {
			ep_rsc[ep] = xfer_rsc(post_ep0_status(status_in, ep0_three_stage));
			ep_busy[ep] = true;
		}
		ep0_state = EP0_STATE_STATUS;
	}
}

static void process_event(uint32_t e)
{
	if (e & 1) {
		/* Device event. */
		switch ((e >> 8) & 0xf) {
		case DEVT_DISCONNECT:
			break;
		case DEVT_USBRST:
			set_address(0);
			ep0_state = EP0_STATE_SETUP;
			configured = false;
			ep1_in_busy = false;
			CHAN->configured = 0;
			break;
		case DEVT_CONNECTDONE: {
			uint32_t spd = rd(DSTS) & DSTS_CONNECTSPD;
			uint32_t mps = (spd == 2) ? 8 : 64;
			reconfigure_ep0(mps);
			end_xfer(EP0_OUT, ep_rsc[EP0_OUT]);
			ep_rsc[EP0_OUT] = xfer_rsc(post_ep0_setup());
			ep_busy[EP0_OUT] = true;
			ep_busy[EP0_IN] = false;
			ep0_state = EP0_STATE_SETUP;
			break;
		}
		default:
			break;
		}
		return;
	}

	/* Endpoint event. */
	unsigned int ep = (e >> 1) & 0x1f;
	uint32_t etype = (e >> 6) & 0xf;
	uint32_t estatus = (e >> 12) & 0xf;

	if (ep == EP1_IN) {
		/*
		 * XferComplete: chunk sent, free EP1 IN and drain the next.
		 * XferNotReady: host polled with no transfer armed — try to send.
		 * Either way the host's IN polling drives the TX pump.
		 */
		if (etype == DEPEVT_XFERCOMPLETE)
			ep1_in_busy = false;
		dwc3_tx_pump();
		return;
	}
	if (ep == EP1_OUT) {
		if (etype == DEPEVT_XFERCOMPLETE) {
			uint32_t residual, got;

			dcache_flush((uintptr_t)MEM->ep1out_trb, 16);
			residual = MEM->ep1out_trb[2];
			got = sizeof(MEM->rx_buf) - residual;
			if (got > 0) {
				dcache_flush((uintptr_t)MEM->rx_buf, got);
				for (uint32_t i = 0; i < got; i++) {
					if (CHAN->rx_head - CHAN->rx_tail >= RX_RING_SZ)
						break; /* ring full; drop */
					CHAN->rx_buf[CHAN->rx_head & (RX_RING_SZ - 1)] =
						MEM->rx_buf[i];
					CHAN->rx_head = CHAN->rx_head + 1;
				}
			}
			post_ep1_out();
		}
		return;
	}
	if (ep != EP0_OUT && ep != EP0_IN)
		return;

	if (etype == DEPEVT_XFERCOMPLETE)
		handle_ep0_xfercomplete(ep);
	else if (etype == DEPEVT_XFERNOTREADY)
		handle_ep0_xfernotready(ep, estatus);
}

/* Drain the interrupter-0 event ring. Called from the FIQ handler. */
void imx95_dwc3_service(void)
{
	uint32_t count = rd(GEVNTCOUNT) & 0xffff;

	if (count == 0)
		return;
	dsb();
	for (uint32_t i = 0; i < count / 4; i++) {
		volatile uint32_t *slot = &MEM->event_buf[evt_idx];
		uint32_t e;
		uint32_t spins = 0;

		dcache_flush((uintptr_t)slot, 4);
		e = *slot;
		while (e == 0 && spins < 5000) {
			dcache_flush((uintptr_t)slot, 4);
			e = *slot;
			spins++;
		}
		evt_idx = (evt_idx + 1) % EVENT_COUNT;
		wr(GEVNTCOUNT, 4);
		process_event(e);
	}
}

/* Platform PSCI system reset (imx95_psci.c) — SCMI full-reset via the SM. */
void __dead2 imx_system_reset(void);

void __dead2 imx95_dwc3_do_reboot(void)
{
	/* Soft-disconnect so the host sees a clean unplug, then reset. */
	wr(DCTL, rd(DCTL) & ~DCTL_RUN_STOP);
	mdelay(50);
	imx_system_reset();
	while (true) {
	}
}

/*
 * SMC entry (IMX_SIP_USB_CONSOLE): osdyne poked us after queueing console output
 * into the TX ring. Drain it to EP1 IN. Runs at EL3 with FIQ masked, so it does
 * not race the FIQ handler.
 */
void imx95_dwc3_console_kick(void)
{
	/*
	 * Mask FIQ so the DWC3 FIQ handler can't preempt this SMC and race on the
	 * controller/DMA state. Restore the caller's DAIF (the SMC may have been
	 * entered with FIQ already masked).
	 */
	u_register_t daif = read_daif();

	write_daifset(DAIF_FIQ_BIT);
	isb();
	dwc3_tx_pump();
	write_daif(daif);
	isb();
}

void imx95_dwc3_gadget_init(void)
{
	ep0_state = EP0_STATE_SETUP;
	evt_idx = 0;

	/* Initialise the EL1<->EL3 console channel before any traffic. */
	CHAN->tx_head = 0;
	CHAN->tx_tail = 0;
	CHAN->rx_head = 0;
	CHAN->rx_tail = 0;
	CHAN->configured = 0;
	CHAN->magic = CONSOLE_CHAN_MAGIC;

	gadget_start();
}
