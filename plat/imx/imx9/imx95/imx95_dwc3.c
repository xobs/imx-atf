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
#include <string.h>

#include <arch.h>
#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>
#include <lib/xlat_tables/xlat_tables_v2.h>

#include <context.h>
#include <lib/el3_runtime/context_mgmt.h>
#include <plat/common/platform.h>

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
#define DSTS_DEVCTRLHLT		(1u << 22)	/* Device Controller Halted */
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
/*
 * Host memory peek/poke (odd's tasru read/write). 32-bit target address is
 * carried in wValue (addr[31:16]) and wIndex (addr[15:0]); MEM_READ returns the
 * bytes in the IN data stage, MEM_WRITE takes them in the OUT data stage. Every
 * access is guarded by mem_range_ok() so an unmapped/invalid address is rejected
 * (STALL) rather than faulting the secure monitor.
 */
#define VENDOR_REQ_MEM_READ	0x02
#define VENDOR_REQ_MEM_WRITE	0x03
/*
 * Debug monitor.  The stub runs in EL3, so when it is entered the interrupted
 * non-secure context is already saved -- the running PC and registers are
 * reachable without any cooperation from osdyne.  "Stopping" is therefore just
 * declining to return to it.  See docs/debug-api.md in the stm32mp2 bringup
 * tree, which specifies this protocol; the two platforms speak the same wire.
 */
#define VENDOR_REQ_STOP		0x04
#define VENDOR_REQ_RESUME	0x05
#define VENDOR_REQ_STATUS	0x06
#define VENDOR_REQ_REGS		0x07
/*
 * Self-hosted debug.  These are the operations memory cannot express: stepping
 * needs MDSCR_EL1.SS and the step exception, and breakpoints and watchpoints
 * live in system registers.  Addresses arrive in the OUT data stage because
 * wValue/wIndex are already spent on the address in the peek/poke requests and
 * a watchpoint needs a length and an access kind besides.
 */
#define VENDOR_REQ_STEP		0x08
#define VENDOR_REQ_SET_BP	0x09
#define VENDOR_REQ_CLEAR_BP	0x0a
#define VENDOR_REQ_SET_WP	0x0b
#define VENDOR_REQ_CLEAR_WP	0x0c
#define VENDOR_REQ_DEBUG_CAPS	0x0d
/*
 * Whether a host is reading the console: wValue 1 when it opens the console
 * endpoint, 0 when it closes it. It sets CHAN_HOST_READING, which is what
 * tells osdyne to wait for room in a full TX ring rather than drop output --
 * the convention RTT uses, where the host sets a channel's blocking mode when
 * it connects and clears it when it goes. BL31 clears it too on a bus reset,
 * a disconnect or a deconfiguration, so a host that vanishes without saying
 * so cannot leave osdyne waiting on a ring nobody will drain.
 */
#define VENDOR_REQ_CONSOLE_READING	0x0e
/*
 * What this gadget is and what it can do, so that one host tool can serve the
 * gadget on every SoC it has been ported to (odd's bl31_gadget module) without
 * telling them apart by USB id.  The reply is struct gadget_info: later
 * versions only append to it, and a host reads as far as it knows.  Firmware
 * that predates the request stalls it, which is how a host tells.
 */
#define VENDOR_REQ_GET_INFO	0x0f

#define INFO_MAGIC		"osdg"
#define INFO_VERSION		1U
/* The chip as osdyne's targets name it. */
#define INFO_CHIP		"MIMX95"

#define INFO_CAP_CONSOLE	(1U << 0)	/* osdyne's console on EP1 IN */
#define INFO_CAP_MEMORY		(1U << 1)	/* VENDOR_REQ_MEM_READ/_WRITE */
#define INFO_CAP_REBOOT		(1U << 2)	/* VENDOR_REQ_REBOOT */
#define INFO_CAP_DEBUG_MONITOR	(1U << 3)	/* VENDOR_REQ_STOP..._DEBUG_CAPS */
#define INFO_CAP_CONSOLE_READING (1U << 4)	/* VENDOR_REQ_CONSOLE_READING */
#define INFO_CAP_HOLD		(1U << 5)	/* struct hold_request */

/* Reply to VENDOR_REQ_GET_INFO.  Little-endian, naturally aligned, no padding. */
struct gadget_info {
	char magic[4];		/* INFO_MAGIC, not NUL-terminated */
	uint16_t version;	/* INFO_VERSION */
	uint16_t length;	/* sizeof(struct gadget_info) */
	uint32_t caps;		/* INFO_CAP_* */
	char chip[16];		/* INFO_CHIP, NUL-padded */
	uint32_t reserved;
	/*
	 * Where BL31 would load a BL33 the host sends it; both 0, since this
	 * one does not take one.
	 */
	uint64_t bl33_base;
	uint64_t bl33_size;
};

/*
 * wValue on VENDOR_REQ_RESUME: 0 resumes where the debuggee stopped, 1 steps
 * over the faulting instruction first.  A fault stop is otherwise a trap --
 * returning to an instruction that aborted just aborts again -- so the skip is
 * what makes "inspect a fault, then carry on" actually possible.
 */
#define RESUME_SKIP_FAULTING_INSN	1U

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
	volatile uint32_t flags;	/* CHAN_*; written by EL3 only */
	volatile uint32_t tx_head;	/* producer: EL1 */
	volatile uint32_t tx_tail;	/* consumer: EL3 */
	volatile uint32_t rx_head;	/* producer: EL3 */
	volatile uint32_t rx_tail;	/* consumer: EL1 */
	volatile uint8_t tx_buf[TX_RING_SZ];
	volatile uint8_t rx_buf[RX_RING_SZ];
};

/* The host has configured the device; BL31 drains the TX ring. */
#define CHAN_CONFIGURED		(1U << 0)
/*
 * A host is reading the console (VENDOR_REQ_CONSOLE_READING), so osdyne waits
 * for room in a full TX ring instead of dropping what it was writing.
 */
#define CHAN_HOST_READING	(1U << 1)

#define CHAN ((volatile struct console_chan *)CONSOLE_CHAN_BASE)

/*
 * A host that starts the payload itself asks BL31 to hold BL33 back until it
 * is reading the console: a probe's reset-halt, for a board booted over USB.
 * Without it osdyne writes its first output, and drops it, before any host
 * can be reading. The host finds the record by its magic.
 */
#define HOLD_MAGIC	"osdyne-hold-v1"

struct hold_request {
	char magic[16];		/* HOLD_MAGIC, zero-padded */
	uint32_t hold;		/* non-zero: hold BL33 */
	uint32_t reserved;
};

/*
 * Written over SDP by odd, which starts the payload itself, before it jumps to
 * the ATF container: the container is hashed, so the request cannot ride in
 * the image. It sits in spare space at the end of the console window, which
 * the channel never reaches, and is consumed at gadget init so that a later
 * boot without the host is not held.
 */
#define HOLD_REQUEST_BASE	(CONSOLE_CHAN_BASE + 0xff00UL)
#define HOLD_REQUEST ((volatile struct hold_request *)HOLD_REQUEST_BASE)

static bool hold_requested;

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

	if ((CHAN->flags & CHAN_CONFIGURED) == 0U || ep1_in_busy)
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
/* A validated MEM_WRITE is staged here between its SETUP and its OUT data stage. */
static bool mem_write_pending;
static uintptr_t mem_write_addr;
static uint32_t mem_write_len;

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
	CHAN->flags = CHAN_CONFIGURED;
}

static void gadget_start(void)
{
	uint32_t residual;

	/*
	 * Shut down whatever the previous stage left running before we present
	 * our own gadget. On a cold boot the SPL tears the DWC3 down for us, but
	 * on a warm reflash it hands us its still-running SDP gadget ("USB Download
	 * Gadget"); bringing our gadget up on top of that leaves the controller in
	 * a state where the host never enumerates "osdyne console". Halt the
	 * controller and power the PHY down here (imx95_dwc3_disconnect), so we
	 * always start from a clean, powered-down state regardless of how we were
	 * entered; phy_init() below powers the PHY back up.
	 */
	imx95_dwc3_disconnect();

	global_init();

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

	/* NB: TF-A's printf has no '#' flag (it aborts the line) — use "0x%x". */
	NOTICE("dwc3(el3): gadget up gsnpsid=0x%x\n", rd(GSNPSID));
}

/* ------------------------------------------------------- debug monitor */

/*
 * Set by the host via VENDOR_REQ_STOP.  The FIQ handler parks in EL3 while it
 * is set, still draining the USB event ring, so the host can inspect and
 * resume.  Volatile because it is written from the EP0 handler and read by the
 * loop that the same handler runs inside.
 */
static volatile bool dbg_stop_requested;
static volatile bool dbg_stopped;
static volatile uint32_t dbg_stop_reason;

/*
 * Fault detail latched on entry to the monitor.  dbg_fault_pc is set only when
 * the reported PC is not simply where EL3 will return (the payload-reported
 * case), and cleared on resume so a later host stop does not inherit it.
 */
static volatile uint64_t dbg_fault_esr;
static volatile uint64_t dbg_fault_far;
static volatile uint64_t dbg_fault_pc;

/* Set by VENDOR_REQ_RESUME with wValue == RESUME_SKIP_FAULTING_INSN. */
static volatile bool dbg_resume_skip;

/*
 * True once the controller is up.  The fault paths run at times the console
 * path never does -- an abort during early BL31 setup, say -- and parking to
 * poll a controller that has no clocks would replace a diagnosable panic with
 * a silent hang, so they check this first.
 */
static volatile bool dwc3_ready;

/*
 * The non-secure context TF-A saved when it took the interrupt that brought us
 * into EL3.  This is what makes a stub debugger practical here: the registers
 * of whatever was running are already spilled, so reporting them costs nothing.
 */
static void *dbg_ns_context;

/* Layout of the VENDOR_REQ_STATUS reply.  Little-endian, packed. */
struct dbg_status {
	uint32_t stopped;
	uint32_t reason;
	uint64_t pc;    /* ELR_EL3: where the non-secure world will resume */
	uint64_t spsr;  /* SPSR_EL3: its exception level and interrupt masks */
	uint64_t sp;    /* the stack the debuggee is actually using */
	uint64_t esr;   /* fault syndrome, 0 when the stop was not a fault */
	uint64_t far;   /* faulting address, where the syndrome provides one */
};

static uint32_t dbg_fill_status(void *out)
{
	struct dbg_status st = {
		.stopped = dbg_stopped ? 1U : 0U,
		.reason = dbg_stop_reason,
		.pc = 0U,
		.spsr = 0U,
		.sp = 0U,
		.esr = dbg_fault_esr,
		.far = dbg_fault_far,
	};

	if (dbg_ns_context != NULL) {
		st.pc = read_ctx_reg(get_el3state_ctx(dbg_ns_context), CTX_ELR_EL3);
		st.spsr = read_ctx_reg(get_el3state_ctx(dbg_ns_context), CTX_SPSR_EL3);

		/*
		 * Which stack pointer is live depends on SPSR.M: an EL1h
		 * debuggee is using SP_EL1, and its SP_EL0 is meaningless.
		 * SP_EL1 is not in the saved context -- TF-A only spills EL1
		 * system registers on a world switch, and an interrupt that
		 * returns to the same world is not one -- but it is still the
		 * debuggee's, so read it live.
		 */
		if ((st.spsr & 0xfU) == 0x5U) {
			st.sp = read_sp_el1();
		} else {
			st.sp = read_ctx_reg(get_gpregs_ctx(dbg_ns_context),
					     CTX_GPREG_SP_EL0);
		}
	}

	/*
	 * When the payload reported the fault itself, ELR_EL3 points into its
	 * exception handler -- the address worth showing is the one it told us
	 * it faulted at.
	 */
	if (dbg_fault_pc != 0U) {
		st.pc = dbg_fault_pc;
	}

	memcpy(out, &st, sizeof(st));
	return (uint32_t)sizeof(st);
}

/* x0..x30 of the interrupted non-secure context, in order. */
static uint32_t dbg_fill_regs(void *out)
{
	uint64_t *dst = (uint64_t *)out;
	unsigned int i;

	if (dbg_ns_context == NULL) {
		memset(out, 0, 31U * sizeof(uint64_t));
		return 31U * (uint32_t)sizeof(uint64_t);
	}

	for (i = 0U; i < 31U; i++) {
		dst[i] = read_ctx_reg(get_gpregs_ctx(dbg_ns_context),
				      CTX_GPREG_X0 + (i * 8U));
	}

	return 31U * (uint32_t)sizeof(uint64_t);
}

/* ------------------------------------------------- self-hosted debug */

/*
 * Self-hosted debug exceptions cannot be taken to EL3.  The architecture sends
 * them to EL1, or to EL2 when a hypervisor sets MDCR_EL2.TDE -- there is no
 * equivalent bit in MDCR_EL3, which governs only external debug and secure
 * debug.  So unlike stopping and unlike external aborts, breakpoints do need
 * the payload's cooperation: EL3 owns the debug registers (they are EL1
 * system registers, writable from EL3) and the payload routes the exception
 * back through STM32_SIP_SMC_DEBUG_FAULT.
 *
 * KDE is what enables debug exceptions taken to the EL being debugged, which
 * is what "EL1 debugging itself" means here.  MDE enables breakpoint and
 * watchpoint events at all.
 */
/* SPSR DAIF is bits [9:6]; D (debug exception mask) is the top one. */
#define SPSR_D_BIT		(ULL(1) << 9)

#define MDSCR_SS_BIT		(U(1) << 0)
#define MDSCR_KDE_BIT		(U(1) << 13)
#define MDSCR_MDE_BIT		(U(1) << 15)

/* PSTATE.SS in SPSR (arch.h) is the other half of the step state machine. */

/*
 * DBGBCR<n>_EL1 / DBGWCR<n>_EL1: enable, byte-select, and the {HMC, SSC, PMC}
 * triple that says which exception levels the match applies to.  Which triple
 * is right depends on where the debuggee runs -- osdyne is at EL2 on this
 * platform and at EL1 on others -- so it is derived from the debuggee's own
 * SPSR rather than fixed.  Getting it wrong fails silently: the breakpoint is
 * programmed, enabled, and simply never matches.
 *
 * SSC = 0b00 with these leaves the match applying in both security states,
 * which is harmless because there is no secure payload to break into.
 */
#define DBGBCR_E		(U(1) << 0)
#define DBGBCR_BAS_ALL		(U(0xf) << 5)
#define DBGBCR_HMC		(U(1) << 13)

#define DBGWCR_E		(U(1) << 0)
#define DBGWCR_LSC_SHIFT	3
#define DBGWCR_BAS_SHIFT	5
#define DBGWCR_HMC		(U(1) << 13)

/* PMC for breakpoints, PAC for watchpoints; same field position and meaning. */
#define DBG_PMC_SHIFT		1

/*
 * The {HMC, PMC} pair for the exception level the debuggee is running at.
 * EL1: HMC=0, PMC=0b11 matches EL1 and EL0 -- the combination verified on the
 * stm32mp2 monitor.  EL2: HMC=1, PMC=0b00.
 */
static uint64_t dbg_el_control(void)
{
	uint64_t spsr = 0U;
	unsigned int el;

	if (dbg_ns_context != NULL) {
		spsr = read_ctx_reg(get_el3state_ctx(dbg_ns_context),
				    CTX_SPSR_EL3);
	}

	/* SPSR.M[3:2] is the exception level the debuggee will return to. */
	el = (unsigned int)((spsr >> 2) & 0x3U);

	if (el >= 2U) {
		return DBGBCR_HMC | (U(0) << DBG_PMC_SHIFT);
	}

	return U(3) << DBG_PMC_SHIFT;
}

#define DBG_MAX_SLOTS		16U

static unsigned int dbg_nr_bp;
static unsigned int dbg_nr_wp;
static uint64_t dbg_bp_addr[DBG_MAX_SLOTS];
static uint64_t dbg_wp_addr[DBG_MAX_SLOTS];
static uint32_t dbg_bp_used;
static uint32_t dbg_wp_used;

/* Set while a single step is armed, so the step exception knows it is ours. */
static volatile bool dbg_step_pending;

/*
 * The vendor request whose 8-byte address argument is still in flight on EP0,
 * applied when the OUT data stage completes.
 */
static uint8_t dbg_arg_pending;
static uint8_t dbg_wp_len;
static uint8_t dbg_wp_access;

/*
 * The register number is part of the instruction encoding, so there is no way
 * to index these other than by unrolling.  Six breakpoints and four
 * watchpoints is what this core has; the tables are sized for the
 * architectural maximum and bounded by what ID_AA64DFR0_EL1 reports.
 */
#define DBG_WRITE_CASE(reg, n, v)					\
	case n:								\
		__asm__ volatile("msr " #reg #n "_el1, %0" : : "r" (v));\
		break

static void dbg_write_bvr(unsigned int n, uint64_t v)
{
	switch (n) {
	DBG_WRITE_CASE(dbgbvr, 0, v); DBG_WRITE_CASE(dbgbvr, 1, v);
	DBG_WRITE_CASE(dbgbvr, 2, v); DBG_WRITE_CASE(dbgbvr, 3, v);
	DBG_WRITE_CASE(dbgbvr, 4, v); DBG_WRITE_CASE(dbgbvr, 5, v);
	DBG_WRITE_CASE(dbgbvr, 6, v); DBG_WRITE_CASE(dbgbvr, 7, v);
	DBG_WRITE_CASE(dbgbvr, 8, v); DBG_WRITE_CASE(dbgbvr, 9, v);
	DBG_WRITE_CASE(dbgbvr, 10, v); DBG_WRITE_CASE(dbgbvr, 11, v);
	DBG_WRITE_CASE(dbgbvr, 12, v); DBG_WRITE_CASE(dbgbvr, 13, v);
	DBG_WRITE_CASE(dbgbvr, 14, v); DBG_WRITE_CASE(dbgbvr, 15, v);
	default: break;
	}
}

static void dbg_write_bcr(unsigned int n, uint64_t v)
{
	switch (n) {
	DBG_WRITE_CASE(dbgbcr, 0, v); DBG_WRITE_CASE(dbgbcr, 1, v);
	DBG_WRITE_CASE(dbgbcr, 2, v); DBG_WRITE_CASE(dbgbcr, 3, v);
	DBG_WRITE_CASE(dbgbcr, 4, v); DBG_WRITE_CASE(dbgbcr, 5, v);
	DBG_WRITE_CASE(dbgbcr, 6, v); DBG_WRITE_CASE(dbgbcr, 7, v);
	DBG_WRITE_CASE(dbgbcr, 8, v); DBG_WRITE_CASE(dbgbcr, 9, v);
	DBG_WRITE_CASE(dbgbcr, 10, v); DBG_WRITE_CASE(dbgbcr, 11, v);
	DBG_WRITE_CASE(dbgbcr, 12, v); DBG_WRITE_CASE(dbgbcr, 13, v);
	DBG_WRITE_CASE(dbgbcr, 14, v); DBG_WRITE_CASE(dbgbcr, 15, v);
	default: break;
	}
}

static void dbg_write_wvr(unsigned int n, uint64_t v)
{
	switch (n) {
	DBG_WRITE_CASE(dbgwvr, 0, v); DBG_WRITE_CASE(dbgwvr, 1, v);
	DBG_WRITE_CASE(dbgwvr, 2, v); DBG_WRITE_CASE(dbgwvr, 3, v);
	DBG_WRITE_CASE(dbgwvr, 4, v); DBG_WRITE_CASE(dbgwvr, 5, v);
	DBG_WRITE_CASE(dbgwvr, 6, v); DBG_WRITE_CASE(dbgwvr, 7, v);
	DBG_WRITE_CASE(dbgwvr, 8, v); DBG_WRITE_CASE(dbgwvr, 9, v);
	DBG_WRITE_CASE(dbgwvr, 10, v); DBG_WRITE_CASE(dbgwvr, 11, v);
	DBG_WRITE_CASE(dbgwvr, 12, v); DBG_WRITE_CASE(dbgwvr, 13, v);
	DBG_WRITE_CASE(dbgwvr, 14, v); DBG_WRITE_CASE(dbgwvr, 15, v);
	default: break;
	}
}

static void dbg_write_wcr(unsigned int n, uint64_t v)
{
	switch (n) {
	DBG_WRITE_CASE(dbgwcr, 0, v); DBG_WRITE_CASE(dbgwcr, 1, v);
	DBG_WRITE_CASE(dbgwcr, 2, v); DBG_WRITE_CASE(dbgwcr, 3, v);
	DBG_WRITE_CASE(dbgwcr, 4, v); DBG_WRITE_CASE(dbgwcr, 5, v);
	DBG_WRITE_CASE(dbgwcr, 6, v); DBG_WRITE_CASE(dbgwcr, 7, v);
	DBG_WRITE_CASE(dbgwcr, 8, v); DBG_WRITE_CASE(dbgwcr, 9, v);
	DBG_WRITE_CASE(dbgwcr, 10, v); DBG_WRITE_CASE(dbgwcr, 11, v);
	DBG_WRITE_CASE(dbgwcr, 12, v); DBG_WRITE_CASE(dbgwcr, 13, v);
	DBG_WRITE_CASE(dbgwcr, 14, v); DBG_WRITE_CASE(dbgwcr, 15, v);
	default: break;
	}
}

/*
 * Arm the debug plumbing the first time anything is set.  Deferred rather than
 * done at init so a board that never attaches a debugger runs with debug
 * exceptions routed where the architecture would put them anyway.
 */
static void dbg_enable_traps(void)
{
	static bool done;
	void *ctx;
	uint64_t dfr0;

	if (done) {
		return;
	}

	ctx = cm_get_context(NON_SECURE);
	if (ctx == NULL) {
		return;
	}

	dfr0 = read_id_aa64dfr0_el1();
	dbg_nr_bp = (unsigned int)(((dfr0 >> 12) & 0xfU) + 1U);
	dbg_nr_wp = (unsigned int)(((dfr0 >> 20) & 0xfU) + 1U);
	if (dbg_nr_bp > DBG_MAX_SLOTS) {
		dbg_nr_bp = DBG_MAX_SLOTS;
	}
	if (dbg_nr_wp > DBG_MAX_SLOTS) {
		dbg_nr_wp = DBG_MAX_SLOTS;
	}

	/*
	 * Both locks gate debug exceptions entirely and at least one of them is
	 * set out of reset, so nothing below matters until they are cleared.
	 * OS lock: OSLAR_EL1 = 0.  Double lock: OSDLR_EL1.DLK = 0.
	 */
	__asm__ volatile("msr oslar_el1, %0" : : "r" (ULL(0)));
	__asm__ volatile("msr osdlr_el1, %0" : : "r" (ULL(0)));
	isb();

	write_mdscr_el1(read_mdscr_el1() | MDSCR_MDE_BIT | MDSCR_KDE_BIT);
	isb();

	NOTICE("dwc3(el3): self-hosted debug armed (%u breakpoints, %u watchpoints)\n",
	       dbg_nr_bp, dbg_nr_wp);

	done = true;
}

static bool dbg_set_bp(uint64_t addr)
{
	unsigned int n;

	dbg_enable_traps();

	for (n = 0U; n < dbg_nr_bp; n++) {
		if ((dbg_bp_used & BIT_32(n)) == 0U) {
			dbg_bp_used |= BIT_32(n);
			dbg_bp_addr[n] = addr;
			dbg_write_bvr(n, addr & ~ULL(3));
			dbg_write_bcr(n, DBGBCR_E | DBGBCR_BAS_ALL |
					 dbg_el_control());
			isb();
			return true;
		}
	}

	return false;
}

static bool dbg_clear_bp(uint64_t addr, bool all)
{
	unsigned int n;
	bool hit = false;

	for (n = 0U; n < dbg_nr_bp; n++) {
		if ((dbg_bp_used & BIT_32(n)) == 0U) {
			continue;
		}
		if (!all && (dbg_bp_addr[n] != addr)) {
			continue;
		}
		dbg_write_bcr(n, 0U);
		dbg_bp_used &= ~BIT_32(n);
		hit = true;
	}
	isb();

	return hit;
}

/*
 * @len is the watched length in bytes and @access is 1 for loads, 2 for stores
 * and 3 for both, matching DBGWCR.LSC.  BAS selects which bytes of the
 * doubleword the address sits in are watched.
 */
static bool dbg_set_wp(uint64_t addr, unsigned int len, unsigned int access)
{
	unsigned int n;
	uint64_t bas;

	dbg_enable_traps();

	if ((len == 0U) || (len > 8U)) {
		return false;
	}
	if ((access == 0U) || (access > 3U)) {
		return false;
	}

	/* BAS is relative to the doubleword-aligned address in DBGWVR. */
	bas = ((ULL(1) << len) - 1U) << (addr & ULL(7));

	for (n = 0U; n < dbg_nr_wp; n++) {
		if ((dbg_wp_used & BIT_32(n)) == 0U) {
			dbg_wp_used |= BIT_32(n);
			dbg_wp_addr[n] = addr;
			dbg_write_wvr(n, addr & ~ULL(7));
			dbg_write_wcr(n, DBGWCR_E | dbg_el_control() |
					 ((uint64_t)access << DBGWCR_LSC_SHIFT) |
					 (bas << DBGWCR_BAS_SHIFT));
			isb();
			return true;
		}
	}

	return false;
}

static bool dbg_clear_wp(uint64_t addr, bool all)
{
	unsigned int n;
	bool hit = false;

	for (n = 0U; n < dbg_nr_wp; n++) {
		if ((dbg_wp_used & BIT_32(n)) == 0U) {
			continue;
		}
		if (!all && (dbg_wp_addr[n] != addr)) {
			continue;
		}
		dbg_write_wcr(n, 0U);
		dbg_wp_used &= ~BIT_32(n);
		hit = true;
	}
	isb();

	return hit;
}

/* Reply to VENDOR_REQ_DEBUG_CAPS. */
struct dbg_caps {
	uint32_t nr_bp;
	uint32_t nr_wp;
	uint32_t bp_used;
	uint32_t wp_used;
	/* Readbacks, so a step that silently does nothing can be diagnosed. */
	uint64_t mdscr;
	uint64_t oslsr;
	uint64_t bvr0;
	uint64_t bcr0;
};

static uint32_t dbg_fill_caps(void *out)
{
	struct dbg_caps c;

	dbg_enable_traps();

	c.nr_bp = dbg_nr_bp;
	c.nr_wp = dbg_nr_wp;
	c.bp_used = dbg_bp_used;
	c.wp_used = dbg_wp_used;
	c.mdscr = read_mdscr_el1();
	__asm__ volatile("mrs %0, oslsr_el1" : "=r" (c.oslsr));
	__asm__ volatile("mrs %0, dbgbvr0_el1" : "=r" (c.bvr0));
	__asm__ volatile("mrs %0, dbgbcr0_el1" : "=r" (c.bcr0));

	memcpy(out, &c, sizeof(c));
	return (uint32_t)sizeof(c);
}

/*
 * Monitor-mode park.  Staying here is what "stopped" means: the non-secure
 * world does not run again until the host resumes it.  The USB event ring is
 * drained by polling, which is what keeps the console and every other vendor
 * request alive while the target is stopped -- including, crucially, when we
 * were entered from a fault rather than from an interrupt.
 *
 * Returns true when the host asked to step over the faulting instruction.
 */
static bool dbg_park(void)
{
	bool skip;

	while (dbg_stop_requested) {
		dbg_stopped = true;
		imx95_dwc3_service();
	}

	skip = dbg_resume_skip;

	if (dbg_ns_context != NULL) {
		el3_state_t *st = get_el3state_ctx(dbg_ns_context);
		uint64_t spsr = read_ctx_reg(st, CTX_SPSR_EL3);

		/*
		 * BL31 hands the payload PSTATE.D set, which masks debug
		 * exceptions entirely -- a breakpoint programmed into the
		 * registers simply never fires.  Unmask on the way out
		 * whenever something is armed; leave it alone otherwise, so a
		 * payload that is not being debugged runs as it always did.
		 */
		if ((dbg_bp_used != 0U) || (dbg_wp_used != 0U) ||
		    dbg_step_pending) {
			spsr &= ~SPSR_D_BIT;
		}

		/*
		 * Arm a single step.  Both halves are needed: MDSCR_EL1
		 * enables stepping, and PSTATE.SS in the restored SPSR is what
		 * makes the exception fire after exactly one instruction.
		 */
		if (dbg_step_pending) {
			write_mdscr_el1(read_mdscr_el1() | MDSCR_SS_BIT);
			isb();
			spsr |= SPSR_SS_BIT;
		}

		write_ctx_reg(st, CTX_SPSR_EL3, spsr);
	}

	dbg_resume_skip = false;
	dbg_stopped = false;
	dbg_stop_reason = DBG_REASON_RUNNING;
	dbg_fault_esr = 0U;
	dbg_fault_far = 0U;
	dbg_fault_pc = 0U;

	return skip;
}

bool imx95_dwc3_debug_fault(unsigned int reason, void *handle,
				 uint64_t esr, uint64_t far, uint64_t pc)
{
	/*
	 * An SMC handler is not given the interrupted context, but TF-A has
	 * saved it by the time we run, so ask for it rather than reporting a
	 * stale one left over from the last interrupt.
	 */
	if (!dwc3_ready) {
		return false;
	}

	if (handle != NULL) {
		dbg_ns_context = handle;
	} else {
		dbg_ns_context = cm_get_context(NON_SECURE);
	}

	dbg_fault_esr = esr;
	dbg_fault_far = far;
	dbg_fault_pc = pc;
	dbg_stop_reason = reason;

	/*
	 * A step has happened, so disarm before parking.  Leaving it armed
	 * would make the next resume step again -- and step the payload's own
	 * exception handler, which is not what anyone asked for.
	 */
	if (reason == DBG_REASON_STEP) {
		write_mdscr_el1(read_mdscr_el1() & ~MDSCR_SS_BIT);
		isb();
		dbg_step_pending = false;
	}

	/*
	 * A fault stops the target whether or not a host is watching: that is
	 * the whole point.  If nobody ever attaches, this is a hang -- but it
	 * is a hang the host can still connect to and inspect, which is what
	 * the alternative (panic, or a payload spinning in its own handler
	 * with USB dead) does not offer.
	 */
	dbg_stop_requested = true;

	return dbg_park();
}

bool imx95_dwc3_debug_ready(void)
{
	return dwc3_ready;
}

void imx95_dwc3_debug_park(void)
{
	(void)dbg_park();
}

void imx95_dwc3_debug_context(void *handle)
{
	dbg_ns_context = handle;
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
 * Guard for host memory peek/poke: true only if every page of [addr, addr+len)
 * is mapped in BL31's own translation tables, so the access cannot fault. When
 * need_write, the mapping must also be writable. This is what makes an invalid
 * host request safe — it is rejected (STALL) instead of taking a data abort in
 * EL3 (which would be fatal).
 */
static bool mem_range_ok(uintptr_t addr, uint32_t len, bool need_write)
{
	if (len == 0U)
		return false;
	if ((addr + len) < addr)	/* wrap-around */
		return false;
	for (uintptr_t page = addr & ~0xFFFUL; page < addr + len; page += 0x1000UL) {
		uint32_t attr;

		if (xlat_get_mem_attributes(page, &attr) != 0)
			return false;	/* unmapped -> would fault */
		if (need_write && (attr & MT_RW) == 0U)
			return false;	/* read-only -> write would fault */
	}
	return true;
}

/*
 * Read/write a validated target range using the widest naturally-aligned access
 * for the length, so device registers are touched with a single correctly-sized
 * access (a byte-at-a-time loop would be wrong for many peripheral registers).
 */
static void mem_read_range(uintptr_t addr, uint8_t *dst, uint32_t len)
{
	if (len == 4U && (addr & 3U) == 0U) {
		uint32_t v = mmio_read_32(addr);
		memcpy(dst, &v, 4);
	} else if (len == 8U && (addr & 7U) == 0U) {
		uint64_t v = mmio_read_64(addr);
		memcpy(dst, &v, 8);
	} else if (len == 2U && (addr & 1U) == 0U) {
		uint16_t v = mmio_read_16(addr);
		memcpy(dst, &v, 2);
	} else {
		for (uint32_t i = 0U; i < len; i++)
			dst[i] = mmio_read_8(addr + i);
	}
}

static void mem_write_range(uintptr_t addr, const uint8_t *src, uint32_t len)
{
	if (len == 4U && (addr & 3U) == 0U) {
		uint32_t v;

		memcpy(&v, src, 4);
		mmio_write_32(addr, v);
	} else if (len == 8U && (addr & 7U) == 0U) {
		uint64_t v;

		memcpy(&v, src, 8);
		mmio_write_64(addr, v);
	} else if (len == 2U && (addr & 1U) == 0U) {
		uint16_t v;

		memcpy(&v, src, 2);
		mmio_write_16(addr, v);
	} else {
		for (uint32_t i = 0U; i < len; i++)
			mmio_write_8(addr + i, src[i]);
	}
}

static uint32_t fill_info(void *out)
{
	struct gadget_info info;

	_Static_assert(sizeof(struct gadget_info) == 48U,
		       "struct gadget_info is part of the USB protocol");

	memset(&info, 0, sizeof(info));
	memcpy(info.magic, INFO_MAGIC, sizeof(info.magic));
	info.version = INFO_VERSION;
	info.length = (uint16_t)sizeof(info);
	info.caps = INFO_CAP_CONSOLE | INFO_CAP_MEMORY | INFO_CAP_REBOOT |
		    INFO_CAP_DEBUG_MONITOR | INFO_CAP_CONSOLE_READING |
		    INFO_CAP_HOLD;
	memcpy(info.chip, INFO_CHIP, sizeof(INFO_CHIP) - 1U);

	memcpy(out, &info, sizeof(info));
	return (uint32_t)sizeof(info);
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
			if (wvalue == 0) {
				configured = false;
				CHAN->flags = 0U;
			}
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
		if (brequest == VENDOR_REQ_CONSOLE_READING && (bmreq & 0x80U) == 0U &&
		    wvalue <= 1U && wlen == 0U)
			return 0;
		/*
		 * Accept these here so the control transfer is not stalled; the
		 * flags themselves are set in the status stage, alongside the
		 * reboot request, so the device has already acknowledged the
		 * request by the time it acts on it.
		 */
		if (brequest == VENDOR_REQ_STOP || brequest == VENDOR_REQ_RESUME ||
		    brequest == VENDOR_REQ_STEP)
			return 0;
		if ((brequest == VENDOR_REQ_CLEAR_BP ||
		     brequest == VENDOR_REQ_CLEAR_WP) && (bmreq & 0x80U) == 0U) {
			/* wValue != 0 means "all"; otherwise the address follows. */
			if (wvalue != 0U)
				return 0;
			if (wlen != sizeof(uint64_t))
				return -1;
			dbg_arg_pending = brequest;
			return (int)wlen;
		}
		if (brequest == VENDOR_REQ_SET_BP && (bmreq & 0x80U) == 0U) {
			if (wlen != sizeof(uint64_t))
				return -1;
			dbg_arg_pending = brequest;
			return (int)wlen;
		}
		if (brequest == VENDOR_REQ_SET_WP && (bmreq & 0x80U) == 0U) {
			/* wIndex carries the length (high) and access kind (low). */
			if (wlen != sizeof(uint64_t))
				return -1;
			dbg_wp_len = (windex >> 8) & 0xffU;
			dbg_wp_access = windex & 0xffU;
			dbg_arg_pending = brequest;
			return (int)wlen;
		}
		if (brequest == VENDOR_REQ_STATUS && (bmreq & 0x80U) != 0U) {
			uint32_t n = dbg_fill_status((void *)MEM->ep0_data);

			dcache_flush((uintptr_t)MEM->ep0_data, n);
			return wlen < n ? wlen : n;
		}
		if (brequest == VENDOR_REQ_REGS && (bmreq & 0x80U) != 0U) {
			uint32_t n = dbg_fill_regs((void *)MEM->ep0_data);

			dcache_flush((uintptr_t)MEM->ep0_data, n);
			return wlen < n ? wlen : n;
		}
		if (brequest == VENDOR_REQ_DEBUG_CAPS && (bmreq & 0x80U) != 0U) {
			uint32_t n = dbg_fill_caps((void *)MEM->ep0_data);

			dcache_flush((uintptr_t)MEM->ep0_data, n);
			return wlen < n ? wlen : n;
		}
		if (brequest == VENDOR_REQ_GET_INFO && (bmreq & 0x80U) != 0U) {
			uint32_t n = fill_info((void *)MEM->ep0_data);

			dcache_flush((uintptr_t)MEM->ep0_data, n);
			return wlen < n ? wlen : n;
		}
		if (brequest == VENDOR_REQ_MEM_READ && (bmreq & 0x80U) != 0U) {
			/* device->host: read [addr, addr+wlen) into ep0_data. */
			uintptr_t addr = ((uintptr_t)wvalue << 16) | windex;
			uint32_t n = wlen;

			if (n > sizeof(MEM->ep0_data))
				n = sizeof(MEM->ep0_data);
			if (!mem_range_ok(addr, n, false))
				return -1;
			mem_read_range(addr, (uint8_t *)MEM->ep0_data, n);
			dcache_flush((uintptr_t)MEM->ep0_data, n ? n : 1);
			return (int)n;
		}
		if (brequest == VENDOR_REQ_MEM_WRITE && (bmreq & 0x80U) == 0U) {
			/*
			 * host->device: validate now (so an invalid request STALLs
			 * before any data moves); the bytes arrive in the OUT data
			 * stage and are applied on its completion.
			 */
			uintptr_t addr = ((uintptr_t)wvalue << 16) | windex;
			uint32_t n = wlen;

			if (n == 0U || n > sizeof(MEM->ep0_data))
				return -1;
			if (!mem_range_ok(addr, n, true))
				return -1;
			mem_write_addr = addr;
			mem_write_len = n;
			mem_write_pending = true;
			return (int)n;
		}
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
		/* New SETUP supersedes any stale EP0_IN / pending-write state. */
		ep_busy[EP0_IN] = false;
		ep_rsc[EP0_IN] = 0;
		mem_write_pending = false;

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
			if (bmreq == 0x40 && brequest == VENDOR_REQ_CONSOLE_READING) {
				if (wvalue != 0U)
					CHAN->flags |= CHAN_HOST_READING;
				else
					CHAN->flags &= ~CHAN_HOST_READING;
			}
			if (bmreq == 0x40 && brequest == VENDOR_REQ_REBOOT &&
			    wvalue == REBOOT_MAGIC)
				reboot_pending = true;
			if (bmreq == 0x40 && brequest == VENDOR_REQ_STOP) {
				dbg_stop_reason = DBG_REASON_HOST;
				dbg_stop_requested = true;
			}
			if (bmreq == 0x40 && brequest == VENDOR_REQ_RESUME) {
				dbg_resume_skip =
					(wvalue == RESUME_SKIP_FAULTING_INSN);
				dbg_stop_requested = false;
			}
			if (bmreq == 0x40 && brequest == VENDOR_REQ_STEP) {
				/*
				 * Stepping needs the same enables a breakpoint
				 * does -- KDE above all, without which a
				 * same-EL debug exception is simply not taken.
				 */
				dbg_enable_traps();
				/* Leaving the park arms the step; see dbg_park. */
				dbg_step_pending = true;
				dbg_stop_requested = false;
			}
			if (bmreq == 0x40 && brequest == VENDOR_REQ_CLEAR_BP &&
			    wvalue != 0U)
				(void)dbg_clear_bp(0U, true);
			if (bmreq == 0x40 && brequest == VENDOR_REQ_CLEAR_WP &&
			    wvalue != 0U)
				(void)dbg_clear_wp(0U, true);
			ep0_state = EP0_STATE_STATUS_PENDING;
		}
	} else if (ep0_state == EP0_STATE_DATA) {
		/* Apply a validated MEM_WRITE now that its OUT data has landed. */
		if (mem_write_pending && ep == EP0_OUT) {
			dcache_flush((uintptr_t)MEM->ep0_data, mem_write_len);
			mem_write_range(mem_write_addr,
					(const uint8_t *)MEM->ep0_data, mem_write_len);
			mem_write_pending = false;
		}
		if (dbg_arg_pending != 0U && ep == EP0_OUT) {
			uint64_t addr;

			dcache_flush((uintptr_t)MEM->ep0_data, sizeof(addr));
			memcpy(&addr, (const void *)MEM->ep0_data, sizeof(addr));

			switch (dbg_arg_pending) {
			case VENDOR_REQ_SET_BP:
				(void)dbg_set_bp(addr);
				break;
			case VENDOR_REQ_CLEAR_BP:
				(void)dbg_clear_bp(addr, false);
				break;
			case VENDOR_REQ_SET_WP:
				(void)dbg_set_wp(addr, dbg_wp_len, dbg_wp_access);
				break;
			case VENDOR_REQ_CLEAR_WP:
				(void)dbg_clear_wp(addr, false);
				break;
			default:
				break;
			}
			dbg_arg_pending = 0U;
		}
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
			CHAN->flags &= ~CHAN_HOST_READING;
			break;
		case DEVT_USBRST:
			set_address(0);
			ep0_state = EP0_STATE_SETUP;
			configured = false;
			ep1_in_busy = false;
			mem_write_pending = false;
			CHAN->flags = 0U;
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
void imx95_dwc3_hold_for_host(void)
{
	if (!(hold_requested))
		return;

	NOTICE("dwc3(el3): holding BL33 until the host reads the console\n");
	while ((CHAN->flags & CHAN_HOST_READING) == 0U)
		imx95_dwc3_service();
	NOTICE("dwc3(el3): the host is reading the console; starting BL33\n");
}

void __dead2 imx_system_reset(void);

void imx95_dwc3_disconnect(void)
{
	/*
	 * Clear RUN_STOP so the controller drops its D+ pull-up: the host sees a
	 * clean unplug and the next stage's serial-download gadget (SPL SDP / ROM
	 * SDPS) can enumerate after the reset. osdyne no longer tears the gadget
	 * down before it resets, and the SPL only does so on a cold boot, so BL31
	 * must do it on every reset path to cover the warm-reset (reflash) case.
	 * Safe to call even if the gadget was never brought up — it is just a
	 * controller register write.
	 *
	 * Clearing RUN_STOP only *requests* a halt; we must wait for the
	 * controller to actually reach the halted state (DSTS.DEVCTRLHLT) before
	 * continuing, else we act mid-halt.
	 */
	wr(DCTL, rd(DCTL) & ~DCTL_RUN_STOP);
	for (int i = 0; i < 1000000; i++) {
		if ((rd(DSTS) & DSTS_DEVCTRLHLT) != 0U)
			break;
	}

	/*
	 * Halting the controller isn't enough: the SPL leaves the USB2 PHY
	 * powered, and the SCMI cold reset doesn't cycle it, so the D+ pull-up
	 * stays asserted and the ROM's serial-download gadget can't re-enumerate
	 * after the reset. Power the PHY down — assert its resets and drop the
	 * reference clock (undo phy_init's REF_SSP_EN) — so the bus is released
	 * and the ROM re-initialises the PHY from a clean, powered-down state.
	 */
	phy_wr(PHY_CTRL1,
	       phy_rd(PHY_CTRL1) | PHY_CTRL1_RESET | PHY_CTRL1_ATERESET);
	phy_wr(PHY_CTRL0, phy_rd(PHY_CTRL0) & ~PHY_CTRL0_REF_SSP_EN);
	mdelay(100);
}

void __dead2 imx95_dwc3_do_reboot(void)
{
	/* imx_system_reset() disconnects the gadget for us before resetting. */
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

	/*
	 * Zero the DWC3 DMA structures (event ring, TRBs, setup/EP0/EP1 buffers).
	 * They live at a fixed DRAM address, not in BL31's BSS, so the C runtime
	 * never clears them. On a cold boot DDR init leaves DRAM clean, but a warm
	 * (SCMI) reset retains the previous osdyne's DWC3 state — stale TRBs (with
	 * HWO set) and event-ring entries — which the controller then acts on and
	 * enumeration silently fails. Clear + flush to DRAM so the controller
	 * always starts from a known-good state regardless of how we booted.
	 */
	memset((void *)(uintptr_t)MEM, 0, sizeof(struct dwc3_mem));
	dcache_flush((uintptr_t)MEM, sizeof(struct dwc3_mem));

	/* Take, and consume, the host's request to hold BL33 back. */
	hold_requested = memcmp((const void *)(uintptr_t)HOLD_REQUEST->magic,
				HOLD_MAGIC, sizeof(HOLD_MAGIC)) == 0 &&
			 HOLD_REQUEST->hold != 0U;
	memset((void *)(uintptr_t)HOLD_REQUEST, 0, sizeof(struct hold_request));

	/* Initialise the EL1<->EL3 console channel before any traffic. */
	CHAN->tx_head = 0;
	CHAN->tx_tail = 0;
	CHAN->rx_head = 0;
	CHAN->rx_tail = 0;
	CHAN->flags = 0U;
	CHAN->magic = CONSOLE_CHAN_MAGIC;

	gadget_start();

	dwc3_ready = true;
}
