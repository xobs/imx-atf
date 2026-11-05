/*
 * i.MX95 DWC3 USB controller driver for osdyne's gadget, resident in BL31
 * (EL3).  The protocol, the console rings and the debug monitor are in
 * plat/imx/common/imx_gadget.c; this moves their bytes over the DWC3.
 *
 * Ported from osdyne's EL1 gadget (os/kernel/src/platform/hal/imx95/dwc3.rs).
 * The controller interrupts on GIC INTID 207 (SPI 175), which imx_gadget.c
 * makes a Group-0 FIQ to EL3.
 *
 * DMA memory lives at 0x93300000 (the only mid-DRAM window the DWC3 DMA master
 * can reach; OCRAM and the DRAM tail are unreachable). Mapped cacheable, with
 * explicit clean/invalidate (flush_dcache_range) around every controller-shared
 * access, mirroring the EL1 driver.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>

#include <imx_gadget_controller.h>

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
/* Word 2 of a TRB: what is left of the buffer once the transfer is done. */
#define TRB_BUFSIZ_MASK		0xffffffu

#define EVENT_COUNT	64
#define EVENT_BYTES	(EVENT_COUNT * 4)

/* ------------------------------------------------------------------- memory */

struct dwc3_mem {
	uint32_t event_buf[EVENT_COUNT];
	uint32_t ep0_setup_trb[4];
	uint32_t ep0_data_trb[4];
	uint32_t ep0_status_trb[4];
	uint32_t ep1out_trb[4];
	uint32_t ep1in_trb[4];
	uint8_t setup_buf[8];
	uint8_t ep0_data[IMX_GADGET_EP0_DATA_MAX];
	uint8_t rx_buf[512];
	uint8_t tx_buf[512];
};

#define MEM ((volatile struct dwc3_mem *)DMA_MEM_BASE)

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
 * Called on EP1 IN XferComplete/XferNotReady (the host's polling drives it),
 * and from osdyne's IMX_SIP_USB_CONSOLE kick.
 */
void imx_gadget_ctrl_tx_pump(void)
{
	uint32_t n;

	if (ep1_in_busy)
		return;
	n = imx_gadget_console_take((uint8_t *)MEM->tx_buf, sizeof(MEM->tx_buf));
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

static unsigned int evt_idx;

void imx_gadget_ctrl_set_address(uint8_t addr)
{
	wr(DCFG, (rd(DCFG) & ~DCFG_DEVADDR_MASK) | (((uint32_t)addr & 0x7f) << 3));
}

void imx_gadget_ctrl_configure(void)
{
	depcmd(EP0_OUT, DEPCMD_STARTCFG | (2 << 16), 0, 0);
	configure_ep(EP1_OUT, EP_TYPE_BULK, 512);
	configure_ep(EP1_IN, EP_TYPE_BULK, 512);
	post_ep1_out();
	ep1_in_busy = false;
}

void imx_gadget_ctrl_start(void)
{
	uint32_t residual;

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

	/*
	 * Shut down whatever the previous stage left running before we present
	 * our own gadget. On a cold boot the SPL tears the DWC3 down for us, but
	 * on a warm reflash it hands us its still-running SDP gadget ("USB Download
	 * Gadget"); bringing our gadget up on top of that leaves the controller in
	 * a state where the host never enumerates "osdyne console". Halt the
	 * controller and power the PHY down here (imx_gadget_ctrl_disconnect), so
	 * we always start from a clean, powered-down state regardless of how we
	 * were entered; phy_init() below powers the PHY back up.
	 */
	imx_gadget_ctrl_disconnect();

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
	 * set up before this runs). Clears GEVNTSIZ_INTMASK so event-ring writes
	 * assert the GIC line -> FIQ 207.
	 */
	wr(GEVNTSIZ, EVENT_BYTES);

	/* NB: TF-A's printf has no '#' flag (it aborts the line) — use "0x%x". */
	NOTICE("dwc3(el3): gadget up gsnpsid=0x%x\n", rd(GSNPSID));
}

/* --------------------------------------------------------- event processing */

static void handle_ep0_xfercomplete(unsigned int ep)
{
	unsigned int epi = ep;

	ep_busy[epi] = false;
	ep_rsc[epi] = 0;

	if (ep0_state == EP0_STATE_SETUP && ep == EP0_OUT) {
		uint8_t s[8];
		uint16_t wlen;
		int n;

		dcache_flush((uintptr_t)MEM->setup_buf, 8);
		for (int i = 0; i < 8; i++)
			s[i] = MEM->setup_buf[i];
		wlen = s[6] | (s[7] << 8);
		ep0_dir_in = (s[0] & 0x80) != 0;
		ep0_three_stage = wlen != 0;
		/* New SETUP supersedes any stale EP0_IN state. */
		ep_busy[EP0_IN] = false;
		ep_rsc[EP0_IN] = 0;

		n = imx_gadget_setup(s, (uint8_t *)MEM->ep0_data);
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
			uint32_t dn = ep0_dir_in ? (uint32_t)n : IMX_GADGET_EP0_DATA_MAX;
			ep0_resp_len = n;
			if (!ep_busy[data_ep]) {
				ep_rsc[data_ep] = xfer_rsc(post_ep0_data(ep0_dir_in, dn));
				ep_busy[data_ep] = true;
			}
			ep0_state = EP0_STATE_DATA;
		} else {
			ep0_state = EP0_STATE_STATUS_PENDING;
		}
	} else if (ep0_state == EP0_STATE_DATA) {
		/* An OUT data stage has landed: hand the core what arrived. */
		if (ep == EP0_OUT) {
			uint32_t got;

			dcache_flush((uintptr_t)MEM->ep0_data_trb, 16);
			got = IMX_GADGET_EP0_DATA_MAX -
			      (MEM->ep0_data_trb[2] & TRB_BUFSIZ_MASK);
			dcache_flush((uintptr_t)MEM->ep0_data, got ? got : 1);
			imx_gadget_ep0_data_out((const uint8_t *)MEM->ep0_data, got);
		}
		ep0_state = EP0_STATE_STATUS_PENDING;
	} else if (ep0_state == EP0_STATE_STATUS) {
		imx_gadget_ep0_status_done();
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

	/*
	 * Only for a transfer that has a data stage: after a STALL the host may
	 * still send the IN tokens that raise this, and answering them would
	 * leave the next SETUP handled as data.
	 */
	if (phase == DEPEVT_STATUS_CONTROL_DATA && ep0_state == EP0_STATE_SETUP &&
	    ep0_three_stage) {
		unsigned int data_ep = ep0_dir_in ? EP0_IN : EP0_OUT;
		uint32_t n = ep0_dir_in ? ep0_resp_len : IMX_GADGET_EP0_DATA_MAX;
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
			imx_gadget_bus_disconnected();
			break;
		case DEVT_USBRST:
			imx_gadget_ctrl_set_address(0);
			ep0_state = EP0_STATE_SETUP;
			ep1_in_busy = false;
			imx_gadget_bus_reset();
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
		imx_gadget_ctrl_tx_pump();
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
				imx_gadget_console_give((const uint8_t *)MEM->rx_buf,
							got);
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

/* Drain the interrupter-0 event ring. */
void imx_gadget_ctrl_service(void)
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

void imx_gadget_ctrl_disconnect(void)
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
