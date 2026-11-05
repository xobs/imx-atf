/*
 * osdyne's USB gadget, resident in BL31 (EL3).
 *
 * Ported from osdyne's EL1 gadget (os/kernel/src/platform/hal/imx95/dwc3.rs).
 * The controller is serviced from a Group-0 secure FIQ, so it keeps running
 * even while EL1/EL2 (osdyne) is CPU-bound, in an interrupt-disabled section,
 * or hung. This makes the USB console immune to osdyne's scheduling, hides it
 * from osdyne's tests, and lets the board be rebooted over USB even when
 * frozen.
 *
 * This file is the protocol: descriptors, vendor requests, the console rings
 * shared with osdyne, host memory access and the debug monitor.  Moving bytes
 * over the bus is the controller driver's job; see imx_gadget_controller.h for
 * the line between the two.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <arch.h>
#include <arch_helpers.h>
#include <bl31/ea_handle.h>
#include <bl31/interrupt_mgmt.h>
#include <common/debug.h>
#include <drivers/arm/gic_common.h>
#include <drivers/arm/gicv3.h>
#include <lib/cassert.h>
#include <lib/mmio.h>
#include <lib/xlat_tables/xlat_tables_v2.h>

#include <context.h>
#include <lib/el3_runtime/context_mgmt.h>
#include <plat/common/platform.h>

#include <ele_api.h>
#include <imx_gadget.h>
#include <imx_gadget_controller.h>
#include <platform_def.h>

/*
 * What each platform says about itself, in its platform_def.h:
 *
 * IMX_GADGET_CHIP          the chip as osdyne's targets name it
 * IMX_GADGET_PID           the USB product ID: the boot ROM's own, so that one
 *                          vid:pid names the board in the ROM and running
 * IMX_GADGET_CONSOLE_BASE  the DRAM window of the console rings shared with
 *                          osdyne, mapped non-cacheable and non-secure
 */
#if !defined(IMX_GADGET_CHIP) || !defined(IMX_GADGET_PID) || \
	!defined(IMX_GADGET_CONSOLE_BASE)
#error "platform_def.h must describe the gadget: IMX_GADGET_CHIP, _PID, _CONSOLE_BASE"
#endif

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
/*
 * Write one register of the stopped core: wValue picks it, numbered as
 * VENDOR_REQ_REGS and VENDOR_REQ_STATUS report them -- x0..x30, then 31 for
 * SP, 32 for PC and 33 for PSTATE -- and the OUT data stage is its new value.
 * STALLs unless the registers BL31 holds are the interrupted code's own; see
 * dbg_regs_writable().
 */
#define VENDOR_REQ_REGS_WRITE	0x10
#define DBG_REG_SP		31U
#define DBG_REG_PC		32U
#define DBG_REG_PSTATE		33U

#define INFO_MAGIC		"osdg"
#define INFO_VERSION		1U

#define INFO_CAP_CONSOLE	(1U << 0)	/* osdyne's console on EP1 IN */
#define INFO_CAP_MEMORY		(1U << 1)	/* VENDOR_REQ_MEM_READ/_WRITE */
#define INFO_CAP_REBOOT		(1U << 2)	/* VENDOR_REQ_REBOOT */
#define INFO_CAP_DEBUG_MONITOR	(1U << 3)	/* VENDOR_REQ_STOP..._DEBUG_CAPS */
#define INFO_CAP_CONSOLE_READING (1U << 4)	/* VENDOR_REQ_CONSOLE_READING */
#define INFO_CAP_HOLD		(1U << 5)	/* struct hold_request */
#define INFO_CAP_REGS_WRITE	(1U << 6)	/* VENDOR_REQ_REGS_WRITE */
#define INFO_CAP_HALT_AT_ENTRY	(1U << 7)	/* HOLD_AT_ENTRY */

/* Reply to VENDOR_REQ_GET_INFO.  Little-endian, naturally aligned, no padding. */
struct gadget_info {
	char magic[4];		/* INFO_MAGIC, not NUL-terminated */
	uint16_t version;	/* INFO_VERSION */
	uint16_t length;	/* sizeof(struct gadget_info) */
	uint32_t caps;		/* INFO_CAP_* */
	char chip[16];		/* IMX_GADGET_CHIP, NUL-padded */
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

/* -------------------------------------------------------------- descriptors */

static const uint8_t DEVICE_DESC[18] = {
	0x12, DESC_DEVICE, 0x00, 0x02, 0xff, 0x00, 0x00, 0x40,
	0xc9, 0x1f, IMX_GADGET_PID & 0xff, (IMX_GADGET_PID >> 8) & 0xff,
	0x00, 0x01, 0x01, 0x02, 0x03, 0x01,
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
/*
 * The chip UID as the boot ROM reports it (its first 8 bytes, in order), so
 * that one serial names a board whether it is in the ROM or running.  "0001"
 * if ELE gave none.  bLength is [0].
 */
extern struct ele_soc_info soc_info;

#define UID_BYTES	8U
static uint8_t str_serial[2U + (4U * UID_BYTES)] = {
	0x0a, DESC_STRING, '0', 0, '0', 0, '0', 0, '1', 0,
};

static void serial_from_uid(void)
{
	static const char digits[] = "0123456789ABCDEF";
	const uint8_t *uid = (const uint8_t *)soc_info.uid;
	unsigned int i;

	inv_dcache_range((uintptr_t)&soc_info, sizeof(soc_info));
	if ((soc_info.uid[0] | soc_info.uid[1]) == 0U) {
		return;
	}
	for (i = 0U; i < UID_BYTES; i++) {
		str_serial[2U + (4U * i)] = (uint8_t)digits[uid[i] >> 4];
		str_serial[3U + (4U * i)] = 0U;
		str_serial[4U + (4U * i)] = (uint8_t)digits[uid[i] & 0xfU];
		str_serial[5U + (4U * i)] = 0U;
	}
	str_serial[0] = (uint8_t)sizeof(str_serial);
}
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

/* ----------------------------------------------------------- console rings */

/*
 * EL1<->EL3 console channel: a pair of SPSC byte rings in a fixed non-cacheable
 * DRAM window shared with osdyne (EL1). osdyne writes console output into tx_buf
 * and reads host input from rx_buf; BL31 drains tx_buf to EP1 IN and fills
 * rx_buf from EP1 OUT. Non-cacheable so no cross-EL cache maintenance is
 * needed. Ring sizes are powers of two.
 */
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

#define CHAN ((volatile struct console_chan *)IMX_GADGET_CONSOLE_BASE)

/*
 * A host that starts the payload itself asks BL31 to hold BL33 back until it
 * is reading the console: a probe's reset-halt, for a board booted over USB.
 * Without it osdyne writes its first output, and drops it, before any host
 * can be reading. The host finds the record by its magic.
 */
#define HOLD_MAGIC	"osdyne-hold-v1"

struct hold_request {
	char magic[16];		/* HOLD_MAGIC, zero-padded */
	uint32_t hold;		/* HOLD_*: what to hold BL33 back for */
	uint32_t reserved;
};

/* Until the host reads the console. */
#define HOLD_FOR_CONSOLE	1U
/*
 * Stopped in the debug monitor at BL33's first instruction, as a probe's
 * reset-halt would leave it: see imx_gadget_halt_at_entry().
 */
#define HOLD_AT_ENTRY		2U

/*
 * Written over SDP by odd, which starts the payload itself, before it jumps to
 * the ATF container: the container is hashed, so the request cannot ride in
 * the image. It sits in spare space at the end of the console window, which
 * the channel never reaches, and is consumed at gadget init so that a later
 * boot without the host is not held.
 */
#define HOLD_REQUEST_BASE	(IMX_GADGET_CONSOLE_BASE + 0xff00UL)
#define HOLD_REQUEST ((volatile struct hold_request *)HOLD_REQUEST_BASE)

CASSERT(sizeof(struct console_chan) <= 0xff00U, assert_console_chan_fits);

/* The host's request, taken at gadget init: one of the HOLD_* modes, or 0. */
static uint32_t hold_mode;

uint32_t imx_gadget_console_take(uint8_t *dst, uint32_t max)
{
	uint32_t n = 0U;

	if ((CHAN->flags & CHAN_CONFIGURED) == 0U) {
		return 0U;
	}
	while ((n < max) && (CHAN->tx_tail != CHAN->tx_head)) {
		dst[n++] = CHAN->tx_buf[CHAN->tx_tail & (TX_RING_SZ - 1)];
		CHAN->tx_tail = CHAN->tx_tail + 1;
	}

	return n;
}

void imx_gadget_console_give(const uint8_t *src, uint32_t len)
{
	uint32_t i;

	for (i = 0U; i < len; i++) {
		if ((CHAN->rx_head - CHAN->rx_tail) >= RX_RING_SZ) {
			break; /* ring full; drop */
		}
		CHAN->rx_buf[CHAN->rx_head & (RX_RING_SZ - 1)] = src[i];
		CHAN->rx_head = CHAN->rx_head + 1;
	}
}

/* ------------------------------------------------------- debug monitor */

/*
 * Set by the host via VENDOR_REQ_STOP.  The FIQ handler parks in EL3 while it
 * is set, still servicing the controller, so the host can inspect and resume.
 * Volatile because it is written from the EP0 handler and read by the loop
 * that the same handler runs inside.
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
static volatile bool gadget_ready;

/*
 * True once the controller driver has been started.  A reset before then --
 * a panic early in BL31, say -- leaves the controller as the previous stage
 * left it: the driver's disconnect would run on a timer and maybe a clock
 * that are not set up yet, and a fault there turns the reset into a hang.
 */
static bool gadget_started;

/* The controller's GIC interrupt, delivered as a Group-0 FIQ to EL3. */
static unsigned int gadget_intid;

/* The EL3 interrupts that are not the gadget's go here; see imx_gadget_chain_fiq(). */
static interrupt_type_handler_t chained_fiq;

/*
 * The non-secure context TF-A saved when it took the interrupt that brought us
 * into EL3.  This is what makes a stub debugger practical here: the registers
 * of whatever was running are already spilled, so reporting them costs nothing.
 */
static void *dbg_ns_context;

/*
 * The registers of the code that took a debug exception, as osdyne hands them
 * over with IMX_SIP_DEBUG_FAULT.  The exception reaches osdyne's handler, not
 * EL3, so by the time osdyne reports it the context TF-A saved at the SMC is
 * the handler's.  osdyne passes the physical address of one of these in x4,
 * and while the core is stopped the monitor serves register reads and writes
 * from it instead; on resume it is written back, and osdyne resumes the
 * interrupted code from it.
 *
 * The layout is osdyne's DebugFrame (os/kernel/src/platform/arch/aarch64/
 * context.rs).  Little-endian:
 *
 *   0    magic, DBG_FRAME_MAGIC
 *   4    size of the frame in bytes; later versions only append
 *   8    x0..x30
 *   256  sp, the stack pointer the interrupted code was using
 *   264  pc
 *   272  pstate, in SPSR form
 */
#define DBG_FRAME_MAGIC		U(0x6664736f)	/* "osdf" */

struct dbg_frame {
	uint32_t magic;
	uint32_t size;
	uint64_t x[31];
	uint64_t sp;
	uint64_t pc;
	uint64_t pstate;
};

CASSERT(sizeof(struct dbg_frame) == 280U, assert_dbg_frame_size);

/*
 * A copy of the payload's frame while the stop it came with lasts, and where
 * it goes back to; dbg_frame_addr is 0 when the stop has none.
 */
static struct dbg_frame dbg_frame;
static uintptr_t dbg_frame_addr;

/* Layout of the VENDOR_REQ_STATUS reply.  Little-endian, packed. */
struct dbg_status {
	uint32_t stopped;
	uint32_t reason;
	uint64_t pc;    /* where the debuggee will resume */
	uint64_t spsr;  /* its PSTATE: exception level and interrupt masks */
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

	if (dbg_frame_addr != 0U) {
		st.pc = dbg_frame.pc;
		st.spsr = dbg_frame.pstate;
		st.sp = dbg_frame.sp;
		memcpy(out, &st, sizeof(st));
		return (uint32_t)sizeof(st);
	}

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

	if (dbg_frame_addr != 0U) {
		memcpy(out, dbg_frame.x, sizeof(dbg_frame.x));
		return (uint32_t)sizeof(dbg_frame.x);
	}

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

/*
 * Whether VENDOR_REQ_REGS_WRITE may change the registers.  Only when they are
 * the interrupted code's own: a host stop, an external abort, or a stop the
 * payload reported with a frame.  Without a frame, such a stop holds the
 * payload's exception handler's registers, and writing those would change
 * nothing the faulting code sees.
 */
static bool dbg_regs_writable(void)
{
	if (!dbg_stopped || (dbg_stop_reason == DBG_REASON_EL3_PANIC)) {
		return false;
	}

	if (dbg_frame_addr != 0U) {
		return true;
	}

	return (dbg_ns_context != NULL) && (dbg_fault_pc == 0U);
}

/* The register VENDOR_REQ_REGS_WRITE's data stage is for. */
static uint16_t dbg_reg_pending;

static void dbg_write_reg(unsigned int reg, uint64_t value)
{
	el3_state_t *st;
	uint64_t spsr;

	if (dbg_frame_addr != 0U) {
		if (reg < 31U) {
			dbg_frame.x[reg] = value;
		} else if (reg == DBG_REG_SP) {
			dbg_frame.sp = value;
		} else if (reg == DBG_REG_PC) {
			dbg_frame.pc = value;
		} else if (reg == DBG_REG_PSTATE) {
			dbg_frame.pstate = (dbg_frame.pstate & ~ULL(0xffffffff)) |
					   (value & ULL(0xffffffff));
		}
		return;
	}

	st = get_el3state_ctx(dbg_ns_context);
	spsr = read_ctx_reg(st, CTX_SPSR_EL3);

	if (reg < 31U) {
		write_ctx_reg(get_gpregs_ctx(dbg_ns_context),
			      CTX_GPREG_X0 + (reg * 8U), value);
	} else if (reg == DBG_REG_SP) {
		/* The stack dbg_fill_status reports, for the same reason. */
		if ((spsr & 0xfU) == 0x5U) {
			write_sp_el1(value);
		} else {
			write_ctx_reg(get_gpregs_ctx(dbg_ns_context),
				      CTX_GPREG_SP_EL0, value);
		}
	} else if (reg == DBG_REG_PC) {
		write_ctx_reg(st, CTX_ELR_EL3, value);
	} else if (reg == DBG_REG_PSTATE) {
		/* PSTATE is 32 bits; keep SPSR_EL3's upper half as it is. */
		write_ctx_reg(st, CTX_SPSR_EL3,
			      (spsr & ~ULL(0xffffffff)) | (value & ULL(0xffffffff)));
	}
}

/* ------------------------------------------------- self-hosted debug */

/*
 * Self-hosted debug exceptions cannot be taken to EL3.  The architecture sends
 * them to EL1, or to EL2 when a hypervisor sets MDCR_EL2.TDE -- there is no
 * equivalent bit in MDCR_EL3, which governs only external debug and secure
 * debug.  So unlike stopping and unlike external aborts, breakpoints do need
 * the payload's cooperation: EL3 owns the debug registers (they are EL1
 * system registers, writable from EL3) and the payload routes the exception
 * back through IMX_SIP_DEBUG_FAULT.
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
 * is right depends on where the debuggee runs -- osdyne is at EL2 on the i.MX
 * and at EL1 on others -- so it is derived from the debuggee's own SPSR rather
 * than fixed.  Getting it wrong fails silently: the breakpoint is programmed,
 * enabled, and simply never matches.
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

	if (dbg_frame_addr != 0U) {
		spsr = dbg_frame.pstate;
	} else if (dbg_ns_context != NULL) {
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
 * watchpoints is what the Cortex-A55 has; the tables are sized for the
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

	NOTICE("gadget: self-hosted debug armed (%u breakpoints, %u watchpoints)\n",
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
 * world does not run again until the host resumes it.  The controller is
 * serviced by polling, which is what keeps the console and every other vendor
 * request alive while the target is stopped -- including, crucially, when we
 * were entered from a fault rather than from an interrupt.
 *
 * Returns true when the host asked to step over the faulting instruction.
 */
static bool dbg_park(void)
{
	bool skip;
	bool held = false;

	while (dbg_stop_requested) {
		held = true;
		dbg_stopped = true;
		imx_gadget_ctrl_service();
	}

	skip = dbg_resume_skip;

	/*
	 * Only a resume from a stop arms anything.  Every gadget FIQ passes
	 * through here, and it can land anywhere -- an FIQ routed to EL3 is
	 * taken even where the payload masks interrupts, such as in the
	 * exception handler a fault report has just returned to.  A step still
	 * pending from the last resume, armed again there, would step that
	 * handler instead of the code the host asked to step.
	 *
	 * With a frame, the PSTATE to resume with is the interrupted code's,
	 * which osdyne restores from the frame; SPSR_EL3 only returns to its
	 * handler, which runs with debug exceptions masked either way.
	 */
	if (held && ((dbg_frame_addr != 0U) || (dbg_ns_context != NULL))) {
		el3_state_t *st = NULL;
		uint64_t spsr;

		if (dbg_frame_addr != 0U) {
			spsr = dbg_frame.pstate;
		} else {
			st = get_el3state_ctx(dbg_ns_context);
			spsr = read_ctx_reg(st, CTX_SPSR_EL3);
		}

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

		if (dbg_frame_addr != 0U) {
			dbg_frame.pstate = spsr;
			memcpy((void *)dbg_frame_addr, &dbg_frame,
			       sizeof(dbg_frame));
		} else {
			write_ctx_reg(st, CTX_SPSR_EL3, spsr);
		}
	}

	dbg_frame_addr = 0U;

	dbg_resume_skip = false;
	dbg_stopped = false;
	dbg_stop_reason = DBG_REASON_RUNNING;
	dbg_fault_esr = 0U;
	dbg_fault_far = 0U;
	dbg_fault_pc = 0U;

	return skip;
}

/*
 * Take a copy of the frame the payload passed, if it passed one: x4 is not
 * part of the call for a payload that predates frames, so whatever it holds
 * has to prove itself.  It must be non-secure, writable normal memory, so
 * that a bad address cannot have the monitor write the host's values into
 * BL31's own memory, and it must start with the magic.
 */
static void dbg_frame_load(uint64_t addr)
{
	uintptr_t page;

	dbg_frame_addr = 0U;

	if ((addr == 0U) || ((addr & 7U) != 0U) ||
	    ((addr + sizeof(dbg_frame)) < addr)) {
		return;
	}

	for (page = addr & ~0xFFFUL; page < (addr + sizeof(dbg_frame));
	     page += 0x1000UL) {
		uint32_t attr;

		if ((xlat_get_mem_attributes(page, &attr) != 0) ||
		    (MT_TYPE(attr) != MT_MEMORY) || ((attr & MT_RW) == 0U) ||
		    ((attr & MT_NS) == 0U)) {
			return;
		}
	}

	memcpy(&dbg_frame, (const void *)addr, sizeof(dbg_frame));
	if ((dbg_frame.magic != DBG_FRAME_MAGIC) ||
	    (dbg_frame.size < sizeof(dbg_frame))) {
		return;
	}

	dbg_frame_addr = (uintptr_t)addr;
}

bool imx_gadget_debug_fault(unsigned int reason, void *handle,
			    uint64_t esr, uint64_t far, uint64_t pc,
			    uint64_t frame)
{
	/*
	 * An SMC handler is not given the interrupted context, but TF-A has
	 * saved it by the time we run, so ask for it rather than reporting a
	 * stale one left over from the last interrupt.
	 */
	if (!gadget_ready) {
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
	dbg_frame_load(frame);

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

bool imx_gadget_debug_fault_smc(uint64_t esr, uint64_t far, uint64_t pc,
				uint64_t frame)
{
	/*
	 * Faults the architecture delivers to EL1 never reach EL3 on their own,
	 * so this is osdyne's way into the same monitor that external aborts
	 * reach by themselves.  It blocks until the host resumes.
	 *
	 * Each debug class has a lower-EL and a same-EL encoding.  osdyne
	 * debugs itself, so in practice these arrive as the same-EL form
	 * (0x31/0x33/0x35); the lower-EL ones are accepted for a kernel that
	 * one day runs something beneath it.
	 *
	 * @pc is the faulting PC and @frame the physical address of the
	 * faulting code's registers (struct dbg_frame), or 0.  The registers
	 * saved at this SMC are osdyne's handler's, so the frame is what the
	 * host is shown.  A payload that predates frames leaves @frame
	 * undefined; dbg_frame_load() checks it.
	 */
	unsigned int reason;

	switch ((unsigned int)((esr >> 26) & 0x3fU)) {
	case 0x30U:
	case 0x31U:
		reason = DBG_REASON_BREAKPOINT;
		break;
	case 0x32U:
	case 0x33U:
		reason = DBG_REASON_STEP;
		break;
	case 0x34U:
	case 0x35U:
		reason = DBG_REASON_WATCHPOINT;
		break;
	case 0x3cU:
		reason = DBG_REASON_BRK;
		break;
	default:
		reason = DBG_REASON_PAYLOAD;
		break;
	}

	return imx_gadget_debug_fault(reason, NULL, esr, far, pc, frame);
}

/*
 * With HANDLE_EA_EL3_FIRST_NS, SCR_EL3.EA routes external aborts and SErrors
 * from the non-secure world to EL3 instead of to osdyne.  TF-A's default
 * handler prints and panics, which leaves a dead board.  Park in the USB
 * monitor instead, so the crash can be inspected over the same pipe that
 * carries the console.
 */
void plat_ea_handler(unsigned int ea_reason, uint64_t syndrome, void *cookie,
		     void *handle, uint64_t flags)
{
	unsigned int reason;
	uint64_t far = 0U;

	if (!gadget_ready) {
		/* No monitor to park in yet: die the way TF-A would have. */
		plat_default_ea_handler(ea_reason, syndrome, cookie, handle,
					flags);
		return;
	}

	if (ea_reason == ERROR_EA_ASYNC) {
		/* SError: asynchronous, so FAR_EL3 means nothing. */
		reason = DBG_REASON_EA_ASYNC;
	} else {
		reason = DBG_REASON_EA_SYNC;
		far = read_far_el3();
	}

	ERROR("gadget: external abort (reason %u, esr 0x%llx) -- parking in the debug monitor\n",
	      ea_reason, (unsigned long long)syndrome);

	/*
	 * Returning resumes the faulting instruction, which will simply abort
	 * again -- so honour a host "resume --skip" by stepping over it.  The
	 * abort was synchronous and precise, so ELR_EL3 is the aborting
	 * instruction and A64 instructions are all four bytes.
	 */
	if (imx_gadget_debug_fault(reason, handle, syndrome, far, 0U, 0U) &&
	    (reason == DBG_REASON_EA_SYNC)) {
		el3_state_t *st = get_el3state_ctx(handle);

		write_ctx_reg(st, CTX_ELR_EL3,
			      read_ctx_reg(st, CTX_ELR_EL3) + 4U);
	}
}

void __dead2 imx_system_reset(void);

/*
 * A BL31 panic is normally a silently dead board: no console, no reset, and
 * nothing left to ask.  The gadget is serviced by polling, so it can still run
 * here; park and let the host look at whatever osdyne was doing.
 */
void __dead2 plat_panic_handler(void)
{
	while (gadget_ready) {
		(void)imx_gadget_debug_fault(DBG_REASON_EL3_PANIC, NULL,
					     0U, 0U, 0U, 0U);
	}

	/* Panicked before the gadget came up; nothing can be inspected. */
	imx_system_reset();

	while (true) {
	}
}

/* --------------------------------------------------------------- handle_setup */

/* Stage a device->host reply into @buf; return bytes to send. */
static uint32_t stage_reply(uint8_t *buf, const uint8_t *src, size_t src_len,
			    uint16_t wlen)
{
	size_t n = src_len;

	if (n > wlen) {
		n = wlen;
	}
	if (n > IMX_GADGET_EP0_DATA_MAX) {
		n = IMX_GADGET_EP0_DATA_MAX;
	}
	memcpy(buf, src, n);

	return (uint32_t)n;
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
	_Static_assert(sizeof(IMX_GADGET_CHIP) <= sizeof(info.chip),
		       "IMX_GADGET_CHIP is too long for gadget_info.chip");

	memset(&info, 0, sizeof(info));
	memcpy(info.magic, INFO_MAGIC, sizeof(info.magic));
	info.version = INFO_VERSION;
	info.length = (uint16_t)sizeof(info);
	info.caps = INFO_CAP_CONSOLE | INFO_CAP_MEMORY | INFO_CAP_REBOOT |
		    INFO_CAP_DEBUG_MONITOR | INFO_CAP_CONSOLE_READING |
		    INFO_CAP_HOLD | INFO_CAP_REGS_WRITE | INFO_CAP_HALT_AT_ENTRY;
	memcpy(info.chip, IMX_GADGET_CHIP, sizeof(IMX_GADGET_CHIP) - 1U);

	memcpy(out, &info, sizeof(info));
	return (uint32_t)sizeof(info);
}

static bool configured;
static bool config_pending;
static bool reboot_pending;
/* A validated MEM_WRITE is staged here between its SETUP and its OUT data stage. */
static bool mem_write_pending;
static uintptr_t mem_write_addr;
static uint32_t mem_write_len;

/*
 * Decode a SETUP packet. Returns >= 0 for the data/no-data length (staged in
 * @buf), or -1 to STALL.
 */
static int handle_setup(uint8_t *buf, uint8_t bmreq, uint8_t brequest,
			uint16_t wvalue, uint16_t windex, uint16_t wlen)
{
	uint32_t req_type = bmreq & 0x60;

	if (req_type == 0x00) {
		switch (brequest) {
		case REQ_GET_DESCRIPTOR: {
			uint8_t dtype = wvalue >> 8;
			uint8_t didx = wvalue & 0xff;
			switch (dtype) {
			case DESC_DEVICE:
				return stage_reply(buf, DEVICE_DESC, sizeof(DEVICE_DESC), wlen);
			case DESC_DEVICE_QUALIFIER:
				return stage_reply(buf, DEVICE_QUALIFIER_DESC,
						   sizeof(DEVICE_QUALIFIER_DESC), wlen);
			case DESC_OTHER_SPEED_CONFIG:
				return stage_reply(buf, OTHER_SPEED_CONFIG_DESC,
						   sizeof(OTHER_SPEED_CONFIG_DESC), wlen);
			case DESC_CONFIG:
				return stage_reply(buf, CONFIG_DESC, sizeof(CONFIG_DESC), wlen);
			case DESC_STRING:
				switch (didx) {
				case 0:
					return stage_reply(buf, STR_LANGID, sizeof(STR_LANGID), wlen);
				case 1:
					return stage_reply(buf, STR_MANUFACTURER,
							   sizeof(STR_MANUFACTURER), wlen);
				case 2:
					return stage_reply(buf, STR_PRODUCT, sizeof(STR_PRODUCT), wlen);
				case 3:
					return stage_reply(buf, str_serial, str_serial[0], wlen);
				case MS_OS_STRING_INDEX:
					return stage_reply(buf, MS_OS_STRING, sizeof(MS_OS_STRING), wlen);
				default:
					return -1;
				}
			default:
				return -1;
			}
		}
		case REQ_SET_ADDRESS:
			imx_gadget_ctrl_set_address((uint8_t)(wvalue & 0x7f));
			return 0;
		case REQ_SET_CONFIGURATION:
			if (wvalue == 0) {
				configured = false;
				CHAN->flags = 0U;
			}
			return 0;
		case REQ_GET_CONFIGURATION:
			buf[0] = configured ? 1 : 0;
			return wlen < 1 ? wlen : 1;
		case REQ_GET_STATUS:
			buf[0] = 0;
			buf[1] = 0;
			return wlen < 2 ? wlen : 2;
		case REQ_SET_FEATURE:
		case REQ_CLEAR_FEATURE:
			return 0;
		default:
			return -1;
		}
	} else if (req_type == 0x40) {
		if (brequest == MS_VENDOR_CODE && windex == 0x0004)
			return stage_reply(buf, MS_COMPAT_ID, sizeof(MS_COMPAT_ID), wlen);
		if (brequest == VENDOR_REQ_REBOOT && wvalue == REBOOT_MAGIC)
			return 0;
		if (brequest == VENDOR_REQ_CONSOLE_READING && (bmreq & 0x80U) == 0U &&
		    wvalue <= 1U && wlen == 0U)
			return 0;
		/*
		 * Accept these here so the control transfer is not stalled; the
		 * flags themselves are set once the SETUP is decoded, or, for a
		 * reboot, in the status stage, so the device has already
		 * acknowledged the request by the time it acts on it.
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
			uint32_t n = dbg_fill_status(buf);

			return wlen < n ? wlen : n;
		}
		if (brequest == VENDOR_REQ_REGS && (bmreq & 0x80U) != 0U) {
			uint32_t n = dbg_fill_regs(buf);

			return wlen < n ? wlen : n;
		}
		if (brequest == VENDOR_REQ_REGS_WRITE && (bmreq & 0x80U) == 0U) {
			if ((wlen != sizeof(uint64_t)) || (wvalue > DBG_REG_PSTATE) ||
			    !dbg_regs_writable())
				return -1;
			dbg_reg_pending = wvalue;
			dbg_arg_pending = brequest;
			return (int)wlen;
		}
		if (brequest == VENDOR_REQ_DEBUG_CAPS && (bmreq & 0x80U) != 0U) {
			uint32_t n = dbg_fill_caps(buf);

			return wlen < n ? wlen : n;
		}
		if (brequest == VENDOR_REQ_GET_INFO && (bmreq & 0x80U) != 0U) {
			uint32_t n = fill_info(buf);

			return wlen < n ? wlen : n;
		}
		if (brequest == VENDOR_REQ_MEM_READ && (bmreq & 0x80U) != 0U) {
			/* device->host: read [addr, addr+wlen) into buf. */
			uintptr_t addr = ((uintptr_t)wvalue << 16) | windex;
			uint32_t n = wlen;

			if (n > IMX_GADGET_EP0_DATA_MAX)
				n = IMX_GADGET_EP0_DATA_MAX;
			if (!mem_range_ok(addr, n, false))
				return -1;
			mem_read_range(addr, buf, n);
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

			if (n == 0U || n > IMX_GADGET_EP0_DATA_MAX)
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

int imx_gadget_setup(const uint8_t s[8], uint8_t *buf)
{
	uint8_t bmreq = s[0];
	uint8_t brequest = s[1];
	uint16_t wvalue = s[2] | (s[3] << 8);
	uint16_t windex = s[4] | (s[5] << 8);
	uint16_t wlen = s[6] | (s[7] << 8);
	int n;

	/* A new SETUP supersedes a data stage that never arrived. */
	mem_write_pending = false;
	dbg_arg_pending = 0U;

	n = handle_setup(buf, bmreq, brequest, wvalue, windex, wlen);
	if ((n < 0) || (wlen != 0U)) {
		return n;
	}

	/* A request without a data stage: act on it now. */
	if ((bmreq & 0x60U) == 0x00U && brequest == REQ_SET_CONFIGURATION &&
	    wvalue != 0)
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
		dbg_resume_skip = (wvalue == RESUME_SKIP_FAULTING_INSN);
		dbg_stop_requested = false;
	}
	if (bmreq == 0x40 && brequest == VENDOR_REQ_STEP) {
		/*
		 * Stepping needs the same enables a breakpoint does -- KDE
		 * above all, without which a same-EL debug exception is
		 * simply not taken.
		 */
		dbg_enable_traps();
		/* Leaving the park arms the step; see dbg_park. */
		dbg_step_pending = true;
		dbg_stop_requested = false;
	}
	if (bmreq == 0x40 && brequest == VENDOR_REQ_CLEAR_BP && wvalue != 0U)
		(void)dbg_clear_bp(0U, true);
	if (bmreq == 0x40 && brequest == VENDOR_REQ_CLEAR_WP && wvalue != 0U)
		(void)dbg_clear_wp(0U, true);

	return n;
}

void imx_gadget_ep0_data_out(const uint8_t *buf, uint32_t len)
{
	/* Apply a validated MEM_WRITE now that its OUT data has landed. */
	if (mem_write_pending) {
		if (len == mem_write_len) {
			mem_write_range(mem_write_addr, buf, mem_write_len);
		}
		mem_write_pending = false;
	}

	if (dbg_arg_pending != 0U) {
		uint64_t addr;

		if (len < sizeof(addr)) {
			dbg_arg_pending = 0U;
			return;
		}
		memcpy(&addr, buf, sizeof(addr));

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
		case VENDOR_REQ_REGS_WRITE:
			dbg_write_reg(dbg_reg_pending, addr);
			break;
		default:
			break;
		}
		dbg_arg_pending = 0U;
	}
}

static void __dead2 gadget_reboot(void)
{
	/* imx_system_reset() disconnects the gadget for us before resetting. */
	imx_system_reset();
	while (true) {
	}
}

void imx_gadget_ep0_status_done(void)
{
	if (reboot_pending)
		gadget_reboot();
	if (config_pending) {
		imx_gadget_ctrl_configure();
		configured = true;
		CHAN->flags = CHAN_CONFIGURED;
		config_pending = false;
		/*
		 * Send what osdyne wrote before there was a host to send it to.
		 * Nothing else would until osdyne writes more or kicks again: a
		 * ChipIdea raises nothing for the host polling an idle endpoint.
		 */
		imx_gadget_ctrl_tx_pump();
	}
}

void imx_gadget_bus_reset(void)
{
	configured = false;
	config_pending = false;
	mem_write_pending = false;
	dbg_arg_pending = 0U;
	CHAN->flags = 0U;
}

void imx_gadget_bus_disconnected(void)
{
	CHAN->flags &= ~CHAN_HOST_READING;
}

/* ------------------------------------------------------------- entry points */

void imx_gadget_halt_at_entry(void)
{
	if ((hold_mode != HOLD_AT_ENTRY) || !gadget_ready)
		return;

	/*
	 * BL31 has built BL33's entry context by now, so the monitor stops on
	 * its first instruction, and a host can set registers before it runs.
	 */
	NOTICE("gadget: stopping at BL33's entry for the host\n");
	dbg_ns_context = cm_get_context(NON_SECURE);
	dbg_stop_reason = DBG_REASON_HOST;
	dbg_stop_requested = true;
	(void)dbg_park();
}

void imx_gadget_hold_for_host(void)
{
	if (hold_mode != HOLD_FOR_CONSOLE)
		return;

	NOTICE("gadget: holding BL33 until the host reads the console\n");
	while ((CHAN->flags & CHAN_HOST_READING) == 0U)
		imx_gadget_ctrl_service();
	NOTICE("gadget: the host is reading the console; starting BL33\n");
}

void imx_gadget_disconnect(void)
{
	if (gadget_started) {
		imx_gadget_ctrl_disconnect();
	}
}

/*
 * SMC entry (IMX_SIP_USB_CONSOLE): osdyne poked us after queueing console output
 * into the TX ring. Drain it to EP1 IN.
 */
void imx_gadget_console_kick(void)
{
	/*
	 * Mask FIQ so the controller's FIQ handler can't preempt this SMC and
	 * race on the controller/DMA state. Restore the caller's DAIF (the SMC
	 * may have been entered with FIQ already masked).
	 */
	u_register_t daif = read_daif();

	write_daifset(DAIF_FIQ_BIT);
	isb();
	imx_gadget_ctrl_tx_pump();
	write_daif(daif);
	isb();
}

/*
 * EL3 FIQ handler for the controller's interrupt (Group-0 secure). Runs the
 * gadget, preempting EL1/EL2 unconditionally.
 */
static uint64_t imx_gadget_fiq_handler(uint32_t id, uint32_t flags,
				       void *handle, void *cookie)
{
	uint32_t irq;

	/*
	 * Look before acknowledging: an interrupt that is not the gadget's
	 * goes to the chained handler, which acknowledges it itself.
	 */
	if ((chained_fiq != NULL) &&
	    (plat_ic_get_pending_interrupt_id() != gadget_intid)) {
		return chained_fiq(id, flags, handle, cookie);
	}

	irq = plat_ic_acknowledge_interrupt();

	if (irq == gadget_intid) {
		/*
		 * TF-A hands us the context of whatever it interrupted, which
		 * for our purposes is the debuggee.  Remember it so the host
		 * can read its PC and registers.
		 */
		dbg_ns_context = handle;

		imx_gadget_ctrl_service();

		/*
		 * Park if the host has asked us to stop.  End-of-interrupt is
		 * deliberately still outstanding, which keeps the GIC priority
		 * masked so no nested FIQ re-enters the monitor.
		 */
		(void)dbg_park();
	}

	if (irq < 1022U)
		plat_ic_end_of_interrupt(irq);

	return 0;
}

void imx_gadget_chain_fiq(interrupt_type_handler_t handler)
{
	chained_fiq = handler;
}

/*
 * Make the controller's interrupt a Group-0 secure FIQ to EL3, so the gadget
 * is serviced even while EL1/EL2 is CPU-bound or hung.
 */
static void gadget_route_fiq(unsigned int intid)
{
	unsigned int cpu = plat_my_core_pos();
	uint32_t flags;
	int rc;

	gadget_intid = intid;

	gicv3_set_interrupt_group(intid, cpu, INTR_GROUP0);
	gicv3_set_interrupt_priority(intid, cpu, GIC_HIGHEST_SEC_PRIORITY);

	/*
	 * On a warm reset the GIC distributor is NOT reset, so the interrupt
	 * can be left stuck in the Active state from the previous boot's FIQ.
	 * gicv3_distif_init() clears enable/pending but not Active, and the GIC
	 * never re-delivers an already-Active interrupt -- so the console FIQ
	 * fires on a cold boot but never on a warm one. Clear both Pending and
	 * Active before enabling it. (It is an SPI, so its Active state lives
	 * in the distributor: GICD_ICACTIVER.)
	 */
	gicv3_clear_interrupt_pending(intid, cpu);
	mmio_write_32(PLAT_GICD_BASE + GICD_ICACTIVER + (intid / 32U) * 4U,
		      1U << (intid % 32U));

	gicv3_enable_interrupt(intid, cpu);

	flags = 0;
	set_interrupt_rm_flag(flags, NON_SECURE);
	rc = register_interrupt_type_handler(INTR_TYPE_EL3,
					     imx_gadget_fiq_handler, flags);
	if (rc != 0)
		panic();
}

void imx_gadget_init(unsigned int intid)
{
	serial_from_uid();

	/* Take, and consume, the host's request to hold BL33 back. */
	hold_mode = (memcmp((const void *)(uintptr_t)HOLD_REQUEST->magic,
			    HOLD_MAGIC, sizeof(HOLD_MAGIC)) == 0) ?
		    HOLD_REQUEST->hold : 0U;
	memset((void *)(uintptr_t)HOLD_REQUEST, 0, sizeof(struct hold_request));

	/* Initialise the EL1<->EL3 console channel before any traffic. */
	CHAN->tx_head = 0;
	CHAN->tx_tail = 0;
	CHAN->rx_head = 0;
	CHAN->rx_tail = 0;
	CHAN->flags = 0U;
	CHAN->magic = CONSOLE_CHAN_MAGIC;

	gadget_route_fiq(intid);

	gadget_started = true;
	imx_gadget_ctrl_start();

	gadget_ready = true;
}
