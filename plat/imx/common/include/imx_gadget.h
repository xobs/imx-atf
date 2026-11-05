/*
 * osdyne's USB gadget, resident in BL31 (EL3): osdyne's console, host memory
 * access and a debug monitor, over one USB device. See imx_gadget.c.
 *
 * The protocol and the monitor are shared by every i.MX that runs the gadget;
 * each SoC supplies a driver for its USB controller (imx_gadget_controller.h).
 */
#ifndef IMX_GADGET_H
#define IMX_GADGET_H

#include <cdefs.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * Bring the gadget up. Call once from bl31_platform_setup, after the GIC is
 * initialised and the SoC info (whose UID is the serial) has been read.
 * @intid is the controller's GIC interrupt, which becomes a Group-0 FIQ to EL3.
 */
void imx_gadget_init(unsigned int intid);

/*
 * Returns once the host is reading the console, if the host asked BL31 to hold
 * BL33 back until then; at once otherwise.
 */
void imx_gadget_hold_for_host(void);

/*
 * Stops BL33 at its first instruction, in the debug monitor, if the host that
 * loaded this image asked for that. Call once BL33's entry context exists.
 */
void imx_gadget_halt_at_entry(void);

/* Drain the console TX ring to the host; call from the IMX_SIP_USB_CONSOLE SMC. */
void imx_gadget_console_kick(void);

/*
 * Soft-disconnect the gadget from the bus, so a reset leaves USB clean for the
 * next stage's serial-download gadget. Call from every reset path (see
 * imx_system_reset). Does nothing if the gadget was never brought up.
 */
void imx_gadget_disconnect(void);

/*
 * Why the debuggee is stopped, reported by the host's "status" request.
 * Anything other than RUNNING/HOST is a fault: the target did not stop
 * because it was asked to, it stopped because it broke.  These values are
 * protocol, shared with the host tooling and the stm32mp2 monitor.
 */
#define DBG_REASON_RUNNING	0U
#define DBG_REASON_HOST		1U
#define DBG_REASON_EA_SYNC	2U /* synchronous external abort from NS */
#define DBG_REASON_EA_ASYNC	3U /* SError from NS */
#define DBG_REASON_PAYLOAD	4U /* osdyne reported its own fault */
#define DBG_REASON_EL3_PANIC	5U /* BL31 itself panicked */
#define DBG_REASON_STEP		6U /* completed a single-step */
#define DBG_REASON_BREAKPOINT	7U /* hardware breakpoint */
#define DBG_REASON_WATCHPOINT	8U /* hardware watchpoint */
#define DBG_REASON_BRK		9U /* the debuggee executed BRK */

/*
 * Monitor-mode fault entry.  Park the core in EL3 servicing USB instead of
 * letting the fault be fatal, so a broken system stays inspectable.  Blocks
 * until the host resumes, then returns true if the host asked to step over the
 * faulting instruction -- the caller owns the PC, so the caller applies it.
 *
 * @handle is the saved non-secure context; NULL asks TF-A for the current one.
 * @pc, when non-zero, overrides the reported PC: for a fault reported by
 * osdyne the interesting address is where it faulted, not where its handler
 * happened to call us from.
 * @frame, when non-zero, is the physical address of the registers of the code
 * that faulted, as osdyne hands them over (struct dbg_frame in imx_gadget.c).
 * The host is shown, and may change, those rather than the handler's; they
 * are written back before this returns.  It is checked before it is trusted.
 */
bool imx_gadget_debug_fault(unsigned int reason, void *handle,
			    uint64_t esr, uint64_t far, uint64_t pc,
			    uint64_t frame);

/*
 * IMX_SIP_DEBUG_FAULT: osdyne reporting a debug exception the architecture
 * delivered to EL1, with x1..x4 as the SMC passed them.  Returns whether the
 * host asked to step over the faulting instruction.
 */
bool imx_gadget_debug_fault_smc(uint64_t esr, uint64_t far, uint64_t pc,
				uint64_t frame);

#endif /* IMX_GADGET_H */
