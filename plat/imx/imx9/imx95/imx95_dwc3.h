/*
 * i.MX95 DWC3 USB console gadget resident in BL31 (EL3). See imx95_dwc3.c.
 */
#ifndef IMX95_DWC3_H
#define IMX95_DWC3_H

#include <cdefs.h>
#include <stdbool.h>
#include <stdint.h>

/* GIC INTID of the DWC3 controller interrupt (GIC SPI 175 -> 32 + 175). */
#define IMX95_DWC3_INTID	207

/* Bring the gadget up (call once from bl31_platform_setup, after GIC init). */
void imx95_dwc3_gadget_init(void);

/* Drain the DWC3 event ring; call from the EL3 FIQ handler for INTID 207. */
void imx95_dwc3_service(void);

/* Drain the console TX ring to EP1 IN; call from the IMX_SIP_USB_CONSOLE SMC. */
void imx95_dwc3_console_kick(void);

/* Soft-disconnect the USB gadget from the bus (clear RUN_STOP) so a reset leaves
 * USB clean for the next stage's serial-download gadget. Call from every reset
 * path (see imx_system_reset). Safe if the gadget was never brought up. */
void imx95_dwc3_disconnect(void);

/* Soft-disconnect USB and PSCI-reset the SoC. Handles the vendor reboot
 * request; usable even when osdyne is frozen. Never returns. */
void __dead2 imx95_dwc3_do_reboot(void);

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
 */
bool imx95_dwc3_debug_fault(unsigned int reason, void *handle,
			    uint64_t esr, uint64_t far, uint64_t pc);

/*
 * Whether the gadget is up.  A fault before it is has nowhere to report to;
 * imx95_dwc3_debug_fault() then returns immediately without parking, and the
 * caller has to fall back to dying the old way.
 */
bool imx95_dwc3_debug_ready(void);

/* Park if the host has asked us to stop; call from the EL3 FIQ handler after
 * imx95_dwc3_service(). */
void imx95_dwc3_debug_park(void);

/* Hand the monitor the interrupted non-secure context; call from the EL3 FIQ
 * handler before servicing, so status/regs report the debuggee. */
void imx95_dwc3_debug_context(void *handle);

#endif /* IMX95_DWC3_H */
