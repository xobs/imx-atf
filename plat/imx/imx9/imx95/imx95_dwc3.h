/*
 * i.MX95 DWC3 USB console gadget resident in BL31 (EL3). See imx95_dwc3.c.
 */
#ifndef IMX95_DWC3_H
#define IMX95_DWC3_H

#include <cdefs.h>

/* GIC INTID of the DWC3 controller interrupt (GIC SPI 175 -> 32 + 175). */
#define IMX95_DWC3_INTID	207

/* Bring the gadget up (call once from bl31_platform_setup, after GIC init). */
void imx95_dwc3_gadget_init(void);

/* Drain the DWC3 event ring; call from the EL3 FIQ handler for INTID 207. */
void imx95_dwc3_service(void);

/* Drain the console TX ring to EP1 IN; call from the IMX_SIP_USB_CONSOLE SMC. */
void imx95_dwc3_console_kick(void);

/* Soft-disconnect USB and PSCI-reset the SoC. Handles the vendor reboot
 * request; usable even when osdyne is frozen. Never returns. */
void __dead2 imx95_dwc3_do_reboot(void);

#endif /* IMX95_DWC3_H */
