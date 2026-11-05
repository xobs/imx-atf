/*
 * The line between osdyne's USB gadget (imx_gadget.c) and the driver for one
 * SoC's USB controller.
 *
 * The core speaks the protocol: it answers SETUP packets, applies what their
 * data stages carry, runs the debug monitor and owns the console rings shared
 * with osdyne. The driver moves bytes: it brings the controller up as a device
 * with a control endpoint and one bulk endpoint pair (EP1 OUT and IN), runs the
 * control transfer's stages, and calls back into the core at each of them.
 *
 * Everything here runs in EL3 with FIQs masked: from the controller's FIQ,
 * from an SMC, or from the monitor's polling loops, never two at once.
 */
#ifndef IMX_GADGET_CONTROLLER_H
#define IMX_GADGET_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/* Largest data stage the core answers or accepts on EP0. */
#define IMX_GADGET_EP0_DATA_MAX	512U

/* ---------------------------------------------- implemented by the driver */

/*
 * Take the controller over from whatever the previous stage left running,
 * bring it up as a device and connect to the bus.  The controller interrupts
 * from here on; imx_gadget_ctrl_service() handles what it raised.
 */
void imx_gadget_ctrl_start(void);

/* Handle everything the controller has raised.  Polled while parked, too. */
void imx_gadget_ctrl_service(void);

/*
 * Drop off the bus, so the host sees the device go and the next stage's own
 * gadget can enumerate.  Safe before imx_gadget_ctrl_start().
 */
void imx_gadget_ctrl_disconnect(void);

/*
 * SET_ADDRESS: take @addr as the device address, from the end of the request's
 * status stage as USB requires.  Called while the SETUP is decoded.
 */
void imx_gadget_ctrl_set_address(uint8_t addr);

/* SET_CONFIGURATION, once its status stage is done: enable EP1 OUT and IN. */
void imx_gadget_ctrl_configure(void);

/*
 * Send queued console output on EP1 IN, if the endpoint is idle: take it with
 * imx_gadget_console_take().  Called whenever there may be more to send.
 */
void imx_gadget_ctrl_tx_pump(void);

/* ------------------------------------------------ implemented by the core */

/*
 * Answer the SETUP packet @setup.  For a request with a device-to-host data
 * stage, the reply is staged in @buf and its length returned.  For one with a
 * host-to-device data stage, the length to receive into the driver's buffer
 * is returned; imx_gadget_ep0_data_out() then takes what arrived.  Without a
 * data stage, 0.  -1 asks the driver to STALL the request.
 */
int imx_gadget_setup(const uint8_t setup[8], uint8_t *buf);

/* A host-to-device data stage arrived: @len bytes in @buf. */
void imx_gadget_ep0_data_out(const uint8_t *buf, uint32_t len);

/*
 * The control transfer's status stage completed.  Requests that must be
 * acknowledged before they are acted on -- a reboot, a configuration -- act
 * here; this may not return, for a reboot.
 */
void imx_gadget_ep0_status_done(void);

/* The bus was reset: the host is starting over, unconfigured. */
void imx_gadget_bus_reset(void);

/* The host went away. */
void imx_gadget_bus_disconnected(void);

/*
 * Take up to @max bytes of osdyne's console output into @dst, for EP1 IN.
 * Returns how many; 0 when there is none, or the host has not configured the
 * device.
 */
uint32_t imx_gadget_console_take(uint8_t *dst, uint32_t max);

/* Hand osdyne @len bytes the host sent on EP1 OUT. */
void imx_gadget_console_give(const uint8_t *src, uint32_t len);

#endif /* IMX_GADGET_CONTROLLER_H */
