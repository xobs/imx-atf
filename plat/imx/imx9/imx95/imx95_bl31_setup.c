/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <stdbool.h>

#include <arch_helpers.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <context.h>
#include <drivers/console.h>
#include <drivers/generic_delay_timer.h>
#include <lib/el3_runtime/context_mgmt.h>
#include <lib/mmio.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <plat/common/platform.h>

#include <ele_api.h>
#include <imx_scmi_client.h>
#include <imx8_lpuart.h>
#include <platform_def.h>
#include <plat_imx8.h>

#include <drivers/arm/gicv3.h>
#include "../drivers/arm/gic/v3/gicv3_private.h"

#include <bl31/interrupt_mgmt.h>
#include "imx95_dwc3.h"

#define TRUSTY_PARAMS_LEN_BYTES      (4096*2)

extern void imx9_init_scmi_server();

static const mmap_region_t imx_mmap[] = {
	/* APIS2 mapping  */
	MAP_REGION_FLAT(AIPS2_BASE, AIPSx_SIZE, MT_DEVICE | MT_RW),
	MAP_REGION_FLAT(PLAT_GICD_BASE, 0x200000, MT_DEVICE | MT_RW),
	MAP_REGION_FLAT(AIPS1_BASE, AIPSx_SIZE, MT_DEVICE | MT_RW),
	/* GPIO2-5 */
	MAP_REGION_FLAT(GPIO2_BASE, 0x20000, MT_DEVICE | MT_RW),
	MAP_REGION_FLAT(GPIO4_BASE, 0x20000, MT_DEVICE | MT_RW),
	MAP_REGION_FLAT(ELE_MU_BASE, 0x10000, MT_DEVICE | MT_RW),
	/* DWC3 USB controller (0x4c100000) + USB PHY CSR (0x4c1f0040). */
	MAP_REGION_FLAT(0x4c100000, 0x100000, MT_DEVICE | MT_RW),
	/* DWC3 DMA window (event ring/TRBs/buffers) — cacheable, flushed. */
	MAP_REGION_FLAT(0x93300000, 0x10000, MT_MEMORY | MT_RW),
	/*
	 * EL1<->EL3 console channel rings — non-cacheable AND non-secure, so
	 * BL31's (EL3/secure) writes land in the same non-secure DRAM view that
	 * osdyne (EL1/non-secure) reads. Without MT_NS the channel writes go to
	 * the secure view and osdyne never sees them.
	 */
	MAP_REGION_FLAT(0x93380000, 0x10000, MT_NON_CACHEABLE | MT_NS | MT_RW),

	{0},
};

static entry_point_info_t bl32_image_ep_info;
static entry_point_info_t bl33_image_ep_info;

/* get SPSR for BL33 entry */
static uint32_t get_spsr_for_bl33_entry(void)
{
	unsigned long el_status;
	unsigned long mode;
	uint32_t spsr;

	/* figure out what mode we enter the non-secure world */
	el_status = read_id_aa64pfr0_el1() >> ID_AA64PFR0_EL2_SHIFT;
	el_status &= ID_AA64PFR0_ELX_MASK;

	mode = (el_status) ? MODE_EL2 : MODE_EL1;

	spsr = SPSR_64(mode, MODE_SP_ELX, DISABLE_ALL_EXCEPTIONS);
	return spsr;
}

void bl31_early_platform_setup2(u_register_t arg0, u_register_t arg1,
		u_register_t arg2, u_register_t arg3)
{
	static console_t console;

	console_lpuart_register(IMX_LPUART_BASE, IMX_BOOT_UART_CLK_IN_HZ,
		     IMX_CONSOLE_BAUDRATE, &console);

	/* This console is only used for boot stage */
	console_set_scope(&console, CONSOLE_FLAG_BOOT);

	/*
	 * tell BL3-1 where the non-secure software image is located
	 * and the entry state information.
	 */
	bl33_image_ep_info.pc = PLAT_NS_IMAGE_OFFSET;
	bl33_image_ep_info.spsr = get_spsr_for_bl33_entry();
	SET_SECURITY_STATE(bl33_image_ep_info.h.attr, NON_SECURE);

#if defined(SPD_opteed) || defined(SPD_trusty)
	/* Populate entry point information for BL32 */
	SET_PARAM_HEAD(&bl32_image_ep_info, PARAM_EP, VERSION_1, 0);
	SET_SECURITY_STATE(bl32_image_ep_info.h.attr, SECURE);
	bl32_image_ep_info.pc = BL32_BASE;
	bl32_image_ep_info.spsr = 0;

	/* Pass TEE base and size to bl33 */
	bl33_image_ep_info.args.arg1 = BL32_BASE;
	bl33_image_ep_info.args.arg2 = BL32_SIZE;

#ifdef SPD_trusty
	bl32_image_ep_info.args.arg0 = BL32_SIZE;
	bl32_image_ep_info.args.arg1 = BL32_BASE;
#else
	/* Make sure memory is clean */
	mmio_write_32(BL32_FDT_OVERLAY_ADDR, 0);
	bl33_image_ep_info.args.arg3 = BL32_FDT_OVERLAY_ADDR;
	bl32_image_ep_info.args.arg3 = BL32_FDT_OVERLAY_ADDR;
#endif
#endif
}

void bl31_plat_arch_setup(void)
{
	/* Assign all the GPIO pins to non-secure world by default */
	mmio_write_32(GPIO2_BASE + 0x10, 0xffffffff);
	mmio_write_32(GPIO2_BASE + 0x14, 0x3);
	mmio_write_32(GPIO2_BASE + 0x18, 0xffffffff);
	mmio_write_32(GPIO2_BASE + 0x1c, 0x3);

	mmio_write_32(GPIO3_BASE + 0x10, 0xffffffff);
	mmio_write_32(GPIO3_BASE + 0x14, 0x3);
	mmio_write_32(GPIO3_BASE + 0x18, 0xffffffff);
	mmio_write_32(GPIO3_BASE + 0x1c, 0x3);

	mmio_write_32(GPIO4_BASE + 0x10, 0xffffffff);
	mmio_write_32(GPIO4_BASE + 0x14, 0x3);
	mmio_write_32(GPIO4_BASE + 0x18, 0xffffffff);
	mmio_write_32(GPIO4_BASE + 0x1c, 0x3);

	mmio_write_32(GPIO5_BASE + 0x10, 0xffffffff);
	mmio_write_32(GPIO5_BASE + 0x14, 0x3);
	mmio_write_32(GPIO5_BASE + 0x18, 0xffffffff);
	mmio_write_32(GPIO5_BASE + 0x1c, 0x3);

	mmap_add_region(BL31_BASE, BL31_BASE, (BL31_LIMIT - BL31_BASE),
		MT_MEMORY | MT_RW | MT_SECURE);
	mmap_add_region(BL_CODE_BASE, BL_CODE_BASE, (BL_CODE_END - BL_CODE_BASE),
		MT_MEMORY | MT_RO | MT_SECURE);

#ifdef SPD_trusty
	mmap_add_region(BL32_BASE, BL32_BASE, BL32_SIZE, MT_MEMORY | MT_RW);
#endif

	mmap_add(imx_mmap);

	init_xlat_tables();

	enable_mmu_el3(0);
}

/*
 * EL3 FIQ handler for the DWC3 controller (INTID 207, Group-0 secure). Runs the
 * resident USB console gadget, preempting EL1/EL2 unconditionally.
 */
static uint64_t imx95_dwc3_fiq_handler(uint32_t id, uint32_t flags,
				       void *handle, void *cookie)
{
	uint32_t irq = plat_ic_acknowledge_interrupt();

	if (irq == IMX95_DWC3_INTID)
		imx95_dwc3_service();

	if (irq < 1022U)
		plat_ic_end_of_interrupt(irq);

	return 0;
}

void bl31_platform_setup(void)
{
	uint32_t gicr_ctlr;
	uintptr_t gicr_base;
	uint32_t flags;
	unsigned int cpu;
	int rc;
	int i;

	generic_delay_timer_init();

	plat_gic_driver_init();
	/* Ensure to mark the core as asleep, required for reset case. */
	plat_gic_cpuif_disable();
	/* Clear LPIs */
	for (i = 0; i < PLATFORM_CORE_COUNT; i++) {
		gicr_base = gicv3_driver_data->rdistif_base_addrs[i];
		gicr_ctlr = gicr_read_ctlr(gicr_base);
		gicr_write_ctlr(gicr_base, gicr_ctlr & ~(GICR_CTLR_EN_LPIS_BIT));
	}
	plat_gic_init();

	/*
	 * Route all SPIs (IRQ 32 and above) as non-secure Group 1
	 * so they are accessible from EL1/EL2 without SMC calls.
	 * Each GICD_IGROUPR/IGRPMODR register covers 32 interrupts.
	 * SPIs 32-255 fit in registers 1-7.
	 */
	for (i = 1; i <= 7; i++) {
		mmio_write_32(PLAT_GICD_BASE + GICD_IGROUPR + (i * 4), ~0U);
		mmio_write_32(PLAT_GICD_BASE + GICD_IGRPMODR + (i * 4), 0U);
	}

	/*
	 * Enable Non-secure Group 1 interrupts at the CPU interface.
	 * gicv3_cpuif_enable() only enables Secure Group 1 (G1S),
	 * which masks all non-secure SPIs at EL3.
	 */
	write_icc_igrpen1_el3(read_icc_igrpen1_el3() |
			      IGRPEN1_EL3_ENABLE_G1NS_BIT);

	/*
	 * Carve the DWC3 controller interrupt (INTID 207) back out of the
	 * blanket Group1NS routing above and make it a Group-0 secure FIQ to
	 * EL3, so BL31's resident USB console gadget services it even while
	 * EL1/EL2 is CPU-bound or hung.
	 */
	cpu = plat_my_core_pos();
	gicv3_set_interrupt_group(IMX95_DWC3_INTID, cpu, INTR_GROUP0);
	gicv3_set_interrupt_priority(IMX95_DWC3_INTID, cpu,
				     GIC_HIGHEST_SEC_PRIORITY);
	gicv3_enable_interrupt(IMX95_DWC3_INTID, cpu);

	flags = 0;
	set_interrupt_rm_flag(flags, NON_SECURE);
	rc = register_interrupt_type_handler(INTR_TYPE_EL3,
					     imx95_dwc3_fiq_handler, flags);
	if (rc != 0)
		panic();

	/* Bring up the resident USB console gadget. */
	imx95_dwc3_gadget_init();

	/* get soc info */
	ele_get_soc_info();

	plat_imx9_scmi_setup();
}

entry_point_info_t *bl31_plat_get_next_image_ep_info(unsigned int type)
{
	if (type == NON_SECURE)
		return &bl33_image_ep_info;

	if (type == SECURE)
		return &bl32_image_ep_info;

	return NULL;
}

unsigned int plat_get_syscnt_freq2(void)
{
	return COUNTER_FREQUENCY;
}

#ifdef SPD_trusty
void plat_trusty_set_boot_args(aapcs64_params_t *args) {
	args->arg0 = BL32_SIZE;
	args->arg1 = BL32_BASE;
	args->arg2 = TRUSTY_PARAMS_LEN_BYTES;
}
#endif
