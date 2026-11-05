/*
 * Copyright 2022-2023 NXP
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
#include <drivers/nxp/trdc/imx_trdc.h>
#include <lib/el3_runtime/context_mgmt.h>
#include <lib/mmio.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <plat/common/platform.h>

#include <ele_api.h>
#include <dram.h>
#include <imx_gadget.h>
#include <imx8_lpuart.h>
#include <plat_common.h>
#include <plat_imx8.h>
#include <platform_def.h>

#include "imx93_chipidea.h"

#define MAP_BL31_TOTAL										   \
	MAP_REGION_FLAT(BL31_BASE, BL31_SIZE, MT_MEMORY | MT_RW | MT_SECURE)
#define MAP_BL31_RO										   \
	MAP_REGION_FLAT(BL_CODE_BASE, BL_CODE_END - BL_CODE_BASE, MT_MEMORY | MT_RO | MT_SECURE)

#define MAP_BL32_TOTAL										   \
	MAP_REGION_FLAT(BL32_BASE, BL32_SIZE, MT_MEMORY | MT_RW)

#define TRUSTY_PARAMS_LEN_BYTES      (4096*2)

static const mmap_region_t imx_mmap[] = {
	AIPS1_MAP, AIPS2_MAP, AIPS4_MAP, GIC_MAP,
	TRDC_A_MAP, TRDC_W_MAP, TRDC_M_MAP,
	TRDC_N_MAP, DDRMIX_MAP, GPIO_MAP,
	S400_MU_MAP, NIC_GPV_MAP, AIPS3_MAP,
	FSB_MAP,
	/*
	 * USB OTG1, the ChipIdea controller osdyne's gadget runs on.  Non-secure:
	 * the TRDC opens HSIOMIX to the A55's domain for non-secure accesses
	 * only (trdc_config.h), and a secure one stalls.
	 */
	MAP_REGION_FLAT(0x4c100000, 0x10000, MT_DEVICE | MT_RW | MT_NS),
	/*
	 * DRAM osdyne leaves to BL31, above where it runs: the gadget's queue
	 * heads, transfer descriptors and buffers, then the console rings it
	 * shares with osdyne. Non-cacheable, so neither the controller nor
	 * osdyne needs cache maintenance, and non-secure, so BL31 sees the
	 * same memory they do.
	 */
	MAP_REGION_FLAT(0x93300000, 0x10000, MT_NON_CACHEABLE | MT_NS | MT_RW),
	MAP_REGION_FLAT(0x93380000, 0x10000, MT_NON_CACHEABLE | MT_NS | MT_RW),
	/*
	 * osdyne's DRAM, so the host can peek and poke it through the gadget,
	 * split around the windows above.  Every access is still checked page
	 * by page against these mappings, so only mapped ranges are touched.
	 */
	MAP_REGION_FLAT(0x80000000, 0x13000000, MT_MEMORY | MT_RW | MT_NS),
	MAP_REGION_FLAT(0x93400000, 0x0cc00000, MT_MEMORY | MT_RW | MT_NS),
	/* osdyne's part of OCRAM, below BL31. */
	MAP_REGION_FLAT(OCRAM_BASE, BL31_BASE - OCRAM_BASE, MT_MEMORY | MT_RW | MT_NS),
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
	imx_bl31_params_parse(arg0, OCRAM_BASE, OCRAM_SIZE,
				    &bl32_image_ep_info, &bl33_image_ep_info);
}

void bl31_plat_arch_setup(void)
{
	/* no coherence memory support on i.MX9 */
	const mmap_region_t bl_regions[] = {
		MAP_BL31_TOTAL,
		MAP_BL31_RO,
#ifdef SPD_trusty
		/* Map Tee memory */
		MAP_BL32_TOTAL,
#endif
		{0}
	};

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

	mmio_write_32(GPIO1_BASE + 0x10, 0xffffffff);
	mmio_write_32(GPIO1_BASE + 0x14, 0x3);
	mmio_write_32(GPIO1_BASE + 0x18, 0xffffffff);
	mmio_write_32(GPIO1_BASE + 0x1c, 0x3);

	setup_page_tables(bl_regions, imx_mmap);
	enable_mmu_el3(0);

	/* trdc must be initialized */
	trdc_config();
}

void bl31_platform_setup(void)
{
	generic_delay_timer_init();

	/* get soc info, whose UID the gadget reports as its serial */
	ele_get_soc_info();

	/* Init the dram info */
	dram_info_init(SAVED_DRAM_TIMING_BASE);

	plat_gic_driver_init();
	plat_gic_init();

	/* Bring up the resident USB console gadget. */
	imx_gadget_init(IMX93_CHIPIDEA_INTID);

	/* Last, so that everything a held board still has to do is set up. */
	imx_gadget_hold_for_host();
}

/* Runs once BL33's entry context is built, just before BL31 enters it. */
void bl31_plat_runtime_setup(void)
{
	imx_gadget_halt_at_entry();
}

entry_point_info_t *bl31_plat_get_next_image_ep_info(unsigned int type)
{
	if (type == NON_SECURE) {
		return &bl33_image_ep_info;
	}

	if (type == SECURE) {
		return &bl32_image_ep_info;
	}

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
