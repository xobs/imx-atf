#
# Copyright 2022-2023 NXP
#
# SPDX-License-Identifier: BSD-3-Clause
#

# Route external aborts and SErrors from the non-secure world to EL3 so the
# debug monitor sees them: a crash in osdyne becomes something inspectable
# over USB instead of a dead board.
HANDLE_EA_EL3_FIRST_NS	:=	1

# osdyne's USB gadget (plat/imx/common/imx_gadget.c), on the ChipIdea.
IMX_GADGET		:=	1
$(eval $(call add_define,IMX_GADGET))

PLAT_INCLUDES		:=	-Iplat/imx/common/include		\
				-Iplat/imx/imx93/include		\
				-Iplat/imx/imx9/include		\
				-Iplat/imx/common
# Translation tables library
include lib/xlat_tables_v2/xlat_tables.mk

GICV3_SUPPORT_GIC600  :=      1
GICV3_OVERRIDE_DISTIF_PWR_OPS	:=	1

# Include GICv3 driver files
include drivers/arm/gic/v3/gicv3.mk

IMX_GIC_SOURCES		:=	${GICV3_SOURCES}			\
				plat/common/plat_gicv3.c		\
				plat/common/plat_psci_common.c		\
				plat/imx/common/plat_imx8_gic.c

IMX_DRAM_SOURCES	:=	plat/imx/imx93/ddr/dram.c		\
				plat/imx/imx93/ddr/ddr_dvfs.c	        \
				plat/imx/imx93/ddr/dram_retention.c

BL31_SOURCES		+=	common/desc_image_load.c			\
				plat/common/aarch64/crash_console_helpers.S	\
				plat/imx/imx93/aarch64/plat_helpers.S		\
				plat/imx/imx93/plat_topology.c			\
				plat/imx/common/imx_common.c			\
				plat/imx/common/lpuart_console.S		\
				plat/imx/imx93/trdc.c			\
				plat/imx/imx93/pwr_ctrl.c			\
				plat/imx/imx93/imx93_bl31_setup.c		\
				plat/imx/imx93/imx93_chipidea.c		\
				plat/imx/common/imx_gadget.c			\
				plat/imx/imx93/imx93_psci.c			\
				plat/imx/imx93/src.c			\
				plat/imx/common/imx_sip_svc.c			\
				plat/imx/common/imx_sip_handler.c			\
				plat/imx/common/ele_api.c			\
				lib/cpus/aarch64/cortex_a55.S			\
				drivers/delay_timer/delay_timer.c		\
				drivers/delay_timer/generic_delay_timer.c	\
				drivers/nxp/trdc/imx_trdc.c			\
				${IMX_GIC_SOURCES}				\
				${IMX_DRAM_SOURCES}				\
				${XLAT_TABLES_LIB_SRCS}

ifeq (${SPD},trusty)
	BL31_SOURCES += plat/imx/common/ffa_shared_mem.c
endif

RESET_TO_BL31		:=	1
HW_ASSISTED_COHERENCY	:= 	1
USE_COHERENT_MEM	:=	0
PROGRAMMABLE_RESET_ADDRESS :=	1
COLD_BOOT_SINGLE_CPU	:=	1

BL32_BASE               ?=      0x96000000
BL32_SIZE               ?=      0x02000000
$(eval $(call add_define,BL32_BASE))
$(eval $(call add_define,BL32_SIZE))
