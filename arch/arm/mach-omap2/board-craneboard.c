/*
 * linux/arch/arm/mach-omap2/board-crane.c
 *
 * Copyright (C) 2010 Mistral Solutions Pvt Ltd <www.mistralsolutions.com>
 * Author: Srinath.R <srinath@mistralsolutions.com>
 *
 * Based on mach-omap2/board-am3517evm.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/input.h>
#include <linux/davinci_emac.h>
#include <linux/regulator/machine.h>
#include <linux/can/platform/ti_hecc.h>
#include <linux/i2c/tps65910.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/usb.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>

#include <media/tvp514x.h>
#include <media/ti-media/vpfe_capture.h>

#include "mmc-craneboard.h"
#include "mux.h"

#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE        SZ_128K

#define GPIO_TPS65910_IRQ	0
#define TPS65910_IRQ_LINE	0
#define USB3320_RESET_GPIO	38
#define USB_ENABLE_GPIO		35

static struct mtd_partition craneboard_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "xloader-nand",
		.offset         = 0,
		.size           = 4*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "uboot-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 14*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "params-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 2*(SZ_128K)
	},
	{
		.name           = "linux-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 40*(SZ_128K)
	},
	{
		.name           = "jffs2-nand",
		.size           = MTDPART_SIZ_FULL,
		.offset         = MTDPART_OFS_APPEND,
	},
};

static struct omap_nand_platform_data craneboard_nand_data = {
	.parts          = craneboard_nand_partitions,
	.nr_parts       = ARRAY_SIZE(craneboard_nand_partitions),
	.nand_setup     = NULL,
	.dma_channel    = -1,           /* disable DMA in OMAP NAND driver */
	.dev_ready      = NULL,
};

static struct resource craneboard_nand_resource = {
	.flags          = IORESOURCE_MEM,
};

static struct platform_device craneboard_nand_device = {
	.name           = "omap2-nand",
	.id             = 0,
	.dev            = {
		.platform_data  = &craneboard_nand_data,
	},
	.num_resources  = 1,
	.resource       = &craneboard_nand_resource,
};

void __init craneboard_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;
	u32 ret;
	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	while (cs < GPMC_CS_NUM) {
		ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if (0x800 == (ret & 0xC00)) {
			/* Found it!! */
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}
	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration in \
				GPMC\n");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		craneboard_nand_data.cs   = nandcs;
		craneboard_nand_data.gpmc_cs_baseaddr = (void *)(gpmc_base_add
				+ GPMC_CS0_BASE + nandcs*GPMC_CS_SIZE);

		craneboard_nand_data.gpmc_baseaddr = (void *) (gpmc_base_add);

		if (platform_device_register(&craneboard_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}


#define AM35XX_EVM_PHY_MASK		(0xF)
#define AM35XX_EVM_MDIO_FREQUENCY    	(1000000)

static struct emac_platform_data craneboard_emac_pdata = {
	.phy_mask       = AM35XX_EVM_PHY_MASK,
	.mdio_max_freq  = AM35XX_EVM_MDIO_FREQUENCY,
	.rmii_en        = 1,
};

static int __init eth_addr_setup(char *str)
{
	int i;

	if (str == NULL)
		return 0;
	for (i = 0; i <  ETH_ALEN; i++)
		craneboard_emac_pdata.mac_addr[i] = strict_strtol(&str[i*3],
				16, (long *)NULL);
	return 1;
}

/* Get MAC address from kernel boot parameter eth=AA:BB:CC:DD:EE:FF */
__setup("eth=", eth_addr_setup);

static struct resource am3517_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x3FFFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.end    = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_emac_device = {
	.name           = "davinci_emac",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(am3517_emac_resources),
	.resource       = am3517_emac_resources,
};

static void am3517_enable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
			AM35XX_CPGMAC_C0_TX_PULSE_CLR |
			AM35XX_CPGMAC_C0_MISC_PULSE_CLR |
			AM35XX_CPGMAC_C0_RX_THRESH_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void am3517_disable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
			AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

void craneboard_ethernet_init(struct emac_platform_data *pdata)
{
	unsigned int regval;

	pdata->ctrl_reg_offset          = AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset      = AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset          = AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->mdio_reg_offset          = AM35XX_EMAC_MDIO_OFFSET;
	pdata->ctrl_ram_size            = AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version                  = EMAC_VERSION_2;
	pdata->hw_ram_addr              = AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable 	= am3517_enable_ethernet_int;
	pdata->interrupt_disable 	= am3517_disable_ethernet_int;
	am3517_emac_device.dev.platform_data     = pdata;
	platform_device_register(&am3517_emac_device);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval, AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);

	return ;
}



#define DVI_ENABLE_GPIO	52
static int dvi_enabled;

static void __init craneboard_display_init(void)
{
	int r;
	omap_mux_init_gpio(DVI_ENABLE_GPIO, OMAP_PIN_OUTPUT);
	/*
	 * Enable GPIO 52  = LCD/DVI Enable/disable
	 */
	r = gpio_request(DVI_ENABLE_GPIO, "dvi_enable");
	if (r) {
		printk(KERN_ERR "failed to get dvi_enable \n");
		return;
	}
	gpio_direction_output(DVI_ENABLE_GPIO, 1);

	gpio_set_value(DVI_ENABLE_GPIO, 0);

	dvi_enabled = 1;

	printk(KERN_INFO "Display initialized successfully\n");
	return;
}



static int craneboard_panel_enable_tv(struct omap_dss_device *dssdev)
{
	return 0;
}

static void craneboard_panel_disable_tv(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_device craneboard_tv_device = {
	.type 			= OMAP_DISPLAY_TYPE_VENC,
	.name 			= "tv",
	.driver_name		= "venc",
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable	= craneboard_panel_enable_tv,
	.platform_disable	= craneboard_panel_disable_tv,
};

static int craneboard_panel_enable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(DVI_ENABLE_GPIO, 0);
	dvi_enabled = 1;

	return 0;
}

static void craneboard_panel_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(DVI_ENABLE_GPIO, 1);
	dvi_enabled = 0;

}

static struct omap_dss_device craneboard_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.phy.dpi.data_lines	= 24,
	.platform_enable	= craneboard_panel_enable_dvi,
	.platform_disable	= craneboard_panel_disable_dvi,
};

static struct omap_dss_device *craneboard_dss_devices[] = {
	&craneboard_tv_device,
	&craneboard_dvi_device,
};

static struct omap_dss_board_info craneboard_dss_data = {
	.num_devices	= ARRAY_SIZE(craneboard_dss_devices),
	.devices	= craneboard_dss_devices,
	.default_device	= &craneboard_dvi_device,
};

struct platform_device craneboard_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data	= &craneboard_dss_data,
	},
};

/*
 * VPFE - Video Decoder interface
 */
#define TVP514X_STD_ALL		(V4L2_STD_NTSC | V4L2_STD_PAL)

/* Inputs available at the TVP5146 */
static struct v4l2_input tvp5146_inputs[] = {
	{
		.index	= 0,
		.name	= "Composite",
		.type	= V4L2_INPUT_TYPE_CAMERA,
		.std	= TVP514X_STD_ALL,
	},
};

static struct tvp514x_platform_data tvp5146_pdata = {
	.clk_polarity	= 0,
	.hs_polarity	= 1,
	.vs_polarity	= 1
};

static struct vpfe_route tvp5146_routes[] = {
	{
		.input	= INPUT_CVBS_VI1A,
		.output	= OUTPUT_10BIT_422_EMBEDDED_SYNC,
	},
	{
		.input	= INPUT_SVIDEO_VI2C_VI1C,
		.output	= OUTPUT_10BIT_422_EMBEDDED_SYNC,
	},
};

static struct vpfe_subdev_info vpfe_sub_devs[] = {
	{
		.name		= "tvp5146",
		.grp_id		= 0,
		.num_inputs	= ARRAY_SIZE(tvp5146_inputs),
		.inputs		= tvp5146_inputs,
		.routes		= tvp5146_routes,
		.can_route	= 1,
		.ccdc_if_params	= {
			.if_type = VPFE_BT656_10BIT,
			.hdpol	= VPFE_PINPOL_POSITIVE,
			.vdpol	= VPFE_PINPOL_POSITIVE,
		},
		.board_info	= {
			I2C_BOARD_INFO("tvp5146", 0x5C),
			.platform_data = &tvp5146_pdata,
		},
	},
};

static void craneboard_clear_vpfe_intr(int vdint)
{
	unsigned int vpfe_int_clr;

	vpfe_int_clr = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);

	switch (vdint) {
		/* VD0 interrrupt */
	case INT_35XX_CCDC_VD0_IRQ:
		vpfe_int_clr &= ~AM35XX_VPFE_CCDC_VD0_INT_CLR;
		vpfe_int_clr |= AM35XX_VPFE_CCDC_VD0_INT_CLR;
		break;
		/* VD1 interrrupt */
	case INT_35XX_CCDC_VD1_IRQ:
		vpfe_int_clr &= ~AM35XX_VPFE_CCDC_VD1_INT_CLR;
		vpfe_int_clr |= AM35XX_VPFE_CCDC_VD1_INT_CLR;
		break;
		/* VD2 interrrupt */
	case INT_35XX_CCDC_VD2_IRQ:
		vpfe_int_clr &= ~AM35XX_VPFE_CCDC_VD2_INT_CLR;
		vpfe_int_clr |= AM35XX_VPFE_CCDC_VD2_INT_CLR;
		break;
		/* Clear all interrrupts */
	default:
		vpfe_int_clr &= ~(AM35XX_VPFE_CCDC_VD0_INT_CLR |
				AM35XX_VPFE_CCDC_VD1_INT_CLR |
				AM35XX_VPFE_CCDC_VD2_INT_CLR);
		vpfe_int_clr |= (AM35XX_VPFE_CCDC_VD0_INT_CLR |
					AM35XX_VPFE_CCDC_VD1_INT_CLR |
					AM35XX_VPFE_CCDC_VD2_INT_CLR);
		break;
	}
	omap_ctrl_writel(vpfe_int_clr, AM35XX_CONTROL_LVL_INTR_CLEAR);
	vpfe_int_clr = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static struct vpfe_config vpfe_cfg = {
	.num_subdevs	= ARRAY_SIZE(vpfe_sub_devs),
	.i2c_adapter_id	= 3,
	.sub_devs	= vpfe_sub_devs,
	.clr_intr	= craneboard_clear_vpfe_intr,
	.card_name	= "DM6446 EVM",
	.ccdc		= "DM6446 CCDC",
};

static struct resource vpfe_resources[] = {
	{
		.start	= INT_35XX_CCDC_VD0_IRQ,
		.end	= INT_35XX_CCDC_VD0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= INT_35XX_CCDC_VD1_IRQ,
		.end	= INT_35XX_CCDC_VD1_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 vpfe_capture_dma_mask = DMA_BIT_MASK(32);
static struct platform_device vpfe_capture_dev = {
	.name		= CAPTURE_DRV_NAME,
	.id		= -1,
	.num_resources	= ARRAY_SIZE(vpfe_resources),
	.resource	= vpfe_resources,
	.dev = {
		.dma_mask		= &vpfe_capture_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &vpfe_cfg,
	},
};

static struct resource dm644x_ccdc_resource[] = {
	/* CCDC Base address */
	{
		.start	= AM35XX_IPSS_VPFE_BASE,
		.end	= AM35XX_IPSS_VPFE_BASE + 0xffff,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device dm644x_ccdc_dev = {
	.name		= "dm644x_ccdc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(dm644x_ccdc_resource),
	.resource	= dm644x_ccdc_resource,
	.dev = {
		.dma_mask		= &vpfe_capture_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};



static struct regulator_consumer_supply craneboard_vdd1_supplies[] = {
	{
		.supply = "vdd_core",
	},
};

static struct regulator_init_data craneboard_regulator_vdd1 = {
	.constraints = {
		.min_uV = 1200000,
		.max_uV = 1200000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(craneboard_vdd1_supplies),
	.consumer_supplies = craneboard_vdd1_supplies,
};

static struct regulator_consumer_supply craneboard_vdd2_supplies[] = {
	{
		.supply = "vddshv",
	},
};

static struct regulator_init_data craneboard_regulator_vdd2 = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(craneboard_vdd2_supplies),
	.consumer_supplies = craneboard_vdd2_supplies,
};


static struct regulator_consumer_supply craneboard_vio_supplies[] = {
	{
		.supply = "vdds",
	},
};

static struct regulator_init_data craneboard_regulator_vio = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(craneboard_vio_supplies),
	.consumer_supplies = craneboard_vio_supplies,
};


static struct regulator_consumer_supply craneboard_vaux1_supplies[] = {
	{
		.supply = "vdd_sram_mpu",
	},
	{
		.supply = "vdd_sram_core_bg0",
	},
	{
		.supply = "vddsosc",
	},
};

static struct regulator_init_data craneboard_regulator_vaux1 = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(craneboard_vaux1_supplies),
	.consumer_supplies = craneboard_vaux1_supplies,
};


static struct regulator_consumer_supply craneboard_vaux2_supplies[] = {
	{
		.supply = "vdda1p8v_usbphy",
	},
};

static struct regulator_init_data craneboard_regulator_vaux2 = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(craneboard_vaux2_supplies),
	.consumer_supplies = craneboard_vaux2_supplies,
};


static struct regulator_consumer_supply craneboard_vdac_supplies[] = {
	{
		.supply = "vdda_dac",
	},
};

static struct regulator_init_data craneboard_regulator_vdac = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(craneboard_vdac_supplies),
	.consumer_supplies = craneboard_vdac_supplies,
};

static struct regulator_consumer_supply craneboard_vmmc_supplies[] = {
	{
		.supply = "vdda3p3v_usbphy",
	},
};

static struct regulator_init_data craneboard_regulator_vmmc = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(craneboard_vmmc_supplies),
	.consumer_supplies = craneboard_vmmc_supplies,
};


static struct regulator_consumer_supply craneboard_vpll_supplies[] = {
	{
		.supply = "vdds_dpll_mpu_usbhost",
	},
	{
		.supply = "vdds_dpll_per_core",
	},
};

static struct regulator_init_data craneboard_regulator_vpll = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(craneboard_vpll_supplies),
	.consumer_supplies = craneboard_vpll_supplies,
};


static int craneboard_tps65910_config(struct tps65910_platform_data *pdata)
{
	u8 val;
	int err;

	/************Initilise TPS65910 for craneboard *****************/

	/* pull up/down default will do, external pullup provided on board */

	/* REF_REG used in bypass mode */

	/* Set RTC regulator on during sleep */

	val = TPS65910_VRTC_OFFMASK;
	err = tps65910_i2c_write_u8(TPS65910_I2C_ID0, val,
				TPS65910_REG_VRTC);
	if (err) {
		printk(KERN_ERR "Unable to write TPS65910_REG_VRTC reg\n");
		return -EIO;
	}

	/* Therm_REG default is fine */

	/*Back-up battery charger control*/

	tps65910_enable_bbch(TPS65910_BBSEL_2P52);

	/* DEVCTRL_REG */
	val &= 0;
	val &= ~TPS65910_RTC_PWDNN;
	val |= (TPS65910_CK32K_CTRL | TPS65910_SR_CTL_I2C_SEL);

	err = tps65910_i2c_write_u8(TPS65910_I2C_ID0, val,
				TPS65910_REG_DEVCTRL);
	if (err) {
		printk(KERN_ERR "Unsbale to write TPS65910_REG_DEVCTRL reg\n");
		return -EIO;
	}
	return 0;
}


static struct tps65910_platform_data craneboard_tps65910_data = {
	.irq_num 	= (unsigned)TPS65910_IRQ_LINE,
	.gpio  		= NULL,
	.vio   		= &craneboard_regulator_vio,
	.vdd1  		= &craneboard_regulator_vdd1,
	.vdd2  		= &craneboard_regulator_vdd2,
	.vdd3  		= NULL,
	.vdig1		= NULL,
	.vdig2		= NULL,
	.vaux33		= NULL,
	.vmmc		= &craneboard_regulator_vmmc,
	.vaux1		= &craneboard_regulator_vaux1,
	.vaux2		= &craneboard_regulator_vaux2,
	.vdac		= &craneboard_regulator_vdac,
	.vpll		= &craneboard_regulator_vpll,
	.board_tps65910_config = craneboard_tps65910_config,
};



static struct i2c_board_info __initdata craneboard_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65910", TPS65910_I2C_ID0),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &craneboard_tps65910_data,
	},
};


static int __init craneboard_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, NULL, 0);
	return 0;
}

/*
 * HECC information
 */

#define CAN_STB		214
static void am3517_hecc_plat_init(void)
{
	int r;

	r = gpio_request(CAN_STB, "can_stb");
	if (r) {
		printk(KERN_ERR "failed to get can_stb \n");
		return;
	}

	gpio_direction_output(CAN_STB, 0);
}

static struct resource am3517_hecc_resources[] = {
	{
		.start  = AM35XX_IPSS_HECC_BASE,
		.end    = AM35XX_IPSS_HECC_BASE + 0x3FFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_HECC0_IRQ,
		.end    = INT_35XX_HECC0_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_hecc_device = {
	.name           = "ti_hecc",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(am3517_hecc_resources),
	.resource       = am3517_hecc_resources,
};

static struct ti_hecc_platform_data craneboard_hecc_pdata = {
	.scc_hecc_offset        = AM35XX_HECC_SCC_HECC_OFFSET,
	.scc_ram_offset         = AM35XX_HECC_SCC_RAM_OFFSET,
	.hecc_ram_offset        = AM35XX_HECC_RAM_OFFSET,
	.mbx_offset            = AM35XX_HECC_MBOX_OFFSET,
	.int_line               = AM35XX_HECC_INT_LINE,
	.version                = AM35XX_HECC_VERSION,
	.platform_init		= am3517_hecc_plat_init,
};

static void craneboard_hecc_init(struct ti_hecc_platform_data *pdata)
{
	am3517_hecc_device.dev.platform_data = pdata;
	platform_device_register(&am3517_hecc_device);
}


/*
 * Board initialization
 */
static struct omap_board_config_kernel craneboard_config[] __initdata = {
};

static struct platform_device *craneboard_devices[] __initdata = {
	&dm644x_ccdc_dev,
	&vpfe_capture_dev,
	&craneboard_dss_device,
};

static void __init craneboard_init_irq(void)
{
	omap_board_config = craneboard_config;
	omap_board_config_size = ARRAY_SIZE(craneboard_config);

	omap2_init_common_hw(NULL, NULL, NULL, NULL, NULL);
	omap_init_irq();
	omap_gpio_init();
}

static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
#if defined(CONFIG_PANEL_SHARP_LQ043T1DG01) || \
	defined(CONFIG_PANEL_SHARP_LQ043T1DG01_MODULE)
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
#else
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
#endif
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = USB3320_RESET_GPIO,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* USB OTG DRVVBUS offset = 0x212 */
	OMAP3_MUX(CHASSIS_DMAREQ3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct craneboard_hsmmc_info mmc[] = {
	{
		.mmc            = 1,
		.wires          = 8,
		.gpio_cd        = 41,
		.gpio_wp        = 40,
	},
	{}      /* Terminator */
};

static void __init craneboard_init(void)
{


	craneboard_i2c_init();

	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	platform_add_devices(craneboard_devices,
			ARRAY_SIZE(craneboard_devices));

	omap_serial_init();
	craneboard_flash_init();
	usb_musb_init();
	/* Configure GPIO for EHCI port */
	omap_mux_init_gpio(USB3320_RESET_GPIO, OMAP_PIN_OUTPUT);
	/* Enable USB EHCI */
	omap_mux_init_gpio(USB_ENABLE_GPIO, OMAP_PIN_OUTPUT);
	gpio_request(USB_ENABLE_GPIO, "usb_ehci_enable");
	gpio_direction_output(USB_ENABLE_GPIO, 1);
	gpio_set_value(USB_ENABLE_GPIO, 1);

	usb_ehci_init(&ehci_pdata);

	i2c_register_board_info(1, craneboard_i2c1_boardinfo,
			ARRAY_SIZE(craneboard_i2c1_boardinfo));

	clk_add_alias("master", "dm644x_ccdc", "master",
			&vpfe_capture_dev.dev);
	clk_add_alias("slave", "dm644x_ccdc", "slave",
			&vpfe_capture_dev.dev);
	/* DSS */
	craneboard_display_init();

	/*Ethernet*/
	craneboard_ethernet_init(&craneboard_emac_pdata);
	craneboard_hecc_init(&craneboard_hecc_pdata);

	/* MMC init function */
	craneboard_mmc_init(mmc);

}

static void __init craneboard_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(CRANEBOARD, "craneboard")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= craneboard_map_io,
	.init_irq	= craneboard_init_irq,
	.init_machine	= craneboard_init,
	.timer		= &omap_timer,
MACHINE_END
