/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2013 CompuLab, Ltd.
 * Copyright (C) 2013 Jasbir
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/i2c/at24.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/gpio.h>
#include <sound/wm8962.h>
#include <linux/mfd/mxc-hdmi-core.h>
#include <linux/gpio-i2cmux.h>
#include <linux/igb.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-cm_utilite.h"


#define CM_UTILITE_USB_HUB_RST		IMX_GPIO_NR(7, 8)
#define CM_UTILITE_USB_OTG_PWR		IMX_GPIO_NR(3, 22)

#define CM_UTILITE_SATA_PWREN		IMX_GPIO_NR(1, 28)
#define CM_UTILITE_SATA_VDDC_CTRL	IMX_GPIO_NR(1, 30)
#define CM_UTILITE_SATA_LDO_EN		IMX_GPIO_NR(2, 16)
#define CM_UTILITE_SATA_nSTANDBY1	IMX_GPIO_NR(3, 20)
#define CM_UTILITE_SATA_PHY_SLP		IMX_GPIO_NR(3, 23)
#define CM_UTILITE_SATA_STBY_REQ	IMX_GPIO_NR(3, 29)
#define CM_UTILITE_SATA_nSTANDBY2	IMX_GPIO_NR(5, 2)
#define CM_UTILITE_SATA_nRSTDLY		IMX_GPIO_NR(6, 6)
#define CM_UTILITE_SATA_PWLOSS_INT	IMX_GPIO_NR(6, 31)

#define CM_UTILITE_PCIE_MUX_PWR		IMX_GPIO_NR(8, 4)

#define CM_UTILITE_ETH_RST			IMX_GPIO_NR(1, 26)
#define MX6_ENET_IRQ		    	IMX_GPIO_NR(1, 6)

#define CM_UTILITE_DVI_DDC_SEL		IMX_GPIO_NR(1, 2)
#define CM_UTILITE_DVI_HPD			IMX_GPIO_NR(1, 4)

#define CM_UTILITE_WIFI_NPD			IMX_GPIO_NR(7, 12)
#define CM_UTILITE_WIFI_NRESET		IMX_GPIO_NR(6, 16)

static struct clk *sata_clk;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern bool enet_to_gpio_6;

static const struct anatop_thermal_platform_data
	cm_utilite_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

/** Connected to AW-NH387 (WLAN/BT) **/ 
static const struct esdhc_platform_data cm_utilite_sd1_data __initconst = {
	.always_present		= 1,
	.keep_power_at_suspend	= 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct esdhc_platform_data cm_utilite_sd3_data __initconst = {
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct imx_pcie_platform_data cm_utilite_pcie_data __initconst = {
	.pcie_pwr_en	= -EINVAL,
	.pcie_rst	= CM_UTILITE_ETH_RST,
	.pcie_wake_up	= -EINVAL,
	.pcie_dis	= -EINVAL,
	.pcie_pwr_en = CM_UTILITE_PCIE_MUX_PWR,
};

static inline void cm_utilite_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(3, NULL);
}

static int cm_utilite_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* Ar8031 phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);

	/* Introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	/*check phy power*/
	val = phy_read(phydev, 0x0);

	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = cm_utilite_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
	.gpio_irq = MX6_ENET_IRQ,
};

#define EEPROM_1ST_MAC_OFF      4
#define EEPROM_BOARD_NAME_OFF   128
#define EEPROM_BOARD_NAME_LEN   16

static int eeprom_read(struct memory_accessor *mem_acc, unsigned char *buf,
		       int offset, int size, const char* objname)
{
	ssize_t ret;

	ret = mem_acc->read(mem_acc, buf, offset, size);
	if (ret != size) {
		pr_warn("CM-FX6: EEPROM %s read failed: %d\n", objname, ret);
		return ret;
	}

	return 0;
}

static void eeprom_read_mac_address(struct memory_accessor *mem_acc,
				    unsigned char *mac) {
	char *objname = "MAC address";

	if (eeprom_read(mem_acc, mac, EEPROM_1ST_MAC_OFF, ETH_ALEN, objname))
		memset(mac, 0, ETH_ALEN);
}

static void __init utilite_eeprom_1_setup(struct memory_accessor *mem_acc, void *context) {
	eeprom_read_mac_address(mem_acc, fec_data.mac);
//	printk(KERN_INFO "FEC MAC is %02X:%02X:%02X:%02X:%02X:%02X\n",fec_data.mac[0],
//		fec_data.mac[1],fec_data.mac[2],fec_data.mac[3],fec_data.mac[4],fec_data.mac[5]);

	/** Can't call "imx6_init_fec(fec_data);" because it will generate a random
        mac. Use internal imx6q_add_fec **/
	imx6q_add_fec(&fec_data);
}

static struct at24_platform_data utilite_eeprom_1_pdata __initdata = {
	.byte_len  	= 256,
	.page_size  = 16,
	.setup		= utilite_eeprom_1_setup,
};

static struct igb_platform_data utilite_igb_pdata;

static void __init utilite_eeprom_2_setup(struct memory_accessor *mem_acc, void *context) {
//	eeprom_read_mac_address(mem_acc, utilite_igb_pdata.mac_address);
//	printk(KERN_INFO "IGB MAC is %02X:%02X:%02X:%02X:%02X:%02X\n",utilite_igb_pdata.mac_address[0],
//		utilite_igb_pdata.mac_address[1],utilite_igb_pdata.mac_address[2],
//		utilite_igb_pdata.mac_address[3],utilite_igb_pdata.mac_address[4],
//		utilite_igb_pdata.mac_address[5]);

	igb_set_platform_data(&utilite_igb_pdata);
	imx6q_add_pcie(&cm_utilite_pcie_data);

}

static struct at24_platform_data utilite_eeprom_2_pdata __initdata = {
	.byte_len  	= 256,
	.page_size  = 16,
	.setup		= utilite_eeprom_2_setup,
};

static void cm_utilite_dvi_init(void)
{
	int err;

	err = gpio_request(CM_UTILITE_DVI_HPD, "dvi detect");
	if (err)
		pr_err("%s > error %d\n", __func__, err);

	gpio_direction_input(CM_UTILITE_DVI_HPD);
}

static int cm_utilite_dvi_update(void)
{
	return gpio_get_value(CM_UTILITE_DVI_HPD);
}

static struct fsl_mxc_dvi_platform_data cm_utilite_dvi_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.init		= cm_utilite_dvi_init,
	.update		= cm_utilite_dvi_update,
};


static struct imxi2c_platform_data cm_utilite_i2c_data = {
    .bitrate = 100000,
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{	/** 1st eeprom with MAC for imx6 phy **/
		I2C_BOARD_INFO("at24", 0x50),
		.platform_data = &utilite_eeprom_1_pdata,
	},
	{
		I2C_BOARD_INFO("wm8731", 0x1a),
	},
};

static struct i2c_board_info cm_utilite_i2c0c3_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("em3027", 0x56),
	},
	{	/** 2nd eeprom with MAC for IGB (PCIE) ethernet **/
		I2C_BOARD_INFO("at24", 0x50),
		.platform_data = &utilite_eeprom_2_pdata,
	}
};

static struct i2c_board_info cm_utilite_i2c0c4_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_dvi", 0x50),
		.irq = gpio_to_irq(CM_UTILITE_DVI_HPD),
		.platform_data = &cm_utilite_dvi_data,
	}
};

static void __init i2c_register_bus_binfo(int busnum,
					  struct imxi2c_platform_data *i2cdata,
					  struct i2c_board_info *info,
					  int info_size) {
	int err;
	struct platform_device *pdev;

	if (i2cdata) {
		pdev = imx6q_add_imx_i2c(busnum, i2cdata);
		if (IS_ERR(pdev))
			pr_err("%s: I2C%d register failed: %ld\n",
			       __func__, busnum, PTR_ERR(pdev));
	}

	if (info) {
		err = i2c_register_board_info(busnum, info, info_size);
		if (err)
			pr_err("%s: I2C%d board info register failed: %d\n",
			       __func__, busnum, err);
	}
}

static const unsigned cm_utilite_i2cmux_gpios[] = {
	CM_UTILITE_DVI_DDC_SEL,
};

static const unsigned cm_utilite_i2cmux_values[] = {
	0, 1,
};

static struct gpio_i2cmux_platform_data cm_utilite_i2cmux_data = {
	.parent		= 0,	/* multiplex I2C-0 */
	.base_nr	= 3,	/* create I2C-3+ */
	.values		= cm_utilite_i2cmux_values,
	.n_values	= ARRAY_SIZE(cm_utilite_i2cmux_values),
	.gpios		= cm_utilite_i2cmux_gpios,
	.n_gpios	= ARRAY_SIZE(cm_utilite_i2cmux_gpios),
	.idle		= GPIO_I2CMUX_NO_IDLE,
};

static struct platform_device cm_utilite_i2cmux = {
	.name	= "gpio-i2cmux",
	.id	= -1,
	.dev	= {
		.platform_data = &cm_utilite_i2cmux_data,
	},
};

static void cm_utilite_i2c_init(void)
{

	imx6q_add_imx_i2c(0, &cm_utilite_i2c_data);
	imx6q_add_imx_i2c(1, &cm_utilite_i2c_data);
	imx6q_add_imx_i2c(2, &cm_utilite_i2c_data);

	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	/* I2C multiplexing: I2C-0 --> I2C-3, I2C-4 */
	platform_device_register(&cm_utilite_i2cmux);

	/* register the virtual bus 3 & 4 */
	i2c_register_bus_binfo(3, NULL, cm_utilite_i2c0c3_board_info,
			       ARRAY_SIZE(cm_utilite_i2c0c3_board_info));

	i2c_register_bus_binfo(4, NULL, cm_utilite_i2c0c4_board_info,
			       ARRAY_SIZE(cm_utilite_i2c0c4_board_info));
}

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}


static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
    .disp_id = 0,
};

static struct ipuv3_fb_platform_data utilite_fb_data[] = {
	{/*fb0*/
		.disp_dev = "hdmi",
		.interface_pix_fmt = IPU_PIX_FMT_RGB32,
		.mode_str = "",
		.default_bpp = 32,
		.int_clk = false,
	}, {
	.disp_dev		= "dvi",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB32,
	.mode_str		= "1280x800@60",
	.default_bpp		= 32,
	.int_clk		= false,
	}
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
    .clk_map_ver = 2,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
		.rev = 4,
    	.csi_clk[0] = "clko2_clk",
	}, {
        .rev = 4,
        .csi_clk[0] = "clko2_clk",
    },
};

static void utilite_suspend_enter(void) {
}

static void utilite_suspend_exit(void) {
}

static const struct pm_platform_data cm_utilite_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = utilite_suspend_enter,
	.suspend_exit = utilite_suspend_exit,
};

static struct regulator_consumer_supply utilite_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data utilite_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(utilite_vmmc_consumers),
	.consumer_supplies = utilite_vmmc_consumers,
};

static struct fixed_voltage_config utilite_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &utilite_vmmc_init,
};

static struct platform_device utilite_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &utilite_vmmc_reg_config,
	},
};

static struct mxc_dvfs_platform_data utilite_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void cm_utilite_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(CM_UTILITE_USB_OTG_PWR, 1);
	else
		gpio_set_value(CM_UTILITE_USB_OTG_PWR, 0);
}

/** Keep this to avoid the usb hub from being suspended **/
static void imx6q_sabresd_host1_vbus(bool on) {
}

static void __init cm_utilite_usb_hub_reset(void) {
	int err;

	err = gpio_request_one(CM_UTILITE_USB_HUB_RST,
			       GPIOF_OUT_INIT_LOW, "usb hub rst");
	if (err) {
		pr_err("CM-FX6: usb hub rst gpio request failed: %d\n", err);
		return;
	}

	udelay(2);
	gpio_set_value(CM_UTILITE_USB_HUB_RST, 1);
}

static void __init cm_utilite_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(CM_UTILITE_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get CM_UTILITE_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(CM_UTILITE_USB_OTG_PWR, 0);

	cm_utilite_usb_hub_reset();

	mxc_iomux_set_gpr_register(1, 13, 1, 0);

	mx6_set_otghost_vbus_func(cm_utilite_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_sabresd_host1_vbus);
}

static struct gpio sata_issd_gpios[] = {
	/* The order of the GPIOs in the array is important! */
	{ CM_UTILITE_SATA_PHY_SLP,	 GPIOF_OUT_INIT_LOW,	"sata phy slp" },
	{ CM_UTILITE_SATA_nRSTDLY,	 GPIOF_OUT_INIT_LOW,	"sata nrst" },
	{ CM_UTILITE_SATA_PWREN,	 GPIOF_OUT_INIT_LOW,	"sata pwren" },
	{ CM_UTILITE_SATA_nSTANDBY1, GPIOF_OUT_INIT_LOW,	"sata nstndby1" },
	{ CM_UTILITE_SATA_nSTANDBY2, GPIOF_OUT_INIT_LOW,	"sata nstndby2" },
	{ CM_UTILITE_SATA_LDO_EN,	 GPIOF_OUT_INIT_LOW,	"sata ldo en" },
};

static void cm_utilite_sata_power_on(bool on)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sata_issd_gpios); i++) {
		gpio_set_value(sata_issd_gpios[i].gpio, on ? 1 : 0);
		udelay(100);
	}
}

/* HW Initialization, if return 0, initialization is successful. */
static int cm_utilite_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

static void cm_utilite_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);

	cm_utilite_sata_power_on(false);
	gpio_free_array(sata_issd_gpios, ARRAY_SIZE(sata_issd_gpios));
}

static struct ahci_platform_data cm_utilite_sata_data = {
	.init = cm_utilite_sata_init,
	.exit = cm_utilite_sata_exit,
};


static int cm_fx6_spdif_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_actual;

	rate_actual = clk_round_rate(clk, rate);
	clk_set_rate(clk, rate_actual);

	return 0;
}

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx		= 1,	/* enable tx */
	.spdif_rx		= 1,	/* enable rx */
	/*
	 * spdif0_clk will be 454.7MHz divided by ccm dividers.
	 *
	 * 44.1KHz: 454.7MHz / 7 (ccm) / 23 (spdif) = 44,128 Hz ~ 0.06% error
	 * 48KHz:   454.7MHz / 4 (ccm) / 37 (spdif) = 48,004 Hz ~ 0.01% error
	 * 32KHz:   454.7MHz / 6 (ccm) / 37 (spdif) = 32,003 Hz ~ 0.01% error
	 */
	.spdif_clk_44100	= 1,    /* tx clk from spdif0_clk_root */
	.spdif_clk_48000	= 1,    /* tx clk from spdif0_clk_root */
	.spdif_div_44100	= 23,
	.spdif_div_48000	= 37,
	.spdif_div_32000	= 37,
	.spdif_rx_clk		= 0,    /* rx clk from spdif stream */
	.spdif_clk_set_rate	= cm_fx6_spdif_clk_set_rate,
	.spdif_clk		= NULL, /* spdif bus clk */
};

static void __init cm_fx6_spdif_init(void)
{
	mxc_spdif_data.spdif_core_clk = clk_get_sys("mxc_spdif.0", NULL);
	clk_put(mxc_spdif_data.spdif_core_clk);
	imx6q_add_spdif(&mxc_spdif_data);
	imx6q_add_spdif_dai();
	imx6q_add_spdif_audio_device();
}


static void cm_wifi_init(void) {
	gpio_request(CM_UTILITE_WIFI_NPD, "wifi pdn");
	gpio_request(CM_UTILITE_WIFI_NRESET, "wifi rstn");
	gpio_direction_output(CM_UTILITE_WIFI_NRESET, 0);
	mdelay(1);
	gpio_direction_output(CM_UTILITE_WIFI_NPD, 1);
	gpio_direction_output(CM_UTILITE_WIFI_NRESET, 1);
}

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{
	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
    u32 value;
    value = readl(mx6_snvs_base + SNVS_LPCR);
    /*set TOP and DP_EN bit*/
    writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi) {
}


/*!
 * Board specific initialization.
 */
static void __init cm_utilite_board_init(void) {
	int i;
    struct clk *clko2;
    struct clk *new_parent;
    int rate;

	mxc_iomux_v3_setup_multiple_pads(utilite_common_pads,
        	ARRAY_SIZE(utilite_common_pads));

	if (enet_to_gpio_6) 
		fec_data.gpio_irq = -1;
	else
		fec_data.gpio_irq = -1;

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	 mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = utilite_dvfscore_data.reg_id;
	soc_reg_id = utilite_dvfscore_data.soc_id;
    pu_reg_id = utilite_dvfscore_data.pu_id;

	cm_utilite_init_uart();

    imx6q_add_mxc_hdmi_core(&hdmi_core_data);
 
	imx6q_add_ipuv3(0, &ipu_data[0]);
  	if (cpu_is_mx6q()) {
   		imx6q_add_ipuv3(1, &ipu_data[1]);
 	    for (i = 0; i < ARRAY_SIZE(utilite_fb_data); i++)
 	      imx6q_add_ipuv3fb(i, &utilite_fb_data[i]);
 	} else
    	for (i = 0; i < (ARRAY_SIZE(utilite_fb_data) + 1) / 2; i++)
	    	imx6q_add_ipuv3fb(i, &utilite_fb_data[i]);    

	
	cm_utilite_i2c_init();
 
	imx6q_add_vdoa();

	imx6q_add_anatop_thermal_imx(1, &cm_utilite_anatop_thermal_data);

	imx6q_add_pm_imx(0, &cm_utilite_pm_data);

	imx6q_add_sdhci_usdhc_imx(0, &cm_utilite_sd1_data);
	imx6q_add_sdhci_usdhc_imx(2, &cm_utilite_sd3_data);

	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);

	cm_utilite_init_usb();

	imx6q_add_mxc_hdmi(&hdmi_data);

#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &cm_utilite_sata_data);
#endif

	imx6q_add_vpu();

	platform_device_register(&utilite_vmmc_reg_devices);

	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_imx_snvs_rtc();

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&utilite_dvfscore_data);

	cm_fx6_spdif_init();

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	pm_power_off = mx6_snvs_poweroff;

	cm_wifi_init();

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);

}

static int __init cm_utilite_init_late(void)
{
	return 0;
}

device_initcall_sync(cm_utilite_init_late);

extern void __iomem *twd_base;
static void __init cm_utilite_timer_init(void) {
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
    twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
    BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

    uart_clk = clk_get_sys("imx-uart.0", NULL);
    early_console_setup(UART4_BASE_ADDR, uart_clk);
}

static struct sys_timer cm_utilite_timer = {
	.init   = cm_utilite_timer_init,
};

static void __init cm_utilite_reserve(void) {
	phys_addr_t phys;
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	if (imx6q_gpu_pdata.reserved_mem_size) {
    	phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
    		SZ_4K, SZ_1G);
    	memblock_free(phys, imx6q_gpu_pdata.reserved_mem_size);
    	memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
    	imx6q_gpu_pdata.reserved_mem_base = phys;
    }
#endif
}

/*
 * initialize __mach_desc_cm_utilite data structure.
 */
MACHINE_START(CM_UTILITE, "Compulab Utilite")
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = cm_utilite_board_init,
	.timer = &cm_utilite_timer,
	.reserve = cm_utilite_reserve,
MACHINE_END
