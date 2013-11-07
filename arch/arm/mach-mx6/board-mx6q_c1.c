/*
 * Copyright (C) 2013 SolidRun ltd.
 * Based on sabresd board from Freescale Semiconductor, Inc. All Rights Reserved
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
#include <linux/mfd/mxc-hdmi-core.h>
#ifdef CONFIG_IR_GPIO_CIR
#include <media/gpio-ir-recv.h>
#endif

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
#include "board-mx6q_c1.h"
#include "board-mx6dl_c1.h"

#define USOM_WL_RST		IMX_GPIO_NR(5, 17)
#define USOM_BT_RST		IMX_GPIO_NR(6, 16)
#define USOM_REG_ON		IMX_GPIO_NR(3, 19)
#define USOM_XTAL_ON		IMX_GPIO_NR(5, 13)
#define USOM_SD2_CD		IMX_GPIO_NR(1, 4)
#define GPIO_IR_IN		IMX_GPIO_NR(1, 2) /* IMX_GPIO_NR(3, 9) for CuBox-i */
#define USOM_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define USOM_USB_H1_PWR		IMX_GPIO_NR(1, 0)
#define USOM_ENET_RST		IMX_GPIO_NR(4, 15)
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
#define MX6_ENET_IRQ		IMX_GPIO_NR(4, 18) /* TODO */
#endif

static struct clk *sata_clk;
static struct clk *clko;
static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

static const struct esdhc_platform_data mx6q_c1_sd2_data __initconst = {
	.cd_gpio = USOM_SD2_CD,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_GPIO,
};

static const struct esdhc_platform_data mx6q_c1_sd1_data __initconst = {
	.cd_gpio = -EINVAL,
	.wp_gpio = -EINVAL,
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data
	mx6q_c1_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static int mx6q_c1_fec_ar8035_phy_init(struct phy_device *phydev)
{
	unsigned short val;
	/* Ar803x phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	gpio_set_value(USOM_ENET_RST, 1);
	mdelay(10);
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);
	/* To enable AR8035 ouput a 125MHz clk from CLK_25M */
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
static int mx6q_c1_fec_ar8030_phy_init(struct phy_device *phydev)
{
	unsigned short val;
	/* Ar803x phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	gpio_set_value(USOM_ENET_RST, 1);
	mdelay(10);
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);
	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}

static struct fec_platform_data fec_data_rgmii __initdata = {
	.init = mx6q_c1_fec_ar8035_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	.gpio_irq = MX6_ENET_IRQ,
#endif
};
static struct fec_platform_data fec_data_rmii __initdata = {
	.init = mx6q_c1_fec_ar8030_phy_init,
	.phy = PHY_INTERFACE_MODE_RMII,
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	.gpio_irq = MX6_ENET_IRQ,
#endif
};

static struct imxi2c_platform_data mx6q_c1_i2c_data = {
	.bitrate = 100000,
};

/* I2C1 */
static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("pcf8523", 0x68),
	},
};

/* I2C2 / HDMI DDC */
static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

/* I2C3 */
static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
};

static void imx6q_c1_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(USOM_USB_OTG_PWR, 1);
	else
		gpio_set_value(USOM_USB_OTG_PWR, 0);
}

static void imx6q_c1_host1_vbus(bool on)
{
	if (on)
		gpio_set_value(USOM_USB_H1_PWR, 1);
	else
		gpio_set_value(USOM_USB_H1_PWR, 0);
}

static void __init imx6q_c1_init_usb(void)
{
	int ret = 0;
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(USOM_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO USOM_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(USOM_USB_OTG_PWR, 0);
	/* keep USB host1 VBUS always on */
	ret = gpio_request(USOM_USB_H1_PWR, "usb-h1-pwr");
	if (ret) {
		pr_err("failed to get GPIO USOM_USB_H1_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(USOM_USB_H1_PWR, 0);
	/*
	 * ID pin is sampled from GPIO_1. Notice that this pad is configured
	 * to be pulled-down 100kOhm by default.
	 */
	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6q_c1_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_c1_host1_vbus);
}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_c1_sata_init(struct device *dev, void __iomem *addr)
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

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_c1_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_c1_sata_data = {
	.init = mx6q_c1_sata_init,
	.exit = mx6q_c1_sata_exit,
};
#endif

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data c1_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	.mode_str = "CLAA-WVGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-VGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	},
};

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

/* On mx6x c1 board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_c1_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_c1_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_c1_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_c1_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_c1_i2c2_pads,
			ARRAY_SIZE(mx6dl_c1_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_c1_i2c2_pads,
			ARRAY_SIZE(mx6q_c1_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 0,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SEP1,
	.sec_ipu_id = 1,
	.sec_disp_id = 0,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

static int spdif_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_actual;
	rate_actual = clk_round_rate(clk, rate);
	printk ("Called to set rate %s, rate = %ld, actual = %ld\n",__FUNCTION__,rate,rate_actual);
	clk_set_rate(clk, rate_actual);
	return 0;
}

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx		= 1,		/* enable tx */
	.spdif_rx		= 0,		/* enable rx */
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
	.spdif_clk_set_rate	= spdif_clk_set_rate,
	.spdif_clk		= NULL, /* spdif bus clk */
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};

static struct imx_esai_platform_data sab_esai_pdata = {
	.flags	= IMX_ESAI_NET,
};
static int __init imx6q_init_audio(void)
{
	struct clk *pll3_pfd, *esai_clk;
	imx6q_add_imx_esai(0, &sab_esai_pdata);

	esai_clk = clk_get(NULL, "esai_clk");
	if (IS_ERR(esai_clk))
		return PTR_ERR(esai_clk);

	pll3_pfd = clk_get(NULL, "pll3_pfd_508M");
	if (IS_ERR(pll3_pfd))
		return PTR_ERR(pll3_pfd);

	clk_set_parent(esai_clk, pll3_pfd);
	clk_set_rate(esai_clk, 101647058);

	return 0;
}

#ifdef CONFIG_IR_GPIO_CIR
static struct gpio_ir_recv_platform_data c1_ir_data = {
	.gpio_nr = GPIO_IR_IN,
	.active_low = 1,
};

static struct platform_device c1_ir = {
        .name   = "gpio-rc-recv",
	.id     = -1,
	.dev    = {
		.platform_data  = &c1_ir_data,
	}
};
#endif

#if 0
/* Following will activate the analog audio out for testing */
static struct platform_pwm_backlight_data mx6_c1_pwm_dummy1_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 200,
	.dft_brightness = 128,
	.pwm_period_ns = 5000000,
};

static struct platform_pwm_backlight_data mx6_c1_pwm_dummy2_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 201,
	.dft_brightness = 128,
	.pwm_period_ns = 1000000,
};
#endif

static struct platform_pwm_backlight_data mx6_c1_pwm_lvds_backlight_data = {
	.pwm_id = 2,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct platform_pwm_backlight_data mx6_c1_pwm_dsi_backlight_data = {
	.pwm_id = 3,
	.max_brightness = 203,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

static const struct imx_pcie_platform_data mx6_c1_pcie_data __initconst = {
	.pcie_pwr_en	= -EINVAL,
	.pcie_rst	= -EINVAL,
	.pcie_wake_up	= -EINVAL,
	.pcie_dis	= -EINVAL,
};

static const struct imxuart_platform_data usom_bt_uart_data = {
        .flags = IMXUART_HAVE_RTSCTS | IMXUART_SDMA,
        .dma_req_tx = MX6Q_DMA_REQ_UART4_TX,
        .dma_req_rx = MX6Q_DMA_REQ_UART4_RX,
};

/*
 * Board specific initialization.
 */
static void __init mx6_c1_board_init(void)
{
	int i;
	struct clk *clko, *clko2, *enet;
	struct clk *new_parent;
	int rate;

	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_c1_pads,
			ARRAY_SIZE(mx6q_c1_pads));
	else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_c1_pads,
			ARRAY_SIZE(mx6dl_c1_pads));
	}

	imx6q_add_imx_uart(0, NULL);

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.sec_ipu_id = 0;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(c1_fb_data); i++)
			imx6q_add_ipuv3fb(i, &c1_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(c1_fb_data); i++)
			imx6q_add_ipuv3fb(i, &c1_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_imx_i2c(0, &mx6q_c1_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_c1_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_c1_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info, /* I2C1 */
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info, /* I2C2 / HDMI DDC */
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info, /* I2C3 */
			ARRAY_SIZE(mxc_i2c2_board_info));
	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_c1_anatop_thermal_data);
	/* Set GPR1, bit 21 to 1 */
	mxc_iomux_set_gpr_register(1, 21, 1, 1);
	/* Set enet clock to 50MHz RMII */
	enet = clk_get_sys("enet.0", NULL);
	if (IS_ERR(enet))
		pr_err("Unable to get enet.0 clock\n");
	else {
		clk_prepare(enet);
		clk_set_rate(enet, 50000000);
		clk_enable(enet);
	}
#if 1
	mxc_iomux_v3_setup_multiple_pads(mx6dl_ar8035_phy, ARRAY_SIZE(mx6dl_ar8035_phy));
#else
	mxc_iomux_v3_setup_multiple_pads(mx6dl_ar8030_phy, ARRAY_SIZE(mx6dl_ar8030_phy));
#endif
	gpio_request(USOM_ENET_RST, "eth-phy-rst");
	udelay(1000); // Maybe not needed since 0 value is already asserted (pull down)
	gpio_direction_output(USOM_ENET_RST, 0);
	udelay(2000); // Maybe not needed since 0 value is already asserted (pull down)
	gpio_set_value(USOM_ENET_RST, 1);
	mdelay(10);
#if 1
	imx6_init_fec(fec_data_rgmii);
#else
	imx6_init_fec(fec_data_rmii);
#endif
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
	mxc_iomux_set_specialbits_register(IOMUX_OBSRV_MUX1_OFFSET,
		OBSRV_MUX1_ENET_IRQ, OBSRV_MUX1_MASK);
#endif
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_c1_sd2_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_c1_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_c1_sata_data);
#else
		mx6q_c1_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	imx6q_init_audio();
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(0, &mx6_c1_pwm_lvds_backlight_data);
	imx6q_add_mxc_pwm_backlight(1, &mx6_c1_pwm_dsi_backlight_data);
#if 0
	imx6q_add_mxc_pwm_backlight(2, &mx6_c1_pwm_dummy1_backlight_data);
	imx6q_add_mxc_pwm_backlight(3, &mx6_c1_pwm_dummy2_backlight_data);
#endif
	mxc_spdif_data.spdif_core_clk = clk_get_sys("mxc_spdif.0", NULL);
	clk_put(mxc_spdif_data.spdif_core_clk);
	imx6q_add_spdif(&mxc_spdif_data);
	imx6q_add_spdif_dai();
	imx6q_add_spdif_audio_device();

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}

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

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);

	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&mx6_c1_pcie_data);
	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
#ifdef CONFIG_IR_GPIO_CIR
	/* Register the infra red receiver as a GPIO device */
	platform_device_register(&c1_ir);
#endif

	/* WLan and BT stuff */
	/* init and put in reset */
	gpio_request(USOM_WL_RST, "wl-rst");
	gpio_direction_output(USOM_WL_RST, 0);
	gpio_request(USOM_BT_RST, "bt-rst");
	gpio_direction_output(USOM_BT_RST, 0);
	gpio_request(USOM_REG_ON, "wl-bt-reg-on");
	gpio_direction_output(USOM_REG_ON, 0);
	gpio_request(USOM_XTAL_ON, "wl-bt-xtal-on");
	gpio_direction_output(USOM_XTAL_ON, 0);
	msleep(100);
	gpio_set_value(USOM_REG_ON, 1);
	gpio_set_value(USOM_XTAL_ON, 1);

	
	/* Now release from reset */
	msleep (200); /* 20 mSec sounds too big */
	gpio_set_value(USOM_WL_RST, 1);
	gpio_set_value(USOM_BT_RST, 1);
	msleep (200); /* 20 mSec sounds too big */

	/* Register SDIO as brfmac */
        imx6q_add_imx_uart(3, &usom_bt_uart_data);
	imx6q_add_sdhci_usdhc_imx(0, &mx6q_c1_sd1_data);
}

extern void __iomem *twd_base;
static void __init mx6_c1_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_c1_timer = {
	.init   = mx6_c1_timer_init,
};

static void __init mx6q_c1_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
}

/*
 * initialize __mach_desc_MX6Q_C1 data structure.
 */
MACHINE_START(C1, "SolidRun i.MX 6Quad/Dual/DualLite/Solo Carrier One Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_c1_board_init,
	.timer = &mx6_c1_timer,
	.reserve = mx6q_c1_reserve,
MACHINE_END
