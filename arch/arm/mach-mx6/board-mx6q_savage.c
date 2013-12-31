/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2013 Poslab Co. Ltd All Rights Reserved.
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
#include <linux/ion.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <sound/wm8903.h>
#include <linux/mfd/mxc-hdmi-core.h>
#include <linux/memory.h>

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
#include "board-mx6q_savage.h"
#include "board-mx6dl_savage.h"

#define SAVAGE_GPIO_1_2		IMX_GPIO_NR(1, 2)
#define SAVAGE_AUD_INT		IMX_GPIO_NR(1, 4)
#define SAVAGE_MIPICSI_PWN	IMX_GPIO_NR(1, 5)
#define SAVAGE_CSI0_PWN		IMX_GPIO_NR(1, 7)
#define SAVAGE_CSI0_RST		IMX_GPIO_NR(1, 8)
#define SAVAGE_GPIO_1_9		IMX_GPIO_NR(1, 9)
#define SAVAGE_MIPICSI_RST	IMX_GPIO_NR(1, 20)
#define SAVAGE_RGMII_RST	IMX_GPIO_NR(1, 25)
#define SAVAGE_RGMII_INT	IMX_GPIO_NR(1, 26)
#define SAVAGE_RGMII_WOL	IMX_GPIO_NR(1, 28)

#define SAVAGE_SD3_CD		IMX_GPIO_NR(2, 0)
#define SAVAGE_ECSPI1_CS0	IMX_GPIO_NR(2, 26)

#define SAVAGE_PCIE_DIS_B	IMX_GPIO_NR(4, 14)

#define SAVAGE_PCIE_WAKE_B	IMX_GPIO_NR(5, 20)

#define SAVAGE_DISP_RST_B	IMX_GPIO_NR(6, 11)
#define SAVAGE_DISP_PWR_EN	IMX_GPIO_NR(6, 14)
#define SAVAGE_LVDS0_EN		IMX_GPIO_NR(6, 15)
#define SAVAGE_LVDS1_EN		IMX_GPIO_NR(6, 16)
#define SAVAGE_GPIO_6_17	IMX_GPIO_NR(6, 17)
#define SAVAGE_GPIO_6_18	IMX_GPIO_NR(6, 18)

#define SAVAGE_GPIO_7_0		IMX_GPIO_NR(7, 0)
#define SAVAGE_GPIO_7_1		IMX_GPIO_NR(7, 1)
#define SAVAGE_PCIE_RST_B_REVB	IMX_GPIO_NR(7, 12)
#define SAVAGE_PMIC_INT_B	IMX_GPIO_NR(7, 13)

static struct clk *sata_clk;
static struct clk *clko;
static int enable_lcd_ldb;
static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

static const struct esdhc_platform_data mx6q_savage_sd2_data __initconst = {
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
	.runtime_pm = 1,
};

static const struct esdhc_platform_data mx6q_savage_sd3_data __initconst = {
	.cd_gpio = SAVAGE_SD3_CD,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
	.runtime_pm = 1,
};

static const struct esdhc_platform_data mx6q_savage_sd4_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data
	mx6q_savage_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_savage_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(2, NULL);
}

static int mx6q_savage_fec_phy_init(struct phy_device *phydev)
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
	.init = mx6q_savage_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
};

static int mx6q_savage_spi_cs[] = {
	SAVAGE_ECSPI1_CS0,
};

static const struct spi_imx_master mx6q_savage_spi_data __initconst = {
	.chipselect     = mx6q_savage_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_savage_spi_cs),
};

static struct spi_board_info imx6_savage_spi_touch_device[] __initdata = {
#if defined(CONFIG_TOUCHSCREEN_AR1020_SPI)
	{
		.modalias = "ar1020-spi",
		.max_speed_hz = 300000, /* max spi clock (SCK) speed in HZ */
		.mode = SPI_MODE_1,
		.bus_num = 0,
		.chip_select = 0,
		.irq = gpio_to_irq(SAVAGE_TOUCH_INT),
		.platform_data = 0,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(imx6_savage_spi_touch_device,
				ARRAY_SIZE(imx6_savage_spi_touch_device));
}

static struct imx_ssi_platform_data mx6_savage_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_savage_audio_wm8903_device = {
	.name = "imx-wm8903",
};

static struct mxc_audio_platform_data wm8903_data;

static int wm8903_clk_enable(int enable)
{
	if (enable)
		clk_enable(clko);
	else
		clk_disable(clko);

	return 0;
}

static int mxc_wm8903_init(void)
{
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	/* both audio codec and comera use CLKO clk*/
	rate = clk_round_rate(clko, 24000000);
	clk_set_rate(clko, rate);

	wm8903_data.sysclk = rate;

	return 0;
}

static struct mxc_audio_platform_data wm8903_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_gpio = -1,
	.hp_active_low = 1,
	.mic_gpio = -1,
	.mic_active_low = 1,
	.init = mxc_wm8903_init,
	.clock_enable = wm8903_clk_enable,
};

static void mx6q_csi0_cam_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(SAVAGE_CSI0_PWN, 1);
	else
		gpio_set_value(SAVAGE_CSI0_PWN, 0);

	msleep(2);
}

static void mx6q_csi0_io_init(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_savage_csi0_sensor_pads,
			ARRAY_SIZE(mx6q_savage_csi0_sensor_pads));
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_savage_csi0_sensor_pads,
			ARRAY_SIZE(mx6dl_savage_csi0_sensor_pads));

	/* Camera reset */
	gpio_request(SAVAGE_CSI0_RST, "cam-reset");
	gpio_direction_output(SAVAGE_CSI0_RST, 1);

	/* Camera power down */
	gpio_request(SAVAGE_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(SAVAGE_CSI0_PWN, 1);
	msleep(5);
	gpio_set_value(SAVAGE_CSI0_PWN, 0);
	msleep(5);
	gpio_set_value(SAVAGE_CSI0_RST, 0);
	msleep(1);
	gpio_set_value(SAVAGE_CSI0_RST, 1);
	msleep(5);
	gpio_set_value(SAVAGE_CSI0_PWN, 1);

	/* For MX6Q:
	 * GPR1 bit19 and bit20 meaning:
	 * Bit19:       0 - Enable mipi to IPU1 CSI0
	 *                      virtual channel is fixed to 0
	 *              1 - Enable parallel interface to IPU1 CSI0
	 * Bit20:       0 - Enable mipi to IPU2 CSI1
	 *                      virtual channel is fixed to 3
	 *              1 - Enable parallel interface to IPU2 CSI1
	 * IPU1 CSI1 directly connect to mipi csi2,
	 *      virtual channel is fixed to 1
	 * IPU2 CSI0 directly connect to mipi csi2,
	 *      virtual channel is fixed to 2
	 *
	 * For MX6DL:
	 * GPR13 bit 0-2 IPU_CSI0_MUX
	 *   000 MIPI_CSI0
	 *   100 IPU CSI0
	 */
	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 19, 1, 1);
	else if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 4);
}

static struct fsl_mxc_camera_platform_data camera_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx6q_csi0_io_init,
	.pwdn = mx6q_csi0_cam_powerdown,
};

static void mx6q_mipi_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(SAVAGE_MIPICSI_PWN, 1);
	else
		gpio_set_value(SAVAGE_MIPICSI_PWN, 0);

	msleep(2);
}

static void mx6q_mipi_sensor_io_init(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_savage_mipi_sensor_pads,
			ARRAY_SIZE(mx6q_savage_mipi_sensor_pads));
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_savage_mipi_sensor_pads,
			ARRAY_SIZE(mx6dl_savage_mipi_sensor_pads));

	/* Camera reset */
	gpio_request(SAVAGE_MIPICSI_RST, "cam-reset");
	gpio_direction_output(SAVAGE_MIPICSI_RST, 1);

	/* Camera power down */
	gpio_request(SAVAGE_MIPICSI_PWN, "cam-pwdn");
	gpio_direction_output(SAVAGE_MIPICSI_PWN, 1);
	msleep(5);
	gpio_set_value(SAVAGE_MIPICSI_PWN, 0);
	msleep(5);

	gpio_set_value(SAVAGE_MIPICSI_RST, 0);
	msleep(1);
	gpio_set_value(SAVAGE_MIPICSI_RST, 1);

	msleep(5);
	gpio_set_value(SAVAGE_MIPICSI_PWN, 1);


	/*for mx6dl, mipi virtual channel 1 connect to csi 1*/
	if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 3, 3, 1);
}

static struct fsl_mxc_camera_platform_data mipi_csi2_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 1,
	.io_init = mx6q_mipi_sensor_io_init,
	.pwdn = mx6q_mipi_powerdown,
};

static struct imxi2c_platform_data mx6q_savage_i2c_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data mx6q_savage_i2c_50k_data = {
	.bitrate = 50000,
};

#ifdef CONFIG_MFD_WM831X
#include <linux/mfd/wm831x/core.h>
#include <linux/mfd/wm831x/pdata.h>
#include <linux/mfd/wm831x/gpio.h>

static struct wm831x *wm831x;

static int wm831x_pre_init(struct wm831x *parm)
{
	wm831x = parm;

	return 0;
}

static struct wm831x_pdata savage_wm8326_pdata = {
	/** Called before subdevices are set up */
	.pre_init = wm831x_pre_init,
	.irq_base = MXC_INT_END + MXC_GPIO_IRQS,


};
#endif

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("wm8903", 0x1a),
		.platform_data = 0,
		.irq = 0,
	},
#ifdef CONFIG_MFD_WM831X
	{
		I2C_BOARD_INFO("wm8326", 0x34),
		.platform_data = (void *)&savage_wm8326_pdata,
		.irq = gpio_to_irq(SAVAGE_PMIC_INT_B),
	},
#endif
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
/*
	{
		I2C_BOARD_INFO("ov5640_mipi", 0x3c),
		.platform_data = (void *)&mipi_csi2_data,
	},
*/
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_ldb_i2c", 0x50),
		.platform_data = (void *)1,	/* lvds port1 */
	},
};

static void imx6q_savage_usbotg_vbus(bool on)
{
}

static void __init imx6q_savage_init_usb(void)
{
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */

	mxc_iomux_set_gpr_register(1, 13, 1, 0);	/*  use ENET_RX_ER */
	mx6_set_otghost_vbus_func(imx6q_savage_usbotg_vbus);

}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_savage_sata_init(struct device *dev, void __iomem *addr)
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
static void mx6q_savage_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_savage_sata_data = {
	.init = mx6q_savage_sata_init,
	.exit = mx6q_savage_sata_exit,
};
#endif

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M + SZ_64M - SZ_16M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static void mx6_reset_mipi_dsi(void)
{
	gpio_set_value(SAVAGE_DISP_PWR_EN, 1);
	gpio_set_value(SAVAGE_DISP_RST_B, 1);
	udelay(10);
	gpio_set_value(SAVAGE_DISP_RST_B, 0);
	udelay(50);
	gpio_set_value(SAVAGE_DISP_RST_B, 1);

	/*
	 * it needs to delay 120ms minimum for reset complete
	 */
	msleep(120);
}

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id		= 0,
	.disp_id	= 1,
	.lcd_panel	= "TRULY-WVGA",
	.reset		= mx6_reset_mipi_dsi,
};

static struct ipuv3_fb_platform_data savage_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "hdmi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1920x1080M@60",
	.default_bpp = 32,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
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

/* On mx6x savage board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_savage_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_savage_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_savage_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_savage_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_savage_i2c2_pads,
			ARRAY_SIZE(mx6dl_savage_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_savage_i2c2_pads,
			ARRAY_SIZE(mx6q_savage_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 1,
	.disp_id = 0,
};
/*
static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};
*/
static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 0,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SEP1,
	.sec_ipu_id = 0,
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

static struct ion_platform_data imx_ion_data = {
	.nr = 1,
	.heaps = {
		{
		.id = 0,
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "vpu_ion",
		.size = SZ_16M,
		.cacheable = 1,
		},
	},
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


static void savage_suspend_enter(void)
{
	/* suspend preparation */
}

static void savage_suspend_exit(void)
{
	/* resume restore */
}
static const struct pm_platform_data mx6q_savage_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = savage_suspend_enter,
	.suspend_exit = savage_suspend_exit,
};

static struct regulator_consumer_supply savage_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data savage_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(savage_vmmc_consumers),
	.consumer_supplies = savage_vmmc_consumers,
};

static struct fixed_voltage_config savage_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &savage_vmmc_init,
};

static struct platform_device savage_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &savage_vmmc_reg_config,
	},
};

static int __init imx6q_init_audio(void)
{
	{
		mxc_register_device(&mx6_savage_audio_wm8903_device,
				    &wm8903_data);
		imx6q_add_imx_ssi(1, &mx6_savage_ssi_pdata);

		mxc_wm8903_init();
	}

	return 0;
}

static void __init imx6q_add_device_gpio_leds(void) {}

static void __init imx6q_add_device_buttons(void) {}

static struct platform_pwm_backlight_data mx6_savage_pwm_backlight_data0 = {
	.pwm_id = 2,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data savage_dvfscore_data = {
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

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;
	struct ipuv3_fb_platform_data *pdata_fb = savage_fb_data;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(savage_fb_data)) {
					str++;
					pdata_fb[i++].res_size[0] = memparse(str, &str);
				}
			}
			/* ION reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "ionmem=");
			if (str != NULL) {
				str += 7;
				imx_ion_data.heaps[0].size = memparse(str, &str);
			}
			/* Primary framebuffer base address */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fb0base=");
			if (str != NULL) {
				str += 8;
				pdata_fb[0].res_base[0] =
						simple_strtol(str, &str, 16);
			}
			/* GPU reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpumem=");
			if (str != NULL) {
				str += 7;
				imx6q_gpu_pdata.reserved_mem_size = memparse(str, &str);
			}
			break;
		}
	}
}

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 1,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

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

static const struct imx_pcie_platform_data mx6_savage_pcie_data __initconst = {
	.pcie_pwr_en	= 0,
	.pcie_rst	= SAVAGE_PCIE_RST_B_REVB,
	.pcie_wake_up	= SAVAGE_PCIE_WAKE_B,
	.pcie_dis	= SAVAGE_PCIE_DIS_B,
	.pcie_power_always_on = 1,
};

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource = {
	.name = "android ram console",
	.flags = IORESOURCE_MEM,
};

static struct platform_device android_ram_console = {
	.name = "ram_console",
	.num_resources = 1,
	.resource = &ram_console_resource,
};

static int __init imx6x_add_ram_console(void)
{
	return platform_device_register(&android_ram_console);
}
#else
#define imx6x_add_ram_console() do {} while (0)
#endif

/*!
 * Board specific initialization.
 */
static void __init mx6_savage_board_init(void)
{
	int i;
	int ret;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;

	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_savage_pads,
			ARRAY_SIZE(mx6q_savage_pads));
	else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_savage_pads,
			ARRAY_SIZE(mx6dl_savage_pads));
	}

#if 0 //def CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	 mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = savage_dvfscore_data.reg_id;
	soc_reg_id = savage_dvfscore_data.soc_id;
	mx6q_savage_init_uart();
	imx6x_add_ram_console();

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.disp_id = 1;
		hdmi_core_data.ipu_id = 0;
		hdmi_core_data.disp_id = 0;
		mipi_dsi_pdata.ipu_id = 0;
		mipi_dsi_pdata.disp_id = 1;
		ldb_data.sec_ipu_id = 0;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(savage_fb_data); i++)
			imx6q_add_ipuv3fb(i, &savage_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(savage_fb_data); i++)
			imx6q_add_ipuv3fb(i, &savage_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_mipi_dsi(&mipi_dsi_pdata);
//	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	gpio_request(SAVAGE_LVDS0_EN, "lvds0-en");
	gpio_request(SAVAGE_LVDS1_EN, "lvds1-en");
	gpio_direction_output(SAVAGE_LVDS0_EN, 1);
	gpio_direction_output(SAVAGE_LVDS1_EN, 1);
	msleep(1);
	gpio_set_value(SAVAGE_LVDS0_EN, 1);
	gpio_set_value(SAVAGE_LVDS1_EN, 1);
/*
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
*/
	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_device_gpio_leds();

#ifdef CONFIG_MFD_WM831X
    ret = gpio_request(SAVAGE_PMIC_INT_B, "wm831x-int");
 	if (ret)
		printk(KERN_ERR"request wm831x-int error!!\n");

    gpio_direction_input(SAVAGE_PMIC_INT_B);
#endif

	imx6q_add_imx_i2c(0, &mx6q_savage_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_savage_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_savage_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	/* SPI */
	imx6q_add_ecspi(0, &mx6q_savage_spi_data);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_savage_anatop_thermal_data);
	imx6_init_fec(fec_data);
	imx6q_add_pm_imx(0, &mx6q_savage_pm_data);

	/* Move sd4 to first because sd4 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	*/
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_savage_sd4_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_savage_sd3_data);
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_savage_sd2_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_savage_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_savage_sata_data);
#else
		mx6q_savage_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&savage_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(0, &mx6_savage_pwm_backlight_data0);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&savage_dvfscore_data);

	if (imx_ion_data.heaps[0].size)
		imx6q_add_ion(0, &imx_ion_data,
			sizeof(imx_ion_data) + sizeof(struct ion_platform_heap));

	imx6q_add_device_buttons();

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

	/* Register charger chips */
	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&mx6_savage_pcie_data);

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init mx6_savage_timer_init(void)
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

static struct sys_timer mx6_savage_timer = {
	.init   = mx6_savage_timer_init,
};

static void __init mx6q_savage_reserve(void)
{
	phys_addr_t phys;
	int i, fb0_reserved = 0, fb_array_size;

	/*
	 * Reserve primary framebuffer memory if its base address
	 * is set by kernel command line.
	 */
	fb_array_size = ARRAY_SIZE(savage_fb_data);
	if (fb_array_size > 0 && savage_fb_data[0].res_base[0] &&
	    savage_fb_data[0].res_size[0]) {
		if (savage_fb_data[0].res_base[0] > SZ_2G)
			printk(KERN_INFO"UI Performance downgrade with FB phys address %x!\n",
			    savage_fb_data[0].res_base[0]);
		memblock_reserve(savage_fb_data[0].res_base[0],
				 savage_fb_data[0].res_size[0]);
		memblock_remove(savage_fb_data[0].res_base[0],
				savage_fb_data[0].res_size[0]);
		savage_fb_data[0].late_init = true;
		ipu_data[ldb_data.ipu_id].bypass_reset = true;
		fb0_reserved = 1;
	}
	for (i = fb0_reserved; i < fb_array_size; i++)
		if (savage_fb_data[i].res_size[0]) {
			/* Reserve for other background buffer. */
			phys = memblock_alloc_base(savage_fb_data[i].res_size[0],
						SZ_4K, SZ_2G);
			memblock_remove(phys, savage_fb_data[i].res_size[0]);
			savage_fb_data[i].res_base[0] = phys;
		}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	phys = memblock_alloc_base(SZ_1M, SZ_4K, SZ_1G);
	memblock_remove(phys, SZ_1M);
	memblock_free(phys, SZ_1M);
	ram_console_resource.start = phys;
	ram_console_resource.end   = phys + SZ_1M - 1;
#endif

#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_2G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

#if defined(CONFIG_ION)
	if (imx_ion_data.heaps[0].size) {
		phys = memblock_alloc(imx_ion_data.heaps[0].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[0].size);
		imx_ion_data.heaps[0].base = phys;
	}
#endif
}

/*
 * initialize __mach_desc_MX6Q_SAVAGE data structure.
 */
MACHINE_START(MX6Q_SAVAGE, "Poslab i.MX6 Savage Board")
	/* Maintainer: Poslab Co. Ltd */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_savage_board_init,
	.timer = &mx6_savage_timer,
	.reserve = mx6q_savage_reserve,
MACHINE_END
