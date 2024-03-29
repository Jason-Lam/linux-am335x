/*
 * Code for IPC335X.
 *
 * Copyright (C) 2012 EMA-Tech - http://www.ema-tech.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/i2c/at24.h>
#include <linux/phy.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/ethtool.h>
#include <linux/mfd/tps65910.h>
#include <linux/mfd/ti_tscadc.h>
#include <linux/reboot.h>
#include <linux/pwm/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/input.h>
#include <linux/input/ti_tsc.h>
#include <linux/gpio_keys.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/cpu_pm.h>

/* LCD controller is similar to DA850 */
#include <video/da8xx-fb.h>

#include <mach/hardware.h>
#include <mach/board-ipc335x.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/lcdc.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>
#include <plat/omap-serial.h>
#include <plat/nand.h>

#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
#include "hsmmc.h"
#include "common.h"
#include "board-flash.h"

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

static int ipc33xx_dockid = -EINVAL;

/*
** ipc335x_dock_set_id - set up board dockid
** @dockid - dock id which needs to be configured
**
** This function is called to configure board dock id.
*/
void ipc335x_dock_set_id(unsigned int dockid)
{
	ipc33xx_dockid = dockid;
	return;
}

/*
** ipc335x_dock_get_id - returns Board Type (EVM/HMI ...)
**
** Note:
**	returns -EINVAL if Board detection hasn't happened yet.
*/
int ipc335x_dock_get_id(void)
{
	return ipc33xx_dockid;
}
EXPORT_SYMBOL(ipc335x_dock_get_id);

/*
** Board Config held in On-Board eeprom device.
**
** Header Format
**
**  Name			Size	Contents
**			(Bytes)
**-------------------------------------------------------------
**  Header		4	0xAA, 0x55, 0x33, 0xEE
**
**  Board Name		8	Name for board in ASCII.
**				Example "IPC335X" = "IPC335x core board"
**
**  Version		2	Hardware version code for board	in ASCII.
**				"A1" = rev.A.1
**
**  Profile		1	Profile of the board.
**
**  Serial Number	12	Serial number of the board. This is a 12
**				character string which is WWYY4P16nnnn, where
**				WW = 2 digit week of the year of production
**				YY = 2 digit year of production
**				nnnn = incrementing board number
**
**  Configuration option	32	Codes(TBD) to show the configuration
**				setup on this board.
**
**  Available		32640	Available space for other non-volatile data.
*/
struct ipc335x_eeprom_config {
	u32	header;
	u8	name[8];
	u8	version[2];
	u8	profile;
	u8	serial[12];
	u8	opt[32];
};

/* 32KByte eeprom on board 128bytes reserve for board config*/
#define EEPROM_BOARD_CFG_OFFSET		(32768 - 128 - 1)
#define EEPROM_MAC_ADDRESS_OFFSET	(EEPROM_BOARD_CFG_OFFSET + 64)
#define AM335X_EEPROM_HEADER		0xEE3355AA
#define EEPROM_NO_OF_MAC_ADDR		3
static char ipc335x_mac_addr[EEPROM_NO_OF_MAC_ADDR][ETH_ALEN];

struct ipc335x_dev_cfg {
	void (*device_init)(int board_type, u8 profile);
	int board_type;	/* Mach board type*/
	u32 profile;	/* Profiles (0-7) in which the module is present */
};

static void _configure_device(int board_type, struct ipc335x_dev_cfg *dev_cfg,
	u8 profile)
{
	for (; dev_cfg->device_init != NULL; dev_cfg++) {
		if (dev_cfg->board_type == board_type)
			if (dev_cfg->profile & profile)
				dev_cfg->device_init(board_type, profile);
	}
}

static struct omap2_hsmmc_info ipc335x_mmc[] = {
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{}      /* Terminator */
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 * USB0 in OTG mode and USB1 in host mode.
	 */
	.mode           = (MUSB_HOST << 4) | MUSB_OTG,
	.power		= 500,
	.instances	= 1,
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
#endif

/* module pin mux structure */
struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
};

static struct omap_board_config_kernel ipc335x_config[] __initdata = {
};

/* Module pin mux for rgmii1 */
static struct pinmux_config rgmii1_pin_mux[] = {
	{"mii1_txen.rgmii1_tctl", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_rxdv.rgmii1_rctl", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txd3.rgmii1_td3", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd2.rgmii1_td2", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.rgmii1_td1", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.rgmii1_td0", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txclk.rgmii1_tclk", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_rxclk.rgmii1_rclk", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd3.rgmii1_rd3", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd2.rgmii1_rd2", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd1.rgmii1_rd1", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.rgmii1_rd0", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config i2c1_pin_mux[] = {
	{"spi0_d1.i2c1_sda",    OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{"spi0_cs0.i2c1_scl",   OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{NULL, 0},
};

static struct pinmux_config i2c2_ipc335x_core_pin_mux[] = {
	{"uart1_ctsn.i2c2_sda",    OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{"uart1_rtsn.i2c2_scl",   OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{NULL, 0},
};

static struct pinmux_config i2c2_som335x_core_pin_mux[] = {
	{"spi0_sclk.i2c2_sda",    OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{"spi0_d0.i2c2_scl",   OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{NULL, 0},
};

static struct pinmux_config mmc0_common_pin_mux[] = {
	{"mmc0_dat3.mmc0_dat3",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat2.mmc0_dat2",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat1.mmc0_dat1",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat0.mmc0_dat0",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_clk.mmc0_clk",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_cmd.mmc0_cmd",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for mmc1 */
static struct pinmux_config mmc1_common_pin_mux[] = {
	{"gpmc_ad3.mmc1_dat3",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad2.mmc1_dat2",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad1.mmc1_dat1",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad0.mmc1_dat0",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn1.mmc1_clk",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn2.mmc1_cmd",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config mmc1_cd_pin_mux[] = {
	{"usb1_drvvbus.gpio3_13", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config d_can_ipc335x_evm_pin_mux[] = {
	{"uart1_rxd.d_can1_tx", OMAP_MUX_MODE2 | AM33XX_PULL_ENBL},
	{"uart1_txd.d_can1_rx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config uart1_pin_mux[] = {
	{"uart1_rxd.uart1_rxd", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"uart1_txd.uart1_txd", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL},
	{NULL, 0},
};

static struct pinmux_config d_can_hmi335x_pin_mux[] = {
	{"uart1_ctsn.d_can0_tx", OMAP_MUX_MODE2 | AM33XX_PULL_ENBL},
	{"uart1_rtsn.d_can0_rx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/*
* @pin_mux - single module pin-mux structure which defines pin-mux
*			details for all its pins.
*/
static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);

}

/* pinmux for usb0 drvvbus */
static struct pinmux_config usb0_pin_mux[] = {
	{"usb0_drvvbus.usb0_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* pinmux for hmi335x 3v3 ctrl */
static struct pinmux_config hmi335x_3v3_pin_mux[] = {
	/*Board power ctrl*/
	{"usb0_drvvbus.gpio0_18",    OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	/*usb hub Nrest*/
	{"gpmc_clk.gpio2_1",    OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	/*wifi Npower*/
	{"gpmc_ben1.gpio1_28",    OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void uart1_init(int board_type, u8 profile)
{
	setup_pin_mux(uart1_pin_mux);
}

static void rgmii1_init(int board_type, u8 profile)
{
	setup_pin_mux(rgmii1_pin_mux);
	return;
}

static int hmi335x_3v3_enable = -1;
static int hmi335x_usb_reset =	-1;
static int hmi335x_wifi_power_ctrl = -1;
static void hmi_power_ctrl(int enable)
{
    if(gpio_is_valid(hmi335x_3v3_enable)){
        gpio_set_value(hmi335x_3v3_enable, enable);
    }
}

static int hmi335x_power_event(struct notifier_block *self, unsigned long cmd,	void *v)
{
	switch (cmd) {
	case CPU_CLUSTER_PM_ENTER:
        hmi_power_ctrl(0);
		break;
	case CPU_CLUSTER_PM_EXIT:
        hmi_power_ctrl(1);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block hmi335x_pm_notifier = {
	.notifier_call = hmi335x_power_event,
};

static void hmi335x_power_init(int board_type, u8 profile)
{
	hmi335x_3v3_enable = GPIO_TO_PIN(0, 18),
	setup_pin_mux(hmi335x_3v3_pin_mux);
	gpio_request(hmi335x_3v3_enable, "dock_3v3power_enable");
	gpio_direction_output(hmi335x_3v3_enable, 0);
	//cpu_pm_register_notifier(&hmi335x_pm_notifier);
	hmi335x_usb_reset = GPIO_TO_PIN(2, 1),
	gpio_request(hmi335x_usb_reset, "dock_usb_Nreset");
	/*Hub stay reset*/
	gpio_direction_output(hmi335x_usb_reset, 0);
	gpio_export(hmi335x_usb_reset, 0);

	hmi335x_wifi_power_ctrl = GPIO_TO_PIN(1, 28),
	gpio_request(hmi335x_wifi_power_ctrl, "hmi335x_wifi_Npower");
	/*wifi power down*/
	gpio_direction_output(hmi335x_wifi_power_ctrl, 1);
	gpio_export(hmi335x_wifi_power_ctrl, 0);

	gpio_direction_output(hmi335x_usb_reset, 1);
	gpio_direction_output(hmi335x_wifi_power_ctrl, 0);
}

static void usb0_init(int board_type, u8 profile)
{
	setup_pin_mux(usb0_pin_mux);
	return;
}

static void mmc1_init(int board_type, u8 profile)
{
	setup_pin_mux(mmc1_common_pin_mux);

	switch (board_type){
	case IPC335X_EVM:
	case HMI335X:
		/* 3,13 gpio used in ddr vtt enable,so disable mmc_cd
		 * setup_pin_mux(mmc1_cd_pin_mux);
		 * ipc335x_mmc[1].gpio_cd = GPIO_TO_PIN(3, 13);
		*/
		ipc335x_mmc[1].mmc = 2;
		ipc335x_mmc[1].caps = MMC_CAP_4_BIT_DATA;
		ipc335x_mmc[1].gpio_cd = -EINVAL;
		ipc335x_mmc[1].gpio_wp = -EINVAL;
		ipc335x_mmc[1].ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34;
		break;
	}
	return;
}

static void d_can_init(int board_type, u8 profile)
{
	switch (board_type){
	case IPC335X_EVM:
		setup_pin_mux(d_can_ipc335x_evm_pin_mux);
		am33xx_d_can_init(1);
		break;
	case HMI335X:
		setup_pin_mux(d_can_hmi335x_pin_mux);
		am33xx_d_can_init(0);
		break;
	}
	return;
}

static void mmc0_init(int board_type, u8 profile)
{
	switch (board_type){
	case IPC335X_CORE:
	case SOM335X_CORE:
		setup_pin_mux(mmc0_common_pin_mux);
		ipc335x_mmc[0].gpio_cd = -EINVAL;
		ipc335x_mmc[0].gpio_wp = -EINVAL;
		ipc335x_mmc[0].nonremovable = true;
		ipc335x_mmc[0].ocr_mask = MMC_VDD_165_195 | MMC_VDD_32_33
			| MMC_VDD_33_34;
		break;
	}
	omap2_hsmmc_init(ipc335x_mmc);
	return;
}

static struct i2c_board_info am335x_i2c_boardinfo2[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x1b),
	},
	{
		I2C_BOARD_INFO("wm8960", 0x1a),
	},
	{
		I2C_BOARD_INFO("ctp-touch", 0x70),
	},
};

static void i2c2_init(int board_type, u8 profile)
{
	switch (board_type){
	case IPC335X_CORE:
		setup_pin_mux(i2c2_ipc335x_core_pin_mux);
		break;
	case SOM335X_CORE:
		setup_pin_mux(i2c2_som335x_core_pin_mux);
		break;
	}
	omap_register_i2c_bus(3, 100, am335x_i2c_boardinfo2,
		ARRAY_SIZE(am335x_i2c_boardinfo2));

	return;
}

/* Module pin mux for mcasp0 */
static struct pinmux_config mcasp0_ipc335x_evm_pin_mux[] = {
	{"mcasp0_aclkx.mcasp0_aclkx", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_fsx.mcasp0_fsx", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_axr0.mcasp0_axr0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_axr1.mcasp0_axr1", OMAP_MUX_MODE0 |
						AM33XX_PIN_INPUT_PULLDOWN},
	{NULL, 0},
};

static struct pinmux_config mcasp1_hmi335x_pin_mux[] = {
	{"mcasp0_aclkr.mcasp1_aclkx", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_fsr.mcasp1_fsx", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_axr1.mcasp1_axr0", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_ahclkx.mcasp1_axr1", OMAP_MUX_MODE3 |
						AM33XX_PIN_INPUT_PULLDOWN},
	{NULL, 0},
};

static u8 ipc335x_evm_iis_serializer_direction[] = {
	TX_MODE,	RX_MODE,	INACTIVE_MODE,  INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static u8 hmi335x_iis_serializer_direction[] = {
	TX_MODE,	RX_MODE,	INACTIVE_MODE,  INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data ipc335x_evm_snd_data = {
	.tx_dma_offset	= 0x46000000,	/* McASP0 */
	.rx_dma_offset	= 0x46000000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer	= ARRAY_SIZE(ipc335x_evm_iis_serializer_direction),
	.tdm_slots	= 2,
	.serial_dir	= ipc335x_evm_iis_serializer_direction,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_3,
	.txnumevt	= 1,
	.rxnumevt	= 1,
};

static struct snd_platform_data hmi335x_snd_data = {
	.tx_dma_offset	= 0x46400000,	/* McASP1 */
	.rx_dma_offset	= 0x46400000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer	= ARRAY_SIZE(hmi335x_iis_serializer_direction),
	.tdm_slots	= 2,
	.serial_dir	= hmi335x_iis_serializer_direction,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_3,
	.txnumevt	= 1,
	.rxnumevt	= 1,
};

/* Setup McASP */
static void mcasp_init(int board_type, u8 profile)
{
	switch (board_type){
	    case IPC335X_EVM:
	        /* Configure McASP */
	        setup_pin_mux(mcasp0_ipc335x_evm_pin_mux);
	        am335x_register_mcasp(&ipc335x_evm_snd_data, 0);
	        break;
	    case HMI335X:
	        /* Configure McASP */
	        setup_pin_mux(mcasp1_hmi335x_pin_mux);
	        am335x_register_mcasp(&hmi335x_snd_data, 1);
	        break;
	}
	return;
}

/* Module pin mux for LCDC */
static struct pinmux_config lcdc_pin_mux[] = {
	{"lcd_data0.lcd_data0",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data1.lcd_data1",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data2.lcd_data2",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data3.lcd_data3",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data4.lcd_data4",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data5.lcd_data5",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data6.lcd_data6",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data7.lcd_data7",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data8.lcd_data8",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data9.lcd_data9",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data10.lcd_data10",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data11.lcd_data11",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data12.lcd_data12",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data13.lcd_data13",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data14.lcd_data14",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data15.lcd_data15",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"gpmc_ad8.lcd_data16",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad9.lcd_data17",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad10.lcd_data18",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad11.lcd_data19",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad12.lcd_data20",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad13.lcd_data21",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad14.lcd_data22",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad15.lcd_data23",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"lcd_vsync.lcd_vsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_hsync.lcd_hsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_pclk.lcd_pclk",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_ac_bias_en.lcd_ac_bias_en", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static const struct display_panel disp_panel = {
	WVGA,
	32,
	32,
	COLOR_ACTIVE,
};

static struct lcd_ctrl_config lcd_cfg = {
	&disp_panel,
	.ac_bias		= 255,
	.ac_bias_intrpt		= 0,
	.dma_burst_sz		= 16,
	.bpp			= 32,
	.fdd			= 0x80,
	.tft_alt_mode		= 0,
	.stn_565_mode		= 0,
	.mono_8bit_mode		= 0,
	.invert_line_clock	= 1,
	.invert_frm_clock	= 1,
	.sync_edge		= 0,
	.sync_ctrl		= 1,
	.raster_order		= 0,
};

static int lcd_enable = -1;

static struct pinmux_config lcd_enable_mux[] = {
	{"gpmc_csn3.gpio2_0", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void lcd_power_ctrl(int enable)
{
	if(gpio_is_valid(lcd_enable)){
		if(enable)
			gpio_set_value(lcd_enable, 1);
		else
			gpio_set_value(lcd_enable, 0);
	}
}

struct da8xx_lcdc_platform_data ilx_at070tn83_v1_pdata = {
	.manu_name		= "Inl_at070",
	.controller_data	= &lcd_cfg,
	.type			= "ilx_at070tn83_v1",
	.panel_power_ctrl	= lcd_power_ctrl,
};

struct da8xx_lcdc_platform_data tm070rdh12_v2_pdata = {
	.manu_name		= "Tianma",
	.controller_data	= &lcd_cfg,
	.type			= "tm070rdh12_v2",
	.panel_power_ctrl	= lcd_power_ctrl,
};

static int __init conf_disp_pll(int rate)
{
	struct clk *disp_pll;
	int ret = -EINVAL;

	disp_pll = clk_get(NULL, "dpll_disp_ck");
	if (IS_ERR(disp_pll)) {
		pr_err("Cannot clk_get disp_pll\n");
		goto out;
	}

	ret = clk_set_rate(disp_pll, rate);
	clk_put(disp_pll);
out:
	return ret;
}

static void lcdc_init(int board_type, u8 profile)
{

	setup_pin_mux(lcdc_pin_mux);
	setup_pin_mux(lcd_enable_mux);
	lcd_enable = GPIO_TO_PIN(2, 0);

	gpio_request(lcd_enable, "lcd_enable");

	if (conf_disp_pll(300000000)) {
		pr_info("Failed configure display PLL, not attempting to"
				"register LCDC\n");
		return;
	}

	switch (board_type){
	case IPC335X_EVM:
		if (am33xx_register_lcdc(&ilx_at070tn83_v1_pdata))
			pr_info("Failed to register at070tn83 device\n");
		break;

	case HMI335X:
		if (am33xx_register_lcdc(&tm070rdh12_v2_pdata))
			pr_info("Failed to register tm070rdh12 device\n");
		break;
	}
	return;
}

/* TSc controller */
static struct tsc_data ipc335x_touchscreen_data  = {
	.wires  = 4,
	.x_plate_resistance = 200,
	.steps_to_configure = 5,
};

static struct mfd_tscadc_board ipc335x_tscadc_data = {
	.tsc_init = &ipc335x_touchscreen_data,
};

static struct pinmux_config tsc_pin_mux[] = {
	{"ain0.ain0",           OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
	{"ain1.ain1",           OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
	{"ain2.ain2",           OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
	{"ain3.ain3",           OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
	{"vrefp.vrefp",         OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
	{"vrefn.vrefn",         OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
	{NULL, 0},
};

static void mfd_tscadc_init(int board_type, u8 profile)
{
	int err;

	setup_pin_mux(tsc_pin_mux);

	err = am33xx_register_mfd_tscadc(&ipc335x_tscadc_data);
	if (err)
		pr_err("failed to register touchscreen device\n");
}

/* Module pin mux for eCAP2 */
static struct pinmux_config ecap2_pin_mux[] = {
	{"mcasp0_ahclkr.ecap2_in_pwm2_out",
		OMAP_MUX_MODE4 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static int backlight_enable;

static void enable_ecap2(int board_type, u8 profile)
{
	backlight_enable = true;
	setup_pin_mux(ecap2_pin_mux);
}

/* LCD backlight platform Data */
#define AM335X_BACKLIGHT_MAX_BRIGHTNESS        100
#define AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS    100
#define AM335X_PWM_PERIOD_NANO_SECONDS        (5000 * 10)

#define PWM_DEVICE_ID   "ecap.2"

static struct platform_pwm_backlight_data ipc335x_backlight_data = {
	.pwm_id         = PWM_DEVICE_ID,
	.ch             = -1,
	.lth_brightness	= 21,
	.max_brightness = AM335X_BACKLIGHT_MAX_BRIGHTNESS,
	.dft_brightness = AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS,
	.pwm_period_ns  = AM335X_PWM_PERIOD_NANO_SECONDS,
};
/* Setup pwm-backlight */
static struct platform_device ipc335x_backlight = {
	.name           = "pwm-backlight",
	.id             = -1,
	.dev            = {
		.platform_data  = &ipc335x_backlight_data,
	}
};

static struct pwmss_platform_data  pwm_pdata[3] = {
	{
		.version = PWM_VERSION_1,
	},
	{
		.version = PWM_VERSION_1,
	},
	{
		.version = PWM_VERSION_1,
	},
};

static int __init ecap2_init(void)
{
	int status = 0;

	if (backlight_enable) {
		am33xx_register_ecap(2, &pwm_pdata[2]);
		platform_device_register(&ipc335x_backlight);
	}
	return status;
}
late_initcall(ecap2_init);

static struct pinmux_config ehrpwm1b_pin_mux[] = {
	{"gpmc_a3.ehrpwm1B", OMAP_MUX_MODE6 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};
/* setup pwm beeper */
#define PWMBEEPER_MAX_FREQ 20000
static void pwm_beeper_init(int board_type, u8 profile)
{
	setup_pin_mux(ehrpwm1b_pin_mux);
	pwm_pdata[2].chan_attrib[1].max_freq = PWMBEEPER_MAX_FREQ;
	am33xx_register_ehrpwm(1, &pwm_pdata[2]);
}

static struct pinmux_config uart3_pin_mux[] = {
	{"spi0_cs1.uart3_rxd", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"ecap0_in_pwm0_out.uart3_txd", OMAP_MUX_MODE1 | AM33XX_PULL_ENBL},
	{NULL, 0},
};

/* setup uart3 */
static void uart3_init(int board_type, u8 profile)
{
	setup_pin_mux(uart3_pin_mux);
	return;
}

static struct pinmux_config uart4_pin_mux[] = {
	{"gpmc_wait0.uart4_rxd", OMAP_MUX_MODE6 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wpn.uart4_txd", OMAP_MUX_MODE6 | AM33XX_PULL_ENBL},
	{NULL, 0},
};

/* setup uart4 */
static void uart4_init(int board_type, u8 profile)
{
	setup_pin_mux(uart4_pin_mux);
	return;
}

static struct pinmux_config uart5_pin_mux[] = {
	{"mii1_col.uart5_rxd", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
	{"rmii1_refclk.uart5_txd", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL},
	{NULL, 0},
};

/* setup uart5 */
static void uart5_init(int board_type, u8 profile)
{
	setup_pin_mux(uart5_pin_mux);
	return;
}

static struct pinmux_config ipc335x_led_pin_mux[] = {
	{"mcasp0_ahclkx.gpio3_21", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

static struct gpio_led ipc335x_gpio_leds[] = {
	{
		.name			= "usr0",
		.default_trigger	= "heartbeat",
		.gpio			= GPIO_TO_PIN(3, 21),
		.active_low		= 1,
	},
};

static struct gpio_led_platform_data ipc335x_gpio_led_info = {
	.leds		= ipc335x_gpio_leds,
	.num_leds	= ARRAY_SIZE(ipc335x_gpio_leds),
};

static struct platform_device ipc335x_core_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &ipc335x_gpio_led_info,
	},
};

static struct pinmux_config hmi335x_led_pin_mux[] = {
	{"gpmc_a6.gpio1_22", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"gpmc_a7.gpio1_23", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"gpmc_a8.gpio1_24", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"gpmc_a9.gpio1_25", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

static struct gpio_led hmi335x_gpio_leds[] = {
	{
		.name			= "mmc0",
		.default_trigger	= "heartbeat",
		.gpio			= GPIO_TO_PIN(1, 22),
		.active_low		= 1,
	},
	{
		.name			= "mmc1",
		.default_trigger	= "heartbeat",
		.gpio			= GPIO_TO_PIN(1, 23),
		.active_low		= 1,
	},
	{
		.name			= "usr2",
		.default_trigger	= "heartbeat",
		.gpio			= GPIO_TO_PIN(1, 24),
		.active_low		= 1,
	},
	{
		.name			= "usr3",
		.default_trigger	= "heartbeat",
		.gpio			= GPIO_TO_PIN(1, 25),
		.active_low		= 1,
	},
};

static struct gpio_led_platform_data hmi335x_gpio_led_info = {
	.leds		= hmi335x_gpio_leds,
	.num_leds	= ARRAY_SIZE(hmi335x_gpio_leds),
};

static struct platform_device hmi335x_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &hmi335x_gpio_led_info,
	},
};

static void led_init(int board_type, u8 profile)
{
	switch (board_type){
	case IPC335X_CORE:
		setup_pin_mux(ipc335x_led_pin_mux);
		platform_device_register(&ipc335x_core_leds_gpio);
		break;
	case HMI335X:
		setup_pin_mux(hmi335x_led_pin_mux);
		platform_device_register(&hmi335x_leds_gpio);
		break;
	}
	return;
}

static struct pinmux_config ipc335x_gpio_keys_pin_mux[] = {
	{"mcasp0_aclkr.gpio3_18", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"mcasp0_fsr.gpio3_19", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* Configure GPIOs for GPIO Keys */
static struct gpio_keys_button ipc335x_evm_gpio_buttons[] = {
	{
		.code                   = KEY_HOME,
		.gpio                   = GPIO_TO_PIN(3, 18),
		.desc                   = "USER KEY1",
		.active_low		= 1,
	},
	{
		.code                   = KEY_ESC,
		.gpio                   = GPIO_TO_PIN(3, 19),
		.desc                   = "USER KEY2",
		.active_low		= 1,
	},
};

static struct gpio_keys_platform_data ipc335x_evm_gpio_key_info = {
	.buttons        = ipc335x_evm_gpio_buttons,
	.nbuttons       = ARRAY_SIZE(ipc335x_evm_gpio_buttons),
};

static struct platform_device ipc335x_evm_gpio_keys = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
		.platform_data  = &ipc335x_evm_gpio_key_info,
	},
};

static struct pinmux_config hmi335x_gpio_keys_pin_mux[] = {
	{"gpmc_a1.gpio1_17", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_a2.gpio1_18", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* Configure GPIOs for GPIO Keys */
static struct gpio_keys_button hmi335x_gpio_buttons[] = {
	{
		.code                   = KEY_HOME,
		.gpio                   = GPIO_TO_PIN(1, 17),
		.desc                   = "USER KEY1",
		.active_low		= 1,
	},
	{
		.code                   = KEY_ESC,
		.gpio                   = GPIO_TO_PIN(1, 18),
		.desc                   = "USER KEY2",
		.active_low		= 1,
	},
};

static struct gpio_keys_platform_data hmi335x_gpio_key_info = {
	.buttons        = hmi335x_gpio_buttons,
	.nbuttons       = ARRAY_SIZE(hmi335x_gpio_buttons),
};

static struct platform_device hmi335x_gpio_keys = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
		.platform_data  = &hmi335x_gpio_key_info,
	},
};

static void gpio_key_init(int board_type, u8 profile)
{
	int err = 0;

	switch (board_type){
	case IPC335X_EVM:
		setup_pin_mux(ipc335x_gpio_keys_pin_mux);
		err = platform_device_register(&ipc335x_evm_gpio_keys);
		break;
	case HMI335X:
		setup_pin_mux(hmi335x_gpio_keys_pin_mux);
		err = platform_device_register(&hmi335x_gpio_keys);
		break;
	}
	if (err)
		pr_err("failed to register gpio key device\n");
}

/* Module pin mux for RS485 */
static struct pinmux_config ipc335x_evm_rs485_pin_mux[] = {
	{"spi0_sclk.uart2_rxd", OMAP_MUX_MODE1 | AM33XX_SLEWCTRL_SLOW |
						AM33XX_PIN_INPUT_PULLUP},
	{"spi0_d0.uart2_txd", OMAP_MUX_MODE1 | AM33XX_PULL_UP |
						AM33XX_PULL_DISA |
						AM33XX_SLEWCTRL_SLOW},
	/* flow ctrl gpio */
	{"xdma_event_intr0.gpio0_19", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static struct pinmux_config hmi335x_rs485_pin_mux[] = {
	{"mii1_crs.uart2_rxd", OMAP_MUX_MODE6 | AM33XX_SLEWCTRL_SLOW |
						AM33XX_PIN_INPUT_PULLUP},
	{"mii1_rxerr.uart2_txd", OMAP_MUX_MODE6 | AM33XX_PULL_UP |
						AM33XX_PULL_DISA |
						AM33XX_SLEWCTRL_SLOW},
	/* flow ctrl gpio */
	{"xdma_event_intr0.gpio0_19", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* setup RS485 */
static void rs485_init(int board_type, u8 profile)
{
	int flow_ctrl_gpio = GPIO_TO_PIN(0, 19);
	switch (board_type){
	case IPC335X_EVM:
		setup_pin_mux(ipc335x_evm_rs485_pin_mux);
		break;
	case HMI335X:
		setup_pin_mux(hmi335x_rs485_pin_mux);
		break;
	}
	uart_omap_port_set_rts_gpio(2, flow_ctrl_gpio);
	return;
}

/* Pin mux for nand flash module */
static struct pinmux_config nand_pin_mux[] = {
	{"gpmc_ad0.gpmc_ad0",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad1.gpmc_ad1",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad2.gpmc_ad2",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad3.gpmc_ad3",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad4.gpmc_ad4",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad5.gpmc_ad5",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad6.gpmc_ad6",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad7.gpmc_ad7",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wait0.gpmc_wait0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wpn.gpmc_wpn",	  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn0.gpmc_csn0",	  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_advn_ale.gpmc_advn_ale",  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_oen_ren.gpmc_oen_ren",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_wen.gpmc_wen",     OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_ben0_cle.gpmc_ben0_cle",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{NULL, 0},
};

/* NAND partition information */
static struct mtd_partition ipc335x_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "SPL",
		.offset         = 0,			/* Offset = 0x0 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup1",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x20000 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup2",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x40000 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup3",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x60000 */
		.size           = SZ_128K,
	},
	{
		.name           = "U-Boot",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x80000 */
		.size           = 15 * SZ_128K,
	},
	{
		.name           = "U-Boot Env",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x260000 */
		.size           = 1 * SZ_128K,
	},
	{
		.name           = "Kernel",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x280000 */
		.size           = 40 * SZ_128K,
	},
	{
		.name           = "File System",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x780000 */
		.size           = MTDPART_SIZ_FULL,
	},
};

static struct gpmc_timings ipc335x_nand_timings = {
	.sync_clk = 0,

	.cs_on = 0,
	.cs_rd_off = 44,
	.cs_wr_off = 44,

	.adv_on = 6,
	.adv_rd_off = 34,
	.adv_wr_off = 44,
	.we_off = 40,
	.oe_off = 54,

	.access = 64,
	.rd_cycle = 82,
	.wr_cycle = 82,

	.wr_access = 40,
	.wr_data_mux_bus = 0,
};

static void nand_init(int board_type, u8 profile)
{
	struct omap_nand_platform_data *pdata;
	struct gpmc_devices_info gpmc_device[2] = {
		{ NULL, 0 },
		{ NULL, 0 },
	};

	setup_pin_mux(nand_pin_mux);
	pdata = omap_nand_init(ipc335x_nand_partitions,
		ARRAY_SIZE(ipc335x_nand_partitions), 0, 0,
		&ipc335x_nand_timings);
	if (!pdata)
		return;
	/*Use softecc by default*/
	pdata->ecc_opt = OMAP_ECC_HAMMING_CODE_DEFAULT;
	gpmc_device[0].pdata = pdata;
	gpmc_device[0].flag = GPMC_DEVICE_NAND;

	omap_init_gpmc(gpmc_device, sizeof(gpmc_device));
	omap_init_elm();
}

static struct pinmux_config gpio_ddr_vtt_pin_mux[] = {
	{"usb1_drvvbus.gpio3_13", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void gpio_ddr_vtt_init(int board_type, u8 profile)
{
	int gpio = GPIO_TO_PIN(3, 13);;

	setup_pin_mux(gpio_ddr_vtt_pin_mux);
	gpio_request(gpio, "ddr_vtt");
	gpio_direction_output(gpio, 1);
	return;
}

#define IPC335X_CORE_PHY_ID		0x4dd074
#define IPC335X_PHY_MASK		0xfffffffe
#define AR80XX_PHY_DEBUG_ADDR_REG	0x1d
#define AR80XX_PHY_DEBUG_DATA_REG	0x1e
#define AR80XX_DEBUG_RGMII_CLK_DLY_REG	0x5
#define AR80XX_RGMII_TX_CLK_DLY		BIT(8)

static int ipc335x_core_tx_clk_dly_phy_fixup(struct phy_device *phydev)
{
	phy_write(phydev, AR80XX_PHY_DEBUG_ADDR_REG,
		  AR80XX_DEBUG_RGMII_CLK_DLY_REG);
	phy_write(phydev, AR80XX_PHY_DEBUG_DATA_REG, AR80XX_RGMII_TX_CLK_DLY);

	return 0;
}

static struct ipc335x_dev_cfg ipc335x_core_dev_cfg[] = {
	{rgmii1_init, IPC335X_CORE, PROFILE_ALL},
	{usb0_init, IPC335X_CORE, PROFILE_ALL},
	{mmc0_init, IPC335X_CORE, PROFILE_0 | PROFILE_2},
	{i2c2_init, IPC335X_CORE, PROFILE_ALL},
	/*nand support will break mmc1 support*/
	{nand_init, IPC335X_CORE, PROFILE_1 | PROFILE_2},
	{led_init, IPC335X_CORE, PROFILE_ALL},
	{NULL, 0, 0},
};

static struct ipc335x_dev_cfg ipc335x_evm_dev_cfg[] = {
	{mmc1_init, IPC335X_EVM, PROFILE_0 | PROFILE_1},
	{uart3_init, IPC335X_EVM, PROFILE_ALL},
	{uart5_init, IPC335X_EVM, PROFILE_ALL},
	{rs485_init, IPC335X_EVM, PROFILE_ALL},
	{d_can_init, IPC335X_EVM, PROFILE_ALL},
	{mcasp_init, IPC335X_EVM, PROFILE_ALL},
	{gpio_key_init, IPC335X_EVM, PROFILE_ALL},
	{lcdc_init, IPC335X_EVM, PROFILE_0 | PROFILE_2},
	{mfd_tscadc_init, IPC335X_EVM, PROFILE_0 | PROFILE_2},
	{enable_ecap2, IPC335X_EVM, PROFILE_0 | PROFILE_2},
	{NULL, 0, 0},
};

static struct ipc335x_dev_cfg som335x_core_dev_cfg[] = {
	/*ddr vtt force init*/
	/*{gpio_ddr_vtt_init, SOM335X_CORE, PROFILE_ALL},*/
	{i2c2_init, SOM335X_CORE, PROFILE_ALL},
	{mmc0_init, SOM335X_CORE, PROFILE_0 | PROFILE_2},
	/*nand support will break mmc1 uart4 support*/
	{nand_init, SOM335X_CORE, PROFILE_1 | PROFILE_2},
	{NULL, 0, 0},
};

static struct ipc335x_dev_cfg hmi335x_dev_cfg[] = {
	{hmi335x_power_init, HMI335X, PROFILE_ALL},
	{mmc1_init, HMI335X, PROFILE_0 | PROFILE_1},
	{uart1_init, HMI335X, PROFILE_ALL},
	{uart4_init, HMI335X, PROFILE_0 | PROFILE_1},
	/*led support will break ctpts support*/
	{led_init, HMI335X, PROFILE_0 | PROFILE_2},
	{rgmii1_init, HMI335X, PROFILE_ALL},
	{rs485_init, HMI335X, PROFILE_ALL},
	{d_can_init, HMI335X, PROFILE_ALL},
	{mcasp_init, HMI335X, PROFILE_ALL},
	{gpio_key_init, HMI335X, PROFILE_ALL},
	{lcdc_init, HMI335X, PROFILE_0 | PROFILE_2},
	{mfd_tscadc_init, HMI335X, PROFILE_0 | PROFILE_2},
	{enable_ecap2, HMI335X, PROFILE_0 | PROFILE_2},
	{pwm_beeper_init, HMI335X, PROFILE_ALL},
	{NULL, 0, 0},
};

static void setup_ipc335x_core(int board_type, u8 profile_shift)
{
	switch(board_type){
	case IPC335X_CORE:
		pr_info("The board is IPC335X CORE in profile %d\n",
			profile_shift);
		_configure_device(IPC335X_CORE, ipc335x_core_dev_cfg,
			1 << profile_shift);
		break;
	case SOM335X_CORE:
		pr_info("The board is SOM335X CORE in profile %d\n",
			profile_shift);
		_configure_device(SOM335X_CORE, som335x_core_dev_cfg,
			1 << profile_shift);
		break;
	}
	/* Atheros Tx Clk delay Phy fixup */
	phy_register_fixup_for_uid(IPC335X_CORE_PHY_ID, IPC335X_PHY_MASK,
		ipc335x_core_tx_clk_dly_phy_fixup);
	am33xx_cpsw_init(AM33XX_CPSW_MODE_RGMII, NULL, NULL);
}

static void setup_ipc335x_dock(int board_type, u8 profile_shift)
{
	ipc335x_dock_set_id(board_type);

	switch(board_type){
	case IPC335X_EVM:
		pr_info("The board is IPC335X EVM in profile %d\n",
			profile_shift);
		_configure_device(IPC335X_EVM, ipc335x_evm_dev_cfg,
			1 << profile_shift);
		break;
	case HMI335X:
		pr_info("The board is HMI335X in profile %d\n",
			profile_shift);
		_configure_device(HMI335X, hmi335x_dev_cfg,
			1 << profile_shift);
		break;
	}
}

static void ipc335x_core_setup(struct memory_accessor *mem_acc, void *context)
{
	int ret;
	struct ipc335x_eeprom_config config;
	char tmp[10];

	/* get board specific data */
	ret = mem_acc->read(mem_acc, (char *)&config, EEPROM_BOARD_CFG_OFFSET,
		sizeof(config));
	if (ret != sizeof(config)) {
		pr_err("IPC335X config read fail, read %d bytes\n", ret);
		pr_err("This likely means that there either is no/or a failed EEPROM\n");
		goto out;
	}

	if (config.header != AM335X_EEPROM_HEADER) {
		pr_err("IPC335X: wrong header 0x%x, expected 0x%x\n",
			config.header, AM335X_EEPROM_HEADER);
		goto out;
	}

	snprintf(tmp, sizeof(config.name) + 1, "%s", config.name);
	pr_info("Board name: %s\n", tmp);
	snprintf(tmp, sizeof(config.version) + 1, "%s", config.version);
	pr_info("Board version: %s\n", tmp);

	if (!strnicmp("IPC335X", config.name, 7)) {
		setup_ipc335x_core(IPC335X_CORE, config.profile);
		return;
	}

	if (!strnicmp("SOM335X", config.name, 7)) {
		setup_ipc335x_core(SOM335X_CORE, config.profile);
	}
	return;

out:
	/*
	 * If the EEPROM hasn't been programed or an incorrect header
	 * or board name are read then the hardware details are unknown.
	 * Notify the user and call machine_halt to stop the boot process.
	 */
	pr_err("The error message above indicates that there is an issue with\n"
		   "the EEPROM or the EEPROM contents.  After verifying the EEPROM\n"
		   "contents, if any, refer to the %s function in the\n"
		   "%s file to modify the board\n"
		   "initialization code to match the hardware configuration\n"
		   "assume SOM335X core on Profile0",
		   __func__ , __FILE__);
	setup_ipc335x_core(SOM335X_CORE, 0);
}

static void ipc335x_dock_setup(struct memory_accessor *mem_acc, void *context)
{
	int ret;
	struct ipc335x_eeprom_config config;
	char tmp[10];

	/* get board specific data */
	ret = mem_acc->read(mem_acc, (char *)&config, EEPROM_BOARD_CFG_OFFSET,
		sizeof(config));
	if (ret != sizeof(config)) {
		pr_err("IPC335X config read fail, read %d bytes\n", ret);
		pr_err("This likely means that there either is no/or a failed EEPROM\n");
		if(ret == -ETIMEDOUT)
			goto out;
		else
			goto default_cfg;
	}

	if (config.header != AM335X_EEPROM_HEADER) {
		pr_err("IPC335X: wrong header 0x%x, expected 0x%x\n",
			config.header, AM335X_EEPROM_HEADER);
		goto default_cfg;
	}

	snprintf(tmp, sizeof(config.name) + 1, "%s", config.name);
	pr_info("Board name: %s\n", tmp);
	snprintf(tmp, sizeof(config.version) + 1, "%s", config.version);
	pr_info("Board version: %s\n", tmp);

	if (!strnicmp("EVM335X", config.name, 7)) {
		setup_ipc335x_dock(IPC335X_EVM, config.profile);
		return;
	}

	if (!strnicmp("HMI335X", config.name, 7)) {
		setup_ipc335x_dock(HMI335X, config.profile);
	}
	return;
default_cfg:
	/*
	 * If the EEPROM hasn't been programed or an incorrect header
	 * or board name are read then the hardware details are unknown.
	 * Notify the user and call machine_halt to stop the boot process.
	 */
	pr_err("The error message above indicates that there is an issue with\n"
		   "the EEPROM or the EEPROM contents.  After verifying the EEPROM\n"
		   "contents, if any, refer to the %s function in the\n"
		   "%s file to modify the board\n"
		   "initialization code to match the hardware configuration\n"
		   "assume HMI335X on Profile0",
		   __func__ , __FILE__);
	setup_ipc335x_dock(HMI335X, 0);
	return;
out:
	/*
	 * EEPROM try read timeout, dock not exist.
	 */
	pr_info("No Dock exist\n");
	return;
}

#define CORE_EEPROM_I2C_ADDR	0x50
#define EVM_EEPROM_I2C_ADDR	0x51

static struct at24_platform_data ipc335x_core_eeprom_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
	.setup          = ipc335x_core_setup,
	.context        = (void *)NULL,
};

static struct at24_platform_data ipc335x_evm_eeprom_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
	.setup          = ipc335x_dock_setup,
	.context        = (void *)NULL,
};

static struct regulator_init_data am335x_dummy = {
	.constraints.always_on	= true,
};

static struct regulator_consumer_supply am335x_vdd1_supply[] = {
	REGULATOR_SUPPLY("vdd_mpu", NULL),
};

static struct regulator_init_data am335x_vdd1 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd1_supply),
	.consumer_supplies	= am335x_vdd1_supply,
};

static struct regulator_consumer_supply am335x_vdd2_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_init_data am335x_vdd2 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd2_supply),
	.consumer_supplies	= am335x_vdd2_supply,
};

#define PMIC_IRQ_GPIO		GPIO_TO_PIN(1, 8)
static struct tps65910_board am335x_tps65910_info = {
	.irq_base	= TWL4030_IRQ_BASE,
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq 		= OMAP_GPIO_IRQ(PMIC_IRQ_GPIO),
	.tps65910_pmic_init_data[TPS65910_REG_VRTC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VIO]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDD1]	= &am335x_vdd1,
	.tps65910_pmic_init_data[TPS65910_REG_VDD2]	= &am335x_vdd2,
	.tps65910_pmic_init_data[TPS65910_REG_VDD3]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG1]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG2]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VPLL]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDAC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX1]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX2]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX33]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VMMC]	= &am335x_dummy,
};

/* Module pin mux for tps65910 irq */
static struct pinmux_config pmic_irq_pin_mux[] = {
	{"uart0_ctsn.gpio1_8", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct i2c_board_info __initdata ipc335x_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65910", TPS65910_I2C_ID1),
		.platform_data  = &am335x_tps65910_info,
	},
};

static struct i2c_board_info am335x_i2c_boardinfo1[] = {
	{
		/* Other board EEPROM init dock first*/
		I2C_BOARD_INFO("24c256", EVM_EEPROM_I2C_ADDR),
		.platform_data  = &ipc335x_evm_eeprom_info,
	},
	{
		/* Core board EEPROM */
		I2C_BOARD_INFO("24c256", CORE_EEPROM_I2C_ADDR),
		.platform_data  = &ipc335x_core_eeprom_info,
	},
};

static void __init ipc335x_i2c_init(void)
{
	setup_pin_mux(pmic_irq_pin_mux);
	omap_register_i2c_bus(1, 100, ipc335x_i2c_boardinfo,
				ARRAY_SIZE(ipc335x_i2c_boardinfo));
	setup_pin_mux(i2c1_pin_mux);
	omap_register_i2c_bus(2, 100, am335x_i2c_boardinfo1,
			ARRAY_SIZE(am335x_i2c_boardinfo1));
}

void __iomem *am33xx_emif_base;

void __iomem * __init am33xx_get_mem_ctlr(void)
{

	am33xx_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

	if (!am33xx_emif_base)
		pr_warning("%s: Unable to map DDR2 controller",	__func__);

	return am33xx_emif_base;
}

void __iomem *am33xx_get_ram_base(void)
{
	return am33xx_emif_base;
}

void __iomem *am33xx_gpio0_base;

void __iomem *am33xx_get_gpio0_base(void)
{
	am33xx_gpio0_base = ioremap(AM33XX_GPIO0_BASE, SZ_4K);

	return am33xx_gpio0_base;
}

static struct resource am33xx_cpuidle_resources[] = {
	{
		.start		= AM33XX_EMIF0_BASE,
		.end		= AM33XX_EMIF0_BASE + SZ_32K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

/* AM33XX devices support DDR2 power down */
static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
	.ddr2_pdown	= 1,
};

static struct platform_device am33xx_cpuidle_device = {
	.name			= "cpuidle-am33xx",
	.num_resources		= ARRAY_SIZE(am33xx_cpuidle_resources),
	.resource		= am33xx_cpuidle_resources,
	.dev = {
		.platform_data	= &am33xx_cpuidle_pdata,
	},
};

static void __init am33xx_cpuidle_init(void)
{
	int ret;

	am33xx_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

	ret = platform_device_register(&am33xx_cpuidle_device);

	if (ret)
		pr_warning("AM33XX cpuidle registration failed\n");

}

static void __init ipc335x_init(void)
{
	am33xx_cpuidle_init();
	am33xx_mux_init(board_mux);
	/*Force ddr vtt enable*/
	gpio_ddr_vtt_init(0, 0);
	omap_serial_init();
	ipc335x_i2c_init();
	omap_sdrc_init(NULL, NULL);

	usb_musb_init(&musb_board_data);
	omap_board_config = ipc335x_config;
	omap_board_config_size = ARRAY_SIZE(ipc335x_config);
	/* Create an alias for icss clock */
	if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL))
		pr_warn("failed to create an alias: icss_uart_gclk --> pruss\n");
	/* Create an alias for gfx/sgx clock */
	if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL))
		pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");
}

static void __init ipc335x_map_io(void)
{
	omap2_set_globals_am33xx();
	omapam33xx_map_common_io();
}

MACHINE_START(IPC335X, "ipc335x")
	/* Maintainer: Jason Lam -lzg@ema-tech.com */
	.atag_offset	= 0x100,
	.map_io		= ipc335x_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq     = omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= ipc335x_init,
MACHINE_END
