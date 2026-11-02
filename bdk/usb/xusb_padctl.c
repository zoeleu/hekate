// SPDX-License-Identifier: GPL-2.0-only
/*
 * Tegra210 USB3 UPHY support.
 * Derived from Linux drivers/phy/tegra/xusb-tegra210.c
 * Copyright (c) 2014-2020 NVIDIA CORPORATION. All rights reserved.
 * Copyright (C) 2015 Google, Inc.
 * Copyright (C) 2022-2023 CTCaer (Switch L4T adaptations).
 * PLLE reference portions: Copyright (c) 2012-2020 NVIDIA CORPORATION.
 */
#include <usb/xusb_padctl.h>
#include <usb/usb_t210.h>
#include <power/max7762x.h>
#include <power/max77620.h>
#include <soc/clock.h>
#include <soc/fuse.h>
#include <soc/i2c.h>
#include <soc/t210.h>
#include <soc/timer.h>

#define XUSB_PADCTL_UPHY_PLL_P0_CTL1 0x360
#define XUSB_PADCTL_UPHY_PLL_CTL1_PWR_OVRD (1 << 4)

#define XUSB_PADCTL_UPHY_PLL_P0_CTL2 0x364
#define XUSB_PADCTL_UPHY_PLL_CTL2_CAL_OVRD (1 << 2)

#define XUSB_PADCTL_UPHY_PLL_P0_CTL4 0x36c

#define XUSB_PADCTL_UPHY_PLL_P0_CTL5 0x370

#define XUSB_PADCTL_UPHY_PLL_P0_CTL8 0x37c
#define XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_OVRD (1 << 15)

#define XUSB_PADCTL_USB3_PAD_MUX 0x28
#define UPHY_PLL_CFG0 0x51c
#define PLLE_SS_CTRL 0x68
#define PLLE_ENABLE BIT(31)
#define PLLE_LOCK BIT(11)
#define SEQ_ENABLE BIT(24)
#define CFG_WRITE (BIT(27) | BIT(24))

static bool owned;
static u32 saved_clock[5];
static const u32 clock_regs[] = {CLK_RST_CONTROLLER_PLLE_BASE, CLK_RST_CONTROLLER_PLLE_MISC,
								 CLK_RST_CONTROLLER_PLLE_AUX, PLLE_SS_CTRL, UPHY_PLL_CFG0};
static const u32 pad_regs[] = {XUSB_PADCTL_USB3_PAD_MUX,
							   XUSB_PADCTL_SS_PORT_MAP,
							   XUSB_PADCTL_ELPG_PROGRAM_1,
							   0xa60,
							   0xa64,
							   0xa68,
							   0xa6c,
							   0xa74,
							   0x84,
							   0xc4,
							   0x104,
							   0x144};
static u32 saved_pad[sizeof(pad_regs) / sizeof(pad_regs[0])];
static const u8 rail_regs[] = {MAX77620_REG_LDO1_CFG, MAX77620_REG_SD3, MAX77620_REG_SD3_CFG, MAX77620_REG_LDO7_CFG};
static u8 saved_rails[4];
static bool mariko;

static bool _wait(u32 reg, u32 mask, u32 expected)
{
	for (u32 i = 0; i < 10000; i++)
	{
		if ((XUSB_PADCTL(reg) & mask) == expected)
			return true;
		usleep(10);
	}
	return false;
}

void xusb_ss_phy_disable(void)
{
	if (!owned)
		return;
	// Isolate SS port before removing its reference clock.
	XUSB_PADCTL(XUSB_PADCTL_ELPG_PROGRAM_1) |= BIT(1);
	usleep(100);
	XUSB_PADCTL(XUSB_PADCTL_ELPG_PROGRAM_1) |= BIT(0);
	usleep(250);
	XUSB_PADCTL(XUSB_PADCTL_ELPG_PROGRAM_1) |= BIT(2);
	CLOCK(CLK_RST_CONTROLLER_RST_DEV_Y_SET) = BIT(CLK_Y_PEX_USB_UPHY);
	CLOCK(CLK_RST_CONTROLLER_PLLE_BASE) &= ~PLLE_ENABLE;
	for (u32 i = 0; i < sizeof(pad_regs) / sizeof(pad_regs[0]); i++)
		XUSB_PADCTL(pad_regs[i]) = saved_pad[i];
	// The incoming PLL was disabled and UPHY was in reset. Never start a
	// hardware sequencer: Linux treats that as an already initialized PLL.
	for (u32 i = 0; i < 5; i++)
		CLOCK(clock_regs[i]) = saved_clock[i];
	for (u32 i = 0; i < (mariko ? 3 : 4); i++)
		i2c_send_byte(I2C_5, MAX77620_I2C_ADDR, rail_regs[i], saved_rails[i]);
	owned = false;
}

bool xusb_ss_phy_enable(void)
{
	if (owned)
		return true;
	if (fuse_read_hw_type() == FUSE_NX_HW_TYPE_HOAG)
		return false;
	mariko = ((APB_MISC(APB_MISC_GP_HIDREV) >> 4) & 0xf) == GP_HIDREV_MAJOR_T210B01;
	// Do not disturb a PLL owned by an earlier payload or PCIe consumer.
	if ((CLOCK(CLK_RST_CONTROLLER_PLLE_BASE) & PLLE_ENABLE) || (CLOCK(CLK_RST_CONTROLLER_PLLE_AUX) & SEQ_ENABLE) ||
		(CLOCK(UPHY_PLL_CFG0) & SEQ_ENABLE) || !(CLOCK(CLK_RST_CONTROLLER_RST_DEVICES_Y) & BIT(CLK_Y_PEX_USB_UPHY)))
		return false;
	for (u32 i = 0; i < 5; i++)
		saved_clock[i] = CLOCK(clock_regs[i]);
	for (u32 i = 0; i < sizeof(pad_regs) / sizeof(pad_regs[0]); i++)
		saved_pad[i] = XUSB_PADCTL(pad_regs[i]);
	for (u32 i = 0; i < (mariko ? 3 : 4); i++)
		if (i2c_recv_buf_small(&saved_rails[i], 1, I2C_5, MAX77620_I2C_ADDR, rail_regs[i]))
			return false;
	owned = true;
	max7762x_regulator_set_voltage(REGULATOR_LDO1, 1050000);
	max7762x_regulator_enable(REGULATOR_LDO1, true);
	max7762x_regulator_set_voltage(REGULATOR_SD3, 1800000);
	max7762x_regulator_enable(REGULATOR_SD3, true);
	if (!mariko)
	{
		max7762x_regulator_set_voltage(REGULATOR_LDO7, 1050000);
		max7762x_regulator_enable(REGULATOR_LDO7, true);
	}
	usleep(1000);
	if (!max77620_regulator_get_status(REGULATOR_LDO1) || !max77620_regulator_get_status(REGULATOR_SD3) ||
		(!mariko && !max77620_regulator_get_status(REGULATOR_LDO7)))
		goto fail;

	// PLLE: 38.4 MHz reference, M=2, N=125, CML divider=14.
	// Keep both PLLE and P0 under software control throughout UMS.
	CLOCK(CLK_RST_CONTROLLER_PLLE_AUX) = (saved_clock[2] & ~(BIT(2) | BIT(28) | SEQ_ENABLE)) | BIT(4) | BIT(6);
	CLOCK(CLK_RST_CONTROLLER_PLLE_BASE) &= ~BIT(30);
	CLOCK(CLK_RST_CONTROLLER_PLLE_MISC) =
		(saved_clock[1] & ~(BIT(13) | (3 << 4) | (2 << 2))) | BIT(9) | BIT(14) | BIT(8);
	usleep(5);
	CLOCK(PLLE_SS_CTRL) |= BIT(10) | BIT(11) | BIT(12);
	CLOCK(CLK_RST_CONTROLLER_PLLE_BASE) =
		(CLOCK(CLK_RST_CONTROLLER_PLLE_BASE) & ~0x1f00ffff) | (14 << 24) | (125 << 8) | 2;
	usleep(1);
	CLOCK(CLK_RST_CONTROLLER_PLLE_BASE) |= PLLE_ENABLE;
	bool locked = false;
	for (u32 i = 0; i < 1000; i++)
	{
		if (CLOCK(CLK_RST_CONTROLLER_PLLE_MISC) & PLLE_LOCK)
		{
			locked = true;
			break;
		}
		usleep(1);
	}
	if (!locked)
		goto fail;
	CLOCK(PLLE_SS_CTRL) = (CLOCK(PLLE_SS_CTRL) & ~(BIT(14) | BIT(15) | 0x3fff01ff)) | (0x23 << 24) | (1 << 16) | 0x21;
	CLOCK(PLLE_SS_CTRL) &= ~(BIT(10) | BIT(12));
	usleep(1);
	CLOCK(PLLE_SS_CTRL) &= ~BIT(11);
	usleep(1);
	CLOCK(UPHY_PLL_CFG0) |= BIT(0) | BIT(2);
	CLOCK(CLK_RST_CONTROLLER_RST_DEV_Y_CLR) = BIT(CLK_Y_PEX_USB_UPHY);
	usleep(10);
	u32 lane = mariko ? 5 : 6;
	u32 shift = 12 + lane * 2;
	// Function 1 is USB3-SS, function 0 is PCIe-x1.
	XUSB_PADCTL(XUSB_PADCTL_USB3_PAD_MUX) = (XUSB_PADCTL(XUSB_PADCTL_USB3_PAD_MUX) & ~(3 << shift)) | (1 << shift);
	if (mariko)
	{
		static const u32 pll_data[] = {0x020000, 0x037051, 0x250130, 0x1e0017};
		static const u32 lane_data[] = {0x010002, 0x040032, 0x070022, 0x352587, 0x490fc7, 0x520001,
										0x533c0f, 0x56c00f, 0x5dff07, 0x5e141a, 0x970080};
		for (u32 i = 0; i < sizeof(lane_data) / sizeof(lane_data[0]); i++)
			XUSB_PADCTL(0x47c + lane * 0x40) = CFG_WRITE | lane_data[i];
		for (u32 i = 0; i < sizeof(pll_data) / sizeof(pll_data[0]); i++)
			XUSB_PADCTL(0x384) = CFG_WRITE | pll_data[i];
	}
	else
	{
		XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL2) =
			(XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL2) & ~0x0ffffff0) | (0x136 << 4);
		XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL5) =
			(XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL5) & ~0x00ff0000) | (0x2a << 16);
	}
	XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL1) |= XUSB_PADCTL_UPHY_PLL_CTL1_PWR_OVRD;
	XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL2) |= XUSB_PADCTL_UPHY_PLL_CTL2_CAL_OVRD;
	XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL8) |= XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_OVRD;
	XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL4) =
		(XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL4) & ~0x30f0) | (2 << 12) | BIT(15);
	XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL1) =
		(XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL1) & ~0x0ff30007) | (0x19 << 20);
	usleep(10);
	XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL4) |= BIT(8);
	XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL2) |= BIT(0);
	if (!_wait(XUSB_PADCTL_UPHY_PLL_P0_CTL2, BIT(1), BIT(1)))
		goto fail;
	XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL2) &= ~BIT(0);
	if (!_wait(XUSB_PADCTL_UPHY_PLL_P0_CTL2, BIT(1), 0))
		goto fail;
	XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL1) |= BIT(3);
	if (!_wait(XUSB_PADCTL_UPHY_PLL_P0_CTL1, BIT(15), BIT(15)))
		goto fail;
	XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL8) |= BIT(12) | BIT(13);
	if (!_wait(XUSB_PADCTL_UPHY_PLL_P0_CTL8, BIT(31), BIT(31)))
		goto fail;
	XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL8) &= ~BIT(12);
	if (!_wait(XUSB_PADCTL_UPHY_PLL_P0_CTL8, BIT(31), 0))
		goto fail;
	XUSB_PADCTL(XUSB_PADCTL_UPHY_PLL_P0_CTL8) &= ~BIT(13);
	XUSB_PADCTL(XUSB_PADCTL_USB3_PAD_MUX) |= BIT(1 + lane);
	// USB-C is external: clear INTERNAL and pair SS0 with USB2-0.
	XUSB_PADCTL(XUSB_PADCTL_SS_PORT_MAP) &= ~0x1f;
	XUSB_PADCTL(0xa60) = (XUSB_PADCTL(0xa60) & ~(3 << 16)) | (2 << 16);
	XUSB_PADCTL(0xa64) = (XUSB_PADCTL(0xa64) & ~0xffff) | 0xfc;
	XUSB_PADCTL(0xa68) = 0xc0077f1f;
	XUSB_PADCTL(0xa6c) = (XUSB_PADCTL(0xa6c) & ~0xffff0000) | (0x1c7 << 16);
	XUSB_PADCTL(0xa74) = 0xfcf01368;
	if (mariko)
	{
		for (u32 i = 0; i < 4; i++)
			XUSB_PADCTL(0x84 + i * 0x40) |= BIT(6);
		// NVIDIA's T210B01 workaround for the unused SS3 port.
		XUSB_PADCTL(XUSB_PADCTL_SS_PORT_MAP) = (XUSB_PADCTL(XUSB_PADCTL_SS_PORT_MAP) & ~(7 << 15)) | (3 << 15);
		XUSB_PADCTL(XUSB_PADCTL_ELPG_PROGRAM_1) &= ~(7 << 9);
	}
	XUSB_PADCTL(XUSB_PADCTL_ELPG_PROGRAM_1) &= ~BIT(2);
	usleep(100);
	XUSB_PADCTL(XUSB_PADCTL_ELPG_PROGRAM_1) &= ~BIT(1);
	usleep(100);
	XUSB_PADCTL(XUSB_PADCTL_ELPG_PROGRAM_1) &= ~BIT(0);
	return true;
fail:
	xusb_ss_phy_disable();
	return false;
}
