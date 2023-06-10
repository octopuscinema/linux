// spdx-license-identifier: gpl-2.0
/*
 * Sony IMX585 CMOS Image Sensor Driver
 *
 * The IMX585 is the successor of IMX485, 3856x2180 1/1.2" STARVIS2 CMOS image sensors.
 *
 * Copyright (C) 2023 Soho Enterprise Ltd.
 * Author: Tetsuya Nomura <tetsuya.nomura@soho-enterprise.com>
 *
 * Based on IMX290 driver
 * Copyright (C) 2019 FRAMOS GmbH.
 * and
 * Copyright (C) 2019 Linaro Ltd.
 * Author: Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX585_STANDBY		0x3000
#define IMX585_REGHOLD		0x3001
#define IMX585_XMSTA		0x3002
#define IMX585_INCK_SEL		0x3014
	#define IMX585_INCK_SEL_74_25	0x00
	#define IMX585_INCK_SEL_37_125	0x01
	#define IMX585_INCK_SEL_72	0x02
	#define IMX585_INCK_SEL_27	0x03
	#define IMX585_INCK_SEL_24	0x04
#define IMX585_LANE_RATE	0x3015
	#define IMX585_LANE_RATE_2376	0x00
	#define IMX585_LANE_RATE_2079	0x01
	#define IMX585_LANE_RATE_1782	0x02
	#define IMX585_LANE_RATE_1440	0x03
	#define IMX585_LANE_RATE_1188	0x04
	#define IMX585_LANE_RATE_891	0x05
	#define IMX585_LANE_RATE_720	0x06
	#define IMX585_LANE_RATE_594	0x07
#define IMX585_FLIP_WINMODEH	0x3020
#define IMX585_FLIP_WINMODEV	0x3021
#define IMX585_ADBIT		0x3022
#define IMX585_MDBIT		0x3023
#define IMX585_VMAX		0x3028
	#define IMX585_VMAX_MAX		0x0fffff
#define IMX585_HMAX		0x302c
	#define IMX585_HMAX_MAX		0xffff
#define IMX585_FR_FDG_SEL0	0x3030
	#define IMX585_FDG_SEL0_LCG	0x00
	#define IMX585_FDG_SEL0_HCG	0x01
#define IMX585_FR_FDG_SEL1	0x3031
#define IMX585_FR_FDG_SEL2	0x3032
#define IMX585_CSI_LANE_MODE	0x3040
#define IMX585_EXPOSURE		0x3050
#define IMX585_GAIN		0x306C

#define IMX585_EXPOSURE_MIN	8
#define IMX585_EXPOSURE_STEP	2
/* Exposure must be this many lines less than VMAX */
#define IMX585_EXPOSURE_OFFSET  4

#define IMX585_NATIVE_WIDTH		3876U
#define IMX585_NATIVE_HEIGHT		2204U
#define IMX585_PIXEL_ARRAY_LEFT		0U
#define IMX585_PIXEL_ARRAY_TOP		20U
#define IMX585_PIXEL_ARRAY_WIDTH	3856U
#define IMX585_PIXEL_ARRAY_HEIGHT	2180U

static const char * const imx585_supply_name[] = {
	"vdda",
	"vddd",
	"vdddo",
};

#define IMX585_NUM_SUPPLIES ARRAY_SIZE(imx585_supply_name)

struct imx585_regval {
	u16 reg;
	u8 val;
};

struct imx585_mode {
	u32 width;
	u32 height;
	u32 hmax;
	u32 vmax;
	struct v4l2_rect crop;

	const struct imx585_regval *mode_data;
	u32 mode_data_size;
};

struct imx585 {
	struct device *dev;
	struct clk *xclk;
	u8 inck_sel;
	struct regmap *regmap;
	u8 nlanes;
	u8 bpp;

	const struct imx585_pixfmt *formats;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt current_format;
	const struct imx585_mode *current_mode;

	struct regulator_bulk_data supplies[IMX585_NUM_SUPPLIES];
	struct gpio_desc *rst_gpio;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *exposure;

	struct mutex lock;
};

struct imx585_pixfmt {
	u32 code;
	u8 bpp;
};

#define IMX585_NUM_FORMATS 2

static const struct imx585_pixfmt imx585_colour_formats[IMX585_NUM_FORMATS] = {
	{ MEDIA_BUS_FMT_SRGGB10_1X10, 10 },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, 12 },
};

static const struct imx585_pixfmt imx585_mono_formats[IMX585_NUM_FORMATS] = {
	{ MEDIA_BUS_FMT_Y10_1X10, 10 },
	{ MEDIA_BUS_FMT_Y12_1X12, 12 },
};

static const struct regmap_config imx585_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static const struct imx585_regval imx585_global_settings[] = {
//{0x3000,        0x01}, // STANDBY
//{0x3001,        0x00}, // REG HOLD
{0x3002,        0x00}, // 0: Master Mode
//{0x3014,        0x04}, // INCK SEL 0x04:24MHz
//{0x3015,        0x03}, // Data Rate 0x03:1440Mbps 0x04:1188Mbps
//{0x3018,        0x10}, // default
{0x301A,        0x00}, // default
{0x301B,        0x00}, // default
{0x301C,        0x00}, // default
{0x301E,        0x01}, // default
//{0x3020,        0x00}, // HREVERSE
//{0x3021,        0x00}, // VREVERSE
//{0x3022,        0x00}, // AD 10bit
//{0x3023,        0x00}, // Mode 10bit
{0x3024,        0x00}, // HDR COMBI_EN
// {0x3028,        0xCA}, // VMAX
// {0x3029,        0x08}, // VMAX
// {0x302A,        0x00}, // VMAX
// {0x302C,        0x28}, // HMAX
// {0x302D,        0x05}, // HMAX
//{0x3030,        0x00}, // FDG SEL 0
//{0x3031,        0x00}, // FDG SEL 1
//{0x3032,        0x00}, // FDG SEL 2
{0x303C,        0x00}, // HSTART
{0x303D,        0x00}, // HSTART
{0x303E,        0x10}, // HWIDTH
{0x303F,        0x0F}, // HWIDTH
{0x3040,        0x01}, // LANE MODE 0x01:2lane
{0x3042,        0x00}, // default
{0x3043,        0x00}, // default
{0x3044,        0x00}, // VSTART
{0x3045,        0x00}, // VSTART
{0x3046,        0x84}, // VWIDTH
{0x3047,        0x08}, // VWIDTH
// {0x3050,    	0x08}, // SHR0
// {0x3051,        0x00}, // SHR0
// {0x3052,        0x00}, // SHR0
{0x3054,        0x0E}, // SHR1
{0x3055,        0x00}, // SHR1
{0x3056,        0x00}, // SHR1
{0x3058,        0x8A}, // SHR2
{0x3059,        0x01}, // SHR2
{0x305A,        0x00}, // SHR2
{0x3060,        0x16}, // RHS1
{0x3061,        0x01}, // RHS1
{0x3062,        0x00}, // RHS1
{0x3064,        0xC4}, // RHS2
{0x3065,        0x0C}, // RHS2
{0x3066,        0x00}, // RHS2
{0x3069,        0x00}, // default
{0x306A,        0x00}, // default
// {0x306C,        0x00}, // GAIN0
// {0x306D,        0x00}, // GAIN0
{0x306E,        0x00}, // GAIN1
{0x306F,        0x00}, // GAIN1
{0x3070,        0x00}, // GAIN2
{0x3071,        0x00}, // GAIN2
{0x3074,        0x64}, // default
{0x3081,        0x00}, // default
{0x308C,        0x00}, // default
{0x308D,        0x01}, // default
{0x3094,        0x00}, // default
{0x3095,        0x00}, // default
{0x3096,        0x00}, // default
{0x3097,        0x00}, // default
{0x309C,        0x00}, // default
{0x309D,        0x00}, // default
{0x30A4,        0xAA}, // XVSOUT/XHSOUT
{0x30A6,        0x00}, // XVSDRIVE/XHSDRIVE
{0x30CC,        0x00}, // HVS LENGTH
{0x30CD,        0x00}, // XHS LENGTH
{0x30D5,        0x04}, // DIG_CLP_VSTART
{0x30DC,        0x32}, // Black Level
{0x30DD,        0x00}, // Black Level
{0x3400,        0x01}, // reserve
{0x3460,        0x21}, // reserve
{0x3478,        0xA1}, // reserve
{0x347C,        0x01}, // reserve
{0x3480,        0x01}, // reserve
{0x36D0,        0x00}, // reserve
{0x36D1,        0x10}, // reserve
{0x36D4,        0x00}, // reserve
{0x36D5,        0x10}, // reserve
{0x36E2,        0x00}, // reserve
{0x36E4,        0x00}, // reserve
{0x36E5,        0x00}, // reserve
{0x36E6,        0x00}, // reserve
{0x36E8,        0x00}, // reserve
{0x36E9,        0x00}, // reserve
{0x36EA,        0x00}, // reserve
{0x36EC,        0x00}, // reserve
{0x36EE,        0x00}, // reserve
{0x36EF,        0x00}, // reserve
{0x3930,        0x66}, // reserve
{0x3931,        0x01}, // reserve
{0x3A4C,        0x39}, // reserve
{0x3A4D,        0x01}, // reserve
{0x3A4E,        0x14}, // reserve
{0x3A50,        0x48}, // reserve
{0x3A51,        0x01}, // reserve
{0x3A52,        0x14}, // reserve
{0x3A56,        0x00}, // reserve
{0x3A5A,        0x00}, // reserve
{0x3A5E,        0x00}, // reserve
{0x3A62,        0x00}, // reserve
{0x3A6A,        0x20}, // reserve
{0x3A6C,        0x42}, // reserve
{0x3A6E,        0xA0}, // reserve
{0x3B2C,        0x0C}, // reserve
{0x3B30,        0x1C}, // reserve
{0x3B34,        0x0C}, // reserve
{0x3B38,        0x1C}, // reserve
{0x3BA0,        0x0C}, // reserve
{0x3BA4,        0x1C}, // reserve
{0x3BA8,        0x0C}, // reserve
{0x3BAC,        0x1C}, // reserve
{0x3D3C,        0x11}, // reserve
{0x3D46,        0x0B}, // reserve
{0x3DE0,        0x3F}, // reserve
{0x3DE1,        0x08}, // reserve
{0x3E10,        0x10}, // reserve
{0x3E14,        0x87}, // reserve
{0x3E16,        0x91}, // reserve
{0x3E18,        0x91}, // reserve
{0x3E1A,        0x87}, // reserve
{0x3E1C,        0x78}, // reserve
{0x3E1E,        0x50}, // reserve
{0x3E20,        0x50}, // reserve
{0x3E22,        0x50}, // reserve
{0x3E24,        0x87}, // reserve
{0x3E26,        0x91}, // reserve
{0x3E28,        0x91}, // reserve
{0x3E2A,        0x87}, // reserve
{0x3E2C,        0x78}, // reserve
{0x3E2E,        0x50}, // reserve
{0x3E30,        0x50}, // reserve
{0x3E32,        0x50}, // reserve
{0x3E34,        0x87}, // reserve
{0x3E36,        0x91}, // reserve
{0x3E38,        0x91}, // reserve
{0x3E3A,        0x87}, // reserve
{0x3E3C,        0x78}, // reserve
{0x3E3E,        0x50}, // reserve
{0x3E40,        0x50}, // reserve
{0x3E42,        0x50}, // reserve
{0x4054,        0x64}, // reserve
{0x4148,        0xFE}, // reserve
{0x4149,        0x05}, // reserve
{0x414A,        0xFF}, // reserve
{0x414B,        0x05}, // reserve
{0x420A,        0x03}, // reserve
{0x4231,        0x18}, // reserve
{0x423D,        0x9C}, // reserve
{0x4242,        0xB4}, // reserve
{0x4246,        0xB4}, // reserve
{0x424E,        0xB4}, // reserve
{0x425C,        0xB4}, // reserve
{0x425E,        0xB6}, // reserve
{0x426C,        0xB4}, // reserve
{0x426E,        0xB6}, // reserve
{0x428C,        0xB4}, // reserve
{0x428E,        0xB6}, // reserve
{0x4708,        0x00}, // reserve
{0x4709,        0x00}, // reserve
{0x470A,        0xFF}, // reserve
{0x470B,        0x03}, // reserve
{0x470C,        0x00}, // reserve
{0x470D,        0x00}, // reserve
{0x470E,        0xFF}, // reserve
{0x470F,        0x03}, // reserve
{0x47EB,        0x1C}, // reserve
{0x47F0,        0xA6}, // reserve
{0x47F2,        0xA6}, // reserve
{0x47F4,        0xA0}, // reserve
{0x47F6,        0x96}, // reserve
{0x4808,        0xA6}, // reserve
{0x480A,        0xA6}, // reserve
{0x480C,        0xA0}, // reserve
{0x480E,        0x96}, // reserve
{0x492C,        0xB2}, // reserve
{0x4930,        0x03}, // reserve
{0x4932,        0x03}, // reserve
{0x4936,        0x5B}, // reserve
{0x4938,        0x82}, // reserve
{0x493C,        0x23}, // reserve
{0x493E,        0x23}, // reserve
{0x4940,        0x23}, // reserve
{0x4BA8,        0x1C}, // reserve
{0x4BA9,        0x03}, // reserve
{0x4BAC,        0x1C}, // reserve
{0x4BAD,        0x1C}, // reserve
{0x4BAE,        0x1C}, // reserve
{0x4BAF,        0x1C}, // reserve
{0x4BB0,        0x1C}, // reserve
{0x4BB1,        0x1C}, // reserve
{0x4BB2,        0x1C}, // reserve
{0x4BB3,        0x1C}, // reserve
{0x4BB4,        0x1C}, // reserve
{0x4BB8,        0x03}, // reserve
{0x4BB9,        0x03}, // reserve
{0x4BBA,        0x03}, // reserve
{0x4BBB,        0x03}, // reserve
{0x4BBC,        0x03}, // reserve
{0x4BBD,        0x03}, // reserve
{0x4BBE,        0x03}, // reserve
{0x4BBF,        0x03}, // reserve
{0x4BC0,        0x03}, // reserve
{0x4C14,        0x87}, // reserve
{0x4C16,        0x91}, // reserve
{0x4C18,        0x91}, // reserve
{0x4C1A,        0x87}, // reserve
{0x4C1C,        0x78}, // reserve
{0x4C1E,        0x50}, // reserve
{0x4C20,        0x50}, // reserve
{0x4C22,        0x50}, // reserve
{0x4C24,        0x87}, // reserve
{0x4C26,        0x91}, // reserve
{0x4C28,        0x91}, // reserve
{0x4C2A,        0x87}, // reserve
{0x4C2C,        0x78}, // reserve
{0x4C2E,        0x50}, // reserve
{0x4C30,        0x50}, // reserve
{0x4C32,        0x50}, // reserve
{0x4C34,        0x87}, // reserve
{0x4C36,        0x91}, // reserve
{0x4C38,        0x91}, // reserve
{0x4C3A,        0x87}, // reserve
{0x4C3C,        0x78}, // reserve
{0x4C3E,        0x50}, // reserve
{0x4C40,        0x50}, // reserve
{0x4C42,        0x50}, // reserve
{0x4D12,        0x1F}, // reserve
{0x4D13,        0x1E}, // reserve
{0x4D26,        0x33}, // reserve
{0x4E0E,        0x59}, // reserve
{0x4E14,        0x55}, // reserve
{0x4E16,        0x59}, // reserve
{0x4E1E,        0x3B}, // reserve
{0x4E20,        0x47}, // reserve
{0x4E22,        0x54}, // reserve
{0x4E26,        0x81}, // reserve
{0x4E2C,        0x7D}, // reserve
{0x4E2E,        0x81}, // reserve
{0x4E36,        0x63}, // reserve
{0x4E38,        0x6F}, // reserve
{0x4E3A,        0x7C}, // reserve
{0x4F3A,        0x3C}, // reserve
{0x4F3C,        0x46}, // reserve
{0x4F3E,        0x59}, // reserve
{0x4F42,        0x64}, // reserve
{0x4F44,        0x6E}, // reserve
{0x4F46,        0x81}, // reserve
{0x4F4A,        0x82}, // reserve
{0x4F5A,        0x81}, // reserve
{0x4F62,        0xAA}, // reserve
{0x4F72,        0xA9}, // reserve
{0x4F78,        0x36}, // reserve
{0x4F7A,        0x41}, // reserve
{0x4F7C,        0x61}, // reserve
{0x4F7D,        0x01}, // reserve
{0x4F7E,        0x7C}, // reserve
{0x4F7F,        0x01}, // reserve
{0x4F80,        0x77}, // reserve
{0x4F82,        0x7B}, // reserve
{0x4F88,        0x37}, // reserve
{0x4F8A,        0x40}, // reserve
{0x4F8C,        0x62}, // reserve
{0x4F8D,        0x01}, // reserve
{0x4F8E,        0x76}, // reserve
{0x4F8F,        0x01}, // reserve
{0x4F90,        0x5E}, // reserve
{0x4F91,        0x02}, // reserve
{0x4F92,        0x69}, // reserve
{0x4F93,        0x02}, // reserve
{0x4F94,        0x89}, // reserve
{0x4F95,        0x02}, // reserve
{0x4F96,        0xA4}, // reserve
{0x4F97,        0x02}, // reserve
{0x4F98,        0x9F}, // reserve
{0x4F99,        0x02}, // reserve
{0x4F9A,        0xA3}, // reserve
{0x4F9B,        0x02}, // reserve
{0x4FA0,        0x5F}, // reserve
{0x4FA1,        0x02}, // reserve
{0x4FA2,        0x68}, // reserve
{0x4FA3,        0x02}, // reserve
{0x4FA4,        0x8A}, // reserve
{0x4FA5,        0x02}, // reserve
{0x4FA6,        0x9E}, // reserve
{0x4FA7,        0x02}, // reserve
{0x519E,        0x79}, // reserve
{0x51A6,        0xA1}, // reserve
{0x51F0,        0xAC}, // reserve
{0x51F2,        0xAA}, // reserve
{0x51F4,        0xA5}, // reserve
{0x51F6,        0xA0}, // reserve
{0x5200,        0x9B}, // reserve
{0x5202,        0x91}, // reserve
{0x5204,        0x87}, // reserve
{0x5206,        0x82}, // reserve
{0x5208,        0xAC}, // reserve
{0x520A,        0xAA}, // reserve
{0x520C,        0xA5}, // reserve
{0x520E,        0xA0}, // reserve
{0x5210,        0x9B}, // reserve
{0x5212,        0x91}, // reserve
{0x5214,        0x87}, // reserve
{0x5216,        0x82}, // reserve
{0x5218,        0xAC}, // reserve
{0x521A,        0xAA}, // reserve
{0x521C,        0xA5}, // reserve
{0x521E,        0xA0}, // reserve
{0x5220,        0x9B}, // reserve
{0x5222,        0x91}, // reserve
{0x5224,        0x87}, // reserve
{0x5226,        0x82}, // reserve
};

static const struct imx585_regval imx585_1080p_common_settings[] = {
	/* mode settings */
	{0x3018, 0x10}, // WINMODE all-pixel
	{ IMX585_FR_FDG_SEL1, 0x00 },
	{ IMX585_FR_FDG_SEL2, 0x00 },
};

/* supported link frequencies */
static const s64 imx585_link_freq_2lanes[] = {
	594000000,
};

static const s64 imx585_link_freq_4lanes[] = {
	297000000,
};

/*
 * In this function and in the similar ones below we rely on imx585_probe()
 * to ensure that nlanes is either 2 or 4.
 */
static inline const s64 *imx585_link_freqs_ptr(const struct imx585 *imx585)
{
	if (imx585->nlanes == 2)
		return imx585_link_freq_2lanes;
	else
		return imx585_link_freq_4lanes;
}

static inline int imx585_link_freqs_num(const struct imx585 *imx585)
{
	if (imx585->nlanes == 2)
		return ARRAY_SIZE(imx585_link_freq_2lanes);
	else
		return ARRAY_SIZE(imx585_link_freq_4lanes);
}

/* Mode configs */
static const struct imx585_mode imx585_modes[] = {
	{
		/*
		 * Note that this mode reads out the areas documented as
		 * "effective matrgin for color processing" and "effective pixel
		 * ignored area" in the datasheet.
		 */
		.width = 3856,
		.height = 2180,
		.hmax = (3944 * 2), // determined by experiment
		.vmax = 0x08ca,
		.crop = {
			.left = IMX585_PIXEL_ARRAY_LEFT,
			.top = IMX585_PIXEL_ARRAY_TOP,
			.width = IMX585_NATIVE_WIDTH,
			.height = IMX585_NATIVE_HEIGHT,
		},
		.mode_data = imx585_1080p_common_settings,
		.mode_data_size = ARRAY_SIZE(imx585_1080p_common_settings),
	},
};

#define IMX585_NUM_MODES ARRAY_SIZE(imx585_modes)

static inline struct imx585 *to_imx585(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx585, sd);
}

static inline int imx585_read_reg(struct imx585 *imx585, u16 addr, u8 *value)
{
	unsigned int regval;
	int ret;

	ret = regmap_read(imx585->regmap, addr, &regval);
	if (ret) {
		dev_err(imx585->dev, "I2C read failed for addr: %x\n", addr);
		return ret;
	}

	*value = regval & 0xff;

	return 0;
}

static int imx585_write_reg(struct imx585 *imx585, u16 addr, u8 value)
{
	int ret;

	ret = regmap_write(imx585->regmap, addr, value);
	if (ret) {
		dev_err(imx585->dev, "I2C write failed for addr: %x\n", addr);
		return ret;
	}

	return ret;
}

static int imx585_set_register_array(struct imx585 *imx585,
				     const struct imx585_regval *settings,
				     unsigned int num_settings)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings) {
		ret = imx585_write_reg(imx585, settings->reg, settings->val);
		if (ret < 0)
			return ret;
	}

	/* Provide 10ms settle time */
	usleep_range(10000, 11000);

	return 0;
}

static int imx585_write_buffered_reg(struct imx585 *imx585, u16 address_low,
				     u8 nr_regs, u32 value)
{
	unsigned int i;
	int ret;

	ret = imx585_write_reg(imx585, IMX585_REGHOLD, 0x01);
	if (ret) {
		dev_err(imx585->dev, "Error setting hold register\n");
		return ret;
	}

	for (i = 0; i < nr_regs; i++) {
		ret = imx585_write_reg(imx585, address_low + i,
				       (u8)(value >> (i * 8)));
		if (ret) {
			dev_err(imx585->dev, "Error writing buffered registers\n");
			return ret;
		}
	}

	ret = imx585_write_reg(imx585, IMX585_REGHOLD, 0x00);
	if (ret) {
		dev_err(imx585->dev, "Error setting hold register\n");
		return ret;
	}

	return ret;
}

static int imx585_set_gain(struct imx585 *imx585, u32 value)
{
	int ret;

	ret = imx585_write_buffered_reg(imx585, IMX585_GAIN, 2, value);
	if (ret) {
		dev_err(imx585->dev, "Unable to write gain\n");
		return ret;
	}

	ret = imx585_write_reg(imx585, IMX585_FR_FDG_SEL0, value < 0x22 ?
			       IMX585_FDG_SEL0_HCG : IMX585_FDG_SEL0_LCG);
			       //IMX585_FDG_SEL0_LCG : IMX585_FDG_SEL0_LCG);
	if (ret)
		dev_err(imx585->dev, "Unable to write LCG/HCG mode\n");

	return ret;
}

static int imx585_set_exposure(struct imx585 *imx585, u32 value)
{
	u32 exposure = (imx585->current_mode->height + imx585->vblank->val) -
						value - 1;
	int ret;

	ret = imx585_write_buffered_reg(imx585, IMX585_EXPOSURE, 3,
					exposure);
	if (ret)
		dev_err(imx585->dev, "Unable to write exposure\n");

	return ret;
}

static int imx585_set_hmax(struct imx585 *imx585, u32 val)
{
	u32 hmax = (val + imx585->current_mode->width) >> 1;
	int ret;

	ret = imx585_write_buffered_reg(imx585, IMX585_HMAX, 2,
					hmax);
					// 0x044c); // for debug
	if (ret)
		dev_err(imx585->dev, "Error setting HMAX register\n");

	return ret;
}

static int imx585_set_vmax(struct imx585 *imx585, u32 val)
{
	u32 vmax = val + imx585->current_mode->height;

	int ret;

	ret = imx585_write_buffered_reg(imx585, IMX585_VMAX, 3,
					vmax);
					// 0x0008ca); // for debug
	if (ret)
		dev_err(imx585->dev, "Unable to write vmax\n");

	/*
	 * Changing vblank changes the allowed range for exposure.
	 * We don't supply the current exposure as default here as it
	 * may lie outside the new range. We will reset it just below.
	 */
	__v4l2_ctrl_modify_range(imx585->exposure,
				 IMX585_EXPOSURE_MIN,
				 vmax - IMX585_EXPOSURE_OFFSET,
				 IMX585_EXPOSURE_STEP,
				 vmax - IMX585_EXPOSURE_OFFSET);

	/*
	 * Becuse of the way exposure works for this sensor, updating
	 * vblank causes the effective exposure to change, so we must
	 * set it back to the "new" correct value.
	 */
	imx585_set_exposure(imx585, imx585->exposure->val);

	return ret;
}

/* Stop streaming */
static int imx585_stop_streaming(struct imx585 *imx585)
{
	int ret;

	ret = imx585_write_reg(imx585, IMX585_STANDBY, 0x01);
	if (ret < 0)
		return ret;

	msleep(30);

	return imx585_write_reg(imx585, IMX585_XMSTA, 0x01);
}

static int imx585_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx585 *imx585 = container_of(ctrl->handler,
					     struct imx585, ctrls);
	int ret = 0;

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(imx585->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx585_set_gain(imx585, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx585_set_exposure(imx585, ctrl->val);
		break;
	case V4L2_CID_HBLANK:
		ret = imx585_set_hmax(imx585, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = imx585_set_vmax(imx585, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = imx585_write_reg(imx585, IMX585_FLIP_WINMODEH, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = imx585_write_reg(imx585, IMX585_FLIP_WINMODEV, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(imx585->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx585_ctrl_ops = {
	.s_ctrl = imx585_set_ctrl,
};

static int imx585_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	const struct imx585 *imx585 = to_imx585(sd);

	if (code->index >= IMX585_NUM_FORMATS)
		return -EINVAL;

	code->code = imx585->formats[code->index].code;

	return 0;
}

static int imx585_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct imx585 *imx585 = to_imx585(sd);

	if (fse->code != imx585->formats[0].code &&
	    fse->code != imx585->formats[1].code)
		return -EINVAL;

	if (fse->index >= IMX585_NUM_MODES)
		return -EINVAL;

	fse->min_width = imx585_modes[fse->index].width;
	fse->max_width = imx585_modes[fse->index].width;
	fse->min_height = imx585_modes[fse->index].height;
	fse->max_height = imx585_modes[fse->index].height;

	return 0;
}

static int imx585_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx585 *imx585 = to_imx585(sd);
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&imx585->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		framefmt = v4l2_subdev_get_try_format(&imx585->sd, sd_state,
						      fmt->pad);
	else
		framefmt = &imx585->current_format;

	fmt->format = *framefmt;

	mutex_unlock(&imx585->lock);

	return 0;
}

static u64 imx585_calc_pixel_rate(struct imx585 *imx585)
{
	return 148500000;
}

static int imx585_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx585 *imx585 = to_imx585(sd);
	const struct imx585_mode *mode;
	struct v4l2_mbus_framefmt *format;
	unsigned int i;

	mutex_lock(&imx585->lock);

	mode = v4l2_find_nearest_size(imx585_modes, IMX585_NUM_MODES,
				      width, height,
				      fmt->format.width, fmt->format.height);

	fmt->format.width = mode->width;
	fmt->format.height = mode->height;

	for (i = 0; i < IMX585_NUM_FORMATS; i++)
		if (imx585->formats[i].code == fmt->format.code)
			break;

	if (i >= IMX585_NUM_FORMATS)
		i = 0;

	fmt->format.code = imx585->formats[i].code;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc =
			V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->format.colorspace);
	fmt->format.quantization =
		V4L2_MAP_QUANTIZATION_DEFAULT(true, fmt->format.colorspace,
					      fmt->format.ycbcr_enc);
	fmt->format.xfer_func =
		V4L2_MAP_XFER_FUNC_DEFAULT(fmt->format.colorspace);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		format = v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
	} else {
		format = &imx585->current_format;
		imx585->current_mode = mode;
		imx585->bpp = imx585->formats[i].bpp;

		if (imx585->pixel_rate)
			__v4l2_ctrl_s_ctrl_int64(imx585->pixel_rate,
						 imx585_calc_pixel_rate(imx585));

		if (imx585->hblank) {
			__v4l2_ctrl_modify_range(imx585->hblank,
						 mode->hmax - mode->width,
						 IMX585_HMAX_MAX - mode->width,
						 1, mode->hmax - mode->width);
			__v4l2_ctrl_s_ctrl(imx585->hblank,
					   mode->hmax - mode->width);
		}
		if (imx585->vblank) {
			__v4l2_ctrl_modify_range(imx585->vblank,
						 mode->vmax - mode->height,
						 IMX585_VMAX_MAX - mode->height,
						 1,
						 mode->vmax - mode->height);
			__v4l2_ctrl_s_ctrl(imx585->vblank,
					   mode->vmax - mode->height);
		}
		if (imx585->exposure)
			__v4l2_ctrl_modify_range(imx585->exposure,
						 IMX585_EXPOSURE_MIN,
						 mode->vmax - 2,
						 IMX585_EXPOSURE_STEP,
						 mode->vmax - 2);
	}

	*format = fmt->format;

	mutex_unlock(&imx585->lock);

	return 0;
}

static int imx585_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = 1936;
	fmt.format.height = 1100;

	imx585_set_fmt(subdev, sd_state, &fmt);

	return 0;
}

static int imx585_write_current_format(struct imx585 *imx585)
{
	u8 ad_md_bit;
	int ret;

	switch (imx585->current_format.code) {
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_Y10_1X10:
		ad_md_bit = 0x00;
		break;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_Y12_1X12:
		ad_md_bit = 0x01;
		break;
	default:
		dev_err(imx585->dev, "Unknown pixel format\n");
		return -EINVAL;
	}

	ret = imx585_write_reg(imx585, IMX585_ADBIT, ad_md_bit);
	if (ret < 0)
		return ret;

	ret = imx585_write_reg(imx585, IMX585_MDBIT, ad_md_bit);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct v4l2_rect *
__imx585_get_pad_crop(struct imx585 *imx585,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&imx585->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx585->current_mode->crop;
	}

	return NULL;
}

static int imx585_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx585 *imx585 = to_imx585(sd);

		mutex_lock(&imx585->lock);
		sel->r = *__imx585_get_pad_crop(imx585, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&imx585->lock);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = IMX585_NATIVE_WIDTH;
		sel->r.height = IMX585_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = IMX585_PIXEL_ARRAY_TOP;
		sel->r.left = IMX585_PIXEL_ARRAY_LEFT;
		sel->r.width = IMX585_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX585_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

/* Start streaming */
static int imx585_start_streaming(struct imx585 *imx585)
{
	int ret;

	/* Set init register settings */
	ret = imx585_set_register_array(imx585, imx585_global_settings,
					ARRAY_SIZE(imx585_global_settings));
	if (ret < 0) {
		dev_err(imx585->dev, "Could not set init registers\n");
		return ret;
	}
pr_err("write INCK_SEL with %02x\n", imx585->inck_sel);
	ret = imx585_write_reg(imx585, IMX585_INCK_SEL, imx585->inck_sel);
	if (ret < 0)
		return ret;

	/* Apply the register values related to current frame format */
	ret = imx585_write_current_format(imx585);
	if (ret < 0) {
		dev_err(imx585->dev, "Could not set frame format\n");
		return ret;
	}

	/* Apply default values of current mode */
	ret = imx585_set_register_array(imx585,
					imx585->current_mode->mode_data,
					imx585->current_mode->mode_data_size);
	if (ret < 0) {
		dev_err(imx585->dev, "Could not set current mode\n");
		return ret;
	}

	/* Apply lane config registers of current mode */
	ret = imx585_write_reg(imx585, IMX585_CSI_LANE_MODE,
			       imx585->nlanes == 2 ? 0x01 : 0x03);
	if (ret < 0)
		return ret;

	ret = imx585_write_reg(imx585, IMX585_LANE_RATE,
			       //imx585->nlanes == 2 ? IMX585_LANE_RATE_720 : // for FPD Limk III
			       imx585->nlanes == 2 ? IMX585_LANE_RATE_1188 :
			       //imx585->nlanes == 2 ? IMX585_LANE_RATE_1188 :
						     IMX585_LANE_RATE_594);
	if (ret < 0)
		return ret;

	/* Apply customized values from user */
	ret = v4l2_ctrl_handler_setup(imx585->sd.ctrl_handler);
	if (ret) {
		dev_err(imx585->dev, "Could not sync v4l2 controls\n");
		return ret;
	}

	ret = imx585_write_reg(imx585, IMX585_STANDBY, 0x00);
	if (ret < 0)
		return ret;

	msleep(30);

	/* Start streaming */
	return imx585_write_reg(imx585, IMX585_XMSTA, 0x00);
}

static int imx585_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx585 *imx585 = to_imx585(sd);
	int ret = 0;

	if (enable) {
		ret = pm_runtime_resume_and_get(imx585->dev);
		if (ret < 0)
			goto unlock_and_return;

		ret = imx585_start_streaming(imx585);
		if (ret) {
			dev_err(imx585->dev, "Start stream failed\n");
			pm_runtime_put(imx585->dev);
			goto unlock_and_return;
		}
	} else {
		imx585_stop_streaming(imx585);
		pm_runtime_put(imx585->dev);
	}
	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx585->vflip, enable);
	__v4l2_ctrl_grab(imx585->hflip, enable);

unlock_and_return:

	return ret;
}

static int imx585_get_regulators(struct device *dev, struct imx585 *imx585)
{
	unsigned int i;

	for (i = 0; i < IMX585_NUM_SUPPLIES; i++)
		imx585->supplies[i].supply = imx585_supply_name[i];

	return devm_regulator_bulk_get(dev, IMX585_NUM_SUPPLIES,
				       imx585->supplies);
}

static int imx585_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx585 *imx585 = to_imx585(sd);
	int ret;

	ret = clk_prepare_enable(imx585->xclk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		return ret;
	}

	ret = regulator_bulk_enable(IMX585_NUM_SUPPLIES, imx585->supplies);
	if (ret) {
		dev_err(dev, "Failed to enable regulators\n");
		clk_disable_unprepare(imx585->xclk);
		return ret;
	}

	usleep_range(1, 2);
	gpiod_set_value_cansleep(imx585->rst_gpio, 0);
	usleep_range(30000, 31000);

	return 0;
}

static int imx585_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx585 *imx585 = to_imx585(sd);

	clk_disable_unprepare(imx585->xclk);
	gpiod_set_value_cansleep(imx585->rst_gpio, 1);
	regulator_bulk_disable(IMX585_NUM_SUPPLIES, imx585->supplies);

	return 0;
}

static const struct dev_pm_ops imx585_pm_ops = {
	SET_RUNTIME_PM_OPS(imx585_power_off, imx585_power_on, NULL)
};

static const struct v4l2_subdev_core_ops imx585_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx585_video_ops = {
	.s_stream = imx585_set_stream,
};

static const struct v4l2_subdev_pad_ops imx585_pad_ops = {
	.init_cfg = imx585_entity_init_cfg,
	.enum_mbus_code = imx585_enum_mbus_code,
	.enum_frame_size = imx585_enum_frame_size,
	.get_fmt = imx585_get_fmt,
	.set_fmt = imx585_set_fmt,
	.get_selection = imx585_get_selection,
};

static const struct v4l2_subdev_ops imx585_subdev_ops = {
	.core = &imx585_core_ops,
	.video = &imx585_video_ops,
	.pad = &imx585_pad_ops,
};

static const struct media_entity_operations imx585_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/*
 * Returns 0 if all link frequencies used by the driver for the given number
 * of MIPI data lanes are mentioned in the device tree, or the value of the
 * first missing frequency otherwise.
 */
static s64 imx585_check_link_freqs(const struct imx585 *imx585,
				   const struct v4l2_fwnode_endpoint *ep)
{
	int i, j;
	const s64 *freqs = imx585_link_freqs_ptr(imx585);
	int freqs_count = imx585_link_freqs_num(imx585);

	for (i = 0; i < freqs_count; i++) {
		for (j = 0; j < ep->nr_of_link_frequencies; j++)
			if (freqs[i] == ep->link_frequencies[j])
				break;
		if (j == ep->nr_of_link_frequencies)
			return freqs[i];
	}
	return 0;
}

static const struct of_device_id imx585_of_match[] = {
	{ .compatible = "sony,imx585", .data = imx585_colour_formats },
	{ .compatible = "sony,imx585-mono", .data = imx585_mono_formats },
	{ /* sentinel */ }
};

static int imx585_probe(struct i2c_client *client)
{
	struct v4l2_fwnode_device_properties props;
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	/* Only CSI2 is supported for now: */
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	const struct of_device_id *match;
	const struct imx585_mode *mode;
	struct v4l2_ctrl *ctrl;
	struct imx585 *imx585;
	u32 xclk_freq;
	s64 fq;
	int ret;

	imx585 = devm_kzalloc(dev, sizeof(*imx585), GFP_KERNEL);
	if (!imx585)
		return -ENOMEM;

	imx585->dev = dev;
	imx585->regmap = devm_regmap_init_i2c(client, &imx585_regmap_config);
	if (IS_ERR(imx585->regmap)) {
		dev_err(dev, "Unable to initialize I2C\n");
		return -ENODEV;
	}

	match = of_match_device(imx585_of_match, dev);
	if (!match)
		return -ENODEV;
	imx585->formats = (const struct imx585_pixfmt *)match->data;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "Endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret == -ENXIO) {
		dev_err(dev, "Unsupported bus type, should be CSI2\n");
		goto free_err;
	} else if (ret) {
		dev_err(dev, "Parsing endpoint node failed\n");
		goto free_err;
	}

	/* Get number of data lanes */
	imx585->nlanes = ep.bus.mipi_csi2.num_data_lanes;
	if (imx585->nlanes != 2 && imx585->nlanes != 4) {
		dev_err(dev, "Invalid data lanes: %d\n", imx585->nlanes);
		ret = -EINVAL;
		goto free_err;
	}

	dev_dbg(dev, "Using %u data lanes\n", imx585->nlanes);

	if (!ep.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		ret = -EINVAL;
		goto free_err;
	}

	/* Check that link frequences for all the modes are in device tree */
	fq = imx585_check_link_freqs(imx585, &ep);
	if (fq) {
		dev_err(dev, "Link frequency of %lld is not supported\n", fq);
		ret = -EINVAL;
		goto free_err;
	}

	/* get system clock (xclk) */
	imx585->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(imx585->xclk)) {
		dev_err(dev, "Could not get xclk");
		ret = PTR_ERR(imx585->xclk);
		goto free_err;
	}

	ret = fwnode_property_read_u32(dev_fwnode(dev), "clock-frequency",
				       &xclk_freq);
	if (ret) {
		dev_err(dev, "Could not get xclk frequency\n");
		goto free_err;
	}

	/* external clock can be one of a range of values - validate it */
	switch (xclk_freq) {
	case 74250000:
		imx585->inck_sel = IMX585_INCK_SEL_74_25;
		break;
	case 37125000:
		imx585->inck_sel = IMX585_INCK_SEL_37_125;
		break;
	case 72000000:
		imx585->inck_sel = IMX585_INCK_SEL_72;
		break;
	case 27000000:
		imx585->inck_sel = IMX585_INCK_SEL_27;
		break;
	case 24000000:
		imx585->inck_sel = IMX585_INCK_SEL_24;
		break;
	default:
		dev_err(dev, "External clock frequency %u is not supported\n",
			xclk_freq);
		ret = -EINVAL;
		goto free_err;
	}

	ret = clk_set_rate(imx585->xclk, xclk_freq);
	if (ret) {
		dev_err(dev, "Could not set xclk frequency\n");
		goto free_err;
	}

	ret = imx585_get_regulators(dev, imx585);
	if (ret < 0) {
		dev_err(dev, "Cannot get regulators\n");
		goto free_err;
	}

	imx585->rst_gpio = devm_gpiod_get_optional(dev, "reset",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(imx585->rst_gpio)) {
		dev_err(dev, "Cannot get reset gpio\n");
		ret = PTR_ERR(imx585->rst_gpio);
		goto free_err;
	}

	mutex_init(&imx585->lock);

	/*
	 * Initialize the frame format. In particular, imx585->current_mode
	 * and imx585->bpp are set to defaults: imx585_calc_pixel_rate() call
	 * below relies on these fields.
	 */
	imx585_entity_init_cfg(&imx585->sd, NULL);

	v4l2_ctrl_handler_init(&imx585->ctrls, 11);

	v4l2_ctrl_new_std(&imx585->ctrls, &imx585_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 0, 100, 1, 0);

	mode = imx585->current_mode;
	imx585->hblank = v4l2_ctrl_new_std(&imx585->ctrls, &imx585_ctrl_ops,
					   V4L2_CID_HBLANK,
					   mode->hmax - mode->width,
					   IMX585_HMAX_MAX - mode->width, 1,
					   mode->hmax - mode->width);

	imx585->vblank = v4l2_ctrl_new_std(&imx585->ctrls, &imx585_ctrl_ops,
					   V4L2_CID_VBLANK,
					   mode->vmax - mode->height,
					   IMX585_VMAX_MAX - mode->height, 1,
					   mode->vmax - mode->height);

	imx585->exposure = v4l2_ctrl_new_std(&imx585->ctrls, &imx585_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX585_EXPOSURE_MIN,
					     mode->vmax - 2,
					     IMX585_EXPOSURE_STEP,
					     mode->vmax - 2);

	imx585->hflip = v4l2_ctrl_new_std(&imx585->ctrls, &imx585_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	imx585->vflip = v4l2_ctrl_new_std(&imx585->ctrls, &imx585_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	ctrl = v4l2_ctrl_new_int_menu(&imx585->ctrls, &imx585_ctrl_ops,
				      V4L2_CID_LINK_FREQ,
				      imx585_link_freqs_num(imx585) - 1, 0,
				      imx585_link_freqs_ptr(imx585));
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx585->pixel_rate = v4l2_ctrl_new_std(&imx585->ctrls, &imx585_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       1, INT_MAX, 1,
					       imx585_calc_pixel_rate(imx585));

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto free_ctrl;

	ret = v4l2_ctrl_new_fwnode_properties(&imx585->ctrls, &imx585_ctrl_ops,
					      &props);
	if (ret)
		goto free_ctrl;

	imx585->sd.ctrl_handler = &imx585->ctrls;

	if (imx585->ctrls.error) {
		dev_err(dev, "Control initialization error %d\n",
			imx585->ctrls.error);
		ret = imx585->ctrls.error;
		goto free_ctrl;
	}

	v4l2_i2c_subdev_init(&imx585->sd, client, &imx585_subdev_ops);
	imx585->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		V4L2_SUBDEV_FL_HAS_EVENTS;
	imx585->sd.dev = &client->dev;
	imx585->sd.entity.ops = &imx585_subdev_entity_ops;
	imx585->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	imx585->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx585->sd.entity, 1, &imx585->pad);
	if (ret < 0) {
		dev_err(dev, "Could not register media entity\n");
		goto free_ctrl;
	}

	/* Initialize the frame format (this also sets imx585->current_mode) */
	imx585_entity_init_cfg(&imx585->sd, NULL);

	ret = v4l2_async_register_subdev(&imx585->sd);
	if (ret < 0) {
		dev_err(dev, "Could not register v4l2 device\n");
		goto free_entity;
	}

	/* Power on the device to match runtime PM state below */
	ret = imx585_power_on(dev);
	if (ret < 0) {
		dev_err(dev, "Could not power on the device\n");
		goto free_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	v4l2_fwnode_endpoint_free(&ep);

	return 0;

free_entity:
	media_entity_cleanup(&imx585->sd.entity);
free_ctrl:
	v4l2_ctrl_handler_free(&imx585->ctrls);
	mutex_destroy(&imx585->lock);
free_err:
	v4l2_fwnode_endpoint_free(&ep);

	return ret;
}

/*
static int imx585_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx585 *imx585 = to_imx585(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	mutex_destroy(&imx585->lock);

	pm_runtime_disable(imx585->dev);
	if (!pm_runtime_status_suspended(imx585->dev))
		imx585_power_off(imx585->dev);
	pm_runtime_set_suspended(imx585->dev);

	return 0;
}
*/

MODULE_DEVICE_TABLE(of, imx585_of_match);

static struct i2c_driver imx585_i2c_driver = {
	.probe_new  = imx585_probe,
	//.remove = imx585_remove,
	.driver = {
		.name  = "imx585",
		.pm = &imx585_pm_ops,
		.of_match_table = of_match_ptr(imx585_of_match),
	},
};

module_i2c_driver(imx585_i2c_driver);

MODULE_DESCRIPTION("Sony IMX585 CMOS Image Sensor Driver");
MODULE_AUTHOR("Soho Enterprise Ltd.");
MODULE_AUTHOR("Tetsuya Nomura <tetsuya.nomura@soho-enterprise.com>");
MODULE_LICENSE("GPL v2");
