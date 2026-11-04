// SPDX-License-Identifier: GPL-2.0
/* Sony IMX577 for Allwinner VIN. Register tables derived from the Radxa driver:
 * Copyright (C) 2022 Rockchip Electronics Co., Ltd.
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "camera.h"
#include "sensor_helper.h"

#define SENSOR_NAME "imx577_mipi"
#define IMX577_VTS 3116
#define EXP_MIN (8 << 4)
#define EXP_MAX ((IMX577_VTS - 22) << 4)
#define GAIN_MIN 16
#define GAIN_MAX (22 * 16)

struct imx577 {
	struct sensor_info info;
	bool powered;
	bool streaming;
	u32 test_pattern;
};

static struct regval_list sensor_global_regs[] = {
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x3C7E, 0x01},
	{0x3C7F, 0x02},
	{0x38A8, 0x1F},
	{0x38A9, 0xFF},
	{0x38AA, 0x1F},
	{0x38AB, 0xFF},
	{0x55D4, 0x00},
	{0x55D5, 0x00},
	{0x55D6, 0x07},
	{0x55D7, 0xFF},
	{0x55E8, 0x07},
	{0x55E9, 0xFF},
	{0x55EA, 0x00},
	{0x55EB, 0x00},
	{0x575C, 0x07},
	{0x575D, 0xFF},
	{0x575E, 0x00},
	{0x575F, 0x00},
	{0x5764, 0x00},
	{0x5765, 0x00},
	{0x5766, 0x07},
	{0x5767, 0xFF},
	{0x5974, 0x04},
	{0x5975, 0x01},
	{0x5F10, 0x09},
	{0x5F11, 0x92},
	{0x5F12, 0x32},
	{0x5F13, 0x72},
	{0x5F14, 0x16},
	{0x5F15, 0xBA},
	{0x5F17, 0x13},
	{0x5F18, 0x24},
	{0x5F19, 0x60},
	{0x5F1A, 0xE3},
	{0x5F1B, 0xAD},
	{0x5F1C, 0x74},
	{0x5F2D, 0x25},
	{0x5F5C, 0xD0},
	{0x6A22, 0x00},
	{0x6A23, 0x1D},
	{0x7BA8, 0x00},
	{0x7BA9, 0x00},
	{0x886B, 0x00},
	{0x9002, 0x0A},
	{0x9004, 0x1A},
	{0x9214, 0x93},
	{0x9215, 0x69},
	{0x9216, 0x93},
	{0x9217, 0x6B},
	{0x9218, 0x93},
	{0x9219, 0x6D},
	{0x921A, 0x57},
	{0x921B, 0x58},
	{0x921C, 0x57},
	{0x921D, 0x59},
	{0x921E, 0x57},
	{0x921F, 0x5A},
	{0x9220, 0x57},
	{0x9221, 0x5B},
	{0x9222, 0x93},
	{0x9223, 0x02},
	{0x9224, 0x93},
	{0x9225, 0x03},
	{0x9226, 0x93},
	{0x9227, 0x04},
	{0x9228, 0x93},
	{0x9229, 0x05},
	{0x922A, 0x98},
	{0x922B, 0x21},
	{0x922C, 0xB2},
	{0x922D, 0xDB},
	{0x922E, 0xB2},
	{0x922F, 0xDC},
	{0x9230, 0xB2},
	{0x9231, 0xDD},
	{0x9232, 0xB2},
	{0x9233, 0xE1},
	{0x9234, 0xB2},
	{0x9235, 0xE2},
	{0x9236, 0xB2},
	{0x9237, 0xE3},
	{0x9238, 0xB7},
	{0x9239, 0xB9},
	{0x923A, 0xB7},
	{0x923B, 0xBB},
	{0x923C, 0xB7},
	{0x923D, 0xBC},
	{0x923E, 0xB7},
	{0x923F, 0xC5},
	{0x9240, 0xB7},
	{0x9241, 0xC7},
	{0x9242, 0xB7},
	{0x9243, 0xC9},
	{0x9244, 0x98},
	{0x9245, 0x56},
	{0x9246, 0x98},
	{0x9247, 0x55},
	{0x9380, 0x00},
	{0x9381, 0x62},
	{0x9382, 0x00},
	{0x9383, 0x56},
	{0x9384, 0x00},
	{0x9385, 0x52},
	{0x9388, 0x00},
	{0x9389, 0x55},
	{0x938A, 0x00},
	{0x938B, 0x55},
	{0x938C, 0x00},
	{0x938D, 0x41},

};

static struct regval_list sensor_30fps_regs[] = {
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x03},
	{0x0342, 0x23},
	{0x0343, 0x18},
	{0x0340, 0x0C},
	{0x0341, 0x2c},
	{0x3210, 0x00},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0F},
	{0x0349, 0xD7},
	{0x034A, 0x0B},
	{0x034B, 0xDF},
	{0x00E3, 0x00},
	{0x00E4, 0x00},
	{0x00E5, 0x01},
	{0x00FC, 0x0A},
	{0x00FD, 0x0A},
	{0x00FE, 0x0A},
	{0x00FF, 0x0A},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x00},
	{0x3140, 0x02},
	{0x3241, 0x11},
	{0x3250, 0x03},
	{0x3E10, 0x00},
	{0x3E11, 0x00},
	{0x3F0D, 0x00},
	{0x3F42, 0x00},
	{0x3F43, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x0F},
	{0x040D, 0xD8},
	{0x040E, 0x0B},
	{0x040F, 0xE0},
	{0x034C, 0x0F},
	{0x034D, 0xD8},
	{0x034E, 0x0B},
	{0x034F, 0xE0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5E},
	{0x0309, 0x0A},
	{0x030B, 0x02},
	{0x030D, 0x02},
	{0x030E, 0x00},
	{0x030F, 0xA6},
	{0x0310, 0x01},
	{0x0820, 0x0F},
	{0x0821, 0x90},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x3E20, 0x01},
	{0x3E37, 0x00},
	{0x3F50, 0x00},
	{0x3F56, 0x00},
	{0x3F57, 0x41},
	{0x3C0A, 0x5A},
	{0x3C0B, 0x55},
	{0x3C0C, 0x28},
	{0x3C0D, 0x07},
	{0x3C0E, 0xFF},
	{0x3C0F, 0x00},
	{0x3C10, 0x00},
	{0x3C11, 0x02},
	{0x3C12, 0x00},
	{0x3C13, 0x03},
	{0x3C14, 0x00},
	{0x3C15, 0x00},
	{0x3C16, 0x0C},
	{0x3C17, 0x0C},
	{0x3C18, 0x0C},
	{0x3C19, 0x0A},
	{0x3C1A, 0x0A},
	{0x3C1B, 0x0A},
	{0x3C1C, 0x00},
	{0x3C1D, 0x00},
	{0x3C1E, 0x00},
	{0x3C1F, 0x00},
	{0x3C20, 0x00},
	{0x3C21, 0x00},
	{0x3C22, 0x3F},
	{0x3C23, 0x0A},
	{0x3E35, 0x01},
	{0x3F4A, 0x03},
	{0x3F4B, 0xBF},
	{0x3F26, 0x00},
	{0x0202, 0x0C},
	{0x0203, 0x16},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},

};

/* Even-aligned, unbinned centre crop, retaining the 30 fps PLL timing. */
static struct regval_list sensor_1080p_regs[] = {
	{0x0344, 0x04}, {0x0345, 0x2c},
	{0x0346, 0x03}, {0x0347, 0xd4},
	{0x0348, 0x0b}, {0x0349, 0xab},
	{0x034a, 0x08}, {0x034b, 0x0b},
	{0x034c, 0x07}, {0x034d, 0x80},
	{0x034e, 0x04}, {0x034f, 0x38},
	{0x040c, 0x07}, {0x040d, 0x80},
	{0x040e, 0x04}, {0x040f, 0x38},
	{0x0101, 0x00},
};

static struct sensor_format_struct sensor_formats[] = {
	{
		.desc = "Raw RGB Bayer",
		.mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.bpp = 1,
	},
};

static struct sensor_win_size sensor_win_sizes[] = {
	{
		.width = 1920,
		.height = 1080,
		.width_input = 1920,
		.height_input = 1080,
		.hts = 8984,
		.vts = IMX577_VTS,
		.pclk = 840000000,
		.mipi_bps = 996000000,
		.fps_fixed = 30,
		.bin_factor = 1,
		.intg_min = EXP_MIN,
		.intg_max = EXP_MAX,
		.gain_min = GAIN_MIN,
		.gain_max = GAIN_MAX,
		.regs = sensor_1080p_regs,
		.regs_size = ARRAY_SIZE(sensor_1080p_regs),
	},
};

static struct imx577 *to_imx577(struct v4l2_subdev *sd)
{
	return container_of(to_state(sd), struct imx577, info);
}

static int sensor_write16(struct v4l2_subdev *sd, u16 reg, u16 value)
{
	int ret = sensor_write(sd, reg, value >> 8);

	if (ret)
		return ret;
	return sensor_write(sd, reg + 1, value & 0xff);
}

/* VIN exposure is in 1/16 line; VIN gain is in 1/16 unity gain. */
static int sensor_apply_exp_gain(struct v4l2_subdev *sd, u32 exp, u32 gain)
{
	struct imx577 *sensor = to_imx577(sd);
	int ret, release_ret;

	exp = clamp_t(u32, exp, EXP_MIN, EXP_MAX);
	gain = clamp_t(u32, gain, GAIN_MIN, GAIN_MAX);
	if (sensor->streaming) {
		ret = sensor_write(sd, 0x0104, 1);
		if (ret)
			return ret;
		ret = sensor_write16(sd, 0x0202, (exp + 8) >> 4);
		if (!ret)
			ret = sensor_write16(sd, 0x0204, 1024 - 16384 / gain);
		release_ret = sensor_write(sd, 0x0104, 0);
		if (ret || release_ret)
			return ret ? ret : release_ret;
	}
	sensor->info.exp = exp;
	sensor->info.gain = gain;
	return 0;
}

static int sensor_power(struct v4l2_subdev *sd, int on)
{
	struct imx577 *sensor = to_imx577(sd);
	int ret = 0;

	/* VIN's registered callback already holds info.lock around s_power. */
	switch (on) {
	case PWR_ON:
		if (sensor->powered)
			break;
		cci_lock(sd);
		vin_gpio_set_status(sd, RESET, 1);
		vin_gpio_set_status(sd, PWDN, 1);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		usleep_range(1000, 2000);
		/* PWDN drives the module's active-high PDN-LDO regulator enable. */
		ret = vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
		if (!ret) {
			/* TPAPX803S on the module holds reset for up to 560 ms. */
			msleep(600);
			vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
			/* On-module 24 MHz oscillator: leave host MCLK off. */
			msleep(30);
			sensor->powered = true;
		}
		cci_unlock(sd);
		break;
	case PWR_OFF:
		cci_lock(sd);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		vin_gpio_set_status(sd, RESET, 0);
		vin_gpio_set_status(sd, PWDN, 0);
		cci_unlock(sd);
		sensor->powered = false;
		sensor->streaming = false;
		break;
	case STBY_ON:
		ret = sensor_write(sd, 0x0100, 0);
		break;
	case STBY_OFF:
		ret = sensor_write(sd, 0x0100, sensor->streaming);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	if (val > 1)
		return -EINVAL;
	vin_gpio_write(sd, RESET, !val);
	usleep_range(10000, 12000);
	return 0;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	data_type high, low;
	int ret;

	ret = sensor_read(sd, 0x0016, &high);
	if (ret)
		return ret;
	ret = sensor_read(sd, 0x0017, &low);
	if (ret)
		return ret;
	if (high != 0x05 || low != 0x77) {
		v4l2_err(sd, "unexpected chip ID %02x%02x\n", high, low);
		return -ENODEV;
	}
	v4l2_info(sd, "IMX577 detected, module oscillator 24 MHz\n");
	return 0;
}

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx577 *sensor = to_imx577(sd);
	struct sensor_info *info = &sensor->info;
	int ret;

	mutex_lock(&info->lock);
	if (sensor->streaming == !!enable) {
		ret = 0;
		goto unlock;
	}
	if (!enable) {
		ret = sensor_write(sd, 0x0100, 0);
		if (!ret)
			sensor->streaming = false;
		goto unlock;
	}
	if (!sensor->powered) {
		ret = -EPIPE;
		goto unlock;
	}
	ret = sensor_write(sd, 0x0100, 0);
	if (ret)
		goto unlock;
	ret = sensor_write_array(sd, sensor_global_regs, ARRAY_SIZE(sensor_global_regs));
	if (ret)
		goto unlock;
	ret = sensor_write_array(sd, sensor_30fps_regs, ARRAY_SIZE(sensor_30fps_regs));
	if (ret)
		goto unlock;
	ret = sensor_write_array(sd, info->current_wins->regs, info->current_wins->regs_size);
	if (ret)
		goto unlock;
	ret = sensor_write16(sd, 0x0600, sensor->test_pattern ? 2 : 0);
	if (ret)
		goto unlock;
	sensor->streaming = true;
	ret = sensor_apply_exp_gain(sd, info->exp, info->gain);
	if (!ret)
		ret = sensor_write(sd, 0x0100, 1);
	if (ret) {
		sensor_write(sd, 0x0100, 0);
		sensor->streaming = false;
	} else {
		v4l2_info(sd, "stream 1920x1080 RAW10 4 lanes, 30 fps\n");
	}
unlock:
	mutex_unlock(&info->lock);
	return ret;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sensor_info *info = to_state(sd);
	struct sensor_exp_gain *exp_gain = arg;
	int ret = 0;

	mutex_lock(&info->lock);
	switch (cmd) {
	case GET_CURRENT_WIN_CFG:
		memcpy(arg, info->current_wins, sizeof(*info->current_wins));
		break;
	case VIDIOC_VIN_SENSOR_EXP_GAIN:
		ret = sensor_apply_exp_gain(sd, exp_gain->exp_val, exp_gain->gain_val);
		break;
	case VIDIOC_VIN_SENSOR_CFG_REQ:
		sensor_cfg_req(sd, arg);
		break;
	case VIDIOC_VIN_SENSOR_SET_FPS:
		/* This initial mode deliberately keeps a fixed frame length. */
		ret = ((struct sensor_fps *)arg)->fps == 30 ? 0 : -EINVAL;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&info->lock);
	return ret;
}

static int sensor_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2_DPHY;
	/* The 6.6 VIN BSP interprets this field as its legacy lane/channel mask. */
	cfg->bus.mipi_csi2.num_data_lanes = V4L2_MBUS_CSI2_4_LANE |
					 V4L2_MBUS_CSI2_CHANNEL_0;
	return 0;
}

static int sensor_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info = container_of(ctrl->handler, struct sensor_info, handler);

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		ctrl->val = info->gain;
		return 0;
	case V4L2_CID_EXPOSURE:
		ctrl->val = info->exp;
		return 0;
	default:
		return -EINVAL;
	}
}

static int sensor_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info = container_of(ctrl->handler, struct sensor_info, handler);
	struct imx577 *sensor = container_of(info, struct imx577, info);
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_apply_exp_gain(&info->sd, info->exp, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_apply_exp_gain(&info->sd, ctrl->val, info->gain);
	case V4L2_CID_TEST_PATTERN:
		if (sensor->streaming) {
			ret = sensor_write16(&info->sd, 0x0600, ctrl->val ? 2 : 0);
			if (ret)
				return ret;
		}
		sensor->test_pattern = ctrl->val;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops sensor_ctrl_ops = {
	.g_volatile_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
};

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl32 = sensor_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_stream = sensor_s_stream,
};

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code = sensor_enum_mbus_code,
	.enum_frame_size = sensor_enum_frame_size,
	.enum_frame_interval = sensor_enum_frame_interval,
	.get_fmt = sensor_get_fmt,
	.set_fmt = sensor_set_fmt,
	.get_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
	.pad = &sensor_pad_ops,
};

static struct cci_driver cci_drv = {
	.name = SENSOR_NAME,
	.addr_width = CCI_BITS_16,
	.data_width = CCI_BITS_8,
};

static const char * const test_patterns[] = { "Disabled", "Color Bars" };

static int sensor_probe(struct i2c_client *client)
{
	struct imx577 *sensor;
	struct sensor_info *info;
	struct v4l2_ctrl *ctrl;
	int ret;

	sensor = kzalloc(sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	info = &sensor->info;
	mutex_init(&info->lock);
	info->fmt = &sensor_formats[0];
	info->fmt_pt = sensor_formats;
	info->win_pt = sensor_win_sizes;
	info->current_wins = &sensor_win_sizes[0];
	info->fmt_num = ARRAY_SIZE(sensor_formats);
	info->win_size_num = ARRAY_SIZE(sensor_win_sizes);
	info->width = 1920;
	info->height = 1080;
	info->tpf.numerator = 1;
	info->tpf.denominator = 30;
	info->sensor_field = V4L2_FIELD_NONE;
	info->stream_seq = MIPI_BEFORE_SENSOR;
	info->combo_mode = CMB_TERMINAL_RES | CMB_PHYA_OFFSET2 | MIPI_NORMAL_MODE;
	info->deskew = 0x04;
	info->exp = 1000 << 4;
	info->gain = GAIN_MIN;
#if IS_ENABLED(CONFIG_SAME_I2C)
	info->sensor_i2c_addr = client ? client->addr : 0x1a;
#endif
	v4l2_ctrl_handler_init(&info->handler, 3);
	info->handler.lock = &info->lock;
	ctrl = v4l2_ctrl_new_std(&info->handler, &sensor_ctrl_ops, V4L2_CID_GAIN,
				 GAIN_MIN, GAIN_MAX, 1, GAIN_MIN);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	ctrl = v4l2_ctrl_new_std(&info->handler, &sensor_ctrl_ops, V4L2_CID_EXPOSURE,
				 EXP_MIN, EXP_MAX, 1, info->exp);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	v4l2_ctrl_new_std_menu_items(&info->handler, &sensor_ctrl_ops,
				     V4L2_CID_TEST_PATTERN, 1, 0, 0, test_patterns);
	ret = info->handler.error;
	if (ret)
		goto free;
	ret = cci_dev_probe_helper(&info->sd, client, &sensor_ops, &cci_drv);
	if (ret)
		goto free;
	info->sd.ctrl_handler = &info->handler;
	return 0;
free:
	v4l2_ctrl_handler_free(&info->handler);
	mutex_destroy(&info->lock);
	kfree(sensor);
	return ret;
}

static void sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = cci_dev_remove_helper(client, &cci_drv);
	struct imx577 *sensor = to_imx577(sd);

	v4l2_ctrl_handler_free(&sensor->info.handler);
	mutex_destroy(&sensor->info.lock);
	kfree(sensor);
}

static const struct i2c_device_id sensor_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_driver = {
	.driver.name = SENSOR_NAME,
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static int __init init_sensor(void)
{
	return cci_dev_init_helper(&sensor_driver);
}

static void __exit exit_sensor(void)
{
	cci_dev_exit_helper(&sensor_driver);
}

VIN_INIT_DRIVERS(init_sensor);
module_exit(exit_sensor);
MODULE_DESCRIPTION("Sony IMX577 RAW10 sensor for Allwinner VIN");
MODULE_LICENSE("GPL");
