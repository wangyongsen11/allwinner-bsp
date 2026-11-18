// SPDX-License-Identifier: GPL-2.0
/* onsemi AR0234 for Allwinner VIN.
 * Register tables: Radxa AR0234 driver, based on Kurokesu/ar0234-rpi-driver.
 * Copyright (c) 2026 Radxa Computer (Shenzhen) Co., Ltd.
 * VIN integration follows the BSP's imx577_mipi driver.
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "camera.h"
#include "sensor_helper.h"

#define SENSOR_NAME "ar0234_mipi"
#define AR0234_VTS 4902
#define EXP_MIN (2 << 4)
#define EXP_MAX ((AR0234_VTS - 6) << 4)
#define GAIN_MIN 16
#define GAIN_MAX (8 * 16)

struct ar0234 {
	struct sensor_info info;
	bool powered;
	bool streaming;
	u32 test_pattern;
};

/* RAW10, four lanes. 24 MHz / 2 * 75 = 900 MHz PLL output.
 * Validated on the A7A module with a 33 ms frame interval.
 * Register sequences derived from radxa/kernel linux-6.1-stan-rkr4.1.
 */
static struct regval_list sensor_global_regs[] = {
	{0x301A, 0x0001}, {REG_DLY, 200}, {0x301A, 0x2058},
	{0x302A, 0x0005}, {0x302C, 0x0001}, {0x302E, 0x0002},
	{0x3030, 0x004B}, {0x3036, 0x000A}, {0x3038, 0x0001},
	{0x31B0, 0x0082}, {0x31B2, 0x005C},
	{0x31B4, 0x4248}, {0x31B6, 0x4258},
	{0x31B8, 0x904B}, {0x31BA, 0x030B},
	{0x31BC, 0x0D89}, {0x3354, 0x002B}, {0x31AC, 0x0A0A},
	{0x31AE, 0x0204},
	{0x306E, 0x9010}, {0x3082, 0x0003},
	{0x31D0, 0x0000}, {0x3088, 0x8050}, {0x3086, 0x9237},
	{0x3096, 0x0280}, {0x31E0, 0x0003},
	{0x3F4C, 0x121F}, {0x3F4E, 0x121F}, {0x3F50, 0x0B81},
	{0x3ED2, 0xFA96}, {0x3180, 0x824F},
	{0x3ECC, 0x0C42}, {0x3ECC, 0x0C42},
	{0x30F0, 0x2283}, {0x3102, 0x5000},
	{0x30B4, 0x0011}, {0x3064, 0x1982},
	{0x30B0, 0x0028}, {0x30BA, 0x7622},
	{0x3060, 0x0000},
};

static struct regval_list sensor_1200p_regs[] = {
	{0x3002, 0x0008}, {0x3004, 0x0008},
	{0x3006, 0x04B7}, {0x3008, 0x0787},
	{0x30A2, 0x0001}, {0x30A6, 0x0001},
	{0x3040, 0x0000}, {0x300C, 612},
};

static struct regval_list sensor_1080p_regs[] = {
	{0x3002, 0x0044}, {0x3004, 0x0008},
	{0x3006, 0x047B}, {0x3008, 0x0787},
	{0x30A2, 1}, {0x30A6, 1}, {0x3040, 0}, {0x300C, 612},
};

static struct regval_list sensor_720p_regs[] = {
	{0x3002, 0x00F8}, {0x3004, 0x0148},
	{0x3006, 0x03C7}, {0x3008, 0x0647},
	{0x30A2, 1}, {0x30A6, 1}, {0x3040, 0}, {0x300C, 612},
};

static struct regval_list sensor_600p_regs[] = {
	{0x3002, 0x0008}, {0x3004, 0x0008},
	{0x3006, 0x04B7}, {0x3008, 0x0787},
	{0x30A2, 3}, {0x30A6, 3}, {0x3040, 0x3000}, {0x300C, 612},
};

static struct regval_list sensor_raw8_regs[] = {
	{0x302A, 4}, {0x3030, 60}, {0x3036, 8},
	{0x31B0, 0x0080}, {0x31B4, 0x5248},
	{0x31B8, 0x904C}, {0x31BA, 0x028B},
	{0x3354, 0x002A}, {0x31AC, 0x0808},
};

static struct sensor_format_struct sensor_formats[] = {
	{
		.desc = "Raw RGB Bayer",
		.mbus_code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.bpp = 1,
	},
	{
		.desc = "Raw RGB Bayer 8-bit",
		.mbus_code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.regs = sensor_raw8_regs,
		.regs_size = ARRAY_SIZE(sensor_raw8_regs),
		.bpp = 1,
	},
};

#define AR0234_WINDOW(w, h, fps, frame_lines, bin, table) { \
	.width = w, .height = h, .width_input = w, .height_input = h, \
	.hts = 2448, .vts = frame_lines, .pclk = 360000000, \
	.mipi_bps = 900000000, .fps_fixed = fps, .bin_factor = bin, \
	.intg_min = EXP_MIN, .intg_max = ((frame_lines) - 6) << 4, \
	.gain_min = GAIN_MIN, .gain_max = GAIN_MAX, \
	.regs = table, .regs_size = ARRAY_SIZE(table), \
}

static struct sensor_win_size sensor_win_sizes[] = {
	AR0234_WINDOW(1920, 1200, 120, 1224, 1, sensor_1200p_regs),
	AR0234_WINDOW(1920, 1080, 120, 1224, 1, sensor_1080p_regs),
	AR0234_WINDOW(1280, 720, 30, 4902, 1, sensor_720p_regs),
	AR0234_WINDOW(960, 600, 60, 2451, 2, sensor_600p_regs),
	{
		.width = 1920,
		.height = 1200,
		.width_input = 1920,
		.height_input = 1200,
		.hts = 2448,
		.vts = AR0234_VTS,
		.pclk = 360000000,
		.mipi_bps = 900000000,
		.fps_fixed = 30,
		.bin_factor = 1,
		.intg_min = EXP_MIN,
		.intg_max = EXP_MAX,
		.gain_min = GAIN_MIN,
		.gain_max = GAIN_MAX,
		.regs = sensor_1200p_regs,
		.regs_size = ARRAY_SIZE(sensor_1200p_regs),
	},
	AR0234_WINDOW(1920, 1200, 60, 2451, 1, sensor_1200p_regs),
	AR0234_WINDOW(1920, 1080, 60, 2451, 1, sensor_1080p_regs),
	AR0234_WINDOW(1920, 1080, 30, 4902, 1, sensor_1080p_regs),
};

static struct ar0234 *to_ar0234(struct v4l2_subdev *sd)
{
	return container_of(to_state(sd), struct ar0234, info);
}

/* VIN S_PARM can follow S_FMT. Resolve that interval before reporting or
 * programming the mode, while preserving the negotiated image dimensions.
 * Caller holds info.lock; do not change an already running mode.
 */
static int ar0234_select_interval(struct ar0234 *sensor)
{
	struct sensor_info *info = &sensor->info;
	unsigned int i;

	if (sensor->streaming)
		return 0;
	if (!info->tpf.numerator || !info->tpf.denominator)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(sensor_win_sizes); i++) {
		struct sensor_win_size *win = &sensor_win_sizes[i];

		if (win->width == info->current_wins->width &&
		    win->height == info->current_wins->height &&
		    (u64)win->fps_fixed * info->tpf.numerator == info->tpf.denominator) {
			info->current_wins = win;
			return 0;
		}
	}
	return -EINVAL;
}

/* VIN units: 1/16 line and 1/16 unity gain. Initial bring-up uses digital
 * gain only (Q7); analog gain remains unity pending ISP calibration.
 */
static int sensor_apply_exp_gain(struct v4l2_subdev *sd, u32 exp, u32 gain)
{
	struct ar0234 *sensor = to_ar0234(sd);
	int ret, release_ret;

	exp = clamp_t(u32, exp, EXP_MIN, sensor->info.current_wins->intg_max);
	gain = clamp_t(u32, gain, GAIN_MIN, GAIN_MAX);
	if (sensor->streaming) {
		ret = cci_write_a16_d8(sd, 0x3022, 1);
		if (ret)
			return ret;
		ret = sensor_write(sd, 0x3012, (exp + 8) >> 4);
		if (!ret)
			ret = sensor_write(sd, 0x305E, gain * 8);
		release_ret = cci_write_a16_d8(sd, 0x3022, 0);
		if (ret || release_ret)
			return ret ? ret : release_ret;
	}
	sensor->info.exp = exp;
	sensor->info.gain = gain;
	return 0;
}

static int sensor_power(struct v4l2_subdev *sd, int on)
{
	struct ar0234 *sensor = to_ar0234(sd);
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
			/* Allow module power to settle before releasing reset. */
			msleep(600);
			vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
			/* On-module 24 MHz oscillator: leave host MCLK off. */
			msleep(250);
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
		ret = sensor_write(sd, 0x301A, 0x2058);
		break;
	case STBY_OFF:
		ret = sensor_write(sd, 0x301A, sensor->streaming ? 0x205C : 0x2058);
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
	struct sensor_info *info = to_state(sd);
	data_type id;
	int ret = sensor_read(sd, 0x3000, &id);

	if (ret)
		return ret;
	if (id != 0x0a56) {
		v4l2_err(sd, "expected color AR0234 (0x0a56), got 0x%04x\n", id);
		return -ENODEV;
	}
	/* VIN rejects S_PARM while a previous zero interval remains cached. */
	if (!info->tpf.numerator || !info->tpf.denominator) {
		info->tpf.numerator = 1;
		info->tpf.denominator = 30;
	}
	v4l2_info(sd, "AR0234 detected, module oscillator 24 MHz\n");
	return 0;
}

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar0234 *sensor = to_ar0234(sd);
	struct sensor_info *info = &sensor->info;
	int ret;

	mutex_lock(&info->lock);
	if (sensor->streaming == !!enable) {
		ret = 0;
		goto unlock;
	}
	if (!enable) {
		ret = sensor_write(sd, 0x301A, 0x2058);
		if (!ret)
			sensor->streaming = false;
		goto unlock;
	}
	if (!sensor->powered) {
		ret = -EPIPE;
		goto unlock;
	}
	ret = ar0234_select_interval(sensor);
	if (ret)
		goto unlock;
	ret = sensor_write(sd, 0x301A, 0x2058);
	if (ret)
		goto unlock;
	ret = sensor_write_array(sd, sensor_global_regs, ARRAY_SIZE(sensor_global_regs));
	if (ret)
		goto unlock;
	ret = sensor_write_array(sd, info->current_wins->regs, info->current_wins->regs_size);
	if (ret)
		goto unlock;
	if (info->fmt->regs_size) {
		ret = sensor_write_array(sd, info->fmt->regs, info->fmt->regs_size);
		if (ret)
			goto unlock;
	}
	ret = sensor_write(sd, 0x300A, info->current_wins->vts);
	if (ret)
		goto unlock;
	ret = sensor_write(sd, 0x3070, sensor->test_pattern ? 2 : 0);
	if (ret)
		goto unlock;
	ret = cci_write_a16_d8(sd, 0x301D, 0);
	if (ret)
		goto unlock;
	sensor->streaming = true;
	ret = sensor_apply_exp_gain(sd, info->exp, info->gain);
	if (!ret)
		ret = cci_write_a16_d8(sd, 0x301C, 1);
	if (ret) {
		sensor_write(sd, 0x301A, 0x2058);
		sensor->streaming = false;
	} else {
		v4l2_info(sd, "stream %ux%u code=%x, %u fps, VTS=%u\n",
			info->current_wins->width, info->current_wins->height,
			info->fmt->mbus_code, info->current_wins->fps_fixed,
			info->current_wins->vts);
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
		ret = ar0234_select_interval(to_ar0234(sd));
		if (ret)
			break;
		memcpy(arg, info->current_wins, sizeof(*info->current_wins));
		if (info->fmt->mbus_code == MEDIA_BUS_FMT_SGRBG8_1X8)
			((struct sensor_win_size *)arg)->mipi_bps = 720000000;
		break;
	case VIDIOC_VIN_SENSOR_EXP_GAIN:
		ret = sensor_apply_exp_gain(sd, exp_gain->exp_val, exp_gain->gain_val);
		break;
	case VIDIOC_VIN_SENSOR_CFG_REQ:
		sensor_cfg_req(sd, arg);
		break;
	case VIDIOC_VIN_SENSOR_SET_FPS:
		/* Each advertised mode has a fixed frame length. */
		ret = ((struct sensor_fps *)arg)->fps == info->current_wins->fps_fixed ? 0 : -EINVAL;
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
	struct ar0234 *sensor = container_of(info, struct ar0234, info);
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_apply_exp_gain(&info->sd, info->exp, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_apply_exp_gain(&info->sd, ctrl->val, info->gain);
	case V4L2_CID_TEST_PATTERN:
		if (sensor->streaming) {
			ret = sensor_write(&info->sd, 0x3070, ctrl->val ? 2 : 0);
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

static int ar0234_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state,
			 struct v4l2_subdev_format *fmt)
{
	struct sensor_info *info = to_state(sd);
	struct mbus_framefmt_res *res = (void *)fmt->format.reserved;
	int ret;

	mutex_lock(&info->lock);
	if (!info->tpf.numerator || !info->tpf.denominator) {
		info->tpf.numerator = 1;
		info->tpf.denominator = 30;
	}
	mutex_unlock(&info->lock);
	ret = sensor_set_fmt(sd, state, fmt);

	if (!ret && fmt->format.code == MEDIA_BUS_FMT_SGRBG8_1X8)
		res->res_mipi_bps = 720000000;
	return ret;
}

static int ar0234_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state,
			 struct v4l2_subdev_format *fmt)
{
	int ret = sensor_get_fmt(sd, state, fmt);
	struct mbus_framefmt_res *res = (void *)fmt->format.reserved;

	if (!ret && fmt->format.code == MEDIA_BUS_FMT_SGRBG8_1X8)
		res->res_mipi_bps = 720000000;
	return ret;
}

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code = sensor_enum_mbus_code,
	.enum_frame_size = sensor_enum_frame_size,
	.enum_frame_interval = sensor_enum_frame_interval,
	.get_fmt = ar0234_get_fmt,
	.set_fmt = ar0234_set_fmt,
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
	.data_width = CCI_BITS_16,
};

static const char * const test_patterns[] = { "Disabled", "Color Bars" };

static int sensor_probe(struct i2c_client *client)
{
	struct ar0234 *sensor;
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
	info->current_wins = &sensor_win_sizes[4];
	info->fmt_num = ARRAY_SIZE(sensor_formats);
	info->win_size_num = ARRAY_SIZE(sensor_win_sizes);
	info->width = 1920;
	info->height = 1200;
	info->tpf.numerator = 1;
	info->tpf.denominator = 30;
	info->sensor_field = V4L2_FIELD_NONE;
	info->stream_seq = MIPI_BEFORE_SENSOR;
	info->combo_mode = CMB_TERMINAL_RES | CMB_PHYA_OFFSET2 | MIPI_NORMAL_MODE;
	info->deskew = 0x04;
	info->exp = 1000 << 4;
	info->gain = GAIN_MIN;
#if IS_ENABLED(CONFIG_SAME_I2C)
	info->sensor_i2c_addr = client ? client->addr : 0x10;
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
	struct ar0234 *sensor = to_ar0234(sd);

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
MODULE_DESCRIPTION("onsemi AR0234 RAW10 sensor for Allwinner VIN");
MODULE_LICENSE("GPL");
