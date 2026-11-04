// SPDX-License-Identifier: GPL-2.0
/*
 *  Huawei WMI laptop extras driver
 *
 *  Copyright (C) 2018	      Ayman Bagabas <ayman.bagabas@gmail.com>
 */

#include <linux/acpi.h>
#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dmi.h>
#include <linux/input.h>
#include <linux/input/sparse-keymap.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/sysfs.h>
#include <linux/wmi.h>
#include <linux/hwmon.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/platform_profile.h>
#include <acpi/battery.h>

#define HWMI_BUFF_SIZE 0x100

/*
 * Force a 1-argument kzalloc_obj() expansion so the out-of-tree source can
 * mirror upstream's huawei-wmi.c verbatim. The in-kernel macro signature has
 * changed between releases (mandatory GFP on 6.18 stable, variadic on
 * mainline), so override it unconditionally with the form upstream's call
 * sites assume.
 */
#undef kzalloc_obj
#define kzalloc_obj(P) kzalloc(sizeof(P), GFP_KERNEL)

/*
 * Huawei WMI GUIDs
 */
#define HWMI_METHOD_GUID "ABBC0F5B-8EA1-11D1-A000-C90629100000"
#define HWMI_EVENT_GUID "ABBC0F5C-8EA1-11D1-A000-C90629100000"

/* Legacy GUIDs */
#define WMI0_EXPENSIVE_GUID "39142400-C6A3-40fa-BADB-8A2652834100"
#define WMI0_EVENT_GUID "59142400-C6A3-40fa-BADB-8A2652834100"

/* HWMI commands */

enum {
	BATTERY_THRESH_GET      = 0x00001103, /* \GBTT */
	BATTERY_THRESH_SET      = 0x00001003, /* \SBTT */
	FN_LOCK_GET             = 0x00000604, /* \GFRS */
	FN_LOCK_SET             = 0x00000704, /* \SFRS */
	KBDLIGHT_GET            = 0x00000602, /* \GLIV */
	KBDLIGHT_SET            = 0x00000702, /* \SLIV */
	MICMUTE_LED_SET         = 0x00000b04, /* \SMLS */
	KBDLIGHT_TIMEOUT_SET    = 0x00001106, /* \SKBT */
	KBDLIGHT_TIMEOUT_GET    = 0x00001206, /* \GKBT */
	KBDLIGHT_MODE_GET       = 0x00001306, /* \GKBM */
	KBDLIGHT_MODE_SET       = 0x00001406, /* \SKBM */
	KBDLIGHT_SET_AUTO       = 0x00001506, /* \SKBL */
	PLATFORM_PROFILE_GET    = 0x00000E04, /* \GTUB */
	PLATFORM_PROFILE_SET    = 0x00000F04, /* \STUB */
	FAN_SPEED_GET           = 0x00000802, /* \GFNS */
	TEMP_GET                = 0x00000202, /* \GTMP */
	TOUCHPAD_GET            = 0x00000F02, /* \GTPS */
	TOUCHPAD_SET            = 0x00001002, /* \STPS */
	BATTERY_CHARGE_MODE_GET = 0x00001603, /* \GBCM */
	BATTERY_CHARGE_MODE_SET = 0x00001503, /* \SBCM */
	BATTERY_CHARGE_MODE_PARAM_GET = 0x00001303, /* \GBAC */
	BATTERY_CHARGE_MODE_PARAM_SET = 0x00001203, /* \SBAC */
};

union hwmi_arg {
	u64 cmd;
	u8 args[8];
};

struct quirk_entry {
	bool battery_reset;
	bool ec_micmute;
	bool report_brightness;
	bool report_volume;
	bool handle_kbdlight;
	bool kbdlight_auto;
	bool fan_ec;
	bool platform_profile_hunter;
};

static struct quirk_entry *quirks;

struct huawei_wmi_debug {
	struct dentry *root;
	u64 arg;
};

struct huawei_wmi {
	bool battery_available;
	bool fn_lock_available;
	bool kbdlight_available;
	bool kbdlight_quirk_input;
	bool kbdlight_timeout_available;
	bool fan_speed_available;
	bool temp_available;
	bool smart_charge_available;
	bool smart_charge_param_available;
	struct delayed_work profile_sync_work;
	/* protects last_perf_mode */
	struct mutex profile_sync_lock;
	int last_perf_mode;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 9, 0)
	struct device *platform_profile_dev;
#else
	bool platform_profile_registered;
	struct platform_profile_handler profile_handler;
#endif

	struct huawei_wmi_debug debug;
	struct input_dev *idev[2];
	struct led_classdev micmute_cdev;
	struct led_classdev kbdlight_cdev;
	struct device *dev;

	struct mutex wmi_lock;
};

static struct huawei_wmi *huawei_wmi;

enum {
	KBDLIGHT_KEY_0 = 0x293,
	KBDLIGHT_KEY_1 = 0x294,
	KBDLIGHT_KEY_2 = 0x295,
	KBDLIGHT_KEY_OFF = 0x2b1,
	KBDLIGHT_KEY_LOW = 0x2b2,
	KBDLIGHT_KEY_HIGH = 0x2b3,
	KBDLIGHT_KEY_AUTO = 0x2b4,
};

enum {
	KBDLIGHT_MODE_OFF = 0x02,
	KBDLIGHT_MODE_LOW = 0x03,
	KBDLIGHT_MODE_HIGH = 0x04,
	KBDLIGHT_MODE_AUTO = 0x10,
};

static const struct key_entry huawei_wmi_keymap[] = {
	{ KE_KEY,     0x109,              { KEY_BATTERY } },
	{ KE_KEY,     0x281,              { KEY_BRIGHTNESSDOWN } },
	{ KE_KEY,     0x282,              { KEY_BRIGHTNESSUP } },
	{ KE_KEY,     0x283,              { KEY_TOUCHPAD_ON } },
	{ KE_KEY,     0x2a3,              { KEY_TOUCHPAD_OFF } },
	{ KE_KEY,     0x284,              { KEY_MUTE } },
	{ KE_KEY,     0x285,              { KEY_VOLUMEDOWN } },
	{ KE_KEY,     0x286,              { KEY_VOLUMEUP } },
	{ KE_KEY,     0x287,              { KEY_MICMUTE } },
	{ KE_KEY,     0x288,              { KEY_CAMERA_ACCESS_TOGGLE } },
	{ KE_KEY,     0x289,              { KEY_WLAN } },
	// Huawei |M| key
	{ KE_KEY,     0x28a,              { KEY_CONTROLPANEL } },
	// Sidebar (notifications) key
	{ KE_KEY,     0x28b,              { KEY_NOTIFICATION_CENTER } },
	{ KE_KEY,     0x28e,              { KEY_SELECTIVE_SCREENSHOT } },
	// Keyboard backlight (F-keys)
	{ KE_IGNORE,  KBDLIGHT_KEY_0,     { KEY_KBDILLUMTOGGLE } },
	{ KE_IGNORE,  KBDLIGHT_KEY_1,     { KEY_KBDILLUMDOWN } },
	{ KE_IGNORE,  KBDLIGHT_KEY_2,     { KEY_KBDILLUMUP } },
	// Power unlock (Fn+P)
	{ KE_KEY,     0x2a0,              { KEY_PROG1 } },
	{ KE_KEY,     0x2a1,              { KEY_PROG1 } },
	{ KE_KEY,     0x2a6,              { KEY_PROG1 } },
	// Refresh rate (Fn+R)
	{ KE_KEY,     0x2a7,              { KEY_REFRESH_RATE_TOGGLE } },
	// Keyboard backlight (space bar, toggles in that order)
	{ KE_KEY,     KBDLIGHT_KEY_OFF,   { KEY_KBDILLUMTOGGLE } },
	{ KE_KEY,     KBDLIGHT_KEY_AUTO,  { KEY_KBDILLUMTOGGLE } },
	{ KE_IGNORE,  KBDLIGHT_KEY_LOW,   { KEY_KBDILLUMDOWN } },
	{ KE_IGNORE,  KBDLIGHT_KEY_HIGH,  { KEY_KBDILLUMUP } },
	{ KE_KEY,     0x2b5,              { KEY_VOICECOMMAND } },
	{ KE_KEY,     0x2bc,              { KEY_CAMERA_ACCESS_ENABLE } },
	{ KE_KEY,     0x2bd,              { KEY_CAMERA_ACCESS_DISABLE } },
	// Ignore Ambient Light Sensoring
	{ KE_IGNORE,  0x2c1,              { KEY_RESERVED } },
	{ KE_KEY,     0x2c3,              { KEY_FIND } },
	// Camera module slot
	{ KE_KEY,     0x2e0,              { KEY_CAMERA_ACCESS_ENABLE } },
	{ KE_KEY,     0x2e1,              { KEY_CAMERA_ACCESS_DISABLE } },
	{ KE_KEY,     0x2e2,              { KEY_PROG2 } },
	{ KE_KEY,     0x2e3,              { KEY_PROG3 } },
	{ KE_END, 0 }
};

static int battery_reset = -1;
static int report_brightness = -1;
static int report_volume = -1;
static int handle_kbdlight = -1;
static int kbdlight_auto = -1;

module_param(battery_reset, bint, 0444);
MODULE_PARM_DESC(battery_reset,
		"Reset battery charge values to (0-0) before disabling it using (0-100)");
module_param(report_brightness, bint, 0444);
MODULE_PARM_DESC(report_brightness,
		"Report brightness keys.");
module_param(report_volume, bint, 0444);
MODULE_PARM_DESC(report_volume,
		"Report volume keys.");
module_param(handle_kbdlight, bint, 0444);
MODULE_PARM_DESC(handle_kbdlight,
		"Handle keyboard backlight events.");
module_param(kbdlight_auto, bint, 0444);
MODULE_PARM_DESC(kbdlight_auto,
		"Keyboard backlight supports the auto mode.");

/* Quirks */

static int __init dmi_matched(const struct dmi_system_id *dmi)
{
	quirks = dmi->driver_data;
	return 1;
}

static struct quirk_entry quirk_unknown = {
	.handle_kbdlight = true,
};

static struct quirk_entry quirk_skip_kbdlight = {
	.handle_kbdlight = false,
};

static struct quirk_entry quirk_kbdlight_auto = {
	.handle_kbdlight = false,
	.kbdlight_auto = true,
};

static struct quirk_entry quirk_honor_zqcp_fan_ec = {
	.handle_kbdlight = true,
	.fan_ec = true,
};

static struct quirk_entry quirk_honor_hunter = {
	.handle_kbdlight = true,
	.platform_profile_hunter = true,
};

static struct quirk_entry quirk_mach_wx9 = {
	.battery_reset = true,
	.handle_kbdlight = false,
};

static struct quirk_entry quirk_matebook_x = {
	.ec_micmute = true,
	.report_brightness = true,
	.handle_kbdlight = false,
};

static struct quirk_entry quirk_matebook_d = {
	.report_brightness = false,
	.report_volume = false,
	.handle_kbdlight = false,
};


static const struct dmi_system_id huawei_quirks[] = {
	{
		.callback = dmi_matched,
		.ident = "Huawei MACH-WX9",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "HUAWEI"),
			DMI_MATCH(DMI_PRODUCT_NAME, "MACH-WX9"),
		},
		.driver_data = &quirk_mach_wx9
	},
	{
		.callback = dmi_matched,
		.ident = "Huawei MateBook X",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "HUAWEI"),
			DMI_MATCH(DMI_PRODUCT_NAME, "HUAWEI MateBook X")
		},
		.driver_data = &quirk_matebook_x
	},
	{
		.callback = dmi_matched,
		.ident = "Huawei MCLF-XX",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "HUAWEI"),
			DMI_MATCH(DMI_PRODUCT_NAME, "MCLF-XX")
		},
		.driver_data = &quirk_matebook_d
	},
	{
		.callback = dmi_matched,
		.ident = "Huawei KPL-W0X",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "HUAWEI"),
			DMI_MATCH(DMI_PRODUCT_NAME, "KPL-W0X")
		},
		.driver_data = &quirk_skip_kbdlight
	},
	{
		.callback = dmi_matched,
		.ident = "Huawei MACHC-WAX9",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "HUAWEI"),
			DMI_MATCH(DMI_PRODUCT_NAME, "MACHC-WAX9")
		},
		.driver_data = &quirk_unknown
	},
	{
		.callback = dmi_matched,
		.ident = "Huawei NBLK-WAX9X",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "HUAWEI"),
			DMI_MATCH(DMI_PRODUCT_NAME, "NBLK-WAX9X")
		},
		.driver_data = &quirk_skip_kbdlight
	},
	{
		.callback = dmi_matched,
		.ident = "Huawei HLYL-WXX9",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "HUAWEI"),
			DMI_MATCH(DMI_PRODUCT_NAME, "HLYL-WXX9")
		},
		.driver_data = &quirk_skip_kbdlight
	},
	{
		.callback = dmi_matched,
		.ident = "Honor MRA-XXX",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "HONOR"),
			DMI_MATCH(DMI_PRODUCT_NAME, "MRA-XXX")
		},
		.driver_data = &quirk_kbdlight_auto
	},
	{
		.callback = dmi_matched,
		.ident = "HONOR ZQC-P M1010 EC fan tachometers",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "HONOR"),
			DMI_MATCH(DMI_PRODUCT_NAME, "ZQC-P"),
			DMI_MATCH(DMI_BOARD_NAME, "ZQC-P-PCB"),
			DMI_MATCH(DMI_BOARD_VERSION, "M1010")
		},
		.driver_data = &quirk_honor_zqcp_fan_ec
	},
	{
		.callback = dmi_matched,
		.ident = "HONOR ZQC-P M1050 EC fan tachometers",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "HONOR"),
			DMI_MATCH(DMI_PRODUCT_NAME, "ZQC-P"),
			DMI_MATCH(DMI_BOARD_NAME, "ZQC-P-PCB"),
			DMI_MATCH(DMI_BOARD_VERSION, "M1050")
		},
		.driver_data = &quirk_honor_zqcp_fan_ec
	},
	{
		.callback = dmi_matched,
		.ident = "HONOR DRA-XX HUNTER M1030",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "HONOR"),
			DMI_MATCH(DMI_PRODUCT_NAME, "DRA-XX"),
			DMI_MATCH(DMI_BOARD_NAME, "DRA-XX-PCB"),
			DMI_MATCH(DMI_BOARD_VERSION, "M1030")
		},
		.driver_data = &quirk_honor_hunter
	},
	{
		.callback = dmi_matched,
		.ident = "HONOR DRA-XX HUNTER M1040",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "HONOR"),
			DMI_MATCH(DMI_PRODUCT_NAME, "DRA-XX"),
			DMI_MATCH(DMI_BOARD_NAME, "DRA-XX-PCB"),
			DMI_MATCH(DMI_BOARD_VERSION, "M1040")
		},
		.driver_data = &quirk_honor_hunter
	},
	{
		.callback = dmi_matched,
		.ident = "HONOR DRB-P HUNTER M1020",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "HONOR"),
			DMI_MATCH(DMI_PRODUCT_NAME, "DRB-P"),
			DMI_MATCH(DMI_BOARD_NAME, "DRB-P-PCB"),
			DMI_MATCH(DMI_BOARD_VERSION, "M1020")
		},
		.driver_data = &quirk_honor_hunter
	},
	{
		.callback = dmi_matched,
		.ident = "HONOR DRB-P HUNTER M1100",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "HONOR"),
			DMI_MATCH(DMI_PRODUCT_NAME, "DRB-P"),
			DMI_MATCH(DMI_BOARD_NAME, "DRB-P-PCB"),
			DMI_MATCH(DMI_BOARD_VERSION, "M1100")
		},
		.driver_data = &quirk_honor_hunter
	},
	{  }
};

/* Utils */

static int huawei_wmi_call(struct huawei_wmi *huawei,
			   struct acpi_buffer *in, struct acpi_buffer *out)
{
	acpi_status status;

	mutex_lock(&huawei->wmi_lock);
	status = wmi_evaluate_method(HWMI_METHOD_GUID, 0, 1, in, out);
	mutex_unlock(&huawei->wmi_lock);
	if (ACPI_FAILURE(status)) {
		dev_err(huawei->dev, "Failed to evaluate wmi method\n");
		return -ENODEV;
	}

	return 0;
}

/* HWMI takes a 64 bit input and returns either a package with 2 buffers, one of
 * 4 bytes and the other of 256 bytes, or one buffer of size 0x104 (260) bytes.
 * The first 4 bytes are ignored, we ignore the first 4 bytes buffer if we got a
 * package, or skip the first 4 if a buffer of 0x104 is used. The first byte of
 * the remaining 0x100 sized buffer has the return status of every call. In case
 * the return status is non-zero, we return -ENODEV but still copy the returned
 * buffer to the given buffer parameter (buf).
 */
static int huawei_wmi_cmd(u64 arg, u8 *buf, size_t buflen)
{
	struct huawei_wmi *huawei = huawei_wmi;
	struct acpi_buffer out = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_buffer in;
	union acpi_object *obj;
	size_t len;
	int err, i;

	in.length = sizeof(arg);
	in.pointer = &arg;

	/* Some models require calling HWMI twice to execute a command. We evaluate
	 * HWMI and if we get a non-zero return status we evaluate it again.
	 */
	for (i = 0; i < 2; i++) {
		err = huawei_wmi_call(huawei, &in, &out);
		if (err)
			goto fail_cmd;

		obj = out.pointer;
		if (!obj) {
			err = -EIO;
			goto fail_cmd;
		}

		switch (obj->type) {
		/* Models that implement both "legacy" and HWMI tend to return a 0x104
		 * sized buffer instead of a package of 0x4 and 0x100 buffers.
		 */
		case ACPI_TYPE_BUFFER:
			if (obj->buffer.length == 0x104) {
				// Skip the first 4 bytes.
				obj->buffer.pointer += 4;
				len = HWMI_BUFF_SIZE;
			} else {
				dev_err(huawei->dev, "Bad buffer length, got %d\n", obj->buffer.length);
				err = -EIO;
				goto fail_cmd;
			}

			break;
		/* HWMI returns a package with 2 buffer elements, one of 4 bytes and the
		 * other is 256 bytes.
		 */
		case ACPI_TYPE_PACKAGE:
			if (obj->package.count != 2) {
				dev_err(huawei->dev, "Bad package count, got %d\n", obj->package.count);
				err = -EIO;
				goto fail_cmd;
			}

			obj = &obj->package.elements[1];
			if (obj->type != ACPI_TYPE_BUFFER) {
				dev_err(huawei->dev, "Bad package element type, got %d\n", obj->type);
				err = -EIO;
				goto fail_cmd;
			}
			len = obj->buffer.length;

			break;
		/* Shouldn't get here! */
		default:
			dev_err(huawei->dev, "Unexpected obj type, got: %d\n", obj->type);
			err = -EIO;
			goto fail_cmd;
		}

		if (!*obj->buffer.pointer)
			break;
	}

	err = (*obj->buffer.pointer) ? -ENODEV : 0;

	if (buf) {
		len = min(buflen, len);
		memcpy(buf, obj->buffer.pointer, len);
	}

fail_cmd:
	kfree(out.pointer);
	return err;
}

/* LEDs */

static int huawei_wmi_micmute_led_set(struct led_classdev *led_cdev,
		enum led_brightness brightness)
{
	/* This is a workaround until the "legacy" interface is implemented. */
	if (quirks && quirks->ec_micmute) {
		char *acpi_method;
		acpi_handle handle;
		acpi_status status;
		union acpi_object args[3];
		struct acpi_object_list arg_list = {
			.pointer = args,
			.count = ARRAY_SIZE(args),
		};

		handle = ec_get_handle();
		if (!handle)
			return -ENODEV;

		args[0].type = args[1].type = args[2].type = ACPI_TYPE_INTEGER;
		args[1].integer.value = 0x04;

		if (acpi_has_method(handle, "SPIN")) {
			acpi_method = "SPIN";
			args[0].integer.value = 0;
			args[2].integer.value = brightness ? 1 : 0;
		} else if (acpi_has_method(handle, "WPIN")) {
			acpi_method = "WPIN";
			args[0].integer.value = 1;
			args[2].integer.value = brightness ? 0 : 1;
		} else {
			return -ENODEV;
		}

		status = acpi_evaluate_object(handle, acpi_method, &arg_list, NULL);
		if (ACPI_FAILURE(status))
			return -ENODEV;

		return 0;
	} else {
		union hwmi_arg arg;

		arg.cmd = MICMUTE_LED_SET;
		arg.args[2] = brightness;

		return huawei_wmi_cmd(arg.cmd, NULL, 0);
	}
}

static int huawei_wmi_kbdlight_set_auto(int level);

static int huawei_wmi_kbdlight_led_set(struct led_classdev *led_cdev,
		enum led_brightness brightness)
{
	return huawei_wmi_kbdlight_set_auto(brightness);
}

static void huawei_wmi_leds_setup(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	huawei->micmute_cdev.name = "huawei::micmute";
	huawei->micmute_cdev.max_brightness = 1;
	huawei->micmute_cdev.brightness_set_blocking = &huawei_wmi_micmute_led_set;
	huawei->micmute_cdev.default_trigger = "audio-micmute";
	huawei->micmute_cdev.dev = dev;
	huawei->micmute_cdev.flags = LED_CORE_SUSPENDRESUME;

	if (acpi_has_method(NULL, "\\SMLS") || (quirks && quirks->ec_micmute))
		devm_led_classdev_register(dev, &huawei->micmute_cdev);

	huawei->kbdlight_cdev.name = "huawei::kbd_backlight";
	huawei->kbdlight_cdev.max_brightness = 100;
	huawei->kbdlight_cdev.brightness_set_blocking = &huawei_wmi_kbdlight_led_set;
	huawei->kbdlight_cdev.dev = dev;

	if (acpi_has_method(NULL, "\\SKBL") || (quirks && quirks->kbdlight_auto))
		devm_led_classdev_register(dev, &huawei->kbdlight_cdev);
}

/* Battery protection */

static int huawei_wmi_battery_get(int *start, int *end)
{
	u8 ret[HWMI_BUFF_SIZE];
	int err, i;

	err = huawei_wmi_cmd(BATTERY_THRESH_GET, ret, HWMI_BUFF_SIZE);
	if (err)
		return err;

	/* Find the last two non-zero values. Return status is ignored. */
	i = ARRAY_SIZE(ret) - 1;
	do {
		if (start)
			*start = ret[i-1];
		if (end)
			*end = ret[i];
	} while (i > 2 && !ret[i--]);

	return 0;
}

static int huawei_wmi_battery_set(int start, int end)
{
	union hwmi_arg arg;
	int err;

	if (start < 0 || end < 0 || start > 100 || end > 100)
		return -EINVAL;

	arg.cmd = BATTERY_THRESH_SET;
	arg.args[2] = start;
	arg.args[3] = end;

	/* This is an edge case were some models turn battery protection
	 * off without changing their thresholds values. We clear the
	 * values before turning off protection. Sometimes we need a sleep delay to
	 * make sure these values make their way to EC memory.
	 */
	if (quirks && quirks->battery_reset && start == 0 && end == 100) {
		err = huawei_wmi_battery_set(0, 0);
		if (err)
			return err;

		msleep(1000);
	}

	err = huawei_wmi_cmd(arg.cmd, NULL, 0);

	return err;
}

static ssize_t charge_control_start_threshold_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err, start;

	err = huawei_wmi_battery_get(&start, NULL);
	if (err)
		return err;

	return sysfs_emit(buf, "%d\n", start);
}

static ssize_t charge_control_end_threshold_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err, end;

	err = huawei_wmi_battery_get(NULL, &end);
	if (err)
		return err;

	return sysfs_emit(buf, "%d\n", end);
}

static ssize_t charge_control_thresholds_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err, start, end;

	err = huawei_wmi_battery_get(&start, &end);
	if (err)
		return err;

	return sysfs_emit(buf, "%d %d\n", start, end);
}

static ssize_t charge_control_start_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err, start, end;

	err = huawei_wmi_battery_get(NULL, &end);
	if (err)
		return err;

	if (sscanf(buf, "%d", &start) != 1)
		return -EINVAL;

	err = huawei_wmi_battery_set(start, end);
	if (err)
		return err;

	return size;
}

static ssize_t charge_control_end_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err, start, end;

	err = huawei_wmi_battery_get(&start, NULL);
	if (err)
		return err;

	if (sscanf(buf, "%d", &end) != 1)
		return -EINVAL;

	err = huawei_wmi_battery_set(start, end);
	if (err)
		return err;

	return size;
}

static ssize_t charge_control_thresholds_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err, start, end;

	if (sscanf(buf, "%d %d", &start, &end) != 2)
		return -EINVAL;

	err = huawei_wmi_battery_set(start, end);
	if (err)
		return err;

	return size;
}

static DEVICE_ATTR_RW(charge_control_start_threshold);
static DEVICE_ATTR_RW(charge_control_end_threshold);
static DEVICE_ATTR_RW(charge_control_thresholds);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
static int huawei_wmi_battery_add(struct power_supply *battery, struct acpi_battery_hook *hook)
#else
static int huawei_wmi_battery_add(struct power_supply *battery)
#endif
{
	int err = 0;

	err = device_create_file(&battery->dev, &dev_attr_charge_control_start_threshold);
	if (err)
		return err;

	err = device_create_file(&battery->dev, &dev_attr_charge_control_end_threshold);
	if (err)
		device_remove_file(&battery->dev, &dev_attr_charge_control_start_threshold);

	return err;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
static int huawei_wmi_battery_remove(struct power_supply *battery, struct acpi_battery_hook *hook)
#else
static int huawei_wmi_battery_remove(struct power_supply *battery)
#endif
{
	device_remove_file(&battery->dev, &dev_attr_charge_control_start_threshold);
	device_remove_file(&battery->dev, &dev_attr_charge_control_end_threshold);

	return 0;
}

static struct acpi_battery_hook huawei_wmi_battery_hook = {
	.add_battery = huawei_wmi_battery_add,
	.remove_battery = huawei_wmi_battery_remove,
	.name = "Huawei Battery Extension"
};

static void huawei_wmi_battery_setup(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	huawei->battery_available = true;
	if (huawei_wmi_battery_get(NULL, NULL)) {
		huawei->battery_available = false;
		return;
	}

	battery_hook_register(&huawei_wmi_battery_hook);
	device_create_file(dev, &dev_attr_charge_control_thresholds);
}

static void huawei_wmi_battery_exit(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	if (huawei->battery_available) {
		battery_hook_unregister(&huawei_wmi_battery_hook);
		device_remove_file(dev, &dev_attr_charge_control_thresholds);
	}
}

/* Smart charge param*/

static int huawei_wmi_smart_charge_param_get(int *value)
{
	u8 ret[HWMI_BUFF_SIZE];
	int err;

	err = huawei_wmi_cmd(BATTERY_CHARGE_MODE_PARAM_GET, ret, HWMI_BUFF_SIZE);
	if (err)
		return err;

	if (value)
		*value = ret[1];

	return 0;
}

static int huawei_wmi_smart_charge_param_set(int value)
{
	union hwmi_arg arg;
	int err;

	if (value < 0 || value > 2)
		return -EINVAL;

	arg.cmd = BATTERY_CHARGE_MODE_PARAM_SET;
	arg.args[2] = (u8) value;

	err = huawei_wmi_cmd(arg.cmd, NULL, 0);
	return err;
}

static ssize_t smart_charge_param_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err, value;

	err = huawei_wmi_smart_charge_param_get(&value);
	if (err)
		return err;

	return sysfs_emit(buf, "%d\n", value);
}

static ssize_t smart_charge_param_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err, value;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	err = huawei_wmi_smart_charge_param_set(value);
	if (err)
		return err;

	return size;
}

static DEVICE_ATTR_RW(smart_charge_param);

static void huawei_wmi_smart_charge_param_setup(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	huawei->smart_charge_param_available = true;
	if (huawei_wmi_smart_charge_param_get(NULL)) {
		huawei->smart_charge_param_available = false;
		return;
	}

	device_create_file(dev, &dev_attr_smart_charge_param);
}

static void huawei_wmi_smart_charge_param_exit(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	if (huawei->smart_charge_param_available) {
		device_remove_file(dev, &dev_attr_smart_charge_param);
	}
}

/* Smart charge */

static int huawei_wmi_smart_charge_get(int *mode, int *unknow, int *start, int *end)
{
	u8 ret[HWMI_BUFF_SIZE];
	int err;

	err = huawei_wmi_cmd(BATTERY_CHARGE_MODE_GET, ret, HWMI_BUFF_SIZE);
	if (err)
		return err;

	if (mode)
		*mode = ret[1];
	if (unknow)
		*unknow = ret[2];
	if (start)
		*start = ret[3];
	if (end)
		*end = ret[4];

	return 0;
}

static int huawei_wmi_smart_charge_set(int mode, int unknow, int start, int end)
{
	union hwmi_arg arg;
	int err;

	if (start < 0 || end < 0 || start > 100 || end > 100 ||
			mode < 0 || mode > 0xff || unknow < 0 || unknow > 0xff)
		return -EINVAL;

	arg.cmd = BATTERY_CHARGE_MODE_SET;
	arg.args[2] = (u8) mode;
	arg.args[3] = (u8) unknow;
	arg.args[4] = (u8) start;
	arg.args[5] = (u8) end;

	err = huawei_wmi_cmd(arg.cmd, NULL, 0);
	return err;
}

static ssize_t smart_charge_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err, start, end, mode, unknow;

	err = huawei_wmi_smart_charge_get(&mode, &unknow, &start, &end);
	if (err)
		return err;

	return sysfs_emit(buf, "%d %d %d %d\n", mode, unknow, start, end);
}

static ssize_t smart_charge_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err, start, end, mode, unknow;

	if (sscanf(buf, "%d %d %d %d", &mode, &unknow, &start, &end) != 4)
		return -EINVAL;

	err = huawei_wmi_smart_charge_set(mode, unknow, start, end);
	if (err)
		return err;

	return size;
}

static DEVICE_ATTR_RW(smart_charge);

static void huawei_wmi_smart_charge_setup(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	huawei->smart_charge_available = true;
	if (huawei_wmi_smart_charge_get(NULL, NULL, NULL, NULL)) {
		huawei->smart_charge_available = false;
		return;
	}

	device_create_file(dev, &dev_attr_smart_charge);
}

static void huawei_wmi_smart_charge_exit(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	if (huawei->smart_charge_available) {
		device_remove_file(dev, &dev_attr_smart_charge);
	}
}

/* Fn lock */

/* GFRS byte[1] / SFRS byte[2] (FRSR) fn-lock state values */
#define FN_LOCK_ACPI_OFF	1
#define FN_LOCK_ACPI_ON		2
#define FN_LOCK_ACPI_STAT_OK	0

/*
 * Newer Huawei models (e.g. HUAWEI FLMH-XX / MateBook 14 2024) use direct
 * ACPI methods \GFRS / \SFRS (Get/Set Fn key Reversal Status) to control
 * Fn-lock via EC registers 0x6B (read) and 0x6C (write).
 *
 * GFRS response buffer layout:
 *   byte[0] = STAT (FN_LOCK_ACPI_STAT_OK = success)
 *   byte[1] = FN_LOCK_ACPI_OFF (fn-lock off) or FN_LOCK_ACPI_ON (fn-lock on)
 *
 * SFRS argument layout (CreateByteField(Arg0, 0x02, FRSR)):
 *   Value is read from byte[2] of the integer argument, so it must be
 *   passed as (value << 16):
 *   (FN_LOCK_ACPI_OFF << 16) = fn-lock off (writes 0x55 to EC 0x6C)
 *   (FN_LOCK_ACPI_ON  << 16) = fn-lock on  (writes 0x5A to EC 0x6C)
 */

static int huawei_acpi_fn_lock_get(int *on)
{
	union acpi_object acpi_arg;
	struct acpi_object_list arg_list = { .count = 1, .pointer = &acpi_arg };
	struct acpi_buffer output = { ACPI_ALLOCATE_BUFFER, NULL };
	acpi_status status;

	acpi_arg.type = ACPI_TYPE_INTEGER;
	acpi_arg.integer.value = 0;

	status = acpi_evaluate_object(NULL, "\\GFRS", &arg_list, &output);
	if (ACPI_FAILURE(status))
		return -EIO;

	union acpi_object *obj __free(kfree) = output.pointer;

	if (!obj || obj->type != ACPI_TYPE_BUFFER || obj->buffer.length < 2)
		return -ENODATA;

	/* byte[0] = STAT, byte[1] = fn-lock state */
	if (obj->buffer.pointer[0] != FN_LOCK_ACPI_STAT_OK)
		return -EIO;

	switch (obj->buffer.pointer[1]) {
	case FN_LOCK_ACPI_OFF:
		if (on)
			*on = 0;
		break;
	case FN_LOCK_ACPI_ON:
		if (on)
			*on = 1;
		break;
	default:
		return -ENODATA;
	}

	return 0;
}

static int huawei_acpi_fn_lock_set(int on)
{
	union acpi_object acpi_arg;
	struct acpi_object_list arg_list = { .count = 1, .pointer = &acpi_arg };
	struct acpi_buffer output = { ACPI_ALLOCATE_BUFFER, NULL };
	acpi_status status;

	/*
	 * SFRS reads byte[2] of its argument via CreateByteField(Arg0, 0x02).
	 * on=0 → FRSR=FN_LOCK_ACPI_OFF → EC gets 0x55 (fn-lock off)
	 * on=1 → FRSR=FN_LOCK_ACPI_ON  → EC gets 0x5A (fn-lock on)
	 */
	acpi_arg.type = ACPI_TYPE_INTEGER;
	acpi_arg.integer.value = (on ? FN_LOCK_ACPI_ON : FN_LOCK_ACPI_OFF) << 16;

	status = acpi_evaluate_object(NULL, "\\SFRS", &arg_list, &output);
	if (ACPI_FAILURE(status))
		return -EIO;

	union acpi_object *obj __free(kfree) = output.pointer;

	if (!obj || obj->type != ACPI_TYPE_BUFFER || obj->buffer.length < 1)
		return -ENODATA;

	if (obj->buffer.pointer[0] != FN_LOCK_ACPI_STAT_OK)
		return -EIO;

	return 0;
}

static int huawei_wmi_fn_lock_get(int *on)
{
	u8 ret[HWMI_BUFF_SIZE] = { 0 };
	int err, i;

	/* Newer models: use direct ACPI \GFRS method */
	if (acpi_has_method(NULL, "\\GFRS"))
		return huawei_acpi_fn_lock_get(on);

	/* Legacy WMI fallback */
	err = huawei_wmi_cmd(FN_LOCK_GET, ret, HWMI_BUFF_SIZE);
	if (err)
		return err;

	/* Find the first non-zero value. Return status is ignored. */
	i = 1;
	do {
		if (on)
			*on = ret[i] - 1; // -1 undefined, 0 off, 1 on.
	} while (i < 0xff && !ret[i++]);

	return 0;
}

static int huawei_wmi_fn_lock_set(int on)
{
	union hwmi_arg arg;

	/* Newer models: use direct ACPI \SFRS method */
	if (acpi_has_method(NULL, "\\SFRS"))
		return huawei_acpi_fn_lock_set(on);

	/* Legacy WMI fallback */
	arg.cmd = FN_LOCK_SET;
	arg.args[2] = on + 1; // 0 undefined, 1 off, 2 on.

	return huawei_wmi_cmd(arg.cmd, NULL, 0);
}

static ssize_t fn_lock_state_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err, on;

	err = huawei_wmi_fn_lock_get(&on);
	if (err)
		return err;

	return sysfs_emit(buf, "%d\n", on);
}

static ssize_t fn_lock_state_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int on, err;

	if (kstrtoint(buf, 10, &on) ||
			on < 0 || on > 1)
		return -EINVAL;

	err = huawei_wmi_fn_lock_set(on);
	if (err)
		return err;

	return size;
}

static DEVICE_ATTR_RW(fn_lock_state);

static void huawei_wmi_fn_lock_setup(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	huawei->fn_lock_available = true;
	if (huawei_wmi_fn_lock_get(NULL)) {
		huawei->fn_lock_available = false;
		return;
	}

	device_create_file(dev, &dev_attr_fn_lock_state);
}

static void huawei_wmi_fn_lock_exit(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	if (huawei->fn_lock_available)
		device_remove_file(dev, &dev_attr_fn_lock_state);
}

/* Keyboard backlight */

static int huawei_wmi_kbdlight_get(int *level)
{
	u8 ret[HWMI_BUFF_SIZE] = { 0 };
	int err;

	err = huawei_wmi_cmd(KBDLIGHT_GET, ret, HWMI_BUFF_SIZE);
	if (err)
		return err;
	if (!ret[2])
		return -ENODEV;

	/* Some models like the MACH-WX9 use 0x01, 0x02, and 0x04 for off, level 1,
	 * and level 2 respectively rather than 0x04, 0x08, and 0x10.
	 */
	huawei_wmi->kbdlight_quirk_input = ret[1] == 0xff;

	if (level) {
		*level = 0;
		while (ret[2] >>= 1) {
			*level += 1;
		}

		if (ret[1] != 0xff)
			*level -= 2;
	}

	return 0;
}

static int huawei_wmi_kbdlight_set(int level)
{
	union hwmi_arg arg;

	// Huawei laptops only support 3 kbdlight levels
	if (level < 0 || level > 2)
		return -EINVAL;
	if (!huawei_wmi->kbdlight_quirk_input)
		level += 2;

	arg.cmd = KBDLIGHT_SET;
	arg.args[2] = 1 << level;

	return huawei_wmi_cmd(arg.cmd, NULL, 0);
}

static int huawei_wmi_kbdlight_set_auto(int level)
{
	union hwmi_arg arg;
	int err;

	if (level < 0 || level > 255)
		return -EINVAL;

	arg.cmd = KBDLIGHT_MODE_SET;
	arg.args[2] = KBDLIGHT_MODE_AUTO;
	err = huawei_wmi_cmd(arg.cmd, NULL, 0);
	if (err)
		return err;

	msleep(10);

	arg.cmd = KBDLIGHT_SET_AUTO;
	arg.args[2] = level;

	return huawei_wmi_cmd(arg.cmd, NULL, 0);
}

static ssize_t kbdlight_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err, level;

	err = huawei_wmi_kbdlight_get(&level);
	if (err)
		return err;

	return sysfs_emit(buf, "%d\n", level);
}

static ssize_t kbdlight_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int level, err;

	if (kstrtoint(buf, 10, &level))
		return -EINVAL;

	if (quirks && quirks->kbdlight_auto)
		err = huawei_wmi_kbdlight_set_auto(level);
	else
		err = huawei_wmi_kbdlight_set(level);
	if (err)
		return err;

	return size;
}

static DEVICE_ATTR_RW(kbdlight);

static void huawei_wmi_kbdlight_setup(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	huawei->kbdlight_available = true;
	if (!(acpi_has_method(NULL, "\\SKBL") || (quirks && quirks->kbdlight_auto))
	    && huawei_wmi_kbdlight_get(NULL)) {
		huawei->kbdlight_available = false;
		return;
	}

	device_create_file(dev, &dev_attr_kbdlight);
}

static void huawei_wmi_kbdlight_exit(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	if (huawei->kbdlight_available)
		device_remove_file(dev, &dev_attr_kbdlight);
}

/* Keyboard backlight timeout */

static int huawei_wmi_kbdlight_timeout_get(int *seconds)
{
	u8 ret[HWMI_BUFF_SIZE] = { 0 };
	int err;

	err = huawei_wmi_cmd(KBDLIGHT_TIMEOUT_GET, ret, HWMI_BUFF_SIZE);
	if (err)
		return err;

	if (seconds)
		*seconds = ret[1] | (ret[2] << 8);

	return 0;
}

static int huawei_wmi_kbdlight_timeout_set(int seconds)
{
	union hwmi_arg arg;

	arg.cmd = KBDLIGHT_TIMEOUT_SET;
	arg.args[2] = (seconds & 0xff);
	arg.args[3] = (seconds >> 8);

	return huawei_wmi_cmd(arg.cmd, NULL, 0);

}

static ssize_t kbdlight_timeout_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err, seconds;

	err = huawei_wmi_kbdlight_timeout_get(&seconds);
	if (err)
		return err;

	return sysfs_emit(buf, "%d\n", seconds);
}

static ssize_t kbdlight_timeout_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int seconds, err;

	if (kstrtoint(buf, 10, &seconds) ||
			seconds < 0 || seconds > 0xffff)
		return -EINVAL;

	err = huawei_wmi_kbdlight_timeout_set(seconds);
	if (err)
		return err;

	return size;
}

static DEVICE_ATTR_RW(kbdlight_timeout);

static void huawei_wmi_kbdlight_timeout_setup(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);
	huawei->kbdlight_timeout_available = true;
	if (huawei_wmi_kbdlight_timeout_get(NULL)) {
		huawei->kbdlight_timeout_available = false;
		return;
	}

	device_create_file(dev, &dev_attr_kbdlight_timeout);
}

static void huawei_wmi_kbdlight_timeout_exit(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	if (huawei->kbdlight_timeout_available)
		device_remove_file(dev, &dev_attr_kbdlight_timeout);
}

/* Platform profile */

/*
 * Performance modes reported by GTUB and accepted by STUB. HUNTER is only
 * present on gaming models; the firmware of other models rejects it.
 */
#define HWMI_PERF_MODE_SMART		0
#define HWMI_PERF_MODE_PERFORMANCE	1
#define HWMI_PERF_MODE_HUNTER		3

/*
 * WMI events reporting performance mode changes. The firmware derives
 * these from the EC mode state, so they also arrive when the mode was
 * changed by the firmware itself rather than by a hotkey. 0x2a0/0x2a1/0x2a8
 * carry the resulting mode, 0x2a5/0x2a6 are toggle requests without state.
 */
#define HWMI_EVENT_MODE_SMART		0x2a0
#define HWMI_EVENT_MODE_PERFORMANCE	0x2a1
#define HWMI_EVENT_MODE_TOGGLE_ALT	0x2a5
#define HWMI_EVENT_MODE_TOGGLE		0x2a6
#define HWMI_EVENT_MODE_HUNTER		0x2a8

#define HWMI_MODE_EVENT_DEBOUNCE_MS	500

static bool huawei_wmi_platform_profile_active(struct huawei_wmi *huawei)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 9, 0)
	return huawei->platform_profile_dev;
#else
	return huawei->platform_profile_registered;
#endif
}

static int huawei_wmi_perf_mode_get(int *mode)
{
	u8 ret[0x100] = { 0 };
	int err;

	err = huawei_wmi_cmd(PLATFORM_PROFILE_GET, ret, sizeof(ret));
	if (err)
		return err;

	*mode = ret[1];

	return 0;
}

static int huawei_wmi_perf_mode_set(int mode)
{
	union hwmi_arg arg;

	arg.cmd = PLATFORM_PROFILE_SET;
	arg.args[2] = mode;

	return huawei_wmi_cmd(arg.cmd, NULL, 0);
}

/*
 * Write the requested mode and read back what the firmware applied. The
 * firmware accepts mode changes it cannot currently sustain (for example
 * HUNTER requires the original power adapter) and then leaves or reverts
 * the mode on its own. When the read-back does not match the request,
 * restore the previous mode and report -EBUSY so userspace is never told
 * a mode was applied that the firmware does not actually run.
 */
static int huawei_wmi_perf_mode_set_checked(int mode)
{
	int prev, actual, err;

	err = huawei_wmi_perf_mode_get(&prev);
	if (err)
		return err;

	err = huawei_wmi_perf_mode_set(mode);
	if (err)
		return err;

	err = huawei_wmi_perf_mode_get(&actual);
	if (err) {
		/*
		 * The write may have been applied but cannot be verified.
		 * Best-effort restore of the previous mode so the firmware
		 * state stays known to the driver.
		 */
		if (huawei_wmi_perf_mode_set(prev))
			dev_warn(huawei_wmi->dev,
				 "Failed to restore performance mode %d after a read-back failure\n",
				 prev);

		return err;
	}

	if (actual == mode)
		return 0;

	dev_dbg(huawei_wmi->dev,
		"Firmware did not apply mode %d (reported %d), restoring mode %d\n",
		mode, actual, prev);

	err = huawei_wmi_perf_mode_set(prev);
	if (err)
		dev_warn(huawei_wmi->dev,
			 "Failed to restore performance mode %d: %d\n",
			 prev, err);

	return -EBUSY;
}

/*
 * The firmware "Smart" and "High performance" modes are exposed as the
 * low-power and balanced profiles; HUNTER, where present, is exposed as
 * the performance profile.
 */
static int huawei_wmi_platform_profile_from_mode(int mode,
		enum platform_profile_option *profile)
{
	switch (mode) {
	case HWMI_PERF_MODE_SMART:
		*profile = PLATFORM_PROFILE_LOW_POWER;
		return 0;
	case HWMI_PERF_MODE_PERFORMANCE:
		*profile = PLATFORM_PROFILE_BALANCED;
		return 0;
	case HWMI_PERF_MODE_HUNTER:
		if (!quirks->platform_profile_hunter)
			return -EINVAL;
		*profile = PLATFORM_PROFILE_PERFORMANCE;
		return 0;
	default:
		return -EINVAL;
	}
}

static int
huawei_wmi_platform_profile_to_mode(enum platform_profile_option profile,
				    int *mode)
{
	switch (profile) {
	case PLATFORM_PROFILE_LOW_POWER:
		*mode = HWMI_PERF_MODE_SMART;
		return 0;
	case PLATFORM_PROFILE_BALANCED:
		*mode = HWMI_PERF_MODE_PERFORMANCE;
		return 0;
	case PLATFORM_PROFILE_PERFORMANCE:
		if (!quirks->platform_profile_hunter)
			return -EOPNOTSUPP;
		*mode = HWMI_PERF_MODE_HUNTER;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 9, 0)
static int huawei_wmi_platform_profile_probe(void *drvdata,
		unsigned long *choices)
{
	set_bit(PLATFORM_PROFILE_LOW_POWER, choices);
	set_bit(PLATFORM_PROFILE_BALANCED, choices);
	if (quirks->platform_profile_hunter)
		set_bit(PLATFORM_PROFILE_PERFORMANCE, choices);

	return 0;
}

static int huawei_wmi_platform_profile_get(struct device *dev,
		enum platform_profile_option *profile)
{
	int mode, err;

	err = huawei_wmi_perf_mode_get(&mode);
	if (err)
		return err;

	return huawei_wmi_platform_profile_from_mode(mode, profile);
}

static int huawei_wmi_platform_profile_set(struct device *dev,
		enum platform_profile_option profile)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);
	int mode, err;

	err = huawei_wmi_platform_profile_to_mode(profile, &mode);
	if (err)
		return err;

	/*
	 * Serialize the whole write/read-back/restore transaction and the
	 * cache update against the sync workers.
	 */
	mutex_lock(&huawei->profile_sync_lock);
	err = huawei_wmi_perf_mode_set_checked(mode);
	if (!err)
		huawei->last_perf_mode = mode;
	mutex_unlock(&huawei->profile_sync_lock);

	/*
	 * A later firmware-side revert, e.g. of HUNTER when the original
	 * power adapter is not connected, is reported through the mode
	 * change events handled above.
	 */
	return err;
}

static const struct platform_profile_ops huawei_wmi_platform_profile_ops = {
	.probe = huawei_wmi_platform_profile_probe,
	.profile_get = huawei_wmi_platform_profile_get,
	.profile_set = huawei_wmi_platform_profile_set,
};
#else
static int
huawei_wmi_platform_profile_get(struct platform_profile_handler *handler,
				enum platform_profile_option *profile)
{
	int mode, err;

	err = huawei_wmi_perf_mode_get(&mode);
	if (err)
		return err;

	return huawei_wmi_platform_profile_from_mode(mode, profile);
}

static int
huawei_wmi_platform_profile_set(struct platform_profile_handler *handler,
				enum platform_profile_option profile)
{
	struct huawei_wmi *huawei = container_of(handler,
			struct huawei_wmi, profile_handler);
	int mode, err;

	err = huawei_wmi_platform_profile_to_mode(profile, &mode);
	if (err)
		return err;

	mutex_lock(&huawei->profile_sync_lock);
	err = huawei_wmi_perf_mode_set_checked(mode);
	if (!err)
		huawei->last_perf_mode = mode;
	mutex_unlock(&huawei->profile_sync_lock);

	return err;
}
#endif

/*
 * Re-read the firmware mode and return true when it differs from the last
 * mode reported to userspace. The firmware can change the mode on its own,
 * e.g. it reverts HUNTER shortly after accepting it when the original power
 * adapter is not connected, so the cached mode must be refreshed from the
 * firmware before userspace is notified. The lock is held across the read
 * and the cache update so that a concurrent profile change cannot be
 * overwritten with a stale read.
 */
static bool huawei_wmi_platform_profile_sync(struct huawei_wmi *huawei)
{
	int mode, err;
	bool changed = false;

	mutex_lock(&huawei->profile_sync_lock);

	err = huawei_wmi_perf_mode_get(&mode);
	if (!err && mode != huawei->last_perf_mode) {
		huawei->last_perf_mode = mode;
		changed = true;
	}

	mutex_unlock(&huawei->profile_sync_lock);

	return changed;
}

/*
 * platform_profile_notify() takes the platform profile core lock and must
 * be called with profile_sync_lock released.
 */
static void
huawei_wmi_platform_profile_notify_changed(struct huawei_wmi *huawei)
{
	if (!huawei_wmi_platform_profile_sync(huawei))
		return;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 9, 0)
	platform_profile_notify(huawei->platform_profile_dev);
#else
	platform_profile_notify();
#endif
}

static void huawei_wmi_platform_profile_sync_work(struct work_struct *work)
{
	struct huawei_wmi *huawei = container_of(to_delayed_work(work),
			struct huawei_wmi, profile_sync_work);

	huawei_wmi_platform_profile_notify_changed(huawei);
}

static void huawei_wmi_platform_profile_setup(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);
	enum platform_profile_option profile;
	int mode;
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 9, 0)
	int err;
#endif

	if (huawei_wmi_perf_mode_get(&mode) ||
	    huawei_wmi_platform_profile_from_mode(mode, &profile))
		return;

	mutex_init(&huawei->profile_sync_lock);
	INIT_DELAYED_WORK(&huawei->profile_sync_work,
			  huawei_wmi_platform_profile_sync_work);
	huawei->last_perf_mode = mode;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 9, 0)
	huawei->platform_profile_dev = platform_profile_register(dev,
			"huawei-wmi", huawei, &huawei_wmi_platform_profile_ops);
	if (IS_ERR(huawei->platform_profile_dev)) {
		dev_warn(dev, "Failed to register platform profile: %ld\n",
			 PTR_ERR(huawei->platform_profile_dev));
		huawei->platform_profile_dev = NULL;
	}
#else
	set_bit(PLATFORM_PROFILE_LOW_POWER, huawei->profile_handler.choices);
	set_bit(PLATFORM_PROFILE_BALANCED, huawei->profile_handler.choices);
	if (quirks->platform_profile_hunter)
		set_bit(PLATFORM_PROFILE_PERFORMANCE,
			huawei->profile_handler.choices);

	huawei->profile_handler.profile_get =
		huawei_wmi_platform_profile_get;
	huawei->profile_handler.profile_set =
		huawei_wmi_platform_profile_set;

	err = platform_profile_register(&huawei->profile_handler);
	if (err) {
		dev_warn(dev, "Failed to register platform profile: %d\n",
			 err);
		return;
	}

	huawei->platform_profile_registered = true;
#endif
}

static void huawei_wmi_platform_profile_exit(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	if (!huawei_wmi_platform_profile_active(huawei))
		return;

	/*
	 * The WMI notify handlers are already removed at this point, so the
	 * work cannot be rescheduled. Drain it, unregister the profile device
	 * to close the profile_set path, then drain once more to catch a work
	 * queued by a concurrent set just before the unregistration.
	 * platform_profile_notify() and platform_profile_remove() are both
	 * serialized by the platform profile core lock, so no notification
	 * can be in flight after the second drain.
	 */
	cancel_delayed_work_sync(&huawei->profile_sync_work);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 9, 0)
	platform_profile_remove(huawei->platform_profile_dev);
	huawei->platform_profile_dev = NULL;
#else
	platform_profile_remove();
	huawei->platform_profile_registered = false;
#endif

	cancel_delayed_work_sync(&huawei->profile_sync_work);
}

/* Hwmon subdriver */

/* GFNS reports two fan channels; reject implausible torn reads. */
#define HWMI_FAN_RPM_MAX	20000

/*
 * Temperature zones reported by GTMP. The first and eighth entries are the
 * CPU and battery zones; the remaining labels mirror the firmware table
 * exposed by the out-of-tree driver.
 */
static const u8 huawei_wmi_temp_zones[] = {
	0x00, 0x01, 0x05, 0x06, 0x07, 0x08, 0x0b, 0x0e, 0x0f, 0x15, 0x16,
};

static const char * const huawei_wmi_temp_labels[] = {
	"cpu", "TP01", "TSLO", "TP06", "TNTC", "CNTC", "DNTC", "battery",
	"TP0C", "TP07", "TP04",
};

static int huawei_wmi_fan_speed_get(u8 fan, long *rpm)
{
	u8 ret[0x100] = { 0 };
	union hwmi_arg arg;
	int fan_rpm, err;
	u8 low, high;
	u8 offset;

	if (quirks->fan_ec) {
		if (fan > 1)
			return -EINVAL;

		offset = fan ? 0x2e : 0x2c;
		err = ec_read(offset, &low);
		if (!err)
			err = ec_read(offset + 1, &high);
		if (!err) {
			fan_rpm = low | (high << 8);
			if (fan_rpm <= HWMI_FAN_RPM_MAX) {
				*rpm = fan_rpm;
				return 0;
			}
		}
	}

	arg.cmd = FAN_SPEED_GET;
	arg.args[2] = fan;

	err = huawei_wmi_cmd(arg.cmd, ret, sizeof(ret));
	if (err)
		return err;

	/* Current RPM, little-endian. */
	*rpm = ret[1] | (ret[2] << 8);
	if (*rpm > HWMI_FAN_RPM_MAX)
		return -ENODATA;

	return 0;
}

static int huawei_wmi_temp_get(u8 zone, long *temp)
{
	u8 ret[0x100] = { 0 };
	union hwmi_arg arg;
	int err;

	arg.cmd = TEMP_GET;
	arg.args[2] = zone;

	err = huawei_wmi_cmd(arg.cmd, ret, sizeof(ret));
	if (err)
		return err;

	/* GTMP reports whole degrees Celsius; hwmon wants millidegrees. */
	*temp = ret[2] * 1000;

	return 0;
}

static umode_t huawei_wmi_hwmon_is_visible(const void *drvdata,
		enum hwmon_sensor_types type, u32 attr, int channel)
{
	const struct huawei_wmi *huawei = drvdata;

	switch (type) {
	case hwmon_fan:
		if (channel < 2 && huawei->fan_speed_available &&
		    attr == hwmon_fan_input)
			return 0444;
		break;
	case hwmon_temp:
		if (channel < ARRAY_SIZE(huawei_wmi_temp_zones) &&
		    huawei->temp_available &&
		    (attr == hwmon_temp_input || attr == hwmon_temp_label))
			return 0444;
		break;
	default:
		break;
	}

	return 0;
}

static int huawei_wmi_hwmon_read(struct device *dev,
		enum hwmon_sensor_types type, u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_fan:
		if (channel >= 2)
			return -EINVAL;
		return huawei_wmi_fan_speed_get(channel, val);
	case hwmon_temp:
		if (channel >= ARRAY_SIZE(huawei_wmi_temp_zones))
			return -EINVAL;
		return huawei_wmi_temp_get(huawei_wmi_temp_zones[channel], val);
	default:
		return -EOPNOTSUPP;
	}
}

static int huawei_wmi_hwmon_read_string(struct device *dev,
		enum hwmon_sensor_types type, u32 attr, int channel,
		const char **str)
{
	if (type != hwmon_temp ||
	    channel >= ARRAY_SIZE(huawei_wmi_temp_labels))
		return -EINVAL;

	*str = huawei_wmi_temp_labels[channel];

	return 0;
}

static const struct hwmon_ops huawei_wmi_hwmon_ops = {
	.is_visible = huawei_wmi_hwmon_is_visible,
	.read = huawei_wmi_hwmon_read,
	.read_string = huawei_wmi_hwmon_read_string,
};

static const struct hwmon_channel_info * const huawei_wmi_hwmon_info[] = {
	HWMON_CHANNEL_INFO(fan,
			HWMON_F_INPUT,
			HWMON_F_INPUT),
	HWMON_CHANNEL_INFO(temp,
			HWMON_T_INPUT | HWMON_T_LABEL,
			HWMON_T_INPUT | HWMON_T_LABEL,
			HWMON_T_INPUT | HWMON_T_LABEL,
			HWMON_T_INPUT | HWMON_T_LABEL,
			HWMON_T_INPUT | HWMON_T_LABEL,
			HWMON_T_INPUT | HWMON_T_LABEL,
			HWMON_T_INPUT | HWMON_T_LABEL,
			HWMON_T_INPUT | HWMON_T_LABEL,
			HWMON_T_INPUT | HWMON_T_LABEL,
			HWMON_T_INPUT | HWMON_T_LABEL,
			HWMON_T_INPUT | HWMON_T_LABEL),
	NULL
};

static const struct hwmon_chip_info huawei_wmi_hwmon_chip_info = {
	.ops = &huawei_wmi_hwmon_ops,
	.info = huawei_wmi_hwmon_info,
};

static void huawei_wmi_hwmon_setup(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);
	struct device *hwmon_dev;
	long value;

	/*
	 * The two sensor groups are independent: some firmware only reports
	 * temperatures, and the ZQC-P EC fan fallback may work even when the
	 * WMI fan command does not.
	 */
	huawei->fan_speed_available = !huawei_wmi_fan_speed_get(0, &value);
	huawei->temp_available =
		!huawei_wmi_temp_get(huawei_wmi_temp_zones[0], &value);

	if (!huawei->fan_speed_available && !huawei->temp_available)
		return;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, "huawei_wmi",
			huawei, &huawei_wmi_hwmon_chip_info, NULL);
	if (IS_ERR(hwmon_dev))
		dev_warn(dev, "Failed to register hwmon device: %ld\n",
			 PTR_ERR(hwmon_dev));
}

/* debugfs */

static void huawei_wmi_debugfs_call_dump(struct seq_file *m, void *data,
		union acpi_object *obj)
{
	struct huawei_wmi *huawei = m->private;
	int i;

	switch (obj->type) {
	case ACPI_TYPE_INTEGER:
		seq_printf(m, "0x%llx", obj->integer.value);
		break;
	case ACPI_TYPE_STRING:
		seq_printf(m, "\"%.*s\"", obj->string.length, obj->string.pointer);
		break;
	case ACPI_TYPE_BUFFER:
		seq_puts(m, "{");
		for (i = 0; i < obj->buffer.length; i++) {
			seq_printf(m, "0x%02x", obj->buffer.pointer[i]);
			if (i < obj->buffer.length - 1)
				seq_puts(m, ",");
		}
		seq_puts(m, "}");
		break;
	case ACPI_TYPE_PACKAGE:
		seq_puts(m, "[");
		for (i = 0; i < obj->package.count; i++) {
			huawei_wmi_debugfs_call_dump(m, huawei, &obj->package.elements[i]);
			if (i < obj->package.count - 1)
				seq_puts(m, ",");
		}
		seq_puts(m, "]");
		break;
	default:
		dev_err(huawei->dev, "Unexpected obj type, got %d\n", obj->type);
		return;
	}
}

static int huawei_wmi_debugfs_call_show(struct seq_file *m, void *data)
{
	struct huawei_wmi *huawei = m->private;
	struct acpi_buffer out = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_buffer in;
	union acpi_object *obj;
	int err;

	in.length = sizeof(u64);
	in.pointer = &huawei->debug.arg;

	err = huawei_wmi_call(huawei, &in, &out);
	if (err)
		return err;

	obj = out.pointer;
	if (!obj) {
		err = -EIO;
		goto fail_debugfs_call;
	}

	huawei_wmi_debugfs_call_dump(m, huawei, obj);

fail_debugfs_call:
	kfree(out.pointer);
	return err;
}

DEFINE_SHOW_ATTRIBUTE(huawei_wmi_debugfs_call);

static void huawei_wmi_debugfs_setup(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	huawei->debug.root = debugfs_create_dir("huawei-wmi", NULL);

	debugfs_create_x64("arg", 0644, huawei->debug.root,
		&huawei->debug.arg);
	debugfs_create_file("call", 0400,
		huawei->debug.root, huawei, &huawei_wmi_debugfs_call_fops);
}

static void huawei_wmi_debugfs_exit(struct device *dev)
{
	struct huawei_wmi *huawei = dev_get_drvdata(dev);

	debugfs_remove_recursive(huawei->debug.root);
}

/* Input */

static void huawei_wmi_process_key(struct input_dev *idev, int code)
{
	struct huawei_wmi *huawei = dev_get_drvdata(idev->dev.parent);
	const struct key_entry *key;

	/*
	 * WMI0 uses code 0x80 to indicate a hotkey event.
	 * The actual key is fetched from the method WQ00
	 * using WMI0_EXPENSIVE_GUID.
	 */
	if (code == 0x80) {
		struct acpi_buffer response = { ACPI_ALLOCATE_BUFFER, NULL };
		union acpi_object *obj;
		acpi_status status;

		status = wmi_query_block(WMI0_EXPENSIVE_GUID, 0, &response);
		if (ACPI_FAILURE(status))
			return;

		obj = (union acpi_object *)response.pointer;
		if (obj && obj->type == ACPI_TYPE_INTEGER)
			code = obj->integer.value;

		kfree(response.pointer);
	}

	/*
	 * Performance mode change notifications are not keys. Toggle events
	 * do not carry the new mode, so re-read it from the firmware in a
	 * work item instead of parsing the event here; the WMI notify
	 * callback must not evaluate WMI methods inline.
	 */
	if (huawei_wmi_platform_profile_active(huawei)) {
		switch (code) {
		case HWMI_EVENT_MODE_SMART:
		case HWMI_EVENT_MODE_PERFORMANCE:
		case HWMI_EVENT_MODE_TOGGLE_ALT:
		case HWMI_EVENT_MODE_TOGGLE:
		case HWMI_EVENT_MODE_HUNTER:
			dev_dbg(&idev->dev, "Platform profile event 0x%04x\n",
				code);
			mod_delayed_work(system_wq,
					 &huawei->profile_sync_work,
					 msecs_to_jiffies(HWMI_MODE_EVENT_DEBOUNCE_MS));
			return;
		default:
			break;
		}
	}

	key = sparse_keymap_entry_from_scancode(idev, code);
	if (!key) {
		dev_info(&idev->dev, "Unknown key pressed, code: 0x%04x\n", code);
		return;
	}

	if (quirks && !quirks->report_brightness &&
			(key->sw.code == KEY_BRIGHTNESSDOWN ||
			key->sw.code == KEY_BRIGHTNESSUP))
		return;

	if (quirks && !quirks->report_volume &&
			(key->sw.code == KEY_VOLUMEUP ||
			key->sw.code == KEY_VOLUMEDOWN ||
			key->sw.code == KEY_MUTE))
		return;

	if (quirks && quirks->handle_kbdlight && huawei->kbdlight_available &&
			(key->code == KBDLIGHT_KEY_0 ||
			key->code == KBDLIGHT_KEY_1 ||
			key->code == KBDLIGHT_KEY_2)) {
		huawei_wmi_kbdlight_set(key->code - KBDLIGHT_KEY_0);
	}

	sparse_keymap_report_entry(idev, key, 1, true);
}

static void huawei_wmi_input_notify(union acpi_object *obj, void *context)
{
	struct input_dev *idev = (struct input_dev *)context;

	if (obj && obj->type == ACPI_TYPE_INTEGER)
		huawei_wmi_process_key(idev, obj->integer.value);
	else
		dev_err(&idev->dev, "Bad response type\n");
}

static int huawei_wmi_input_setup(struct device *dev,
		const char *guid,
		struct input_dev **idev)
{
	acpi_status status;
	int err;

	*idev = devm_input_allocate_device(dev);
	if (!*idev)
		return -ENOMEM;

	(*idev)->name = "Huawei WMI hotkeys";
	(*idev)->phys = "wmi/input0";
	(*idev)->id.bustype = BUS_HOST;
	(*idev)->dev.parent = dev;

	err = sparse_keymap_setup(*idev, huawei_wmi_keymap, NULL);
	if (err)
		return err;

	err = input_register_device(*idev);
	if (err)
		return err;

	status = wmi_install_notify_handler(guid, huawei_wmi_input_notify,
			*idev);
	if (ACPI_FAILURE(status))
		return -EIO;

	return 0;
}

static void huawei_wmi_input_exit(struct device *dev, const char *guid)
{
	wmi_remove_notify_handler(guid);
}

/* Huawei driver */

static const struct wmi_device_id huawei_wmi_events_id_table[] = {
	{ .guid_string = WMI0_EVENT_GUID },
	{ .guid_string = HWMI_EVENT_GUID },
	{  }
};

static int huawei_wmi_probe(struct platform_device *pdev)
{
	const struct wmi_device_id *guid = huawei_wmi_events_id_table;
	struct input_dev *idev = *huawei_wmi->idev;
	int err;

	platform_set_drvdata(pdev, huawei_wmi);
	huawei_wmi->dev = &pdev->dev;

	while (*guid->guid_string) {
		if (wmi_has_guid(guid->guid_string)) {
			err = huawei_wmi_input_setup(&pdev->dev, guid->guid_string, &idev);
			if (err) {
				dev_err(&pdev->dev, "Failed to setup input on %s\n", guid->guid_string);
				return err;
			}
		}

		idev++;
		guid++;
	}

	if (wmi_has_guid(HWMI_METHOD_GUID)) {
		mutex_init(&huawei_wmi->wmi_lock);

		huawei_wmi_smart_charge_setup(&pdev->dev);
		huawei_wmi_smart_charge_param_setup(&pdev->dev);
		huawei_wmi_kbdlight_timeout_setup(&pdev->dev);
		huawei_wmi_kbdlight_setup(&pdev->dev);
		huawei_wmi_leds_setup(&pdev->dev);
		huawei_wmi_fn_lock_setup(&pdev->dev);
		huawei_wmi_battery_setup(&pdev->dev);
		huawei_wmi_platform_profile_setup(&pdev->dev);
		huawei_wmi_hwmon_setup(&pdev->dev);
		huawei_wmi_debugfs_setup(&pdev->dev);
	}

	return 0;
}

static void huawei_wmi_remove(struct platform_device *pdev)
{
	const struct wmi_device_id *guid = huawei_wmi_events_id_table;

	while (*guid->guid_string) {
		if (wmi_has_guid(guid->guid_string))
			huawei_wmi_input_exit(&pdev->dev, guid->guid_string);

		guid++;
	}

	if (wmi_has_guid(HWMI_METHOD_GUID)) {
		huawei_wmi_debugfs_exit(&pdev->dev);
		huawei_wmi_battery_exit(&pdev->dev);
		huawei_wmi_fn_lock_exit(&pdev->dev);
		huawei_wmi_kbdlight_exit(&pdev->dev);
		huawei_wmi_kbdlight_timeout_exit(&pdev->dev);
		huawei_wmi_platform_profile_exit(&pdev->dev);
		huawei_wmi_smart_charge_exit(&pdev->dev);
		huawei_wmi_smart_charge_param_exit(&pdev->dev);
	}
}

static struct platform_driver huawei_wmi_driver = {
	.driver = {
		.name = "huawei-wmi",
	},
	.probe = huawei_wmi_probe,
	.remove = huawei_wmi_remove,
};

static __init int huawei_wmi_init(void)
{
	struct platform_device *pdev;
	int err;

	huawei_wmi = kzalloc_obj(struct huawei_wmi);
	if (!huawei_wmi)
		return -ENOMEM;

	quirks = &quirk_unknown;
	dmi_check_system(huawei_quirks);
	if (battery_reset != -1)
		quirks->battery_reset = battery_reset;
	if (report_brightness != -1)
		quirks->report_brightness = report_brightness;
	if (report_volume != -1)
		quirks->report_volume = report_volume;
	if (handle_kbdlight != -1)
		quirks->handle_kbdlight = handle_kbdlight;
	if (kbdlight_auto != -1)
		quirks->kbdlight_auto = kbdlight_auto;

	err = platform_driver_register(&huawei_wmi_driver);
	if (err)
		goto pdrv_err;

	pdev = platform_device_register_simple("huawei-wmi", PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(pdev)) {
		err = PTR_ERR(pdev);
		goto pdev_err;
	}
	return 0;

pdev_err:
	platform_driver_unregister(&huawei_wmi_driver);
pdrv_err:
	kfree(huawei_wmi);
	return err;
}

static __exit void huawei_wmi_exit(void)
{
	struct platform_device *pdev = to_platform_device(huawei_wmi->dev);

	platform_device_unregister(pdev);
	platform_driver_unregister(&huawei_wmi_driver);

	kfree(huawei_wmi);
}

module_init(huawei_wmi_init);
module_exit(huawei_wmi_exit);

MODULE_ALIAS("wmi:"HWMI_METHOD_GUID);
MODULE_DEVICE_TABLE(wmi, huawei_wmi_events_id_table);
MODULE_AUTHOR("Ayman Bagabas <ayman.bagabas@gmail.com>");
MODULE_DESCRIPTION("Huawei WMI laptop extras driver");
MODULE_LICENSE("GPL v2");
