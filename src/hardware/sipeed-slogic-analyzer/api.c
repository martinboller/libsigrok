/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2023 taorye <taorye@outlook.com>
 * Copyright (C) 2026 Martin <martin@bollers.dk> with help from different LLM's
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <string.h>
#include "protocol.h"

// Forward declarations for hardware communication helpers
static int slogic_usb_control_write(const struct sr_dev_inst *sdi, uint8_t request, uint16_t value, uint16_t index, uint8_t *data, size_t len, int timeout);
static int slogic_usb_control_read(const struct sr_dev_inst *sdi, uint8_t request, uint16_t value, uint16_t index, uint8_t *data, size_t len, int timeout);

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
};

static const uint32_t drvopts[] = {
	SR_CONF_LOGIC_ANALYZER,
};

static const uint32_t devopts[] = {
    SR_CONF_CONTINUOUS,
    SR_CONF_LIMIT_SAMPLES     | SR_CONF_GET | SR_CONF_SET,
    SR_CONF_SAMPLERATE        | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
    SR_CONF_BUFFERSIZE        | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
    SR_CONF_PATTERN_MODE      | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
    SR_CONF_TRIGGER_MATCH     | SR_CONF_GET | SR_CONF_LIST,                
    SR_CONF_VOLTAGE_THRESHOLD | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST, /* <-- Clean standard key */
};

static void slogic_submit_raw_data(void *data, size_t len, const struct sr_dev_inst *sdi);
static int slogic_lite_8_remote_run(const struct sr_dev_inst *sdi);
static int slogic_lite_8_remote_stop(const struct sr_dev_inst *sdi);
static int slogic_basic_16_remote_run(const struct sr_dev_inst *sdi);
static int slogic_basic_16_remote_stop(const struct sr_dev_inst *sdi);

static const struct slogic_model support_models[] = {
	{
		.name = "Slogic Lite 8",
		.pid = 0x0300,
		.ep_in = 0x01 | LIBUSB_ENDPOINT_IN,
		.max_samplerate = SR_MHZ(160),
		.max_samplechannel = 8,
		.max_bandwidth = SR_MHZ(320),
		.operation = {
			.remote_run = slogic_lite_8_remote_run,
			.remote_stop = slogic_lite_8_remote_stop,
		},
		.submit_raw_data = slogic_submit_raw_data,
	},
	{
		.name = "Slogic Basic 16 U3",
		.pid = 0x3031,
		.ep_in = 0x02 | LIBUSB_ENDPOINT_IN,
		.max_samplerate = SR_MHZ(1600),
		.max_samplechannel = 16,
		.max_bandwidth = SR_MHZ(3200),
		.operation = {
			.remote_run = slogic_basic_16_remote_run,
			.remote_stop = slogic_basic_16_remote_stop,
		},
		.submit_raw_data = slogic_submit_raw_data,
	},
	{
		.name = NULL,
		.pid = 0x0000,
	}
};

static const uint64_t samplerates[] = {
	SR_MHZ(1),   SR_MHZ(2),   SR_MHZ(4),   SR_MHZ(5),
	SR_MHZ(8),   SR_MHZ(10),  SR_MHZ(16),  SR_MHZ(20),
	SR_MHZ(32),  SR_MHZ(40),  SR_MHZ(80),  SR_MHZ(125),
	SR_MHZ(160), SR_MHZ(200), SR_MHZ(400), SR_MHZ(800),
	SR_MHZ(1600),
};

static const uint64_t buffersizes[] = {
	2, 4, 8, 16
};

static const char *patterns[] = {
#define GEN_PATTERN(P) [P] = #P
	GEN_PATTERN(PATTERN_MODE_NORMAL),
	GEN_PATTERN(PATTERN_MODE_TEST_MAX_SPEED),
#undef GEN_PATTERN
};

// Standard logic levels: { low_threshold, high_threshold } for RS232 use a 10k resistor to protect pin
static const double thresholds_3u[][2] = {
	{0.6, 0.6}, // 1.2 V Logic
	{0.8, 0.8}, // 1.8 V Logic
	{1.0, 1.0}, // Low 2.5 V Logic
	{1.6, 1.6}, // Standard 3.3 V Logic
	{2.5, 2.5}, // Standard 5.0 V Logic
	{4.0, 4.0}, // High-Voltage Signals
	{6.0, 6.0}, // Max Voltage Limit
};

static const int32_t trigger_matches[] = {
	SR_TRIGGER_ZERO,
	SR_TRIGGER_ONE,
	SR_TRIGGER_RISING,
	SR_TRIGGER_FALLING,
	SR_TRIGGER_EDGE,
};

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	int ret;
	struct sr_dev_inst *sdi;
	struct sr_usb_dev_inst *usb;
	struct drv_context *drvc;
	struct dev_context *devc;

	const struct slogic_model *model;
	struct sr_config *option;
	struct libusb_device_descriptor des;
	GSList *devices;
	GSList *l, *conn_devices;
	const char *conn;
	char cbuf[128];
	char *iManufacturer, *iProduct, *iSerialNumber, *iPortPath;

	struct sr_channel *ch;
	unsigned int i;
	gchar *channel_name;

	(void)options;
	conn = NULL;
	devices = NULL;
	drvc = di->context;
	
	for (l = options; l; l = l->next) {
		option = l->data;
		switch (option->key) {
		case SR_CONF_CONN:
			conn = g_variant_get_string(option->data, NULL);
			sr_info("Use conn: %s", conn);
			sr_err("Not supported now!");
			return NULL;
		default:
			sr_warn("Unhandled option key: %u", option->key);
		}
	}

	for (model = &support_models[0]; model->name; model++) {
		conn = g_strdup_printf("%04x.%04x", USB_VID_SIPEED, model->pid);
		conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
		for (l = conn_devices; l; l = l->next) {
			usb = l->data;
			ret = sr_usb_open(drvc->sr_ctx->libusb_ctx, usb);
			if (SR_OK != ret) continue;
			libusb_get_device_descriptor(libusb_get_device(usb->devhdl), &des);
			
			libusb_get_string_descriptor_ascii(usb->devhdl, des.iManufacturer, (unsigned char *)cbuf, sizeof(cbuf));
			iManufacturer = g_strdup(cbuf);
			libusb_get_string_descriptor_ascii(usb->devhdl, des.iProduct, (unsigned char *)cbuf, sizeof(cbuf));
			iProduct = g_strdup(cbuf);
			libusb_get_string_descriptor_ascii(usb->devhdl, des.iSerialNumber, (unsigned char *)cbuf, sizeof(cbuf));
			iSerialNumber = g_strdup(cbuf);
			usb_get_port_path(libusb_get_device(usb->devhdl), cbuf, sizeof(cbuf));
			iPortPath = g_strdup(cbuf);

			sdi = sr_dev_inst_user_new(iManufacturer, iProduct, NULL);
			sdi->serial_num = iSerialNumber;
			sdi->connection_id = iPortPath;
			sdi->status = SR_ST_INACTIVE;
			sdi->conn = usb;
			sdi->inst_type = SR_INST_USB;
			sdi->driver = di;

			devc = g_malloc0(sizeof(struct dev_context));
			sdi->priv = devc;

			*(const struct slogic_model **)&devc->model = model;
			
			devc->limit_samplechannel = devc->model->max_samplechannel;
			devc->limit_samplerate = devc->model->max_bandwidth / devc->model->max_samplechannel;

			devc->cur_samplechannel = devc->limit_samplechannel;
			devc->cur_samplerate = devc->limit_samplerate;
			devc->cur_pattern_mode_idx = PATTERN_MODE_NORMAL;
			devc->cur_threshold_idx = 2; // Evaluates to 1.6 V default
			devc->cur_voltage_threshold = 1.6;

			devc->digital_group = sr_channel_group_new(sdi, "LA", NULL);
			for (i = 0; i < devc->model->max_samplechannel; i++) {
				channel_name = g_strdup_printf("D%u", i);
				ch = sr_channel_new(sdi, i, SR_CHANNEL_LOGIC, TRUE, channel_name);
				g_free(channel_name);
				devc->digital_group->channels = g_slist_append(devc->digital_group->channels, ch);
			}

			devc->speed = libusb_get_device_speed(libusb_get_device(usb->devhdl));

			sr_usb_close(usb);
			devices = g_slist_append(devices, sdi);
		}
		g_free((void *)conn);
	}

	return std_scan_complete(di, devices);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	int ret;
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct drv_context *drvc = sdi->driver->context;

	ret = sr_usb_open(drvc->sr_ctx->libusb_ctx, usb);
	if (SR_OK != ret) return ret;

	ret = libusb_claim_interface(usb->devhdl, 0);
	if (ret != LIBUSB_SUCCESS) {
		sr_err("Unable to claim interface: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb = sdi->conn;

	libusb_release_interface(usb->devhdl, 0);
	sr_usb_close(usb);
	return SR_OK;
}

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	(void)cg;

	if (!sdi || !(devc = sdi->priv))
		return SR_ERR_ARG;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;
	case SR_CONF_BUFFERSIZE:
		*data = g_variant_new_uint64(devc->cur_samplechannel);
		break;
	case SR_CONF_PATTERN_MODE:
		*data = g_variant_new_string(patterns[devc->cur_pattern_mode_idx]);
		break;
	case SR_CONF_LIMIT_SAMPLES:
        *data = g_variant_new_uint64(devc->cur_limit_samples);
        break;
    case SR_CONF_VOLTAGE_THRESHOLD:
        /* Hand back the current precision index value mapped out as a paired tuple */
        *data = std_gvar_tuple_double(devc->cur_voltage_threshold, devc->cur_voltage_threshold);
        break;
		
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_set(uint32_t key, GVariant *data, const struct sr_dev_inst *sdi,
	const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	uint64_t requested_rate;
	const char *pattern_str;
	//const char *threshold_str;
	GSList *l;
	size_t active_channels = 0;
	size_t i;
	(void)cg;

	if (!sdi || !(devc = sdi->priv))
		return SR_ERR_ARG;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		requested_rate = g_variant_get_uint64(data);
		if (requested_rate < SR_KHZ(10))
			return SR_ERR_SAMPLERATE;

		for (l = sdi->channels; l; l = l->next) {
			struct sr_channel *ch = l->data;
			if (ch->enabled && ch->type == SR_CHANNEL_LOGIC)
				active_channels++;
		}
		if (active_channels == 0)
			active_channels = devc->model->max_samplechannel;

		if (requested_rate * active_channels > devc->model->max_bandwidth) {
			sr_err("Requested rate %" PRIu64 " MHz exceeds max bandwidth for %zu active channels.", 
			       requested_rate / SR_MHZ(1), active_channels);
			return SR_ERR_SAMPLERATE;
		}

		devc->cur_samplerate = requested_rate;
		return SR_OK;

	case SR_CONF_LIMIT_SAMPLES:
        uint64_t requested_samples = g_variant_get_uint64(data);
        devc->cur_limit_samples = requested_samples;
        return SR_OK;

	case SR_CONF_PATTERN_MODE:
		pattern_str = g_variant_get_string(data, NULL);
		for (i = 0; i < G_N_ELEMENTS(patterns); i++) {
			if (g_strcmp0(pattern_str, patterns[i]) == 0) {
				devc->cur_pattern_mode_idx = i;
				return SR_OK;
			}
		}
		return SR_ERR_ARG;
		
	case SR_CONF_VOLTAGE_THRESHOLD:
        {
            GVariant *rangeval;
            double low_thresh;

            rangeval = g_variant_get_child_value(data, 0);
            low_thresh = g_variant_get_double(rangeval);
            g_variant_unref(rangeval);

            /* Clamping safety guard rails */
            if (low_thresh < 0.4) low_thresh = 0.4;
            if (low_thresh > 6.0) low_thresh = 6.0;

            devc->cur_voltage_threshold = low_thresh;
            sr_info("Voltage threshold selected: %f V", devc->cur_voltage_threshold);
            return SR_OK;
        }
	
		default:
		return SR_ERR_NA;
	}
}

static int config_list(uint32_t key, GVariant **data, const struct sr_dev_inst *sdi,
	const struct sr_channel_group *cg)
{
	struct dev_context *devc = NULL;
	GSList *l;
	size_t active_channels = 0;
	size_t num_samplerates = 0;

	if (sdi)
		devc = sdi->priv;

	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
	case SR_CONF_SAMPLERATE:
		if (!devc) {
			*data = std_gvar_samplerates(samplerates, G_N_ELEMENTS(samplerates));
			return SR_OK;
		}

		for (l = sdi->channels; l; l = l->next) {
			struct sr_channel *ch = l->data;
			if (ch->enabled && ch->type == SR_CHANNEL_LOGIC)
				active_channels++;
		}
		
		if (active_channels == 0)
			active_channels = devc->model->max_samplechannel;

		for (num_samplerates = 0; num_samplerates < G_N_ELEMENTS(samplerates); num_samplerates++) {
			if (samplerates[num_samplerates] * active_channels > devc->model->max_bandwidth) {
				break;
			}
		}

		if (num_samplerates == 0)
			num_samplerates = 1;

		*data = std_gvar_samplerates(samplerates, num_samplerates);
		return SR_OK;

	case SR_CONF_TRIGGER_MATCH:
		*data = std_gvar_array_i32(trigger_matches, G_N_ELEMENTS(trigger_matches));
		return SR_OK;
	case SR_CONF_BUFFERSIZE:
		*data = std_gvar_array_u64(buffersizes, G_N_ELEMENTS(buffersizes));
		return SR_OK;
	case SR_CONF_PATTERN_MODE:
		*data = g_variant_new_strv(patterns, G_N_ELEMENTS(patterns));
		return SR_OK;

	case SR_CONF_VOLTAGE_THRESHOLD:
        {
            GVariantBuilder gvb;
            GVariant *tuple;
            double v;

            g_variant_builder_init(&gvb, G_VARIANT_TYPE_ARRAY);

            /* Generate values from 0.4V to 6.0V in 0.1V increments */
            for (v = 0.4; v <= 6.05; v += 0.1) {
                tuple = std_gvar_tuple_double(v, v);
                g_variant_builder_add_value(&gvb, tuple);
            }

            *data = g_variant_builder_end(&gvb);
            return SR_OK;
        }

	default:
		return SR_ERR_NA;
	}
}

static void slogic_submit_raw_data(void *data, size_t len, const struct sr_dev_inst *sdi) {
	struct dev_context *devc = sdi->priv;
	uint8_t *ptr = data;
	uint64_t nCh = devc->cur_samplechannel;

	if (nCh < 8) {
		size_t nsp_in_bytes = 8 / nCh;
		ptr = g_malloc(len * nsp_in_bytes);
		for (size_t i = 0; i < len; i += nCh) {
			for (size_t j = 0; j < 8; j++) {
				ptr[i * nsp_in_bytes + j] = (((uint8_t *)data)[i + j / nsp_in_bytes] >> (j % nsp_in_bytes * nCh)) & ((1 << nCh) - 1);
			}
		}
		len *= nsp_in_bytes;
	}

	sr_session_send(sdi, &(struct sr_datafeed_packet) {
		.type = SR_DF_LOGIC,
		.payload = &(struct sr_datafeed_logic) {
			.length = len,
			.unitsize = (nCh + 7) / 8,
			.data = ptr,
		}
	});

	if (nCh < 8)
		g_free(ptr);
}

#pragma pack(push, 1)
struct cmd_start_acquisition {
	union {
		struct {
			uint8_t sample_rate_l;
			uint8_t sample_rate_h;
		};
		uint16_t sample_rate;
	};
	uint8_t sample_channel;
};
#pragma pack(pop)

#define CMD_START   0xb1
#define CMD_STOP    0xb3

static int slogic_lite_8_remote_run(const struct sr_dev_inst *sdi) {
	struct dev_context *devc = sdi->priv;
	const struct cmd_start_acquisition cmd_run = {
		.sample_rate = devc->cur_samplerate / SR_MHZ(1),
		.sample_channel = devc->cur_samplechannel,
	};
	return slogic_usb_control_write(sdi, CMD_START, 0x0000, 0x0000, (uint8_t *)&cmd_run, sizeof(cmd_run), 500);
}

static int slogic_lite_8_remote_stop(const struct sr_dev_inst *sdi) {
	(void)sdi;
	return SR_OK;
}

#define SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ     0x00
#define SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE    0x01
#define SLOGIC_BASIC_16_R32_CTRL    0x0004
#define SLOGIC_BASIC_16_R32_FLAG    0x0008
#define SLOGIC_BASIC_16_R32_AUX     0x000c

static int slogic_basic_16_remote_run(const struct sr_dev_inst *sdi) {
	struct dev_context *devc = sdi->priv;
	const uint8_t cmd_derst[] = {0x00, 0x00, 0x00, 0x00};
	const uint8_t cmd_run[] = {0x01, 0x00, 0x00, 0x00};
	uint8_t cmd_aux[64] = {0};

	slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_CTRL, 0x0000, (uint8_t *)cmd_derst, 4, 500);

	{
		unsigned int retry = 0;
		memset(cmd_aux, 0, sizeof(cmd_aux));
		*(uint32_t*)(cmd_aux) = 0x00000001;
		slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_AUX, 0x0000, cmd_aux, 4, 500);
		do {
			slogic_usb_control_read(sdi, SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ, SLOGIC_BASIC_16_R32_AUX, 0x0000, cmd_aux, 4, 500);
			sr_dbg("[%u]read aux channel: %08x.", retry, ((uint32_t*)cmd_aux)[0]);
			retry += 1;
			if (retry > 5)
				return SR_ERR_TIMEOUT;
		} while (!(cmd_aux[2] & 0x01));
		
		slogic_usb_control_read(sdi, SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ, SLOGIC_BASIC_16_R32_AUX + 4, 0x0000, cmd_aux + 4, (*(uint16_t*)cmd_aux) >> 9, 500);
		*(uint32_t*)(cmd_aux + 4) = (1U << devc->cur_samplechannel) - 1;

		slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_AUX + 4, 0x0000, cmd_aux + 4, (*(uint16_t*)cmd_aux) >> 9, 500);
		slogic_usb_control_read(sdi, SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ, SLOGIC_BASIC_16_R32_AUX + 4, 0x0000, cmd_aux + 4, (*(uint16_t*)cmd_aux) >> 9, 500);

		if (((1U << devc->cur_samplechannel) - 1) != *(uint32_t*)(cmd_aux + 4)) {
			sr_dbg("Failed to configure sample channel.");
		} else {
			sr_dbg("Succeed to configure sample channel.");
		}
	}

	{
		unsigned int retry = 0;
		memset(cmd_aux, 0, sizeof(cmd_aux));
		*(uint32_t*)(cmd_aux) = 0x00000002;
		slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_AUX, 0x0000, cmd_aux, 4, 500);
		do {
			slogic_usb_control_read(sdi, SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ, SLOGIC_BASIC_16_R32_AUX, 0x0000, cmd_aux, 4, 500);
			sr_dbg("[%u]read aux samplerate: %08x.", retry, ((uint32_t*)cmd_aux)[0]);
			retry += 1;
			if (retry > 5)
				return SR_ERR_TIMEOUT;
		} while (!(cmd_aux[2] & 0x01));

		while (((uint16_t*)(cmd_aux + 4))[0] <= 1) {
			slogic_usb_control_read(sdi, SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ, SLOGIC_BASIC_16_R32_AUX + 4, 0x0000, cmd_aux + 4, (*(uint16_t*)cmd_aux) >> 9, 500);

			uint64_t base = SR_MHZ(1) * ((uint16_t*)(cmd_aux + 4))[1];
			if (base % devc->cur_samplerate) {
				((uint16_t*)(cmd_aux + 4))[0] += 1;
				slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_AUX + 4, 0x0000, cmd_aux + 4, 4, 500);
				continue;
			}
			uint32_t div = base / devc->cur_samplerate;
			((uint32_t*)(cmd_aux + 4))[1] = div;

			slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_AUX + 4, 0x0000, cmd_aux + 4, (*(uint16_t*)cmd_aux) >> 9, 500);
			slogic_usb_control_read(sdi, SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ, SLOGIC_BASIC_16_R32_AUX + 4, 0x0000, cmd_aux + 4, (*(uint16_t*)cmd_aux) >> 9, 500);
			break;
		}
	}

	return slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_CTRL, 0x0000, (uint8_t *)cmd_run, 4, 500);
}

static int slogic_basic_16_remote_stop(const struct sr_dev_inst *sdi) {
	const uint8_t cmd_rst[] = {0x02, 0x00, 0x00, 0x00};
	return slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_CTRL, 0x0000, (uint8_t *)cmd_rst, 4, 500);
}

static int slogic_usb_control_write(const struct sr_dev_inst *sdi, uint8_t request, uint16_t value, uint16_t index, uint8_t *data, size_t len, int timeout)
{
	int ret;
	struct sr_usb_dev_inst *usb = sdi->conn;

	if (!data && len) {
		len = 0;
	} else if (len & 0x3) {
		size_t len_aligndup = (len + 0x3) & (~0x3);
		len = len_aligndup;
	}

	ret = 0;
	for (size_t i = 0; i < len; i += 4) {
		ret += libusb_control_transfer(
			usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
			request, value + i, index, (unsigned char *)data + i, 4, timeout
		);
		if (ret < 0) {
			return SR_ERR_NA;
		}
	}
	return ret;
}

static int slogic_usb_control_read(const struct sr_dev_inst *sdi, uint8_t request, uint16_t value, uint16_t index, uint8_t *data, size_t len, int timeout)
{
	int ret;
	struct sr_usb_dev_inst *usb = sdi->conn;

	if (!data && len) {
		return SR_ERR_ARG;
	} else if (len & 0x3) {
		size_t len_aligndup = (len + 0x3) & (~0x3);
		len = len_aligndup;
	}

	ret = 0;
	for (size_t i = 0; i < len; i += 4) {
		ret += libusb_control_transfer(
			usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
			request, value + i, index, (unsigned char *)data + i, 4, timeout
		);
		if (ret < 0) {
			return SR_ERR_NA;
		}
	}
	return ret;
}

static int dev_clear(const struct sr_dev_driver *di)
{
	return std_dev_clear_with_callback(di, NULL);
}

static int acquisition_start(const struct sr_dev_inst *sdi)
{
	return sipeed_slogic_acquisition_start((struct sr_dev_inst *)sdi);
}

static int acquisition_stop(struct sr_dev_inst *sdi)
{
	return sipeed_slogic_acquisition_stop(sdi);
}

static struct sr_dev_driver sipeed_slogic_analyzer_driver_info = {
	.name = "sipeed-slogic-analyzer",
	.longname = "Sipeed Slogic Analyzer",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = acquisition_start,
	.dev_acquisition_stop = acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(sipeed_slogic_analyzer_driver_info);