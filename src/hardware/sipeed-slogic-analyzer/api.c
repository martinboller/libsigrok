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
    SR_CONF_VOLTAGE_THRESHOLD | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
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
        .max_samplerate = SR_MHZ(800),
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
    SR_MHZ(5),   SR_MHZ(8),   SR_MHZ(10),  SR_MHZ(16),  
    SR_MHZ(20),  SR_MHZ(25),  SR_MHZ(32),  SR_MHZ(50),  
    SR_MHZ(80),  SR_MHZ(100), SR_MHZ(160), SR_MHZ(200), 
    SR_MHZ(400), SR_MHZ(800), SR_MHZ(1600),
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
    struct sr_usb_dev_inst *usb_find;
    struct drv_context *drvc;
    struct dev_context *devc;

    const struct slogic_model *model;
    struct sr_config *option;
    struct libusb_device_descriptor des;
    GSList *devices = NULL;
    GSList *l, *conn_devices;
    char *conn;

    struct sr_channel *ch;
    unsigned int i;
    gchar *channel_name;

    drvc = di->context;
    
    for (l = options; l; l = l->next) {
        option = l->data;
        switch (option->key) {
        case SR_CONF_CONN:
            sr_info("Use conn: %s", g_variant_get_string(option->data, NULL));
            sr_err("Explicit conn option matching not supported yet!");
            return NULL;
        default:
            sr_warn("Unhandled option key: %u", option->key);
        }
    }

    for (model = &support_models[0]; model->name; model++) {
        conn = g_strdup_printf("%04x.%04x", USB_VID_SIPEED, model->pid);
        conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
        g_free(conn);

        for (l = conn_devices; l; l = l->next) {
            usb_find = l->data;
            
            ret = sr_usb_open(drvc->sr_ctx->libusb_ctx, usb_find);
            if (ret != SR_OK) 
                continue;

            libusb_get_device_descriptor(libusb_get_device(usb_find->devhdl), &des);
            
            char manufacturer_buf[64] = {0};
            libusb_get_string_descriptor_ascii(usb_find->devhdl, des.iManufacturer, 
                (unsigned char *)manufacturer_buf, sizeof(manufacturer_buf) - 1);
            char *iManufacturer = g_strdup(manufacturer_buf);

            char product_buf[64] = {0};
            libusb_get_string_descriptor_ascii(usb_find->devhdl, des.iProduct, 
                (unsigned char *)product_buf, sizeof(product_buf) - 1);
            char *iProduct = g_strdup(product_buf);

            char serial_buf[64] = {0};
            libusb_get_string_descriptor_ascii(usb_find->devhdl, des.iSerialNumber, 
                (unsigned char *)serial_buf, sizeof(serial_buf) - 1);
            char *iSerialNumber = g_strdup(serial_buf);

            char path_buf[32] = {0};
            usb_get_port_path(libusb_get_device(usb_find->devhdl), path_buf, sizeof(path_buf) - 1);
            char *iPortPath = g_strdup(path_buf);

            sr_usb_close(usb_find);

            sdi = sr_dev_inst_user_new(iManufacturer, iProduct, NULL);
            g_free(iManufacturer);
            g_free(iProduct);

            sdi->serial_num = iSerialNumber;   
            sdi->connection_id = iPortPath;   
            sdi->status = SR_ST_INACTIVE;
            sdi->inst_type = SR_INST_USB;
            sdi->driver = di;

            /* Safe Allocation: Copy parameters without breaking iteration */
            sdi->conn = sr_usb_dev_inst_new(usb_find->bus, usb_find->address, NULL);

            devc = g_malloc0(sizeof(struct dev_context));
            sdi->priv = devc;
            g_mutex_init(&devc->mutex);

            *(const struct slogic_model **)&devc->model = model;
            
            devc->limit_samplechannel = devc->model->max_samplechannel;
            devc->limit_samplerate = devc->model->max_bandwidth / devc->model->max_samplechannel;
            devc->cur_samplechannel = devc->limit_samplechannel;
            devc->cur_samplerate = devc->limit_samplerate;
            devc->cur_pattern_mode_idx = PATTERN_MODE_NORMAL;
            devc->cur_threshold_idx = 2; 
            devc->cur_voltage_threshold = 1.6;

            devc->digital_group = sr_channel_group_new(sdi, g_strdup("LA"), NULL);
            
            for (i = 0; i < devc->model->max_samplechannel; i++) {
                channel_name = g_strdup_printf("D%u", i);
                ch = sr_channel_new(sdi, i, SR_CHANNEL_LOGIC, TRUE, channel_name);
                g_free(channel_name);
                devc->digital_group->channels = g_slist_append(devc->digital_group->channels, ch);
            }

            devices = g_slist_append(devices, sdi);
        }
        g_slist_free_full(conn_devices, (GDestroyNotify)sr_usb_dev_inst_free);
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

    if (usb && usb->devhdl) {
        libusb_release_interface(usb->devhdl, 0);
        // Do NOT close or clear devhdl manually here; let libsigrok 
        // transport layers teardown the handle natively.
    }
    
    sdi->status = SR_ST_INACTIVE;
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
        if (!sdi) {
            *data = g_variant_new_uint64(SR_MHZ(10)); 
            return SR_OK;
        }
        devc = sdi->priv;
        if (!devc || devc->cur_samplerate == 0) {
            *data = g_variant_new_uint64(SR_MHZ(10));
        } else {
            *data = g_variant_new_uint64(devc->cur_samplerate);
        }
        return SR_OK;
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
        *data = std_gvar_tuple_double(devc->cur_voltage_threshold, devc->cur_voltage_threshold);
        break;
    case SR_CONF_TRIGGER_MATCH:
        *data = std_gvar_array_i32(trigger_matches, G_N_ELEMENTS(trigger_matches));
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

        uint64_t max_allowed_rate;
        if (active_channels <= 4)        max_allowed_rate = SR_MHZ(800);
        else if (active_channels <= 8)   max_allowed_rate = SR_MHZ(400);
        else                             max_allowed_rate = SR_MHZ(200);

        if (requested_rate > max_allowed_rate) {
            sr_err("Requested rate %" PRIu64 " MHz exceeds hardware limit for %zu active channels.", 
                   requested_rate / SR_MHZ(1), active_channels);
            return SR_ERR_SAMPLERATE;
        }

        devc->cur_samplerate = requested_rate;
        devc->cur_samplechannel = active_channels; 
        return SR_OK;
    case SR_CONF_BUFFERSIZE:
        {
            uint64_t requested_channels = g_variant_get_uint64(data);
            if (requested_channels > devc->model->max_samplechannel)
                return SR_ERR_ARG;
                
            devc->cur_samplechannel = requested_channels;
            sr_info("Slogic hardware data track width adjusted to: %" PRIu64 " channels.", requested_channels);
            return SR_OK;
        }
    case SR_CONF_LIMIT_SAMPLES:
        devc->cur_limit_samples = g_variant_get_uint64(data);
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
    (void)cg;

    switch (key) {
    case SR_CONF_SCAN_OPTIONS:
        *data = std_gvar_array_u32(scanopts, G_N_ELEMENTS(scanopts));
        return SR_OK;
    case SR_CONF_DEVICE_OPTIONS:
        if (!sdi) {
            *data = std_gvar_array_u32(drvopts, G_N_ELEMENTS(drvopts));
        } else {
            *data = std_gvar_array_u32(devopts, G_N_ELEMENTS(devopts));
        }
        return SR_OK;
    default:
        break;
    }

    if (sdi)
        devc = sdi->priv;

    switch (key) {
    /* --- START OF IMPLEMENTED REPLACEMENT --- */
    case SR_CONF_SAMPLERATE:
        if (!devc) {
            /* If no device context exists yet, expose all supported samplerates */
            *data = std_gvar_samplerates(samplerates, G_N_ELEMENTS(samplerates));
            return SR_OK;
        }

        /* Calculate how many channels are currently checked/enabled by the user */
        for (l = sdi->channels; l; l = l->next) {
            struct sr_channel *ch = l->data;
            if (ch->enabled && ch->type == SR_CHANNEL_LOGIC)
                active_channels++;
        }
        
        /* Fallback if no channels are explicitly enabled yet */
        if (active_channels == 0)
            active_channels = devc->model->max_samplechannel;

        /* Enforce dynamic bandwidth cap rules based on active channels */
        uint64_t max_allowed_rate;
        if (active_channels <= 4)       max_allowed_rate = SR_MHZ(800);
        else if (active_channels <= 8)  max_allowed_rate = SR_MHZ(400);
        else                            max_allowed_rate = SR_MHZ(200);

        /* Filter the standard samplerates array up to the permitted cap */
        for (num_samplerates = 0; num_samplerates < G_N_ELEMENTS(samplerates); num_samplerates++) {
            if (samplerates[num_samplerates] > max_allowed_rate) {
                break; 
            }
        }

        if (num_samplerates == 0)
            num_samplerates = 1;

        *data = std_gvar_samplerates(samplerates, num_samplerates);
        return SR_OK;
    /* --- END OF IMPLEMENTED REPLACEMENT --- */

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
    
    if (!sdi || !devc || !data || len == 0)
        return;

    g_mutex_lock(&devc->mutex); 
    
    // Safety check: If acquisition was already stopped elsewhere, drop incoming data
    if (sdi->status != SR_ST_ACTIVE) {
        g_mutex_unlock(&devc->mutex);
        return;
    }

    uint64_t nCh = devc->cur_samplechannel;

    if (nCh < 8) {
        size_t nsp_in_bytes = 8 / nCh;
        size_t unpacked_len = len * nsp_in_bytes;
        ptr = g_try_malloc(unpacked_len);
        if (!ptr) {
            sr_err("Unpacking malloc failed at low channel count.");
            g_mutex_unlock(&devc->mutex);
            return;
        }

        uint8_t *src = (uint8_t *)data;
        size_t dst_idx = 0;
        uint8_t mask = (1 << nCh) - 1;

        for (size_t src_idx = 0; src_idx < len; src_idx++) {
            uint8_t raw_byte = src[src_idx];
            for (size_t sample = 0; sample < nsp_in_bytes; sample++) {
                if (dst_idx < unpacked_len) {
                    ptr[dst_idx++] = (raw_byte >> (sample * nCh)) & mask;
                }
            }
        }
        len = unpacked_len;
    }

    // Prepare and dispatch datafeed packet
    struct sr_datafeed_logic logic;
    struct sr_datafeed_packet packet;

    logic.length = len;
    logic.unitsize = (nCh <= 8) ? 1 : 2; // 16 channels requires a 2-byte unitsize mapping!
    logic.data = ptr;

    packet.type = SR_DF_LOGIC;
    packet.payload = &logic;

    sr_session_send(sdi, &packet);

    if (nCh < 8)
        g_free(ptr);

    // Track and enforce sample limits for large acquisitions (like 50M)
    if (devc->cur_limit_samples > 0) {
        // Handle unitsize differences: 16 channels uses 2 bytes per sample point
        size_t samples_received = (nCh <= 8) ? len : (len / 2);
        devc->num_samples += samples_received;
        
        if (devc->num_samples >= devc->cur_limit_samples) {
            sr_info("Target sample depth of %" PRIu64 " reached. Stopping acquisition.", devc->cur_limit_samples);
            
            // Unlock before calling stop to prevent recursive deadlocks if stop clears the mutex
            g_mutex_unlock(&devc->mutex);
            sr_dev_acquisition_stop((struct sr_dev_inst *)sdi);
            return;
        }
    }

    g_mutex_unlock(&devc->mutex);
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
            g_usleep(1000);
        } while (!(cmd_aux[2] & 0x01));

        unsigned int loop_guard = 0;
        while (1) {
            if (loop_guard++ > 50) {
                sr_err("Clock division negotiation timed out. Aborting setup loop.");
                slogic_basic_16_remote_stop(sdi);
                return SR_ERR_TIMEOUT;
            }

            // Read the current hardware clock layout
            slogic_usb_control_read(sdi, SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ, SLOGIC_BASIC_16_R32_AUX + 4, 0x0000, cmd_aux + 4, (*(uint16_t*)cmd_aux) >> 9, 500);

            // Safely extract the profile attributes using explicit 16-bit array mapping
            uint16_t *hw_profile = (uint16_t*)(cmd_aux + 4);
            uint64_t base = SR_MHZ(1) * hw_profile[1]; // Evaluates cleanly to 200MHz or 800MHz
            
            if (base == 0) {
                sr_err("Hardware reported an invalid base clock profile.");
                return SR_ERR_DATA;
            }

            // If the current base clock profile cannot satisfy our target frequency, shift profiles
            if (base < devc->cur_samplerate || (base % devc->cur_samplerate) != 0) {
                hw_profile[0] += 1; // Try next clock index configuration profile
                slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_AUX + 4, 0x0000, cmd_aux + 4, 4, 500);
                g_usleep(2000);
                continue;
            }
            
            // Clean division matched! 
            // CRITICAL FIX: Only change the divider scalar factor (hw_profile[1]), 
            // leaving the rest of the layout telemetry untouched.
            uint32_t target_div = base / devc->cur_samplerate;
            hw_profile[1] = (uint16_t)target_div; 

            // Send back the safely modified profile segment
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
    int ret = 0;
    struct sr_usb_dev_inst *usb = sdi->conn;
    struct dev_context *devc = sdi->priv;
    uint8_t chunk[4];

    if (!data && len)
        return SR_ERR_ARG;

    g_mutex_lock(&devc->mutex);

    for (size_t i = 0; i < len; i += 4) {
        size_t size = (len - i >= 4) ? 4 : (len - i);
        if (size < 4) {
            memset(chunk, 0, 4);
            memcpy(chunk, data + i, size);
        } else {
            memcpy(chunk, data + i, 4);
        }

        int chunk_ret = libusb_control_transfer(
            usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
            request, value + i, index, chunk, 4, timeout
        );
        if (chunk_ret < 0) {
            g_mutex_unlock(&devc->mutex);
            return SR_ERR_NA;
        }
        ret += chunk_ret;
    }

    g_mutex_unlock(&devc->mutex);
    return ret;
}

static int slogic_usb_control_read(const struct sr_dev_inst *sdi, uint8_t request, uint16_t value, uint16_t index, uint8_t *data, size_t len, int timeout)
{
    int ret = 0;
    struct sr_usb_dev_inst *usb = sdi->conn;
    struct dev_context *devc = sdi->priv;
    uint8_t chunk[4];

    if (!data && len)
        return SR_ERR_ARG;

    g_mutex_lock(&devc->mutex);

    for (size_t i = 0; i < len; i += 4) {
        size_t size = (len - i >= 4) ? 4 : (len - i);
        int chunk_ret = libusb_control_transfer(
            usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
            request, value + i, index, chunk, 4, timeout
        );
        if (chunk_ret < 0) {
            g_mutex_unlock(&devc->mutex);
            return SR_ERR_NA;
        }
        memcpy(data + i, chunk, size);
        ret += chunk_ret;
    }

    g_mutex_unlock(&devc->mutex);
    return ret;
}

static int acquisition_start(const struct sr_dev_inst *sdi)
{
    return sipeed_slogic_acquisition_start((struct sr_dev_inst *)sdi);
}

static int acquisition_stop(struct sr_dev_inst *sdi)
{
    return sipeed_slogic_acquisition_stop(sdi);
}

static void clear_dev_context(void *priv)
{
    struct dev_context *devc = priv;
    if (!devc)
        return;

    sr_dbg("Cleaning up dev_context at %p", devc);

    // FIX: Clear it directly without the if-statement conditional wrapper
    g_mutex_clear(&devc->mutex);
    
    // Explicitly un-reference your dangling group pointer
    devc->digital_group = NULL;

    /* Remember: NO g_free(devc); here! Let libsigrok handle it. */
}

static int dev_clear(const struct sr_dev_driver *di)
{
    return std_dev_clear_with_callback(di, clear_dev_context);
}

static struct sr_dev_driver sipeed_slogic_analyzer_driver_info = {
    .name = "sipeed-slogic-analyzer",
    .longname = "Sipeed Slogic Analyzer",
    .api_version = 1,
    .init = std_init,         
    .cleanup = std_cleanup,   
    .dev_clear = dev_clear,       
    .scan = scan,
    .dev_list = std_dev_list,
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