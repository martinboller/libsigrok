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
#include <inttypes.h>
#include <string.h>
#include "protocol.h"

#define SHUTDOWN_MARKER GINT_TO_POINTER(0xDEAD)

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer) {
    const struct sr_dev_inst *sdi;
    struct dev_context *devc;

    sdi = transfer->user_data;
    if (!sdi || !(devc = sdi->priv))
        return;

    switch (transfer->status) {
        case LIBUSB_TRANSFER_COMPLETED: 
        case LIBUSB_TRANSFER_TIMED_OUT: {
            if (devc->acq_aborted) {
                goto decommission;
            }

            /* Handle raw pacing timeouts with zero bytes payload gracefully */
            if (transfer->status == LIBUSB_TRANSFER_TIMED_OUT && transfer->actual_length == 0) {
                if (!devc->acq_aborted) {
                    int ret = libusb_submit_transfer(transfer);
                    if (ret == 0) break;
                }
                goto decommission;
            }

            /* Enforce limit clipping bounds */
            if ((uint64_t)transfer->actual_length > devc->samples_need_nbytes - devc->samples_got_nbytes)
                transfer->actual_length = devc->samples_need_nbytes - devc->samples_got_nbytes;
            
            devc->samples_got_nbytes += transfer->actual_length;

            if (devc->samples_got_nbytes >= devc->samples_need_nbytes) {
                devc->acq_aborted = 1;
            }

            if (transfer->actual_length == 0) {
                goto decommission;
            }

            /* Copy payload memory out of the hot libusb context to protect thread isolated bounds */
            if (devc->cur_pattern_mode_idx != PATTERN_MODE_TEST_MAX_SPEED && devc->raw_data_queue) {
                uint8_t *payload_copy = g_try_malloc(transfer->actual_length);
                if (payload_copy) {
                    memcpy(payload_copy, transfer->buffer, transfer->actual_length);
                    struct raw_packet_chunk *chunk = g_malloc(sizeof(struct raw_packet_chunk));
                    chunk->data = payload_copy;
                    chunk->length = transfer->actual_length;
                    g_async_queue_push(devc->raw_data_queue, chunk);
                }
            }

            /* Re-submit transfer safely exclusively on native libusb engine threads */
            if (!devc->acq_aborted) {
                transfer->actual_length = 0;
                int ret = libusb_submit_transfer(transfer);
                if (ret == 0) {
                    break; 
                }
                sr_dbg("Failed to resubmit transfer: %s", libusb_error_name(ret));
            }

decommission:
            for (size_t i = 0; i < NUM_MAX_TRANSFERS; i++) {
                if (devc->transfers[i] == transfer) {
                    devc->transfers[i] = NULL;
                    break;
                }
            }

            devc->num_transfers_used -= 1;
            g_free(transfer->buffer);
            libusb_free_transfer(transfer);
            
            if (devc->num_transfers_used == 0 && devc->raw_data_queue) {
                g_async_queue_push(devc->raw_data_queue, SHUTDOWN_MARKER);
            }
        } break;

        case LIBUSB_TRANSFER_CANCELLED:
        case LIBUSB_TRANSFER_OVERFLOW:
        case LIBUSB_TRANSFER_STALL:
        case LIBUSB_TRANSFER_NO_DEVICE:
        default: {
            sr_dbg("Transfer returned with terminal status: %s.", libusb_error_name(transfer->status));
            
            for (size_t i = 0; i < NUM_MAX_TRANSFERS; i++) {
                if (devc->transfers[i] == transfer) {
                    devc->transfers[i] = NULL;
                    break;
                }
            }

            g_free(transfer->buffer);
            libusb_free_transfer(transfer);
            devc->num_transfers_used -= 1;
            
            if (devc->num_transfers_used == 0 && devc->raw_data_queue) {
                g_async_queue_push(devc->raw_data_queue, SHUTDOWN_MARKER);
            }
        } break;
    }
    devc->num_transfers_completed += 1;
}

static int handle_events(int fd, int revents, void *cb_data)
{
    struct sr_dev_inst *sdi;
    struct sr_dev_driver *di;
    struct dev_context *devc;
    struct drv_context *drvc;

    (void)fd;
    (void)revents;

    sdi = cb_data;
    if (!sdi || !(devc = sdi->priv))
        return TRUE;

    di = sdi->driver;
    drvc = di->context;

    if (devc->acq_aborted) {
        if (devc->num_transfers_used > 0) {
            /* Gently issue async cancellations without breaking tracker allocations mid-flight */
            for (size_t i = 0; i < NUM_MAX_TRANSFERS; ++i) {
                struct libusb_transfer *transfer = devc->transfers[i];
                if (transfer) {
                    libusb_cancel_transfer(transfer);
                }
            }
        } else {
            /* No more hardware context threads remain alive: secure to teardown core allocations */
            if (devc->raw_data_queue) {
                g_async_queue_push(devc->raw_data_queue, SHUTDOWN_MARKER);
            }

            if (devc->raw_data_handle_thread) {
                g_thread_join(devc->raw_data_handle_thread);
                devc->raw_data_handle_thread = NULL;
            }

            if (devc->raw_data_queue) {
                gpointer xfer;
                while ((xfer = g_async_queue_try_pop(devc->raw_data_queue)) != NULL) {
                    if (xfer != SHUTDOWN_MARKER) {
                        struct raw_packet_chunk *chunk = (struct raw_packet_chunk *)xfer;
                        g_free(chunk->data);
                        g_free(chunk);
                    }
                }
                g_async_queue_unref(devc->raw_data_queue);
                devc->raw_data_queue = NULL;
            }

            sr_session_source_remove(sdi->session, -1 * (size_t)drvc->sr_ctx->libusb_ctx);
            sr_info("Bulk processing finalized cleanly. Total transfers handled: %" PRIu64 ".", (uint64_t)devc->num_transfers_completed);
            return TRUE;
        }
    }

    libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx, &(struct timeval){0, 1000}, NULL);

    return TRUE;
}

static gpointer raw_data_handle_thread_func(gpointer user_data)
{
    struct sr_dev_inst *sdi = user_data;
    struct dev_context *devc = sdi->priv;

    while (1) {
        gpointer data = g_async_queue_pop(devc->raw_data_queue);
        
        if (data != NULL && data != SHUTDOWN_MARKER) {
            struct raw_packet_chunk *chunk = (struct raw_packet_chunk *)data;
            
            if (devc->trigger_fired && !devc->acq_aborted) {
                devc->model->submit_raw_data(chunk->data, chunk->length, sdi);
            }

            g_free(chunk->data);
            g_free(chunk);
        } else {
            break;
        }
    }

    std_session_send_df_end(sdi);
    return NULL;
}

SR_PRIV int sipeed_slogic_acquisition_start(const struct sr_dev_inst *sdi)
{
    struct dev_context *devc;
    struct drv_context *drvc;
    struct sr_usb_dev_inst *usb;
    int ret;
    GSList *l;
    size_t active_channels = 0;

    devc = sdi->priv;
    drvc = sdi->driver->context;
    usb = sdi->conn;
    devc->num_samples = 0;
    
    if ((ret = devc->model->operation.remote_stop(sdi)) < 0) {
        sr_err("Unhandled `CMD_STOP`");
        return ret;
    }

    for (l = sdi->channels; l; l = l->next) {
        struct sr_channel *ch = l->data;
        if (ch->enabled && ch->type == SR_CHANNEL_LOGIC)
            active_channels++;
    }
    if (active_channels == 0) active_channels = devc->cur_samplechannel;
    
    devc->cur_samplechannel = active_channels;

    /* Fixed byte target math supporting multi-byte unitsize layouts (16 channels = 2 bytes) */
    uint64_t unitsize = (devc->cur_samplechannel <= 8) ? 1 : 2;
    devc->samples_got_nbytes = 0;
    devc->samples_need_nbytes = devc->cur_limit_samples * unitsize;
    
    if (devc->samples_need_nbytes == 0) {
        devc->samples_need_nbytes = 4096; 
    }

    sr_info("Need %" PRIu64 "x %" PRIu64 "ch@%" PRIu64 "MHz in %" PRIu64 "ms.", 
            devc->cur_limit_samples,
            devc->cur_samplechannel, 
            devc->cur_samplerate / SR_MHZ(1),
            devc->cur_samplerate > 0 ? (1000 * devc->cur_limit_samples / devc->cur_samplerate) : 0
    );

    devc->per_transfer_duration = 40;
    devc->per_transfer_nbytes = devc->per_transfer_duration * devc->cur_samplerate * devc->cur_samplechannel / 8 / SR_KHZ(1);
    devc->per_transfer_nbytes = (devc->per_transfer_nbytes + (2 * 16 * 1024 - 1)) & ~(2 * 16 * 1024 - 1);

    devc->acq_aborted = 0;
    devc->num_transfers_used = 0;
    devc->num_transfers_completed = 0;
    memset(devc->transfers, 0, sizeof(devc->transfers));
    devc->transfers_reached_nbytes = 0;
    
    devc->raw_data_queue = g_async_queue_new();
    if (!devc->raw_data_queue) {
        sr_err("New g_async_queue failed!");
        return SR_ERR_MALLOC;
    }

    devc->raw_data_handle_thread = g_thread_new("raw_data_handle_thread", raw_data_handle_thread_func, (gpointer)sdi);
    if (!devc->raw_data_handle_thread) {
        sr_err("Create processing thread failed!");
        g_async_queue_unref(devc->raw_data_queue);
        devc->raw_data_queue = NULL;
        return SR_ERR_MALLOC;
    }

    while (devc->num_transfers_used < NUM_MAX_TRANSFERS && devc->samples_got_nbytes + devc->num_transfers_used * devc->per_transfer_nbytes < devc->samples_need_nbytes)
    {
        uint8_t *dev_buf = g_malloc(devc->per_transfer_nbytes);
        if (!dev_buf) {
            sr_dbg("Failed to allocate memory[%" PRIu64 "]", (uint64_t)devc->num_transfers_used);
            break;
        }

        struct libusb_transfer *transfer = libusb_alloc_transfer(0);
        if (!transfer) {
            sr_dbg("Failed to allocate transfer[%" PRIu64 "]", (uint64_t)devc->num_transfers_used);
            g_free(dev_buf);
            break;
        }

        /* 2-second timeout window provides relaxed bounds for low pacing rates */
        unsigned int calculated_timeout = 2000; 
        libusb_fill_bulk_transfer(transfer, usb->devhdl, devc->model->ep_in,
                                    dev_buf, (int)devc->per_transfer_nbytes, receive_transfer,
                                    (gpointer)sdi, calculated_timeout);
        transfer->actual_length = 0;

        ret = libusb_submit_transfer(transfer);
        if (ret) {
            sr_dbg("Failed to submit transfer[%" PRIu64 "]: %s.", (uint64_t)devc->num_transfers_used, libusb_error_name(ret));
            g_free(transfer->buffer);
            libusb_free_transfer(transfer);
            break;
        }
        devc->transfers[devc->num_transfers_used] = transfer;
        devc->num_transfers_used += 1;
    }
    sr_dbg("Submitted %" PRIu64 " transfers", (uint64_t)devc->num_transfers_used);

    if (!devc->num_transfers_used) {
        g_thread_join(devc->raw_data_handle_thread);
        devc->raw_data_handle_thread = NULL;
        g_async_queue_unref(devc->raw_data_queue);
        devc->raw_data_queue = NULL;
        return SR_ERR_IO;
    }

    std_session_send_df_header(sdi);
    std_session_send_df_frame_begin(sdi);

    uint32_t polling_interval = (uint32_t)(devc->per_transfer_duration / 2);
    sr_session_source_add(sdi->session, -1 * (size_t)drvc->sr_ctx->libusb_ctx, 0, (int)(polling_interval ? polling_interval : 1), handle_events, (void *)sdi);

    devc->trigger_fired = TRUE;
    if ((ret = devc->model->operation.remote_run(sdi)) < 0) {
        sr_err("Unhandled `CMD_RUN`");
        sipeed_slogic_acquisition_stop((struct sr_dev_inst *)sdi);
        return ret;
    }

    devc->transfers_reached_time_start = g_get_monotonic_time();
    devc->transfers_reached_time_latest = devc->transfers_reached_time_start;

    return SR_OK;
}

SR_PRIV int sipeed_slogic_acquisition_stop(struct sr_dev_inst *sdi)
{
    struct dev_context *devc = sdi->priv;

    devc->trigger_fired = FALSE;
    devc->acq_aborted = 1;

    if (devc->raw_data_queue) {
        g_async_queue_push(devc->raw_data_queue, SHUTDOWN_MARKER);
    }

    return SR_OK;
}