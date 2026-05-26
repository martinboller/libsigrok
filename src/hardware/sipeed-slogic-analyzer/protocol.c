/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2023 taorye <taorye@outlook.com>
 * Copyright (C) 2026 AI Collaborator <gemini@google.com>
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

/* Max buffering limit. 100 items * 125ms per transfer provides ~12.5 seconds
 * of thread scheduling jitter protection while keeping RAM strictly bounded. */
/* Max queue depth = max RAM for buffering / devc->per_transfer_nbytes
For 1 GB Buffer Limit: 80 items max depth.
For 4 GB Buffer Limit: 320 items max depth.
For 8 GB Buffer Limit: 640 max depth.
*/
#define MAX_QUEUE_DEPTH 1920

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer) {
    int ret;
    const struct sr_dev_inst *sdi;
    struct dev_context *devc;

    sdi = transfer->user_data;
    if (!sdi)
        return;
    devc = sdi->priv;

    int64_t transfers_reached_time_now = g_get_monotonic_time();

    switch (transfer->status) {
        case LIBUSB_TRANSFER_COMPLETED: 
        case LIBUSB_TRANSFER_TIMED_OUT: {
            devc->transfers_reached_nbytes_latest = transfer->actual_length;
            devc->transfers_reached_nbytes += devc->transfers_reached_nbytes_latest;
            
            if ((uint64_t)transfer->actual_length > devc->samples_need_nbytes - devc->samples_got_nbytes)
                transfer->actual_length = devc->samples_need_nbytes - devc->samples_got_nbytes;
            
            devc->samples_got_nbytes += transfer->actual_length;
            devc->transfers_reached_time_latest = transfers_reached_time_now;

            if (transfer->actual_length == 0) {
                devc->num_transfers_used -= 1;
                break;
            }

            if (devc->cur_pattern_mode_idx != PATTERN_MODE_TEST_MAX_SPEED) {
                uint8_t *d = transfer->buffer;
                size_t len = transfer->actual_length;

                /* Thread-safe check on queue depth boundary */
                if (g_async_queue_length(devc->raw_data_queue) < MAX_QUEUE_DEPTH) {
                    uint8_t *ptr = g_malloc(devc->per_transfer_nbytes);
                    if (!ptr) {
                        sr_err("Failed to allocate memory: %" PRIu64 " bytes!", devc->per_transfer_nbytes);
                        devc->acq_aborted = 1;
                        devc->num_transfers_used -= 1;
                        break;
                    }
                    /* Swap buffer out to the consumer thread queue */
                    transfer->buffer = ptr; 
                    g_async_queue_push(devc->raw_data_queue, d);
                } else {
                    /* Queue full: Safely drop payload. Keep transfer->buffer 
                     * attached so libusb can reuse it on the next submission. */
                    sr_warn("Queue limit reached! Safely bypassing chunk of %zu bytes to prevent OOM.", len);
                }
            }

            devc->num_transfers_used -= 1;
            
            /* Resubmit transfer if we still need more samples and haven't aborted */
            if (!devc->acq_aborted && (devc->samples_got_nbytes + devc->num_transfers_used * devc->per_transfer_nbytes < devc->samples_need_nbytes)) {
                transfer->actual_length = 0;
                transfer->timeout = (TRANSFERS_DURATION_TOLERANCE + 1) * devc->per_transfer_duration * (devc->num_transfers_used + 2);
                ret = libusb_submit_transfer(transfer);
                if (ret) {
                    sr_dbg("Failed to submit transfer: %s", libusb_error_name(ret));
                    g_free(transfer->buffer);
                    libusb_free_transfer(transfer);
                } else {
                    devc->num_transfers_used += 1;
                }
            } else {
                /* Not resubmitting: Free data immediately to prevent shutdown leaks */
                g_free(transfer->buffer);
                libusb_free_transfer(transfer);
            }
        } break;

        case LIBUSB_TRANSFER_CANCELLED:
        case LIBUSB_TRANSFER_OVERFLOW:
        case LIBUSB_TRANSFER_STALL:
        case LIBUSB_TRANSFER_NO_DEVICE:
        default: {
            /* Crucial Fix: Any aborted, cancelled or failed transfer catches here, 
             * cleaning up both its tracking buffer and contextual hardware handle. */
            sr_dbg("Transfer returned with terminal status: %s.", libusb_error_name(transfer->status));
            g_free(transfer->buffer);
            libusb_free_transfer(transfer);
            devc->num_transfers_used -= 1;
        } break;
    }

    if (devc->num_transfers_used == 0) {
        devc->acq_aborted = 1;
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
    devc = sdi->priv;
    di = sdi->driver;
    drvc = di->context;

    if (devc->acq_aborted) {
        if (devc->num_transfers_used > 0) {
            /* Actively cancel running transfers; their cleanup will 
             * hit the case branches inside receive_transfer callback */
            for (size_t i = 0; i < NUM_MAX_TRANSFERS; ++i) {
                struct libusb_transfer *transfer = devc->transfers[i];
                if (transfer) {
                    libusb_cancel_transfer(transfer);
                    devc->transfers[i] = NULL;
                }
            }
        } else {
            /* Everything has completely unwound from the hardware layer.
             * Safe to stop data loop, join worker thread, and dismantle queue structures */
            if (devc->raw_data_handle_thread) {
                g_thread_join(devc->raw_data_handle_thread);
                devc->raw_data_handle_thread = NULL;
            }

            if (devc->raw_data_queue) {
                /* Flush out any remnants sitting inside queue */
                uint8_t *remain_data;
                while ((remain_data = g_async_queue_try_pop(devc->raw_data_queue)) != NULL) {
                    g_free(remain_data);
                }
                g_async_queue_unref(devc->raw_data_queue);
                devc->raw_data_queue = NULL;
            }

            sr_session_source_remove(sdi->session, -1 * (size_t)drvc->sr_ctx->libusb_ctx);
            sr_info("Bulk processing finalized. Total transfers handled: %" PRIu64 ".", (uint64_t)devc->num_transfers_completed);
        }
    }

    libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx, &(struct timeval){0, 1000}, NULL);

    return TRUE;
}

static gpointer raw_data_handle_thread_func(gpointer user_data)
{
    struct sr_dev_inst *sdi = user_data;
    struct dev_context *devc = sdi->priv;
    size_t len = devc->per_transfer_nbytes;

    while (1) {
        uint8_t *raw_data = g_async_queue_try_pop(devc->raw_data_queue);
        
        if (raw_data != NULL) {
            if (devc->trigger_fired) {
                devc->model->submit_raw_data(raw_data, len, sdi);
            }
            g_free(raw_data); 
        } else {
            /* Thread Exit Strategy: Only break if acquisition is declared dead 
             * by parent thread AND our data work pipeline is empty. */
            if (devc->acq_aborted) {
                std_session_send_df_end(sdi);
                break;
            }
            g_usleep(100);
        }
    }

    return NULL;
}

SR_PRIV int sipeed_slogic_acquisition_start(const struct sr_dev_inst *sdi)
{
    struct dev_context *devc;
    struct drv_context *drvc;
    struct sr_usb_dev_inst *usb;
    int ret;

    devc = sdi->priv;
    drvc = sdi->driver->context;
    usb = sdi->conn;

    if ((ret = devc->model->operation.remote_stop(sdi)) < 0) {
        sr_err("Unhandled `CMD_STOP`");
        return ret;
    }

    devc->samples_got_nbytes = 0;
    devc->samples_need_nbytes = devc->cur_limit_samples * devc->cur_samplechannel / 8;
    
    sr_info("Need %" PRIu64 "x %" PRIu64 "ch@%" PRIu64 "MHz in %" PRIu64 "ms.", 
            devc->cur_limit_samples,
            devc->cur_samplechannel, 
            devc->cur_samplerate / SR_MHZ(1),
            devc->cur_samplerate > 0 ? (1000 * devc->cur_limit_samples / devc->cur_samplerate) : 0
    );

    devc->per_transfer_duration = 125;
    devc->per_transfer_nbytes = devc->per_transfer_duration * devc->cur_samplerate * devc->cur_samplechannel / 8 / SR_KHZ(1);

    do {
        struct libusb_transfer *transfer = libusb_alloc_transfer(0);
        if (!transfer) {
            sr_err("Failed to allocate libusb transfer!");
            return SR_ERR_IO;
        }
        do {
            devc->per_transfer_nbytes = (devc->per_transfer_nbytes + (2 * 16 * 1024 - 1)) & ~(2 * 16 * 1024 - 1);
            if (devc->cur_samplerate * devc->cur_samplechannel > 0) {
                devc->per_transfer_duration = devc->per_transfer_nbytes * SR_KHZ(1) * 8 / (devc->cur_samplerate * devc->cur_samplechannel);
            }
            sr_dbg("Plan to receive %" PRIu64 " bytes per %" PRIu64 "ms...", devc->per_transfer_nbytes, devc->per_transfer_duration);
            
            uint8_t *dev_buf = g_malloc(devc->per_transfer_nbytes);
            if (!dev_buf) {
                sr_dbg("Failed to allocate memory: %" PRIu64 " bytes! Halving allocation boundary.", devc->per_transfer_nbytes);
                devc->per_transfer_nbytes >>= 1;
                continue;
            }

            libusb_fill_bulk_transfer(transfer, usb->devhdl, devc->model->ep_in,
                                        dev_buf, (int)devc->per_transfer_nbytes, NULL,
                                        NULL, 0);

            ret = libusb_submit_transfer(transfer);
            if (ret) {
                g_free(transfer->buffer);
                if (ret == LIBUSB_ERROR_NO_MEM) {
                    sr_dbg("Failed to submit transfer: %s!", libusb_error_name(ret));
                    devc->per_transfer_nbytes >>= 1;
                    continue;
                } else {
                    sr_err("Failed to submit transfer: %s!", libusb_error_name(ret));
                    libusb_free_transfer(transfer);
                    return SR_ERR_IO;
                }
            } else {
                ret = libusb_cancel_transfer(transfer);
                if (ret) {
                    sr_dbg("Failed to cancel transfer: %s!", libusb_error_name(ret));
                }
                libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx, &(struct timeval){3, 0}, NULL);
                g_free(transfer->buffer);

                devc->per_transfer_nbytes >>= 1;
                uint64_t divisor = (devc->cur_samplerate / SR_KHZ(1) * devc->cur_samplechannel / 8);
                devc->per_transfer_duration = divisor > 0 ? (devc->per_transfer_nbytes / divisor) : 1;
                
                if (devc->per_transfer_duration == 0) {
                    devc->per_transfer_duration = 1;
                }
                break;
            }
        } while (devc->per_transfer_nbytes > 32 * 1024);
        libusb_free_transfer(transfer);
        sr_info("Nice plan! :) => %" PRIu64 " bytes per %" PRIu64 "ms.", devc->per_transfer_nbytes, devc->per_transfer_duration);
    } while (0);

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

        unsigned int calculated_timeout = (unsigned int)((TRANSFERS_DURATION_TOLERANCE + 1) * devc->per_transfer_duration * (devc->num_transfers_used + 2));
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
    if (polling_interval == 0) {
        polling_interval = 1;
    }
    sr_session_source_add(sdi->session, -1 * (size_t)drvc->sr_ctx->libusb_ctx, 0, (int)polling_interval, handle_events, (void *)sdi);

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

    return SR_OK;
}