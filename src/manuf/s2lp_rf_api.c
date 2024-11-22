/*!*****************************************************************
 * \file    s2lp_rf_api.c
 * \brief   Sigfox S2LP RF API implementation.
 *******************************************************************
 * \copyright
 *
 * Copyright (c) 2022, UnaBiz SAS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1 Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  2 Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************/

#include "manuf/s2lp_rf_api.h"

// Sigfox EP library.
#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_error.h"
#include "sigfox_types.h"
#include "manuf/mcu_api.h"
#include "manuf/rf_api.h"
// S2LP library.
#include "S2LP_Commands.h"
#include "S2LP_Fifo.h"
#include "S2LP_General.h"
#include "S2LP_Gpio.h"
#include "S2LP_PktBasic.h"
#include "S2LP_Qi.h"
#include "S2LP_Radio.h"
#include "S2LP_Regs.h"
#include "S2LP_Timer.h"
// S2LP hardware driver.
#include "board/S2LP_CORE_SPI.h"
#include "board/s2lp_hw_api.h"

/*** S2LP RF API local macros ***/

#define S2LP_RF_API_FREQUENCY_MIN_HZ                826000000
#define S2LP_RF_API_FREQUENCY_MAX_HZ                958000000

#define S2LP_RF_API_TX_POWER_MAX_DBM                14

#define S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES       40
#define S2LP_RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES   (2 * S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES) // Size is twice to store PA and FDEV values.

#define S2LP_RF_API_POLAR_DATARATE_MULTIPLIER       8

#define S2LP_RF_API_FDEV_NEGATIVE                   0x7F // FDEV * (+1)
#define S2LP_RF_API_FDEV_POSITIVE                   0x81 // FDEV * (-1)

#define S2LP_RF_API_FIFO_BUFFER_FDEV_IDX            (S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES / 2) // Index where deviation is performed to invert phase.

#define S2LP_RF_API_FIFO_TX_ALMOST_EMPTY_THRESHOLD  (S2LP_RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES / 2) // Threshold set to the middle of a symbol.

#ifdef SIGFOX_EP_BIDIRECTIONAL
#define S2LP_RF_API_DL_PR_SIZE_BITS                 36
#define S2LP_RF_API_RX_BANDWIDTH_HZ                 3000
#define S2LP_RF_API_DOWNLINK_RSSI_THRESHOLD_DBM     -139
#endif

#define S2LP_RF_API_SEND_LATENCY_MARGIN_US          100

#define S2LP_RF_API_TIMEOUT_COUNT                   1000000

/*** S2LP RF API local structures ***/

/*******************************************************************/
typedef enum {
    S2LP_RF_API_STATE_READY = 0,
    S2LP_RF_API_STATE_TX_RAMP_UP,
    S2LP_RF_API_STATE_TX_BITSTREAM,
    S2LP_RF_API_STATE_TX_RAMP_DOWN,
    S2LP_RF_API_STATE_TX_PADDING_BIT,
    S2LP_RF_API_STATE_TX_END,
#ifdef SIGFOX_EP_BIDIRECTIONAL
    S2LP_RF_API_STATE_RX_START,
    S2LP_RF_API_STATE_RX,
#endif
    S2LP_RF_API_STATE_LAST
} S2LP_RF_API_state_t;

/*******************************************************************/
typedef union {
    struct {
        unsigned gpio_irq_enable :1;
        unsigned gpio_irq_process :1;
    } field;
    sfx_u8 all;
} S2LP_RF_API_flags_t;

/*******************************************************************/
typedef struct {
    // Common.
    RF_API_config_t config;
    S2LP_RF_API_state_t state;
    volatile S2LP_RF_API_flags_t flags;
    // TX.
    sfx_u8 tx_bitstream[SIGFOX_UL_BITSTREAM_SIZE_BYTES];
    sfx_u8 tx_bitstream_size_bytes;
    sfx_u8 tx_byte_idx;
    sfx_u8 tx_bit_idx;
    sfx_u8 tx_fdev;
    sfx_u8 tx_ramp_amplitude_profile[S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES];
    sfx_u8 tx_bit0_amplitude_profile[S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES];
    sfx_u8 symbol_fifo_buffer[S2LP_RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES];
    sfx_u8 ramp_fifo_buffer[S2LP_RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES];
#ifdef SIGFOX_EP_ASYNCHRONOUS
    RF_API_tx_cplt_cb_t tx_cplt_cb;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // RX.
    sfx_u8 dl_phy_content[SIGFOX_DL_PHY_CONTENT_SIZE_BYTES];
    sfx_s16 dl_rssi_dbm;
    sfx_u32 sync_word_u32;
#ifdef SIGFOX_EP_ASYNCHRONOUS
    RF_API_rx_data_received_cb_t rx_data_received_cb;
#endif
#endif
} S2LP_RF_API_context_t;

/*** S2LP RF API local global variables ***/

#ifdef SIGFOX_EP_VERBOSE
static const sfx_u8 S2LP_RF_API_VERSION[] = "v4.0";
#endif
// Amplitude profile tables for ramp and bit 0 transmission at maximum output power.
static const sfx_u8 S2LP_RF_API_RAMP_AMPLITUDE_PROFILE[S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 5, 7, 10, 14, 19, 25, 31, 39, 60, 220 };
static const sfx_u8 S2LP_RF_API_BIT0_AMPLITUDE_PROFILE[S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES] = { 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 5, 7, 10, 14, 19, 25, 31, 39, 60, 220, 220, 60, 39, 31, 25, 19, 14, 10, 7, 5, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1 };
#ifdef SIGFOX_EP_BIDIRECTIONAL
static const sfx_u8 S2LP_RF_API_SIGFOX_DL_FT[SIGFOX_DL_FT_SIZE_BYTES] = SIGFOX_DL_FT;
#endif
#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
// Latency values (for core clock at 32MHz and SPI interface at 8MHz).
static sfx_u32 S2LP_RF_API_LATENCY_MS[RF_API_LATENCY_LAST] = {
    0, // Wake-up (depends on HW latency).
    1, // TX init (600µs).
    0, // Send start (depends on bit rate and will be computed during init function).
    0, // Send stop (depends on bit rate and will be computed during init function).
    0, // TX de-init (30µs).
    0, // Sleep (depends on HW latency).
#ifdef SIGFOX_EP_BIDIRECTIONAL
    1, // RX init (1.1ms).
    5, // Receive start (150µs).
    10, // Receive stop (6.7ms).
    0, // RX de-init (30µs).
#endif
};
#endif
static S2LP_RF_API_context_t s2lp_rf_api_ctx;

/*** S2LP RF API local functions ***/

/*******************************************************************/
#define FEM_Operation(x) { }

/*******************************************************************/
static void _s2lp_gpio_irq_callback(void) {
    // Set flag if IRQ is enabled.
    if (s2lp_rf_api_ctx.flags.field.gpio_irq_enable != 0) {
        s2lp_rf_api_ctx.flags.field.gpio_irq_process = 1;
#ifdef SIGFOX_EP_ASYNCHRONOUS
        // Call Sigfox callback to process IRQ in main context.
        s2lp_rf_api_ctx.config.process_cb();
#endif
    }
}

/*******************************************************************/
static void _s2lp_reset_context(void) {
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Local variables.
    sfx_u8 idx = 0;
#endif
    // Init context.
    s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_READY;
    s2lp_rf_api_ctx.flags.all = 0;
    s2lp_rf_api_ctx.config.rc = SIGFOX_NULL;
    s2lp_rf_api_ctx.tx_bitstream_size_bytes = 0;
    s2lp_rf_api_ctx.tx_byte_idx = 0;
    s2lp_rf_api_ctx.tx_bit_idx = 0;
#ifdef SIGFOX_EP_ASYNCHRONOUS
    s2lp_rf_api_ctx.config.process_cb = SIGFOX_NULL;
    s2lp_rf_api_ctx.config.error_cb = SIGFOX_NULL;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Build synchronization word in integer format (0xB2270000).
    s2lp_rf_api_ctx.sync_word_u32 = 0;
    for (idx = 0; idx < SIGFOX_DL_FT_SIZE_BYTES; idx++) {
        s2lp_rf_api_ctx.sync_word_u32 |= S2LP_RF_API_SIGFOX_DL_FT[idx] << (8 * (3 - idx));
    }
#endif
}

/*******************************************************************/
static RF_API_status_t _s2lp_wait_for_state_switch(S2LPState new_state) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    uint32_t loop_count = 0;
    // Refresh MC_STATE until state is reached.
    while (g_xStatus.MC_STATE != new_state) {
        S2LPRefreshStatus();
        // Manage timeout.
        loop_count++;
        if (loop_count > S2LP_RF_API_TIMEOUT_COUNT) {
            SIGFOX_EXIT_ERROR((RF_API_status_t) S2LP_RF_API_ERROR_STATE_SWITCH_TIMEOUT);
        }
    }
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
static RF_API_status_t _s2lp_convert_gpio(S2LP_HW_API_gpio_t hw_api_gpio, S2LPGpioPin *s2lp_gpio) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    switch (hw_api_gpio) {
    case S2LP_HW_API_GPIO_0:
        (*s2lp_gpio) = S2LP_GPIO_0;
        break;
    case S2LP_HW_API_GPIO_1:
        (*s2lp_gpio) = S2LP_GPIO_1;
        break;
    case S2LP_HW_API_GPIO_2:
        (*s2lp_gpio) = S2LP_GPIO_2;
        break;
    case S2LP_HW_API_GPIO_3:
        (*s2lp_gpio) = S2LP_GPIO_3;
        break;
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) S2LP_RF_API_ERROR_GPIO);
        break;
    }
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
static void _s2lp_compute_amplitude_tables(sfx_s8 s2lp_tx_power_dbm) {
    // Local variables.
    sfx_u8 pa_value_min = S2LP_RF_API_RAMP_AMPLITUDE_PROFILE[S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES - 1];
    sfx_u8 pa_value_max = S2LP_RF_API_RAMP_AMPLITUDE_PROFILE[0];
    sfx_u8 new_pa_value_max = 0;
    sfx_u8 idx = 0;
    // Compute maximum PA value for the given output power.
    if (s2lp_tx_power_dbm >= S2LP_RF_API_TX_POWER_MAX_DBM) {
        new_pa_value_max = 1;
    }
    else {
        if (s2lp_tx_power_dbm >= 11) {
            new_pa_value_max = (15 - s2lp_tx_power_dbm);
        }
        else {
            new_pa_value_max = (25 - (2 * s2lp_tx_power_dbm));
        }
    }
    // Sub-symbols loop.
    for (idx = 0; idx < S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES; idx++) {
        // Compute amplitude tables.
        s2lp_rf_api_ctx.tx_ramp_amplitude_profile[idx] = pa_value_min - ((pa_value_min - S2LP_RF_API_RAMP_AMPLITUDE_PROFILE[idx]) * (pa_value_min - new_pa_value_max)) / (pa_value_min - pa_value_max);
        s2lp_rf_api_ctx.tx_bit0_amplitude_profile[idx] = pa_value_min - ((pa_value_min - S2LP_RF_API_BIT0_AMPLITUDE_PROFILE[idx]) * (pa_value_min - new_pa_value_max)) / (pa_value_min - pa_value_max);
    }
}

/*******************************************************************/
static RF_API_status_t _s2lp_internal_process(void) {
    // Local variables.
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    // Perform state machine.
    switch (s2lp_rf_api_ctx.state) {
    case S2LP_RF_API_STATE_READY:
        // Nothing to do.
        break;
    case S2LP_RF_API_STATE_TX_RAMP_UP:
        // Fill ramp-up.
        for (idx = 0; idx < S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES; idx++) {
            s2lp_rf_api_ctx.ramp_fifo_buffer[(2 * idx)] = 0; // Deviation.
            s2lp_rf_api_ctx.ramp_fifo_buffer[(2 * idx) + 1] = s2lp_rf_api_ctx.tx_ramp_amplitude_profile[S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES - idx - 1]; // PA output power.
        }
        // Load ramp-up buffer into FIFO.
        S2LPCmdStrobeFlushTxFifo();
        S2LPSpiWriteFifo(S2LP_RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES, (sfx_u8*) s2lp_rf_api_ctx.ramp_fifo_buffer);
        // Enable external GPIO interrupt.
        S2LPGpioIrqClearStatus();
        s2lp_rf_api_ctx.flags.field.gpio_irq_enable = 1;
        // Start radio.
        S2LPCmdStrobeLockTx();
#ifdef SIGFOX_EP_ERROR_CODES
        status = _s2lp_wait_for_state_switch(MC_STATE_LOCKON);
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
        _s2lp_wait_for_state_switch(MC_STATE_LOCKON);
#endif
        S2LPCmdStrobeTx();
#ifdef SIGFOX_EP_ERROR_CODES
        status = _s2lp_wait_for_state_switch(MC_STATE_TX);
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
        _s2lp_wait_for_state_switch(MC_STATE_TX);
#endif
        // Update state.
        s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_TX_BITSTREAM;
        break;
    case S2LP_RF_API_STATE_TX_BITSTREAM:
        // Check FIFO flag.
        if (S2LPGpioIrqCheckFlag(TX_FIFO_ALMOST_EMPTY) != 0) {
            // Check bit.
            if ((s2lp_rf_api_ctx.tx_bitstream[s2lp_rf_api_ctx.tx_byte_idx] & (1 << (7 - s2lp_rf_api_ctx.tx_bit_idx))) == 0) {
                // Phase shift and amplitude shaping required.
                s2lp_rf_api_ctx.tx_fdev = (s2lp_rf_api_ctx.tx_fdev == S2LP_RF_API_FDEV_NEGATIVE) ? S2LP_RF_API_FDEV_POSITIVE : S2LP_RF_API_FDEV_NEGATIVE; // Toggle deviation.
                for (idx = 0; idx < S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES; idx++) {
                    s2lp_rf_api_ctx.symbol_fifo_buffer[(2 * idx)] = (idx == S2LP_RF_API_FIFO_BUFFER_FDEV_IDX) ? s2lp_rf_api_ctx.tx_fdev : 0; // Deviation.
                    s2lp_rf_api_ctx.symbol_fifo_buffer[(2 * idx) + 1] = s2lp_rf_api_ctx.tx_bit0_amplitude_profile[idx]; // PA output power.
                }
            }
            else {
                // Constant CW.
                for (idx = 0; idx < S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES; idx++) {
                    s2lp_rf_api_ctx.symbol_fifo_buffer[(2 * idx)] = 0; // Deviation.
                    s2lp_rf_api_ctx.symbol_fifo_buffer[(2 * idx) + 1] = s2lp_rf_api_ctx.tx_bit0_amplitude_profile[0]; // PA output power.
                }
            }
            // Load bit into FIFO.
            S2LPSpiWriteFifo(S2LP_RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES, (sfx_u8*) s2lp_rf_api_ctx.symbol_fifo_buffer);
            // Increment bit index..
            s2lp_rf_api_ctx.tx_bit_idx++;
            if (s2lp_rf_api_ctx.tx_bit_idx >= 8) {
                // Reset bit index.
                s2lp_rf_api_ctx.tx_bit_idx = 0;
                // Increment byte index.
                s2lp_rf_api_ctx.tx_byte_idx++;
                // Check end of bitstream.
                if (s2lp_rf_api_ctx.tx_byte_idx >= (s2lp_rf_api_ctx.tx_bitstream_size_bytes)) {
                    s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_TX_RAMP_DOWN;
                }
            }
            // Clear flag.
            S2LPGpioIrqClearStatus();
        }
        break;
    case S2LP_RF_API_STATE_TX_RAMP_DOWN:
        // Check FIFO flag.
        if (S2LPGpioIrqCheckFlag(TX_FIFO_ALMOST_EMPTY) != 0) {
            // Fill ramp-down.
            for (idx = 0; idx < S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES; idx++) {
                s2lp_rf_api_ctx.ramp_fifo_buffer[(2 * idx)] = 0; // FDEV.
                s2lp_rf_api_ctx.ramp_fifo_buffer[(2 * idx) + 1] = s2lp_rf_api_ctx.tx_ramp_amplitude_profile[idx]; // PA output power for ramp-down.
            }
            // Load ramp-down buffer into FIFO.
            S2LPSpiWriteFifo(S2LP_RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES, (sfx_u8*) s2lp_rf_api_ctx.ramp_fifo_buffer);
            // Update state.
            s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_TX_PADDING_BIT;
            // Clear flag.
            S2LPGpioIrqClearStatus();
        }
        break;
    case S2LP_RF_API_STATE_TX_PADDING_BIT:
        // Check FIFO flag.
        if (S2LPGpioIrqCheckFlag(TX_FIFO_ALMOST_EMPTY) != 0) {
            // Padding bit to ensure last ramp down is completely transmitted.
            for (idx = 0; idx < S2LP_RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES; idx++) {
                s2lp_rf_api_ctx.symbol_fifo_buffer[idx] = 0x00;
            }
            // Load padding buffer into FIFO.
            S2LPSpiWriteFifo(S2LP_RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES, (sfx_u8*) s2lp_rf_api_ctx.symbol_fifo_buffer);
            // Update state.
            s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_TX_END;
            // Clear flag.
            S2LPGpioIrqClearStatus();
        }
        break;
    case S2LP_RF_API_STATE_TX_END:
        // Check FIFO flag.
        if (S2LPGpioIrqCheckFlag(TX_FIFO_ALMOST_EMPTY) != 0) {
            // Stop radio.
            S2LPCmdStrobeSabort();
            // Disable interrupt.
            s2lp_rf_api_ctx.flags.field.gpio_irq_enable = 0;
            S2LPGpioIrqClearStatus();
#ifdef SIGFOX_EP_ASYNCHRONOUS
            // Call TX completion callback.
            s2lp_rf_api_ctx.tx_cplt_cb();
#endif
            // Update state.
            s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_READY;
        }
        break;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case S2LP_RF_API_STATE_RX_START:
        S2LPCmdStrobeFlushRxFifo();
        // Enable external GPIO interrupt.
        S2LPGpioIrqClearStatus();
        s2lp_rf_api_ctx.flags.field.gpio_irq_enable = 1;
        // Start radio.
        S2LPCmdStrobeLockRx();
#ifdef SIGFOX_EP_ERROR_CODES
        status = _s2lp_wait_for_state_switch(MC_STATE_LOCKON);
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
        _s2lp_wait_for_state_switch(MC_STATE_LOCKON);
#endif
        S2LPCmdStrobeRx();
#ifdef SIGFOX_EP_ERROR_CODES
        status = _s2lp_wait_for_state_switch(MC_STATE_RX);
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
        _s2lp_wait_for_state_switch(MC_STATE_RX);
#endif
        // Update state.
        s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_RX;
        break;
    case S2LP_RF_API_STATE_RX:
        // Check RX data flag.
        if (S2LPGpioIrqCheckFlag(RX_DATA_READY) != 0) {
            // Read FIFO and RSSI.
            S2LPSpiReadFifo(SIGFOX_DL_PHY_CONTENT_SIZE_BYTES, (sfx_u8*) s2lp_rf_api_ctx.dl_phy_content);
            s2lp_rf_api_ctx.dl_rssi_dbm = (sfx_s16) S2LPRadioGetRssidBm();
            // Stop radio.
            S2LPCmdStrobeSabort();
            // Clear flag.
            S2LPGpioIrqClearStatus();
            // Disable interrupt.
            s2lp_rf_api_ctx.flags.field.gpio_irq_enable = 0;
#ifdef SIGFOX_EP_ASYNCHRONOUS
            // Call RX completion callback.
            s2lp_rf_api_ctx.rx_data_received_cb();
#endif
            // Update state.
            s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_READY;
        }
        break;
#endif /* SIGFOX_EP_BIDIRECTIONAL */
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) S2LP_RF_API_ERROR_STATE);
        break;
    }
errors:
    // Clear flag.
    s2lp_rf_api_ctx.flags.field.gpio_irq_process = 0;
    SIGFOX_RETURN();
}

/*** S2LP RF API functions ***/

#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
RF_API_status_t S2LP_RF_API_open(RF_API_config_t *rf_api_config) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
    S2LP_HW_API_status_t s2lp_hw_api_status = S2LP_HW_API_SUCCESS;
#endif
    S2LP_HW_API_config_t s2lp_hw_api_config;
    // Reset context.
    _s2lp_reset_context();
    // Save parameters.
    s2lp_rf_api_ctx.config.rc = (rf_api_config->rc);
#ifdef SIGFOX_EP_ASYNCHRONOUS
    s2lp_rf_api_ctx.config.process_cb = (rf_api_config->process_cb);
    s2lp_rf_api_ctx.config.error_cb = (rf_api_config->error_cb);
#endif
    // Init board.
    s2lp_hw_api_config.rc = (rf_api_config->rc);
    s2lp_hw_api_config.gpio_irq_callback = &_s2lp_gpio_irq_callback;
#ifdef SIGFOX_EP_ERROR_CODES
    s2lp_hw_api_status = S2LP_HW_API_open(&s2lp_hw_api_config);
    S2LP_HW_API_check_status((RF_API_status_t) S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
#else
    S2LP_HW_API_open(&s2lp_hw_api_config);
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
RF_API_status_t S2LP_RF_API_close(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    S2LP_HW_API_status_t s2lp_hw_api_status = S2LP_HW_API_SUCCESS;
#endif
    // Reset context.
    _s2lp_reset_context();
    // De-init board.
#ifdef SIGFOX_EP_ERROR_CODES
    s2lp_hw_api_status = S2LP_HW_API_close();
    S2LP_HW_API_check_status((RF_API_status_t) S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
#else
    S2LP_HW_API_close();
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
RF_API_status_t S2LP_RF_API_process(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    // Run internal process.
    while (s2lp_rf_api_ctx.flags.field.gpio_irq_process != 0) {
#ifdef SIGFOX_EP_ERROR_CODES
        status = _s2lp_internal_process();
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
        _s2lp_internal_process();
#endif
    }
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}
#endif

/*******************************************************************/
RF_API_status_t S2LP_RF_API_wake_up(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    S2LP_HW_API_status_t s2lp_hw_api_status = S2LP_HW_API_SUCCESS;
#endif
    // Exit shutdown.
#ifdef SIGFOX_EP_ERROR_CODES
    s2lp_hw_api_status = S2LP_HW_API_exit_shutdown();
    S2LP_HW_API_check_status((RF_API_status_t) S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
#else
    S2LP_HW_API_exit_shutdown();
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t S2LP_RF_API_sleep(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    S2LP_HW_API_status_t s2lp_hw_api_status = S2LP_HW_API_SUCCESS;
#endif
    // Enter shutdown.
#ifdef SIGFOX_EP_ERROR_CODES
    s2lp_hw_api_status = S2LP_HW_API_enter_shutdown();
    S2LP_HW_API_check_status((RF_API_status_t) S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
#else
    S2LP_HW_API_enter_shutdown();
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t S2LP_RF_API_init(RF_API_radio_parameters_t *radio_parameters) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
    S2LP_HW_API_status_t s2lp_hw_api_status = S2LP_HW_API_SUCCESS;
#endif
    sfx_u8 s2lp_status = 0;
    S2LP_radio_parameters_t s2lp_hw_radio_parameters;
    S2LP_HW_API_oscillator_type_t s2lp_xo_type;
    sfx_u32 s2lp_xo_frequency_hz = 0;
    ModeExtRef s2lp_ref_mode = MODE_EXT_XO;
    ModulationSelect s2lp_modulation = MOD_NO_MOD;
    sfx_u32 s2lp_datarate = 0;
    sfx_u32 s2lp_deviation = 0;
    sfx_s8 expected_tx_power_dbm = 0;
    sfx_s8 s2lp_tx_power_dbm = 0;
    S2LP_HW_API_gpio_t hw_api_gpio = 0;
    SGpioInit s2lp_gpio_init;
    sfx_u8 reg_value = 0;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    PktBasicInit downlink_pkt_init;
    SAfcInit afc_init;
    SSymClkRecInit clock_rec_init;
#endif
    // Go to ready state.
    S2LPCmdStrobeSres();
    S2LPCmdStrobeReady();
#ifdef SIGFOX_EP_ERROR_CODES
    status = _s2lp_wait_for_state_switch(MC_STATE_READY);
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
    _s2lp_wait_for_state_switch(MC_STATE_READY);
#endif
    // Crystal.
#ifdef SIGFOX_EP_ERROR_CODES
    s2lp_hw_api_status = S2LP_HW_API_get_oscillator(&s2lp_xo_type, &s2lp_xo_frequency_hz);
    S2LP_HW_API_check_status((RF_API_status_t) S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
#else
    S2LP_HW_API_get_oscillator(&s2lp_xo_type, &s2lp_xo_frequency_hz);
#endif
    s2lp_ref_mode = (s2lp_xo_type == S2LP_HW_API_OSCILLATOR_TYPE_QUARTZ) ? MODE_EXT_XO : MODE_EXT_XIN;
    S2LPRadioSetXtalFrequency(s2lp_xo_frequency_hz);
    S2LPGeneralSetExtRef(s2lp_ref_mode);
    S2LPRadioSetDigDiv(S_ENABLE);
    S2LPTimerCalibrationRco(S_ENABLE);
    // Frequency.
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    if (((radio_parameters->frequency_hz) < S2LP_RF_API_FREQUENCY_MIN_HZ) || ((radio_parameters->frequency_hz) > S2LP_RF_API_FREQUENCY_MAX_HZ)) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) S2LP_RF_API_ERROR_FREQUENCY);
    }
#endif
    s2lp_status = S2LPRadioSetFrequencyBase(radio_parameters->frequency_hz);
    if (s2lp_status != 0) SIGFOX_EXIT_ERROR((RF_API_status_t) S2LP_RF_API_ERROR_FREQUENCY);
    // Modulation and bit rate.
    switch (radio_parameters->modulation) {
    case RF_API_MODULATION_NONE:
        s2lp_modulation = MOD_NO_MOD;
        s2lp_datarate = (radio_parameters->bit_rate_bps);
#ifdef SIGFOX_EP_BIDIRECTIONAL
        s2lp_deviation = (radio_parameters->deviation_hz);
#endif
        break;
    case RF_API_MODULATION_DBPSK:
        s2lp_modulation = MOD_POLAR;
        s2lp_datarate = ((radio_parameters->bit_rate_bps) * S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES) / (S2LP_RF_API_POLAR_DATARATE_MULTIPLIER);
        s2lp_deviation = ((radio_parameters->bit_rate_bps) * S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES) / (2);
        break;
    case RF_API_MODULATION_GFSK:
        s2lp_modulation = MOD_2GFSK_BT1;
        s2lp_datarate = (radio_parameters->bit_rate_bps);
#ifdef SIGFOX_EP_BIDIRECTIONAL
        s2lp_deviation = (radio_parameters->deviation_hz);
#endif
        break;
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) S2LP_RF_API_ERROR_MODULATION);
        break;
    }
    S2LPRadioSetModulation(s2lp_modulation);
    S2LPRadioSetDatarate(s2lp_datarate);
    S2LPRadioSetFrequencyDev(s2lp_deviation);
    // Disable all IRQ.
    S2LPGpioIrqDeInit(SIGFOX_NULL);
    // Configure IRQ GPIO.
#ifdef SIGFOX_EP_ERROR_CODES
    s2lp_hw_api_status = S2LP_HW_API_get_gpio(S2LP_HW_API_SIGNAL_NIRQ, &hw_api_gpio);
    S2LP_HW_API_check_status((RF_API_status_t) S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
    status = _s2lp_convert_gpio(hw_api_gpio, &(s2lp_gpio_init.xS2LPGpioPin));
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
    S2LP_HW_API_get_gpio(S2LP_HW_API_SIGNAL_NIRQ, &hw_api_gpio);
    _s2lp_convert_gpio(hw_api_gpio, &(s2lp_gpio_init.xS2LPGpioPin));
#endif
    s2lp_gpio_init.xS2LPGpioMode = S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP;
    s2lp_gpio_init.xS2LPGpioIO = S2LP_GPIO_DIG_OUT_IRQ;
    S2LPGpioInit(&s2lp_gpio_init);
    // Configure specific registers.
    // Note: SPMS configuration is performed by the TX/RX command strobe.
    switch (radio_parameters->rf_mode) {
    case RF_API_MODE_TX:
        // Configure IRQ mask.
        S2LPGpioIrqConfig(TX_FIFO_ALMOST_EMPTY, S_ENABLE);
        S2LPFifoMuxRxFifoIrqEnable(S_DISABLE);
        // FIFO.
        S2LPPacketHandlerSetTxMode(DIRECT_TX_FIFO_MODE);
        S2LPFifoSetAlmostEmptyThresholdTx(S2LP_RF_API_FIFO_TX_ALMOST_EMPTY_THRESHOLD);
        // PA configuration.
        reg_value = 0x00; // Disable all features and select slot 0.
        S2LPSpiWriteRegisters(PA_POWER0_ADDR, 1, &reg_value);
        S2LPRadioSetAutoRampingMode(S_DISABLE);
        S2LPSpiReadRegisters(MOD1_ADDR, 1, &reg_value);
        reg_value |= PA_INTERP_EN_REGMASK; // Enable interpolator.
        S2LPSpiWriteRegisters(MOD1_ADDR, 1, &reg_value);
        // Get effective output power to program.
#ifdef SIGFOX_EP_TX_POWER_DBM_EIRP
        expected_tx_power_dbm = SIGFOX_EP_TX_POWER_DBM_EIRP;
#else
        expected_tx_power_dbm = (radio_parameters->tx_power_dbm_eirp);
#endif
#ifdef SIGFOX_EP_ERROR_CODES
        s2lp_hw_api_status = S2LP_HW_API_get_tx_power(expected_tx_power_dbm, &s2lp_tx_power_dbm);
        S2LP_HW_API_check_status((RF_API_status_t) S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
#else
        S2LP_HW_API_get_tx_power(expected_tx_power_dbm, &s2lp_tx_power_dbm);
#endif
        // Check returned value.
        if (s2lp_tx_power_dbm > S2LP_RF_API_TX_POWER_MAX_DBM) {
            SIGFOX_EXIT_ERROR((RF_API_status_t) S2LP_RF_API_ERROR_TX_POWER);
        }
        // Set output power.
        if ((radio_parameters->modulation) == RF_API_MODULATION_NONE) {
            S2LPRadioSetPALeveldBm(0, s2lp_tx_power_dbm);
        }
        else {
            _s2lp_compute_amplitude_tables(s2lp_tx_power_dbm);
        }
#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
        // Start latency = 1 symbol of ramp-up.
        S2LP_RF_API_LATENCY_MS[RF_API_LATENCY_SEND_START] = ((1000 - S2LP_RF_API_SEND_LATENCY_MARGIN_US) / ((sfx_u32) (radio_parameters->bit_rate_bps)));
        // Stop latency = 1 symbol of ramp-down + half of padding symbol (since IRQ is raised at the middle of the symbol).
        S2LP_RF_API_LATENCY_MS[RF_API_LATENCY_SEND_STOP] =  ((1500 - S2LP_RF_API_SEND_LATENCY_MARGIN_US) / ((sfx_u32) (radio_parameters->bit_rate_bps)));
#endif
        break;
#if (defined SIGFOX_EP_BIDIRECTIONAL) || ((defined SIGFOX_EP_REGULATORY && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT)))
    case RF_API_MODE_RX:
#ifdef SIGFOX_EP_BIDIRECTIONAL
        // Configure IRQ mask.
        S2LPGpioIrqConfig(RX_DATA_READY, S_ENABLE);
        S2LPFifoMuxRxFifoIrqEnable(S_ENABLE);
        // FIFO.
        S2LPPacketHandlerSetRxMode(NORMAL_RX_MODE);
        // RX bandwidth.
        S2LPRadioSetChannelBW(S2LP_RF_API_RX_BANDWIDTH_HZ);
        // RSSI threshold.
        S2LPRadioSetRssiThreshdBm(S2LP_RF_API_DOWNLINK_RSSI_THRESHOLD_DBM);
        // Disable equalization, antenna switching and carrier sense.
        S2LPRadioSetIsiEqualizationMode(ISI_EQUALIZATION_DISABLED);
        S2LPRadioAntennaSwitching(S_DISABLE);
        S2LPRadioCsBlanking(S_DISABLE);
        // Downlink packet structure.
        downlink_pkt_init.xPreambleLength = S2LP_RF_API_DL_PR_SIZE_BITS;
        downlink_pkt_init.xSyncLength = (SIGFOX_DL_FT_SIZE_BYTES * 8);
        downlink_pkt_init.lSyncWords = s2lp_rf_api_ctx.sync_word_u32;
        downlink_pkt_init.xFixVarLength = S_DISABLE;
        downlink_pkt_init.cExtendedPktLenField = S_DISABLE;
        downlink_pkt_init.xCrcMode = PKT_NO_CRC;
        downlink_pkt_init.xAddressField = S_DISABLE;
        downlink_pkt_init.xFec = S_DISABLE;
        downlink_pkt_init.xDataWhitening = S_DISABLE;
        S2LPPktBasicInit(&downlink_pkt_init);
        S2LPPktBasicSetPayloadLength(SIGFOX_DL_PHY_CONTENT_SIZE_BYTES);
        // Disable AFC.
        afc_init.xAfcEnable = 0;
        afc_init.xAfcFreezeOnSync = 1;
        afc_init.xAfcMode = AFC_MODE_LOOP_CLOSED_ON_SLICER;
        afc_init.cAfcSlowGain = 5;
        afc_init.cAfcFastGain = 2;
        afc_init.cAfcFastPeriod = 24;
        S2LPRadioAfcInit(&afc_init);
        // Configure clock recovery.
        clock_rec_init.cClkRecPGainSlow = 1;
        clock_rec_init.xSClkRecMode = CLKREC_DLL_MODE;
        clock_rec_init.cClkRecIGainSlow = 0;
        clock_rec_init.cClkRecPGainFast = 3;
        clock_rec_init.cClkRec16SymPostFlt = S_ENABLE;
        clock_rec_init.cClkRecIGainFast = 0;
        S2LPRadioSymClkRecoverInit(&clock_rec_init);
        break;
#endif /* SIGFOX_EP_BIDIRECTIONAL */
#endif
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) S2LP_RF_API_ERROR_MODE);
        break;
    }
    // Optional init on hardware side.
    s2lp_hw_radio_parameters.rf_mode = (radio_parameters->rf_mode);
    s2lp_hw_radio_parameters.expected_tx_power_dbm = expected_tx_power_dbm;
#ifdef SIGFOX_EP_ERROR_CODES
    s2lp_hw_api_status = S2LP_HW_API_init(&s2lp_hw_radio_parameters);
    S2LP_HW_API_check_status((RF_API_status_t) S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
#else
    S2LP_HW_API_init(&s2lp_hw_radio_parameters);
#endif
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t S2LP_RF_API_de_init(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    S2LP_HW_API_status_t s2lp_hw_api_status = S2LP_HW_API_SUCCESS;
#endif
    // Disable external GPIO interrupt.
    s2lp_rf_api_ctx.flags.all = 0;
    // Stop radio.
    S2LPCmdStrobeSabort();
    S2LPCmdStrobeSres();
    S2LPCmdStrobeReady();
#ifdef SIGFOX_EP_ERROR_CODES
    status = _s2lp_wait_for_state_switch(MC_STATE_READY);
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
    _s2lp_wait_for_state_switch(MC_STATE_READY);
#endif
    // Optional release on hardware side.
#ifdef SIGFOX_EP_ERROR_CODES
    s2lp_hw_api_status = S2LP_HW_API_de_init();
    S2LP_HW_API_check_status((RF_API_status_t) S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
#else
    S2LP_HW_API_de_init();
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t S2LP_RF_API_send(RF_API_tx_data_t *tx_data) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    sfx_u8 idx = 0;
    // Store TX data.
    s2lp_rf_api_ctx.tx_bitstream_size_bytes = (tx_data->bitstream_size_bytes);
    for (idx = 0; idx < (s2lp_rf_api_ctx.tx_bitstream_size_bytes); idx++) {
        s2lp_rf_api_ctx.tx_bitstream[idx] = (tx_data->bitstream)[idx];
    }
#ifdef SIGFOX_EP_ASYNCHRONOUS
    // Store callback.
    s2lp_rf_api_ctx.tx_cplt_cb = (tx_data->cplt_cb);
#endif
    // Init state.
    s2lp_rf_api_ctx.tx_bit_idx = 0;
    s2lp_rf_api_ctx.tx_byte_idx = 0;
    s2lp_rf_api_ctx.flags.all = 0;
    // Trigger TX.
    s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_TX_RAMP_UP;
#ifdef SIGFOX_EP_ERROR_CODES
    status = _s2lp_internal_process();
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
    _s2lp_internal_process();
#endif
#ifndef SIGFOX_EP_ASYNCHRONOUS
    // Wait for transmission to complete.
    while (s2lp_rf_api_ctx.state != S2LP_RF_API_STATE_READY) {
        // Wait for GPIO interrupt.
        while (s2lp_rf_api_ctx.flags.field.gpio_irq_process == 0);
        // Call process function.
#ifdef SIGFOX_EP_ERROR_CODES
        status = _s2lp_internal_process();
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
        _s2lp_internal_process();
#endif
    }
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t S2LP_RF_API_receive(RF_API_rx_data_t *rx_data) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#ifndef SIGFOX_EP_ASYNCHRONOUS
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#endif
#endif
#ifdef SIGFOX_EP_ASYNCHRONOUS
    // Store callback.
    s2lp_rf_api_ctx.rx_data_received_cb = (rx_data->data_received_cb);
#else
    sfx_bool dl_timeout = SIGFOX_FALSE;
    // Reset flag.
    (rx_data->data_received) = SIGFOX_FALSE;
#endif
    // Init state.
    s2lp_rf_api_ctx.flags.all = 0;
    // Trigger RX.
    s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_RX_START;
#ifdef SIGFOX_EP_ERROR_CODES
    status = _s2lp_internal_process();
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
    _s2lp_internal_process();
#endif
#ifndef SIGFOX_EP_ASYNCHRONOUS
    // Wait for reception to complete.
    while (s2lp_rf_api_ctx.state != S2LP_RF_API_STATE_READY) {
        // Wait for GPIO interrupt.
        while (s2lp_rf_api_ctx.flags.field.gpio_irq_process == 0) {
            // Check timeout.
#ifdef SIGFOX_EP_ERROR_CODES
            mcu_api_status = MCU_API_timer_status(MCU_API_TIMER_INSTANCE_T_RX, &dl_timeout);
            MCU_API_check_status((RF_API_status_t) S2LP_RF_API_ERROR_DRIVER_MCU_API);
#else
            MCU_API_timer_status(MCU_API_TIMER_INSTANCE_T_RX, &dl_timeout);
#endif
            // Exit if timeout.
            if (dl_timeout == SIGFOX_TRUE) {
                // Stop radio.
                S2LPCmdStrobeSabort();
                goto errors;
            }
        }
        // Call process function.
#ifdef SIGFOX_EP_ERROR_CODES
        status = _s2lp_internal_process();
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
        _s2lp_internal_process();
#endif
    }
    // Update status flag.
    (rx_data->data_received) = SIGFOX_TRUE;
#endif
#if (defined SIGFOX_EP_ERROR_CODES) || !(defined SIGFOX_EP_ASYNCHRONOUS)
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t S2LP_RF_API_get_dl_phy_content_and_rssi(sfx_u8 *dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16 *dl_rssi_dbm) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameters.
    if ((dl_phy_content == SIGFOX_NULL) || (dl_rssi_dbm == SIGFOX_NULL)) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) S2LP_RF_API_ERROR_NULL_PARAMETER);
    }
    if (dl_phy_content_size > SIGFOX_DL_PHY_CONTENT_SIZE_BYTES) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) S2LP_RF_API_ERROR_BUFFER_SIZE);
    }
#endif
    // Fill data.
    for (idx = 0; idx < dl_phy_content_size; idx++) {
        dl_phy_content[idx] = s2lp_rf_api_ctx.dl_phy_content[idx];
    }
    (*dl_rssi_dbm) = (sfx_s16) s2lp_rf_api_ctx.dl_rssi_dbm;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT)
/*******************************************************************/
RF_API_status_t S2LP_RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params) {
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    /* To be implemented by the device manufacturer */
    SIGFOX_UNUSED(carrier_sense_params);
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*******************************************************************/
RF_API_status_t S2LP_RF_API_get_latency(RF_API_latency_t latency_type, sfx_u32 *latency_ms) {
    // Local variables.
    sfx_u32 hw_api_latency[S2LP_HW_API_LATENCY_LAST];
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
    S2LP_HW_API_status_t s2lp_hw_api_status = S2LP_HW_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Reset result.
    (*latency_ms) = 0;
    // Check parameter.
    if (latency_type >= RF_API_LATENCY_LAST) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) S2LP_RF_API_ERROR_LATENCY_TYPE);
    }
#endif
    // Set base latency.
    (*latency_ms) = S2LP_RF_API_LATENCY_MS[latency_type];
    // Read hardware latencies.
    for (idx = 0; idx < S2LP_HW_API_LATENCY_LAST; idx++) {
        // Reset value.
        hw_api_latency[idx] = 0;
#ifdef SIGFOX_EP_ERROR_CODES
        s2lp_hw_api_status = S2LP_HW_API_get_latency(idx, &(hw_api_latency[idx]));
        S2LP_HW_API_check_status((RF_API_status_t) S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
#else
        S2LP_HW_API_get_latency(idx, &(hw_api_latency[idx]));
#endif
    }
    // Add HW API latencies.
    if (latency_type == RF_API_LATENCY_WAKE_UP) {
        (*latency_ms) += hw_api_latency[S2LP_HW_API_LATENCY_EXIT_SHUTDOWN];
    }
    if (latency_type == RF_API_LATENCY_SLEEP) {
        (*latency_ms) += hw_api_latency[S2LP_HW_API_LATENCY_ENTER_SHUTDOWN];
    }
    if (latency_type == RF_API_LATENCY_INIT_TX) {
        (*latency_ms) += hw_api_latency[S2LP_HW_API_LATENCY_INIT_TX];
    }
    if (latency_type == RF_API_LATENCY_DE_INIT_TX) {
        (*latency_ms) += hw_api_latency[S2LP_HW_API_LATENCY_DE_INIT_TX];
    }
#ifdef SIGFOX_EP_BIDIRECTIONAL
    if (latency_type == RF_API_LATENCY_INIT_RX) {
        (*latency_ms) += hw_api_latency[S2LP_HW_API_LATENCY_INIT_RX];
    }
    if (latency_type == RF_API_LATENCY_DE_INIT_RX) {
        (*latency_ms) += hw_api_latency[S2LP_HW_API_LATENCY_DE_INIT_RX];
    }
#endif
#if (defined SIGFOX_EP_PARAMETERS_CHECK) || (defined SIGFOX_EP_ERROR_CODES)
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_CERTIFICATION
/*******************************************************************/
RF_API_status_t S2LP_RF_API_start_continuous_wave(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    // Modulated CW is not supported for now.
    if (S2LPRadioGetModulation() != MOD_NO_MOD) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) S2LP_RF_API_ERROR_MODULATION);
    }
    // Start radio.
    S2LPCmdStrobeLockTx();
#ifdef SIGFOX_EP_ERROR_CODES
    status = _s2lp_wait_for_state_switch(MC_STATE_LOCKON);
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
    _s2lp_wait_for_state_switch(MC_STATE_LOCKON);
#endif
    S2LPCmdStrobeTx();
#ifdef SIGFOX_EP_ERROR_CODES
    status = _s2lp_wait_for_state_switch(MC_STATE_TX);
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
    _s2lp_wait_for_state_switch(MC_STATE_TX);
#endif
errors:
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_VERBOSE
/*******************************************************************/
RF_API_status_t S2LP_RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    (*version) = (sfx_u8*) S2LP_RF_API_VERSION;
    (*version_size_char) = (sfx_u8) sizeof(S2LP_RF_API_VERSION);
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_ERROR_CODES
/*******************************************************************/
void S2LP_RF_API_error(void) {
    // Turn radio off.
    S2LP_RF_API_de_init();
    S2LP_RF_API_sleep();
}
#endif

/*** S2LP RF API functions mapping ***/

#ifndef SIGFOX_EP_DYNAMIC_RF_API

#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
RF_API_status_t RF_API_open(RF_API_config_t *rf_api_config) {
    return S2LP_RF_API_open(rf_api_config);
}
#endif

#ifdef SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
RF_API_status_t RF_API_close(void) {
    return S2LP_RF_API_close();
}
#endif

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
RF_API_status_t RF_API_process(void) {
    return S2LP_RF_API_process();
}
#endif

/*******************************************************************/
RF_API_status_t RF_API_wake_up(void) {
    return S2LP_RF_API_wake_up();
}

/*******************************************************************/
RF_API_status_t RF_API_sleep(void) {
    return S2LP_RF_API_sleep();
}

/*******************************************************************/
RF_API_status_t RF_API_init(RF_API_radio_parameters_t *radio_parameters) {
    return S2LP_RF_API_init(radio_parameters);
}

/*******************************************************************/
RF_API_status_t RF_API_de_init(void) {
    return S2LP_RF_API_de_init();
}

/*******************************************************************/
RF_API_status_t RF_API_send(RF_API_tx_data_t *tx_data) {
    return S2LP_RF_API_send(tx_data);
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_receive(RF_API_rx_data_t *rx_data) {
    return S2LP_RF_API_receive(rx_data);
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_get_dl_phy_content_and_rssi(sfx_u8 *dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16 *dl_rssi_dbm) {
    return S2LP_RF_API_get_dl_phy_content_and_rssi(dl_phy_content, dl_phy_content_size, dl_rssi_dbm);
}
#endif

#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT)
/*******************************************************************/
RF_API_status_t RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params) {
    return S2LP_RF_API_carrier_sense(carrier_sense_params);
}
#endif

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*******************************************************************/
RF_API_status_t RF_API_get_latency(RF_API_latency_t latency_type, sfx_u32 *latency_ms) {
    return S2LP_RF_API_get_latency(latency_type, latency_ms);
}
#endif

#ifdef SIGFOX_EP_CERTIFICATION
/*******************************************************************/
RF_API_status_t RF_API_start_continuous_wave(void) {
    return S2LP_RF_API_start_continuous_wave();
}
#endif

#ifdef SIGFOX_EP_VERBOSE
/*******************************************************************/
RF_API_status_t RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
    return S2LP_RF_API_get_version(version, version_size_char);
}
#endif

#ifdef SIGFOX_EP_ERROR_CODES
/*******************************************************************/
void RF_API_error(void) {
    S2LP_RF_API_error();
}
#endif

#endif /* SIGFOX_EP_DYNAMIC_RF_API */
