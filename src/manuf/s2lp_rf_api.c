/*!*****************************************************************
 * \file    s2lp_rf_api.c
 * \brief   Sigfox s2lp RF api.
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
#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_error.h"
#include "sigfox_types.h"
#include "manuf/mcu_api.h"
#include "manuf/rf_api.h"
// S2LP library.
#include "MCU_Interface_template.h"
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
#include "board/s2lp_hw_api.h"

/*** S2LP RF API local macros ***/

#define S2LP_RF_API_RAMP_PROFILE_SIZE_BYTES			40
#define S2LP_RF_API_RAMP_FIFO_BUFFER_SIZE_BYTES		(2 * S2LP_RF_API_RAMP_PROFILE_SIZE_BYTES) // Size is twice to store PA and FDEV values.

#define S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES		40
#define S2LP_RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES	(2 * S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES) // Size is twice to store PA and FDEV values.

#define S2LP_RF_API_POLAR_DATARATE_MULTIPLIER		8

#define S2LP_RF_API_FDEV_NEGATIVE					0x7F // fdev * (+1)
#define S2LP_RF_API_FDEV_POSITIVE					0x81 // fdev * (-1)

#define S2LP_RF_API_FIFO_BUFFER_FDEV_IDX			(S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES / 2) // Index where deviation is performed to invert phase.

#ifdef BIDIRECTIONAL
#define S2LP_RF_API_DL_PR_SIZE_BITS					36
#define S2LP_RF_API_RX_BANDWIDTH_HZ					3000
#define S2LP_RF_API_DOWNLINK_RSSI_THRESHOLD_DBM		-139
#endif

#define S2LP_FIFO_SIZE_BYTES						128

#ifdef VERBOSE
static const sfx_u8 S2LP_RF_API_VERSION[] = 		"v1.0";
#endif

// Ramp profile table is written for ramp-down direction (reverse table for ramp up).
// Ampltude profile table contains whole bit 0 transmission.
#if !(defined TX_POWER_DBM_EIRP) || (TX_POWER_DBM_EIRP == 0)
static const sfx_u8 RF_API_RAMP_AMPLITUDE_PROFILE_00_DBM[S2LP_RF_API_RAMP_PROFILE_SIZE_BYTES] = {29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30, 30, 30, 30, 31, 31, 31, 31, 32, 32, 32, 33, 33, 34, 35, 36, 38, 40, 42, 47, 56, 80, 120, 220};
static const sfx_u8 RF_API_BIT0_AMPLITUDE_PROFILE_00_DBM[S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES] = {29, 29, 29, 29, 29, 29, 30, 30, 30, 31, 31, 32, 33, 34, 35, 36, 38, 42, 54, 220, 220, 54, 42, 38, 36, 35, 33, 32, 31, 31, 30, 30, 30, 29, 29, 29, 29, 29, 29, 29};
#endif
#if !(defined TX_POWER_DBM_EIRP) || (TX_POWER_DBM_EIRP == 3)
static const sfx_u8 RF_API_RAMP_AMPLITUDE_PROFILE_03_DBM[S2LP_RF_API_RAMP_PROFILE_SIZE_BYTES] = {23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24, 24, 25, 25, 25, 26, 26, 27, 28, 29, 30, 31, 32, 33, 35, 37, 39, 42, 47, 56, 80, 120, 220};
static const sfx_u8 RF_API_BIT0_AMPLITUDE_PROFILE_03_DBM[S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES] = {23, 23, 23, 23, 23, 23, 24, 24, 25, 26, 27, 28, 29, 31, 33, 35, 38, 42, 54, 220, 220, 54, 42, 38, 35, 33, 31, 29, 28, 27, 26, 25, 24, 24, 23, 23, 23, 23, 23, 23};
#endif
#if !(defined TX_POWER_DBM_EIRP) || (TX_POWER_DBM_EIRP == 5)
static const sfx_u8 RF_API_RAMP_AMPLITUDE_PROFILE_05_DBM[S2LP_RF_API_RAMP_PROFILE_SIZE_BYTES] = {19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 21, 21, 21, 22, 22, 23, 23, 24, 25, 26, 27, 28, 29, 31, 33, 35, 38, 42, 47, 56, 80, 120, 220};
static const sfx_u8 RF_API_BIT0_AMPLITUDE_PROFILE_05_DBM[S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES] = {19, 19, 19, 19, 19, 19, 20, 20, 21, 22, 23, 24, 26, 28, 30, 33, 36, 42, 54, 220, 220, 54, 42, 36, 33, 30, 28, 26, 24, 23, 22, 21, 20, 20, 19, 19, 19, 19, 19, 19};
#endif
#if !(defined TX_POWER_DBM_EIRP) || (TX_POWER_DBM_EIRP == 7)
static const sfx_u8 RF_API_RAMP_AMPLITUDE_PROFILE_07_DBM[S2LP_RF_API_RAMP_PROFILE_SIZE_BYTES] = {15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 17, 17, 17, 18, 18, 19, 20, 21, 22, 23, 24, 25, 26, 28, 30, 32, 35, 38, 42, 47, 56, 80, 120, 220};
static const sfx_u8 RF_API_BIT0_AMPLITUDE_PROFILE_07_DBM[S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES] = {15, 15, 15, 15, 15, 15, 16, 16, 17, 18, 20, 22, 24, 26, 28, 32, 36, 42, 54, 220, 220, 54, 42, 36, 32, 28, 26, 24, 22, 20, 18, 17, 16, 16, 15, 15, 15, 15, 15, 15};
#endif
#if !(defined TX_POWER_DBM_EIRP) || (TX_POWER_DBM_EIRP == 10)
static const sfx_u8 RF_API_RAMP_AMPLITUDE_PROFILE_10_DBM[S2LP_RF_API_RAMP_PROFILE_SIZE_BYTES] = {9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 12, 13, 14, 15, 16, 17, 18, 19, 21, 23, 25, 27, 29, 32, 35, 38, 42, 47, 56, 80, 120, 220};
static const sfx_u8 RF_API_BIT0_AMPLITUDE_PROFILE_10_DBM[S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES] = {9, 9, 9, 9, 9, 9, 10, 10, 11, 12, 14, 16, 18, 21, 24, 28, 32, 39, 54, 220, 220, 54, 39, 32, 28, 24, 21, 18, 16, 14, 12, 11, 10, 10, 9, 9, 9, 9, 9, 9};
#endif
#if !(defined TX_POWER_DBM_EIRP) || (TX_POWER_DBM_EIRP == 14)
static const sfx_u8 RF_API_RAMP_AMPLITUDE_PROFILE_14_DBM[S2LP_RF_API_RAMP_PROFILE_SIZE_BYTES] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 11, 13, 15, 17, 20, 22, 24, 27, 30, 34, 39, 45, 54, 80, 120, 220};
static const sfx_u8 RF_API_BIT0_AMPLITUDE_PROFILE_14_DBM[S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES] = {1, 1, 1, 1, 1, 1, 2, 2, 3, 4, 6, 8, 11, 15, 20, 24, 30, 39, 54, 220, 220, 54, 39, 30, 24, 20, 15, 11, 8, 6, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1};
#endif

/*** S2LP RF API local structures ***/

/*******************************************************************/
typedef enum {
	S2LP_RF_API_STATE_READY = 0,
	S2LP_RF_API_STATE_TX_RAMP_UP,
	S2LP_RF_API_STATE_TX_BITSTREAM,
	S2LP_RF_API_STATE_TX_RAMP_DOWN,
	S2LP_RF_API_STATE_TX_PADDING_BIT,
	S2LP_RF_API_STATE_TX_END,
#ifdef BIDIRECTIONAL
	S2LP_RF_API_STATE_RX_START,
	S2LP_RF_API_STATE_RX,
#endif
	S2LP_RF_API_STATE_LAST
} S2LP_RF_API_state_t;

/*******************************************************************/
typedef union {
	struct {
		unsigned irq_enable : 1;
		unsigned irq_process : 1;
	} field;
	sfx_u8 all;
} S2LP_RF_API_flags_t;

/*******************************************************************/
typedef struct {
	// Common.
	RF_API_config_t config;
	S2LP_RF_API_state_t state;
	volatile S2LP_RF_API_flags_t flags;
	sfx_u8 symbol_fifo_buffer[S2LP_RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES];
	sfx_u8 ramp_fifo_buffer[S2LP_RF_API_RAMP_FIFO_BUFFER_SIZE_BYTES];
	// TX.
	sfx_u8 tx_bitstream[SIGFOX_UL_BITSTREAM_SIZE_BYTES];
	sfx_u8 tx_bitstream_size_bytes;
	sfx_u8 tx_byte_idx;
	sfx_u8 tx_bit_idx;
	sfx_u8 tx_fdev;
	sfx_u8* tx_ramp_amplitude_profile_ptr;
	sfx_u8* tx_bit_0_amplitude_profile_ptr;
#ifdef ASYNCHRONOUS
	RF_API_tx_cplt_cb_t tx_cplt_cb;
#endif
#ifdef BIDIRECTIONAL
	// RX.
	sfx_u8 dl_phy_content[SIGFOX_DL_PHY_CONTENT_SIZE_BYTES];
	int16_t dl_rssi_dbm;
	sfx_u32 sync_word_u32;
#ifdef ASYNCHRONOUS
	RF_API_rx_data_received_cb_t rx_data_received_cb;
#endif
#endif
} S2LP_RF_API_context_t;

/*** S2LP RF API local global variables ***/

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
// Latency values.
static sfx_u32 S2LP_RF_API_LATENCY_MS[RF_API_LATENCY_LAST] = {
	0, // Wake-up (depends on HW latency).
	1, // TX init (600µs).
	0, // Send start (depends on bit rate and will be computed during init function).
	0, // Send end (depends on bit rate and will be computed during init function).
	0, // TX de init (30µs).
	0, // Sleep (depends on HW latency).
#ifdef BIDIRECTIONAL
	1, // RX init (1.1ms).
	0, // Receive start (150µs).
	7, // Receive end (6.7ms).
	0, // RX de init (30µs).
#endif
};
#endif
static S2LP_RF_API_context_t s2lp_rf_api_ctx;

/*** S2LP RF API local functions ***/

/*******************************************************************/
static void _reset_context(void) {
#ifdef BIDIRECTIONAL
	// Local variables.
	sfx_u8 idx = 0;
	const sfx_u8 S2LP_RF_API_SIGFOX_DL_FT[SIGFOX_DL_FT_SIZE_BYTES] = SIGFOX_DL_FT;
#endif
	// Init context.
	s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_READY;
	s2lp_rf_api_ctx.flags.all = 0;
	s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr = SFX_NULL;
	s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr = SFX_NULL;
	s2lp_rf_api_ctx.config.rc = SFX_NULL;
#ifdef ASYNCHRONOUS
	s2lp_rf_api_ctx.config.process_cb = SFX_NULL;
	s2lp_rf_api_ctx.config.error_cb = SFX_NULL;
#endif
#ifdef BIDIRECTIONAL
	// Build synchronization word in integer format (0xB2270000).
	s2lp_rf_api_ctx.sync_word_u32 = 0;
	for (idx=0 ; idx<4 ; idx++) {
		if (idx >= SIGFOX_DL_FT_SIZE_BYTES) break;
		s2lp_rf_api_ctx.sync_word_u32 |= S2LP_RF_API_SIGFOX_DL_FT[idx] << (8 * (3 - idx));
	}
#endif
}

/*******************************************************************/
static void _wait_for_s2lp_state_switch(S2LPState new_state) {
	// Refresh MC_STATE until state is reached.
	while (g_xStatus.MC_STATE != new_state) {
		S2LPRefreshStatus();
	}
}

/*******************************************************************/
static void _s2lp_gpio_irq_callback(void) {
	// Set flag if IRQ is enabled.
	if (s2lp_rf_api_ctx.flags.field.irq_enable != 0) {
		s2lp_rf_api_ctx.flags.field.irq_process = 1;
#ifdef ASYNCHRONOUS
		// Call Sigfox callback to process IRQ in main context.
		s2lp_rf_api_ctx.config.process_cb();
#endif
	}
}

/*******************************************************************/
static RF_API_status_t _internal_process(void) {
	// Local variables.
	sfx_u8 idx = 0;
#ifdef ERROR_CODES
	RF_API_status_t status = S2LP_RF_API_SUCCESS;
#endif
	// Perform state machine.
	switch (s2lp_rf_api_ctx.state) {
	case S2LP_RF_API_STATE_READY:
		// Nothing to do.
		break;
	case S2LP_RF_API_STATE_TX_RAMP_UP:
		// Reset context.
		s2lp_rf_api_ctx.tx_bit_idx = 0;
		s2lp_rf_api_ctx.tx_byte_idx = 0;
		// Fill ramp-up.
		for (idx=0 ; idx<S2LP_RF_API_RAMP_PROFILE_SIZE_BYTES ; idx++) {
			s2lp_rf_api_ctx.ramp_fifo_buffer[(2 * idx)] = 0; // Deviation.
			s2lp_rf_api_ctx.ramp_fifo_buffer[(2 * idx) + 1] = s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr[S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES - idx - 1]; // PA output power.
		}
		// Load ramp-up buffer into FIFO.
		S2LPCmdStrobeFlushTxFifo();
		SdkEvalSpiWriteFifo(S2LP_RF_API_RAMP_FIFO_BUFFER_SIZE_BYTES, (sfx_u8*) s2lp_rf_api_ctx.ramp_fifo_buffer);
		// Enable external GPIO interrupt.
		S2LPGpioIrqClearStatus();
		s2lp_rf_api_ctx.flags.field.irq_enable = 1;
		// Start radio.
		S2LPCmdStrobeLockTx();
		_wait_for_s2lp_state_switch(MC_STATE_LOCKON);
		S2LPCmdStrobeTx();
		_wait_for_s2lp_state_switch(MC_STATE_TX);
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
				for (idx=0 ; idx<S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES ; idx++) {
					s2lp_rf_api_ctx.symbol_fifo_buffer[(2 * idx)] = (idx == S2LP_RF_API_FIFO_BUFFER_FDEV_IDX) ? s2lp_rf_api_ctx.tx_fdev : 0; // Deviation.
					s2lp_rf_api_ctx.symbol_fifo_buffer[(2 * idx) + 1] = s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr[idx]; // PA output power.
				}
			}
			else {
				// Constant CW.
				for (idx=0 ; idx<S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES ; idx++) {
					s2lp_rf_api_ctx.symbol_fifo_buffer[(2 * idx)] = 0; // Deviation.
					s2lp_rf_api_ctx.symbol_fifo_buffer[(2 * idx) + 1] = s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr[0]; // PA output power.
				}
			}
			// Load bit into FIFO.
			SdkEvalSpiWriteFifo(S2LP_RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES, (sfx_u8*) s2lp_rf_api_ctx.symbol_fifo_buffer);
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
			for (idx=0 ; idx<S2LP_RF_API_RAMP_PROFILE_SIZE_BYTES ; idx++) {
				s2lp_rf_api_ctx.ramp_fifo_buffer[(2 * idx)] = 0; // FDEV.
				s2lp_rf_api_ctx.ramp_fifo_buffer[(2 * idx) + 1] = s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr[idx]; // PA output power for ramp-down.
			}
			// Load ramp-down buffer into FIFO.
			SdkEvalSpiWriteFifo(S2LP_RF_API_RAMP_FIFO_BUFFER_SIZE_BYTES, (sfx_u8*) s2lp_rf_api_ctx.ramp_fifo_buffer);
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
			for (idx=0 ; idx<S2LP_RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES ; idx++) {
				s2lp_rf_api_ctx.symbol_fifo_buffer[idx] = 0x00;
			}
			// Load padding buffer into FIFO.
			SdkEvalSpiWriteFifo(S2LP_RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES, (sfx_u8*) s2lp_rf_api_ctx.symbol_fifo_buffer);
			// Update state.
			s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_TX_END;
			// Clear flag.
			S2LPGpioIrqClearStatus();
		}
		break;
	case S2LP_RF_API_STATE_TX_END:
		// Check FIFO flag.
		if (S2LPGpioIrqCheckFlag(TX_FIFO_ALMOST_EMPTY) != 0) {
			// Disable interrupt.
			s2lp_rf_api_ctx.flags.field.irq_enable = 0;
			S2LPGpioIrqClearStatus();
#ifdef ASYNCHRONOUS
			// Call TX completion callback.
			s2lp_rf_api_ctx.tx_cplt_cb();
#endif
			// Update state.
			s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_READY;
		}
		break;
#ifdef BIDIRECTIONAL
	case S2LP_RF_API_STATE_RX_START:
		S2LPCmdStrobeFlushRxFifo();
		// Enable external GPIO interrupt.
		S2LPGpioIrqClearStatus();
		s2lp_rf_api_ctx.flags.field.irq_enable = 1;
		// Start radio.
		S2LPCmdStrobeRx();
		_wait_for_s2lp_state_switch(MC_STATE_RX);
		// Update state.
		s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_RX;
		break;
	case S2LP_RF_API_STATE_RX:
		// Check RX data flag.
		if (S2LPGpioIrqCheckFlag(RX_DATA_READY) != 0) {
			// Read FIFO and RSSI.
			SdkEvalSpiReadFifo(SIGFOX_DL_PHY_CONTENT_SIZE_BYTES, s2lp_rf_api_ctx.dl_phy_content);
			s2lp_rf_api_ctx.dl_rssi_dbm = (int16_t) S2LPRadioGetRssidBm();
			// Clear flag.
			S2LPGpioIrqClearStatus();
			// Disable interrupt.
			s2lp_rf_api_ctx.flags.field.irq_enable = 0;
#ifdef ASYNCHRONOUS
			// Call TX completion callback.
			s2lp_rf_api_ctx.rx_data_received_cb();
#endif
			// Update state.
			s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_READY;
		}
		break;
#endif /* BIDIRECTIONAL */
	default:
		EXIT_ERROR(S2LP_RF_API_ERROR_STATE);
		break;
	}
errors:
	// Clear flag.
	s2lp_rf_api_ctx.flags.field.irq_process = 0;
	RETURN();
}

/*** S2LP RF API functions ***/

#if (defined ASYNCHRONOUS) || (defined LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
RF_API_status_t S2LP_RF_API_open(RF_API_config_t *rf_api_config) {
	// Local variables.
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
	S2LP_HW_API_status_t s2lp_hw_api_status = S2LP_HW_API_SUCCESS;
#endif
	// Reset context.
	_reset_context();
	// Save parameters.
	s2lp_rf_api_ctx.config.rc = (rf_api_config -> rc);
#ifdef ASYNCHRONOUS
	s2lp_rf_api_ctx.config.process_cb = (rf_api_config -> process_cb);
	s2lp_rf_api_ctx.config.error_cb = (rf_api_config -> error_cb);
#endif
	// Init board.
#ifdef ERROR_CODES
	s2lp_hw_api_status = S2LP_HW_API_open(&_s2lp_gpio_irq_callback);
	S2LP_HW_API_check_status(S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
#else
	S2LP_HW_API_open(&_s2lp_gpio_irq_callback);
#endif
	SdkEvalSpiInit();
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}
#endif

#ifdef LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
RF_API_status_t S2LP_RF_API_close(void) {
#ifdef ERROR_CODES
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	S2LP_HW_API_status_t s2lp_hw_api_status = S2LP_HW_API_SUCCESS;
#endif
	// Reset context.
	_reset_context();
	// De init board.
	SdkEvalSpiDeinit();
#ifdef ERROR_CODES
	s2lp_hw_api_status = S2LP_HW_API_close();
	S2LP_HW_API_check_status(S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
#else
	S2LP_HW_API_close();
#endif
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}
#endif

#ifdef ASYNCHRONOUS
/*******************************************************************/
RF_API_status_t S2LP_RF_API_process(void) {
#ifdef ERROR_CODES
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	// Run internal process.
	while (s2lp_rf_api_ctx.flags.field.irq_process != 0) {
#ifdef ERROR_CODES
		status = _internal_process();
		CHECK_STATUS(RF_API_SUCCESS);
#else
		_internal_process();
#endif
	}
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}
#endif

/*******************************************************************/
RF_API_status_t S2LP_RF_API_wake_up(void) {
#ifdef ERROR_CODES
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	// Exit shutdown.
	SdkEvalExitShutdown();
	// Enter standby state.
	S2LPCmdStrobeSres();
	S2LPCmdStrobeStandby();
	_wait_for_s2lp_state_switch(MC_STATE_STANDBY);
	RETURN();
}

/*******************************************************************/
RF_API_status_t S2LP_RF_API_sleep(void) {
#ifdef ERROR_CODES
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	// Enter shutdown.
	SdkEvalEnterShutdown();
	RETURN();
}

/*******************************************************************/
RF_API_status_t S2LP_RF_API_init(RF_API_radio_parameters_t *radio_parameters) {
	// Local variables.
#ifdef ERROR_CODES
	RF_API_status_t status = S2LP_RF_API_SUCCESS;
	S2LP_HW_API_status_t s2lp_hw_api_status = S2LP_HW_API_SUCCESS;
#endif
	sfx_u8 s2lp_status = 0;
	S2LP_HW_API_oscillator_type_t s2lp_xo_type;
	sfx_u32 s2lp_xo_frequency_hz = 0;
	ModeExtRef s2lp_ref_mode = MODE_EXT_XO;
	ModulationSelect s2lp_modulation = MOD_NO_MOD;
	sfx_u32 s2lp_datarate = 0;
	sfx_u32 s2lp_deviation = 0;
	S2LP_HW_API_gpio_t s2lp_gpio = 0;
	SGpioInit s2lp_gpio_init;
	sfx_u8 reg_value = 0;
#ifdef BIDIRECTIONAL
	PktBasicInit downlink_pkt_init;
	SAfcInit afc_init;
	SSymClkRecInit clock_rec_init;
#endif
	// Crystal.
#ifdef ERROR_CODES
	s2lp_hw_api_status = S2LP_HW_API_get_oscillator(&s2lp_xo_type, &s2lp_xo_frequency_hz);
	S2LP_HW_API_check_status(S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
#else
	S2LP_HW_API_get_oscillator(&s2lp_xo_type, &s2lp_xo_frequency_hz);
#endif
	s2lp_ref_mode = (s2lp_xo_type == S2LP_HW_API_OSCILLATOR_TYPE_QUARTZ) ? MODE_EXT_XO : MODE_EXT_XIN;
	S2LPRadioSetXtalFrequency(s2lp_xo_frequency_hz);
	S2LPGeneralSetExtRef(s2lp_ref_mode);
	S2LPRadioSetDigDiv(S_ENABLE);
	S2LPTimerCalibrationRco(S_ENABLE);
	// Frequency.
	s2lp_status = S2LPRadioSetFrequencyBase(radio_parameters -> frequency_hz);
	if (s2lp_status != 0) EXIT_ERROR(S2LP_RF_API_ERROR_FREQUENCY);
	// Modulation and bit rate.
	switch (radio_parameters -> modulation) {
	case RF_API_MODULATION_NONE:
		s2lp_modulation = MOD_NO_MOD;
		s2lp_datarate = (radio_parameters -> bit_rate_bps);
#ifdef BIDIRECTIONAL
		s2lp_deviation = (radio_parameters -> deviation_hz);
#endif
		break;
	case RF_API_MODULATION_DBPSK:
		s2lp_modulation = MOD_POLAR;
		s2lp_datarate = ((radio_parameters -> bit_rate_bps) * S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES) / (S2LP_RF_API_POLAR_DATARATE_MULTIPLIER);
		s2lp_deviation = ((radio_parameters -> bit_rate_bps) * S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES) / (2);
		break;
	case RF_API_MODULATION_GFSK:
		s2lp_modulation = MOD_2GFSK_BT1;
		s2lp_datarate = (radio_parameters -> bit_rate_bps);
#ifdef BIDIRECTIONAL
		s2lp_deviation = (radio_parameters -> deviation_hz);
#endif
		break;
	default:
		EXIT_ERROR(S2LP_RF_API_ERROR_MODULATION);
		break;
	}
	S2LPRadioSetModulation(s2lp_modulation);
	S2LPRadioSetDatarate(s2lp_datarate);
	S2LPRadioSetFrequencyDev(s2lp_deviation);
	// Disable all IRQ.
	S2LPGpioIrqDeInit(NULL);
	// Configure GPIO.
#ifdef ERROR_CODES
	s2lp_hw_api_status = S2LP_HW_API_get_gpio(&s2lp_gpio);
	S2LP_HW_API_check_status(S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
#else
	S2LP_HW_API_get_gpio(&s2lp_gpio);
#endif
	s2lp_gpio_init.xS2LPGpioPin = (S2LPGpioPin) s2lp_gpio; // Note: value mapping is the same.
	s2lp_gpio_init.xS2LPGpioMode = S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP;
	s2lp_gpio_init.xS2LPGpioIO = S2LP_GPIO_DIG_OUT_IRQ;
	S2LPGpioInit(&s2lp_gpio_init);
	// Configure specific registers.
	// Note: SPMS configuration is performed by the TX/RX command strobe.
	switch (radio_parameters -> rf_mode) {
	case RF_API_MODE_TX:
#ifdef TX_POWER_DBM_EIRP
#if (TX_POWER_DBM_EIRP == 0)
		s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr = (sfx_u8*) RF_API_RAMP_AMPLITUDE_PROFILE_00_DBM;
		s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr = (sfx_u8*) RF_API_BIT0_AMPLITUDE_PROFILE_00_DBM;
#elif (TX_POWER_DBM_EIRP == 3)
		s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr = (sfx_u8*) RF_API_RAMP_AMPLITUDE_PROFILE_03_DBM;
		s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr = (sfx_u8*) RF_API_BIT0_AMPLITUDE_PROFILE_03_DBM;
#elif (TX_POWER_DBM_EIRP == 5)
		s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr = (sfx_u8*) RF_API_RAMP_AMPLITUDE_PROFILE_05_DBM;
		s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr = (sfx_u8*) RF_API_BIT0_AMPLITUDE_PROFILE_05_DBM;
#elif (TX_POWER_DBM_EIRP == 7)
		s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr = (sfx_u8*) RF_API_RAMP_AMPLITUDE_PROFILE_07_DBM;
		s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr = (sfx_u8*) RF_API_BIT0_AMPLITUDE_PROFILE_07_DBM;
#elif (TX_POWER_DBM_EIRP == 10)
		s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr = (sfx_u8*) RF_API_RAMP_AMPLITUDE_PROFILE_10_DBM;
		s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr = (sfx_u8*) RF_API_BIT0_AMPLITUDE_PROFILE_10_DBM;
#elif (TX_POWER_DBM_EIRP == 14)
		s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr = (sfx_u8*) RF_API_RAMP_AMPLITUDE_PROFILE_14_DBM;
		s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr = (sfx_u8*) RF_API_BIT0_AMPLITUDE_PROFILE_14_DBM;
#else
#error "S2LP RF API flags error: unsupported TX_POWER_DBM_EIRP value."
#endif
#else /* TX_POWER_DBM_EIRP */
		// Allocate output power tables.
		switch (radio_parameters -> tx_power_dbm_eirp) {
		case 0:
			s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr = (sfx_u8*) RF_API_RAMP_AMPLITUDE_PROFILE_00_DBM;
			s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr = (sfx_u8*) RF_API_BIT0_AMPLITUDE_PROFILE_00_DBM;
			break;
		case 3:
			s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr = (sfx_u8*) RF_API_RAMP_AMPLITUDE_PROFILE_03_DBM;
			s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr = (sfx_u8*) RF_API_BIT0_AMPLITUDE_PROFILE_03_DBM;
			break;
		case 5:
			s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr = (sfx_u8*) RF_API_RAMP_AMPLITUDE_PROFILE_05_DBM;
			s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr = (sfx_u8*) RF_API_BIT0_AMPLITUDE_PROFILE_05_DBM;
			break;
		case 7:
			s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr = (sfx_u8*) RF_API_RAMP_AMPLITUDE_PROFILE_07_DBM;
			s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr = (sfx_u8*) RF_API_BIT0_AMPLITUDE_PROFILE_07_DBM;
			break;
		case 10:
			s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr = (sfx_u8*) RF_API_RAMP_AMPLITUDE_PROFILE_10_DBM;
			s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr = (sfx_u8*) RF_API_BIT0_AMPLITUDE_PROFILE_10_DBM;
			break;
		case 14:
			s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr = (sfx_u8*) RF_API_RAMP_AMPLITUDE_PROFILE_14_DBM;
			s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr = (sfx_u8*) RF_API_BIT0_AMPLITUDE_PROFILE_14_DBM;
			break;
		default:
			EXIT_ERROR(S2LP_RF_API_ERROR_TX_POWER);
			break;
		}
#endif /* TX_POWER_DBM_EIRP */
		// Configure IRQ mask.
		S2LPGpioIrqConfig(TX_FIFO_ALMOST_EMPTY, S_ENABLE);
		// PA configuration.
		S2LPRadioSetMaxPALevel(S_DISABLE);
		S2LPRadioSetManualRampingMode(S_DISABLE);
		S2LPRadioSetAutoRampingMode(S_DISABLE);
		SdkEvalSpiReadRegisters(MOD1_ADDR, 1, &reg_value);
		reg_value |= PA_INTERP_EN_REGMASK; // Enable interpolator.
		SdkEvalSpiWriteRegisters(MOD1_ADDR, 1, &reg_value);
		// FIFO.
		S2LPPacketHandlerSetTxMode(DIRECT_TX_FIFO_MODE);
		S2LPFifoMuxRxFifoIrqEnable(S_DISABLE);
		S2LPFifoSetAlmostEmptyThresholdTx(S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES / 2);
#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
		// Start latency = ramp-up.
		S2LP_RF_API_LATENCY_MS[RF_API_LATENCY_SEND_START] = (1000 * S2LP_RF_API_RAMP_PROFILE_SIZE_BYTES) / (s2lp_datarate * S2LP_RF_API_POLAR_DATARATE_MULTIPLIER);
		// Stop latency = ramp-down + half of padding bit (since IRQ is raised when FIFO is half empty).
		S2LP_RF_API_LATENCY_MS[RF_API_LATENCY_SEND_STOP] = (1000 * (S2LP_RF_API_RAMP_PROFILE_SIZE_BYTES + (S2LP_RF_API_SYMBOL_PROFILE_SIZE_BYTES / 2))) / (s2lp_datarate * S2LP_RF_API_POLAR_DATARATE_MULTIPLIER);
#endif
		break;
#if (defined BIDIRECTIONAL) || ((defined REGULATORY && (defined SPECTRUM_ACCESS_LBT)))
	case RF_API_MODE_RX:
#ifdef BIDIRECTIONAL
		// Configure IRQ mask.
		S2LPGpioIrqConfig(RX_DATA_READY, S_ENABLE);
		// RX bandwidth.
		S2LPRadioSetChannelBW(S2LP_RF_API_RX_BANDWIDTH_HZ);
		// Disable equalization, antenna switching and carrier sense.
		S2LPRadioSetIsiEqualizationMode(ISI_EQUALIZATION_DISABLED);
		S2LPRadioAntennaSwitching(S_DISABLE);
		S2LPRadioCsBlanking(S_DISABLE);
		// Downnlink packet structure.
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
		// Set RSSI threshold.
		S2LPRadioSetRssiThreshdBm(S2LP_RF_API_DOWNLINK_RSSI_THRESHOLD_DBM);
		// FIFO.
		S2LPPacketHandlerSetRxMode(NORMAL_RX_MODE);
		S2LPFifoMuxRxFifoIrqEnable(S_DISABLE);
		S2LPFifoSetAlmostFullThresholdTx(SIGFOX_DL_PHY_CONTENT_SIZE_BYTES);
		break;
#endif /* BIDIRECTIONAL */
#endif
	default:
		EXIT_ERROR(S2LP_RF_API_ERROR_MODE);
		break;
	}
	S2LPCmdStrobeReady();
	_wait_for_s2lp_state_switch(MC_STATE_READY);
errors:
	RETURN();
}

/*******************************************************************/
RF_API_status_t S2LP_RF_API_de_init(void) {
#ifdef ERROR_CODES
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	// Disable external GPIO interrupt.
	s2lp_rf_api_ctx.flags.all = 0;
	// Stop radio.
	S2LPCmdStrobeSabort();
	S2LPCmdStrobeSres();
	S2LPCmdStrobeReady();
	_wait_for_s2lp_state_switch(MC_STATE_READY);
	RETURN();
}

/*******************************************************************/
RF_API_status_t S2LP_RF_API_send(RF_API_tx_data_t *tx_data) {
	// Local variables.
#ifdef ERROR_CODES
	RF_API_status_t status = S2LP_RF_API_SUCCESS;
#endif
	sfx_u8 idx = 0;
	// Store TX data.
	s2lp_rf_api_ctx.tx_bitstream_size_bytes = (tx_data -> bitstream_size_bytes);
	for (idx=0 ; idx<(s2lp_rf_api_ctx.tx_bitstream_size_bytes) ; idx++) {
		s2lp_rf_api_ctx.tx_bitstream[idx] = (tx_data -> bitstream)[idx];
	}
#ifdef ASYNCHRONOUS
	// Store callback.
	s2lp_rf_api_ctx.tx_cplt_cb = (tx_data -> cplt_cb);
#endif
	// Check symbol tables.
	if ((s2lp_rf_api_ctx.tx_bit_0_amplitude_profile_ptr == SFX_NULL) || (s2lp_rf_api_ctx.tx_ramp_amplitude_profile_ptr == NULL)) {
		EXIT_ERROR(S2LP_RF_API_ERROR_SYMBOL_TABLE);
	}
	// Trigger TX.
	s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_TX_RAMP_UP;
#ifdef ERROR_CODES
	status = _internal_process();
	CHECK_STATUS(RF_API_SUCCESS);
#else
	_internal_process();
#endif
#ifndef ASYNCHRONOUS
	while (s2lp_rf_api_ctx.state != S2LP_RF_API_STATE_READY) {
#ifdef ERROR_CODES
		status = _internal_process();
		CHECK_STATUS(RF_API_SUCCESS);
#else
		_internal_process();
#endif
	}
#endif
errors:
	RETURN();
}

#ifdef BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t S2LP_RF_API_receive(RF_API_rx_data_t *rx_data) {
	// Local variables.
#ifdef ERROR_CODES
	RF_API_status_t status = S2LP_RF_API_SUCCESS;
#ifndef ASYNCHRONOUS
	MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#endif
#endif
#ifdef ASYNCHRONOUS
	// Store callback.
	s2lp_rf_api_ctx.rx_data_received_cb = (rx_data -> data_received_cb);
#else
	sfx_bool dl_timeout = SFX_FALSE;
	// Reset flag.
	(rx_data -> data_received) = SFX_FALSE;
#endif
	// Trigger RX.
	s2lp_rf_api_ctx.state = S2LP_RF_API_STATE_RX_START;
#ifdef ERROR_CODES
	status = _internal_process();
	CHECK_STATUS(RF_API_SUCCESS);
#else
	_internal_process();
#endif
#ifndef ASYNCHRONOUS
	while (s2lp_rf_api_ctx.state != S2LP_RF_API_STATE_READY) {
#ifdef ERROR_CODES
		status = _internal_process();
		CHECK_STATUS(RF_API_SUCCESS);
#else
		_internal_process();
#endif
		// Check timeout.
#ifdef ERROR_CODES
		mcu_api_status = MCU_API_timer_status(MCU_API_TIMER_INSTANCE_T_RX, &dl_timeout);
		MCU_API_check_status(S2LP_RF_API_ERROR_DRIVER_MCU_API);
#else
		MCU_API_timer_status(MCU_API_TIMER_INSTANCE_T_RX, &dl_timeout);
#endif
		if (dl_timeout == SFX_TRUE) break;
	}
#endif
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}
#endif

#ifdef BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t S2LP_RF_API_get_dl_phy_content_and_rssi(sfx_u8 *dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16 *dl_rssi_dbm) {
	// Local variables.
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	sfx_u8 idx = 0;
#ifdef PARAMETERS_CHECK
	// Check parameters.
	if ((dl_phy_content == SFX_NULL) || (dl_rssi_dbm == SFX_NULL)) {
		EXIT_ERROR(S2LP_RF_API_ERROR_NULL_PARAMETER);
	}
	if (dl_phy_content_size > SIGFOX_DL_PHY_CONTENT_SIZE_BYTES) {
		EXIT_ERROR(S2LP_RF_API_ERROR_BUFFER_SIZE);
	}
#endif
	// Fill data.
	for (idx=0 ; idx<dl_phy_content_size ; idx++) {
		dl_phy_content[idx] = s2lp_rf_api_ctx.dl_phy_content[idx];
	}
	*(dl_rssi_dbm) = (sfx_s16) s2lp_rf_api_ctx.dl_rssi_dbm;
#ifdef PARAMETERS_CHECK
errors:
#endif
	RETURN();
}
#endif

#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_LBT)
/*******************************************************************/
RF_API_status_t S2LP_RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params) {
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	/* To be implemented by the device manufacturer */
	RETURN();
}
#endif

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
/*******************************************************************/
RF_API_status_t S2LP_RF_API_get_latency(RF_API_latency_t latency_type, sfx_u32 *latency_ms) {
	// Local variables.
	sfx_u32 hw_api_latency[S2LP_HW_API_LATENCY_LAST];
	sfx_u8 idx = 0;
#ifdef ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
    S2LP_HW_API_status_t s2lp_hw_api_status = S2LP_HW_API_SUCCESS;
#endif
#ifdef PARAMETERS_CHECK
    // Reset result.
     (*latency_ms) = 0;
    // Check parameter.
    if (latency_type >= RF_API_LATENCY_LAST) {
    	EXIT_ERROR(S2LP_RF_API_ERROR_LATENCY_TYPE);
    }
#endif
    // Set base latency.
    (*latency_ms) = S2LP_RF_API_LATENCY_MS[latency_type];
    // Read hardware latencies.
    for (idx=0 ; idx<S2LP_HW_API_LATENCY_LAST ; idx++) {
#ifdef ERROR_CODES
    	s2lp_hw_api_status = S2LP_HW_API_get_latency(idx, &(hw_api_latency[idx]));
    	S2LP_HW_API_check_status(S2LP_RF_API_ERROR_DRIVER_S2LP_HW_API);
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
#if (defined PARAMETERS_CHECK) || (defined ERROR_CODES)
errors:
#endif
    RETURN();
}
#endif

#ifdef VERBOSE
/*******************************************************************/
RF_API_status_t S2LP_RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
    (*version) = (sfx_u8*) S2LP_RF_API_VERSION;
    (*version_size_char) = (sfx_u8) sizeof(S2LP_RF_API_VERSION);
	RETURN();
}
#endif

#ifdef ERROR_CODES
/*******************************************************************/
void S2LP_RF_API_error(void) {
	// Turn radio off and close driver.
	S2LP_RF_API_sleep();
#ifdef LOW_LEVEL_OPEN_CLOSE
	S2LP_RF_API_close();
#endif
}
#endif
