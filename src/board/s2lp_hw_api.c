/*!*****************************************************************
 * \file    s2lp_hw_api.c
 * \brief   Sigfox S2LP HW interface.
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

#include "board/s2lp_hw_api.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
// Sigfox EP library.
#include "manuf/rf_api.h"
#include "sigfox_error.h"
#include "sigfox_types.h"

/*** S2LP HW API functions ***/

/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_open(S2LP_HW_API_irq_cb_t gpio_irq_callback) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif

	// Configure the S2LP_SDN pin as output.
	// Enter shutdown mode.
	// Perform a 100ms delay to ensure S2LP_SDN is kept high during a minimum time.

	// Configure the S2LP_GPIOx pin as IRQ input.
	// A falling edge on this pins wakes-up the MCU in order to handle radio interrupts.
	// The gpio_irq_callback function has to be called when such event occurs.

	RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_close(void) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif

	// Release the S2LP_SDN and S2LP_GPIOx pins.

	RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_get_oscillator(S2LP_HW_API_oscillator_type_t *xo_type, sfx_u32 *xo_frequency_hz) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
	RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_get_gpio(S2LP_HW_API_signal_t signal, S2LP_HW_API_gpio_t *s2lp_gpio) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
	RETURN();
}

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_get_latency(S2LP_HW_API_latency_t latency_type, sfx_u32 *latency_ms) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    RETURN();
}
#endif
