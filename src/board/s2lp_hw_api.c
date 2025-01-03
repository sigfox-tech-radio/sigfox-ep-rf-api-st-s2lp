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

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_error.h"
#include "sigfox_types.h"
#include "manuf/rf_api.h"

/*** S2LP HW API functions ***/

/*******************************************************************/
S2LP_HW_API_status_t __attribute__((weak)) S2LP_HW_API_open(S2LP_HW_API_config_t *hw_api_config) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    SIGFOX_UNUSED(hw_api_config);
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t __attribute__((weak)) S2LP_HW_API_close(void) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t __attribute__((weak)) S2LP_HW_API_init(S2LP_radio_parameters_t *radio_parameters) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    SIGFOX_UNUSED(radio_parameters);
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t __attribute__((weak)) S2LP_HW_API_de_init(void) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t __attribute__((weak)) S2LP_HW_API_enter_shutdown(void) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t __attribute__((weak)) S2LP_HW_API_exit_shutdown(void) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t __attribute__((weak)) S2LP_HW_API_get_oscillator(S2LP_HW_API_oscillator_type_t *xo_type, sfx_u32 *xo_frequency_hz) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    SIGFOX_UNUSED(xo_type);
    SIGFOX_UNUSED(xo_frequency_hz);
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t __attribute__((weak)) S2LP_HW_API_get_gpio(S2LP_HW_API_signal_t signal, S2LP_HW_API_gpio_t *s2lp_gpio) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    SIGFOX_UNUSED(signal);
    SIGFOX_UNUSED(s2lp_gpio);
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t __attribute__((weak)) S2LP_HW_API_get_tx_power(sfx_s8 expected_tx_power_dbm, sfx_s8 *s2lp_tx_power_dbm) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    SIGFOX_UNUSED(expected_tx_power_dbm);
    SIGFOX_UNUSED(s2lp_tx_power_dbm);
    SIGFOX_RETURN();
}

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*******************************************************************/
S2LP_HW_API_status_t __attribute__((weak)) S2LP_HW_API_get_latency(S2LP_HW_API_latency_t latency_type, sfx_u32 *latency_ms) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    SIGFOX_UNUSED(latency_type);
    SIGFOX_UNUSED(latency_ms);
    SIGFOX_RETURN();
}
#endif
