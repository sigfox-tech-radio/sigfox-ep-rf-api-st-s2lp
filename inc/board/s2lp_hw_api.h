/*!*****************************************************************
 * \file    s2lp_hw_api.h
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

#ifndef __S2LP_HW_API_H__
#define __S2LP_HW_API_H__

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"

/*** S2LP HW API structures ***/

#ifdef ERROR_CODES
/*!******************************************************************
 * \enum S2LP_HW_API_status_t
 * \brief S2LP HW driver error codes.
 *******************************************************************/
typedef enum {
	S2LP_HW_API_SUCCESS = 0,
	S2LP_HW_API_ERROR,
	// Additional custom error codes can be added here (up to sfx_u32).
	// They will be logged in the library error stack if the ERROR_STACK flag is defined (SIGFOX_ERROR_SOURCE_HW base).
	// Last index.
	S2LP_HW_API_ERROR_LAST
} S2LP_HW_API_status_t;
#else
typedef void S2LP_HW_API_status_t;
#endif

/*!******************************************************************
 * \enum S2LP_HW_API_oscillator_type_t
 * \brief S2LP oscillator types.
 *******************************************************************/
typedef enum {
	S2LP_HW_API_OSCILLATOR_TYPE_QUARTZ = 0,
	S2LP_HW_API_OSCILLATOR_TYPE_TCXO,
	S2LP_HW_API_OSCILLATOR_TYPE_LAST
} S2LP_HW_API_oscillator_type_t;

/*!******************************************************************
 * \enum S2LP_HW_API_gpio_t
 * \brief S2LP GPIO pins list.
 *******************************************************************/
typedef enum {
	S2LP_HW_API_GPIO_0 = 0,
	S2LP_HW_API_GPIO_1,
	S2LP_HW_API_GPIO_2,
	S2LP_HW_API_GPIO_3,
	S2LP_HW_API_GPIO_LAST
} S2LP_HW_API_gpio_t;

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
/*!******************************************************************
 * \enum S2LP_HW_API_latency_t
 * \brief S2LP hardware functions latency delay type.
 *******************************************************************/
typedef enum {
	S2LP_HW_API_LATENCY_EXIT_SHUTDOWN = 0,
	S2LP_HW_API_LATENCY_ENTER_SHUTDOWN,
	S2LP_HW_API_LATENCY_LAST
} S2LP_HW_API_latency_t;
#endif

/********************************
 * \brief S2LP driver callback functions.
 * \fn S2LP_HW_API_irq_cb_t		To be called when a falling edge is detected on the S2LP GPIO.
 *******************************/
typedef void (S2LP_HW_API_irq_cb_t)(void);

/*** S2LP HW API functions ***/

/*!******************************************************************
 * \fn S2LP_HW_API_status_t S2LP_HW_API_open(S2LP_HW_irq_cb_t gpio_irq_callback)
 * \brief Open the S2LP hardware interface. This function is called during RF_API_open() function of the manufacturer layer.
 * \param[in]  	gpio_irq_callback: GPIO interrupt callback that must be called on S2LP GPIOx pin interrupt.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_open(S2LP_HW_API_irq_cb_t gpio_irq_callback);

/*!******************************************************************
 * \fn S2LP_HW_API_status_t S2LP_HW_API_close(void)
 * \brief Close the S2LP hardware interface. This function is called during RF_API_close() function of the manufacturer layer.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_close(void);

/*!******************************************************************
 * \fn S2LP_HW_API_status_t S2LP_HW_API_get_xo_frequency_hz(sfx_u32 *xo_frequency_hz)
 * \brief Returns the S2LP crystal oscillator frequency.
 * \param[in]  	none
 * \param[out] 	xo_type: Pointer to the type of oscillator attached to S2LP.
 * \param[out] 	xo_frequency_hz: Pointer to the frequency of the oscillator attached to S2LP.
 * \retval		Function execution status.
 *******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_get_oscillator(S2LP_HW_API_oscillator_type_t *xo_type, sfx_u32 *xo_frequency_hz);

/*!******************************************************************
 * \fn S2LP_HW_API_status_t S2LP_HW_API_get_gpio(sfx_u8 *s2lp_gpio)
 * \brief Returns the S2LP GPIO attached to the MCU.
 * \param[in]  	none
 * \param[out] 	s2lp_gpio_pin: Pointer to the S2LP GPIO pin number used as external interrupt.
 * \retval		Function execution status.
 *******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_get_gpio(S2LP_HW_API_gpio_t *s2lp_gpio);

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
/*!******************************************************************
 * \fn S2LP_HW_API_status_t S2LP_HW_API_get_latency(S2LP_HW_API_latency_t latency_type, sfx_u32 *latency_ms)
 * \brief Read HW functions latency in milliseconds.
 * \param[in]	latency_type: Type of latency to get.
 * \param[out] 	latency_ms: Pointer to integer that will contain the latency in milliseconds.
 * \retval		Function execution status.
 *******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_get_latency(S2LP_HW_API_latency_t latency_type, sfx_u32 *latency_ms);
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void S2LP_HW_API_stack_error(void)
 * \brief Generic macro which calls the error stack function for S2LP_HW_API errors (if enabled).
 * \param[in]  	none
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#ifdef ERROR_STACK
#define S2LP_HW_API_stack_error(void) SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_HW_API, s2lp_hw_api_status)
#else
#define S2LP_HW_API_stack_error(void)
#endif
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void S2LP_HW_API_check_status(error)
 * \brief Generic macro to check an S2LP_HW_API function status and exit.
 * \param[in]  	error: High level error code to rise.
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#define S2LP_HW_API_check_status(error) { if (s2lp_hw_api_status != S2LP_HW_API_SUCCESS) { S2LP_HW_API_stack_error(); EXIT_ERROR(error) } }
#endif

#endif /* __S2LP_HW_API_H__ */
