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
#include "manuf/rf_api.h"

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
 * \enum S2LP_HW_API_signal_t
 * \brief S2LP external signals list.
 *******************************************************************/
typedef enum {
	S2LP_HW_API_SIGNAL_NIRQ = 0,
	S2LP_HW_API_SIGNAL_LAST
} S2LP_HW_API_signal_t;

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
	S2LP_HW_API_LATENCY_INIT_TX,
	S2LP_HW_API_LATENCY_DE_INIT_TX,
#ifdef BIDIRECTIONAL
	S2LP_HW_API_LATENCY_INIT_RX,
	S2LP_HW_API_LATENCY_DE_INIT_RX,
#endif
	S2LP_HW_API_LATENCY_LAST
} S2LP_HW_API_latency_t;
#endif

/*!******************************************************************
 * \brief S2LP driver IRQ callback functions.
 * \fn S2LP_HW_API_irq_cb_t		To be called when the corresponding IRQ occurs.
 *******************************************************************/
typedef void (*S2LP_HW_API_irq_cb_t)(void);

/*!******************************************************************
 * \struct S2LP_HW_API_config_t
 * \brief S2LP driver configuration structure.
 *******************************************************************/
typedef struct {
	S2LP_HW_API_irq_cb_t gpio_irq_callback; // S2LP GPIOx pin interrupt callback.
} S2LP_HW_API_config_t;

/*!******************************************************************
 * \struct S2LP_radio_parameters_t
 * \brief S2LP radio parameters structure.
 *******************************************************************/
typedef struct {
	RF_API_mode_t rf_mode;
	sfx_s8 expected_tx_power_dbm;
} S2LP_radio_parameters_t;

/*** S2LP HW API functions ***/

/*!******************************************************************
 * \fn S2LP_HW_API_status_t S2LP_HW_API_open(S2LP_HW_API_config_t* hw_api_config)
 * \brief This function is called during the RF_API_open() function to open the S2LP hardware interface. It should:
 * \brief - Configure the S2LP_SDN pin as output.
 * \brief - Enter shutdown mode and perform a delay to ensure S2LP_SDN is kept high during a minimum time.
 * \brief - Configure the S2LP_GPIOx pin as IRQ input: a falling edge on this pin must call the gpio_irq_callback function passed as parameter.
 * \param[in]  	hw_api_config: Pointer to the HW API configuration.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_open(S2LP_HW_API_config_t* hw_api_config);

/*!******************************************************************
 * \fn S2LP_HW_API_status_t S2LP_HW_API_close(void)
 * \brief This function is called during the RF_API_close() function to close the S2LP hardware interface. It should:
 * \brief - Release the S2LP_SDN and S2LP_GPIOx pins.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_close(void);

/*!******************************************************************
 * \fn S2LP_HW_API_status_t S2LP_HW_API_init(S2LP_radio_parameters_t *radio_parameters)
 * \brief This optional function is called during the RF_API_init() function to configure additional hardware parameters.
 * \param[in]  	radio_parameters: Pointers to the radio parameters.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_init(S2LP_radio_parameters_t *radio_parameters);

/*!******************************************************************
 * \fn S2LP_HW_API_status_t S2LP_HW_API_de_init(void)
 * \brief This optional function is called during the RF_API_de_init() function to release additional hardware parameters.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_de_init(void);

/*!******************************************************************
 * \fn S2LP_HW_API_status_t S2LP_HW_API_get_oscillator(S2LP_HW_API_oscillator_type_t *xo_type, sfx_u32 *xo_frequency_hz)
 * \brief Returns the crystal oscillator parameters attached to the S2LP.
 * \param[in]  	none
 * \param[out] 	xo_type: Pointer to the type of oscillator attached to S2LP.
 * \param[out] 	xo_frequency_hz: Pointer to the frequency of the oscillator attached to S2LP.
 * \retval		Function execution status.
 *******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_get_oscillator(S2LP_HW_API_oscillator_type_t *xo_type, sfx_u32 *xo_frequency_hz);

/*!******************************************************************
 * \fn S2LP_HW_API_status_t S2LP_HW_API_get_gpio(sfx_u8 *s2lp_gpio)
 * \brief Returns the S2LP GPIO attached to the MCU.
 * \param[in]  	signal: External S2LP signal.
 * \param[out] 	s2lp_gpio: Pointer to the S2LP GPIO pin number connected to the signal.
 * \retval		Function execution status.
 *******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_get_gpio(S2LP_HW_API_signal_t signal, S2LP_HW_API_gpio_t *s2lp_gpio);

/*!******************************************************************
 * \fn S2LP_HW_API_status_t S2LP_HW_API_get_tx_power(sfx_s8 expected_tx_power_dbm, sfx_s8 *s2lp_tx_power_dbm)
 * \brief Returns the effective RF output power to program on the S2LP to get the expected value at board level.
 * \brief This function is required when an external gain has to be compensated (typical case of an external PA). Otherwise the S2LP output power equals the expected value.
 * \param[in]  	expected_tx_power_dbm: Expected output power in dBm (given by applicative level).
 * \param[out] 	s2lp_tx_power_dbm: Pointer to the effective output power in dBm to program on the S2LP transceiver.
 * \retval		Function execution status.
 *******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_get_tx_power(sfx_s8 expected_tx_power_dbm, sfx_s8 *s2lp_tx_power_dbm);

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
