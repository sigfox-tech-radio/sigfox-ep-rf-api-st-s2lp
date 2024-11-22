/*!*****************************************************************
 * \file   S2LP_CORE_SPI.c
 * \brief  S2LP library low level functions.
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

#include "board/S2LP_CORE_SPI.h"

#include "sigfox_types.h"

/*** S2LP CORE SPI functions ***/

/*******************************************************************/
sfx_u8 __attribute__((weak)) S2LPSpiWriteRegisters(sfx_u8 register_address, sfx_u8 data_size, sfx_u8 *data) {
    /* To be implemented by the device manufacturer */
    SIGFOX_UNUSED(register_address);
    SIGFOX_UNUSED(data_size);
    SIGFOX_UNUSED(data);
    return 0;
}

/*******************************************************************/
sfx_u8 __attribute__((weak)) S2LPSpiReadRegisters(sfx_u8 register_address, sfx_u8 data_size, sfx_u8 *data) {
    /* To be implemented by the device manufacturer */
    SIGFOX_UNUSED(register_address);
    SIGFOX_UNUSED(data_size);
    SIGFOX_UNUSED(data);
    return 0;
}

/*******************************************************************/
sfx_u8 __attribute__((weak)) S2LPSpiCommandStrobes(sfx_u8 command) {
    /* To be implemented by the device manufacturer */
    SIGFOX_UNUSED(command);
    return 0;
}

/*******************************************************************/
sfx_u8 __attribute__((weak)) S2LPSpiWriteFifo(sfx_u8 data_size, sfx_u8 *data) {
    /* To be implemented by the device manufacturer */
    SIGFOX_UNUSED(data_size);
    SIGFOX_UNUSED(data);
    return 0;
}

/*******************************************************************/
sfx_u8 __attribute__((weak)) S2LPSpiReadFifo(sfx_u8 data_size, sfx_u8 *data) {
    /* To be implemented by the device manufacturer */
    SIGFOX_UNUSED(data_size);
    SIGFOX_UNUSED(data);
    return 0;
}
