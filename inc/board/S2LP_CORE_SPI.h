/*!*****************************************************************
 * \file   S2LP_CORE_SPI.h
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

#ifndef __S2LP_CORE_SPI_H__
#define __S2LP_CORE_SPI_H__

#include "sigfox_types.h"

/*** S2LP CORE SPI functions ***/

/*!******************************************************************
 * \fn sfx_u8 S2LPSpiWriteRegisters(sfx_u8 register_address, sfx_u8 data_size, sfx_u8* data)
 * \brief Write S2LP registers.
 * \param[in]   register_address: Address of the register to write.
 * \param[in]   data_size: Number of bytes to write.
 * \param[in]   data: Input data to write.
 * \param[out]  none
 * \retval      MC_STATE0 value.
 *******************************************************************/
sfx_u8 S2LPSpiWriteRegisters(sfx_u8 register_address, sfx_u8 data_size, sfx_u8 *data);

/*!******************************************************************
 * \fn sfx_u8 S2LPSpiReadRegisters(sfx_u8 register_address, sfx_u8 data_size, sfx_u8* data)
 * \brief Read S2LP registers.
 * \param[in]   register_address: Address of the register to read.
 * \param[in]   data_size: Number of bytes to read.
 * \param[out]  data: Pointer to the read data.
 * \retval      MC_STATE0 value.
 *******************************************************************/
sfx_u8 S2LPSpiReadRegisters(sfx_u8 register_address, sfx_u8 data_size, sfx_u8 *data);

/*!******************************************************************
 * \fn sfx_u8 S2LPSpiCommandStrobes(sfx_u8 command)
 * \brief Send a command strobe to S2LP.
 * \param[in]   command: Command to send.
 * \param[out]  none
 * \retval      MC_STATE0 value.
 *******************************************************************/
sfx_u8 S2LPSpiCommandStrobes(sfx_u8 command);

/*!******************************************************************
 * \fn sfx_u8 S2LPSpiWriteFifo(sfx_u8 data_size, sfx_u8* data)
 * \brief Write S2LP FIFO.
 * \param[in]   data_size: Number of bytes to write.
 * \param[in]   data: Input data to write.
 * \param[out]  none
 * \retval      MC_STATE0 value.
 *******************************************************************/
sfx_u8 S2LPSpiWriteFifo(sfx_u8 data_size, sfx_u8 *data);

/*!******************************************************************
 * \fn sfx_u8 S2LPSpiReadFifo(sfx_u8 data_size, sfx_u8* data)
 * \brief Read S2LP FIFO.
 * \param[in]   data_size: Number of bytes to read.
 * \param[out]  data: Pointer to the read data.
 * \retval      MC_STATE0 value.
 *******************************************************************/
sfx_u8 S2LPSpiReadFifo(sfx_u8 data_size, sfx_u8 *data);

#endif /* __S2LP_CORE_SPI_H__ */
