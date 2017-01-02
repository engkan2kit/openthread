/*
 *  Copyright (c) 2017, Zolertia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

// I2C values
#define I2C_100KHZ_AT_16MHZ 0x07
#define I2C_400KHZ_AT_16MHZ 0x02

#define I2C_RECEIVE         0x01
#define I2C_SEND            0x00

#define I2C_MASTER_ERR_NONE 0x00000000
#define I2CM_STAT_BUSY      0x00000001
#define I2CM_STAT_ERROR     0x00000002
#define I2CM_STAT_ADRACK    0x00000004
#define I2CM_STAT_DATACK    0x00000008
#define I2CM_STAT_ARBLST    0x00000010
#define I2CM_STAT_IDLE      0x00000020
#define I2CM_STAT_BUSBSY    0x00000040
#define I2CM_STAT_INVALID   0x00000080

#define I2C_CMD_SINGLE_SEND              0x00000007
#define I2C_CMD_SINGLE_RECEIVE           0x00000007
#define I2C_CMD_BURST_SEND_START         0x00000003
#define I2C_CMD_BURST_SEND_CONT          0x00000001
#define I2C_CMD_BURST_SEND_FINISH        0x00000005
#define I2C_CMD_BURST_SEND_ERROR_STOP    0x00000004
#define I2C_CMD_BURST_RECEIVE_START      0x0000000b
#define I2C_CMD_BURST_RECEIVE_CONT       0x00000009
#define I2C_CMD_BURST_RECEIVE_FINISH     0x00000005
#define I2C_CMD_BURST_RECEIVE_ERROR_STOP 0x00000004

// I2C functions
uint8_t cc2538I2cSingleSend(uint8_t slave_addr, uint8_t data);
uint8_t cc2538I2cSingleReceive(uint8_t slave_addr, uint8_t *data);
uint8_t cc2538I2cBurstSend(uint8_t slave_addr, uint8_t *data, uint8_t len);
uint8_t cc2538I2cBurstReceive(uint8_t slave_addr, uint8_t *data, uint8_t len);
uint8_t cc2538I2cBusy(void);
void cc2538I2cMasterEnable(void);
void cc2538I2cPinInit(uint8_t sda_port, uint8_t sda_pin, uint8_t scl_port, uint8_t scl_pin);

#ifdef __cplusplus
} // end extern "C"
#endif
#endif // I2C_H_
