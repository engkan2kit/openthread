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

/**
 * @file
 *   This file implements a basic I2C driver
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "cc2538-reg.h"
#include "i2c.h"
#include "gpio.h"

static void delay(void)
{
    volatile int i;
    for(i = 0; i < 1200; ++i)
    {
      __asm("nop");
    }
}

static void cc2538I2cDataPut(uint8_t data)
{
    HWREG(I2CM_DR) = data;
}

static uint8_t cc2538I2cDataGet(void)
{
    return HWREG(I2CM_DR);
}

static void cc2538I2cCommand(uint8_t cmd)
{
    HWREG(I2CM_CTRL) = cmd;
    delay();
}

static void cc2538I2cSlaveAddress(uint8_t slave_addr, uint8_t access_mode)
{
    if(access_mode) {
      HWREG(I2CM_SA) = ((slave_addr << 1) | 1);
    } else {
      HWREG(I2CM_SA) = (slave_addr << 1);
    }
}

static uint8_t cc2538I2cError(void)
{
    uint8_t temp;
    temp = HWREG(I2CM_STAT);
    if (temp & I2CM_STAT_BUSY)
    {
        return I2C_MASTER_ERR_NONE;
    }
    else if (temp & (I2CM_STAT_ERROR | I2CM_STAT_ARBLST))
    {
        return temp;
    }
    return I2C_MASTER_ERR_NONE;
}

uint8_t cc2538I2cBusy(void)
{
    return HWREG(I2CM_STAT) & I2CM_STAT_BUSY;
}


uint8_t cc2538I2cSingleSend(uint8_t slave_addr, uint8_t data)
{
    cc2538I2cSlaveAddress(slave_addr, I2C_SEND);
    cc2538I2cDataPut(data);
    cc2538I2cCommand(I2C_CMD_SINGLE_SEND);

    while(cc2538I2cBusy());

    return cc2538I2cError();
}

uint8_t cc2538I2cSingleReceive(uint8_t slave_addr, uint8_t *data)
{
    uint8_t temp;

    cc2538I2cSlaveAddress(slave_addr, I2C_RECEIVE);
    cc2538I2cCommand(I2C_CMD_SINGLE_RECEIVE);

    while(cc2538I2cBusy());
    temp = cc2538I2cError();
    if(temp == I2C_MASTER_ERR_NONE) {
      *data = cc2538I2cDataGet();
    }
    return temp;
}

uint8_t cc2538I2cBurstSend(uint8_t slave_addr, uint8_t *data, uint8_t len)
{
    uint8_t sent;

    if((len == 0) || (data == 0)) {
      return I2CM_STAT_INVALID;
    }

    if(len == 1) {
      return cc2538I2cSingleSend(slave_addr, data[0]);
    }

    cc2538I2cSlaveAddress(slave_addr, I2C_SEND);
    cc2538I2cDataPut(data[0]);
    cc2538I2cCommand(I2C_CMD_BURST_SEND_START);

    while(cc2538I2cBusy());

    if(cc2538I2cError() == I2C_MASTER_ERR_NONE) {
      for(sent = 1; sent <= (len - 2); sent++) {
        cc2538I2cDataPut(data[sent]);
        cc2538I2cCommand(I2C_CMD_BURST_SEND_CONT);
        while(cc2538I2cBusy());
      }
  
      cc2538I2cDataPut(data[len - 1]);
      cc2538I2cCommand(I2C_CMD_BURST_SEND_FINISH);
      while(cc2538I2cBusy());
    }

    return cc2538I2cError();
}

uint8_t cc2538I2cBurstReceive(uint8_t slave_addr, uint8_t *data, uint8_t len)
{
    uint8_t recv = 0;
    if((len == 0) || data == 0) {
      return I2CM_STAT_INVALID;
    }

    if(len == 1) {
      return cc2538I2cSingleReceive(slave_addr, &data[0]);
    }

    cc2538I2cSlaveAddress(slave_addr, I2C_RECEIVE);
    cc2538I2cCommand(I2C_CMD_BURST_RECEIVE_START);

    while(cc2538I2cBusy());
    if(cc2538I2cError() == I2C_MASTER_ERR_NONE) {
      data[0] = cc2538I2cDataGet();

      for(recv = 1; recv <= (len - 2); recv++) {
        cc2538I2cCommand(I2C_CMD_BURST_RECEIVE_CONT);
        while(cc2538I2cBusy());
        data[recv] = cc2538I2cDataGet();
      }

      cc2538I2cCommand(I2C_CMD_BURST_RECEIVE_FINISH);
      while(cc2538I2cBusy());
      data[len - 1] = cc2538I2cDataGet();
    }
    return cc2538I2cError();
}

void cc2538I2cMasterEnable(void)
{
    HWREG(I2CM_CR) = 0x10;
}

void cc2538I2cPinInit(uint8_t sda_port, uint8_t sda_pin, uint8_t scl_port,
                      uint8_t scl_pin)
{
    HWREG(SYS_CTRL_RCGCI2C) |= 1;
    HWREG(SYS_CTRL_DCGCI2C) |= 1;

    HWREG(SYS_CTRL_SRI2C) |= 1;

    delay();

    HWREG(SYS_CTRL_SRI2C) &= ~1;

    cc2538GpioDirInput(sda_port, sda_pin);
    cc2538GpioDirInput(scl_port, scl_pin);

    cc2538GpioIocOver(sda_port, sda_pin, IOC_OVERRIDE_DIS);
    cc2538GpioIocOver(scl_port, scl_pin, IOC_OVERRIDE_DIS);

    cc2538GpioHardwareControl(sda_port, sda_pin);
    cc2538GpioHardwareControl(scl_port, scl_pin);

    HWREG(IOC_I2CMSSDA) = ((sda_port << 3) | sda_pin);
    HWREG(IOC_I2CMSSCL) = ((scl_port << 3) | scl_pin);

    cc2538GpioIocSel(sda_port, sda_pin, IOC_MUX_SEL_I2C_CMSSDA);
    cc2538GpioIocSel(scl_port, scl_pin, IOC_MUX_SEL_I2C_CMSSCL);

    cc2538I2cMasterEnable();
    HWREG(I2CM_TPR) = I2C_100KHZ_AT_16MHZ;
}

