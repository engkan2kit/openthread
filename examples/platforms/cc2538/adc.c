/*
 *  Copyright (c) 2016, Zolertia
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
 *   This file implements a basic ADC driver
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "cc2538-reg.h"
#include "adc.h"
#include "gpio.h"

void cc2538AdcPinInit(uint8_t port, uint8_t pin)
{
    cc2538GpioSoftwareControl(port, pin);
    cc2538GpioDirInput(port, pin);
    cc2538GpioIocOver(port, pin, IOC_OVERRIDE_ANA);
    HWREG(SOC_ADC_ADCCON1) |= SOC_ADC_ADCCON1_STSEL;
}

int16_t cc2538AdcReadChannel(uint8_t channel)
{
    int16_t res;

    HWREG(SOC_ADC_ADCCON3) = HWREG(SOC_ADC_ADCCON3) &
                             ~(SOC_ADC_ADCCON3_EREF | SOC_ADC_ADCCON3_EDIV |
                             SOC_ADC_ADCCON3_ECH);
    HWREG(SOC_ADC_ADCCON3) |= SOC_ADC_ADCCON_REF;
    HWREG(SOC_ADC_ADCCON3) |= SOC_ADC_ADCCON_DIV;
    HWREG(SOC_ADC_ADCCON3) |= channel;

    while(!(HWREG(SOC_ADC_ADCCON1) & SOC_ADC_ADCCON1_EOC));

    res  = HWREG(SOC_ADC_ADCL_ADC) & 0xFC;
    res |= HWREG(SOC_ADC_ADCH_ADC) << 8;
    return res;
}
