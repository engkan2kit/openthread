/*
 *  Copyright (c) 2016, The OpenThread Authors.
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
 * @brief
 *   This file includes the platform-specific initializers.
 */

#include "board-cc2538dk.h"

otInstance *sInstance;

void cc2538dkLedInit()
{
    /* Place under software control */
    cc2538GpioSoftwareControl(CC2538DK_LED1_PORT, CC2538DK_LED1_PIN);
    cc2538GpioSoftwareControl(CC2538DK_LED2_PORT, CC2538DK_LED2_PIN);
    cc2538GpioSoftwareControl(CC2538DK_LED3_PORT, CC2538DK_LED3_PIN);
    cc2538GpioSoftwareControl(CC2538DK_LED4_PORT, CC2538DK_LED4_PIN);
    /* Make the pins all outputs */
    cc2538GpioDirOutput(CC2538DK_LED1_PORT, CC2538DK_LED1_PIN);
    cc2538GpioDirOutput(CC2538DK_LED2_PORT, CC2538DK_LED2_PIN);
    cc2538GpioDirOutput(CC2538DK_LED3_PORT, CC2538DK_LED3_PIN);
    cc2538GpioDirOutput(CC2538DK_LED4_PORT, CC2538DK_LED4_PIN);
}

void cc2538dkButtonInit()
{
    /* Place under software control */
    cc2538GpioSoftwareControl(CC2538DK_BTN_LEFT_PORT, CC2538DK_BTN_LEFT_PIN);
    cc2538GpioSoftwareControl(CC2538DK_BTN_RIGHT_PORT, CC2538DK_BTN_RIGHT_PIN);
    cc2538GpioSoftwareControl(CC2538DK_BTN_UP_PORT, CC2538DK_BTN_UP_PIN);
    cc2538GpioSoftwareControl(CC2538DK_BTN_DOWN_PORT, CC2538DK_BTN_DOWN_PIN);
    cc2538GpioSoftwareControl(CC2538DK_BTN_SELECT_PORT, CC2538DK_BTN_SELECT_PIN);
    /* Make the pins all inputs */
    cc2538GpioDirInput(CC2538DK_BTN_LEFT_PORT, CC2538DK_BTN_LEFT_PIN);
    cc2538GpioDirInput(CC2538DK_BTN_RIGHT_PORT, CC2538DK_BTN_RIGHT_PIN);
    cc2538GpioDirInput(CC2538DK_BTN_UP_PORT, CC2538DK_BTN_UP_PIN);
    cc2538GpioDirInput(CC2538DK_BTN_DOWN_PORT, CC2538DK_BTN_DOWN_PIN);
    cc2538GpioDirInput(CC2538DK_BTN_SELECT_PORT, CC2538DK_BTN_SELECT_PIN);
}

void cc2538dkAlsInit()
{
    /* Make the signal pin an analogue input */
    cc2538AdcPinInit(CC2538DK_ALS_SIG_PORT, CC2538DK_ALS_SIG_PIN);
    /* Make the power enable pin an output and turn it on */
    cc2538GpioSoftwareControl(CC2538DK_ALS_POWER_PORT, CC2538DK_ALS_POWER_PIN);
    cc2538GpioDirOutput(CC2538DK_ALS_POWER_PORT, CC2538DK_ALS_POWER_PIN);
    cc2538GpioSetPin(CC2538DK_ALS_POWER_PORT, CC2538DK_ALS_POWER_PIN);
}

int16_t cc2538dkAlsRead()
{
    return cc2538AdcReadChannel(CC2538DK_ALS_SIG_PIN);
}

void cc2538BoardInit(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    cc2538dkLedInit();
    cc2538dkButtonInit();
    cc2538dkAlsInit();
}

void cc2538BoardProcess(otInstance *aInstance)
{
    static uint32_t timer = 0;
    static uint8_t led = 0;
    (void)aInstance;

    if (timer)
    {
        timer--;
    }
    else
    {
        timer = 10000;
        switch (led++) {
            case 0:
                cc2538GpioSetPin(CC2538DK_LED1_PORT, CC2538DK_LED1_PIN);
                cc2538GpioClearPin(CC2538DK_LED2_PORT, CC2538DK_LED2_PIN);
                cc2538GpioClearPin(CC2538DK_LED3_PORT, CC2538DK_LED3_PIN);
                cc2538GpioClearPin(CC2538DK_LED4_PORT, CC2538DK_LED4_PIN);
                break;
            case 1:
                cc2538GpioClearPin(CC2538DK_LED1_PORT, CC2538DK_LED1_PIN);
                cc2538GpioSetPin(CC2538DK_LED2_PORT, CC2538DK_LED2_PIN);
                cc2538GpioClearPin(CC2538DK_LED3_PORT, CC2538DK_LED3_PIN);
                cc2538GpioClearPin(CC2538DK_LED4_PORT, CC2538DK_LED4_PIN);
                break;
            case 2:
                cc2538GpioClearPin(CC2538DK_LED1_PORT, CC2538DK_LED1_PIN);
                cc2538GpioClearPin(CC2538DK_LED2_PORT, CC2538DK_LED2_PIN);
                cc2538GpioSetPin(CC2538DK_LED3_PORT, CC2538DK_LED3_PIN);
                cc2538GpioClearPin(CC2538DK_LED4_PORT, CC2538DK_LED4_PIN);
                break;
            case 3:
                cc2538GpioClearPin(CC2538DK_LED1_PORT, CC2538DK_LED1_PIN);
                cc2538GpioClearPin(CC2538DK_LED2_PORT, CC2538DK_LED2_PIN);
                cc2538GpioClearPin(CC2538DK_LED3_PORT, CC2538DK_LED3_PIN);
                cc2538GpioSetPin(CC2538DK_LED4_PORT, CC2538DK_LED4_PIN);
                break;
            default:
                cc2538GpioClearPin(CC2538DK_LED1_PORT, CC2538DK_LED1_PIN);
                cc2538GpioClearPin(CC2538DK_LED2_PORT, CC2538DK_LED2_PIN);
                cc2538GpioClearPin(CC2538DK_LED3_PORT, CC2538DK_LED3_PIN);
                cc2538GpioClearPin(CC2538DK_LED4_PORT, CC2538DK_LED4_PIN);
                led = 0;
        }
    }
}
