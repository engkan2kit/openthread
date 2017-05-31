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
 *   This file includes the board-specific initializers for the TI CC2538DK.
 *
 */

#ifndef BOARD_CC2538DK_H_
#define BOARD_CC2538DK_H_

#include <stdint.h>

#include <openthread/types.h>

#include "../../platform-cc2538.h"
#include "../../gpio.h"
#include "../../adc.h"

/* ----- LEDs ----- */

/* LED port and pin definitions */
#define CC2538DK_LED1_PORT      (2)     /**< LED1 port number */
#define CC2538DK_LED1_PIN       (0)     /**< LED1 pin number */
#define CC2538DK_LED2_PORT      (2)     /**< LED2 port number */
#define CC2538DK_LED2_PIN       (1)     /**< LED2 pin number */
#define CC2538DK_LED3_PORT      (2)     /**< LED3 port number */
#define CC2538DK_LED3_PIN       (2)     /**< LED3 pin number */
#define CC2538DK_LED4_PORT      (2)     /**< LED4 port number */
#define CC2538DK_LED4_PIN       (3)     /**< LED4 pin number */

/**
 * Initialise the LED GPIOs as outputs.
 */
static inline void cc2538dkLedInit() {
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

/* ----- Buttons ----- */

#define CC2538_BTN_LEFT_PORT    (2)     /**< LEFT pushbutton port number */
#define CC2538_BTN_LEFT_PIN     (4)     /**< LEFT pushbutton pin number */
#define CC2538_BTN_RIGHT_PORT   (2)     /**< RIGHT pushbutton port number */
#define CC2538_BTN_RIGHT_PIN    (5)     /**< RIGHT pushbutton pin number */
#define CC2538_BTN_UP_PORT      (2)     /**< UP pushbutton port number */
#define CC2538_BTN_UP_PIN       (6)     /**< UP pushbutton pin number */
#define CC2538_BTN_DOWN_PORT    (2)     /**< DOWN pushbutton port number */
#define CC2538_BTN_DOWN_PIN     (7)     /**< DOWN pushbutton pin number */
#define CC2538_BTN_SELECT_PORT  (0)     /**< SELECT pushbutton port number */
#define CC2538_BTN_SELECT_PIN   (3)     /**< SELECT pushbutton pin number */

/**
 * Initialise the pushbutton GPIOs as outputs.
 */
static inline void cc2538dkButtonInit() {
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

/* ----- Light Sensor (Analogue) ----- */

#define CC2538DK_ALS_POWER_PORT (0)     /**< ALS power enable port number */
#define CC2538DK_ALS_POWER_PIN  (7)     /**< ALS power enable pin number */
#define CC2538DK_ALS_SIG_PORT   (0)     /**< ALS signal port number */
#define CC2538DK_ALS_SIG_PIN    (6)     /**< ALS signal pin number */

/**
 * Initialise the ambient light sensor.
 */
static inline void cc2538dkAlsInit() {
    /* Make the signal pin an analogue input */
    cc2538AdcPinInit(CC2538DK_ALS_SIG_PORT, CC2538DK_ALS_SIG_PIN);
    /* Make the power enable pin an output and turn it on */
    cc2538GpioSoftwareControl(CC2538DK_ALS_POWER_PORT, CC2538DK_ALS_POWER_PIN);
    cc2538GpioDirOutput(CC2538DK_ALS_POWER_PORT, CC2538DK_ALS_POWER_PIN);
    cc2538GpioSetPin(CC2538DK_ALS_POWER_PORT, CC2538DK_ALS_POWER_PIN);
}

/**
 * Read the ambient light level.
 */
static inline int16_t cc2538dkAlsRead() {
    return cc2538AdcReadChannel(CC2538DK_ALS_SIG_PIN);
}

#endif  // BOARD_CC2538DK_H_
