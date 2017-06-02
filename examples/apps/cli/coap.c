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

#ifdef OPENTHREAD_CONFIG_FILE
#include OPENTHREAD_CONFIG_FILE
#else
#include <openthread-config.h>
#endif

#include <assert.h>
#include <stdlib.h>

#include <openthread/types.h>
#include <openthread/coap.h>
#include <openthread/openthread.h>
#include <openthread/platform/platform.h>

#include "boards/cc2538dk/board-cc2538dk.h"

/**
 * Context information to be passed to our handlers.
 */
struct CoapHandlerContext {
    otInstance *aInstance;
    uint8_t leds;
};

/**
 * The instance of our context information structure.
 */
static struct CoapHandlerContext instanceContext =
{
    .aInstance = NULL,
    .leds = 0
};

/**
 * Our default handler, this will display a page for all requests.
 */
static void defaultHandler(
        void *aContext, otCoapHeader *aHeader, otMessage *aMessage,
        const otMessageInfo *aMessageInfo);

/**
 * Handler for an ambient light sensor.
 */
static void ambientLightSensorHandler(
        void *aContext, otCoapHeader *aHeader, otMessage *aMessage,
        const otMessageInfo *aMessageInfo);
/**
 * Resource definition for the ambient light sensor.
 */
static otCoapResource ambientLightSensorResource = {
    .mUriPath = "als",
    .mHandler = &ambientLightSensorHandler,
    .mContext = (void*)&instanceContext,
    .mNext = NULL
};

/**
 * Handler for LEDs.
 */
static void ledsHandler(
        void *aContext, otCoapHeader *aHeader, otMessage *aMessage,
        const otMessageInfo *aMessageInfo);
/**
 * Resource definition for the LEDs.
 */
static otCoapResource ledsResource = {
    .mUriPath = "leds",
    .mHandler = &ledsHandler,
    .mContext = (void*)&instanceContext,
    .mNext = NULL
};

otError coapExampleInit(otInstance *aInstance)
{
    instanceContext.aInstance = aInstance;
    otCoapSetDefaultHandler(aInstance, &defaultHandler, (void*)&instanceContext);
    otCoapAddResource(aInstance, &ambientLightSensorResource);
    otCoapAddResource(aInstance, &ledsResource);
    return otCoapStart(aInstance, OT_DEFAULT_COAP_PORT);
}

static void defaultHandler(
        void *aContext, otCoapHeader *aHeader, otMessage *aMessage,
        const otMessageInfo *aMessageInfo)
{
    /* We ignore the message content */
    (void)aMessage;

    /* Pick up our context passed in earlier */
    struct CoapHandlerContext *handlerContext =
        (struct CoapHandlerContext*)aContext;

    /*
     * The default handler.  We need to know:
     * - what type of message this is (Confirmable, Non-confirmable,
     *   Acknowlegement or Reset)
     * - what type of request this is (GET, PUT, POST, DELETE, … etc).
     *
     * For this simple demo, we only care about confirmable requests, as these
     * are what carry our HTTP requests.  We will reply with a so-called
     * "piggy-back" response by appending it to the ACK reply we send.
     */

    if (otCoapHeaderGetType(aHeader) != kCoapTypeConfirmable)
    {
        /* Not a confirmable request, so ignore it. */
        return;
    }

    switch (otCoapHeaderGetCode(aHeader)) {
        case kCoapRequestGet:   /* A GET request */
            {
                /*
                 * In our case, we don't care about the message, we just
                 * send a reply.  We need to copy the message ID and token
                 * from the original message.  The reply is an ACK with
                 * content.
                 */
                otCoapHeader replyHeader;
                otMessage *replyMessage;

                otCoapHeaderInit(
                        &replyHeader,
                        kCoapTypeAcknowledgment,
                        kCoapResponseContent
                );

                otCoapHeaderSetToken(&replyHeader,
                        otCoapHeaderGetToken(aHeader),
                        otCoapHeaderGetTokenLength(aHeader)
                );

                otCoapHeaderSetMessageId(
                        &replyHeader,
                        otCoapHeaderGetMessageId(aHeader)
                );

                replyMessage = otCoapNewMessage(
                        handlerContext->aInstance, &replyHeader
                );

                if (replyMessage)
                {
                    otError result;
                    /* Our reply is tacked onto the end. */
                    result = otMessageAppend(replyMessage,
                            "Hello World!", 12);

                    if (result != OT_ERROR_NONE)
                    {
                        /* All good, now send it */
                        result = otCoapSendResponse(
                                handlerContext->aInstance, replyMessage,
                                aMessageInfo);
                    }

                    if (result != OT_ERROR_NONE)
                    {
                        /* There was an issue above, free up the message */
                        otMessageFree(replyMessage);
                    }
                }
            }
            break;
        default:
            break;
    }
}

static void ambientLightSensorHandler(
        void *aContext, otCoapHeader *aHeader, otMessage *aMessage,
        const otMessageInfo *aMessageInfo)
{
    /* Ignore message */
    (void)aMessage;

    /* Pick up our context passed in earlier */
    struct CoapHandlerContext *handlerContext =
        (struct CoapHandlerContext*)aContext;

    if (otCoapHeaderGetType(aHeader) != kCoapTypeConfirmable)
    {
        /* Not a confirmable request, so ignore it. */
        return;
    }

    switch (otCoapHeaderGetCode(aHeader)) {
        case kCoapRequestGet:   /* A GET request */
            {
                /*
                 * In our case, we don't care about the message, we just
                 * send a reply.  We need to copy the message ID and token
                 * from the original message.
                 */
                otCoapHeader replyHeader;
                otMessage *replyMessage;

                otCoapHeaderInit(
                        &replyHeader,
                        kCoapTypeAcknowledgment,
                        kCoapResponseContent
                );

                otCoapHeaderSetToken(&replyHeader,
                        otCoapHeaderGetToken(aHeader),
                        otCoapHeaderGetTokenLength(aHeader)
                );

                otCoapHeaderSetMessageId(
                        &replyHeader,
                        otCoapHeaderGetMessageId(aHeader)
                );

                replyMessage = otCoapNewMessage(
                        handlerContext->aInstance, &replyHeader);

                if (replyMessage)
                {
                    otError result;
                    uint16_t reading = cc2538dkAlsRead();
                    result = otMessageAppend(replyMessage,
                            &reading, sizeof(reading));

                    if (result != OT_ERROR_NONE)
                    {
                        /* All good, now send it */
                        result = otCoapSendResponse(
                                handlerContext->aInstance, replyMessage,
                                aMessageInfo);
                    }

                    if (result != OT_ERROR_NONE)
                    {
                        /* There was an issue above, free up the message */
                        otMessageFree(replyMessage);
                    }
                }
            }
            break;
        default:
            break;
    }
}

/**
 * Defining how to reply to a LEDs request… we do this here as we'll need
 * to do it a few times.
 */
static void ledsReplyHandler(struct CoapHandlerContext *handlerContext,
        otCoapHeader *aHeader, const otMessageInfo *aMessageInfo)
{
    otCoapHeader replyHeader;
    otMessage *replyMessage;

    otCoapHeaderInit(
            &replyHeader,
            kCoapTypeAcknowledgment,
            kCoapResponseContent
    );

    otCoapHeaderSetToken(&replyHeader,
            otCoapHeaderGetToken(aHeader),
            otCoapHeaderGetTokenLength(aHeader)
    );

    otCoapHeaderSetMessageId(
            &replyHeader,
            otCoapHeaderGetMessageId(aHeader)
    );

    replyMessage = otCoapNewMessage(
            handlerContext->aInstance, &replyHeader
    );

    if (replyMessage)
    {
        otError result;
        uint8_t leds = handlerContext->leds;
        result = otMessageAppend(
                replyMessage, &leds, sizeof(leds)
        );

        if (result != OT_ERROR_NONE)
        {
            /* All good, now send it */
            result = otCoapSendResponse(
                    handlerContext->aInstance, replyMessage,
                    aMessageInfo
            );
        }

        if (result != OT_ERROR_NONE)
        {
            /* There was an issue above, free up the message */
            otMessageFree(replyMessage);
        }
    }
}

static void ledsHandler(
        void *aContext, otCoapHeader *aHeader, otMessage *aMessage,
        const otMessageInfo *aMessageInfo)
{
    /* Pick up our context passed in earlier */
    struct CoapHandlerContext *handlerContext =
        (struct CoapHandlerContext*)aContext;

    if (otCoapHeaderGetType(aHeader) != kCoapTypeConfirmable)
    {
        /* Not a confirmable request, so ignore it. */
        return;
    }

    switch (otCoapHeaderGetCode(aHeader)) {
        case kCoapRequestGet:   /* A GET request */
            ledsReplyHandler(handlerContext, aHeader, aMessageInfo);
            break;
        case kCoapRequestPost:  /* A POST request */
            {
                /*
                 * Inspect the message, we should be given two bytes, one
                 * with an operation code, the other with the operand.  We
                 * read it in as a stream of bytes (as there's no guarantee
                 * a struct will order the data the same way).
                 *
                 * The message data starts at a given offset, which we must
                 * retrieve first.
                 */
                char raw[2];
                int read = otMessageRead(aMessage,
                    otMessageGetOffset(aMessage),
                    &raw, sizeof(raw));
                if (read == sizeof(raw))
                {
                    uint8_t leds = handlerContext->leds;
                    /* Decode the mask in byte 2: hex */
                    uint8_t mask = 0;
                    if ((raw[1] >= '0') && (raw[1] <= '9'))
                    {
                        mask = raw[1] - '0';
                    }
                    else if ((raw[1] >= 'A') && (raw[1] <= 'F'))
                    {
                        mask = raw[1] - 'A' + 10;
                    }
                    else if ((raw[1] >= 'a') && (raw[1] <= 'f'))
                    {
                        mask = raw[1] - 'a' + 10;
                    }

                    /* We'll make the first byte the op code */
                    switch (raw[0])
                    {
                        case '1': /* turn on LEDs */
                            leds |= mask;
                            break;
                        case '0': /* turn off LEDs */
                            leds &= ~mask;
                            break;
                        case 't': /* toggle LEDs */
                            leds ^= mask;
                            break;
                    }

                    /* Update the state of the LEDs if they changed */
                    if (leds != handlerContext->leds)
                    {
                        if (leds & (1 << 0))
                            cc2538GpioSetPin(CC2538DK_LED1_PORT,
                                    CC2538DK_LED1_PIN);
                        else
                            cc2538GpioClearPin(CC2538DK_LED1_PORT,
                                    CC2538DK_LED1_PIN);

                        if (leds & (1 << 1))
                            cc2538GpioSetPin(CC2538DK_LED2_PORT,
                                    CC2538DK_LED2_PIN);
                        else
                            cc2538GpioClearPin(CC2538DK_LED2_PORT,
                                    CC2538DK_LED2_PIN);

                        if (leds & (1 << 2))
                            cc2538GpioSetPin(CC2538DK_LED3_PORT,
                                    CC2538DK_LED3_PIN);
                        else
                            cc2538GpioClearPin(CC2538DK_LED3_PORT,
                                    CC2538DK_LED3_PIN);

                        if (leds & (1 << 3))
                            cc2538GpioSetPin(CC2538DK_LED4_PORT,
                                    CC2538DK_LED4_PIN);
                        else
                            cc2538GpioClearPin(CC2538DK_LED4_PORT,
                                    CC2538DK_LED4_PIN);
                    }
                }
                ledsReplyHandler(handlerContext, aHeader, aMessageInfo);
            }
            break;
        default:
            break;
    }
}
