/* vim: set tw=78 sw=4 ts=4 et:
 *  Copyright (c) 2017, VRT Systems
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

#ifndef AES_H_
#define AES_H_

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include "cc2538-reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Enable the AES crypto engine
 */
void cc2538AesEnable();

/**
 * Disable the AES crypto engine
 */
void cc2538AesDisable();

/**
 * Determine the result of the hash operation.
 *
 * @retval      0               Operation competed successfully
 * @retval      -EINPROGRESS    Device is still busy performing an operation
 * @retval      -EIO            Memory transfer failed
 */
int32_t cc2538AesHashStatus();

/**
 * Clear the state of the hashing registers.  This acknowledges the related
 * interrupt flags and resets the state ready for use.
 */
void cc2538AesHashFinish();

/**
 * Begin hashing a given block of data, returning the hash value to the given
 * block of memory.  The SHA-256 hash of the data block will be computed in
 * hardware and returned to the given block of memory.
 *
 * The following assumptions are made:
 * - the block will not be re-located or modified during the operation
 * - the block start address is correctly aligned (32-bit word alignment)
 * - Both input and output hash buffers are 32-bit word aligned.
 *
 * @param[in]   dataIn  Data input to be hashed
 * @param[in]   dataLen Length of data to be hashed
 * @param[in]   hashIn  Initial hash value input, NULL if creating a new hash.
 * @param[out]  hashOut Hash value output.
 * @param[in]   hashLen Hash value length.
 * @param[in]   pad     Whether or not to pad the data
 *
 * @retval      0       Operation begun
 * @retval      -EBUSY  Device is busy with another operation
 */
int32_t cc2538AesHashStart(const void *dataIn, uint32_t dataLen,
        void *hashIn, void *hashOut, uint32_t hashLen, _Bool pad);

#ifdef __cplusplus
} // end extern "C"
#endif
#endif // AES_H_
