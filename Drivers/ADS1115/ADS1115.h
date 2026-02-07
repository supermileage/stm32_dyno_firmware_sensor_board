// I2Cdev library collection - ADS1115 I2C device class header file
// Based on Texas Instruments ADS1113/4/5 datasheet, May 2009 (SBAS444B, revised October 2009)
// Note that the ADS1115 uses 16-bit registers, not 8-bit registers.
// 8/2/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2013-05-05 - Add debug information.  Clean up Single Shot implementation
//     2011-10-29 - added getDifferentialx() methods, F. Farzanegan
//     2011-08-02 - initial release


/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef ADS1115_ADS1115_H_
#define ADS1115_ADS1115_H_

#include <stdint.h>
#include <stdbool.h>

#include "ADS1115_defines.h"

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif


bool ADS1115_Constr(I2C_HandleTypeDef* i2cHandle, uint8_t address);

bool ADS1115_Initialize();
bool ADS1115_TestConnection();

// SINGLE SHOT utilities
bool ADS1115_PollConversion(uint16_t max_retries);
bool ADS1115_TriggerConversion();

// Read the current CONVERSION register
bool ADS1115_GetConversion(int16_t* value, bool triggerAndPoll);

// Differential
bool ADS1115_GetConversionP0N1(int16_t* value);
bool ADS1115_GetConversionP0N3(int16_t* value);
bool ADS1115_GetConversionP1N3(int16_t* value);
bool ADS1115_GetConversionP2N3(int16_t* value);
// Single-ended
bool ADS1115_GetConversionP0GND(int16_t* value);
bool ADS1115_GetConversionP1GND(int16_t* value);
bool ADS1115_GetConversionP2GND(int16_t* value);
bool ADS1115_GetConversionP3GND(int16_t* value);

// Utility
float ADS1115_GetMilliVolts(bool triggerAndPoll);
float ADS1115_GetMvPerCount();

// CONFIG register
bool ADS1115_IsConversionReady();
bool ADS1115_GetMultiplexer(uint8_t* muxMode);
bool ADS1115_SetMultiplexer(uint8_t mux);
bool ADS1115_GetGain(uint8_t* gain);
bool ADS1115_SetGain(uint8_t gain);
bool ADS1115_GetMode();
bool ADS1115_SetMode(bool mode);
bool ADS1115_GetRate(uint8_t* rate);
bool ADS1115_SetRate(uint8_t rate);
bool ADS1115_GetComparatorMode();
bool ADS1115_SetComparatorMode(bool mode);
bool ADS1115_GetComparatorPolarity();
bool ADS1115_SetComparatorPolarity(bool polarity);
bool ADS1115_GetComparatorLatchEnabled();
bool ADS1115_SetComparatorLatchEnabled(bool enabled);
bool ADS1115_GetComparatorQueueMode(uint8_t* mode);
bool ADS1115_SetComparatorQueueMode(uint8_t mode);
bool ADS1115_SetConversionReadyPinMode();

// *_THRESH registers
bool ADS1115_GetLowThreshold(int16_t* threshold);
bool ADS1115_SetLowThreshold(int16_t threshold);
bool ADS1115_GetHighThreshold(int16_t* threshold);
bool ADS1115_SetHighThreshold(int16_t threshold);

// DEBUG
bool ADS1115_ShowConfigRegister();


#ifdef __cplusplus
}
#endif   

#endif /* ADS1115_ADS1115_H_ */

