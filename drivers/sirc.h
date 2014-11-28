#ifndef _SIRC_H
#define _SIRC_H

/*
 * Sony IR code decoder.
 *
 * For a comprehensive introduction see:
 * http://www.righto.com/2010/03/understanding-sony-ir-remote-codes-lirc.html
 *
 * Also see:
 * http://www.hifi-remote.com/sony/
 *
 * This decoder implements bit decoding where pulses are considered to be
 * preceded by delays rather than pulses preceding delays. Since the last pulse
 * is followed by a delay of arbitrary length, that delay cannot trigger an
 * interrupt. If instead the header is viewed to be preceded by a delay of
 * arbitrary length, that delay can be ignored upon seeing the header pulse.
 */

#include <stdint.h>

/*
 * Pin state marking the start of a pulse.
 * If the pin is high when idle, the pulse starts with low (0).
 * If the pin is low when idle, the pulse starts with high (1).
 */
#ifndef _SIRC_PULSE_START
#define _SIRC_PULSE_START 0
#endif

// Length of header pulse.
#ifndef _SIRC_HEADER_PULSE_US
#define _SIRC_HEADER_PULSE_US 2400
#endif

// Header pulse error margin.
#ifndef _SIRC_HEADER_ERROR_US
#define _SIRC_HEADER_ERROR_US 200
#endif

// Length of one pulse.
#ifndef _SIRC_ONE_PULSE_US
#define _SIRC_ONE_PULSE_US 1200
#endif

// One pulse error margin.
#ifndef _SIRC_ONE_ERROR_US
#define _SIRC_ONE_ERROR_US 200
#endif

// Length of zero pulse.
#ifndef _SIRC_ZERO_PULSE_US
#define _SIRC_ZERO_PULSE_US 600
#endif

// Zero pulse error margin.
#ifndef _SIRC_ZERO_ERROR_US
#define _SIRC_ZERO_ERROR_US 200
#endif

// Length of delay between pulses.
#ifndef _SIRC_DELAY_US
#define _SIRC_DELAY_US 600
#endif

// Delay error margin.
#ifndef _SIRC_DELAY_ERROR_US
#define _SIRC_DELAY_ERROR_US 200
#endif

// Number of bits to capture.
#define BITS 12

void sirc_init();

uint16_t sirc_read();

#endif
