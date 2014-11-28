#include <avr/interrupt.h>
#include <avr/io.h>
#include <stddef.h>

#include <drivers/sirc.h>
#include <task.h>

// Length of header pulse.
#define HEADER_MIN_US (uint16_t)(_SIRC_HEADER_PULSE_US - _SIRC_HEADER_ERROR_US)
#define HEADER_MAX_US (uint16_t)(_SIRC_HEADER_PULSE_US + _SIRC_HEADER_ERROR_US)

// Length of one pulse.
#define ONE_MIN_US (uint16_t)(_SIRC_ONE_PULSE_US - _SIRC_ONE_ERROR_US)
#define ONE_MAX_US (uint16_t)(_SIRC_ONE_PULSE_US + _SIRC_ONE_ERROR_US)

// Length of zero pulse.
#define ZERO_MIN_US (uint16_t)(_SIRC_ZERO_PULSE_US - _SIRC_ZERO_ERROR_US)
#define ZERO_MAX_US (uint16_t)(_SIRC_ZERO_PULSE_US + _SIRC_ZERO_ERROR_US)

// Length of delay between pulses.
#define DELAY_MIN_US (uint16_t)(_SIRC_DELAY_US - _SIRC_DELAY_ERROR_US)
#define DELAY_MAX_US (uint16_t)(_SIRC_DELAY_US + _SIRC_DELAY_ERROR_US)

// Initialize the decoder.
void sirc_init() {
  // Configure DDB0 as input (port B, pin 0, Arduino pin 8).
  DDRB &= ~_BV(DDB0);
  PORTB |= _BV(PORTB0); // Pull-up

  // Enable pin change interrupts on bank 1 (port B)
  PCICR |= _BV(PCIE0);
}

// Enable pin change interrupt on bank 1, pin 0 (pin 8).
static void sirc__enable() {
  PCMSK0 |= _BV(PCINT0);
}

// Disable pin change interrupt on bank 1, pin 0 (pin 8).
static void sirc__disable() {
  PCMSK0 &= ~_BV(PCINT0);
}

// Task waiting for code.
static task_t *task = NULL;

// Bit index.
static uint8_t bit = 0;

// Code accumulator.
static uint16_t code = 0;

// Last interrupt trigger.
static uint16_t prev_us = 0;

// Most recent delay duration.
static uint16_t delay_us = 0;

ISR(PCINT0_vect) {
  uint16_t now_us, diff_us, pulse_us;

  now_us = task_us();
  if (prev_us < now_us) {
    diff_us = now_us - prev_us;
  } else {
    diff_us = now_us + (UINT16_MAX - prev_us);
  }

  // Store current time for next edge.
  prev_us = now_us;

  // Pin flipped to state for pulse start; store diff as delay time.
  if ((PINB & _BV(PINB0)) == _SIRC_PULSE_START) {
    delay_us = diff_us;
    return;
  }

  // Pin flipped to state for pulse end.
  pulse_us = diff_us;

  // Check for header if needed.
  if (bit == 0) {
    if (pulse_us >= HEADER_MIN_US && pulse_us <= HEADER_MAX_US) {
      bit = 1;
      code = 0;
    }
    return;
  }

  // Expect delay.
  if (delay_us < DELAY_MIN_US || delay_us > DELAY_MAX_US) {
    // Reset.
    bit = 0;
    return;
  }

  // Expect one or zero.
  if (pulse_us >= ONE_MIN_US && pulse_us <= ONE_MAX_US) {
    code |= (1 << (bit - 1));
  } else if (pulse_us >= ZERO_MIN_US && pulse_us <= ZERO_MAX_US) {
    // No-op.
  } else {
    // Reset.
    bit = 0;
    return;
  }

  // Wake up task after receiving enough bits.
  if (bit == BITS) {
    sirc__disable();
    task_wakeup(task);

    // Reset.
    bit = 0;
  } else {
    bit++;
  }
}

// Block until code is read.
uint16_t sirc_read() {
  sirc__enable();
  task = task_current();
  task_suspend(NULL);
  return code;
}
