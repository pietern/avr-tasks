#include <avr/io.h>
#include <stdint.h>

#include <task.h>
#include "hd44780u.h"

typedef enum {
  INSTRUCTION = 0x0,
  DATA        = 0x4,
} mode_t;

static void lcd_send(mode_t m, uint8_t b) {
  int8_t i;

  // Should be unrolled
  for (i = 0; i < 8; i++) {
    // Last bit first
    PORTB = (PORTB & 0b11111100) | (b & 0x80) >> 7;
    // Clock bit into shift register
    PORTB |= 0b00000010;
    b <<= 1;
  }

  // Last clock to move last bit to shift register output
  PORTB &= 0b11111100;
  PORTB |= 0b00000010;

  // Clock instruction into LCD
  PORTB |= m | 0b00001000;

  // Reset pins
  PORTB &= 0b11110000;
}

static void lcd_yield_usec(int16_t usec) {
  uint16_t t1;
  uint16_t t2;
  uint16_t dt;

  t1 = task_usec();
  for (;;) {
    task_yield();
    t2 = task_usec();

    if (t2 > t1) {
      dt = t2 - t1;
    } else {
      dt = t2 + (UINT16_MAX - t1) + 1;
    }

    if (dt >= usec) {
      break;
    }

    usec -= dt;
    t1 = t2;
  }
}

void lcd_init(void) {
  // Function set: 8-bit data, 2 display lines, 5x8 font
  lcd_send(INSTRUCTION, 0b00111000);
  lcd_yield_usec(37);

  // Display control: on, no cursor, no blink
  lcd_send(INSTRUCTION, 0b00001100);
  lcd_yield_usec(37);

  // Entry mode set: increment cursor by 1
  lcd_send(INSTRUCTION, 0b00000110);
  lcd_yield_usec(37);

  // Clear and initialize cursor
  lcd_clear_display();
  lcd_return_home();
}

void lcd_clear_display(void) {
  lcd_send(INSTRUCTION, 0b00000001);
  lcd_yield_usec(1520);
}

void lcd_return_home(void) {
  lcd_send(INSTRUCTION, 0b00000010);
  lcd_yield_usec(1520);
}

void lcd_write(char c) {
  lcd_send(DATA, c);
  lcd_yield_usec(37);
}

void lcd_puts(const char *buf, int8_t len) {
  int8_t i;

  for (i = 0; i < len; i++) {
    lcd_write(buf[i]);
  }

  // The LCD advances to the next line after character 40
  for (; i < 40; i++) {
    lcd_send(INSTRUCTION, 0b00010100);
    lcd_yield_usec(37);
  }
}
