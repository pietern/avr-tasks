#ifndef _HD44780U_H
#define _HD44780U_H

#include <stdint.h>

/*
 * Hitachi HD44780U LCD controller driver.
 *
 * This implementation expects the LCD data lines to be hooked up
 * through a shift register. This only requires 2 lines to hook up the
 * board to the shift register (data and latch), and 2 lines to hook
 * up the board to the LCD to clock data in (instruction and latch).
 */

// Initialize LCD.
void lcd_init(void);

// Clear LCD display.
void lcd_clear_display(void);

// Return cursor to origin of LCD.
void lcd_return_home(void);

// Write character to LCD.
void lcd_write(char c);

// Write buffer to LCD and advance cursor to the next line.
void lcd_puts(const char *buf, int8_t len);

#endif
