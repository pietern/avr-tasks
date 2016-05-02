#include <string.h>

#include "readline.h"
#include "uart.h"

#define VT100_ERASE_EOL ("\x1b[K")
#define VT100_CURSOR_FORWARD ("\x1b[C")
#define VT100_CURSOR_BACKWARD ("\x1b[D")

int8_t readline(const char *prompt, char *buf, int8_t bufsz) {
  char tbuf[6];
  int8_t tlen;

  if (prompt != NULL) {
    uart_write(prompt, strlen(prompt));
  }

  int8_t len = 0;
  int8_t pos = 0;

  for (;;) {
    char c = uart_getc(NULL);
    char c_;

    if (c >= 0x20 && c < 0x7f) {
      // Readable character
      if (len >= bufsz) {
        continue;
      }

      if (pos < len) {
        int8_t tail = len - pos;

        // Move tail one character to the end
        memmove(&buf[pos+1], &buf[pos], tail);
        buf[pos] = c;
        uart_write(VT100_ERASE_EOL, sizeof(VT100_ERASE_EOL));
        uart_write(&buf[pos], tail+1);
        tlen = snprintf(tbuf, sizeof(tbuf), "\x1b[%dD", tail);
        uart_write(tbuf, tlen);
      } else {
        // Add to tail
        buf[pos] = c;
        uart_write(&c, 1);
      }
      pos++;
      len++;
    } else if (c == 0x0d) {
      // Write CRLF to confirm
      uart_write("\r\n", 2);
      break;
    } else if (c == 0x08) {
      // Backspace
      if (pos == 0) {
        continue;
      }

      if (pos < len) {
        int8_t tail = len - pos;

        // Move tail one character to the start
        memmove(&buf[pos-1], &buf[pos], tail);
        uart_write(VT100_CURSOR_BACKWARD, sizeof(VT100_CURSOR_BACKWARD));
        uart_write(VT100_ERASE_EOL, sizeof(VT100_ERASE_EOL));
        uart_write(&buf[pos-1], tail);
        tlen = snprintf(tbuf, sizeof(tbuf), "\x1b[%dD", tail);
        uart_write(tbuf, tlen);
      } else {
        // Remove from tail
        uart_write(VT100_CURSOR_BACKWARD, sizeof(VT100_CURSOR_BACKWARD));
        uart_write(VT100_ERASE_EOL, sizeof(VT100_ERASE_EOL));
      }
      pos--;
      len--;
    } else if (c == 0x1b) {
      // Escape sequence
      c_ = uart_getc(NULL);
      if (c_ != '[') {
        // Discard
        continue;
      }

      c_ = uart_getc(NULL);
      if (c_ == 'C') {
        // Move cursor forward one char
        if (pos < len) {
          pos++;

          // Confirm
          uart_write("\x1b[C", 3);
        }
      } else if (c_ == 'D') {
        // Move cursor backward one char
        if (pos > 0) {
          pos--;

          // Confirm
          uart_write("\x1b[D", 3);
        }
      } else {
        // Not handled
      }
    } else {
      // Not handled
    }
  }

  return len;
}
