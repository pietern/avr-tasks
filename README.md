# avr-tasks

Multitasking library for the Atmel AVR processor.

* Preemptive.
* Uses TIMER0 for scheduler ticks (default: 2ms per tick).
* Supports as many tasks as you can fit in RAM (default: 256 bytes per task).

An example can be found in [example.c](example.c).
This program blinks the LED connected to pin 13, with the actual blinking and
the blink rate controlled by two different tasks.

The code was made to run on an ATmega328p, without portability in mind.
The code can probably be ported to other variants without much effort.
As I only have ATmega328p devices (Arduino's), I don't have an incentive to do
this work. Pull requests are welcome.

Links to a few resources I used:

* [Multitasking on an AVR][1] -- outline for context save/restore routines
* [avr-gcc][2] -- details about the compiler
* [AVR instruction set][3]

[1]: http://www.avrfreaks.net/modules/FreaksArticles/files/14/Multitasking%20on%20an%20AVR.pdf
[2]: http://gcc.gnu.org/wiki/avr-gcc
[3]: http://www.atmel.com/images/doc0856.pdf

# Why?

* I want to be able to execute unrelated code while waiting for I/O but be
  interrupted when I/O _does_ happen.
* For fun.

# How?

* Every task is offset at some point in the stack.
* On task interruption, all relevant registers are pushed onto its stack.
* A scheduler figures out which task to run next.
* On task resume, all its context is popped off of its stack.

## License

MIT (see ``LICENSE``).