# avr-tasks

Multitasking library for the Atmel AVR processor.

* Preemptive.
* Uses TIMER0 for scheduler ticks (default: 2ms per tick).
* Supports as many tasks as you can fit in RAM (default: 256 bytes per task).
* Expects to be run on an ATmega328p.
* For missing features, see _TODO_ below.

## Example

An example can be found in [blink.c](examples/blink/blink.c).
This program blinks the LED connected to pin 13, with the actual blinking and
the blink rate controlled by two different tasks.

Find more examples in the [examples](examples/) directory.

## Why?

* To be able to execute arbitrary code while waiting for I/O but be
  interrupted when I/O _does_ happen.

## How?

* Every task is offset at some point in the stack.
* On task interruption, all relevant registers are pushed onto its stack.
* A scheduler figures out which task to run next.
* On task resume, all its context is popped off of its stack.

## Hacking

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

## TODO

* Communication / synchronization between tasks
* Add task status field for run/sleep/wait/etc...
* If waiting for I/O, how will a task be woken up? Likely through ISR not known
  to `task.c`. Maybe just call `task_yield()` from that handler. Maybe have
  some condition variable like apparatus to associate event X with task Y
  waiting for that event.

## License

MIT (see ``LICENSE``).
