#include <avr/interrupt.h>
#include <avr/io.h>

#include "task.h"

// Pointer to current task (user or idle).
// May only be changed by schedule routine.
static task_t *_task__current = 0;

// Pointer to user task.
// Since user tasks are linked, it doesn't matter which one this points to.
static task_t *_task__user = 0;

// Pointer to idle task.
// This task is scheduled when no other task can be scheduled.
static task_t *_task__idle = 0;

// Push a task's context onto its own stack.
static inline void task__push(void) __attribute__ ((always_inline));
static inline void task__push(void) {
  asm volatile(
    "push r0\n"

    // Save status register
    "in r0, 0x3f\n"
    "cli\n"
    "push r0\n"

    // Save general registers
    "push r1\n"
    "push r2\n"
    "push r3\n"
    "push r4\n"
    "push r5\n"
    "push r6\n"
    "push r7\n"
    "push r8\n"
    "push r9\n"
    "push r10\n"
    "push r11\n"
    "push r12\n"
    "push r13\n"
    "push r14\n"
    "push r15\n"
    "push r16\n"
    "push r17\n"
    "push r18\n"
    "push r19\n"
    "push r20\n"
    "push r21\n"
    "push r22\n"
    "push r23\n"
    "push r24\n"
    "push r25\n"
    "push r26\n"
    "push r27\n"
    "push r28\n"
    "push r29\n"
    "push r30\n"
    "push r31\n"

    // Compiler expects r1 to be zero. As this code may interrupt anything,
    // the register may temporarily hold a non-zero value.
    "clr r1\n"

    // Save stack pointer in current task struct
    "lds r26, _task__current\n" // Low
    "lds r27, _task__current+1\n" // High
    "in r0, 0x3d\n" // Low
    "st x+, r0\n"
    "in r0, 0x3e\n" // High
    "st x+, r0\n"
  );
}

// Pop a task's context off of its own stack.
static inline void task__pop(void) __attribute__ ((always_inline));
static inline void task__pop(void) {
  asm volatile(
    // Restore stack pointer from current task struct
    "lds r26, _task__current\n" // Low
    "lds r27, _task__current+1\n" // High
    "ld r0, x+\n"
    "out 0x3d, r0\n" // Low
    "ld r0, x+\n"
    "out 0x3e, r0\n" // High

    // Restore general registers
    "pop r31\n"
    "pop r30\n"
    "pop r29\n"
    "pop r28\n"
    "pop r27\n"
    "pop r26\n"
    "pop r25\n"
    "pop r24\n"
    "pop r23\n"
    "pop r22\n"
    "pop r21\n"
    "pop r20\n"
    "pop r19\n"
    "pop r18\n"
    "pop r17\n"
    "pop r16\n"
    "pop r15\n"
    "pop r14\n"
    "pop r13\n"
    "pop r12\n"
    "pop r11\n"
    "pop r10\n"
    "pop r9\n"
    "pop r8\n"
    "pop r7\n"
    "pop r6\n"
    "pop r5\n"
    "pop r4\n"
    "pop r3\n"
    "pop r2\n"
    "pop r1\n"

    // Restore status register
    "pop r0\n"
    "out 0x3f, r0\n"

    "pop r0\n"
  );
}

// Internal task initializer.
// Written in assembly so that it can mangle the stack pointer to initialize
// the task's stack. Note that interrupts are temporarily disabled.
static void * task__internal_initialize(void *sp, task_fn fn, void *data) {
  void *result;

  asm volatile(
    // Arguments
    // - r24:r25 -- Top of new task's stack
    // - r22:r23 -- Task function pointer
    // - r20:r21 -- Task argument
    // Returns:
    // - r24:r25 -- Top of new task's stack

    // About to overwrite the stack pointer// disable interrupts
    "in r18, 0x3f\n" // r18 can be clobbered
    "cli\n"

    // Save current stack pointer
    "in r26, 0x3d\n"
    "in r27, 0x3e\n"

    // Set new task's stack pointer
    "out 0x3d, %A1\n"
    "out 0x3e, %B1\n"

    // Store location of task body as return address, such that
    // executing "ret" after "context_restore" will jump to it.
    "push %A2\n" // low
    "push %B2\n" // high

    // Store r0
    "ldi r19, 0\n" // r19 can be clobbered
    "push r19\n"

    // Store status register
    "ldi r19, 0x80\n" // Start task with interrupts enabled
    "push r19\n"

    // Store general registers
    "ldi r19, 0\n"
    "push r19\n" // r1
    "push r19\n" // r2
    "push r19\n" // r3
    "push r19\n" // r4
    "push r19\n" // r5
    "push r19\n" // r6
    "push r19\n" // r7
    "push r19\n" // r8
    "push r19\n" // r9
    "push r19\n" // r10
    "push r19\n" // r11
    "push r19\n" // r12
    "push r19\n" // r13
    "push r19\n" // r14
    "push r19\n" // r15
    "push r19\n" // r16
    "push r19\n" // r17
    "push r19\n" // r18
    "push r19\n" // r19
    "push r19\n" // r20
    "push r19\n" // r21
    "push r19\n" // r22
    "push r19\n" // r23

    // Argument to task function
    "push %A3\n" // r24
    "push %B3\n" // r25

    "push r19\n" // r26
    "push r19\n" // r27
    "push r19\n" // r28
    "push r19\n" // r29
    "push r19\n" // r30
    "push r19\n" // r31

    // Store new task's stack pointer at return register
    "in %A0, 0x3d\n"
    "in %B0, 0x3e\n"

    // Restore stack pointer
    "out 0x3d, r26\n"
    "out 0x3e, r27\n"

    // Restore status register
    "out 0x3f, r18\n"

    : "=r" (result)
    : "r" (sp), "r" (fn), "r" (data)
    : "r18", "r19", "r26", "r27"
  );

  return result;
}

// Creates a task for the specified function.
task_t *task__internal_create(task_fn fn, void *data) {
  static void *start = (void *)(RAMEND - sizeof(task_t) + 1);
  void *sp;
  task_t *t;

  start -= 0x100; // Should protect this against underflow..
  sp = start-1;
  t = start;

  t->sp = task__internal_initialize(sp, fn, data);
  t->delay = 0;
  t->next = 0;

  return t;
}

// Creates a task for the specified function.
// Adds it to the list of user tasks.
task_t *task_create(task_fn fn, void *data) {
  task_t *t = task__internal_create(fn, data);

  if (_task__user == 0) {
    _task__user = t;
    t->next = t;
  } else {
    t->next = _task__user->next;
    _task__user->next = t;
  }

  return t;
}

static void task__schedule() {
  task_t *t = _task__user;

  while (1) {
    // Ignore sleeping task
    if (t->delay > 0) {
      goto next_task;
    }

    // Ignore current task
    if (t == _task__current) {
      goto next_task;
    }

    // Task t can be scheduled.
    _task__current = t;

    // Cycle _task__user to avoid starving tasks at the end of the list.
    _task__user = t;

    return;

  next_task:
    if (t->next == _task__user) {
      break;
    }

    t = t->next;
  }

  // Nothing to schedule, go to sleep.
  _task__current = _task__idle;

  return;
}

static void task__tick() {
  task_t *t = _task__user;

  while (1) {
    if (t->delay) {
      t->delay--;
    }

    if (t->next == _task__user) {
      break;
    }

    t = t->next;
  }
}

// Must be naked to avoid mangling the stack.
static void task__yield_from_timer(void) __attribute__((naked));
static void task__yield_from_timer(void) {
  task__push();

  task__tick();

  task__schedule();

  task__pop();

  asm volatile ("ret");
}

// ISR can be naked because the first thing it does is push the current task.
ISR(TIMER0_COMPA_vect, ISR_NAKED) {
  task__yield_from_timer();

  // This interrupt handler was saved as part of the task's state.
  // We know the task was interrupted, or this handler wouldn't have triggered.
  // Because SREG was stored from within the ISR, we know the Global Interrupt
  // Enable bit is not set and we have to re-enable it upon returning from the
  // interrupt handler. Hence, the RETI.
  asm volatile ("reti");
}

// Use TIMER0 for OS ticks.
// Configure it to trigger a Output Compare Register interrupt every 2ms.
// Assumes F_CPU == 16Mhz.
static void task__setup_timer() {
  // Waveform generation mode: CTC
  // WGM02: 0
  // WGM01: 1
  // WGM00: 0
  TCCR0A = _BV(WGM01);

  // Clock select: prescaler = 1/256
  // CS02: 1
  // CS01: 0
  // CS00: 0
  TCCR0B = _BV(CS02);

  // Output compare register
  // 0.002 * (F_CPU / 256) - 1
  OCR0A = 124;
}

static void task__idle(void *unused) {
  while (1) {
    // ZZZzzz...
    asm volatile ("sleep");
  }
}

static void task__create_idle_task() {
  _task__idle = task_create(task__idle, 0);
}

void task_initialize(void) {
  task__setup_timer();
  task__create_idle_task();
}

// Call this function to start task execution.
// Returns into the task pointed to by _task__current, not its caller.
void task_start(void) {
  if (_task__current == 0) {
    _task__current = _task__user;
  }

  task__pop();

  // Enable interrupt on OCR0A match
  TIMSK0 |= _BV(OCIE0A);

  asm volatile ("ret");
}

// Yield execution to any other schedulable task.
void task_yield(void) __attribute__((naked));
void task_yield(void) {
  task__push();

  task__schedule();

  task__pop();

  asm volatile ("ret");
}

// Make current task sleep for specified number of ticks.
void task_sleep(uint16_t ms) {
  _task__current->delay = ms / MS_PER_TICK;
  task_yield();
}
