#include <avr/interrupt.h>
#include <avr/io.h>

#include "task.h"

// Pointer to current task.
// May only be changed by schedule routine.
static task_t *_task__current = 0;

// Queue with runnable tasks.
// Holds tasks that may be scheduled immediately.
static QUEUE _tasks__runnable;

// Queue with suspended tasks.
// Holds tasks that called "task_suspend".
static QUEUE _tasks__suspended;

// Queue with sleeping tasks.
// Holds tasks that called "task_sleep".
static QUEUE _tasks__sleeping;

#if TASK_COUNT_SEC
static TASK_SEC_T _task_sec = 0;

// Counts down until a second has passed.
static uint16_t _task_sec_countdown;

TASK_SEC_T task_sec(void) {
  return _task_sec;
}

void task_set_sec(TASK_SEC_T t) {
  uint8_t sreg = SREG;

  cli();
  _task_sec = t;
  _task_sec_countdown = 1000 / MS_PER_TICK;
  SREG = sreg;
}
#endif // TASK_COUNT_SEC

#if TASK_COUNT_MSEC
static TASK_MSEC_T _task_msec = 0;

TASK_MSEC_T task_msec(void) {
  return _task_msec;
}

void task_set_msec(TASK_MSEC_T t) {
  uint8_t sreg = SREG;

  cli();
  _task_msec = t;
  SREG = sreg;
}
#endif // TASK_COUNT_MSEC

#if TASK_COUNT_USEC
static TASK_USEC_T _task_usec = 0;

TASK_USEC_T task_usec(void) {
  return (_task_usec + (TCNT0 * US_PER_COUNT));
}

void task_set_usec(TASK_USEC_T t) {
  uint8_t sreg = SREG;

  cli();
  _task_usec = t;
  SREG = sreg;
}
#endif // TASK_COUNT_USEC

// Push a task's context onto its own stack.
static inline void task__push(void) __attribute__ ((always_inline));
static inline void task__push(void) {
  asm volatile(
    "push r0\n"

    // Save status register
    "in r0, 0x3f\n"
    "cli\n"
    "push r0\n"

    // Save Z register pair (so it can be used below)
    "push r30\n"
    "push r31\n"

    // Load _task__current into Z register pair
    "lds r30, _task__current\n" // Low
    "lds r31, _task__current+1\n" // High

    // Z=1 if (r30 | r31) == 0
    "mov r0, r30\n"
    "or r0, r31\n"

    // Save if Z=0 (_task__current != NULL)
    "brne 2f\n"

  "1:\n"
    // Restore Z register pair
    "pop r31\n"
    "pop r30\n"

    // Restore status register and real r0
    "pop r0\n"
    "out 0x3f, r0\n"
    "pop r0\n"

    "rjmp 3f\n"

  "2:\n"
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

    // Compiler expects r1 to be zero. As this code may interrupt anything,
    // the register may temporarily hold a non-zero value.
    "clr r1\n"

    // Save stack pointer in current task struct
    "in r0, 0x3d\n" // Low
    "st z+, r0\n"
    "in r0, 0x3e\n" // High
    "st z+, r0\n"

  "3:\n"
  );
}

// Pop a task's context off of its own stack and resume it.
static void task__pop(void) __attribute__ ((naked));
static void task__pop(void) {
  asm volatile(
    // Restore stack pointer from current task struct
    "lds r26, _task__current\n" // Low
    "lds r27, _task__current+1\n" // High
    "ld r0, x+\n"
    "out 0x3d, r0\n" // Low
    "ld r0, x+\n"
    "out 0x3e, r0\n" // High

    // Restore general registers
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

    // Restore Z register pair
    "pop r31\n"
    "pop r30\n"

    // Restore status register.
    //
    // In theory, if we set the status register directly, and it enabled
    // interrupts, it is possible for an interrupt to trigger when we're
    // executing the final instructions of 'task__pop'. If this happens, all
    // registers will be pushed onto the stack again, but this time the task
    // hasn't been fully restored yet since we still have the real r0 sitting
    // on the stack. If this happens a number of times successively, we risk a
    // stack overflow. To prevent this from happening, we have two paths:
    //
    // A) Interrupts should NOT be enabled: the status register is restored, we
    // pop the real r0, and execute 'ret' to return to the task.
    //
    // B) Interrupts should be enabled: the status register is restored without
    // the interrupt bit set, we pop the real r0, and execute 'reti' to return
    // to the task AND re-enable interrupts.
    //
    "pop r0\n"
    "sbrs r0, 7\n" // Skip if bit in register set
    "jmp task_pop_ret\n"
    "jmp task_pop_reti\n"

  "task_pop_ret:\n"
    "out 0x3f, r0\n" // Restore status register
    "pop r0\n" // Restore the real r0
    "ret\n"

  "task_pop_reti:\n"
    "clt\n" // Clear T in SREG
    "bld r0, 7\n" // Bit load from T to r0 bit 7 (interrupt bit)
    "out 0x3f, r0\n" // Restore status register (without interrupt bit set)
    "pop r0\n" // Restore the real r0
    "reti\n"

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
    "push r19\n" // r30
    "push r19\n" // r31
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
  static void *start = (void *)RAMEND;
  void *sp;
  task_t *t;

  // Should protect this against underflow.
  start -= 0x100;

  // Stack grows down, don't overwrite first byte of task struct.
  t = start - sizeof(task_t);
  sp = (void *)t - 1;

  t->sp = task__internal_initialize(sp, fn, data);
  t->delay = 0;
  QUEUE_INIT(&t->member);

  return t;
}

// Creates a task for the specified function.
// Adds it to the list of user tasks.
task_t *task_create(task_fn fn, void *data) {
  task_t *t = task__internal_create(fn, data);

  QUEUE_INSERT_TAIL(&_tasks__runnable, &t->member);

  return t;
}

static void task__schedule() {
  QUEUE *q;
  task_t *t;

  QUEUE_FOREACH(q, &_tasks__runnable) {
    t = QUEUE_DATA(q, task_t, member);

    // Make [head..q] the new tail, so that q->next can be scheduled next.
    QUEUE_ROTATE(&_tasks__runnable, q);

    // Task t can be scheduled.
    _task__current = t;

    return;
  }

  // Nothing to schedule, go to sleep.
  _task__current = 0;

  return;
}

static void task__tick() {
  QUEUE *q, *r;
  task_t *t;

#if TASK_COUNT_SEC
  if (--_task_sec_countdown == 0) {
    _task_sec++;
    _task_sec_countdown = 1000 / MS_PER_TICK;
  }
#endif

#if TASK_COUNT_MSEC
  _task_msec += MS_PER_TICK;
#endif

#if TASK_COUNT_USEC
  _task_usec += US_PER_TICK;
#endif

  q = QUEUE_NEXT(&_tasks__sleeping);
  r = 0;
  for (; q != &_tasks__sleeping; q = r) {
    // Save pointer to next element so q can be
    // removed without breaking iteration.
    r = QUEUE_NEXT(q);
    t = QUEUE_DATA(q, task_t, member);

    if (t->delay) {
      t->delay--;
    }

    if (t->delay == 0) {
      task_wakeup(t);
    }
  }
}

static void task__scheduler(void) {
  // Overwrite stack pointer to RAMEND.
  // The task scheduler runs in its own piece of stack to prevent polluting (or
  // even overflowing) task stacks when interrupt handlers are executed.
  asm volatile(
    "out 0x3d, %A0\n"
    "out 0x3e, %B0\n"
    :: "x" (RAMEND)
  );

  for (;;) {
    task__schedule();

    // Run scheduled task, if any.
    if (_task__current != 0x0) {
      // This function doesn't continue execution beyond this point.
      // The task__pop function RETs back into the task.
      task__pop();
    }

    // The processor wakes up from sleep to handle interrupts.
    //
    // 1. It is woken up by the task timer interrupt and the interrupt handler
    // jumps to this function (i.e. this function doesn't continue execution
    // beyond the sleep instruction).
    //
    // 2. It is woken up by another interrupt, the sleep instruction returns
    // after the handler has executed, and this function continues execution.
    //

    sei();
    asm volatile ("sleep");
    cli();
  }
}

static void task__jmp_scheduler(void) {
  asm volatile ("ijmp" :: "z" (task__scheduler));
}

// Must be naked to avoid mangling the stack.
static void task__yield_from_timer(void) __attribute__((naked));
static void task__yield_from_timer(void) {
  task__push();

  task__tick();

  task__jmp_scheduler();
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
static void task__setup_timer() {
  // Waveform generation mode: CTC
  // WGM02: 0
  // WGM01: 1
  // WGM00: 0
  TCCR0A = _BV(WGM01);

  // Clock select (see task.h)
  TCCR0B = _TCCR0B;

  // Output compare register
  OCR0A = COUNTS_PER_TICK - 1;
}

void task_init(void) {
  QUEUE_INIT(&_tasks__runnable);
  QUEUE_INIT(&_tasks__suspended);
  QUEUE_INIT(&_tasks__sleeping);

  task__setup_timer();

#if TASK_COUNT_SEC
  task_set_sec(0);
#endif

#if TASK_COUNT_MSEC
  task_set_msec(0);
#endif

#if TASK_COUNT_USEC
  task_set_usec(0);
#endif
}

// Call this function to start task execution.
// Never returns.
void task_start(void) {
  // Global interrupt bit will be enabled when task is popped.
  cli();

  // Enable interrupt on OCR0A match
  TIMSK0 |= _BV(OCIE0A);

  // Schedule next task to run.
  task__jmp_scheduler();
}

// Yield execution to any other schedulable task.
void task_yield(void) __attribute__((naked));
void task_yield(void) {
  task__push();

  task__jmp_scheduler();
}

// Return pointer to current task.
task_t *task_current(void) {
  return _task__current;
}

void task__suspend(QUEUE *h) {
  uint8_t sreg = SREG;

  cli();

  QUEUE *q = &_task__current->member;
  QUEUE_REMOVE(q);
  QUEUE_INSERT_TAIL(h, q);

  task_yield();

  SREG = sreg;
}

// Suspend task until it is woken up explicitly.
void task_suspend(QUEUE *h) {
  if (h == 0) {
    h = &_tasks__suspended;
  }

  task__suspend(h);
}

// Wake up task.
void task_wakeup(task_t *t) {
  uint8_t sreg = SREG;

  cli();

  QUEUE *q = &t->member;
  QUEUE_REMOVE(q);
  QUEUE_INSERT_TAIL(&_tasks__runnable, q);

  SREG = sreg;
}

// Make current task sleep for specified number of ticks.
void task_sleep(uint16_t ms) {
  _task__current->delay = ms / MS_PER_TICK;
  task__suspend(&_tasks__sleeping);
}
