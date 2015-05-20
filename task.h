#ifndef _TASK_H
#define _TASK_H

#include <stdint.h>

#include "queue.h"

#ifndef F_CPU
#error "Define F_CPU"
#endif

#define MS_PER_TICK 2
#define US_PER_TICK (1000 * MS_PER_TICK)
#define US_PER_COUNT (US_PER_TICK / COUNTS_PER_TICK)

#if F_CPU == 16000000L
// Clock select: prescaler = 1/256
#define _TCCR0B (_BV(CS02))
#define COUNTS_PER_TICK ((F_CPU / 256) / (1000 / MS_PER_TICK))
#elif F_CPU == 8000000L
// Clock select: prescaler = 1/64
#define _TCCR0B (_BV(CS01) | _BV(CS00))
#define COUNTS_PER_TICK ((F_CPU / 64) / (1000 / MS_PER_TICK))
#else
#error "Unsupported F_CPU"
#endif

typedef void (*task_fn)(void *);

typedef struct task_s task_t;

struct task_s {
  void *sp; // Stack pointer this task can be resumed from.
  uint16_t delay; // Ticks until task can be scheduled again.

  QUEUE member;
};

// Initialize internal structures, tick timer, etc.
void task_init(void);

// Creates a task for the specified function.
task_t *task_create(task_fn fn, void *data);

// Starts task execution. Never returns.
void task_start(void);

// Yield control from current task.
void task_yield(void);

// Return pointer to current task.
task_t *task_current(void);

// Suspend task until it is woken up explicitly.
// The task is added to the tail of the queue pointed to by q. If q is NULL,
// it is added to the system wide queue for suspended tasks.
void task_suspend(QUEUE *h);

// Wake up task.
void task_wakeup(task_t *t);

// Sleep current task for specified number of milliseconds.
void task_sleep(uint16_t ms);

// Only count seconds if specified
#if TASK_COUNT_SEC
#ifndef TASK_SEC_T
#define TASK_SEC_T uint16_t
#endif
TASK_SEC_T task_sec(void);
void task_set_sec(TASK_SEC_T);
#endif

// Only count milliseconds if specified
#if TASK_COUNT_MSEC
#ifndef TASK_MSEC_T
#define TASK_MSEC_T uint16_t
#endif
TASK_MSEC_T task_msec(void);
void task_set_msec(TASK_MSEC_T);
#endif

// Only count microseconds if specified
#if TASK_COUNT_USEC
#ifndef TASK_USEC_T
#define TASK_USEC_T uint16_t
#endif
TASK_USEC_T task_usec(void);
void task_set_usec(TASK_USEC_T);
#endif

#endif
