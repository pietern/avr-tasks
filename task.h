#ifndef _TASK_H
#define _TASK_H

#include <stdint.h>

#include "queue.h"

#define MS_PER_TICK 2

typedef void (*task_fn)(void *);

typedef struct task_s task_t;

struct task_s {
  void *sp; // Stack pointer this task can be resumed from.
  uint16_t delay; // Ticks until task can be scheduled again.

  QUEUE member;
};

// Initialize internal structures, tick timer, etc.
void task_initialize(void);

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

#endif
