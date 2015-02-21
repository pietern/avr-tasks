#include <avr/interrupt.h>

#include "cond.h"

void cond_init(cond_t *c) {
  QUEUE_INIT(&c->waiting);
}

// Assume the mutex is held by the caller.
// It is too expensive to do run-time integrity checks on this processor.
void cond_wait(cond_t *c, mutex_t *m) {
  uint8_t sreg;

  sreg = SREG;
  cli();

  // Unlocking and suspending must happen atomically.
  // If it doesn't, a race could cause a cond_{signal,broadcast} from another
  // task holding the lock before this task has been suspended.
  mutex_unlock(m);

  // Suspend task until woken up through cond_{signal,broadcast}.
  task_suspend(&c->waiting);

  // Task may be interrupted again.
  SREG = sreg;

  // Reacquire mutex.
  mutex_lock(m);
}

void cond_signal(cond_t *c) {
  uint8_t sreg;
  QUEUE *q;
  task_t *t;

  sreg = SREG;
  cli();

  // Wake up first waiting task (FIFO order).
  if (!QUEUE_EMPTY(&c->waiting)) {
    q = QUEUE_HEAD(&c->waiting);
    t = QUEUE_DATA(q, task_t, member);
    QUEUE_REMOVE(q);
    task_wakeup(t);
  }

  SREG = sreg;
}

void cond_broadcast(cond_t *c) {
  uint8_t sreg;
  QUEUE *q;
  task_t *t;

  sreg = SREG;
  cli();

  // Wake up all waiting tasks.
  while (!QUEUE_EMPTY(&c->waiting)) {
    q = QUEUE_HEAD(&c->waiting);
    t = QUEUE_DATA(q, task_t, member);
    QUEUE_REMOVE(q);
    task_wakeup(t);
  }

  SREG = sreg;
}
