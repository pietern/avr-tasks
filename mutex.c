#include <avr/interrupt.h>

#include "mutex.h"

void mutex_init(mutex_t *m) {
  m->status = MUTEX_UNLOCKED;
  QUEUE_INIT(&m->waiting);
}

void mutex_lock(mutex_t *m) {
  uint8_t sreg;

  sreg = SREG;
  cli();

  if (m->status == MUTEX_LOCKED) {
    // Lock is transferred to this task when it is woken up.
    // m->status will still be set to MUTEX_LOCKED, to avoid any other tasks
    // being scheduled before this one and grabbing the lock.
    task_suspend(&m->waiting);
  } else {
    m->status = MUTEX_LOCKED;
  }

  SREG = sreg;
}

void mutex_unlock(mutex_t *m) {
  uint8_t sreg;
  QUEUE *q;
  task_t *t;

  sreg = SREG;
  cli();

  if (QUEUE_EMPTY(&m->waiting)) {
    m->status = MUTEX_UNLOCKED;
  } else {
    // Wake up first waiting task to transfer lock
    q = QUEUE_HEAD(&m->waiting);
    t = QUEUE_DATA(q, task_t, member);
    QUEUE_REMOVE(q);
    task_wakeup(t);
  }

  SREG = sreg;
}
