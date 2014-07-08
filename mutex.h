#ifndef _MUTEX_H
#define _MUTEX_H

/*
 * When a task calls 'mutex_lock' and the mutex is not yet locked, it will
 * become locked and the calling task continues execution. This task
 * subsequently has to call 'mutex_unlock' to give up the lock and let other
 * tasks access the protected resource.
 *
 * When a task calls 'mutex_lock' and the mutex is locked, the task will be
 * suspended and pushed onto the list of waiting tasks. It is then suspended
 * until it both 1) becomes the first element of the list, and 2) another task
 * holding the mutex unlocks it. Then, it is woken up and the mutex is set to
 * be locked by this task. The latter is necessary to prevent the task that
 * unlocked the mutex from immediately locking it again and thereby starving
 * other tasks waiting for access to the same protected resource.
 */

#include "task.h"

#define MUTEX_UNLOCKED 0
#define MUTEX_LOCKED 1

typedef struct mutex_s mutex_t;

struct mutex_s {
  unsigned status:1;
  QUEUE waiting;
};

void mutex_init(mutex_t *mutex);

void mutex_lock(mutex_t *mutex);

void mutex_unlock(mutex_t *mutex);

#endif
