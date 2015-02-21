#ifndef _COND_H
#define _COND_H

#include "mutex.h"

typedef struct cond_s cond_t;

struct cond_s {
  QUEUE waiting;
};

void cond_init(cond_t *c);

void cond_wait(cond_t *c, mutex_t *m);

void cond_signal(cond_t *c);

void cond_broadcast(cond_t *c);

#endif
