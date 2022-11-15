#ifndef RBCLOCK_H
#define RBCLOCK_H

#include <time.h>

void timespec_add_us(struct timespec *t, long us);
int timespec_cmp(struct timespec *a, struct timespec *b);
int timediff_us(struct timespec *before, struct timespec *after);

#endif // RBCLOCK_H
