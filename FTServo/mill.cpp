/*
 * Copyright 2023 Ethan. All rights reserved.
 */
#include "mill.h"
#include <time.h>

unsigned long millis() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}
