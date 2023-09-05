#pragma once

#include <stdint.h>

typedef struct {
    int16_t* buffer;
    int32_t sum;
    int buffer_length;
    int index;
} dc_blocker_t;

dc_blocker_t *dc_blocker_create(int buffer_length);
void dc_blocker_destroy(dc_blocker_t **blocker);
int16_t dc_blocker_filter(dc_blocker_t* self, int16_t sample);
