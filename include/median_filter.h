#pragma once

#include <stdint.h>

typedef struct {
    int window_size;
    int16_t *values;
    int16_t *temp_values; // Temporary array for sorting
} median_filter_t;

median_filter_t * median_filter_create(int window_size);
int16_t median_filter_process(median_filter_t *filter, int16_t sample);
void median_filter_destroy(median_filter_t **filter);
