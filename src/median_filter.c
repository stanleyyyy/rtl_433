#include "median_filter.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static int compare_int16(const void *a, const void *b)
{
    return (*(int16_t *)a - *(int16_t *)b);
}

median_filter_t * median_filter_create(int window_size)
{
    median_filter_t *filter = (median_filter_t *)malloc(sizeof(median_filter_t));
    assert(filter != NULL);

    filter->window_size = window_size;
    filter->values = (int16_t *)malloc(window_size * sizeof(int16_t));
    assert(filter->values != NULL);

    filter->temp_values = (int16_t *)malloc(window_size * sizeof(int16_t));
    assert(filter->temp_values != NULL);

    // Initialize values array with zeros
    for (int i = 0; i < window_size; ++i) {
        filter->values[i] = 0;
    }

	return filter;
}

int16_t median_filter_process(median_filter_t *filter, int16_t sample)
{
    assert(filter != NULL);

    // Shift values in the buffer
    for (int i = filter->window_size - 1; i > 0; --i) {
        filter->values[i] = filter->values[i - 1];
    }

    filter->values[0] = sample;

    // Copy values to the temporary array
    for (int i = 0; i < filter->window_size; ++i) {
        filter->temp_values[i] = filter->values[i];
    }

    // Sort the temporary array
    qsort(filter->temp_values, filter->window_size, sizeof(int16_t), compare_int16);

    // Calculate and return the median
    int middle_index = filter->window_size / 2;
    return filter->temp_values[middle_index];
}

void median_filter_destroy(median_filter_t **filter)
{
    if (*filter != NULL) {
        free((*filter)->values);
        free((*filter)->temp_values);
        free(*filter);
        *filter = NULL;
    }
}