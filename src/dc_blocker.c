#include "dc_blocker.h"
#include <stdio.h>
#include <assert.h>

dc_blocker_t *dc_blocker_create(int buffer_length)
{
    dc_blocker_t* new_blocker = (dc_blocker_t*) malloc(sizeof(dc_blocker_t));
    new_blocker->buffer = (int16_t*) calloc(buffer_length, sizeof(int16_t));
    new_blocker->sum = 0;
    new_blocker->buffer_length = buffer_length;
    new_blocker->index = 0;
    return new_blocker;
}

void dc_blocker_destroy(dc_blocker_t **blocker)
{
	if (blocker && *blocker) {
		free((*blocker)->buffer);
		free((*blocker));
		*blocker = NULL;
	}
}

int16_t dc_blocker_filter(dc_blocker_t* self, int16_t sample)
{
    self->sum -= self->buffer[self->index];
    self->buffer[self->index] = sample;
    self->sum += sample;
    self->index = (self->index + 1) % self->buffer_length;
    int16_t mean_value = (int16_t)(self->sum / self->buffer_length);
    return sample - mean_value;
}