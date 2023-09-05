#pragma once

#include <stdint.h>
#include <stdio.h>

typedef struct {
    FILE *file;
    size_t buffer_size;
    int16_t *buffer;
    size_t samples_written;
} wav_dumper_t;

wav_dumper_t *wav_dumper_create(const char *filename, uint32_t sample_rate, size_t buffer_size);
void wav_dumper_destroy(wav_dumper_t **writer);
void wav_dumper_write_sample(wav_dumper_t *writer, int16_t sample);