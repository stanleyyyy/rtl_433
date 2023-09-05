#include "wav_dumper.h"
#include <stdlib.h>

typedef struct {
    char chunk_id[4];
    uint32_t chunk_size;
    char format[4];
    char subchunk1_id[4];
    uint32_t subchunk1_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char subchunk2_id[4];
    uint32_t subchunk2_size;
} WavHeader;

wav_dumper_t* wav_dumper_create(const char *filename, uint32_t sample_rate, size_t buffer_size)
{
    wav_dumper_t *writer = (wav_dumper_t *)malloc(sizeof(wav_dumper_t));
    if (writer == NULL) {
        return NULL;
    }

    writer->file = fopen(filename, "wb");
    if (writer->file == NULL) {
        free(writer);
        return NULL;
    }

    writer->buffer_size = buffer_size;
    writer->buffer = (int16_t *)malloc(buffer_size * sizeof(int16_t));
    if (writer->buffer == NULL) {
        fclose(writer->file);
        free(writer);
        return NULL;
    }

    writer->samples_written = 0;

    WavHeader header = {
        .chunk_id = "RIFF",
        .chunk_size = 0x0FFFFFFF, // Set a large initial chunk size
        .format = "WAVE",
        .subchunk1_id = "fmt ",
        .subchunk1_size = 16,
        .audio_format = 1,
        .num_channels = 1,
        .sample_rate = sample_rate,
        .byte_rate = sample_rate * sizeof(int16_t),
        .block_align = sizeof(int16_t),
        .bits_per_sample = 16,
        .subchunk2_id = "data",
        .subchunk2_size = 0x0FFFFFFF // Set a large initial chunk size
    };

    fwrite(&header, sizeof(WavHeader), 1, writer->file);

    return writer;
}

static void flush_buffer(wav_dumper_t *writer)
{
    if (writer == NULL) {
        return;
    }

    size_t remaining_samples = writer->samples_written % writer->buffer_size;
    if (remaining_samples > 0) {
        fwrite(writer->buffer, sizeof(int16_t), remaining_samples, writer->file);
    }

    writer->samples_written += remaining_samples;
}

void wav_dumper_destroy(wav_dumper_t **writer)
{
    if (!writer || !*writer) {
        return;
    }

    flush_buffer(*writer);

    // Update the WAV header with the actual sizes
    fseek((*writer)->file, 4, SEEK_SET);
    uint32_t data_size = (*writer)->samples_written * sizeof(int16_t);
    uint32_t chunk_size = 36 + data_size;
    fwrite(&chunk_size, sizeof(uint32_t), 1, (*writer)->file);
    fseek((*writer)->file, 40, SEEK_SET);
    fwrite(&data_size, sizeof(uint32_t), 1, (*writer)->file);

    fclose((*writer)->file);
    free((*writer)->buffer);
    free((*writer));
	*writer = NULL;
}

void wav_dumper_write_sample(wav_dumper_t *writer, int16_t sample)
{
    if (writer == NULL) {
        return;
    }

    writer->buffer[writer->samples_written % writer->buffer_size] = sample;

    if ((writer->samples_written + 1) % writer->buffer_size == 0) {
        fwrite(writer->buffer, sizeof(int16_t), writer->buffer_size, writer->file);
    }

    writer->samples_written++;
}
