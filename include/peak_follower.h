#pragma once
#include <stdint.h>

typedef struct {
    double attack_rate;
    double release_rate;
    double current_high_peak;
    double current_low_peak;
    double min_val;
} peak_follower_t;

peak_follower_t *peak_follower_create(double attack_rate, double release_rate, int mindB);
void peak_follower_destroy(peak_follower_t **peak_follower);
void peak_follower_process(peak_follower_t *follower, int16_t sample, int16_t *high_peak, int16_t *low_peak);