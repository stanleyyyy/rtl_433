#pragma once

#include <stdint.h>

typedef struct {
    double attack_rate;
    double release_rate;
    double current_peak;
	double min_val;
} peak_follower_t;

peak_follower_t *peak_follower_create(double attack_rate, double release_rate, int mindB);
void peak_follower_destroy(peak_follower_t **peak_follower);
int16_t peak_follower_process(peak_follower_t *follower, int16_t sample);
