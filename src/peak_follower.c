#include "peak_follower.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>

// scale value in dB to linear gain as value between <0; 32767> (16 bit equivalent gain value)
static int16_t db_to_linear(double db)
{
    // Calculate the linear amplitude from dB value
    double linear_amplitude = pow(10.0, db / 20.0);

    // Convert linear amplitude to 16-bit signed integer range
    int16_t value = (int16_t)(linear_amplitude * 32767.0);
    return value;
}

peak_follower_t *peak_follower_create(double attack_rate, double release_rate, int mindB)
{
    peak_follower_t *follower = (peak_follower_t *)malloc(sizeof(peak_follower_t));
    assert(follower);

	if (mindB > 0) {
		mindB = 0;
	}

    follower->attack_rate = attack_rate;
    follower->release_rate = release_rate;
    follower->current_peak = 0.0;
	follower->min_val = db_to_linear(mindB);
	return follower;
}

void peak_follower_destroy(peak_follower_t **peak_follower)
{
	if (peak_follower && *peak_follower) {
		free(*peak_follower);
		*peak_follower = NULL;
	}
}

int16_t peak_follower_process(peak_follower_t *follower, int16_t sample)
{
    assert(follower);
	double fsample = sample;

    if (fabs(sample) > follower->current_peak) {
        follower->current_peak = follower->attack_rate * follower->current_peak + (1 - follower->attack_rate) * fabs(sample);
    } else {
        follower->current_peak = follower->release_rate * follower->current_peak;
    }

    // ignore anything below follower->min_val
    if (follower->current_peak < follower->min_val) {
		return 0;
    }

    return follower->current_peak;
}
