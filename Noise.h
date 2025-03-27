#ifndef _NOISE_H
#define _NOISE_H

#include <stddef.h>

typedef float real;
typedef double lreal;

void Noise_MeasureStart(size_t n, lreal (*noise)[n][n], lreal (*progress_noise)[n][n]);

void Noise_MeasureUpdate(size_t i, size_t n, const lreal (*sample)[n],
                         lreal prev_sample[2][n][1], lreal (*noise)[n][n], lreal (*progress_noise)[n][n]);

void Noise_MeasureFinish(size_t m, size_t n, lreal (*noise)[n][n], lreal (*progress_noise)[n][n], lreal dt);

#endif