#ifndef UTILS_ARRAY_H
#define UTILS_ARRAY_H

#include "pmsis.h"
#include <math.h>

void delete_duplicates_and_sort(int16_t *array, int16_t size,
                                int16_t *new_size);

float euclidean_dist(float *a, float *b);

#endif
