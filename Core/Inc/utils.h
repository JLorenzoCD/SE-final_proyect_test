#ifndef __UTILS_H
#define __UTILS_H

#define BEAM_MIN_CM 2.0f
#define BEAM_MAX_CM 40.0f
#define EMA_ALPHA 0.45f /* entre 0.35 y 0.60 */

float filter_distance_cm(float raw_cm);

void filter_reset();

#endif // __UTILS_H
