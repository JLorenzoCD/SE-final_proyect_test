#include "utils.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float buf[3];
    uint8_t count;
    uint8_t idx;
    float ema;
    bool ema_initialized;
    float last_valid_raw;
    bool has_last_valid;
} DistanceFilter_t;

static DistanceFilter_t dist_filter = {0};

static float median_of_three(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

float filter_distance_cm(float raw_cm) {
    // Descartar lecturas fuera de rango físico de la viga
    if (raw_cm < BEAM_MIN_CM || raw_cm > BEAM_MAX_CM) {
        if (dist_filter.ema_initialized) {
            return dist_filter.ema;
        }
        return (raw_cm < BEAM_MIN_CM) ? BEAM_MIN_CM : BEAM_MAX_CM;
    }

    // Mediana móvil de 3 muestras
    dist_filter.buf[dist_filter.idx] = raw_cm;
    dist_filter.idx = (dist_filter.idx + 1U) % 3U;
    if (dist_filter.count < 3U) {
        dist_filter.count++;
    }

    float med;
    if (dist_filter.count == 1U) {
        med = dist_filter.buf[0];
    } else if (dist_filter.count == 2U) {
        med = 0.5f * (dist_filter.buf[0] + dist_filter.buf[1]);
    } else {
        med = median_of_three(dist_filter.buf[0], dist_filter.buf[1], dist_filter.buf[2]);
    }

    /* Filtro exponencial (EMA) */
    if (!dist_filter.ema_initialized) {
        dist_filter.ema = med;
        dist_filter.ema_initialized = true;
    } else {
        dist_filter.ema = (EMA_ALPHA * med) + ((1.0f - EMA_ALPHA) * dist_filter.ema);
    }

    dist_filter.last_valid_raw = raw_cm;
    dist_filter.has_last_valid = true;

    return dist_filter.ema;
}

void filter_reset() {
	dist_filter.ema_initialized = false;
	dist_filter.count = 0;
}
