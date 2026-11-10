#ifndef __SG90_F4_H
#define __SG90_F4_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef struct {
	// Public
	TIM_HandleTypeDef   *htim;
    uint32_t            channel;
    uint16_t            current_angle;
	bool             	is_moving;      // Flag de estado
	uint16_t            speed_delay_ms; // Milisegundos entre cada grado

	// Private
    uint16_t            _target_angle;
    uint16_t			_max_angle;
    uint32_t			_pwm_min_pulse;
    uint32_t			_pwm_max_pulse;
	uint32_t            _last_tick;
} SG90_HandleTypeDef;

void SG90_Init(SG90_HandleTypeDef *hsg90, uint16_t max_angle, uint32_t pwm_min_pulse, uint32_t pwm_max_pulse);

void SG90_Set_Angle_Target(SG90_HandleTypeDef *hsg90, uint16_t angle, uint16_t speed_ms);

void SG90_Update_Angle(SG90_HandleTypeDef *hsg90);

#endif /* __SG90_F4_H */
