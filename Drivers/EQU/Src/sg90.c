#include <sg90.h>

static void SG90_Update_PWM(SG90_HandleTypeDef *hsg90) {
	uint32_t pulse_length = hsg90->_pwm_min_pulse + (hsg90->current_angle * (hsg90->_pwm_max_pulse - hsg90->_pwm_min_pulse)/hsg90->_max_angle);
	__HAL_TIM_SET_COMPARE(hsg90->htim, hsg90->channel, pulse_length);
}

void SG90_Init(
		SG90_HandleTypeDef *hsg90,
		uint16_t max_angle,
		uint32_t pwm_min_pulse,
		uint32_t pwm_max_pulse
		) {
	HAL_TIM_PWM_Start(hsg90->htim, hsg90->channel);

	hsg90->current_angle = 0;
	hsg90->_target_angle = 0;
	hsg90->is_moving = false;
	hsg90->speed_delay_ms = 10;

	hsg90->_last_tick = 0;
	hsg90->_max_angle = max_angle;
	hsg90->_pwm_min_pulse = pwm_min_pulse;
	hsg90->_pwm_max_pulse = pwm_max_pulse;

	// Por defecto se posiciona a 0° al iniciar
	SG90_Update_PWM(hsg90);
}

void SG90_Set_Angle_Target(SG90_HandleTypeDef *hsg90, uint16_t angle, uint16_t speed_ms) {
    if (angle > hsg90->_max_angle) angle = hsg90->_max_angle;

    hsg90->_target_angle = angle;
    hsg90->speed_delay_ms = speed_ms;

    if (hsg90->current_angle != hsg90->_target_angle) {
    	hsg90->is_moving = true;
    	hsg90->_last_tick = HAL_GetTick();
    }
}

void SG90_Update_Angle(SG90_HandleTypeDef *hsg90) {
    if (!hsg90->is_moving) {
        return;
    }

    uint32_t current_tick = HAL_GetTick();

    // Comprobar si ya es momento de dar el siguiente paso según el deley configurado
    if ((current_tick - hsg90->_last_tick) >= hsg90->speed_delay_ms) {
    	hsg90->_last_tick = current_tick;

        // Ajustar un grado hacia el objetivo
        if (hsg90->current_angle < hsg90->_target_angle) {
        	hsg90->current_angle++;
        } else if (hsg90->current_angle > hsg90->_target_angle) {
        	hsg90->current_angle--;
        }

        SG90_Update_PWM(hsg90);

        if (hsg90->current_angle == hsg90->_target_angle) {
        	hsg90->is_moving = false;
        }
    }
}

