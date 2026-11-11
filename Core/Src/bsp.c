#include "bsp.h"

#include "stm32f4xx_hal.h"
#include "tim.h"

#include "sg90.h"
#include "HC_SR04.h"

#define HCSR04_MEASUREMENT_INTERVAL_MS 20
#define HCSR04_TIMEOUT_ECO_MS 40

// Perifericos
HCSR04_HandleTypeDef hcsr04_medidor_dist;
SG90_HandleTypeDef sg90_servo_motor;

// Variables de estado
volatile uint32_t hcsr04_last_tick_read = 0;
volatile bool hcsr04_is_measurement = false;

// Funciones externas
BSP_State BSP_Init(Distancia* medidor_distancia, Motor* motor) {
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

	// Se inicializa el medidor de distancia
	hcsr04_medidor_dist.htim = &htim2;
	hcsr04_medidor_dist.IC_Channel = TIM_CHANNEL_2;
	hcsr04_medidor_dist.Trig_Pin = Trigger_Pin;
	hcsr04_medidor_dist.Trig_Port = Trigger_GPIO_Port;
	hcsr04_init(&hcsr04_medidor_dist);

	// Se inicializa el servo motor
	sg90_servo_motor.htim = &htim2;
	sg90_servo_motor.channel = TIM_CHANNEL_1;
	SG90_Init(&sg90_servo_motor, 180, 500, 2500);

	*medidor_distancia = (HCSR04_HandleTypeDef*)&hcsr04_medidor_dist;
	*motor = (SG90_HandleTypeDef*)&sg90_servo_motor;

	return BSP_OK;
}

// Funciones utilizadas para el servomotor SG90
BSP_State BSP_MOTOR_Update(Motor motor) {
	if (motor == NULL) return BSP_ERROR_ARG;

	SG90_Update_Angle((SG90_HandleTypeDef*)motor);
	return BSP_OK;
}

BSP_State BSP_MOTOR_IsMoving(Motor motor, bool* is_moving) {
	if (motor == NULL) return BSP_ERROR_ARG;

	*is_moving = ((SG90_HandleTypeDef*)motor)->is_moving;
	return BSP_OK;
}

BSP_State BSP_MOTOR_SetAngle(Motor motor, uint16_t angle, uint16_t speed_ms) {
	if (motor == NULL) return BSP_ERROR_ARG;

	SG90_Set_Angle_Target((SG90_HandleTypeDef*)motor, angle, speed_ms);
	return BSP_OK;
}

// Funciones utilizadas para el medidor de distancia ultrasonico HC-SR04
BSP_State BSP_DISTANCIA_CanMeasure(Distancia medidor_distancia, bool* can_measure) {
	if (medidor_distancia == NULL) return BSP_ERROR_ARG;

	*can_measure = !hcsr04_is_measurement &&
			(HAL_GetTick() - hcsr04_last_tick_read > HCSR04_MEASUREMENT_INTERVAL_MS);

	return BSP_OK;
}

BSP_State BSP_DISTANCIA_StartMeasurement(Distancia medidor_distancia) {
	if (medidor_distancia == NULL) return BSP_ERROR_ARG;

	hcsr04_is_measurement = 1;
	hcsr04_start_measurement((HCSR04_HandleTypeDef*)medidor_distancia);

	return BSP_OK;
}

BSP_State BSP_DISTANCIA_IsDataRedy(Distancia medidor_distancia, bool* is_data_ready) {
	if (medidor_distancia == NULL) return BSP_ERROR_ARG;

	*is_data_ready = hcsr04_is_data_ready((HCSR04_HandleTypeDef*)medidor_distancia);

	return BSP_OK;
}

BSP_State BSP_DISTANCIA_GetDistance(Distancia medidor_distancia, float* distancia_cm) {
	if (medidor_distancia == NULL) return BSP_ERROR_ARG;

	hcsr04_is_measurement = 0;
	hcsr04_last_tick_read = HAL_GetTick();

	*distancia_cm = hcsr04_get_distance_cm((HCSR04_HandleTypeDef*)medidor_distancia);

	return BSP_OK;
}

BSP_State BSP_DISTANCIA_IsTimeout(Distancia medidor_distancia, bool* is_timeout) {
	if (medidor_distancia == NULL) return BSP_ERROR_ARG;

	*is_timeout = hcsr04_is_measurement
			&& (HAL_GetTick() - hcsr04_last_tick_read > HCSR04_TIMEOUT_ECO_MS);

	return BSP_OK;
}

BSP_State BSP_DISTANCIA_Reset(Distancia medidor_distancia) {
	if (medidor_distancia == NULL) return BSP_ERROR_ARG;

	hcsr04_is_measurement = 0;
	hcsr04_last_tick_read = HAL_GetTick();

	// Limpiar channel del timer
	TIM_HandleTypeDef* 	htim =((HCSR04_HandleTypeDef*)medidor_distancia)->htim;
	uint32_t            chanel = ((HCSR04_HandleTypeDef*)medidor_distancia)->IC_Channel;
	HAL_TIM_IC_Stop_IT(htim, chanel);

	return BSP_OK;
}

// Callback
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	hcsr04_tim_ic_callback(&hcsr04_medidor_dist, htim);
}

