#include "bsp.h"

#include "stm32f4xx_hal.h"
#include "tim.h"

#include "sg90.h"
#include "HC_SR04.h"

// Sensores y actuadores de la APP
extern Motor APP_motor;
extern Distancia APP_distancia;

#define HCSR04_MEASUREMENT_INTERVAL_MS 20
#define HCSR04_TIMEOUT_ECO_MS 40

// Perifericos
HCSR04_HandleTypeDef hcsr04_medidor_dist;
SG90_HandleTypeDef sg90_servo_motor;

// Variables de estado
volatile uint32_t hcsr04_last_tick_read = 0;
volatile bool hcsr04_is_measurement = false;

// Funciones externas
BSP_State BSP_Init() {
	if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK) {
		return BSP_ERROR;
	}

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

	APP_distancia = (HCSR04_HandleTypeDef*)&hcsr04_medidor_dist;
	APP_motor = (SG90_HandleTypeDef*)&sg90_servo_motor;

	return BSP_OK;
}

// Funciones utilizadas para el servomotor SG90
void BSP_MOTOR_Update() {
	SG90_Update_Angle((SG90_HandleTypeDef*)APP_motor);
}

bool BSP_MOTOR_IsMoving() {
	bool is_moving = ((SG90_HandleTypeDef*)APP_motor)->is_moving;
	return is_moving;
}

void BSP_MOTOR_SetAngle(uint16_t angle, uint16_t speed_ms) {
	SG90_Set_Angle_Target((SG90_HandleTypeDef*)APP_motor, angle, speed_ms);
}

// Funciones utilizadas para el medidor de distancia ultrasonico HC-SR04
bool BSP_DISTANCIA_CanMeasure() {
	bool can_measure = !hcsr04_is_measurement &&
			(HAL_GetTick() - hcsr04_last_tick_read > HCSR04_MEASUREMENT_INTERVAL_MS);

	return can_measure;
}

void BSP_DISTANCIA_StartMeasurement() {
	hcsr04_is_measurement = 1;
	hcsr04_start_measurement((HCSR04_HandleTypeDef*)APP_distancia);
}

bool BSP_DISTANCIA_IsDataRedy() {
	bool is_data_ready = hcsr04_is_data_ready((HCSR04_HandleTypeDef*)APP_distancia);

	return is_data_ready;
}

float BSP_DISTANCIA_GetDistance() {
	hcsr04_is_measurement = 0;
	hcsr04_last_tick_read = HAL_GetTick();

	float distancia_cm = hcsr04_get_distance_cm((HCSR04_HandleTypeDef*)APP_distancia);

	return distancia_cm;
}

bool BSP_DISTANCIA_IsTimeout() {
	bool is_timeout = hcsr04_is_measurement
			&& (HAL_GetTick() - hcsr04_last_tick_read > HCSR04_TIMEOUT_ECO_MS);

	return is_timeout;
}

BSP_State BSP_DISTANCIA_Reset() {
	hcsr04_is_measurement = 0;
	hcsr04_last_tick_read = HAL_GetTick();

	// Limpiar channel del timer
	TIM_HandleTypeDef* 	htim =((HCSR04_HandleTypeDef*)APP_distancia)->htim;
	uint32_t            chanel = ((HCSR04_HandleTypeDef*)APP_distancia)->IC_Channel;
	if (HAL_TIM_IC_Stop_IT(htim, chanel) != HAL_OK) {
		return BSP_ERROR;
	}

	return BSP_OK;
}

// Callback
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (APP_distancia == NULL) return;

	hcsr04_tim_ic_callback((HCSR04_HandleTypeDef*)APP_distancia, htim);
}

