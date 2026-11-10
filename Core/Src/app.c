#include "app.h"

void* APP_motor = NULL;
void* APP_distancia = NULL;

uint8_t curr_angle = 0;
uint16_t angles[5] = {180, 45, 135, 0, 90};
uint16_t delays[5] = {100, 10, 10, 50, 5};

bool is_moving = false;
bool can_measure = false;
bool is_data_redy = false;
bool is_timeout = false;

float distancia_cm = 0.0;

void APP_Init() {
	if (BSP_Init(&APP_distancia, &APP_motor) != BSP_OK) {
		// Error
	}
}

void APP_Run() {
	BSP_MOTOR_Update(&APP_motor);

	BSP_MOTOR_IsMoving(&APP_motor, &is_moving);
	if (!is_moving) {
		BSP_MOTOR_SetAngle(&APP_motor, angles[curr_angle], delays[curr_angle]);
		curr_angle = (curr_angle + 1) % 5;
	}

	BSP_DISTANCIA_CanMeasure(&APP_distancia, &can_measure);
	BSP_DISTANCIA_IsDataRedy(&APP_distancia, &is_data_redy);
	BSP_DISTANCIA_IsTimeout(&APP_distancia, &is_timeout);

	if (can_measure) {
		BSP_DISTANCIA_StartMeasurement(&APP_distancia);
	} else if (is_data_redy) {
		BSP_DISTANCIA_GetDistance(&APP_distancia, &distancia_cm);
	} else if (is_timeout) {
		BSP_DISTANCIA_Reset(&APP_distancia);
	}
}

void APP_GetSensors(Distancia* medidor_distancia, Motor* motor) {
	*medidor_distancia = APP_distancia;
	*motor = APP_motor;
}
