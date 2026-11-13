#include <stdlib.h>
#include <stdint.h>

#include "bsp.h"
#include "pid.h"

Motor APP_motor = NULL;
Distancia APP_distancia = NULL;
PID_Controller APP_pid;

int main(void) {
	BSP_Init();

	PID_Init(&APP_pid,
			1.2f, 0.05f, 0.4f,
			0.02f,
			0.0, 180.0f,
			-30.0f, 30.0f
	);
	// Distancia hasta el centro
	PID_SetTarget(&APP_pid, 20.0f);

	while (1) {
		BSP_MOTOR_Update();

		if (BSP_DISTANCIA_CanMeasure()) {
			BSP_DISTANCIA_StartMeasurement();

		} else if (BSP_DISTANCIA_IsDataRedy()) {
			float distancia_cm = BSP_DISTANCIA_GetDistance();
			float salida_motor = PID_Compute(&APP_pid, distancia_cm);
			BSP_MOTOR_SetAngle((uint16_t)salida_motor);

		} else if (BSP_DISTANCIA_IsTimeout()) {
			BSP_DISTANCIA_Reset();
		}
	}
}

