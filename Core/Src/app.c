#include "app.h"

typedef struct {
    float kp;
    float ki;
    float kd;

    float target;       // Posición deseada (ej. centro)
    float integral;
    float prev_error;
    float dt;           // Tiempo de muestreo en segundos (ej. 0.02s = 20ms)

    // Límites del Actuador (Servo en grados)
	float out_min;
	float out_max;

	// Límites específicos para el Anti-Windup de la integral
	float integral_min;
	float integral_max;
} PID_Controller;

// Variables utilizadas en la APP
Motor APP_motor = NULL;
Distancia APP_distancia = NULL;
PID_Controller APP_pid;

float PID_Compute(PID_Controller *pid, float current_position);

void APP_Init() {
	if (BSP_Init(&APP_distancia, &APP_motor) != BSP_OK) {
		// Error
	}

	// Distancia hasta el centro
	APP_pid.target = 20.0f; // Supongamos que esta a 20cm

	// Default
	APP_pid.integral = 0.0f;
	APP_pid.prev_error = 0.0f;

	// Tengo que probar
	APP_pid.kp = 1.2f;
	APP_pid.ki = 0.05f;
	APP_pid.kd = 0.4f;

	APP_pid.dt = 0.02f;

	// Limites
	APP_pid.out_min = 0.0f;
	APP_pid.out_max = 180.0f;
	APP_pid.integral_min = -30.0f;
	APP_pid.integral_max = 30.0f;
}

void APP_Run() {
	BSP_MOTOR_Update(APP_motor);

	if (BSP_DISTANCIA_CanMeasure(APP_distancia)) {
		BSP_DISTANCIA_StartMeasurement(APP_distancia);

	} else if (BSP_DISTANCIA_IsDataRedy(APP_distancia)) {
		float distancia_cm = BSP_DISTANCIA_GetDistance(APP_distancia);
		float salida_motor = PID_Compute(&APP_pid, distancia_cm);
		BSP_MOTOR_SetAngle(APP_motor, (uint16_t)salida_motor, 10);

	} else if (BSP_DISTANCIA_IsTimeout(APP_distancia)) {
		BSP_DISTANCIA_Reset(APP_distancia);
	}
}


float PID_Compute(PID_Controller *pid, float curr_position) {
	// Error actual
    float error = pid->target - curr_position;

    // Término Proporcional
    float p = pid->kp * error;

    // Término Integral
    pid->integral += error * pid->dt;
    if (pid->integral > pid->integral_max) {
		pid->integral = pid->integral_max;
	} else if (pid->integral < pid->integral_min) {
		pid->integral = pid->integral_min;
	}
	float i = pid->ki * pid->integral;

    // Término Derivativo
    float derivative = (error - pid->prev_error) / pid->dt;
    float d = pid->kd * derivative;

    pid->prev_error = error;

    // Salida del PID
    float output = p + i + d;
    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    return output;
}

