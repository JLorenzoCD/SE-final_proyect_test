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

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float dt,
		float out_min, float out_max, float int_min, float int_max);
float PID_Compute(PID_Controller *pid, float current_position);

void APP_Init() {
	if (BSP_Init(&APP_distancia, &APP_motor) != BSP_OK) {
		// Error
	}

	PID_Init(&APP_pid, 1.2f, 0.05f, 0.4f, 0.02f, 0.0f, 180.0f, -30.0f, 30.0f);
	APP_pid.target = 20.0f; // Supongamos que esta es la distancia hasta el centro
}

void APP_Run() {
	bool distancia_can_measure = false;
	bool distancia_is_data_ready = false;
	bool distancia_is_timeout = false;

	BSP_MOTOR_Update(APP_motor);

	BSP_DISTANCIA_CanMeasure(APP_distancia, &distancia_can_measure);
	BSP_DISTANCIA_IsDataRedy(APP_distancia, &distancia_is_data_ready);
	BSP_DISTANCIA_IsTimeout(APP_distancia, &distancia_is_timeout);

	if (distancia_can_measure) {
		BSP_DISTANCIA_StartMeasurement(APP_distancia);
	} else if (distancia_is_data_ready) {
		float distancia_cm = 0.0;
		BSP_DISTANCIA_GetDistance(APP_distancia, &distancia_cm);

		float salida_servo = PID_Compute(&APP_pid, distancia_cm);
		BSP_MOTOR_SetAngle(APP_motor, (uint16_t)salida_servo, 10);
	} else if (distancia_is_timeout) {
		BSP_DISTANCIA_Reset(APP_distancia);
	}
}


void PID_Init(
		PID_Controller *pid,
		float kp,
		float ki,
		float kd,
		float dt,
		float out_min,
		float out_max,
		float int_min,
		float int_max
		) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_min = int_min;
    pid->integral_max = int_max;
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

