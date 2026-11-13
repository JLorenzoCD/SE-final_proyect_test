#include "pid.h"

void PID_Init(PID_Controller *pid,
		float kp, float ki, float kd,
		float dt,
		float out_min, float out_max,
		float integral_min, float integral_max
		) {
	// Default
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;

	// Constantes de proporcionalidad
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	// Tiempo de muestreo
	pid->dt = dt;

	// Limites
	pid->out_min = out_min;
	pid->out_max = out_max;
	pid->integral_min = integral_min;
	pid->integral_max = integral_max;
}

void PID_SetTarget(PID_Controller *pid, float target) {
	pid->target = target;
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
