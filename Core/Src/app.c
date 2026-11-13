#include "app.h"

#define APP_MOTOR_DELAY_IN_MS_PER_DEGREE 10

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

void PID_Init(PID_Controller *pid,
		float kp, float ki, float kd,
		float dt,
		float out_min, float out_max,
		float integral_min, float integral_max
		);
void PID_SetTarget(PID_Controller *pid, float target);
float PID_Compute(PID_Controller *pid, float current_position);

void APP_Init() {
	if (BSP_Init() != BSP_OK) {
		// Error
	}

	PID_Init(&APP_pid,
			1.2f, 0.05f, 0.4f,
			0.02f,
			0.0, 180.0f,
			-30.0f, 30.0f
	);

	// Distancia hasta el centro
	PID_SetTarget(&APP_pid, 20.0f);
}

void APP_Run() {
	BSP_MOTOR_Update();

	if (BSP_DISTANCIA_CanMeasure()) {
		BSP_DISTANCIA_StartMeasurement();

	} else if (BSP_DISTANCIA_IsDataRedy()) {
		float distancia_cm = BSP_DISTANCIA_GetDistance();
		float salida_motor = PID_Compute(&APP_pid, distancia_cm);
		BSP_MOTOR_SetAngle((uint16_t)salida_motor, APP_MOTOR_DELAY_IN_MS_PER_DEGREE);

	} else if (BSP_DISTANCIA_IsTimeout()) {
		BSP_DISTANCIA_Reset();
	}
}

void PID_Init(PID_Controller *pid,
		float kp, float ki, float kd,
		float dt,
		float out_min, float out_max,
		float integral_min, float integral_max
		) {
	// Default
	APP_pid.integral = 0.0f;
	APP_pid.prev_error = 0.0f;

	// Constantes de proporcionalidad
	APP_pid.kp = kp;
	APP_pid.ki = ki;
	APP_pid.kd = kd;

	// Tiempo de muestreo
	APP_pid.dt = dt;

	// Limites
	APP_pid.out_min = out_min;
	APP_pid.out_max = out_max;
	APP_pid.integral_min = integral_min;
	APP_pid.integral_max = integral_max;
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

