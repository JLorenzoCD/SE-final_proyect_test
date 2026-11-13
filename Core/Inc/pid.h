#ifndef __PID_H
#define __PID_H

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

void PID_Init(PID_Controller *pid,
		float kp, float ki, float kd,
		float dt,
		float out_min, float out_max,
		float integral_min, float integral_max
		);
void PID_SetTarget(PID_Controller *pid, float target);
float PID_Compute(PID_Controller *pid, float current_position);

#endif /* __PID_H */
