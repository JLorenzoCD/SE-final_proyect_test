#ifndef __BSP_H
#define __BSP_H

#include <stdbool.h>
#include <stdint.h>

typedef void* Motor;
typedef void* Distancia;

typedef enum
{
  BSP_OK = 0,
  BSP_ERROR_ARG,
  BSP_ERROR
}BSP_State;

BSP_State BSP_Init(Distancia* medidor_distancia, Motor* motor);

void BSP_MOTOR_Update(Motor motor);

bool BSP_MOTOR_IsMoving(Motor motor);

void BSP_MOTOR_SetAngle(Motor motor, uint16_t angle, uint16_t speed_ms);

bool BSP_DISTANCIA_CanMeasure(Distancia medidor_distancia);

void BSP_DISTANCIA_StartMeasurement(Distancia medidor_distancia);

bool BSP_DISTANCIA_IsDataRedy(Distancia medidor_distancia);

float BSP_DISTANCIA_GetDistance(Distancia medidor_distancia);

bool BSP_DISTANCIA_IsTimeout(Distancia medidor_distancia);

BSP_State BSP_DISTANCIA_Reset(Distancia medidor_distancia);

#endif /* __BSP_H */
