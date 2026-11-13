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

BSP_State BSP_Init();

void BSP_MOTOR_Update();

bool BSP_MOTOR_IsMoving();

void BSP_MOTOR_SetAngle(uint16_t angle);

bool BSP_DISTANCIA_CanMeasure();

void BSP_DISTANCIA_StartMeasurement();

bool BSP_DISTANCIA_IsDataRedy();

float BSP_DISTANCIA_GetDistance();

bool BSP_DISTANCIA_IsTimeout();

BSP_State BSP_DISTANCIA_Reset();

#endif /* __BSP_H */
