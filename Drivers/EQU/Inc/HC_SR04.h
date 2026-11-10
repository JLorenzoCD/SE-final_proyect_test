#ifndef HCSR04_F4_H
#define HCSR04_F4_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

//#define OS_INTERRUPT_SYNC

#ifdef OS_INTERRUPT_SYNC
#include "FreeRTOS.h"
#include "semphr.h"
#endif

// --- Constantes ---
// (Velocidad del sonido 343 m/s -> 0.0343 cm/µs) / 2 (ida y vuelta) = 0.01715
// 1 / 0.01715 = 58.3 cm por microsegundo
#define HCSR04_CM_CONVERSION_FACTOR 58.3f

/**
 * @brief Estructura de la instancia del sensor HC-SR04.
 */
typedef struct {
    // --- Configuración Pública (Asignar en hcsr04_init) ---
    TIM_HandleTypeDef* 	htim;        // Puntero al handle del Timer con base de tiempo de 1 us (ej: &htim2)
    uint32_t            IC_Channel; // Canal del timer usado para Input Capture (ej: TIM_CHANNEL_1)

    GPIO_TypeDef* 		Trig_Port;  // Puerto del pin Trigger (ej: GPIOA)
    uint16_t            Trig_Pin;   // Pin de Trigger (ej: GPIO_PIN_1)

    // --- Estado Interno (Privado) ---
    volatile uint32_t   _start_time_us;   // Timestamp del flanco de subida
    volatile uint32_t   _pulse_width_us;  // Ancho de pulso medido en µs
    volatile uint8_t    _data_ready;      // Flag (booleano)
#ifdef OS_INTERRUPT_SYNC
    SemaphoreHandle_t	OS_BinarySemaphore;
#endif

} HCSR04_HandleTypeDef;

/* --- Definición Local de la Macro GET_CAPTUREPOLARITY --- */
/*
 * (Definición estándar de la HAL, incluida aquí por si no está disponible)
 *
 * Lee directamente el registro CCER (Capture/Compare Enable Register) del timer
 * para determinar la polaridad de captura configurada.
 *
 * TIM_INPUTCHANNELPOLARITY_RISING:  (Valor 0x0)
 * TIM_INPUTCHANNELPOLARITY_FALLING: (Valor 0x2, TIM_CCER_CC1P)
 * TIM_INPUTCHANNELPOLARITY_BOTHEDGE:(Valor 0xA, TIM_CCER_CC1P | TIM_CCER_CC1NP)
 */
#define __HAL_TIM_GET_CAPTUREPOLARITY(__HANDLE__, __CHANNEL__) \
  ( ( (__HANDLE__)->Instance->CCER >> (__CHANNEL__) ) & \
    (TIM_CCER_CC1P | TIM_CCER_CC1NP) )
/* --- Fin de la Definición Local --- */

/**
 * @brief Inicializa la estructura del sensor y arranca el Input Capture.
 *
 * @param sensor Puntero a la estructura del sensor.
 * @param htim Puntero al handle del Timer (configurado en CubeMX).
 * @param IC_Channel Canal de Input Capture (ej: TIM_CHANNEL_1).
 * @param Trig_Port Puerto GPIO del pin Trigger.
 * @param Trig_Pin Pin GPIO del pin Trigger.
 */
void hcsr04_init(HCSR04_HandleTypeDef* sensor);

/**
 * @brief Inicia una nueva medición (envía el pulso de trigger).
 * Esta función retorna inmediatamente.
 *
 * @param sensor Puntero a la estructura del sensor.
 */
void hcsr04_start_measurement(HCSR04_HandleTypeDef* sensor);

/**
 * @brief Verifica si hay un nuevo dato de distancia disponible.
 *
 * @param sensor Puntero a la estructura del sensor.
 * @return 1 (true) si hay un nuevo dato, 0 (false) si no.
 */
int hcsr04_is_data_ready(const HCSR04_HandleTypeDef* sensor);

/**
 * @brief Obtiene la última distancia calculada.
 * No realiza una nueva medición. Resetea el flag 'data_ready'.
 *
 * @param sensor Puntero a la estructura del sensor.
 * @return La última distancia medida en cm. 0.0f si no hay dato.
 */
float hcsr04_get_distance_cm(HCSR04_HandleTypeDef* sensor);

/**
 * @brief MANEJADOR DE CALLBACKS DEL TIMER (IMPORTANTE).
 * Esta función DEBE ser llamada desde la callback global HAL_TIM_IC_CaptureCallback()
 * en tu archivo stm32f4xx_it.c.
 *
 * @param sensor Puntero a la estructura del sensor.
 * @param htim Puntero al handle del timer (provisto por la callback de HAL).
 */
void hcsr04_tim_ic_callback(HCSR04_HandleTypeDef* sensor, TIM_HandleTypeDef *htim);


#endif // HCSR04_F4_H
