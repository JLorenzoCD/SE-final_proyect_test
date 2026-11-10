#include "HC_SR04.h"

// --- Función privada para delay en microsegundos ---
// (Sin cambios)
static void delay_us_approx(uint32_t us)
{
    volatile uint32_t count = us * (SystemCoreClock / 1000000 / 20);
    while(count--);
}

/**
 * @brief Helper para obtener el flag de interrupción del timer
 * correspondiente al canal.
 */
static uint32_t get_tim_it_source(uint32_t IC_Channel)
{
    switch (IC_Channel)
    {
        case TIM_CHANNEL_1: return TIM_IT_CC1;
        case TIM_CHANNEL_2: return TIM_IT_CC2;
        case TIM_CHANNEL_3: return TIM_IT_CC3;
        case TIM_CHANNEL_4: return TIM_IT_CC4;
        default: return 0;
    }
}

/**
 * @brief Helper para obtener el canal activo de HAL
 * correspondiente al canal.
 */
static uint32_t get_hal_active_channel(uint32_t IC_Channel)
{
    switch (IC_Channel)
    {
        case TIM_CHANNEL_1: return HAL_TIM_ACTIVE_CHANNEL_1;
        case TIM_CHANNEL_2: return HAL_TIM_ACTIVE_CHANNEL_2;
        case TIM_CHANNEL_3: return HAL_TIM_ACTIVE_CHANNEL_3;
        case TIM_CHANNEL_4: return HAL_TIM_ACTIVE_CHANNEL_4;
        default: return 0;
    }
}


/**
 * @brief Inicializa la estructura del sensor.
 * (MODIFICADO)
 */
void hcsr04_init(HCSR04_HandleTypeDef* sensor){
    sensor->_data_ready = 0;
    sensor->_pulse_width_us = 0;
    sensor->_start_time_us = 0;

    // Asegurarse que el pin de Trigger esté en bajo
    HAL_GPIO_WritePin(sensor->Trig_Port, sensor->Trig_Pin, GPIO_PIN_RESET);

    // --- CAMBIO ---
    // 1. Configurar la polaridad inicial para RISING (lista para la 1ra medición)
    __HAL_TIM_SET_CAPTUREPOLARITY(sensor->htim, sensor->IC_Channel, TIM_INPUTCHANNELPOLARITY_RISING);

    // 2. Limpiar cualquier flag de interrupción pendiente
    uint32_t it_source = get_tim_it_source(sensor->IC_Channel);
    if(it_source) {
        __HAL_TIM_CLEAR_IT(sensor->htim, it_source);
    }

    // 3. Iniciar SOLO el contador del timer (la base).
    // NO iniciamos la interrupción de IC todavía.
    if (HAL_TIM_Base_Start(sensor->htim) != HAL_OK)
    {
        // Manejar error
    }
}

/**
 * @brief Inicia una nueva medición (envía el pulso de trigger).
 * (MODIFICADO)
 */
void hcsr04_start_measurement(HCSR04_HandleTypeDef* sensor)
{
    // Resetear el flag de datos
    sensor->_data_ready = 0;

    // --- CAMBIO ---
    // 1. Asegurar que la polaridad esté en RISING (para el inicio del pulso)
    // Esto es crucial si una medición anterior falló y quedó en FALLING.
    __HAL_TIM_SET_CAPTUREPOLARITY(sensor->htim, sensor->IC_Channel, TIM_INPUTCHANNELPOLARITY_RISING);

    // 2. Limpiar cualquier flag de interrupción pendiente del canal
    uint32_t it_source = get_tim_it_source(sensor->IC_Channel);
    if(it_source) {
        __HAL_TIM_CLEAR_IT(sensor->htim, it_source);
    } else {
        return; // Canal no válido
    }

    // 3. HABILITAR la interrupción de Input Capture
    if (HAL_TIM_IC_Start_IT(sensor->htim, sensor->IC_Channel) != HAL_OK)
    {
		HAL_TIM_IC_DeInit(sensor->htim);
		HAL_TIM_IC_Init(sensor->htim);
    	// Manejar error
        return;
    }

    // 4. Generar pulso de Trigger de 10µs (como antes)
    HAL_GPIO_WritePin(sensor->Trig_Port, sensor->Trig_Pin, GPIO_PIN_SET);
    delay_us_approx(10); // Espera 10 microsegundos
    HAL_GPIO_WritePin(sensor->Trig_Port, sensor->Trig_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Verifica si hay un nuevo dato de distancia disponible.
 * (Sin cambios)
 */
int hcsr04_is_data_ready(const HCSR04_HandleTypeDef* sensor)
{
    return (int)sensor->_data_ready;
}

/**
 * @brief Obtiene la última distancia calculada.
 * (Sin cambios)
 */
float hcsr04_get_distance_cm(HCSR04_HandleTypeDef* sensor)
{
    if (sensor->_data_ready)
    {
        uint32_t pulse_width = sensor->_pulse_width_us;
        sensor->_data_ready = 0;

        return (float)pulse_width / HCSR04_CM_CONVERSION_FACTOR;
    }

    return 0.0f;
}


/**
 * @brief MANEJADOR DE CALLBACKS DEL TIMER
 * (MODIFICADO)
 */
void hcsr04_tim_ic_callback(HCSR04_HandleTypeDef* sensor, TIM_HandleTypeDef *htim)
{
    // 1. Verificar si la interrupción viene del timer que nos interesa
    if (htim->Instance != sensor->htim->Instance)
    {
        return;
    }

    // 2. Determinar el canal activo esperado
    uint32_t active_channel = get_hal_active_channel(sensor->IC_Channel);
    if (active_channel == 0) return; // Canal no válido

    // 3. Verificar si la interrupción es de NUESTRO canal
    if (htim->Channel != active_channel)
    {
        return;
    }

    // --- Lógica de Flanco de Subida / Bajada (Lógica de polaridad sin cambios) ---

    if (__HAL_TIM_GET_CAPTUREPOLARITY(htim, sensor->IC_Channel) == TIM_INPUTCHANNELPOLARITY_RISING)
    {
        // --- ACABA DE OCURRIR UN FLANCO DE SUBIDA ---

        // 1. Leer el valor de captura (tiempo de inicio)
        sensor->_start_time_us = HAL_TIM_ReadCapturedValue(htim, sensor->IC_Channel);

        // 2. Cambiar la polaridad para capturar el flanco de bajada
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, sensor->IC_Channel, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else // La polaridad actual es FALLING (TIM_INPUTCHANNELPOLARITY_FALLING)
    {
        // --- ACABA DE OCURRIR UN FLANCO DE BAJADA ---

        // 1. Leer el valor de captura (tiempo de fin)
        uint32_t end_time_us = HAL_TIM_ReadCapturedValue(htim, sensor->IC_Channel);

        // 2. Calcular el ancho del pulso
        if (end_time_us > sensor->_start_time_us)
        {
            sensor->_pulse_width_us = end_time_us - sensor->_start_time_us;
        }
        else // Desborde del timer
        {
            uint32_t timer_max = __HAL_TIM_GET_AUTORELOAD(htim);
            sensor->_pulse_width_us = (timer_max - sensor->_start_time_us) + end_time_us + 1;
        }

        // 3. Marcar el flag de "Dato Listo"
        sensor->_data_ready = 1;

        // 4. Cambiar la polaridad de nuevo a flanco de subida (para la próxima medición)
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, sensor->IC_Channel, TIM_INPUTCHANNELPOLARITY_RISING);

        // 5. --- CAMBIO ---
        // DESHABILITAR la interrupción de IC.
        // Se volverá a habilitar en la próxima llamada a hcsr04_start_measurement()
        HAL_TIM_IC_Stop_IT(sensor->htim, sensor->IC_Channel);

#ifdef OS_INTERRUPT_SYNC
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xSemaphoreGiveFromISR(sensor->OS_BinarySemaphore, &xHigherPriorityTaskWoken );

        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
    }
}

