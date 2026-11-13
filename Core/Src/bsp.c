#include "bsp.h"

#include "stm32f4xx_hal.h"
#include "main.h"
#include "tim.h"
#include "gpio.h"

#include "sg90.h"
#include "HC_SR04.h"

// Sensores y actuadores de la APP
extern Motor APP_motor;
extern Distancia APP_distancia;

#define HCSR04_MEASUREMENT_INTERVAL_MS 20
#define HCSR04_TIMEOUT_ECO_MS 40

#define SG90_MAX_ANGLE 180
#define SG90_PWM_MIN_PULSE 500
#define SG90_PWM_MAX_PULSE 2500
#define SG90_DELAY_IN_MS_PER_DEGREE 10

// Perifericos
HCSR04_HandleTypeDef hcsr04_medidor_dist;
SG90_HandleTypeDef sg90_servo_motor;

// Variables de estado
volatile uint32_t hcsr04_last_tick_read = 0;
volatile bool hcsr04_is_measurement = false;

void SystemClock_Config(void);
void Error_Handler(void);

// Funciones externas
BSP_State BSP_Init() {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM2_Init();

	if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK) {
		return BSP_ERROR;
	}

	// Se inicializa el medidor de distancia
	hcsr04_medidor_dist.htim = &htim2;
	hcsr04_medidor_dist.IC_Channel = TIM_CHANNEL_2;
	hcsr04_medidor_dist.Trig_Pin = Trigger_Pin;
	hcsr04_medidor_dist.Trig_Port = Trigger_GPIO_Port;
	hcsr04_init(&hcsr04_medidor_dist);

	// Se inicializa el servo motor
	sg90_servo_motor.htim = &htim2;
	sg90_servo_motor.channel = TIM_CHANNEL_1;
	SG90_Init(&sg90_servo_motor, SG90_MAX_ANGLE, SG90_PWM_MIN_PULSE, SG90_PWM_MAX_PULSE);

	APP_distancia = (HCSR04_HandleTypeDef*)&hcsr04_medidor_dist;
	APP_motor = (SG90_HandleTypeDef*)&sg90_servo_motor;

	return BSP_OK;
}

// Funciones utilizadas para el servomotor SG90
void BSP_MOTOR_Update() {
	if (APP_motor == NULL) {
		Error_Handler();
	}

	SG90_Update_Angle((SG90_HandleTypeDef*)APP_motor);
}

bool BSP_MOTOR_IsMoving() {
	if (APP_motor == NULL) {
		Error_Handler();
	}

	bool is_moving = ((SG90_HandleTypeDef*)APP_motor)->is_moving;
	return is_moving;
}

void BSP_MOTOR_SetAngle(uint16_t angle) {
	if (APP_motor == NULL) {
		Error_Handler();
	}

	SG90_Set_Angle_Target((SG90_HandleTypeDef*)APP_motor, angle, SG90_DELAY_IN_MS_PER_DEGREE);
}

// Funciones utilizadas para el medidor de distancia ultrasonico HC-SR04
bool BSP_DISTANCIA_CanMeasure() {
	if (APP_distancia == NULL) {
		Error_Handler();
	}

	bool can_measure = !hcsr04_is_measurement &&
			(HAL_GetTick() - hcsr04_last_tick_read > HCSR04_MEASUREMENT_INTERVAL_MS);

	return can_measure;
}

void BSP_DISTANCIA_StartMeasurement() {
	if (APP_distancia == NULL) {
		Error_Handler();
	}

	hcsr04_is_measurement = 1;
	hcsr04_start_measurement((HCSR04_HandleTypeDef*)APP_distancia);
}

bool BSP_DISTANCIA_IsDataRedy() {
	if (APP_distancia == NULL) {
		Error_Handler();
	}

	bool is_data_ready = hcsr04_is_data_ready((HCSR04_HandleTypeDef*)APP_distancia);

	return is_data_ready;
}

float BSP_DISTANCIA_GetDistance() {
	if (APP_distancia == NULL) {
		Error_Handler();
	}

	hcsr04_is_measurement = 0;
	hcsr04_last_tick_read = HAL_GetTick();

	float distancia_cm = hcsr04_get_distance_cm((HCSR04_HandleTypeDef*)APP_distancia);

	return distancia_cm;
}

bool BSP_DISTANCIA_IsTimeout() {
	if (APP_distancia == NULL) {
		Error_Handler();
	}

	bool is_timeout = hcsr04_is_measurement
			&& (HAL_GetTick() - hcsr04_last_tick_read > HCSR04_TIMEOUT_ECO_MS);

	return is_timeout;
}

BSP_State BSP_DISTANCIA_Reset() {
	if (APP_distancia == NULL) {
		Error_Handler();
	}

	hcsr04_is_measurement = 0;
	hcsr04_last_tick_read = HAL_GetTick();

	// Limpiar channel del timer
	TIM_HandleTypeDef* 	htim =((HCSR04_HandleTypeDef*)APP_distancia)->htim;
	uint32_t            chanel = ((HCSR04_HandleTypeDef*)APP_distancia)->IC_Channel;
	if (HAL_TIM_IC_Stop_IT(htim, chanel) != HAL_OK) {
		return BSP_ERROR;
	}

	return BSP_OK;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

// Callback para realizar las mediciones del medidor de distancia
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (APP_distancia == NULL) return;

	hcsr04_tim_ic_callback((HCSR04_HandleTypeDef*)APP_distancia, htim);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */


