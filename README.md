# Laboratorio 4 INTEGRADOR: Beam & Ball

- Materia: Sistemas de instrumentalización - Sistemas Embebidos - Famaf
- Alumno: José Lorenzo Canovas
- Año: 2026
- Microcontrolador utilizado: STM32F407G-Disc1

---

## Código utilizado
> Nota: La explicación del código se encuentra en comentarios para facilitar la lectura y evitar referencias en el texto a este.

#### Archivo **main.c**
```c
#include <stdlib.h>
#include <stdint.h>

// Archivo de abstracción de hardware
#include "bsp.h"

// Archivos extra, para legibilidad del main.c
#include "pid.h" // controlador PID
#include "utils.h" // se obtiene el filtro para la distancia

// Variables de los sensores
Motor APP_motor = NULL;
Distancia APP_distancia = NULL;

// Variable del controlador PID
PID_Controller APP_pid;

int main(void) {
    // Se inicializa el hardware abstraído
	BSP_Init();

	PID_Init(&APP_pid,
			1.2f, 0.05f, 0.4f,
			0.06f,
			0.0f, 180.0f,
			-30.0f, 30.0f
	);
	// Distancia hasta el centro de la viga
	PID_SetTarget(&APP_pid, 20.0f);

	while (1) {
        // Se va actualizando la posición del motor a través del tiempo
		BSP_MOTOR_Update();

		if (BSP_DISTANCIA_CanMeasure()) {
			BSP_DISTANCIA_StartMeasurement();

		} else if (BSP_DISTANCIA_IsDataRedy()) {
			float distancia_cm = BSP_DISTANCIA_GetDistance();

            // Filtro la distancia obtenida de la medición y calculo PID
            float salida_motor = PID_Compute(
				&APP_pid,
				filter_distance_cm(distancia_cm)
			);
            // Actualizo el angulo del motor según PID
			BSP_MOTOR_SetAngle((uint16_t)salida_motor);

		} else if (BSP_DISTANCIA_IsTimeout()) {
            // Si hay un error en la medición de la distancia y salta el timeout,
            // entonces reinicio la medición de distancia y reinicio los filtros,
            // ya que el nuevo valor de la distancia posiblemente este muy
            // alejado de los anteriores.

			BSP_DISTANCIA_Reset();
			filter_reset();
		}
	}
}
```

#### Archivo **bsp.c**
```c
#include "bsp.h"

// ...

// Callback para realizar las mediciones del medidor de distancia
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (APP_distancia == NULL) return;

	hcsr04_tim_ic_callback((HCSR04_HandleTypeDef*)APP_distancia, htim);
}

// ...
```

## Conclusiones
Siguiendo la recomendación del profe, termine por utilizar un archivo **BSP**, el cual reconozco que le dio un gran cambio al proyecto. Este cambio se me hizo mas notorio cuando me dio por realizar algunos tests creando otro proyecto, y por la reutilización de código de laboratorios anteriores. La separación entre el código de la aplicación y el código del hardware le da una gran **legibilidad** a la aplicación, ademas del beneficio de **abstraerte del hardware**.

Para mejorar mas la legibilidad a la aplicación termine por separar funciones utilizadas tanto para el **Controlador PID** como para una función de **filtro para la distancia** obtenida.

En un comienzo no tenia filtro, pero viendo que el hardware utilizado me daba mediciones diferentes para objetos fijos y también teniendo en cuenta la posibilidad de que me diera falsas mediciones debido a que la "viga" estaba rodeada por un cilindro hueco, termine por meter dicho filtro. El **filtro utilizado** fue una combinación de:
- eliminar valores fuera de rango (del largo de la viga)
- filtro de mediana de 3 muestras
- filtro exponencial (EMA)

Aunque no pude ajustar correctamente el **PID** para que el proyecto funcionase como se suponía que debía de hacerlo, este proyecto me mostró la dificultad que se tiene al trabajar con **sistemas inerciales**, del cual, **nunca había tenido que tener en cuenta este tipo de problemas** y por lo tanto ni siguiera tenia idea de las **posibles soluciones**.

## Posibles mejoras
1. **Cambiar el hardware** de distancia por ultrasonido por el de luz, lo cual debería de mejorar la estabilidad de la viga ya que se quitarían las **mediciones erráticas** que recibe de vez en cuando el sensor por ultrasonido. Ademas de permitirme **experimentar** la facilidad del **cambio del hardware por el archivo BSP**.
2. Utilizar **UART** o **USB** para obtener y enviar datos del micro a la PC, ya que al querer hacerlo en el proyecto de prueba termine fallando. Esto debería de **facilitar muchísimo el cambio de las constantes K del PID**.
