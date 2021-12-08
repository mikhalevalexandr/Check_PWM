#include "main.h"
#include <string.h>
uint32_t MedianArray(uint32_t *arr, size_t size);
uint32_t GETPCLK2TIM(void);
void InsulationConditionFromIMD (uint32_t* BuffPeriod, uint32_t* BuffDutyCycle);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
uint32_t GETPCLK1TIM(void);