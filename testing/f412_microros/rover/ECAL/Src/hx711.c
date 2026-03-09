/*
 * hx711.c
 *
 *  Created on: Feb 11, 2026
 *      Author: Hassan Khaled & Adel Raef
 */


#include "hx711.h"

/* Pin definitions */
#define DT_PIN   GPIO_PIN_8
#define DT_PORT  GPIOB
#define SCK_PIN  GPIO_PIN_9
#define SCK_PORT GPIOB

/* Private variables */
static TIM_HandleTypeDef *hx711_tim;

static int32_t hx711_tare = 0;
static float hx711_coefficient = 1.0f;

/* Private function */
static void HX711_DelayUs(uint16_t delay)
{
    __HAL_TIM_SET_COUNTER(hx711_tim, 0);
    while (__HAL_TIM_GET_COUNTER(hx711_tim) < delay);
}

static int32_t HX711_ReadRaw(void)
{
    uint32_t data = 0;
    uint32_t startTime = HAL_GetTick();

    /* Wait until data ready */
    while(HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
    {
        if(HAL_GetTick() - startTime > 200)
            return 0;
    }

    /* Read 24 bits */
    for(int8_t i = 0; i < 24; i++)
    {
        HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
        HX711_DelayUs(1);

        data <<= 1;

        HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
        HX711_DelayUs(1);

        if(HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
            data++;
    }

    /* Set gain = 128 (1 extra pulse) */
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
    HX711_DelayUs(1);
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
    HX711_DelayUs(1);

    /* Convert signed */
    data ^= 0x800000;

    return (int32_t)data;
}

/* ================= PUBLIC FUNCTIONS ================= */

void HX711_Init(TIM_HandleTypeDef *htim)
{
    hx711_tim = htim;

    HAL_TIM_Base_Start(hx711_tim);

    /* HX711 reset sequence */
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
}

void HX711_SetCalibration(int32_t tare, float knownOriginal, float knownHX711)
{
    hx711_tare = tare;
    hx711_coefficient = knownOriginal / knownHX711;
}

int32_t HX711_GetWeight()
{
//    int32_t total = 0;
//    const uint8_t samples = 1;
//
//    for(uint8_t i = 0; i < samples; i++)
//    {
//        total += HX711_ReadRaw();
//    }
//
//    int32_t average = total / samples;
int32_t reading = HX711_ReadRaw();
    return (int32_t)((reading - hx711_tare) * hx711_coefficient);
}
