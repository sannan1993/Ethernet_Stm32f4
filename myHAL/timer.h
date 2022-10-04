#ifndef TIMER_H
#define TIMER_H
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
void TM_PWM_Init(void);
void TM_TIMER_Init(void);
void TM_LEDS_Init(void);

void TM_PWM_Init_TIM2(void);
void TM_TIMER2_Init(void);
void TM_LEDS2_Init(void);

void TM_PWM_Init_TIM1(void);
void TM_TIMER1_Init(void);
void TM_LEDS1_Init(void);
// Tim5
void TIM5_Init();
#endif