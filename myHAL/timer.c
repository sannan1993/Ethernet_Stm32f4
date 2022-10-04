#include "timer.h"

void TM_PWM_Init(void) {
  TIM_OCInitTypeDef TIM_OCStruct;
  
  /* Common settings */
  
  /* PWM mode 2 = Clear on compare match */
  /* PWM mode 1 = Set on compare match */
  TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
  
  /*
  To get proper duty cycle, you have simple equation
  
  pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
  
  where DutyCycle is in percent, between 0 and 100%
  
  25% duty cycle: 	pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
  50% duty cycle: 	pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
  75% duty cycle: 	pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
  100% duty cycle:	pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399
  
  Remember: if pulse_length is larger than TIM_Period, you will have output HIGH all the time
  */
  TIM_OCStruct.TIM_Pulse = 2099; /* 25% duty cycle */
  TIM_OC1Init(TIM4, &TIM_OCStruct);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  /*
  TIM_OCStruct.TIM_Pulse = 4199; // 50% duty cycle
  TIM_OC2Init(TIM4, &TIM_OCStruct);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
  TIM_OCStruct.TIM_Pulse = 6299; // 75% duty cycle 
  TIM_OC3Init(TIM4, &TIM_OCStruct);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
  TIM_OCStruct.TIM_Pulse = 8399; // 100% duty cycle
  TIM_OC4Init(TIM4, &TIM_OCStruct);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  */
}

void TM_PWM_Init_TIM2(void) {
  TIM_OCInitTypeDef TIM_OCStruct;
  
  /* Common settings */
  
  /* PWM mode 2 = Clear on compare match */
  /* PWM mode 1 = Set on compare match */
  TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
  
  /*
  To get proper duty cycle, you have simple equation
  
  pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
  
  where DutyCycle is in percent, between 0 and 100%
  
  25% duty cycle: 	pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
  50% duty cycle: 	pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
  75% duty cycle: 	pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
  100% duty cycle:	pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399
  
  Remember: if pulse_length is larger than TIM_Period, you will have output HIGH all the time
  */
  /*
  TIM_OCStruct.TIM_Pulse = 2099; // 25% duty cycle
  TIM_OC1Init(TIM2, &TIM_OCStruct);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
  */
  TIM_OCStruct.TIM_Pulse = 4199; // 50% duty cycle
  TIM_OC2Init(TIM2, &TIM_OCStruct);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  /*
  TIM_OCStruct.TIM_Pulse = 6299; // 75% duty cycle 
  TIM_OC3Init(TIM2, &TIM_OCStruct);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
  
  TIM_OCStruct.TIM_Pulse = 8399; // 100% duty cycle
  TIM_OC4Init(TIM2, &TIM_OCStruct);
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
  */
}
void TM_PWM_Init_TIM1(void) {
  TIM_OCInitTypeDef TIM_OCStruct;
  
  /* Common settings */
  
  /* PWM mode 2 = Clear on compare match */
  /* PWM mode 1 = Set on compare match */
  TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCStruct.TIM_OutputNState = TIM_OutputNState_Enable; // output n state
  TIM_OCStruct.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  
  /*
  To get proper duty cycle, you have simple equation
  
  pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
  
  where DutyCycle is in percent, between 0 and 100%
  
  25% duty cycle: 	pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
  50% duty cycle: 	pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
  75% duty cycle: 	pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
  100% duty cycle:	pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399
  
  Remember: if pulse_length is larger than TIM_Period, you will have output HIGH all the time
  */
  /*
  TIM_OCStruct.TIM_Pulse = 2099; // 25% duty cycle
  TIM_OC1Init(TIM2, &TIM_OCStruct);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
  */
  TIM_OCStruct.TIM_Pulse = 4199; // 50% duty cycle
  TIM_OC2Init(TIM1, &TIM_OCStruct);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  /*
  TIM_OCStruct.TIM_Pulse = 6299; // 75% duty cycle 
  TIM_OC3Init(TIM2, &TIM_OCStruct);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
  
  TIM_OCStruct.TIM_Pulse = 8399; // 100% duty cycle
  TIM_OC4Init(TIM2, &TIM_OCStruct);
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
  */
}
void TM_TIMER2_Init(void) {
  TIM_TimeBaseInitTypeDef TIM_BaseStruct;
  
  /* Enable clock for TIM4 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  /*	
  TIM4 is connected to APB1 bus, which has on F407 device 42MHz clock 				
  But, timer has internal PLL, which double this frequency for timer, up to 84MHz 	
  Remember: Not each timer is connected to APB1, there are also timers connected 	
  on APB2, which works at 84MHz by default, and internal PLL increase 				
  this to up to 168MHz 															
  
  Set timer prescaller 
  Timer count frequency is set with 
  
  timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)		
  
  In our case, we want a max frequency for timer, so we set prescaller to 0 		
  And our timer will have tick frequency		
  
  timer_tick_frequency = 84000000 / (0 + 1) = 84000000 
  */	
  TIM_BaseStruct.TIM_Prescaler = 0;
  /* Count up */
  TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  /*
  Set timer period when it have reset
  First you have to know max value for timer
  In our case it is 16bit = 65535
  To get your frequency for PWM, equation is simple
  
  PWM_frequency = timer_tick_frequency / (TIM_Period + 1)
  
  If you know your PWM frequency you want to have timer period set correct
  
  TIM_Period = timer_tick_frequency / PWM_frequency - 1
  
  In our case, for 10Khz PWM_frequency, set Period to
  
  TIM_Period = 84000000 / 10000 - 1 = 8399
  If you get TIM_Period larger than max timer value (in our case 65535),
  you have to choose larger prescaler and slow down timer tick frequency
  */
  TIM_BaseStruct.TIM_Period = 8399; /* 10kHz PWM */
  TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_BaseStruct.TIM_RepetitionCounter = 0;
  /* Initialize TIM2 */
  TIM_TimeBaseInit(TIM2, &TIM_BaseStruct);
  /* Start count on TIM2 */
  TIM_Cmd(TIM2, ENABLE);
}
void TM_TIMER1_Init(void) {
  TIM_TimeBaseInitTypeDef TIM_BaseStruct;
  
  /* Enable clock for TIM4 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  /*	
  TIM4 is connected to APB1 bus, which has on F407 device 42MHz clock 				
  But, timer has internal PLL, which double this frequency for timer, up to 84MHz 	
  Remember: Not each timer is connected to APB1, there are also timers connected 	
  on APB2, which works at 84MHz by default, and internal PLL increase 				
  this to up to 168MHz 															
  
  Set timer prescaller 
  Timer count frequency is set with 
  
  timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)		
  
  In our case, we want a max frequency for timer, so we set prescaller to 0 		
  And our timer will have tick frequency		
  
  timer_tick_frequency = 84000000 / (0 + 1) = 84000000 
  */	
  TIM_BaseStruct.TIM_Prescaler = 0;
  /* Count up */
  TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  /*
  Set timer period when it have reset
  First you have to know max value for timer
  In our case it is 16bit = 65535
  To get your frequency for PWM, equation is simple
  
  PWM_frequency = timer_tick_frequency / (TIM_Period + 1)
  
  If you know your PWM frequency you want to have timer period set correct
  
  TIM_Period = timer_tick_frequency / PWM_frequency - 1
  
  In our case, for 10Khz PWM_frequency, set Period to
  
  TIM_Period = 84000000 / 10000 - 1 = 8399
  If you get TIM_Period larger than max timer value (in our case 65535),
  you have to choose larger prescaler and slow down timer tick frequency
  */
  TIM_BaseStruct.TIM_Period = 8399; /* 10kHz PWM */
  TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_BaseStruct.TIM_RepetitionCounter = 0;
  /* Initialize TIM1 */
  TIM_TimeBaseInit(TIM1, &TIM_BaseStruct);
  /* Start count on TIM1 */
  TIM_Cmd(TIM1, ENABLE);
}
void TM_TIMER_Init(void) {
  TIM_TimeBaseInitTypeDef TIM_BaseStruct;
  
  /* Enable clock for TIM4 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  /*	
  TIM4 is connected to APB1 bus, which has on F407 device 42MHz clock 				
  But, timer has internal PLL, which double this frequency for timer, up to 84MHz 	
  Remember: Not each timer is connected to APB1, there are also timers connected 	
  on APB2, which works at 84MHz by default, and internal PLL increase 				
  this to up to 168MHz 															
  
  Set timer prescaller 
  Timer count frequency is set with 
  
  timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)		
  
  In our case, we want a max frequency for timer, so we set prescaller to 0 		
  And our timer will have tick frequency		
  
  timer_tick_frequency = 84000000 / (0 + 1) = 84000000 
  */	
  TIM_BaseStruct.TIM_Prescaler = 0;
  /* Count up */
  TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  /*
  Set timer period when it have reset
  First you have to know max value for timer
  In our case it is 16bit = 65535
  To get your frequency for PWM, equation is simple
  
  PWM_frequency = timer_tick_frequency / (TIM_Period + 1)
  
  If you know your PWM frequency you want to have timer period set correct
  
  TIM_Period = timer_tick_frequency / PWM_frequency - 1
  
  In our case, for 10Khz PWM_frequency, set Period to
  
  TIM_Period = 84000000 / 10000 - 1 = 8399
  
  If you get TIM_Period larger than max timer value (in our case 65535),
  you have to choose larger prescaler and slow down timer tick frequency
  */
  TIM_BaseStruct.TIM_Period = 8399; /* 10kHz PWM */
  TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_BaseStruct.TIM_RepetitionCounter = 0;
  /* Initialize TIM4 */
  TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
  /* Start count on TIM4 */
  TIM_Cmd(TIM4, ENABLE);
}
void TM_LEDS1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Clock for GPIOA */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  /* Alternating functions for pins */
  //GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  //GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
  //GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  //GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB ,GPIO_PinSource14, GPIO_AF_TIM1);
  /* Set pins */
  //GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  //GPIO_Init(GPIOD, &GPIO_InitStruct);
  GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void TM_LEDS2_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Clock for GPIOA */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* Alternating functions for pins */
  //GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  //GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
  //GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  //GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOA ,GPIO_PinSource1, GPIO_AF_TIM2);
  /* Set pins */
  //GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  //GPIO_Init(GPIOD, &GPIO_InitStruct);
  GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void TM_LEDS_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Clock for GPIOD */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  /* Alternating functions for pins */
  //GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  //GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
  //GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  //GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB ,GPIO_PinSource6, GPIO_AF_TIM4);
  /* Set pins */
  //GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  //GPIO_Init(GPIOD, &GPIO_InitStruct);
  GPIO_Init(GPIOB, &GPIO_InitStruct);
}
// Timer 3
void TM_TIMER_Init_3(void) {
  TIM_TimeBaseInitTypeDef TIM_BaseStruct;
  
  /* Enable clock for TIM4 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  /*	
  TIM4 is connected to APB1 bus, which has on F407 device 42MHz clock 				
  But, timer has internal PLL, which double this frequency for timer, up to 84MHz 	
  Remember: Not each timer is connected to APB1, there are also timers connected 	
  on APB2, which works at 84MHz by default, and internal PLL increase 				
  this to up to 168MHz 															
  
  Set timer prescaller 
  Timer count frequency is set with 
  
  timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)		
  
  In our case, we want a max frequency for timer, so we set prescaller to 0 		
  And our timer will have tick frequency		
  
  timer_tick_frequency = 84000000 / (0 + 1) = 84000000 
  */	
  TIM_BaseStruct.TIM_Prescaler = 0;
  /* Count up */
  TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  /*
  Set timer period when it have reset
  First you have to know max value for timer
  In our case it is 16bit = 65535
  To get your frequency for PWM, equation is simple
  
  PWM_frequency = timer_tick_frequency / (TIM_Period + 1)
  
  If you know your PWM frequency you want to have timer period set correct
  
  TIM_Period = timer_tick_frequency / PWM_frequency - 1
  
  In our case, for 10Khz PWM_frequency, set Period to
  
  TIM_Period = 84000000 / 10000 - 1 = 8399
  
  If you get TIM_Period larger than max timer value (in our case 65535),
  you have to choose larger prescaler and slow down timer tick frequency
  */
  TIM_BaseStruct.TIM_Period = 8399; /* 10kHz PWM */
  TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_BaseStruct.TIM_RepetitionCounter = 0;
  /* Initialize TIM3 */
  TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
  /* Start count on TIM3 */
  TIM_Cmd(TIM4, ENABLE);
  
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}
void TIM5_Init()
{
  // Enable clock for TIM2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  
  // TIM2 initialization overflow every 500ms
  // TIM2 by default has clock of 84MHz
  // Here, we must set value of prescaler and period,
  // so update event is 0.5Hz or 500ms
  // Update Event (Hz) = timer_clock / ((TIM_Prescaler + 1) * 
  // (TIM_Period + 1))
  // Update Event (Hz) = 84MHz / ((4199 + 1) * (9999 + 1)) = 2 Hz
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseInitStruct.TIM_Prescaler = 2;
  TIM_TimeBaseInitStruct.TIM_Period = 2680;//8300
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  
  // TIM2 initialize
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);
  // Enable TIM2 interrupt
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
  // Start TIM2
  TIM_Cmd(TIM5, ENABLE);
  
  // Nested vectored interrupt settings
  // TIM2 interrupt is most important (PreemptionPriority and 
  // SubPriority = 0)
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
}