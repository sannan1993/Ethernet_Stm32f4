/*
Author: Moiz khan
Designed for : GMS 
Description : This library supports the gpio api of arduino IDE
*/

#include "tm_stm32f4_spi.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include <misc.h>
#include "defines.h"
void pinMode ( GPIO_TypeDef* GPIOx, uint16_t pin,  uint16_t mode , uint16_t pullup_mode);
void digitalWrite ( GPIO_TypeDef* GPIOx, uint16_t pin, uint8_t logic);
//void serial1_begin(void);

//begin
void Serial1_begin(void);
void Serial1_print(char* str);
void serial2_begin(void);
void Serial2_print(char* str);
void Serial3_begin(void);
void Serial3_print (char* str);
void Serial4_begin(void);
void Serial5_begin(void);
void Serial5_print(char* str);
void Serial6_begin(void);
uint16_t Serial_read(void);
void usart_SendString(USART_TypeDef* USARTx,char* str);
char* Serial_readString( void );


void debug_print(char* str);
void debug_println(char* str);
//SPI
uint8_t readReg_SPI (uint8_t reg);
unsigned long readReg24_SPI (uint32_t reg2);
void writeByte_SPI (int reg, int msb);
