#ifndef USART_H
#define USART_H
#include <stdint.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_rcc.h>
#include <misc.h>
/* Private define ------------------------------------------------------------*/
#define BUFLEN3 100
#define BUFLEN2 100
#define BUFLEN1 100
//#define ROW 10 // number of packet that can be stored
//#define COL 20 // length of packet
// pointer structure
struct pointer{
   uint8_t r_ptr;
   uint8_t w_ptr;
   uint8_t full;
   uint8_t notEmpty; 
};
struct uart_buffer{
  uint32_t iter;
  struct pointer row;
  uint8_t buff[BUFLEN3]; // main buffer//10byte gard
  //uint8_t sub_buff[ROW][COL]; // no sub buffer
};
// USART
void USART1_Configuration(uint32_t baud_rate);
void USART2_Configuration(uint32_t baud_rate);
void USART3_Configuration(uint32_t baud_rate);
// UART
void UART4_Configuration(uint32_t baud_rate);
void UART5_Configuration(uint32_t baud_rate);

void usart_SendString(USART_TypeDef* USARTx, uint8_t*);
void usart_SendChar(USART_TypeDef* USARTx,uint8_t);
void usart_SendCharArry(USART_TypeDef* USARTx, uint8_t * str, uint16_t len);
// reading function
void usart2_readAll(uint8_t *data);
uint8_t usart2_avaliable();
void usart2_readAll(uint8_t *data);
uint8_t usart2_readByte();
void usart2_clrPtr();
/*
ring buffer 
// write increament
if(!rx3.row.full) // if not full
{
rx3.row.w_ptr=(rx3.row.w_ptr+1)%ROW; // next location
rx3.row.notEmpty = 1; // not empty flage high indicatede that there is data
     }
// if next location is the raed poiter, buffer full
if(rx3.row.w_ptr==rx3.row.r_ptr)
{
rx3.row.full=1;
     }
rx3.iter = 0; // flush main buffer


// read logic
if(rx3.row.notEmpty==1) // buff not empty
{
if(rx3.row.notEmpty==1) // if not empty
{
rx3.row.r_ptr = (rx3.row.r_ptr+1)%ROW;
rx3.row.full = 0; // not full
      }
// if next location is the raed poiter, buffer empty
if(rx3.row.r_ptr%ROW==rx3.row.w_ptr)
{
rx3.row.notEmpty=0;
}
}
*/
#endif
