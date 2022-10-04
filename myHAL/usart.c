#include "usart.h"
/* Private variables ---------------------------------------------------------*/
//struct uart_buffer rx3;
struct uart_buffer rx2;
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
void USART1_Configuration(uint32_t baud_rate)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate  		    = baud_rate;
  USART_InitStructure.USART_WordLength			= USART_WordLength_8b;
  USART_InitStructure.USART_StopBits   			= USART_StopBits_1;
  USART_InitStructure.USART_Parity     			= USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE);

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
void USART2_Configuration(uint32_t baud_rate)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate    = baud_rate;
  USART_InitStructure.USART_WordLength  = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits    = USART_StopBits_1;
  USART_InitStructure.USART_Parity      = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode        = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);
  USART_Cmd(USART2, ENABLE);

  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
void USART3_Configuration(uint32_t baud_rate)
{

  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10 , GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11,  GPIO_AF_USART3);

  GPIO_InitStructure.GPIO_OType					= GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd 					= GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode 					= GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin 					= GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed 				= GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority 	   = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


  USART_InitStructure.USART_BaudRate   			= baud_rate;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity  		    = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode				= USART_Mode_Rx | USART_Mode_Tx;

  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  USART_Init(USART3, &USART_InitStructure);
  USART_Cmd(USART3, ENABLE);
}
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
void UART4_Configuration(uint32_t baud_rate)
{

  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  // Enable GPIO clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

  GPIO_InitStructure.GPIO_OType					= GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd 					= GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode 					= GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin 					= GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed 				= GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate   			= baud_rate;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity  		    = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode				= USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(UART4, &USART_InitStructure);
  USART_Cmd(UART4, ENABLE);
}
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
//=> UART5 CONFIGURATION <=
// New debugging Uart
void UART5_Configuration(uint32_t baud_rate) {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// Enable GPIO clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //Configure only TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baud_rate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Tx; // only tx
	//USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(UART5, &USART_InitStructure);
	USART_Cmd(UART5, ENABLE);
}

// Helper Functions
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
void usart_SendString(USART_TypeDef* USARTx, uint8_t * str)
{
	while (*str != '\0')
	{
		usart_SendChar(USARTx,*str);
		str++;
	}
}

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
void usart_SendChar(USART_TypeDef* USARTx, uint8_t ch) {
	while (!(USARTx->SR & USART_SR_TXE))
		; // wait for TX to empty
	USART_SendData(USARTx, ch & 0xFF);
	while (!(USARTx->SR & USART_SR_TC))
		; // wait for Transmit complete
}

void usart_SendCharArry(USART_TypeDef* USARTx, uint8_t * str, uint16_t len)
{
	for(int i=0;i<len;i++)
	{
		usart_SendChar(USARTx,*(str+i));
	}
}
void usart2_clrPtr()
{
  // clear rx buffer
  rx2.iter=0;
  rx2.row.full=0;//not full
  rx2.row.notEmpty=0;//not empty
  rx2.row.r_ptr=0;
  rx2.row.w_ptr=0;
}
uint8_t usart2_avaliable()
{
  uint8_t rvalue=0;
  if(rx2.row.r_ptr != rx2.row.w_ptr)
  {
    rvalue=1; // data avaliable
  }
  return rvalue;
}
uint8_t usart2_readByte()
{
  uint8_t rvalue=0;
  rvalue=rx2.buff[rx2.row.r_ptr];
  if(rx2.row.r_ptr!=rx2.row.w_ptr)
  {
    rx2.row.r_ptr++;
    if (rx2.row.r_ptr > BUFLEN1-1)
    {
     rx2.row.r_ptr=0;
    }
  }
  return rvalue;
}
// todo
void usart2_readAll(uint8_t *data)
{
  uint32_t i=0;
  while(rx2.row.r_ptr != rx2.row.w_ptr)
  {
    *(data+i)=rx2.buff[rx2.row.w_ptr];
    rx2.row.r_ptr++;
  }
}
void USART2_IRQHandler(void)
{
  if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) // check the type of inturrupt
  {
    rx2.buff[rx2.row.w_ptr] = USART_ReceiveData(USART2);
    rx2.row.w_ptr++;
    if (rx2.row.w_ptr > BUFLEN1-1)
    {
     rx2.row.w_ptr=0;
    }
    /*
    rx1.buff[rx1.iter] = USART_ReceiveData(USART2);
    if (rx1.iter >= BUFLEN1)
    {
     rx1.iter=0;
    }
    else
    {
      rx1.iter++;
    }
    // nextion lcd end sequence
    if ((rx1.buff[rx1.iter-1] == 0xff) && (rx1.buff[rx1.iter-2] == 0xff) && (rx1.buff[rx1.iter-3] == 0xff) && (rx1.iter >= 2))
    {
     for (int i = 0; i < rx1.iter; i++)
     {
      rx1.sub_buff[rx1.row.w_ptr][i] = rx1.buff[i];
      //sub_buff[row][i] = Rxbuffer[i];
     }
     if(!rx1.row.full) // if not full
     {
        rx1.row.w_ptr=(rx1.row.w_ptr+1)%ROW; // next location
        rx1.row.notEmpty = 1; // not empty flage high indicatede that there is data
     }
     // if next location is the raed poiter, buffer full
     if(rx1.row.w_ptr==rx1.row.r_ptr)
     {
       rx1.row.full=1;
     }
     rx1.iter = 0; // flush main buffer
    }
    */
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
}
void USART1_IRQHandler(void)
{
  /*
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // check the type of inturrupt
  {
    rx1.buff[rx1.iter] = USART_ReceiveData(USART1);
    if (rx1.iter >= BUFLEN1)
    {
     rx1.iter=0;
    }
    else
    {
      rx1.iter++;
    }
    // nextion lcd end sequence
    if ((rx1.buff[rx1.iter-1] == 0xff) && (rx1.buff[rx1.iter-2] == 0xff) && (rx1.buff[rx1.iter-3] == 0xff) && (rx1.iter >= 2))
    {
     for (int i = 0; i < rx1.iter; i++)
     {
      rx1.sub_buff[rx1.row.w_ptr][i] = rx1.buff[i];
      //sub_buff[row][i] = Rxbuffer[i];
     }
     if(!rx1.row.full) // if not full
     {
        rx1.row.w_ptr=(rx1.row.w_ptr+1)%ROW; // next location
        rx1.row.notEmpty = 1; // not empty flage high indicatede that there is data
     }
     // if next location is the raed poiter, buffer full
     if(rx1.row.w_ptr==rx1.row.r_ptr)
     {
       rx1.row.full=1;
     }
     rx1.iter = 0; // flush main buffer
    }
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
  */
}
// Inturrupt Handler
// A LIFO buffer in implemented, whcih is not ideal in most situation but easy to implement
/*
void USART3_IRQHandler(void)
{
  if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) // check the type of inturrupt
  {
    rx3.buff[rx3.iter] = USART_ReceiveData(USART3);
    if (rx3.iter >= BUFLEN3)
    {
     rx3.iter=0;
    }
    else
    {
      rx3.iter++;
    }
    // nextion lcd end sequence
    if ((rx3.buff[rx3.iter-1] == 0xff) && (rx3.buff[rx3.iter-2] == 0xff) && (rx3.buff[rx3.iter-3] == 0xff) && (rx3.iter >= 2))
    {
     for (int i = 0; i < rx3.iter; i++)
     {
      rx3.sub_buff[rx3.row.w_ptr][i] = rx3.buff[i];
      //sub_buff[row][i] = Rxbuffer[i];
     }
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
    }
     USART_ClearITPendingBit(USART3, USART_IT_RXNE);
  }
}
*/