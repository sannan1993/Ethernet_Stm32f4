#include "my_arduino.h"
// digital mode .......................................................................
void pinMode ( GPIO_TypeDef* GPIOx, uint16_t pin,  const uint16_t  mode , const uint16_t pullup_mode)
{
  GPIO_InitTypeDef gpio_init;
  
  if (GPIOx == GPIOA){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  }
  if (GPIOx == GPIOB){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  }
  if (GPIOx == GPIOC){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  }
  if (GPIOx == GPIOD){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  }
  if (GPIOx == GPIOE){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  }   
  //gpio led 
  gpio_init.GPIO_Mode  = mode; //GPIO_Mode_OUT;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_Pin   = pin;//
  gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
  gpio_init.GPIO_PuPd  = pullup_mode;//GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOx, &gpio_init);
}

void digitalWrite ( GPIO_TypeDef* GPIOx, uint16_t pin, uint8_t logic){
  GPIO_WriteBit (GPIOx , pin ,logic);
}  

// USART1 For GSM 

void Serial1_begin(void)
{
  ///PA9 TX and PA10 is RX
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
  
  // Initialization of GPIOA
  GPIO_InitStructure.GPIO_OType		= GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_9 | GPIO_Pin_10 ;					
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  // Initialization of USART1
  USART_InitStructure.USART_BaudRate   	= 9600;
  USART_InitStructure.USART_WordLength  = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits    = USART_StopBits_1;
  USART_InitStructure.USART_Parity  	= USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode	= USART_Mode_Rx | USART_Mode_Tx;
  
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
}
////////////USART2 For 485 

void Serial2_begin(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
  
  // Initialization of GPIOA
  GPIO_InitStructure.GPIO_OType		= GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_2 | GPIO_Pin_3 ;					
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  // Initialization of USART2
  USART_InitStructure.USART_BaudRate   	= 9600;
  USART_InitStructure.USART_WordLength  = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits    = USART_StopBits_1;
  USART_InitStructure.USART_Parity  	= USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode	= USART_Mode_Rx | USART_Mode_Tx;
  
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  USART_Init(USART2, &USART_InitStructure);
  USART_Cmd(USART2, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

////////USART3 FOR ESP 

void Serial3_begin(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  // Enable GPIO clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
  
  // Initialization of GPIOB
  GPIO_InitStructure.GPIO_OType		= GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_10 | GPIO_Pin_11 ;					
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  // Initialization of USART3
  USART_InitStructure.USART_BaudRate   	= 115200;
  USART_InitStructure.USART_WordLength  = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits    = USART_StopBits_1;
  USART_InitStructure.USART_Parity  	= USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode	= USART_Mode_Rx | USART_Mode_Tx;
  
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  USART_Init(USART3, &USART_InitStructure);
  USART_Cmd(USART3, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Serial4_begin(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
  
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  // Initialization of GPIOC
  GPIO_InitStructure.GPIO_OType		= GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_10 | GPIO_Pin_11;				
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
 
  
  // Initialization of  UART4
  USART_InitStructure.USART_BaudRate   	= 9600;
  USART_InitStructure.USART_WordLength  = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits    = USART_StopBits_1;
  USART_InitStructure.USART_Parity  	= USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode	= USART_Mode_Rx | USART_Mode_Tx;
  
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
  USART_Init(UART4, &USART_InitStructure);
  USART_Cmd(UART4, ENABLE);  
  
  
}

//////// UART5 FOR EEPROM   C12 and D2

void Serial5_begin(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  // Initialization of GPIOC
  GPIO_InitStructure.GPIO_OType		= GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_12 ;				
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
  GPIO_InitStructure.GPIO_OType		= GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_2 ;				
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
  
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  
  // Initialization of  UART5
  USART_InitStructure.USART_BaudRate   	= 115200;
  USART_InitStructure.USART_WordLength  = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits    = USART_StopBits_1;
  USART_InitStructure.USART_Parity  	= USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode	= USART_Mode_Rx | USART_Mode_Tx;
  
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
  USART_Init(UART5, &USART_InitStructure);
  USART_Cmd(UART5, ENABLE);  
  
  
}
/////////////////////////////////////////////////////USART6  FOR DEBUG

void Serial6_begin(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  // Enable GPIO clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
  
  // Initialization of GPIOC
  GPIO_InitStructure.GPIO_OType		= GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_6 | GPIO_Pin_7 ;					
  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  // Initialization of USART6
  USART_InitStructure.USART_BaudRate   	= 115200;
  USART_InitStructure.USART_WordLength  = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits    = USART_StopBits_1;
  USART_InitStructure.USART_Parity  	= USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode	= USART_Mode_Rx | USART_Mode_Tx;
  
  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
  USART_Init(USART6, &USART_InitStructure);
  USART_Cmd(USART6, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


/////////////////////////////////////////////////////

void usart_SendString(USART_TypeDef* USARTx,char* str)
{
  while (*str != '\0')
  {
    while (USART_GetFlagStatus(USARTx,USART_FLAG_TC) == RESET);
    USART_SendData(USARTx,*str);
    while (USART_GetFlagStatus(USARTx,USART_FLAG_TC) == RESET);
    str++;
  }
}


void Serial1_print(char* str)
{
  usart_SendString(USART1,str);
}

void Serial2_print(char* str)
{
  usart_SendString(USART2,str);
}

/////////////////// uart5 sending command
void Serial5_print (char* str)
{
  usart_SendString(UART5,str);
}

/////////////////// send string in usart3 

void Serial3_print (char* str)
{
  usart_SendString(USART3,str);
}

///////////////////for debug

void debug_print (char* str)
{
  usart_SendString(UART4,str); 
}
void debug_println(char* str)
{
  debug_print(str);
  debug_print("\r\n");
}
//////////////////for send character in usart3
void Serialchar (char ch)
{
  while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
  USART_SendData(USART3,ch);
}
uint16_t Serial_read(void)

{
  // Wait until data is received
  while ((USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET ) &&  (USART_GetFlagStatus(USART3, USART_FLAG_IDLE) == RESET ));
  //USART_FLAG_IDLE
  // Read received char
  return USART_ReceiveData(USART3);
}

//get String 
char* Serial_readString( void ){
  int i=0;
  static char buf[100]; 
  
  while (1)
  {
    //Serialprint ("\r\n -> receive the character");
    buf[i++] = Serial_read(); //read the serial 
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
    USART_SendData(USART3,buf[i-1]);
    if (buf[i-1] == '$'){
      buf[i] = 0;
      //  Serialprint ("\r\n -> string completed!!!!....");
      break;
    }
    if (i >= 99){
      i=0; //clear the variable
      //   Serialprint ("\r\n -> serial buffer overflowed....");
      break;
      
    }
  } 
  return buf;
}

//read byte from a register in SPI protocol

///////////////////////////Function for read 8 bit resister


uint8_t readReg_SPI (uint8_t reg)
{    
  uint8_t data;
  digitalWrite (GPIOA, GPIO_Pin_4, 0);
  TM_SPI_Send(SPI1,reg);
  data = TM_SPI_Send(SPI1,0x00);
  digitalWrite (GPIOA, GPIO_Pin_4, 1);
  return data;
}


///////////////////////////Function for read 24 bit register

unsigned long readReg24_SPI (uint32_t reg2)
{
  
  unsigned long ret0=0;
  unsigned int ret1=0;
  unsigned char ret2=0;
  
  digitalWrite (GPIOA, GPIO_Pin_4, 0);
  TM_SPI_Send(SPI1,reg2);
  
  ret0  = TM_SPI_Send(SPI1,0x00); //lsb
  ret1 = TM_SPI_Send(SPI1,0x00);
  ret2 = TM_SPI_Send(SPI1,0x00); //msb
  
  digitalWrite (GPIOA, GPIO_Pin_4, 1);
  ret0= (ret0<<16)|(ret1<<8)|ret2;
  
  return ret0;

}


unsigned long readReg16_SPI (uint16_t reg3)
{
 unsigned int ret=0;
 unsigned char ret0=0;
 digitalWrite (GPIOA, GPIO_Pin_4, 0);
   TM_SPI_Send(SPI1,reg3);

  ret  = TM_SPI_Send(SPI1,0x00); //lsb
  ret0 = TM_SPI_Send(SPI1,0x00);
  
   digitalWrite (GPIOA, GPIO_Pin_4, 1);
   
    ret= (ret<<8)|ret0;
    return ret;
 
}


//////////////////////////function for write 8 bit register
void writeByte_SPI (int reg, int msb){
    digitalWrite (GPIOA, GPIO_Pin_4, 0);
    TM_SPI_Send(SPI1, (reg|0x80));
    TM_SPI_Send(SPI1, msb); // msb
    TM_SPI_Send(SPI1,0x00);  //lsb
    digitalWrite (GPIOA, GPIO_Pin_4, 1);
    
}


