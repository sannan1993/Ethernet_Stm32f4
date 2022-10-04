// https://www.geeksforgeeks.org/convert-floating-point-number-string/
#ifndef MY_STDLIB
#define MY_STDLIB
#include<stdint.h>
void ftoa(float n, char *res, int afterpoint);
int intToStr(int x, char str[], int d);
void reverse(char *str, int len);
int8_t charToInt(char ch);
int32_t strToInt(char *str);
double strToFloat(char *str);
/*
 
break parameters

// templete 
int32_t startIndex = k, stopIndex = 0;
char temp[50];
// 1st field is the command 02
stopIndex = indexFrom(str, 0x03, startIndex);
if (stopIndex > 0)
{
	for (int i = startIndex; i < stopIndex; i++)
		temp[i - startIndex] = str[i];
	temp[stopIndex - startIndex] = 0; // add null
	startIndex = stopIndex + 1; //move forwared
	usart_SendString(UART5, "Found\r\n");
	usart_SendString(UART5, "Value=");
	usart_SendString(UART5, temp);
	usart_SendString(UART5, "\r\n");
}
// 2nd field is printer mode
stopIndex = indexFrom(str, 0x03, startIndex);
if (stopIndex > 0)
{
	for (int i = startIndex; i < stopIndex; i++)
		printer_mode[i - startIndex] = str[i];
	printer_mode[stopIndex - startIndex] = 0; // add null

	startIndex = stopIndex + 1; //move forwared
	usart_SendString(UART5, "Printer Mode=");
	usart_SendString(UART5, printer_mode);
	usart_SendString(UART5, "\r\n");
}
*/
#endif
