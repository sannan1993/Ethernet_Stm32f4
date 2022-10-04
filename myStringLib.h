#ifndef MYSTRINGLIB_H
#define MYSTRINGLIB_H
#include<stdint.h>
#include<string.h>
#include "my_stdLib.h"
int32_t indexOf(char*,char);
int32_t indexFrom(char*,char,int32_t);
void shiftBufferRight(char *str,int k);
void clearBuff(char *buff,uint32_t size);
int arryCmp(char *ch1,char *ch2,uint32_t len);
int strSearch(char *s1,char *s2,uint32_t len);
// helper function
void loadStrFromIndexWithSaperator(char *temp,int32_t *startIndex,int32_t *stopIndex,char *data_buf,char saperator);
void loadStrFromIndex(char *temp,int32_t startIndex,int32_t stopIndex,char *data_buf);
#endif
