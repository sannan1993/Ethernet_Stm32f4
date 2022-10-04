#include "myStringLib.h"

// return the index of character
int32_t indexOf(char *str, char ch)
{
	return indexFrom(str,ch,0);
}
// find the first occureence starting from position sp
int32_t indexFrom(char *str,char ch,int32_t sp)
{
	int32_t len;
	int32_t index;
	int found=0;
	len = strlen(str);
	for(index=sp;index<len;index++)
	{
		if(str[index] == ch) // found character
		{
			found=1; // found the character
			break;
		}
	}
	if(found == 0)
	{
		index = -1;
	}
	return index;
}
// shit the buffer to right
// [1,4,5,6] ->2->[5,6,0,0]
void shiftBufferRight(char *str,int k)
{
    int len=strlen(str);
    if(k>0) // if k is non zero
    {
		for(int i=0;i<len-k;i++)
		{
			str[i]=str[i+k];
		}
		str[len-k]=0;
	}
}

void clearBuff(char *buff,uint32_t size)
{
	for(int i=0;i<size;i++)
	{
		buff[i] = 0;
	}
}

int arryCmp(char *ch1,char *ch2,uint32_t len)
{
	for(int i=0;i<len;i++)
	{
		if(ch1[i] != ch2[i])
		{
			return 1; // fail
		}
	}
	return 0; // ok
}
// strSearch((char*)str,"\x01Write_Error",str_len) // usage
// return the index of the string match other wise return -1
int strSearch(char *s1,char *s2,uint32_t len)
{
	uint32_t s2_len;
	int fundCount=0;
	//char temp[10];
	int rvalue=-1;
	s2_len = strlen(s2);
	if(len>=s2_len)
	{
		for(int i=0;i<=len-s2_len;i++)
		{
		    fundCount=0;
			for(int j=0;j<s2_len;j++)
			{
				if(s1[j+i]==s2[j])
				{
					fundCount++;
				}
				else
				{
				    fundCount=0;
				}
			}
			if(fundCount==s2_len)
			{
				rvalue=i;
				break;
			}
		}
	}
	return rvalue;
}


void loadStrFromIndexWithSaperator(char *temp,int32_t *startIndex,int32_t *stopIndex,char *data_buf,char saperator)
{
	*stopIndex = indexFrom(data_buf, saperator, *startIndex);
	if (*stopIndex > 0)
	{
		loadStrFromIndex(temp,*startIndex,*stopIndex,data_buf);
		*startIndex = *stopIndex + 1; //move forwared
	}
}
void loadStrFromIndex(char *temp,int32_t startIndex,int32_t stopIndex,char *data_buf)
{
	for (int i = startIndex; i < stopIndex; i++)
		temp[i - startIndex] = data_buf[i];
	temp[stopIndex - startIndex] = 0; // add null
}
