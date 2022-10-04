#include <my_stdLib.h>
#include <math.h>
#include <string.h>
// Stack overflow
// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}
// Conver a int 'x' into a char array str[] with d digits (this is a base 10 function)
int intToStr(int x, char str[], int d)
{
    int i = 0;
    int flage_negative=0;
    // if the number is zero, special case
    if(x==0)
    {
    	int n=0;
    	for(n=0;n<d;n++)
    	{
    		str[n] = '0';
    	}
    	str[n] = '\0';
    	return 1;
    }
    else if(x<0) // negative number
    {
    	x = (-1)*x;//turn it to possitive
    	flage_negative=1;
    }
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    if(flage_negative==1) // if negative number
    {
    	str[i++]='-';
    }
    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}
// Converts a floating point number to string.
// n=float number, res pointer to the char array, atfterpoint = the number of digits after the decimal point
void ftoa(float n, char *res, int afterpoint)
{
	int flage_negative = 0;
	if(n<0) // negative
	{
		n = n*(-1);
		flage_negative=1;
	}
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;
    if(flage_negative==1)
    {
    	ipart = (-1)*ipart;
    }
    // convert integer part to string
    int i = intToStr(ipart, res, 1);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

int8_t charToInt(char ch)
{
    return ch-'0';
}

int32_t strToInt(char *str)
{
    int32_t len;
    int32_t i;
    int32_t j;
    int32_t rval=0;
    len = strlen(str);
    for(i=0,j=len-1;i<len;i++,j--)
    {
        //printf("%c %d\r\n",str[j],i);
        rval += charToInt(str[j])*pow(10,i);
    }
    return rval;
}

double strToFloat(char *str)
{
    uint32_t point,len;
    char pValue[10];
    char iValue[10];
    len = strlen(str);
    if(len == 0 )
    {
        return 0.0;
    }
    for(uint32_t i=0;i<len;i++)
    {
        point = i;
        if(str[i]=='.')
        {
            iValue[i] = 0;
            break;
        }
        else
        {
            iValue[i] = str[i];
        }
    }
    iValue[len] = 0;
    for(int i=0;i<len-point;i++)
    {
        pValue[i] = str[i+point+1];
    }
    pValue[len-point] = 0;
    int d,f;
    double ans;
    d = strToInt(iValue);
    f = strToInt(pValue);
    ans = d + f/pow(10,len-point-1);
    return ans;
}
