#include<semihosting.h>
#include <stdio.h>
#define delay_time 700000
#define maxlen 50

int leesNString(char * output, int max )
{
	char karakter;
	int len = 0;
	while(len < max -1)
	{
		karakter  = SH_GetChar();
		switch (karakter)
		{
		case '\r': break;
		case '\n':
			output[len++] = '\0';
			return len;
		default:
			output[len++] = karakter;
		}
	}
	output[len++] = '\0';
	return len;
}

int main(void)
{
	volatile unsigned long ulLoop;
	char message[maxlen];

	while(1)
    {
		leesNString(message, maxlen);
		printf("%s\r\n", message);
    }
	return 0;
}
