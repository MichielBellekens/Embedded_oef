#include <stdio.h>
#include<hw_types.h>
#include <uart.h>
#include<gpio.h>
#include <hw_memmap.h>
#define delay_time 700000
#define maxlen 50

int leesNString(char * output, int max )
{
	char karakter;
	int len = 0;
	while(len < max -1)
	{
		karakter  = UARTCharGet(UART0_BASE);
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
	SysCtlClockSet(SYSCTL_USE_PLL | SYSCTL_SYSDIV_1 | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	GPIOPinTypeUART(GPIO_PORTA_BASE, 0x03);
	UARTConfigSetExpClk(UART0_BASE,SysCtlClockGet(),9600,UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE);
	UARTEnable(UART0_BASE);

	volatile unsigned long ulLoop;
	char message[maxlen];

	while(1)
    {
		//leesNString(message, maxlen);
		//printf("%s\r\n", message);
		printf("Hello world\r\n");
		for(ulLoop=0; ulLoop<10000; ulLoop++);
    }
	return 0;
}
