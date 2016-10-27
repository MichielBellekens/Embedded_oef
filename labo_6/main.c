#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include <limits.h>

volatile unsigned long counter = 0;

void sleep(unsigned long time)
{
	unsigned long tempvar = counter;
	//as long as the counter doesn't exceed the stored value + the wanted wait we are going to keep
	//running the while loop
	while(counter <= tempvar + time );
	return;
}

void SysTick_Handler(void)
{
	counter++;
	return;
}

void init()
{
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);
	SysTickPeriodSet(8000);
	SysTickEnable();
	SysTickIntEnable();
	return;
}

int main(void)
{
	init();
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	//Set the directions of pin F0,F2,F3 to output
	HWREG(0x40025000+ 0x400) = 13;
	//Digital enable of pin F0,F2,F3
	HWREG(0x40025000+ (0x51c)) = 13;
	while(1)
	{
		//Add mask of the address to make sure the necessary bits can be written
		//--> the value that is written is 1 pin 0 is set high --> the leds of the ethernet port are
		//active low
		HWREG(0x40025000 +(13<<2)) = 1;
		sleep(500);
		//Add mask of the address to make sure the necessary bits can be written
		HWREG(0x40025000 +(13<<2)) = 12;
		sleep(500);
	}
	return 0;
}

