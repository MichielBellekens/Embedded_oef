/*
* Hands-on 1 : blinky (GPIO)
* Doel : Kennismaken met de IDE en vervolgens met de GPIO van de LM3S6965
* Beginsituatie : Het programma zal een status ledje laten knipperen.
*
* Opdracht1 : Compileer dit programma, laad het in de controller en start het
* Opdracht2 : Bestudeer wat voor de tijdsvertraging zorgt.
* Is dit een goede manier? Waarom wel/niet?
* Opdracht3 : In dit voorbeeld worden de registers niet rechtstreeks aangesproken
* Kan je de betrokken registers en hun werking terug vinden in de datasheet?
* Opdracht4 : Pas het programma aan zodat je de registers gebruikt (tip HWREG)
* Opdracht5 : Pas het programma verder aan zodat ook de leds van de
* ethernet poort mee knipperen.
*/
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#define delay 150000

int main(void)
{
	//create a variable for the delays
	volatile unsigned long uLoop;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	//Set the directions of pin F0,F2,F3 to output
	HWREG(0x40025000+ 0x400) = 13;
	//Digital enable of pin F0,F2,F3
	HWREG(0x40025000+ (0x51c)) = 13;
	while(1)
	{
		for(uLoop = 0; uLoop < delay; uLoop++);
		//Add mask of the adress to make sure the necessary bits can be written
		HWREG(0x40025000 +(13<<2)) = 13;
		for(uLoop = 0; uLoop < delay; uLoop++);
		//Add mask of the adress to make sure the necessary bits can be written
		HWREG(0x40025000 +(13<<2)) = 0;
	}

}

/*int main(void)
{
// A VARIABLE USED TO KEEP THE PROGRAM BUSY __> CAUSES THE DELAY
volatile unsigned long ulLoop;

// Enable the GPIO port that is used for the on-board LED.
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//
// Enable the GPIO pin for the LED (PF0). Set the direction as output, and
// enable the GPIO pin for digital function.
//
GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
// Loop forever.
while(1)
{
//
// Delay some time
//
for(ulLoop = 0; ulLoop < delay; ulLoop++);
//
// Output high level
//
GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
//
// Delay some time
//
for(ulLoop = 0; ulLoop < delay; ulLoop++);
//
// Output low level
//
GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
}
}
*/
