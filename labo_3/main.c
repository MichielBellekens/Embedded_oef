/*
 *  Hands-on 2    : Pushbutton (GPIO)
 *  Doel          : Verder leren werken met GPIO (input)
 *  Beginsituatie : Het programma werkt niet. Het is een begin vanwaar je kan verder bouwen.
 *
 *	Opdracht1     : Bekijk op welke GPIO pinnen de drukknoppen staan
 *                  (Up, Down, Left, Right, Select)
 *	Opdracht2     : Configureer de betreffende GPIO pinnen als input.
 *                  Let wel op !!!
 *					    1. Enable eerst de poort.
 *  				    2. Kijk na of de pin "speciaal" moet worden ingesteld.
 *  				       Zoek bijvoorbeeld in het schema naar een weerstand.
 *  Opdracht3     : Maak het programma zo dat:
 *                      1. als je op een knop drukt de status led laat branden
 *                      2. als je de knop terug loslaat de status led niet brandt
 *  Opdracht4     : Maak het programma zo dat de drukknoppen de status led laten toggelen.
 *                      1. als je op de knop drukt en terug los laat => led brand
 *                      2. als je dan terug drukt en terug los laat => led gaat uit
 *                  Hou hierbij rekening met dender (aka Bounce).
 */

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#define delay 150000

int main(void)
{
	volatile unsigned long ulLoop;

	/* Enable hier al de GPIO poorten die je wil gebruiken (zie SysCtlPeripheralEnable)*/
	//Poort van de drukknoppen
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//poort van de leds
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	/* Configureer hier de pinnen ( input, output, ...) --> input/output */
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 |GPIO_PIN_2 |GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
	//configure the pullups for the switches
	GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 |GPIO_PIN_2 |GPIO_PIN_3, GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPU); //status switch
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD); //status led

//OPDRACHT 4
    while(1)
    {
		//Schrijf hier je eigenlijke programma afhankelijk van opdr 3 of 4
    	//Indien de knop laag is (actief laag) wordt de waarde van de led geinverteerd
    	if(!GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_0)|!GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_1)|!GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_2)|!GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_3))
    	{
    		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0, ~GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0));
    	}
    	for(ulLoop = 0; ulLoop < delay; ulLoop++);

    }

/* OPDRACHT 3
    while(1)
    {
		//Schrijf hier je eigenlijke programma afhankelijk van opdr 3 of 4
    	if(!GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_0)|!GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_1)|!GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_2)|!GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_3))
    	{
    		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0, 1);
    	}
    	else
    	{
    		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0,0);
    	}

    	//for(ulLoop = 0; ulLoop < delay; ulLoop++);

    }*/
}

