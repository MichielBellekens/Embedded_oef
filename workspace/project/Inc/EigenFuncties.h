/*
 * EigenFuncties.h
 *
 *  Created on: 17-dec.-2016
 *      Author: Gebruiker
 */

#ifndef EIGENFUNCTIES_H_
#define EIGENFUNCTIES_H_

#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_ts.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "fatfs.h"
#include "ErrorMessage_data.h"

//linked list struct with the properties of a radiobutton
typedef struct RadioButtons
{
	char * pLabel;
	char* pCategory;
	uint32_t ulValue;
	uint32_t ulXpos;
	uint32_t ulYpos;
	uint32_t ulWidth;
	uint32_t ulHeight;
	bool isvisible;
	char* pButtonimage;
	void (*func)(struct RadioButtons*);
	struct RadioButtons * next;

} RadioButtons;

//DEDCLARE ALL THE VARIABLES --> NOT DEFINED HERE BUT ON AN EXTERNAL PLACE (EigenFunctie.h)
extern RadioButtons* pSpeed;			//linked radiobuttons list  for the pSpeed
extern RadioButtons* pBackground;	//linked radiobuttons list for the backgroundcolor
extern RadioButtons* pOptions[2];			//array of linked radiobuttons list --> easier to iterate through
extern volatile uint32_t ulPictureDelay;	//Variable for the delay between pictures --> default to 5000;

extern TS_StateTypeDef TouchState;	//variables that stores the touchstate
extern volatile bool InterruptActive;		//bool to see if interupt functionalitie has to be executed
extern uint32_t ulMainIterator;		//iterator to loop in delay while loop
extern const uint32_t ulInterruptDebounce;	//const value (can not be changed at runtime) to see when to re-activate the interrupt functionalities
extern FIL fp;						//file pointer to acces files from SD card
extern FIL imagefp;
extern FIL radiofp;
extern FIL optionsfp;
extern uint8_t ucBytesRead;			//uint8_t for bytesread value from f_read()
extern uint8_t ucImgBuffer[310000]; 		// buffer to store the image read
//uint8_t * ImgBuffer = (uint8_t*)0xC007F800;
extern uint8_t ucRadiobuff[3100];		//buffer to store the radiobuttons image read
extern FATFS FS;						//FATFS variable to use in f_mount
extern volatile uint32_t ulBackGroundColor; //variable that stores the currently selected background color
extern uint8_t ucXOffset[4];			//variabled to store the width of the read image --> needed to center image on screen
extern uint8_t ulYOffset[4];			//variabled to store the height of the read image --> needed to center image on screen
extern DIR Imagedir;					//Directory object --> needed for reading files from SD card directory
extern FILINFO fileinf;				//struct FILINFO variable that stores the file information while searching for file in dir

//function prototypes
void vEigFunInitProg(void);				//initialize screen, radios, ...
void vEigFunCheckButtons(void);			//check where on the touchscreen was pressed and what needs to be doen
void vEigFunReadBmpIntoBuffer(const char*);		//Read the bitmap with the name given as paramter into the ImgBuffer
void vEigFunReadRadioIntoBuffer(const char*);	//Read one of the radiobutton bitmaps into the Radiobuff
void vEigFunDrawBuffer(void);				//draw the imag from the ImgBuffer onto the screen
void vEigFunCreateRadioButtons(void);		//fill the needed linked lists and the arrat of linked lists
void vEigFunUnselectRadios(RadioButtons*);	//Set all the radiobuttons int he linked list pointed at by the given pointer to unselected
void vEigFunDrawMenu(void);				//draw the radiobuttons on the screen
void vEigFunToggleMenu(void);				//Toggle the menu between visible and invisible
void vEigFunSaveOptions(void);				//write the settings options into a file in the Config folder
void vEigFunReadOptions(void);				//read the settings options from the file
void vEigFunSetRadioButtons(void);			//set the radiobuttons in accordance to the settings
void vEigFunCheckAndMountSD(void);
void vEigFunErrorMsg(const char *);
#endif /* EIGENFUNCTIES_H_ */
