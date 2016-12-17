/*
 * EigenFuncties.c
 *
 *  Created on: 17-dec.-2016
 *      Author: Gebruiker
 */
#include "EigenFuncties.h"

//ALL THE VARIABLES DECLARED IN HEADER FILE ARE DEFINED HERE
RadioButtons* pSpeed = NULL;			//linked radiobuttons list  for the speed
RadioButtons* pBackground = NULL;	//linked radiobuttons list for the backgroundcolor
RadioButtons* pOptions[2];			//array of linked radiobuttons list --> easier to iterate through
uint32_t ulPictureDelay = 5000;	//Variable for the delay between pictures --> default to 5000;

TS_StateTypeDef TouchState;	//variables that stores the touchstate
FIL fp;						//file pointer to acces files from SD card
FIL imagefp;
FIL radiofp;
FIL optionsfp;
uint8_t ucBytesRead;			//uint8_t for bytesread value from f_read()
uint8_t ucImgBuffer[310000]; 		// buffer to store the image read
//uint8_t * ImgBuffer = (uint8_t*)0xC007F800;
uint8_t ucRadiobuff[3100];		//buffer to store the radiobuttons image read
FATFS FS;						//FATFS variable to use in f_mount
uint32_t ulBackGroundColor = LCD_COLOR_RED; //variable that stores the currently selected background color
uint8_t ucXOffset[4];			//variabled to store the width of the read image --> needed to center image on screen
uint8_t ulYOffset[4];			//variabled to store the height of the read image --> needed to center image on screen
DIR Imagedir;					//Directory object --> needed for reading files from SD card directory
FILINFO fileinf;				//struct FILINFO variable that stores the file information while searching for file in dir


void vEigFunInitProg(void)
{
	  BSP_LCD_Init();				//init the lcd
	  BSP_LCD_LayerDefaultInit(0,LCD_FB_START_ADDRESS);	//init layer 0
	  BSP_LCD_DisplayOn();			//turn the display on
	  BSP_LCD_SelectLayer(0);		//select layer 0
	  BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());	//init the touchscreen
	  BSP_LCD_SetFont(&Font16);		//select a font
	  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);	//Set the text color to blue
	  vEigFunCreateRadioButtons();		//fill all the linked lists and the options array

	  if(BSP_TS_ITConfig() != TS_OK)	//config the interrupt of the touchscreen
	  {
		  BSP_LCD_Clear(LCD_COLOR_RED);	//if interrupt config failed lcd red and infinite loop
	  }
	  //BSP_LCD_Clear(ulBackGroundColor);	//clears the screen to the background color
	  vEigFunCheckAndMountSD();
	  vEigFunReadOptions();	//call this function to read the settings from the SD card
	  vEigFunSetRadioButtons();	//set the radiobuttons to match the read options
	  BSP_LCD_Clear(ulBackGroundColor);	//clears the screen to the background color
	  return;
}

//set the radiobuttons in accordance to the settings
void vEigFunSetRadioButtons(void)
{
	for(uint32_t i =0; i < sizeof(pOptions)/sizeof(pOptions[0]);i++)	//iterate over the array of options
	{
		vEigFunUnselectRadios(pOptions[i]);	//unselect all the radiobuttons in the selected radiobuttons linked list
		RadioButtons * temp = pOptions[i];	//store the pointer to first element in a temp value
		while(temp != NULL)	//as long as temp isn't zero
		{
			if(strcmp(pOptions[i]->pCategory,"Color")  == 0)	//if the category of the currently select list is color --> use option[] and not temp beacause only the first node has a value for category
			{
				if(ulBackGroundColor == temp->ulValue)	//if the currently set backgroundcolor is the same as the value in the current node
				{
					temp->pButtonimage = "Config/Radiosel.bmp";	//set this node's image to the selected radiobutton
				}
			}
			else if(strcmp(pOptions[i]->pCategory,"Speed") == 0)	//same as aboce but with speed category
			{
				if(ulPictureDelay == temp->ulValue)
				{
					temp->pButtonimage = "Config/Radiosel.bmp";
				}
			}
			else
			{
				//In normal operation not accesible
			}
			temp = temp->next;		//change the radiobuttons pointer to the next node
		}
	}
	return;
}

//save the currently set options to a file in the Config folder
void vEigFunSaveOptions(void)
{
	if(f_open(&optionsfp, "Config/Options.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)	//open the settings file --> create new file (if exist overwrite) & write access
	{
		vEigFunErrorMsg("Open error saveoptions");
	}
	f_printf(&optionsfp, "%x,%x",ulBackGroundColor, ulPictureDelay);		//write the setting values to the file
	if(f_close(&optionsfp) != FR_OK)	//close the file
	{
		vEigFunErrorMsg("Close error saveoptions");
	}
	return;
}

//read the option from the file
void vEigFunReadOptions(void)
{
	uint8_t readbuffer[50];		//create a buffer to store the read values
	if(f_open(&fp, "Config/Options.txt", FA_READ) != FR_OK)		//open the config file in read mode
	{
		vEigFunErrorMsg("Open error readoptions");
	}
	if(f_read(&fp, readbuffer, sizeof(readbuffer), (UINT*)&ucBytesRead) != FR_OK)		//read the file into the buffer
	{
		vEigFunErrorMsg("Read error readoptions");
	}
	sscanf((const char*)readbuffer, "%x,%x", (unsigned int*)&ulBackGroundColor,(unsigned int*)&ulPictureDelay);	//read the formatted data from the buffer
	if(f_close(&fp) != FR_OK)	//close the file
	{
		vEigFunErrorMsg("Close error readoptions");
	}
	return;
}
//Fills the linked lists values and adds them to the Options array
void vEigFunCreateRadioButtons(void)
{
	uint32_t ulStdWidth = 32;
	uint32_t ulStdHeight = 32;
	uint32_t ulStdPadding = 5;
	uint32_t ulStdTitleoffset=16;
	pSpeed = malloc(sizeof(RadioButtons));
	pSpeed->pCategory = "Speed";
	pSpeed->pLabel = "5s";
	pSpeed->ulValue = 5000;
	pSpeed->ulXpos = ulStdPadding;
	pSpeed->ulYpos=ulStdPadding+ulStdTitleoffset;
	pSpeed->ulHeight = ulStdHeight;
	pSpeed->ulWidth = ulStdWidth;
	pSpeed->isvisible = false;
	pSpeed->pButtonimage = "Config/Radiosel.bmp";
	pSpeed->func = &vEigFunUnselectRadios;
	pSpeed->next = malloc(sizeof(RadioButtons));
	pSpeed->next->pLabel = "10s";
	pSpeed->next->ulValue = 10000;
	pSpeed->next->ulXpos = ulStdPadding;
	pSpeed->next->ulYpos=ulStdHeight+ulStdPadding + ulStdPadding+ulStdTitleoffset;
	pSpeed->next->ulHeight = ulStdHeight;
	pSpeed->next->ulWidth = ulStdWidth;
	pSpeed->next->isvisible = false;
	pSpeed->next->pButtonimage = "Config/Radiouns.bmp";
	pSpeed->next->func = &vEigFunUnselectRadios;
	pSpeed->next->next = malloc(sizeof(RadioButtons));
	pSpeed->next->next->pLabel = "30s";
	pSpeed->next->next->ulValue = 30000;
	pSpeed->next->next->ulXpos = ulStdPadding;
	pSpeed->next->next->ulYpos=2*(ulStdHeight+ulStdPadding)+ulStdPadding+ulStdTitleoffset;
	pSpeed->next->next->ulHeight = ulStdHeight;
	pSpeed->next->next->ulWidth = ulStdWidth;
	pSpeed->next->next->isvisible = false;
	pSpeed->next->next->pButtonimage = "Config/Radiouns.bmp";
	pSpeed->next->next->func = &vEigFunUnselectRadios;
	pSpeed->next->next->next = NULL;

	pOptions[0]=pSpeed;	//The first element of the array is a linked list of radiobuttons for the speed

	uint32_t ulExtraOffset = 130;
	pBackground = malloc(sizeof(RadioButtons));
	pBackground->pCategory = "Color";
	pBackground->pLabel = "Red";
	pBackground->ulValue = LCD_COLOR_RED;
	pBackground->ulXpos = ulStdPadding;
	pBackground->ulYpos=ulStdPadding+ulStdTitleoffset+ulExtraOffset;
	pBackground->ulHeight = ulStdHeight;
	pBackground->ulWidth = ulStdWidth;
	pBackground->isvisible = false;
	pBackground->pButtonimage = "Config/Radiosel.bmp";
	pBackground->func = &vEigFunUnselectRadios;
	pBackground->next = malloc(sizeof(RadioButtons));
	pBackground->next->pLabel = "Black";
	pBackground->next->ulValue = LCD_COLOR_BLACK;
	pBackground->next->ulXpos = ulStdPadding;
	pBackground->next->ulYpos=ulStdHeight+ulStdPadding + ulStdPadding+ulStdTitleoffset+ulExtraOffset;
	pBackground->next->ulHeight = ulStdHeight;
	pBackground->next->ulWidth = ulStdWidth;
	pBackground->next->isvisible = false;
	pBackground->next->pButtonimage = "Config/Radiouns.bmp";
	pBackground->next->func = &vEigFunUnselectRadios;
	pBackground->next->next = malloc(sizeof(RadioButtons));
	pBackground->next->next->pLabel = "White";
	pBackground->next->next->ulValue = LCD_COLOR_WHITE;
	pBackground->next->next->ulXpos = ulStdPadding;
	pBackground->next->next->ulYpos=2*(ulStdHeight+ulStdPadding)+ulStdPadding+ulStdTitleoffset+ulExtraOffset;
	pBackground->next->next->ulHeight = ulStdHeight;
	pBackground->next->next->ulWidth = ulStdWidth;
	pBackground->next->next->isvisible = false;
	pBackground->next->next->pButtonimage = "Config/Radiouns.bmp";
	pBackground->next->next->func = &vEigFunUnselectRadios;
	pBackground->next->next->next = NULL;

	pOptions[1]=pBackground;	//The second element of the array is a linked list of radiobuttons for the background color
	return;
}

//Draw the image from the buffer in the center of the screen
void vEigFunDrawBuffer(void)
{
	BSP_LCD_Clear(ulBackGroundColor);		//clear the screen to the current backgroundcolor
	uint32_t ulimgwidth = ucXOffset[3]<<24 | ucXOffset[2]<<16 | ucXOffset[1]<<8 | ucXOffset[0];	//create the uint32_t value image width from the x_offset array
	uint32_t ulimgheight = ulYOffset[3]<<24 | ulYOffset[2]<<16 | ulYOffset[1]<<8 | ulYOffset[0];	//create the uint32_t value image height from the y_offset array
	uint32_t ulx_offset  = (BSP_LCD_GetXSize()-ulimgwidth)/2;	//calculate x_offset needed to center the image on the screen
	uint32_t uly_offset  = (BSP_LCD_GetYSize()-ulimgheight)/2;	//calculate y_offset needed to center the image on the screen
	BSP_LCD_DrawBitmap(ulx_offset,uly_offset, (uint8_t*)ucImgBuffer);		//Print the image on screen with the correct offsets
	vEigFunDrawMenu();	//call the draw menu function (doesn't always draw the menu --> only if visible)
	return;
}

//loads every element of the given linked list pointer to unchecked
void vEigFunUnselectRadios(RadioButtons* clearing)
{
	RadioButtons* temp = clearing;	//temperary pointer to loop over all the elements --> set to first node
	while (temp != NULL)			//as long as temp isn't a NULL pointer
	{
		temp->pButtonimage = "Config/Radiouns.bmp";	//put the buttonimage property of the currently selected node to unchecked
		temp = temp->next;		//Let the temp pointer point to the next node

	}
	return;
}

//function called from the interrupt if touch detected --> checks what needs to be done
void vEigFunCheckButtons(void)
{
	if(!pOptions[0]->isvisible || TouchState.touchX[0] > 100)	//if the options menu isn't visible or the touch was not near the positions of the radio buttons
	{
		vEigFunToggleMenu();		//toggle the options menu visible to invisible and invisible to visible
	}
	else	//Else check if one of the radiobuttons was pressed
	{
		for(int i=0; i < sizeof(pOptions)/sizeof(pOptions[0]); i++)	//loop over all linked lists contained in the options array
		{
			RadioButtons *temp = pOptions[i];	//temporary RadioBUttons pointer to loop over all nodes of all the linked lists
			while(temp != NULL)	//As long as the currently selected node exists (not a null pointer
			{
				if(TouchState.touchX[0] > temp->ulXpos && TouchState.touchX[0] < (temp->ulXpos + temp->ulWidth))	//check if touch was inside x boundaries of current node
				{
					if(TouchState.touchY[0] > temp->ulYpos && TouchState.touchY[0] < (temp->ulYpos + temp->ulHeight))	//check if touch was inside x boundaries of current node
					{
						temp->func(pOptions[i]);		//call the function from the function pointer inside the node --> sets all the radiobuttons of the current linked list to unselected
						if(strcmp(pOptions[i]->pCategory,"Speed")==0)	//if the category of the current linked list is Speed
						{
							ulPictureDelay = temp->ulValue;	//set the picture delay to the value of the current node
						}
						else if(strcmp(pOptions[i]->pCategory,"Color")==0)	//if the category of the current linked list is Color
						{
							ulBackGroundColor = (uint32_t)temp->ulValue;		//set the backgroundcolor to the value of the current node
						}
						else	//if this loop is entered something went wrong
						{
							vEigFunErrorMsg("Something went wrong");
						}
						temp->pButtonimage = "Config/Radiosel.bmp";	//set the current node to active --> only one radiobutton per linked list can be active (rest is cleared by functionpointer function see above)
						vEigFunDrawMenu();	//call the draw menu function
						break;	//break from while since the touch was found, don't check the rest of the buttons
					}
				}
				temp = temp->next;	//select the next node of the current linked list
			}
		}
	}
	return;
}

//draw the options menu if it's visible
void vEigFunDrawMenu(void)
{
	if(pOptions[0]->isvisible)	//if one element is visible, they all are
	{
		int padding = 3;
		for(int i=0; i < sizeof(pOptions)/sizeof(pOptions[0]); i++)	//iterate over all the linked lists in the options array
		{
			BSP_LCD_DisplayStringAt(0,i*130,(uint8_t*)pOptions[i]->pCategory,LEFT_MODE);	//display the categroy as title --> second linked list title needs an offset so it doesn't overlap with the first
			RadioButtons * temp = pOptions[i];	//temp pointer (otherwise pointer to 1st node gets lost)
			while(temp != NULL)	//as long as there is a valid radiobutton element
			{
				vEigFunReadRadioIntoBuffer(temp->pButtonimage);		//read the image needed for this node
				BSP_LCD_DrawBitmap(temp->ulXpos, temp->ulYpos, (uint8_t*)ucRadiobuff);	//draw it at the correct position
				BSP_LCD_DisplayStringAt(temp->ulXpos+temp->ulWidth+padding, temp->ulYpos, (uint8_t*)temp->pLabel,LEFT_MODE);	//print the label next to the button
				temp = temp->next;	//select the next node
			}
		}
	}
	return;
}

//Set all the radiobuttons visible if they are invisible and invisible if they are visible
void vEigFunToggleMenu()
{
	for(int i=0; i < sizeof(pOptions)/sizeof(pOptions[0]); i++)	//iterate over all the linked lists
	{
		RadioButtons * temp = pOptions[i];	//temp pointer (otherwise pointer to 1st node gets lost)
		while(temp != NULL)		//as long as there are valid nodes
		{
			if(temp->isvisible)	//if the node is set to visible
			{
				temp->isvisible = false;	//set to invisible
			}
			else
			{
				temp->isvisible = true;		//else to visible
			}
			temp = temp->next;		//Select the next node
		}
	}
	vEigFunSaveOptions();
	vEigFunDrawBuffer();	//update the screen
	return;
}

//read the needed bitmap for a radiobutton.
void vEigFunReadRadioIntoBuffer(const char* filename)
{
	if(f_open(&radiofp, filename, FA_READ) != FR_OK)		//open the file with the given name in read mode
	{
		vEigFunErrorMsg("Open error ReadRadioIntoBuffer");
	}
	for(int i =0; i < sizeof(ucRadiobuff); i++)	//clear the buffer so we don't see parts of previous image
	{
		ucRadiobuff[i]=0;
	}
	if(f_read(&radiofp, ucRadiobuff, sizeof(ucRadiobuff), (UINT*)&ucBytesRead) != FR_OK)	//read the image from the file and store in the radiobuffer
	{
		vEigFunErrorMsg("Read error from ReadRadioIntoBuffer");
	}
	if(f_close(&radiofp) != FR_OK)	//close the file
	{
		vEigFunErrorMsg("Close error from ReadRadioIntoBuffer");
	}
	return;
}

//Read the bitmap to display on the screen from the file with the given name
void vEigFunReadBmpIntoBuffer(const char* filename)
{
    char* prefix = "Images/";	//The name of the dir containing all the available images
    char* result= malloc(strlen(prefix)+strlen(filename)+1);	//allocate memory for the path to the image (+1 for the nullbyte at the end)
    strcpy(result, prefix);		//copy the prefix to the result pointer
    strcat(result, filename);	//concatenate the filename to result
	for(int i =0; i < sizeof(ucImgBuffer); i++)	//clear the ImgBuffer --> don't want parts of prev image
	{
		ucImgBuffer[i]=0;
	}
	if(f_open(&imagefp, result, FA_READ) != FR_OK)	//open the file of the build path in read mode
	{
		vEigFunErrorMsg("Open error from ReadBMPIntoBuffer");
	}
	if(f_read(&imagefp, ucImgBuffer, sizeof(ucImgBuffer), (UINT*)&ucBytesRead) != FR_OK)	//read the image into the ImgBuffer
	{
		vEigFunErrorMsg("Read 1 error from ReadBMPIntoBuffer");
	}
	if(f_lseek(&imagefp,18) != FR_OK)	//set the pointer to position 12h to read the width of the bmp
	{
		vEigFunErrorMsg("Flseek 1 error from ReadBMPIntoBuffer");
	}
	if(f_read(&imagefp,ucXOffset, sizeof(ucXOffset), (UINT*)&ucBytesRead) != FR_OK)	//read the width and store in x_offset array
	{
		vEigFunErrorMsg("Read 2 error from ReadBMPIntoBuffer");
	}
	if(f_lseek(&imagefp,22) != FR_OK)	//set the pointer to position 16h to read the height of the bmp
	{
		vEigFunErrorMsg("Flseek 2 error from ReadBMPIntoBuffer");
	}
	if(f_read(&imagefp, ulYOffset, sizeof(ulYOffset), (UINT*)&ucBytesRead) != FR_OK)	//read the height and store in y_offset array
	{
		vEigFunErrorMsg("Read 3 error from ReadBMPIntoBuffer");
	}
	if(f_close(&imagefp) != FR_OK)		//close the file
	{
		vEigFunErrorMsg("Close error from ReadBMPIntoBuffer");
	}
	return;
}

void vEigFunCheckAndMountSD(void)
{
	while(!BSP_SD_IsDetected())	//check if there is an SD card inserted
	{
	  BSP_LCD_DisplayStringAt(0,0,(uint8_t *)"no sd card found", CENTER_MODE);	//while not print a message
	}
	if(f_mount(&FS, SD_Path, 1) != FR_OK)		//try to mount the SD card
	{
	  BSP_LCD_DisplayStringAtLine(0, (uint8_t *)"Error while mounting");	//if failed print message
	}
	return;
}


void vEigFunErrorMsg(const char * message)
{
	BSP_LCD_Clear(LCD_COLOR_YELLOW);
	BSP_LCD_DisplayStringAtLine(0, (uint8_t*) message);
	while(1)
	{
		//hanging since an error occured
	}
	return;
}
