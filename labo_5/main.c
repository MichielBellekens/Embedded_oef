//*****************************************************************************
//
// graphics.c - Simple Graphics Display Example
//
// Copyright (c) 2006-2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 9453 of the EK-LM3S6965 Firmware Package.
//
//*****************************************************************************

#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "drivers/rit128x96x4.h"
#include "lisa1_data.h"
#include "lisa2_data.h"
#include "lisa3_data.h"
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Graphics Example (graphics)</h1>
//!
//! A simple application that displays scrolling text on the top line of the
//! OLED display, along with a 4-bit gray scale image.
//
//*****************************************************************************

//*****************************************************************************
//
// The size of the header on the bitmap image. For a typical 4bpp Windows
// bitmap this is 0x76 bytes comprising the total number of bytes in the
// bitmap file header, the bitmap info header and the palette.
//
//*****************************************************************************
#define BITMAP_HEADER_SIZE   0x76

//*****************************************************************************
//
// The byte offsets into the image at which we can find the height and width.
// Each of these values are encoded in 4 bytes.
//
//*****************************************************************************
#define BITMAP_WIDTH_OFFSET  0x12
#define BITMAP_HEIGHT_OFFSET 0x16

//*****************************************************************************
//
// Simple 4-bit gray scale image for test.  This image was dumped directly from
// a 16 color Windows-format bitmap (BMP) file. The size must be
// (BITMAP_HEADER_SIZE + ((width /2) * height)) bytes.
//
//*****************************************************************************


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//*****************************************************************************
//
// Display scrolling text plus graphics on the OLED display.
//
//*****************************************************************************
int
main(void)
{
    volatile int iDelay;
    static char pucHello[] =
    {
        "                      "
        "how're you doing?"
        "                      "
    };

    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_8MHZ);

    //
    // Initialize the OLED display.
    //
    RIT128x96x4Init(1000000);


    //
    // Simple scrolling text display
    //
    while(1)
    {
    	for (unsigned long ulCol = 0; ulCol <= 53; ulCol++)
    	{
            //
            // Display the text.
            //
            RIT128x96x4StringDraw(&pucHello[ulCol++], 0, 0, 11);

            //
            // Delay for a bit.
            //
            for(iDelay = 0; iDelay < 100000; iDelay++)
            {
            }
    	}


        RIT128x96x4ImageDraw(Lisa_1, 0,0,128,96);
        for(iDelay = 0; iDelay < 100000; iDelay++);
        RIT128x96x4ImageDraw(Lisa_2, 0,0,128,96);
        for(iDelay = 0; iDelay < 100000; iDelay++);
        RIT128x96x4ImageDraw(Lisa_3, 0,0,128,96);
        for(iDelay = 0; iDelay < 100000; iDelay++);
        RIT128x96x4ImageDraw(Lisa_2, 0,0,128,96);
        for(iDelay = 0; iDelay < 100000; iDelay++);
        RIT128x96x4ImageDraw(Lisa_1, 0,0,128,96);
        for(iDelay = 0; iDelay < 100000; iDelay++);
    }
}
