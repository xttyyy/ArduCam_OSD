/*

Copyright (c) 2011.  All rights reserved.
An Open Source Arduino based OSD and Camera Control project.

Program  : ArduCAM-OSD (MinimOSD [and variants] Firmware)
Version  : V2.2, May 8th 2014
Author(s): Sandro Benigno
Coauthor(s):
Jani Hirvinen   (All the EEPROM routines)
Michael Oborne  (OSD Configutator)
Z贸ltan G谩bor, Pedro Santos and MinimOSD-Extra Team (Extra OSD Tools/Panels)
Mike Smith      (BetterStream and Fast Serial libraries)
Special Contribuitor:
Andrew Tridgell by all the support on MAVLink
Doug Weibel by his great orientation since the start of this project
Contributors: James Goppert, Max Levine
and all other members of DIY Drones Dev team
Thanks to: Chris Anderson and Jordi Munoz


This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>

*/

/* ************************************************************ */
/* **************** MAIN PROGRAM - MODULES ******************** */
/* ************************************************************ */

#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 


/* **********************************************/
/* ***************** INCLUDES *******************/

//#define membug 
//#define FORCEINIT  // You should never use this unless you know what you are doing 

// AVR Includes
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <math.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "wiring.h"
#endif
#include <EEPROM.h>
#include <SimpleTimer.h>
#include <GCS_MAVLink.h>

#ifdef membug
#include <MemoryFree.h>
#endif

// Configurations
#include "OSD_Config.h"
#include "ArduCam_Max7456.h"
#include "OSD_Vars.h"
#include "OSD_Func.h"

/* *************************************************/
/* ***************** DEFINITIONS *******************/

//OSD Hardware 
//#define ArduCAM328
#define MinimOSD

#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port
#define BOOTTIME         2000   // Time in milliseconds that we show boot loading bar and wait user input

// Objects and Serial definitions
FastSerialPort0(Serial);
OSD osd; //OSD object 

SimpleTimer  mavlinkTimer;

/* **********************************************/
/* ***************** SETUP() *******************/

void setup() 
{
#ifdef ArduCAM328
    pinMode(10, OUTPUT); // USB ArduCam Only
#endif
    pinMode(MAX7456_SELECT,  OUTPUT); // OSD CS

    Serial.begin(TELEMETRY_SPEED);
    // setup mavlink port
    mavlink_comm_0_port = &Serial;

#ifdef membug
    Serial.println(freeMem());
#endif

    // Prepare OSD for displaying 
    unplugSlaves();
    osd.init();

    // Start 
    startPanels();
    delay(500);

    // OSD debug for development (Shown at start)
#ifdef membug
    osd.setPanel(1,1);
    osd.openPanel();
    osd.printf("%i",freeMem()); 
    osd.closePanel();
#endif

    // Check EEPROM to for a new version that needs EEPROM reset
    if(readEEPROM(CHK_VERSION) != VER) {
        osd.setPanel(3,9);
        osd.openPanel();
        osd.printf_P(PSTR("EEPROM mapping outdated!|Update with the OSD Tool.")); 
        osd.closePanel();
        // run for ever until EEPROM version is OK 
        for(;;) {}
    }

    // Get correct panel settings from EEPROM
    readSettings();
    readPanelSettings();
    //panel = 0; //set panel to 0 to start in the first navigation screen
    // Show bootloader bar
    loadBar();

    // Startup MAVLink timers  
    mavlinkTimer.Set(&OnMavlinkTimer, 120);

    // House cleaning, clear display and enable timers
    osd.clear();
    mavlinkTimer.Enable();


} // END of setup();



/* ***********************************************/
/* ***************** MAIN LOOP *******************/

// Mother of all happenings, The loop()
// As simple as possible.
void loop() 
{

    if(enable_mav_request == 1){//Request rate control
        osd.clear();
        osd.setPanel(3,10);
        osd.openPanel();
        osd.printf_P(PSTR("Requesting DataStreams...")); 
        osd.closePanel();
        for(int n = 0; n < 3; n++){
            request_mavlink_rates();//Three times to certify it will be readed
            delay(50);
        }
        enable_mav_request = 0;
        delay(2000);
        osd.clear();
        waitingMAVBeats = 0;
        lastMAVBeat = millis();//Preventing error from delay sensing
    }


	
    read_mavlink();
    mavlinkTimer.Run();
}

/* *********************************************** */
/* ******** functions used in main loop() ******** */
void OnMavlinkTimer()
{
    setHeadingPatern();  // generate the heading patern

    //  osd_battery_pic_A = setBatteryPic(osd_battery_remaining_A);     // battery A remmaning picture
    //osd_battery_pic_B = setBatteryPic(osd_battery_remaining_B);     // battery B remmaning picture

    setHomeVars(osd);   // calculate and set Distance from home and Direction to home

    writePanels();       // writing enabled panels (check OSD_Panels Tab)
}
int minUnit[100];

int values[100];
//char* menus[100] = {
//	"AHRS Ctrl Set",
//	"init sensor",
//	"cal compass",
//	"max roll",
//	"max pitch",
//	"elevon P",
//	"roll P",
//	"7",
//	"8",
//	"9",
//	
//	
//		"Plane Set",
//		"Plane mode",
//		"elevon inverse",
//		"pitch inverse",
//		"rudder inverse",
//		"Vtail mode",
//		"elevon mode",
//		"7",
//		"8",
//		"9",
//	
//	 "FlightMode Set",
//	"mode1",
//	"mode2",
//	"mode3",
//	"mode4",
//	"mode5",
//	"mode6",
//	"mode7",
//	"mode8",
//	"mode9",
//	
//		 "Rtl Set",
//		"Rtl Height",
//		"Rtl Thr Value",
//		"Rtl min Thr",
//		"Rtl max Thr",
//		"5",
//		"6",
//		"7",
//		"8",
//		"9",
//
//		"Osd Set", 
//		"dddd",
//		"2",
//		"3",
//		"4",
//		"5",
//		"6",
//		"7",
//		"8",
//		"9",
//
//		"0",
//		"1",
//		"2",
//		"3",
//		"4",
//		"5",
//		"6",
//		"7",
//		"8",
//		"9",
//
//		"0",
//		"1",
//		"2",
//		"3",
//		"4",
//		"5",
//		"6",
//		"7",
//		"8",
//		"9",
//
//		"0",
//		"1",
//		"2",
//		"3",
//		"4",
//		"5",
//		"6",
//		"7",
//		"8",
//		"9",
//
//		"0",
//		"1",
//		"2",
//		"3",
//		"4",
//		"5",
//		"6",
//		"7",
//		"8",
//		"9",
//
//		"0",
//		"1",
//		"2",
//		"3",
//		"4",
//		"5",
//		"6",
//		"7",
//		"8",
//		"9",
//		
//		
//};

char* mmmm = "asdfasdfsadfdsafasdfasfsadfaskdjfksadjflkajsdfkljsadkfljasdf|asdfasdfasdf|asdfasdfasdf|asdfasdfasdf|asdfasdfsdafasdfasdfffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
unsigned long c_last_chan1_move_time = 0;
char menuDeeps = 1;
char rowDeeps = -1;
char parrent = -1;
int currentValue = 0;
int newValue = 0;
void onRCInput()
{
	
	if (c_getchan1middle == 0 && c_chan1_middle == 10000 && chan1_raw>1200)
	{
		c_chan1_middle = chan1_raw;
		c_getchan1middle = 1;
		return;
	}
	if (c_getchan2middle == 0 && c_chan2_middle == 10000 && chan2_raw>1200)
	{
		c_chan2_middle = chan2_raw;
		c_getchan2middle = 1;
		return;
	}
	if (c_getchan1middle == 0 || c_getchan2middle == 0)
		return;
	if (c_configuring == 0)
	{
		if (chan1_raw - c_chan1_middle > 300 || c_chan1_middle - chan1_raw > 300)
		{
			unsigned long ms = millis();
			if (c_last_chan1_move_time == 0)
			{
				c_last_chan1_move_time = ms;
				return;
			}
			else if (ms - c_last_chan1_move_time >5000)
			{
				if (c_chan1_middle - chan1_raw > 300)
					c_chan1_rev = -1;
				parrent = -1;
				c_configuring = 1;
				osd.clear();
				/*
				c_last_chan1_move_time = 0;
				menuDeeps = 0;
				rowDeeps = 0;*/
			}
		}
	}
	else
	{
		int16_t xshift = (chan1_raw - c_chan1_middle)* c_chan1_rev;
		if (xshift > 300)
		{
			menuDeeps++;
			if (menuDeeps == 1)
			{
				parrent = rowDeeps;
				rowDeeps = 0;
			}
			else if (menuDeeps == 2)
			{
				////load(parrent,rowDeeps)
				//currentValue = values[parrent][rowDeeps];
				//newValue = currentValue;
			}
			else if (menuDeeps >= 3)
			{
				if (parrent!=-1)
					newValue = currentValue;
				menuDeeps = 2;
			}
		}
		if (xshift < -300)
		{
			if (menuDeeps == 2)
			{
				//save(menuDeeps,rowDeeps)
			}
			if (menuDeeps == 1)
			{
				parrent = -1;
				rowDeeps = 0;
			}
			menuDeeps--;
		}
		
		if (menuDeeps == -1)
		{
			menuDeeps = 0;
			rowDeeps = 0;
			c_configuring = 0;
			osd.clear();
		}


		int16_t yshift = (chan2_raw - c_chan2_middle)* c_chan2_rev;
		if (menuDeeps<2)
		{
			if (yshift > 300)
			{
				rowDeeps++;
			}
			if (yshift < -300)
			{
				rowDeeps--;
			}
			if (rowDeeps <= 0)
			{
				rowDeeps = 0;
			}
			if (rowDeeps >= 9)
			{
				rowDeeps = 9;
			}
		}
		else
		{
			if (yshift > 300)
			{
				newValue += 1;// minUnit[parrent][rowDeeps];
			}
			if (yshift < -300)
			{
				newValue -= 1;// minUnit[parrent][rowDeeps];
			}
		}
	}
}
void unplugSlaves(){
    //Unplug list of SPI
#ifdef ArduCAM328
    digitalWrite(10,  HIGH); // unplug USB HOST: ArduCam Only
#endif
    digitalWrite(MAX7456_SELECT,  HIGH); // unplug OSD
}
char* myStrings2[15] = {
	"AHRS Ctrl Set",
"init sensor",
"cal compass",
"max roll",
"max pitch",
	"Plane Set",
	"Plane mode",
	"elevon inverse",
	"pitch inverse",
	"rudder inverse",
	"Plane Set",
	"Plane mode",
	"elevon inverse",
	"pitch inverse",
	"rudder inverse" };

