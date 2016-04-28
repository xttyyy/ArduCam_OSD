/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Pro or Pro Mini w/ ATmega328 (5V, 16 MHz), Platform=avr, Package=arduino
*/

#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
#define __AVR_ATmega328p__
#define __AVR_ATmega328P__
#define F_CPU 16000000L
#define ARDUINO 10605
#define ARDUINO_AVR_PRO
#define ARDUINO_ARCH_AVR
#define __cplusplus 201103L
#define __AVR__
#define __inline__
#define __asm__(x)
#define __extension__
#define __inline__
#define __volatile__
#define GCC_VERSION 40801
#define volatile(va_arg) 
#define _CONST
#define __builtin_va_start
#define __builtin_va_end
#define __attribute__(x)
#define NOINLINE __attribute__((noinline))
#define prog_void
#define PGM_VOID_P int
#ifndef __builtin_constant_p
	#define __builtin_constant_p __attribute__((__const__))
#endif
#ifndef __builtin_strlen
	#define __builtin_strlen  __attribute__((__const__))
#endif
#define NEW_H
typedef void *__builtin_va_list;
typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {;}



#include <arduino.h>
#include <pins_arduino.h> 
#undef F
#define F(string_literal) ((const PROGMEM char *)(string_literal))
#undef PSTR
#define PSTR(string_literal) ((const PROGMEM char *)(string_literal))
#undef cli
#define cli()
#define pgm_read_byte(address_short)
#define pgm_read_word(address_short)
#define pgm_read_word2(address_short)
#define digitalPinToPort(P)
#define digitalPinToBitMask(P) 
#define digitalPinToTimer(P)
#define analogInPinToBit(P)
#define portOutputRegister(P)
#define portInputRegister(P)
#define portModeRegister(P)
#include <..\ArduCAM_OSD\ArduCAM_OSD.ino>
#include <..\ArduCAM_OSD\ArduCam_Max7456.cpp>
#include <..\ArduCAM_OSD\ArduCam_Max7456.h>
#include <..\ArduCAM_OSD\ArduNOTES.ino>
#include <..\ArduCAM_OSD\BOOT_Func.ino>
#include <..\ArduCAM_OSD\Font.ino>
#include <..\ArduCAM_OSD\MAVLink.ino>
#include <..\ArduCAM_OSD\OSD_Config.h>
#include <..\ArduCAM_OSD\OSD_Config_Func.ino>
#include <..\ArduCAM_OSD\OSD_Func.h>
#include <..\ArduCAM_OSD\OSD_Panels.ino>
#include <..\ArduCAM_OSD\OSD_Vars.h>
#include <..\ArduCAM_OSD\Spi.cpp>
#include <..\ArduCAM_OSD\Spi.h>
#include <..\ArduCAM_OSD\timer.h>
#endif
