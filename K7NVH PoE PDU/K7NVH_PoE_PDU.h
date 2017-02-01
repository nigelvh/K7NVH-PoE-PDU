/* (c) 2015 Nigel Vander Houwen */
#ifndef _K7NVH_PoE_PDU_H_
#define _K7NVH_PoE_PDU_H_

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Includes
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

#include "Descriptors.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Macros
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Enable ANSI color codes to be sent. Uses a small bit of extra program space for 
// storage of color codes/modified strings.
#define ENABLECOLORS

#define SOFTWARE_STR "\r\nK7NVH PoE PDU"
#define HARDWARE_VERS "1.0"
#define SOFTWARE_VERS "1.0"
#define PORT_CNT    12
#define INPUT_CNT	12
#define DATA_BUFF_LEN    32
#define ADC_AVG_POINTS   5

// Output port controls
#define P1EN PD0
#define P2EN PD1
#define P3EN PD2
#define P4EN PD3
#define P5EN PD5
#define P6EN PD4
#define P7EN PD6
#define P8EN PD7
#define P9EN PB4
#define P10EN PB5
#define P11EN PB6
#define P12EN PC6

// LED1 = Status (Left), LED2 = Error (Right)
#define LED1 PB3
#define LED2 PB7

// Limits
#define PCYCLE_MAX_TIME 30 // Seconds
#define VREF_MAX 4300 // 4.3V * 1000
#define VREF_MIN 4100 // 4.1V * 1000
#define VCAL_MAX 150 // 15.0
#define VCAL_MIN 70 // 7.0
#define LIMIT_MAX 100 // Stored as amps*10 so 50==5.0A
#define ICAL_MAX 160 // 160/10 = 16.0
#define ICAL_MIN 60 // 60/10 = 6.0
#define VMAX 40

// Timing
#define VCTL_DELAY 20 // Ticks. ~5s
#define ICTL_DELAY 1 // Ticks. ~0.25s

// EEPROM Offsets
#define EEPROM_OFFSET_PORT_DEFAULTS 0 // 12 bytes at offset 0
#define EEPROM_OFFSET_REF_V 12 // 4 bytes at offset 12
#define EEPROM_OFFSET_V_CAL 16 // 1 bytes at offset 16
#define EEPROM_OFFSET_CYCLE_TIME 17 // 1 byte at offset 17
// 18-23
#define EEPROM_OFFSET_LIMIT 24 // 8 Bytes at offset 24
#define EEPROM_OFFSET_I_CAL 32 // 8 Bytes at offset 32
// 40-47
#define EEPROM_OFFSET_PDUNAME 48 // 16 bytes at offset 48
#define EEPROM_OFFSET_P0NAME 64 // 16 bytes at offset 64
#define EEPROM_OFFSET_P1NAME 80 // 16 bytes at offset 80
#define EEPROM_OFFSET_P2NAME 96 // 16 bytes at offset 96
#define EEPROM_OFFSET_P3NAME 112 // 16 bytes at offset 112
#define EEPROM_OFFSET_P4NAME 128 // 16 bytes at offset 128
#define EEPROM_OFFSET_P5NAME 144 // 16 bytes at offset 144
#define EEPROM_OFFSET_P6NAME 160 // 16 bytes at offset 160
#define EEPROM_OFFSET_P7NAME 176 // 16 bytes at offset 176
#define EEPROM_OFFSET_P8NAME 192 // 16 Bytes at offset 192
#define EEPROM_OFFSET_P9NAME 208 // 16 Bytes at offset 208
#define EEPROM_OFFSET_P10NAME 224 // 16 Bytes at offset 224
#define EEPROM_OFFSET_P11NAME 240 // 16 Bytes at offset 240

#define EEPROM_OFFSET_V_CUTOFF 256 // 16 Bytes at offset 256
#define EEPROM_OFFSET_V_CUTON 272 // 16 Bytes at offset 272

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Globals
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Timer
volatile unsigned long timer = 0;
// Schedule
volatile uint8_t check_voltage = 0;
volatile uint8_t check_current = 0;

typedef uint16_t pd_set; // Port Descriptor Set - bitmap of ports

// Port State Set - bitmap of port state
// (NUL,NUL,NUL,NUL,VCTL Changing,VCTL Enabled?,Overload,Enabled?)
typedef uint8_t ps_set;
// Port Boot State Set - bitmap of port boot state
// (NUL,NUL,NUL,NUL,NUL,NUL,VCTL Enabled?,Enabled?)
typedef uint8_t pbs_set;

// Standard file stream for the CDC interface when set up, so that the
// virtual CDC COM port can be used like any regular character stream
// in the C APIs.
static FILE USBSerialStream;

// Help string
const char STR_Help_Info[] PROGMEM = "\r\nVisit https://github.com/nigelvh/K7NVH-PoE-PDU for full docs.";

// Reused strings
#ifdef ENABLECOLORS
//	const char STR_Color_Red[] PROGMEM = "\x1b[31m";
//	const char STR_Color_Green[] PROGMEM = "\x1b[32m";
//	const char STR_Color_Blue[] PROGMEM = "\x1b[34m";
//	const char STR_Color_Cyan[] PROGMEM = "\x1b[36m";
//	const char STR_Color_Reset[] PROGMEM = "\x1b[0m";
	const char STR_Unrecognized[] PROGMEM = "\r\n\x1b[31mINVALID COMMAND\x1b[0m";
	const char STR_Enabled[] PROGMEM = "\x1b[32mENABLED\x1b[0m";
	const char STR_Disabled[] PROGMEM = "\x1b[31mDISABLED\x1b[0m";
	const char STR_Overload[] PROGMEM = " \x1b[31m!OVERLOAD!\x1b[0m";
	const char STR_VCTL[] PROGMEM = " \x1b[36mVOLTAGE CTL\x1b[0m";
#else
	const char STR_Unrecognized[] PROGMEM = "\r\nINVALID COMMAND";
	const char STR_Enabled[] PROGMEM = "ENABLED";
	const char STR_Disabled[] PROGMEM = "DISABLED";
	const char STR_Overload[] PROGMEM = " !OVERLOAD!";
	const char STR_VCTL[] PROGMEM = " VOLTAGE CTL";
#endif	

const char STR_Backspace[] PROGMEM = "\x1b[D \x1b[D";
const char STR_NR_Port[] PROGMEM = "\r\nPORT ";
const char STR_Port_Default[] PROGMEM = "\r\nPORT DEFAULT ";
const char STR_PCYCLE_Time[] PROGMEM = "\r\nPCYCLE TIME: ";
const char STR_Port_Limit[] PROGMEM = "\r\nPORT LIMIT: ";
const char STR_Port_CutOff[] PROGMEM = "\r\nPORT CUTOFF: ";
const char STR_Port_CutOn[] PROGMEM = "\r\nPORT CUTON: ";
const char STR_Port_VCTL[] PROGMEM = "\r\nPORT VCTL: ";
const char STR_VREF[] PROGMEM = "\r\nVREF: ";
const char STR_VCAL[] PROGMEM = "\r\nVCAL: ";
const char STR_ICAL[] PROGMEM = "\r\nICAL: ";

// Command strings
const char STR_Command_HELP[] PROGMEM = "HELP";
const char STR_Command_STATUS[] PROGMEM = "STATUS";
const char STR_Command_PSTATUS[] PROGMEM = "PSTATUS";
const char STR_Command_DEBUG[] PROGMEM = "DEBUG";
const char STR_Command_PON[] PROGMEM = "PON";
const char STR_Command_POFF[] PROGMEM = "POFF";
const char STR_Command_PCYCLE[] PROGMEM = "PCYCLE";
const char STR_Command_SETCYCLE[] PROGMEM = "SETCYCLE";
const char STR_Command_SETDEF[] PROGMEM = "SETDEF";
const char STR_Command_SETVREF[] PROGMEM = "SETVREF";
const char STR_Command_SETVCAL[] PROGMEM = "SETVCAL";
const char STR_Command_SETICAL[] PROGMEM = "SETICAL";
const char STR_Command_SETNAME[] PROGMEM = "SETNAME";
const char STR_Command_SETLIMIT[] PROGMEM = "SETLIMIT";
const char STR_Command_VCTL[] PROGMEM = "VCTL";
const char STR_Command_SETVCTL[] PROGMEM = "SETVCTL";

// Port to pin lookup table
const uint8_t Ports_Pins[PORT_CNT] = \
		{PD0, PD1, PD2, PD3, PD5, PD4, PD6, PD7, PB4, PB5, PB6, PC6};

// Port to ADC Address look up table
// PORT 1,2,3,4,5,6,7,8
const uint8_t ADC_Ports[PORT_CNT] = \
		{0b10000000, 0b10010000, 0b10100000, 0b10110000, \
		 0b11000000, 0b11010000, 0b11100000, 0b11110000};
// Input to ADC Address look up table
// EXT1, EXT2, EXT3, EXT4, EXT5, EXT6, INPUT, TEMP
const uint8_t ADC_Inputs[INPUT_CNT] = \
		{0b10000000, 0b10010000, 0b10100000, 0b11010000, \
		 0b11100000, 0b11110000, 0b10110000, 0b11000000};

// State Variables
ps_set PORT_STATE[PORT_CNT];
pbs_set PORT_BOOT_STATE[PORT_CNT];
char DATA_IN[DATA_BUFF_LEN];
uint8_t DATA_IN_POS = 0;

/** LUFA CDC Class driver interface configuration and state information.
 * This structure is passed to all CDC Class driver functions, so that
 * multiple instances of the same class within a device can be
 * differentiated from one another.
 */ 
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface = {
	.Config = {
		.ControlInterfaceNumber   = INTERFACE_ID_CDC_CCI,
		.DataINEndpoint           = {
			.Address          = CDC_TX_EPADDR,
			.Size             = CDC_TXRX_EPSIZE,
			.Banks            = 1,
		},
		.DataOUTEndpoint = {
			.Address          = CDC_RX_EPADDR,
			.Size             = CDC_TXRX_EPSIZE,
			.Banks            = 1,
		},
		.NotificationEndpoint = {
			.Address          = CDC_NOTIFICATION_EPADDR,
			.Size             = CDC_NOTIFICATION_EPSIZE,
			.Banks            = 1,
		},
	},
};

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Prototypes
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Set up a fake function that points to a program address where the bootloader should be
// based on the part type.
#ifdef __AVR_ATmega32U4__
	void (*bootloader)(void) = 0x3800;
#endif

// USB
static inline void run_lufa(void);

// LED & Port Control
static inline void LED_CTL(uint8_t led, uint8_t state);
static inline void PORT_CTL(uint8_t port, uint8_t state);
static inline void PORT_Set_Ctl(pd_set *pd, uint8_t state);

// Check Limits
static inline void Check_Current_Limits(void);
static inline uint8_t PORT_Check_Current_Limit(uint8_t port);
static inline void Check_Voltage_Cutoff(void);

// EEPROM Read & Write
static inline uint8_t EEPROM_Read_Port_Boot_State(uint8_t port);
static inline void EEPROM_Write_Port_Boot_State(uint8_t port, uint8_t state);
static inline float EEPROM_Read_REF_V(void);
static inline void EEPROM_Write_REF_V(float reference);
static inline float EEPROM_Read_V_CAL(void);
static inline void EEPROM_Write_V_CAL(float div);
static inline float EEPROM_Read_I_CAL(uint8_t port);
static inline void EEPROM_Write_I_CAL(uint8_t port, float cal);
static inline uint8_t EEPROM_Read_PCycle_Time(void);
static inline void EEPROM_Write_PCycle_Time(uint8_t time);
static inline void EEPROM_Read_Port_Name(int8_t port, char *str);
static inline void EEPROM_Write_Port_Name(int8_t port, char *str);
static inline uint8_t EEPROM_Read_Port_Limit(uint8_t port);
static inline void EEPROM_Write_Port_Limit(uint8_t port, uint8_t limit);
static inline float EEPROM_Read_Port_CutOff(uint8_t port);
static inline void EEPROM_Write_Port_CutOff(uint8_t port, uint16_t cutoff);
static inline float EEPROM_Read_Port_CutOn(uint8_t port);
static inline void EEPROM_Write_Port_CutOn(uint8_t port, uint16_t cuton);
static inline void EEPROM_Reset(void);

// DEBUG
static inline void DEBUG_Dump(void);

// ADC
static inline float ADC_Read_Port_Current(uint8_t port);
static inline float ADC_Read_Input_Voltage(void);
static inline float ADC_Read_Temperature(void);
static inline float ADC_Read_Raw_Voltage(uint8_t port, uint8_t adc);
static inline uint16_t ADC_Read_Raw(uint8_t port, uint8_t adc);

// Output
static inline void printPGMStr(PGM_P s);
static inline void PRINT_Status(void);
static inline void PRINT_Status_Prog(void);
static inline void PRINT_Help(void);

// Input
static inline void INPUT_Clear(void);
static inline void INPUT_Parse(void);
static inline void INPUT_Parse_args(pd_set *pd, char *str);

#endif
