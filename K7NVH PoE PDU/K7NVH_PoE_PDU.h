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

// Adjust the pins used to correspond with the prototype V1.0 build
//#define TESTBOARD

// Enable ANSI color codes to be sent. Uses a small bit of extra program space for 
// storage of color codes/modified strings.
#define ENABLECOLORS

#define SOFTWARE_STR "\r\nK7NVH PoE PDU"
#define HARDWARE_VERS "1.1"
#define SOFTWARE_VERS "1.1"
#define PORT_CNT    12
#define INPUT_CNT	12
#define DATA_BUFF_LEN    32
#define ADC_AVG_POINTS   5

// SPI pins
#ifndef TESTBOARD

#define SPI_SS_1 PB0
#define SPI_SS_2 PB7
#define SPI_SCK PB1
#define SPI_MOSI PB2
#define SPI_MISO PB3

#else

#define SPI_SS_1 PF5
#define SPI_SS_2 PF4
#define SPI_SCK PB1
#define SPI_MOSI PB2
#define SPI_MISO PB0

#endif

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
#ifndef TESTBOARD

#define LED1 PF6
#define LED2 PF7

#else

#define LED1 PB3
#define LED2 PB7

#endif

// Limits
#define PCYCLE_MAX_TIME 30 // Seconds
#define VREF_MAX 4300 // 4.3V * 1000
#define VREF_MIN 4100 // 4.1V * 1000
#define VCAL_MAX 170 // 17.0x
#define VCAL_MIN 130 // 13.0x
#define LIMIT_MAX 100 // Stored as amps*10 so 50==5.0A
#define ICAL_MAX 520 // 52x
#define ICAL_MIN 480 // 48x
#define OFFSET_MAX 100 // Raw ADC counts
#define VMAX 50

// Timing
#define TICKS_PER_SECOND 4
#define VCTL_DELAY 20 // Ticks. ~5s
#define ICTL_DELAY 1 // Ticks. ~0.25s
#define IRST_DELAY 1200 // Ticks. ~5min

// EEPROM Offsets
// Stored settings
#define EEPROM_OFFSET_PORT_DEFAULTS 0 // 16 bytes at offset 0
#define EEPROM_OFFSET_CYCLE_TIME 16 // 1 byte at offset 17
// Calibration values
#define EEPROM_OFFSET_I_OFFSET 142 // 12 bytes - ADC counts of the current sense offset
#define EEPROM_OFFSET_REF_V 154 // 4 bytes - Calibrate the ADC reference voltage
#define EEPROM_OFFSET_V_CAL_MAIN 158 // 1 byte - Calibrate the main bus voltage sense divider
#define EEPROM_OFFSET_V_CAL_ALT 159 // 1 byte - Calibrate the alt bus voltage sense divider
#define EEPROM_OFFSET_SENSE_CAL 160 // 16 Bytes - Calibrate the 0 current sensor offset (in ADC counts)
#define EEPROM_OFFSET_I_CAL 176 // 32 Bytes - Calibrate the current sense gain figure
// Per port current thresholds
#define EEPROM_OFFSET_LIMIT 208 // 32 Bytes at offset 24
// Per port voltage thresholds
#define EEPROM_OFFSET_V_CUTOFF 240 // 32 Bytes
#define EEPROM_OFFSET_V_CUTON 272 // 32 Bytes
// Names
#define EEPROM_OFFSET_PDUNAME 304 // 16 bytes
#define EEPROM_OFFSET_P0NAME 320 // 16 bytes
#define EEPROM_OFFSET_P1NAME 336 // 16 bytes
#define EEPROM_OFFSET_P2NAME 352 // 16 bytes
#define EEPROM_OFFSET_P3NAME 368 // 16 bytes
#define EEPROM_OFFSET_P4NAME 384 // 16 bytes
#define EEPROM_OFFSET_P5NAME 400 // 16 bytes
#define EEPROM_OFFSET_P6NAME 416 // 16 bytes
#define EEPROM_OFFSET_P7NAME 432 // 16 bytes
#define EEPROM_OFFSET_P8NAME 448 // 16 Bytes
#define EEPROM_OFFSET_P9NAME 464 // 16 Bytes
#define EEPROM_OFFSET_P10NAME 480 // 16 Bytes
#define EEPROM_OFFSET_P11NAME 496 // 16 Bytes

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Globals
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Timer
volatile unsigned long timer = 0;
// Schedule
volatile uint8_t schedule_check_voltage = 0;
volatile uint8_t schedule_check_current = 0;
volatile uint8_t schedule_port_cycle = 0;
volatile uint8_t schedule_reset_current = 0;

// Port Set - bitmap of ports
typedef uint16_t pd_set;
// Port State Set - bitmap of port state
// (NUL,NUL,Locked?,AUX bus?,VCTL Changing,VCTL Enabled?,Overload,Enabled?)
typedef uint8_t ps_set;
// Port Boot State Set - bitmap of port boot state
// (NUL,NUL,NUL,NUL,Locked?,AUX bus?,VCTL Enabled?,Enabled?)
typedef uint8_t pbs_set;

// Port Cycle Tracking
pd_set cycle_ports;
volatile uint8_t cycle_timer = 0;

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
	const char STR_Locked[] PROGMEM = "\x1b[33mLOCKED\x1b[0m";
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
const char STR_MAIN[] PROGMEM = "MAIN";
const char STR_ALT[] PROGMEM = "ALT";
const char STR_OFFSET[] PROGMEM = "\r\nOFFSET: ";
const char STR_Port_Lock[] PROGMEM = "\r\nPORT LOCK ";

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
const char STR_Command_SETBUS[] PROGMEM = "SETBUS";
const char STR_Command_SETOFFSET[] PROGMEM = "SETOFFSET";
const char STR_Command_PLOCK[] PROGMEM = "PLOCK";

// Port to pin lookup table
const uint8_t Ports_Pins[PORT_CNT] = \
		{PD0, PD1, PD2, PD3, PD5, PD4, PD6, PD7, PB4, PB5, PB6, PC6};

// Port to ADC channel lookup table
const uint8_t Ports_ADC[PORT_CNT + 2] = \
		{0b10000000, 0b10010000, 0b10100000, 0b10110000, 0b11000000, 0b11010000, \
		 0b10000000, 0b10010000, 0b10100000, 0b10110000, 0b11000000, 0b11010000, \
		 0b11100000, 0b11110000};

// State Variables
ps_set PORT_STATE[PORT_CNT];
pbs_set PORT_BOOT_STATE[PORT_CNT];
char * DATA_IN;
uint8_t DATA_IN_POS = 0;
uint8_t PORT_HIGH_WATER[PORT_CNT];

uint8_t BOOT_RESET_VECTOR = 0;

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

// SPI
static inline void SPI_begin(void);
static inline uint8_t SPI_transfer(uint8_t _data);

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
static inline float EEPROM_Read_V_CAL_MAIN(void);
static inline void EEPROM_Write_V_CAL_MAIN(float div);
static inline float EEPROM_Read_V_CAL_ALT(void);
static inline void EEPROM_Write_V_CAL_ALT(float div);
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
static inline uint8_t EEPROM_Read_I_Offset(uint8_t port);
static inline void EEPROM_Write_I_Offset(uint8_t port, uint8_t offset);
static inline void EEPROM_Reset(void);

// DEBUG
static inline void DEBUG_Dump(void);

// ADC
static inline float ADC_Read_Port_Current(uint8_t port);
static inline float ADC_Read_Main_Voltage(void);
static inline float ADC_Read_Alt_Voltage(void);
static inline int16_t ADC_Read_Temperature(void);
static inline uint16_t ADC_Read_Raw(uint8_t adc);

// Output
static inline void printPGMStr(PGM_P s);
static inline void PRINT_Status(void);
static inline void PRINT_Status_Prog(void);
static inline void PRINT_Help(void);

// Input
static inline void INPUT_Clear(void);
static inline void INPUT_Parse(void);
static inline void INPUT_Parse_args(pd_set *pd, char *str);
static inline int8_t INPUT_Parse_port(void);

// Watchdog
static inline void Watchdog_Disable(void);
static inline void Watchdog_Enable(void);

#endif
