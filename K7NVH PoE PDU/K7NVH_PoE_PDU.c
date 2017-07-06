/* (c) 2017 Nigel Vander Houwen */
//
// TODO
// Port locking?
// Watchdog - Problem exists where reset dumps into DFU, and waits there.
// Overload current time adjustable?

#include "K7NVH_PoE_PDU.h"

// Main Scheduling Interrupt
ISR(TIMER1_COMPA_vect){
	timer++;

	if ((timer) % VCTL_DELAY == 0){ schedule_check_voltage = 1; }
	if ((timer) % ICTL_DELAY == 0){ schedule_check_current = 1; }
	if (cycle_ports > 0) {
		cycle_timer--;
		if (cycle_timer == 0) {
			schedule_port_cycle = 1;
		}
	}
}

// Main program entry point.
int main(void) {
	// Initialize some variables
	int16_t BYTE_IN = -1;
	DATA_IN = malloc(DATA_BUFF_LEN);
	for(uint8_t i = 0; i < PORT_CNT; i++){
		PORT_HIGH_WATER[i] = 0;
	}
	
	// Set the watchdog timer to interrupt for timekeeping
//	MCUSR &= ~(1 << WDRF);
//	WDTCSR |= 0b00011000; // Set WDCE and WDE to enable WDT changes
//	WDTCSR = 0b01000011; // Enable watchdog interrupt and Interrupt at 0.25s

	// Set up timer 1 for 0.25s interrupts
	TCCR1A = 0b00000000; // No pin changes on compare match
	TCCR1B = 0b00001010; // Clear timer on compare match, clock /8
	TCCR1C = 0b00000000; // No forced output compare
	OCR1A = 31250; // Set timer clear at this count value
	TCNT1 = 0;
	TIMSK1 = 0b00000010; // Enable interrupts on the A compare match

	// Divide 16MHz crystal down to 1MHz for CPU clock.
	clock_prescale_set(clock_div_16);

	// Init USB hardware and create a regular character stream for the
	// USB interface so that it can be used with the stdio.h functions
	USB_Init();
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

	// Enable interrupts
	GlobalInterruptEnable();

	run_lufa();

	// Wait 5 seconds so that we can open a console to catch startup messages
	for (uint16_t i = 0; i < 500; i++) {
		_delay_ms(10);
	}

	// Print startup message
	printPGMStr(PSTR(SOFTWARE_STR));
	fprintf(&USBSerialStream, " V%s,%s", HARDWARE_VERS, SOFTWARE_VERS);
	run_lufa();

	// Set up LED pins
#ifndef TESTBOARD
	DDRF |= (1 << LED1)|(1 << LED2);
#else
	DDRB |= (1 << LED1)|(1 << LED2);
#endif

	// Set up SPI
	SPI_begin();

	// Enable the ADC
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // Enable ADC, clocked by /128 divider

	// Port control pins are currently inputs, set what we want them to be, then set them as outputs
	// This avoids a blip turning ports off before re-enabling.
	// Read in stored port on/off states, and turn them on/off to match
	for (uint8_t i = 0; i < PORT_CNT; i++) {
		PORT_BOOT_STATE[i] = EEPROM_Read_Port_Boot_State(i);
		if (PORT_BOOT_STATE[i] & 0b00000001) { PORT_CTL(i, 1); } else { PORT_CTL(i, 0); } // Enable port if set
		if (PORT_BOOT_STATE[i] & 0b00000010) { PORT_STATE[i] |= 0b00000100; } // Enable VCTL if set
		if (PORT_BOOT_STATE[i] & 0b00000100) { PORT_STATE[i] |= 0b00010000; } // Port is on AUX bus (used for power calculations)
	}
	// Set up control pins
	DDRD |= (1 << P1EN)|(1 << P2EN)|(1 << P3EN)|(1 << P4EN)|(1 << P5EN)|(1 << P6EN)|(1 << P7EN)|(1 << P8EN);
	DDRB |= (1 << P9EN)|(1 << P10EN)|(1 << P11EN);
	DDRC |= (1 << P12EN);
	
	run_lufa();

	INPUT_Clear();

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Main system loop
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	for (;;) {
		// Read a byte from the USB serial stream
		BYTE_IN = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

		// USB Serial stream will return <0 if no bytes are available.
		if (BYTE_IN >= 0) {
			// We've gotten a char, so lets blink the status LED onboard. This LED will 
			// remain lit while the board is processing commands. This is most evident 
			// during a PCYCLE.
			LED_CTL(0, 1);
			
			// Echo the char we just received back out the serial stream so the user's 
			// console will display it.
			fputc(BYTE_IN, &USBSerialStream);

			// Switch on the input byte to determine what is is and what to do.
			switch (BYTE_IN) {
				case 8:
				case 127:
					// Handle Backspace chars.
					if (DATA_IN_POS > 0){
						DATA_IN_POS--;
						DATA_IN[DATA_IN_POS] = 0;
						printPGMStr(STR_Backspace);
					}
					break;

				case '\n':
				case '\r':
					// Newline, Parse our command
					INPUT_Parse();
					INPUT_Clear();
					break;

				case 3:
					// Ctrl-c bail out on partial command
					INPUT_Clear();
					break;
				
				case 29:
					// Ctrl-] reset all eeprom values
					EEPROM_Reset();
					INPUT_Clear();
					break;
					
				case 30:
					// Ctrl-^ jump into the bootloader
					TIMSK0 = 0b00000000; // Interrupt will mess with the bootloader
					TIMSK1 = 0b00000000; 
					bootloader();
					break; // We should never get here...

				default:
					// Normal char buffering
					if (DATA_IN_POS < (DATA_BUFF_LEN - 1)) {
						DATA_IN[DATA_IN_POS] = BYTE_IN;
						DATA_IN_POS++;
						DATA_IN[DATA_IN_POS] = 0;
					} else {
						// Input is too long.
						printPGMStr(STR_Unrecognized);
						INPUT_Clear();
					}
					break;
			}
		}

		// Turn the status LED back off again
		LED_CTL(0, 0);
		
		// Check for above threshold current usage
		if (schedule_check_current) {
			Check_Current_Limits();
			schedule_check_current = 0;
		}
		
		// Timer interval, check voltage control
		if (schedule_check_voltage) {
			Check_Voltage_Cutoff();
			schedule_check_voltage = 0;
		}
		
		// Handle port cycles
		if (schedule_port_cycle) {
			PORT_Set_Ctl(&cycle_ports, 1);
			
			cycle_ports = 0;
			cycle_timer = 0;
			schedule_port_cycle = 0;
			
			INPUT_Clear();
		}
		
		// Keep the LUFA USB stuff fed regularly.
		run_lufa();
	}
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Command Parsing Functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Flush out our data input buffer, reset our position variable, and print a new prompt.
static inline void INPUT_Clear(void) {
	memset(&DATA_IN[0], 0, sizeof(DATA_IN));
	DATA_IN_POS = 0;
	
	// Read PDU name
	char temp_name[16];
	EEPROM_Read_Port_Name(-1, temp_name);
	
#ifdef ENABLECOLORS
	fprintf(&USBSerialStream, "\r\n\r\n#\x1b[32m%s \x1b[36m>\x1b[0m ", temp_name);
#else
	fprintf(&USBSerialStream, "\r\n\r\n#%s > ", temp_name);
#endif
}

// Parse command arguments and return pd_set bitmap with relevant port
// bits set.
static inline void INPUT_Parse_args(pd_set *pd, char *str) {
	*pd = 0;
	uint8_t temp = 0;
	
	while (*str != 0 && str < (DATA_IN + DATA_BUFF_LEN)) {
		if (*str == 'A' || *str == 'a') {
			*pd = 0b1111111111111111;
		} else if (*str >= '0' && *str <= '9') {
			if (temp > 0) temp = temp * 10;
			temp = temp + (*str - '0');
		} else if (temp >= 1 && temp <= 12) {
			*pd = *pd | (1 << (temp - 1));
			temp = 0;
		}
		str++;
	}
	if (temp >= 1 && temp <= 12) {
		*pd = *pd | (1 << (temp - 1));
	}
}

// Parse out a single port number argument
static inline int8_t INPUT_Parse_port() {
	int8_t temp = 0;
	
	while (*DATA_IN != 0) {
		if (*DATA_IN == 'P' || *DATA_IN == 'p') { DATA_IN++; return 'P'; }
		if (*DATA_IN >= '0' && *DATA_IN <= '9') {
			if (temp > 0) temp = temp * 10;
			temp = temp + (*DATA_IN - '0');
		} else if (temp > 0 && temp <= 12) {
			return (temp);
		}
		DATA_IN++;
	}
	if (temp > 0 && temp <= 12) {
		return (temp);
	}
	
	return -1;
}

// We've gotten a new command, parse out what they want.
static inline void INPUT_Parse(void) {
	pd_set pd; // Port descriptor bitmap

	// HELP - Print a basic help menu
	if (strncasecmp_P(DATA_IN, STR_Command_HELP, 4) == 0) {
		PRINT_Help();
		return;
	}
	// STATUS - Print a port status summary for all ports
	if (strncasecmp_P(DATA_IN, STR_Command_STATUS, 6) == 0) {
		PRINT_Status();
		return;
	}
	// PSTATUS - Print a status summary in a parser friendly output
	if (strncasecmp_P(DATA_IN, STR_Command_PSTATUS, 7) == 0) {
		PRINT_Status_Prog();
		return;
	}
	// DEBUG - Print a report of debugging information, including EEPROM variables
	if (strncasecmp_P(DATA_IN, STR_Command_DEBUG, 10) == 0) {
		DEBUG_Dump();
		return;
	}
	// PON - Turn on a port or list of ports
	if (strncasecmp_P(DATA_IN, STR_Command_PON, 3) == 0) {
		DATA_IN += 3;
		INPUT_Parse_args(&pd, DATA_IN);
		PORT_Set_Ctl(&pd, 1);
		return;
	}
	// POFF - Turn off a port or a list of ports
	if (strncasecmp_P(DATA_IN, STR_Command_POFF, 4) == 0) {
		DATA_IN += 4;
		INPUT_Parse_args(&pd, DATA_IN);
		PORT_Set_Ctl(&pd, 0);
		return;
	}
	// PCYCLE - Power cycle a port or list of ports. Time is defined by EEPROM_Read_PCycle_Time().
	if (strncasecmp_P(DATA_IN, STR_Command_PCYCLE, 6) == 0) {
		DATA_IN += 6;
		INPUT_Parse_args(&pd, DATA_IN);
		
		// If a port cycle is already in progress, error out.
		if (cycle_ports > 0) {
			printPGMStr(STR_Unrecognized);
		} else {
			PORT_Set_Ctl(&pd, 0);
			cycle_ports = pd;
			cycle_timer = EEPROM_Read_PCycle_Time() * TICKS_PER_SECOND;
		}
		return;
	}
	// SETCYCLE - Set PCYCLE_TIME and store in EEPROM
	if (strncasecmp_P(DATA_IN, STR_Command_SETCYCLE, 8) == 0) {
		DATA_IN += 8;
		uint16_t temp_set_time = atoi(DATA_IN);
		if (temp_set_time <= PCYCLE_MAX_TIME) {
			printPGMStr(STR_PCYCLE_Time);
			fprintf(&USBSerialStream, "%i", temp_set_time);
			EEPROM_Write_PCycle_Time((uint8_t)temp_set_time);
			return;
		}
	}
	// SETDEF - Set the port default state
	if (strncasecmp_P(DATA_IN, STR_Command_SETDEF, 6) == 0) {
		DATA_IN += 6;
		uint8_t state = 255;
		if (strncasecmp_P(DATA_IN, PSTR("ON"), 2) == 0) {
			state = 1;
			DATA_IN += 2;
		}
		if (strncasecmp_P(DATA_IN, PSTR("OFF"), 3) == 0) {
			state = 0;
			DATA_IN += 3;
		}
		if (state <= 1) {
			INPUT_Parse_args(&pd, DATA_IN);
			for (uint8_t i = 0; i < PORT_CNT; i++) {
				if (pd & (1 << i)) {
					if (state == 1) {
						PORT_BOOT_STATE[i] |= 0b00000001;
					}else{
						PORT_BOOT_STATE[i] &= 0b11111110;
					}
					EEPROM_Write_Port_Boot_State(i, PORT_BOOT_STATE[i]);
				
					printPGMStr(STR_Port_Default);
					fprintf(&USBSerialStream, "%i ", i+1);
					printPGMStr(state ? STR_Enabled : STR_Disabled);
				}
			}
			return;
		}
	}
	// VCTL - Enable/Disable Voltage Control
	if (strncasecmp_P(DATA_IN, STR_Command_VCTL, 4) == 0) {
		DATA_IN += 4;
		uint8_t setting = 255;
		if (strncasecmp_P(DATA_IN, PSTR("ON"), 2) == 0) {
			setting = 1;
			DATA_IN += 2;
		}
		if (strncasecmp_P(DATA_IN, PSTR("OFF"), 3) == 0) {
			setting = 0;
			DATA_IN += 3;
		}
		if (setting <= 1) {
			INPUT_Parse_args(&pd, DATA_IN);
			for (uint8_t i = 0; i < PORT_CNT; i++) {
				if (pd & (1 << i)) {
					if (setting == 1) {
						PORT_BOOT_STATE[i] |= 0b00000010;
						PORT_STATE[i] |= 0b00000100;
					}else{
						PORT_BOOT_STATE[i] &= 0b11111101;
						PORT_STATE[i] &= 0b11111011;
					}
					EEPROM_Write_Port_Boot_State(i, PORT_BOOT_STATE[i]);
					
					fprintf(&USBSerialStream, "\r\n");
					printPGMStr(STR_Command_VCTL);
					fprintf(&USBSerialStream, " %i ", i+1);
					printPGMStr(setting ? STR_Enabled : STR_Disabled);
				}
			}
			return;
		}
	}
	// SETVCTL - Set ON/OFF Voltages for Voltage Control
	if (strncasecmp_P(DATA_IN, STR_Command_SETVCTL, 7) == 0) {
		DATA_IN += 7;
		int8_t portid;
		uint8_t setting = 255;
		if (strncasecmp_P(DATA_IN, PSTR("ON"), 2) == 0) {
			setting = 1;
			DATA_IN += 2;
		}
		if (strncasecmp_P(DATA_IN, PSTR("OFF"), 3) == 0) {
			setting = 0;
			DATA_IN += 3;
		}
		if (setting <= 1) {
			portid = INPUT_Parse_port();
			while (*DATA_IN == ' ' || *DATA_IN == '\t') DATA_IN++;
			if (portid > 0 && portid <= PORT_CNT) {
				uint16_t temp_set_voltage = atoi(DATA_IN);
				if ((float)(temp_set_voltage/100) <= VMAX){
					if (setting == 1) {
						EEPROM_Write_Port_CutOn((portid - 1), temp_set_voltage);
					} else {
						EEPROM_Write_Port_CutOff((portid - 1), temp_set_voltage);
					}
					printPGMStr(STR_Port_VCTL);
					fprintf(&USBSerialStream, "%.2fV", (float)temp_set_voltage/100);
					return;
				}
			}
		}
	}
	// SETNAME - Set the name for a given port.
	if (strncasecmp_P(DATA_IN, STR_Command_SETNAME, 7) == 0) {
		int8_t portid;
		char temp_name[16];

		DATA_IN += 7;
		portid = INPUT_Parse_port();		
		while (*DATA_IN == ' ' || *DATA_IN == '\t') DATA_IN++;
		
		if (portid > 0 && portid <= PORT_CNT) {
			EEPROM_Write_Port_Name(portid - 1, DATA_IN);
			printPGMStr(STR_NR_Port);
			EEPROM_Read_Port_Name(portid - 1, temp_name);
			fprintf(&USBSerialStream, "%i NAME: %s", portid, temp_name);
			return;
		} else if (portid == 'P') {
			EEPROM_Write_Port_Name(-1, DATA_IN);
			return;
		}
	}
	// SETLIMIT - Set the current limit for a given port.
	if (strncasecmp_P(DATA_IN, STR_Command_SETLIMIT, 8) == 0) {
		DATA_IN += 8;
		uint8_t portid;
		
		portid = INPUT_Parse_port();
		while (*DATA_IN == ' ' || *DATA_IN == '\t') DATA_IN++;
		
		if (portid > 0 && portid <= PORT_CNT) {
			uint16_t temp_set_limit = (atoi(DATA_IN) / 100);
			if (temp_set_limit <= LIMIT_MAX){
				EEPROM_Write_Port_Limit((portid - 1), temp_set_limit);
				printPGMStr(STR_Port_Limit);
				fprintf(&USBSerialStream, "%.1fA", (float)temp_set_limit/10);
				return;
			}
		}
	}
	// SETVREF - Set the VREF voltage and store in EEPROM to correct voltage readings.
	if (strncasecmp_P(DATA_IN, STR_Command_SETVREF, 7) == 0) {
		DATA_IN += 7;
		uint16_t temp_set_vref = atoi(DATA_IN);
		if (temp_set_vref >= VREF_MIN && temp_set_vref <= VREF_MAX){
			float temp_vref = (float)temp_set_vref / 1000.0;
			EEPROM_Write_REF_V(temp_vref);
			printPGMStr(STR_VREF);
			fprintf(&USBSerialStream, "%.3fV", temp_vref);
			return;
		}
	}
	// SETVCAL - Set the ALT bus VCAL and store in EEPROM to correct voltage readings.
	if (strncasecmp_P(DATA_IN, STR_Command_SETVCAL, 7) == 0) {
		DATA_IN += 7;
		uint8_t setting = 255;
		if (strncasecmp_P(DATA_IN, STR_MAIN, 4) == 0) {
			setting = 0;
			DATA_IN += 4;
		}
		if (strncasecmp_P(DATA_IN, STR_ALT, 3) == 0) {
			setting = 1;
			DATA_IN += 3;
		}
		if (setting <= 1) {
			uint16_t temp_set_vdiv = atoi(DATA_IN);
			if (temp_set_vdiv >= VCAL_MIN && temp_set_vdiv <= VCAL_MAX){
				float temp_vdiv = (float)temp_set_vdiv / 10.0;
				if (setting == 0) {
					EEPROM_Write_V_CAL_MAIN(temp_vdiv);
				} else {
					EEPROM_Write_V_CAL_ALT(temp_vdiv);
				}
				printPGMStr(STR_VCAL);
				fprintf(&USBSerialStream, "%.1f", temp_vdiv);
				return;
			}
		}
	}
	// SETICAL - Set the current calibration for a given port and store in EEPROM to 
	// correct current readings.
	if (strncasecmp_P(DATA_IN, STR_Command_SETICAL, 7) == 0) {
		DATA_IN += 7;
		uint8_t portid;
		
		portid = INPUT_Parse_port();
		while (*DATA_IN == ' ' || *DATA_IN == '\t') DATA_IN++;
		
		if (portid > 0 && portid <= PORT_CNT) {
			uint16_t temp_i_cal = atoi(DATA_IN);
			if (temp_i_cal >= ICAL_MIN && temp_i_cal <= ICAL_MAX){
				EEPROM_Write_I_CAL((portid - 1), (float)(temp_i_cal / 10.0));
				printPGMStr(STR_ICAL);
				fprintf(&USBSerialStream, "%.1f", (float)(temp_i_cal / 10.0));
				return;
			}
		}
	}
	// SETBUS - Set ports to be referenced to the Main bus to correct power readings and voltage control
	if (strncasecmp_P(DATA_IN, STR_Command_SETBUS, 6) == 0) {
		DATA_IN += 6;
		uint8_t setting = 255;
		if (strncasecmp_P(DATA_IN, STR_MAIN, 4) == 0) {
			setting = 0;
			DATA_IN += 4;
		}
		if (strncasecmp_P(DATA_IN, STR_ALT, 3) == 0) {
			setting = 1;
			DATA_IN += 3;
		}
		if (setting <= 1) {
			INPUT_Parse_args(&pd, DATA_IN);
			for (uint8_t i = 0; i < PORT_CNT; i++) {
				if (pd & (1 << i)) {
					if (setting == 1) {
						PORT_BOOT_STATE[i] |= 0b00000100;
						PORT_STATE[i] |= 0b00010000;
					} else {
						PORT_BOOT_STATE[i] &= 0b11111011;
						PORT_STATE[i] &= 0b11101111;
					}
					EEPROM_Write_Port_Boot_State(i, PORT_BOOT_STATE[i]);
					
					fprintf(&USBSerialStream, "\r\n");
					printPGMStr(STR_Command_SETBUS);
					fprintf(&USBSerialStream, " %i ", i+1);
					printPGMStr(setting ? STR_ALT : STR_MAIN);
				}
			}
			return;
		}
	}
	// SETOFFSET - Store the ADC count offset for the current sensor, port should be on and disconnected.
	if (strncasecmp_P(DATA_IN, STR_Command_SETOFFSET, 9) == 0) {
		DATA_IN += 9;
		
		uint8_t portid = INPUT_Parse_port();
		while (*DATA_IN == ' ' || *DATA_IN == '\t') DATA_IN++;
		
		if (portid > 0 && portid <= PORT_CNT) {
			uint8_t temp_offset = atoi(DATA_IN);
			if (temp_offset <= OFFSET_MAX) {
				EEPROM_Write_I_Offset((portid - 1), temp_offset);
				printPGMStr(STR_OFFSET);
				fprintf(&USBSerialStream, "%i", temp_offset);
				return;
			}
		}
	}
	
	// If none of the above commands were recognized, print a generic error.
	printPGMStr(STR_Unrecognized);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Printing Functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Print a summary of all ports status'
static inline void PRINT_Status(void) {
	float main_voltage, alt_voltage, current, power;
	
	// Voltage
	main_voltage = ADC_Read_Main_Voltage();
	printPGMStr(PSTR("\r\nMain Voltage: "));
	fprintf(&USBSerialStream, "%.2fV", main_voltage);
	alt_voltage = ADC_Read_Alt_Voltage();
	printPGMStr(PSTR("\r\nAlt Voltage: "));
	fprintf(&USBSerialStream, "%.2fV", alt_voltage);
		
	// Temperature
	printPGMStr(PSTR("\tTemperature: "));
	fprintf(&USBSerialStream, "%dC", ADC_Read_Temperature());
	
	// Ports
	for(uint8_t i = 0; i < PORT_CNT; i++) {
		// PORT
		printPGMStr(STR_NR_Port);
		
		// Number and Name
		char temp_name[16];
		EEPROM_Read_Port_Name(i, temp_name);
		fprintf(&USBSerialStream, "%i\t\"%s\":\t", i+1, temp_name);
		
		// Enabled/Disabled
		if (PORT_STATE[i] & 0b00000001) { printPGMStr(STR_Enabled); } else { printPGMStr(STR_Disabled); }
		
		// Current reading
		current = ADC_Read_Port_Current(i);
		printPGMStr(PSTR("\t\tCurrent: "));
		fprintf(&USBSerialStream, "%.2fA", current);
		// Power reading
		printPGMStr(PSTR("\tPower: "));
		if (PORT_STATE[i] & 0b00010000) {
			power = alt_voltage * current;
		} else {
			power = main_voltage * current;
		}
		fprintf(&USBSerialStream, "%.1fW (", power);
		if (PORT_STATE[i] & 0b00010000) {
			printPGMStr(STR_ALT);
		} else {
			printPGMStr(STR_MAIN);
		}
		fprintf(&USBSerialStream, " BUS)\t");
		
		// Overload?
		if (PORT_STATE[i] & 0b00000010) { printPGMStr(STR_Overload); }
		
		// Voltage Control?
		if (PORT_STATE[i] & 0b00000100) { printPGMStr(STR_VCTL); }
	}
}

// Print programmatical status output
static inline void PRINT_Status_Prog(void){
	char temp_name[16];
	float main_voltage = ADC_Read_Main_Voltage();
	float alt_voltage = ADC_Read_Alt_Voltage();
	float power;
	
	// Device Description,Software version,Unit Name
	EEPROM_Read_Port_Name(-1, temp_name); //PDU Name
	printPGMStr(PSTR(SOFTWARE_STR));
	fprintf(&USBSerialStream, ",%s,%s", SOFTWARE_VERS, temp_name);
	
	// Input Voltage,Temperature
	fprintf(&USBSerialStream, "\r\n%.2f,%.2f,%d", main_voltage, alt_voltage, ADC_Read_Temperature());
	
	// Port Number,Port Name,Enabled?,Current,Power,Overload,AltBus?
	for (uint8_t i = 0; i < PORT_CNT; i++) {
		EEPROM_Read_Port_Name(i, temp_name);
		
		uint8_t port_state = (PORT_STATE[i] & 0b00000001);
		uint8_t port_overload = (PORT_STATE[i] & 0b00000010) >> 1;
		uint8_t port_vctl = (PORT_STATE[i] & 0b00000100) >> 2;
		uint8_t port_altbus = (PORT_STATE[i] & 0b00010000) >> 4;
		
		float current = ADC_Read_Port_Current(i);
		if (PORT_STATE[i] & 0b00010000) {
			power = alt_voltage * current;
		} else {
			power = main_voltage * current;
		}
		
		fprintf(&USBSerialStream, "\r\n%i,%s,%i,%.2f,%.1f,%i,%i,%i", i+1, temp_name, port_state, \
			current, power, port_overload, port_vctl, port_altbus);
	}
}

// Print a quick help command
static inline void PRINT_Help(void) {
	printPGMStr(STR_Help_Info);
}

// Print a PGM stored string
static inline void printPGMStr(PGM_P s) {
	char c;
	while((c = pgm_read_byte(s++)) != 0) fputc(c, &USBSerialStream);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Port/LED Control Functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Set all ports in a port descriptor set to a state
static inline void PORT_Set_Ctl(pd_set *pd, uint8_t state) {
	for (uint8_t i = 0; i < PORT_CNT; i++) {
		if (*pd & (1 << i)) {
			PORT_CTL(i, state);
			
			// If a port is controlled manually, make sure that voltage control gets disabled
			// Does not disable voltage control settings stored in EEPROM
			PORT_STATE[i] &= 0b11111011;
		}
	}
}

// Turn a port ON (state == 1) or OFF (state == 0)
static inline void PORT_CTL(uint8_t port, uint8_t state) {
	printPGMStr(STR_NR_Port);
	fprintf(&USBSerialStream, "%i ", port+1);

	if (state == 1) {
		if (port <= 7) {
			PORTD |= (1 << Ports_Pins[port]);
		} else if (port <= 10) {
			PORTB |= (1 << Ports_Pins[port]);
		} else if (port <= 11) {
			PORTC |= (1 << Ports_Pins[port]);
		}
		printPGMStr(STR_Enabled);
		PORT_STATE[port] |= 0b00000001;
	} else {
		if (port <= 7) {
			PORTD &= ~(1 << Ports_Pins[port]);
		} else if (port <= 10) {
			PORTB &= ~(1 << Ports_Pins[port]);
		} else if (port <= 11) {
			PORTC &= ~(1 << Ports_Pins[port]);
		}
		printPGMStr(STR_Disabled);
		PORT_STATE[port] &= 0b11111110;
	}
	
	// Clear any overload marks since we're manually setting this port
	PORT_STATE[port] &= 0b11111101;
}

// Turn a LED ON (state == 1) or OFF (state == 0)
// LED 0 == Green, LED 1 == Red
static inline void LED_CTL(uint8_t led, uint8_t state) {
#ifndef TESTBOARD
	if (state == 1) {
		if (led == 1) {
			PORTF |= (1 << LED2);
		} else {
			PORTF |= (1 << LED1);
		}
	} else {
		if (led == 1) {
			PORTF &= ~(1 << LED2);
		} else {
			PORTF &= ~(1 << LED1);
		}
	}
#else
	if (state == 1) {
		if (led == 1) {
			PORTB |= (1 << LED2);
		} else {
			PORTB |= (1 << LED1);
		}
	} else {
		if (led == 1) {
			PORTB &= ~(1 << LED2);
		} else {
			PORTB &= ~(1 << LED1);
		}
	}
#endif
}

// Check all ports for exceeding current limits, disable the port, and set the RED led.
static inline void Check_Current_Limits(void){
	// Check for above threshold current usage
	for (uint8_t i = 0; i < PORT_CNT; i++) {
		if (PORT_Check_Current_Limit(i)) {
			// If this port has already overloaded, don't repeat the message.
			if ((PORT_STATE[i] & 0x02) > 0) break;
		
			// Current is above threshold. Print a warning message.
			fprintf(&USBSerialStream, "\r\n");
			printPGMStr(STR_Overload);
						
			// Disable the port
			PORT_CTL(i, 0);
			
			// Mark the overload bit for this port
			PORT_STATE[i] |= 0b00000010;
				
			// If a port overloads, make sure that voltage control gets disabled
			// Does not disable voltage control settings stored in EEPROM
			PORT_STATE[i] &= 0b11111011;
				
			// Turn the error LED on.
			LED_CTL(1, 1);
				
			INPUT_Clear();
		}
	}
		
	// If no ports are listed as overload anymore, disable the error led.
	uint8_t error = 0;
	for (uint8_t i = 0; i < PORT_CNT; i++) {
		if ((PORT_STATE[i] & 0x02) > 0) error++;
	}
	if (error == 0) LED_CTL(1, 0);
}

// Checks a port against the stored current limit. Returns 0 if below limits, and 1 if
// the port has exceeded current limits.
static inline uint8_t PORT_Check_Current_Limit(uint8_t port){
	// Check for above threshold current flow, and return 1.
	if (ADC_Read_Port_Current(port) > ((float)EEPROM_Read_Port_Limit(port) / 10.0)) { return 1; }
	
	// Else return 0;
	return 0;
}

// Checks the disable voltage setting for each port and disables the port if it has fallen 
// below the cutoff threshold, and re-enables if it is above the cuton threshold.
static inline void Check_Voltage_Cutoff(void){
	float voltage;
	
	for (uint8_t i = 0; i < PORT_CNT; i++) {
		if ((PORT_STATE[i] & 0b00000100) > 0) {
			// Check the voltage as appropriate for the bus this port is on.
			if (PORT_STATE[i] & 0b00010000) {
				voltage = ADC_Read_Alt_Voltage();
			} else {
				voltage = ADC_Read_Main_Voltage();
			}
			if ((voltage < EEPROM_Read_Port_CutOff(i) && (PORT_STATE[i] & 0b00000001)) || (voltage > EEPROM_Read_Port_CutOn(i) && !(PORT_STATE[i] & 0b00000001))) {
				// Check if this is the second time through, if so, disable the port.
				// If it's the first time through, set the VCTL changing bit
				if ((PORT_STATE[i] & 0b00001000) > 0) {
					// If it's below the cutoff, turn the port off
					// If it's above the cuton, turn the port on
					if (voltage < EEPROM_Read_Port_CutOff(i)) {
						// Disable the port
						PORT_CTL(i, 0);
					}
					if (voltage > EEPROM_Read_Port_CutOn(i)) {
						// Enable the port
						PORT_CTL(i, 1);
					}
					
					// Clear the VCTL changing bit
					PORT_STATE[i] &= 0b11110111;
					
					INPUT_Clear();
				} else {
					PORT_STATE[i] |= 0b00001000;
				}
			// If voltage is above thresholds, reset any VCTL changing bit that was applied.
			} else {
				// Clear the VCTL changing bit
				PORT_STATE[i] &= 0b11110111;
			}
		}
	}
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ EEPROM Functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 

// Read the default port state setting
static inline uint8_t EEPROM_Read_Port_Boot_State(uint8_t port) {
	uint8_t state = eeprom_read_byte((uint8_t*)(EEPROM_OFFSET_PORT_DEFAULTS + port));
	if (state == 255) state = 1;
	return state;
}
// Write the default port state setting
static inline void EEPROM_Write_Port_Boot_State(uint8_t port, uint8_t state) {
	eeprom_update_byte((uint8_t*)(EEPROM_OFFSET_PORT_DEFAULTS + port), state);
}

// Read the stored reference voltage from EEPROM
static inline float EEPROM_Read_REF_V(void) {
	float REF_V = eeprom_read_float((float*)(EEPROM_OFFSET_REF_V));
	// If the value seems out of range (uninitialized), default it to 4.5
	if (REF_V < 4.1 || REF_V > 4.3 || isnan(REF_V)) REF_V = 4.2;
	return REF_V;
}
// Write the reference voltage to EEPROM
static inline void EEPROM_Write_REF_V(float reference) {
	eeprom_update_float((float*)(EEPROM_OFFSET_REF_V), reference);
}

// Read the main bus divider calibration from EEPROM
static inline float EEPROM_Read_V_CAL_MAIN(void) {
	uint8_t V_CAL = eeprom_read_byte((uint8_t*)(EEPROM_OFFSET_V_CAL_MAIN));
	// If the value seems out of range (uninitialized), default it to 15
	if (V_CAL < VCAL_MIN || V_CAL > VCAL_MAX) V_CAL = 150;
	return (float)(V_CAL / 10.0);
}
// Write the main bus divider calibration to EEPROM
static inline void EEPROM_Write_V_CAL_MAIN(float div) {
	eeprom_update_byte((uint8_t*)(EEPROM_OFFSET_V_CAL_MAIN), (int)(div * 10.0));
}
// Read the alt bus divider calibration from EEPROM
static inline float EEPROM_Read_V_CAL_ALT(void) {
	uint8_t V_CAL = eeprom_read_byte((uint8_t*)(EEPROM_OFFSET_V_CAL_ALT));
	// If the value seems out of range (uninitialized), default it to 15
	if (V_CAL < VCAL_MIN || V_CAL > VCAL_MAX) V_CAL = 150;
	return (float)(V_CAL / 10.0);
}
// Write the alt bus divider calibration to EEPROM
static inline void EEPROM_Write_V_CAL_ALT(float div) {
	eeprom_update_byte((uint8_t*)(EEPROM_OFFSET_V_CAL_ALT), (int)(div * 10.0));
}

// Read the stored port current calibration
static inline float EEPROM_Read_I_CAL(uint8_t port) {
	uint16_t I_CAL = eeprom_read_word((uint16_t*)(EEPROM_OFFSET_I_CAL + (port*2)));
	if(I_CAL < ICAL_MIN || I_CAL > ICAL_MAX) I_CAL = 500;
	return (float)(I_CAL / 10.0);
}
// Write the port current calibration to EEPROM
static inline void EEPROM_Write_I_CAL(uint8_t port, float cal) {
	eeprom_update_word((uint16_t*)(EEPROM_OFFSET_I_CAL + (port*2)), (int)(cal * 10.0));
}

// Read PCYCLE_TIME from EEPROM
// Stored as Seconds
static inline uint8_t EEPROM_Read_PCycle_Time(void) {
	uint8_t PCYCLE_TIME = eeprom_read_byte((uint8_t*)(EEPROM_OFFSET_CYCLE_TIME));
	if (PCYCLE_TIME > PCYCLE_MAX_TIME) PCYCLE_TIME = 1;
	return PCYCLE_TIME;
}
// Write the PCYCLE_TIME to EEPROM
static inline void EEPROM_Write_PCycle_Time(uint8_t time) {
	eeprom_update_byte((uint8_t*)(EEPROM_OFFSET_CYCLE_TIME), time);
}

// Read the stored port name
static inline void EEPROM_Read_Port_Name(int8_t port, char *str) {
	char working = 0;
	uint8_t count = 0;
	
	while (1) {
		// Read a byte from the EEPROM
		working = eeprom_read_byte((uint8_t*)(EEPROM_OFFSET_P0NAME+(port*16)+count));
		
		// If we've reached the end of the string, terminate the string, and break.
		if (working  == 255 || working == 0) {
			*str = 0;
			break;
		}
		
		// Take the byte we've read, and attach it to the string
		*str = working;
		str++;
		count++;
	}
}
// Write the port name to EEPROM
static inline void EEPROM_Write_Port_Name(int8_t port, char *str) {
	// While we haven't reached the end of the string, or reached the end of the buffer
	// Write the name bytes to EEPROM
	for (uint8_t i = 0; i < 15; i++) { eeprom_update_byte((uint8_t*)(EEPROM_OFFSET_P0NAME+(port*16)+i), 0); }
	for (uint8_t i = 0; i < 15; i++) {
		if (*str == 0) { break; }
		eeprom_update_byte((uint8_t*)(EEPROM_OFFSET_P0NAME+(port*16)+i), *str);
		str++;
	}
	eeprom_update_byte((uint8_t*)(EEPROM_OFFSET_P0NAME+(port*16)+15), 0);
}

// Stored as amps*10 so 50==5.0A
static inline uint8_t EEPROM_Read_Port_Limit(uint8_t port) {
	uint8_t limit = eeprom_read_byte((uint8_t*)(EEPROM_OFFSET_LIMIT+(port)));
	if (limit > LIMIT_MAX) { limit = LIMIT_MAX; }
	return limit;
}
// Stored as amps*10 so 50==5.0A
static inline void EEPROM_Write_Port_Limit(uint8_t port, uint8_t limit) {
	eeprom_update_byte((uint8_t*)(EEPROM_OFFSET_LIMIT+(port)), limit);
}

// Reads the cutoff voltage from EEPROM. Voltage = value/100
static inline float EEPROM_Read_Port_CutOff(uint8_t port) {
	float cutoff = eeprom_read_word((uint16_t*)(EEPROM_OFFSET_V_CUTOFF+(port*2))) / 100.0;
	if (cutoff > VMAX) { cutoff = 0; }
	return cutoff;
}
// Stored as (int)cutoff*100
static inline void EEPROM_Write_Port_CutOff(uint8_t port, uint16_t cutoff) {
	eeprom_update_word((uint16_t*)(EEPROM_OFFSET_V_CUTOFF+(port*2)), cutoff);
}

// Reads the cuton voltage from EEPROM. Voltage = value/100
static inline float EEPROM_Read_Port_CutOn(uint8_t port) {
	float cuton = eeprom_read_word((uint16_t*)(EEPROM_OFFSET_V_CUTON+(port*2))) / 100.0;
	if (cuton > VMAX) { cuton = VMAX; }
	return cuton;
}
// Stored as (int)cuton*100
static inline void EEPROM_Write_Port_CutOn(uint8_t port, uint16_t cuton) {
	eeprom_update_word((uint16_t*)(EEPROM_OFFSET_V_CUTON+(port*2)), cuton);
}

// Reads the ADC offset for the port current sense from EEPROM. Raw ADC counts.
static inline uint8_t EEPROM_Read_I_Offset(uint8_t port) {
	uint8_t offset = eeprom_read_byte((uint8_t*)(EEPROM_OFFSET_I_OFFSET+(port)));
	if (offset > OFFSET_MAX) { offset = 0; }
	return offset;
}
// Stored as raw ADC counts.
static inline void EEPROM_Write_I_Offset(uint8_t port, uint8_t offset) {
	eeprom_update_byte((uint16_t*)(EEPROM_OFFSET_I_OFFSET+(port)), offset);
}

// Reset all EEPROM values to 255
static inline void EEPROM_Reset(void) {
	for (uint16_t i = 0; i < 512; i++) {
		eeprom_update_byte((uint8_t*)(i), 255);
	}
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Debugging Functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Dump debugging data
static inline void DEBUG_Dump(void) {
	// Print hardware and software versions
	fprintf(&USBSerialStream, "\r\nV%s,%s", HARDWARE_VERS, SOFTWARE_VERS);

	// Read port defaults
	printPGMStr(STR_Port_Default);
	for (uint8_t i = 0; i < PORT_CNT; i++) {
		fprintf(&USBSerialStream, "%i ", eeprom_read_byte((uint8_t*)(EEPROM_OFFSET_PORT_DEFAULTS + i)));
	}
	
	// Read REF_V
	printPGMStr(STR_VREF);
	fprintf(&USBSerialStream, "%.2f:%.2f", eeprom_read_float((float*)(EEPROM_OFFSET_REF_V)), EEPROM_Read_REF_V());
	// Read V_CAL
	printPGMStr(STR_VCAL);
	fprintf(&USBSerialStream, "MAIN: %i:%.1f", eeprom_read_byte((uint8_t*)(EEPROM_OFFSET_V_CAL_MAIN)), EEPROM_Read_V_CAL_MAIN());
	fprintf(&USBSerialStream, " ALT: %i:%.1f", eeprom_read_byte((uint8_t*)(EEPROM_OFFSET_V_CAL_ALT)), EEPROM_Read_V_CAL_ALT());
	// Read I_CAL
	printPGMStr(STR_ICAL);
	for (uint8_t i = 0; i < PORT_CNT; i++) {
		fprintf(&USBSerialStream, "%i:%.1f ", eeprom_read_word((uint16_t*)(EEPROM_OFFSET_I_CAL + (i*2))), EEPROM_Read_I_CAL(i));
	}
	// Read I_OFFSET
	printPGMStr(STR_OFFSET);
	for (uint8_t i = 0; i < PORT_CNT; i++) {
		fprintf(&USBSerialStream, "%i:%i:%i ", ADC_Read_Raw(i), eeprom_read_byte((uint8_t*)(EEPROM_OFFSET_I_OFFSET + i)), EEPROM_Read_I_Offset(i));
	}
	
	// Read Port Cycle Time
	printPGMStr(STR_PCYCLE_Time);
	fprintf(&USBSerialStream, "%iS", eeprom_read_byte((uint8_t*)(EEPROM_OFFSET_CYCLE_TIME)));
	
	// Read Port Limits
	printPGMStr(STR_Port_Limit);
	for (uint8_t i = 0; i < PORT_CNT; i++) {
		fprintf(&USBSerialStream, "%i:%i ", eeprom_read_byte((uint8_t*)(EEPROM_OFFSET_LIMIT + i)), EEPROM_Read_Port_Limit(i));
	}
	
	// Read Port Cutoffs
	printPGMStr(STR_Port_CutOff);
	for (uint8_t i = 0; i < PORT_CNT; i++) {
		fprintf(&USBSerialStream, "%u:%.1f ", eeprom_read_word((uint16_t*)(EEPROM_OFFSET_V_CUTOFF + i*2)), EEPROM_Read_Port_CutOff(i));
	}
	// Read Port Cutons
	printPGMStr(STR_Port_CutOn);
	for (uint8_t i = 0; i < PORT_CNT; i++) {
		fprintf(&USBSerialStream, "%u:%.1f ", eeprom_read_word((uint16_t*)(EEPROM_OFFSET_V_CUTON + i*2)), EEPROM_Read_Port_CutOn(i));
	}
	
	// Read Port Names
	printPGMStr(PSTR("\r\nPNAMES: "));
	for (int8_t i = -1; i < PORT_CNT; i++) {
		for (int8_t j = 0; j < 16; j++) {
			fputc(eeprom_read_byte((uint8_t*)(EEPROM_OFFSET_P0NAME+(i*16)+j)), &USBSerialStream);
		}
		fputc(' ', &USBSerialStream);
	}
	
	// Read Port High Water Marks
	printPGMStr(PSTR("\r\nI High Water: "));
	for (uint8_t i = 0; i < PORT_CNT; i++) {
		fprintf(&USBSerialStream, "%.2f ", (PORT_HIGH_WATER[i] / 100.0));
	}
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ ADC Functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Read current flow on a given port
static inline float ADC_Read_Port_Current(uint8_t port) {
	// Read the raw current sense voltage value
	int16_t raw = ADC_Read_Raw(port) - EEPROM_Read_I_Offset(port);
	if(raw < 0) raw = 0;
	
	// Calculate the current from the voltage reading
	float current = (raw * (EEPROM_Read_REF_V() / 1024) / EEPROM_Read_I_CAL(port)) / 0.02;
	
	// Check current reading against the high water mark
	if (current > (float)(PORT_HIGH_WATER[port] / 100.0)){
		if((current * 100) < 255){
			PORT_HIGH_WATER[port] = (uint8_t)(current * 100);
		} else {
			PORT_HIGH_WATER[port] = 255;
		}
	}
	
	return current;
}

// Read temperature (die temperature, uncalibrated, +/-10C)
static inline int16_t ADC_Read_Temperature(void) {
	ADMUX = 0b11000111;
	ADCSRB = 0b00100000;
	ADCSRA |= (1<<ADSC); // Start first conversion (throw away)
	while (ADCSRA & (1<<ADSC)); // Wait for conversion to complete
	ADCSRA |= (1<<ADSC); // Start second conversion (valid)
	while (ADCSRA & (1<<ADSC)); // Wait for conversion to complete
	return ADCW - 273;
}

// Read MAIN input voltage
static inline float ADC_Read_Main_Voltage(void) {
	return (ADC_Read_Raw(12) * (EEPROM_Read_REF_V() / 1024) * EEPROM_Read_V_CAL_MAIN());
}

// Read ALT input voltage
static inline float ADC_Read_Alt_Voltage(void) {
	return (ADC_Read_Raw(13) * (EEPROM_Read_REF_V() / 1024) * EEPROM_Read_V_CAL_ALT());
}

// Return raw counts from the ADC
// Ports 0-11 are current sensors for the 12 ports
// Port 12 is the ADC channel for the MAIN bus
// Port 13 is the ADC channel for the ALT bus
static inline uint16_t ADC_Read_Raw(uint8_t port) {
	uint8_t temp1,temp2,count;
	uint16_t average = 0;
	
	// Take ADC_AVG_POINTS samples
	for (count = 0; count < ADC_AVG_POINTS; count++) {
		if ((port >= 0 && port < 6) || port == 12 || port == 13) {
#ifndef TESTBOARD
			PORTB &= ~(1 << SPI_SS_1);
#else
			PORTF &= ~(1 << SPI_SS_1);
#endif
		} else if (port >= 6 && port < 12) {
#ifndef TESTBOARD
			PORTB &= ~(1 << SPI_SS_2);
#else
			PORTF &= ~(1 << SPI_SS_2);
#endif		
		} else {
			// An invalid port was requested.
			return 0;
		}
		
		SPI_transfer(0x01); // Start bit
		temp1 = SPI_transfer(Ports_ADC[port]); // Single ended, input number, clocking in 4 bits
		temp2 = SPI_transfer(0x00); // Clocking in 8 bits.

		if ((port >= 0 && port < 6) || port == 12 || port == 13) {
#ifndef TESTBOARD
			PORTB |= (1 << SPI_SS_1);
#else
			PORTF |= (1 << SPI_SS_1);
#endif
		} else if (port >= 6 && port < 12) {
#ifndef TESTBOARD
			PORTB |= (1 << SPI_SS_2);
#else
			PORTF |= (1 << SPI_SS_2);
#endif		
		}
		
		// Add each sample to our average variable
		average += (uint16_t)((temp1 & 0b00000011) << 8) | temp2;
	}
	
	return (average / ADC_AVG_POINTS);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ USB Functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Run the LUFA USB tasks (except reading)
static inline void run_lufa(void) {
	//CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
	CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
	USB_USBTask();
}

// Event handler for the library USB Connection event.
void EVENT_USB_Device_Connect(void) {
	// We're enumerated. Act on that as desired.
}

// Event handler for the library USB Disconnection event.
void EVENT_USB_Device_Disconnect(void) {
	// We're no longer enumerated. Act on that as desired.
}

// Event handler for the library USB Configuration Changed event.
void EVENT_USB_Device_ConfigurationChanged(void) {
	bool ConfigSuccess = true;
	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
	// USB is ready. Act on that as desired.
}

// Event handler for the library USB Control Request reception event.
void EVENT_USB_Device_ControlRequest(void) {
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ SPI Functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Set up the port registers for SPI
static inline void SPI_begin(void) {
	// Set up SPI pins
#ifndef TESTBOARD	
	DDRB |= (1 << SPI_SS_1) | (1 << SPI_SS_2);
	DDRB |= (1 << SPI_SCK)|(1 << SPI_MOSI);
	PORTB |= (1 << SPI_SS_1) | (1 << SPI_SS_2);
	PORTB &= ~(1 << SPI_SCK);
	DDRB &= ~(1 << SPI_MISO);
	PORTB &= ~(1 << SPI_MISO);
#else
	DDRF |= (1 << SPI_SS_1) | (1 << SPI_SS_2);
	DDRB |= (1 << SPI_SCK)|(1 << SPI_MOSI);
	PORTF |= (1 << SPI_SS_1) | (1 << SPI_SS_2);
	PORTB &= ~(1 << SPI_SCK);
	DDRB &= ~(1 << SPI_MISO);
	PORTB &= ~(1 << SPI_MISO);
#endif
}

// Transfer out a byte on the SPI port, and simultaneously read a byte from SPI
static inline uint8_t SPI_transfer(uint8_t data) {
	uint8_t value = 0;
	uint8_t i;
	
	for (i = 0; i < 8; i++){
		if (!!(data & (1 << (7 - i)))) {
			PORTB |= (1 << SPI_MOSI);
		} else {
			PORTB &= ~(1 << SPI_MOSI);
		}
		PORTB |= (1 << SPI_SCK); // Clock pin high
		value |= ((PINB & (1 << SPI_MISO)) >> SPI_MISO) << (7 - i);
		PORTB &= ~(1 << SPI_SCK); // Clock pin low
	}
		
	return value;
}

