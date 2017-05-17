/*
 * MWDisplay
 * Copyright (C) 2017, László Dániel
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Created: 2017.04.27. 16:07:18
 * Description: Digital display for Motowell Magnet Sport 2T 50cc AC scooters.
 */ 

/*
 *           SYSTEM INFO          
 * -------------------------------
 *    MCU: ATmega1284P-AU
 *  CLOCK: 16MHz
 *  FLASH: 128kB
 *    RAM: 16kB
 * EEPROM: 4kB
 *   PROG: ISP
 * -------------------------------
 *    USB: ATmega8U2-AU
 *  CLOCK: 16MHz
 *  FLASH: 8kB
 *    RAM: 0.5kB
 * EEPROM: 0.5kB
 *   PROG: ISP
 * -------------------------------
 *    LCD: ???
 * DRIVER: ???
 */


#include <avr/io.h>
#include <avr/eeprom.h>


/* 
-------------------------------------------------------------------------------
Internal EEPROM offsets ($0000-$03FF -> Total of 1 kbytes)
-------------------------------------------------------------------------------

$0000: ODOMETER (4 bytes)
$0004: TRIPMETER (4 bytes)
	Scaling: 1 bit = 0.000125 km
	RAW value (unsigned) / 8000 = km value

	$00000000 = 000000.000000 km <- MIN value
	$00000001 = 000000.000125 km
	$00000002 = 000000.000250 km
	$00000003 = 000000.000375 km
	$00000004 = 000000.000500 km
	$00000005 = 000000.000625 km
	...
	$000001FF = 000000.063875 km
	$00000200 = 000000.064000 km
	...
	$001FABCD = 000259.449625 km
	...
	$01CA86F3 = 003756.254375 km
	...
	$0ED475AC = 031000.597500 km
	...
	$FFFFFFFF = 536870.911900 km <- MAX value


$0008: MAX_SPEED (2 bytes)
	Scaling: 1 bit = 0.0025 km/h
	RAW value (unsigned) / 400 = km/h value

	$0000 = 000.0000 km/h <- MIN value
	$0001 = 000.0025 km/h
	$0002 = 000.0050 km/h
	$0003 =	000.0075 km/h
	$0004 = 000.0100 km/h
	$0005 = 000.0125 km/h
	...
	$01FF = 001.2755 km/h
	$0200 = 001.2800 km/h
	...
	$3FD4 = 040.8500 km/h
	...
	$5FA5 = 061.2125 km/h
	...
	$D053 = 133.3275 km/h
	...
	$FFFF = 163.8375 km/h <- MAX value


$000A: MAX_RPM (2 bytes)
	Scaling: 1 bit = 0.25 rpm
	RAW value (unsigned) / 4 = rpm value

	$0000 = 00000.00 rpm <- MIN value
	$0001 = 00000.25 rpm
	$0002 = 00000.50 rpm
	$0003 = 00000.75 rpm
	$0004 = 00001.00 rpm
	$0005 = 00001.25 rpm
	...
	$7D00 = 08000.00 rpm
	...
	$ABCD = 10955.25 rpm
	...
	$FFFF = 16383.75 rpm <- MAX value


$000C: MIN_CHT_TEMP (2 bytes)
$000E: MAX_CHT_TEMP (2 bytes)
$0010: MIN_CVT_TEMP (2 bytes)
$0012: MAX_CVT_TEMP (2 bytes)
$0014: MIN_AMB_TEMP (2 bytes)
$0016: MAX_AMB_TEMP (2 bytes)
	Scaling: 1 bit = 0.005 °C
	(RAW value (signed) - 32767) / 200 = signed °C value

	$0000 = -163.835 °C <- MIN value
	...
	$705F = -020.000 °C
	$7FFE = -000.005 °C
	$7FFF =  000.000 °C
	$8000 =  000.005 °C
	...
	$FFFF =  163.840 °C <- MAX value


$0018-$00FF: RESERVED


$0100: REAR_TIRE_CIRC (2 bytes)
	Scaling: 1 bit = 0.05 mm
	RAW value (unsigned) / 20 = mm value

	$0000 = 0000.00 mm <- MIN value
	$0001 = 0000.05 mm
	$0002 = 0000.10 mm
	$0003 = 0000.15 mm
	$0004 = 0000.20 mm
	$0005 = 0000.25 mm
	...
	$794A = 1552.50 mm
	...
	$FFFF = 3276.75 mm <- MAX value


$0102: REAR_TIRE_DIAM (2 bytes)
	Scaling: 1 bit = 0.0125 mm
	RAW value (unsigned) / 80 = mm value

	$0000 = 000.0000 mm <- MIN value
	$0001 = 000.0125 mm
	$0002 = 000.0250 mm
	$0003 = 000.0375 mm
	$0004 = 000.0500 mm
	$0005 = 000.0625 mm
	...
	$8CA0 = 450.0000 mm
	...
	$FFFF = 819.1875 mm <- MAX value


$0104: FINAL_DRIVE_GEARS (4 bytes)
	Scaling: 1 bit = 1 tooth

	$0D340D2C =
		$0D: primary   #1, z1 = 13 tooth
		$34: primary   #2, z2 = 52 tooth
		$0D: secondary #1, z3 = 13 tooth
		$2C: secondary #2, z4 = 44 tooth


$0108: FINAL_DRIVE_RATIO (2 bytes)
	Scaling: 1 bit = 0.00032
	RAW value (unsigned) / 3125 = 1:X (X value dimensionless)

	$0000 = 00.00000 <- MIN value
	$0001 = 00.00032
	$0002 = 00.00064
	$0003 = 00.00096
	$0004 = 00.01280
	$0005 = 00.01600
	...
	$9EB1 = 13.00000
	...
	$FFFF = 20.97120 <- MAX value


$010A-$01FF: RESERVED


$0200-$03FF: NOT USED


Example:

OFFSET	|	0	1	2	3	4	5	6	7	8	9	A	B	C	D	E	F
--------------------------------------------------------------------------
$0000	|	03	AB	34	E6	00	00	FE	CD	62	D7	AB	0F	72	D0	CC	A4
$0010	|	81	4F	9C	D1	73	19	9E	13	00	00	00	00	00	00	00	00
$0020	|	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00
$0030	|	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00
...		|
$0100	|	79	4A	9A	6E	0D	34	0D	2C	A5	44	00	00	00	00	00	00
$0110	|	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00
$0120	|	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00
$0130	|	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00
...		|
$03D0	|	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00
$03E0	|	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00
$03F0	|	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00	00


-------------------------------------------------------------------------------
VARIABLES ($0000 - $00FF):
-------------------------------------------------------------------------------
OFFSET:				VALUE:			SCALING:					RESULT:
-------------------------------------------------------------------------------
ODOMETER:			$03AB34E6	->  61551846 / 8000			=  7693.980750 km
TRIPMETER:			$0000FECD	->  65229 / 8000			=     8.153625 km
MAX_SPEED:			$62D7		->  25303 / 400				=    63.2575 km/h
MAX_RPM:			$AB0F		->  43791 / 4				= 10947.75 rpm
MIN_CHT_TEMP:		$72D0		-> (29392 - 32767) / 200	=   -16.875 °C
MAX_CHT_TEMP:		$CCA4		-> (52388 - 32767) / 200	=    98.105 °C
MIN_CVT_TEMP:		$814F		-> (33103 - 32767) / 200	=     1.680 °C
MAX_CVT_TEMP:		$9CD1		-> (40145 - 32767) / 200	=    36.890 °C
MIN_AMB_TEMP:		$7319		-> (29465 - 32767) / 200	=   -16.510 °C		
MAX_AMB_TEMP:		$9E13		-> (40467 - 32767) / 200	=    38.500 °C
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
CONSTANTS ($0100 - $01FF):
-------------------------------------------------------------------------------
OFFSET:				VALUE:			SCALING:					RESULT:
-------------------------------------------------------------------------------
REAR_TIRE_CIRC:		$794A		->  31050 / 20				=  1552.5 mm
REAR_TIRE_DIAM:		$9A6E		->  39534 / 80				=   494.175 mm
FINAL_DRIVE_GEARS:	$0D340D2C	->  p1 = 13, p2 = 52, 
									s1 = 13, s2 = 44		= tooth numbers
FINAL_DRIVE_RATIO:	$A544		->	42308 / 3125			=     1:13.53856

REAR_TIRE_CIRC and REAR_TIRE_DIAM are interchangeable. One is necessary.
FINAL_DRIVE_GEARS and FINAL_DRIVE_RATIO are interchangeable. One is necessary.
-------------------------------------------------------------------------------
*/


// Constants
// EEPROM offsets
const uint32_t ODOMETER_OFFSET			= 0x0000;
const uint32_t TRIPMETER_OFFSET			= 0x0004;
const uint16_t MAX_SPEED_OFFSET			= 0x0008;
const uint16_t MAX_RPM_OFFSET			= 0x000A;
const uint16_t MIN_CHT_TEMP_OFFSET		= 0x000C;
const uint16_t MAX_CHT_TEMP_OFFSET		= 0x000E;
const uint16_t MIN_CVT_TEMP_OFFSET		= 0x0010;
const uint16_t MAX_CVT_TEMP_OFFSET		= 0x0012;
const uint16_t MIN_AMBIENT_TEMP_OFFSET	= 0x0014;
const uint16_t MAX_AMBIENT_TEMP_OFFSET	= 0x0016;
const uint16_t REAR_TIRE_CIRC_OFFSET	= 0x0100;
const uint16_t REAR_TIRE_DIAM_OFFSET	= 0x0102;
const uint32_t FINAL_DRIVE_GEARS_OFFSET = 0x0104;
const uint16_t FINAL_DRIVE_RATIO_OFFSET = 0x0108;

// Variables
// Interrupt controlled global variables (changing really fast)
volatile uint32_t fw_cnt	= 0; // FlyWheel rotation counter for engine rpm
volatile uint32_t cb_cnt	= 0; // Clutch Bell rotation counter for speed

// Global variables (these are all raw values that need scaling)...
// ...updated regularly
uint32_t odometer			= 0; // Current odometer value [km]
uint32_t tripmeter			= 0; // Current tripmeter value [km]
uint16_t speed				= 0; // Current speed [km/h]
uint16_t rpm				= 0; // Current crankshaft rotation per minute [rpm]
uint16_t cht_temp			= 0; // Current cylinder head temperature [°C]
uint16_t cvt_temp			= 0; // Current CVT cover temperature [°C]
uint16_t ambient_temp		= 0; // Current ambient temperature [°C]
uint16_t battery_voltage	= 0; // Current battery voltage / charging voltage [V]
uint16_t fuel_level			= 0; // Current fuel level [%]
uint16_t oil_level			= 0; // Current oil level [%]

// ...occasionally updated and saved to EEPROM
uint16_t max_speed			= 0; // Highest speed measured [km/h]
uint16_t max_rpm			= 0; // Highest crankshaft rotation measured [rpm]
uint16_t min_cht_temp		= 0; // Lowest cylinder head temperature measured [°C]
uint16_t max_cht_temp		= 0; // Highest cylinder head temperature measured [°C]
uint16_t min_cvt_temp		= 0; // Lowest CVT cover temperature measured [°C]
uint16_t max_cvt_temp		= 0; // Highest CVT cover temperature measured [°C]
uint16_t min_amb_temp		= 0; // Lowest ambient temperature measured [°C]
uint16_t max_amb_temp		= 0; // Highest ambient temperature measured [°C]

// ...updated once after setup and saved to EEPROM
uint16_t rear_tire_circ		= 0; // Rear tire circumference [mm]
uint16_t rear_tire_diam		= 0; // Rear tire diameter [mm]
uint32_t final_drive_gears	= 0; // Tooth number of the four gears in the final drive (primary #1, primary #2, secondary #1, secondary #2)
uint16_t final_drive_ratio	= 0; // Final drive input:output ratio


// Main entry point of the program
int main(void)
{
	// Load stored values from EEPROM into SRAM
	// Wait until the internal EEPROM is available to read...
	eeprom_busy_wait();

	// ...then begin reading last saved values
	odometer			= eeprom_read_dword(&ODOMETER_OFFSET);
	tripmeter			= eeprom_read_dword(&TRIPMETER_OFFSET);
	max_speed			= eeprom_read_word(&MAX_SPEED_OFFSET);
	max_rpm				= eeprom_read_word(&MAX_RPM_OFFSET);
	min_cht_temp		= eeprom_read_word(&MIN_CHT_TEMP_OFFSET);
	max_cht_temp		= eeprom_read_word(&MAX_CHT_TEMP_OFFSET);
	min_cvt_temp		= eeprom_read_word(&MIN_CVT_TEMP_OFFSET);
	max_cvt_temp		= eeprom_read_word(&MAX_CVT_TEMP_OFFSET);
	min_amb_temp		= eeprom_read_word(&MIN_AMBIENT_TEMP_OFFSET);
	max_amb_temp		= eeprom_read_word(&MAX_AMBIENT_TEMP_OFFSET);
	rear_tire_circ		= eeprom_read_word(&REAR_TIRE_CIRC_OFFSET);
	rear_tire_diam		= eeprom_read_word(&REAR_TIRE_DIAM_OFFSET);
	final_drive_gears	= eeprom_read_dword(&FINAL_DRIVE_GEARS_OFFSET);
	final_drive_ratio	= eeprom_read_word(&FINAL_DRIVE_RATIO_OFFSET);
	
	// Enter infinite loop
	for (;;) 
    {
		// Magic goes here...
    }

	return 0;
}
