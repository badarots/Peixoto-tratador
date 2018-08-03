EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:ESP8266
LIBS:esp12-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ESP-12E U1
U 1 1 5B3E1B76
P 3050 2600
F 0 "U1" H 3050 2500 50  0000 C CNN
F 1 "ESP-12E" H 3050 2700 50  0000 C CNN
F 2 "RF_Modules:ESP-12E" H 3050 2600 50  0001 C CNN
F 3 "" H 3050 2600 50  0001 C CNN
	1    3050 2600
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5B3E1E06
P 4100 3300
F 0 "R1" V 4180 3300 50  0000 C CNN
F 1 "10k" V 4100 3300 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 4030 3300 50  0001 C CNN
F 3 "" H 4100 3300 50  0001 C CNN
	1    4100 3300
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5B3E1E57
P 2450 3900
F 0 "C1" H 2475 4000 50  0000 L CNN
F 1 "100n" H 2475 3800 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 2488 3750 50  0001 C CNN
F 3 "" H 2450 3900 50  0001 C CNN
	1    2450 3900
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x08_Male J2
U 1 1 5B3E22AE
P 4700 2600
F 0 "J2" H 4700 3000 50  0000 C CNN
F 1 "Conn_01x08_Male" H 4700 2100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x08_Pitch2.54mm" H 4700 2600 50  0001 C CNN
F 3 "" H 4700 2600 50  0001 C CNN
	1    4700 2600
	-1   0    0    -1  
$EndComp
$Comp
L Conn_01x08_Male J1
U 1 1 5B3E24C9
P 1250 2600
F 0 "J1" H 1250 3000 50  0000 C CNN
F 1 "Conn_01x08_Male" H 1250 2100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x08_Pitch2.54mm" H 1250 2600 50  0001 C CNN
F 3 "" H 1250 2600 50  0001 C CNN
	1    1250 2600
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x06_Male J3
U 1 1 5B3E2810
P 3100 1350
F 0 "J3" H 3100 1650 50  0000 C CNN
F 1 "Conn_01x06_Male" H 3100 950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 3100 1350 50  0001 C CNN
F 3 "" H 3100 1350 50  0001 C CNN
	1    3100 1350
	0    1    1    0   
$EndComp
Wire Wire Line
	1450 2300 2150 2300
Wire Wire Line
	2150 2400 1450 2400
Wire Wire Line
	1450 2500 2150 2500
Wire Wire Line
	2150 2600 1450 2600
Wire Wire Line
	1450 2700 2150 2700
Wire Wire Line
	2150 2800 1450 2800
Wire Wire Line
	1450 2900 2150 2900
Wire Wire Line
	3950 2900 4500 2900
Wire Wire Line
	3950 2800 4500 2800
Wire Wire Line
	3950 2700 4500 2700
Wire Wire Line
	3950 2600 4500 2600
Wire Wire Line
	4500 2500 3950 2500
Wire Wire Line
	3950 2400 4500 2400
Wire Wire Line
	3950 2300 4500 2300
$Comp
L R R3
U 1 1 5B3E3202
P 1700 3100
F 0 "R3" V 1780 3100 50  0000 C CNN
F 1 "10k" V 1700 3100 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 1630 3100 50  0001 C CNN
F 3 "" H 1700 3100 50  0001 C CNN
	1    1700 3100
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5B3E323D
P 4300 3650
F 0 "R2" V 4380 3650 50  0000 C CNN
F 1 "10k" V 4300 3650 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 4230 3650 50  0001 C CNN
F 3 "" H 4300 3650 50  0001 C CNN
	1    4300 3650
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 5B3E328B
P 1950 3100
F 0 "R4" V 2030 3100 50  0000 C CNN
F 1 "10k" V 1950 3100 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 1880 3100 50  0001 C CNN
F 3 "" H 1950 3100 50  0001 C CNN
	1    1950 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 1700 2150 3750
Wire Wire Line
	1100 4100 4500 4100
Wire Wire Line
	2450 4050 2450 4100
Connection ~ 2450 4100
Wire Wire Line
	2150 3750 2450 3750
Connection ~ 2150 3750
$Comp
L LM1117-3.3 U2
U 1 1 5B3E1D4B
P 1850 3750
F 0 "U2" H 1700 3875 50  0000 C CNN
F 1 "LM1117-3.3" H 1850 3875 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-223" H 1850 3750 50  0001 C CNN
F 3 "" H 1850 3750 50  0001 C CNN
	1    1850 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 4100 1850 4050
Wire Wire Line
	1450 3750 1550 3750
Wire Wire Line
	4100 3150 4100 2900
Connection ~ 4100 2900
Wire Wire Line
	4100 3450 4100 4100
Connection ~ 4100 4100
Wire Wire Line
	1700 2950 1700 2500
Connection ~ 1700 2500
Wire Wire Line
	1950 1850 1950 2950
Connection ~ 1950 2300
Wire Wire Line
	4300 1750 4300 3500
Connection ~ 4300 2700
Wire Wire Line
	4300 3800 4300 4000
Wire Wire Line
	4300 4000 2650 4000
Wire Wire Line
	2650 4000 2650 3550
Wire Wire Line
	2650 3550 2150 3550
Connection ~ 2150 3550
Wire Wire Line
	1950 3250 2150 3250
Connection ~ 2150 3250
Wire Wire Line
	1700 3250 1700 3350
Wire Wire Line
	1700 3350 2150 3350
Connection ~ 2150 3350
Connection ~ 2150 3000
Wire Wire Line
	2100 2300 2100 1800
Wire Wire Line
	2100 1800 2900 1800
Wire Wire Line
	2900 1800 2900 1550
Connection ~ 2100 2300
Wire Wire Line
	4000 2300 4000 1900
Wire Wire Line
	4000 1900 3000 1900
Wire Wire Line
	3000 1900 3000 1550
Connection ~ 4000 2300
Wire Wire Line
	4150 2400 4150 1800
Wire Wire Line
	4150 1800 3100 1800
Wire Wire Line
	3100 1800 3100 1550
Connection ~ 4150 2400
Wire Wire Line
	4300 1750 3200 1750
Wire Wire Line
	3200 1750 3200 1550
Wire Wire Line
	4400 3000 4400 1600
Wire Wire Line
	4400 1600 3300 1600
Wire Wire Line
	3300 1600 3300 1550
Connection ~ 4400 3000
Wire Wire Line
	2150 1700 2800 1700
Wire Wire Line
	2800 1700 2800 1550
Wire Wire Line
	4500 4100 4500 3000
Wire Wire Line
	1450 3000 1450 3750
Wire Wire Line
	4500 3000 3950 3000
NoConn ~ 2800 3500
NoConn ~ 2900 3500
NoConn ~ 3000 3500
NoConn ~ 3100 3500
NoConn ~ 3200 3500
NoConn ~ 3300 3500
$Comp
L SW_Push_Dual SW1
U 1 1 5B3E9DA5
P 1650 2050
F 0 "SW1" H 1700 2150 50  0000 L CNN
F 1 "SW_Push_Dual" H 1650 1780 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_TH_Tactile_Omron_B3F-10xx" H 1650 2250 50  0001 C CNN
F 3 "" H 1650 2250 50  0001 C CNN
	1    1650 2050
	1    0    0    1   
$EndComp
NoConn ~ 1450 2050
NoConn ~ 1850 2050
Wire Wire Line
	1100 4100 1100 1850
Wire Wire Line
	1100 1850 1450 1850
Connection ~ 1850 4100
Wire Wire Line
	1850 1850 1950 1850
$EndSCHEMATC
