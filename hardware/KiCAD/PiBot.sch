EESchema Schematic File Version 2
LIBS:power
LIBS:device
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
LIBS:mk20dx64vlh7
LIBS:PiBot-cache
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
L MK20DX64VLH7 U?
U 1 1 582D41FB
P 2250 2050
F 0 "U?" H 2900 3000 60  0000 C CNN
F 1 "MK20DX64VLH7" H 2300 2250 60  0000 C CNB
F 2 "Housings_QFP:LQFP-64_10x10mm_Pitch0.5mm" H 2250 2050 60  0001 C CNN
F 3 "" H 2250 2050 60  0001 C CNN
	1    2250 2050
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 582E9263
P 2150 650
F 0 "#PWR?" H 2150 500 50  0001 C CNN
F 1 "+3.3V" H 2000 700 50  0000 C CNN
F 2 "" H 2150 650 50  0000 C CNN
F 3 "" H 2150 650 50  0000 C CNN
	1    2150 650 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 582E92A4
P 2100 3000
F 0 "#PWR?" H 2100 2750 50  0001 C CNN
F 1 "GND" H 2100 2850 50  0000 C CNN
F 2 "" H 2100 3000 50  0000 C CNN
F 3 "" H 2100 3000 50  0000 C CNN
	1    2100 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 3000 2400 3000
Connection ~ 2100 3000
Wire Wire Line
	2150 650  2150 1050
Wire Wire Line
	2150 1050 1950 1050
Connection ~ 2100 1050
Connection ~ 2050 1050
NoConn ~ 3150 1250
Connection ~ 2150 3000
NoConn ~ 2350 1050
$Comp
L C C?
U 1 1 582E9431
P 1800 800
F 0 "C?" H 1825 900 50  0000 L CNN
F 1 "0.1uF" H 1825 700 50  0000 L CNN
F 2 "" H 1838 650 50  0000 C CNN
F 3 "" H 1800 800 50  0000 C CNN
	1    1800 800 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 582E946B
P 1800 950
F 0 "#PWR?" H 1800 700 50  0001 C CNN
F 1 "GND" H 1700 950 50  0000 C CNN
F 2 "" H 1800 950 50  0000 C CNN
F 3 "" H 1800 950 50  0000 C CNN
	1    1800 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 650  1800 650 
$EndSCHEMATC
