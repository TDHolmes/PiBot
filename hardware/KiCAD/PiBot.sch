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
LIBS:PiBot
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
NoConn ~ 3150 1250
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
$Comp
L MC33926PNB U?
U 1 1 58301234
P 4650 650
F 0 "U?" H 4800 450 60  0000 C CNN
F 1 "MC33926PNB" V 4650 -150 60  0000 C CNB
F 2 "KiCAD_lib:PQFN-32-EP-0.8mm" H 4600 650 60  0001 C CNN
F 3 "" H 4600 650 60  0001 C CNN
	1    4650 650 
	1    0    0    -1  
$EndComp
$Comp
L MC33926PNB U?
U 1 1 5830127C
P 4650 2650
F 0 "U?" H 4800 2450 60  0000 C CNN
F 1 "MC33926PNB" V 4650 1850 60  0000 C CNB
F 2 "KiCAD_lib:PQFN-32-EP-0.8mm" H 4600 2650 60  0001 C CNN
F 3 "" H 4600 2650 60  0001 C CNN
	1    4650 2650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5830134B
P 4500 2200
F 0 "#PWR?" H 4500 1950 50  0001 C CNN
F 1 "GND" H 4500 2050 50  0000 C CNN
F 2 "" H 4500 2200 50  0000 C CNN
F 3 "" H 4500 2200 50  0000 C CNN
	1    4500 2200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 583017E9
P 4500 4200
F 0 "#PWR?" H 4500 3950 50  0001 C CNN
F 1 "GND" H 4500 4050 50  0000 C CNN
F 2 "" H 4500 4200 50  0000 C CNN
F 3 "" H 4500 4200 50  0000 C CNN
	1    4500 4200
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR?
U 1 1 58301A5C
P 4550 2800
F 0 "#PWR?" H 4550 2650 50  0001 C CNN
F 1 "+BATT" H 4550 2940 50  0000 C CNN
F 2 "" H 4550 2800 50  0000 C CNN
F 3 "" H 4550 2800 50  0000 C CNN
	1    4550 2800
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR?
U 1 1 58301B18
P 4550 800
F 0 "#PWR?" H 4550 650 50  0001 C CNN
F 1 "+BATT" H 4550 940 50  0000 C CNN
F 2 "" H 4550 800 50  0000 C CNN
F 3 "" H 4550 800 50  0000 C CNN
	1    4550 800 
	1    0    0    -1  
$EndComp
$Comp
L Motor U?
U 1 1 58327243
P 5500 1300
F 0 "U?" H 5450 1300 60  0000 C CNN
F 1 "Motor" H 5350 950 60  0000 C CNN
F 2 "" H 5500 1300 60  0001 C CNN
F 3 "" H 5500 1300 60  0001 C CNN
	1    5500 1300
	1    0    0    -1  
$EndComp
$Comp
L Motor U?
U 1 1 5832728C
P 5450 3300
F 0 "U?" H 5400 3300 60  0000 C CNN
F 1 "Motor" H 5300 2950 60  0000 C CNN
F 2 "" H 5450 3300 60  0001 C CNN
F 3 "" H 5450 3300 60  0001 C CNN
	1    5450 3300
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58328431
P 6050 1450
F 0 "C?" H 6075 1550 50  0000 L CNN
F 1 "0.1uF" H 6075 1350 50  0000 L CNN
F 2 "" H 6088 1300 50  0000 C CNN
F 3 "" H 6050 1450 50  0000 C CNN
	1    6050 1450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 58328A1C
P 6050 1300
F 0 "#PWR?" H 6050 1150 50  0001 C CNN
F 1 "+3.3V" H 5900 1350 50  0000 C CNN
F 2 "" H 6050 1300 50  0000 C CNN
F 3 "" H 6050 1300 50  0000 C CNN
	1    6050 1300
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58328DC2
P 6000 3450
F 0 "C?" H 6025 3550 50  0000 L CNN
F 1 "0.1uF" H 6025 3350 50  0000 L CNN
F 2 "" H 6038 3300 50  0000 C CNN
F 3 "" H 6000 3450 50  0000 C CNN
	1    6000 3450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 58328DD4
P 6000 3300
F 0 "#PWR?" H 6000 3150 50  0001 C CNN
F 1 "+3.3V" H 5850 3350 50  0000 C CNN
F 2 "" H 6000 3300 50  0000 C CNN
F 3 "" H 6000 3300 50  0000 C CNN
	1    6000 3300
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR?
U 1 1 58329B83
P 4800 2200
F 0 "#PWR?" H 4800 2000 50  0001 C CNN
F 1 "GNDPWR" H 4800 2070 50  0000 C CNN
F 2 "" H 4800 2150 50  0000 C CNN
F 3 "" H 4800 2150 50  0000 C CNN
	1    4800 2200
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR?
U 1 1 58329C25
P 4800 4200
F 0 "#PWR?" H 4800 4000 50  0001 C CNN
F 1 "GNDPWR" H 4800 4070 50  0000 C CNN
F 2 "" H 4800 4150 50  0000 C CNN
F 3 "" H 4800 4150 50  0000 C CNN
	1    4800 4200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58329E30
P 6000 3600
F 0 "#PWR?" H 6000 3350 50  0001 C CNN
F 1 "GND" H 6000 3450 50  0000 C CNN
F 2 "" H 6000 3600 50  0000 C CNN
F 3 "" H 6000 3600 50  0000 C CNN
	1    6000 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58329EF2
P 6050 1600
F 0 "#PWR?" H 6050 1350 50  0001 C CNN
F 1 "GND" H 6050 1450 50  0000 C CNN
F 2 "" H 6050 1600 50  0000 C CNN
F 3 "" H 6050 1600 50  0000 C CNN
	1    6050 1600
	1    0    0    -1  
$EndComp
Text GLabel 5700 1900 2    60   Input ~ 0
left_encB
Text GLabel 5700 1100 2    60   Input ~ 0
left_encA
Text GLabel 5650 3100 2    60   Input ~ 0
right_encA
Text GLabel 5650 3900 2    60   Input ~ 0
right_encB
$Comp
L MK20DX64VLH7 U?
U 1 1 5833AEE0
P 2250 2050
F 0 "U?" H 2900 3000 60  0000 C CNN
F 1 "MK20DX64VLH7" H 2300 2250 60  0000 C CNB
F 2 "" H 2250 2050 60  0001 C CNN
F 3 "" H 2250 2050 60  0001 C CNN
	1    2250 2050
	1    0    0    -1  
$EndComp
$Comp
L BH1745NUC U?
U 1 1 5834C6CF
P 2300 4150
F 0 "U?" H 2400 3700 60  0000 C CNN
F 1 "BH1745NUC" H 2550 4100 60  0000 R CNB
F 2 "" H 2300 4150 60  0001 C CNN
F 3 "" H 2300 4150 60  0001 C CNN
	1    2300 4150
	1    0    0    -1  
$EndComp
$Comp
L BH1745NUC U?
U 1 1 5834C70B
P 2300 5250
F 0 "U?" H 2400 4800 60  0000 C CNN
F 1 "BH1745NUC" H 2550 5200 60  0000 R CNB
F 2 "" H 2300 5250 60  0001 C CNN
F 3 "" H 2300 5250 60  0001 C CNN
	1    2300 5250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5834C7AD
P 1750 5450
F 0 "#PWR?" H 1750 5200 50  0001 C CNN
F 1 "GND" H 1750 5300 50  0001 C CNN
F 2 "" H 1750 5450 50  0000 C CNN
F 3 "" H 1750 5450 50  0000 C CNN
	1    1750 5450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5834CA77
P 1750 3950
F 0 "#PWR?" H 1750 3800 50  0001 C CNN
F 1 "+3.3V" H 1600 4000 50  0000 C CNN
F 2 "" H 1750 3950 50  0000 C CNN
F 3 "" H 1750 3950 50  0000 C CNN
	1    1750 3950
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5834CAFA
P 1750 5050
F 0 "#PWR?" H 1750 4900 50  0001 C CNN
F 1 "+3.3V" H 1600 5100 50  0000 C CNN
F 2 "" H 1750 5050 50  0000 C CNN
F 3 "" H 1750 5050 50  0000 C CNN
	1    1750 5050
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5834CB23
P 2800 4500
F 0 "#PWR?" H 2800 4350 50  0001 C CNN
F 1 "+3.3V" H 2950 4550 50  0000 C CNN
F 2 "" H 2800 4500 50  0000 C CNN
F 3 "" H 2800 4500 50  0000 C CNN
	1    2800 4500
	1    0    0    -1  
$EndComp
Text GLabel 2800 3950 2    50   BiDi ~ 0
SDA
Text GLabel 2800 4050 2    50   Input ~ 0
SCL
Text GLabel 2800 5150 2    50   Input ~ 0
SCL
Text GLabel 2800 5050 2    50   BiDi ~ 0
SDA
$Comp
L GND #PWR?
U 1 1 5834D682
P 1750 5600
F 0 "#PWR?" H 1750 5350 50  0001 C CNN
F 1 "GND" H 1750 5450 50  0001 C CNN
F 2 "" H 1750 5600 50  0000 C CNN
F 3 "" H 1750 5600 50  0000 C CNN
	1    1750 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5834D6AB
P 2800 5600
F 0 "#PWR?" H 2800 5350 50  0001 C CNN
F 1 "GND" H 2800 5450 50  0001 C CNN
F 2 "" H 2800 5600 50  0000 C CNN
F 3 "" H 2800 5600 50  0000 C CNN
	1    2800 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5834D6D4
P 1750 4500
F 0 "#PWR?" H 1750 4250 50  0001 C CNN
F 1 "GND" H 1750 4350 50  0001 C CNN
F 2 "" H 1750 4500 50  0000 C CNN
F 3 "" H 1750 4500 50  0000 C CNN
	1    1750 4500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5834D6FD
P 1750 4350
F 0 "#PWR?" H 1750 4100 50  0001 C CNN
F 1 "GND" H 1750 4200 50  0001 C CNN
F 2 "" H 1750 4350 50  0000 C CNN
F 3 "" H 1750 4350 50  0000 C CNN
	1    1750 4350
	1    0    0    -1  
$EndComp
$Comp
L Teensy-Feather-Adaper P?
U 1 1 58363CA3
P 5200 5850
F 0 "P?" H 5300 6650 60  0000 C CNN
F 1 "Teensy-Feather-Adaper" V 5150 5850 60  0000 C CNB
F 2 "" H 4900 6250 60  0001 C CNN
F 3 "" H 4900 6250 60  0001 C CNN
	1    5200 5850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58363DCB
P 6100 6700
F 0 "#PWR?" H 6100 6450 50  0001 C CNN
F 1 "GND" H 6100 6550 50  0001 C CNN
F 2 "" H 6100 6700 50  0000 C CNN
F 3 "" H 6100 6700 50  0000 C CNN
	1    6100 6700
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 58363F96
P 4200 5050
F 0 "#PWR?" H 4200 4900 50  0001 C CNN
F 1 "+5V" H 4100 5150 50  0000 C CNN
F 2 "" H 4200 5050 50  0000 C CNN
F 3 "" H 4200 5050 50  0000 C CNN
	1    4200 5050
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 58364113
P 6200 5050
F 0 "#PWR?" H 6200 4900 50  0001 C CNN
F 1 "+3.3V" H 6200 5190 50  0000 C CNN
F 2 "" H 6200 5050 50  0000 C CNN
F 3 "" H 6200 5050 50  0000 C CNN
	1    6200 5050
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR?
U 1 1 5836421A
P 4300 5050
F 0 "#PWR?" H 4300 4900 50  0001 C CNN
F 1 "+BATT" H 4450 5100 50  0000 C CNN
F 2 "" H 4300 5050 50  0000 C CNN
F 3 "" H 4300 5050 50  0000 C CNN
	1    4300 5050
	1    0    0    -1  
$EndComp
Text GLabel 3150 1950 2    25   BiDi ~ 0
PTC2
Text GLabel 6100 6000 2    50   BiDi ~ 0
PTD1
Text GLabel 3150 2550 2    25   BiDi ~ 0
PTD1
Text GLabel 6100 5900 2    50   BiDi ~ 0
PTB0
Text GLabel 6100 5800 2    50   BiDi ~ 0
PTB1
Text GLabel 6100 5700 2    50   BiDi ~ 0
PTC0
Text GLabel 6100 5600 2    50   BiDi ~ 0
DAC0
Text GLabel 6100 6100 2    50   BiDi ~ 0
PTD5
Text GLabel 4300 6100 0    50   BiDi ~ 0
PTD3
Text GLabel 4300 6000 0    50   BiDi ~ 0
PTA12
Text GLabel 4300 5900 0    50   BiDi ~ 0
PTA13
Text GLabel 4300 5800 0    50   BiDi ~ 0
PTC4
Text GLabel 4300 5700 0    50   BiDi ~ 0
PTC3
Text GLabel 4300 5600 0    50   BiDi ~ 0
PTD4
Text GLabel 4300 5500 0    50   BiDi ~ 0
PTD7
Text GLabel 4150 5300 0    50   BiDi ~ 0
PTD0
Text GLabel 6100 5400 2    50   BiDi ~ 0
VREFH
Wire Wire Line
	6100 5500 6500 5500
Wire Wire Line
	4300 5300 4150 5300
Wire Wire Line
	2050 3000 2400 3000
Connection ~ 2100 3000
Wire Wire Line
	2150 1050 2150 650 
Wire Wire Line
	1950 1050 2150 1050
Connection ~ 2100 1050
Connection ~ 2050 1050
Connection ~ 2150 3000
Wire Wire Line
	2150 650  1800 650 
Wire Wire Line
	4600 2200 4850 2200
Connection ~ 4800 2200
Connection ~ 4750 2200
Connection ~ 4700 2200
Connection ~ 4650 2200
Wire Wire Line
	4500 2200 4450 2200
Wire Wire Line
	4600 4200 4850 4200
Connection ~ 4800 4200
Connection ~ 4750 4200
Connection ~ 4700 4200
Connection ~ 4650 4200
Wire Wire Line
	4450 4200 4500 4200
Wire Wire Line
	4500 800  4650 800 
Connection ~ 4550 800 
Connection ~ 4600 800 
Wire Wire Line
	4500 2800 4650 2800
Connection ~ 4550 2800
Connection ~ 4600 2800
Connection ~ 5000 1250
Connection ~ 5000 1300
Connection ~ 5000 1700
Connection ~ 5000 1750
Connection ~ 5000 1350
Connection ~ 5000 1650
Wire Wire Line
	5000 3550 5000 3800
Connection ~ 5000 3750
Connection ~ 5000 3700
Wire Wire Line
	5000 3550 5050 3550
Connection ~ 5000 3650
Wire Wire Line
	5000 3400 5050 3400
Wire Wire Line
	5000 3200 5000 3400
Connection ~ 5000 3350
Connection ~ 5000 3300
Connection ~ 5000 3250
Wire Wire Line
	5000 1200 5000 1400
Wire Wire Line
	5000 1400 5100 1400
Wire Wire Line
	5100 1550 5000 1550
Wire Wire Line
	5000 1550 5000 1800
Wire Wire Line
	5600 1550 5850 1550
Wire Wire Line
	5850 1550 5850 1600
Wire Wire Line
	5850 1600 6050 1600
Wire Wire Line
	5600 1400 5850 1400
Wire Wire Line
	5850 1400 5850 1300
Wire Wire Line
	5850 1300 6050 1300
Wire Wire Line
	5550 3550 5800 3550
Wire Wire Line
	5800 3550 5800 3600
Wire Wire Line
	5800 3600 6000 3600
Wire Wire Line
	5550 3400 5800 3400
Wire Wire Line
	5800 3400 5800 3300
Wire Wire Line
	5800 3300 6000 3300
Wire Wire Line
	5600 1450 5650 1450
Wire Wire Line
	5650 1450 5650 1100
Wire Wire Line
	5650 1100 5700 1100
Wire Wire Line
	5600 1500 5650 1500
Wire Wire Line
	5650 1500 5650 1900
Wire Wire Line
	5650 1900 5700 1900
Wire Wire Line
	5550 3450 5600 3450
Wire Wire Line
	5600 3450 5600 3100
Wire Wire Line
	5600 3100 5650 3100
Wire Wire Line
	5550 3500 5600 3500
Wire Wire Line
	5600 3500 5600 3900
Wire Wire Line
	5600 3900 5650 3900
Wire Wire Line
	4300 5400 4200 5400
Wire Wire Line
	4200 5400 4200 5050
Wire Wire Line
	6100 5300 6200 5300
Wire Wire Line
	6200 5300 6200 5050
Wire Wire Line
	4300 5200 4300 5050
$Comp
L GND #PWR?
U 1 1 58364E98
P 6500 5500
F 0 "#PWR?" H 6500 5250 50  0001 C CNN
F 1 "GND" H 6500 5350 50  0001 C CNN
F 2 "" H 6500 5500 50  0000 C CNN
F 3 "" H 6500 5500 50  0000 C CNN
	1    6500 5500
	1    0    0    -1  
$EndComp
$Comp
L LIS3MDL U?
U 1 1 5833E450
P 7900 1800
F 0 "U?" H 8100 2100 60  0000 C CNN
F 1 "LIS3MDL" H 7900 1750 60  0000 C CNB
F 2 "" H 7500 2200 60  0001 C CNN
F 3 "" H 7500 2200 60  0001 C CNN
	1    7900 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5833EABA
P 7900 2150
F 0 "#PWR?" H 7900 1900 50  0001 C CNN
F 1 "GND" H 7900 2000 50  0000 C CNN
F 2 "" H 7900 2150 50  0000 C CNN
F 3 "" H 7900 2150 50  0000 C CNN
	1    7900 2150
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5833EDD8
P 7850 1100
F 0 "#PWR?" H 7850 950 50  0001 C CNN
F 1 "+3.3V" H 7700 1150 50  0000 C CNN
F 2 "" H 7850 1100 50  0000 C CNN
F 3 "" H 7850 1100 50  0000 C CNN
	1    7850 1100
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5833F06B
P 7550 1300
F 0 "C?" H 7575 1400 50  0000 L CNN
F 1 "0.1uF" H 7575 1200 50  0000 L CNN
F 2 "" H 7588 1150 50  0000 C CNN
F 3 "" H 7550 1300 50  0000 C CNN
	1    7550 1300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5833F21B
P 7550 1450
F 0 "#PWR?" H 7550 1200 50  0001 C CNN
F 1 "GND" H 7550 1300 50  0000 C CNN
F 2 "" H 7550 1450 50  0000 C CNN
F 3 "" H 7550 1450 50  0000 C CNN
	1    7550 1450
	1    0    0    -1  
$EndComp
Text GLabel 3150 2500 2    25   BiDi ~ 0
PTD0
Text GLabel 3150 2750 2    25   BiDi ~ 0
PTD5
Text GLabel 3150 2650 2    25   BiDi ~ 0
PTD3
Text GLabel 3150 2700 2    25   BiDi ~ 0
PTD4
Text GLabel 3150 2850 2    25   BiDi ~ 0
PTD7
Text GLabel 3150 1350 2    25   BiDi ~ 0
PTB0
Text GLabel 3150 1400 2    25   BiDi ~ 0
PTB1
Text GLabel 1550 2150 0    25   BiDi ~ 0
PTA12
Text GLabel 1550 2200 0    25   BiDi ~ 0
PTA13
Text GLabel 1550 2450 0    25   BiDi ~ 0
DAC0
Text GLabel 2650 1050 1    25   BiDi ~ 0
VREFH
$Comp
L GND #PWR?
U 1 1 5836562F
P 2800 950
F 0 "#PWR?" H 2800 700 50  0001 C CNN
F 1 "GND" H 2900 950 50  0000 C CNN
F 2 "" H 2800 950 50  0000 C CNN
F 3 "" H 2800 950 50  0000 C CNN
	1    2800 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 1050 2700 900 
Wire Wire Line
	2700 900  2800 900 
Wire Wire Line
	2800 900  2800 950 
Text GLabel 1550 2550 0    25   BiDi ~ 0
~RESET
$Comp
L Crystal Y?
U 1 1 58367D50
P 1100 2300
F 0 "Y?" H 1100 2450 50  0000 C CNN
F 1 "16 MHz" H 1100 2150 50  0000 C CNN
F 2 "" H 1100 2300 50  0000 C CNN
F 3 "" H 1100 2300 50  0000 C CNN
	1    1100 2300
	1    0    0    1   
$EndComp
$Comp
L Crystal Y?
U 1 1 58367F1E
P 1100 2750
F 0 "Y?" H 1100 2900 50  0000 C CNN
F 1 "32 kHz" H 1100 2600 50  0000 C CNN
F 2 "" H 1100 2750 50  0000 C CNN
F 3 "" H 1100 2750 50  0000 C CNN
	1    1100 2750
	1    0    0    1   
$EndComp
Wire Wire Line
	1250 2750 1550 2750
Wire Wire Line
	1550 2700 1300 2700
Wire Wire Line
	1300 2700 1300 2550
Wire Wire Line
	1300 2550 950  2550
Wire Wire Line
	950  2550 950  2750
Wire Wire Line
	1250 2300 1550 2300
Wire Wire Line
	1550 2250 1300 2250
Wire Wire Line
	1300 2250 1300 2100
Wire Wire Line
	1300 2100 950  2100
Wire Wire Line
	950  2100 950  2300
$Comp
L USB_OTG P?
U 1 1 5836866A
P 950 800
F 0 "P?" H 1275 675 50  0000 C CNN
F 1 "USB_OTG" H 950 1000 50  0000 C CNN
F 2 "" V 900 700 50  0000 C CNN
F 3 "" V 900 700 50  0000 C CNN
	1    950  800 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58368742
P 1150 1100
F 0 "#PWR?" H 1150 850 50  0001 C CNN
F 1 "GND" H 1250 1100 50  0000 C CNN
F 2 "" H 1150 1100 50  0000 C CNN
F 3 "" H 1150 1100 50  0000 C CNN
	1    1150 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 1300 850  1300
Wire Wire Line
	850  1300 850  1100
Wire Wire Line
	1550 1250 950  1250
Wire Wire Line
	950  1250 950  1100
$Comp
L +5V #PWR?
U 1 1 58368D02
P 600 1000
F 0 "#PWR?" H 600 850 50  0001 C CNN
F 1 "+5V" H 550 1150 50  0000 C CNN
F 2 "" H 600 1000 50  0000 C CNN
F 3 "" H 600 1000 50  0000 C CNN
	1    600  1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  1100 750  1150
Wire Wire Line
	750  1150 600  1150
Wire Wire Line
	600  1150 600  1000
NoConn ~ 1050 1100
NoConn ~ 1350 700 
Wire Wire Line
	7850 2150 7950 2150
Connection ~ 7900 2150
Wire Wire Line
	7850 1450 7950 1450
Wire Wire Line
	7850 1450 7850 1100
Wire Wire Line
	7850 1100 7550 1100
Wire Wire Line
	7550 1100 7550 1150
Text GLabel 7350 1750 0    50   BiDi ~ 0
SDA
Text GLabel 7350 1600 0    50   Input ~ 0
SCL
Wire Wire Line
	7350 1600 7450 1600
Wire Wire Line
	7450 1600 7450 1650
Wire Wire Line
	7450 1650 7550 1650
Wire Wire Line
	7550 1700 7450 1700
Wire Wire Line
	7450 1700 7450 1750
Wire Wire Line
	7450 1750 7350 1750
$EndSCHEMATC
