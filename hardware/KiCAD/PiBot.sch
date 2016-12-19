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
L MC33926PNB U?
U 1 1 58301234
P 4650 650
F 0 "U?" H 4800 450 60  0000 C CNN
F 1 "MC33926PNB" V 4650 -150 60  0000 C CNB
F 2 "PiBot:PQFN-32-EP-0.8mm" H 4600 650 60  0001 C CNN
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
F 2 "PiBot:PQFN-32-EP-0.8mm" H 4600 2650 60  0001 C CNN
F 3 "" H 4600 2650 60  0001 C CNN
	1    4650 2650
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
L BH1745NUC U?
U 1 1 5834C6CF
P 2300 4150
F 0 "U?" H 2400 3700 60  0000 C CNN
F 1 "BH1745NUC" H 2550 4100 60  0000 R CNB
F 2 "PiBot:WSON008X2120" H 2300 4150 60  0001 C CNN
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
F 2 "PiBot:WSON008X2120" H 2300 5250 60  0001 C CNN
F 3 "" H 2300 5250 60  0001 C CNN
	1    2300 5250
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
F 1 "+3.3V" H 2950 4500 50  0000 C CNN
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
U 1 1 58363DCB
P 2950 3600
F 0 "#PWR?" H 2950 3350 50  0001 C CNN
F 1 "GND" H 2950 3450 50  0001 C CNN
F 2 "" H 2950 3600 50  0000 C CNN
F 3 "" H 2950 3600 50  0000 C CNN
	1    2950 3600
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 58364113
P 3250 1800
F 0 "#PWR?" H 3250 1650 50  0001 C CNN
F 1 "+3.3V" H 3250 1940 50  0000 C CNN
F 2 "" H 3250 1800 50  0000 C CNN
F 3 "" H 3250 1800 50  0000 C CNN
	1    3250 1800
	1    0    0    -1  
$EndComp
Text GLabel 2950 2300 2    50   BiDi ~ 0
VREF
$Comp
L MolexMicroSD-RESCUE-PiBot U?
U 1 1 5834082C
P 7850 3350
F 0 "U?" H 8250 3600 60  0000 C CNN
F 1 "MolexMicroSD" H 7850 3350 50  0000 C CNB
F 2 "" H 7500 3700 60  0001 C CNN
F 3 "" H 7500 3700 60  0001 C CNN
	1    7850 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58340D24
P 7850 3650
F 0 "#PWR?" H 7850 3400 50  0001 C CNN
F 1 "GND" H 7850 3500 50  0000 C CNN
F 2 "" H 7850 3650 50  0000 C CNN
F 3 "" H 7850 3650 50  0000 C CNN
	1    7850 3650
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5834102E
P 7850 3050
F 0 "#PWR?" H 7850 2900 50  0001 C CNN
F 1 "+3.3V" H 7700 3100 50  0000 C CNN
F 2 "" H 7850 3050 50  0000 C CNN
F 3 "" H 7850 3050 50  0000 C CNN
	1    7850 3050
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
Text GLabel 7350 1750 0    50   BiDi ~ 0
SDA
Text GLabel 7350 1600 0    50   Input ~ 0
SCL
Text GLabel 2800 5450 2    50   Output ~ 0
COLOR_INT_B
Text GLabel 2900 4350 2    50   Output ~ 0
COLOR_INT_A
$Comp
L R R?
U 1 1 583A7150
P 1050 6900
F 0 "R?" V 1130 6900 50  0000 C CNN
F 1 "330R" V 1050 6900 50  0000 C CNN
F 2 "" V 980 6900 50  0000 C CNN
F 3 "" H 1050 6900 50  0000 C CNN
	1    1050 6900
	1    0    0    -1  
$EndComp
$Comp
L LED D?
U 1 1 583A71A9
P 1050 7250
F 0 "D?" H 1050 7350 50  0000 C CNN
F 1 "LED" H 1050 7150 50  0000 C CNN
F 2 "" H 1050 7250 50  0000 C CNN
F 3 "" H 1050 7250 50  0000 C CNN
	1    1050 7250
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 583A722B
P 1050 7450
F 0 "#PWR?" H 1050 7200 50  0001 C CNN
F 1 "GND" H 1050 7300 50  0001 C CNN
F 2 "" H 1050 7450 50  0000 C CNN
F 3 "" H 1050 7450 50  0000 C CNN
	1    1050 7450
	1    0    0    -1  
$EndComp
Text GLabel 950  6650 0    60   Input ~ 0
MP_LED
Text Notes 700  6500 0    60   ~ 0
LED 1\n
Text Notes 7350 7500 0    60   ~ 12
PycoBot (Teensy uPython Robot)
Text Notes 7050 7050 0    60   ~ 12
By:\nBroderick Carlin\nTyler Holmes
Text Notes 7950 7050 0    60   ~ 12
MIT License\n
$Comp
L GND #PWR?
U 1 1 5857395C
P 3350 2400
F 0 "#PWR?" H 3350 2150 50  0001 C CNN
F 1 "GND" H 3350 2250 50  0001 C CNN
F 2 "" H 3350 2400 50  0000 C CNN
F 3 "" H 3350 2400 50  0000 C CNN
	1    3350 2400
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58574365
P 3950 2100
F 0 "C?" H 3950 2200 50  0000 L CNN
F 1 "0.1uF" V 3900 1850 50  0000 L CNN
F 2 "" H 3988 1950 50  0000 C CNN
F 3 "" H 3950 2100 50  0000 C CNN
	1    3950 2100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58574401
P 4250 2300
F 0 "#PWR?" H 4250 2050 50  0001 C CNN
F 1 "GND" H 4250 2150 50  0001 C CNN
F 2 "" H 4250 2300 50  0000 C CNN
F 3 "" H 4250 2300 50  0000 C CNN
	1    4250 2300
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 585744BB
P 3950 4100
F 0 "C?" H 3850 4200 50  0000 L CNN
F 1 "0.1uF" H 3700 4000 50  0000 L CNN
F 2 "" H 3988 3950 50  0000 C CNN
F 3 "" H 3950 4100 50  0000 C CNN
	1    3950 4100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58574621
P 4250 4300
F 0 "#PWR?" H 4250 4050 50  0001 C CNN
F 1 "GND" H 4250 4150 50  0000 C CNN
F 2 "" H 4250 4300 50  0000 C CNN
F 3 "" H 4250 4300 50  0000 C CNN
	1    4250 4300
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 58574979
P 3650 1750
F 0 "R?" V 3550 1750 50  0000 C CNN
F 1 "10k" V 3650 1750 50  0000 C CNN
F 2 "" V 3580 1750 50  0000 C CNN
F 3 "" H 3650 1750 50  0000 C CNN
	1    3650 1750
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 585749CA
P 3650 1600
F 0 "#PWR?" H 3650 1450 50  0001 C CNN
F 1 "+3.3V" H 3500 1650 50  0000 C CNN
F 2 "" H 3650 1600 50  0000 C CNN
F 3 "" H 3650 1600 50  0000 C CNN
	1    3650 1600
	1    0    0    -1  
$EndComp
Text GLabel 3600 1900 0    39   Output ~ 0
~MotorSF
$Comp
L R R?
U 1 1 58574C41
P 3750 900
F 0 "R?" V 3830 900 50  0000 C CNN
F 1 "10k" V 3750 900 50  0000 C CNN
F 2 "" V 3680 900 50  0000 C CNN
F 3 "" H 3750 900 50  0000 C CNN
	1    3750 900 
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 58574C93
P 3750 750
F 0 "#PWR?" H 3750 600 50  0001 C CNN
F 1 "+3.3V" H 3600 800 50  0000 C CNN
F 2 "" H 3750 750 50  0000 C CNN
F 3 "" H 3750 750 50  0000 C CNN
	1    3750 750 
	1    0    0    -1  
$EndComp
NoConn ~ 4300 1250
NoConn ~ 4300 3250
$Comp
L R R?
U 1 1 58574F6B
P 4250 2100
F 0 "R?" V 4330 2100 50  0000 C CNN
F 1 "10k" V 4250 2100 50  0000 C CNN
F 2 "" V 4180 2100 50  0000 C CNN
F 3 "" H 4250 2100 50  0000 C CNN
	1    4250 2100
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 585750A5
P 4250 4100
F 0 "R?" V 4330 4100 50  0000 C CNN
F 1 "10k" V 4250 4100 50  0000 C CNN
F 2 "" V 4180 4100 50  0000 C CNN
F 3 "" H 4250 4100 50  0000 C CNN
	1    4250 4100
	1    0    0    -1  
$EndComp
$Comp
L Feather-Adaper P?
U 1 1 585759D3
P 2050 2750
F 0 "P?" H 2150 3550 60  0000 C CNN
F 1 "Feather-Adaper" V 2050 2800 60  0000 C CNB
F 2 "" H 1750 3150 60  0001 C CNN
F 3 "" H 1750 3150 60  0001 C CNN
	1    2050 2750
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 585766A1
P 4100 1800
F 0 "R?" V 3950 1800 50  0000 C CNN
F 1 "0R" V 4100 1800 50  0000 C CNN
F 2 "" V 4030 1800 50  0000 C CNN
F 3 "" H 4100 1800 50  0000 C CNN
	1    4100 1800
	0    1    1    0   
$EndComp
Text GLabel 3850 1750 1    60   Output ~ 0
FB_L
$Comp
L R R?
U 1 1 5857794E
P 4100 3800
F 0 "R?" V 3950 3800 50  0000 C CNN
F 1 "0R" V 4100 3800 50  0000 C CNN
F 2 "" V 4030 3800 50  0000 C CNN
F 3 "" H 4100 3800 50  0000 C CNN
	1    4100 3800
	0    1    1    0   
$EndComp
Text GLabel 3800 3750 1    60   Output ~ 0
FB_R
Text GLabel 700  3200 0    50   BiDi ~ 0
SDA
Text GLabel 700  3100 0    50   Input ~ 0
SCL
$Comp
L R R?
U 1 1 58579234
P 2950 1950
F 0 "R?" V 2850 1950 50  0000 C CNN
F 1 "10k" V 2950 1950 50  0000 C CNN
F 2 "" V 2880 1950 50  0000 C CNN
F 3 "" H 2950 1950 50  0000 C CNN
	1    2950 1950
	1    0    0    -1  
$EndComp
Text GLabel 3000 2100 2    60   Input ~ 0
~RST
$Comp
L R R?
U 1 1 58579A93
P 1450 6900
F 0 "R?" V 1530 6900 50  0000 C CNN
F 1 "330R" V 1450 6900 50  0000 C CNN
F 2 "" V 1380 6900 50  0000 C CNN
F 3 "" H 1450 6900 50  0000 C CNN
	1    1450 6900
	1    0    0    -1  
$EndComp
$Comp
L LED D?
U 1 1 58579A99
P 1450 7250
F 0 "D?" H 1450 7350 50  0000 C CNN
F 1 "LED" H 1450 7150 50  0000 C CNN
F 2 "" H 1450 7250 50  0000 C CNN
F 3 "" H 1450 7250 50  0000 C CNN
	1    1450 7250
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 58579A9F
P 1450 7450
F 0 "#PWR?" H 1450 7200 50  0001 C CNN
F 1 "GND" H 1450 7300 50  0001 C CNN
F 2 "" H 1450 7450 50  0000 C CNN
F 3 "" H 1450 7450 50  0000 C CNN
	1    1450 7450
	1    0    0    -1  
$EndComp
Text Notes 1300 6500 0    60   ~ 0
PWR
$Comp
L +3.3V #PWR?
U 1 1 58579B71
P 1450 6650
F 0 "#PWR?" H 1450 6500 50  0001 C CNN
F 1 "+3.3V" H 1300 6700 50  0000 C CNN
F 2 "" H 1450 6650 50  0000 C CNN
F 3 "" H 1450 6650 50  0000 C CNN
	1    1450 6650
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 5857B193
P 1000 1950
F 0 "#PWR?" H 1000 1800 50  0001 C CNN
F 1 "VCC" H 1000 2100 50  0000 C CNN
F 2 "" H 1000 1950 50  0000 C CNN
F 3 "" H 1000 1950 50  0000 C CNN
	1    1000 1950
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 5857B1F0
P 2550 900
F 0 "#PWR?" H 2550 750 50  0001 C CNN
F 1 "VCC" H 2650 1000 50  0000 C CNN
F 2 "" H 2550 900 50  0000 C CNN
F 3 "" H 2550 900 50  0000 C CNN
	1    2550 900 
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5857B248
P 1800 900
F 0 "#PWR?" H 1800 750 50  0001 C CNN
F 1 "+5V" H 1700 1000 50  0000 C CNN
F 2 "" H 1800 900 50  0000 C CNN
F 3 "" H 1800 900 50  0000 C CNN
	1    1800 900 
	1    0    0    -1  
$EndComp
$Comp
L F_Small PTC?
U 1 1 5857B32D
P 2000 900
F 0 "PTC?" H 2000 1000 50  0000 C TNN
F 1 "0.5A" H 2000 850 50  0000 C TNN
F 2 "Resistors_SMD:R_1206" H 2000 900 50  0001 C CNN
F 3 "" H 2000 900 50  0000 C CNN
F 4 "0ZCJ0050FF2G" H 2000 900 60  0001 C CNN "MPN"
F 5 "http://www.digikey.com/product-detail/en/bel-fuse-inc/0ZCJ0050FF2G/507-1802-1-ND/4156236" H 2000 900 60  0001 C CNN "DigiKey Link"
	1    2000 900 
	1    0    0    -1  
$EndComp
$Comp
L ZENER D?
U 1 1 5857B8A5
P 2500 1100
F 0 "D?" H 2550 1000 50  0000 C CNN
F 1 "5.1V-1W" H 2550 1200 50  0000 C CNN
F 2 "Diodes_SMD:SMA_Standard" H 2500 1100 50  0001 C CNN
F 3 "" H 2500 1100 50  0000 C CNN
F 4 "SMAZ5V1-13-F" H 2500 1100 60  0001 C CNN "MPN"
F 5 "http://www.digikey.com/product-detail/en/diodes-incorporated/SMAZ5V1-13-F/SMAZ5V1-FDICT-ND/725074" H 2500 1100 60  0001 C CNN "DigiKey Link"
	1    2500 1100
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5857C031
P 2500 1300
F 0 "#PWR?" H 2500 1050 50  0001 C CNN
F 1 "GND" H 2500 1150 50  0001 C CNN
F 2 "" H 2500 1300 50  0000 C CNN
F 3 "" H 2500 1300 50  0000 C CNN
	1    2500 1300
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5857CA92
P 4150 3450
F 0 "R?" V 4230 3450 50  0000 C CNN
F 1 "10k" V 4150 3450 50  0000 C CNN
F 2 "" V 4080 3450 50  0000 C CNN
F 3 "" H 4150 3450 50  0000 C CNN
	1    4150 3450
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5857CBEA
P 4000 3450
F 0 "#PWR?" H 4000 3200 50  0001 C CNN
F 1 "GND" H 4000 3300 50  0001 C CNN
F 2 "" H 4000 3450 50  0000 C CNN
F 3 "" H 4000 3450 50  0000 C CNN
	1    4000 3450
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5857CCD5
P 4150 1450
F 0 "R?" V 4230 1450 50  0000 C CNN
F 1 "10k" V 4150 1450 50  0000 C CNN
F 2 "" V 4080 1450 50  0000 C CNN
F 3 "" H 4150 1450 50  0000 C CNN
	1    4150 1450
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5857CD4D
P 4000 1450
F 0 "#PWR?" H 4000 1200 50  0001 C CNN
F 1 "GND" H 4000 1300 50  0001 C CNN
F 2 "" H 4000 1450 50  0000 C CNN
F 3 "" H 4000 1450 50  0000 C CNN
	1    4000 1450
	1    0    0    -1  
$EndComp
NoConn ~ 4300 3300
NoConn ~ 4300 1300
$Comp
L R R?
U 1 1 5857D573
P 750 2950
F 0 "R?" V 650 2950 50  0000 C CNN
F 1 "10k" V 750 2950 50  0000 C CNN
F 2 "" V 680 2950 50  0000 C CNN
F 3 "" H 750 2950 50  0000 C CNN
	1    750  2950
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5857D5EA
P 850 2900
F 0 "R?" V 930 2900 50  0000 C CNN
F 1 "10k" V 850 2900 50  0000 C CNN
F 2 "" V 780 2900 50  0000 C CNN
F 3 "" H 850 2900 50  0000 C CNN
	1    850  2900
	1    0    0    -1  
$EndComp
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
	2950 2400 3350 2400
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
	2950 2200 3250 2200
Wire Wire Line
	3250 2200 3250 1800
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
Wire Wire Line
	7800 3650 7850 3650
Wire Wire Line
	2800 4350 2900 4350
Wire Wire Line
	1750 5450 1750 5600
Wire Wire Line
	1750 4350 1750 4500
Wire Notes Line
	3600 7800 3600 6000
Wire Notes Line
	3600 6000 450  6000
Wire Wire Line
	1050 6650 1050 6750
Wire Wire Line
	1050 6650 950  6650
Wire Wire Line
	3950 1700 3950 1950
Wire Wire Line
	4000 3700 3950 3750
Wire Wire Line
	3650 3900 4300 3900
Wire Wire Line
	3600 1900 4300 1900
Wire Wire Line
	3650 1900 3650 3900
Connection ~ 3650 1900
Wire Wire Line
	4300 3050 3750 3050
Wire Wire Line
	3750 3050 3750 1050
Wire Wire Line
	3750 1050 4300 1050
Wire Wire Line
	4250 1800 4300 1800
Wire Wire Line
	4250 1800 4250 1950
Wire Wire Line
	4250 3950 4250 3800
Wire Wire Line
	4250 3800 4300 3800
Wire Wire Line
	3950 2250 3950 2300
Wire Wire Line
	3950 2300 4500 2300
Wire Wire Line
	4500 2300 4500 2200
Connection ~ 4250 2300
Wire Wire Line
	4250 2250 4250 2300
Wire Wire Line
	3950 4250 3950 4300
Wire Wire Line
	3950 4300 4500 4300
Wire Wire Line
	4250 4300 4250 4250
Wire Wire Line
	4500 4300 4500 4200
Connection ~ 4250 4300
Connection ~ 4250 1800
Wire Wire Line
	3950 1800 3900 1800
Wire Wire Line
	3900 1800 3850 1750
Wire Wire Line
	4300 1750 4250 1750
Wire Wire Line
	4250 1750 4250 1700
Wire Wire Line
	4250 1700 3950 1700
Wire Wire Line
	4300 3750 4250 3750
Wire Wire Line
	4250 3750 4200 3700
Wire Wire Line
	4200 3700 4000 3700
Wire Wire Line
	3800 3800 3800 3750
Wire Wire Line
	3950 3750 3950 3950
Wire Wire Line
	3950 3800 3800 3800
Wire Wire Line
	3250 1800 2950 1800
Wire Wire Line
	2950 2100 3000 2100
Wire Wire Line
	1450 6650 1450 6750
Wire Wire Line
	1800 900  1900 900 
Wire Wire Line
	700  3100 1150 3100
Wire Wire Line
	700  3200 1150 3200
Wire Wire Line
	850  3050 850  3200
Connection ~ 850  3200
Connection ~ 750  3100
$Comp
L +3.3V #PWR?
U 1 1 5857E15B
P 750 2750
F 0 "#PWR?" H 750 2600 50  0001 C CNN
F 1 "+3.3V" H 750 2890 50  0000 C CNN
F 2 "" H 750 2750 50  0000 C CNN
F 3 "" H 750 2750 50  0000 C CNN
	1    750  2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  2800 750  2750
Wire Wire Line
	750  2750 850  2750
Wire Wire Line
	2450 900  2550 900 
Connection ~ 2500 900 
Wire Wire Line
	2100 900  2150 900 
$Comp
L D_Schottky D?
U 1 1 5857EBB6
P 2300 900
F 0 "D?" H 2300 1000 50  0000 C CNN
F 1 "Schottky" H 2300 800 50  0000 C CNN
F 2 "Diodes_SMD:SOD-323" H 2300 900 50  0001 C CNN
F 3 "" H 2300 900 50  0000 C CNN
F 4 "CUS10S30H3FCT-ND" H 2300 900 60  0001 C CNN "MPN"
F 5 "230mV @ 100mA" H 2300 900 60  0001 C CNN "Vf"
	1    2300 900 
	-1   0    0    1   
$EndComp
NoConn ~ 1150 2100
Wire Wire Line
	1150 2300 1000 2300
Wire Wire Line
	1000 2300 1000 1950
Wire Wire Line
	4300 1350 4300 1400
Wire Wire Line
	4200 1350 4300 1350
Wire Wire Line
	4300 1200 4250 1200
Wire Wire Line
	4250 1200 4250 1250
Wire Wire Line
	4250 1250 4200 1250
Wire Wire Line
	4300 1150 4200 1150
Text GLabel 4200 1150 0    39   Input ~ 0
MotorL_In2
Text GLabel 4200 1250 0    39   Input ~ 0
MotorL_In1
Text GLabel 4200 1350 0    39   Input ~ 0
~MotorL_D
Wire Wire Line
	4300 3400 4300 3350
Wire Wire Line
	4300 3350 4200 3350
Wire Wire Line
	4300 3200 4250 3200
Wire Wire Line
	4250 3200 4250 3250
Wire Wire Line
	4250 3250 4200 3250
Wire Wire Line
	4300 3150 4200 3150
Text GLabel 4200 3150 0    39   Input ~ 0
MotorR_In2
Text GLabel 4200 3250 0    39   Input ~ 0
MotorR_In1
Text GLabel 4200 3350 0    39   Input ~ 0
~MotorR_D
$EndSCHEMATC
