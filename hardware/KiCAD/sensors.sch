EESchema Schematic File Version 2
LIBS:PiBot-rescue
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
LIBS:74xgxx
LIBS:ac-dc
LIBS:actel
LIBS:allegro
LIBS:Altera
LIBS:analog_devices
LIBS:battery_management
LIBS:bbd
LIBS:bosch
LIBS:brooktre
LIBS:cmos_ieee
LIBS:dc-dc
LIBS:diode
LIBS:elec-unifil
LIBS:ESD_Protection
LIBS:ftdi
LIBS:gennum
LIBS:graphic
LIBS:hc11
LIBS:ir
LIBS:Lattice
LIBS:logo
LIBS:maxim
LIBS:mechanical
LIBS:microchip_dspic33dsc
LIBS:microchip_pic10mcu
LIBS:microchip_pic12mcu
LIBS:microchip_pic16mcu
LIBS:microchip_pic18mcu
LIBS:microchip_pic32mcu
LIBS:motor_drivers
LIBS:motors
LIBS:msp430
LIBS:nordicsemi
LIBS:nxp_armmcu
LIBS:onsemi
LIBS:Oscillators
LIBS:powerint
LIBS:Power_Management
LIBS:pspice
LIBS:references
LIBS:relays
LIBS:rfcom
LIBS:sensors
LIBS:silabs
LIBS:stm8
LIBS:stm32
LIBS:supertex
LIBS:switches
LIBS:transf
LIBS:ttl_ieee
LIBS:video
LIBS:wiznet
LIBS:Worldsemi
LIBS:Xicor
LIBS:zetex
LIBS:Zilog
LIBS:PiBot-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
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
L BH1745NUC U?
U 1 1 58759DAD
P 2150 1600
F 0 "U?" H 2250 1150 60  0000 C CNN
F 1 "BH1745NUC" H 2400 1550 60  0000 R CNB
F 2 "PiBot:WSON008X2120" H 2150 1600 60  0001 C CNN
F 3 "" H 2150 1600 60  0001 C CNN
	1    2150 1600
	1    0    0    -1  
$EndComp
$Comp
L BH1745NUC U?
U 1 1 58759DB4
P 2150 3000
F 0 "U?" H 2250 2550 60  0000 C CNN
F 1 "BH1745NUC" H 2400 2950 60  0000 R CNB
F 2 "PiBot:WSON008X2120" H 2150 3000 60  0001 C CNN
F 3 "" H 2150 3000 60  0001 C CNN
	1    2150 3000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58759DD1
P 1600 3350
F 0 "#PWR?" H 1600 3100 50  0001 C CNN
F 1 "GND" H 1600 3200 50  0001 C CNN
F 2 "" H 1600 3350 50  0000 C CNN
F 3 "" H 1600 3350 50  0000 C CNN
	1    1600 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58759DD7
P 2650 3350
F 0 "#PWR?" H 2650 3100 50  0001 C CNN
F 1 "GND" H 2650 3200 50  0001 C CNN
F 2 "" H 2650 3350 50  0000 C CNN
F 3 "" H 2650 3350 50  0000 C CNN
	1    2650 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58759DDD
P 1600 1950
F 0 "#PWR?" H 1600 1700 50  0001 C CNN
F 1 "GND" H 1600 1800 50  0001 C CNN
F 2 "" H 1600 1950 50  0000 C CNN
F 3 "" H 1600 1950 50  0000 C CNN
	1    1600 1950
	1    0    0    -1  
$EndComp
$Comp
L TEST_1P W?
U 1 1 58759DE5
P 2700 2800
F 0 "W?" H 2600 3050 50  0000 L TNN
F 1 "TP" H 2550 2950 50  0000 L CNN
F 2 "" H 2900 2800 50  0000 C CNN
F 3 "" H 2900 2800 50  0000 C CNN
	1    2700 2800
	1    0    0    -1  
$EndComp
$Comp
L TEST_1P W?
U 1 1 58759DEC
P 3050 2900
F 0 "W?" H 3050 3150 50  0000 L TNN
F 1 "TP" H 3100 3050 50  0000 L CNN
F 2 "" H 3250 2900 50  0000 C CNN
F 3 "" H 3250 2900 50  0000 C CNN
	1    3050 2900
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58759DF3
P 1450 1600
F 0 "C?" H 1475 1700 50  0000 L CNN
F 1 "0.1uF" H 1475 1500 50  0000 L CNN
F 2 "" H 1488 1450 50  0000 C CNN
F 3 "" H 1450 1600 50  0000 C CNN
	1    1450 1600
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58759DFA
P 1400 3000
F 0 "C?" H 1425 3100 50  0000 L CNN
F 1 "0.1uF" H 1425 2900 50  0000 L CNN
F 2 "" H 1438 2850 50  0000 C CNN
F 3 "" H 1400 3000 50  0000 C CNN
	1    1400 3000
	1    0    0    -1  
$EndComp
$Comp
L L3GD20H U?
U 1 1 58759E01
P 5450 2150
F 0 "U?" H 5850 2600 60  0000 C CNN
F 1 "L3GD20H" H 5450 2150 60  0000 C CNB
F 2 "" H 4700 2550 60  0001 C CNN
F 3 "" H 4700 2550 60  0001 C CNN
	1    5450 2150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58759E08
P 5800 3000
F 0 "C?" H 5825 3100 50  0000 L CNN
F 1 "10nF" H 5825 2900 50  0000 L CNN
F 2 "" H 5838 2850 50  0000 C CNN
F 3 "" H 5800 3000 50  0000 C CNN
	1    5800 3000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58759E0F
P 5800 3200
F 0 "#PWR?" H 5800 2950 50  0001 C CNN
F 1 "GND" H 5800 3050 50  0001 C CNN
F 2 "" H 5800 3200 50  0000 C CNN
F 3 "" H 5800 3200 50  0000 C CNN
	1    5800 3200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58759E1B
P 6250 1400
F 0 "C?" H 6275 1500 50  0000 L CNN
F 1 "0.1uF" H 6275 1300 50  0000 L CNN
F 2 "" H 6288 1250 50  0000 C CNN
F 3 "" H 6250 1400 50  0000 C CNN
	1    6250 1400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58759E22
P 6000 1600
F 0 "#PWR?" H 6000 1350 50  0001 C CNN
F 1 "GND" H 6000 1450 50  0001 C CNN
F 2 "" H 6000 1600 50  0000 C CNN
F 3 "" H 6000 1600 50  0000 C CNN
	1    6000 1600
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58759E28
P 6000 1400
F 0 "C?" H 6025 1500 50  0000 L CNN
F 1 "1uF" H 6025 1300 50  0000 L CNN
F 2 "" H 6038 1250 50  0000 C CNN
F 3 "" H 6000 1400 50  0000 C CNN
	1    6000 1400
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58759E2F
P 5750 1400
F 0 "C?" H 5775 1500 50  0000 L CNN
F 1 "10uF" H 5775 1300 50  0000 L CNN
F 2 "" H 5788 1250 50  0000 C CNN
F 3 "" H 5750 1400 50  0000 C CNN
	1    5750 1400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58759E38
P 6700 2400
F 0 "#PWR?" H 6700 2150 50  0001 C CNN
F 1 "GND" H 6700 2250 50  0001 C CNN
F 2 "" H 6700 2400 50  0000 C CNN
F 3 "" H 6700 2400 50  0000 C CNN
	1    6700 2400
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58759E4B
P 9300 1400
F 0 "C?" H 9325 1500 50  0000 L CNN
F 1 "10uF" H 9325 1300 50  0000 L CNN
F 2 "" H 9338 1250 50  0000 C CNN
F 3 "" H 9300 1400 50  0000 C CNN
	1    9300 1400
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58759E52
P 9550 1400
F 0 "C?" H 9575 1500 50  0000 L CNN
F 1 "1uF" H 9575 1300 50  0000 L CNN
F 2 "" H 9588 1250 50  0000 C CNN
F 3 "" H 9550 1400 50  0000 C CNN
	1    9550 1400
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58759E59
P 9800 1400
F 0 "C?" H 9825 1500 50  0000 L CNN
F 1 "0.1uF" H 9825 1300 50  0000 L CNN
F 2 "" H 9838 1250 50  0000 C CNN
F 3 "" H 9800 1400 50  0000 C CNN
	1    9800 1400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58759E66
P 9550 1600
F 0 "#PWR?" H 9550 1350 50  0001 C CNN
F 1 "GND" H 9550 1450 50  0001 C CNN
F 2 "" H 9550 1600 50  0000 C CNN
F 3 "" H 9550 1600 50  0000 C CNN
	1    9550 1600
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58759E6E
P 9250 3000
F 0 "C?" H 9275 3100 50  0000 L CNN
F 1 "4.7uF" H 9275 2900 50  0000 L CNN
F 2 "" H 9288 2850 50  0000 C CNN
F 3 "" H 9250 3000 50  0000 C CNN
	1    9250 3000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58759E75
P 8850 3200
F 0 "#PWR?" H 8850 2950 50  0001 C CNN
F 1 "GND" H 8850 3050 50  0001 C CNN
F 2 "" H 8850 3200 50  0000 C CNN
F 3 "" H 8850 3200 50  0000 C CNN
	1    8850 3200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58759E7B
P 10000 2250
F 0 "C?" H 10025 2350 50  0000 L CNN
F 1 "0.22uF" H 10025 2150 50  0000 L CNN
F 2 "" H 10038 2100 50  0000 C CNN
F 3 "" H 10000 2250 50  0000 C CNN
	1    10000 2250
	1    0    0    -1  
$EndComp
Connection ~ 2700 2800
Connection ~ 3050 2900
Wire Wire Line
	2650 2800 2750 2800
Wire Wire Line
	2650 2900 3100 2900
Wire Wire Line
	1600 1800 1600 1950
Wire Wire Line
	1600 3200 1600 3350
Wire Wire Line
	1400 3150 1400 3200
Wire Wire Line
	1400 3200 1600 3200
Wire Wire Line
	1400 2850 1400 2800
Wire Wire Line
	1400 2800 1600 2800
Wire Wire Line
	1600 1800 1450 1800
Wire Wire Line
	1450 1800 1450 1750
Wire Wire Line
	1450 1450 1450 1400
Wire Wire Line
	1450 1400 1600 1400
Wire Wire Line
	5800 2800 5800 2850
Wire Wire Line
	5800 3150 5800 3200
Wire Wire Line
	5800 3200 5500 3200
Wire Wire Line
	5500 3200 5500 2800
Wire Wire Line
	4700 2800 5700 2800
Connection ~ 5600 2800
Connection ~ 5500 2800
Connection ~ 5400 2800
Connection ~ 5300 2800
Wire Wire Line
	4700 1250 6250 1250
Connection ~ 6000 1250
Wire Wire Line
	5400 1200 5400 1550
Connection ~ 5400 1250
Wire Wire Line
	5400 1550 5500 1550
Wire Wire Line
	5750 1550 6250 1550
Connection ~ 6000 1550
Wire Wire Line
	6000 1550 6000 1600
Connection ~ 5750 1250
Wire Wire Line
	4700 2100 4800 2100
Wire Wire Line
	4700 2200 4800 2200
Wire Wire Line
	4800 2300 4700 2300
Wire Wire Line
	4700 2300 4700 2800
Connection ~ 5200 2800
Wire Wire Line
	4800 2000 4700 2000
Wire Wire Line
	4700 2000 4700 1250
Wire Wire Line
	6150 2100 6700 2100
Wire Wire Line
	6700 2050 6700 2150
Connection ~ 6700 2100
Wire Wire Line
	6700 1800 6700 1850
Wire Wire Line
	6700 2350 6700 2400
Wire Wire Line
	9300 1550 9800 1550
Connection ~ 9550 1550
Wire Wire Line
	8950 1250 9800 1250
Connection ~ 9550 1250
Wire Wire Line
	8950 1200 8950 1550
Connection ~ 9300 1250
Connection ~ 8950 1250
Wire Wire Line
	9550 1600 9550 1550
Wire Wire Line
	8750 2800 8950 2800
Connection ~ 8850 2800
Wire Wire Line
	9250 3150 9250 3200
Wire Wire Line
	9250 3200 8850 3200
Wire Wire Line
	9250 2800 9250 2850
Wire Wire Line
	9800 2300 9850 2300
Wire Wire Line
	9850 2300 9850 2400
Wire Wire Line
	9850 2400 10000 2400
Wire Wire Line
	10000 2100 9850 2100
Wire Wire Line
	9850 2100 9850 2200
Wire Wire Line
	9850 2200 9800 2200
NoConn ~ 6150 2300
NoConn ~ 6150 2200
$Comp
L Jumper_NC_Small JP?
U 1 1 58759EF3
P 6700 1950
F 0 "JP?" H 6700 2030 50  0000 C CNN
F 1 "Jumper_NC_Small" H 6710 1890 50  0001 C CNN
F 2 "" H 6700 1950 50  0000 C CNN
F 3 "" H 6700 1950 50  0000 C CNN
	1    6700 1950
	0    -1   -1   0   
$EndComp
$Comp
L Jumper_NO_Small JP?
U 1 1 58759EFA
P 6700 2250
F 0 "JP?" H 6700 2330 50  0000 C CNN
F 1 "Jumper_NO_Small" H 6710 2190 50  0001 C CNN
F 2 "" H 6700 2250 50  0000 C CNN
F 3 "" H 6700 2250 50  0000 C CNN
	1    6700 2250
	0    -1   -1   0   
$EndComp
NoConn ~ 1600 1550
NoConn ~ 1600 2950
Wire Notes Line
	450  3800 11200 3800
Wire Notes Line
	3900 500  3900 6100
Wire Notes Line
	7350 3800 7350 500 
Text Notes 4000 600  0    60   ~ 12
Gyroscope
Text Notes 7500 700  0    60   ~ 12
Accelerometer\nMagnetometer
Text Notes 600  600  0    60   ~ 12
Color Sensors
$Comp
L SW_SPDT SW?
U 1 1 5875CDB3
P 2350 4750
F 0 "SW?" H 2350 4920 50  0000 C CNN
F 1 "SW_SPDT" H 2350 4550 50  0001 C CNN
F 2 "" H 2350 4750 50  0000 C CNN
F 3 "" H 2350 4750 50  0000 C CNN
	1    2350 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5875CDBA
P 2150 5150
F 0 "#PWR?" H 2150 4900 50  0001 C CNN
F 1 "GND" H 2100 5000 50  0001 C CNN
F 2 "" H 2150 5150 50  0000 C CNN
F 3 "" H 2150 5150 50  0000 C CNN
	1    2150 5150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5875CDCB
P 1650 4950
F 0 "C?" H 1675 5050 50  0000 L CNN
F 1 "0.1uF" H 1675 4850 50  0000 L CNN
F 2 "" H 1688 4800 50  0000 C CNN
F 3 "" H 1650 4950 50  0000 C CNN
	1    1650 4950
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5875CDD2
P 2150 4950
F 0 "R?" V 2230 4950 50  0000 C CNN
F 1 "10k" V 2150 4950 50  0000 C CNN
F 2 "" V 2080 4950 50  0000 C CNN
F 3 "" H 2150 4950 50  0000 C CNN
	1    2150 4950
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5875CDDB
P 1900 4750
F 0 "R?" V 1980 4750 50  0000 C CNN
F 1 "10k" V 1900 4750 50  0000 C CNN
F 2 "" V 1830 4750 50  0000 C CNN
F 3 "" H 1900 4750 50  0000 C CNN
	1    1900 4750
	0    1    1    0   
$EndComp
Wire Wire Line
	1450 4750 1750 4750
Wire Wire Line
	1650 4750 1650 4800
Connection ~ 1650 4750
Wire Wire Line
	2050 4750 2150 4750
$Comp
L GND #PWR?
U 1 1 5875CDE7
P 1650 5150
F 0 "#PWR?" H 1650 4900 50  0001 C CNN
F 1 "GND" H 1600 5000 50  0001 C CNN
F 2 "" H 1650 5150 50  0000 C CNN
F 3 "" H 1650 5150 50  0000 C CNN
	1    1650 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 5150 1650 5100
$Comp
L SW_SPDT SW?
U 1 1 5875CDEE
P 2350 5400
F 0 "SW?" H 2350 5570 50  0000 C CNN
F 1 "SW_SPDT" H 2350 5200 50  0001 C CNN
F 2 "" H 2350 5400 50  0000 C CNN
F 3 "" H 2350 5400 50  0000 C CNN
	1    2350 5400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5875CDF5
P 2150 5800
F 0 "#PWR?" H 2150 5550 50  0001 C CNN
F 1 "GND" H 2100 5650 50  0001 C CNN
F 2 "" H 2150 5800 50  0000 C CNN
F 3 "" H 2150 5800 50  0000 C CNN
	1    2150 5800
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5875CE04
P 1650 5600
F 0 "C?" H 1675 5700 50  0000 L CNN
F 1 "0.1uF" H 1675 5500 50  0000 L CNN
F 2 "" H 1688 5450 50  0000 C CNN
F 3 "" H 1650 5600 50  0000 C CNN
	1    1650 5600
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5875CE0B
P 2150 5600
F 0 "R?" V 2230 5600 50  0000 C CNN
F 1 "10k" V 2150 5600 50  0000 C CNN
F 2 "" V 2080 5600 50  0000 C CNN
F 3 "" H 2150 5600 50  0000 C CNN
	1    2150 5600
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5875CE14
P 1900 5400
F 0 "R?" V 1980 5400 50  0000 C CNN
F 1 "10k" V 1900 5400 50  0000 C CNN
F 2 "" V 1830 5400 50  0000 C CNN
F 3 "" H 1900 5400 50  0000 C CNN
	1    1900 5400
	0    1    1    0   
$EndComp
Wire Wire Line
	1450 5400 1750 5400
Wire Wire Line
	1650 5400 1650 5450
Connection ~ 1650 5400
Wire Wire Line
	2050 5400 2150 5400
$Comp
L GND #PWR?
U 1 1 5875CE20
P 1650 5800
F 0 "#PWR?" H 1650 5550 50  0001 C CNN
F 1 "GND" H 1600 5650 50  0001 C CNN
F 2 "" H 1650 5800 50  0000 C CNN
F 3 "" H 1650 5800 50  0000 C CNN
	1    1650 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 5800 1650 5750
Wire Notes Line
	3900 6100 500  6100
Text Notes 550  3950 0    60   ~ 12
Bumpers
Text HLabel 2650 4850 2    60   Input ~ 12
+3.3V
Text HLabel 2650 5500 2    60   Input ~ 12
+3.3V
Text HLabel 1600 2750 1    60   Input ~ 12
+3.3V
Wire Wire Line
	1600 2800 1600 2750
Text HLabel 1600 1350 1    60   Input ~ 12
+3.3V
Wire Wire Line
	1600 1400 1600 1350
Text HLabel 5400 1200 1    60   Input ~ 12
+3.3V
Text HLabel 8950 1200 1    60   Input ~ 12
+3.3V
Text HLabel 6700 1800 1    60   Input ~ 12
+3.3V
Text HLabel 2700 1950 2    60   Input ~ 12
+3.3V
Wire Wire Line
	2700 1950 2650 1950
Text HLabel 4700 2100 0    60   Input ~ 12
SCL
Text HLabel 8250 2100 0    60   Input ~ 12
SCL
Text HLabel 3100 2900 2    60   Input ~ 12
SCL
Text HLabel 2650 1500 2    60   Input ~ 12
SCL
Text HLabel 2650 1400 2    60   Input ~ 12
SDA
Text HLabel 2750 2800 2    60   Input ~ 12
SDA
Text HLabel 4700 2200 0    60   Input ~ 12
SDA
Text HLabel 8250 2200 0    60   Input ~ 12
SDA
NoConn ~ 2650 3200
NoConn ~ 2650 1800
Text HLabel 1450 4750 0    60   Input ~ 12
rear_bumper
Text HLabel 1450 5400 0    60   Input ~ 12
front_bumper
$Comp
L LSM303DLHCTR U?
U 1 1 586C66E2
P 9000 2150
F 0 "U?" H 9400 2600 60  0000 C CNN
F 1 "LSM303DLHCTR" H 9000 2150 60  0000 C CNB
F 2 "" H 8250 2550 60  0001 C CNN
F 3 "" H 8250 2550 60  0001 C CNN
	1    9000 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 1550 9050 1550
Wire Wire Line
	8850 3200 8850 2800
Text Notes 8250 600  0    60   ~ 0
addr: 0x32
Text Notes 8250 700  0    60   ~ 0
addr: 0x3C
Text Notes 4550 600  0    60   ~ 0
addr: 0xD6 (alt 0xD4)
Text Notes 1350 700  0    60   ~ 0
addr: 0x70\n      0x72
Wire Wire Line
	2150 5150 2150 5100
Wire Wire Line
	2150 4750 2150 4800
Wire Wire Line
	2150 5400 2150 5450
Wire Wire Line
	2150 5800 2150 5750
Wire Wire Line
	2650 5500 2550 5500
Wire Wire Line
	2650 4850 2550 4850
$EndSCHEMATC
