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
LIBS:MF_Connectors
LIBS:PiBot-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 4
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
L DMG2302U U?
U 1 1 586935AF
P 4000 4700
AR Path="/586935AF" Ref="U?"  Part="1" 
AR Path="/5869317C/586935AF" Ref="U3"  Part="1" 
F 0 "U3" V 3850 4550 60  0000 C CNN
F 1 "DMG2302U" V 4190 4700 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 4000 4700 60  0001 C CNN
F 3 "" H 4000 4700 60  0001 C CNN
	1    4000 4700
	0    -1   1    0   
$EndComp
$Comp
L GND #PWR061
U 1 1 586935B0
P 4750 4750
F 0 "#PWR061" H 4750 4500 50  0001 C CNN
F 1 "GND" H 4700 4600 50  0001 C CNN
F 2 "" H 4750 4750 50  0000 C CNN
F 3 "" H 4750 4750 50  0000 C CNN
	1    4750 4750
	1    0    0    -1  
$EndComp
$Comp
L Battery_Cell BT?
U 1 1 586935B2
P 1450 5050
AR Path="/586935B2" Ref="BT?"  Part="1" 
AR Path="/5869317C/586935B2" Ref="BT4"  Part="1" 
F 0 "BT4" H 1550 5100 50  0000 L CNN
F 1 "Battery" H 1350 4800 50  0001 L CNN
F 2 "" V 1450 5090 50  0001 C CNN
F 3 "" V 1450 5090 50  0000 C CNN
	1    1450 5050
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 586935B3
P 3500 4500
F 0 "C9" H 3525 4600 50  0000 L CNN
F 1 "0.1uF" H 3525 4400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 3538 4350 50  0001 C CNN
F 3 "" H 3500 4500 50  0000 C CNN
	1    3500 4500
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 586935B4
P 4750 4500
F 0 "C10" H 4775 4600 50  0000 L CNN
F 1 "0.1uF" H 4775 4400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4788 4350 50  0001 C CNN
F 3 "" H 4750 4500 50  0000 C CNN
	1    4750 4500
	1    0    0    -1  
$EndComp
$Comp
L D_Zener_Small D8
U 1 1 586935B5
P 2950 4400
F 0 "D8" V 2950 4500 50  0000 C CNN
F 1 "DZ2J051M0L" V 3300 4350 50  0000 C CNN
F 2 "Diodes_SMD:SOD-323" H 2950 4400 50  0001 C CNN
F 3 "" H 2950 4400 50  0000 C CNN
	1    2950 4400
	0    1    1    0   
$EndComp
$Comp
L D_Zener_Small D9
U 1 1 586935B6
P 2950 4600
F 0 "D9" V 2950 4500 50  0000 C CNN
F 1 "DZ2J051M0L" V 3300 4450 50  0000 C CNN
F 2 "Diodes_SMD:SOD-323" H 2950 4600 50  0001 C CNN
F 3 "" H 2950 4600 50  0000 C CNN
	1    2950 4600
	0    -1   -1   0   
$EndComp
$Comp
L L L1
U 1 1 586935B7
P 6500 3350
F 0 "L1" H 6350 3450 50  0000 C CNN
F 1 "1 uH" V 6600 3350 50  0000 C CNN
F 2 "Inductors_NEOSID:Neosid_Inductor_SM-NE29_SMD1008" H 6500 3350 50  0001 C CNN
F 3 "" H 6500 3350 50  0000 C CNN
	1    6500 3350
	0    -1   -1   0   
$EndComp
$Comp
L C C11
U 1 1 586935B9
P 5100 3850
F 0 "C11" H 5125 3950 50  0000 L CNN
F 1 "10uF" V 4950 3750 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 5138 3700 50  0001 C CNN
F 3 "" H 5100 3850 50  0000 C CNN
	1    5100 3850
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 586935BA
P 5350 3850
F 0 "C12" H 5375 3950 50  0000 L CNN
F 1 "10uF" H 5375 3750 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 5388 3700 50  0001 C CNN
F 3 "" H 5350 3850 50  0000 C CNN
	1    5350 3850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR062
U 1 1 586935BB
P 5650 4150
F 0 "#PWR062" H 5650 3900 50  0001 C CNN
F 1 "GND" H 5650 4000 50  0001 C CNN
F 2 "" H 5650 4150 50  0000 C CNN
F 3 "" H 5650 4150 50  0000 C CNN
	1    5650 4150
	1    0    0    -1  
$EndComp
$Comp
L C C13
U 1 1 586935BC
P 7200 4150
F 0 "C13" H 7225 4250 50  0000 L CNN
F 1 "0.1uF" V 7050 4000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 7238 4000 50  0001 C CNN
F 3 "" H 7200 4150 50  0000 C CNN
	1    7200 4150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR063
U 1 1 586935BD
P 7200 4350
F 0 "#PWR063" H 7200 4100 50  0001 C CNN
F 1 "GND" H 7200 4200 50  0001 C CNN
F 2 "" H 7200 4350 50  0000 C CNN
F 3 "" H 7200 4350 50  0000 C CNN
	1    7200 4350
	1    0    0    -1  
$EndComp
$Comp
L R_Small R13
U 1 1 586935BF
P 5750 3350
F 0 "R13" H 5780 3370 50  0000 L CNN
F 1 "1M" V 5750 3300 50  0000 L CNN
F 2 "Resistors_SMD:R_0402" H 5750 3350 50  0001 C CNN
F 3 "" H 5750 3350 50  0000 C CNN
	1    5750 3350
	1    0    0    -1  
$EndComp
$Comp
L R_Small R14
U 1 1 586935C0
P 7750 3850
F 0 "R14" H 7780 3870 50  0000 L CNN
F 1 "1M" V 7750 3800 50  0000 L CNN
F 2 "Resistors_SMD:R_0402" H 7750 3850 50  0001 C CNN
F 3 "" H 7750 3850 50  0000 C CNN
	1    7750 3850
	1    0    0    -1  
$EndComp
$Comp
L R_Small R15
U 1 1 586935C1
P 7750 4100
F 0 "R15" H 7780 4120 50  0000 L CNN
F 1 "111k" V 7750 4000 50  0000 L CNN
F 2 "Resistors_SMD:R_0402" H 7750 4100 50  0001 C CNN
F 3 "" H 7750 4100 50  0000 C CNN
	1    7750 4100
	1    0    0    -1  
$EndComp
$Comp
L C C14
U 1 1 586935C2
P 7550 4100
F 0 "C14" H 7575 4200 50  0000 L CNN
F 1 "10pf" V 7500 4150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 7588 3950 50  0001 C CNN
F 3 "" H 7550 4100 50  0000 C CNN
	1    7550 4100
	1    0    0    -1  
$EndComp
$Comp
L C C15
U 1 1 586935C3
P 8200 3950
F 0 "C15" H 8225 4050 50  0000 L CNN
F 1 "22uf" H 8225 3850 50  0000 L CNN
F 2 "Resistors_SMD:R_0402" H 8238 3800 50  0001 C CNN
F 3 "" H 8200 3950 50  0000 C CNN
	1    8200 3950
	1    0    0    -1  
$EndComp
$Comp
L C C16
U 1 1 586935C4
P 8450 3950
F 0 "C16" H 8475 4050 50  0000 L CNN
F 1 "22uf" H 8475 3850 50  0000 L CNN
F 2 "Resistors_SMD:R_0402" H 8488 3800 50  0001 C CNN
F 3 "" H 8450 3950 50  0000 C CNN
	1    8450 3950
	1    0    0    -1  
$EndComp
$Comp
L C C17
U 1 1 586935C5
P 8700 3950
F 0 "C17" H 8725 4050 50  0000 L CNN
F 1 "22uf" H 8725 3850 50  0000 L CNN
F 2 "Resistors_SMD:R_0402" H 8738 3800 50  0001 C CNN
F 3 "" H 8700 3950 50  0000 C CNN
	1    8700 3950
	1    0    0    -1  
$EndComp
$Comp
L TPS63060 U4
U 1 1 586935C6
P 6300 3700
F 0 "U4" H 6510 3950 60  0000 C CNN
F 1 "TPS63060" V 6580 3550 60  0000 C CNB
F 2 "Housings_DFN_QFN:DFN-10-1EP_3x3mm_Pitch0.5mm" H 6300 3700 60  0001 C CNN
F 3 "" H 6300 3700 60  0001 C CNN
	1    6300 3700
	1    0    0    -1  
$EndComp
$Comp
L SW_SPST SW1
U 1 1 586935C7
P 2400 4300
F 0 "SW1" H 2400 4400 50  0000 C CNN
F 1 "SPST" H 2400 4200 50  0000 C CNN
F 2 "" H 2400 4300 50  0001 C CNN
F 3 "" H 2400 4300 50  0000 C CNN
	1    2400 4300
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR064
U 1 1 586935C8
P 4950 4750
F 0 "#PWR064" H 4950 4550 50  0001 C CNN
F 1 "GNDPWR" H 5000 4600 50  0000 C CNN
F 2 "" H 4950 4700 50  0000 C CNN
F 3 "" H 4950 4700 50  0000 C CNN
	1    4950 4750
	1    0    0    -1  
$EndComp
$Comp
L D_Zener_Small D10
U 1 1 586935C9
P 4450 4500
F 0 "D10" V 4350 4400 50  0000 C CNN
F 1 "DZ2J075M0L" V 4200 4600 50  0000 C CNN
F 2 "Diodes_SMD:SOD-323" H 4450 4500 50  0001 C CNN
F 3 "" H 4450 4500 50  0000 C CNN
	1    4450 4500
	0    1    1    0   
$EndComp
$Comp
L R_Small R11
U 1 1 586935CB
P 4050 4150
F 0 "R11" V 3950 4100 50  0000 L CNN
F 1 "0" V 4050 4125 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 4050 4150 50  0001 C CNN
F 3 "" H 4050 4150 50  0000 C CNN
	1    4050 4150
	-1   0    0    1   
$EndComp
$Comp
L R_Small R16
U 1 1 586935CC
P 7950 2950
F 0 "R16" V 7850 2900 50  0000 L CNN
F 1 "0" V 7950 2925 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 7950 2950 50  0001 C CNN
F 3 "" H 7950 2950 50  0000 C CNN
	1    7950 2950
	-1   0    0    1   
$EndComp
Text Notes 7350 7500 0    60   ~ 12
PycoBot (Teensy uPython Robot)
Text Notes 7050 7050 0    60   ~ 12
By:\nBroderick Carlin\nTyler Holmes
Text Notes 7950 7050 0    60   ~ 12
MIT License\n
Text Label 7250 3700 0    39   ~ 0
PSU_VOUT
Text Label 7150 3800 0    39   ~ 0
PSU_FB
Text Label 5950 3350 0    39   ~ 0
PSU_IND
Text Label 2100 4700 0    30   ~ 0
BATT_GND_TERM
Text Label 1750 4200 1    30   ~ 0
BATT_V+_TERM
Text Label 3550 4300 0    30   ~ 0
+BATT_ISOLATED
$Comp
L R_Small R12
U 1 1 586935CE
P 5250 3450
F 0 "R12" V 5150 3400 50  0000 L CNN
F 1 "0" V 5250 3425 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 5250 3450 50  0001 C CNN
F 3 "" H 5250 3450 50  0000 C CNN
	1    5250 3450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7950 2850 7950 2850
Connection ~ 7950 3150
Wire Wire Line
	5750 3150 7950 3150
Wire Wire Line
	4050 3700 4050 4050
Wire Wire Line
	4450 5000 4100 5000
Wire Wire Line
	3500 5000 3900 5000
Connection ~ 4450 4300
Wire Wire Line
	4450 4300 4450 4400
Connection ~ 4450 4700
Wire Wire Line
	4450 4600 4450 5000
Wire Wire Line
	4750 4300 4750 4350
Wire Wire Line
	4950 4700 4950 4750
Connection ~ 2950 4300
Connection ~ 2950 4700
Connection ~ 3500 4300
Wire Wire Line
	5750 4000 5900 4000
Wire Wire Line
	5750 3450 5750 4000
Wire Wire Line
	5750 3150 5750 3250
Wire Wire Line
	5650 3900 5900 3900
Wire Wire Line
	5650 3900 5650 4150
Wire Wire Line
	5100 4100 5900 4100
Connection ~ 7750 3700
Wire Wire Line
	7750 3750 7750 3700
Connection ~ 8450 3700
Wire Wire Line
	8700 3700 8700 3800
Connection ~ 8200 3700
Wire Wire Line
	8450 3700 8450 3800
Connection ~ 7950 3700
Wire Wire Line
	8200 3700 8200 3800
Connection ~ 8450 4300
Wire Wire Line
	8700 4300 8700 4100
Connection ~ 8200 4300
Wire Wire Line
	8450 4300 8450 4100
Wire Wire Line
	8200 4300 8200 4100
Connection ~ 7550 4300
Wire Wire Line
	7550 4300 7550 4250
Wire Wire Line
	7550 3800 7100 3800
Wire Wire Line
	7550 3950 7550 3800
Connection ~ 7750 3950
Wire Wire Line
	7750 3950 7750 4000
Wire Wire Line
	7550 3950 7750 3950
Connection ~ 7750 4300
Connection ~ 7400 4300
Wire Wire Line
	7750 4300 7750 4200
Wire Wire Line
	7100 3700 8700 3700
Wire Wire Line
	7950 3050 7950 3700
Wire Wire Line
	7200 4300 8700 4300
Wire Wire Line
	7400 3900 7400 4300
Wire Wire Line
	7100 3900 7400 3900
Wire Wire Line
	7100 4000 7200 4000
Wire Wire Line
	7200 4300 7200 4350
Connection ~ 5350 3700
Wire Wire Line
	4050 3700 5900 3700
Wire Wire Line
	7100 3350 7100 3600
Wire Wire Line
	6650 3350 7100 3350
Wire Wire Line
	5900 3350 5900 3600
Wire Wire Line
	5900 3350 6350 3350
Wire Wire Line
	2950 4500 2950 4500
Connection ~ 4750 4700
Wire Wire Line
	4250 4700 4950 4700
Wire Wire Line
	4750 4650 4750 4750
Wire Wire Line
	3500 4300 3500 4350
Connection ~ 3500 4700
Wire Wire Line
	3500 4650 3500 5000
Wire Wire Line
	1750 4700 3750 4700
Connection ~ 4050 4300
Wire Wire Line
	4050 4250 4050 4450
Wire Wire Line
	2600 4300 4750 4300
Wire Wire Line
	5100 4100 5100 4000
Connection ~ 5650 4100
Wire Wire Line
	5350 4000 5350 4100
Connection ~ 5350 4100
Wire Wire Line
	5900 3800 5850 3800
Wire Wire Line
	5850 3800 5850 3700
Connection ~ 5850 3700
Wire Wire Line
	5150 3450 5000 3450
Text HLabel 8050 2800 2    60   Output ~ 0
5v_out
Wire Wire Line
	7950 2850 7950 2800
Wire Wire Line
	7950 2800 8050 2800
Text HLabel 5000 3450 0    60   Output ~ 0
PG
Text HLabel 4200 4000 2    60   Output ~ 0
vbatt_out
Wire Wire Line
	4050 4000 4200 4000
Connection ~ 4050 4000
Connection ~ 5100 3700
$Comp
L Jumper_NO_Small JP4
U 1 1 586DB13F
P 4000 5000
F 0 "JP4" H 4000 5080 50  0000 C CNN
F 1 "Jumper_NO_Small" H 4010 4940 50  0001 C CNN
F 2 "" H 4000 5000 50  0001 C CNN
F 3 "" H 4000 5000 50  0000 C CNN
	1    4000 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 3450 5750 3450
$Comp
L Battery_Cell BT3
U 1 1 586E837E
P 1450 4700
F 0 "BT3" H 1550 4750 50  0000 L CNN
F 1 "Battery" H 1300 4950 50  0001 L CNN
F 2 "" V 1450 4740 50  0001 C CNN
F 3 "" V 1450 4740 50  0000 C CNN
	1    1450 4700
	1    0    0    -1  
$EndComp
$Comp
L Battery_Cell BT2
U 1 1 586E892D
P 1450 4350
F 0 "BT2" H 1550 4400 50  0000 L CNN
F 1 "Battery" H 1450 4250 50  0001 L CNN
F 2 "" V 1450 4390 50  0001 C CNN
F 3 "" V 1450 4390 50  0000 C CNN
	1    1450 4350
	1    0    0    -1  
$EndComp
$Comp
L Battery_Cell BT1
U 1 1 586E8EA5
P 1450 4000
F 0 "BT1" H 1550 4050 50  0000 L CNN
F 1 "Battery" H 1450 3900 50  0001 L CNN
F 2 "" V 1450 4040 50  0001 C CNN
F 3 "" V 1450 4040 50  0000 C CNN
	1    1450 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 5150 1750 4700
Wire Wire Line
	1750 4300 2200 4300
Wire Wire Line
	1750 4300 1750 3800
Wire Wire Line
	1750 3800 1450 3800
Wire Wire Line
	1750 5150 1450 5150
Wire Wire Line
	1450 4800 1450 4850
Wire Wire Line
	1450 4500 1450 4450
Wire Wire Line
	1450 4150 1450 4100
$EndSCHEMATC
