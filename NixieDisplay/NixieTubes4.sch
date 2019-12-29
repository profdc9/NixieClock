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
LIBS:in14_lib
LIBS:arduino_nano
LIBS:NixieClock-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 7
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
L IN14 TBE401
U 1 1 5E081CFD
P 5550 1250
F 0 "TBE401" H 5450 1250 60  0000 C CNN
F 1 "IN14" H 5200 950 60  0000 C CNN
F 2 "NixieClock:IN-14-mod" H 5450 1250 60  0000 C CNN
F 3 "" H 5450 1250 60  0000 C CNN
	1    5550 1250
	1    0    0    -1  
$EndComp
$Comp
L 74HC595 U401
U 1 1 5E081D02
P 4050 5100
F 0 "U401" H 4200 5700 50  0000 C CNN
F 1 "74HC595" H 4050 4500 50  0000 C CNN
F 2 "Housings_DIP:DIP-16_W7.62mm_Socket" H 4050 5100 50  0001 C CNN
F 3 "" H 4050 5100 50  0001 C CNN
	1    4050 5100
	0    -1   -1   0   
$EndComp
$Comp
L R R406
U 1 1 5E081D06
P 5350 2400
F 0 "R406" V 5400 2600 50  0000 C CNN
F 1 "680k" V 5350 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5280 2400 50  0001 C CNN
F 3 "" H 5350 2400 50  0001 C CNN
	1    5350 2400
	1    0    0    -1  
$EndComp
$Comp
L R R408
U 1 1 5E081D08
P 5450 2400
F 0 "R408" V 5500 2600 50  0000 C CNN
F 1 "180k" V 5450 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5380 2400 50  0001 C CNN
F 3 "" H 5450 2400 50  0001 C CNN
	1    5450 2400
	1    0    0    -1  
$EndComp
$Comp
L R R409
U 1 1 5E081D0E
P 5550 2400
F 0 "R409" V 5600 2600 50  0000 C CNN
F 1 "180k" V 5550 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5480 2400 50  0001 C CNN
F 3 "" H 5550 2400 50  0001 C CNN
	1    5550 2400
	1    0    0    -1  
$EndComp
$Comp
L R R410
U 1 1 5E081D11
P 5650 2400
F 0 "R410" V 5700 2600 50  0000 C CNN
F 1 "180k" V 5650 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5580 2400 50  0001 C CNN
F 3 "" H 5650 2400 50  0001 C CNN
	1    5650 2400
	1    0    0    -1  
$EndComp
$Comp
L R R411
U 1 1 5E081D17
P 5750 2400
F 0 "R411" V 5800 2600 50  0000 C CNN
F 1 "180k" V 5750 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5680 2400 50  0001 C CNN
F 3 "" H 5750 2400 50  0001 C CNN
	1    5750 2400
	1    0    0    -1  
$EndComp
Text GLabel 5750 1000 0    60   Input ~ 0
NIXIEPWR
$Comp
L R R413
U 1 1 5E081D3B
P 5850 2400
F 0 "R413" V 5900 2600 50  0000 C CNN
F 1 "180k" V 5850 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5780 2400 50  0001 C CNN
F 3 "" H 5850 2400 50  0001 C CNN
	1    5850 2400
	1    0    0    -1  
$EndComp
$Comp
L R R414
U 1 1 5E081D3E
P 5950 2400
F 0 "R414" V 6000 2600 50  0000 C CNN
F 1 "180k" V 5950 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5880 2400 50  0001 C CNN
F 3 "" H 5950 2400 50  0001 C CNN
	1    5950 2400
	1    0    0    -1  
$EndComp
$Comp
L R R415
U 1 1 5E07C164
P 6050 2400
F 0 "R415" V 6100 2600 50  0000 C CNN
F 1 "180k" V 6050 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5980 2400 50  0001 C CNN
F 3 "" H 6050 2400 50  0001 C CNN
	1    6050 2400
	1    0    0    -1  
$EndComp
$Comp
L R R416
U 1 1 5E081D45
P 6150 2400
F 0 "R416" V 6200 2600 50  0000 C CNN
F 1 "180k" V 6150 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6080 2400 50  0001 C CNN
F 3 "" H 6150 2400 50  0001 C CNN
	1    6150 2400
	1    0    0    -1  
$EndComp
$Comp
L R R418
U 1 1 5E081D49
P 6250 2400
F 0 "R418" V 6300 2600 50  0000 C CNN
F 1 "180k" V 6250 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6180 2400 50  0001 C CNN
F 3 "" H 6250 2400 50  0001 C CNN
	1    6250 2400
	1    0    0    -1  
$EndComp
$Comp
L R R419
U 1 1 5E07C167
P 6350 2400
F 0 "R419" V 6400 2600 50  0000 C CNN
F 1 "180k" V 6350 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6280 2400 50  0001 C CNN
F 3 "" H 6350 2400 50  0001 C CNN
	1    6350 2400
	1    0    0    -1  
$EndComp
$Comp
L R R420
U 1 1 5E07C168
P 6450 2400
F 0 "R420" V 6500 2600 50  0000 C CNN
F 1 "180k" V 6450 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6380 2400 50  0001 C CNN
F 3 "" H 6450 2400 50  0001 C CNN
	1    6450 2400
	1    0    0    -1  
$EndComp
$Comp
L 2N3904 Q401
U 1 1 5E081D54
P 3550 3250
F 0 "Q401" H 3400 3300 50  0000 L CNN
F 1 "MPSA42" H 3300 3400 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 3750 3175 50  0001 L CIN
F 3 "" H 3550 3250 50  0001 L CNN
	1    3550 3250
	1    0    0    -1  
$EndComp
$Comp
L R R401
U 1 1 5E081D58
P 3350 3500
F 0 "R401" V 3400 3700 50  0000 C CNN
F 1 "10k" V 3350 3500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3280 3500 50  0001 C CNN
F 3 "" H 3350 3500 50  0001 C CNN
	1    3350 3500
	1    0    0    -1  
$EndComp
$Comp
L 2N3904 Q402
U 1 1 5E081D5D
P 3950 3250
F 0 "Q402" H 3800 3300 50  0000 L CNN
F 1 "MPSA42" H 3700 3400 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 4150 3175 50  0001 L CIN
F 3 "" H 3950 3250 50  0001 L CNN
	1    3950 3250
	1    0    0    -1  
$EndComp
$Comp
L 2N3904 Q403
U 1 1 5E081D1B
P 4350 3250
F 0 "Q403" H 4200 3300 50  0000 L CNN
F 1 "MPSA42" H 4100 3400 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 4550 3175 50  0001 L CIN
F 3 "" H 4350 3250 50  0001 L CNN
	1    4350 3250
	1    0    0    -1  
$EndComp
$Comp
L 2N3904 Q404
U 1 1 5E07A8CF
P 4750 3250
F 0 "Q404" H 4600 3300 50  0000 L CNN
F 1 "MPSA42" H 4500 3400 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 4950 3175 50  0001 L CIN
F 3 "" H 4750 3250 50  0001 L CNN
	1    4750 3250
	1    0    0    -1  
$EndComp
$Comp
L 2N3904 Q405
U 1 1 5E081D21
P 5150 3250
F 0 "Q405" H 5000 3300 50  0000 L CNN
F 1 "MPSA42" H 4900 3400 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 5350 3175 50  0001 L CIN
F 3 "" H 5150 3250 50  0001 L CNN
	1    5150 3250
	1    0    0    -1  
$EndComp
$Comp
L 2N3904 Q406
U 1 1 5E081D24
P 5550 3250
F 0 "Q406" H 5400 3300 50  0000 L CNN
F 1 "MPSA42" H 5300 3400 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 5750 3175 50  0001 L CIN
F 3 "" H 5550 3250 50  0001 L CNN
	1    5550 3250
	1    0    0    -1  
$EndComp
$Comp
L 2N3904 Q407
U 1 1 5E081D2A
P 5950 3250
F 0 "Q407" H 5800 3300 50  0000 L CNN
F 1 "MPSA42" H 5700 3400 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 6150 3175 50  0001 L CIN
F 3 "" H 5950 3250 50  0001 L CNN
	1    5950 3250
	1    0    0    -1  
$EndComp
$Comp
L 2N3904 Q408
U 1 1 5E081D61
P 6350 3250
F 0 "Q408" H 6200 3300 50  0000 L CNN
F 1 "MPSA42" H 6100 3400 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 6550 3175 50  0001 L CIN
F 3 "" H 6350 3250 50  0001 L CNN
	1    6350 3250
	1    0    0    -1  
$EndComp
$Comp
L 2N3904 Q409
U 1 1 5E081D66
P 6750 3250
F 0 "Q409" H 6600 3300 50  0000 L CNN
F 1 "MPSA42" H 6500 3400 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 6950 3175 50  0001 L CIN
F 3 "" H 6750 3250 50  0001 L CNN
	1    6750 3250
	1    0    0    -1  
$EndComp
$Comp
L 2N3904 Q410
U 1 1 5E081D6A
P 7150 3250
F 0 "Q410" H 7000 3300 50  0000 L CNN
F 1 "MPSA42" H 6900 3400 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 7350 3175 50  0001 L CIN
F 3 "" H 7150 3250 50  0001 L CNN
	1    7150 3250
	1    0    0    -1  
$EndComp
$Comp
L 2N3904 Q411
U 1 1 5E081D2C
P 7550 3250
F 0 "Q411" H 7400 3300 50  0000 L CNN
F 1 "MPSA42" H 7300 3400 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 7750 3175 50  0001 L CIN
F 3 "" H 7550 3250 50  0001 L CNN
	1    7550 3250
	1    0    0    -1  
$EndComp
$Comp
L 2N3904 Q412
U 1 1 5E081D6E
P 7950 3250
F 0 "Q412" H 7800 3300 50  0000 L CNN
F 1 "MPSA42" H 7700 3400 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 8150 3175 50  0001 L CIN
F 3 "" H 7950 3250 50  0001 L CNN
	1    7950 3250
	1    0    0    -1  
$EndComp
$Comp
L R R402
U 1 1 5E081D70
P 3750 3500
F 0 "R402" V 3800 3700 50  0000 C CNN
F 1 "10k" V 3750 3500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3680 3500 50  0001 C CNN
F 3 "" H 3750 3500 50  0001 C CNN
	1    3750 3500
	1    0    0    -1  
$EndComp
$Comp
L R R403
U 1 1 5E081D77
P 4150 3500
F 0 "R403" V 4200 3700 50  0000 C CNN
F 1 "10k" V 4150 3500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4080 3500 50  0001 C CNN
F 3 "" H 4150 3500 50  0001 C CNN
	1    4150 3500
	1    0    0    -1  
$EndComp
$Comp
L R R404
U 1 1 5E081D7B
P 4550 3500
F 0 "R404" V 4600 3700 50  0000 C CNN
F 1 "10k" V 4550 3500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4480 3500 50  0001 C CNN
F 3 "" H 4550 3500 50  0001 C CNN
	1    4550 3500
	1    0    0    -1  
$EndComp
$Comp
L R R405
U 1 1 5E081D7F
P 4950 3500
F 0 "R405" V 5000 3700 50  0000 C CNN
F 1 "10k" V 4950 3500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4880 3500 50  0001 C CNN
F 3 "" H 4950 3500 50  0001 C CNN
	1    4950 3500
	1    0    0    -1  
$EndComp
$Comp
L R R407
U 1 1 5E081D80
P 5350 3500
F 0 "R407" V 5400 3700 50  0000 C CNN
F 1 "10k" V 5350 3500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5280 3500 50  0001 C CNN
F 3 "" H 5350 3500 50  0001 C CNN
	1    5350 3500
	1    0    0    -1  
$EndComp
$Comp
L R R412
U 1 1 5E07C17B
P 5750 3500
F 0 "R412" V 5800 3700 50  0000 C CNN
F 1 "10k" V 5750 3500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5680 3500 50  0001 C CNN
F 3 "" H 5750 3500 50  0001 C CNN
	1    5750 3500
	1    0    0    -1  
$EndComp
$Comp
L R R417
U 1 1 5E081D8A
P 6150 3500
F 0 "R417" V 6200 3700 50  0000 C CNN
F 1 "10k" V 6150 3500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6080 3500 50  0001 C CNN
F 3 "" H 6150 3500 50  0001 C CNN
	1    6150 3500
	1    0    0    -1  
$EndComp
$Comp
L R R421
U 1 1 5E081D8C
P 6550 3500
F 0 "R421" V 6600 3700 50  0000 C CNN
F 1 "10k" V 6550 3500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6480 3500 50  0001 C CNN
F 3 "" H 6550 3500 50  0001 C CNN
	1    6550 3500
	1    0    0    -1  
$EndComp
$Comp
L R R422
U 1 1 5E081D32
P 6950 3500
F 0 "R422" V 7000 3700 50  0000 C CNN
F 1 "10k" V 6950 3500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6880 3500 50  0001 C CNN
F 3 "" H 6950 3500 50  0001 C CNN
	1    6950 3500
	1    0    0    -1  
$EndComp
$Comp
L R R423
U 1 1 5E07BB07
P 7350 3500
F 0 "R423" V 7400 3700 50  0000 C CNN
F 1 "10k" V 7350 3500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7280 3500 50  0001 C CNN
F 3 "" H 7350 3500 50  0001 C CNN
	1    7350 3500
	1    0    0    -1  
$EndComp
$Comp
L R R424
U 1 1 5E081D91
P 7750 3500
F 0 "R424" V 7800 3700 50  0000 C CNN
F 1 "10k" V 7750 3500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7680 3500 50  0001 C CNN
F 3 "" H 7750 3500 50  0001 C CNN
	1    7750 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR74
U 1 1 5E07C181
P 3650 3500
F 0 "#PWR74" H 3650 3250 50  0001 C CNN
F 1 "GND" H 3650 3350 50  0000 C CNN
F 2 "" H 3650 3500 50  0001 C CNN
F 3 "" H 3650 3500 50  0001 C CNN
	1    3650 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR75
U 1 1 5E081D9A
P 4050 3500
F 0 "#PWR75" H 4050 3250 50  0001 C CNN
F 1 "GND" H 4050 3350 50  0000 C CNN
F 2 "" H 4050 3500 50  0001 C CNN
F 3 "" H 4050 3500 50  0001 C CNN
	1    4050 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR76
U 1 1 5E07C183
P 4450 3500
F 0 "#PWR76" H 4450 3250 50  0001 C CNN
F 1 "GND" H 4450 3350 50  0000 C CNN
F 2 "" H 4450 3500 50  0001 C CNN
F 3 "" H 4450 3500 50  0001 C CNN
	1    4450 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR78
U 1 1 5E07C184
P 4850 3500
F 0 "#PWR78" H 4850 3250 50  0001 C CNN
F 1 "GND" H 4850 3350 50  0000 C CNN
F 2 "" H 4850 3500 50  0001 C CNN
F 3 "" H 4850 3500 50  0001 C CNN
	1    4850 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR81
U 1 1 5E081DA5
P 5250 3500
F 0 "#PWR81" H 5250 3250 50  0001 C CNN
F 1 "GND" H 5250 3350 50  0000 C CNN
F 2 "" H 5250 3500 50  0001 C CNN
F 3 "" H 5250 3500 50  0001 C CNN
	1    5250 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR82
U 1 1 5E081DAB
P 5650 3500
F 0 "#PWR82" H 5650 3250 50  0001 C CNN
F 1 "GND" H 5650 3350 50  0000 C CNN
F 2 "" H 5650 3500 50  0001 C CNN
F 3 "" H 5650 3500 50  0001 C CNN
	1    5650 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR83
U 1 1 5E07C187
P 6050 3500
F 0 "#PWR83" H 6050 3250 50  0001 C CNN
F 1 "GND" H 6050 3350 50  0000 C CNN
F 2 "" H 6050 3500 50  0001 C CNN
F 3 "" H 6050 3500 50  0001 C CNN
	1    6050 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR85
U 1 1 5E081DB2
P 6450 3500
F 0 "#PWR85" H 6450 3250 50  0001 C CNN
F 1 "GND" H 6450 3350 50  0000 C CNN
F 2 "" H 6450 3500 50  0001 C CNN
F 3 "" H 6450 3500 50  0001 C CNN
	1    6450 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR86
U 1 1 5E07C189
P 6850 3500
F 0 "#PWR86" H 6850 3250 50  0001 C CNN
F 1 "GND" H 6850 3350 50  0000 C CNN
F 2 "" H 6850 3500 50  0001 C CNN
F 3 "" H 6850 3500 50  0001 C CNN
	1    6850 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR87
U 1 1 5E081DB9
P 7250 3500
F 0 "#PWR87" H 7250 3250 50  0001 C CNN
F 1 "GND" H 7250 3350 50  0000 C CNN
F 2 "" H 7250 3500 50  0001 C CNN
F 3 "" H 7250 3500 50  0001 C CNN
	1    7250 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR88
U 1 1 5E081DBF
P 7650 3500
F 0 "#PWR88" H 7650 3250 50  0001 C CNN
F 1 "GND" H 7650 3350 50  0000 C CNN
F 2 "" H 7650 3500 50  0001 C CNN
F 3 "" H 7650 3500 50  0001 C CNN
	1    7650 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR89
U 1 1 5E081DC0
P 8050 3500
F 0 "#PWR89" H 8050 3250 50  0001 C CNN
F 1 "GND" H 8050 3350 50  0000 C CNN
F 2 "" H 8050 3500 50  0001 C CNN
F 3 "" H 8050 3500 50  0001 C CNN
	1    8050 3500
	1    0    0    -1  
$EndComp
$Comp
L 74HC595 U402
U 1 1 5E07C18D
P 5700 5100
F 0 "U402" H 5850 5700 50  0000 C CNN
F 1 "74HC595" H 5700 4500 50  0000 C CNN
F 2 "Housings_DIP:DIP-16_W7.62mm_Socket" H 5700 5100 50  0001 C CNN
F 3 "" H 5700 5100 50  0001 C CNN
	1    5700 5100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5350 2250 5350 2100
Wire Wire Line
	5450 2250 5450 2100
Wire Wire Line
	5550 2250 5550 2100
Wire Wire Line
	5650 2250 5650 2100
Wire Wire Line
	5750 2250 5750 2100
Wire Wire Line
	5750 1000 5850 1000
Wire Wire Line
	5850 2250 5850 2100
Wire Wire Line
	5950 2250 5950 2100
Wire Wire Line
	6050 2250 6050 2100
Wire Wire Line
	6150 2100 6150 2250
Wire Wire Line
	6250 2250 6250 2100
Wire Wire Line
	6350 2250 6350 2100
Wire Wire Line
	6450 2250 6450 2100
Wire Wire Line
	3350 3350 3350 3250
Wire Wire Line
	3650 3050 3650 2550
Wire Wire Line
	3650 2550 5350 2550
Wire Wire Line
	4050 3050 4050 2600
Wire Wire Line
	4050 2600 5450 2600
Wire Wire Line
	5450 2600 5450 2550
Wire Wire Line
	4450 3050 4450 2650
Wire Wire Line
	4450 2650 5550 2650
Wire Wire Line
	5550 2650 5550 2550
Wire Wire Line
	4850 3050 4850 2700
Wire Wire Line
	4850 2700 5650 2700
Wire Wire Line
	5650 2700 5650 2550
Wire Wire Line
	5250 3050 5250 2750
Wire Wire Line
	5250 2750 5750 2750
Wire Wire Line
	5750 2750 5750 2550
Wire Wire Line
	5650 3050 5650 2800
Wire Wire Line
	5650 2800 5850 2800
Wire Wire Line
	5850 2800 5850 2550
Wire Wire Line
	6050 3050 5950 3050
Wire Wire Line
	5950 3050 5950 2550
Wire Wire Line
	6450 3050 6450 3000
Wire Wire Line
	6450 3000 6050 3000
Wire Wire Line
	6050 3000 6050 2550
Wire Wire Line
	6850 3050 6850 2950
Wire Wire Line
	6850 2950 6150 2950
Wire Wire Line
	6150 2950 6150 2550
Wire Wire Line
	7250 3050 7250 2900
Wire Wire Line
	7250 2900 6250 2900
Wire Wire Line
	6250 2900 6250 2550
Wire Wire Line
	7650 3050 7650 2850
Wire Wire Line
	7650 2850 6350 2850
Wire Wire Line
	6350 2850 6350 2550
Wire Wire Line
	8050 2750 6450 2750
Wire Wire Line
	6450 2750 6450 2550
Wire Wire Line
	3750 3350 3750 3250
Wire Wire Line
	4150 3250 4150 3350
Wire Wire Line
	4550 3350 4550 3250
Wire Wire Line
	4950 3350 4950 3250
Wire Wire Line
	5350 3350 5350 3250
Wire Wire Line
	5750 3350 5750 3250
Wire Wire Line
	6150 3350 6150 3250
Wire Wire Line
	6550 3350 6550 3250
Wire Wire Line
	6950 3350 6950 3250
Wire Wire Line
	7350 3250 7350 3350
Wire Wire Line
	7750 3350 7750 3250
Wire Wire Line
	3650 3500 3650 3450
Wire Wire Line
	4050 3500 4050 3450
Wire Wire Line
	4450 3500 4450 3450
Wire Wire Line
	4850 3500 4850 3450
Wire Wire Line
	5250 3500 5250 3450
Wire Wire Line
	5650 3500 5650 3450
Wire Wire Line
	6050 3500 6050 3450
Wire Wire Line
	6450 3500 6450 3450
Wire Wire Line
	6850 3500 6850 3450
Wire Wire Line
	7250 3500 7250 3450
Wire Wire Line
	7650 3500 7650 3450
Wire Wire Line
	8050 3500 8050 3450
Wire Wire Line
	3350 3650 3350 4400
Wire Wire Line
	3350 4400 3600 4400
Wire Wire Line
	3700 4400 3700 3700
Wire Wire Line
	3700 3700 3750 3700
Wire Wire Line
	3750 3700 3750 3650
Wire Wire Line
	4150 3650 4150 3750
Wire Wire Line
	4150 3750 3800 3750
Wire Wire Line
	3800 3750 3800 4400
Wire Wire Line
	3900 4400 3900 3800
Wire Wire Line
	3900 3800 4550 3800
Wire Wire Line
	4550 3800 4550 3650
Wire Wire Line
	4000 4400 4000 3850
Wire Wire Line
	4000 3850 4950 3850
Wire Wire Line
	4950 3850 4950 3650
Wire Wire Line
	4100 4400 4100 3900
Wire Wire Line
	4100 3900 5350 3900
Wire Wire Line
	5350 3900 5350 3650
Wire Wire Line
	4200 4400 4200 3950
Wire Wire Line
	4200 3950 5750 3950
Wire Wire Line
	5750 3950 5750 3650
Wire Wire Line
	4300 4400 4300 4000
Wire Wire Line
	4300 4000 6150 4000
Wire Wire Line
	6150 4000 6150 3650
Wire Wire Line
	5250 4400 5250 4050
Wire Wire Line
	5250 4050 6550 4050
Wire Wire Line
	6550 4050 6550 3650
Wire Wire Line
	5350 4400 5350 4100
Wire Wire Line
	5350 4100 6950 4100
Wire Wire Line
	6950 4100 6950 3650
Wire Wire Line
	5450 4400 5450 4150
Wire Wire Line
	5450 4150 7350 4150
Wire Wire Line
	7350 4150 7350 3650
Wire Wire Line
	5550 4400 5550 4200
Wire Wire Line
	5550 4200 7750 4200
Wire Wire Line
	7750 4200 7750 3650
Text GLabel 3600 6050 3    60   Input ~ 0
SERDATA4
Wire Wire Line
	3600 6050 3600 5800
Wire Wire Line
	5450 5800 5450 5950
Wire Wire Line
	5450 5950 3800 5950
Wire Wire Line
	3800 5800 3800 6050
Text GLabel 3800 6050 3    60   Input ~ 0
SERCLK
Connection ~ 3800 5950
Text GLabel 4100 6050 3    60   Input ~ 0
SERLATCH
Wire Wire Line
	4100 5800 4100 6050
Wire Wire Line
	4100 5900 5750 5900
Wire Wire Line
	5750 5900 5750 5800
Connection ~ 4100 5900
$Comp
L GND #PWR77
U 1 1 5E081DC9
P 4650 5600
F 0 "#PWR77" H 4650 5350 50  0001 C CNN
F 1 "GND" H 4650 5450 50  0000 C CNN
F 2 "" H 4650 5600 50  0001 C CNN
F 3 "" H 4650 5600 50  0001 C CNN
	1    4650 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR84
U 1 1 5E081DCD
P 6350 5600
F 0 "#PWR84" H 6350 5350 50  0001 C CNN
F 1 "GND" H 6350 5450 50  0000 C CNN
F 2 "" H 6350 5600 50  0001 C CNN
F 3 "" H 6350 5600 50  0001 C CNN
	1    6350 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 5600 6350 5400
Wire Wire Line
	6350 5400 6250 5400
Wire Wire Line
	4600 5400 4650 5400
$Comp
L +5V #PWR72
U 1 1 5E081DD1
P 3300 5200
F 0 "#PWR72" H 3300 5050 50  0001 C CNN
F 1 "+5V" H 3300 5340 50  0000 C CNN
F 2 "" H 3300 5200 50  0001 C CNN
F 3 "" H 3300 5200 50  0001 C CNN
	1    3300 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 5200 3300 5450
Wire Wire Line
	3300 5400 3500 5400
Wire Wire Line
	4650 5400 4650 5600
Wire Wire Line
	5250 5800 4750 5800
Wire Wire Line
	4750 5800 4750 4400
Wire Wire Line
	4750 4400 4500 4400
$Comp
L +5V #PWR79
U 1 1 5E07C191
P 5000 5300
F 0 "#PWR79" H 5000 5150 50  0001 C CNN
F 1 "+5V" H 5000 5440 50  0000 C CNN
F 2 "" H 5000 5300 50  0001 C CNN
F 3 "" H 5000 5300 50  0001 C CNN
	1    5000 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 5300 5000 5450
Wire Wire Line
	5000 5400 5150 5400
$Comp
L C_Small C402
U 1 1 5E081DDB
P 5000 5550
F 0 "C402" H 5010 5620 50  0000 L CNN
F 1 "100 nF" H 5010 5470 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 5000 5550 50  0001 C CNN
F 3 "" H 5000 5550 50  0001 C CNN
	1    5000 5550
	1    0    0    -1  
$EndComp
Connection ~ 5000 5400
$Comp
L GND #PWR80
U 1 1 5E081DDD
P 5000 6000
F 0 "#PWR80" H 5000 5750 50  0001 C CNN
F 1 "GND" H 5000 5850 50  0000 C CNN
F 2 "" H 5000 6000 50  0001 C CNN
F 3 "" H 5000 6000 50  0001 C CNN
	1    5000 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 6000 5000 5650
$Comp
L C_Small C401
U 1 1 5E081DE3
P 3300 5550
F 0 "C401" H 3310 5620 50  0000 L CNN
F 1 "100 nF" H 3310 5470 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 3300 5550 50  0001 C CNN
F 3 "" H 3300 5550 50  0001 C CNN
	1    3300 5550
	1    0    0    -1  
$EndComp
Connection ~ 3300 5400
$Comp
L GND #PWR73
U 1 1 5E081DE6
P 3300 5800
F 0 "#PWR73" H 3300 5550 50  0001 C CNN
F 1 "GND" H 3300 5650 50  0000 C CNN
F 2 "" H 3300 5800 50  0001 C CNN
F 3 "" H 3300 5800 50  0001 C CNN
	1    3300 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 5800 3300 5650
Wire Wire Line
	8050 3050 8050 2750
Text GLabel 6450 4400 3    60   Input ~ 0
SERDATA5
Wire Wire Line
	6150 4400 6150 4350
Wire Wire Line
	6150 4350 6450 4350
Wire Wire Line
	6450 4350 6450 4400
$EndSCHEMATC