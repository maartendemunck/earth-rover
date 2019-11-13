EESchema Schematic File Version 4
LIBS:position-encoder-cache
EELAYER 30 0
EELAYER END
$Descr A4 8268 11693 portrait
encoding utf-8
Sheet 1 1
Title "Position Encoder for Earth Rover"
Date "2019-11-11"
Rev "1"
Comp "Vijfendertig BVBA"
Comment1 "Designed by Maarten De Munck"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L earth-rover-custom-symbols:TLE4905L U1
U 1 1 5DC99980
P 2700 3000
F 0 "U1" H 2473 3046 50  0000 R CNN
F 1 "TLE4905L" H 2473 2955 50  0000 R CNN
F 2 "earth-rover-custom-footprints:PG-SSO-3-2_W2.54mm_Horizontal_Down" H 2700 3000 50  0001 C CNN
F 3 "https://www.infineon.com/cms/en/product/sensor/magnetic-sensors/magnetic-position-sensors/hall-switches/tle4905l/" H 2700 3000 50  0001 C CNN
	1    2700 3000
	1    0    0    -1  
$EndComp
$Comp
L earth-rover-custom-symbols:TLE4905L U2
U 1 1 5DC9BFD9
P 2700 5000
F 0 "U2" H 2473 5046 50  0000 R CNN
F 1 "TLE4905L" H 2473 4955 50  0000 R CNN
F 2 "earth-rover-custom-footprints:PG-SSO-3-2_W2.54mm_Horizontal_Down" H 2700 5000 50  0001 C CNN
F 3 "https://www.infineon.com/cms/en/product/sensor/magnetic-sensors/magnetic-position-sensors/hall-switches/tle4905l/" H 2700 5000 50  0001 C CNN
	1    2700 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5DC9E08A
P 3200 3250
F 0 "C1" H 3315 3296 50  0000 L CNN
F 1 "4.7nF" H 3315 3205 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3238 3100 50  0001 C CNN
F 3 "~" H 3200 3250 50  0001 C CNN
	1    3200 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5DC9E995
P 3200 5250
F 0 "C2" H 3315 5296 50  0000 L CNN
F 1 "4.7nF" H 3315 5205 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3238 5100 50  0001 C CNN
F 3 "~" H 3200 5250 50  0001 C CNN
	1    3200 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5DC9EE2B
P 3700 5250
F 0 "C4" H 3815 5296 50  0000 L CNN
F 1 "4.7nF" H 3815 5205 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3738 5100 50  0001 C CNN
F 3 "~" H 3700 5250 50  0001 C CNN
	1    3700 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5DCA1A9C
P 3200 4750
F 0 "R2" H 3270 4796 50  0000 L CNN
F 1 "10kΩ" H 3270 4705 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3130 4750 50  0001 C CNN
F 3 "~" H 3200 4750 50  0001 C CNN
	1    3200 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5DCA141F
P 3200 2750
F 0 "R1" H 3270 2796 50  0000 L CNN
F 1 "10kΩ" H 3270 2705 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3130 2750 50  0001 C CNN
F 3 "~" H 3200 2750 50  0001 C CNN
	1    3200 2750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5DC9D68E
P 3700 3250
F 0 "C3" H 3815 3296 50  0000 L CNN
F 1 "4.7nF" H 3815 3205 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3738 3100 50  0001 C CNN
F 3 "~" H 3700 3250 50  0001 C CNN
	1    3700 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 2600 3200 2500
Wire Wire Line
	3200 2500 2700 2500
Wire Wire Line
	2700 2500 2700 2700
Wire Wire Line
	3200 3400 3200 3500
Wire Wire Line
	3200 3500 2700 3500
Wire Wire Line
	2700 3500 2700 3300
Wire Wire Line
	3000 3000 3200 3000
Wire Wire Line
	3200 3000 3200 2900
Wire Wire Line
	3200 3100 3200 3000
Connection ~ 3200 3000
Wire Wire Line
	3000 5000 3200 5000
Wire Wire Line
	3200 5000 3200 4900
Wire Wire Line
	3200 5100 3200 5000
Connection ~ 3200 5000
Wire Wire Line
	3200 4600 3200 4500
Wire Wire Line
	3200 4500 2700 4500
Wire Wire Line
	2700 4500 2700 4700
Connection ~ 2700 4500
Wire Wire Line
	3200 5500 3200 5400
Wire Wire Line
	2700 5300 2700 5500
Connection ~ 2700 5500
Wire Wire Line
	2700 5500 3200 5500
Connection ~ 2700 2500
Connection ~ 2700 3500
$Comp
L power:GND #PWR0101
U 1 1 5DCAC839
P 2700 3600
F 0 "#PWR0101" H 2700 3350 50  0001 C CNN
F 1 "GND" H 2705 3427 50  0000 C CNN
F 2 "" H 2700 3600 50  0001 C CNN
F 3 "" H 2700 3600 50  0001 C CNN
	1    2700 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5DCB396B
P 2700 5600
F 0 "#PWR0102" H 2700 5350 50  0001 C CNN
F 1 "GND" H 2705 5427 50  0000 C CNN
F 2 "" H 2700 5600 50  0001 C CNN
F 3 "" H 2700 5600 50  0001 C CNN
	1    2700 5600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 5DCB51EF
P 2700 4400
F 0 "#PWR0103" H 2700 4250 50  0001 C CNN
F 1 "+5V" H 2715 4573 50  0000 C CNN
F 2 "" H 2700 4400 50  0001 C CNN
F 3 "" H 2700 4400 50  0001 C CNN
	1    2700 4400
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0104
U 1 1 5DCB6492
P 2700 2400
F 0 "#PWR0104" H 2700 2250 50  0001 C CNN
F 1 "+5V" H 2715 2573 50  0000 C CNN
F 2 "" H 2700 2400 50  0001 C CNN
F 3 "" H 2700 2400 50  0001 C CNN
	1    2700 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 2400 2700 2500
Wire Wire Line
	2700 3500 2700 3600
Wire Wire Line
	2700 4400 2700 4500
Wire Wire Line
	2700 5500 2700 5600
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 5DCB9A7C
P 5900 2900
F 0 "J1" H 5872 2874 50  0000 R CNN
F 1 "Conn_01x04_Male" H 5872 2783 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal" H 5900 2900 50  0001 C CNN
F 3 "~" H 5900 2900 50  0001 C CNN
	1    5900 2900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3200 5000 4200 5000
Wire Wire Line
	4200 5000 4200 3100
$Comp
L power:GND #PWR0105
U 1 1 5DCC09FC
P 5500 3300
F 0 "#PWR0105" H 5500 3050 50  0001 C CNN
F 1 "GND" H 5505 3127 50  0000 C CNN
F 2 "" H 5500 3300 50  0001 C CNN
F 3 "" H 5500 3300 50  0001 C CNN
	1    5500 3300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0106
U 1 1 5DCC1747
P 5300 2600
F 0 "#PWR0106" H 5300 2450 50  0001 C CNN
F 1 "+5V" H 5315 2773 50  0000 C CNN
F 2 "" H 5300 2600 50  0001 C CNN
F 3 "" H 5300 2600 50  0001 C CNN
	1    5300 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 4500 3700 4500
Wire Wire Line
	3700 4500 3700 5100
Connection ~ 3200 4500
Wire Wire Line
	3200 5500 3700 5500
Wire Wire Line
	3700 5500 3700 5400
Connection ~ 3200 5500
Wire Wire Line
	3200 3500 3700 3500
Wire Wire Line
	3700 3500 3700 3400
Connection ~ 3200 3500
Wire Wire Line
	3700 3100 3700 2500
Wire Wire Line
	3700 2500 3200 2500
Connection ~ 3200 2500
Wire Wire Line
	3200 3000 5700 3000
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5DD0D13F
P 4900 2600
F 0 "#FLG0101" H 4900 2675 50  0001 C CNN
F 1 "PWR_FLAG" H 4900 2773 50  0000 C CNN
F 2 "" H 4900 2600 50  0001 C CNN
F 3 "~" H 4900 2600 50  0001 C CNN
	1    4900 2600
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5DD0E731
P 5100 3300
F 0 "#FLG0102" H 5100 3375 50  0001 C CNN
F 1 "PWR_FLAG" H 5100 3473 50  0000 C CNN
F 2 "" H 5100 3300 50  0001 C CNN
F 3 "~" H 5100 3300 50  0001 C CNN
	1    5100 3300
	1    0    0    1   
$EndComp
Wire Wire Line
	4200 3100 5700 3100
Wire Wire Line
	5700 2800 5500 2800
Wire Wire Line
	5500 2800 5500 3200
Wire Wire Line
	5100 3300 5100 3200
Wire Wire Line
	5100 3200 5500 3200
Connection ~ 5500 3200
Wire Wire Line
	5500 3200 5500 3300
Wire Wire Line
	5300 2600 5300 2700
Wire Wire Line
	5300 2900 5700 2900
Wire Wire Line
	4900 2600 4900 2700
Wire Wire Line
	4900 2700 5300 2700
Connection ~ 5300 2700
Wire Wire Line
	5300 2700 5300 2900
$EndSCHEMATC
