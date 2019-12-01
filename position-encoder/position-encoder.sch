EESchema Schematic File Version 4
LIBS:position-encoder-cache
EELAYER 30 0
EELAYER END
$Descr A4 8268 11693 portrait
encoding utf-8
Sheet 1 1
Title "Earth Rover"
Date "2019-12-01"
Rev "2"
Comp "Vijfendertig BVBA"
Comment1 "Maarten De Munck"
Comment2 "Magnetic Position Encoder"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L earth-rover-custom-symbols:TLE4905L U1
U 1 1 5DC99980
P 3200 5800
F 0 "U1" H 2973 5846 50  0000 R CNN
F 1 "TLE4905L" H 2973 5755 50  0000 R CNN
F 2 "earth-rover-custom-footprints:PG-SSO-3-2_W2.54mm_Horizontal_Down" H 3200 5800 50  0001 C CNN
F 3 "https://www.infineon.com/cms/en/product/sensor/magnetic-sensors/magnetic-position-sensors/hall-switches/tle4905l/" H 3200 5800 50  0001 C CNN
	1    3200 5800
	1    0    0    -1  
$EndComp
$Comp
L earth-rover-custom-symbols:TLE4905L U2
U 1 1 5DC9BFD9
P 3200 7800
F 0 "U2" H 2973 7846 50  0000 R CNN
F 1 "TLE4905L" H 2973 7755 50  0000 R CNN
F 2 "earth-rover-custom-footprints:PG-SSO-3-2_W2.54mm_Horizontal_Down" H 3200 7800 50  0001 C CNN
F 3 "https://www.infineon.com/cms/en/product/sensor/magnetic-sensors/magnetic-position-sensors/hall-switches/tle4905l/" H 3200 7800 50  0001 C CNN
	1    3200 7800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5DC9E08A
P 3700 6050
F 0 "C1" H 3815 6096 50  0000 L CNN
F 1 "4.7nF" H 3815 6005 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3738 5900 50  0001 C CNN
F 3 "~" H 3700 6050 50  0001 C CNN
	1    3700 6050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5DC9E995
P 3700 8050
F 0 "C2" H 3815 8096 50  0000 L CNN
F 1 "4.7nF" H 3815 8005 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3738 7900 50  0001 C CNN
F 3 "~" H 3700 8050 50  0001 C CNN
	1    3700 8050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5DC9EE2B
P 4200 8050
F 0 "C4" H 4315 8096 50  0000 L CNN
F 1 "4.7nF" H 4315 8005 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 4238 7900 50  0001 C CNN
F 3 "~" H 4200 8050 50  0001 C CNN
	1    4200 8050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5DCA1A9C
P 3700 7550
F 0 "R2" H 3770 7596 50  0000 L CNN
F 1 "10kΩ" H 3770 7505 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3630 7550 50  0001 C CNN
F 3 "~" H 3700 7550 50  0001 C CNN
	1    3700 7550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5DCA141F
P 3700 5550
F 0 "R1" H 3770 5596 50  0000 L CNN
F 1 "10kΩ" H 3770 5505 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3630 5550 50  0001 C CNN
F 3 "~" H 3700 5550 50  0001 C CNN
	1    3700 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5DC9D68E
P 4200 6050
F 0 "C3" H 4315 6096 50  0000 L CNN
F 1 "4.7nF" H 4315 6005 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 4238 5900 50  0001 C CNN
F 3 "~" H 4200 6050 50  0001 C CNN
	1    4200 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 5400 3700 5300
Wire Wire Line
	3700 5300 3200 5300
Wire Wire Line
	3200 5300 3200 5500
Wire Wire Line
	3700 6200 3700 6300
Wire Wire Line
	3700 6300 3200 6300
Wire Wire Line
	3200 6300 3200 6100
Wire Wire Line
	3500 5800 3700 5800
Wire Wire Line
	3700 5800 3700 5700
Wire Wire Line
	3700 5900 3700 5800
Connection ~ 3700 5800
Wire Wire Line
	3500 7800 3700 7800
Wire Wire Line
	3700 7800 3700 7700
Wire Wire Line
	3700 7900 3700 7800
Connection ~ 3700 7800
Wire Wire Line
	3700 7400 3700 7300
Wire Wire Line
	3700 7300 3200 7300
Wire Wire Line
	3200 7300 3200 7500
Connection ~ 3200 7300
Wire Wire Line
	3700 8300 3700 8200
Wire Wire Line
	3200 8100 3200 8300
Connection ~ 3200 8300
Wire Wire Line
	3200 8300 3700 8300
Connection ~ 3200 5300
Connection ~ 3200 6300
$Comp
L power:GND #PWR0101
U 1 1 5DCAC839
P 3200 6400
F 0 "#PWR0101" H 3200 6150 50  0001 C CNN
F 1 "GND" H 3205 6227 50  0000 C CNN
F 2 "" H 3200 6400 50  0001 C CNN
F 3 "" H 3200 6400 50  0001 C CNN
	1    3200 6400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5DCB396B
P 3200 8400
F 0 "#PWR0102" H 3200 8150 50  0001 C CNN
F 1 "GND" H 3205 8227 50  0000 C CNN
F 2 "" H 3200 8400 50  0001 C CNN
F 3 "" H 3200 8400 50  0001 C CNN
	1    3200 8400
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 5DCB51EF
P 3200 7200
F 0 "#PWR0103" H 3200 7050 50  0001 C CNN
F 1 "+5V" H 3215 7373 50  0000 C CNN
F 2 "" H 3200 7200 50  0001 C CNN
F 3 "" H 3200 7200 50  0001 C CNN
	1    3200 7200
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0104
U 1 1 5DCB6492
P 3200 5200
F 0 "#PWR0104" H 3200 5050 50  0001 C CNN
F 1 "+5V" H 3215 5373 50  0000 C CNN
F 2 "" H 3200 5200 50  0001 C CNN
F 3 "" H 3200 5200 50  0001 C CNN
	1    3200 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 5200 3200 5300
Wire Wire Line
	3200 6300 3200 6400
Wire Wire Line
	3200 7200 3200 7300
Wire Wire Line
	3200 8300 3200 8400
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 5DCB9A7C
P 5900 5700
F 0 "J1" H 5872 5674 50  0000 R CNN
F 1 "Conn_01x04_Male" H 5872 5583 50  0001 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal" H 5900 5700 50  0001 C CNN
F 3 "~" H 5900 5700 50  0001 C CNN
	1    5900 5700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3700 7800 4700 7800
Wire Wire Line
	4700 7800 4700 5900
$Comp
L power:GND #PWR0105
U 1 1 5DCC09FC
P 5500 6100
F 0 "#PWR0105" H 5500 5850 50  0001 C CNN
F 1 "GND" H 5505 5927 50  0000 C CNN
F 2 "" H 5500 6100 50  0001 C CNN
F 3 "" H 5500 6100 50  0001 C CNN
	1    5500 6100
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0106
U 1 1 5DCC1747
P 5300 5400
F 0 "#PWR0106" H 5300 5250 50  0001 C CNN
F 1 "+5V" H 5315 5573 50  0000 C CNN
F 2 "" H 5300 5400 50  0001 C CNN
F 3 "" H 5300 5400 50  0001 C CNN
	1    5300 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 7300 4200 7300
Wire Wire Line
	4200 7300 4200 7900
Connection ~ 3700 7300
Wire Wire Line
	3700 8300 4200 8300
Wire Wire Line
	4200 8300 4200 8200
Connection ~ 3700 8300
Wire Wire Line
	3700 6300 4200 6300
Wire Wire Line
	4200 6300 4200 6200
Connection ~ 3700 6300
Wire Wire Line
	4200 5900 4200 5300
Wire Wire Line
	4200 5300 3700 5300
Connection ~ 3700 5300
Wire Wire Line
	3700 5800 5700 5800
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5DD0D13F
P 5200 5400
F 0 "#FLG0101" H 5200 5475 50  0001 C CNN
F 1 "PWR_FLAG" H 5200 5573 50  0001 C CNN
F 2 "" H 5200 5400 50  0001 C CNN
F 3 "~" H 5200 5400 50  0001 C CNN
	1    5200 5400
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5DD0E731
P 5400 6100
F 0 "#FLG0102" H 5400 6175 50  0001 C CNN
F 1 "PWR_FLAG" H 5400 6273 50  0001 C CNN
F 2 "" H 5400 6100 50  0001 C CNN
F 3 "~" H 5400 6100 50  0001 C CNN
	1    5400 6100
	1    0    0    1   
$EndComp
Wire Wire Line
	4700 5900 5700 5900
Wire Wire Line
	5700 5600 5500 5600
Wire Wire Line
	5500 5600 5500 6000
Wire Wire Line
	5400 6100 5400 6000
Wire Wire Line
	5400 6000 5500 6000
Connection ~ 5500 6000
Wire Wire Line
	5500 6000 5500 6100
Wire Wire Line
	5300 5400 5300 5500
Wire Wire Line
	5300 5700 5700 5700
Wire Wire Line
	5200 5400 5200 5500
Wire Wire Line
	5200 5500 5300 5500
Connection ~ 5300 5500
Wire Wire Line
	5300 5500 5300 5700
Text Notes 2500 9150 0    50   ~ 0
The TLE4905L unipolar magnetic field switches have open collector outputs.\nWhen using 3.3V logic, omit pull-up resistors R1 and R2 and use external pull-up resistors.
$EndSCHEMATC
