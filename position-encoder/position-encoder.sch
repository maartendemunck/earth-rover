EESchema Schematic File Version 4
LIBS:position-encoder-cache
EELAYER 30 0
EELAYER END
$Descr A4 8268 11693 portrait
encoding utf-8
Sheet 1 1
Title "Earth Rover"
Date "2019-11-11"
Rev "1"
Comp "Vijfendertig BVBA"
Comment1 "Maarten De Munck"
Comment2 "Magnetic Position Encoder"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L earth-rover-custom-symbols:TLE4905L U1
U 1 1 5DC99980
P 2600 3950
F 0 "U1" H 2373 3996 50  0000 R CNN
F 1 "TLE4905L" H 2373 3905 50  0000 R CNN
F 2 "earth-rover-custom-footprints:PG-SSO-3-2_W2.54mm_Horizontal_Down" H 2600 3950 50  0001 C CNN
F 3 "https://www.infineon.com/cms/en/product/sensor/magnetic-sensors/magnetic-position-sensors/hall-switches/tle4905l/" H 2600 3950 50  0001 C CNN
	1    2600 3950
	1    0    0    -1  
$EndComp
$Comp
L earth-rover-custom-symbols:TLE4905L U2
U 1 1 5DC9BFD9
P 2600 5950
F 0 "U2" H 2373 5996 50  0000 R CNN
F 1 "TLE4905L" H 2373 5905 50  0000 R CNN
F 2 "earth-rover-custom-footprints:PG-SSO-3-2_W2.54mm_Horizontal_Down" H 2600 5950 50  0001 C CNN
F 3 "https://www.infineon.com/cms/en/product/sensor/magnetic-sensors/magnetic-position-sensors/hall-switches/tle4905l/" H 2600 5950 50  0001 C CNN
	1    2600 5950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5DC9E08A
P 3100 4200
F 0 "C1" H 3215 4246 50  0000 L CNN
F 1 "4.7nF" H 3215 4155 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3138 4050 50  0001 C CNN
F 3 "~" H 3100 4200 50  0001 C CNN
	1    3100 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5DC9E995
P 3100 6200
F 0 "C2" H 3215 6246 50  0000 L CNN
F 1 "4.7nF" H 3215 6155 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3138 6050 50  0001 C CNN
F 3 "~" H 3100 6200 50  0001 C CNN
	1    3100 6200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5DC9EE2B
P 3600 6200
F 0 "C4" H 3715 6246 50  0000 L CNN
F 1 "4.7nF" H 3715 6155 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3638 6050 50  0001 C CNN
F 3 "~" H 3600 6200 50  0001 C CNN
	1    3600 6200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5DCA1A9C
P 3100 5700
F 0 "R2" H 3170 5746 50  0000 L CNN
F 1 "10kΩ" H 3170 5655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3030 5700 50  0001 C CNN
F 3 "~" H 3100 5700 50  0001 C CNN
	1    3100 5700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5DCA141F
P 3100 3700
F 0 "R1" H 3170 3746 50  0000 L CNN
F 1 "10kΩ" H 3170 3655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3030 3700 50  0001 C CNN
F 3 "~" H 3100 3700 50  0001 C CNN
	1    3100 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5DC9D68E
P 3600 4200
F 0 "C3" H 3715 4246 50  0000 L CNN
F 1 "4.7nF" H 3715 4155 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3638 4050 50  0001 C CNN
F 3 "~" H 3600 4200 50  0001 C CNN
	1    3600 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 3550 3100 3450
Wire Wire Line
	3100 3450 2600 3450
Wire Wire Line
	2600 3450 2600 3650
Wire Wire Line
	3100 4350 3100 4450
Wire Wire Line
	3100 4450 2600 4450
Wire Wire Line
	2600 4450 2600 4250
Wire Wire Line
	2900 3950 3100 3950
Wire Wire Line
	3100 3950 3100 3850
Wire Wire Line
	3100 4050 3100 3950
Connection ~ 3100 3950
Wire Wire Line
	2900 5950 3100 5950
Wire Wire Line
	3100 5950 3100 5850
Wire Wire Line
	3100 6050 3100 5950
Connection ~ 3100 5950
Wire Wire Line
	3100 5550 3100 5450
Wire Wire Line
	3100 5450 2600 5450
Wire Wire Line
	2600 5450 2600 5650
Connection ~ 2600 5450
Wire Wire Line
	3100 6450 3100 6350
Wire Wire Line
	2600 6250 2600 6450
Connection ~ 2600 6450
Wire Wire Line
	2600 6450 3100 6450
Connection ~ 2600 3450
Connection ~ 2600 4450
$Comp
L power:GND #PWR0101
U 1 1 5DCAC839
P 2600 4550
F 0 "#PWR0101" H 2600 4300 50  0001 C CNN
F 1 "GND" H 2605 4377 50  0000 C CNN
F 2 "" H 2600 4550 50  0001 C CNN
F 3 "" H 2600 4550 50  0001 C CNN
	1    2600 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5DCB396B
P 2600 6550
F 0 "#PWR0102" H 2600 6300 50  0001 C CNN
F 1 "GND" H 2605 6377 50  0000 C CNN
F 2 "" H 2600 6550 50  0001 C CNN
F 3 "" H 2600 6550 50  0001 C CNN
	1    2600 6550
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 5DCB51EF
P 2600 5350
F 0 "#PWR0103" H 2600 5200 50  0001 C CNN
F 1 "+5V" H 2615 5523 50  0000 C CNN
F 2 "" H 2600 5350 50  0001 C CNN
F 3 "" H 2600 5350 50  0001 C CNN
	1    2600 5350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0104
U 1 1 5DCB6492
P 2600 3350
F 0 "#PWR0104" H 2600 3200 50  0001 C CNN
F 1 "+5V" H 2615 3523 50  0000 C CNN
F 2 "" H 2600 3350 50  0001 C CNN
F 3 "" H 2600 3350 50  0001 C CNN
	1    2600 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 3350 2600 3450
Wire Wire Line
	2600 4450 2600 4550
Wire Wire Line
	2600 5350 2600 5450
Wire Wire Line
	2600 6450 2600 6550
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 5DCB9A7C
P 5800 3850
F 0 "J1" H 5772 3824 50  0000 R CNN
F 1 "Conn_01x04_Male" H 5772 3733 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal" H 5800 3850 50  0001 C CNN
F 3 "~" H 5800 3850 50  0001 C CNN
	1    5800 3850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3100 5950 4100 5950
Wire Wire Line
	4100 5950 4100 4050
$Comp
L power:GND #PWR0105
U 1 1 5DCC09FC
P 5400 4250
F 0 "#PWR0105" H 5400 4000 50  0001 C CNN
F 1 "GND" H 5405 4077 50  0000 C CNN
F 2 "" H 5400 4250 50  0001 C CNN
F 3 "" H 5400 4250 50  0001 C CNN
	1    5400 4250
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0106
U 1 1 5DCC1747
P 5200 3550
F 0 "#PWR0106" H 5200 3400 50  0001 C CNN
F 1 "+5V" H 5215 3723 50  0000 C CNN
F 2 "" H 5200 3550 50  0001 C CNN
F 3 "" H 5200 3550 50  0001 C CNN
	1    5200 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 5450 3600 5450
Wire Wire Line
	3600 5450 3600 6050
Connection ~ 3100 5450
Wire Wire Line
	3100 6450 3600 6450
Wire Wire Line
	3600 6450 3600 6350
Connection ~ 3100 6450
Wire Wire Line
	3100 4450 3600 4450
Wire Wire Line
	3600 4450 3600 4350
Connection ~ 3100 4450
Wire Wire Line
	3600 4050 3600 3450
Wire Wire Line
	3600 3450 3100 3450
Connection ~ 3100 3450
Wire Wire Line
	3100 3950 5600 3950
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5DD0D13F
P 4800 3550
F 0 "#FLG0101" H 4800 3625 50  0001 C CNN
F 1 "PWR_FLAG" H 4800 3723 50  0000 C CNN
F 2 "" H 4800 3550 50  0001 C CNN
F 3 "~" H 4800 3550 50  0001 C CNN
	1    4800 3550
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5DD0E731
P 5000 4250
F 0 "#FLG0102" H 5000 4325 50  0001 C CNN
F 1 "PWR_FLAG" H 5000 4423 50  0000 C CNN
F 2 "" H 5000 4250 50  0001 C CNN
F 3 "~" H 5000 4250 50  0001 C CNN
	1    5000 4250
	1    0    0    1   
$EndComp
Wire Wire Line
	4100 4050 5600 4050
Wire Wire Line
	5600 3750 5400 3750
Wire Wire Line
	5400 3750 5400 4150
Wire Wire Line
	5000 4250 5000 4150
Wire Wire Line
	5000 4150 5400 4150
Connection ~ 5400 4150
Wire Wire Line
	5400 4150 5400 4250
Wire Wire Line
	5200 3550 5200 3650
Wire Wire Line
	5200 3850 5600 3850
Wire Wire Line
	4800 3550 4800 3650
Wire Wire Line
	4800 3650 5200 3650
Connection ~ 5200 3650
Wire Wire Line
	5200 3650 5200 3850
Text Notes 2400 7300 0    50   ~ 0
The TLE4905L unipolar magnetic field switches have open collector outputs.\nWhen using 3.3V logic, omit pull-up resistors R1 and R2 and use external pull-up resistors.
$EndSCHEMATC
