EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 8268 11693 portrait
encoding utf-8
Sheet 1 1
Title "Earth Rover"
Date "2020-06-11"
Rev "3"
Comp "Vijfendertig BVBA"
Comment1 "Maarten De Munck"
Comment2 "Joystick board"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L earth-rover-symbols:AnalogMiniJoystick JS1
U 1 1 5DF2B0F3
P 4850 6750
F 0 "JS1" H 4850 7025 50  0000 C CNN
F 1 "Alps Alpine RKJXK122000D" H 4850 6925 50  0000 C CNN
F 2 "earth-rover-footprints:Alps_RKJXK122000D" H 4900 6425 50  0001 C CNN
F 3 "~" H 4900 6425 50  0001 C CNN
	1    4850 6750
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 5DF2BB03
P 3850 7950
F 0 "J1" V 3958 7662 50  0000 R CNN
F 1 "Conn_01x04_Male" V 3913 7662 50  0001 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal" H 3850 7950 50  0001 C CNN
F 3 "~" H 3850 7950 50  0001 C CNN
	1    3850 7950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4450 7150 3750 7150
Wire Wire Line
	3750 7150 3750 7750
Wire Wire Line
	3750 7150 3750 6950
Wire Wire Line
	3750 6950 4450 6950
Connection ~ 3750 7150
Wire Wire Line
	3850 6750 4450 6750
Wire Wire Line
	3950 7750 3950 7250
Wire Wire Line
	4050 7750 4050 6850
Text Notes 3790 7975 3    50   ~ 0
GND
Text Notes 3890 7975 3    50   ~ 0
V+
Text Notes 3990 7975 3    50   ~ 0
X
Text Notes 4090 7975 3    50   ~ 0
Y
Text Label 4150 7250 0    50   ~ 0
X
Text Label 4150 6850 0    50   ~ 0
Y
Text Label 4150 6950 0    50   ~ 0
GND
Text Label 4150 6750 0    50   ~ 0
V+
Wire Wire Line
	4050 6850 4450 6850
Connection ~ 3850 7350
Wire Wire Line
	3850 7350 3850 6750
Wire Wire Line
	3850 7750 3850 7350
Wire Wire Line
	3950 7250 4450 7250
Wire Wire Line
	3850 7350 4450 7350
$EndSCHEMATC
