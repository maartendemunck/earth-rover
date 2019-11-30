EESchema Schematic File Version 4
LIBS:vehicle-control-unit-cache
EELAYER 30 0
EELAYER END
$Descr A4 8268 11693 portrait
encoding utf-8
Sheet 3 3
Title "Earth Rover"
Date "2019-11-29"
Rev "1"
Comp "Vijfendertig BVBA"
Comment1 "Maarten De Munck"
Comment2 "Vehicle Control Unit"
Comment3 "Power supply"
Comment4 ""
$EndDescr
$Comp
L Regulator_Linear:MIC29502WT U4
U 1 1 5E4FA451
P 4750 8000
F 0 "U4" H 4750 8367 50  0000 C CNN
F 1 "MIC29502WT" H 4750 8276 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-5_P3.4x3.7mm_StaggerEven_Lead3.8mm_Vertical" H 4850 7750 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/devicedoc/20005685a.pdf" H 4750 8000 50  0001 C CNN
	1    4750 8000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R16
U 1 1 5E4FA75B
P 5500 8050
F 0 "R16" H 5570 8096 50  0000 L CNN
F 1 "820E" H 5570 8005 50  0000 L CNN
F 2 "" V 5430 8050 50  0001 C CNN
F 3 "~" H 5500 8050 50  0001 C CNN
	1    5500 8050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R17
U 1 1 5E4FA94E
P 5500 8750
F 0 "R17" H 5570 8796 50  0000 L CNN
F 1 "240E" H 5570 8705 50  0000 L CNN
F 2 "" V 5430 8750 50  0001 C CNN
F 3 "~" H 5500 8750 50  0001 C CNN
	1    5500 8750
	1    0    0    -1  
$EndComp
$Comp
L Device:R_POT_TRIM RV1
U 1 1 5E4FAFA2
P 5500 8400
F 0 "RV1" H 5430 8446 50  0000 R CNN
F 1 "100E" H 5430 8355 50  0000 R CNN
F 2 "" H 5500 8400 50  0001 C CNN
F 3 "~" H 5500 8400 50  0001 C CNN
	1    5500 8400
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR034
U 1 1 5E4FB449
P 5500 8950
F 0 "#PWR034" H 5500 8700 50  0001 C CNN
F 1 "GND" H 5505 8777 50  0000 C CNN
F 2 "" H 5500 8950 50  0001 C CNN
F 3 "" H 5500 8950 50  0001 C CNN
	1    5500 8950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR033
U 1 1 5E4FB653
P 4750 8950
F 0 "#PWR033" H 4750 8700 50  0001 C CNN
F 1 "GND" H 4755 8777 50  0000 C CNN
F 2 "" H 4750 8950 50  0001 C CNN
F 3 "" H 4750 8950 50  0001 C CNN
	1    4750 8950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR032
U 1 1 5E4FB89E
P 3100 8950
F 0 "#PWR032" H 3100 8700 50  0001 C CNN
F 1 "GND" H 3105 8777 50  0000 C CNN
F 2 "" H 3100 8950 50  0001 C CNN
F 3 "" H 3100 8950 50  0001 C CNN
	1    3100 8950
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C1
U 1 1 5E4FBB47
P 3100 8400
F 0 "C1" H 3218 8446 50  0000 L CNN
F 1 "10u" H 3218 8355 50  0000 L CNN
F 2 "" H 3138 8250 50  0001 C CNN
F 3 "~" H 3100 8400 50  0001 C CNN
	1    3100 8400
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C2
U 1 1 5E4FBE0F
P 5900 8400
F 0 "C2" H 6018 8446 50  0000 L CNN
F 1 "22u" H 6018 8355 50  0000 L CNN
F 2 "" H 5938 8250 50  0001 C CNN
F 3 "~" H 5900 8400 50  0001 C CNN
	1    5900 8400
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR029
U 1 1 5E4FC127
P 2700 7800
F 0 "#PWR029" H 2700 7650 50  0001 C CNN
F 1 "+BATT" H 2715 7973 50  0000 C CNN
F 2 "" H 2700 7800 50  0001 C CNN
F 3 "" H 2700 7800 50  0001 C CNN
	1    2700 7800
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J13
U 1 1 5E4FCACC
P 2300 8350
F 0 "J13" H 2408 8439 50  0000 C CNN
F 1 "Conn_01x02_Male" H 2408 8440 50  0001 C CNN
F 2 "" H 2300 8350 50  0001 C CNN
F 3 "~" H 2300 8350 50  0001 C CNN
	1    2300 8350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 5E4FCEBE
P 2700 8950
F 0 "#PWR031" H 2700 8700 50  0001 C CNN
F 1 "GND" H 2705 8777 50  0000 C CNN
F 2 "" H 2700 8950 50  0001 C CNN
F 3 "" H 2700 8950 50  0001 C CNN
	1    2700 8950
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR030
U 1 1 5E4FD133
P 6300 7800
F 0 "#PWR030" H 6300 7650 50  0001 C CNN
F 1 "+5V" H 6315 7973 50  0000 C CNN
F 2 "" H 6300 7800 50  0001 C CNN
F 3 "" H 6300 7800 50  0001 C CNN
	1    6300 7800
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5E4FD579
P 2800 7800
F 0 "#FLG01" H 2800 7875 50  0001 C CNN
F 1 "PWR_FLAG" H 2800 7973 50  0001 C CNN
F 2 "" H 2800 7800 50  0001 C CNN
F 3 "~" H 2800 7800 50  0001 C CNN
	1    2800 7800
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5E4FD9B0
P 2800 8950
F 0 "#FLG02" H 2800 9025 50  0001 C CNN
F 1 "PWR_FLAG" H 2800 9123 50  0001 C CNN
F 2 "" H 2800 8950 50  0001 C CNN
F 3 "~" H 2800 8950 50  0001 C CNN
	1    2800 8950
	1    0    0    1   
$EndComp
$Comp
L earth-rover-symbols:Jumper_3_Bridged12 JP6
U 1 1 5E4FDD73
P 4000 8450
F 0 "JP6" V 4000 8517 50  0000 L CNN
F 1 "Jumper_3_Bridged12" V 4045 8517 50  0001 L CNN
F 2 "" H 4000 8450 50  0001 C CNN
F 3 "~" H 4000 8450 50  0001 C CNN
	1    4000 8450
	0    -1   1    0   
$EndComp
$Comp
L Switch:SW_SPST SW1
U 1 1 5E4FF21F
P 3750 8200
F 0 "SW1" H 3750 8343 50  0000 C CNN
F 1 "SW_SPST" H 3750 8344 50  0001 C CNN
F 2 "" H 3750 8200 50  0001 C CNN
F 3 "~" H 3750 8200 50  0001 C CNN
	1    3750 8200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 8950 5500 8900
Wire Wire Line
	5500 8600 5500 8550
Wire Wire Line
	5500 8250 5500 8200
Wire Wire Line
	5150 7900 5500 7900
Wire Wire Line
	5150 8100 5250 8100
Wire Wire Line
	5250 8100 5250 8400
Wire Wire Line
	5250 8400 5350 8400
Wire Wire Line
	5900 8250 5900 7900
Wire Wire Line
	5900 7900 5500 7900
Connection ~ 5500 7900
$Comp
L power:GND #PWR035
U 1 1 5E50947F
P 5900 8950
F 0 "#PWR035" H 5900 8700 50  0001 C CNN
F 1 "GND" H 5905 8777 50  0000 C CNN
F 2 "" H 5900 8950 50  0001 C CNN
F 3 "" H 5900 8950 50  0001 C CNN
	1    5900 8950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 8950 5900 8550
Wire Wire Line
	6300 7900 5900 7900
Connection ~ 5900 7900
Wire Wire Line
	6300 7800 6300 7900
Wire Wire Line
	4750 8950 4750 8300
Wire Wire Line
	3950 8200 4000 8200
Wire Wire Line
	4150 8450 4250 8450
Wire Wire Line
	4250 8450 4250 8100
Wire Wire Line
	4250 8100 4350 8100
Wire Wire Line
	4000 8700 3600 8700
Text HLabel 3600 8700 0    50   Input ~ 0
BEC
Wire Wire Line
	3550 8200 3500 8200
Wire Wire Line
	3500 8200 3500 7900
Wire Wire Line
	3500 7900 4350 7900
Wire Wire Line
	3100 8250 3100 7900
Wire Wire Line
	3100 7900 3500 7900
Connection ~ 3500 7900
Wire Wire Line
	3100 8950 3100 8550
Wire Wire Line
	2700 7800 2700 7900
Wire Wire Line
	2700 7900 2800 7900
Connection ~ 3100 7900
Wire Wire Line
	2800 7800 2800 7900
Connection ~ 2800 7900
Wire Wire Line
	2800 7900 3100 7900
Wire Wire Line
	2500 8350 2700 8350
Wire Wire Line
	2700 8350 2700 7900
Connection ~ 2700 7900
Wire Wire Line
	2500 8450 2700 8450
Wire Wire Line
	2700 8450 2700 8850
Wire Wire Line
	2800 8950 2800 8850
Wire Wire Line
	2800 8850 2700 8850
Connection ~ 2700 8850
Wire Wire Line
	2700 8850 2700 8950
Text Notes 3400 8950 0    50   ~ 0
Jumper selects switch or BEC\npower supply to switch power
$EndSCHEMATC
