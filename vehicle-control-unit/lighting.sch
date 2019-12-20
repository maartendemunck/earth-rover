EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 8268 11693 portrait
encoding utf-8
Sheet 2 3
Title "Earth Rover"
Date "2019-12-01"
Rev "2"
Comp "Vijfendertig BVBA"
Comment1 "Maarten De Munck"
Comment2 "Vehicle Control Unit"
Comment3 "Automotive Lighting"
Comment4 ""
$EndDescr
$Comp
L Transistor_Array:ULN2803A U3
U 1 1 5DE029FD
P 3600 5550
F 0 "U3" H 3600 6117 50  0000 C CNN
F 1 "ULN2803A" H 3600 6026 50  0000 C CNN
F 2 "" H 3650 4900 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/uln2803a.pdf" H 3700 5350 50  0001 C CNN
	1    3600 5550
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:IRF540N Q1
U 1 1 5DE047D6
P 3700 8450
F 0 "Q1" H 3904 8496 50  0000 L CNN
F 1 "IRF540N" H 3904 8405 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 3950 8375 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf540n.pdf" H 3700 8450 50  0001 L CNN
	1    3700 8450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5DE05301
P 3400 8700
F 0 "R7" H 3470 8746 50  0000 L CNN
F 1 "10k" H 3470 8655 50  0000 L CNN
F 2 "" V 3330 8700 50  0001 C CNN
F 3 "~" H 3400 8700 50  0001 C CNN
	1    3400 8700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5DE05680
P 3150 8450
F 0 "R6" V 2943 8450 50  0000 C CNN
F 1 "100E" V 3034 8450 50  0000 C CNN
F 2 "" V 3080 8450 50  0001 C CNN
F 3 "~" H 3150 8450 50  0001 C CNN
	1    3150 8450
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 5DE05B60
P 3600 7900
F 0 "R8" H 3670 7946 50  0000 L CNN
F 1 "R" H 3670 7855 50  0000 L CNN
F 2 "" V 3530 7900 50  0001 C CNN
F 3 "~" H 3600 7900 50  0001 C CNN
	1    3600 7900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5DE05ED0
P 4000 7900
F 0 "R9" H 4070 7946 50  0000 L CNN
F 1 "R" H 4070 7855 50  0000 L CNN
F 2 "" V 3930 7900 50  0001 C CNN
F 3 "~" H 4000 7900 50  0001 C CNN
	1    4000 7900
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5DE06587
P 3600 7500
F 0 "D1" V 3639 7383 50  0000 R CNN
F 1 "LED" V 3548 7383 50  0000 R CNN
F 2 "" H 3600 7500 50  0001 C CNN
F 3 "~" H 3600 7500 50  0001 C CNN
	1    3600 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D2
U 1 1 5DE06CA4
P 4000 7500
F 0 "D2" V 4039 7383 50  0000 R CNN
F 1 "LED" V 3948 7383 50  0000 R CNN
F 2 "" H 4000 7500 50  0001 C CNN
F 3 "~" H 4000 7500 50  0001 C CNN
	1    4000 7500
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x06_Male J12
U 1 1 5DE075DD
P 5900 6350
F 0 "J12" H 6100 6700 50  0000 R CNN
F 1 "Conn_01x06_Male" H 5872 6233 50  0001 R CNN
F 2 "" H 5900 6350 50  0001 C CNN
F 3 "~" H 5900 6350 50  0001 C CNN
	1    5900 6350
	-1   0    0    -1  
$EndComp
$Comp
L Device:LED D4
U 1 1 5DE08B62
P 4700 4600
F 0 "D4" V 4775 4575 50  0000 R TNN
F 1 "Red 10mm" H 4700 4700 50  0000 C BNN
F 2 "" H 4700 4600 50  0001 C CNN
F 3 "~" H 4700 4600 50  0001 C CNN
	1    4700 4600
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D3
U 1 1 5DE0A330
P 4700 4000
F 0 "D3" V 4775 3975 50  0000 R TNN
F 1 "Red 10mm" H 4700 4100 50  0000 C BNN
F 2 "" H 4700 4000 50  0001 C CNN
F 3 "~" H 4700 4000 50  0001 C CNN
	1    4700 4000
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D8
U 1 1 5DE0EAFE
P 6100 4000
F 0 "D8" V 6175 3975 50  0000 R TNN
F 1 "Orange 10mm" H 6100 4100 50  0000 C BNN
F 2 "" H 6100 4000 50  0001 C CNN
F 3 "~" H 6100 4000 50  0001 C CNN
	1    6100 4000
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R14
U 1 1 5DE0EB12
P 6100 5100
F 0 "R14" H 6170 5146 50  0000 L CNN
F 1 "R" H 6170 5055 50  0000 L CNN
F 2 "" V 6030 5100 50  0001 C CNN
F 3 "~" H 6100 5100 50  0001 C CNN
	1    6100 5100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R15
U 1 1 5DE0EB26
P 6500 5100
F 0 "R15" H 6570 5146 50  0000 L CNN
F 1 "R" H 6570 5055 50  0000 L CNN
F 2 "" V 6430 5100 50  0001 C CNN
F 3 "~" H 6500 5100 50  0001 C CNN
	1    6500 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 7650 3600 7750
Wire Wire Line
	4000 7650 4000 7750
Wire Wire Line
	4000 8050 4000 8150
Wire Wire Line
	4000 8150 3800 8150
Wire Wire Line
	3600 8150 3600 8050
Wire Wire Line
	3800 8150 3800 8250
Connection ~ 3800 8150
Wire Wire Line
	3800 8150 3600 8150
Wire Wire Line
	3500 8450 3400 8450
Wire Wire Line
	3400 8550 3400 8450
Connection ~ 3400 8450
Wire Wire Line
	3400 8450 3300 8450
$Comp
L power:GND #PWR027
U 1 1 5DE2147C
P 3400 8950
F 0 "#PWR027" H 3400 8700 50  0001 C CNN
F 1 "GND" H 3405 8777 50  0000 C CNN
F 2 "" H 3400 8950 50  0001 C CNN
F 3 "" H 3400 8950 50  0001 C CNN
	1    3400 8950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR030
U 1 1 5DE219A4
P 3800 8950
F 0 "#PWR030" H 3800 8700 50  0001 C CNN
F 1 "GND" H 3805 8777 50  0000 C CNN
F 2 "" H 3800 8950 50  0001 C CNN
F 3 "" H 3800 8950 50  0001 C CNN
	1    3800 8950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 8950 3800 8650
Wire Wire Line
	3400 8950 3400 8850
Wire Wire Line
	3000 8450 2600 8450
$Comp
L power:+BATT #PWR029
U 1 1 5DE22B0F
P 3800 7150
F 0 "#PWR029" H 3800 7000 50  0001 C CNN
F 1 "+BATT" H 3815 7323 50  0000 C CNN
F 2 "" H 3800 7150 50  0001 C CNN
F 3 "" H 3800 7150 50  0001 C CNN
	1    3800 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 7150 3800 7250
Wire Wire Line
	3800 7250 4000 7250
Wire Wire Line
	4000 7250 4000 7350
Wire Wire Line
	3800 7250 3600 7250
Wire Wire Line
	3600 7250 3600 7350
Connection ~ 3800 7250
Wire Wire Line
	4700 4750 4700 4850
Connection ~ 4700 4850
Wire Wire Line
	4700 4150 4700 4450
Wire Wire Line
	6500 4950 6500 4150
Wire Wire Line
	6100 4950 6100 4750
Wire Wire Line
	6100 4450 6100 4150
Wire Wire Line
	6100 5250 6100 5350
Wire Wire Line
	6100 5350 6300 5350
Wire Wire Line
	6500 5350 6500 5250
Wire Wire Line
	5500 5350 5500 5550
Wire Wire Line
	5500 5550 4000 5550
Connection ~ 5500 5350
Wire Wire Line
	6300 5350 6300 5650
Wire Wire Line
	6300 5650 4000 5650
Connection ~ 6300 5350
Wire Wire Line
	6300 5350 6500 5350
$Comp
L Device:LED D9
U 1 1 5DE0EB08
P 6100 4600
F 0 "D9" V 6175 4575 50  0000 R TNN
F 1 "Orange 5mm" H 6100 4700 50  0000 C BNN
F 2 "" H 6100 4600 50  0001 C CNN
F 3 "~" H 6100 4600 50  0001 C CNN
	1    6100 4600
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D10
U 1 1 5DE0EB1C
P 6500 4000
F 0 "D10" V 6575 3975 50  0000 R TNN
F 1 "White 5mm" H 6500 4100 50  0000 C BNN
F 2 "" H 6500 4000 50  0001 C CNN
F 3 "~" H 6500 4000 50  0001 C CNN
	1    6500 4000
	0    -1   -1   0   
$EndComp
$Comp
L power:+BATT #PWR032
U 1 1 5DE93A99
P 4700 3650
F 0 "#PWR032" H 4700 3500 50  0001 C CNN
F 1 "+BATT" H 4715 3823 50  0000 C CNN
F 2 "" H 4700 3650 50  0001 C CNN
F 3 "" H 4700 3650 50  0001 C CNN
	1    4700 3650
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR033
U 1 1 5DE9402F
P 5500 3650
F 0 "#PWR033" H 5500 3500 50  0001 C CNN
F 1 "+BATT" H 5515 3823 50  0000 C CNN
F 2 "" H 5500 3650 50  0001 C CNN
F 3 "" H 5500 3650 50  0001 C CNN
	1    5500 3650
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR036
U 1 1 5DE945F6
P 6300 3650
F 0 "#PWR036" H 6300 3500 50  0001 C CNN
F 1 "+BATT" H 6315 3823 50  0000 C CNN
F 2 "" H 6300 3650 50  0001 C CNN
F 3 "" H 6300 3650 50  0001 C CNN
	1    6300 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 3650 6300 3750
Wire Wire Line
	6300 3750 6100 3750
Wire Wire Line
	6100 3750 6100 3850
Wire Wire Line
	6300 3750 6500 3750
Wire Wire Line
	6500 3750 6500 3850
Connection ~ 6300 3750
Wire Wire Line
	5500 3650 5500 3750
Connection ~ 5500 3750
Wire Wire Line
	4700 3650 4700 3850
Wire Wire Line
	3200 5350 3100 5350
Wire Wire Line
	2600 5450 3000 5450
Wire Wire Line
	3200 5550 2900 5550
Wire Wire Line
	2600 5650 2800 5650
Wire Wire Line
	3100 5350 3100 5750
Wire Wire Line
	3100 5750 3200 5750
Connection ~ 3100 5350
Wire Wire Line
	3100 5350 2600 5350
Wire Wire Line
	3000 5450 3000 5850
Wire Wire Line
	3000 5850 3200 5850
Connection ~ 3000 5450
Wire Wire Line
	3000 5450 3200 5450
Wire Wire Line
	2900 5550 2900 5950
Wire Wire Line
	2900 5950 3200 5950
Connection ~ 2900 5550
Wire Wire Line
	2900 5550 2600 5550
Wire Wire Line
	2800 5650 2800 6050
Wire Wire Line
	2800 6050 3200 6050
Connection ~ 2800 5650
Wire Wire Line
	2800 5650 3200 5650
Text Notes 5950 6175 0    50   ~ 0
+BATT
Text Notes 5950 6275 0    50   ~ 0
Position lamps
Text Notes 5950 6375 0    50   ~ 0
Stop lamps
Text Notes 5950 6475 0    50   ~ 0
Left turn signal
Text Notes 5950 6575 0    50   ~ 0
Right turn signal
Text Notes 5950 6675 0    50   ~ 0
GND
Text Notes 5950 6050 0    50   ~ 0
To trailer connector:
Wire Wire Line
	5700 6150 5600 6150
Wire Wire Line
	5600 6150 5600 6050
Wire Wire Line
	5700 6650 5600 6650
Wire Wire Line
	5600 6650 5600 6750
$Comp
L power:+BATT #PWR034
U 1 1 5DEBDC35
P 5600 6050
F 0 "#PWR034" H 5600 5900 50  0001 C CNN
F 1 "+BATT" H 5615 6223 50  0000 C CNN
F 2 "" H 5600 6050 50  0001 C CNN
F 3 "" H 5600 6050 50  0001 C CNN
	1    5600 6050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR035
U 1 1 5DEBE138
P 5600 6750
F 0 "#PWR035" H 5600 6500 50  0001 C CNN
F 1 "GND" H 5605 6577 50  0000 C CNN
F 2 "" H 5600 6750 50  0001 C CNN
F 3 "" H 5600 6750 50  0001 C CNN
	1    5600 6750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR028
U 1 1 5DEC191A
P 3600 6350
F 0 "#PWR028" H 3600 6100 50  0001 C CNN
F 1 "GND" H 3605 6177 50  0000 C CNN
F 2 "" H 3600 6350 50  0001 C CNN
F 3 "" H 3600 6350 50  0001 C CNN
	1    3600 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 6350 3600 6250
Text HLabel 2600 5350 0    50   Input ~ 0
POSITION_LAMPS
Text HLabel 2600 5450 0    50   Input ~ 0
STOP_LAMPS
Text HLabel 2600 5550 0    50   Input ~ 0
LEFT_TURN_SIGNAL
Text HLabel 2600 5650 0    50   Input ~ 0
RIGHT_TURN_SIGNAL
Text Notes 4400 3350 0    50   ~ 0
Rear position\nand stop lamps
Text Notes 5200 3350 0    50   ~ 0
Left turn signals
Text Notes 6000 3350 0    50   ~ 0
Right turn signals
Text HLabel 2600 8450 0    50   Input ~ 0
HEADLAMPS
Text Notes 2100 7050 0    50   ~ 0
Headlamps\nDipped and main beam using PWM
Wire Wire Line
	5700 3750 5700 3850
Wire Wire Line
	5500 3750 5700 3750
Wire Wire Line
	5500 5350 5700 5350
Wire Wire Line
	5700 5350 5700 5250
Wire Wire Line
	5700 4950 5700 4150
$Comp
L Device:R R13
U 1 1 5DE0CEAF
P 5700 5100
F 0 "R13" H 5770 5146 50  0000 L CNN
F 1 "R" H 5770 5055 50  0000 L CNN
F 2 "" V 5630 5100 50  0001 C CNN
F 3 "~" H 5700 5100 50  0001 C CNN
	1    5700 5100
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D7
U 1 1 5DE0C5C7
P 5700 4000
F 0 "D7" V 5775 3975 50  0000 R TNN
F 1 "White 5mm" H 5700 4100 50  0000 C BNN
F 2 "" H 5700 4000 50  0001 C CNN
F 3 "~" H 5700 4000 50  0001 C CNN
	1    5700 4000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5300 3750 5300 3850
Wire Wire Line
	5500 3750 5300 3750
$Comp
L Device:R R12
U 1 1 5DE0C01C
P 5300 5100
F 0 "R12" H 5370 5146 50  0000 L CNN
F 1 "R" H 5370 5055 50  0000 L CNN
F 2 "" V 5230 5100 50  0001 C CNN
F 3 "~" H 5300 5100 50  0001 C CNN
	1    5300 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 5250 5300 5350
Wire Wire Line
	5300 4950 5300 4750
Wire Wire Line
	5300 5350 5500 5350
Wire Wire Line
	5300 4450 5300 4150
$Comp
L Device:LED D6
U 1 1 5DE0B3F5
P 5300 4600
F 0 "D6" V 5375 4575 50  0000 R TNN
F 1 "Orange 5mm" H 5300 4700 50  0000 C BNN
F 2 "" H 5300 4600 50  0001 C CNN
F 3 "~" H 5300 4600 50  0001 C CNN
	1    5300 4600
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D5
U 1 1 5DE0AB28
P 5300 4000
F 0 "D5" V 5375 3975 50  0000 R TNN
F 1 "Orange 10mm" H 5300 4100 50  0000 C BNN
F 2 "" H 5300 4000 50  0001 C CNN
F 3 "~" H 5300 4000 50  0001 C CNN
	1    5300 4000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4900 5450 4900 5250
Wire Wire Line
	4000 5450 4900 5450
Wire Wire Line
	4700 4850 4900 4850
Wire Wire Line
	4900 4850 4900 4950
$Comp
L Device:R R11
U 1 1 5DE09B97
P 4900 5100
F 0 "R11" H 4970 5146 50  0000 L CNN
F 1 "R" H 4970 5055 50  0000 L CNN
F 2 "" V 4830 5100 50  0001 C CNN
F 3 "~" H 4900 5100 50  0001 C CNN
	1    4900 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 5350 4500 5250
Wire Wire Line
	4000 5350 4500 5350
Wire Wire Line
	4500 4850 4700 4850
Wire Wire Line
	4500 4950 4500 4850
$Comp
L Device:R R10
U 1 1 5DE096EB
P 4500 5100
F 0 "R10" H 4570 5146 50  0000 L CNN
F 1 "R" H 4570 5055 50  0000 L CNN
F 2 "" V 4430 5100 50  0001 C CNN
F 3 "~" H 4500 5100 50  0001 C CNN
	1    4500 5100
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR031
U 1 1 5DE4AC10
P 4100 5050
F 0 "#PWR031" H 4100 4900 50  0001 C CNN
F 1 "+BATT" H 4115 5223 50  0000 C CNN
F 2 "" H 4100 5050 50  0001 C CNN
F 3 "" H 4100 5050 50  0001 C CNN
	1    4100 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 5050 4100 5250
Wire Wire Line
	4100 5250 4000 5250
Wire Wire Line
	5700 6250 5300 6250
Wire Wire Line
	5300 6250 5300 5750
Wire Wire Line
	5300 5750 4000 5750
Wire Wire Line
	4000 5850 5200 5850
Wire Wire Line
	5200 5850 5200 6350
Wire Wire Line
	5200 6350 5700 6350
Wire Wire Line
	5700 6450 5100 6450
Wire Wire Line
	5100 6450 5100 5950
Wire Wire Line
	5100 5950 4000 5950
Wire Wire Line
	4000 6050 5000 6050
Wire Wire Line
	5000 6050 5000 6550
Wire Wire Line
	5000 6550 5700 6550
$EndSCHEMATC
