EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 8268 11693 portrait
encoding utf-8
Sheet 2 3
Title "Earth Rover"
Date "2020-08-05"
Rev "9"
Comp "Vijfendertig BVBA"
Comment1 "Maarten De Munck"
Comment2 "Vehicle Control Unit"
Comment3 "Automotive Lighting"
Comment4 ""
$EndDescr
$Comp
L Device:R R9
U 1 1 5DE05301
P 2400 7350
F 0 "R9" H 2470 7396 50  0000 L CNN
F 1 "100k" H 2470 7305 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2330 7350 50  0001 C CNN
F 3 "~" H 2400 7350 50  0001 C CNN
	1    2400 7350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 5DE05680
P 2150 7100
F 0 "R8" V 1943 7100 50  0000 C CNN
F 1 "10k" V 2034 7100 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2080 7100 50  0001 C CNN
F 3 "~" H 2150 7100 50  0001 C CNN
	1    2150 7100
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 5DE05B60
P 2650 6150
F 0 "R6" H 2720 6196 50  0000 L CNN
F 1 "10E" H 2720 6105 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2580 6150 50  0001 C CNN
F 3 "~" H 2650 6150 50  0001 C CNN
	1    2650 6150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5DE05ED0
P 2950 6550
F 0 "R7" H 3020 6596 50  0000 L CNN
F 1 "10E" H 3020 6505 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2880 6550 50  0001 C CNN
F 3 "~" H 2950 6550 50  0001 C CNN
	1    2950 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 6700 2950 6800
Wire Wire Line
	2950 6800 2800 6800
Wire Wire Line
	2650 6800 2650 6300
Wire Wire Line
	2800 6800 2800 6900
Connection ~ 2800 6800
Wire Wire Line
	2800 6800 2650 6800
Wire Wire Line
	2500 7100 2400 7100
Wire Wire Line
	2400 7200 2400 7100
Connection ~ 2400 7100
Wire Wire Line
	2400 7100 2300 7100
$Comp
L power:GND #PWR030
U 1 1 5DE2147C
P 2400 7600
F 0 "#PWR030" H 2400 7350 50  0001 C CNN
F 1 "GND" H 2405 7427 50  0000 C CNN
F 2 "" H 2400 7600 50  0001 C CNN
F 3 "" H 2400 7600 50  0001 C CNN
	1    2400 7600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 5DE219A4
P 2800 7600
F 0 "#PWR031" H 2800 7350 50  0001 C CNN
F 1 "GND" H 2805 7427 50  0000 C CNN
F 2 "" H 2800 7600 50  0001 C CNN
F 3 "" H 2800 7600 50  0001 C CNN
	1    2800 7600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 7600 2800 7300
Wire Wire Line
	2400 7600 2400 7500
Text HLabel 1900 7100 0    50   Input ~ 0
HEADLAMPS
$Comp
L Transistor_BJT:BC517 Q1
U 1 1 5E5387A1
P 2700 7100
F 0 "Q1" H 2891 7146 50  0000 L CNN
F 1 "BC517" H 2891 7055 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 2900 7025 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC517.pdf" H 2700 7100 50  0001 L CNN
	1    2700 7100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 5DE0C01C
P 3300 7850
F 0 "R10" H 3370 7896 50  0000 L CNN
F 1 "330E" H 3370 7805 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3230 7850 50  0001 C CNN
F 3 "~" H 3300 7850 50  0001 C CNN
	1    3300 7850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 5DE0EB12
P 3600 8250
F 0 "R14" H 3670 8296 50  0000 L CNN
F 1 "330E" H 3670 8205 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3530 8250 50  0001 C CNN
F 3 "~" H 3600 8250 50  0001 C CNN
	1    3600 8250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 5EA8801C
P 4500 7850
F 0 "R12" H 4570 7896 50  0000 L CNN
F 1 "180E" H 4570 7805 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4430 7850 50  0001 C CNN
F 3 "~" H 4500 7850 50  0001 C CNN
	1    4500 7850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 5EA5792D
P 5100 7850
F 0 "R13" H 5170 7896 50  0000 L CNN
F 1 "180E" H 5170 7805 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5030 7850 50  0001 C CNN
F 3 "~" H 5100 7850 50  0001 C CNN
	1    5100 7850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 8400 2900 8400
Wire Wire Line
	3000 8200 3000 8400
$Comp
L power:+BATT #PWR032
U 1 1 5DE4AC10
P 3000 8200
F 0 "#PWR032" H 3000 8050 50  0001 C CNN
F 1 "+BATT" H 3015 8373 50  0000 C CNN
F 2 "" H 3000 8200 50  0001 C CNN
F 3 "" H 3000 8200 50  0001 C CNN
	1    3000 8200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R11
U 1 1 5DE096EB
P 3900 7850
F 0 "R11" H 3970 7896 50  0000 L CNN
F 1 "470E" H 3970 7805 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3830 7850 50  0001 C CNN
F 3 "~" H 3900 7850 50  0001 C CNN
	1    3900 7850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 8500 3900 8500
$Comp
L Device:R R15
U 1 1 5DE09B97
P 4200 8250
F 0 "R15" H 4270 8296 50  0000 L CNN
F 1 "82E" H 4270 8205 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4130 8250 50  0001 C CNN
F 3 "~" H 4200 8250 50  0001 C CNN
	1    4200 8250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 8700 4200 8700
$Comp
L Device:R R16
U 1 1 5DE0CEAF
P 4800 8250
F 0 "R16" H 4870 8296 50  0000 L CNN
F 1 "330E" H 4870 8205 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4730 8250 50  0001 C CNN
F 3 "~" H 4800 8250 50  0001 C CNN
	1    4800 8250
	1    0    0    -1  
$EndComp
Text HLabel 1900 9100 0    50   Input ~ 0
RIGHT_TURN_SIGNAL
Text HLabel 1900 8900 0    50   Input ~ 0
LEFT_TURN_SIGNAL
Text HLabel 1900 8700 0    50   Input ~ 0
STOP_LAMPS
Text HLabel 1900 8500 0    50   Input ~ 0
POSITION_LAMPS
Wire Wire Line
	2500 9500 2500 9400
$Comp
L power:GND #PWR035
U 1 1 5DEC191A
P 2500 9500
F 0 "#PWR035" H 2500 9250 50  0001 C CNN
F 1 "GND" H 2505 9327 50  0000 C CNN
F 2 "" H 2500 9500 50  0001 C CNN
F 3 "" H 2500 9500 50  0001 C CNN
	1    2500 9500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR034
U 1 1 5DEBE138
P 6300 9400
F 0 "#PWR034" H 6300 9150 50  0001 C CNN
F 1 "GND" H 6305 9227 50  0000 C CNN
F 2 "" H 6300 9400 50  0001 C CNN
F 3 "" H 6300 9400 50  0001 C CNN
	1    6300 9400
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR033
U 1 1 5DEBDC35
P 6300 8700
F 0 "#PWR033" H 6300 8550 50  0001 C CNN
F 1 "+BATT" H 6315 8873 50  0000 C CNN
F 2 "" H 6300 8700 50  0001 C CNN
F 3 "" H 6300 8700 50  0001 C CNN
	1    6300 8700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 9300 6300 9400
Wire Wire Line
	6400 9300 6300 9300
Wire Wire Line
	6300 8800 6300 8700
Wire Wire Line
	6400 8800 6300 8800
Text Notes 6650 8700 0    50   ~ 0
To trailer connector\n(resistors in trailer):
Text Notes 6650 9325 0    50   ~ 0
GND
Text Notes 6650 9225 0    50   ~ 0
Right turn signal sink
Text Notes 6650 9125 0    50   ~ 0
Left turn signal sink
Text Notes 6650 9025 0    50   ~ 0
Stop lamps sink
Text Notes 6650 8925 0    50   ~ 0
Position lamps sink
Text Notes 6650 8825 0    50   ~ 0
+BATT
$Comp
L Device:R R17
U 1 1 5DE0EB26
P 5400 8250
F 0 "R17" H 5470 8296 50  0000 L CNN
F 1 "330E" H 5470 8205 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5330 8250 50  0001 C CNN
F 3 "~" H 5400 8250 50  0001 C CNN
	1    5400 8250
	1    0    0    -1  
$EndComp
$Comp
L Transistor_Array:ULN2803A U3
U 1 1 5DE029FD
P 2500 8700
F 0 "U3" H 2500 9267 50  0000 C CNN
F 1 "ULN2803A" H 2500 9176 50  0000 C CNN
F 2 "Package_DIP:DIP-18_W7.62mm" H 2550 8050 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/uln2803a.pdf" H 2600 8500 50  0001 C CNN
	1    2500 8700
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR029
U 1 1 5EDEBB0E
P 5400 7000
F 0 "#PWR029" H 5400 6850 50  0001 C CNN
F 1 "+BATT" H 5415 7173 50  0000 C CNN
F 2 "" H 5400 7000 50  0001 C CNN
F 3 "" H 5400 7000 50  0001 C CNN
	1    5400 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 7100 5400 7000
Wire Wire Line
	5500 7100 5400 7100
Text Notes 5750 7000 0    50   ~ 0
To rear lamps:
Text Notes 5750 7625 0    50   ~ 0
Right side turn signal (5mm orange LED) sink
Text Notes 5750 7525 0    50   ~ 0
Right rear turn signal (10mm orange LED) sink
Text Notes 5750 7425 0    50   ~ 0
Left side turn signal (5mm orange LED) sink
Text Notes 5750 7325 0    50   ~ 0
Left rear turn signal (10mm orange LED) sink
Text Notes 5750 7225 0    50   ~ 0
Position lamps (2 * 10mm red LED) sink
Text Notes 5750 7125 0    50   ~ 0
+BATT
$Comp
L power:+BATT #PWR028
U 1 1 5EDFFF12
P 5400 6100
F 0 "#PWR028" H 5400 5950 50  0001 C CNN
F 1 "+BATT" H 5415 6273 50  0000 C CNN
F 2 "" H 5400 6100 50  0001 C CNN
F 3 "" H 5400 6100 50  0001 C CNN
	1    5400 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 6200 5400 6100
Wire Wire Line
	5500 6200 5400 6200
Text Notes 5750 6100 0    50   ~ 0
To front right lamps:
Text Notes 5750 6425 0    50   ~ 0
Right front turn signal (1W white LED) sink
Text Notes 5750 6325 0    50   ~ 0
Right headlamp (1W white LED) sink
Text Notes 5750 6225 0    50   ~ 0
+BATT
$Comp
L Connector:Conn_01x03_Male J8
U 1 1 5EDFFF21
P 5700 6300
F 0 "J8" H 5900 6550 50  0000 R CNN
F 1 "Conn_01x03_Male" H 5672 6183 50  0001 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 5700 6300 50  0001 C CNN
F 3 "~" H 5700 6300 50  0001 C CNN
	1    5700 6300
	-1   0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR027
U 1 1 5EE086C6
P 5400 5200
F 0 "#PWR027" H 5400 5050 50  0001 C CNN
F 1 "+BATT" H 5415 5373 50  0000 C CNN
F 2 "" H 5400 5200 50  0001 C CNN
F 3 "" H 5400 5200 50  0001 C CNN
	1    5400 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 5300 5400 5200
Wire Wire Line
	5500 5300 5400 5300
Text Notes 5750 5200 0    50   ~ 0
To front left lamps:
Text Notes 5750 5525 0    50   ~ 0
Left front turn signal (1W white LED) sink
Text Notes 5750 5425 0    50   ~ 0
Left headlamp (1W white LED) sink
Text Notes 5750 5325 0    50   ~ 0
+BATT
$Comp
L Connector:Conn_01x03_Male J7
U 1 1 5EE086D2
P 5700 5400
F 0 "J7" H 5900 5650 50  0000 R CNN
F 1 "Conn_01x03_Male" H 5672 5283 50  0001 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 5700 5400 50  0001 C CNN
F 3 "~" H 5700 5400 50  0001 C CNN
	1    5700 5400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3300 8000 3300 8900
Wire Wire Line
	3300 8900 2900 8900
Wire Wire Line
	3600 8400 3600 9100
Wire Wire Line
	3300 7700 3300 5500
Wire Wire Line
	3300 5500 5500 5500
Wire Wire Line
	3600 8100 3600 6400
Wire Wire Line
	3600 6400 5500 6400
Wire Wire Line
	3900 8000 3900 8500
Wire Wire Line
	4200 8400 4200 8700
Wire Wire Line
	3900 7700 3900 7600
Wire Wire Line
	3900 7600 4050 7600
Wire Wire Line
	4200 7600 4200 8100
Wire Wire Line
	4050 7600 4050 7200
Wire Wire Line
	4050 7200 5500 7200
Connection ~ 4050 7600
Wire Wire Line
	4050 7600 4200 7600
Wire Wire Line
	4500 7700 4500 7300
Wire Wire Line
	4500 7300 5500 7300
Wire Wire Line
	4800 8100 4800 7400
Wire Wire Line
	4800 7400 5500 7400
Wire Wire Line
	5100 7700 5100 7500
Wire Wire Line
	5100 7500 5500 7500
Wire Wire Line
	4500 8000 4500 8900
Wire Wire Line
	4500 8900 3300 8900
Connection ~ 3300 8900
Wire Wire Line
	4500 8900 4800 8900
Wire Wire Line
	4800 8900 4800 8400
Connection ~ 4500 8900
Wire Wire Line
	3600 9100 5100 9100
Wire Wire Line
	5100 9100 5100 8000
Wire Wire Line
	5100 9100 5400 9100
Wire Wire Line
	5400 9100 5400 8400
Connection ~ 5100 9100
Wire Wire Line
	5500 5400 2650 5400
Wire Wire Line
	2650 5400 2650 6000
Wire Wire Line
	5500 6300 2950 6300
Wire Wire Line
	2950 6300 2950 6400
Wire Wire Line
	2000 7100 1900 7100
$Comp
L earth-rover-symbols:Conn_02x06_Male_UnitPerRow J9
U 2 1 5EF26BB7
P 5700 7300
F 0 "J9" H 5900 7650 50  0000 R CNN
F 1 "Conn_02x06_Male_UnitPerRow" H 5672 7183 50  0001 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x06_P2.54mm_Horizontal" H 5700 7300 50  0001 C CNN
F 3 "~" H 5700 7300 50  0001 C CNN
	2    5700 7300
	-1   0    0    -1  
$EndComp
$Comp
L earth-rover-symbols:Conn_02x06_Male_UnitPerRow J9
U 1 1 5EF2B7C6
P 6600 9000
F 0 "J9" H 6800 9350 50  0000 R CNN
F 1 "Conn_02x06_Male_UnitPerRow" H 6572 8883 50  0001 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x06_P2.54mm_Horizontal" H 6600 9000 50  0001 C CNN
F 3 "~" H 6600 9000 50  0001 C CNN
	1    6600 9000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2900 9100 3600 9100
Connection ~ 3600 9100
Wire Wire Line
	2900 9200 6400 9200
Wire Wire Line
	1900 8700 2000 8700
Wire Wire Line
	1900 9100 2000 9100
Wire Wire Line
	2000 9100 2000 9200
Wire Wire Line
	2000 9200 2100 9200
Connection ~ 2000 9100
Wire Wire Line
	2000 9100 2100 9100
Wire Wire Line
	1900 8500 2000 8500
Wire Wire Line
	1900 8900 2000 8900
Wire Wire Line
	2000 8900 2000 9000
Wire Wire Line
	2000 9000 2100 9000
Connection ~ 2000 8900
Wire Wire Line
	2000 8900 2100 8900
Wire Wire Line
	6100 8900 6100 8600
Wire Wire Line
	6100 8600 2900 8600
Wire Wire Line
	6100 8900 6400 8900
Wire Wire Line
	6400 9000 6000 9000
Wire Wire Line
	6000 9000 6000 8800
Wire Wire Line
	6000 8800 2900 8800
Wire Wire Line
	6400 9100 5900 9100
Wire Wire Line
	5900 9100 5900 9000
Wire Wire Line
	5900 9000 2900 9000
Wire Wire Line
	2000 8500 2000 8600
Wire Wire Line
	2000 8600 2100 8600
Connection ~ 2000 8500
Wire Wire Line
	2000 8500 2100 8500
Wire Wire Line
	2000 8700 2000 8800
Wire Wire Line
	2000 8800 2100 8800
Connection ~ 2000 8700
Wire Wire Line
	2000 8700 2100 8700
Wire Wire Line
	5400 8100 5400 7600
Wire Wire Line
	5400 7600 5500 7600
$EndSCHEMATC
