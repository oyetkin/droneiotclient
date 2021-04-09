EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	5700 3950 5100 3950
Connection ~ 4800 4850
Wire Wire Line
	5700 4050 5350 4050
Wire Wire Line
	4550 3850 4550 4250
Wire Wire Line
	4650 3550 4650 4150
$Comp
L power:PWR_FLAG #FLG01
U 1 1 604ABF72
P 5100 3950
F 0 "#FLG01" H 5100 4025 50  0001 C CNN
F 1 "PWR_FLAG" H 5100 4123 50  0000 C CNN
F 2 "" H 5100 3950 50  0001 C CNN
F 3 "~" H 5100 3950 50  0001 C CNN
	1    5100 3950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x15_Female ESP1
U 1 1 604B94AB
P 2150 4150
F 0 "ESP1" H 2400 4050 50  0000 C CNN
F 1 "ESP 32 Left VIN GND to VP EN" V 2250 4100 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x15_P2.54mm_Vertical" H 2150 4150 50  0001 C CNN
F 3 "~" H 2150 4150 50  0001 C CNN
	1    2150 4150
	-1   0    0    1   
$EndComp
Wire Wire Line
	4350 3850 4400 3850
Wire Wire Line
	4350 3550 4500 3550
$Comp
L Connector:Conn_01x04_Female BME280
U 1 1 604E2AAF
P 5900 4050
F 0 "BME280" H 5800 4350 50  0000 L CNN
F 1 "BMP180 VIN GND SCL SDA" H 5500 4250 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 5900 4050 50  0001 C CNN
F 3 "~" H 5900 4050 50  0001 C CNN
	1    5900 4050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female OLED1
U 1 1 604E4BCE
P 5900 4950
F 0 "OLED1" H 5900 4550 50  0000 L CNN
F 1 "SSD1306 GND VCC SCL SDA" H 5400 4650 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 5900 4950 50  0001 C CNN
F 3 "~" H 5900 4950 50  0001 C CNN
	1    5900 4950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x15_Female ESP2
U 1 1 604EB918
P 4150 4150
F 0 "ESP2" H 4042 3225 50  0000 C CNN
F 1 "ESP32 Right 3V3 GND to D22 D23" V 4250 4100 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x15_P2.54mm_Vertical" H 4150 4150 50  0001 C CNN
F 3 "~" H 4150 4150 50  0001 C CNN
	1    4150 4150
	-1   0    0    1   
$EndComp
Text Label 4600 4750 0    50   ~ 0
GND
Text Label 4450 3850 0    50   ~ 0
SDA
Text Label 4450 3550 0    50   ~ 0
SCL
Connection ~ 4450 4750
$Comp
L Device:R R1
U 1 1 604D89C5
P 3800 5350
F 0 "R1" H 3870 5396 50  0000 L CNN
F 1 "10K" H 3870 5305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 3730 5350 50  0001 C CNN
F 3 "~" H 3800 5350 50  0001 C CNN
	1    3800 5350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 604DC4F8
P 3800 5650
F 0 "R2" H 3870 5696 50  0000 L CNN
F 1 "10K" H 3870 5605 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 3730 5650 50  0001 C CNN
F 3 "~" H 3800 5650 50  0001 C CNN
	1    3800 5650
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push Trigger1
U 1 1 604FFFBF
P 3000 5650
F 0 "Trigger1" H 3046 5602 50  0000 R CNN
F 1 "SW_Push" H 3150 5500 50  0000 R CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 3000 5850 50  0001 C CNN
F 3 "~" H 3000 5850 50  0001 C CNN
	1    3000 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 5000 3500 5350
Wire Wire Line
	3650 5350 3500 5350
Connection ~ 3500 5350
Wire Wire Line
	3950 5350 4450 5350
Wire Wire Line
	3950 5650 4100 5650
Wire Wire Line
	4450 5650 4450 5350
Connection ~ 4450 5350
Wire Wire Line
	3200 5650 3400 5650
Wire Wire Line
	3400 5650 3400 5100
Wire Wire Line
	3400 5100 2400 5100
Wire Wire Line
	2400 5100 2400 4650
Wire Wire Line
	2400 4650 2350 4650
Connection ~ 3400 5650
Wire Wire Line
	3400 5650 3650 5650
Wire Wire Line
	4650 4850 4650 5950
Wire Wire Line
	2700 5950 2700 5650
Connection ~ 4650 4850
Wire Wire Line
	4650 4850 4750 4850
Wire Wire Line
	2800 5650 2700 5650
Connection ~ 2700 5650
Wire Wire Line
	2700 5650 2700 5350
Text Label 3100 5950 0    50   ~ 0
PWR
Wire Wire Line
	2500 5000 3500 5000
Wire Wire Line
	2350 4350 2500 4350
$Comp
L Device:R R4
U 1 1 605E82CC
P 4800 6400
F 0 "R4" V 5007 6400 50  0000 C CNN
F 1 "100K" V 4916 6400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 4730 6400 50  0001 C CNN
F 3 "~" H 4800 6400 50  0001 C CNN
	1    4800 6400
	0    -1   -1   0   
$EndComp
Connection ~ 4100 5650
Wire Wire Line
	4100 5650 4450 5650
$Comp
L Device:R R3
U 1 1 6060242F
P 3400 6100
F 0 "R3" V 3200 6100 50  0000 C CNN
F 1 "10K" V 3300 6100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 3330 6100 50  0001 C CNN
F 3 "~" H 3400 6100 50  0001 C CNN
	1    3400 6100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2500 4350 2500 5000
Wire Wire Line
	2350 4450 2450 4450
$Comp
L Transistor_BJT:2N2219 Q1
U 1 1 605D56C8
P 3900 6200
F 0 "Q1" V 4228 6200 50  0000 C CNN
F 1 "2N2219" V 3900 5900 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23_Handsoldering" H 4100 6125 50  0001 L CIN
F 3 "http://www.onsemi.com/pub_link/Collateral/2N2219-D.PDF" H 3900 6200 50  0001 L CNN
	1    3900 6200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4100 5650 4100 6100
Wire Wire Line
	2700 5950 3100 5950
Wire Wire Line
	3100 5950 3100 6100
Connection ~ 3100 5950
Wire Wire Line
	3100 5950 4650 5950
Wire Wire Line
	3250 6100 3100 6100
Wire Wire Line
	3550 6100 3600 6100
Wire Wire Line
	3600 6100 3600 6350
Wire Wire Line
	3600 6350 2450 6350
Wire Wire Line
	2450 4450 2450 6350
Connection ~ 3600 6100
Wire Wire Line
	3600 6100 3700 6100
Wire Wire Line
	4650 6400 3900 6400
$Comp
L Connector:Conn_01x03_Female IR_REC1
U 1 1 60683231
P 6250 5950
F 0 "IR_REC1" H 6278 5976 50  0000 L CNN
F 1 "IR_RECVR_GND_VIN_DAT" H 6278 5885 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 6250 5950 50  0001 C CNN
F 3 "~" H 6250 5950 50  0001 C CNN
	1    6250 5950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female I2C1
U 1 1 605AD1B2
P 5950 3450
F 0 "I2C1" H 5850 3750 50  0000 L CNN
F 1 "MISC VIN GND SCL SDA" H 5550 3650 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 5950 3450 50  0001 C CNN
F 3 "~" H 5950 3450 50  0001 C CNN
	1    5950 3450
	1    0    0    -1  
$EndComp
Connection ~ 4800 3950
Wire Wire Line
	5350 4050 5350 3450
Wire Wire Line
	5350 3450 5750 3450
Connection ~ 5350 4050
Wire Wire Line
	5450 4150 5450 3550
Wire Wire Line
	5450 3550 5750 3550
Connection ~ 5450 4150
Wire Wire Line
	5450 4150 5700 4150
Wire Wire Line
	5550 4250 5550 3650
Wire Wire Line
	5550 3650 5750 3650
Connection ~ 5550 4250
Wire Wire Line
	5550 4250 5700 4250
Wire Wire Line
	4800 3950 4800 4850
$Comp
L Connector:Conn_01x15_Female J7
U 1 1 606017A5
P 2650 4150
F 0 "J7" H 2542 3225 50  0000 C CNN
F 1 "BRKOUT_LEFT" H 2542 3316 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x15_P2.54mm_Vertical" H 2650 4150 50  0001 C CNN
F 3 "~" H 2650 4150 50  0001 C CNN
	1    2650 4150
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x15_Female J8
U 1 1 6060AB5F
P 3550 4150
F 0 "J8" H 3442 3225 50  0000 C CNN
F 1 "BRKOUT_RIGHT" H 3442 3316 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x15_P2.54mm_Vertical" H 3550 4150 50  0001 C CNN
F 3 "~" H 3550 4150 50  0001 C CNN
	1    3550 4150
	-1   0    0    1   
$EndComp
Wire Wire Line
	3750 4850 4350 4850
Connection ~ 4350 4850
Wire Wire Line
	3750 4750 4350 4750
Connection ~ 4350 4750
Wire Wire Line
	4350 4750 4450 4750
Wire Wire Line
	3750 4650 4350 4650
Wire Wire Line
	4350 4550 3750 4550
Wire Wire Line
	3750 4450 4350 4450
Wire Wire Line
	4350 4350 3750 4350
Wire Wire Line
	3750 4250 4350 4250
Wire Wire Line
	4350 4150 3750 4150
Wire Wire Line
	3750 4050 4350 4050
Wire Wire Line
	4350 3950 3750 3950
Wire Wire Line
	3750 3850 4350 3850
Connection ~ 4350 3850
Wire Wire Line
	4350 3750 3750 3750
Wire Wire Line
	3750 3650 4350 3650
Wire Wire Line
	4350 3550 3750 3550
Connection ~ 4350 3550
Wire Wire Line
	4350 3450 3750 3450
Wire Wire Line
	2350 3450 2850 3450
Wire Wire Line
	2850 3550 2350 3550
Wire Wire Line
	2350 3650 2850 3650
Wire Wire Line
	2850 3750 2350 3750
Wire Wire Line
	2350 3850 2850 3850
Wire Wire Line
	2850 3950 2350 3950
Wire Wire Line
	2350 4050 2850 4050
Wire Wire Line
	2850 4150 2350 4150
Wire Wire Line
	2350 4250 2850 4250
Wire Wire Line
	2500 4350 2850 4350
Connection ~ 2500 4350
Wire Wire Line
	2450 4450 2850 4450
Connection ~ 2450 4450
Wire Wire Line
	2350 4550 2850 4550
Wire Wire Line
	2850 4650 2400 4650
Connection ~ 2400 4650
Wire Wire Line
	2350 4750 2850 4750
Wire Wire Line
	2850 4850 2350 4850
Wire Wire Line
	4450 4750 4450 5350
Wire Wire Line
	4350 4850 4400 4850
Wire Wire Line
	4800 3350 4800 3950
Wire Wire Line
	5350 4050 5350 4750
Wire Wire Line
	5700 4750 5700 4850
Connection ~ 5350 4750
Wire Wire Line
	4800 4850 5500 4850
Wire Wire Line
	5650 4850 5650 4950
Wire Wire Line
	5650 4950 5700 4950
Connection ~ 5100 3950
Wire Wire Line
	5100 3950 4800 3950
Wire Wire Line
	4650 4150 5150 4150
Wire Wire Line
	4550 4250 5000 4250
Wire Wire Line
	5000 4250 5000 5150
Wire Wire Line
	5000 5150 5700 5150
Connection ~ 5000 4250
Wire Wire Line
	5000 4250 5550 4250
Wire Wire Line
	5150 4150 5150 5050
Wire Wire Line
	5150 5050 5700 5050
Connection ~ 5150 4150
Wire Wire Line
	5150 4150 5450 4150
$Comp
L Connector:TestPoint 3.3V1
U 1 1 607606D6
P 4950 3350
F 0 "3.3V1" H 4900 3550 50  0000 L CNN
F 1 "TestPoint" V 4850 3350 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D2.0mm_Drill1.0mm" H 5150 3350 50  0001 C CNN
F 3 "~" H 5150 3350 50  0001 C CNN
	1    4950 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 3350 4950 3350
Wire Wire Line
	4950 3350 5150 3350
Connection ~ 4950 3350
$Comp
L Connector:TestPoint 3.3V2
U 1 1 6076206D
P 5150 3350
F 0 "3.3V2" H 5100 3550 50  0000 L CNN
F 1 "TestPoint" V 5050 3350 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D2.0mm_Drill1.0mm" H 5350 3350 50  0001 C CNN
F 3 "~" H 5350 3350 50  0001 C CNN
	1    5150 3350
	1    0    0    -1  
$EndComp
Connection ~ 5150 3350
$Comp
L Connector:TestPoint GND1
U 1 1 60762F17
P 5300 4750
F 0 "GND1" H 5250 4950 50  0000 L CNN
F 1 "TestPoint" V 5200 4750 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D2.0mm_Drill1.0mm" H 5500 4750 50  0001 C CNN
F 3 "~" H 5500 4750 50  0001 C CNN
	1    5300 4750
	1    0    0    -1  
$EndComp
Connection ~ 5300 4750
Wire Wire Line
	5300 4750 5350 4750
$Comp
L Connector:TestPoint GND2
U 1 1 6076314C
P 5700 4750
F 0 "GND2" H 5650 4950 50  0000 L CNN
F 1 "TestPoint" V 5600 4750 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D2.0mm_Drill1.0mm" H 5900 4750 50  0001 C CNN
F 3 "~" H 5900 4750 50  0001 C CNN
	1    5700 4750
	1    0    0    -1  
$EndComp
Connection ~ 5700 4750
Text Label 2550 4850 0    50   ~ 0
5VIN
Text Label 2550 4750 0    50   ~ 0
5VGD
Wire Wire Line
	2350 4750 1450 4650
Connection ~ 2350 4750
Wire Wire Line
	2350 4850 1450 5000
Connection ~ 2350 4850
$Comp
L Connector:TestPoint 5V_IN1
U 1 1 605CDAFA
P 1450 5000
F 0 "5V_IN1" H 1300 5200 50  0000 L CNN
F 1 "TestPoint" V 1450 5150 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D2.0mm_Drill1.0mm" H 1650 5000 50  0001 C CNN
F 3 "~" H 1650 5000 50  0001 C CNN
	1    1450 5000
	0    -1   -1   0   
$EndComp
$Comp
L Connector:TestPoint 5V_GD1
U 1 1 605CE0C4
P 1450 4650
F 0 "5V_GD1" H 1400 4850 50  0000 L CNN
F 1 "TestPoint" V 1550 4650 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D2.0mm_Drill1.0mm" H 1650 4650 50  0001 C CNN
F 3 "~" H 1650 4650 50  0001 C CNN
	1    1450 4650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4900 4750 4900 5850
Wire Wire Line
	4900 5850 5600 5850
Connection ~ 4900 4750
Wire Wire Line
	4900 4750 5050 4750
Wire Wire Line
	4750 4850 4750 5950
Wire Wire Line
	4750 5950 5000 5950
Connection ~ 4750 4850
Wire Wire Line
	4750 4850 4800 4850
Wire Wire Line
	4950 6400 5250 6400
Wire Wire Line
	5150 3350 5750 3350
Wire Wire Line
	5350 4750 5700 4750
$Comp
L Switch:SW_Push Mode1
U 1 1 6063A1F9
P 3000 5350
F 0 "Mode1" H 3046 5302 50  0000 R CNN
F 1 "SW_Push" H 3150 5200 50  0000 R CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 3000 5550 50  0001 C CNN
F 3 "~" H 3000 5550 50  0001 C CNN
	1    3000 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 5350 3500 5350
Wire Wire Line
	2800 5350 2700 5350
$Comp
L Connector:TestPoint 5V_IN2
U 1 1 60657975
P 1450 5000
F 0 "5V_IN2" H 1300 5200 50  0000 L CNN
F 1 "TestPoint" V 1450 5150 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D2.0mm_Drill1.0mm" H 1650 5000 50  0001 C CNN
F 3 "~" H 1650 5000 50  0001 C CNN
	1    1450 5000
	-1   0    0    1   
$EndComp
Connection ~ 1450 5000
$Comp
L Connector:TestPoint 5V_GD2
U 1 1 60658198
P 1450 4650
F 0 "5V_GD2" H 1300 4850 50  0000 L CNN
F 1 "TestPoint" V 1450 4800 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D2.0mm_Drill1.0mm" H 1650 4650 50  0001 C CNN
F 3 "~" H 1650 4650 50  0001 C CNN
	1    1450 4650
	1    0    0    -1  
$EndComp
Connection ~ 1450 4650
$Comp
L Device:R R5
U 1 1 605D37E2
P 4400 4000
F 0 "R5" H 4470 4046 50  0000 L CNN
F 1 "2400" H 4470 3955 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 4330 4000 50  0001 C CNN
F 3 "~" H 4400 4000 50  0001 C CNN
	1    4400 4000
	1    0    0    -1  
$EndComp
Connection ~ 4400 3850
Wire Wire Line
	4400 3850 4550 3850
Wire Wire Line
	4400 4150 4400 4850
Connection ~ 4400 4850
Wire Wire Line
	4400 4850 4500 4850
$Comp
L Device:R R6
U 1 1 605D9489
P 4500 4500
F 0 "R6" H 4570 4546 50  0000 L CNN
F 1 "2400" H 4570 4455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 4430 4500 50  0001 C CNN
F 3 "~" H 4500 4500 50  0001 C CNN
	1    4500 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 3550 4500 4350
Connection ~ 4500 3550
Wire Wire Line
	4500 3550 4650 3550
Connection ~ 4500 4850
Wire Wire Line
	4500 4850 4650 4850
Wire Wire Line
	4500 4650 4500 4850
$Comp
L Mechanical:MountingHole H2
U 1 1 6062E33E
P 3000 2350
F 0 "H2" H 3100 2396 50  0000 L CNN
F 1 "MountingHole" H 3100 2305 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 3000 2350 50  0001 C CNN
F 3 "~" H 3000 2350 50  0001 C CNN
	1    3000 2350
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 6062E7A6
P 3800 2300
F 0 "H3" H 3900 2346 50  0000 L CNN
F 1 "MountingHole" H 3900 2255 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 3800 2300 50  0001 C CNN
F 3 "~" H 3800 2300 50  0001 C CNN
	1    3800 2300
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 6062E97D
P 3800 2650
F 0 "H4" H 3900 2696 50  0000 L CNN
F 1 "MountingHole" H 3900 2605 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 3800 2650 50  0001 C CNN
F 3 "~" H 3800 2650 50  0001 C CNN
	1    3800 2650
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H1
U 1 1 6062EB1C
P 2950 2650
F 0 "H1" H 3050 2696 50  0000 L CNN
F 1 "MountingHole" H 3050 2605 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 2950 2650 50  0001 C CNN
F 3 "~" H 2950 2650 50  0001 C CNN
	1    2950 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 4750 4900 4750
Text Label 5350 3350 0    50   ~ 0
3V3
$Comp
L Transistor_BJT:2N2219 Q2
U 1 1 60640972
P 7000 4600
F 0 "Q2" H 7190 4646 50  0000 L CNN
F 1 "2N2219" H 7190 4555 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23_Handsoldering" H 7200 4525 50  0001 L CIN
F 3 "http://www.onsemi.com/pub_link/Collateral/2N2219-D.PDF" H 7000 4600 50  0001 L CNN
	1    7000 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 4850 5500 4400
Wire Wire Line
	5500 4400 7100 4400
Connection ~ 5500 4850
Wire Wire Line
	5500 4850 5650 4850
$Comp
L Device:R R9
U 1 1 6064E0C0
P 7400 4950
F 0 "R9" H 7470 4996 50  0000 L CNN
F 1 "220" H 7470 4905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 7330 4950 50  0001 C CNN
F 3 "~" H 7400 4950 50  0001 C CNN
	1    7400 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 6064EAB3
P 6900 4950
F 0 "R8" H 6970 4996 50  0000 L CNN
F 1 "220" H 6970 4905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 6830 4950 50  0001 C CNN
F 3 "~" H 6900 4950 50  0001 C CNN
	1    6900 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 4750 5050 5450
Connection ~ 5050 4750
Wire Wire Line
	5050 4750 5300 4750
$Comp
L Connector:Conn_01x02_Female LED1
U 1 1 60659CCB
P 7100 5200
F 0 "LED1" H 7128 5176 50  0000 L CNN
F 1 "AN CAT" H 7128 5085 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 7100 5200 50  0001 C CNN
F 3 "~" H 7100 5200 50  0001 C CNN
	1    7100 5200
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female LED2
U 1 1 6065A556
P 7600 5200
F 0 "LED2" H 7628 5176 50  0000 L CNN
F 1 "ANO CAT" H 7628 5085 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 7600 5200 50  0001 C CNN
F 3 "~" H 7600 5200 50  0001 C CNN
	1    7600 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 4800 6900 4800
Wire Wire Line
	7100 4800 7400 4800
Connection ~ 7100 4800
Wire Wire Line
	4350 4250 4450 4250
Wire Wire Line
	4450 4250 4450 4300
Wire Wire Line
	4450 4300 6500 4300
Wire Wire Line
	6500 4300 6500 4600
Connection ~ 4350 4250
$Comp
L Device:R R7
U 1 1 60675A04
P 6650 4600
F 0 "R7" V 6800 4600 50  0000 C CNN
F 1 "100" V 6534 4600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 6580 4600 50  0001 C CNN
F 3 "~" H 6650 4600 50  0001 C CNN
	1    6650 4600
	0    1    1    0   
$EndComp
$Comp
L Sensor_Optical:S5971 TSOP382
U 1 1 60653390
P 6100 6700
F 0 "TSOP382" V 6054 6878 50  0000 L CNN
F 1 "1 2 3" V 6145 6878 50  0000 L CNN
F 2 "TSOP38238:XDCR_TSOP38238" H 6100 7050 50  0001 C CNN
F 3 "https://www.hamamatsu.com/resources/pdf/ssd/s5971_etc_kpin1025e.pdf" H 6100 6700 50  0001 C CNN
	1    6100 6700
	0    1    1    0   
$EndComp
Wire Wire Line
	6050 6050 5250 6050
Wire Wire Line
	5250 6050 5250 6400
Wire Wire Line
	6100 6400 5250 6400
Connection ~ 5250 6400
Wire Wire Line
	5600 5850 5600 6700
Connection ~ 5600 5850
Wire Wire Line
	5600 5850 6050 5850
$Comp
L Device:R R10
U 1 1 6068EEF5
P 5350 7000
F 0 "R10" V 5557 7000 50  0000 C CNN
F 1 "200" V 5466 7000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5280 7000 50  0001 C CNN
F 3 "~" H 5350 7000 50  0001 C CNN
	1    5350 7000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5200 7000 5000 7000
Wire Wire Line
	5000 7000 5000 5950
Connection ~ 5000 5950
Wire Wire Line
	5000 5950 6050 5950
Text Label 4800 5950 0    50   ~ 0
3V3
Text Label 5200 4850 0    50   ~ 0
3V3
Wire Wire Line
	5600 6700 5700 6700
Wire Wire Line
	5500 7000 5700 7000
$Comp
L Device:CP C1
U 1 1 606B0062
P 5700 6850
F 0 "C1" H 5582 6804 50  0000 R CNN
F 1 "1u" H 5582 6895 50  0000 R CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-1608-08_AVX-J_Pad1.25x1.05mm_HandSolder" H 5738 6700 50  0001 C CNN
F 3 "~" H 5700 6850 50  0001 C CNN
	1    5700 6850
	-1   0    0    1   
$EndComp
Connection ~ 5700 7000
Wire Wire Line
	5700 7000 6100 7000
Connection ~ 5700 6700
Wire Wire Line
	5700 6700 5800 6700
Wire Wire Line
	5050 5450 6900 5450
Wire Wire Line
	6900 5100 6900 5200
Wire Wire Line
	6900 5300 6900 5450
Connection ~ 6900 5450
Wire Wire Line
	6900 5450 7400 5450
Wire Wire Line
	7400 5300 7400 5450
Wire Wire Line
	7400 5100 7400 5200
$EndSCHEMATC