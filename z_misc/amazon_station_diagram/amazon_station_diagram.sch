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
Text Label 3700 4150 0    50   ~ 0
GND
Text Label 3350 3250 0    50   ~ 0
SDA
Text Label 3350 2950 0    50   ~ 0
SCL
$Comp
L Device:R R1
U 1 1 604D89C5
P 2350 4600
F 0 "R1" V 2450 4650 50  0000 L CNN
F 1 "10K" V 2450 4450 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2280 4600 50  0001 C CNN
F 3 "~" H 2350 4600 50  0001 C CNN
	1    2350 4600
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 604DC4F8
P 2350 4800
F 0 "R2" V 2250 4850 50  0000 L CNN
F 1 "10K" V 2250 4650 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2280 4800 50  0001 C CNN
F 3 "~" H 2350 4800 50  0001 C CNN
	1    2350 4800
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push Trigger1
U 1 1 604FFFBF
P 1450 4800
F 0 "Trigger1" H 1550 4750 50  0000 R CNN
F 1 "SW_Push" H 1600 4650 50  0000 R CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 1450 5000 50  0001 C CNN
F 3 "~" H 1450 5000 50  0001 C CNN
	1    1450 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 4600 1700 4600
Connection ~ 1700 4600
Wire Wire Line
	2500 4800 3300 4800
Wire Wire Line
	1150 5050 1150 4800
Wire Wire Line
	1250 4800 1150 4800
Connection ~ 1150 4800
Wire Wire Line
	1150 4800 1150 4600
$Comp
L Device:R R4
U 1 1 605E82CC
P 3750 5500
F 0 "R4" V 3957 5500 50  0000 C CNN
F 1 "100K" V 3866 5500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 3680 5500 50  0001 C CNN
F 3 "~" H 3750 5500 50  0001 C CNN
	1    3750 5500
	0    -1   -1   0   
$EndComp
Connection ~ 3300 4800
$Comp
L Device:R R3
U 1 1 6060242F
P 2600 5200
F 0 "R3" V 2700 5200 50  0000 C CNN
F 1 "10K" V 2500 5200 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2530 5200 50  0001 C CNN
F 3 "~" H 2600 5200 50  0001 C CNN
	1    2600 5200
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_BJT:2N2219 Q1
U 1 1 605D56C8
P 3100 5300
F 0 "Q1" V 3300 5300 50  0000 C CNN
F 1 "2N2222" V 3100 5000 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23_Handsoldering" H 3300 5225 50  0001 L CIN
F 3 "http://www.onsemi.com/pub_link/Collateral/2N2219-D.PDF" H 3100 5300 50  0001 L CNN
	1    3100 5300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3300 4800 3300 5200
Wire Wire Line
	1150 5050 2300 5050
Wire Wire Line
	2300 5050 2300 5200
Connection ~ 2300 5050
Wire Wire Line
	2450 5200 2300 5200
Wire Wire Line
	2750 5200 2800 5200
Wire Wire Line
	2800 5200 2800 5350
Connection ~ 2800 5200
Wire Wire Line
	2800 5200 2900 5200
Wire Wire Line
	3600 5500 3100 5500
Wire Wire Line
	1700 3750 2050 3750
Text Label 2300 5050 0    50   ~ 0
3V3
$Comp
L weather_station_parts:ESP32-WROOM U1
U 1 1 60955915
P 2650 3600
F 0 "U1" H 2680 4615 50  0000 C CNN
F 1 "ESP32-WROOM" H 2680 4524 50  0000 C CNN
F 2 "" H 2650 3600 50  0001 C CNN
F 3 "" H 2650 3600 50  0001 C CNN
	1    2650 3600
	1    0    0    -1  
$EndComp
$Comp
L weather_station_parts:OLED_SSD1306 U3
U 1 1 60962F53
P 4850 3800
F 0 "U3" V 4896 3372 50  0000 R CNN
F 1 "OLED_SSD1306" V 4805 3372 50  0000 R CNN
F 2 "" H 4850 3800 50  0001 C CNN
F 3 "" H 4850 3800 50  0001 C CNN
	1    4850 3800
	0    -1   -1   0   
$EndComp
$Comp
L weather_station_parts:IR_Recvr_Amazon U4
U 1 1 6096B115
P 4650 4750
F 0 "U4" V 4604 5078 50  0000 L CNN
F 1 "IR_Recvr_HX_M21" V 4695 5078 50  0000 L CNN
F 2 "" H 4650 4750 50  0001 C CNN
F 3 "" H 4650 4750 50  0001 C CNN
	1    4650 4750
	0    1    1    0   
$EndComp
Wire Wire Line
	1700 3750 1700 4600
Wire Wire Line
	2050 4050 2000 4050
Wire Wire Line
	2800 5350 1850 5350
Wire Wire Line
	1850 3850 2050 3850
Wire Wire Line
	1850 3850 1850 5350
Wire Wire Line
	1650 4800 2000 4800
Wire Wire Line
	2000 4800 2200 4800
Connection ~ 2000 4800
Wire Wire Line
	2000 4050 2000 4800
$Comp
L weather_station_parts:BMP180 U2
U 1 1 609603E7
P 4700 2900
F 0 "U2" V 4741 2572 50  0000 R CNN
F 1 "BME280" V 4650 2572 50  0000 R CNN
F 2 "" H 4700 2900 50  0001 C CNN
F 3 "" H 4700 2900 50  0001 C CNN
	1    4700 2900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4500 2850 4300 2850
Wire Wire Line
	4300 2850 4300 3850
Wire Wire Line
	4000 2950 4000 3750
Wire Wire Line
	4000 3750 4500 3750
Connection ~ 4000 2950
Wire Wire Line
	4000 2950 4500 2950
Wire Wire Line
	4500 3850 4300 3850
Connection ~ 4300 3850
Wire Wire Line
	4300 3850 4300 4150
Wire Wire Line
	4500 3950 4450 3950
Wire Wire Line
	4450 3950 4450 4250
Wire Wire Line
	1650 4600 1700 4600
Wire Wire Line
	1250 4600 1150 4600
$Comp
L Switch:SW_Push Mode1
U 1 1 6063A1F9
P 1450 4600
F 0 "Mode1" H 1500 4750 50  0000 R CNN
F 1 "SW_Push" H 1600 4850 50  0000 R CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 1450 4800 50  0001 C CNN
F 3 "~" H 1450 4800 50  0001 C CNN
	1    1450 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 4850 4050 4850
Wire Wire Line
	4050 4850 4050 5500
Wire Wire Line
	4050 5500 3900 5500
Wire Wire Line
	3300 4250 3400 4250
Wire Wire Line
	3300 3250 3400 3250
Connection ~ 3400 3250
Wire Wire Line
	3400 4050 3400 4250
Connection ~ 3400 4250
Wire Wire Line
	3400 3250 3400 3750
$Comp
L Device:R R5
U 1 1 605D37E2
P 3400 3900
F 0 "R5" V 3500 3750 50  0000 L CNN
F 1 "2.4K" V 3500 3900 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 3330 3900 50  0001 C CNN
F 3 "~" H 3400 3900 50  0001 C CNN
	1    3400 3900
	1    0    0    -1  
$EndComp
Text Label 3700 4250 0    50   ~ 0
3V3
Wire Wire Line
	3300 4800 3500 4800
Wire Wire Line
	3500 4800 3500 4600
Wire Wire Line
	2500 4600 3500 4600
Connection ~ 3500 4150
Wire Wire Line
	3500 4150 3300 4150
Connection ~ 3500 4600
Wire Wire Line
	3500 4150 3500 4600
Wire Wire Line
	3400 4250 3600 4250
Connection ~ 3600 4250
Wire Wire Line
	3600 2950 4000 2950
Connection ~ 3600 2950
Wire Wire Line
	3600 2950 3300 2950
Wire Wire Line
	4450 4250 4150 4250
Connection ~ 4150 4250
Wire Wire Line
	4500 2750 4150 2750
Wire Wire Line
	4150 2750 4150 4250
Wire Wire Line
	4050 4650 4200 4650
Wire Wire Line
	3500 4150 4050 4150
Wire Wire Line
	4050 4150 4300 4150
Connection ~ 4050 4150
Wire Wire Line
	4050 4150 4050 4650
Wire Wire Line
	3900 4750 4200 4750
Wire Wire Line
	3900 4250 4150 4250
Connection ~ 3900 4250
Wire Wire Line
	3900 4250 3900 4750
Wire Wire Line
	2300 5050 3750 5050
Wire Wire Line
	3600 4250 3750 4250
Wire Wire Line
	3750 4250 3900 4250
Connection ~ 3750 4250
Wire Wire Line
	3750 4250 3750 5050
Wire Wire Line
	3850 3650 4500 3650
Wire Wire Line
	4500 3050 3850 3050
Wire Wire Line
	3850 3050 3850 3250
Wire Wire Line
	3600 3600 3600 4250
Wire Wire Line
	3600 2950 3600 3300
$Comp
L Device:R R6
U 1 1 605D9489
P 3600 3450
F 0 "R6" V 3700 3300 50  0000 L CNN
F 1 "2.4K" V 3700 3450 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 3530 3450 50  0001 C CNN
F 3 "~" H 3600 3450 50  0001 C CNN
	1    3600 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 3250 3850 3250
Connection ~ 3850 3250
Wire Wire Line
	3850 3250 3850 3650
$Comp
L weather_station_parts:ESP32-WROOM U5
U 1 1 606D1C07
P 7600 4050
F 0 "U5" H 7630 5065 50  0000 C CNN
F 1 "ESP32-WROOM" H 7630 4974 50  0000 C CNN
F 2 "" H 7600 4050 50  0001 C CNN
F 3 "" H 7600 4050 50  0001 C CNN
	1    7600 4050
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push Trig1
U 1 1 606DD79A
P 9350 3750
F 0 "Trig1" H 9400 3900 50  0000 R CNN
F 1 "SW_Push" H 9500 4000 50  0000 R CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 9350 3950 50  0001 C CNN
F 3 "~" H 9350 3950 50  0001 C CNN
	1    9350 3750
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push Upload1
U 1 1 606DF583
P 9350 4800
F 0 "Upload1" H 9500 4950 50  0000 R CNN
F 1 "SW_Push" H 9500 5050 50  0000 R CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 9350 5000 50  0001 C CNN
F 3 "~" H 9350 5000 50  0001 C CNN
	1    9350 4800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 606E2605
P 9750 3750
F 0 "R7" V 9850 3800 50  0000 L CNN
F 1 "10K" V 9850 3600 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 9680 3750 50  0001 C CNN
F 3 "~" H 9750 3750 50  0001 C CNN
	1    9750 3750
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R9
U 1 1 606E2C68
P 9750 4800
F 0 "R9" V 9850 4850 50  0000 L CNN
F 1 "10K" V 9850 4650 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 9680 4800 50  0001 C CNN
F 3 "~" H 9750 4800 50  0001 C CNN
	1    9750 4800
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_BJT:2N2219 Q2
U 1 1 606EEAF9
P 8750 4100
F 0 "Q2" H 9000 4250 50  0000 C CNN
F 1 "2N2222" H 9100 4100 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23_Handsoldering" H 8950 4025 50  0001 L CIN
F 3 "http://www.onsemi.com/pub_link/Collateral/2N2219-D.PDF" H 8750 4100 50  0001 L CNN
	1    8750 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 606F5E56
P 9350 4300
F 0 "D1" H 9343 4045 50  0000 C CNN
F 1 "LED" H 9343 4136 50  0000 C CNN
F 2 "" H 9350 4300 50  0001 C CNN
F 3 "~" H 9350 4300 50  0001 C CNN
	1    9350 4300
	-1   0    0    1   
$EndComp
$Comp
L Device:R R8
U 1 1 606F6B79
P 9750 4300
F 0 "R8" V 9850 4350 50  0000 L CNN
F 1 "100" V 9850 4150 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 9680 4300 50  0001 C CNN
F 3 "~" H 9750 4300 50  0001 C CNN
	1    9750 4300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9150 3750 8850 3750
Wire Wire Line
	8250 4700 8450 4700
Wire Wire Line
	9550 3750 9550 3300
Wire Wire Line
	9550 3300 8250 3300
Connection ~ 9550 3750
Wire Wire Line
	9550 3750 9600 3750
Wire Wire Line
	9500 4300 9600 4300
Wire Wire Line
	8850 4300 9200 4300
Wire Wire Line
	8450 4800 9150 4800
Wire Wire Line
	9550 4800 9600 4800
Wire Wire Line
	9900 4300 9950 4300
Wire Wire Line
	9950 4300 9950 3750
Wire Wire Line
	9950 3750 9900 3750
Connection ~ 9950 4300
Wire Wire Line
	8850 3900 8850 3750
Connection ~ 8850 3750
Wire Wire Line
	8850 3750 8450 3750
Wire Wire Line
	9950 4300 9950 4800
Wire Wire Line
	9900 4800 9950 4800
Connection ~ 9950 4800
Wire Wire Line
	9950 4800 9950 5050
Wire Wire Line
	8250 4500 9600 4500
Wire Wire Line
	9600 4500 9600 4800
Connection ~ 9600 4800
Wire Wire Line
	8250 4600 8350 4600
Wire Wire Line
	8350 4600 8350 5050
Wire Wire Line
	8350 5050 9950 5050
Text Label 9950 4200 0    50   ~ 0
GND
Wire Wire Line
	8250 4100 8550 4100
Wire Wire Line
	8450 3750 8450 4700
Connection ~ 8450 4700
Wire Wire Line
	8450 4700 8450 4800
Text Label 8500 3750 0    50   ~ 0
3V3
Wire Notes Line
	10300 2900 6750 2900
Wire Notes Line
	10300 5200 10300 2900
Wire Notes Line
	6750 5200 10300 5200
Wire Notes Line
	6750 2900 6750 5200
Wire Notes Line
	1050 5600 5900 5600
Wire Notes Line
	5900 5600 5900 2500
Wire Notes Line
	5900 2500 1050 2500
Wire Notes Line
	1050 2500 1050 5600
Text Notes 2750 2400 0    157  ~ 0
Data Logger
Text Notes 7900 2800 0    157  ~ 0
Data Receiver
$EndSCHEMATC