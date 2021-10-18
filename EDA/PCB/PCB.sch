EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "MicroNova Controller"
Date "August 2021"
Rev "1.0"
Comp "philibertc"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Isolator:PC817 U4
U 1 1 610E5DA6
P 4750 4950
F 0 "U4" H 4750 5275 50  0001 C CNN
F 1 "PC817" H 4750 5183 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W7.62mm" H 4550 4750 50  0001 L CIN
F 3 "http://www.soselectronic.cz/a_info/resource/d/pc817.pdf" H 4750 4950 50  0001 L CNN
	1    4750 4950
	1    0    0    -1  
$EndComp
$Comp
L Isolator:PC817 U2
U 1 1 610E5EDD
P 4750 3950
F 0 "U2" H 4750 4275 50  0001 C CNN
F 1 "PC817" H 4750 4183 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W7.62mm" H 4550 3750 50  0001 L CIN
F 3 "http://www.soselectronic.cz/a_info/resource/d/pc817.pdf" H 4750 3950 50  0001 L CNN
	1    4750 3950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5050 5050 7900 5050
Wire Wire Line
	7900 5050 7900 4400
Wire Wire Line
	7550 4850 7550 4250
$Comp
L Device:R R3
U 1 1 610ECA4A
P 4050 5050
F 0 "R3" V 3843 5050 50  0001 C CNN
F 1 "500" V 3935 5050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3980 5050 50  0001 C CNN
F 3 "~" H 4050 5050 50  0001 C CNN
	1    4050 5050
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 610ECC5F
P 5300 4050
F 0 "R2" V 5093 4050 50  0001 C CNN
F 1 "500" V 5185 4050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5230 4050 50  0001 C CNN
F 3 "~" H 5300 4050 50  0001 C CNN
	1    5300 4050
	0    1    1    0   
$EndComp
Wire Wire Line
	5050 4050 5150 4050
Wire Wire Line
	4450 5050 4200 5050
Wire Wire Line
	5450 4050 7550 4050
Connection ~ 7550 4250
$Comp
L Isolator:PC817 U1
U 1 1 610F21D3
P 6050 3550
F 0 "U1" H 6050 3875 50  0001 C CNN
F 1 "PC817" H 6050 3783 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W7.62mm" H 5850 3350 50  0001 L CIN
F 3 "http://www.soselectronic.cz/a_info/resource/d/pc817.pdf" H 6050 3550 50  0001 L CNN
	1    6050 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 3850 6350 3850
Wire Wire Line
	6350 3850 6350 3650
$Comp
L Device:R R1
U 1 1 610F453C
P 5300 3650
F 0 "R1" V 5093 3650 50  0001 C CNN
F 1 "500" V 5185 3650 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5230 3650 50  0001 C CNN
F 3 "~" H 5300 3650 50  0001 C CNN
	1    5300 3650
	0    1    1    0   
$EndComp
Wire Wire Line
	5750 3650 5450 3650
Wire Wire Line
	5050 4850 7550 4850
$Comp
L MCU_Module:WeMos_D1_mini U3
U 1 1 61170FA0
P 2650 4300
F 0 "U3" H 2650 3411 50  0001 C CNN
F 1 "WeMos_D1_mini" H 2650 3411 50  0000 C CNN
F 2 "Module:WEMOS_D1_mini_light" H 2650 3150 50  0001 C CNN
F 3 "https://wiki.wemos.cc/products:d1:d1_mini#documentation" H 800 3150 50  0001 C CNN
	1    2650 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 3650 3200 3650
Wire Wire Line
	3200 3650 3200 4100
Wire Wire Line
	3200 4100 3050 4100
Wire Wire Line
	4450 3850 3300 3850
Wire Wire Line
	3300 3850 3300 4200
Wire Wire Line
	3300 4200 3050 4200
Wire Wire Line
	4450 4050 3550 4050
Wire Wire Line
	3550 4050 3550 5100
Wire Wire Line
	3550 5100 2650 5100
Wire Wire Line
	5750 3450 4150 3450
Wire Wire Line
	4450 4850 4150 4850
Wire Wire Line
	4150 4850 4150 3450
Connection ~ 4150 3450
Wire Wire Line
	4150 3450 2750 3450
Wire Wire Line
	3900 5050 3750 5050
Wire Wire Line
	3750 5050 3750 4300
Wire Wire Line
	3750 4300 3050 4300
Wire Wire Line
	7850 3150 2750 3150
Wire Wire Line
	2750 3150 2750 3450
Connection ~ 2750 3450
Wire Wire Line
	2750 3450 2750 3500
Wire Wire Line
	8300 2800 2000 2800
Wire Wire Line
	2000 2800 2000 5100
Wire Wire Line
	2000 5100 2650 5100
Connection ~ 2650 5100
Wire Wire Line
	7550 4050 7550 4250
Wire Wire Line
	7900 4400 8400 4400
Wire Wire Line
	7550 4250 8500 4250
Wire Wire Line
	2250 3350 2250 3900
$Comp
L Connector:Conn_01x06_Female J1
U 1 1 616EEB49
P 3300 5600
F 0 "J1" H 3328 5576 50  0001 L CNN
F 1 "Conn_01x06_Female" H 3328 5530 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 3328 5485 50  0001 L CNN
F 3 "~" H 3300 5600 50  0001 C CNN
	1    3300 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 4300 2250 5400
Wire Wire Line
	2250 5400 3100 5400
Wire Wire Line
	2250 4200 2200 4200
Wire Wire Line
	2200 4200 2200 5500
Wire Wire Line
	2200 5500 3100 5500
Wire Wire Line
	3050 4000 3150 4000
Wire Wire Line
	3150 4000 3150 5300
Wire Wire Line
	2250 3350 3100 3350
Wire Wire Line
	3100 3350 3100 3900
Wire Wire Line
	3100 3900 3050 3900
Wire Wire Line
	2950 5600 3100 5600
Wire Wire Line
	2950 5300 2950 5600
Wire Wire Line
	3150 5300 2950 5300
Wire Wire Line
	3050 4500 3200 4500
Wire Wire Line
	3200 4500 3200 5250
Wire Wire Line
	3200 5250 2900 5250
Wire Wire Line
	2900 5250 2900 5900
Wire Wire Line
	2900 5900 3100 5900
Wire Wire Line
	3050 4600 3250 4600
Wire Wire Line
	3250 4600 3250 5200
Wire Wire Line
	3250 5200 2850 5200
Wire Wire Line
	2850 5200 2850 5800
Wire Wire Line
	2850 5800 3100 5800
Wire Wire Line
	3050 4700 3300 4700
Wire Wire Line
	3300 4700 3300 5150
Wire Wire Line
	3300 5150 2800 5150
Wire Wire Line
	2800 5150 2800 5700
Wire Wire Line
	2800 5700 3100 5700
$Comp
L Connector:TestPoint_Flag TP1
U 1 1 617261BC
P 3650 5350
F 0 "TP1" H 3910 5444 50  0001 L CNN
F 1 "TestPoint_Flag" H 3910 5353 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 3850 5350 50  0001 C CNN
F 3 "~" H 3850 5350 50  0001 C CNN
	1    3650 5350
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Flag TP2
U 1 1 617267A4
P 4150 5350
F 0 "TP2" H 4106 5444 50  0001 R CNN
F 1 "TestPoint_Flag" H 4106 5353 50  0001 R CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 4350 5350 50  0001 C CNN
F 3 "~" H 4350 5350 50  0001 C CNN
	1    4150 5350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3050 4400 3500 4400
Wire Wire Line
	3500 4400 3500 5350
Wire Wire Line
	3500 5350 3650 5350
Wire Wire Line
	3550 5100 4150 5100
Wire Wire Line
	4150 5100 4150 5350
Connection ~ 3550 5100
$Comp
L Connector:Screw_Terminal_01x03 J3
U 1 1 61743262
P 9100 4650
F 0 "J3" H 9180 4692 50  0001 L CNN
F 1 "Screw_Terminal_01x03" H 9180 4601 50  0001 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-3_P5.08mm" H 9100 4650 50  0001 C CNN
F 3 "~" H 9100 4650 50  0001 C CNN
	1    9100 4650
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J2
U 1 1 6173CFF2
P 9100 3850
F 0 "J2" H 9180 3842 50  0001 L CNN
F 1 "Screw_Terminal_01x02" H 9180 3751 50  0001 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 9100 3850 50  0001 C CNN
F 3 "~" H 9100 3850 50  0001 C CNN
	1    9100 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 2800 8300 3850
Wire Wire Line
	8300 3850 8900 3850
Wire Wire Line
	7850 3950 8900 3950
Wire Wire Line
	8400 4400 8400 4700
Wire Wire Line
	7700 3450 7700 4100
Wire Wire Line
	6350 3450 7700 3450
Wire Wire Line
	7850 3150 7850 3950
Wire Wire Line
	8600 4550 8600 4700
Wire Wire Line
	8600 4700 8400 4700
Wire Wire Line
	8600 4550 8900 4550
Wire Wire Line
	8500 4650 8900 4650
Wire Wire Line
	8500 4250 8500 4650
Wire Wire Line
	7700 4100 8800 4100
Wire Wire Line
	8800 4100 8800 4750
Wire Wire Line
	8800 4750 8900 4750
$EndSCHEMATC
