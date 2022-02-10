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
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J3
U 1 1 600C7FCC
P 9550 2450
F 0 "J3" H 9600 2867 50  0000 C CNN
F 1 "Conn_02x05_Odd_Even" H 9600 2776 50  0000 C CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical" H 9550 2450 50  0001 C CNN
F 3 "~" H 9550 2450 50  0001 C CNN
	1    9550 2450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 600C9412
P 9050 2650
F 0 "#PWR0101" H 9050 2400 50  0001 C CNN
F 1 "GND" H 9055 2477 50  0000 C CNN
F 2 "" H 9050 2650 50  0001 C CNN
F 3 "" H 9050 2650 50  0001 C CNN
	1    9050 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	9350 2350 9250 2350
Wire Wire Line
	9250 2350 9250 2450
Wire Wire Line
	9350 2450 9250 2450
Connection ~ 9250 2450
Wire Wire Line
	9250 2450 9250 2650
Wire Wire Line
	9350 2650 9250 2650
Connection ~ 9250 2650
NoConn ~ 9350 2550
Wire Wire Line
	9950 3750 9950 2650
Wire Wire Line
	9950 2650 9850 2650
Wire Wire Line
	7100 3450 10050 3450
Wire Wire Line
	10050 3450 10050 2550
Wire Wire Line
	10050 2550 9850 2550
Wire Wire Line
	7100 3350 10150 3350
Wire Wire Line
	10150 3350 10150 2450
Wire Wire Line
	10150 2450 9850 2450
Wire Wire Line
	7100 3250 10250 3250
Wire Wire Line
	10250 3250 10250 2350
Wire Wire Line
	10250 2350 9850 2350
Wire Wire Line
	7100 3150 10350 3150
Wire Wire Line
	10350 3150 10350 2250
Wire Wire Line
	10350 2250 9850 2250
Text Label 8150 3750 0    50   ~ 0
XDS_RESET
Text Label 8150 3350 0    50   ~ 0
XDS_TDO
Text Label 8150 3250 0    50   ~ 0
XDS_TCK
Text Label 8150 3150 0    50   ~ 0
XDS-TMS
$Comp
L Device:R R4
U 1 1 600D6091
P 6100 4250
F 0 "R4" V 5893 4250 50  0000 C CNN
F 1 "100" V 5984 4250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6030 4250 50  0001 C CNN
F 3 "~" H 6100 4250 50  0001 C CNN
	1    6100 4250
	0    1    1    0   
$EndComp
Wire Wire Line
	5950 4250 5700 4250
Wire Wire Line
	5700 4250 5700 4100
$Comp
L power:GND #PWR0102
U 1 1 600DC372
P 7150 4350
F 0 "#PWR0102" H 7150 4100 50  0001 C CNN
F 1 "GND" H 7155 4177 50  0000 C CNN
F 2 "" H 7150 4350 50  0001 C CNN
F 3 "" H 7150 4350 50  0001 C CNN
	1    7150 4350
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0104
U 1 1 600E6EC6
P 9050 2250
F 0 "#PWR0104" H 9050 2100 50  0001 C CNN
F 1 "VCC" V 9065 2377 50  0000 L CNN
F 2 "" H 9050 2250 50  0001 C CNN
F 3 "" H 9050 2250 50  0001 C CNN
	1    9050 2250
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 600F46EE
P 5450 3650
F 0 "#PWR0106" H 5450 3400 50  0001 C CNN
F 1 "GND" H 5455 3477 50  0000 C CNN
F 2 "" H 5450 3650 50  0001 C CNN
F 3 "" H 5450 3650 50  0001 C CNN
	1    5450 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 3650 5450 3650
Wire Wire Line
	5450 3250 5450 3650
Connection ~ 5450 3650
$Comp
L Device:R R2
U 1 1 601039A2
P 3850 4450
F 0 "R2" V 3643 4450 50  0000 C CNN
F 1 "100" V 3734 4450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3780 4450 50  0001 C CNN
F 3 "~" H 3850 4450 50  0001 C CNN
	1    3850 4450
	0    1    1    0   
$EndComp
Wire Wire Line
	4000 4450 4150 4450
Wire Wire Line
	3700 4450 3450 4450
$Comp
L power:GND #PWR0107
U 1 1 601039AB
P 4900 4550
F 0 "#PWR0107" H 4900 4300 50  0001 C CNN
F 1 "GND" H 4905 4377 50  0000 C CNN
F 2 "" H 4900 4550 50  0001 C CNN
F 3 "" H 4900 4550 50  0001 C CNN
	1    4900 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 4450 4900 4450
Wire Wire Line
	4900 4450 4900 4550
$Comp
L power:VDD #PWR0109
U 1 1 601062A9
P 4200 3150
F 0 "#PWR0109" H 4200 3000 50  0001 C CNN
F 1 "VDD" H 4215 3323 50  0000 C CNN
F 2 "" H 4200 3150 50  0001 C CNN
F 3 "" H 4200 3150 50  0001 C CNN
	1    4200 3150
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0110
U 1 1 60106981
P 3300 1050
F 0 "#PWR0110" H 3300 900 50  0001 C CNN
F 1 "VDD" H 3315 1223 50  0000 C CNN
F 2 "" H 3300 1050 50  0001 C CNN
F 3 "" H 3300 1050 50  0001 C CNN
	1    3300 1050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 60108142
P 2900 1250
F 0 "J1" H 2818 825 50  0000 C CNN
F 1 "Conn_01x04" H 2818 916 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 2900 1250 50  0001 C CNN
F 3 "~" H 2900 1250 50  0001 C CNN
	1    2900 1250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 60109C62
P 3350 1350
F 0 "#PWR0111" H 3350 1100 50  0001 C CNN
F 1 "GND" H 3355 1177 50  0000 C CNN
F 2 "" H 3350 1350 50  0001 C CNN
F 3 "" H 3350 1350 50  0001 C CNN
	1    3350 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 1350 3350 1350
Wire Wire Line
	3100 1050 3300 1050
Wire Wire Line
	5250 1700 5250 1150
Wire Wire Line
	5250 1150 3100 1150
Wire Wire Line
	5250 1700 5750 1700
Wire Wire Line
	5750 1800 5150 1800
Wire Wire Line
	5150 1800 5150 1250
Wire Wire Line
	5150 1250 3100 1250
Text Label 4300 1150 0    50   ~ 0
UART_RXD
Text Label 4300 1250 0    50   ~ 0
UART_TXD
Text Label 4800 2500 0    50   ~ 0
SPI_SCLK
Text Label 7550 1700 0    50   ~ 0
FLASH_CS
Text Label 4800 2400 0    50   ~ 0
SPI_MOSI
Wire Wire Line
	4800 2400 5750 2400
Wire Wire Line
	7100 1700 7550 1700
Text Label 4800 2300 0    50   ~ 0
SPI_MISO
Wire Wire Line
	4800 2300 5750 2300
$Comp
L power:GND #PWR0112
U 1 1 601746F8
P 10200 5850
F 0 "#PWR0112" H 10200 5600 50  0001 C CNN
F 1 "GND" H 10205 5677 50  0000 C CNN
F 2 "" H 10200 5850 50  0001 C CNN
F 3 "" H 10200 5850 50  0001 C CNN
	1    10200 5850
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0113
U 1 1 60174E0D
P 8350 4250
F 0 "#PWR0113" H 8350 4100 50  0001 C CNN
F 1 "VDD" H 8365 4423 50  0000 C CNN
F 2 "" H 8350 4250 50  0001 C CNN
F 3 "" H 8350 4250 50  0001 C CNN
	1    8350 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 6018F286
P 7950 4800
F 0 "R3" H 8020 4846 50  0000 L CNN
F 1 "2.2k" H 8020 4755 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 7880 4800 50  0001 C CNN
F 3 "~" H 7950 4800 50  0001 C CNN
	1    7950 4800
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0114
U 1 1 60190828
P 7950 4650
F 0 "#PWR0114" H 7950 4500 50  0001 C CNN
F 1 "VDD" H 7965 4823 50  0000 C CNN
F 2 "" H 7950 4650 50  0001 C CNN
F 3 "" H 7950 4650 50  0001 C CNN
	1    7950 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 4950 8700 4950
Text Label 7350 5150 0    50   ~ 0
SPI_SCLK
Text Label 7350 4950 0    50   ~ 0
FLASH_CS
Wire Wire Line
	7950 4950 7350 4950
Connection ~ 7950 4950
Wire Wire Line
	7350 5150 8700 5150
NoConn ~ 10100 5350
NoConn ~ 10100 5150
Text Label 10350 4950 0    50   ~ 0
SPI_MISO
Text Label 10350 4750 0    50   ~ 0
SPI_MOSI
Wire Wire Line
	10100 4750 10350 4750
Wire Wire Line
	10350 4950 10100 4950
Text Label 4800 1900 0    50   ~ 0
SCL
Text Label 4800 2000 0    50   ~ 0
SDA
$Comp
L Device:LED D3
U 1 1 602211B9
P 9700 1800
F 0 "D3" H 9693 1545 50  0000 C CNN
F 1 "LED" H 9693 1636 50  0000 C CNN
F 2 "daughter_board:Lite-On-LTST-S220KRKT-0-0-0" H 9700 1800 50  0001 C CNN
F 3 "~" H 9700 1800 50  0001 C CNN
	1    9700 1800
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D2
U 1 1 60223882
P 9700 1350
F 0 "D2" H 9693 1095 50  0000 C CNN
F 1 "LED" H 9693 1186 50  0000 C CNN
F 2 "daughter_board:Lite-On-LTST-S220KRKT-0-0-0" H 9700 1350 50  0001 C CNN
F 3 "~" H 9700 1350 50  0001 C CNN
	1    9700 1350
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D1
U 1 1 60224EF9
P 9700 1000
F 0 "D1" H 9693 745 50  0000 C CNN
F 1 "LED" H 9693 836 50  0000 C CNN
F 2 "daughter_board:Lite-On-LTST-S220KRKT-0-0-0" H 9700 1000 50  0001 C CNN
F 3 "~" H 9700 1000 50  0001 C CNN
	1    9700 1000
	-1   0    0    1   
$EndComp
$Comp
L Device:R R7
U 1 1 60225A05
P 9300 1800
F 0 "R7" V 9093 1800 50  0000 C CNN
F 1 "150" V 9184 1800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9230 1800 50  0001 C CNN
F 3 "~" H 9300 1800 50  0001 C CNN
	1    9300 1800
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 6022632E
P 9300 1350
F 0 "R6" V 9093 1350 50  0000 C CNN
F 1 "150" V 9184 1350 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9230 1350 50  0001 C CNN
F 3 "~" H 9300 1350 50  0001 C CNN
	1    9300 1350
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 60226ECC
P 9300 1000
F 0 "R5" V 9093 1000 50  0000 C CNN
F 1 "150" V 9184 1000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9230 1000 50  0001 C CNN
F 3 "~" H 9300 1000 50  0001 C CNN
	1    9300 1000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 6022752B
P 10100 1800
F 0 "#PWR0121" H 10100 1550 50  0001 C CNN
F 1 "GND" H 10105 1627 50  0000 C CNN
F 2 "" H 10100 1800 50  0001 C CNN
F 3 "" H 10100 1800 50  0001 C CNN
	1    10100 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 1000 9550 1000
Wire Wire Line
	8900 1100 8900 1350
Wire Wire Line
	8900 1350 9150 1350
Wire Wire Line
	9450 1350 9550 1350
Wire Wire Line
	9450 1800 9550 1800
Wire Wire Line
	9050 1800 9150 1800
Wire Wire Line
	9050 1800 9050 2250
Wire Wire Line
	9050 2250 9350 2250
Wire Wire Line
	9850 1800 10100 1800
Wire Wire Line
	10100 1800 10100 1350
Wire Wire Line
	10100 1000 9850 1000
Connection ~ 10100 1800
Wire Wire Line
	9850 1350 10100 1350
Connection ~ 10100 1350
Wire Wire Line
	10100 1350 10100 1000
Text Label 8000 1100 0    50   ~ 0
BLED
Text Label 8000 1000 0    50   ~ 0
GLED
Wire Wire Line
	5700 3350 5750 3350
Wire Wire Line
	5450 3250 5750 3250
Text Label 4800 2100 0    50   ~ 0
BLED
Text Label 4800 2200 0    50   ~ 0
GLED
Wire Wire Line
	4800 2100 5750 2100
Wire Wire Line
	5750 2200 4800 2200
Wire Wire Line
	3450 2800 3450 4450
NoConn ~ 5750 2700
NoConn ~ 5750 3000
NoConn ~ 7100 2700
NoConn ~ 7100 2600
NoConn ~ 7100 2500
NoConn ~ 7100 1600
NoConn ~ 7100 1500
NoConn ~ 5750 1500
NoConn ~ 5750 1600
Connection ~ 3300 1050
Wire Wire Line
	3300 1050 3350 1050
Wire Wire Line
	9050 2650 9250 2650
Connection ~ 9050 2250
$Comp
L cc2652_module_rf_star:BAT-HLD-001 BT1
U 1 1 603C80C5
P 1650 6450
F 0 "BT1" H 1862 6184 50  0000 L CNN
F 1 "BAT-HLD-001" H 1862 6093 50  0000 L CNN
F 2 "daughter_board:Linx-BAT-HLD-001-0" H 1650 6650 50  0001 L CNN
F 3 "https://www.linxtechnologies.com/resources/diagrams/bat-hld-001.pdf" H 1650 6750 50  0001 L CNN
F 4 "Batt" H 1650 6850 50  0001 L CNN "category"
F 5 "HOLDER BATTERY 20MM COIN" H 1650 6950 50  0001 L CNN "digikey description"
F 6 "BAT-HLD-001-ND" H 1650 7050 50  0001 L CNN "digikey part number"
F 7 "yes" H 1650 7150 50  0001 L CNN "lead free"
F 8 "46b8dec5928a9d45" H 1650 7250 50  0001 L CNN "library id"
F 9 "Linx" H 1650 7350 50  0001 L CNN "manufacturer"
F 10 "Surface Mount" H 1650 7450 50  0001 L CNN "mounting type"
F 11 "712-BAT-HLD-001" H 1650 7550 50  0001 L CNN "mouser part number"
F 12 "BAT_SMT_21MM1_15MM5" H 1650 7650 50  0001 L CNN "package"
F 13 "yes" H 1650 7750 50  0001 L CNN "rohs"
	1    1650 6450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 603DD846
P 1650 7100
F 0 "#PWR01" H 1650 6850 50  0001 C CNN
F 1 "GND" H 1655 6927 50  0000 C CNN
F 2 "" H 1650 7100 50  0001 C CNN
F 3 "" H 1650 7100 50  0001 C CNN
	1    1650 7100
	1    0    0    -1  
$EndComp
Connection ~ 5700 3750
Wire Wire Line
	5700 3750 9950 3750
Wire Wire Line
	5700 3350 5700 3750
$Comp
L pspice:CAP C2
U 1 1 600E91B3
P 4950 3400
F 0 "C2" H 5128 3446 50  0000 L CNN
F 1 "100n" H 5128 3355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4950 3400 50  0001 C CNN
F 3 "~" H 4950 3400 50  0001 C CNN
	1    4950 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 3150 5750 3150
$Comp
L cc2652_module_rf_star:BLM18HE152SN1D FB1
U 1 1 60629957
P 4400 3150
F 0 "FB1" H 4650 3395 50  0000 C CNN
F 1 "BLM18HE152SN1D" H 4650 3304 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 4400 3550 50  0001 L CNN
F 3 "https://www.murata.com/en-us/products/productdetail?partno=BLM18HE152SN1%23" H 4400 3650 50  0001 L CNN
F 4 "500mA" H 4400 3750 50  0001 L CNN "RMS current"
F 5 "No" H 4400 3850 50  0001 L CNN "automotive"
F 6 "FBead" H 4400 4050 50  0001 L CNN "category"
F 7 "Passive Components" H 4400 4150 50  0001 L CNN "device class L1"
F 8 "EMI / RFI Components" H 4400 4250 50  0001 L CNN "device class L2"
F 9 "Ferrite Beads and Chips" H 4400 4350 50  0001 L CNN "device class L3"
F 10 "0.95mm" H 4400 4450 50  0001 L CNN "height"
F 11 "1500Ω @ 100MHz" H 4400 4550 50  0001 L CNN "impedance at frequency"
F 12 "CAPC16080X80" H 4400 4650 50  0001 L CNN "ipc land pattern name"
F 13 "Yes" H 4400 4750 50  0001 L CNN "lead free"
F 14 "79fe80c99abd3e2a" H 4400 4850 50  0001 L CNN "library id"
F 15 "Murata" H 4400 4950 50  0001 L CNN "manufacturer"
F 16 "0603" H 4400 5050 50  0001 L CNN "package"
F 17 "Yes" H 4400 5150 50  0001 L CNN "rohs"
F 18 "0.5Ω" H 4400 5250 50  0001 L CNN "series resistance"
F 19 "+125℃" H 4400 5350 50  0001 L CNN "temperature range high"
F 20 "-55°C" H 4400 5450 50  0001 L CNN "temperature range low"
F 21 "100MHz" H 4400 5550 50  0001 L CNN "test frequency"
F 22 "25%" H 4400 5650 50  0001 L CNN "tolerance"
	1    4400 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 3150 4500 3150
Wire Wire Line
	4800 3150 4950 3150
Connection ~ 4950 3150
$Comp
L Connector_Generic:Conn_01x04 AHT10
U 1 1 606EDA88
P 2400 5300
F 0 "AHT10" H 2250 5625 59  0000 L BNN
F 1 "Conn_01x04" H 2250 5000 59  0001 L BNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 2400 5300 50  0001 C CNN
F 3 "" H 2400 5300 50  0001 C CNN
	1    2400 5300
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0116
U 1 1 606EDA8E
P 1450 5200
F 0 "#PWR0116" H 1450 5050 50  0001 C CNN
F 1 "VDD" H 1465 5373 50  0000 C CNN
F 2 "" H 1450 5200 50  0001 C CNN
F 3 "" H 1450 5200 50  0001 C CNN
	1    1450 5200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 606EDA94
P 1450 5300
F 0 "#PWR0117" H 1450 5050 50  0001 C CNN
F 1 "GND" H 1455 5127 50  0000 C CNN
F 2 "" H 1450 5300 50  0001 C CNN
F 3 "" H 1450 5300 50  0001 C CNN
	1    1450 5300
	1    0    0    -1  
$EndComp
Text Label 1650 5500 0    50   ~ 0
SDA
Text Label 1650 5400 0    50   ~ 0
SCL
Wire Wire Line
	1450 5300 2200 5300
Wire Wire Line
	2200 5200 1450 5200
Wire Wire Line
	1650 5500 2200 5500
Wire Wire Line
	1650 5400 2200 5400
Text Notes 1150 4900 0    50   ~ 0
AHT10  Temperature and Humidity Sensor
$Comp
L cc2652_module_rf_star:MX25R8035FM1IH1 U1
U 1 1 60790A43
P 8600 4750
F 0 "U1" H 9400 5065 50  0000 C CNN
F 1 "MX25R8035FM1IH1" H 9400 4974 50  0000 C CNN
F 2 "Package_SO:SOP-8_3.9x4.9mm_P1.27mm" H 8600 5150 50  0001 L CNN
F 3 "http://www.macronix.com/Lists/Datasheet/Attachments/7461/MX25R8035F,%20Wide%20Range,%208Mb,%20v1.6.pdf" H 8600 5250 50  0001 L CNN
F 4 "12ns" H 8600 5350 50  0001 L CNN "access time"
F 5 "+85°C" H 8600 5450 50  0001 L CNN "ambient temperature range high"
F 6 "-40°C" H 8600 5550 50  0001 L CNN "ambient temperature range low"
F 7 "No" H 8600 5650 50  0001 L CNN "automotive"
F 8 "IC" H 8600 5750 50  0001 L CNN "category"
F 9 "Integrated Circuits (ICs)" H 8600 5850 50  0001 L CNN "device class L1"
F 10 "Memory" H 8600 5950 50  0001 L CNN "device class L2"
F 11 "Flash" H 8600 6050 50  0001 L CNN "device class L3"
F 12 "IC FLASH 8M SPI 33MHZ 8USON" H 8600 6150 50  0001 L CNN "digikey description"
F 13 "1092-1207-1-ND" H 8600 6250 50  0001 L CNN "digikey part number"
F 14 "http://www.macronix.com/Lists/ApplicationNote/Attachments/1997/AN0159V4-%20Recommended%20PCB%20Pad%20Layouts%20for%208-USON%20and%208-WSON%20Packages_2.pdf" H 8600 6350 50  0001 L CNN "footprint url"
F 15 "33MHz" H 8600 6450 50  0001 L CNN "frequency"
F 16 "0.6mm" H 8600 6550 50  0001 L CNN "height"
F 17 "SPI" H 8600 6650 50  0001 L CNN "interface"
F 18 "DFN200X300X55-8" H 8600 6750 50  0001 L CNN "ipc land pattern name"
F 19 "Yes" H 8600 6850 50  0001 L CNN "lead free"
F 20 "845247ac2f357ca3" H 8600 6950 50  0001 L CNN "library id"
F 21 "Macronix International Co., Ltd." H 8600 7050 50  0001 L CNN "manufacturer"
F 22 "3.6V" H 8600 7150 50  0001 L CNN "max supply voltage"
F 23 "8Mbits" H 8600 7250 50  0001 L CNN "memory size"
F 24 "FLASH,NOR" H 8600 7350 50  0001 L CNN "memory type"
F 25 "1.65V" H 8600 7450 50  0001 L CNN "min supply voltage"
F 26 "Surface Mount" H 8600 7550 50  0001 L CNN "mounting type"
F 27 "4.5-6mA" H 8600 7650 50  0001 L CNN "nominal supply current"
F 28 "USON8" H 8600 7750 50  0001 L CNN "package"
F 29 "Yes" H 8600 7850 50  0001 L CNN "rohs"
F 30 "0mm" H 8600 7950 50  0001 L CNN "standoff height"
F 31 "+85°C" H 8600 8050 50  0001 L CNN "temperature range high"
F 32 "-40°C" H 8600 8150 50  0001 L CNN "temperature range low"
	1    8600 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 5550 10100 5550
Wire Wire Line
	10200 5550 10200 5850
Wire Wire Line
	4800 2500 5750 2500
Text Label 8150 3450 0    50   ~ 0
XDS_TDI
Wire Wire Line
	4350 4250 4150 4250
Wire Wire Line
	4150 4250 4150 4450
Connection ~ 4150 4450
Wire Wire Line
	4150 4450 4350 4450
Wire Wire Line
	4750 4250 4900 4250
Wire Wire Line
	4900 4250 4900 4450
Connection ~ 4900 4450
Wire Wire Line
	6250 4250 6450 4250
Wire Wire Line
	6450 4050 6250 4050
Wire Wire Line
	6250 4050 6250 4250
Connection ~ 6250 4250
Wire Wire Line
	6850 4250 7150 4250
Wire Wire Line
	7150 4250 7150 4350
Wire Wire Line
	6850 4050 7150 4050
Wire Wire Line
	7150 4050 7150 4250
Connection ~ 7150 4250
NoConn ~ 5750 2900
$Comp
L Device:R R1
U 1 1 60285FB1
P 5550 4100
F 0 "R1" V 5343 4100 50  0000 C CNN
F 1 "10k" V 5434 4100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5480 4100 50  0001 C CNN
F 3 "~" H 5550 4100 50  0001 C CNN
	1    5550 4100
	0    1    1    0   
$EndComp
$Comp
L pspice:CAP C3
U 1 1 6028BE8B
P 5700 4500
F 0 "C3" H 5878 4546 50  0000 L CNN
F 1 "100n" H 5878 4455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5700 4500 50  0001 C CNN
F 3 "~" H 5700 4500 50  0001 C CNN
	1    5700 4500
	1    0    0    -1  
$EndComp
Connection ~ 4500 3150
Wire Wire Line
	4500 3150 4600 3150
$Comp
L power:VDD #PWR04
U 1 1 6029856A
P 5300 4100
F 0 "#PWR04" H 5300 3950 50  0001 C CNN
F 1 "VDD" H 5315 4273 50  0000 C CNN
F 2 "" H 5300 4100 50  0001 C CNN
F 3 "" H 5300 4100 50  0001 C CNN
	1    5300 4100
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 602A6044
P 5700 4750
F 0 "#PWR05" H 5700 4500 50  0001 C CNN
F 1 "GND" H 5705 4577 50  0000 C CNN
F 2 "" H 5700 4750 50  0001 C CNN
F 3 "" H 5700 4750 50  0001 C CNN
	1    5700 4750
	1    0    0    -1  
$EndComp
Connection ~ 5700 4250
Connection ~ 5700 4100
Wire Wire Line
	5700 4100 5700 3750
Wire Wire Line
	5300 4100 5400 4100
Wire Wire Line
	1650 6950 1650 7050
$Comp
L cc2652_module_rf_star:CC2652 U3
U 1 1 600A0C80
P 6400 2250
F 0 "U3" H 6425 3365 50  0000 C CNN
F 1 "CC2652" H 6425 3274 50  0000 C CNB
F 2 "daughter_board:rf_star_cc2652" H 6050 2400 50  0001 C CNN
F 3 "" H 6050 2400 50  0001 C CNN
	1    6400 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 2000 7100 2000
Wire Wire Line
	7100 2100 7800 2100
Wire Wire Line
	5400 7000 5700 7000
Connection ~ 5400 7000
Wire Wire Line
	5400 6500 5400 7000
Wire Wire Line
	5150 6500 5400 6500
Wire Wire Line
	5700 6900 5700 7000
Wire Wire Line
	5150 7000 5400 7000
$Comp
L pspice:DIODE D5
U 1 1 6061B15F
P 4950 6500
F 0 "D5" H 4950 6765 50  0000 C CNN
F 1 "DIODE" H 4950 6674 50  0000 C CNN
F 2 "daughter_board:SOD-123FL" H 4950 6500 50  0001 C CNN
F 3 "~" H 4950 6500 50  0001 C CNN
	1    4950 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 7000 4750 7000
$Comp
L power:VDD #PWR03
U 1 1 605DD41F
P 5700 6900
F 0 "#PWR03" H 5700 6750 50  0001 C CNN
F 1 "VDD" H 5715 7073 50  0000 C CNN
F 2 "" H 5700 6900 50  0001 C CNN
F 3 "" H 5700 6900 50  0001 C CNN
	1    5700 6900
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR02
U 1 1 605D30F5
P 4550 7000
F 0 "#PWR02" H 4550 6850 50  0001 C CNN
F 1 "VCC" V 4565 7127 50  0000 L CNN
F 2 "" H 4550 7000 50  0001 C CNN
F 3 "" H 4550 7000 50  0001 C CNN
	1    4550 7000
	0    -1   -1   0   
$EndComp
$Comp
L pspice:DIODE D4
U 1 1 604B4750
P 4950 7000
F 0 "D4" H 4950 7265 50  0000 C CNN
F 1 "DIODE" H 4950 7174 50  0000 C CNN
F 2 "daughter_board:SOD-123FL" H 4950 7000 50  0001 C CNN
F 3 "~" H 4950 7000 50  0001 C CNN
	1    4950 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 7100 2650 7000
$Comp
L power:GND #PWR0122
U 1 1 602F2682
P 2650 7100
F 0 "#PWR0122" H 2650 6850 50  0001 C CNN
F 1 "GND" H 2655 6927 50  0000 C CNN
F 2 "" H 2650 7100 50  0001 C CNN
F 3 "" H 2650 7100 50  0001 C CNN
	1    2650 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 6500 4750 6500
$Comp
L pspice:CAP C6
U 1 1 602C7B9C
P 2650 6750
F 0 "C6" H 2828 6796 50  0000 L CNN
F 1 "100n" H 2828 6705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2650 6750 50  0001 C CNN
F 3 "~" H 2650 6750 50  0001 C CNN
	1    2650 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 6500 1650 6550
NoConn ~ 7100 2400
NoConn ~ 5750 2600
$Comp
L pspice:CAP C1
U 1 1 6022143B
P 8750 4250
F 0 "C1" H 8928 4296 50  0000 L CNN
F 1 "100n" H 8928 4205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8750 4250 50  0001 C CNN
F 3 "~" H 8750 4250 50  0001 C CNN
	1    8750 4250
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 602333A3
P 9100 4250
F 0 "#PWR09" H 9100 4000 50  0001 C CNN
F 1 "GND" H 9105 4077 50  0000 C CNN
F 2 "" H 9100 4250 50  0001 C CNN
F 3 "" H 9100 4250 50  0001 C CNN
	1    9100 4250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8350 4250 8500 4250
Wire Wire Line
	9100 4250 9000 4250
Wire Wire Line
	8500 4250 8500 4750
Wire Wire Line
	8500 4750 8700 4750
Connection ~ 8500 4250
Wire Wire Line
	3650 7100 3650 7000
$Comp
L power:GND #PWR08
U 1 1 60260269
P 3650 7100
F 0 "#PWR08" H 3650 6850 50  0001 C CNN
F 1 "GND" H 3655 6927 50  0000 C CNN
F 2 "" H 3650 7100 50  0001 C CNN
F 3 "" H 3650 7100 50  0001 C CNN
	1    3650 7100
	1    0    0    -1  
$EndComp
$Comp
L pspice:CAP C4
U 1 1 6026026F
P 3650 6750
F 0 "C4" H 3828 6796 50  0000 L CNN
F 1 "1u" H 3828 6705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3650 6750 50  0001 C CNN
F 3 "~" H 3650 6750 50  0001 C CNN
	1    3650 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 6500 2650 6500
Connection ~ 3650 6500
Wire Wire Line
	3450 2800 5750 2800
$Comp
L cc2652_module_rf_star:EVQ-P7A01P-switch2 SW1
U 1 1 605C7098
P 4350 4150
F 0 "SW1" H 4794 3996 50  0000 L CNN
F 1 "EVQ-P7A01P-switch2" H 4794 3905 50  0000 L CNN
F 2 "daughter_board:Panasonic-EVQ-P7A01P-0-0-MFG" H 4350 4350 50  0001 L CNN
F 3 "https://industrial.panasonic.com/cdbs/www-data/pdf/ATK0000/ATK0000C378.pdf" H 4350 4450 50  0001 L CNN
F 4 "No" H 4350 4550 50  0001 L CNN "automotive"
F 5 "Switch" H 4350 4650 50  0001 L CNN "category"
F 6 "50mA" H 4350 4750 50  0001 L CNN "contact current rating"
F 7 "500mΩ" H 4350 4850 50  0001 L CNN "contact resistance"
F 8 "Electromechanical" H 4350 4950 50  0001 L CNN "device class L1"
F 9 "Switches" H 4350 5050 50  0001 L CNN "device class L2"
F 10 "Tactile Switches" H 4350 5150 50  0001 L CNN "device class L3"
F 11 "SWITCH TACTILE SPST-NO 0.05A 12V" H 4350 5250 50  0001 L CNN "digikey description"
F 12 "P16763CT-ND" H 4350 5350 50  0001 L CNN "digikey part number"
F 13 "100000Cycles" H 4350 5450 50  0001 L CNN "electromechanical life"
F 14 "1.55mm" H 4350 5550 50  0001 L CNN "height"
F 15 "Yes" H 4350 5650 50  0001 L CNN "lead free"
F 16 "2f293019bba4f771" H 4350 5750 50  0001 L CNN "library id"
F 17 "Panasonic" H 4350 5850 50  0001 L CNN "manufacturer"
F 18 "Surface Mount" H 4350 5950 50  0001 L CNN "mount"
F 19 "SWITCH TACTILE SPST-NO 0.05A 12V" H 4350 6050 50  0001 L CNN "mouser description"
F 20 "667-EVQP7A01P" H 4350 6150 50  0001 L CNN "mouser part number"
F 21 "2.2N" H 4350 6250 50  0001 L CNN "operating force"
F 22 "SMT_SW_3MM5_2MM9" H 4350 6350 50  0001 L CNN "package"
F 23 "Yes" H 4350 6450 50  0001 L CNN "rohs"
F 24 "+70°C" H 4350 6550 50  0001 L CNN "temperature range high"
F 25 "-20°C" H 4350 6650 50  0001 L CNN "temperature range low"
F 26 "SPST-NO" H 4350 6750 50  0001 L CNN "throw configuration"
F 27 "12V" H 4350 6850 50  0001 L CNN "voltage rating DC"
	1    4350 4150
	1    0    0    -1  
$EndComp
$Comp
L cc2652_module_rf_star:EVQ-P7A01P-switch2 SW2
U 1 1 605EE2DD
P 6450 3950
F 0 "SW2" H 6894 3796 50  0000 L CNN
F 1 "EVQ-P7A01P-switch2" H 6894 3705 50  0000 L CNN
F 2 "daughter_board:Panasonic-EVQ-P7A01P-0-0-MFG" H 6450 4150 50  0001 L CNN
F 3 "https://industrial.panasonic.com/cdbs/www-data/pdf/ATK0000/ATK0000C378.pdf" H 6450 4250 50  0001 L CNN
F 4 "No" H 6450 4350 50  0001 L CNN "automotive"
F 5 "Switch" H 6450 4450 50  0001 L CNN "category"
F 6 "50mA" H 6450 4550 50  0001 L CNN "contact current rating"
F 7 "500mΩ" H 6450 4650 50  0001 L CNN "contact resistance"
F 8 "Electromechanical" H 6450 4750 50  0001 L CNN "device class L1"
F 9 "Switches" H 6450 4850 50  0001 L CNN "device class L2"
F 10 "Tactile Switches" H 6450 4950 50  0001 L CNN "device class L3"
F 11 "SWITCH TACTILE SPST-NO 0.05A 12V" H 6450 5050 50  0001 L CNN "digikey description"
F 12 "P16763CT-ND" H 6450 5150 50  0001 L CNN "digikey part number"
F 13 "100000Cycles" H 6450 5250 50  0001 L CNN "electromechanical life"
F 14 "1.55mm" H 6450 5350 50  0001 L CNN "height"
F 15 "Yes" H 6450 5450 50  0001 L CNN "lead free"
F 16 "2f293019bba4f771" H 6450 5550 50  0001 L CNN "library id"
F 17 "Panasonic" H 6450 5650 50  0001 L CNN "manufacturer"
F 18 "Surface Mount" H 6450 5750 50  0001 L CNN "mount"
F 19 "SWITCH TACTILE SPST-NO 0.05A 12V" H 6450 5850 50  0001 L CNN "mouser description"
F 20 "667-EVQP7A01P" H 6450 5950 50  0001 L CNN "mouser part number"
F 21 "2.2N" H 6450 6050 50  0001 L CNN "operating force"
F 22 "SMT_SW_3MM5_2MM9" H 6450 6150 50  0001 L CNN "package"
F 23 "Yes" H 6450 6250 50  0001 L CNN "rohs"
F 24 "+70°C" H 6450 6350 50  0001 L CNN "temperature range high"
F 25 "-20°C" H 6450 6450 50  0001 L CNN "temperature range low"
F 26 "SPST-NO" H 6450 6550 50  0001 L CNN "throw configuration"
F 27 "12V" H 6450 6650 50  0001 L CNN "voltage rating DC"
	1    6450 3950
	1    0    0    -1  
$EndComp
NoConn ~ 7100 1800
NoConn ~ 7100 1900
NoConn ~ 7100 2200
NoConn ~ 7100 2300
$Comp
L Switch:SW_Reed_Opener SW3
U 1 1 618E4CE7
P 1500 1200
F 0 "SW3" H 1500 1422 50  0000 C CNN
F 1 "SW_Reed_Opener" H 1500 1331 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P20.32mm_Horizontal" H 1500 1200 50  0001 C CNN
F 3 "~" H 1500 1200 50  0001 C CNN
	1    1500 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 618E9CC4
P 1950 1200
F 0 "#PWR014" H 1950 950 50  0001 C CNN
F 1 "GND" H 1955 1027 50  0000 C CNN
F 2 "" H 1950 1200 50  0001 C CNN
F 3 "" H 1950 1200 50  0001 C CNN
	1    1950 1200
	1    0    0    -1  
$EndComp
Text Label 7350 2000 0    50   ~ 0
DIO_23
Text Label 7350 2100 0    50   ~ 0
DIO_24
Text Label 850  1200 0    50   ~ 0
DIO_23
Wire Wire Line
	850  1200 1300 1200
Wire Wire Line
	1700 1200 1950 1200
$Comp
L Device:R R12
U 1 1 6191D62C
P 2200 2200
F 0 "R12" V 1993 2200 50  0000 C CNN
F 1 "18" V 2084 2200 50  0000 C CNN
F 2 "daughter_board:RESC1633X60N" V 2130 2200 50  0001 C CNN
F 3 "~" H 2200 2200 50  0001 C CNN
	1    2200 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 61922E7D
P 1800 2200
F 0 "R10" V 1593 2200 50  0000 C CNN
F 1 "470k" V 1684 2200 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1730 2200 50  0001 C CNN
F 3 "~" H 1800 2200 50  0001 C CNN
	1    1800 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R11
U 1 1 6192875C
P 1800 2550
F 0 "R11" V 1593 2550 50  0000 C CNN
F 1 "470k" V 1684 2550 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1730 2550 50  0001 C CNN
F 3 "~" H 1800 2550 50  0001 C CNN
	1    1800 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 6193940A
P 1800 2800
F 0 "#PWR011" H 1800 2550 50  0001 C CNN
F 1 "GND" H 1805 2627 50  0000 C CNN
F 2 "" H 1800 2800 50  0001 C CNN
F 3 "" H 1800 2800 50  0001 C CNN
	1    1800 2800
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR010
U 1 1 6194646D
P 1800 1950
F 0 "#PWR010" H 1800 1800 50  0001 C CNN
F 1 "VDD" H 1815 2123 50  0000 C CNN
F 2 "" H 1800 1950 50  0001 C CNN
F 3 "" H 1800 1950 50  0001 C CNN
	1    1800 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 1950 1800 2050
Wire Wire Line
	1800 2350 1800 2400
Wire Wire Line
	1800 2700 1800 2800
Wire Wire Line
	1800 2350 2200 2350
Connection ~ 1800 2350
Wire Wire Line
	2550 2050 2200 2050
Text Label 2300 1550 0    50   ~ 0
DIO_24
Text Notes 2700 2800 0    50   ~ 0
Power Sensor
Connection ~ 2650 6500
$Comp
L Connector_Generic:Conn_01x03 J4
U 1 1 619A6716
P 2300 3850
F 0 "J4" H 2150 4175 59  0000 L BNN
F 1 "Conn_01x03" H 2150 3550 59  0001 L BNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 2300 3850 50  0001 C CNN
F 3 "" H 2300 3850 50  0001 C CNN
	1    2300 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 619B7353
P 1900 4000
F 0 "#PWR013" H 1900 3750 50  0001 C CNN
F 1 "GND" H 1905 3827 50  0000 C CNN
F 2 "" H 1900 4000 50  0001 C CNN
F 3 "" H 1900 4000 50  0001 C CNN
	1    1900 4000
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR012
U 1 1 619BFDD8
P 1900 3750
F 0 "#PWR012" H 1900 3600 50  0001 C CNN
F 1 "VDD" H 1915 3923 50  0000 C CNN
F 2 "" H 1900 3750 50  0001 C CNN
F 3 "" H 1900 3750 50  0001 C CNN
	1    1900 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 3850 2100 3850
Text Label 1350 3850 0    50   ~ 0
DIO_23
Wire Wire Line
	1900 4000 1900 3950
Wire Wire Line
	1900 3950 2100 3950
Wire Wire Line
	2100 3750 1900 3750
Text Notes 1000 3600 0    50   ~ 0
PIR Sensor
Wire Wire Line
	8000 1100 8900 1100
Wire Wire Line
	8000 1000 9150 1000
Wire Wire Line
	5750 2000 4800 2000
Wire Wire Line
	5750 1900 4800 1900
$Comp
L Mechanical:MountingHole H1
U 1 1 61AB4A10
P 3550 5100
F 0 "H1" H 3650 5146 50  0000 L CNN
F 1 "MountingHole" H 3650 5055 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_DIN965_Pad" H 3550 5100 50  0001 C CNN
F 3 "~" H 3550 5100 50  0001 C CNN
	1    3550 5100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 61AB5468
P 3550 5350
F 0 "H2" H 3650 5396 50  0000 L CNN
F 1 "MountingHole" H 3650 5305 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_DIN965_Pad" H 3550 5350 50  0001 C CNN
F 3 "~" H 3550 5350 50  0001 C CNN
	1    3550 5350
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 61AB74BF
P 4300 5100
F 0 "H3" H 4400 5146 50  0000 L CNN
F 1 "MountingHole" H 4400 5055 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_DIN965_Pad" H 4300 5100 50  0001 C CNN
F 3 "~" H 4300 5100 50  0001 C CNN
	1    4300 5100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 61ABDFBD
P 4300 5350
F 0 "H4" H 4400 5396 50  0000 L CNN
F 1 "MountingHole" H 4400 5305 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_DIN965_Pad" H 4300 5350 50  0001 C CNN
F 3 "~" H 4300 5350 50  0001 C CNN
	1    4300 5350
	1    0    0    -1  
$EndComp
$Comp
L cc2652_module_rf_star:SJ-3524-SMT J2
U 1 1 618CE460
P 3250 2250
F 0 "J2" H 2820 2204 50  0000 R CNN
F 1 "SJ1-3523N" H 2820 2295 50  0000 R CNN
F 2 "daughter_board:CUI_SJ-3524-SMT" H 3250 2250 50  0001 L BNN
F 3 "" H 3250 2250 50  0001 L BNN
F 4 "1.02" H 3250 2250 50  0001 L BNN "PARTREV"
F 5 "CUI" H 3250 2250 50  0001 L BNN "MANUFACTURER"
F 6 "Manufacturer recommendation" H 3250 2250 50  0001 L BNN "STANDARD"
	1    3250 2250
	-1   0    0    1   
$EndComp
Wire Wire Line
	2750 2150 2550 2150
Wire Wire Line
	2550 2150 2550 2050
NoConn ~ 2750 2250
NoConn ~ 2750 2350
$Comp
L Device:R R8
U 1 1 6195F132
P 2450 1850
F 0 "R8" V 2243 1850 50  0000 C CNN
F 1 "1k" V 2334 1850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2380 1850 50  0001 C CNN
F 3 "~" H 2450 1850 50  0001 C CNN
	1    2450 1850
	0    1    1    0   
$EndComp
$Comp
L Device:D_Zener D6
U 1 1 6196DCC6
P 2950 1850
F 0 "D6" H 2950 2067 50  0000 C CNN
F 1 "D_Zener" H 2950 1976 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-923" H 2950 1850 50  0001 C CNN
F 3 "~" H 2950 1850 50  0001 C CNN
	1    2950 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 619CC4FA
P 3200 1850
F 0 "#PWR06" H 3200 1600 50  0001 C CNN
F 1 "GND" H 3205 1677 50  0000 C CNN
F 2 "" H 3200 1850 50  0001 C CNN
F 3 "" H 3200 1850 50  0001 C CNN
	1    3200 1850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3100 1850 3200 1850
Wire Wire Line
	2300 1850 2200 1850
Wire Wire Line
	2200 1850 2200 2050
Connection ~ 2200 2050
Wire Wire Line
	2800 1850 2700 1850
Connection ~ 2700 1850
Wire Wire Line
	2700 1850 2600 1850
Wire Wire Line
	2700 1550 2300 1550
Wire Wire Line
	2700 1550 2700 1850
$Comp
L Connector_Generic:Conn_01x02 J5
U 1 1 61A47F25
P 900 6600
F 0 "J5" H 818 6275 50  0000 C CNN
F 1 "Conn_01x02" H 818 6366 50  0000 C CNN
F 2 "Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical" H 900 6600 50  0001 C CNN
F 3 "~" H 900 6600 50  0001 C CNN
	1    900  6600
	-1   0    0    1   
$EndComp
Wire Wire Line
	1100 6500 1650 6500
Connection ~ 1650 6500
Wire Wire Line
	1650 6500 2650 6500
Wire Wire Line
	1100 6600 1300 6600
Wire Wire Line
	1300 6600 1300 7050
Wire Wire Line
	1300 7050 1650 7050
Connection ~ 1650 7050
Wire Wire Line
	1650 7050 1650 7100
Text Label 3050 6500 0    50   ~ 0
VBAT
Wire Wire Line
	2750 2450 2550 2450
Wire Wire Line
	2550 2450 2550 2350
Wire Wire Line
	2550 2350 2200 2350
Connection ~ 2200 2350
$EndSCHEMATC
