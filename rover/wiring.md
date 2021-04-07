# Wiring

## Loose power wiring

| From | To | Notes |
|---|---|---|
| XT60 Connector +ve | Switch |  |
| XT60 Connector -ve | GND | Bus bars on both boards |
| Switch | +8V |  |
| +8V | RRB V+ | For motors |
| +8V | BDC IN+ | Buck down convertor |
| XT60 GND | RRB GND | For motors |
| XT60 GND | BDC IN- | Buck down convertor |
| BDC OUT+ | Protoboard header + | Output set to 5V5 |
| BDC OUT- | Protoboard header - | GND for protoboard |
| RRB motor left 1 | Left motor red | Loose wire |
| RRB motor left 2 | Left motor black | Loose wire |
| RRB motor right 1 | Right motor red | Loose wire |
| RRB motor right 2 | Right motor black | Loose wire |

## Protoboard power wiring

| From | To | Notes |
|---|---|---|
| Protoboard header - | GND bar near header | |
| Protoboard header + | Servo 1 + | 5V5 at up to 1A |
| Protoboard header + | Servo 2 + | 5V5 at up to 1A |
| Protoboard header + | ESP32 VIN | Max. 15V so fine with 8V |
| ESP32 3v3 | Encoder VCC | |
| ESP32 3v3 | RRB 1 | 3V3 for pull ups for SW1/2 |

## EPS32 to RasPiRover Board

| ESP32 Pin | Direction | RRB | Notes |
|---|---|---|---|
| D12 | Out | 11 | AIN1 |
| D13 | Out |  7 | AIN2 |
| D14 | Out | 18 | PWMA |
| D22 | Out | 24 | LED1 |
| D23 | Out | 26 | LED2 |
| D25 | Out | 19 | BIN1 |
| D26 | Out | 22 | BIN2 |
| D27 | Out |  8 | PWMB |
| D32 | Out | 12 | Sonar Trigger |
| D33 | In | 16 | Sonar Echo |
| NC | In | 23 | SW1 |
| NC | In | 21 | SW2 |
| NC | Out | 13 | OC2 |
| NC | Out | 15 | OC1 |
| NC | ? |  3 | I2C SDA |
| NC | ? |  5 | I2C SCL |

## Other ESP32 wiring

| ESP32 Pin | Direction | To |
|---|---|---|
| D5 | Out | Servo 1 PWM |
| D18 | Out | Servo 2 PWM |
| D19 | In | Encoder Left |
| D21 | In | Encoder Right |
| D4 | | Spare |
| D15 | | Spare |
| RX0 | | Spare |
| TX0 | | Spare |
| RX2 | | Spare |
| TX2 | | Spare |

## Sonar connector

Connects to the `SONAR` RasPiRobot board connector.

| SR-04 Name | Colour |
|---|---|
| GND | Black |
| Echo | White |
| Trigger | Grey |
| VCC | Purple |

## Servo connectors

| Name | Colour | Notes |
|---|---|---|
| GND | Brown | |
| VCC | Red | |
| PWM Input | Orange | ESP32 D4,D5 |

## Battery voltage monitor

ESP32 ADCs are 12 bit over a range of 0V to 1.024V.  In practice, the ADC inputs are noisy, using 10 bit resolution averaged over 8 samples is recommended.  Voltage ranges are:

-0dB: 0.04V to 1.024V
-6dB: 0.08V to 2.00V
-11dB: 0.1V to ~3.20V

The best result, reading from the graph, seems to be the -0dB for linearity over the range 0.04V (count 1 ish) to 1.00V (count 990 ish).

Absolute maximum input voltage is VDD (the 3V3 pin).

Source: https://www.esp32.com/viewtopic.php?t=1045

### Implementation

ESP32 pin D15 can be used as an ADC input (ADC2_CH3).

A hardware voltage divider is needed to reduce the battery voltage to the ADC voltage range.  I have a limited range of resistors at home so 100k:10k gives 11 to 1 reduction, so 11V range which will work nicely with the 2S battery (7.0 to 8.4V).

## Hall sensor

The ESP32 has a hall sensor.  Can be used for what?

SENSOR_VP, SENSOR_VN feed into SAR ADC 1 inputs.

Could be used to turn off the robot or help with docking.  Needs to be tested.
