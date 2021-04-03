# Wiring

## Power wiring

| From | To | Notes |
|---|---|---|
| XT60 Connector +ve | Switch |  |
| XT60 Connector -ve | GND | Bus bars on both boards |
| Switch | +8V |  |
| +8V | RRB V+ | For motors |
| +8V | ESP32 VIN | Max. 15V so fine with 8V |
| +8V | Convertor In |  |
| Convertor Out | Servo 1 + | 5V at up to 1A |
| Convertor Out | Servo 2 + | 5V at up to 1A |
| GND | RRB GND | |
| GND | RRB pin 6 | |
| GND | Encoder GND | |
| ESP32 3v3 | RRB 1 | 3V3 for pull ups for SW1/2 |
| ESP32 3v3 | Encoder VCC | |
| RRB motor left 1 | Left motor red | Loose wire |
| RRB motor left 2 | Left motor black | Loose wire |
| RRB motor right 1 | Right motor red | Loose wire |
| RRB motor right 2 | Right motor black | Loose wire |

## EPS32 to RasPiRover Board

| ESP32 Pin | Direction | RRB | Notes |
|---|---|---|---|
| D12 | Out | 11 | AIN1 |
| D13 | Out |  7 | AIN2 |
| D14 | Out | 18 | PWMA |
| D18 | In | 21 | SW2 |
| D19 | In | 23 | SW1 |
| D22 | Out | 24 | LED1 |
| D23 | Out | 26 | LED2 |
| D25 | Out | 19 | BIN1 |
| D26 | Out | 22 | BIN2 |
| D27 | Out |  8 | PWMB |
| D32 | Out | 12 | Sonar Trigger |
| D33 | In | 16 | Sonar Echo |
| NC | Out | 13 | OC2 |
| NC | Out | 15 | OC1 |
| NC | ? |  3 | I2C SDA |
| NC | ? |  5 | I2C SCL |

## Other wiring

| ESP32 Pin | Direction | To |
|---|---|---|
| D4 | Out | Servo 1 PWM |
| D5 | Out | Servo 2 PWM |
| D34 | In | Encoder Left |
| D35 | In | Encoder Right |

## Notes

The RasPiRobot board H-bridge motor controller is a TB6612FNG and the data sheet is [here](../resources/TB6612FNG_datasheet_en_20141001.pdf).
