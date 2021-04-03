# Wiring

## Power wiring

```text
| From | To | Notes |
|---|---|---|
| XT60 Connector +ve | Switch |  |
| XT60 Connector -ve | GND | Bus bars on both boards |
| Switch | +8V |  |
| +8V | RRB V+ | For motors |
| +8V | ESP32 VIN | Max. 15V so fine with 8V |
| +8V | Convertor In |  |
| Convertor Out | Servo 1 + | |
| Convertor Out | Servo 2 + | |
| GND | RRB GND | |
| GND | RRB pin 6 | |
| ESP32 3v3 | RRB 1 | 3V3 for pull ups for SW1/2 |
| RRB motor left 1 | Left motor red | Loose wire |
| RRB motor left 2 | Left motor black | Loose wire |
| RRB motor right 1 | Right motor red | Loose wire |
| RRB motor right 2 | Right motor black | Loose wire |
```

## EPS32 to RasPiRover Board

```text
| ESP32 Pin | Direction | RRB | Notes |
|---|---|---|---|
| NC | ? |  3 | I2C SDA |
| NC | ? |  5 | I2C SCL |
| D? | O |  7 | AIN2 |
| D? | O |  8 | PWMB |
| D? | O | 11 | AIN1 |
| D? | O | 12 | Sonar Trigger |
| NC | O | 13 | OC2 |
| NC | O | 15 | OC1 |
| D? | I | 16 | Sonar Echo |
| D? | O | 18 | PWMA |
| D? | O | 19 | BIN1 |
| NC | I | 21 | SW2 |
| D? | O | 22 | BIN2 |
| NC | I | 23 | SW1 |
| D? | O | 24 | LED1 |
| D? | O | 26 | LED2 |
```

## Other wiring

```text
| From | To | Notes |
|---|---|---|
| ESP32 D?? (output) | Servo 1 PWM | |
| ESP32 D?? (output) | Servo 2 PWM | |
| ESP32 D?? (input) | Encoder Left | |
| ESP32 D?? (input) | Encoder Right | |
```

## Notes

The RasPiRobot board H-bridge motor controller is a TB6612FNG and the data sheet is [here](../resources/TB6612FNG_datasheet_en_20141001.pdf).
