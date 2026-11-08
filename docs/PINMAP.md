# 핀맵 (NUCLEO-F103RB / STM32F103RBT6)

`lib/MySensors/balancer_pinouts.h`, `examples/can_receiver/main.cpp`, 노션 개발 기록을 기준으로 정리했습니다.
CubeMX에서 열어 볼 수 있는 핀 할당 전용 파일은 [`hardware/cubemx/self_balancing_pinmap.ioc`](../hardware/cubemx/self_balancing_pinmap.ioc)입니다. 코드 생성용이 아니라 핀 배치만 담겨 있습니다.

## 모터 A / 모터 B

| 기능 | 모터 A | 주변장치 | 모터 B | 주변장치 |
|---|---|---|---|---|
| 엔코더 SCK | PA5 | SPI1 | PB13 | SPI2 |
| 엔코더 MISO | PA6 | SPI1 | PB14 | SPI2 |
| 엔코더 MOSI | PA7 | SPI1 | PB15 | SPI2 |
| 엔코더 CS | PB5 | GPIO | PB2 | GPIO |
| PWM A상 | PB10 | TIM2_CH3 | PA8 | TIM1_CH1 |
| PWM B상 | PB6 | TIM4_CH1 | PA9 | TIM1_CH2 |
| PWM C상 | PB4 | TIM3_CH1 | PA10 | TIM1_CH3 |
| 드라이버 Enable | PC7 | GPIO | PA11 → **PC8** | GPIO |

- 엔코더: AS5048A (SPI Mode 1, 1MHz, 14bit)
- 드라이버: SimpleFOC Shield v3.2 × 2. 쉴드 기본 배선은 두 모터의 타이머가 겹쳐서, PWM 선을 끊고 위 핀으로 다시 납땜했습니다.
- 모터 A는 타이머 3개(TIM2/TIM4/TIM3)에 나뉘어 있고, 모터 B는 TIM1 한 개로 3상을 모두 냅니다.

## 통신

| 기능 | 핀 | 주변장치 | 연결 |
|---|---|---|---|
| CAN RX | PA11 | CAN | TJA1050 RXD |
| CAN TX | PA12 | CAN | TJA1050 TXD |
| I2C SCL | PB8 | I2C1 (remap) | IMU(BNO055), LCD1602 |
| I2C SDA | PB9 | I2C1 (remap) | IMU(BNO055), LCD1602 |
| UART TX | PA2 | USART2 | ST-LINK 가상 COM (디버그, 115200) |
| UART RX | PA3 | USART2 | ST-LINK 가상 COM |

- CAN: 500kbps 기준 비트 타이밍 (PCLK1 36MHz, Prescaler 9, BS1 5TQ, BS2 2TQ, SJW 1TQ)
- 라즈베리파이 → STM32 메시지: ID `0x100`, 4바이트, `[throttle×1000 (int16 BE), steer×1000 (int16 BE)]`

## 시스템

| 핀 | 기능 |
|---|---|
| PA13 / PA14 | SWD (SWDIO / SWCLK) |
| PD0 | HSE 8MHz 입력 (ST-LINK MCO) |
| PC13 | B1 사용자 버튼 |

## ⚠️ 핀 충돌과 정리한 내용

| 항목 | 기존 코드 | `.ioc` 할당 | 이유 |
|---|---|---|---|
| CAN | PB8(RX) / PB9(TX), remap 2 | **PA11 / PA12** | Arduino `Wire`(I2C1)가 기본으로 PB8/PB9를 써서 IMU와 겹침 |
| 모터 B Enable | PA11 | **PC8** | CAN을 PA11/PA12로 옮기면서 비움 |

- CAN 수신 테스트(`examples/can_receiver`)는 IMU 없이 단독으로 돌려서 PB8/PB9에서도 동작했습니다. 밸런싱 코드와 합치려면 위처럼 옮겨야 합니다.
- I2C1 기본 핀(PB6/PB7)과 I2C2(PB10/PB11)는 모터 A PWM(PB6, PB10)과 겹칩니다. 그래서 IMU와 LCD는 I2C1 remap(PB8/PB9) 하나에 같이 묶었습니다. 주소는 BNO055 0x28, LCD1602 I2C 모듈 보통 0x27로 서로 다릅니다.
- PB4는 기본이 JTAG NJTRST입니다. TIM3_CH1로 쓰려면 디버그를 SWD 전용으로 두어야 합니다.
- TIM2_CH3(PB10)은 TIM2 partial remap, TIM3_CH1(PB4)은 TIM3 partial remap이 필요합니다. CubeMX에서 핀을 고르면 자동으로 잡힙니다.
- PA5는 보드의 LD2 LED와 연결되어 있어서, 엔코더 A 통신 중에 LED가 깜빡입니다.

## 인터페이스 PCB (`hardware/interface_pcb`)

| 커넥터 | 핀 | 신호 |
|---|---|---|
| BT1 | 배럴잭 | 12V 배터리 입력 → 전원 스위치(SW1) → 역전압 방지 다이오드(D1) |
| J1 (1×4) | 1, 3 / 2 / 4 | GND / 5V 입력 (AMS1117 입력) / 12V 출력 |
| J4 (1×8) | 1 / 2~4 / 5~8 | 12V / 5V / 3.3V (AMS1117-3.3 출력) |
| J6 (1×8) | 1~8 | GND |
| J2 (1×4) | 1~4 | 버튼 1~4 출력 → 라즈베리파이 GPIO (10kΩ 풀다운, 누르면 3.3V) |

- 버튼 신호가 풀다운 기준이라, 라즈베리파이 GND를 이 PCB GND와 같이 묶어야 동작합니다.
- C3(100µF)가 12V 라인에 직렬로 들어가 있어, J1-4/J4-1로 DC 12V가 나가지 않습니다. 다음 버전에서는 C3를 12V와 GND 사이(병렬)로 옮겨야 합니다.
