# FOC 정밀 제어 기반 자율주행 밸런싱 로봇

BLDC 모터 2개로 균형을 잡는 2륜 셀프 밸런싱 로봇입니다. 이전 프로젝트([라즈베리파이 자율주행 버스](https://github.com/GunCHimSaK/raspi-autonomous-car))의 차선 인식 주행을 이 로봇에 옮기는 것을 목표로 합니다.
모터는 FOC(Field-Oriented Control)로 토크를 직접 제어해서, DC 모터보다 부드럽고 정밀하게 움직입니다.

- **MCU**: STM32 NUCLEO-F446RE (PlatformIO + Arduino 프레임워크). 처음에는 NUCLEO-F103RB로 시작했습니다.
- **IMU**: BNO055 (MPU6050에서 교체)
- **상위 제어기**: Raspberry Pi 4B (카메라 비전, 주행 명령)
- **통신**: CAN (Raspberry Pi MCP2515 HAT ↔ STM32 TJA1050)
- **기간**: 2025.06 ~ 2026.02

## 시스템 구조

```
[Raspberry Pi 4B]  카메라 → 차선 인식 → throttle / steer
        │  CAN 500kbps (MCP2515 HAT ↔ TJA1050), ID 0x100
        ▼
[STM32 NUCLEO-F446RE]
   BNO055 (I2C1) ──▶ 현재 기울기 ──────────────┐
   AS5048A ×2 (SPI1/SPI2) ──▶ 바퀴 속도 ──▶ 속도 PID ──▶ 목표 기울기
                                               ▼
                                     기울기 PID ──▶ 토크 명령 ± steering
                                               ▼
                               SimpleFOC 토크 제어 (FOC)
        ├─ SimpleFOC Shield A → GM4108 모터 A
        └─ SimpleFOC Shield B → GM4108 모터 B
```

## 제어 구조

**1. FOC 토크 제어 (SimpleFOC)**
- 모터: GM4108 (11 pole pairs), 전원 설정 10V
- 제어 모드: `MotionControlType::torque` (전압 기반 토크 제어)
- 회전자 각도: AS5048A 14bit 자기 엔코더 (SPI 500kHz, 저수준 직접 읽기)

**2. 캐스케이드 밸런싱 제어** (`src/main.cpp`)

```
바퀴 평균 속도 = (모터1 속도 + 모터2 속도) / 2
목표 기울기 = LPF( pid_vel(바퀴 평균 속도 − LPF(throttle)) ) + 0.045      ← 바깥 루프 (속도)
토크 명령   = −pid_stb(목표 기울기 − 현재 기울기)                         ← 안쪽 루프 (기울기)
모터1 = 토크 명령 + LPF(steering),  모터2 = 토크 명령 − LPF(steering)
```

| 제어기 | P | I | D | 출력 제한 |
|---|---|---|---|---|
| `pid_stb` (기울기 → 토크) | 30 | 100 | 1 | 8V |
| `pid_vel` (속도 → 목표 기울기) | 0.01 | 0.03 | 0 | π/10 rad |

- 속도 루프가 로봇이 한쪽으로 굴러가는 것을 막고, `throttle`을 주면 그 속도로 움직입니다.
- `+0.045 rad`는 무게중심이 치우친 만큼의 보정값입니다.
- 시리얼 Commander로 PID 실시간 튜닝 (`A` = 기울기 PID, `B` = 속도 PID)

**3. 자세 추정 (BNO055)**
- 칩 안에서 9축 센서 퓨전을 하는 BNO055의 Euler pitch 레지스터(`0x1E`)를 2바이트 직접 읽습니다 (1° = 16 LSB).
- I2C1(PB8/PB9) 400kHz

| 버전 | 방식 | 결과 |
|---|---|---|
| MPU6050 + 상보 필터 | 레지스터 직접 읽기(burst read) + 가속도 LPF + 상보 필터 | 급격한 기울기에서 멈춤 |
| MPU6050 DMP | DMP 쿼터니언으로 pitch 계산 | 튜닝 성공, 그러나 한 방향 drift |
| **BNO055** | 9축 센서 퓨전 결과 직접 읽기 | drift 해결 (최종) |

**4. CAN 통신** (단독 테스트 완료, 밸런싱 코드와 통합 예정)
- STM32 CAN을 HAL로 직접 초기화한 수신 테스트 (500kbps, F103RB)
- 메시지 `0x100`: `throttle×1000`, `steer×1000`을 big-endian int16 2개로 전송

코드가 단계별로 어떻게 바뀌었는지는 [docs/DEVELOPMENT_LOG.md](docs/DEVELOPMENT_LOG.md)에 정리했습니다.

## 폴더 구조

```
├── src/main.cpp                  밸런싱 메인 펌웨어 (최종: F446RE, BNO055, 캐스케이드 PID)
├── lib/MySensors/
│   ├── balancer_pinouts.h        핀 정의
│   ├── CustomAS5048A.*           AS5048A SPI 저수준 드라이버 (SimpleFOC Sensor 상속)
│   ├── MPU6050_Handler.*         MPU6050 레지스터 + 상보 필터
│   └── mpu6050v2.*               MPU6050 DMP
├── versions/                     단계별 이전 코드 (v1 MPU6050, v2 CAN 수신, v3 BNO055)
├── raspberry_pi/can_sender.py    라즈베리파이 CAN 송신 테스트 (python-can)
├── hardware/
│   ├── cubemx/                   핀 할당 전용 CubeMX 파일 (F446RE, F103RB)
│   └── interface_pcb/            전원·버튼 인터페이스 PCB (KiCad, 거버, BOM)
├── docs/PINMAP.md                전체 핀맵과 핀 충돌 정리
├── docs/DEVELOPMENT_LOG.md       단계별 코드 변화 기록
└── platformio.ini
```

## 핀맵 요약

| 기능 | 모터 A | 모터 B |
|---|---|---|
| 엔코더 SPI (SCK/MISO/MOSI/CS) | SPI1: PA5 / PA6 / PA7 / PB5 | SPI2: PB13 / PB14 / PB15 / PB2 |
| PWM A/B/C상 | PB10 (TIM2) / PB6 (TIM4) / PB4 (TIM3) | PA8 / PA9 / PA10 (TIM1) |
| Enable | PC7 | PA11 (`.ioc`에서는 PC8) |

| 기능 | 핀 |
|---|---|
| I2C1 (BNO055, LCD) | PB8 SCL / PB9 SDA |
| CAN | PA11 RX / PA12 TX (`.ioc` 기준) |
| 디버그 UART | PA2 / PA3 (USART2, ST-LINK) |

자세한 내용과 핀 충돌 정리는 [docs/PINMAP.md](docs/PINMAP.md)에 있습니다.

## 빌드

```bash
pio run -t upload          # 밸런싱 펌웨어 업로드
pio device monitor         # 115200bps, Commander로 PID 튜닝
```
CAN 수신 테스트는 `versions/v2_can_receiver_hal_f103rb.cpp`를 `src/main.cpp`로 바꾸고, `platformio.ini`의 보드를 `nucleo_f103rb`로 바꿔 빌드합니다.
라즈베리파이 쪽은 `sudo ip link set can0 up type can bitrate 500000` 후 `python3 raspberry_pi/can_sender.py`를 실행합니다.

## 개발 중 문제와 해결

| 문제 | 원인 | 해결 |
|---|---|---|
| SimpleFOC에서 AS5048A 값이 안 읽힘 | 원인을 찾지 못함 (라이브러리 기본 SPI 센서 클래스 문제로 추정) | SPI를 직접 읽는 `CustomAS5048A` 클래스를 만들어 SimpleFOC `Sensor`로 연결 |
| 모터 2개를 동시에 못 돌림 | 두 쉴드의 PWM 핀이 같은 타이머를 사용 | PWM 선을 끊고 타이머가 겹치지 않는 핀(모터 B → TIM1)으로 다시 납땜 |
| 모터 1개 고장 | 모터 규격보다 긴 나사가 내부 코일을 끊음 | 모터 교체, 나사 길이 확인 |
| 급격히 기울이면 로봇이 멈춤 | IMU 통신 중 멈춤 | burst read로 I2C 통신 횟수를 줄이고, DMP 방식으로 변경 |
| 각도가 한 방향으로 흘러감 (drift) | 6축 센서 한계 또는 센서 불량 | BNO055 9축 센서로 교체 |
| 엔코더 값 불안정 | SPI 속도와 CS 타이밍 | SPI 1MHz → 500kHz, CS를 내린 뒤 1µs 대기 |
| 제자리에 서지 못하고 굴러감 | 기울기만 제어 | 바퀴 속도 루프를 바깥에 둔 캐스케이드 PID |
| 디커플링 커패시터가 제 역할을 못 함 (PCB) | 100µF 커패시터를 전원–GND 사이가 아니라 12V 라인에 직렬로 배치 | 2차 설계에서 전원–GND 사이, IC 핀 가까이 배치 |
| 센서·버튼 입력값이 튐 (PCB) | 커넥터에 GND 핀이 없어 기준 전위가 공유되지 않음 | GND 선을 따로 연결, 2차 설계에서 모든 신호 커넥터에 GND 핀 추가 |
| CAN과 IMU 핀 충돌 | CAN remap(PB8/PB9)과 I2C1 핀이 같음 | CAN을 PA11/PA12로 옮기는 핀 배치 정리 (`.ioc`) |

## 사용 도구
STM32CubeMX · PlatformIO · VS Code · SimpleFOC · KiCad · Fusion 360
