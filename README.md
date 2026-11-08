# FOC 정밀 제어 기반 자율주행 밸런싱 로봇

BLDC 모터 2개로 균형을 잡는 2륜 셀프 밸런싱 로봇입니다. 이전 프로젝트([라즈베리파이 자율주행 버스](https://github.com/GunCHimSaK/raspi-autonomous-car))의 차선 인식 주행을 이 로봇에 옮기는 것을 목표로 합니다.
모터는 FOC(Field-Oriented Control)로 토크를 직접 제어해서, DC 모터보다 부드럽고 정밀하게 움직입니다.

- **MCU**: STM32 NUCLEO-F103RB (PlatformIO + Arduino 프레임워크, 일부 HAL 직접 사용)
- **상위 제어기**: Raspberry Pi 4B (카메라 비전, 주행 명령)
- **통신**: CAN (Raspberry Pi MCP2515 HAT ↔ STM32 TJA1050)
- **기간**: 2025.06 ~ 2026.01

## 시스템 구조

```
┌──────── Raspberry Pi 4B ────────┐   CAN 500kbps    ┌───────────────── STM32 F103RB ─────────────────┐
│ 카메라 → 차선 인식 → throttle/steer │ ───────────────▶ │ CAN 수신 (ID 0x100)                             │
│ (raspberry_pi/can_sender.py)    │  MCP2515 ↔ TJA1050│                                                │
└─────────────────────────────────┘                  │ IMU(I2C) → 기울기 → 밸런싱 PID ─┐               │
                                                     │                               ├─▶ FOC 토크 제어 │
                                                     │ AS5048A(SPI1/SPI2) → 회전자 각도 ┘   (SimpleFOC)  │
                                                     └───────────────┬────────────────┬───────────────┘
                                                              SimpleFOC Shield A   SimpleFOC Shield B
                                                                GM4108 모터 A        GM4108 모터 B
```

## 제어 구조

**1. FOC 토크 제어 (SimpleFOC)**
- 모터: GM4108 (11 pole pairs), 전원 12V, 센서 정렬 전압 5V
- 제어 모드: `MotionControlType::torque` (전압 기반 토크 제어)
- 회전자 각도: AS5048A 14bit 자기 엔코더

**2. 밸런싱 제어**
- 기울기(roll) 오차를 PID(`pid_stb`, P=17, 출력 제한 11V)에 넣어 두 모터의 토크 명령으로 사용
- 좌우 회전은 두 모터 토크에 `±steering`을 더하고 빼는 방식
- 시리얼 Commander로 PID 실시간 튜닝 (`A` = 밸런싱 PID, `B` = 속도 PID)
- PID 출력이 8V를 넘거나 IMU 값이 NaN이면 경고 출력

**3. 자세 추정 (IMU)**

| 버전 | 파일 | 방식 | 결과 |
|---|---|---|---|
| 1 | `MPU6050_Handler.cpp` | 레지스터 직접 읽기(14바이트 burst read) + 가속도 LPF + 상보 필터 | 급격한 기울기에서 멈춤 |
| 2 | `mpu6050v2.cpp` | MPU6050 DMP 쿼터니언으로 pitch 계산 | 튜닝 성공, 그러나 한 방향 drift 발생 |
| 3 | — | BNO055 9축 센서로 교체 | roll/pitch drift 해결 |

**4. CAN 통신**
- STM32 CAN을 HAL로 직접 초기화 (500kbps, 모든 ID 수신 필터)
- 메시지 `0x100`: `throttle×1000`, `steer×1000`을 big-endian int16 2개로 전송

## 폴더 구조

```
├── src/main.cpp                  밸런싱 메인 펌웨어
├── lib/MySensors/
│   ├── balancer_pinouts.h        핀 정의
│   ├── CustomAS5048A.*           AS5048A SPI 저수준 드라이버 (SimpleFOC Sensor 상속)
│   ├── MPU6050_Handler.*         MPU6050 레지스터 + 상보 필터
│   └── mpu6050v2.*               MPU6050 DMP
├── examples/can_receiver/        STM32 CAN 수신 테스트 (HAL)
├── raspberry_pi/can_sender.py    라즈베리파이 CAN 송신 테스트 (python-can)
├── hardware/
│   ├── cubemx/self_balancing_pinmap.ioc   핀 할당 전용 CubeMX 파일
│   └── interface_pcb/            전원·버튼 인터페이스 PCB (KiCad, 거버, BOM)
├── docs/PINMAP.md                전체 핀맵과 핀 충돌 정리
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
| I2C1 (IMU, LCD) | PB8 SCL / PB9 SDA |
| CAN | PA11 RX / PA12 TX (`.ioc` 기준) |
| 디버그 UART | PA2 / PA3 (USART2, ST-LINK) |

자세한 내용과 핀 충돌 정리는 [docs/PINMAP.md](docs/PINMAP.md)에 있습니다.

## 빌드

```bash
pio run -t upload          # 밸런싱 펌웨어 업로드
pio device monitor         # 115200bps, roll / PID 출력 / q축 전압 로그
```
CAN 테스트는 `examples/can_receiver/main.cpp`를 `src/`에 넣고 빌드합니다.
라즈베리파이 쪽은 `sudo ip link set can0 up type can bitrate 500000` 후 `python3 raspberry_pi/can_sender.py`를 실행합니다.

## 개발 중 문제와 해결

| 문제 | 원인 | 해결 |
|---|---|---|
| SimpleFOC에서 AS5048A 값이 안 읽힘 | 원인을 찾지 못함 (라이브러리 기본 SPI 센서 클래스 문제로 추정) | SPI를 직접 읽는 `CustomAS5048A` 클래스를 만들어 SimpleFOC `Sensor`로 연결 |
| 모터 2개를 동시에 못 돌림 | 두 쉴드의 PWM 핀이 같은 타이머를 사용 | PWM 선을 끊고 타이머가 겹치지 않는 핀(모터 B → TIM1)으로 다시 납땜 |
| 모터 1개 고장 | 모터 규격보다 긴 나사가 내부 코일을 끊음 | 모터 교체, 나사 길이 확인 |
| 급격히 기울이면 로봇이 멈춤 | IMU 통신 중 멈춤 | burst read로 I2C 통신 횟수를 줄이고, DMP 방식으로 변경 |
| 각도가 한 방향으로 흘러감 (drift) | 6축 센서 한계 또는 센서 불량 | BNO055 9축 센서로 교체 |
| CAN과 IMU 핀 충돌 | CAN remap(PB8/PB9)과 I2C1 핀이 같음 | CAN을 PA11/PA12로 옮기는 핀 배치 정리 (`.ioc`) |

## 사용 도구
STM32CubeMX · PlatformIO · VS Code · SimpleFOC · KiCad · Fusion 360
