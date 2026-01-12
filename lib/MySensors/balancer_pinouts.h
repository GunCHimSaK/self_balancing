#ifndef BALANCER_PINOUTS_H
#define BALANCER_PINOUTS_H

// 이 파일은 프로젝트에서 사용하는 모든 하드웨어 핀 번호를
// 한 곳에서 관리하기 위한 헤더 파일입니다.

// =================================================================
// 모터 A 관련 핀 (Motor A Pins)
// =================================================================

// --- SPI1 핀 정의 (모터 A 엔코더용) ---
#define SENSOR_A_SCLK PA5
#define SENSOR_A_MISO PA6
#define SENSOR_A_MOSI PA7
#define SENSOR_A_CS PB5

// --- A모터 드라이버 핀 정의 ---
#define MOTA_A PB10
#define MOTA_B PB6
#define MOTA_C PB4
#define MOTA_EN PC7

// =================================================================
// 모터 B 관련 핀 (Motor B Pins)
// =================================================================

// --- SPI2 핀 정의 (모터 B 엔코더용) ---
#define SENSOR_B_SCLK PB13
#define SENSOR_B_MISO PB14
#define SENSOR_B_MOSI PB15
#define SENSOR_B_CS PB2

// --- B모터 드라이버 핀 정의 ---
#define MOTB_A PA8
#define MOTB_B PA9
#define MOTB_C PA10
#define MOTB_EN PA11

#endif // BALANCER_PINOUTS_H
