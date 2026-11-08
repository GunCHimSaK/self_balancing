#include "MPU6050_Handler.h"
#include <Wire.h>
#include <math.h>

// MPU 레지스터
#define REG_WHO_AM_I 0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H 0x43

uint8_t i2c_addr = 0x68;

// 전역 변수
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
unsigned long lastTime = 0;
float bias_gx = 0, bias_gy = 0, bias_gz = 0;

// 필터/변수
const float DEG2RAD = 0.017453292519943295f;
const float ACC_LPF_TAU = 0.02f;
const float COMP_TAU = 0.05f;
float acc_roll_lpf = 0, acc_pitch_lpf = 0;

// 유틸리티 프로토타입
uint8_t i2c_scan_try(uint8_t addr);
uint8_t read_who_am_i(uint8_t addr7);
bool write_reg(uint8_t addr7, uint8_t reg, uint8_t val);
int16_t read16(uint8_t addr7, uint8_t reg);
void mpu_wake_and_config(uint8_t addr7);
void calibrateGyroBias(uint8_t addr7, float &bias_gx, float &bias_gy, float &bias_gz);

bool debugMPU = false;

void setupMPU6050()
{
    Wire.begin();
    Wire.setClock(400000); // 400kHz

    // [수정] setWireTimeout 삭제 (STM32 호환성 문제 해결)
    // 대신 아래 loopMPU6050의 Burst Read가 통신 횟수를 줄여 멈춤을 방지합니다.

    delay(50);

    Serial.println("=== MPU6050 auto-detect and init ===");
    if (i2c_scan_try(0x68) == 0)
    {
        i2c_addr = 0x68;
        Serial.println("Found device at 0x68");
    }
    else if (i2c_scan_try(0x69) == 0)
    {
        i2c_addr = 0x69;
        Serial.println("Found device at 0x69");
    }
    else
    {
        Serial.println("No I2C device found. Halting.");
        while (1)
            delay(1000);
    }

    mpu_wake_and_config(i2c_addr);
    delay(100);

    Serial.println("Calibrating gyro bias... keep still");
    calibrateGyroBias(i2c_addr, bias_gx, bias_gy, bias_gz);
    Serial.println("Calibration done.");

    // 초기값 설정을 위해 한 번 읽기
    int16_t ax = read16(i2c_addr, REG_ACCEL_XOUT_H);
    int16_t ay = read16(i2c_addr, REG_ACCEL_XOUT_H + 2);
    int16_t az = read16(i2c_addr, REG_ACCEL_XOUT_H + 4);
    float ax_g = ax / 16384.0f, ay_g = ay / 16384.0f, az_g = az / 16384.0f;

    acc_roll_lpf = atan2(ay_g, az_g);
    acc_pitch_lpf = atan2(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));
    roll = acc_roll_lpf;
    pitch = acc_pitch_lpf;
    yaw = 0;

    lastTime = micros();
}

void loopMPU6050()
{
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    if (dt <= 0.0f)
        dt = 0.000001f;
    lastTime = now;

    // [핵심] Burst Read (한 번에 14바이트 읽기)
    // 통신 시작/종료 횟수를 6회 -> 1회로 줄여 노이즈 피격 확률을 1/6로 낮춤
    Wire.beginTransmission(i2c_addr);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission() != 0)
    {
        return; // 에러 시 멈추지 않고 리턴 (이전 값 유지)
    }

    Wire.requestFrom(i2c_addr, (uint8_t)14);
    if (Wire.available() < 14)
    {
        return; // 데이터 부족 시 리턴
    }

    // 버퍼에서 순서대로 읽기 (가속도 -> 온도 -> 자이로)
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    Wire.read();
    Wire.read(); // 온도 값 버림
    int16_t gx_raw = (Wire.read() << 8) | Wire.read();
    int16_t gy_raw = (Wire.read() << 8) | Wire.read();
    int16_t gz_raw = (Wire.read() << 8) | Wire.read();

    float ax_g = ax / 16384.0f;
    float ay_g = ay / 16384.0f;
    float az_g = az / 16384.0f;

    float gx_dps = gx_raw / 131.0f - bias_gx;
    float gy_dps = gy_raw / 131.0f - bias_gy;
    float gz_dps = gz_raw / 131.0f - bias_gz;

    // 가속도 각도 계산 (0으로 나누기 방지)
    float acc_roll = 0, acc_pitch = 0;
    // 센서가 0,0,0을 보내는 경우(통신불량) 방어
    if (!((ax == 0) && (ay == 0) && (az == 0)))
    {
        acc_roll = atan2(ay_g, az_g);
        acc_pitch = atan2(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));
    }
    else
    {
        acc_roll = acc_roll_lpf; // 이전 값 유지
        acc_pitch = acc_pitch_lpf;
    }

    // LPF 적용
    float acc_lpf_alpha = ACC_LPF_TAU / (ACC_LPF_TAU + dt);
    acc_roll_lpf = acc_lpf_alpha * acc_roll_lpf + (1.0f - acc_lpf_alpha) * acc_roll;
    acc_pitch_lpf = acc_lpf_alpha * acc_pitch_lpf + (1.0f - acc_lpf_alpha) * acc_pitch;

    float gx_rps = gx_dps * DEG2RAD;
    float gy_rps = gy_dps * DEG2RAD;
    float gz_rps = gz_dps * DEG2RAD;

    // 상보 필터
    float alpha = COMP_TAU / (COMP_TAU + dt);
    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha > 1.0f)
        alpha = 1.0f;

    roll = alpha * (roll + gx_rps * dt) + (1.0f - alpha) * acc_roll_lpf;
    pitch = alpha * (pitch + gy_rps * dt) + (1.0f - alpha) * acc_pitch_lpf;
    yaw += gz_rps * dt;

    if (debugMPU)
    {
        Serial.print("roll:");
        Serial.println(roll, 2);
    }
}

// 유틸리티 함수 (그대로 유지)
uint8_t i2c_scan_try(uint8_t addr)
{
    Wire.beginTransmission(addr);
    return Wire.endTransmission();
}
int16_t read16(uint8_t a, uint8_t r)
{
    Wire.beginTransmission(a);
    Wire.write(r);
    if (Wire.endTransmission() != 0)
        return 0;
    Wire.requestFrom(a, (uint8_t)2);
    if (Wire.available() < 2)
        return 0;
    uint8_t hi = Wire.read(), lo = Wire.read();
    return (int16_t)((hi << 8) | lo);
}
void mpu_wake_and_config(uint8_t a)
{
    write_reg(a, REG_PWR_MGMT_1, 0x00);
    delay(20);
    write_reg(a, REG_SMPLRT_DIV, 0x07);
    write_reg(a, REG_CONFIG, 0x03);
    write_reg(a, REG_GYRO_CONFIG, 0x00);
    write_reg(a, REG_ACCEL_CONFIG, 0x00);
    delay(10);
}
bool write_reg(uint8_t a, uint8_t r, uint8_t v)
{
    Wire.beginTransmission(a);
    Wire.write(r);
    Wire.write(v);
    return Wire.endTransmission() == 0;
}
void calibrateGyroBias(uint8_t a, float &bx, float &by, float &bz)
{
    const int N = 1000;
    long sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < N; i++)
    {
        sx += read16(a, REG_GYRO_XOUT_H);
        sy += read16(a, REG_GYRO_XOUT_H + 2);
        sz += read16(a, REG_GYRO_XOUT_H + 4);
        delay(2);
    }
    bx = (sx / (float)N) / 131.0f;
    by = (sy / (float)N) / 131.0f;
    bz = (sz / (float)N) / 131.0f;
}
uint8_t read_who_am_i(uint8_t a)
{
    Wire.beginTransmission(a);
    Wire.write(REG_WHO_AM_I);
    if (Wire.endTransmission() != 0)
        return 0xFF;
    Wire.requestFrom(a, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}