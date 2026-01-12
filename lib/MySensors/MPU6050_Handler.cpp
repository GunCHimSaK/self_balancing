// MPU6050_Handler.cpp (수정본)
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

// 전역 변수 (헤더에서 extern으로 선언돼있다고 가정)
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
unsigned long lastTime = 0; // 마이크로초 단위로 사용
float bias_gx = 0, bias_gy = 0, bias_gz = 0;

// 필터/변수
const float DEG2RAD = 0.017453292519943295f;
const float ACC_LPF_TAU = 0.02f; // 가속도 LPF 시간상수 (선택)
const float COMP_TAU = 0.05f;    // 상보필터 시간상수 (권장: 0.02~0.1)
float acc_roll_lpf = 0, acc_pitch_lpf = 0;

// 유틸리티 프로토타입
uint8_t i2c_scan_try(uint8_t addr);
uint8_t read_who_am_i(uint8_t addr7);
bool write_reg(uint8_t addr7, uint8_t reg, uint8_t val);
int16_t read16(uint8_t addr7, uint8_t reg);
void mpu_wake_and_config(uint8_t addr7);
void calibrateGyroBias(uint8_t addr7, float &bias_gx, float &bias_gy, float &bias_gz);

// 디버그 옵션
bool debugMPU = false;

void setupMPU6050()
{
    Wire.begin();
    Wire.setClock(400000); // I2C 400kHz -> 더 빠른 샘플링과 안정성
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
        Serial.println("No I2C device found at 0x68/0x69. Halting.");
        while (1)
            delay(1000);
    }

    uint8_t who = read_who_am_i(i2c_addr);
    Serial.print("WHO_AM_I = 0x");
    Serial.println(who, HEX);
    Serial.println("Initialization...");
    mpu_wake_and_config(i2c_addr);
    delay(100);

    Serial.println("Calibrating gyro bias... keep still");
    calibrateGyroBias(i2c_addr, bias_gx, bias_gy, bias_gz);
    Serial.print("Bias gx,gy,gz (dps): ");
    Serial.print(bias_gx);
    Serial.print(", ");
    Serial.print(bias_gy);
    Serial.print(", ");
    Serial.println(bias_gz);
    Serial.println("Calibration done.");

    // 초기 가속도 읽고 각도 초기화
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
    Serial.println("MPU6050 setup complete.");
}

void loopMPU6050()
{
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    if (dt <= 0.0f)
        dt = 0.000001f;
    lastTime = now;

    // raw 읽기
    int16_t ax = read16(i2c_addr, REG_ACCEL_XOUT_H);
    int16_t ay = read16(i2c_addr, REG_ACCEL_XOUT_H + 2);
    int16_t az = read16(i2c_addr, REG_ACCEL_XOUT_H + 4);
    int16_t gx_raw = read16(i2c_addr, REG_GYRO_XOUT_H);
    int16_t gy_raw = read16(i2c_addr, REG_GYRO_XOUT_H + 2);
    int16_t gz_raw = read16(i2c_addr, REG_GYRO_XOUT_H + 4);

    float ax_g = ax / 16384.0f;
    float ay_g = ay / 16384.0f;
    float az_g = az / 16384.0f;

    float gx_dps = gx_raw / 131.0f;
    float gy_dps = gy_raw / 131.0f;
    float gz_dps = gz_raw / 131.0f;

    // bias 보정 (dps)
    gx_dps -= bias_gx;
    gy_dps -= bias_gy;
    gz_dps -= bias_gz;

    // acc 각도 (rad)
    float acc_roll = atan2(ay_g, az_g);
    float acc_pitch = atan2(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));

    // 간단한 가속도 LPF (선택적)
    float acc_lpf_alpha = ACC_LPF_TAU / (ACC_LPF_TAU + dt);
    acc_roll_lpf = acc_lpf_alpha * acc_roll_lpf + (1.0f - acc_lpf_alpha) * acc_roll;
    acc_pitch_lpf = acc_lpf_alpha * acc_pitch_lpf + (1.0f - acc_lpf_alpha) * acc_pitch;
    // 실제로 상보필터에 넣을 acc 값은 LPF된 값 사용
    float acc_roll_use = acc_roll_lpf;
    float acc_pitch_use = acc_pitch_lpf;

    // 자이로를 rad/s 로 변환
    float gx_rps = gx_dps * DEG2RAD;
    float gy_rps = gy_dps * DEG2RAD;
    float gz_rps = gz_dps * DEG2RAD;

    // 동적 α 계산 (tau 기반) -> alpha = tau/(tau+dt)
    float alpha = COMP_TAU / (COMP_TAU + dt);
    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha > 1.0f)
        alpha = 1.0f;

    // 상보필터 (rad 단위)
    roll = alpha * (roll + gx_rps * dt) + (1.0f - alpha) * acc_roll_use;
    pitch = alpha * (pitch + gy_rps * dt) + (1.0f - alpha) * acc_pitch_use;
    yaw += gz_rps * dt;

    // 디버그 출력(원하면 켜서 확인)
    if (debugMPU)
    {
        Serial.print("dt:");
        Serial.print(dt, 6);
        Serial.print(" axg:");
        Serial.print(ax_g, 4);
        Serial.print(" ayg:");
        Serial.print(ay_g, 4);
        Serial.print(" azg:");
        Serial.print(az_g, 4);
        Serial.print(" gx_dps:");
        Serial.print(gx_dps, 3);
        Serial.print(" gy_dps:");
        Serial.print(gy_dps, 3);
        Serial.print(" acc_roll:");
        Serial.print(acc_roll_use, 4);
        Serial.print(" roll:");
        Serial.println(roll, 4);
    }
}

// 유틸 함수 (변경 거의 없음)
uint8_t i2c_scan_try(uint8_t addr)
{
    Wire.beginTransmission(addr);
    return Wire.endTransmission();
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
bool write_reg(uint8_t a, uint8_t r, uint8_t v)
{
    Wire.beginTransmission(a);
    Wire.write(r);
    Wire.write(v);
    return Wire.endTransmission() == 0;
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
    write_reg(a, REG_SMPLRT_DIV, 0x07);   // 필요하면 0로 변경
    write_reg(a, REG_CONFIG, 0x03);       // DLPF 설정: 3은 중간(예: ~44Hz). 필요에 따라 변경
    write_reg(a, REG_GYRO_CONFIG, 0x00);  // +-250dps
    write_reg(a, REG_ACCEL_CONFIG, 0x00); // +-2g
    delay(10);
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
        delay(2); // 전체 샘플 시간: ~2s
    }
    bx = (sx / (float)N) / 131.0f;
    by = (sy / (float)N) / 131.0f;
    bz = (sz / (float)N) / 131.0f;
}
