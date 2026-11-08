#include "mpu6050v2.h"
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// 수학 상수 정의
#ifndef _PI_2
#define _PI_2 1.57079632679
#endif
#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))

// MPU6050 객체 생성
MPU6050 mpu;

// MPU 제어/상태 변수
bool imuReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// 방향 계산을 위한 변수들 (전역 또는 static으로 유지해야 함)
VectorFloat gravity; // [x, y, z]            중력 벡터
float ypr[3];        // [yaw, pitch, roll]   각도 저장 배열

// ================================================================
// ===                      함수 구현                           ===
// ================================================================

int initIMU()
{
    Wire.setSCL(PA8);
    Wire.setSDA(PC9);
    Wire.begin();
    Wire.setClock(400000);
    // Wire.setWireTimeout(3000, true); // STM32Duino 구버전이면 주석 처리 유지

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setSleepEnabled(false);
    // 연결 확인
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // DMP 초기화
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        // mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println();
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        mpuIntStatus = mpu.getIntStatus();
        imuReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    delay(2000);
    Serial.println(F("Adjusting DMP sensor fusion gain..."));
    mpu.setMemoryBank(0);
    mpu.setMemoryStartAddress(0x60);
    mpu.writeMemoryByte(0);
    mpu.writeMemoryByte(0x20);
    mpu.writeMemoryByte(0);
    mpu.writeMemoryByte(0);

    return imuReady;
}

int hasDataIMU()
{
    // DMP가 준비되지 않았거나 FIFO 패킷을 못 가져오면 0 반환
    return imuReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
}

float getPitchIMU()
{
    static float pitch;
    Quaternion q;

    // 1. DMP 내부 버퍼에서 쿼터니언 데이터 가져오기
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    // 2. 쿼터니언을 이용해 중력 벡터 계산
    // mpu.dmpGetGravity(&gravity, &q);

    // 3. 쿼터니언과 중력 벡터를 이용해 정확한 Yaw/Pitch/Roll 분리
    //    이 함수가 회전(Yaw)과 기울기(Pitch/Roll)를 섞이지 않게 해줍니다.
    // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // float pitch_new = ypr[1];
    // 4. [축 선택] 로봇의 기울기 축 찾기
    //    ypr[1] : Pitch (일반적으로 앞뒤 기울기)
    //    ypr[2] : Roll  (일반적으로 좌우 기울기, 또는 센서를 돌려 단 경우)
    //
    //    * 중요 * : 아래 코드를 'ypr[1]'로 먼저 테스트해보고,
    //    로봇을 앞뒤로 기울였는데 반응이 없으면 'ypr[2]'로 숫자를 바꿔주세요.

    float pitch_new = atan2(2 * (q.x * q.z - q.w * q.y), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);

    // a bit of debouncing
    if (abs(pitch_new - pitch) > 0.1)
        pitch += _sign(pitch_new - pitch) * 0.01;
    else
        pitch = pitch_new;

    return pitch;
}