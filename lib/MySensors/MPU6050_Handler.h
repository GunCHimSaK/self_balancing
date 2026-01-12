#ifndef MPU6050_HANDLER_H
#define MPU6050_HANDLER_H

#include <Arduino.h>

// MPU6050의 Roll, Pitch, Yaw 값을 다른 파일에서도 사용할 수 있도록 extern으로 선언합니다.
extern float roll;
extern float pitch;
extern float yaw; // 주의: 이 값은 자이로스코프만으로 계산되어 시간이 지나면 오차가 누적됩니다(드리프트).

/**
 * @brief MPU6050 센서를 찾아 초기화하고 자이로 바이어스를 보정합니다.
 * setup() 함수에서 한 번만 호출해주세요.
 */
void setupMPU6050();

/**
 * @brief MPU6050 센서 값을 계속해서 읽고 상보 필터를 적용하여 각도를 계산합니다.
 * loop() 함수 안에서 계속 호출해주세요.
 */
void loopMPU6050();

#endif // MPU6050_HANDLER_H
