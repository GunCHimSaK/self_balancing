#ifndef MPU6050_V2_H
#define MPU6050_V2_H

#include <Arduino.h>

/**
 * @brief IMU(DMP) 초기화 함수
 * @return 1: 성공, 0: 실패
 */
int initIMU();

/**
 * @brief 새로운 데이터가 도착했는지 확인하는 함수
 * @return 1: 데이터 있음, 0: 없음
 */
int hasDataIMU();

/**
 * @brief 현재 Pitch 각도를 읽어오는 함수
 * @return float Pitch 각도 (라디안)
 */
float getPitchIMU();

#endif