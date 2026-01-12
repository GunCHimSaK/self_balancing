#ifndef CUSTOM_AS5048A_H
#define CUSTOM_AS5048A_H

#include <SimpleFOC.h>
#include <SPI.h>

/**
 * @class CustomAS5048A
 * @brief AS5048A SPI 엔코더를 저수준 직접 읽기 방식으로 제어하는 커스텀 센서 클래스입니다.
 * SimpleFOC 라이브러리와 함께 사용하기 위해 Sensor 클래스를 상속받습니다.
 */
class CustomAS5048A : public Sensor
{
private:
    SPIClass *spi;
    int cs_pin;

public:
    /**
     * @brief CustomAS5048A 센서 객체의 생성자입니다.
     * @param _spi 사용할 SPIClass 객체의 포인터.
     * @param _cs 이 센서에 연결된 Chip Select(CS) 핀 번호.
     */
    CustomAS5048A(SPIClass *_spi, int _cs);

    /**
     * @brief 센서의 CS핀을 초기화합니다.
     */
    void init();

    /**
     * @brief SimpleFOC 라이브러리가 요구하는 필수 함수입니다.
     * 센서로부터 현재 각도 값을 라디안 단위로 읽어옵니다.
     * @return float 현재 각도 (라디안).
     */
    float getSensorAngle() override;
};

#endif
