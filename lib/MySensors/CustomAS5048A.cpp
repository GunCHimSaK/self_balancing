#include "CustomAS5048A.h"
#include <Arduino.h>

// 생성자
CustomAS5048A::CustomAS5048A(SPIClass *_spi, int _cs)
{
    this->spi = _spi;
    this->cs_pin = _cs;
}

void CustomAS5048A::init()
{
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
}

float CustomAS5048A::getSensorAngle()
{
    spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    digitalWrite(cs_pin, LOW);
    uint16_t raw_data = spi->transfer16(0xFFFF);
    digitalWrite(cs_pin, HIGH);
    spi->endTransaction();

    uint16_t angle_data = raw_data & 0x3FFF;
    float angle_rad = (float)angle_data / 16383.0f * TWO_PI;
    return angle_rad;
}
