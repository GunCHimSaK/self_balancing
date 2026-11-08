#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "balancer_pinouts.h"
#include "CustomAS5048A.h"

#define BNO055_ADDR 0x28
// 테스트 때 값이 잘 나왔던 레지스터 주소 (0x1E)
#define REG_ANGLE_TARGET 0x1E

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDR);

const int POLE_PAIRS = 11;

SPIClass spi1(SENSOR_A_MOSI, SENSOR_A_MISO, SENSOR_A_SCLK);
SPIClass spi2(SENSOR_B_MOSI, SENSOR_B_MISO, SENSOR_B_SCLK);

CustomAS5048A sensor1(&spi1, SENSOR_A_CS);
CustomAS5048A sensor2(&spi2, SENSOR_B_CS);

BLDCMotor motor1 = BLDCMotor(POLE_PAIRS);
BLDCMotor motor2 = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(MOTA_A, MOTA_B, MOTA_C, MOTA_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(MOTB_A, MOTB_B, MOTB_C, MOTB_EN);

PIDController pid_stb(30, 100, 1, 100000, 8);
PIDController pid_vel(0.01, 0.03, 0, 10000, PI / 10);

LowPassFilter lpf_pitch_cmd(0.07);
LowPassFilter lpf_throttle(0.5);
LowPassFilter lpf_steering(0.1);

float roll = 0;
float steering = 0;
float throttle = 0;
float max_throttle = 20; // 20 rad/s
float max_steering = 1;  // 1 V
int state = 1;           // 1 on / 0 off
float target_roll = 0.05;

Commander command = Commander(Serial);

// --- Debug variables ---
unsigned long lastPrint = 0;
float lastRoll = 0;
float lastVoltage = 0;

// --- callbacks ---
void onPidStab(char *cmd) { command.pid(&pid_stb, cmd); }
void onPidVel(char *cmd) { command.pid(&pid_vel, cmd); }

unsigned long last_sensor_time = 0;

float readFastRawAngle()
{
    Wire.beginTransmission(BNO055_ADDR);
    Wire.write(REG_ANGLE_TARGET);
    if (Wire.endTransmission() != 0)
        return -999.0f; // 에러 체크

    Wire.requestFrom(BNO055_ADDR, 2);
    if (Wire.available() == 2)
    {
        int16_t rawValue = Wire.read() | (Wire.read() << 8);
        return (float)rawValue / 16.0f; // Degree 단위 반환
    }
    return -999.0f;
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    // setupMPU6050();
    Wire.setSCL(PB8);
    Wire.setSDA(PB9);

    if (!bno.begin())
    {
        Serial.println("BNO055 Fail");
        while (1)
            ;
    }
    bno.setExtCrystalUse(true);

    // 속도 400kHz (테스트에서 성공했으므로 유지)
    Wire.setClock(400000);

    spi1.begin();
    spi2.begin();
    sensor1.init();
    sensor2.init();

    driver1.voltage_power_supply = 10;
    driver2.voltage_power_supply = 10;
    driver1.init();
    driver2.init();

    motor1.linkSensor(&sensor1);
    motor1.linkDriver(&driver1);
    motor2.linkSensor(&sensor2);
    motor2.linkDriver(&driver2);

    motor1.controller = MotionControlType::torque;
    motor2.controller = MotionControlType::torque;

    motor1.init();
    motor2.init();
    motor1.initFOC();
    motor2.initFOC();

    command.add('A', onPidStab, "Stab PID");
    command.add('B', onPidVel, "Vel PID");

    Serial.println(F("=== Balancer Debug Mode ==="));
    Serial.println("Logging: roll, pid_output, voltage_q, motor_enabled");
}

void loop()
{
    motor1.loopFOC();
    motor2.loopFOC();

    motor1.move();
    motor2.move();



    float angle_deg = readFastRawAngle();
    roll = angle_deg * DEG_TO_RAD;
    float target_roll = lpf_pitch_cmd(pid_vel((motor1.shaft_velocity + motor2.shaft_velocity) / 2 - lpf_throttle(throttle))) + 0.045; 
    float voltage_control = -pid_stb(target_roll - roll);
    float steering_adj = lpf_steering(steering);

    motor1.target = voltage_control + steering_adj;
    motor2.target = voltage_control - steering_adj;

  

    command.run();
}
