#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

#include "balancer_pinouts.h"
#include "CustomAS5048A.h"
#include "MPU6050_Handler.h"

const int POLE_PAIRS = 11;

SPIClass spi1(SENSOR_A_MOSI, SENSOR_A_MISO, SENSOR_A_SCLK);
SPIClass spi2(SENSOR_B_MOSI, SENSOR_B_MISO, SENSOR_B_SCLK);

CustomAS5048A sensor1(&spi1, SENSOR_A_CS);
CustomAS5048A sensor2(&spi2, SENSOR_B_CS);

BLDCMotor motor1 = BLDCMotor(POLE_PAIRS);
BLDCMotor motor2 = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(MOTA_A, MOTA_B, MOTA_C, MOTA_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(MOTB_A, MOTB_B, MOTB_C, MOTB_EN);

PIDController pid_stb(17, 0, 0, 100000, 11);
PIDController pid_vel(0.01, 0.03, 0, 10000, PI / 10);

LowPassFilter lpf_pitch_cmd(0.07);
LowPassFilter lpf_throttle(0.5);
LowPassFilter lpf_steering(0.1);

float steering = 0;
float throttle = 0;
float max_throttle = 20; // 20 rad/s
float max_steering = 1;  // 1 V
int state = 1;           // 1 on / 0 off

Commander command = Commander(Serial);

// --- Debug variables ---
unsigned long lastPrint = 0;
float lastRoll = 0;
float lastVoltage = 0;

// --- callbacks ---
void onPidStab(char *cmd) { command.pid(&pid_stb, cmd); }
void onPidVel(char *cmd) { command.pid(&pid_vel, cmd); }

void setup()
{
    Serial.begin(115200);
    delay(500);

    setupMPU6050();
    delay(1000);

    spi1.begin();
    spi2.begin();
    sensor1.init();
    sensor2.init();

    driver1.voltage_power_supply = 12;
    driver2.voltage_power_supply = 12;
    driver1.init();
    driver2.init();

    motor1.linkSensor(&sensor1);
    motor1.linkDriver(&driver1);
    motor2.linkSensor(&sensor2);
    motor2.linkDriver(&driver2);

    motor1.controller = MotionControlType::torque;
    motor2.controller = MotionControlType::torque;

    motor1.voltage_sensor_align = 5;
    motor2.voltage_sensor_align = 5;

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
    command.run();
    loopMPU6050();

    motor1.loopFOC();
    motor2.loopFOC();

    float current_roll = roll;
    float target_roll = 0;

    float voltage_control = pid_stb(target_roll - current_roll);
    float steering_adj = 0;

    motor1.target = voltage_control + steering_adj;
    motor2.target = voltage_control - steering_adj;

    motor1.move();
    motor2.move();

    // === 디버그 출력 (0.1초마다) ===
    if (millis() - lastPrint > 100)
    {
        lastPrint = millis();
        Serial.print("roll:"); Serial.print(current_roll, 2);
        Serial.print(", pid_out:"); Serial.print(voltage_control, 2);
        Serial.print(", mot1_q:"); Serial.print(motor1.voltage.q, 2);
        Serial.print(", mot2_q:"); Serial.print(motor2.voltage.q, 2);
        Serial.print(", enabled:");
        Serial.println(motor1.enabled ? "1" : "0");
    }

    // === 이상 감지 ===
    if (fabs(voltage_control) > 8)
    {
        Serial.println("!!! PID output saturated (>8V) !!!");
    }

    if (isnan(current_roll))
    {
        Serial.println("!!! roll NaN detected (IMU fail) !!!");
    }
}
