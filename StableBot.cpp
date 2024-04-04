/*
------------ StableBot ------------

Arduino robot that uses a PID controller to balance itself on two wheels.
Uses a MPU6050 accelerometer and gyroscope to measure the angle of the robot.

With love @AlmartDev :)

*/

#include <Wire.h>
#include <PID_v1.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Fancy debug here
#define DEBUG 1
#define DEBUG_START(x) if(DEBUG) Serial.begin(x)
#define DEBUG_PRINT(x) if(DEBUG) Serial.print(x)
#define DEBUG_PRINTLN(x) if(DEBUG) Serial.println(x)

// Variables
double setpoint = 0;
double input, output;
double Kp = 0.5, Ki = 0.1, Kd = 0.1;

// Objects
Adafruit_MPU6050 mpu;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

byte statusPin = 6; // LED pin

void setup() {
    DEBUG_START(9600);

    pinMode(statusPin, OUTPUT);

    Wire.begin();

    if (!mpu.begin()) {
        DEBUG_PRINT("Failed to initialize MPU6050 sensor!");
        while (1) {}     // Do not continue
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpu.setSampleRate(50);

    // Initialize PID controller
    
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-255, 255);
    pid.SetSampleTime(10);

    // If everything is OK, the led will blink
    digitalWrite(statusPin, HIGH);
    delay(500);
    digitalWrite(statusPin, LOW);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    input = a.acceleration.y;

    pid.Compute();

    DEBUG_PRINT("Input: ");
    DEBUG_PRINT(input);
    DEBUG_PRINT(" Output: ");
    DEBUG_PRINT(output);
    DEBUG_PRINT(" Setpoint: ");
    DEBUG_PRINTLN(setpoint);

    // Move the robot with the output value here
    // TODO: Implement the motor control please

    delay(10);
}