/*
------------ StableBot ------------ (CODE NOT TESTED)

MegaPi robot that uses a PID controller to balance itself on two wheels.
With love @AlmartDev :)

*/

#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

// Define MPU6050
MPU6050 mpu;

// Define encoder pins
#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 3
#define RIGHT_ENCODER_A 4
#define RIGHT_ENCODER_B 5

// Define motor pins
#define LEFT_MOTOR_PIN1 6
#define LEFT_MOTOR_PIN2 7
#define RIGHT_MOTOR_PIN1 8
#define RIGHT_MOTOR_PIN2 9

double Kp = 15;
double Ki = 0.1;
double Kd = 0.1;

double setpoint = 0;
double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
int prevLeftEncoderState = 0;
int prevRightEncoderState = 0;

void updateLeftEncoder();
void updateRightEncoder();

void setup()
{
	Wire.begin();
	mpu.initialize();

	pinMode(LEFT_ENCODER_A, INPUT);
	pinMode(LEFT_ENCODER_B, INPUT);
	pinMode(RIGHT_ENCODER_A, INPUT);
	pinMode(RIGHT_ENCODER_B, INPUT);

	attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), updateLeftEncoder, CHANGE);
	attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), updateRightEncoder, CHANGE);

	pinMode(LEFT_MOTOR_PIN1, OUTPUT);
	pinMode(LEFT_MOTOR_PIN2, OUTPUT);
	pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
	pinMode(RIGHT_MOTOR_PIN2, OUTPUT);

	pid.SetMode(AUTOMATIC);
}

void loop()
{
	// Read MPU6050 data
	int16_t ax, ay, az, gx, gy, gz;
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	float gyroAngle = gx / 131.0; // Sensitivity: 131 LSB/degree/s, Using the X-axis

	input = gyroAngle;

	pid.Compute();

	leftMotorSpeed = constrain(output, -255, 255);
	rightMotorSpeed = constrain(output, -255, 255);

	if (leftMotorSpeed > 0)
	{
		digitalWrite(LEFT_MOTOR_PIN1, HIGH);
		digitalWrite(LEFT_MOTOR_PIN2, LOW);
	}
	else
	{
		digitalWrite(LEFT_MOTOR_PIN1, LOW);
		digitalWrite(LEFT_MOTOR_PIN2, HIGH);
	}
	analogWrite(LEFT_MOTOR_SPEED, abs(leftMotorSpeed));

	if (rightMotorSpeed > 0)
	{
		digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
		digitalWrite(RIGHT_MOTOR_PIN2, LOW);
	}
	else
	{
		digitalWrite(RIGHT_MOTOR_PIN1, LOW);
		digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
	}
	analogWrite(RIGHT_MOTOR_SPEED, abs(rightMotorSpeed));
}

void updateLeftEncoder()
{
	int state = digitalRead(LEFT_ENCODER_B);
	if (prevLeftEncoderState == LOW && state == HIGH)
	{
		leftEncoderCount++;
	}
	prevLeftEncoderState = state;
}

void updateRightEncoder()
{
	int state = digitalRead(RIGHT_ENCODER_B);
	if (prevRightEncoderState == LOW && state == HIGH)
	{
		rightEncoderCount++;
	}
	prevRightEncoderState = state;
}