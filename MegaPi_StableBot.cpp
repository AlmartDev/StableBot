/*
------------ StableBot ------------ (CODE NEEDS VALUE TUNING!)

MegaPi (MeAuriga V1.3) robot that uses a PID controller to balance itself on two wheels.
With love @AlmartDev :)

*/

#include <MeAuriga.h>

#include <Wire.h>
#include <PID_v1.h>

double setpoint = 0;
double input, output;

double Kp = 0.5;
double Ki = 0.1;
double Kd = 0.1;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

Gyro gyro(0, 0x69);

void isr_process_encoder1(void)
{
	if (digitalRead(Encoder_1.getPortB()) == 0)
	{
		Encoder_1.pulsePosMinus();
	}
	else
	{
		Encoder_1.pulsePosPlus();
	}
}

void isr_process_encoder2(void)
{
	if (digitalRead(Encoder_2.getPortB()) == 0)
	{
		Encoder_2.pulsePosMinus();
	}
	else
	{
		Encoder_2.pulsePosPlus();
	}
}

void setup()
{
	Serial.begin(9600);
	gyro.begin();

	attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
	attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
	Serial.begin(115200);

	// Initialize PID controller (might have to change the values)
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-255, 255);
    pid.SetSampleTime(10);

	// Set PWM 8KHz
	TCCR1A = _BV(WGM10);
	TCCR1B = _BV(CS11) | _BV(WGM12);

	TCCR2A = _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(CS21);
}

void setSpeed(int speed)
{
	if (speed > 0)	// IMPORTANT: Motors are inverted with each other!
	{
		Encoder_1.setMotorPwm(speed);
		Encoder_2.setMotorPwm(-speed);
	}
	else
	{
		Encoder_1.setMotorPwm(-speed);
		Encoder_2.setMotorPwm(speed);
	}

	Encoder_1.updateSpeed();
	Encoder_2.updateSpeed();
}

void loop()
{
	gyro.update();
  	Serial.read();

	input = gyro.getAngleX(); // Using the X-axis to balance the robot

	pid.Compute();

	setSpeed(output);

	delay(10);
}