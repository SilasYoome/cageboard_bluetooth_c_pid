#include <Arduino.h>
#include <CageBoard.h>
#include <QTRSensors.h>
#define DEBUG 0
const uint8_t SENSOR_VALUE = 5;

CageBoard cage;
QTRSensors qtr;

struct MotorNumber
{
  int LeftMotor = 0;
  int RightMotor = 1;
};

struct PID
{
  float kp = 0.7;
  float ki = 0.0014;
  float kd = 1.5;
  int position = 0;
  int lastError = 0;
  int integral = 0;;
  int derivative = 0;
};

struct Motor
{
public:
  uint16_t MaxSpeed = 100;
  uint16_t MinSpeed = 0;
  uint16_t GoalSpeed = 65;
};

MotorNumber motorNumber;
PID pid;
Motor motor;
uint16_t SensorValue[SENSOR_VALUE];


void pidLineTracking() {
  pid.position = qtr.readLineWhite(SensorValue);
  int32_t error = pid.position - 2000;
  pid.integral += error;
  pid.derivative = error - pid.lastError;
  int32_t motorSpeed = pid.kp * error + pid.ki * pid.integral + pid.kd * pid.derivative;
  pid.lastError = error;

  int32_t rightMotorSpeed = motor.GoalSpeed + motorSpeed;
  int32_t leftMotorSpeed = motor.GoalSpeed - motorSpeed;

  if (rightMotorSpeed > motor.MaxSpeed) { rightMotorSpeed = motor.MaxSpeed; }
  else if (rightMotorSpeed < motor.MinSpeed) { rightMotorSpeed = motor.MinSpeed; }
  if (leftMotorSpeed > motor.MaxSpeed) { leftMotorSpeed = motor.MaxSpeed; }
  else if (leftMotorSpeed < motor.MinSpeed) { leftMotorSpeed = motor.MinSpeed; }

#if DEBUG
  cage.println("position: " + String(pid.position) + " error: " + String(error) + " motorSpeed: " + String(motorSpeed) + " rightMotorSpeed: " + String(rightMotorSpeed) + " leftMotorSpeed: " + String(leftMotorSpeed));
#endif

  cage.MotorMove(motorNumber.RightMotor, rightMotorSpeed, MotorDirection::FORWARD);
  cage.MotorMove(motorNumber.LeftMotor, leftMotorSpeed, MotorDirection::FORWARD);
}


void setup() {
  cage.UARTInit(9600);
  cage.MotorInit();
  cage.ButtonInit();

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) { A1, A2, A3, A4, A5 }, SENSOR_VALUE);

  cage.WaitingForButton();
#if DEBUG
  cage.println("Start");
#endif
  for (int j = 0;j < 1;j++) {
    for (int i = 0; i < 50;i++) {
      if (i < 12 || i >= 37) {
        cage.MotorMove(motorNumber.RightMotor, 50, MotorDirection::FORWARD);
        cage.MotorMove(motorNumber.LeftMotor, 50, MotorDirection::BACK);
      }
      else
      {

        cage.MotorMove(motorNumber.RightMotor, 50, MotorDirection::BACK);
        cage.MotorMove(motorNumber.LeftMotor, 50, MotorDirection::FORWARD);
      }
      qtr.calibrate();
      delay(20);
    }
  }

  qtr.calibrate();
  cage.MotorStop(motorNumber.LeftMotor);
  cage.MotorStop(motorNumber.RightMotor);
  cage.Delay(1000);
}

void loop() {
  qtr.read(SensorValue);
  pidLineTracking();
}


