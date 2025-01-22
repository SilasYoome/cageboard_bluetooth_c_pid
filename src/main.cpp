#include <Arduino.h>
#include <CageBoard.h>
#include <QTRSensors.h>
#define DEBUG 0
const uint8_t SENSOR_VALUE = 5;

CageBoard cage;
QTRSensors qtr;

enum MESSAGE_ID {
  // PID
  SET_PID_STATUS = 'F', // pid flag
  KP_INCREASE = 'P',    // kp++
  KP_DECREASE = 'p',    // kp--
  KI_INCREASE = 'I',    // ki++
  KI_DECREASE = 'i',    // ki--
  KD_INCREASE = 'D',    // kd++
  KD_DECREASE = 'd',     // kd--

  //  MOTOR
  MAX_SPEED_INCREASE = 'M', // max speed++
  MAX_SPEED_DECREASE = 'm', // max speed--
  GOAL_SPEED_INCREASE = 'G', // goal speed++
  GOAL_SPEED_DECREASE = 'g' // goal speed--
};

struct MotorNumber
{
  int LeftMotor = 0;
  int RightMotor = 1;
};

struct PID
{
  bool isLineTracking = false;
  float kp = 0.10;
  float ki = 0.00;
  float kd = 0.15;
  int position = 0;
  int lastError = 0;
  int integral = 0;;
  int derivative = 0;
};

struct Motor
{
public:
  uint16_t MaxSpeed = 200;
  uint16_t MinSpeed = 0;
  uint16_t BaseSpeed = 140;

  int32_t RightMotorSpeed = 0;
  int32_t LeftMotorSpeed = 0;
};

struct UartParameter
{
  String label;
  String value;
};

MESSAGE_ID msg;
MotorNumber motorNumber;
PID pid;
Motor motor;
uint16_t SensorValue[SENSOR_VALUE];


void SerialEvent() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == SET_PID_STATUS)
      pid.isLineTracking = !pid.isLineTracking;
    if (pid.isLineTracking == false) {
      switch (c)
      {
        /**************PID**************/
      case KP_INCREASE: //  kp++
        pid.kp += 0.01;
        break;
      case KP_DECREASE: //  kp--
        pid.kp -= 0.01;
        break;

      case KI_INCREASE:
        /* code */
        break;

      case KI_DECREASE:
        /* code */
        break;

      case KD_INCREASE: //  kd++
        pid.kd += 0.01;
        break;

      case KD_DECREASE: //  kd--
        pid.kd -= 0.01;
        break;

        /**************MOTOR**************/

      case MAX_SPEED_INCREASE: //  max speed++
        motor.MaxSpeed += 1;
        break;

      case MAX_SPEED_DECREASE: //  max speed--
        motor.MaxSpeed -= 1;
        break;

      case GOAL_SPEED_INCREASE: //  goal speed++
        motor.BaseSpeed += 1;
        break;

      case GOAL_SPEED_DECREASE: //  goal speed--
        motor.BaseSpeed -= 1;
        break;

      default:
        break;
      }
    }
  }

}

void uartSend() {
  if (pid.isLineTracking) {
    UartParameter params[] = { 
      "*S", String(pid.position),
      "*R", String(motor.RightMotorSpeed),
      "*L", String(motor.LeftMotorSpeed)
      };
    for (size_t i = 0; i < sizeof(params) / sizeof(params[0]); i++) {
      cage.print(params[i].label + params[i].value + "*");
    }
  }
  else {
    UartParameter params[] = {
    {"*P", String(pid.kp)},
    {"*I", String(pid.ki)},
    {"*D", String(pid.kd)},
    {"*M", String(motor.MaxSpeed)},
    {"*G", String(motor.BaseSpeed)}
    };

    for (size_t i = 0; i < sizeof(params) / sizeof(params[0]); i++) {
      cage.print(params[i].label + params[i].value + "*");
    }
  }

}

void pidLineTracking(int goalspeed) {
  pid.position = qtr.readLineWhite(SensorValue);
  int32_t error = pid.position - 2000;
  pid.integral += error;
  pid.derivative = error - pid.lastError;
  int32_t motorSpeed = pid.kp * error + pid.ki * pid.integral + pid.kd * pid.derivative;
  pid.lastError = error;

  motor.RightMotorSpeed = goalspeed + motorSpeed;
  motor.LeftMotorSpeed = goalspeed - motorSpeed;

  if (motor.RightMotorSpeed > motor.MaxSpeed) { motor.RightMotorSpeed = motor.MaxSpeed; }
  else if (motor.RightMotorSpeed < motor.MinSpeed) { motor.RightMotorSpeed = motor.MinSpeed; }
  if (motor.LeftMotorSpeed > motor.MaxSpeed) { motor.LeftMotorSpeed = motor.MaxSpeed; }
  else if (motor.LeftMotorSpeed < motor.MinSpeed) { motor.LeftMotorSpeed = motor.MinSpeed; }

#if DEBUG
  cage.println("position: " + String(pid.position) + " error: " + String(error) + " motorSpeed: " + String(motorSpeed) + " motor.RightMotorSpeed: " + String(motor.RightMotorSpeed) + " motor.LeftMotorSpeed: " + String(motor.LeftMotorSpeed));
#endif

  cage.MotorMove(motorNumber.RightMotor, motor.RightMotorSpeed, MotorDirection::FORWARD);
  cage.MotorMove(motorNumber.LeftMotor, motor.LeftMotorSpeed, MotorDirection::FORWARD);
}

void motorStop() {
  cage.MotorStop(motorNumber.LeftMotor);
  cage.MotorStop(motorNumber.RightMotor);
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
  uartSend();
  qtr.read(SensorValue);
  if (pid.isLineTracking == true) {
    pidLineTracking(motor.BaseSpeed);
  }
  else {
    motorStop();
  }
  SerialEvent();
}


