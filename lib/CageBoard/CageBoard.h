#ifndef CAGEBOARD_H_
#define CAGEBOARD_H_
#include <Arduino.h>

// BUTTON
#define BUTTON_PIN 12
#define DEBOUNCE_DELAY_TIME 50
// MOTOR
#define RIGHT_MOTOR_PIN 4
#define RIGHT_MOTOR_PWM_PIN 6
#define LEFT_MOTOR_PIN 7
#define LEFT_MOTOR_PWM_PIN 5

enum class MotorDirection : uint8_t {
   BACK,
   FORWARD
};

class CageBoard
{
public:
   CageBoard() = default;

   ~CageBoard() = default;

   void Init();

   void UARTInit(uint16_t baud);

   void ButtonInit();

   void MotorInit();

   void WaitingForButton();

   void MotorMove(uint8_t motorNum, int32_t speed, MotorDirection direction);

   void MotorStop(uint8_t motorNum);

   void Delay(uint64_t delayTime);

   // overloaded methods for print
   void print(const String &message);

   void println(const String &message);

   void print(uint32_t value);

   void println(uint32_t value);

   void print(float value);

   void println(float value);

private:
   uint32_t debounceDelay;


};
#endif
