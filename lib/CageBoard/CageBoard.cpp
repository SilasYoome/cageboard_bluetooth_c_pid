#include <CageBoard.h>

void CageBoard::Init(){
}

void CageBoard::UARTInit(uint16_t baud){
   Serial.begin(baud);
}

void CageBoard::ButtonInit(){
   pinMode(BUTTON_PIN,INPUT);
   debounceDelay = DEBOUNCE_DELAY_TIME;
}

void CageBoard::MotorInit() {
      pinMode(LEFT_MOTOR_PIN, OUTPUT);
      pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
      pinMode(RIGHT_MOTOR_PIN, OUTPUT);
      pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
}

void CageBoard::WaitingForButton(){
   uint8_t buttonState = HIGH;
   uint8_t lastButtonState = HIGH;
   uint32_t lastDebounceTime = 0;

   while (1)
   {
      uint8_t reading = digitalRead(BUTTON_PIN);

      if(reading != lastButtonState){
         lastDebounceTime = millis();
      }

      if(millis() - lastDebounceTime > debounceDelay){
         
         if(reading != buttonState){
            buttonState = reading;
            
            if(buttonState == LOW){
               break;
            }
         }
      }
      lastButtonState = reading;
   }
   

}

void CageBoard::MotorMove(uint8_t motorNum, int32_t speed, MotorDirection direction){
   uint8_t inPin = HIGH;
   if(direction == MotorDirection::FORWARD){
      inPin = HIGH;
   }
   else if(direction == MotorDirection::BACK){
      inPin = LOW;
   }

   if(motorNum == 0){
      digitalWrite(LEFT_MOTOR_PIN,!inPin);
      analogWrite(LEFT_MOTOR_PWM_PIN,speed);
   }
   else if(motorNum == 1){
      digitalWrite(RIGHT_MOTOR_PIN, inPin);
      analogWrite(RIGHT_MOTOR_PWM_PIN,speed);
   }
}

void CageBoard::MotorStop(uint8_t motorNum){
   if(motorNum == 0){
      analogWrite(LEFT_MOTOR_PWM_PIN,0);
   }
   else if(motorNum == 1){
      analogWrite(RIGHT_MOTOR_PWM_PIN,0);
   }
}

void CageBoard::Delay(uint64_t delayTime){
   delay(delayTime);
}

void CageBoard::print(const String &message){
   Serial.print(message);
}

void CageBoard::println(const String &message){
   Serial.println(message);
}

void CageBoard::print(uint32_t value){
   Serial.print(value);
}

void CageBoard::println(uint32_t value){
   Serial.println(value);
}

void CageBoard::print(float value){
   Serial.print(value);
}

void CageBoard::println(float value){
   Serial.println(value);
}