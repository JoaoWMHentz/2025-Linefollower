#include "esp32-hal.h"
#ifndef LOCOMOTION.H
#define LOCOMOTION.H

#include <Arduino.h>
#include "Definitions.h"
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <driver/pcnt.h>

class Locomotion
{
public:
	Servo servo;

  int leftEncoderVirtualCount = 0;
  int rightEncoderVirtualCount = 0;

  long brakeTimer = 0;
  uint16_t targetVelocityRight = 0;
  uint16_t targetVelocityLeft = 0;

  int16_t leftEncoderLastCount = 0;
  int16_t rightEncoderLastCount = 0;

  void setupLed(){
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
  }

  void ledControl(bool R, bool G, bool B){
    digitalWrite(LED_R, !R);
    digitalWrite(LED_G, !G);
    digitalWrite(LED_B, !B);
  }

	void begin() {
		pinMode(IN1, OUTPUT);
		pinMode(IN2, OUTPUT);
		pinMode(IN3, OUTPUT);
		pinMode(IN4, OUTPUT);
		ledcSetup(PWM_CHN_1, FREQ, RESULUTION);
		ledcAttachPin(IN1, PWM_CHN_1);
		ledcSetup(PWM_CHN_2, FREQ, RESULUTION);
		ledcAttachPin(IN2, PWM_CHN_2);
		ledcSetup(PWM_CHN_3, FREQ, RESULUTION);
		ledcAttachPin(IN3, PWM_CHN_3);
		ledcSetup(PWM_CHN_4, FREQ, RESULUTION);
		ledcAttachPin(IN4, PWM_CHN_4);
	}

	void setupSuc() {
		ESP32PWM::allocateTimer(3);
		servo.setPeriodHertz(50);
		servo.attach(SUC_1, 1000, 2000);
		servo.attach(SUC_2, 1000, 2000);
		servo.write(900);
		delay(2000);
		servo.write(0);
	}

	void setPotSuc(int pos) {
		servo.write(pos);
	}

	void motorControl(int16_t speedLeft, int16_t speedRight) {
		if (speedLeft < 0) {
			speedLeft = -speedLeft;
			ledcWrite(PWM_CHN_1, speedLeft);
			ledcWrite(PWM_CHN_2, 0);
		}
		else {
			ledcWrite(PWM_CHN_2, speedLeft);
			ledcWrite(PWM_CHN_1, 0);
		}

		if (speedRight < 0) {
			speedRight = -speedRight;
			ledcWrite(PWM_CHN_3, speedRight);
			ledcWrite(PWM_CHN_4, 0);
		}
		else {
			ledcWrite(PWM_CHN_4, speedRight);
			ledcWrite(PWM_CHN_3, 0);
		}
    targetVelocityRight = speedRight < 0 ? -speedRight : speedRight; 
    targetVelocityLeft = speedLeft < 0 ? -speedLeft : speedLeft; 
	}

	void brake() {
    if((millis() - brakeTimer) > 100){
      if(targetVelocityLeft > 0 || targetVelocityRight > 0){
        motorControl(targetVelocityLeft - 20, targetVelocityRight - 20);
      }else{
        motorControl(0,0);
      }
    }
	}

  void setupEncoder(){
    setupPCNT(PCNT_UNIT_RI, ENCODER_RI_A, ENCODER_RI_B);
    setupPCNT(PCNT_UNIT_LF, ENCODER_LF_A, ENCODER_LF_B);
  }

  int32_t readLeftEncoder() {
      int16_t currentCount;
      pcnt_get_counter_value(PCNT_UNIT_LF, &currentCount);

      if (currentCount < leftEncoderLastCount && (leftEncoderLastCount - currentCount) > (PCNT_H_LIM / 2)) {
          leftEncoderVirtualCount += (PCNT_H_LIM - leftEncoderLastCount + currentCount + 1);
      } else if (currentCount > leftEncoderLastCount && (currentCount - leftEncoderLastCount) > (PCNT_H_LIM / 2)) {
          leftEncoderVirtualCount -= (PCNT_H_LIM - currentCount + leftEncoderLastCount + 1);
      } else {
          leftEncoderVirtualCount += (currentCount - leftEncoderLastCount);
      }

      leftEncoderLastCount = currentCount; 
      return leftEncoderVirtualCount;
  }

  int32_t readRightEncoder() {
      int16_t currentCount;
      pcnt_get_counter_value(PCNT_UNIT_RI, &currentCount);
      if (currentCount < rightEncoderLastCount && (rightEncoderLastCount - currentCount) > (PCNT_H_LIM / 2)) {
          rightEncoderVirtualCount += (PCNT_H_LIM - rightEncoderLastCount + currentCount + 1);
      } else if (currentCount > rightEncoderLastCount && (currentCount - rightEncoderLastCount) > (PCNT_H_LIM / 2)) {
          rightEncoderVirtualCount -= (PCNT_H_LIM - currentCount + rightEncoderLastCount + 1);
      } else {
          rightEncoderVirtualCount += (currentCount - rightEncoderLastCount);
      }

      rightEncoderLastCount = currentCount; 
      return rightEncoderVirtualCount;
  }

private:
  void setupPCNT(pcnt_unit_t unit, uint8_t pinA, uint8_t pinB) {
      
      pcnt_config_t pcnt_config = {};
      pcnt_config.pulse_gpio_num = pinA;    
      pcnt_config.ctrl_gpio_num = pinB;     
      pcnt_config.channel = PCNT_CHANNEL_0;
      pcnt_config.unit = unit;
      pcnt_config.pos_mode = PCNT_COUNT_INC;   
      pcnt_config.neg_mode = PCNT_COUNT_DEC;   
      pcnt_config.lctrl_mode = PCNT_MODE_KEEP; 
      pcnt_config.hctrl_mode = PCNT_MODE_REVERSE; 
      pcnt_config.counter_h_lim = PCNT_H_LIM; 
      pcnt_config.counter_l_lim = PCNT_L_LIM; 
      pcnt_unit_config(&pcnt_config);
      pcnt_set_filter_value(unit, 100);
      pcnt_filter_enable(unit);
      pcnt_counter_pause(unit);
      pcnt_counter_clear(unit);
      pcnt_counter_resume(unit);
  }
};
#endif
