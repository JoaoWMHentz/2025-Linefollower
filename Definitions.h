#ifndef DEFINITIONS.H
#define DEFINITIONS.H

//Motors

#define IN1 22
#define IN2 21
#define IN3 23
#define IN4 25

#define SUC_1 15
#define SUC_2 15

#define FREQ 2600
#define RESULUTION 10
#define PWM_CHN_1 0
#define PWM_CHN_2 1
#define PWM_CHN_3 2
#define PWM_CHN_4 3

//Jsumo Sensor

#define S_OUT 13
#define S_BIT 4
#define S_QTD 16
#define CALIB_RUNS 32
const int S_PINS[4] = { 19, 18, 17, 16 };

const int S_WEIGHT[16] = { -14, -10 ,-7 ,-5 ,-4 ,-3, -2, -1, 1, 2, 3, 4, 5, 7, 10, 14 };

#define THRESHOLD 1400

#define LEFT_SENSOR 35
#define RIGHT_SENSOR 34

//PID

#define SETPOINT 0
#define DELTA_TIME 0.01
#define I_WINDUP_LIMIT 100
#define MAP_MIN_VALUE -1024
#define MAP_MAX_VALUE 1024
#define MAX_PWM 1024

//BLUETOOTH

#define QTDEPARAM 16
#endif

// ENCODER

#define ENCODER_RI_A 32
#define ENCODER_RI_B 33
#define ENCODER_LF_A 26
#define ENCODER_LF_B 27

#define PCNT_H_LIM 10000
#define PCNT_L_LIM -10000

#define PCNT_UNIT_RI PCNT_UNIT_0
#define PCNT_UNIT_LF PCNT_UNIT_1

//  LED

#define LED_R 2
#define LED_G 14
#define LED_B 5



