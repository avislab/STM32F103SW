#ifndef _PMSM_LIB_H_
#define _PMSM_LIB_H_

// PWM Frequency = 72000000/PMSM_CHOPPER_PERIOD/2
//#define PMSM_CHOPPER_PERIOD 9000 // 4 KHz PWM
//#define PMSM_CHOPPER_PERIOD 4500 // 8 KHz PWM
#define PMSM_CHOPPER_PERIOD 2250 // 16 KHz PWM
//#define PMSM_CHOPPER_PERIOD 1125 // 32 KHz PWM
//#define PMSM_CHOPPER_PERIOD 562 // 64 KHz PWM


// Dead time = PMSM_NOL/72000000  (on 72MHz: 7 is 98ns)
// (on 72MHz: 72 is 1000ns)
//#define PMSM_NOL 72
#define PMSM_NOL 72 // 1000ns DeadTime

#define PMSM_ADC_START 200
#define PMSM_ADC_STOP 50
#define PMSM_ADC_MAX 4000

// ==========================
// motor start options
// ==========================
#define PMSM_START_PWM 10 // 10 % // 9 for wheel
#define PMSM_START_PWM_VALUE PMSM_CHOPPER_PERIOD*PMSM_START_PWM/100

#define PMSM_START_SIN_HERTZ 2
#define PMSM_START_SIN_TIMER 36000000/PMSM_SIN_TIMER_PRESCALER/PMSM_SINTABLESIZE/PMSM_START_SIN_HERTZ
// ==========================

#define PMSM_TIMING 30 // in degrees //10
#define PMSM_TIMING_DEFAULT PMSM_TIMING*192/360

#define PMSM_SIN_TIMER_PRESCALER	6
#define PMSM_PRESCALER_K	6
#define PMSM_SPEED_TIMER_PRESCALER	PMSM_SIN_TIMER_PRESCALER * PMSM_SINTABLESIZE / PMSM_PRESCALER_K
#define PMSM_SPEED_TIMER_PERIOD	0xFFFF // 65535

//=============================================================================
#define PMSM_CW		0
#define PMSM_CCW	1

#define ADC_CN_CURRENT 0
#define ADC_CN_SUPPLY 1
#define ADC_CN_POT 2

void PMSM_Init(void);
void PMSM_MotorSetSpin(uint8_t spin);
void PMSM_MotorSetRun(void);
void PMSM_Start(void);
void PMSM_MotorStop(void);
uint8_t PMSM_MotorIsRun(void);
void PMSM_MotorCommutation(uint16_t hallpos);
uint8_t PMSM_HallSensorsGetPosition(void);
uint16_t PMSM_ADCToPWM(uint16_t ADC_VALUE);
void PMSM_SetPWM(uint16_t PWM);
void PMSM_ButtonsInit(void);
uint8_t PMSM_GetButtonRevers(void);
uint8_t PMSM_GetButtonKey1(void);
void PMSM_HallSensorsInit(void);
void PMSM_PWMTimerInit(void);
void PMSM_SinTimerInit(void);
void PMSM_SpeedTimerInit(void);
void PMSM_SetPWM_UVW(uint16_t PWM1, uint16_t PWM2, uint16_t PWM3);
void PMSM_PWM_Update(void);
uint16_t PMSM_GetSpeed(void);
uint8_t PMSM_MotorSpeedIsOK(void);
uint8_t	PMSM_GetNormPos(int16_t position);
uint8_t	PMSM_GetState(uint8_t index);
void PMSM_SetSensorCheckMode(uint8_t value);
uint8_t PMSM_GetSensorCheckMode(void);
uint8_t PMSM_GetSensorPosition(void);
void PMSM_SaveSensorPosition (void);
uint8_t PMSM_GetSinTableIndex(void);

#endif
