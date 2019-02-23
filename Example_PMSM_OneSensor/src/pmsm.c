#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h"
#include "misc.h"
#include "string.h"
#include "stdio.h"

#include "pmsm.h"
#include "adc_dma.h"
#include "systickdelay.h"

// Variables
volatile uint8_t PMSM_MotorRunFlag = 0;
volatile uint8_t PMSM_StararPositionFlag = 1;
volatile uint8_t PMSM_SensorCheckMode = 0;
volatile uint8_t PMSM_SensorPosition = 0xFF;
volatile uint16_t PMSM_SensorPositionSum = 0;
volatile uint8_t PMSM_SensorPositionCount = 0;
volatile uint8_t PMSM_MotorSpin = PMSM_CW;
volatile uint8_t PMSM_SinTableIndex = 0;
volatile uint8_t PMSM_SinTableLimit = 0;
volatile uint16_t PMSM_PWM = 0;
volatile uint16_t PMSM_Speed = 0;
volatile uint16_t PMSM_Speed_prev = 0;
volatile static int8_t PMSM_Timing = PMSM_TIMING_DEFAULT;

// Sin table
#define PMSM_SINTABLESIZE	192
static const uint8_t PMSM_SINTABLE [PMSM_SINTABLESIZE][3] =
{
		{0,       0,      221},
		{8,       0,      225},
		{17,      0,      229},
		{25,      0,      232},
		{33,      0,      236},
		{42,      0,      239},
		{50,      0,      241},
		{58,      0,      244},
		{66,      0,      246},
		{74,      0,      248},
		{82,      0,      250},
		{90,      0,      252},
		{98,      0,      253},
		{105,     0,      254},
		{113,     0,      254},
		{120,     0,      255},
		{128,     0,      255},
		{135,     0,      255},
		{142,     0,      254},
		{149,     0,      254},
		{155,     0,      253},
		{162,     0,      252},
		{168,     0,      250},
		{174,     0,      248},
		{180,     0,      246},
		{186,     0,      244},
		{192,     0,      241},
		{197,     0,      239},
		{202,     0,      236},
		{207,     0,      232},
		{212,     0,      229},
		{217,     0,      225},
		{221,     0,      221},
		{225,     0,      217},
		{229,     0,      212},
		{232,     0,      207},
		{236,     0,      202},
		{239,     0,      197},
		{241,     0,      192},
		{244,     0,      186},
		{246,     0,      180},
		{248,     0,      174},
		{250,     0,      168},
		{252,     0,      162},
		{253,     0,      155},
		{254,     0,      149},
		{254,     0,      142},
		{255,     0,      135},
		{255,     0,      127},
		{255,     0,      120},
		{254,     0,      113},
		{254,     0,      105},
		{253,     0,      98},
		{252,     0,      90},
		{250,     0,      82},
		{248,     0,      74},
		{246,     0,      66},
		{244,     0,      58},
		{241,     0,      50},
		{239,     0,      42},
		{236,     0,      33},
		{232,     0,      25},
		{229,     0,      17},
		{225,     0,      8},
		{221,     0,      0},
		{225,     8,      0},
		{229,     17,     0},
		{232,     25,     0},
		{236,     33,     0},
		{239,     42,     0},
		{241,     50,     0},
		{244,     58,     0},
		{246,     66,     0},
		{248,     74,     0},
		{250,     82,     0},
		{252,     90,     0},
		{253,     98,     0},
		{254,     105,    0},
		{254,     113,    0},
		{255,     120,    0},
		{255,     127,    0},
		{255,     135,    0},
		{254,     142,    0},
		{254,     149,    0},
		{253,     155,    0},
		{252,     162,    0},
		{250,     168,    0},
		{248,     174,    0},
		{246,     180,    0},
		{244,     186,    0},
		{241,     192,    0},
		{239,     197,    0},
		{236,     202,    0},
		{232,     207,    0},
		{229,     212,    0},
		{225,     217,    0},
		{221,     221,    0},
		{217,     225,    0},
		{212,     229,    0},
		{207,     232,    0},
		{202,     236,    0},
		{197,     239,    0},
		{192,     241,    0},
		{186,     244,    0},
		{180,     246,    0},
		{174,     248,    0},
		{168,     250,    0},
		{162,     252,    0},
		{155,     253,    0},
		{149,     254,    0},
		{142,     254,    0},
		{135,     255,    0},
		{128,     255,    0},
		{120,     255,    0},
		{113,     254,    0},
		{105,     254,    0},
		{98,      253,    0},
		{90,      252,    0},
		{82,      250,    0},
		{74,      248,    0},
		{66,      246,    0},
		{58,      244,    0},
		{50,      241,    0},
		{42,      239,    0},
		{33,      236,    0},
		{25,      232,    0},
		{17,      229,    0},
		{8,       225,    0},
		{0,       221,    0},
		{0,       225,    8},
		{0,       229,    17},
		{0,       232,    25},
		{0,       236,    33},
		{0,       239,    42},
		{0,       241,    50},
		{0,       244,    58},
		{0,       246,    66},
		{0,       248,    74},
		{0,       250,    82},
		{0,       252,    90},
		{0,       253,    98},
		{0,       254,    105},
		{0,       254,    113},
		{0,       255,    120},
		{0,       255,    128},
		{0,       255,    135},
		{0,       254,    142},
		{0,       254,    149},
		{0,       253,    155},
		{0,       252,    162},
		{0,       250,    168},
		{0,       248,    174},
		{0,       246,    180},
		{0,       244,    186},
		{0,       241,    192},
		{0,       239,    197},
		{0,       236,    202},
		{0,       232,    207},
		{0,       229,    212},
		{0,       225,    217},
		{0,       221,    221},
		{0,       217,    225},
		{0,       212,    229},
		{0,       207,    232},
		{0,       202,    236},
		{0,       197,    239},
		{0,       192,    241},
		{0,       186,    244},
		{0,       180,    246},
		{0,       174,    248},
		{0,       168,    250},
		{0,       162,    252},
		{0,       155,    253},
		{0,       149,    254},
		{0,       142,    254},
		{0,       135,    255},
		{0,       128,    255},
		{0,       120,    255},
		{0,       113,    254},
		{0,       105,    254},
		{0,       98,     253},
		{0,       90,     252},
		{0,       82,     250},
		{0,       74,     248},
		{0,       66,     246},
		{0,       58,     244},
		{0,       50,     241},
		{0,       42,     239},
		{0,       33,     236},
		{0,       25,     232},
		{0,       17,     229},
		{0,       8,      225}
};

// Phase correction table
volatile uint8_t PMSM_STATE_TABLE_INDEX[2][2] = { {0, 0}, {0, 0} };

#define UH	0
#define UL	1
#define VH	2
#define VL	3
#define WH	4
#define WL	5

#define MY_FLASH_PAGE_ADDR 0x800FC00
#define SETTINGS_WORDS 1

void FLASH_Init(void) {
    /* Next commands may be used in SysClock initialization function
       In this case using of FLASH_Init is not obligatorily */
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);
    /* Flash 2 wait state */
    FLASH_SetLatency( FLASH_Latency_2);
}

void FLASH_ReadSettings(void) {
    //Read settings
    uint32_t *source_addr = (uint32_t *)MY_FLASH_PAGE_ADDR;
    uint32_t *dest_addr = (void *)&PMSM_STATE_TABLE_INDEX;
    for (uint16_t i=0; i<SETTINGS_WORDS; i++) {
        *dest_addr = *(__IO uint32_t*)source_addr;
        source_addr++;
        dest_addr++;
    }
}

void FLASH_WriteSettings(void) {
    FLASH_Unlock();
    FLASH_ErasePage(MY_FLASH_PAGE_ADDR);

    // Write settings
    uint32_t *source_addr = (void *)&PMSM_STATE_TABLE_INDEX;
    uint32_t *dest_addr = (uint32_t *) MY_FLASH_PAGE_ADDR;
    for (uint16_t i=0; i<SETTINGS_WORDS; i++) {
        FLASH_ProgramWord((uint32_t)dest_addr, *source_addr);
        source_addr++;
        dest_addr++;
    }

    FLASH_Lock();
}

// Initialize of all needed peripheral
void PMSM_Init(void) {
	PMSM_HallSensorsInit();
	PMSM_PWMTimerInit();
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	PMSM_SinTimerInit();
	PMSM_SpeedTimerInit();
	PMSM_ButtonsInit();
	PMSM_MotorStop();
	FLASH_ReadSettings();
}

//Start motor
void PMSM_Start(void) {
	//uint16_t arr;
	// Turn PWM outputs for working with sine wave
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

	// Set To Start Position
	PMSM_StararPositionFlag = 1;
	PMSM_SinTableIndex = 0;
	PMSM_SetPWM(PMSM_START_PWM_VALUE);
	PMSM_PWM_Update();
	sysTickDalay(PMSM_START_DELAY);

	// Start sine
	TIM4->ARR = PMSM_START_SIN_TIMER;
	TIM_Cmd(TIM4, ENABLE);

	///// This line for very slow engines
	//sysTickDalay(PMSM_START_DELAY);
	/////

	PMSM_StararPositionFlag = 0;
	PMSM_Speed = 0;
	PMSM_Speed_prev = 0;
	PMSM_MotorSetRun();

	TIM_Cmd(TIM3, ENABLE);
	TIM_SetCounter(TIM3, 0);
}


// Stop a motor
void PMSM_MotorStop(void)
{
	PMSM_SetPWM(0);

	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);

	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	TIM_Cmd(TIM3, DISABLE);
	TIM_Cmd(TIM4, DISABLE);
	PMSM_Speed = 0;
	PMSM_Speed_prev = 0;
	PMSM_MotorRunFlag = 0;
	PMSM_StararPositionFlag = 1;
}

void PMSM_ButtonsInit(void) {
	// Reverse Key Init
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_0);

	// Key 1 Init
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_15);
}

uint8_t PMSM_GetButtonRevers(void) {
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);

}

uint8_t PMSM_GetButtonKey1(void) {
	return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15);
}

// Configure GPIO, NVIC, EXTI for 3 Hall sensors
void PMSM_HallSensorsInit(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	// Enable clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	// Init GPIO
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU; //GPIO_Mode_IN_FLOATING
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Init NVIC
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

    // Tell system that you will use PB lines for EXTI
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);

    // EXTI
    EXTI_InitStruct.EXTI_Line = EXTI_Line8;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_Init(&EXTI_InitStruct);
}

// Every time when hall sensors change state executed this IRQ handler
void EXTI9_5_IRQHandler(void) {
	uint16_t tmp;

	if ( EXTI_GetITStatus(EXTI_Line8) != RESET) {
    	// Clear interrupt flag
    	EXTI_ClearITPendingBit(EXTI_Line8);

    	////////////////////////////////////////////////////
    	// FILTERS
    	////////////////////////////////////////////////////
    	// So small time - noise in fronts
		tmp = TIM_GetCounter(TIM3);
		if (tmp < 10) {
			return;
		}

		// Speed raise so fast (unreal fast) - sensor noise
		//if (tmp < PMSM_Speed - PMSM_Speed/4) {
		//	return;
		//}

		// Interrupt is happened but sensor state hasn't changed.
		// "Double interrupt" bug
		if (PMSM_SensorPosition == PMSM_HallSensorsGetPosition()) {
			return;
		}
		////////////////////////////////////////////////////

		////////////////////////////////////////////////////
		// Settings in start position
		////////////////////////////////////////////////////
    	if (PMSM_StararPositionFlag == 1) {
    		return;
    	}
    	////////////////////////////////////////////////////

    	////////////////////////////////////////////////////
    	// Sensor check mode
    	////////////////////////////////////////////////////
		if (PMSM_SensorCheckMode == 1) {
    		if (PMSM_HallSensorsGetPosition() == 1) {
    			if (PMSM_SensorPositionCount < 5) {
   					PMSM_SensorPositionSum += PMSM_SinTableIndex;
    				PMSM_SensorPositionCount += 1;
    			}
    			else {
    				PMSM_SensorPosition = PMSM_SensorPositionSum / PMSM_SensorPositionCount;
    				PMSM_SaveSensorPosition();
            		PMSM_SetSensorCheckMode(0);
    			}
    		}
    		return;
		}
		////////////////////////////////////////////////////

		PMSM_SensorPosition = PMSM_HallSensorsGetPosition();
   		PMSM_Speed_prev = PMSM_Speed;
   		PMSM_Speed = tmp;

   		//TIM_Cmd(TIM3, ENABLE);
   		TIM_SetCounter(TIM3, 0);

   		// It requires at least two measurement to correct calculate the rotor speed
   		if (PMSM_MotorSpeedIsOK()) {
   			TIM_SetCounter(TIM4, 0);
   			TIM4->ARR = PMSM_Speed;
   		}

   		// Phase correction
   		PMSM_SinTableIndex = PMSM_GetState(PMSM_HallSensorsGetPosition());
   		PMSM_SinTableLimit = 0;
   		PMSM_PWM_Update();
    }
}

void TIM1_UP_IRQHandler(void) {
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		if (TIM1->CNT < PMSM_CHOPPER_PERIOD / 2 ) {
			// Start ADC Conversion
			ADC_SoftwareStartConvCmd( ADC1 , ENABLE );
		}
	}
}

// Initialize Timer TIM1 & PWM output. Timer TIM1 generate 6-PWM outputs
void PMSM_PWMTimerInit(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	// Initialize Tim1 PWM outputs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	//TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1; // Center Aligned PWM
	TIM_TimeBaseStructure.TIM_Period = PMSM_CHOPPER_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2, 3 set to PWM mode - all 6 outputs
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; // initialize to zero output

	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; ///
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset; ///

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;

	// DeadTime[ns] = value * (1/SystemCoreFreq) (on 72MHz: 7 is 98ns)
	TIM_BDTRInitStructure.TIM_DeadTime = PMSM_NOL;

	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

	// Break functionality
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

	// NVIC Configuration
	// Enable the TIM1_IRQn Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM1, ENABLE);
	 // Enable motor timer main output (the bridge signals)
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

// Initialize TIM4 which generate 3-phase sine signal
void PMSM_SinTimerInit(void) {
	TIM_TimeBaseInitTypeDef TIMER_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseStructInit(&TIMER_InitStructure);
	TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIMER_InitStructure.TIM_Prescaler = PMSM_SIN_TIMER_PRESCALER;//PMSM_SPEED_TIMER_PRESCALER;
	TIMER_InitStructure.TIM_Period = 0;
	TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	//TIM_Cmd(TIM4, ENABLE);

	// NVIC Configuration
	// Enable the TIM4_IRQn Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// Calculate 3-phase PWM and increment position in sine table
void TIM4_IRQHandler(void) {
	//uint16_t PWM1, PWM2, PWM3;

	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		PMSM_PWM_Update();
	}
}

// Initialize TIM3. It used to calculate the speed
void PMSM_SpeedTimerInit(void) {
	TIM_TimeBaseInitTypeDef TIMER_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructInit(&TIMER_InitStructure);
	TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIMER_InitStructure.TIM_Prescaler = PMSM_SPEED_TIMER_PRESCALER;
	TIMER_InitStructure.TIM_Period = PMSM_SPEED_TIMER_PERIOD;
	TIM_TimeBaseInit(TIM3, &TIMER_InitStructure);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM3, 0);
	//TIM_Cmd(TIM3, ENABLE);

	// NVIC Configuration
	// Enable the TIM3_IRQn Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM3_IRQHandler(void) {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		// Overflow - the motor is stopped
		if (PMSM_MotorSpeedIsOK()) {
			PMSM_MotorStop();
		}
	}
}

// Get data from hall sensors
uint8_t PMSM_HallSensorsGetPosition(void) {
	return (uint8_t) ((GPIO_ReadInputData(GPIOB) & GPIO_Pin_8) >> 8);
}

// Transform ADC value to value for writing to the timer register
uint16_t PMSM_ADCToPWM(uint16_t ADC_VALUE) {
	uint32_t tmp;

	if (ADC_VALUE < PMSM_ADC_STOP) {
		return 0;
	} else {
		if (ADC_VALUE > PMSM_ADC_MAX) {
			return PMSM_CHOPPER_PERIOD+1;
		}
		else {
			tmp = (uint32_t)(ADC_VALUE-PMSM_ADC_STOP) * (uint32_t)PMSM_CHOPPER_PERIOD / (uint32_t)(PMSM_ADC_MAX - PMSM_ADC_START);
			return (uint16_t) tmp;
		}
	}
}

// Set PWM (same for all phases)
void PMSM_SetPWM(uint16_t PWM)
{
	if (PMSM_MotorSpeedIsOK()) {
		PMSM_PWM = PWM;
	}
	else {
		PMSM_PWM = PMSM_START_PWM_VALUE;
	}
}

// Set PWM
void PMSM_SetPWM_UVW(uint16_t PWM1, uint16_t PWM2, uint16_t PWM3)
{
	TIM1->CCR1 = PWM1;
	TIM1->CCR2 = PWM2;
	TIM1->CCR3 = PWM3;
}

void PMSM_PWM_Update(void) {
	uint16_t PWM1, PWM2, PWM3;

	// Calculate PWM for 3-phase
	PWM1 = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][0]/255);
	PWM2 = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][1]/255);
	PWM3 = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][2]/255);

	if (PMSM_MotorSpin == PMSM_CW) {
		// Forward rotation
		PMSM_SetPWM_UVW(PWM1, PWM2, PWM3);
	}
	else {
		// Backward rotation
		PMSM_SetPWM_UVW(PWM1, PWM3, PWM2);
	}


	// If engine is running
	// PMSM_SinTableLimit don't allow to increment the PMSM_SinTableIndex
	// when PMSM_SinTableIndex already reached next sensor position
	// but sensor not changed yet.
	if (PMSM_Speed > 0) {
		if (PMSM_SinTableLimit < PMSM_SINTABLESIZE) {
			PMSM_SinTableLimit++;
		}
	}

	// Increment position in sine table
	if ( PMSM_SinTableLimit < PMSM_SINTABLESIZE/2 ) {
		PMSM_SinTableIndex++;
		if (PMSM_SinTableIndex > PMSM_SINTABLESIZE-1) {
			PMSM_SinTableIndex = 0;
		}
	}
}

uint8_t	PMSM_GetNormPos(int16_t position) {
	if (position > PMSM_SINTABLESIZE-1) {
		position = position - PMSM_SINTABLESIZE;
	}
	else {
		if (position < 0) {
			position = PMSM_SINTABLESIZE + position;
		}
	}
	return (uint8_t) position;
}

// Get index in sine table based on the sensor data, the timing and the direction of rotor rotation
uint8_t	PMSM_GetState(uint8_t SensorsPosition) {
	int16_t index;
	index = PMSM_STATE_TABLE_INDEX[PMSM_MotorSpin][SensorsPosition] + (int16_t)PMSM_Timing;

	return PMSM_GetNormPos(index);
}

uint8_t PMSM_MotorIsRun(void) {
	return PMSM_MotorRunFlag;
}

uint8_t PMSM_MotorSpeedIsOK(void) {
	return ((PMSM_Speed_prev > 0) & (PMSM_Speed > 0));
}

uint16_t PMSM_GetSpeed(void) {
	return PMSM_Speed;
}

void PMSM_MotorSetRun(void) {
	PMSM_MotorRunFlag = 1;
}

void PMSM_MotorSetSpin(uint8_t spin) {
	PMSM_MotorSpin = spin;
}

void PMSM_SetSensorCheckMode(uint8_t value) {
	PMSM_SensorCheckMode = value;
}

uint8_t PMSM_GetSensorCheckMode(void) {
	return PMSM_SensorCheckMode;
}

uint8_t PMSM_GetSensorPosition(void) {
	return PMSM_SensorPosition;
}

void PMSM_SaveSensorPosition(void) {
	PMSM_STATE_TABLE_INDEX[0][0] = PMSM_GetNormPos(PMSM_SensorPosition - PMSM_SINTABLESIZE/4 - PMSM_SINTABLESIZE/3);
	PMSM_STATE_TABLE_INDEX[0][1] = PMSM_GetNormPos(PMSM_SensorPosition + PMSM_SINTABLESIZE/4 - PMSM_SINTABLESIZE/3);
	PMSM_STATE_TABLE_INDEX[1][0] = PMSM_GetNormPos(PMSM_SensorPosition - PMSM_SINTABLESIZE/4);
	PMSM_STATE_TABLE_INDEX[1][1] = PMSM_GetNormPos(PMSM_SensorPosition + PMSM_SINTABLESIZE/4);

	FLASH_WriteSettings();
}

uint16_t get_PMSM_Speed_prev() {
	return PMSM_Speed_prev;
}
