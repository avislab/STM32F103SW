#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "misc.h"
#include "sysclk.h"
#include "stdio.h"
#include "adc_dma.h"
#include "pmsm.h"
#include "systickdelay.h"

int main(void)
{
	uint16_t ADC_POT = 0;

	SetSysClockTo72();
	sysTickDalayInit();

	// ADC Init
	ADC_DMA_Init();

	// PMSM Init
	PMSM_Init();

    while(1)
    {
    	// ======== Damper =========
    	/*
    	sysTickDalay(1);
		if (ADC_DMA_GET(ADC_CN_POT) > ADC_POT) {
			if (ADC_POT < 4096) {
				ADC_POT++;
			}
		}
		else {
			if (ADC_POT > 0) {
				ADC_POT--;
			}
		}
    	*/
    	ADC_POT = ADC_DMA_GET(ADC_CN_POT);

    	// ================================
    	if (ADC_POT > PMSM_ADC_START) {
    		// If Motor Is not runing
    		if (PMSM_MotorIsRun() == 0) {
    			// Start motor
    			// Check Reverse pin
    			if (PMSM_GetButtonRevers() == Bit_RESET) {
    				// Forward
    				PMSM_MotorSetSpin(PMSM_CW);
    			}
    			else {
    				// Backward
    				PMSM_MotorSetSpin(PMSM_CCW);
    			}

    			if (PMSM_GetButtonKey1() == Bit_RESET) {
    				PMSM_SetSensorCheckMode(1);
    			}

    			PMSM_Start();
    		}
   			PMSM_SetPWM(PMSM_ADCToPWM(ADC_POT));
    	}
    	else {
    		PMSM_MotorStop();
    	}

    }
}
