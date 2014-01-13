//--------------------------------------------------------------
// File     : main.c
// Datum    : 04.03.2013
// CPU      : STM32F4
//--------------------------------------------------------------

#include "spi.h"
#include "slist.h"
#include "stm32_ub_dac_dma.h"

#include "stm32f4xx.h"

#include "stm32f4xx.h"

/* leds in the board will fade */

uint32_t ccr[4][3];
uint8_t flag[4] = {1,1,1,1};
int period = 36000;
//uint8_t ccr_order[4];


void TIM4_IRQHandler(void)
{
//	static uint8_t i = 0;
//	static uint32_t * ccr_p = {
//			&(TIM4->CCR1),
//			&(TIM4->CCR2),
//			&(TIM4->CCR3),
//			&(TIM4->CCR4),
//	};

	if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET) {
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
		TIM4->CCR1 += (flag[0] == 2 ?
				ccr[0][2] + ccr[0][0] : ccr[0][flag[0]]);
		TIM4->CCR1 %= period;
		flag[0]++;
		if (flag[0] == 3)
			flag[0] = 1;
	}
	if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET) {
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
		TIM4->CCR2 += (flag[1] == 2 ?
				ccr[1][2] + ccr[1][0] : ccr[1][flag[1]]);
		TIM4->CCR2 %= period;
		flag[1]++;
		if (flag[1] == 3)
			flag[1] = 1;
	}
	if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET) {
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
		TIM4->CCR3 += (flag[2] == 2 ?
				ccr[2][2] + ccr[2][0] : ccr[2][flag[2]]);
		TIM4->CCR3 %= period;
		flag[2]++;
		if (flag[2] == 3)
			flag[2] = 1;	}
	if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET) {
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
		TIM4->CCR4 += (flag[3] == 2 ?
				ccr[3][2] + ccr[3][0] : ccr[3][flag[3]]);
		TIM4->CCR4 %= period;
		flag[3]++;
		if (flag[3] == 3)
			flag[3] = 1;
	}

//	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
//	{
//
//		ccr_p[i] = ccr[i][flag];
//		i++;
//
//		i %= 4;
//		if (!i) {
//			if (flag)
//				flag = 0;
//			else
//				flag = 1;
//		}
//
//		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
//	}
}

void INTTIM_Config(void)
{
NVIC_InitTypeDef NVIC_InitStructure;
/* Enable the TIM4 gloabal Interrupt */
NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);



/* TIM IT enable */
TIM_ITConfig(TIM4, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE);
}


void RCC_Configuration(void)
{
  /* enable peripheral clock for TIM4 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
}

//******************************************************************************

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOD Configuration:  TIM4 on PD12/PD13/PD14 LED */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Connect TIM4 pin  */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4); // PD12 TIM4_CH1 GREEN
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); // PD13 TIM4_CH2 ORANGE
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4); // PD14 TIM4_CH3 RED
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4); // PD15 TIM4_CH4 BLUE

}

//******************************************************************************

void TIM4_Configuration(uint32_t new_ccr[4])
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  int Prescaler;//, Period;

  Prescaler = 80;//((SystemCoreClock / 2) / 1000000);//360000); // ~360 KHz timebase, assumes APB1 H/4 TIMCLK4 H/2

  period = 65535; // ~10 Hz -> ~ 5 Hz

  // The toggle halves the frequency

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = period - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = Prescaler - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* Output Compare Toggle Mode configuration: Channel 1, 2 and 3 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Don't want to set 0 or Period, but have 3 points at 120 degrees from each other */
  TIM_OCInitStructure.TIM_Pulse = new_ccr[0];// - 1; added 1 to all phase values
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);   // doesn't change anything but now
  	  	  	  	  	  	  	  	  	  	  	 // 0 as phase value is ok
  TIM_OCInitStructure.TIM_Pulse = new_ccr[1];// - 1;
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = new_ccr[2];// - 1;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = new_ccr[3];// - 1;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  INTTIM_Config();

  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);
}

uint32_t percent2time(uint16_t f, uint32_t percent)
{
	const double t = 2.976190476E-6 * 100;

	double temp = f*t;

	uint32_t ret= (percent / temp) + 0.5;
	return ret;
}

int TIM_reset(uint16_t f, uint8_t duty[4], uint8_t phase[4])
{
	uint8_t i;
	uint32_t low;
	uint32_t new_ccr[4];

    TIM_Cmd(TIM4, DISABLE);
    TIM_ITConfig(TIM4, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, DISABLE);

    if (f < 5 || f > 3000)
    		return -1;

    for (i = 0; i < 4; i++) {
    		low = 100 - duty[i];
    		if (phase[i] <= low) {
    			ccr[i][0] = percent2time(f, phase[i]);
    			ccr[i][1] = percent2time(f, duty[i]);
    			ccr[i][2] = percent2time(f, low - phase[i]);

    			new_ccr[i] = (ccr[i][0] == 0 ?
    					ccr[i][0] + percent2time(f, 100) : ccr[i][0]);
    		}
    		else
    		{
    			ccr[i][0] = percent2time(f, 0);
    			ccr[i][1] = percent2time(f, duty[i]);
    			ccr[i][2] = percent2time(f, low);

    			new_ccr[i] = (ccr[i] == 0 ?
    					ccr[i][0] + percent2time(f, 100) :
    					percent2time(f, phase[i]));
    		}
    }

    for (i = 0; i < 4; i++) {
    	flag[i] = 1;
    }
    //TIM4->CNT = 0;
    TIM_ITConfig(TIM4, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE);
    TIM4_Configuration(new_ccr);


    return 0;
}

int main(void)
{
 ccr[0][0] = 65535/2;
 ccr[0][1] = 65535/2;
 ccr[0][2] = 1;
 ccr[1][0] = 10;
 ccr[1][1] = 10;
 ccr[1][2] = 10;
 ccr[2][0] = 65535 - 400;
 ccr[2][1] = 399;
 ccr[2][2] = 1;
 ccr[3][0] = 65535 - 801;
 ccr[3][1] = 800;
 ccr[3][2] = 1;
 //int Period = 36000; // ~10 Hz -> ~ 5 Hz
 SystemInit();
 SystemCoreClockUpdate();

  RCC_Configuration();

  GPIO_Configuration();

  //TIM4_Configuration();


int i;
uint8_t d[4] = {50,50,50,50};
uint8_t p[4] = {0, 25, 50, 100};
  //while(1){ /* Infinite loop */
//	  ccr[0][0]+=1;
//	  ccr[0][0] %= 360;
//
//	  ccr[0][1] -= 1;
//	  ccr[0][1] %= 360;

            for(i=0;i<900000;i++);  // delay
            for(i=0;i<900000;i++);  // delay
            for(i=0;i<900000;i++);  // delay

            TIM_reset(100, d, p);
while(1);

 // }
}
