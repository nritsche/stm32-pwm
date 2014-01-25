//--------------------------------------------------------------
// File     : main.c
// Datum    : 04.03.2013
// CPU      : STM32F4
//--------------------------------------------------------------

#include "spi.h"
#include "slist.h"
#include "stm32_ub_dac_dma.h"
#include "stm32f4xx.h"

uint32_t ccr[4][3];
uint8_t flag[4] = {1,1,1,1};
uint8_t toggle_mode[4] = {0,0,0,0};
int period = 36000;


void TIM4_IRQHandler(void)
{
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

  GPIO_InitStructure.GPIO_Pin = 0;

  /* GPIOD Configuration:  TIM4 on PD12/PD13/PD14/PD15 LED */
  if (toggle_mode[0])
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  if (toggle_mode[1])
	  GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_13;
  if (toggle_mode[2])
	  GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_14;
  if (toggle_mode[3])
	  GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_15;

  if (GPIO_InitStructure.GPIO_Pin != 0) {
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
  }

  GPIO_InitStructure.GPIO_Pin = 0;

  if (!toggle_mode[0])
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  if (!toggle_mode[1])
	  GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_13;
  if (!toggle_mode[2])
	  GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_14;
  if (!toggle_mode[3])
	  GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_15;
  if (GPIO_InitStructure.GPIO_Pin != 0) {
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
  }

  /* Connect TIM4 pin  */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4); // PD12 TIM4_CH1 GREEN
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); // PD13 TIM4_CH2 ORANGE
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4); // PD14 TIM4_CH3 RED
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4); // PD15 TIM4_CH4 BLUE

  // PB10 is the CS pin for SPI
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//******************************************************************************

void TIM4_Configuration(uint32_t new_ccr[4])
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  int Prescaler;//, Period;

  Prescaler = 80;

  period = 65535;

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

void set_channel(uint8_t i, uint8_t state)
{
	if (state)
		GPIOD->BSRRL = 0x0001 << (i + 12);
	else
		GPIOD->BSRRH = 0x0001 << (i + 12);
}

int TIM_reset(uint16_t f, uint8_t duty[4], uint8_t phase[4])
{
	uint8_t i;
	uint32_t low;
	uint32_t new_ccr[4];
	uint8_t state[4];

	TIM_ITConfig(TIM4, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, DISABLE);
    TIM_Cmd(TIM4, DISABLE);

    if (f < 5 || f > 3000)
    		return -1;

    for (i = 0; i < 4; i++) {
    	toggle_mode[i] = 1;
		low = 100 - duty[i];
		if (duty[i] == 0) {
			toggle_mode[i] = 0;
			state[i] = 0;
		}
		if (duty[i] == 100) {
			toggle_mode[i] = 0;
			state[i] = 1;
		}
		else if (phase[i] <= low) {
			ccr[i][0] = percent2time(f, phase[i]);
			ccr[i][1] = percent2time(f, duty[i]);
			ccr[i][2] = percent2time(f, low - phase[i]);

			new_ccr[i] = (ccr[i][0] == 0 ?
					percent2time(f, 100) : ccr[i][0]);
		}
		else
		{
			ccr[i][0] = percent2time(f, 0);
			ccr[i][1] = percent2time(f, duty[i]);
			ccr[i][2] = percent2time(f, low);

			new_ccr[i] = percent2time(f, phase[i]);
		}
    }

    for (i = 0; i < 4; i++) {
    	flag[i] = 1;
    }
    //TIM4->CNT = 0;
    TIM_DeInit(TIM4);
    GPIO_DeInit(GPIOD);
    RCC_DeInit();

    SystemInit();
     SystemCoreClockUpdate();
    RCC_Configuration();
    GPIO_Configuration();
    TIM_ITConfig(TIM4, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE);
    TIM4_Configuration(new_ccr);

    for (i = 0; i < 4; i++) {
    	if (!toggle_mode[i])
    		set_channel(i, state[i]);
    }

    return 0;
}

int main(void)
{
	SystemInit();
	SystemCoreClockUpdate();

	RCC_Configuration();

	GPIO_Configuration();

	UB_SPI2_Init(SPI_MODE_2);


	uint8_t duty[4] = {50,50,50,50};
	uint8_t phase[4] = {25, 50, 75, 100};
	uint16_t data, xor = 0x0000, i, freq, answer = 0x00;

	while (1) {
		i = 0;
		xor = 0;
		answer = 0;

		// wait for CS pin to be active
		while ((GPIOB->IDR & (0x1 << 10)));

		while (!(GPIOB->IDR & (0x1 << 10))) {
			data = UB_SPI2_SendByte(answer);

			if (i == 0)
				freq = data;
			else if (i < 5)
				duty[i - 1] = data;
			else if (i < 9)
				phase[i - 5] = data;
			else if (i == 9 && data == xor)
					answer = xor;

			xor ^= data;
			i++;
		}

		TIM_reset(freq, duty, phase);
	}
}
