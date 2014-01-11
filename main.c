//--------------------------------------------------------------
// File     : main.c
// Datum    : 04.03.2013
// CPU      : STM32F4
//--------------------------------------------------------------

#include "spi.h"
#include "slist.h"
#include "stm32_ub_dac_dma.h"

uint8_t spi_send(uint8_t value);

void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

//--------------------------------------------------------------
// send n receive one byte via SPI
//--------------------------------------------------------------
uint8_t spi_send(uint8_t value)
{
  uint8_t ret;

  // send n receive one byte
  ret =UB_SPI2_SendByte(value);

  return(ret);
}




void init_GPIO(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/*
	 * The LEDs on the STM324F Discovery are connected to the
	 * pins PD12 thru PD15
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12; // we want to configure all LED GPIO pins
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOD, &GPIO_InitStruct); 			// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.

	/* This enables the peripheral clock to
	 * the GPIOE IO module
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);


	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;		  // we want to configure PE15
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 	  // we want it to be an input
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   // this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;   // this enables the pulldown resistor --> we want to detect a high level
	GPIO_Init(GPIOA, &GPIO_InitStruct);			  // this passes the configuration to the Init function which takes care of the low level stuff
}

void blink()
{
	while(1) {
		GPIOD->BSRRL = 0xF000; // this sets LEDs
		Delay(1000000);
		GPIOD->BSRRH = 0xF000; // this resets LEDs
		Delay(1000000);
	}
}

void init()
{
	  init_GPIO();

	  GPIOD->BSRRL = 0xF000; // set PD12 thru PD15
	  Delay(3000000L);		 // wait a short period of time
	  GPIOD->BSRRH = 0xF000; // reset PD12 thru PD15

	  SystemInit(); // Quarz Einstellungen aktivieren

	  // SPI2 im Mode0 initialisieren
	  ErrorStatus err = UB_SPI2_Init(SPI_MODE_0);

	  if (err == ERROR)
		  blink();

	  // init vom DAC im DMA-Mode (DAC-1 und DAC-2)
	  UB_DAC_DMA_Init(DUAL_DAC_DMA);
}

//--------------------------------------------------------------
int main(void)
{
    linked_list_t *list = NULL;
    uint16_t *array;
	uint8_t first = 0x00, sec = 0x00;
	uint16_t data;
	uint16_t len = 0;
	uint16_t prescaler;
	uint16_t overflow;
	uint16_t num;

	init();


while (1) {
	while ((GPIOB->IDR & (0x1 << 10)));
	GPIOD->BSRRL = 0xF000; // this sets LEDs

	while (!(GPIOB->IDR & (0x1 << 10))) {
		first = spi_send(first);//_first);
		sec = spi_send(sec);//_sec);
		data = first + (sec << 8);
		if (addItem(&list, data) == ERROR)
			blink();
		len++;
	}

	if (!len)
		continue;

	removeLast(&list);
	overflow = removeLast(&list);
	prescaler = removeLast(&list);
	num = removeFirst(&list);
	len -= 4;

	array = list2array(list, len);
	if (array == NULL)
		blink();
	clear(&list);

	if (num == 1) {
		UB_DAC_DMA_SetWaveform1_man(array, len);
		UB_DAC_DMA_SetFrq1(prescaler-1, overflow-1);
	}
	else if (num == 2) {
		UB_DAC_DMA_SetWaveform2_man(array, len);
		UB_DAC_DMA_SetFrq2(prescaler-1, overflow-1);
	}

	len = 0;
	free(array);

	GPIOD->BSRRH = 0xF000; // this resets LEDs
}

	return 0;
}
