#include <stdio.h>
#include <stm32f0xx.h>
#include <stm32f070x6.h>

const int counter = 800;
uint16_t BUFF[counter];
int flag = 0;

int main(void){
	
	int j;
	uint16_t data;
	uint32_t address;
		
	//HSI (8 MHz) on
	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY_Msk)) {}
		
	// RCC initialization for GPIO Port A
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// PA1 for ADC input now, written lower
	GPIOA->MODER &= ~GPIO_MODER_MODER1;

	// PA2 init output for oscilloscope
	GPIOA->MODER |= GPIO_MODER_MODER2_0;
	//GPIOA->OTYPER |= GPIO_OTYPER_OT_0; //god no, don't write this sh*t!
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR2_0;
	
	//PA0 - button
	GPIOA->MODER &= ~GPIO_MODER_MODER0; //unnecessary coz default value is 00
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1; //without PULL_DOWN button works only ones
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR2_0;
		
	//configuring prescelers
	//AHB = 1, SYSCLK not  divided
	RCC->CFGR &=~ RCC_CFGR_HPRE_Pos;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	//APB1 = 1, HCLK not divided
	RCC->CFGR &=~ RCC_CFGR_PPRE_Pos;
	RCC->CFGR |= RCC_CFGR_PPRE_DIV1;
		
	//PLL output selected as system clock
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	
	// PLL configuration
	RCC->CFGR |= RCC_CFGR_PLLMUL14; //0100: PLL input clock x 14
	RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV4; //0011: PREDIV input clock divided by 4
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_PREDIV; //HSI/PREDIV clock selected as PLL entry clock source
	//final frequency - 28 MHz
	RCC->CR |= RCC_CR_PLLON;
	
	FLASH->ACR = 3;
	///////////////////////////////////////
	
	//TIM3 initialization
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	TIM3->SMCR &= ~TIM_SMCR_SMS; //the prescaler is clocked by the internal clock
	
	TIM3->PSC = 14-1; // Set the prescaler to 13 so that the timer frequency is 2 MHz
	TIM3->ARR = 2-1; // Interrupt every 1 mks
	
	TIM3->CR1 = TIM_CR1_CEN; //TIM3 enable
	
	TIM3->CR2 &= ~(TIM_CR2_MMS_0 | TIM_CR2_MMS_2); //update event is selected as trigger output (TRGO)
	TIM3->CR2 |= TIM_CR2_MMS_1;
	////////////////////////////////////////
	
	//Button interruption (it really works)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //enable selection of source for the exti interruption
	//interrupt setting for button
	EXTI->IMR = 0;
	EXTI->IMR |= EXTI_IMR_MR0;// Configure the corresponding mask bit in the EXTI_IMR register
	EXTI->RTSR |= EXTI_RTSR_TR0;//Configure the Trigger Selection bits of the Interrupt line on rising edge
	EXTI->FTSR &= ~EXTI_FTSR_TR0;//Configure the Trigger Selection bits of the Interrupt line NOT on falling edge
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;  ////unnecessary coz default value is 0000
	
	__NVIC_EnableIRQ(EXTI0_1_IRQn); // enable button interrupt
	__NVIC_SetPriority(EXTI0_1_IRQn, 3);
	///////////////////////////////////////
	
	//DMA initialization
	RCC->AHBENR |= RCC_AHBENR_DMAEN; //DMA clocking enable
	
	SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_ADC_DMA_RMP;
	
	DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR)); // extract data from ADC
	DMA1_Channel1->CMAR = (uint32_t) (&BUFF[0]); //storage array
	DMA1_Channel1->CNDTR = counter;	//buffer length
	DMA1_Channel1->CCR |= DMA_CCR_MINC; //memory address increment
	DMA1_Channel1->CCR &= ~DMA_CCR_PINC; //no peripheral address increment
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0; //16 bit
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0; //16 bit
	DMA1_Channel1->CCR &= ~DMA_CCR_CIRC; //no circular mode 
	DMA1_Channel1->CCR &= ~DMA_CCR_DIR; //00: peripheral-to-Memory
	DMA1_Channel1->CCR |= DMA_CCR_PL; //11: very high priority
	DMA1_Channel1->CCR |= DMA_CCR_TCIE;	//Transfer complete interrupt enable
	DMA1_Channel1->CCR |= DMA_CCR_EN; //enable data transfer
	
	__NVIC_EnableIRQ (DMA1_Channel1_IRQn); 
	////////////////////////////////////////

	//ADC initialization
	//Enable ADC clocking
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
		
	//Prohibit everything in control registers
	ADC1->CR = 0;
  ADC1->CFGR1 = 0;
  ADC1->CFGR2 = 0;	

//	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; /* (1) */
//	RCC->CR2 |= RCC_CR2_HSI14ON; /* (2) */
//	while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) /* (3) */
//	{
//	/* For robust implementation, add here time-out management */
//	}
//	ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;
//	
	// ADC clock source is PCLK divided by 2
	ADC1->CFGR2 |= ADC_CFGR2_CKMODE_0;
	
	// ADC Calibration code //
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* Ensure that ADEN = 0 */
	{
		ADC1->CR |= ADC_CR_ADDIS; /*  Clear ADEN by setting ADDIS */
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
		/* For robust implementation, add here time-out management */
	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* Clear DMAEN */
	ADC1->CR |= ADC_CR_ADCAL; /* Launch the calibration by setting ADCAL */
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* Wait until ADCAL = 0 */
	{
		/* For robust implementation, add here time-out management */
	}
	
	// ADC enable code //
	ADC1->CR |= ADC_CR_ADEN; /* Enable the ADC */
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* Wait until ADC ready */
	{
		/* For robust implementation, add here time-out management */
	}
	
	//Sampling time is 1.5 cycles
	ADC1->SMPR &= ~ADC_SMPR_SMP;	
	//Resolution is 12 cycles 
	ADC1->CFGR1 &= ~ADC_CFGR1_RES;
//	ADC1->CFGR1 |= ADC_CFGR1_RES_0 | ADC_CFGR1_RES_1;
	//Total convertion time is 1.5 + 12 = 13.5 cycles
	
	
	ADC1->CHSELR |= ADC_CHSELR_CHSEL1; //One regular channel (IN1 - PA1)
	ADC1->CFGR1 &= ~ADC_CFGR1_CONT; // Disable continuous mode	
	
	ADC1->CFGR1 |= ADC_CFGR1_EXTEN_1; //Hardware trigger detection on the rising edge
	ADC1->CFGR1 |= ADC_CFGR1_EXTSEL_0 | ADC_CFGR1_EXTSEL_1; //trigger on TIM3_TRGO
	
	ADC1->CFGR1 |= ADC_CFGR1_OVRMOD; //ADC_DR register is overwritten when an overrun
	
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN; // DMA enable
	ADC1->CFGR1 &= ~ADC_CFGR1_DMACFG; //DMA one shot mode
	//////////////////////////////////////////////////////

				
	//Enable clocking UART1 (48 MHz)
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; 
	
	// PA9 - USART1-TX, PB10 - USART1-RX
	GPIOA->MODER |= GPIO_MODER_MODER9_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_0;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_9;
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR9_0;
	GPIOA->MODER |= GPIO_MODER_MODER10_1;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_10;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0;
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR10_0;
	
	//choosing of alternate function for GPIO for USART
	GPIOA->AFR[1] |= (0x1UL << GPIO_AFRH_AFSEL9_Pos);
	GPIOA->AFR[1] |= (0x1UL << GPIO_AFRH_AFSEL10_Pos);
	//GPIOB->AFR[1] = 0x00000110
	
	//USART1 congiguration
//	USART1->BRR = 0x9C2; //baud rate 38400 (PLL)
	USART1->BRR = 0x2710; //baud rate 9600 (PLL)
	
	USART1->CR1 |= USART_CR1_OVER8; //Oversampling by 8-bit
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE ; // enable receiver and transmitter
	USART1->CR2 = 0;
	USART1->CR3 = 0;

	USART1->CR1 |= USART_CR1_UE; //USART enable
	////////////////////////////////////////
	
		
  while(1) {
		if (flag == 1){
	
			// PLL configuration
			RCC->CFGR &= ~RCC_CFGR_SW_PLL;
			RCC->CR &= ~RCC_CR_PLLON;
			RCC->CFGR &= ~RCC_CFGR_PLLMUL14;
			RCC->CFGR |= RCC_CFGR_PLLMUL6; //0100: PLL input clock x 6
			RCC->CFGR2 &= ~RCC_CFGR2_PREDIV_DIV4;
			RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV1; //0000: PREDIV input clock not divided
			RCC->CFGR |= RCC_CFGR_SW_PLL;
			//final frequency - 48 MHz
			RCC->CR |= RCC_CR_PLLON;
			while ((RCC->CR & RCC_CR_PLLRDY) == 0){
			}
			
			for (j = 0; j < counter; j++){
				
				address = (uint32_t)&BUFF[0]+2*j;
				data = (*(__IO uint16_t*) address);
			
				while ((USART1->ISR & USART_ISR_TXE) == 0) {}
				USART1->TDR = data>>8;
				while ((USART1->ISR & USART_ISR_TXE) == 0) {}
				USART1->TDR = data>>0;
				while ((USART1->ISR & USART_ISR_TXE) == 0) {}
		
			}
	
			// PLL configuration
			RCC->CFGR &= ~RCC_CFGR_SW_PLL;
			RCC->CR &= ~RCC_CR_PLLON;
			RCC->CFGR &= ~RCC_CFGR_PLLMUL6;
			RCC->CFGR |= RCC_CFGR_PLLMUL14; //0100: PLL input clock x 14
			RCC->CFGR2 &= ~RCC_CFGR2_PREDIV_DIV1;
			RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV4; //0011: PREDIV input clock divided by 4
			RCC->CFGR |= RCC_CFGR_SW_PLL;
			//final frequency - 28 MHz
			RCC->CR |= RCC_CR_PLLON;
			while ((RCC->CR & RCC_CR_PLLRDY) == 0){
			}
	
			DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR)); // extract data from ADC
			DMA1_Channel1->CMAR = (uint32_t) (&BUFF[0]); //storage array
			DMA1_Channel1->CNDTR = counter;	//buffer length
			DMA1_Channel1->CCR |= DMA_CCR_MINC; //memory address increment
			DMA1_Channel1->CCR &= ~DMA_CCR_PINC; //no peripheral address increment
			DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0; //16 bit
			DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0; //16 bit
			DMA1_Channel1->CCR &= ~DMA_CCR_CIRC; //no circular mode 
			DMA1_Channel1->CCR &= ~DMA_CCR_DIR; //00: peripheral-to-Memory
			DMA1_Channel1->CCR |= DMA_CCR_PL; //11: very high priority
			DMA1_Channel1->CCR |= DMA_CCR_TCIE;	//Transfer complete interrupt enable
			DMA1_Channel1->CCR |= DMA_CCR_EN; //enable data transfer
	
	
			flag = 0;
			
//			while (j < UINT16_MAX*16){
//				j++;
//			}
//			
			__NVIC_EnableIRQ(EXTI0_1_IRQn); // enable button interrupt
//			__NVIC_EnableIRQ(DMA1_Channel1_IRQn);
			
		}
	}
 
}

	
void EXTI0_1_IRQHandler (void){ //button interrupt
//	
	__NVIC_DisableIRQ(EXTI0_1_IRQn);
	
	ADC1->CR |= ADC_CR_ADSTART; //start convertation 
	
	EXTI->PR = EXTI_PR_PR0;
	
	GPIOA->BSRR |= GPIO_BSRR_BS_2;
	
	__NVIC_ClearPendingIRQ(EXTI0_1_IRQn);//exit from button interrupt
	
	
}

void DMA1_Channel1_IRQHandler(void){
	
	GPIOA->BSRR |= GPIO_BSRR_BR_2;
		
	DMA1->IFCR |= DMA_IFCR_CTCIF1; // Channel 1 transfer complete clear
	
	DMA1_Channel1->CCR |= DMA_CCR_TCIE;	//Transfer complete interrupt enable
	
	DMA1_Channel1->CCR &= ~DMA_CCR_EN; //DMA disable
	ADC1->CR |= ADC_CR_ADSTP; //Stop any ongoing conversion
	
	flag = 1;	
	
	__NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
//	__NVIC_DisableIRQ(DMA1_Channel1_IRQn);
}
