/* 	STM32 Self-Learning
		
		Creating a new project:
		- http://www.ocfreaks.com/create-new-stm32-project-keil-uvision-5-tutorial/
			- Keil - include Core and Device Startup Files	

		Setup:
		- Keil MDK with:
			- CMSIS core 
			- Device Files (RTE_Device.h, startup_stm32f10x_md.s, system_stm32f10x.c)
			- cmsis_armcc.h
			- cmsis_compiler.h
			- cmsis_version.h
			- RTE_Components.h
			- stdint.h
			- stm32f10x.h
			- system_stm32f10x.h
				- make sure set up options for target correctly in Keil (select correct chip, frequency, debugger)
		- STM32 Blue Pill
		- STM32 programmer cable
		- PuTTy + FT232 USB to TTL cable for serial UART communication testing
		
		Tutorials:
		- https://vivonomicon.com/2020/06/28/bare-metal-stm32-programming-part-10-uart-communication/
		
		- To download new program onto STM32:
			- press build, then download
		
		I/O:
		- Pin A0: LED output
		- Pin A1: External interrupt
		- Pin A4: ADC input
		- Pin A6: Timer 3 PWM output
		- Pin A9: TX UART
		- Pin A10: RX UART
		
		FT232 USB to TTL Serial cable
		- Red = VCC
		- Black = GND
		- White = TXD (from point of view of microcontroller - connect to TX pin)
		- Green = RXD (from point of view of microcontroller - connect to RX pin)
		
		RCC:
		- reset and clock control
		APB: Advanced Peripheral Bus
		- APB2ENR: APB clock enable register (for USART, SPI, Timers, IOs) 
		AHB: Advanced High-performance Bus
		- AHBENR (for ethernet, USB communications)
*/


#include <stm32f10x.h>
#include <stdint.h>

//Function declarations
void setupPLLClock_frmHSE(uint32_t freq);
void setupPLLClock_frmHSI(void);
void setupGPIO_A(void);
void configureGPIO_A(void);
void setPinHigh(void);
void setPinLow(void);
void setupAlternateFunctions(void);
void configureExternalInterrupts(void);
void EXTI1_IRQHandler(void);
void setupTimers(void);
void startTimer(TIM_TypeDef* TIMx, uint32_t ms);
void TIM2_IRQHandler(void);
void setupPWM(void);
void setupADC(void);
void ADC1_2_IRQHandler(void);
void setupUART(void);


// Clock variables
uint32_t PLLfreq;
uint32_t ticks;
uint16_t led_on = 0;
uint32_t idr_val;
uint32_t core_clock_hz;
uint16_t adc_data;

// Timer and duty cycle variables
uint16_t duty = 10; //only want to vary from 10 to 20% duty cycle
uint16_t delay = 0; //general purpose delay
uint16_t tim2InterruptOccurred = 0;

// UART variables
char rxb = '\0';
char STX = '\x02';
char ETX = '\x03';

//Constants
#define INPUT_PIN 3

int main(void){
	
	//Set up clock - realize that clock is actually set in system_stm32f10x.c on reset button. 
	//Function call here is just for consistency.
	PLLfreq = 72; //want PLLfreq of 72MHz
	setupPLLClock_frmHSE(PLLfreq); 
	
	setupGPIO_A(); //enable GPIO A Clock, reset GPIO A registers
	setupAlternateFunctions(); //enable alternate function clock - needed for timer output to pins and pin external interrupts
	configureGPIO_A(); //configure IOs on pins - A0 output, A1 is input for an interrupt
	// configureExternalInterrupts(); // NOT USED FOR THIS PROJECT
	setupPWM(); //set up pin A6 to output timer 3 PWM signal
	//setupADC(); //set up pin A4 for ADC input
	//setupUART(); //set up pin A9, A10 for TX/RX for UART - NOT USED FOR THIS PROJECT
	setupTimers(); //enable timer 2 and timer 2 interrupt
	startTimer(TIM2, 1000); // one second general purpose timer
	
	// Wait two seconds for ESCs to arm
	do{} while(delay < 2);
		
	startTimer(TIM2, 100); // debounce in ms
	
	//main loop
  while(1){
		//echo
    //wait for a byte of data to arrive.
    //while( !( USART1->SR & USART_SR_RXNE ) ) {};
    //rxb = USART1->DR; //this works

    //re-transmit the received byte.
    //while( !( USART1->SR & USART_SR_TXE ) ) {};
    //USART1->DR = rxb; //this works
			
		//for (int i = 0; i < 500000; i++) {}; //delay 
		//while( !( USART1->SR & USART_SR_TXE ) ) {};
    //USART1->DR = 'A'; //this works
		
		// Don't user interrupts because they don't debounce well
		if(GPIOA->IDR & (1 << INPUT_PIN))
		{
			// debounce is incremented by timer2
			if(delay == 3) //200ms, assuming timer 2 is started with 100ms delay
			{
				duty = 15;
				TIM3->CCR1 = duty;
				setPinHigh();
			}
		}
		else
		{
			delay = 0;
			duty = 10;
			TIM3->CCR1 = duty;
			setPinLow();
		}

	}
}


/* 	
Setup PLL Clock
Input: desired PLL frequency
JN 20200428
*/
void setupPLLClock_frmHSE(uint32_t freq)
{
	
	//Reset registers
	RCC->CFGR  &= ~(RCC_CFGR_PLLMULL | RCC_CFGR_PLLSRC);
	RCC->CR &= ~RCC_CR_PLLON;
	//Set up high speed external clock. In STM Blue Pill, the external oscillator is 8 MHz
	RCC->CR |= RCC_CR_HSEON; //RCC is a pointer to the address of the RCC register
	while (!(RCC->CR & RCC_CR_HSERDY)) {}; //Wait until HSE stabilizes
	
	//Divide HSE before feeding it as source for RCC
	//**RCC->CFGR |= RCC_CFGR_PLLXTPRE; //If set this, then divide HSE by 2 before feed to RCC
	
	//Set up source for RCC
	RCC->CFGR |= RCC_CFGR_PLLSRC; //Select HSE as PLL source. PLL stands for phased lock loop -> allows microcontroller to run at higher freq. than oscillator
	
	switch(freq)
	{
		case 16:
			RCC->CFGR |= RCC_CFGR_PLLMULL2; //Select multiplier of 2
			core_clock_hz = 16000000;
		case 32:
			RCC->CFGR |= RCC_CFGR_PLLMULL4; //Select multiplier of 4
			core_clock_hz = 32000000;
		case 1:
			core_clock_hz = 8000000;
		case 64:
			RCC->CFGR |= RCC_CFGR_PLLMULL8; //Select multiplier of 8
			core_clock_hz = 64000000;
		case 72:
			RCC->CFGR |= RCC_CFGR_PLLMULL9; //Select multiplier of 9
			core_clock_hz = 72000000;
	}
	
	//Enable the PLL
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC_CR_PLLRDY & RCC->CR)){}; // wait until PLL is locked
	
	// Select the System Clock Source (in this case we want the PLL)
	RCC->CFGR  &= ~(RCC_CFGR_SW);
  RCC->CFGR  |=  (RCC_CFGR_SW_PLL); //select PLL as system clock source
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)){}; 
	
	// Chip reads memory at 24MHz. If operating faster, we need to tell it to wait when it retrieves data from memory 
	// Set up 2 bits of latency
  FLASH->ACR |=  (FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE);
	
}


/* 	
Setup PLL Clock
Input: desired PLL frequency
JN 20210124
*/
void setupPLLClock_frmHSI(void){
	
	RCC->CFGR  &= ~(RCC_CFGR_PLLMULL |	RCC_CFGR_PLLSRC);
	RCC->CFGR  |=  (RCC_CFGR_PLLSRC_HSI_Div2 |	RCC_CFGR_PLLMULL12);
	// Turn the PLL on and wait for it to be ready.
	RCC->CR|= (RCC_CR_PLLON);
	while (!(RCC->CR & RCC_CR_PLLRDY)) {};
	// Select the PLL as the system clock source.
	RCC->CFGR  &= ~(RCC_CFGR_SW);
	RCC->CFGR  |=  (RCC_CFGR_SW_PLL);
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) {};
}


/*
 * Setup GPIO A 
 * Enable GPIO A Clock
 * Reset GPIO A registers
*/
void setupGPIO_A()
{
	//enable GPIOA clock  - set clock so microcontroller can set the register bits
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	//Clear GPIO
	GPIOA->CRL = 0;
	GPIOA->CRH = 0;
}


/*
 * Setup alternate function I/O (eg. output timer on pin, interrupts)
 * Enable AFIO clock
 */
void setupAlternateFunctions()
{
	//Enable Alternate function clock - AFIO clock should first be enabled before mapping external interrupt/event lines
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
}

/* 
Setup different pins as inputs/outputs 
*/
void configureGPIO_A()
{
	//Setup GPIO A0 as output
	GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0); //reset MODE0 and CNF0 (output mode, push-pull default)
	GPIOA->CRL |= (GPIO_CRL_MODE0_1 | GPIO_CRL_MODE0_0); //output mode, max spd 50MHz 
	
	/*
	//Setup GPIO A1 as input for interrupt
	//Note: For alternate function inputs, the port must be configured in Input mode (floating, pullup or pull-down) and the input pin must be driven externally (Manual Section 9.1.4)
	GPIOA->CRL &= ~GPIO_CRL_MODE1; //reset MODE1 - input mode
	GPIOA->CRL &= ~GPIO_CRL_CNF1; //reset CNF1_1 and CNF1_0
	GPIOA->CRL |= GPIO_CRL_CNF1_1; //set CNF1_1 - input mode, pull-up/down
	
	//Setup GPIO A2 as input for interrupt
	//Note: For alternate function inputs, the port must be configured in Input mode (floating, pullup or pull-down) and the input pin must be driven externally (Manual Section 9.1.4)
	GPIOA->CRL &= ~GPIO_CRL_MODE2; //reset MODE1 - input mode
	GPIOA->CRL &= ~GPIO_CRL_CNF2; //reset CNF1_1 and CNF1_0
	GPIOA->CRL |= GPIO_CRL_CNF2_1; //set CNF1_1 - input mode, pull-up/down
	*/
	
  //Set up A3 as input
	GPIOA->CRL &= ~GPIO_CRL_MODE3;
	GPIOA->CRL &= ~GPIO_CRL_CNF3; //reset MODE1 and CNF1 (A1 pin)
	GPIOA->CRL |= GPIO_CRL_CNF3_1; //input with push/pull
	
	/*
	//Setup GPIO A3 as input for interrupt
	//Note: For alternate function inputs, the port must be configured in Input mode (floating, pullup or pull-down) and the input pin must be driven externally (Manual Section 9.1.4)
	GPIOA->CRL &= ~GPIO_CRL_MODE3; //reset MODE1 - input mode
	GPIOA->CRL &= ~GPIO_CRL_CNF3; //reset CNF1_1 and CNF1_0
	GPIOA->CRL |= GPIO_CRL_CNF3_1; //set CNF1_1 - input mode, pull-up/down
	
	//Setup GPIO A4 as input for interrupt
	//Note: For alternate function inputs, the port must be configured in Input mode (floating, pullup or pull-down) and the input pin must be driven externally (Manual Section 9.1.4)
	GPIOA->CRL &= ~GPIO_CRL_MODE4; //reset MODE1 - input mode
	GPIOA->CRL &= ~GPIO_CRL_CNF4; //reset CNF1_1 and CNF1_0
	GPIOA->CRL |= GPIO_CRL_CNF4_1; //set CNF1_1 - input mode, pull-up/down
	*/
	
	// Rest of pins below...
}


/* 
Setup Interrupts
JN 20200804
*/
void configureExternalInterrupts()
{
	//View definition of AFIO struct in stm32f10x.h
	//There are four EXTICR arrays (index 0 to 3)
	//16 EXTI blocks -> select which IO bank to use A-G for each EXTI block
	//First clear all registers, then set PA1 as interrupt pin
	AFIO->EXTICR[0] = 0;
	AFIO->EXTICR[1] = 0;
	AFIO->EXTICR[2] = 0;
	AFIO->EXTICR[3] = 0;
	
	
	//select PA1 as interrupt pin - can only have interrupt on one port in an input bank at one time
	//eg. if PA1 is an interrupt pin, PB1 cannot be used as an interrupt because they are multiplexed to the same EXTI channel. But can use PB2.
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PA;
	
	//also set up PA2 - PA4 as alternate function - interrupt pins
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI2_PA;
	
	/*
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PA;
	AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI4_PA;
	*/
	
	//Set up desired edge detection	
	EXTI->RTSR |= EXTI_RTSR_TR1; // Interrupt on rising edge for PA1
	EXTI->FTSR &= ~(EXTI_FTSR_TR1); // Don't interrupt on falling edge
		
	EXTI->RTSR &= ~(EXTI_RTSR_TR2); // Don't interrupt on rising edge for PA2
	EXTI->FTSR |= EXTI_FTSR_TR2; // Interrupt on falling edge
	
	/*
	EXTI->RTSR |= EXTI_RTSR_TR3; //Interrupt on rising edge for PA3
	EXTI->FTSR &= ~(EXTI_FTSR_TR3); //Don't interrupt on falling edge
	
	EXTI->RTSR |= EXTI_RTSR_TR4; //Interrupt on rising edge for PA4
	EXTI->FTSR &= ~(EXTI_FTSR_TR4); //Don't interrupt on falling edge
	*/
	
	//Write 1 to the corresponding bit in the external interrupt mask register to enable interrupt
	EXTI->IMR |= EXTI_IMR_MR1;
	EXTI->IMR |= EXTI_IMR_MR2;
	
	/*
	EXTI->IMR |= EXTI_IMR_MR3;
	EXTI->IMR |= EXTI_IMR_MR4;
	*/
	
	// Enable the NVIC interrupt for EXTI1 at minimum priority (priority levels depend on ARM CMSIS Core architecture).
	// Pass in interrupt number (IRQn) and priority 
	// note that if want to use interrupts on pins 5 and higher they are not directly connected to the NVIC EXTI
	// they are nested and grouped into two groups: pins 5-9/10-15
	NVIC_SetPriority(EXTI1_IRQn, 0x03);
	NVIC_EnableIRQ(EXTI1_IRQn);	
	
	NVIC_SetPriority(EXTI2_IRQn, 0x03);
	NVIC_EnableIRQ(EXTI2_IRQn);	
	
	/*
	NVIC_SetPriority(EXTI3_IRQn, 0x03);
	NVIC_EnableIRQ(EXTI3_IRQn);	
	
	NVIC_SetPriority(EXTI4_IRQn, 0x03);
	NVIC_EnableIRQ(EXTI4_IRQn);	
	*/
}


/* 
Setup timers and timer interrupts 
JN 20200813
*/
void setupTimers(void){
	//Enable Timer 2 (General Purpose Timer)
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	//Enable timer interrupt - no need to for EXTI mask here because this is an internal interrupt
	NVIC_SetPriority(TIM2_IRQn, 0x03);
	NVIC_EnableIRQ(TIM2_IRQn);
}


/* 
Start a timer 
- basic timer that counts up to an auto-reload value
- triggers an interrupt upon auto-reload
- code from https://vivonomicon.com/2018/05/20/bare-metal-stm32-programming-part-5-timer-peripherals-and-the-system-clock/ 
*/
void startTimer(TIM_TypeDef* TIMx, uint32_t ms)
{
	uint32_t prescaler = 7200;
	uint32_t tickspersec = core_clock_hz/prescaler; //10,000 tickspersec
	
	uint32_t msToTicksPerSec = tickspersec/1000; // 10 ticks in one ms
	
	uint16_t arrVal = ms*msToTicksPerSec;
	
	//First make sure timer is off
	TIMx->CR1 &= ~(TIM_CR1_CEN);
	
	//Reset timer
	if(TIMx == TIM2)
	{
		RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
		RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM2RST);
	}
	
	//Set up the timer prescaler and autoreload values
	TIMx->PSC = prescaler; //prescaler is only 16 bits (65535)
	TIMx->ARR = arrVal; //ARR is 16 bits (16 bit counter)
	
	// Send an update event to reset the timer and apply settings
	TIMx->EGR  |= TIM_EGR_UG;
  // Enable the hardware interrupt.
  TIMx->DIER |= TIM_DIER_UIE;
  // Enable the timer.
  TIMx->CR1  |= TIM_CR1_CEN;
}


/* 
Set up PWM on a pin from timer 3
- code based off https://github.com/fcayci/stm32f1-bare-metal/blob/master/pwm/pwm.c
*/
void setupPWM(void){

	//Setup GPIO A6 as Alternate Function output (0b1001 - 10MHZ Output, alternate function push-pull)
	GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6); //reset MODE2 and CNF2 (output mode, push-pull default)
	GPIOA->CRL |= (GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6_0); //output mode - alternate function, max spd 10MHz 
	
	//Enable Timer 3 (General Purpose Timer)
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	//Map TIM3 Channel 1 to Pin A6 (code not needed, but possible to change mapping)
	AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_NOREMAP;
	
	//First make sure timer is off
	TIM3->CR1 &= ~(TIM_CR1_CEN);
	
	// Set prescaler for timer clock
	TIM3->PSC = 7200; //7200;
	// Set freq to 100Hz by setting count up value to 100
	// Set freq to 50Hz by setting count up to 200
	TIM3->ARR = 200; //100; 
	// Set duty cycle
	TIM3->CCR1 = duty; //set compare/capture register 1 to 50
	
	// Enable Capture/Compare 4 interrupt (in sample code they adjust duty cycle on interrupt)
	// TIM3->DIER = (1 << 4);

	// Select pwm
	// Channel preload enable and PWM mode 1 (edge aligned) for CH1
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE; //Preload enable for CH1
	TIM3->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); //PWM Mode 1 - OCM1 value 0b110 (edge aligned)

	// Timer preload enable
	TIM3->CR1 |= TIM_CR1_ARPE; 

	// Enable capture/compare 1 output and polarity active-high in capture/compare enable register
	TIM3->CCER |= TIM_CCER_CC1E;

	// Enable Main Output
	TIM3->BDTR |= TIM_BDTR_MOE;

	//enable_interrupt(TIM1_CC_IRQn);
		
	// Send an update event to reset the timer and apply settings
	TIM3->EGR  |= TIM_EGR_UG;
	
	// Finally enable TIM3 module
	TIM3->CR1 |= TIM_CR1_CEN;
	
}


/* Set up ADC
JN 20200813
*/
void setupADC(void)
{
	//ADC clock can only be 14 MHz max
	//ADCCLK is synchronous with the APB clock (see RCC section in manual)
	//Divide system core clock by 6 to make it 12 MHz
	RCC->CFGR &= ~(RCC_CFGR_ADCPRE); // Reset prescaler bits
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6; //Divide by 6
		
	//Enable ADC clock from RCC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	//Select pin A4 as analog input
	GPIOA->CRL &= ~GPIO_CRL_MODE4; //reset MODE1 - input mode
	GPIOA->CRL &= ~GPIO_CRL_CNF4; //reset CNF1_1 and CNF1_0 - analog mode
	
	// set sample time for ch 0 to 28.5 cycles (0b011)
  ADC1->SMPR2 |= ( ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_0);
	
	//Wake up ADC
	ADC1->CR2 |= ADC_CR2_ADON;
	
	//Select input channel
	// First, set the number of channels to read during each sequence.
	// (# of channels = L + 1, so set L to 0)
	ADC1->SQR1  &= ~(ADC_SQR1_L);
	// Configure the first (and only) conversion in the sequence to read channel 4.
	ADC1->SQR3  &= ~(ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_2 | ADC_SQR3_SQ1_2 | ADC_SQR3_SQ1_3 | ADC_SQR3_SQ1_4); //clear out sequence 1
	ADC1->SQR3  |=  (ADC_SQR3_SQ1_2); //channel 4

	//Enable continuous conversion
	ADC1->CR2 |= ADC_CR2_CONT;

	//Enable End of Conversion (EOC) interrupt
	ADC1->CR1 |= ADC_CR1_EOCIE;

	//Enable interrupt
	NVIC_SetPriority(ADC1_2_IRQn, 0x03);
	NVIC_EnableIRQ(ADC1_2_IRQn);

	//Calibration reset and start
	//Before starting calibration ADC must have been on for at least 2 cycles
	//Optional for better accuracy.
	ADC1->CR2 |= ADC_CR2_RSTCAL;
	while(ADC1->CR2 & ADC_CR2_RSTCAL);
	ADC1->CR2 |= ADC_CR2_CAL;
	while(ADC1->CR2 & ADC_CR2_CAL);

	//Enable A/D conversion (ADC conversion starts when ADON value is 1 and a 1 is written to it)
	ADC1->CR2 |= ADC_CR2_ADON;
	
}

/*
Setup UART
- 
*/
void setupUART(void)
{
	// Enable clock for UART1 clock. Bit 14 in RCC APB2ENR register
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	// Blue pill STM32 - what are RX TX tied to? Pin 9 and 10
	// Make GPIOA Pin 9,10 (PA9, PA10) alternate-function output (0b1010)
	// Pin 9: TX
	// Pin 10: RX
	// push pull Alternate Function output, max speed 2 MHz
	GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9 | GPIO_CRH_MODE10 | GPIO_CRH_CNF10); //clear bits first
	//- Pin 9 - output, max speed 2MHz, Alternate function push pull
	//- Pin 10 - floating input instead of Alternate Function Output
	GPIOA->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF10_0);                
	
	// Baud rate
	// How to determine BRR: System Core Clock/(16*BRR) = baud rate
	// BRR should be 468.75 for 9600 baud rate (if clock is 72 MHz)
	// Thus matissa is 468 (0x1d4), and the fraction is 12 (0xc) (12/16 is .75) 
	// - Notes: Mantissa is the number left of the decimal point. The Fraction is expressed out of 16. (so 0.75 = 3/4 = 12/16).
	// BRR register: Bits 15:4 are mantissa, Bits 0:3 are fraction
	// Therefore BRR should be [mantissa >= 0b111010100/1100 <= fraction] which in hex is 0x1d4c
	USART1->BRR = 0x1d4c;
	
	// Enable USART
	USART1->CR1 |= USART_CR1_UE;
	// Word length - leave default (8 data)
	USART1->CR1 &= ~(USART_CR1_M); // (8 data bits = 0)
	// Number of stop bits - leave default (1 stop bit = 00)
	USART1->CR2 &= ~(USART_CR2_STOP);

	
	// Transmitter enable
	USART1->CR1 |= USART_CR1_TE;
	// Receiver enable
	USART1->CR1 |= USART_CR1_RE;
	
	// Transmit instructions
	// 1) write to USART1->DR
	// 2) wait until TC is 1 before writing again (bit 6 in SR register)
	
	// Receive instructions
	// 1) wait until RXNE bit set
	// 2) read from RDR
	
}



// INTERRUPT SERVICE ROUTINES BELOW ***********************************************************************************************
/* 
Interrupt Handler for Pin A1 interrupt
- declaration of interrupt service routine EXTI1_IRQHandler is found in startup_stm32f10x_md.s - MUST MATCH NAME in vector table
- flash an LED
*/

void EXTI1_IRQHandler(void) { 
  if (EXTI->PR & EXTI_PR_PR1) { //if interrupt was casued by bit 0
    // Clear the EXTI status flag. (write 1 into it to clear)
    EXTI->PR |= EXTI_PR_PR1;

		/*
		if (button1_on == 0)
		{
			while( !( USART1->SR & USART_SR_TXE ) ) {};
			USART1->DR = STX;
			while( !( USART1->SR & USART_SR_TXE ) ) {};
			USART1->DR = 'A';
			while( !( USART1->SR & USART_SR_TXE ) ) {};
			USART1->DR = ETX;
			button1_on = 1;
		}
		*/
		
		/*		
		if(tim2InterruptOccurred == 1)
		{
			duty = 15;
			TIM3->CCR1 = duty;
			
			setPinHigh();
			
			tim2InterruptOccurred = 0;
		}
		*/
  }
}

void EXTI2_IRQHandler(void) { 
  if (EXTI->PR & EXTI_PR_PR2) { //if interrupt was casued by bit 0
    // Clear the EXTI status flag. (write 1 into it to clear)
    EXTI->PR |= EXTI_PR_PR2;

		/*
		if (button1_on == 1)
		{
			while( !( USART1->SR & USART_SR_TXE ) ) {};
			USART1->DR = STX;			
			while( !( USART1->SR & USART_SR_TXE ) ) {};
			USART1->DR = 'B';
			while( !( USART1->SR & USART_SR_TXE ) ) {};
			USART1->DR = ETX; 
			button1_on = 0; //for debouncing 
		}
		*/
		
		/*
		if(tim2InterruptOccurred == 1)
		{
			duty = 10;
			TIM3->CCR1 = duty;
			
			setPinLow(); //debug
			
			tim2InterruptOccurred = 0;
		}
		*/
		
  }
}

/*
void EXTI3_IRQHandler(void) { 
  if (EXTI->PR & EXTI_PR_PR3) { //if interrupt was casued by bit 0
    // Clear the EXTI status flag. (write 1 into it to clear)
    EXTI->PR |= EXTI_PR_PR3;
    // Toggle the global 'led on?' variable.
		
		while( !( USART1->SR & USART_SR_TXE ) ) {};
    USART1->DR = 255;
		USART1->DR = 0x03;
			
  }
}

void EXTI4_IRQHandler(void) { 
  if (EXTI->PR & EXTI_PR_PR4) { //if interrupt was casued by bit 0
    // Clear the EXTI status flag. (write 1 into it to clear)
    EXTI->PR |= EXTI_PR_PR4;
    // Toggle the global 'led on?' variable.

		while( !( USART1->SR & USART_SR_TXE ) ) {};
    USART1->DR = 255;
		USART1->DR = 0x04;
  }
}
*/

/* 
Interrupt Handler for TIM2 interrupt
- declaration of interrupt service routine TIM2_IRQHandler is found in startup_stm32f10x_md.s - MUST MATCH NAME in vector table
- flash an LED
*/
void TIM2_IRQHandler(void){
	if (TIM2->SR & TIM_SR_UIF) {
    TIM2->SR &= ~(TIM_SR_UIF);
		
		delay++;
		
		tim2InterruptOccurred = 1;
		
		/*
		if(led_on == 1)
			setPinLow(); //debug
		else
			setPinHigh();
		*/
		
	}
}


/*
// Interrupt Handler for ADC done conversion interrupt
// - set PWM freq to match ADC input
void ADC1_2_IRQHandler(void){ 
	//adc_data = (0xFFF & ADC1->DR); //ADC data stored in first 12 bits of DR register - range: 0 to 4095
	//duty = adc_data*100/4096;
	duty = (ADC1->DR)*100/4096;
		
	//Change timer duty to match
	TIM3->CCR1 = duty; //set compare/capture register 1 to 50

}
*/

// Other functions --------------------------------------------------------------------
/*
Set a Pin
- set Bit Set register
- reset Bit Reset register
*/
void setPinHigh()
{
	GPIOA->BSRR &= ~GPIO_BSRR_BR0; //unset BR0 (bit reset register)
	GPIOA->BSRR |= GPIO_BSRR_BS0; //set BS0 (bit set register)
	led_on = 1;
}

/*
Reset a Pin
- reset Bit Set register
- set Bit Reset register
*/
void setPinLow()
{
	GPIOA->BSRR &= ~GPIO_BSRR_BS0; //unset BS0
	GPIOA->BSRR |= GPIO_BSRR_BR0; //set BR0
	led_on = 0;
}

