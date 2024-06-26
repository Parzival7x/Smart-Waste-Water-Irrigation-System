#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"

char adcValue;
char adcValue1;
char motorstatus;
uint32_t PosedgeCapturedValue=0x00, NegedgeCapturedValue=0x00;

uint32_t TimePeriodCalulated=0x00;

float DistanceCalculated=0.0;

//uint32_t milliseconds=0x00; uint32_t dummy=0x00;

uint8_t posedge=0, togg=0;
	
unsigned int i=0;

	
int x=1;

/*****UART2 Code Starts Here********************/

/*==== 16MHz sys clock and peripheral ===*/
void SysClockConfig (void)
{
	#define PLL_M 	4
	#define PLL_N 	168
	#define PLL_P	0 //PLL_P = 2
	//1. ENABLE HSE and wait for the HSE to become Ready
	RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY));
	
	//2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;
	
	//3. Configure the FLASH PREFETCH and the LATENCY Related Settings
	FLASH->ACR |= FLASH_ACR_ICEN|FLASH_ACR_DCEN|FLASH_ACR_PRFTEN|FLASH_ACR_LATENCY_5WS;
	
	//4. Configure the PRESCALARS HCLK,PCLK1,PCLK2
	//AHB PR
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	
	//APB1 PR
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
	
	//APB2 PR
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
	
	//5.Configure Main PLL
	RCC->PLLCFGR = (PLL_M<<0)|(PLL_N<<6)|(PLL_P<<16)|(RCC_PLLCFGR_PLLSRC_HSE);
	
	//6.Enable the PLL and wait for it to become ready
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));
	
	//7.Select the Clock source and wait for it to be set
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

/*=====USART 9600, 1 start bit, 1 stop bit, 8 data bit, 0 parity bit ====*/
/* UART4 is connected in PC10,11 */
void UART_config(void)
{
	//Enable UART clock and GPIO clock
	RCC->APB1ENR |= (1<<17); //UART2 CLOCK
	RCC->AHB1ENR |= (1<<0);  //GPIOA CLOCK
	
	//GPIOA Alternate function
	GPIOA->MODER &= 0XFFFFFF0F;
	GPIOA->MODER |= (2<<2*2); //Alternate Function PA2 USART2 Tx
	GPIOA->MODER |= (2<<2*3);	//Alternate Function PA3 USART2 Rx

	//GPIOA->OSPEEDR |= (3<<4)|(3<<6); //High Speed for Pins PA2 and PA3
	GPIOA->AFR[0] |= (7<<2*4);  //AF7 USART2 at PA2
	GPIOA->AFR[0] |= (7<<3*4);  //AF7 USART2 at PA3
	
	//Select the 9600 baud rate 42Mhz
	USART2->BRR = 0x1117;
	
	//Enable UART2
	/*USART2->CR1 = 0x00; //Clear all
	USART2->CR1=0X200C;
	USART2->CR1 |= 0x0020;
	NVIC_EnableIRQ(USART2_IRQn); // Enable IRQ for UART4 in NVIC */
	
	USART2->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE; // Enable receiver and RXNE interrupt
  NVIC_EnableIRQ(USART2_IRQn); // Enable USART2 interrupt in NVIC

  USART2->CR1 |= USART_CR1_UE;
}

int32_t SER_GetChar (void){
	#ifdef __DBG_ITM
if(ITM_CheckChar())
	return ITM_ReceiveChar();
#else
if(USART2->SR&0X0020)
	return (USART2->DR);

	#endif

	return(-1);
}

int rxdata;

void USART2_IRQHandler(void)   {   
  // Receive RX IRQ part 
	// Store byte in buffer
	
	rxdata=SER_GetChar();
	
	if(rxdata==1 || rxdata ==2|| rxdata==3)
	{
		motorstatus=rxdata;
	}
	
}






void GPIOSetupTriggerPin(void) // Trigger PA07
{
	RCC->AHB1ENR |= (1UL<<0);//GPIOCEN : Clock enabled in port A
	GPIOA->MODER |= (1UL<<14); // PA07 as Output
	GPIOA->BSRR=0xFFFF0000; // reset LED
}

void DistLed(void) // LED PD12
{
	RCC->AHB1ENR |= (1UL<<3);//GPIODEN : Clock enabled in port D
	GPIOD->MODER |= (1UL<<24); // PD12 as Output
	GPIOD->MODER &= ~(1UL<<25);
	GPIOD->BSRR=0xFFFF0000; // reset LED
}

void motor1(void)	//LED1 PD13 MOTOR1 PB11
{
	RCC->AHB1ENR |= (1UL<<3);//GPIODEN : Clock enabled in port D
	GPIOD->MODER |= (1UL<<26); // PD13 as Output
	GPIOD->MODER &= ~(1UL<<27);
	GPIOD->BSRR=0xFFFF0000; // reset LED
	
	RCC->AHB1ENR |= (1UL<<1);//GPIOBEN : Clock enabled in port B for MOTOR
	GPIOB->MODER |= (1UL<<22); // PB11 as Output
	GPIOB->MODER &= ~(1UL<<23);
	GPIOB->BSRR=0x0000FFFF; 
}

void motor2(void)  //LED2 PD14 MOTOR2 PB12
{
	RCC->AHB1ENR |= (1UL<<3);//GPIODEN : Clock enabled in port D
	GPIOD->MODER |= (1UL<<28); // PD12 as Output
	GPIOD->MODER &= ~(1UL<<29);
	GPIOD->BSRR=0xFFFF0000; // reset LED
	
	RCC->AHB1ENR |= (1UL<<1);//GPIOBEN : Clock enabled in port B for MOTOR
	GPIOB->MODER |= (1UL<<24); // PB12 as Output
	GPIOB->MODER &= ~(1UL<<25);
	GPIOB->BSRR=0x0000FFFF;
}

void aquaguardoff(void)  // AQUA OFF PD15
{
	RCC->AHB1ENR |= (1UL<<3);//GPIODEN : Clock enabled in port D
	GPIOD->MODER |= (1UL<<30); // PD12 as Output
	GPIOD->MODER &= ~(1UL<<31);
	GPIOD->BSRR=0xFFFF0000; // reset LED
}

/*Initialise timer TIMER5 CH1 -> PA15 as - pin*/
void init_TIMER(void){
	//Config alternate function @PA15
	RCC->AHB1ENR |= 1<<0; //Enable clock in GPIO A
	GPIOA->MODER |= 2<<12; //alternate fn chose
	GPIOA->AFR[1] |= 1<<28; // AF1 is timer function
	
	RCC->APB1ENR |= 1<<0; // TIM2 clock supply enabled
	TIM2->PSC = 16-1; // 16Mhz/16 -> 1Mhz clock source
	
	TIM2->CCMR1 |= 1<<0; // channel 1 as IC1 mapped on TI1
	TIM2->DIER |= 1<<1; // Capture interrupt enable
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->CCER |= 1<<0 | 1<<1 | 1<<3; //enable capture, 11-> both edge
	TIM2->CR1 |= 1<<0; //Counter enable
}




void TIM2_IRQHandler (void){
	if(posedge==0){PosedgeCapturedValue=TIM2->CCR1;posedge=1;}
	else {NegedgeCapturedValue=TIM2->CCR1;posedge=0;}
	togg=~togg;
}


int main(void){
	SysClockConfig();
	UART_config();
	GPIOSetupTriggerPin();
	DistLed();
	motor1();
	motor2();
	aquaguardoff();
	init_TIMER();
	
	unsigned int i=0;
	
	GPIOA->ODR &= ~(1<<7); //reset 1 to TRIGGER
	//for(unsigned int i=0;i<32000;i++);
	int x=1;
	
	GPIOB->BSRR |=(1<<11);
	GPIOB->BSRR |=(1<<12);
	
	while(1){
		//trigger part
		//adcValue = uartRxBuffer[0];
		//adcValue1 = uartRxBuffer[1];	
		if(x==1)
		{
		GPIOA->ODR &= ~(1<<7); //reset 1 to TRIGGER
		//for(unsigned int i=0;i<32000;i++);
		GPIOA->ODR |= 1<<7; //set 1 to TRIGGER
		for(unsigned int i=0;i<10;i++);//while(TIM3->CNT - mills < 10); //delay for 10uS
		GPIOA->ODR &= ~(1<<7); //reset 1 to TRIGGER
		x=0;
		}
		//trigger part end
		
		if(NegedgeCapturedValue>PosedgeCapturedValue)
			TimePeriodCalulated=(NegedgeCapturedValue-PosedgeCapturedValue);
		else 
			TimePeriodCalulated=((int32_t)0xFFFFFFFF - PosedgeCapturedValue)+NegedgeCapturedValue;
		
		DistanceCalculated = TimePeriodCalulated/1000; //(TimePeriodCalulated * 0.034) / 2;
		
		//motor control loop	
			
			if(motorstatus==1)	//adcValue = PA0; adcValue1 = PA1
			{
				GPIOD->ODR |=(1<<13);
				GPIOD->ODR &=~(1<<14);
				GPIOD->ODR &=~(1<<15);
				
				GPIOB->BSRR |=(1<<27);
				GPIOB->BSRR |=(1<<12);
				for(i=0;i<320000;i++);
			}
			else if(motorstatus==2)
			{
				GPIOD->ODR &=~(1<<13);
				GPIOD->ODR |=(1<<14);
				GPIOD->ODR &=~(1<<15);
				
				GPIOB->BSRR |=(1<<11);
				GPIOB->BSRR |=(1<<28);
				for(i=0;i<320000;i++);
			}
			else
			{
				GPIOD->ODR &=~(1<<13);
				GPIOD->ODR &=~(1<<14);
				GPIOD->ODR |=(1<<15);
				
				GPIOB->BSRR |=(1<<11);
				GPIOB->BSRR |=(1<<12);
				for(i=0;i<320000;i++);
			}	
		
			GPIOB->BSRR |=(1<<11);
			GPIOB->BSRR |=(1<<12);
		x=1;
	}
	return(0);
}