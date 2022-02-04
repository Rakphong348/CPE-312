/*Base register adddress header file*/
#include "stm32l1xx.h"
/*Library related header files*/
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_tim.h"
#include <string.h>
#include "stm32l1xx_ll_lcd.h"
#include "stm32l152_glass_lcd.h"
#include "stdio.h"

void SystemClock_Config(void);

void USER_GPIO_Config(void);

void HCSR04_GPIO_Config(void);

void L293D_GPIO_Config(void);
void MOTOR_TIM_BASE_Config(void);
void MOTOR_TIM_OC_GPIO_Config(void);
void MOTOR_TIM_OC_Config(void);
void Motor_Config_LEFT(void);
void Motor_Config_RIGHT(void);
void Motor_Config_STOP(void);

void SOUND_TIM_BASE_DurationConfig(void);
void SOUND_TIM_BASE_Config(uint16_t);
void SOUND_TIM_OC_GPIO_Config(void);
void SOUND_TIM_OC_Config(uint16_t,float);
void TIM4_IRQHandler(void);
void EXTI0_IRQHandler(void);
/* For 0.01 s update event */
#define TIMx_PSC    3200
#define TIMx_ARR    100

int test=0;

uint16_t rise_timestamp = 0;
uint16_t fall_timestamp = 0;
uint16_t up_cycle = 0;

uint8_t state = 0;
float period = 0;
float distance = 0;
float distanceCM = 0;

uint32_t TIM2CLK;
uint32_t PSC;

int motor_state = 0;

#define E_O6					(uint16_t)1318
#define MUTE					(uint16_t) 1
#define ARR_CALCULATE(N) ((32000000) / ((TIMx_PSC) * (N)))
float sound_ctl = 0.5;


int main()
{
	SystemClock_Config();
	 MOTOR_TIM_OC_Config();
	  //GPIO_Config_G1293d();
	 Motor_Config_LEFT();
	//Motor_Config_STOP();
	HCSR04_GPIO_Config();
	 USER_GPIO_Config();
		NVIC_SetPriority(EXTI0_IRQn, 1);
	NVIC_EnableIRQ(EXTI0_IRQn);
	//LL_TIM_EnableIT_CC1(TIM3);*/
		

	
	while(1)
	{
		
		
		switch(state)
			{
				case 0:
					//Trigger measurement
					LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2);
					LL_mDelay(1);
					LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);
					state = 1;
				break;
				
				case 1:
					if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_1))
					{
						rise_timestamp = LL_TIM_GetCounter(TIM2);
						state = 2;
					}
				break;
					
				case 2:
					if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_1) == RESET)
					{
						fall_timestamp = LL_TIM_GetCounter(TIM2);
						//Calculate uptime
						if(fall_timestamp > rise_timestamp)
						{
							up_cycle = fall_timestamp - rise_timestamp;
						}
						else if(fall_timestamp < rise_timestamp)
						{
							up_cycle = (LL_TIM_GetAutoReload(TIM2) - rise_timestamp) + fall_timestamp + 1; 
						}
						else
						{
							//cannot measure at this freq
							up_cycle = 0;
						}
						
						if(up_cycle != 0)
						{
							PSC = LL_TIM_GetPrescaler(TIM2) + 1;
							TIM2CLK = SystemCoreClock / PSC;
							
							period = (up_cycle*(PSC) * 1.0) / (TIM2CLK * 1.0); //calculate uptime period
							distance = (period * 350) / 2; //meter unit
							distanceCM = distance*100;
						
						
							if (distanceCM < 5 && motor_state == 0){
								Motor_Config_STOP();
									LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0);
									motor_state = 1;
									SOUND_TIM_OC_Config(ARR_CALCULATE(E_O6),sound_ctl);
							}
								else if (distanceCM < 9 && motor_state == 0){
									sound_ctl = 0.19;
									SOUND_TIM_OC_Config(ARR_CALCULATE(E_O6),sound_ctl);
							}
							else if(distanceCM >= 5 && LL_GPIO_IsInputPinSet(GPIOA,LL_GPIO_PIN_0))
							{
								Motor_Config_RIGHT();
								SOUND_TIM_OC_Config(ARR_CALCULATE(MUTE),sound_ctl);
				
							}
						
						}
						state = 0;
					}
				break;
			}

	
	}
	
}
void EXTI0_IRQHandler(void){
	
/* if(LL_GPIO_IsInputPinSet(GPIOA,LL_GPIO_PIN_0))
    {
		

	
}*/
}

////////////////////////////////////////////////////////////
///FOR MOTOR //
//use TIM3 PB4 PB5 PB7 //
void MOTOR_TIM_BASE_Config(void){
	
	//USE TIM3 for TIME_BASE for MOTOR
  LL_TIM_InitTypeDef timbase_initstructure;

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3); 

  timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV2;
  timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
  timbase_initstructure.Autoreload = TIMx_ARR - 1;
  timbase_initstructure.Prescaler = TIMx_PSC - 1;

  LL_TIM_Init(TIM3, &timbase_initstructure);
	
}
void MOTOR_TIM_OC_GPIO_Config(void){
  LL_GPIO_InitTypeDef l293d_initstruct;
	
  /* Enable GPIOB clock */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /* Config PB5 and PB4 as alternate function (TIM3_CH2) */
  l293d_initstruct.Mode = LL_GPIO_MODE_ALTERNATE;
  l293d_initstruct.Alternate = LL_GPIO_AF_2;
  l293d_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  l293d_initstruct.Pull = LL_GPIO_PULL_NO;
	l293d_initstruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

  l293d_initstruct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_4; 
  LL_GPIO_Init(GPIOB, &l293d_initstruct);
		
}

void MOTOR_TIM_OC_Config(void){
  LL_TIM_OC_InitTypeDef tim_oc_initstructure;

  MOTOR_TIM_BASE_Config();
  MOTOR_TIM_OC_GPIO_Config(); 
	L293D_GPIO_Config(); ////for PIN4 is H , PIN5 L and PIN7 is EN


  tim_oc_initstructure.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1;
	tim_oc_initstructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	tim_oc_initstructure.CompareValue = TIMx_ARR; //20% duty cycle
	//tim_oc_initstructure.CompareValue = TIMx_ARR * 0.4; //40% duty cycle
	//tim_oc_initstructure.CompareValue = TIMx_ARR * 0.6; //60% duty cycle
	//tim_oc_initstructure.CompareValue = TIMx_ARR * 0.8; //80% duty cycle
	//tim_oc_initstructure.CompareValue = TIMx_ARR ; //100% duty cycle


  /*PB4 -> (TIM3_CH1) */
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &tim_oc_initstructure);
	LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &tim_oc_initstructure);

  /*//Interrupt Configure
	NVIC_SetPriority(TIM3_IRQn, 1);
	NVIC_EnableIRQ(TIM3_IRQn);
	LL_TIM_EnableIT_CC1(TIM3);*/

  /*Start Output Compare in PWM Mode and Open LED*/
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_EnableCounter(TIM3);
}


void L293D_GPIO_Config(void)
{
	//for active MOTOR
	  LL_GPIO_InitTypeDef l293d_initstruct;
	
    //Move LEFT//
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
		l293d_initstruct.Mode = LL_GPIO_MODE_OUTPUT;
    l293d_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    l293d_initstruct.Pull = LL_GPIO_PULL_NO;
    l293d_initstruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    l293d_initstruct.Pin = LL_GPIO_PIN_4 |LL_GPIO_PIN_7; 
		LL_GPIO_Init(GPIOB, &l293d_initstruct);
	
		//Motor_Config_LEFT();
	
		//Move RIGHT//

	/*	l293d_initstruct.Mode = LL_GPIO_MODE_OUTPUT;
    l293d_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    l293d_initstruct.Pull = LL_GPIO_PULL_NO;
    l293d_initstruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    l293d_initstruct.Pin = LL_GPIO_PIN_5 |LL_GPIO_PIN_7 ;
    LL_GPIO_Init(GPIOB, &l293d_initstruct);
		
		Motor_Config_RIGHT();*/	
}

void Motor_Config_RIGHT(void){
	//Enable
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
	//PB4 is L
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);

}
void Motor_Config_LEFT(void){
	//Enable
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
	//PB5 is L
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
}
void Motor_Config_STOP(void){
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
}

///////////////////////////////////////////////////////////////
//FOR USER BUTTON //
//use PA0 //
void USER_GPIO_Config(void){
	
	//config PA0 for USER BUTTON
		LL_GPIO_InitTypeDef user_gpio;
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
		
		user_gpio.Mode = LL_GPIO_MODE_INPUT;
		user_gpio.Pull = LL_GPIO_PULL_NO;
		user_gpio.Pin = LL_GPIO_PIN_0;
		user_gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		user_gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		user_gpio.Alternate = LL_GPIO_AF_1;
		LL_GPIO_Init(GPIOA,&user_gpio);
	
}
///////////////////////////////////////////////////////////////
////////////////
//FOR ULTRA SONIC//
// USE PA1 PA2//
void HCSR04_GPIO_Config(void){
	
		//config PA1 and PA2 for UltraSonic
		LL_GPIO_InitTypeDef hcsr04_gpio;
	
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
		
		hcsr04_gpio.Mode = LL_GPIO_MODE_INPUT;
		hcsr04_gpio.Pin = LL_GPIO_PIN_1;
		hcsr04_gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		hcsr04_gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		LL_GPIO_Init(GPIOA, &hcsr04_gpio);
		
		hcsr04_gpio.Mode = LL_GPIO_MODE_OUTPUT;
		hcsr04_gpio.Pull = LL_GPIO_PULL_NO;
		hcsr04_gpio.Pin = LL_GPIO_PIN_2;
		hcsr04_gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		hcsr04_gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		LL_GPIO_Init(GPIOA, &hcsr04_gpio);
	
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
		LL_TIM_EnableCounter(TIM2);
	
}

////////////////////////////////////////
//FOR SPEAKER ONLY//
//USE TIM4 TIM2 PB6


void SOUND_TIM_BASE_Config(uint16_t ARR)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = ARR - 1;
	timbase_initstructure.Prescaler =  TIMx_PSC- 1;
	LL_TIM_Init(TIM4, &timbase_initstructure);
	
	LL_TIM_EnableCounter(TIM4);
}

void SOUND_TIM_OC_GPIO_Config(void)
{
	LL_GPIO_InitTypeDef gpio_initstructure;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	gpio_initstructure.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_initstructure.Alternate = LL_GPIO_AF_2;
	gpio_initstructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_initstructure.Pin = LL_GPIO_PIN_6;
	gpio_initstructure.Pull = LL_GPIO_PULL_NO;
	gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &gpio_initstructure);
	
	/*LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	
	gpio_initstructure.Mode = LL_GPIO_MODE_INPUT;
	gpio_initstructure.Pin = LL_GPIO_PIN_0;
	gpio_initstructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_initstructure.Pull = LL_GPIO_PULL_NO;
	gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	
	LL_GPIO_Init(GPIOA, &gpio_initstructure);*/

}

void SOUND_TIM_OC_Config(uint16_t note,float sctl)
{
	LL_TIM_OC_InitTypeDef tim_oc_initstructure;
	
	
	SOUND_TIM_OC_GPIO_Config();
	SOUND_TIM_BASE_Config(note);
	
	tim_oc_initstructure.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1;
	tim_oc_initstructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
//	tim_oc_initstructure.CompareValue = LL_TIM_GetAutoReload(TIM4); //100% duty
	tim_oc_initstructure.CompareValue = LL_TIM_GetAutoReload(TIM4) * sctl; //50% duty
	//tim_oc_initstructure.CompareValue = LL_TIM_GetAutoReload(TIM4) / 4 ;//25% duty
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &tim_oc_initstructure);
	

	/*Interrupt Configure*/
	NVIC_SetPriority(TIM4_IRQn, 1);
	NVIC_EnableIRQ(TIM4_IRQn);
	LL_TIM_EnableIT_CC1(TIM4);
	
	/*Start Output Compare in PWM Mode*/
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM4);
}

void TIM4_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_CC1(TIM4) == SET)
	{
		LL_TIM_ClearFlag_CC1(TIM4);
	}
}

///////////////////////////////////////////////////

void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}