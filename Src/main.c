/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim11;

typedef enum {STABILIZE, STOP, KEEPGOING} STATE;
STATE state;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t timems = 0;
uint8_t motorphase;
float acc;
float accAU;
float accStartPos = 0;
uint32_t accStartTime;
int32_t oldPWM = 0;
float accP;
float accI;
float accD;
float accIntegral = 0;
float angleP;
float angleI;
float angleD;
int32_t angleIntegral;
float posP;
float posD;
float v = 0;
float vnew = 0;
float angleShift = 0;
int16_t angle0 = 0;
int16_t angle1 = 0;
int16_t angleHistory [5] = {};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void print(char* string, ...);
void motorNsleep(int nsleep);
void motorNreset(int nreset);
void motorPhase(int phase);
void motorBrake(int on);
void motorLimit(float limit);
void LED(int n, int on);
void setAcc(float value);
void setAccAU(float value);
void setV(float value);
float getProjectedPosDiff();
float getProjectedPos();
void setPWM(int32_t value);
int16_t getAngle();
int32_t getPos();
void adjustPWM();
int32_t getPWM();
void setState(STATE newstate);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */

  //LED flash
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

  //LEDS
  LED(0,0);
  LED(1,0);
  //LED(2,0);
  LED(3,1);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);

  motorBrake(0);
  motorPhase(1);
  //current limit at 1.0 is 4.95A
  motorLimit(1);
  motorNreset(1);
  motorNsleep(1);
  setState(STOP);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim6);

  print("Welcome!\r\n");
  HAL_Delay(10);
  print("Starting...");
  HAL_Delay(150);

  setState(STABILIZE);
//  setState(KEEPGOING);
//  setV(0.2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  uint32_t count = 0;
  while (1)
  {
//	  print("Hey\r\n");
//	  print("Pos: %d\r\n", getPos());b

	  print("Angle: %3d; AccAU: %4.1f; acc: %.5f vnew: %.3f; v: %.3f; Time: %5d; Timestart: %5d; Timediff: %4d; StartPos: %6.2f; Pos: %4d, Proj. pos: %6.2f; PWM: %6d; Integral: %7.1f; AngleIntegral: %5d; Angle Der: %d; Angle shift: %4.1f\r\n", \
			  getAngle(), accAU, acc, vnew, v, timems, accStartTime, timems-accStartTime, accStartPos, getPos(), getProjectedPos(), getPWM(), accIntegral, angleIntegral, getAngle()-angleHistory[0], angleShift);
	  HAL_Delay(2);
//	  if(acc ==0)
//		  setAccAU(-10);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 25;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 9999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim5);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 89;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 18000;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 10000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim11);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED0_Pin|IO2_Pin|IO4_Pin 
                          |IO3_Pin|DECAY_Pin|NSLEEP_Pin|PHASE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IO1_Pin|IO0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DRVNRST_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED0_Pin IO2_Pin IO4_Pin 
                           IO3_Pin DECAY_Pin NSLEEP_Pin PHASE_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED0_Pin|IO2_Pin|IO4_Pin 
                          |IO3_Pin|DECAY_Pin|NSLEEP_Pin|PHASE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC15 PC6 PC7 PC8 
                           PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IO1_Pin IO0_Pin */
  GPIO_InitStruct.Pin = IO1_Pin|IO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DRVNRST_Pin LED3_Pin */
  GPIO_InitStruct.Pin = DRVNRST_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : NFAULT_Pin */
  GPIO_InitStruct.Pin = NFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NFAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB12 PB13 
                           PB14 PB15 PB3 PB4 
                           PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA13 
                           PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

float getProjectedPos(){
	return acc*(timems-accStartTime)*(timems-accStartTime)/2.0f + v*(timems-accStartTime) + accStartPos;
//	return acc;
}

float getProjectedPosDiff(){
	return (int32_t)(getProjectedPos() - accStartPos + 0.5f);
}

uint32_t count10 = 0;

void timeStep(){
	timems++;
	count10++;
	vnew += acc;
	if(count10==10){
		count10=0;
		angleP = 0.9f;
		angleI = 0.0045;//0.0065;
		angleD = 5;//3.25f;
		posP = 3.0f/150.0f;
		posD = 5.0f/1.0f;
		angleShift = posP*getPos();
		if(state == STABILIZE){
			setAccAU(angleP*(getAngle()+angleShift) + angleD*(getAngle()-angleHistory[0]) + angleI*angleIntegral - vnew*posD);
//			printf("AP: %.1f; AD: %.1f; AI: %.1f; PP: %.1f; PD: %.1f\r\n", angleP*getAngle(),angleD*(getAngle()-angleHistory[0]),angleI*angleIntegral,posP*getPos(),vnew*posD);
		}
		for(uint8_t i = 0; i < 5-1; ++i){
			angleHistory[i] = angleHistory[i+1];
		}
		angleHistory[4] = getAngle();
	}
	if(state == STABILIZE || state == KEEPGOING){
		angleIntegral += getAngle()+angleShift;
		adjustPWM();
		if(abs(getPos()) > 400){
			setState(STOP);
		}
	}
	if(state == STOP){
		setAccAU(0);
		setPWM(0);
		motorBrake(0);
	}
}

void setState(STATE newstate){
	state = newstate;
	if(newstate == STABILIZE || newstate == KEEPGOING){
		motorBrake(0);
	}
	if(newstate == STOP){
		motorBrake(1);
		setAcc(0);
		setPWM(0);
		setV(0);
	}
}

void adjustPWM(){
	accP = 1500;
	accI = 1;
	accD = 0;

	float error = getProjectedPos() - getPos();
	if((accI*accIntegral <= 10000 && error > 0) || (accI*accIntegral >= -10000 && error < 0))
		accIntegral += error;
	int32_t changePWM = accP*error + accI*accIntegral - vnew * accD;
	setPWM(changePWM);
}

void setAcc(float value){
	accAU = value/(gms*(2*PI/2400.0f));
	acc = value;
	accStartPos = getProjectedPos();
	accStartTime = timems;
	v = vnew;
}

void setAccAU(float value){
	accAU = value;
	acc = value*gms*(2*PI/2400.0f);
	accStartPos = getProjectedPos();
	accStartTime = timems;
	v = vnew;
}

void setV(float value){
	setAcc(0);
	v = value;
	vnew = value;
}

int32_t getPos(){
	return TIM2->CNT;
}

int16_t getAngle(){
	return TIM3->CNT;
}

int32_t PWMChangeLimit = 300;
//PWM value from 0 to 10k, which goes from 0% to 100%
void setPWM(int32_t value){
	if(value > 10000)
		value = 10000;
	if(value < -10000)
		value = -10000;
	uint32_t phasenew = 0;
	if(value >= 0)
		phasenew = 1;
	else
		value = -value;
	if(phasenew != motorphase)
		motorPhase(phasenew);
	if(value-oldPWM > PWMChangeLimit)
		value = oldPWM + PWMChangeLimit;
	else if(value-oldPWM < -PWMChangeLimit)
		value = oldPWM - PWMChangeLimit;
	TIM5->CCR4 = value;
	oldPWM = value;
}

int32_t getPWM(){
	return (motorphase ? 1 : -1)*(TIM5->CCR4);
}

void motorNsleep(int nsleep){
	HAL_GPIO_WritePin(NSLEEP_GPIO_Port, NSLEEP_Pin, nsleep);
}

void motorNreset(int nreset){
	HAL_GPIO_WritePin(DRVNRST_GPIO_Port, DRVNRST_Pin, nreset);
}

void motorPhase(int phase){
	HAL_GPIO_WritePin(PHASE_GPIO_Port, PHASE_Pin, phase);
	motorphase = phase;
}

void motorBrake(int on){
	HAL_GPIO_WritePin(DECAY_GPIO_Port, DECAY_Pin, on);
}

void motorLimit(float limit){
	uint8_t value = limit*((1<<5)-1);
	HAL_GPIO_WritePin(IO0_GPIO_Port, IO0_Pin, (value>>0)&1);
	HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, (value>>1)&1);
	HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, (value>>2)&1);
	HAL_GPIO_WritePin(IO3_GPIO_Port, IO3_Pin, (value>>3)&1);
	HAL_GPIO_WritePin(IO4_GPIO_Port, IO4_Pin, (value>>4)&1);
}

void LED(int n, int on){
	if(n==0){
		HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, !on);
	}
	else if(n==1){
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, !on);
	}
	else if(n==2){
//		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, !on);
	}
	else if(n==3){
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, !on);
	}
}

void print(char* string, ...){
	size_t maxsize = 256;
	size_t size;
	uint8_t buffer [maxsize];
	va_list args;
	va_start(args, string);
	size = vsnprintf(buffer, maxsize, string, args);
	va_end(args);

	if(size >= maxsize){
		uint8_t err [] = "\r\nPRINT BUFFER OVERFLOW\r\n";
		int count = 0;
		while(count < 10){
			volatile uint8_t res = CDC_Transmit_FS(err,strlen(err));
			if(res == USBD_BUSY){
				for(volatile int n = 0; n < 18000*2; n++){
					volatile int a = TIM2->CNT;
				}
			}
			else{
				break;
			}
			count++;
		}
	}
	int count = 0;
	while(count < 10){
		volatile uint8_t res = CDC_Transmit_FS(buffer,size);
		if(res == USBD_BUSY){
			for(volatile int n = 0; n < 18000*2; n++){
				volatile int a = TIM2->CNT;
			}
		}
		else{
			break;
		}
		count++;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
