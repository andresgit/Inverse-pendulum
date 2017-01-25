/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_14
#define LED0_GPIO_Port GPIOC
#define IO2_Pin GPIO_PIN_0
#define IO2_GPIO_Port GPIOC
#define IO4_Pin GPIO_PIN_1
#define IO4_GPIO_Port GPIOC
#define IO3_Pin GPIO_PIN_2
#define IO3_GPIO_Port GPIOC
#define DECAY_Pin GPIO_PIN_3
#define DECAY_GPIO_Port GPIOC
#define ENC21_Pin GPIO_PIN_0
#define ENC21_GPIO_Port GPIOA
#define ENC22_Pin GPIO_PIN_1
#define ENC22_GPIO_Port GPIOA
#define IO1_Pin GPIO_PIN_4
#define IO1_GPIO_Port GPIOA
#define IO0_Pin GPIO_PIN_5
#define IO0_GPIO_Port GPIOA
#define ENC11_Pin GPIO_PIN_6
#define ENC11_GPIO_Port GPIOA
#define ENC12_Pin GPIO_PIN_7
#define ENC12_GPIO_Port GPIOA
#define NSLEEP_Pin GPIO_PIN_4
#define NSLEEP_GPIO_Port GPIOC
#define PHASE_Pin GPIO_PIN_5
#define PHASE_GPIO_Port GPIOC
#define DRVNRST_Pin GPIO_PIN_0
#define DRVNRST_GPIO_Port GPIOB
#define NFAULT_Pin GPIO_PIN_1
#define NFAULT_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define PI 3.14159274
#define gms 3.1864069e-2
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
