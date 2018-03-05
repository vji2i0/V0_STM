/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */
#include "steps.h"
#include "SendMessage.h"
#include "DriverControl.h"
#include "Temperature.h"
#include "global_variables.h"


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;
extern DMA_HandleTypeDef hdma_usart2_rx;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 channel6 global interrupt.
*/
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
* @brief This function handles TIM1 break interrupt and TIM15 global interrupt.
*/
void TIM1_BRK_TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 0 */
  static int i = 0;
  static int steps_with_acc = 0;

  if ((state0 & NEW_TASK) && (translation_discret_length != 0)){

    if (translation_discret.x > 0){
      if (progress.x < i*translation_discret.x/translation_discret_length){
        progress.x += 1;
        location.x += 1.0/STEPS_PER_X;
        doStep(INC_X);
      }
    }else if (translation_discret.x < 0){
      if (progress.x > i*translation_discret.x/translation_discret_length){
        progress.x -= 1;
        location.x -= 1.0/STEPS_PER_X;
        doStep(DEC_X);
      }
    }
    if (translation_discret.y > 0){
      if (progress.y < i*translation_discret.y/translation_discret_length){
        progress.y += 1;
        location.y += 1.0/STEPS_PER_Y;
        doStep(INC_Y);
      }
    }else if (translation_discret.y < 0){
      if (progress.y > i*translation_discret.y/translation_discret_length){
        progress.y -= 1;
        location.y -= 1.0/STEPS_PER_Y;
        doStep(DEC_Y);
      }
    }
    if (translation_discret.z > 0){
      if (progress.z < i*translation_discret.z/translation_discret_length){
        progress.z += 1;
        location.z += 1.0/STEPS_PER_Z;
        doStep(INC_Z);
      }
    }else if (translation_discret.z < 0){
      if (progress.z > i*translation_discret.z/translation_discret_length){
        progress.z -= 1;
        location.z -= 1.0/STEPS_PER_Z;
        doStep(DEC_Z);
      }
    }
    if (translation_discret.e > 0){
      if (progress.e < i*translation_discret.e/translation_discret_length){
        progress.e += 1;
        location.e += 1.0/STEPS_PER_E;
        doStep(INC_E);
      }
    }else if (translation_discret.e < 0){
      if (progress.e > i*translation_discret.e/translation_discret_length){
        progress.e -= 1;
        location.e -= 1.0/STEPS_PER_E;
        doStep(DEC_E);
      }
    }
    if (i < translation_discret_length){
      i++;

      if (translation_discret_length - i > steps_with_acc){
        if (htim15.Init.Period != kappa){
          steps_with_acc++;
          counter = counter - 2*counter/(4*i + 1);
          if (counter >= kappa){
            htim15.Init.Period = counter;
            HAL_TIM_Base_Init(&htim15);
          }else{
            htim15.Init.Period = kappa;
            HAL_TIM_Base_Init(&htim15);
          }
        }
      }else{
        //Start deceleration
        htim15.Init.Period = htim15.Init.Period - 2*htim15.Init.Period/(4*(translation_discret_length - i - 2*steps_with_acc) + 1);
        HAL_TIM_Base_Init(&htim15);
      }
/*

      if (i >= translation_discret_length/2){
        if (OCR1A > kappa){
          counter = counter - 2*counter/(4*(i - translation_discret_length) + 1);
        }
      }else{
        counter = counter - 2*counter/(4*i + 1);
        if (counter >= kappa){
          OCR1A = counter;
        }else{
          OCR1A = kappa;
        }
      }
*/
      //OCR1A = OCR1A - 2*OCR1A/(4*i + 1);
    }else{
      /*
        When task has been complete
      */
      translation_discret_length = 0;
      steps_with_acc = 0;
    }
  }else if ((state0 & NEW_TASK) && (translation_discret_length == 0)){
    state0 &= ~NEW_TASK;
    i = 0;
    counter = C0;
    htim15.Init.Period = C0;
    HAL_TIM_Base_Init(&htim15);
    sendStaicMessage("OK\n");
  }


  /* USER CODE END TIM1_BRK_TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim15);
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  if (!(state0 & NEW_TASK)){
    //PWM_PORT &= ~(1<<ENABLE_X);
    DriverDisableX();
    //PWM_PORT &= ~(1<<ENABLE_Y);
    DriverDisableY();
    //PWM_PORT &= ~(1<<ENABLE_Z);
    DriverDisableZ();
    //PWM_PORT &= ~(1<<ENABLE_E);
    /*Для охлаждения*/
    //PORTC = 0b00000000;
    //PORTB &= ~CLEAR_E;
  }
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  HAL_ADC_Start(&hadc1);
  static uint32_t ADCValue_extruder;
  static uint32_t ADCValue_bed;

  if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK)
  {
    ADCValue_extruder = HAL_ADC_GetValue(&hadc1);
    ADCValue_bed = HAL_ADC_GetValue(&hadc1);
  }

  if (TEMPERATURE_OF_EXTRUDER + TEMPERATURE_WINDOW < getTemperature_extruder((unsigned int) ADCValue_extruder))
  {
    HeatExtruderEnable();
  }else{
    HeatExtruderDisable();
  }

  if (TEMPERATURE_OF_BED + TEMPERATURE_WINDOW < getTemperature_bed((unsigned int) ADCValue_bed))
  {
    HeatBedEnable();
  }else{
    HeatBedDisable();
  }

  HAL_ADC_Stop(&hadc1);

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
