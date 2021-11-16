/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1416
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
float tempo = 0.0;
float tamos = 0.05;
float t_max = 10*PI;
float a = 0.047753;
float s = 0.185;
float H0 = 0.175;
float b[6][4] = {{0.0814, 0.0487, 0.0025,1},{0.0015, 0.0948, 0.0025,1},{-0.0828, 0.0461, 0.0025,1},
    {-0.0828, -0.0461, 0.0025,1},{0.0015, -0.0948, 0.0025,1},{0.0814, -0.0487, 0.0025,1}};
float p[6][4] = {{0.0866, 0.0297, 0,1},{-0.0176, 0.0898, 0,1}, {-0.0690, 0.0602, 0,1}, {-0.0690, -0.0602, 0,1},{-0.0176, -0.0898, 0,1}, {0.0866, -0.0297, 0,1}};
float q[6][4];
float r[4] = {0, 0, 0, 0};
float T[4][4];
float RX[4][4];
float RY[4][4];
float RZ[4][4];
float x,y,z,angx,angy,angx_,angy_,angz,absl,ek,fk,gk;
float l[3];
float alphas[6];
float betas[6] = {1.5708, 6.8068, 3.6652, -3.6652,   -0.5236, 4.7124};
int calibra = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim1;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
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
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
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
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
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
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
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
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
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
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt.
  */
void TIM1_BRK_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_IRQn 0 */

  /* USER CODE END TIM1_BRK_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_IRQn 1 */

  /* USER CODE END TIM1_BRK_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
  /*Cálculo de ângulo dos servos de acordo com a posição desejada*/
  // //Experimento 1
  //	x = 0.02*cos(tempo-t0)*(tempo>t0);
  //	y = 0.02*sin(tempo-t0)*(tempo>t0);
  //	z = (0.025-0.05*(tempo-t0)/t_max)*(tempo>t0);
  //	angx = 0*(tempo>t0);
  //	angy = 0*(tempo>t0);
  //	angz = 0*(tempo>t0);
//    //Experimento 2
//    x = 0;
//    y = 0;
//    z = 0;
//    angx = (PI/10)*sin(tempo-PI)*(tempo>PI)*(tempo<3*PI);
//    angy = (PI/10)*sin(tempo-4*PI)*(tempo>4*PI)*(tempo<6*PI);
//    angz = 0;//(PI/6)*sin(tempo-7*PI)*(tempo>7*PI)*(tempo<9*PI);
//    //Experimento 3
//    x = 0.06*sin(2*tempo)*(tempo>4*PI);
//    y = 0.06*sin(2*tempo)*cos(2*tempo)*(tempo>4*PI);
//    z = 0;
//    angx = 0;
//    angy = 0;
//    angz = 0;
    //Experimento Helicoide, permanece parado ate t = 4pi, y se move de 0 a 0.06 entre 4 e 6pi
    //e entao comeca o movimento de rotacao ate z = 0.03 e depois de volta ate z = 0, repete
    float amp = 0.05;
    x = amp*sin(tempo-6*PI)*(tempo>6*PI);
    y = amp*((tempo-4*PI)/(2*PI))*(tempo>4*PI)*(tempo<6*PI)+amp*cos(tempo-6*PI)*(tempo>6*PI);
    z = 0.03*(tempo-6*PI)/(4*PI)*(tempo > 6*PI);
    angx = 0;
    angy = 0;
    angz = 0;
    //Experimento 'cone'
//    float amp = (PI/10);
//    x = 0;
//    y = 0;
//    z = 0;
//    angx = amp*sin(tempo)*(tempo > 4*PI);
//    angy = amp*cos(tempo)*(tempo > 4*PI) + amp*(tempo-2*PI)/(2*PI)*(tempo>2*PI)*(tempo<4*PI);
//    angz = 0;
//  //Experimento varia x
//  x = 0.06*sin(2*tempo)*(tempo>4*PI);
//  y = 0;
//  z = 0;
//  angx = 0;
//  angy = 0;
//  angz = 0;
  //Experimento varia y
//  x = 0;
//  y = 0.06*sin(2*tempo)*(tempo>4*PI);
//  z = 0;
//  angx = 0;
//  angy = 0;
//  angz = 0;


    float T[4][4] = {{1, 0, 0, x}, {0, 1, 0, y}
    , {0, 0, 1, z+H0}, {0, 0, 0, 1}};
    float RX[4][4] = {{1, 0, 0, 0}, {0, cos(angx), -sin(angx), 0}
    , {0, sin(angx), cos(angx), 0}, {0, 0, 0, 1}};
    float RY[4][4] = {{cos(angy), 0, sin(angy), 0}, {0, 1, 0, 0}, {-sin(angy), 0, cos(angy), 0}, {0, 0, 0, 1}};
    float RZ[4][4] = {{cos(angz), -sin(angz), 0}, {sin(angz), cos(angz), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    for (int cont3 = 0; cont3 < 6; cont3++){
  	for (int cont1 = 0; cont1 < 4; cont1++){
  	  r[cont1] = 0;
  	  for (int cont2 = 0; cont2 < 4; cont2++){
  		r[cont1] += RX[cont1][cont2]*p[cont3][cont2];
  	  }
  	  q[cont3][cont1] = r[cont1];
  	}
    }
    for (int cont3 = 0; cont3 < 6; cont3++){
  	for (int cont1 = 0; cont1 < 4; cont1++){
  	  r[cont1] = 0;
  	  for (int cont2 = 0; cont2 < 4; cont2++){
  		r[cont1] += RY[cont1][cont2]*q[cont3][cont2];
  	  }
  	  q[cont3][cont1] = r[cont1];
  	}
    }
    for (int cont3 = 0; cont3 < 6; cont3++){
  	for (int cont1 = 0; cont1 < 4; cont1++){
  	  r[cont1] = 0;
  	  for (int cont2 = 0; cont2 < 4; cont2++){
  		r[cont1] += RZ[cont1][cont2]*q[cont3][cont2];
  	  }
  	  q[cont3][cont1] = r[cont1];
  	}
    }
    for (int cont3 = 0; cont3 < 6; cont3++){
  	for (int cont1 = 0; cont1 < 4; cont1++){
  	  r[cont1] = 0;
  	  for (int cont2 = 0; cont2 < 4; cont2++){
  		r[cont1] += T[cont1][cont2]*q[cont3][cont2];
  	  }
  	  q[cont3][cont1] = r[cont1];
  	}
    }
    for (int cont1 = 0; cont1 < 6; cont1++){
		for (int cont2 = 0; cont2 < 3; cont2++){
		 l[cont2] = q[cont1][cont2]-b[cont1][cont2];
		}
		absl = sqrt(l[0]*l[0]+l[1]*l[1]+l[2]*l[2]);
		ek = 2*a*l[2];
		fk = 2*a*(cos(betas[cont1])*l[0]+sin(betas[cont1])*l[1]);
		gk = absl*absl-(s*s-a*a);

		alphas[cont1] = (asin(gk/sqrt(ek*ek+fk*fk))-atan2(fk,ek))*180/PI;
	}
    htim2.Instance->CCR2 = (750/180)*(90-9+alphas[0])+500;
    htim2.Instance->CCR3 = (750/180)*(90+15-alphas[1])+500;
    htim2.Instance->CCR4 = (750/180)*(90+5+alphas[2])+500;
    htim3.Instance->CCR1 = (750/180)*(90+12-alphas[3])+500;
    htim3.Instance->CCR2 = (750/180)*(90-16+alphas[4])+500;
    htim3.Instance->CCR3 = (750/180)*(90+8-alphas[5])+500;
    tempo += tamos;
    if ((tempo >= t_max) || (tempo <= 0))
  	  tamos = -tamos;
  /* USER CODE END TIM1_UP_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
