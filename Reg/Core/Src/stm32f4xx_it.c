/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <testfunksjoner.h>
#include <variabler_ext.h>
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "funksjoner.h"
#include <string.h>
#include <stdlib.h>
#include <verdier.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
extern CAN_HandleTypeDef hcan1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Pre-fetch fault, memory access fault.
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
	// PushButton avprelling
	teller_bryter++;
	if (teller_bryter >= 10){
		sjekk_brytertrykk();		// Sjekker om det har vært et brukertrykk siden sist sjekk.
		teller_bryter = 0;
	}

	teller_reg_hiv++;
	if (teller_reg_hiv >= 250){		// 250 ms
		if(flagg.hiv_pause){		// Sjekker om piloten nylig har endret dybde
			hpid.yr = hiv_m;		// Dybdereferansen oppdateres
      // hpid.yr = sensordata.z/10;
      ctrl_ref.z = model_states.z;
			hpid.es = hpid.fbfs = hpid.uds = hpid.uis = 0; // nullstiller følgeverdier.
      reset_pid_values(&pid_heave); reset_smc_values(&smc_heave);
			flagg.hpid = 1;			// Hiv-regulering restartes
			flagg.hiv_pause = 0;	// Avslutter hiv-pausen.
		}
		teller_reg_hiv = 0;
	}

	teller_reg_rull++;
	if (teller_reg_rull >= 250){		// 250 ms
		if(flagg.rull_pause){		// Sjekker om piloten nylig har endret rull
			rpid.es = rpid.fbfs = rpid.uds = rpid.uis = 0; // nullstiller følgeverdier.
      reset_pid_values(&pid_roll); reset_smc_values(&smc_roll);
			flagg.rpid = 1;			// gir-regulering restartes
			flagg.rull_pause = 0;	// Avslutter gir-pausen.
		}
		teller_reg_rull = 0;
	}

	teller_reg_stamp++;
	if (teller_reg_stamp >= 250){		// 250 ms
		if(flagg.stamp_pause){		// Sjekker om piloten nylig har endret stamp
			spid.es = spid.fbfs = spid.uds = spid.uis = 0; // nullstiller følgeverdier.
      reset_pid_values(&pid_pitch); reset_smc_values(&smc_pitch);
			flagg.spid = 1;			// stamp-regulering restartes
			flagg.stamp_pause = 0;	// Avslutter gir-pausen.
		}
		teller_reg_stamp = 0;
	}

	teller_reg_gir++;
	if (teller_reg_gir >= 250){		// 250 ms
		if(flagg.gir_pause){		// Sjekker om piloten nylig har endret gir
			gpid.yr = gir_m;		// Girreferansen oppdateres
			gpid.es = gpid.fbfs = gpid.uds = gpid.uis = 0; // nullstiller følgeverdier.
			flagg.gpid = 1;			// gir-regulering restartes
			flagg.gir_pause = 0;	// Avslutter gir-pausen.
		}
		teller_reg_gir = 0;
	}

  if (step_counter.TIMER_STATUS) {
    step_counter.seq_time++;
    step_counter.incr_timer++;
  }

	teller_diode++;
	if(teller_diode >= 250){
		led_status = !led_status;	// Endrer status for blinkende dioder.
		teller_diode = 0;
	}
	
	teller_idle++;					// Sjekker om det mottas styremeldinger.
	if(teller_idle >=  250){		// 250 ms
		memset(&hivbidrag, 0, sizeof(thruster_sett_float));
		memset(&rullbidrag, 0, sizeof(thruster_sett_float));
		memset(&stampbidrag, 0, sizeof(thruster_sett_float));
		memset(&girbidrag, 0, sizeof(thruster_sett_float));
		memset(&styrebidrag, 0, sizeof(thruster_sett_float));
		memset(&styrefilter, 0, sizeof(thruster_sett_float));
		behandle_paadrag();
//		memcpy(&thruster_PWM, &oppstart_PWM, sizeof(thruster_sett));
//		oppdater_ledlys();			// Slår av dioder
//		skriv_thruster_PWM();		// Setter thrustere i nøytral
//		GPIOE->ODR &= 0x0000;
		teller_idle = 0;
	}

	if(flagg.test_thrustere){
		teller_test_thruster++;
		if(!flagg.testfunksjon){	test_thruster(thruster_ID);	}
		if(teller_test_thruster >= 5000){
			memcpy(&thruster_PWM, &oppstart_PWM, sizeof(thruster_sett));
			skriv_thruster_PWM();
			flagg.test_thrustere = 0;
			flagg.testfunksjon = 0;
			teller_test_thruster = 0;
		}
	}

//	test_reg++;
//	teller_test_reg++;
//	if(flagg.regulering & (test_reg >= 50)){
//		flagg.can_70 = 1;
//		test_reg = 0;
//	}


  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
