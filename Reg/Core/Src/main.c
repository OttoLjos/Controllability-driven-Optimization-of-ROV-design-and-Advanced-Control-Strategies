/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <structures.h>

#include <variabler_ext.h>
#include <variabler.h>

#include <funksjoner.h>
#include <pid.h>
#include <testfunksjoner.h>
#include <verdier.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 CAN_HandleTypeDef hcan1;

I2S_HandleTypeDef hi2s3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */


//styrestruct styremelding;
//gyrostruct gyrodata;
//can_FLAGG can_rx;
//uint8_t hiv_flagg;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2S3_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void GPIO_setup(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//
// CAN-Buss oppsett Start
//
CAN_RxHeaderTypeDef rxHeader; //CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader; //CAN Bus Receive Header
uint8_t canRX[8] = {0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
uint8_t csend[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08}; // Tx Buffer
CAN_FilterTypeDef canfilter;
uint32_t canMailbox; //CAN Bus Mail box variable

void oppstartCAN(uint8_t filterGruppe, CAN_HandleTypeDef *canPort) {
    canfilter.FilterBank = 0;
    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfilter.FilterIdHigh = filterGruppe << 10 | 0x1F;
    canfilter.FilterIdLow = 0xFFF8;
    canfilter.FilterMaskIdHigh = 0x3F << 10 | 0x1F;
    canfilter.FilterMaskIdLow = 0xFFF8;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilter.FilterActivation = ENABLE;
    canfilter.SlaveStartFilterBank = 14;

    txHeader.DLC = 8; // Number of bites to be transmitted max- 8
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.StdId = 0x00;
    txHeader.ExtId = 0x00;
    txHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_ConfigFilter(canPort, &canfilter);
    HAL_CAN_Start(canPort);
    HAL_CAN_ActivateNotification(canPort, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void sendDataCAN(uint16_t id, CAN_HandleTypeDef *canPort) {
    txHeader.StdId = id;
    HAL_CAN_AddTxMessage(canPort, &txHeader, csend, &canMailbox);
}
//
// CAN-Buss oppsett Stopp
//


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2S3_Init();
  MX_USB_HOST_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  oppstartCAN(2, &hcan1);
  GPIO_setup();
//  int16_t var1 = 0;
  oppstartsverdier();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_Delay(1000);
//	  memcpy(csend, var1, 6);
//	  sendDataCAN(170, &hcan1);
//	  var1++;

	  if(flagg.can_70){					// Flagget signaliserer ny mottatt styremelding

//		  if(flagg.can_69){
//			  kombiner_styring();
//			  flagg.can_69 = 0;
//		  }
		  if (active_step.SEQ_STATUS){
			if (step_counter.seq_time > active_step.test_length){
			memcpy(&ctrl_ref, &active_step.old_ref, sizeof(NED_eta));
			active_step.SEQ_STATUS = 0;
			memset(&step_counter, 0, sizeof(step_timers));
			}
			else {
				step_machine(&active_step.heave, &ctrl_ref.z, active_step.old_ref.z);
				step_machine(&active_step.roll, &ctrl_ref.phi, active_step.old_ref.phi);
				step_machine(&active_step.pitch, &ctrl_ref.theta, active_step.old_ref.theta);
				step_counter.incr_timer = 0;
			}
		  }

		  oppdater_styrebidrag();		// Behandler pilotinstruksjoner for navigasjon
		  manipulator_styring();		// Behandler pilotinstruksjoner for manipulatoren

		  

		  if (controller_status == 1){
			  // PID Controller
			  thrust_NED.z = pid(&pid_heave, model_states.z, ctrl_ref.z);
			  thrust_NED.phi = pid(&pid_roll, model_states.phi, ctrl_ref.phi);
			  thrust_NED.theta = pid(&pid_pitch, model_states.theta, ctrl_ref.theta);
			  // NED2BFF(&thrust_NED, &thrust_BFF, &model_states);
		  }
		  else if (controller_status == 2){
			  // Linear Quadratic Regulator
			  // lqr(&lqr_reg, &model_states, &ctrl_ref, &thrust_BFF);
			  lqr(&lqr_reg, &model_states, &ctrl_ref, &thrust_NED);
			  // NED2BFF(&thrust_NED, &thrust_BFF, &model_states);
		  }
		  else if (controller_status == 3){
			  // Sliding Mode Controller
			  thrust_NED.z = smc(&smc_heave, model_states.z, ctrl_ref.z);
			  thrust_NED.phi = smc(&smc_roll, model_states.phi, ctrl_ref.phi);
			  thrust_NED.theta = smc(&smc_pitch, model_states.theta, ctrl_ref.theta);
			  // NED2BFF(&thrust_NED, &thrust_BFF, &model_states);
		  }
		  else if (controller_status == 4){
			if(flagg.spid){  stamp_regulator(); }	// PID regulering av stampvinkelen
			if(flagg.rpid){  rull_regulator();  }	// PID regulering av rullvinkelen
			if(flagg.hpid){  hiv_regulator();   }	// PID regulering av hivposisjon
			// if(flagg.gpid){  gir_regulator();	  }	// PID regulering av girvinkel
		  }
		  if (controller_status){
			  NED2BFF(&thrust_NED, &thrust_BFF, &model_states);
			  BFF2ThrustPercent(&thrust_BFF, &ctrl_contribution);
		  }

		  if(test_thrustere){
			  styrebidrag.hhf = 40.0;
			  styrebidrag.hhb = 40.0;
			  styrebidrag.hvb = 40.0;
			  styrebidrag.hvf = 40.0;
		  }

		  behandle_paadrag();			// Pådragsprosessering

		  memcpy(&csend, &thrusterdata, 8);	// Sender pådragsdata til toppsiden
		  sendDataCAN(170, &hcan1);

		  csend[0] = flagg.spid;
		  csend[1] = flagg.rpid;
		  csend[2] = flagg.hpid;
		  csend[3] = flagg.gpid;
		  csend[4] = flagg.stamp_pause;
		  csend[5] = flagg.rull_pause;
		  csend[6] = flagg.hiv_pause;
		  csend[7] = flagg.gir_pause;
		  sendDataCAN(172, &hcan1);
		  HAL_Delay(1);

		  float nullpunkt[2] = {((float) controller_status), ((float) active_step.SEQ_STATUS)};
		  memcpy(&csend, &nullpunkt, sizeof(nullpunkt));
		  sendDataCAN(173, &hcan1);
		  HAL_Delay(1);

		  float nullpunkt2[2] = {10.0, ctrl_ref.z};
		  memcpy(&csend, &nullpunkt2, sizeof(nullpunkt));
		  sendDataCAN(173, &hcan1);
		  HAL_Delay(1);

		  float nullpunkt3[2] = {20.0, ctrl_ref.phi};
		  memcpy(&csend, &nullpunkt3, sizeof(nullpunkt));
		  sendDataCAN(173, &hcan1);
		  HAL_Delay(1);

		  float nullpunkt4[2] = {30.0, ctrl_ref.theta};
		  memcpy(&csend, &nullpunkt4, sizeof(nullpunkt));
		  sendDataCAN(173, &hcan1);


		  flagg.can_70 = 0;				// Styring og regulering fullført
	  }

	  if(esc_reset){
		  memcpy(&thruster_PWM, &oppstart_PWM, sizeof(thruster_sett));
		  skriv_thruster_PWM();
		  HAL_Delay(100);
		  csend[0] = 0b00111100;
		  sendDataCAN(139, &hcan1);
		  HAL_Delay(2000);
		  esc_reset = 0;

      gpid.yr = gir_m;								// Girreferansen oppdateres
      gpid.es = gpid.fbfs = gpid.uds = gpid.uis = 0; 	// nullstiller følgeverdier.
      spid.es = spid.fbfs = spid.uds = spid.uis = 0; 	// nullstiller følgeverdier.
      rpid.es = rpid.fbfs = rpid.uds = rpid.uis = 0; 	// nullstiller følgeverdier.
      hpid.yr = hiv_m;								// Dybdereferansen oppdateres
      hpid.es = hpid.fbfs = hpid.uds = hpid.uis = 0; 	// nullstiller følgeverdier.

      memset(&ctrl_ref, 0, sizeof(NED_eta));
      ctrl_ref.z = model_states.z;
      reset_pid_values(&pid_heave); reset_pid_values(&pid_roll); reset_pid_values(&pid_pitch);
      reset_lqr_values(&lqr_reg);
      reset_smc_values(&smc_heave); reset_smc_values(&smc_roll); reset_smc_values(&smc_pitch);
	  }



	  if(brytertrykk){
//		  oppdater_parameter( 3, 0.0);
//		  oppdater_parameter( 4, 0.0);
//		  oppdater_parameter( 5, 0.0);
//		  GPIOE->ODR = 0xFFFF;
//		  if(!test_dioder_toggle){
//			  test_dioder_toggle = 1;
//		  }
//		  else{
//			  test_dioder_toggle = 0;
//
//			  GPIOE->ODR = 0x00;
//		  }
//		  if(!flagg.regulering){
//			  flagg.regulering = 1;
//		  }
//		  else{
//			  flagg.regulering = 0;
//		  }
//		  test_led_dioder();
//		  test_mpwm_pinner();
//		  test_tpwm_pinner();
//		  test_styremeldinger();
		  brytertrykk = 0;
	  }



    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD3_Pin|LD5_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD5_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD5_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void GPIO_setup(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/*Configure GPIO pins : LED custom config  */
	GPIO_InitStruct.Pin = LED_HHF | LED_HHB | LED_HVB | LED_HVF | LED_VHF | LED_VHB | LED_VVB | LED_VVF;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

//	/*Configure GPIO pins :GPIOA */
//	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_15;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//	/*Configure GPIO pins : GPIOB */
//	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//
//	/*Configure GPIO pins : GPIOC */
//	GPIO_InitStruct.Pin = GPIO_PIN_6;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//
//	/*Configure GPIO pins : GPIOD */
//	GPIO_InitStruct.Pin = GPIO_PIN_12;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}


// Interupt for CAN-Buss mottak av data
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan11) {
    HAL_CAN_GetRxMessage(hcan11, CAN_RX_FIFO0, &rxHeader, canRX); //Receive CAN bus message to canRX buffer
    switch (rxHeader.StdId) {
    case 64:
        // Kjør kode
    	/*
    	int8_t styrekommandoer = {0,0,0,0,0,0,0,0};

    	styrekommandoer = (int8_t) canRX;
    	int8_t* Buffer = (int8_t*) &canRX[0];
    	for (i=0;i<8;i++){
    		styrekommandoer[i] = *Buffer;
    		Buffer++;
    	}
    	*/
        break;

    case 65:
        // Kjør kode
		/*
		int16_t* Buffer = (int16_t*) &canRX[0];
		for (i=0;i<4;i++){
			sensordata[i] = *Buffer;
			Buffer++;
		}
		*/
        break;


    case 69:
    	// Autonom kjørekommando
    	memcpy(&autonomstyring, &canRX, sizeof(styrestruct));
    	flagg.can_69 = 1;
        break;


    case 70:
        // Styrekommando
    	memcpy(&styremelding, &canRX, sizeof(styrestruct));
    	flagg.can_70 = 1;
    	teller_idle = 0;
        break;

    case 71:
    	// oppdater parameter
		memcpy(&ny_param, &canRX, sizeof(can_param));
		oppdater_parameter(ny_param.param_id, ny_param.parameter);
		break;

    case 80:
    	// Gyromålinger
    	memcpy(&gyrodata, &canRX, sizeof(gyrostruct));
    	sensordata.phi_old = sensordata.phi;
    	sensordata.theta_old = sensordata.theta;
    	sensordata.psi_old = sensordata.psi;
    	sensordata.z_old = sensordata.z;

    	sensordata.phi = -gyrodata.rull;
    	sensordata.theta = gyrodata.stamp;
    	sensordata.psi = gyrodata.gir;
    	sensordata.z = gyrodata.hiv;

    	model_states.z = (float) sensordata.z 											/100;
    	model_states.phi =(float) sensordata.phi 						*D2R 			/10;
    	model_states.theta = (float) sensordata.theta 					*D2R 			/10;
    	model_states.psi = (float) sensordata.psi 						*D2R 			/10;
    	model_states.w = (float) (sensordata.z-sensordata.z_old)				/TSTEP 	/100;
    	model_states.p = (float) (sensordata.phi-sensordata.phi_old)	*D2R	/TSTEP 	/10;
    	model_states.q = (float) (sensordata.theta-sensordata.theta_old)*D2R	/TSTEP 	/10;
    	model_states.r = (float) (sensordata.psi-sensordata.psi_old) 	*D2R	/TSTEP 	/10;

    	hiv_m =   (float) gyrodata.hiv / 100;
    	rull_m =  (float) gyrodata.rull / 10;
    	stamp_m = (float) gyrodata.stamp / 10;
    	gir_m =   (float) gyrodata.gir / 10;
    	gir_mf = gir_mf *0.95 + 0.05*gir_m;
//    	flagg.can_80 = 1;
        break;

    case 81:
    	// Akselerasjonsmålinger
    	memcpy(&aksdata, &canRX, sizeof(imustruct));
    	sensordata.u_dot = aksdata.x;
    	sensordata.v_dot = aksdata.y;
    	sensordata.w_dot = aksdata.z;

    	model_states.u = (float) model_states.u + sensordata.u_dot*TSTEP /100;
    	model_states.v = (float) model_states.v + sensordata.v_dot*TSTEP /100;
//    	model_states.w = (float) model_states.w + sensordata.w_dot*TSTEP /100;
    	model_states.x = (float) model_states.x + model_states.u*TSTEP;
    	model_states.y = (float) model_states.y + model_states.v*TSTEP;


//    	flagg.can_81 = 1;
        break;

    case 82:
    	// Magnometermålinger
    	memcpy(&magnodata, &canRX, sizeof(imustruct));
//    	gir_m = (float) magnodata.heading / 10;
//    	gir_mf = gir_mf *0.95 + 0.05*gir_m;
//    	flagg.can_82 = 1;
        break;

    case 85:
    	// testmelding sensor
    	memcpy(&csend, &canRX, 8);
    	sendDataCAN(97, &hcan1);
        break;


    case 90:
	// effektforbruk
    	memcpy(&effekt_forbruk, &canRX, sizeof(effekt_struct));
    	power12vf = 0.9*power12vf + 0.1*effekt_forbruk.thruster_12v;
    	break;


    case 95: // IKKE FJERN DENNE
        memcpy(csend, (uint8_t*) &"pong!\n", 6);
        sendDataCAN(189, &hcan1);

    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
