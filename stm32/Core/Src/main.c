/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Mapping L298N sur GPIOA
#define M1_IN1_Pin        GPIO_PIN_1   // IN1 moteur 1
#define M1_IN1_GPIO_Port  GPIOB
#define M1_IN2_Pin        GPIO_PIN_4   // IN2 moteur 1
#define M1_IN2_GPIO_Port  GPIOA

#define M2_IN3_Pin        GPIO_PIN_0   // IN3 moteur 2
#define M2_IN3_GPIO_Port  GPIOB
#define M2_IN4_Pin        GPIO_PIN_7   // IN4 moteur 2
#define M2_IN4_GPIO_Port  GPIOA

// Directions des moteurs
#define DIR_BACKWARD   1
#define DIR_FORWARD    0

// Valeurs de test pour la trajectoire rectiligne (0..255)
#define M1_DUTY_STRAIGHT   207   // PWM moteur 1 // Droite
#define M2_DUTY_STRAIGHT   200   // PWM moteur 2 // Gauche (plus fort)

// ==================== Paramètres encodeurs ====================

// période de mise à jour des vitesses (ms)
#define ENC_UPDATE_PERIOD_MS   100
// même valeur en secondes (10 ms = 0.01 s)
#define ENC_UPDATE_PERIOD_S    0.01f

// Nombre d'impulsions par tour de l'engrenage qui entraîne la chenille
// ⚠️ À AJUSTER selon vos capteurs / engrenages
#define PULSES_PER_REV_M1      300.0f   // moteur 1 (TIM1)
#define PULSES_PER_REV_M2      300.0f   // moteur 2 (LPTIM1)

// Distance linéaire par impulsion (cm)
// 66 cm pour 300 impulsions  ->  66.0 / 300.0 cm par pulse
#define CM_PER_PULSE          (66.0f / 300.0f)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
// Buffers I2C : [M1_Speed, M1_Dir, M2_Speed, M2_Dir]
uint8_t i2c_rx_buf[4];
uint8_t i2c_tx_buf[4];

// Compteurs debug (facultatif)
volatile uint32_t addr_callback_count = 0;
volatile uint32_t rx_complete_count   = 0;
volatile uint32_t tx_complete_count   = 0;
volatile uint32_t error_count         = 0;

// dernier instant où on a reçu une trame I2C
volatile uint32_t last_i2c_rx_tick = 0;

// ==================== Encodeurs : états internes ====================
// Motor 1 : TIM1 en mode encodeur
static uint16_t prevCntM1 = 0;     // dernière valeur lue TIM1->CNT (16 bits)
static int32_t  totalCountsM1 = 0; // total de "crans" depuis le démarrage
static int32_t  tM1 = 0; // total de "crans" depuis le démarrage
// Motor 2 : LPTIM1 en compteur externe (pulses Hall)
static uint16_t prevCntM2 = 0;     // dernière valeur lue LPTIM1->CNT (16 bits)
static int32_t  totalCountsM2 = 0; // total de pulses depuis le démarrage

// Résultats calculés (accessibles partout)
int32_t   speedM1_cm_s = 0;  // vitesse moteur 1 en cm/s
int32_t  speedM2_cm_s = 0;  // vitesse moteur 2 en cm/s
volatile uint32_t dt_ms;
int32_t toursM1      = 0;     // nombre de "tours" / segments parcourus (optionnel)
int32_t toursM2      = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void Start_I2C_Listen(void);
void Debug_Blink(int count, int delay_ms);
void Debug_I2C_State(void);

// Fonctions de contrôle moteurs
void Motors_Stop(void);
void Motors_ApplyFromI2C(void);
void Test_Trajectoire_Rectiligne(uint8_t m1_speed, uint8_t m2_speed, uint32_t duration_ms);

// ======== Nouvelles fonctions : encodeurs & vitesses ========
void Encoders_Init(void);
void Encoders_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_TIM15_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // Démarrer le PWM sur TIM2 CH1 et CH2 (ENA / ENB du L298N)
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);

  // Moteurs arrêtés au début
  Motors_Stop();

  // Petit blink de démarrage
  Debug_Blink(3, 100);

  // I2C slave en écoute
  Start_I2C_Listen();
  Debug_I2C_State();
  last_i2c_rx_tick = HAL_GetTick();

  // ======== Démarrer les encodeurs (M1 = TIM1, M2 = LPTIM1) ========
  Encoders_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    // Sécurité : si on ne reçoit plus de trame I2C pendant > 500 ms, on arrête les moteurs
	    uint32_t now = HAL_GetTick();

	    if (now - last_i2c_rx_tick > 500) {   // 500 ms sans nouvelle trame
	      Motors_Stop();
	    }

	    // Mise à jour des vitesses + nombre de tours (toutes les 10 ms)
	    Encoders_Update();

	    // Période d'échantillonnage (doit rester = ENC_UPDATE_PERIOD_MS)
	    HAL_Delay(ENC_UPDATE_PERIOD_MS);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 174;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 32-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 500-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Convertit un duty 0..255 en valeur CCR (0..ARR)
static uint16_t DutyToCCR(uint8_t speed)
{
  uint16_t period = __HAL_TIM_GET_AUTORELOAD(&htim15); // ici 500-1 = 499
  return (uint16_t)((speed * period) / 255);
}

// Met les PWM sur ENA/ENB (TIM2_CH1/CH2)
static void Motors_SetPWM(uint8_t m1_speed, uint8_t m2_speed)
{
  uint16_t ccr1 = DutyToCCR(m1_speed);
  uint16_t ccr2 = DutyToCCR(m2_speed);

  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, ccr1); // ENA
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, ccr2); // ENB
}

// Applique un ordre aux moteurs : vitesse + direction
// m1_dir, m2_dir : DIR_FORWARD ou DIR_BACKWARD
static void Motors_Apply(uint8_t m1_speed, uint8_t m1_dir, uint8_t m2_speed, uint8_t m2_dir)
{
  // --- Moteur 1 : IN1/IN2 + ENA ---
  if (m1_speed == 0) {
    // Stop : IN1 = IN2 = 0
    HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
  } else {
    if (m1_dir == DIR_FORWARD) {
      // Avant
      HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_SET);
    } else {
      // Arrière
      HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
    }
  }

  // --- Moteur 2 : IN3/IN4 + ENB ---
  if (m2_speed == 0) {
    HAL_GPIO_WritePin(M2_IN3_GPIO_Port, M2_IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_IN4_GPIO_Port, M2_IN4_Pin, GPIO_PIN_RESET);
  } else {
    if (m2_dir == DIR_FORWARD) {
      // Avant
      HAL_GPIO_WritePin(M2_IN3_GPIO_Port, M2_IN3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(M2_IN4_GPIO_Port, M2_IN4_Pin, GPIO_PIN_SET);
    } else {
      // Arrière
      HAL_GPIO_WritePin(M2_IN3_GPIO_Port, M2_IN3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(M2_IN4_GPIO_Port, M2_IN4_Pin, GPIO_PIN_RESET);
    }
  }

  // Appliquer les PWM
  Motors_SetPWM(m1_speed, m2_speed);
}

// Applique la trame I2C [M1_V, M1_Dir, M2_V, M2_Dir] aux moteurs (L298N)
void Motors_ApplyFromI2C(void)
{
  uint8_t m1_speed   = i2c_rx_buf[0];
  uint8_t m1_dir_bit = i2c_rx_buf[1]; // 1 = avant, 0 = arrière (vu depuis l'ESP32)
  uint8_t m2_speed   = i2c_rx_buf[2];
  uint8_t m2_dir_bit = i2c_rx_buf[3];

  // Traduction vers les constantes internes DIR_FORWARD / DIR_BACKWARD
  uint8_t m1_dir = (m1_dir_bit == 1) ? DIR_FORWARD : DIR_BACKWARD;
  uint8_t m2_dir = (m2_dir_bit == 1) ? DIR_FORWARD : DIR_BACKWARD;

  Motors_Apply(m1_speed, m1_dir, m2_speed, m2_dir);
}

// Test de trajectoire rectiligne : avance tout droit pendant "duration_ms"
void Test_Trajectoire_Rectiligne(uint8_t m1_speed, uint8_t m2_speed, uint32_t duration_ms)
{
  // Les deux moteurs en avant
  Motors_Apply(m1_speed, DIR_FORWARD, m2_speed, DIR_FORWARD);

  // On roule pendant duration_ms
  HAL_Delay(duration_ms);

  // Puis on arrête les moteurs et on laisse une pause pour se repositionner
  Motors_Stop();
  HAL_Delay(3000);  // 3 s de pause
}

// Stop complet des moteurs
void Motors_Stop(void)
{
  // Vitesse = 0
  Motors_SetPWM(0, 0);

  // H-bridge à l'arrêt
  HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(M2_IN3_GPIO_Port, M2_IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(M2_IN4_GPIO_Port, M2_IN4_Pin, GPIO_PIN_RESET);
}

/* ============================================================================ */
/* I2C HAL CALLBACKS */
/* ============================================================================ */

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  if (hi2c->Instance != I2C1) return;

  addr_callback_count++;

  if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
    // Master (ESP32) écrit vers STM32 -> on prépare une réception de 4 octets
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, i2c_rx_buf, 4, I2C_FIRST_AND_LAST_FRAME);
  } else {
    // Master lit depuis STM32 -> on renvoie un écho simple
    for (int i = 0; i < 4; i++) {
      i2c_tx_buf[i] = i2c_rx_buf[i];
    }
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2c_tx_buf, 4, I2C_FIRST_AND_LAST_FRAME);
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	  if (hi2c->Instance != I2C1) return;

	  rx_complete_count++;

	  // on note le moment de la dernière trame
	  last_i2c_rx_tick = HAL_GetTick();

	  // 1) Appliquer la trame reçue aux moteurs (comme avant)
	  Motors_ApplyFromI2C();

	  // 2) Préparer le feedback pour l'ESP32 :
	  //    on envoie les compteurs totaux (partie basse 16 bits)
	  uint16_t c1 = (uint16_t)totalCountsM1;
	  uint16_t c2 = (uint16_t)totalCountsM2;

	  // Ces 4 octets seront "échos" par HAL_I2C_AddrCallback lors du prochain READ
	  i2c_rx_buf[0] = (uint8_t)(c1 & 0xFF);        // LSB M1
	  i2c_rx_buf[1] = (uint8_t)((c1 >> 8) & 0xFF); // MSB M1
	  i2c_rx_buf[2] = (uint8_t)(c2 & 0xFF);        // LSB M2
	  i2c_rx_buf[3] = (uint8_t)((c2 >> 8) & 0xFF); // MSB M2
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C1) return;

  tx_complete_count++;

  // Debug : LED qui toggle à chaque envoi vers l'ESP32
  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C1) return;

  // Re-activer l'écoute I2C après la fin de la transaction
  Start_I2C_Listen();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C1) return;

  error_count++;

  // Option : allumer LED si gros problème
  // HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
}

/* ============================================================================ */
/* HELPER FUNCTIONS */
/* ============================================================================ */

static void Start_I2C_Listen(void)
{
  HAL_StatusTypeDef status = HAL_I2C_EnableListen_IT(&hi2c1);

  if (status != HAL_OK) {
    // Blink rapide si problème
    Debug_Blink(10, 50);
  }
}

void Debug_Blink(int count, int delay_ms)
{
  for (int i = 0; i < count; i++) {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    HAL_Delay(delay_ms);
  }
}

void Debug_I2C_State(void)
{
  HAL_I2C_StateTypeDef state = HAL_I2C_GetState(&hi2c1);

  int blinks = 0;
  if (state == HAL_I2C_STATE_READY)  blinks = 1;
  else if (state == HAL_I2C_STATE_BUSY)   blinks = 2;
  else if (state == HAL_I2C_STATE_LISTEN) blinks = 3; // bon état attendu
  else blinks = 5; // erreur

  Debug_Blink(blinks, 200);
  HAL_Delay(1000);
}

/* ====================== ENCODEURS & VITESSES ====================== */

// Initialisation des compteurs encodeurs
void Encoders_Init(void)
{
  // --- Moteur 1 : TIM1 en mode encodeur (quadrature sur PB5/PB7) ---
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  prevCntM1     = 0;
  totalCountsM1 = 0;

  // Démarrer l'encodeur sur les deux canaux
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);


  // --- Moteur 2 : LPTIM1 en compteur externe (pulses simples) ---
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  prevCntM2     = 0;
  totalCountsM2 = 0;

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  // Init sorties
  speedM1_cm_s = 0.0f;
  speedM2_cm_s = 0.0f;
  toursM1      = 0;
  toursM2      = 0;

}

// À appeler périodiquement (ici toutes les 10 ms dans la boucle while)
void Encoders_Update(void)
{
  // ================= Moteur 1 : TIM1 encodeur =================
  uint16_t cnt1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);

  // delta sur 16 bits avec gestion overflow (cast en int16_t)
  int16_t delta1 = (int16_t)(cnt1 - prevCntM1);
  prevCntM1 = cnt1;

  totalCountsM1 += abs ((int32_t)delta1);

  // ================= Moteur 2 : LPTIM1 compteur externe =================
  uint16_t cnt2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

  // delta sur 16 bits avec gestion overflow (cast en int16_t)
  int16_t delta2 = (int16_t)(cnt2 - prevCntM2);
  prevCntM2 = cnt2;

  totalCountsM2 += abs ((int32_t)delta2);

  // ================= Conversion en tours & vitesses =================
  static int32_t prevTotalM1 = 0;
  static int32_t prevTotalM2 = 0;
  static uint32_t lastTick = 0;
  uint32_t now = HAL_GetTick();           // temps actuel en ms
  uint32_t dt_ms = now - lastTick;        // delta temps en ms
  lastTick = now;

  // Sécurité (au cas où c'est la 1ère fois / reset)
  if (dt_ms == 0) dt_ms = ENC_UPDATE_PERIOD_MS;

  float dt_s = dt_ms / 10000.0f;           // en secondes

  // ... ton code delta1 / delta2 ...
  if (dt_s <= 0) return;

  // 2. Différence totale
  int32_t diffM1 = totalCountsM1 - prevTotalM1;
  int32_t diffM2 = totalCountsM2 - prevTotalM2;

  prevTotalM1 = totalCountsM1;
  prevTotalM2 = totalCountsM2;

  // 3. Distance parcourue
  float d1 = diffM1 * (66.0f / 300.0f);
  float d2 = diffM2 * (66.0f / 300.0f);

  // 4. Vitesse
  speedM1_cm_s = d1 / (dt_s*36);
  speedM2_cm_s = d2 / (dt_s*36);

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
#ifdef USE_FULL_ASSERT
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
