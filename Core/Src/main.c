/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "motorControl.h"
#include "nRF24l01.h"
#include "mpu6500.h"
#include "flightController.h"
#include "bmp390.h"
#include "movingAvgFilter.h"
#include "filter.h"
#include "simpleKalmanFilter.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TOGGLE TEST/FLIGHT MODE HERE
#define RUN_MODE TEST
#define ANGLE_MAX 30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void initializeIWDG();
static void refreshIWDG();
static void applyKalman();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static BMP390 bmp;
static MPU6500 mpu;
static float altOffset;

/* MATRIX STUFF */
// 0.0202 is the approximate time in seconds for each loop
float32_t matrixF[2][2] = {{1, 0.0202},
                           {0, 1}};
float32_t matrixP[2][2] = {{0, 0},
                           {0, 0}};
float32_t matrixS[2][1] = {{0},
                           {0}};
float32_t matrixI[2][2] = {{1, 0},
                           {0, 1}};
float32_t matrixK[2][1] = {0};
float32_t matrixL[1][1] = {0};
float32_t matrixG[2][1] = {{0.5*0.0202*0.0202},
                           {0.0202}};
float32_t matrixQ[2][2] = {{0.000000000416241604, 0.00000004121204},
                           {0.00000004121204, 0.0000040804}}; // G * G_T * 0.01
float32_t matrixH[1][2] = {{1, 0}};
float32_t matrixR[1][1] = {{0.09}};
float32_t matrixM[1][1] = {0};

arm_matrix_instance_f32 matrixFInstance;
arm_matrix_instance_f32 matrixPInstance;
arm_matrix_instance_f32 matrixSInstance;
arm_matrix_instance_f32 matrixIInstance;
arm_matrix_instance_f32 matrixKInstance;
arm_matrix_instance_f32 matrixLInstance;
arm_matrix_instance_f32 matrixGInstance;
arm_matrix_instance_f32 matrixQInstance;
arm_matrix_instance_f32 matrixHInstance;
arm_matrix_instance_f32 matrixRInstance;
arm_matrix_instance_f32 matrixMInstance;

float32_t kalmanAltitude = 0;
float32_t kalmanVerticalVelocity = 0;
/****************/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /**
   * STUFF TODO:
   * - Move higher level functions like the angle kalman filter to the flight controller file.
   * - Make a flight controller struct? Would update all sensors on the flight controller and store the filtered angles/altitude, etc.
   */

  /* MORE MATRIX STUFF TODO REMOVE*/
  arm_mat_init_f32(&matrixFInstance, 2, 2, &matrixF[0][0]);
  arm_mat_init_f32(&matrixPInstance, 2, 2, &matrixP[0][0]);
  arm_mat_init_f32(&matrixSInstance, 2, 1, &matrixS[0][0]);
  arm_mat_init_f32(&matrixIInstance, 2, 2, &matrixI[0][0]);
  arm_mat_init_f32(&matrixKInstance, 2, 1, &matrixK[0][0]);
  arm_mat_init_f32(&matrixLInstance, 1, 1, &matrixL[0][0]);
  arm_mat_init_f32(&matrixGInstance, 2, 1, &matrixG[0][0]);
  arm_mat_init_f32(&matrixQInstance, 2, 2, &matrixQ[0][0]);
  arm_mat_init_f32(&matrixHInstance, 1, 2, &matrixH[0][0]);
  arm_mat_init_f32(&matrixRInstance, 1, 1, &matrixR[0][0]);
  arm_mat_init_f32(&matrixMInstance, 1, 1, &matrixM[0][0]);
  /********************************/

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
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2);

  uint8_t rxAddress[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
  uint8_t channelNum = 10;
  uint8_t rxPipe = 1;
  uint8_t rxData[8];

  nRF24Init();
  nRF24RxMode(rxAddress, channelNum);

  float desAngles[2] = {0, 0}; // desired angles
  float desAngleRates[3] = {0, 0, 0}; // for rate controller: desired angle rate (pitch, roll, yaw)
  int16_t ctrlSignals[3] = {0, 0, 0}; // value from 0-20, output from pid controller (represents adjustments in pitch/roll/yaw directions to hit desired value)
  uint8_t rcThrottle = 50; // raw joystick value mapped to a desired motor output
  uint8_t motorThrottle[4] = {50, 50, 50, 50}; // actual motor output, changed based on pitch, roll, yaw change from controller
  int16_t verticalVelocityMotorAdjustment = 0;
  float desVertVelocity = 1;

  unsigned int errors = 0;

  const uint8_t altSamplesToAverage = 25;

  errors += mpu6500Init(&mpu);
  flightControllerInit();
  initializeMotors();

  errors += bmp390Init(&bmp);
  movingAvgFilter altFiltered;
  movingAvgFilterInit(&altFiltered, altSamplesToAverage);

  // TODO put this stuff into a function
  // Fill altitude buffer before calculating offset
  for (int i = 0; i < altSamplesToAverage; i++) {
    errors += bmp390Update(&bmp);
    movingAvgFilterUpdate(&altFiltered, bmp.alt);
  }

  altOffset = movingAvgFilterUpdate(&altFiltered, bmp.alt);

  // Wait for buffer to fill with normalized data
  for (int i = 0; i < altSamplesToAverage; i++) {
    errors += bmp390Update(&bmp);
    movingAvgFilterUpdate(&altFiltered, bmp.alt - altOffset);
  }

  simpleKalmanFilter altitudeFilter;
  kalmanInit(&altitudeFilter, 0.1, 0.1, 0.1);

  char buf[1000];

  uint32_t prevTime = htim2.Instance->CNT;
  float prevBaroAlt = 0;

  // Start 125ms watchdog timer
//  initializeIWDG();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (errors > 0) {
      sprintf(buf, "ERRORS: %u\r\n", errors);
      HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    }

    // Read sensor values and update structs
    errors += bmp390Update(&bmp);
    errors += mpu6500Update(&mpu);
    kalmanUpdateEstimate(&altitudeFilter, bmp.alt - altOffset);

    uint32_t curTime = htim2.Instance->CNT;
    float timeDiff = (float)(curTime - prevTime) / US_TO_S;
    float baroVerticalVelocity = (altitudeFilter.curEstimate - prevBaroAlt) / timeDiff;
    prevTime = curTime;
    prevBaroAlt = altitudeFilter.curEstimate;

    applyKalman();
    sprintf(buf, "kalmanVerticalVelocity:%f\r\n", kalmanVerticalVelocity);

//    sprintf(buf, "pitch:%f,roll:%f,motorThrottle0:%u,motorThrottle1:%u,motorThrottle2:%u,motorThrottle3:%u\r\n", mpu.pitch, mpu.roll, motorThrottle[0], motorThrottle[1], motorThrottle[2], motorThrottle[3]);
//    sprintf(buf, "bmpAlt:%f, kalmanAlt:%f\r\n", bmp.alt - altOffset, altitudeFilter.curEstimate);
//    sprintf(buf, "baroVerticalVelocity:%f,motorThrottle0:%u,motorThrottle1:%u,motorThrottle2:%u,motorThrottle3:%u\r\n", baroVerticalVelocity, motorThrottle[0], motorThrottle[1], motorThrottle[2], motorThrottle[3]);
//    sprintf(buf, "timeDiff:%f\r\n", timeDiff);
    HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

    switch (RUN_MODE) {
      case TEST: {
        refreshIWDG();
        rcThrottle = 50;

        angleController(&mpu, desAngles, desAngleRates, ctrlSignals);
        updateVerticalVelocityControl(baroVerticalVelocity, desVertVelocity, &verticalVelocityMotorAdjustment);
        actuateMotors(motorThrottle, rcThrottle, ctrlSignals, verticalVelocityMotorAdjustment);

        break;
      }

      case FLIGHT: {
        // Handle user input from remote controller
        if(isDataAvailable(rxPipe)) {
          refreshIWDG();

          nRF24Receive(rxData);
          uint32_t xVal = (rxData[0] << 24 | rxData[1] << 16 | rxData[2] << 8 | rxData[3]);
          rcThrottle = mapPWM(xVal);

//          angleController(&mpu, desAngles, desAngleRates, ctrlSignals);
//          actuateMotors(motorThrottle, rcThrottle, ctrlSignals);

          // Shut off all motors permanently if angle is too sharp
          if(fabsf(mpu.pitch) > ANGLE_MAX || fabsf(mpu.roll) > ANGLE_MAX) {
            setAllMotors(50);
            while (1);
          }
        }

        break;
      }

      default: {
        break;
      }
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 320;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1600-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 250000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MPU6500_CS_Pin|NRF24_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NRF24_CE_Pin|CAM_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : BATT_ADC_Pin */
  GPIO_InitStruct.Pin = BATT_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BATT_ADC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MPU6500_CS_Pin NRF24_CS_Pin */
  GPIO_InitStruct.Pin = MPU6500_CS_Pin|NRF24_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF24_CE_Pin CAM_CS_Pin */
  GPIO_InitStruct.Pin = NRF24_CE_Pin|CAM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * I AM MANUALLY INITIALIZING THE IWDG HERE SO IT DOESN'T MESS THE PROGRAM UP WHEN THE CODE IS REGENERATED.
 */
static void initializeIWDG() {
    // Enable the IWDG by writing 0x0000 CCCC in the IWDG key register (IWDG_KR)
    IWDG->KR = 0x0000CCCC;

    // Enable register access by writing 0x0000 5555 in the IWDG key register (IWDG_KR)
    IWDG->KR = 0x00005555;

    // Write the prescaler by programming the IWDG prescaler register (IWDG_PR) from 0 to 7
    IWDG->PR = 0;

    // Write the IWDG reload register (IWDG_RLR)
    IWDG->RLR = 1000;

    // Wait for the registers to be updated (IWDG_SR = 0x0000 0000)
    while (IWDG->SR);

    // Refresh the counter value with IWDG_RLR (IWDG_KR = 0x0000 AAAA)
    IWDG->KR = 0x0000AAAA;
}

/**
 * Restarts the counter for the IWDG.
 * This function must be called in order for the IWDG to work properly.
 */
static void refreshIWDG() {
  IWDG->KR = 0x0000AAAA;
}

/**
 * god help us
 *
 * TODO
 */
static void applyKalman() {
  // S = F*S + G*Acc
  float32_t matrixGAcc[2][1];
  arm_matrix_instance_f32 matrixGAccInstance;
  arm_mat_init_f32(&matrixGAccInstance, 2, 1, &matrixGAcc[0][0]);
  arm_mat_scale_f32(&matrixGInstance, mpu.verticalAcceleration, &matrixGAccInstance); // G*Acc

  float32_t matrixFS[2][1];
  arm_matrix_instance_f32 matrixFSInstance;
  arm_mat_init_f32(&matrixFSInstance, 2, 1, &matrixFS[0][0]);
  arm_mat_mult_f32(&matrixFInstance, &matrixSInstance, &matrixFSInstance); // F*S

  arm_mat_add_f32(&matrixFSInstance, &matrixGAccInstance, &matrixSInstance); // F*S + G*Acc

  // P = F*P*F_T + Q
  float32_t matrixFP[2][2];
  arm_matrix_instance_f32 matrixFPInstance;
  arm_mat_init_f32(&matrixFPInstance, 2, 2, &matrixFP[0][0]);
  arm_mat_mult_f32(&matrixFInstance, &matrixPInstance, &matrixFPInstance); // F*P

  float32_t matrixF_T[2][2];
  arm_matrix_instance_f32 matrixF_TInstance;
  arm_mat_init_f32(&matrixF_TInstance, 2, 2, &matrixF_T[0][0]);
  arm_mat_trans_f32(&matrixFInstance, &matrixF_TInstance); // F_T

  float32_t matrixFPF_T[2][2];
  arm_matrix_instance_f32 matrixFPF_TInstance;
  arm_mat_init_f32(&matrixFPF_TInstance, 2, 2, &matrixFPF_T[0][0]);
  arm_mat_mult_f32(&matrixFPInstance, &matrixF_TInstance, &matrixFPF_TInstance); // F*P*F_T

  arm_mat_add_f32(&matrixFPF_TInstance, &matrixQInstance, &matrixPInstance); // F*P*F_T + Q

  // L = H*P*H_T + R;
  float32_t matrixHP[1][2];
  arm_matrix_instance_f32 matrixHPInstance;
  arm_mat_init_f32(&matrixHPInstance, 1, 2, &matrixHP[0][0]);
  arm_mat_mult_f32(&matrixHInstance, &matrixPInstance, &matrixHPInstance); // H*P

  float32_t matrixH_T[2][1];
  arm_matrix_instance_f32 matrixH_TInstance;
  arm_mat_init_f32(&matrixH_TInstance, 2, 1, &matrixH_T[0][0]);
  arm_mat_trans_f32(&matrixHInstance, &matrixH_TInstance); // H_T

  float32_t matrixHPH_T[1][1];
  arm_matrix_instance_f32 matrixHPH_TInstance;
  arm_mat_init_f32(&matrixHPH_TInstance, 1, 1, &matrixHPH_T[0][0]);
  arm_mat_mult_f32(&matrixHPInstance, &matrixH_TInstance, &matrixHPH_TInstance); // H*P*H_T

  arm_mat_add_f32(&matrixHPH_TInstance, &matrixRInstance, &matrixLInstance); // H*P*H_T + R

  // K = P*H_T*(L^-1)
  float32_t matrixPH_T[2][1];
  arm_matrix_instance_f32 matrixPH_TInstance;
  arm_mat_init_f32(&matrixPH_TInstance, 2, 1, &matrixPH_T[0][0]);
  arm_mat_mult_f32(&matrixPInstance, &matrixH_TInstance, &matrixPH_TInstance); // P*H_T

  float32_t matrixLinv[1][1];
  arm_matrix_instance_f32 matrixLinvInstance;
  arm_mat_init_f32(&matrixLinvInstance, 1, 1, &matrixLinv[0][0]);
  arm_mat_inverse_f32(&matrixLInstance, &matrixLinvInstance); // L^-1

  arm_mat_mult_f32(&matrixPH_TInstance, &matrixLinvInstance, &matrixKInstance); // P*H_T*(L^-1)

  matrixMInstance.pData[0] = bmp.alt - altOffset;

  // S = S + K*(M - H*S)
  float32_t matrixHS[1][1];
  arm_matrix_instance_f32 matrixHSInstance;
  arm_mat_init_f32(&matrixHSInstance, 1, 1, &matrixHS[0][0]);
  arm_mat_mult_f32(&matrixHInstance, &matrixSInstance, &matrixHSInstance); // H*S

  float32_t matrixMsubHS[1][1];
  arm_matrix_instance_f32 matrixMsubHSInstance;
  arm_mat_init_f32(&matrixMsubHSInstance, 1, 1, &matrixMsubHS[0][0]);
  arm_mat_sub_f32(&matrixMInstance, &matrixHSInstance, &matrixMsubHSInstance); // M - H*S

  float32_t matrixKMsubHS[1][1];
  arm_matrix_instance_f32 matrixKMsubHSInstance;
  arm_mat_init_f32(&matrixKMsubHSInstance, 1, 1, &matrixKMsubHS[0][0]);
  arm_mat_mult_f32(&matrixKInstance, &matrixMsubHSInstance, &matrixKMsubHSInstance); // K*(M - H*S)

  arm_mat_add_f32(&matrixSInstance, &matrixKMsubHSInstance, &matrixSInstance); // S + K*(M - H*S)

  // Update output values
  kalmanAltitude = matrixSInstance.pData[0]; // element (0,0)
  kalmanVerticalVelocity = matrixSInstance.pData[2]; // element (1,0)

  // P = (I - K*H)*P
  float32_t matrixKH[2][2];
  arm_matrix_instance_f32 matrixKHInstance;
  arm_mat_init_f32(&matrixKHInstance, 2, 2, &matrixKH[0][0]);
  arm_mat_mult_f32(&matrixKInstance, &matrixHInstance, &matrixKHInstance); // K*H

  float32_t matrixIsubKH[2][2];
  arm_matrix_instance_f32 matrixIsubKHInstance;
  arm_mat_init_f32(&matrixIsubKHInstance, 2, 2, &matrixIsubKH[0][0]);
  arm_mat_sub_f32(&matrixIInstance, &matrixKHInstance, &matrixIsubKHInstance); // I - K*H

  arm_mat_mult_f32(&matrixIsubKHInstance, &matrixPInstance, &matrixPInstance); // (I - K*H)*P
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
