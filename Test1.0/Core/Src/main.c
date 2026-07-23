/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
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

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
typedef struct {
	int32_t position;
	int32_t target;

	float speed;
	float maxSpeed;
	float acceleration;

	uint32_t stepInterval;
	uint32_t stepCounter;

	GPIO_TypeDef *STEP_Port;
	uint16_t STEP_Pin;

	GPIO_TypeDef *DIR_Port;
	uint16_t DIR_Pin;

	bool pulsePending; // STEP pin is HIGH, needs clearing at start of next tick

	int8_t encStepSign;

} Stepper_t;

Stepper_t motors[6];

/* ---- Per-joint configuration, all in JOINT order J1..J6 (base -> tip) ---- */

/* Joint (base -> tip) to motor-slot mapping. motors[] is 0-indexed: M1=0 .. M6=5.
 *   J1 Base rotation   -> M6 -> motors[5]
 *   J2 2nd link        -> M4 -> motors[3]
 *   J3 3rd link        -> M5 -> motors[4]
 *   J4 4th joint       -> M2 -> motors[1]
 *   J5 5th joint       -> M1 -> motors[0]
 *   J6 End effector rot-> M3 -> motors[2]                                     */
const uint8_t jointToMotor[6] = { 5, 3, 4, 1, 0, 2 };
const int8_t jointDir[6] = { -1, 1, -1, 1, 1, 1 };   // all positive for now

/* Per-joint speed & acceleration (steps/s and steps/s^2). Tune each link. */
const float jointMaxSpeed[6] = { 400, 400, 400, 400, 400, 400 };
const float jointAccel[6] = { 150, 150, 150, 150, 150, 150 };

/* Pulses per JOINT revolution (pulses/motor-rev × gear ratio), J1..J6 */
const float pulsesPerJointRev[6] = { 2800, 5000, 5000, 6400, 1600, 1600 };

/* Joint angle offsets (deg): software zero -> your CAD/model zero.
 J3 and J5 live on the 180°-centered band, so subtract 180 here. */
const float jointOffsetDeg[6] = { 0, 0, 180, 0, 180, 0 };

extern volatile uint8_t rxFlag;
extern char rxTemp[128];

/* Encoder CS pin per JOINT (J1..J6) */
GPIO_TypeDef *encCSPort[6] = { M6_ENC_CS_GPIO_Port, M4_ENC_CS_GPIO_Port,
M5_ENC_CS_GPIO_Port, M2_ENC_CS_GPIO_Port,
M1_ENC_CS_GPIO_Port, M3_ENC_CS_GPIO_Port };
const uint16_t encCSPin[6] = { M6_ENC_CS_Pin, M4_ENC_CS_Pin,
M5_ENC_CS_Pin, M2_ENC_CS_Pin,
M1_ENC_CS_Pin, M3_ENC_CS_Pin };

float encAngle[6] = { 0 }; /* latest encoder angle per joint, degrees */

uint16_t encRawPrev[6] = { 0 };      // last raw 14-bit reading per joint
int32_t encTicks[6] = { 0 };      // cumulative signed ticks per joint
uint8_t encInit[6] = { 0 };      // has the first reading been taken?

int32_t motorEncTicks[6] = { 0 };   // cumulative signed ticks, indexed by MOTOR
uint16_t motorEncRaw[6] = { 0 };   // last raw reading, per motor
uint8_t motorEncInit[6] = { 0 };   // first-read flag, per motor

#define CAP_MAX 10000
int32_t capTicks[CAP_MAX][6];   // encoder ticks per sample, per motor
uint16_t capCount = 0;           // how many samples captured
uint8_t capturing = 0;          // 1 while recording

/* Home reference — absolute encoder angle (deg) at the home pose */
const float homeAngle[6] = { 356.36f, 221.75f, 193.63f, 349.88f, 148.61f,
		347.36f };

/* Motor pulses per ENCODER revolution (before joint gearing).
 J4/J5/J6 are 1:1 so = pulsesPerJointRev. J1/J2/J3: motor pulses per
 one encoder turn. ADJUST J1/J2/J3 to your real motor-side pulses/rev. */
const float pulsesPerEncRev[6] = { 400, 2000, 2000, 1600, 1600, 1600 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
void Stepper_InitAll(void);
void Stepper_SetEnableM1M2M3(bool enable);
void Stepper_SetEnableM4M5M6(bool enable);
void Stepper_Update(Stepper_t *m);
void Stepper_MoveAll(int32_t steps[6]);
void Stepper_MoveAllJoints(int32_t jointSteps[6]);
uint8_t Stepper_AnyMoving(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Buzzer_SetFreq(uint32_t freq);
void AS5047P_ClearError(void);
float AS5047P_ReadJoint(uint8_t j);
void Encoders_ReadAll(void);
void AS5047P_ClearErrorCS(uint8_t j);
uint16_t AS5047P_ReadRaw(uint8_t j);
void Encoder_UpdateTicks(uint8_t j);
void Gripper_Init(void);
void Gripper_Open(void);
void Gripper_Close(void);
void Motor_UpdateEncoder(uint8_t m);
void Home_All(void);
void Encoder_Test30(void);
static uint16_t AS5047P_SPITransfer(uint8_t j, uint16_t tx);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	HAL_PWREx_EnableUSBVoltageDetector();

	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
//  MX_SDMMC1_SD_Init();
  MX_SPI3_Init();
//  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
	Stepper_InitAll();
	Stepper_SetEnableM1M2M3(true); // enable M1, M2, M3 (onboard TMC2209) drivers before moving
	Stepper_SetEnableM4M5M6(false); // enable M4, M5, M6 (external CL57T/CL57T/DM542) drivers before moving
	HAL_TIM_Base_Start_IT(&htim6);

	// Test: move ONE joint at a time to verify mapping/direction.

	int32_t jointSteps[6] = { 0, 0, 0, 0, 0, 0 };
	Stepper_MoveAllJoints(jointSteps);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	Gripper_Init();

	for (uint8_t j = 0; j < 6; j++)
		HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_SET); // all CS idle high

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t lastEncPrint = 0;

	while (1) {

//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
//		HAL_Delay(2000);
//
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2000);
//		HAL_Delay(2000);
//		Gripper_Open();
////
//		HAL_Delay(500);
//
//		Gripper_Close();
//
//		HAL_Delay(500);
//
//		Gripper_Open();
//	  while(1){
//		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		  HAL_Delay(500);   // 500 ms on, 500 ms off → 1 Hz blink
//
//	  }

//	  for (uint8_t j = 0; j < 6; j++) Encoder_UpdateTicks(j);

		if (rxFlag) {
			rxFlag = 0;
			if (strncmp(rxTemp, "HOME", 4) == 0) {
				Home_All();
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			}
			else if (strncmp(rxTemp, "TEST", 4) == 0) {
				Encoder_Test30();
			}
			else if (rxTemp[0] == 'G' && rxTemp[1] == ',') {   // <-- flat, no nested rxFlag
				if (rxTemp[2] == '1')
					Gripper_Close();
				else
					Gripper_Open();
			}
			else {
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				float ang[6] = { 0 };
				int n = sscanf(rxTemp, "%f,%f,%f,%f,%f,%f", &ang[0], &ang[1],
						&ang[2], &ang[3], &ang[4], &ang[5]);
				if (n == 6) {
					int32_t jointSteps[6];
					for (int j = 0; j < 6; j++)
						jointSteps[j] = (int32_t) lroundf(
								pulsesPerJointRev[j] * ang[j] / 360.0f);
					for (int k = 0; k < 6; k++)
						motors[k].position = 0;
					capCount = 0;
					capturing = 1;
					Stepper_MoveAllJoints(jointSteps);
				}
			}
		}
//
////		 when motion finishes, stop capturing and dump the whole buffer
//		if (capturing && !Stepper_AnyMoving()) {
//			capturing = 0;
//			for (uint16_t r = 0; r < capCount; r++) {
//				char msg[96];
//				int len = snprintf(msg, sizeof(msg),
//						"%u,%ld,%ld,%ld,%ld,%ld,%ld\r\n", r,
//						(long) capTicks[r][0], (long) capTicks[r][1],
//						(long) capTicks[r][2], (long) capTicks[r][3],
//						(long) capTicks[r][4], (long) capTicks[r][5]);
//				CDC_Transmit_FS((uint8_t*) msg, len);
//				HAL_Delay(2);           // let each USB packet flush
//			}
//			CDC_Transmit_FS((uint8_t*) "END\r\n", 5);
//		}

//		static uint32_t last = 0;
//
//		if (HAL_GetTick() - last >= 200) {
//			last = HAL_GetTick();
//
//			char msg[300];
//			int len = snprintf(msg, sizeof(msg), "enc |");
//
//			for (uint8_t j = 0; j < 6; j++) {
//				if (j==3){
//					uint16_t raw = AS5047P_ReadRaw(j);
//					float angle = AS5047P_ReadJoint(j);
//
//					if ((raw == 0xFFFF) || (angle < 0.0f)) {
//						len += snprintf(msg + len, sizeof(msg) - len,
//								" J%d:raw=ERR ang=---", j + 1);
//					} else {
//						len += snprintf(msg + len, sizeof(msg) - len,
//								" J%d:raw=%u ang=%.2f", j + 1, raw, angle);
//					}
//
//				}
//
//			}
//
//			len += snprintf(msg + len, sizeof(msg) - len, "\r\n");
//
//			CDC_Transmit_FS((uint8_t*) msg, len);
//		}

    /* USER CODE END WHILE */

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 74;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 499;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M3_Step_Pin|LED_Pin|M4_Step_Pin|M5_Step_Pin
                          |M6_Step_Pin|M2_Step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M1_EN_Pin|M2_EN_Pin|M3_EN_Pin|M4_EN_Pin
                          |M5_EN_Pin|M6_EN_Pin|M5_ENC_CS_Pin|M4_ENC_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M1_ENC_CS_Pin|M2_ENC_CS_Pin|M3_ENC_CS_Pin|SPI3_CS_Pin
                          |RGB_DIN_Pin|M1_Step_Pin|M4_Dir_Pin|M2_MS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, M2_MS2_Pin|M3_MS1_Pin|M3_Dir_Pin|M6_ENC_CS_Pin
                          |M1_Dir_Pin|M2_Dir_Pin|M5_Dir_Pin|M6_Dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : M3_Step_Pin M2_Step_Pin */
  GPIO_InitStruct.Pin = M3_Step_Pin|M2_Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M4_Step_Pin M5_Step_Pin M6_Step_Pin */
  GPIO_InitStruct.Pin = M4_Step_Pin|M5_Step_Pin|M6_Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_EN_Pin M2_EN_Pin M3_EN_Pin */
  GPIO_InitStruct.Pin = M1_EN_Pin|M2_EN_Pin|M3_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M4_EN_Pin M5_EN_Pin M6_EN_Pin */
  GPIO_InitStruct.Pin = M4_EN_Pin|M5_EN_Pin|M6_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_ENC_CS_Pin M2_ENC_CS_Pin M3_ENC_CS_Pin RGB_DIN_Pin
                           M1_Step_Pin */
  GPIO_InitStruct.Pin = M1_ENC_CS_Pin|M2_ENC_CS_Pin|M3_ENC_CS_Pin|RGB_DIN_Pin
                          |M1_Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : M2_MS2_Pin */
  GPIO_InitStruct.Pin = M2_MS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M2_MS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M3_MS1_Pin M3_Dir_Pin M6_ENC_CS_Pin M1_Dir_Pin
                           M2_Dir_Pin */
  GPIO_InitStruct.Pin = M3_MS1_Pin|M3_Dir_Pin|M6_ENC_CS_Pin|M1_Dir_Pin
                          |M2_Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : M5_ENC_CS_Pin M4_ENC_CS_Pin */
  GPIO_InitStruct.Pin = M5_ENC_CS_Pin|M4_ENC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M5_Dir_Pin M6_Dir_Pin */
  GPIO_InitStruct.Pin = M5_Dir_Pin|M6_Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI3_CS_Pin M2_MS1_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin|M2_MS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : M4_Dir_Pin */
  GPIO_InitStruct.Pin = M4_Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(M4_Dir_GPIO_Port, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Encoder_Test30(void) {
	uint8_t j = 5;                       // J6
	uint8_t m = jointToMotor[j];
	char msg[100];

	// --- averaged read BEFORE ---
	float sum = 0;
	int cnt = 0;
	for (int t = 0; t < 20; t++) {
		float v = AS5047P_ReadJoint(j);
		if (v >= 0) {
			sum += v;
			cnt++;
		}
	}
	if (cnt == 0) {
		CDC_Transmit_FS((uint8_t*) "no read\r\n", 9);
		return;
	}
	float before = sum / cnt;

	// --- command exactly 30 deg worth of steps ---
	int32_t step = (int32_t) lroundf(30.0f * pulsesPerEncRev[j] / 360.0f); // 30 deg -> steps
	int32_t stepsMotor[6] = { 0 };
	stepsMotor[m] = step;         // positive; flip sign to test other direction
	motors[m].position = 0;
	Stepper_MoveAll(stepsMotor);
	while (Stepper_AnyMoving()) {
	}
	HAL_Delay(100);                      // let it settle

	// --- averaged read AFTER ---
	sum = 0;
	cnt = 0;
	for (int t = 0; t < 20; t++) {
		float v = AS5047P_ReadJoint(j);
		if (v >= 0) {
			sum += v;
			cnt++;
		}
	}
	if (cnt == 0) {
		CDC_Transmit_FS((uint8_t*) "no read2\r\n", 10);
		return;
	}
	float after = sum / cnt;

	// --- shortest signed encoder change ---
	float delta = after - before;
	while (delta > 180.0f)
		delta -= 360.0f;
	while (delta < -180.0f)
		delta += 360.0f;

	int L =
			snprintf(msg, sizeof(msg),
					"before=%.2f after=%.2f | commanded=30.0 measured=%.2f | steps=%ld\r\n",
					before, after, delta, (long) step);
	CDC_Transmit_FS((uint8_t*) msg, L);
}

void Home_J6(void) {
	uint8_t j = 5;                      // J6
	uint8_t m = jointToMotor[j];        // Corresponding motor

	//---------------- Get average encoder reading ----------------//

	float sum = 0.0f;
	uint8_t count = 0;

	for (int i = 0; i < 20; i++) {
		float angle = AS5047P_ReadJoint(j);

		if (angle >= 0.0f) {
			sum += angle;
			count++;
		}
	}

	// No valid readings
	if (count == 0)
		return;

	float now = sum / (float) count;

	//---------------- Calculate shortest angular error ------------//

	float errDeg = homeAngle[j] - now;

	while (errDeg > 180.0f)
		errDeg -= 360.0f;

	while (errDeg < -180.0f)
		errDeg += 360.0f;

	//---------------- Already at home ? --------------------------//

	if (fabsf(errDeg) < 1.0f)
		return;

	//---------------- Convert angle -> motor steps ----------------//

	int32_t step = (int32_t) lroundf(
			motors[m].encStepSign * errDeg * pulsesPerEncRev[j] / 360.0f);

	//---------------- Move the motor ------------------------------//

	int32_t stepsMotor[6] = { 0 };

	stepsMotor[m] = step;

	motors[m].position = 0;
	motors[m].target = 0;

	Stepper_MoveAll(stepsMotor);

	//---------------- Debug message -------------------------------//

	char msg[200];

	int len = snprintf(msg, sizeof(msg),

	"now=%.2f\r\n"
			"err=%.2f\r\n"
			"encSign=%d\r\n"
			"step=%ld\r\n",

	now, errDeg, motors[m].encStepSign, (long) step);

	CDC_Transmit_FS((uint8_t*) msg, len);
}




//5th joint
void Home_J5(void)
{
   uint8_t j = 4;                      // J5
   uint8_t m = jointToMotor[j];        // Corresponding motor

   //---------------- Get average encoder reading ----------------//

   float sum = 0.0f;
   uint8_t count = 0;

   for (int i = 0; i < 20; i++)
   {
       float angle = AS5047P_ReadJoint(j);

       if (angle >= 0.0f)
       {
           sum += angle;
           count++;
       }
   }

   // No valid readings
   if (count == 0)
       return;

   float now = sum / (float)count;


   //---------------- Calculate angle to DECREASE ----------------//

   float errDeg;

   if (now >= homeAngle[j])
   {
       errDeg = now - homeAngle[j];
   }
   else
   {
       errDeg = now + (360.0f - homeAngle[j]);
   }


   //---------------- Already at home ? --------------------------//

   if (fabsf(errDeg) < 1.0f)
       return;


   //---------------- Convert angle -> motor steps ----------------//

   int32_t step = (int32_t)lroundf(
           motors[m].encStepSign *
           (-errDeg) *
           pulsesPerEncRev[j] /
           360.0f);


   //---------------- Move the motor ------------------------------//

   int32_t stepsMotor[6] = {0};

   stepsMotor[m] = step;

   motors[m].position = 0;
   motors[m].target = 0;

   Stepper_MoveAll(stepsMotor);


   //---------------- Debug message -------------------------------//

   char msg[200];

   int len = snprintf(msg,
                      sizeof(msg),
                      "now=%.2f\r\n"
                      "home=%.2f\r\n"
                      "err=%.2f\r\n"
                      "encSign=%d\r\n"
                      "step=%ld\r\n",
                      now,
                      homeAngle[j],
                      errDeg,
                      motors[m].encStepSign,
                      (long)step);

   CDC_Transmit_FS((uint8_t *)msg, len);
}

void Home_J4(void)
{
	uint8_t j = 3;                      // J3
	uint8_t m = jointToMotor[j];        // Corresponding motor

	//------------- Get average encoder reading ----------------//

	float sum = 0.0f;
	uint8_t count = 0;

	for (int i = 0; i < 20; i++)
	{
		float angle = AS5047P_ReadJoint(j);

		if (angle >= 0.0f)
		{
			sum += angle;
			count++;
		}
	}

	// No valid readings
	if (count == 0)
		return;

	float now = sum / (float) count;


	//------------- First motion : DECREASE to 200° ------------//

	float decreaseDeg;

	if (now >= 200.0f)
	{
		decreaseDeg = now - 200.0f;
	}
	else
	{
		decreaseDeg = now + (360.0f - 200.0f);
	}


	if (fabsf(decreaseDeg) >= 1.0f)
	{
		int32_t step1 = (int32_t) lroundf(
				motors[m].encStepSign *
				(-decreaseDeg) *
				pulsesPerEncRev[j] /
				360.0f);

		int32_t stepsMotor[6] = { 0 };

		stepsMotor[m] = step1;

		motors[m].position = 0;
		motors[m].target = 0;

		Stepper_MoveAll(stepsMotor);

		while (Stepper_AnyMoving())
		{
		}

		HAL_Delay(100);
	}


	//------------- Read encoder AGAIN -------------------------//

	sum = 0.0f;
	count = 0;

	for (int i = 0; i < 20; i++)
	{
		float angle = AS5047P_ReadJoint(j);

		if (angle >= 0.0f)
		{
			sum += angle;
			count++;
		}
	}

	// No valid readings
	if (count == 0)
		return;

	now = sum / (float) count;


	//------------- Second motion : INCREASE to HOME -----------//

	float increaseDeg;

	if (homeAngle[j] >= now)
	{
		increaseDeg = homeAngle[j] - now;
	}
	else
	{
		increaseDeg = (360.0f - now) + homeAngle[j];
	}


	if (fabsf(increaseDeg) >= 1.0f)
	{
		int32_t step2 = (int32_t) lroundf(
				motors[m].encStepSign *
				(increaseDeg) *
				pulsesPerEncRev[j] /
				360.0f);

		int32_t stepsMotor[6] = { 0 };

		stepsMotor[m] = step2;

		motors[m].position = 0;
		motors[m].target = 0;

		Stepper_MoveAll(stepsMotor);

		while (Stepper_AnyMoving())
		{
		}

		HAL_Delay(100);
	}


	//------------- Debug message ------------------------------//

	char msg[250];

	int len = snprintf(msg,
			sizeof(msg),

			"Final Encoder Angle = %.2f\r\n"
			"Home Angle          = %.2f\r\n"
			"Decrease Motion     = %.2f\r\n"
			"Increase Motion     = %.2f\r\n",

			now,
			homeAngle[j],
			decreaseDeg,
			increaseDeg);

	CDC_Transmit_FS((uint8_t*) msg, len);
}

void Home_All(void)
{
	//----------------------------------------------------------
	// J6
	//----------------------------------------------------------

	Home_J6();

	while (Stepper_AnyMoving())
	{
	}

	HAL_Delay(100);


	//----------------------------------------------------------
	// J5
	//----------------------------------------------------------

	Home_J5();

	while (Stepper_AnyMoving())
	{
	}

	HAL_Delay(100);


	//----------------------------------------------------------
	// J4
	//----------------------------------------------------------

	Home_J4();

	while (Stepper_AnyMoving())
	{
	}

	HAL_Delay(100);


	//----------------------------------------------------------
	// DONE
	//----------------------------------------------------------

	char msg[] = "\r\nAll joints homed successfully.\r\n";

	CDC_Transmit_FS((uint8_t *)msg, sizeof(msg) - 1);
}

/* Read the encoder on motor m and update its cumulative tick count.
 Safe to call from the TIM6 ISR at 2 kHz (short SPI timeout). */
void Motor_UpdateEncoder(uint8_t m) {
	// find which joint uses this motor, to reach its CS pin
	int8_t j = -1;
	for (uint8_t k = 0; k < 6; k++)
		if (jointToMotor[k] == m) {
			j = k;
			break;
		}
	if (j < 0)
		return;

	uint16_t cmd = 0xFFFF, rx = 0;
	HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_RESET);
	HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &cmd,
			(uint8_t*) &rx, 1, 2); // 2ms max
	HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_SET);

	if (st != HAL_OK || (rx & 0x4000))
		return;      // bad read or error flag -> skip
	uint16_t raw = rx & 0x3FFF;

	if (!motorEncInit[m]) {
		motorEncRaw[m] = raw;
		motorEncInit[m] = 1;
		return;
	}

	int32_t d = (int32_t) raw - (int32_t) motorEncRaw[m];
	if (d > 8192)
		d -= 16384;
	if (d < -8192)
		d += 16384;
	motorEncTicks[m] += d;
	motorEncRaw[m] = raw;
}

/* Configure TIM1 for servo use: 1 us tick, 20 ms (50 Hz) period, start CH2 */
void Gripper_Init(void) {
	uint32_t timClk = HAL_RCC_GetPCLK2Freq() * 2;      // TIM1 is on APB2

	__HAL_TIM_SET_PRESCALER(&htim1, (timClk / 1000000u) - 1);  // 1 MHz tick
	__HAL_TIM_SET_AUTORELOAD(&htim1, 20000 - 1);               // 20 ms frame

	htim1.Instance->EGR = TIM_EGR_UG;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

//	__HAL_TIM_MOE_ENABLE(&htim1);
}

/* Set servo pulse width in microseconds (typical range 1000-2000) */
static void Gripper_SetPulse(uint16_t us) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, us);
}

void Gripper_Open(void) {
	Gripper_SetPulse(930);
}   // tune this value
void Gripper_Close(void) {
	Gripper_SetPulse(1100);
}   // tune this value   // tune this value

/* Read raw 14-bit count for joint j. Returns 0xFFFF on error. */
uint16_t AS5047P_ReadRaw(uint8_t j) {
	uint16_t cmd = 0xFFFF, rx = 0;
	for (int attempt = 0; attempt < 3; attempt++) {
		HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &cmd, (uint8_t*) &rx, 1,
				100);
		HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_SET);
		if (!(rx & 0x4000))
			return (rx & 0x3FFF);
		AS5047P_ClearErrorCS(j);
	}
	return 0xFFFF;
}

//uint16_t AS5047P_ReadRaw(uint8_t j)
//{
//	uint16_t angleCmd = 0x3FFF;
//	uint16_t nopCmd = 0x0000;
//
//	uint16_t rx;
//
//	/* Request the ANGLE register */
//	AS5047P_SPITransfer(j, angleCmd);
//
//	/* Read the result */
//	rx = AS5047P_SPITransfer(j, nopCmd);
//
//	/* Check the error flag */
//	if (rx & 0x4000)
//	{
//		AS5047P_ClearErrorCS(j);
//		return 0xFFFF;
//	}
//
//	return (rx & 0x3FFF);
//}

/* Update cumulative signed tick counter for joint j (handles 0/16384 wrap).
 +ve = increasing raw count, -ve = decreasing. */
void Encoder_UpdateTicks(uint8_t j) {
	uint16_t raw = AS5047P_ReadRaw(j);
	if (raw == 0xFFFF)
		return;                 // bad read, skip

	if (!encInit[j]) {
		encRawPrev[j] = raw;
		encInit[j] = 1;
		return;
	}

	int32_t d = (int32_t) raw - (int32_t) encRawPrev[j];
	if (d > 8192)
		d -= 16384;                 // wrapped downward through 0
	if (d < -8192)
		d += 16384;                 // wrapped upward through max
	encTicks[j] += d;
	encRawPrev[j] = raw;
}

void AS5047P_ClearErrorCS(uint8_t j) {
	uint16_t cmd = 0x4001, rx;
	HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &cmd, (uint8_t*) &rx, 1, 100);
	HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_SET);
	HAL_Delay(1);
	cmd = 0xFFFF;
	HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &cmd, (uint8_t*) &rx, 1, 100);
	HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_SET);
}

///* Read encoder for joint j (0..5). Returns degrees, or -1 on error. */
//float AS5047P_ReadJoint(uint8_t j) {
//	uint16_t cmd = 0xFFFF, rx = 0;
//	for (int attempt = 0; attempt < 3; attempt++) {
//		HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_RESET);
//		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &cmd, (uint8_t*) &rx, 1,
//				100);
//		HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_SET);
//
//		if (!(rx & 0x4000))
//			return ((rx & 0x3FFF) * 360.0f) / 16384.0f;
//
//		AS5047P_ClearErrorCS(j);
//	}
//	return -1.0f;
//}

static uint16_t AS5047P_SPITransfer(uint8_t j, uint16_t tx) {
	uint16_t rx = 0;

	HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &tx, (uint8_t*) &rx, 1, 100);

	HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_SET);

	return rx;
}
//float AS5047P_ReadJoint(uint8_t j)
//{
//	uint16_t raw;
//
//	raw = AS5047P_ReadRaw(j);
//
//	if(raw == 0xFFFF)
//		return -1.0f;
//
//	return ((float)raw * 360.0f) / 16384.0f;
//}

float AS5047P_ReadJoint(uint8_t j) {
	uint16_t cmd = 0xFFFF, rx = 0;

	for (int attempt = 0; attempt < 3; attempt++) {
		HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_RESET);

		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &cmd, (uint8_t*) &rx, 1,
				100);

		HAL_GPIO_WritePin(encCSPort[j], encCSPin[j], GPIO_PIN_SET);

		if (!(rx & 0x4000)) {
			float angle = ((rx & 0x3FFF) * 360.0f) / 16384.0f;

			/* Only for Joint 6 */
			if (j == 5 || j == 4 || j == 3) {
				angle = ((angle - 180.0f) * 360.0f) / 180.0f;

				if (angle < 0.0f)
					angle += 360.0f;

				if (angle >= 360.0f)
					angle -= 360.0f;
			}

			return angle;
		}

		AS5047P_ClearErrorCS(j);
	}

	return -1.0f;
}

/* Set buzzer tone frequency (Hz) on TIM2_CH2 / PA1. freq=0 -> silent. */
void Buzzer_SetFreq(uint32_t freq) {
	if (freq == 0) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		return;
	}
	uint32_t timClk = HAL_RCC_GetPCLK1Freq() * 2;   // TIM2 kernel clock
	uint32_t psc = (timClk / 1000000u) - 1;      // counter ticks at 1 MHz
	uint32_t arr = (1000000u / freq) - 1;

	__HAL_TIM_SET_PRESCALER(&htim2, psc);
	__HAL_TIM_SET_AUTORELOAD(&htim2, arr);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, arr / 2);   // 50% duty
	htim2.Instance->EGR = TIM_EGR_UG;               // load new PSC/ARR now
}

/**
 * @brief Drive EN for M1-M3 (CL57T x2 + TB6600, all wired common-anode:
 *        EN+ tied to external +5V, EN- to this open-drain GPIO pin).
 *
 *        Open-drain behavior: GPIO_PIN_RESET (LOW) actively sinks current
 *        -> opto LED conducts. GPIO_PIN_SET (HIGH) releases the pin
 *        (Hi-Z) -> no current path -> opto LED off.
 *
 *        IMPORTANT - polarity is a best guess from datasheets, not yet
 *        verified on your bench:
 *          - CL57T (M1, M2): spec implies LED CONDUCTING = enabled.
 *            -> drive LOW to enable.
 *          - TB6600 (M3): spec implies LED CONDUCTING = "EN valid" =
 *            motor FREE/disabled (opposite convention).
 *            -> drive HIGH (released) to enable.
 *
 *        TEST THIS: with the motor powered and enabled per the logic
 *        below, you should NOT be able to easily turn the shaft by hand
 *        (holding torque present). If it spins freely instead, that
 *        axis's polarity is backwards - flip the corresponding line below.
 */
void Stepper_SetEnableM1M2M3(bool enable) {
	// Onboard TMC2209 x3 (M1, M2, M3): EN is active-LOW (LOW = driver enabled).
	// All three now share identical push-pull, active-low polarity - if any
	// single axis still doesn't lock after this change, flip only that line.
	HAL_GPIO_WritePin(M1_EN_GPIO_Port, M1_EN_Pin,
			enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(M2_EN_GPIO_Port, M2_EN_Pin,
			enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(M3_EN_GPIO_Port, M3_EN_Pin,
			enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief Enable/disable M4-M6 external drivers (open-drain, common-anode).
 *        M4, M5 = CL57T (NEMA24): LOW = enabled.
 *        M6 = DM542 (NEMA23): HIGH (released) = enabled - opposite of M4/M5.
 *        Same polarity logic that used to live on M1-M3 before those moved
 *        to the onboard TMC2209s - verify on hardware, one axis at a time,
 *        same as the original M1-M3 bring-up.
 */
void Stepper_SetEnableM4M5M6(bool enable) {
	HAL_GPIO_WritePin(M4_EN_GPIO_Port, M4_EN_Pin,
			enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(M5_EN_GPIO_Port, M5_EN_Pin,
			enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(M6_EN_GPIO_Port, M6_EN_Pin,
			enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Stepper_InitAll(void) {
	// Motor 1
	motors[0].STEP_Port = M1_Step_GPIO_Port;
	motors[0].STEP_Pin = M1_Step_Pin;
	motors[0].DIR_Port = M1_Dir_GPIO_Port;
	motors[0].DIR_Pin = M1_Dir_Pin;

	// Motor 2
	motors[1].STEP_Port = M2_Step_GPIO_Port;
	motors[1].STEP_Pin = M2_Step_Pin;
	motors[1].DIR_Port = M2_Dir_GPIO_Port;
	motors[1].DIR_Pin = M2_Dir_Pin;

	// Motor 3
	motors[2].STEP_Port = M3_Step_GPIO_Port;
	motors[2].STEP_Pin = M3_Step_Pin;
	motors[2].DIR_Port = M3_Dir_GPIO_Port;
	motors[2].DIR_Pin = M3_Dir_Pin;

	// Motor 4
	motors[3].STEP_Port = M4_Step_GPIO_Port;
	motors[3].STEP_Pin = M4_Step_Pin;
	motors[3].DIR_Port = M4_Dir_GPIO_Port;
	motors[3].DIR_Pin = M4_Dir_Pin;

	// Motor 5
	motors[4].STEP_Port = M5_Step_GPIO_Port;
	motors[4].STEP_Pin = M5_Step_Pin;
	motors[4].DIR_Port = M5_Dir_GPIO_Port;
	motors[4].DIR_Pin = M5_Dir_Pin;

	// Motor 6
	motors[5].STEP_Port = M6_Step_GPIO_Port;
	motors[5].STEP_Pin = M6_Step_Pin;
	motors[5].DIR_Port = M6_Dir_GPIO_Port;
	motors[5].DIR_Pin = M6_Dir_Pin;

	for (int j = 0; j < 6; j++) {
		uint8_t m = jointToMotor[j];
		motors[m].position = 0;
		motors[m].target = 0;

		motors[m].speed = 0;
		motors[m].maxSpeed = jointMaxSpeed[j];   // per-joint now
		motors[m].acceleration = jointAccel[j];      // per-joint now

		motors[m].stepInterval = 1000;
		motors[m].stepCounter = 0;
		motors[m].pulsePending = false;
	}

	/* step -> encoder direction sign, per joint (measured manually).
	 0 = unknown / no encoder wired. */
	int8_t encSignByJoint[6] = { +1,   // J1  <- set from your measurement
			0,   // J2  (no encoder wired)
			0,   // J3  (no encoder wired)
			+1,   // J4  <- set from your measurement
			-1,   // J5  <- set from your measurement
			-1    // J6  (no encoder wired)
			};
	for (int j = 0; j < 6; j++)
		motors[jointToMotor[j]].encStepSign = encSignByJoint[j];
}

/**
 * @brief Command all 6 axes to move by the given number of steps
 *        (relative to their current position). Direction is encoded
 *        in the sign of each value (negative = reverse).
 *        The trapezoidal ramp in Stepper_Update() takes it from there.
 */
void Stepper_MoveAll(int32_t steps[6]) {
	for (int i = 0; i < 6; i++) {
		motors[i].target = motors[i].position + steps[i];
	}
}

/* Command all axes in JOINT order J1..J6. Handles wiring order + direction. */
void Stepper_MoveAllJoints(int32_t jointSteps[6]) {
	for (int j = 0; j < 6; j++) {
		uint8_t m = jointToMotor[j];
		motors[m].target = motors[m].position + jointDir[j] * jointSteps[j];
	}
}
/**
 * @brief Returns 1 if any axis still has steps left to complete.
 */
uint8_t Stepper_AnyMoving(void) {
	for (int i = 0; i < 6; i++) {
		if (motors[i].position != motors[i].target)
			return 1;
	}
	return 0;
}

void Stepper_Update(Stepper_t *m) {
	/* Finish whatever pulse was started on the PREVIOUS tick first.
	 * This guarantees every HIGH pulse lasts one full 20us tick (50kHz
	 * TIM6) instead of a hand-tuned busy-wait that was only ~1-2us -
	 * below the minimum pulse width the drivers need. */
	if (m->pulsePending) {
		HAL_GPIO_WritePin(m->STEP_Port, m->STEP_Pin, GPIO_PIN_RESET);
		m->pulsePending = false;
	}

	if (m->position == m->target)
		return;

	int32_t distance = m->target - m->position;

	// Direction
	HAL_GPIO_WritePin(m->DIR_Port, m->DIR_Pin,
			(distance > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	float dt = 0.0005f; /* matches TIM6 now reconfigured to 50kHz */

	float stepsToStop = (m->speed * m->speed) / (2.0f * m->acceleration);

	if (fabs(distance) < stepsToStop) {
		m->speed -= m->acceleration * dt;
		if (m->speed < 0)
			m->speed = 0;
	} else {
		if (m->speed < m->maxSpeed) {
			m->speed += m->acceleration * dt;
			if (m->speed > m->maxSpeed)
				m->speed = m->maxSpeed;
		}
	}

	if (m->speed > 0)
		m->stepInterval = (uint32_t) (2000.0f / m->speed);

	m->stepCounter++;

	if (m->stepCounter >= m->stepInterval) {
		m->stepCounter = 0;

		HAL_GPIO_WritePin(m->STEP_Port, m->STEP_Pin, GPIO_PIN_SET);
		m->pulsePending = true; /* cleared at the START of the next tick, not here */

		if (distance > 0)
			m->position++;
		else
			m->position--;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		for (int i = 0; i < 6; i++) {
			Stepper_Update(&motors[i]);
			Motor_UpdateEncoder(i);   // <-- read this motor's encoder each tick
		}

		if (capturing && capCount < CAP_MAX) {
			for (int i = 0; i < 6; i++)
				capTicks[capCount][i] = motorEncTicks[i];
			capCount++;
		}
	}
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	__disable_irq();
	while (1) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		for (volatile int i = 0; i < 400000; i++)
			;   // fast blink (busy-wait, since IRQs are off)
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
