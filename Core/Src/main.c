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
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint8_t ID;
	volatile int16_t count;//pulse
	const float l_angle;//locate
	volatile float vel;//rad/ms
} Encoder;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1415


#define r 60//mm

#define R1 187//mm
#define R2 73//mm


#define swright 0x0c
#define swleft  0xc0
#define swfront 0x03
#define swback  0x30

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_FilterTypeDef sFilterConfig;

uint8_t RxData[8] = {};
uint8_t TxData[8] = {};
uint32_t TxMailbox;

const float ppr[3] = {1000, 1100, 1050};

Encoder encoder[3] = {
		{0, 0, 0},
		{1, 0, 0},
		{2, 0, 0}
};

volatile float x = 0, y = 0;//mm
volatile float theta = 0;//rad
volatile uint8_t swstate = 0;//前�?�右、後ろ、左

uint8_t state = 0;
uint8_t sub_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {

	        /* Retrieve Rx messages from RX FIFO0 */

		if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK) {
			printf("fdcan_getrxmessage is error\r\n");
			Error_Handler();
		}

		if (RxHeader.Identifier == 0x100) {
			state = RxData[0];
			sub_state = RxData[1];
		}
	}
}

int16_t read_encoder_value_1(void)
{
  int32_t count_t = 0;
  uint32_t enc_buff = TIM2->CNT;
  TIM2->CNT = 0;
  if (enc_buff > 0x8fffffff)
  {
    count_t = (int32_t)enc_buff*-1;
  }
  else
  {
    count_t = (int32_t)enc_buff;
  }
  return (int16_t)count_t;
}

int16_t read_encoder_value_2(void)
{
  int32_t count_t = 0;
  uint32_t enc_buff = TIM3->CNT;
  TIM3->CNT = 0;
  if (enc_buff > 0x8fffffff)
  {
    count_t = (int32_t)enc_buff*-1;
  }
  else
  {
    count_t = (int32_t)enc_buff;
  }
  return (int16_t)count_t;
}

int16_t read_encoder_value_3(void)
{
  int32_t count_t = 0;
  uint32_t enc_buff = TIM5->CNT;
  TIM5->CNT = 0;
  if (enc_buff > 0x8fffffff)
  {
    count_t = (int32_t)enc_buff*-1;
  }
  else
  {
    count_t = (int32_t)enc_buff;
  }
  return (int16_t)count_t;
}

void vel_calc(float Theta, float w1, float w2, float w3, float *Vx, float *Vy, float *Omega){

	float w[3] = {w1, w2, w3};

	float sint = sin(Theta);
	float cost = cos(Theta);

	float a[3][3] = {
			{-sint, cost, R1},
			{sint, -cost, R1},
			{cost, sint, R2}
	};

	float det = a[0][0]*a[1][1]*a[2][2];
	det += a[1][0]*a[2][1]*a[0][2];
	det += a[2][0]*a[0][1]*a[1][2];
	det -= a[2][0]*a[1][1]*a[0][2];
	det -= a[1][0]*a[0][1]*a[2][2];
	det -= a[0][0]*a[2][1]*a[1][2];

	float a_in[3][3] = {
			{( a[1][1]*a[2][2]-a[1][2]*a[2][1])/det, (-a[0][1]*a[2][2]+a[0][2]*a[2][1])/det, ( a[0][1]*a[1][2]-a[0][2]*a[1][1])/det},
			{(-a[1][0]*a[2][2]+a[1][2]*a[2][0])/det, ( a[0][0]*a[2][2]-a[0][2]*a[2][0])/det, (-a[0][0]*a[1][2]+a[0][2]*a[1][0])/det},
			{( a[1][0]*a[2][1]-a[1][1]*a[2][0])/det, (-a[0][0]*a[2][1]+a[0][1]*a[2][0])/det, ( a[0][0]*a[1][1]-a[0][1]*a[1][0])/det}
	};

	*Vx =    r*(a_in[0][0]*w[0]+a_in[0][1]*w[1]+a_in[0][2]*w[2])/2;
	*Vy =    r*(a_in[1][0]*w[0]+a_in[1][1]*w[1]+a_in[1][2]*w[2])/2;
	*Omega = r*(a_in[2][0]*w[0]+a_in[2][1]*w[1]+a_in[2][2]*w[2])/2;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim6){
		float dt = 100;
		float vx = 0, vy = 0;//mm/ms
		float omega = 0;//rad/ms
		encoder[0].count = read_encoder_value_1();
		encoder[1].count = read_encoder_value_2();
		encoder[2].count = read_encoder_value_3();


		for (int i = 0; i < 3;i++) {
			encoder[i].vel = 2*PI*(encoder[i].count/ppr[i])/dt;
		}

		vel_calc(theta, encoder[0].vel, encoder[1].vel, encoder[2].vel, &vx, &vy, &omega);

		x += vx * dt;
		y += vy * dt;
		theta += omega * dt	;

		theta = fmodf(theta, 2*PI);
//		uint16_t theta_syi;
//		uint8_t theta_se;
//		if (theta > 0){
//			theta_se = (int16_t)theta;
//			float theta_syf = theta - (int16_t)theta;
//			theta_syi = (uint16_t)(theta_syf*10000);
//		}
//		else{
//			theta_se = (int16_t)theta;
//			float theta_k = -theta;
//			float theta_syf = theta_k - (int16_t)theta_k;
//			theta_syi = (uint16_t)(theta_syf*10000);
//
//		}
		if (4 == state){
			if (0 == sub_state){
				if ((swstate & swfront) == swfront) {//front
					y = 1860;//edge y
					TxData[6] = 1;
				}
				else {
					TxData[6] = 0;
				}
			}
			else if (1 == sub_state) {
				if ((swstate & swright) == swright) {//right
					x = 0;//edge x
					TxData[6] = 1;
				}
				else {
					TxData[6] = 0;
				}
			}
			else if (2 == sub_state) {
				if ((swstate & swleft) == swleft) {//left
					y = 0;//
					TxData[6] = 1;
				}
				else {
					TxData[6] = 0;
				}
			}
			else if (3 == sub_state) {
				if ((swstate & swfront) == swfront) {//front
					x = -2900;//edge x
					TxData[6] = 1;
				}
				else {
					TxData[6] = 0;
				}
			}
			else if (4 == sub_state) {
				if ((swstate & swleft) == swleft) {//left
					y = 0;//edge y
					TxData[6] = 1;
				}
				else {
					TxData[6] = 0;
				}
			}
/*			else if (5 == sub_state) {
				if ((swstate & 0x03) == 0x03) {
					x = 0;//edge x
					RxData[6] = 1;
				}
				else {
					RxData[6] = 0;
				}
			}
			else if (6 == sub_state) {
				if ((swstate & 0xc0) == 0xc0) {
					y = 0;//edge y
					RxData[6] = 1;
				}
				else {
					RxData[6] = 0;
				}
			}*/
			else {
				TxData[6] = 0;
			}
		}
		else {
			TxData[6] = 0;
		}

		theta *= 400;
		TxData[0] = (int16_t)(x) >> 8;
		TxData[1] = (uint8_t)((int16_t)(x) & 0xff);
		TxData[2] = (int16_t)(y) >> 8;
		TxData[3] = (uint8_t)((int16_t)(y) & 0xff);
		TxData[4] = (int16_t)(theta) >> 8;
		TxData[5] = (uint8_t)((int16_t)(theta) & 0xff);
		theta /= 400;
		//TxData[6] = theta_se;
		TxHeader.Identifier = 0x400;
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK){
			printf("add_message_zahyo is error\r\n");
			Error_Handler();
		}
	}

	if(htim == &htim4){
		if(HAL_GPIO_ReadPin(lmt_sw1_GPIO_Port,lmt_sw1_Pin) == GPIO_PIN_RESET){
			//printf("sw1 on");
			swstate = swstate|0x01;
		}else {
			swstate = swstate&0xfe;
		}
		if(HAL_GPIO_ReadPin(lmt_sw2_GPIO_Port,lmt_sw2_Pin) == GPIO_PIN_RESET){
			//printf("sw2 on");
			swstate = swstate|0x02;
		}else {
			swstate = swstate&0xfd;
		}
		if(HAL_GPIO_ReadPin(lmt_sw3_GPIO_Port,lmt_sw3_Pin) == GPIO_PIN_RESET){
			//printf("sw3 on");
			swstate = swstate|0x04;
		}else {
			swstate = swstate&0xfb;
		}
		if(HAL_GPIO_ReadPin(lmt_sw4_GPIO_Port,lmt_sw4_Pin) == GPIO_PIN_RESET){
			//printf("sw4 on\r\n");
			swstate = swstate|0x08;
		}else {
			swstate = swstate&0xf7;
		}
		if(HAL_GPIO_ReadPin(lmt_sw5_GPIO_Port,lmt_sw5_Pin) == GPIO_PIN_RESET){
			swstate = swstate|0x10;
		}else {
			swstate = swstate&0xef;
		}
		if(HAL_GPIO_ReadPin(lmt_sw6_GPIO_Port,lmt_sw6_Pin) == GPIO_PIN_RESET){
			swstate = swstate|0x20;
		}else {
			swstate = swstate&0xdf;
		}
		if(HAL_GPIO_ReadPin(lmt_sw7_GPIO_Port,lmt_sw7_Pin) == GPIO_PIN_RESET){
			swstate = swstate|0x40;
		}else {
			swstate = swstate&0xbf;
		}
		if(HAL_GPIO_ReadPin(lmt_sw8_GPIO_Port,lmt_sw8_Pin) == GPIO_PIN_RESET){
			swstate = swstate|0x80;
		}else {
			swstate = swstate&0x7f;
		}
//		if ((swstate & swleft) == swleft) {//left
//			printf("left on\r\n");
//		}
//		if ((swstate & swright) == swright) {//left
//			printf("right on\r\n");
//		}
//		if ((swstate & swfront) == swfront) {//left
//			printf("front on\r\n");
//		}
	}
}


void FDCAN_RxTxSettings(void){
	FDCAN_FilterTypeDef FDCAN_Filter_settings;
	FDCAN_Filter_settings.IdType = FDCAN_STANDARD_ID;
	FDCAN_Filter_settings.FilterIndex = 0;
	FDCAN_Filter_settings.FilterType = FDCAN_FILTER_RANGE;
	FDCAN_Filter_settings.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	FDCAN_Filter_settings.FilterID1 = 0x000;
	FDCAN_Filter_settings.FilterID2 = 0x510;

	TxHeader.Identifier = 0x400;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader.FDFormat = FDCAN_FD_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;


	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter_settings) != HAL_OK){
		printf("fdcan_configfilter is error\r\n");
		Error_Handler();
	}

	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_FILTER_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK){
		printf("fdcan_configglobalfilter is error\r\n");
		Error_Handler();
	}

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		printf("fdcan_start is error\r\n");
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK){
		printf("fdcan_activatenotification is error\r\n");
		Error_Handler();
	}
}


int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	setbuf(stdout, NULL);

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_FDCAN1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  printf("start\r\n");
  printf("can tx start\r\n");
  FDCAN_RxTxSettings();
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim4);
  //int en = 0;
  while (1)
  {
	  //int16_t count = read_encoder_value_1();
	  //printf("%d\r\n", count);
	  //printf("%d.%d.%d\r\n", encoder[0].count, encoder[1].count, encoder[2].count);
	  //printf("%d", encoder[2].count);
	  //printf("\r\n");
	  //en += read_encoder_value_3();
	  //printf("%d\r\n", en);
	  //printf("(%f, %f, %f)\r\n", x, y, theta);
	  //printf("swstate:%d\r\n",swstate);
	  HAL_Delay(1);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 2*2;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
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
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  htim6.Init.Prescaler = 999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(eno_rst_GPIO_Port, eno_rst_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : eno_rst_Pin */
  GPIO_InitStruct.Pin = eno_rst_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(eno_rst_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : lmt_sw7_Pin lmt_sw6_Pin lmt_sw5_Pin lmt_sw1_Pin */
  GPIO_InitStruct.Pin = lmt_sw7_Pin|lmt_sw6_Pin|lmt_sw5_Pin|lmt_sw1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : lmt_sw8_Pin lmt_sw3_Pin lmt_sw2_Pin lmt_sw4_Pin */
  GPIO_InitStruct.Pin = lmt_sw8_Pin|lmt_sw3_Pin|lmt_sw2_Pin|lmt_sw4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  printf("Error\r\n");
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
