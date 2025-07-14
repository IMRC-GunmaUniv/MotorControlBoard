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
#include "stdio.h"
#include "stdlib.h"
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
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader_RM;
uint32_t TxMailbox_RM;
uint8_t TxData_RM[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // CAN送信用データ

uint32_t id_ESP;
uint32_t dlc_ESP;
uint8_t data_ESP[8];
CAN_RxHeaderTypeDef RxHeader_ESP;
uint8_t RxData_ESP[8];

uint8_t getBtnState[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // ボタンの状態を格納する配列
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void sendMotorPower(int, int);
void print_CAN_data();
void allBtnState();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{ // printfを使えるようにする関数
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 100);
  return ch;
}
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  setbuf(stdout, NULL); // printfのバッファー

  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

  // 起動サイン
  HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, GPIO_PIN_RESET);
  printf("start\n\r"); //\n\r改行

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_SET)
    {
      // ボタンが押されている（Highレベル）
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      sendMotorPower(MOTOR_A, 2000);
      sendMotorPower(MOTOR_B, 2000);
      sendMotorPower(MOTOR_C, 2000);
      // printf("Motor ON\n\r");
    }
    else
    {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      sendMotorPower(MOTOR_A, 0);
      sendMotorPower(MOTOR_B, 0);
      sendMotorPower(MOTOR_C, 0);
      // printf("Motor OFF\n\r");
    }
    allBtnState(); // ボタンの状態を更新
    print_CAN_data();
    /*
    printf("0:%d 1:%d 2:%d 3:%d 4:%d 5:%d 6:%d 7:%d\n\r",
           data_ESP[0], data_ESP[1], data_ESP[2], data_ESP[3],
           data_ESP[4], data_ESP[5], data_ESP[6], data_ESP[7]);
    */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  CAN_FilterTypeDef CANfilter1;
  uint32_t fId11 = 0x201 << 5; // フィルターID1
  uint32_t fId12 = 0x202 << 5; // フィルターID2
  uint32_t fId13 = 0x203 << 5; // フィルターID3
  uint32_t fId14 = 0x204 << 5; // フィルターID4

  /*
  CANfilter1.FilterMode = CAN_FILTERMODE_IDMASK; // ← マスクモードに変更
  CANfilter1.FilterIdHigh = 0x0000;
  CANfilter1.FilterIdLow  = 0x0000;
  CANfilter1.FilterMaskIdHigh = 0x0000;
  CANfilter1.FilterMaskIdLow  = 0x0000;              // フィルターID4
  CANfilter1.FilterScale = CAN_FILTERSCALE_16BIT;     // 16モード
  CANfilter1.FilterFIFOAssignment = CAN_FILTER_FIFO0; // FIFO0へ格納
  CANfilter1.FilterBank = 0;
  CANfilter1.FilterMode = CAN_FILTERMODE_IDLIST; // IDリストモード
  CANfilter1.SlaveStartFilterBank = 14;
  CANfilter1.FilterActivation = ENABLE;
  */

  CANfilter1.FilterBank = 0;                      // フィルターバンク番号
  CANfilter1.FilterMode = CAN_FILTERMODE_IDMASK;  // マスクモード（IDの一部を無視できる）
  CANfilter1.FilterScale = CAN_FILTERSCALE_32BIT; // 32ビットスケーリング
  CANfilter1.FilterIdHigh = 0x0000;               // 受け入れるID → 無視（全て通す）
  CANfilter1.FilterIdLow = 0x0000;
  CANfilter1.FilterMaskIdHigh = 0x0000; // マスク全ビット0で全て受信
  CANfilter1.FilterMaskIdLow = 0x0000;
  CANfilter1.FilterFIFOAssignment = CAN_FILTER_FIFO1; // FIFO1に割り当て
  CANfilter1.FilterActivation = ENABLE;
  CANfilter1.SlaveStartFilterBank = 14; // CAN2との共用境界（必要なら）

  HAL_CAN_ConfigFilter(&hcan1, &CANfilter1);
  /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN_FilterTypeDef CANfilter2;
  uint32_t fId21 = 0x201 << 5; // フィルターID1
  uint32_t fId22 = 0x202 << 5; // フィルターID2
  uint32_t fId23 = 0x203 << 5; // フィルターID3
  uint32_t fId24 = 0x204 << 5; // フィルターID4

  CANfilter2.FilterIdHigh = fId21;     // フィルターID1
  CANfilter2.FilterIdLow = fId22;      // フィルターID2
  CANfilter2.FilterMaskIdHigh = fId23; // フィルターID3
  CANfilter2.FilterMaskIdLow = fId24;
  CANfilter2.FilterScale = CAN_FILTERSCALE_16BIT;     // 16モード
  CANfilter2.FilterFIFOAssignment = CAN_FILTER_FIFO0; // FIFO0へ格納
  CANfilter2.FilterBank = 14;
  CANfilter2.FilterMode = CAN_FILTERMODE_IDLIST; // IDリストモード
  CANfilter2.SlaveStartFilterBank = 14;
  CANfilter2.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan2, &CANfilter2);
  /* USER CODE END CAN2_Init 2 */
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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CAN_LED2_Pin | LED3_Pin | LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin | LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M3_DIR_Pin | M4_DIR_Pin | M2_DIR_Pin | M2_DIRB10_Pin | BZ_Pin | CAN_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CAN_LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = CAN_LED2_Pin | LED3_Pin | LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC1_A_Pin ENC1_B_Pin ENC3_A_Pin ENC3_B_Pin
                           ENC4_A_Pin ENC4_B_Pin SW2_Pin */
  GPIO_InitStruct.Pin = ENC1_A_Pin | ENC1_B_Pin | ENC3_A_Pin | ENC3_B_Pin | ENC4_A_Pin | ENC4_B_Pin | SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP1_Pin DIP2_Pin DIP3_Pin DIP4_Pin */
  GPIO_InitStruct.Pin = DIP1_Pin | DIP2_Pin | DIP3_Pin | DIP4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC2_A_Pin ENC2_B_Pin */
  GPIO_InitStruct.Pin = ENC2_A_Pin | ENC2_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : M3_DIR_Pin M4_DIR_Pin M2_DIR_Pin M2_DIRB10_Pin
                           BZ_Pin CAN_LED1_Pin */
  GPIO_InitStruct.Pin = M3_DIR_Pin | M4_DIR_Pin | M2_DIR_Pin | M2_DIRB10_Pin | BZ_Pin | CAN_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW5_Pin SW4_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW5_Pin | SW4_Pin | SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_PWM_Pin M2_PWM_Pin */
  GPIO_InitStruct.Pin = M1_PWM_Pin | M2_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : M3_PWM_Pin M4_PWM_Pin */
  GPIO_InitStruct.Pin = M3_PWM_Pin | M4_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint32_t id;
uint32_t dlc;
uint8_t data[8];
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
uint32_t CAN_Rx_timer[3] = {0, 0, 0};          // CAN受信タイマー
uint16_t CAN_ID_Rx[3] = {0x201, 0x202, 0x203}; // CAN受信ID
uint16_t CAN_Rx_Counter[3] = {0, 0, 0};        // CAN受信カウンター
uint8_t CAN_Rx_Motor_Read = 0;                 // CAN受信モーターデータ読み込みフラグ

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    id = (RxHeader.IDE == CAN_ID_STD) ? RxHeader.StdId : RxHeader.ExtId; // ID
    dlc = RxHeader.DLC;                                                  // DLC
    for (int i = 0; i < 8; i++)
    {
      data[i] = RxData[i]; // データを配列に格納
    }
  }
}

void print_CAN_data()
{
  for (int i = 0; i < 3; i++)
  {
    if (id == CAN_ID_Rx[i])
    {
      CAN_Rx_timer[i] = HAL_GetTick(); // タイマーリセット
      // HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET); LED2を点灯

      int16_t _angle = (data[0] << 8) | data[1];
      int16_t _speed = (data[2] << 8) | data[3];
      int16_t _torque = (data[4] << 8) | data[5];
      CAN_Rx_Counter[i]++;
      if (CAN_Rx_Motor_Read == 1)
        printf("%d, Angle:%d, speed:%d, torque:%d\n\r", i, _angle, _speed, _torque);
    }
  }
  // HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); LED2を消灯
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader_ESP, RxData_ESP) == HAL_OK)
  {
    // id_ESP = (RxHeader_ESP.IDE == CAN_ID_STD) ? RxHeader_ESP.StdId : RxHeader_ESP.ExtId;
    // dlc_ESP = RxHeader_ESP.DLC;
    for (int i = 0; i < 8; i++)
    {
      data_ESP[i] = RxData_ESP[i]; // データを配列に格納
    }

    printf("CAN1 FIFO1: ID=0x%03X, DLC=%d, Data= %d %d %d %d %d %d %d %d\n\r",
           RxHeader_ESP.StdId, RxHeader_ESP.DLC,
           data_ESP[0], data_ESP[1], data_ESP[2], data_ESP[3],
           data_ESP[4], data_ESP[5], data_ESP[6], data_ESP[7]);
  }
}

uint32_t CAN_timer = 0; // CAN受信タイマー
char btnName[14] = {'U', 'D', 'L', 'R', 'A', 'B', 'X', 'Y',
                    'L', 'R', '3', '4', '5', '6'}; // ボタンの名前

void allBtnState()
{
  for (int i = 0; i <= 7; i++)
  {
    getBtnState[i] = ((data_ESP[1] >> i) & 1); // 初期化
  }
  for (int i = 0; i <= 5; i++)
  {
    getBtnState[8+i] = ((data_ESP[2] >> i) & 1); // 初期化
  }
  if ((HAL_GetTick() - CAN_timer) > 50)
  {
    CAN_timer = HAL_GetTick(); // タイマーリセット

    for (int i = 0; i < 14; i++)
    {
      //printf("%c:%d ", btnName[i], getBtnState[i]);
    }
    //printf("\n\r");
  }
}

void sendMotorPower(int _id, int _mA)
{
  // 送信メールボックスに空きがあったら送信開始
  if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan2))
  {
    // 送信用インスタンスの設定
    TxHeader_RM.StdId = 0x200; // 受取手のCANのID
    TxHeader_RM.RTR = CAN_RTR_DATA;
    TxHeader_RM.IDE = CAN_ID_STD;
    TxHeader_RM.DLC = 8; // データ長を8byteに設定
    TxHeader_RM.TransmitGlobalTime = DISABLE;

    if (_mA > 20000)
      _mA = 20000; // 最大値を20000mAに制限
    if (_mA < -20000)
      _mA = -20000; // 最小値を-20000mAに制限
    // 送信データの設定
    _mA = (int)(((double)_mA / 20000.0) * 16384.0); // 送信値を-16384から16384の範囲に変換
    TxData_RM[_id * 2] = (_mA >> 8) & 0xFF;
    TxData_RM[_id * 2 + 1] = _mA & 0xFF;

    // printf("%d send \n\r", HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));
    if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader_RM, TxData_RM, &TxMailbox_RM) != HAL_OK)
    {
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
      Error_Handler();
    }
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
    HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, GPIO_PIN_RESET);
    HAL_Delay(200);
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
