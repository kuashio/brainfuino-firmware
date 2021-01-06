/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define STRB_IDLE  		1
#define STRB_PULSE 		0
#define T 				10
#define STATE_RUN 		0
#define STATE_PROGRAM 	1
#define OUTBOX_CAPACITY 1024

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//get a bit from a variable
#define GETBIT(var, bit)    (((var) >> (bit)) & 1)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

/* USER CODE BEGIN PV */

extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t out_pending;
uint8_t code_dump;
uint8_t state;
uint8_t freq;
uint8_t serial_input;
uint8_t board_incoming;
uint8_t adc_input;
uint8_t ADC_ON;
uint32_t code_size;
uint8_t outbox[OUTBOX_CAPACITY];
uint32_t head;
uint32_t leaving;
uint32_t tail;
uint32_t tail_temp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

void initROMNormal(void);
uint8_t readBFOutput(void);
uint8_t wait(uint32_t);
void writeBFInput(uint8_t);
uint8_t readROM(uint32_t);
uint8_t writeROM(uint32_t, uint8_t);
uint8_t eraseROM(void);
void initROMWrite(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

  if(GPIO_Pin == GPIO_PIN_3){  //OutStrobe
	  //outbox[tail] = readBFOutput();
	  //tail = (tail==(OUTBOX_CAPACITY-1))?0:(tail + 1);
	  outbox[tail] = (uint8_t)GPIOB->IDR;
	  tail = (tail+1) % OUTBOX_CAPACITY;
  }
  else
	  if (GPIO_Pin == GPIO_PIN_1){  //InStrobe
		  HAL_GPIO_WritePin(BF_INCMG_GPIO_Port, BF_INCMG_Pin, GPIO_PIN_RESET);
		  serial_input = 0;
  }
}

void set_freq(uint8_t f){
	// ICS VERSION
	//return;
	switch(f){
		case '1': // 500kHz
			  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSI, RCC_MCODIV_16);
			break;
		case '2': // 3MHz
			HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_16);
			break;
		case '3': // 6MHz
		    HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_8);
			break;
		case '4': // 8MHz
			  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
			break;
		case '5': // 12MHz
			HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_4);
			break;
		case '6': // 24MHz
			HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_2);
			break;
		case '7': // 48MHz
			HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_1);
			break;
	}

}

void CDC_Receive_Callback(uint8_t *buff, uint32_t len){
	if (len<3){
		switch(*buff){
			case '!':
				code_dump = 1;
				break;
			case '@':
				CDC_Transmit_FS(outbox,tail);
				break;
			case '1':
				freq = *buff;
				set_freq(*buff); // 500kHz
				CDC_Transmit_FS((uint8_t *)"Frequency = 500kHz", 18);
			    break;
			case '2':
				freq = *buff;
				set_freq(*buff); // 3MHz
				CDC_Transmit_FS((uint8_t *)"Frequency = 3MHz  ", 18);
				break;
			case '3':
				freq = *buff;
				set_freq(*buff); // 6MHz
				CDC_Transmit_FS((uint8_t *)"Frequency = 6MHz  ", 18);
				break;
			case '4':
				freq = *buff;
				set_freq(*buff); // 8MHz
				CDC_Transmit_FS((uint8_t *)"Frequency = 8MHz  ", 18);
				break;
			case '5':
				freq = *buff;
				set_freq(*buff); // 12MHz
				CDC_Transmit_FS((uint8_t *)"Frequency = 12MHz ", 18);
				break;
			case '6':
				freq = *buff;
				set_freq(*buff); // 24MHz
				CDC_Transmit_FS((uint8_t *)"Frequency = 24MHz ", 18);
				break;
			case '7':
				freq = *buff;
				set_freq(*buff); // 48MHz
				CDC_Transmit_FS((uint8_t *)"Frequency = 48MHz ", 18);
				break;
			default:
				writeBFInput(*buff);
				HAL_GPIO_WritePin(BF_INCMG_GPIO_Port, BF_INCMG_Pin, GPIO_PIN_SET);
				serial_input = 1;
				break;
		}
	}
	else{
		uint32_t i;
		char str[50];
		if(state == STATE_RUN){
			state = STATE_PROGRAM;
			HAL_GPIO_WritePin(BF_RST_GPIO_Port, BF_RST_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			eraseROM();
		}

		for (i = 0; i < len; i++){
			writeROM(code_size,buff[i]);
			code_size++;
		}

		sprintf(str,"Wrote %d bytes\n",(int)len);
		CDC_Transmit_FS((uint8_t *)str,strlen(str));

	}
}

uint8_t TxBusy(){
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0)
	  return 1;
  return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  uint8_t board_reset;

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
  MX_USB_DEVICE_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  serial_input = 0;
  head = 0;
  leaving = 0;
  tail = 0;

  ADC_ON = 0;

  code_size = 0;
  initROMNormal();
  wait(1000);
  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_RESET);
  wait(1000);
  HAL_GPIO_WritePin(BF_RST_GPIO_Port, BF_RST_Pin, GPIO_PIN_SET);
  state = STATE_RUN;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1){
	  // update reset by pin
	  board_reset = HAL_GPIO_ReadPin(BRD_RST_GPIO_Port, BRD_RST_Pin);
	  HAL_GPIO_WritePin(BF_RST_GPIO_Port, BF_RST_Pin, board_reset);

	  // reset in program mode
	  if (!board_reset && (state == STATE_PROGRAM)){
		  serial_input = 0;
		  head = 0;
		  leaving = 0;
		  tail = 0;
		  code_size = 0;
		  initROMNormal();
		  wait(1000);
		  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_RESET);
		  wait(1000);
		  HAL_GPIO_WritePin(BF_RST_GPIO_Port, BF_RST_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		  state = STATE_RUN;
	  }

	  // Stop BF clock when BF output buffer is full
	  if( ((tail < leaving) && ((leaving-tail) < 3)) ||
		((leaving < 4) && (tail > OUTBOX_CAPACITY-4))  ){
		  //pause_clock()vvvvvvvvvvv
		  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_NOCLOCK, RCC_MCODIV_1);
		  // ICS VERSION
	  	  //HAL_GPIO_WritePin(BF_CLK_INH_GPIO_Port,BF_CLK_INH_Pin,GPIO_PIN_SET);
	  }

	  // Send data out to host computer
	  if((head != tail) && !TxBusy()){
		  HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
		  tail_temp = tail;                  // Critical Section
	  	  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
		  leaving = head;
	  	  if(head < tail_temp){
	  		  if( (tail_temp-head) < 60 ){ // easy case
				  CDC_Transmit_FS(outbox+head, tail_temp-head);
				  head = tail_temp;
	  		  }
	  		  else{ 				// chunk it
				  CDC_Transmit_FS(outbox+head, 60);
				  head += 60;
	  		  }
	  	  }
	  	  else{
	  		  if(head >= (OUTBOX_CAPACITY-60)){ // easy case
				  CDC_Transmit_FS(outbox+head, OUTBOX_CAPACITY-head);
				  head = 0;
	  		  }
	  		  else{ 				// chunk it
				  CDC_Transmit_FS(outbox+head, 60);
				  head += 60;
	  		  }
	  	  }
		  //resume_clock()vvvvvvvv
	  	  set_freq(freq);
	  	  //ICS VERSION
	  	  //HAL_GPIO_WritePin(BF_CLK_INH_GPIO_Port,BF_CLK_INH_Pin,GPIO_PIN_RESET);
	  }

	  // update hardware input (digital or analog)
	  __disable_irq();
	  if(!serial_input){
		  __enable_irq();
		  board_incoming = HAL_GPIO_ReadPin(BRD_INCMG_GPIO_Port, BRD_INCMG_Pin);
		  if(board_incoming){
			  __disable_irq();
			  writeBFInput((uint8_t)GPIOD->IDR);
			  HAL_GPIO_WritePin(BF_INCMG_GPIO_Port, BF_INCMG_Pin, GPIO_PIN_SET);
			  __enable_irq();
		  }
		  else{
			  adc_input = HAL_GPIO_ReadPin(AIEN_GPIO_Port, AIEN_Pin);
			  if(adc_input){
				  if(!ADC_ON){
					  HAL_ADC_Start(&hadc);
					  HAL_ADC_PollForConversion(&hadc, 1);
					  ADC_ON = 1;
				  }
				  __disable_irq();
				  writeBFInput((uint8_t)HAL_ADC_GetValue(&hadc));
				  HAL_GPIO_WritePin(BF_INCMG_GPIO_Port, BF_INCMG_Pin, GPIO_PIN_SET);
				  __enable_irq();
			  }
			  else{
				  HAL_GPIO_WritePin(BF_INCMG_GPIO_Port, BF_INCMG_Pin, GPIO_PIN_RESET);
			  }
		  }
	  }
	  __enable_irq();

	  // dump the code
	  if(code_dump){
		  uint32_t i,j;
		  const uint32_t N = 5000;
		  uint8_t code_str[N];

		  HAL_GPIO_WritePin(BF_RST_GPIO_Port, BF_RST_Pin, GPIO_PIN_RESET);
		  HAL_Delay(10);

		  for(i = 0; i < N; i++){
			  code_str[i] = readROM(i);
			  if (code_str[i] == 0xFF) break;
		  }

		  for(j = 0; (i-j) >= 60; j+=60){
			  while(TxBusy()){};
			  CDC_Transmit_FS(code_str+j, 60);
		  }
		  if(j<i){
			  while(TxBusy()){};
			  CDC_Transmit_FS(code_str+j, i-j);
		  }

		  if(state==STATE_RUN){
			  initROMNormal();
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_RESET);
			  HAL_Delay(10);
			  if(HAL_GPIO_ReadPin(BRD_RST_GPIO_Port, BRD_RST_Pin))  // if RST button pressed, don't un-reset
				  HAL_GPIO_WritePin(BF_RST_GPIO_Port, BF_RST_Pin, GPIO_PIN_SET);
			  HAL_Delay(1);
		  }

		  code_dump = 0;
	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSI, RCC_MCODIV_16);
  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();
  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_8B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BRD_INSTRB_Pin|OE_Pin|WE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BF_IN0_Pin|BF_IN1_Pin|BF_IN2_Pin|BF_IN3_Pin
                          |BF_IN4_Pin|BF_IN5_Pin|BF_IN6_Pin|BF_IN7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BF_INCMG_Pin|BF_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : A2_Pin A3_Pin A4_Pin A5_Pin
                           A6_Pin A7_Pin A8_Pin A9_Pin
                           A10_Pin A11_Pin A12_Pin A13_Pin
                           A14_Pin A15_Pin A0_Pin A1_Pin */
  GPIO_InitStruct.Pin = A2_Pin|A3_Pin|A4_Pin|A5_Pin
                          |A6_Pin|A7_Pin|A8_Pin|A9_Pin
                          |A10_Pin|A11_Pin|A12_Pin|A13_Pin
                          |A14_Pin|A15_Pin|A0_Pin|A1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BF_INSTRB_Pin */
  GPIO_InitStruct.Pin = BF_INSTRB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BF_INSTRB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BRD_INSTRB_Pin */
  GPIO_InitStruct.Pin = BRD_INSTRB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BRD_INSTRB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BF_OUTSTRB_Pin */
  GPIO_InitStruct.Pin = BF_OUTSTRB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BF_OUTSTRB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BF_IN0_Pin BF_IN1_Pin BF_IN2_Pin BF_IN3_Pin
                           BF_IN4_Pin BF_IN5_Pin BF_IN6_Pin BF_IN7_Pin */
  GPIO_InitStruct.Pin = BF_IN0_Pin|BF_IN1_Pin|BF_IN2_Pin|BF_IN3_Pin
                          |BF_IN4_Pin|BF_IN5_Pin|BF_IN6_Pin|BF_IN7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BRD_RST_Pin */
  GPIO_InitStruct.Pin = BRD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BRD_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AIEN_Pin BRD_INCMG_Pin */
  GPIO_InitStruct.Pin = AIEN_Pin|BRD_INCMG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BF_OUT0_Pin BF_OUT1_Pin BF_OUT2_Pin D2_Pin
                           D3_Pin D4_Pin D5_Pin D6_Pin
                           D7_Pin BF_OUT3_Pin BF_OUT4_Pin BF_OUT5_Pin
                           BF_OUT6_Pin BF_OUT7_Pin D0_Pin D1_Pin */
  GPIO_InitStruct.Pin = BF_OUT0_Pin|BF_OUT1_Pin|BF_OUT2_Pin|D2_Pin
                          |D3_Pin|D4_Pin|D5_Pin|D6_Pin
                          |D7_Pin|BF_OUT3_Pin|BF_OUT4_Pin|BF_OUT5_Pin
                          |BF_OUT6_Pin|BF_OUT7_Pin|D0_Pin|D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BF_INCMG_Pin BF_RST_Pin */
  GPIO_InitStruct.Pin = BF_INCMG_Pin|BF_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : A16_Pin A17_Pin */
  GPIO_InitStruct.Pin = A16_Pin|A17_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BF_CLK_Pin */
  GPIO_InitStruct.Pin = BF_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(BF_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OE_Pin WE_Pin */
  GPIO_InitStruct.Pin = OE_Pin|WE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN0_Pin IN1_Pin IN2_Pin IN3_Pin
                           IN4_Pin IN5_Pin IN6_Pin IN7_Pin */
  GPIO_InitStruct.Pin = IN0_Pin|IN1_Pin|IN2_Pin|IN3_Pin
                          |IN4_Pin|IN5_Pin|IN6_Pin|IN7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

}

/* USER CODE BEGIN 4 */

void initROMNormal(){
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, GPIO_PIN_SET);

	  /*Configure GPIO pins : D2_Pin D3_Pin D4_Pin D5_Pin
	                           D6_Pin D7_Pin D0_Pin D1_Pin */
	  GPIO_InitStruct.Pin = D2_Pin|D3_Pin|D4_Pin|D5_Pin
	                          |D6_Pin|D7_Pin|D0_Pin|D1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pins : A16_Pin A17_Pin */
	  GPIO_InitStruct.Pin = A16_Pin|A17_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : A2_Pin A3_Pin A4_Pin A5_Pin
	                           A6_Pin A7_Pin A8_Pin A9_Pin
	                           A10_Pin A11_Pin A12_Pin A13_Pin
	                           A14_Pin A15_Pin A0_Pin A1_Pin */
	  GPIO_InitStruct.Pin = A2_Pin|A3_Pin|A4_Pin|A5_Pin
	                          |A6_Pin|A7_Pin|A8_Pin|A9_Pin
	                          |A10_Pin|A11_Pin|A12_Pin|A13_Pin
	                          |A14_Pin|A15_Pin|A0_Pin|A1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void initROMRead(){
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, GPIO_PIN_SET);

	  /*Configure GPIO pins : D2_Pin D3_Pin D4_Pin D5_Pin
	                           D6_Pin D7_Pin D0_Pin D1_Pin */
	  GPIO_InitStruct.Pin = D2_Pin|D3_Pin|D4_Pin|D5_Pin
	                          |D6_Pin|D7_Pin|D0_Pin|D1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pins : A16_Pin A17_Pin */
	  GPIO_InitStruct.Pin = A16_Pin|A17_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : A2_Pin A3_Pin A4_Pin A5_Pin
	                           A6_Pin A7_Pin A8_Pin A9_Pin
	                           A10_Pin A11_Pin A12_Pin A13_Pin
	                           A14_Pin A15_Pin A0_Pin A1_Pin */
	  GPIO_InitStruct.Pin = A2_Pin|A3_Pin|A4_Pin|A5_Pin
	                          |A6_Pin|A7_Pin|A8_Pin|A9_Pin
	                          |A10_Pin|A11_Pin|A12_Pin|A13_Pin
	                          |A14_Pin|A15_Pin|A0_Pin|A1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void initROMWrite(){
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, GPIO_PIN_SET);

	  /*Configure GPIO pins : A16_Pin A17_Pin */
	  GPIO_InitStruct.Pin = A16_Pin|A17_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : A2_Pin A3_Pin A4_Pin A5_Pin
	                           A6_Pin A7_Pin A8_Pin A9_Pin
	                           A10_Pin A11_Pin A12_Pin A13_Pin
	                           A14_Pin A15_Pin A0_Pin A1_Pin */
	  GPIO_InitStruct.Pin = A2_Pin|A3_Pin|A4_Pin|A5_Pin
	                          |A6_Pin|A7_Pin|A8_Pin|A9_Pin
	                          |A10_Pin|A11_Pin|A12_Pin|A13_Pin
	                          |A14_Pin|A15_Pin|A0_Pin|A1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


	  /*Configure GPIO pins : D2_Pin D3_Pin D4_Pin D5_Pin
	                           D6_Pin D7_Pin D0_Pin D1_Pin */
	  GPIO_InitStruct.Pin = D2_Pin|D3_Pin|D4_Pin|D5_Pin
	                          |D6_Pin|D7_Pin|D0_Pin|D1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void setAddrBus(uint32_t addr){
	HAL_GPIO_WritePin(A17_GPIO_Port, A17_Pin, GETBIT(addr,17));
	HAL_GPIO_WritePin(A16_GPIO_Port, A16_Pin, GETBIT(addr,16));
	HAL_GPIO_WritePin(A15_GPIO_Port, A15_Pin, GETBIT(addr,15));
	HAL_GPIO_WritePin(A14_GPIO_Port, A14_Pin, GETBIT(addr,14));
	HAL_GPIO_WritePin(A13_GPIO_Port, A13_Pin, GETBIT(addr,13));
	HAL_GPIO_WritePin(A12_GPIO_Port, A12_Pin, GETBIT(addr,12));
	HAL_GPIO_WritePin(A11_GPIO_Port, A11_Pin, GETBIT(addr,11));
	HAL_GPIO_WritePin(A10_GPIO_Port, A10_Pin, GETBIT(addr,10));
	HAL_GPIO_WritePin( A9_GPIO_Port,  A9_Pin, GETBIT(addr, 9));
	HAL_GPIO_WritePin( A8_GPIO_Port,  A8_Pin, GETBIT(addr, 8));
	HAL_GPIO_WritePin( A7_GPIO_Port,  A7_Pin, GETBIT(addr, 7));
	HAL_GPIO_WritePin( A6_GPIO_Port,  A6_Pin, GETBIT(addr, 6));
	HAL_GPIO_WritePin( A5_GPIO_Port,  A5_Pin, GETBIT(addr, 5));
	HAL_GPIO_WritePin( A4_GPIO_Port,  A4_Pin, GETBIT(addr, 4));
	HAL_GPIO_WritePin( A3_GPIO_Port,  A3_Pin, GETBIT(addr, 3));
	HAL_GPIO_WritePin( A2_GPIO_Port,  A2_Pin, GETBIT(addr, 2));
	HAL_GPIO_WritePin( A1_GPIO_Port,  A1_Pin, GETBIT(addr, 1));
	HAL_GPIO_WritePin( A0_GPIO_Port,  A0_Pin, GETBIT(addr, 0));
}

void setDataBus(uint8_t data){
	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, GETBIT(data,7));
	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, GETBIT(data,6));
	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GETBIT(data,5));
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GETBIT(data,4));
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GETBIT(data,3));
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GETBIT(data,2));
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GETBIT(data,1));
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, GETBIT(data,0));
}

uint8_t readDataBus(){
	uint8_t temp = 0;
	temp |= (HAL_GPIO_ReadPin(D7_GPIO_Port, D7_Pin) << 7);
	temp |= (HAL_GPIO_ReadPin(D6_GPIO_Port, D6_Pin) << 6);
	temp |= (HAL_GPIO_ReadPin(D5_GPIO_Port, D5_Pin) << 5);
	temp |= (HAL_GPIO_ReadPin(D4_GPIO_Port, D4_Pin) << 4);
	temp |= (HAL_GPIO_ReadPin(D3_GPIO_Port, D3_Pin) << 3);
	temp |= (HAL_GPIO_ReadPin(D2_GPIO_Port, D2_Pin) << 2);
	temp |= (HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin) << 1);
	temp |= (HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin) << 0);
	return temp;
}

uint8_t wait(uint32_t x){
	volatile uint32_t y;
	y=x;
	while(y--);
	return((uint8_t)y);
}

uint8_t LLReadROM(uint32_t addr){
	volatile uint8_t temp;
	setAddrBus(addr);
	temp = wait(T);
	HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_RESET);
	temp = wait(T);
	temp = readDataBus();
	HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_SET);
	return(temp);
}

uint8_t readROM(uint32_t addr){
	initROMRead();
	return(LLReadROM(addr));
}

void LLWriteROM(uint32_t addr, uint8_t data){
	volatile uint8_t temp;
	setAddrBus(addr);
	setDataBus(data);
	HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, GPIO_PIN_RESET);
	temp = wait(T);
	HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, GPIO_PIN_SET);
	wait(temp);
}

uint8_t ToggleBitWait(uint32_t addr){
	uint8_t first, second;
	//initROMRead();
	first  = LLReadROM(addr) & 1<<6;
	second = LLReadROM(addr) & 1<<6;
	return (first != second);
}

uint8_t writeROM(uint32_t addr, uint8_t data){
	initROMWrite();
	// Step 1
	LLWriteROM(0x5555, 0xAA);
	LLWriteROM(0x2AAA, 0x55);
	LLWriteROM(0x5555, 0xA0);

	// Step 2
	LLWriteROM(addr, data);

	// Step 3

	initROMRead();
	do{
		do{} while (ToggleBitWait(addr));
	} while (ToggleBitWait(addr));

	return 0;
}

uint8_t eraseROM(){
	uint32_t addr = 10;
	initROMWrite();
	// Step 1
	LLWriteROM(0x5555, 0xAA);
	LLWriteROM(0x2AAA, 0x55);
	LLWriteROM(0x5555, 0x80);
	LLWriteROM(0x5555, 0xAA);
	LLWriteROM(0x2AAA, 0x55);
	LLWriteROM(0x5555, 0x10);

	// Step 2

	initROMRead();
	do{
		do{} while (ToggleBitWait(addr));
	} while (ToggleBitWait(addr));

	return 0;
}

void writeBFInput(uint8_t data){
	HAL_GPIO_WritePin(BF_IN7_GPIO_Port, BF_IN7_Pin, GETBIT(data,7));
	HAL_GPIO_WritePin(BF_IN6_GPIO_Port, BF_IN6_Pin, GETBIT(data,6));
	HAL_GPIO_WritePin(BF_IN5_GPIO_Port, BF_IN5_Pin, GETBIT(data,5));
	HAL_GPIO_WritePin(BF_IN4_GPIO_Port, BF_IN4_Pin, GETBIT(data,4));
	HAL_GPIO_WritePin(BF_IN3_GPIO_Port, BF_IN3_Pin, GETBIT(data,3));
	HAL_GPIO_WritePin(BF_IN2_GPIO_Port, BF_IN2_Pin, GETBIT(data,2));
	HAL_GPIO_WritePin(BF_IN1_GPIO_Port, BF_IN1_Pin, GETBIT(data,1));
	HAL_GPIO_WritePin(BF_IN0_GPIO_Port, BF_IN0_Pin, GETBIT(data,0));
}

uint8_t readBFOutput(){/*
	uint8_t temp = 0;
	temp |= (HAL_GPIO_ReadPin(BF_OUT7_GPIO_Port, BF_OUT7_Pin) << 7);
	temp |= (HAL_GPIO_ReadPin(BF_OUT6_GPIO_Port, BF_OUT6_Pin) << 6);
	temp |= (HAL_GPIO_ReadPin(BF_OUT5_GPIO_Port, BF_OUT5_Pin) << 5);
	temp |= (HAL_GPIO_ReadPin(BF_OUT4_GPIO_Port, BF_OUT4_Pin) << 4);
	temp |= (HAL_GPIO_ReadPin(BF_OUT3_GPIO_Port, BF_OUT3_Pin) << 3);
	temp |= (HAL_GPIO_ReadPin(BF_OUT2_GPIO_Port, BF_OUT2_Pin) << 2);
	temp |= (HAL_GPIO_ReadPin(BF_OUT1_GPIO_Port, BF_OUT1_Pin) << 1);
	temp |= (HAL_GPIO_ReadPin(BF_OUT0_GPIO_Port, BF_OUT0_Pin) << 0);
	return temp;*/
	return (uint8_t)GPIOB->IDR;
}



void OLDECDC_Receive_Callback(uint8_t *buff, uint32_t len){
	static uint8_t prev, temp;
	switch(*buff){
		case 'g':
				CDC_Transmit_FS((uint8_t *)"Gato", 4U);
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				break;
		case 'c':
				//CDC_Transmit_FS((uint8_t *)"\nErasing ROM", 12U);
				if(!eraseROM())
					CDC_Transmit_FS((uint8_t *)" ... Done!\n", 10U);
				else
					CDC_Transmit_FS((uint8_t *)" ... Nope!\n", 10U);
				break;
		case 'w':
				//CDC_Transmit_FS((uint8_t *)"\nWriting ", 9U);
				//CDC_Transmit_FS(&prev, 1U);
				if(!writeROM(10U,prev))
					CDC_Transmit_FS((uint8_t *)" ... Done!\n", 10U);
				else
					CDC_Transmit_FS((uint8_t *)" ... Nope!\n", 10U);
				break;
		case 'r':
				//CDC_Transmit_FS((uint8_t *)"\nReading ... ", 13U);
				temp = readROM(10U);
				CDC_Transmit_FS(&temp, 1U);
				//CDC_Transmit_FS((uint8_t *)"\n", 1U);
				break;
		case '0':
				temp = readROM(0U);
				CDC_Transmit_FS(&temp, 1U);
				break;
		case '1':
				temp = readROM(1U);
				CDC_Transmit_FS(&temp, 1U);
				break;
		case '2':
				temp = readROM(2U);
				CDC_Transmit_FS(&temp, 1U);
				break;
		case '3':
				temp = readROM(3U);
				CDC_Transmit_FS(&temp, 1U);
				break;
		case '4':
				temp = readROM(4U);
				CDC_Transmit_FS(&temp, 1U);
				break;
		case '5':
				temp = readROM(5U);
				CDC_Transmit_FS(&temp, 1U);
				break;
		case '@':
				writeROM(0U,',');
				writeROM(1U,'[');
				writeROM(2U,'+');
				writeROM(3U,'.');
				writeROM(4U,',');
				if(!writeROM(5U,']'))
					CDC_Transmit_FS((uint8_t *)" ... Done!\n", 10U);
				break;

		default:
				CDC_Transmit_FS(buff, len);
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				break;
	}

	prev=*buff;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
