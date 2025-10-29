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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    START_LED,
    STOP_LED,
	//ADC_Sampler
	SET_PERIOD,
	GET_PERIOD,
	GET_ADC,
	//Alert_Monitor
	SET_ALERT,
	//Data_Analyzer
	START_ANALYSIS,
	STOP_ANALYSIS,
	SET_ANALYSIS_AVG,
	SET_ANALYSIS_MAX,
	SET_ANALYSIS_MIN,
	GET_ANALYSIS,
	SET_SAMPLES,
	//
	UNKNOWN_COMMAND// wartość domyślna, gdy komenda nie jest rozpoznana
} Command;

typedef enum{
	AVG,
	MAX,
	MIN
} AnalyzerMode_t;

typedef enum{
	ON,
	OFF
} Analyzer_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
osThreadState_t UART_CommandsState;
osThreadState_t LED_BlinkingState;
osThreadState_t ADC_SamplerState;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef hlpuart1;

/* Definitions for UART_Commands */
osThreadId_t UART_CommandsHandle;
const osThreadAttr_t UART_Commands_attributes = {
  .name = "UART_Commands",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for LED_Blinking */
osThreadId_t LED_BlinkingHandle;
const osThreadAttr_t LED_Blinking_attributes = {
  .name = "LED_Blinking",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for ADC_Sampler */
osThreadId_t ADC_SamplerHandle;
const osThreadAttr_t ADC_Sampler_attributes = {
  .name = "ADC_Sampler",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for AlertMonitor */
osThreadId_t AlertMonitorHandle;
const osThreadAttr_t AlertMonitor_attributes = {
  .name = "AlertMonitor",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Data_Analyzer1 */
osThreadId_t Data_Analyzer1Handle;
const osThreadAttr_t Data_Analyzer1_attributes = {
  .name = "Data_Analyzer1",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for uartRxQueue */
osMessageQueueId_t uartRxQueueHandle;
const osMessageQueueAttr_t uartRxQueue_attributes = {
  .name = "uartRxQueue"
};
/* Definitions for adcAlertQueue */
osMessageQueueId_t adcAlertQueueHandle;
const osMessageQueueAttr_t adcAlertQueue_attributes = {
  .name = "adcAlertQueue"
};
/* Definitions for adcAnalyzerQueue */
osMessageQueueId_t adcAnalyzerQueueHandle;
const osMessageQueueAttr_t adcAnalyzerQueue_attributes = {
  .name = "adcAnalyzerQueue"
};
/* Definitions for UART_EventFlag */
osEventFlagsId_t UART_EventFlagHandle;
const osEventFlagsAttr_t UART_EventFlag_attributes = {
  .name = "UART_EventFlag"
};
/* USER CODE BEGIN PV */

// UART
volatile uint8_t rxByte;
volatile uint8_t cmd_buffer[20];
// ADC
volatile uint16_t adc_value=200;
volatile uint16_t sample_period = 1000;

// Analyzer
volatile AnalyzerMode_t analyzer_mode = MIN;
volatile uint16_t analyzer_samples = 10;
volatile Analyzer_State_t analyzer_state = OFF;
// Alert
volatile uint16_t alert_threshold = 1000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_ADC1_Init(void);
void StartUART_Commands(void *argument);
void StartLED_Blinking(void *argument);
void StartADC_Sampler(void *argument);
void StartAlertMonitor(void *argument);
void StartData_Analyzer(void *argument);

/* USER CODE BEGIN PFP */
char* ParseCommand(char *inputBuffer);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
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
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uartRxQueue */
  uartRxQueueHandle = osMessageQueueNew (32, sizeof(uint8_t), &uartRxQueue_attributes);

  /* creation of adcAlertQueue */
  adcAlertQueueHandle = osMessageQueueNew (32, sizeof(uint16_t), &adcAlertQueue_attributes);

  /* creation of adcAnalyzerQueue */
  adcAnalyzerQueueHandle = osMessageQueueNew (32, sizeof(uint16_t), &adcAnalyzerQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UART_Commands */
  UART_CommandsHandle = osThreadNew(StartUART_Commands, NULL, &UART_Commands_attributes);

  /* creation of LED_Blinking */
  LED_BlinkingHandle = osThreadNew(StartLED_Blinking, NULL, &LED_Blinking_attributes);

  /* creation of ADC_Sampler */
  ADC_SamplerHandle = osThreadNew(StartADC_Sampler, NULL, &ADC_Sampler_attributes);

  /* creation of AlertMonitor */
  AlertMonitorHandle = osThreadNew(StartAlertMonitor, NULL, &AlertMonitor_attributes);

  /* creation of Data_Analyzer1 */
  Data_Analyzer1Handle = osThreadNew(StartData_Analyzer, NULL, &Data_Analyzer1_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of UART_EventFlag */
  UART_EventFlagHandle = osEventFlagsNew(&UART_EventFlag_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
    	osMessageQueuePut(uartRxQueueHandle, &rxByte, NULL, 0);
        /* Ponowne uruchomienie odbioru kolejnego bajtu */
        HAL_UART_Receive_IT(&hlpuart1, &rxByte, 1);
    }
}

// Funkcja parsująca komendę do odpowiedniej wartości typu Command
char* ParseCommand(char *inputBuffer){

	static char output_msg[64];
	Command cmd;

	//LED_Blinking
     if (strcmp(inputBuffer,"START_LED\r\n") == 0){
        cmd = START_LED;
    }else if (strcmp(inputBuffer,"STOP_LED\r\n") == 0){
        cmd = STOP_LED;
    }

    //ADC_Sampler
     else if (strncmp(inputBuffer,"SET_PERIOD",10) == 0){
        cmd = SET_PERIOD;
    }else if (strcmp(inputBuffer,"GET_PERIOD\r\n") == 0){
        cmd = GET_PERIOD;
    }else if (strcmp(inputBuffer,"GET_ADC\r\n") == 0){
        cmd = GET_ADC;
    }

    //Alert_Monitor
     else if (strncmp(inputBuffer,"SET_ALERT",9) == 0){
        cmd = SET_ALERT;
    }

    //Data_Analyzer
     else if (strcmp(inputBuffer,"START_ANALYSIS\r\n") == 0){
        cmd = START_ANALYSIS;
    }else if (strcmp(inputBuffer,"STOP_ANALYSIS\r\n") == 0){
        cmd = STOP_ANALYSIS;
    }else if (strcmp(inputBuffer,"SET_ANALYSIS_AVG\r\n") == 0){
        cmd = SET_ANALYSIS_AVG;
    }else if (strcmp(inputBuffer,"SET_ANALYSIS_MAX\r\n") == 0){
        cmd = SET_ANALYSIS_MAX;
    }else if (strcmp(inputBuffer,"SET_ANALYSIS_MIN\r\n") == 0){
        cmd = SET_ANALYSIS_MIN;
    }else if (strcmp(inputBuffer,"GET_ANALYSIS\r\n") == 0){
        cmd = GET_ANALYSIS;
    }else if (strncmp(inputBuffer,"SET_SAMPLES",11) == 0){
        cmd = SET_SAMPLES;
    }else{
        cmd = UNKNOWN_COMMAND;
    }

	 // Obsługa komendy za pomocą switch-case
	 switch (cmd) {

	 	//LED_Blinking
		case START_LED:{
			/* Ustawiamy bit w event group, aby uruchomić miganie */
			osEventFlagsSet(UART_EventFlagHandle, EVENT_BIT_BLINK);
			snprintf(output_msg, sizeof(output_msg), "LED blinking started\r\n");
			break;
		}
		case STOP_LED:{
			/* Czyszczenie flagi – zatrzymanie migania LED */
			osEventFlagsClear(UART_EventFlagHandle, EVENT_BIT_BLINK);
			snprintf(output_msg, sizeof(output_msg), "LED blinking stopped\r\n");
			break;
		}

		//ADC_Sampler
		case SET_PERIOD:{
			sscanf(inputBuffer, "SET_PERIOD %hu", &sample_period);
			snprintf(output_msg, sizeof(output_msg), "PERIOD SET\r\n");
			break;
		}
		case GET_PERIOD:{
			snprintf(output_msg, sizeof(output_msg), "PERIOD: %u\r\n", sample_period);
			break;
		}
		case GET_ADC:{
			snprintf(output_msg, sizeof(output_msg), "ADC: %u\r\n", adc_value);
			break;
		}

		//Alert_Monitor
		case SET_ALERT:{
			sscanf(inputBuffer, "SET_ALERT %hu", &alert_threshold);
			snprintf(output_msg, sizeof(output_msg), "Alert SET\r\n");
			break;
		}

		//Data_Analyzer
		case START_ANALYSIS :{
			analyzer_state = ON;
			snprintf(output_msg, sizeof(output_msg), "Analysis started\r\n");
			break;
		}
		case STOP_ANALYSIS :{
			analyzer_state = OFF;
			snprintf(output_msg, sizeof(output_msg), "Analysis stopped\r\n");
			break;
		}
		case SET_ANALYSIS_AVG :{
			analyzer_mode = AVG;
			snprintf(output_msg, sizeof(output_msg), "MODE: AVG\r\n");
			break;
		}
		case SET_ANALYSIS_MAX:{
			analyzer_mode = MAX;
			snprintf(output_msg, sizeof(output_msg), "MODE: MAX\r\n");
			break;
		}
		case SET_ANALYSIS_MIN :{
			analyzer_mode = MIN;
			snprintf(output_msg, sizeof(output_msg), "MODE: MIN\r\n");
			break;
		}
		case GET_ANALYSIS:{
			const char* mode_str = "UNKNOWN";
	    	switch(analyzer_mode){
	    		case AVG: mode_str = "AVG"; break;
	    		case MAX: mode_str = "MAX"; break;
	    		case MIN: mode_str = "MIN"; break;
	    	}
    	snprintf(output_msg, sizeof(output_msg), "Analysis mode: %s\r\n",mode_str);
			break;
		}
		case SET_SAMPLES:{
			uint16_t buffor_samples;
			sscanf(inputBuffer, "SET_SAMPLES %hu", &buffor_samples);

			if(buffor_samples <=200)
			{
				analyzer_samples = buffor_samples;
				snprintf(output_msg, sizeof(output_msg), "Samples set\r\n");
			}
			else
				snprintf(output_msg, sizeof(output_msg), "Wrong value, buffor size = 200\r\n");
			break;
		}
		case UNKNOWN_COMMAND:{
			char output_msg[] = "Unknown command\r\n";
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)output_msg, strlen(output_msg), 100);
			break;
		}
		default:{
			char output_msg[] = "Error\r\n";
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)output_msg, strlen(output_msg), 100);
			break;
		}
	}
	 return output_msg;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUART_Commands */
/**
  * @brief  Function implementing the UART_Commands thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUART_Commands */
void StartUART_Commands(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	char inputBuffer[RX_BUFFER_SIZE];

	uint8_t inputIdx = 0;
	char receivedChar;

	HAL_UART_Receive_IT(&hlpuart1, (uint8_t*)&rxByte, 1);

	for (;;)
	{
		if (osMessageQueueGet(uartRxQueueHandle, &receivedChar, NULL, 1) == osOK)
		{
			if (inputIdx < RX_BUFFER_SIZE)
			{
				/* Dodajemy odebrany znak do bufora */
				inputBuffer[inputIdx++] = receivedChar;

				/* Sprawdź, czy odebrany znak to '\n' – zakończenie komendy */
				if (receivedChar == '\n')
				{
					inputBuffer[inputIdx] = '\0'; // zakończenie łańcucha

					/* Interpretacja komendy: */
					char* msg = ParseCommand(inputBuffer);
					HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 100);

					/* Zerujemy bufor i przygotowujemy się na nową komendę */
					inputIdx = 0;
				}
			}
			else
			{
				/* Bufor przepełniony, a znak '\n' nie został jeszcze odebrany */
				char errorMsg[] = "Syntax Error\r\n";
				HAL_UART_Transmit(&hlpuart1, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
				inputBuffer[inputIdx] = '\0';
				HAL_UART_Transmit(&hlpuart1, (uint8_t*)inputBuffer, strlen(inputBuffer), HAL_MAX_DELAY);
				inputIdx = 0;
			}
		}
		osDelay(5);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLED_Blinking */
/**
* @brief Function implementing the LED_Blinking thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED_Blinking */
void StartLED_Blinking(void *argument)
{
  /* USER CODE BEGIN StartLED_Blinking */
  /* Infinite loop */
  for(;;)
  {
	osEventFlagsWait(UART_EventFlagHandle, EVENT_BIT_BLINK, osFlagsNoClear, osWaitForever);
	for(int i = 0; i<10; i++)
	  {
	  	HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
	  	HAL_Delay(50);
	  }
    osDelay(1000);
  }
  /* USER CODE END StartLED_Blinking */
}

/* USER CODE BEGIN Header_StartADC_Sampler */
/**
* @brief Function implementing the ADC_Sampler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADC_Sampler */
void StartADC_Sampler(void *argument)
{
  /* USER CODE BEGIN StartADC_Sampler */
  /* Infinite loop */
  for(;;)
  {
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK){
		adc_value = HAL_ADC_GetValue(&hadc1);
		osMessageQueuePut(adcAnalyzerQueueHandle, &adc_value, 0, 0);
		osMessageQueuePut(adcAlertQueueHandle, &adc_value, 0, 0);
	}
	osDelay(sample_period);
  }
  /* USER CODE END StartADC_Sampler */
}

/* USER CODE BEGIN Header_StartAlertMonitor */
/**
* @brief Function implementing the AlertMonitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAlertMonitor */
void StartAlertMonitor(void *argument)
{
  /* USER CODE BEGIN StartAlertMonitor */
  uint8_t active_alert = 0;
  uint16_t adc_value = 0;

  char msg[64];
  /* Infinite loop */
  for(;;)
  {
	  if (osMessageQueueGet(adcAlertQueueHandle, &adc_value, NULL, 1) == osOK)
	  {
		if(adc_value > alert_threshold){
			if(!active_alert){
				snprintf(msg, sizeof(msg),"ALERT: Value %u above limit \r\n",adc_value);
				HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 100);
				active_alert = 1;
			}
		}else{
			active_alert = 0;
		}
	  }
	  osDelay(1);
  }

  /* USER CODE END StartAlertMonitor */
}

/* USER CODE BEGIN Header_StartData_Analyzer */
/**
* @brief Function implementing the Data_Analyzer1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartData_Analyzer */
void StartData_Analyzer(void *argument)
{
  /* USER CODE BEGIN StartData_Analyzer */
  uint16_t ADC_buffor[200];
  uint16_t sample_id = 0;
  uint16_t i = 0;
  uint16_t result;

  uint16_t adc_value;
  uint16_t sample_max;
  uint16_t sample_min;
  char msg[64];
  Command receivedCommand;
  /* Infinite loop */
  for(;;)
  {
	  if (osMessageQueueGet(adcAnalyzerQueueHandle, &adc_value, NULL, 1) == osOK)
	  {
		  ADC_buffor[sample_id] = adc_value;
		  if(adc_value>sample_max)
			  sample_max = adc_value;
		  if(adc_value<sample_min)
			  sample_min = adc_value;

		  if(sample_id >= analyzer_samples )
		  {
			  if(analyzer_state!=OFF)
			  {
				  result = 0;
				  switch(analyzer_mode){
					  case AVG:{
						  for(i=0;i<analyzer_samples;i++)
						  {
							  result+= ADC_buffor[i];
						  }
						  result/= analyzer_samples;

						  snprintf(msg, sizeof(msg),"AVG: %u \r\n",result);
						  HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 100);
						  break;
					  }
					  case MAX:{
						  snprintf(msg, sizeof(msg),"MAX: %u \r\n",sample_max);
						  HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 100);
						  break;
					  }
					  case MIN:{
						  snprintf(msg, sizeof(msg),"MIN: %u \r\n",sample_min);
						  HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 100);
						  break;
					  }
				  }
			  }
			  sample_max = 0;
			  sample_min = 0xFFFF;
			  sample_id = 0;
		  }
		  sample_id++;
	  }
    osDelay(1);
  }
  /* USER CODE END StartData_Analyzer */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
