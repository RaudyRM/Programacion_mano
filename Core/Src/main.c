/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

#define RED_LED GPIO_PIN_0
#define BLUE_LED GPIO_PIN_1
#define BUTTON_IN GPIO_PIN_4
#define HIGH 1
#define LOW 0

//Controlan las pulsaciones
#define HUMBRAL_TIME 28//Lo ideal es 35, Tiempo en HIGH
//#define TIME_LOOP 70 //Lo ideal es 35, Periodo

//Espacio en memoria de la maquina de estado
#define ESTADO_INICIO 0
#define ESTADO_ABIERTO 1
#define ESTADO_SEMICERRADO 2
#define ESTADO_CERRADO 3

#define FALSE 0
#define TRUE 1

#define LOW 0
#define HIGH 1

uint16_t FUN_ESTADO_INICIO (void);
uint16_t FUN_ESTADO_ABIERTO (void);
uint16_t FUN_ESTADO_SEMICERRADO (void);
uint16_t FUN_ESTADO_CERRADO (void);

uint16_t ESTADO_SIGUIENTE=ESTADO_INICIO;
uint16_t ESTADO_ACTUAL=ESTADO_INICIO;
uint16_t ESTADO_ANTERIOR=ESTADO_INICIO;
uint16_t INICIO_STATE=TRUE;
uint16_t SENAL_STATE=0;

void SERVOS(uint8_t servo1, uint8_t servo2);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  // Start timer
  /*HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);*/
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  ESTADO_SIGUIENTE = FUN_ESTADO_INICIO();

	  for(;;)
	  {
	    if(ESTADO_SIGUIENTE==ESTADO_ABIERTO)
	    {
	      ESTADO_SIGUIENTE=FUN_ESTADO_ABIERTO();
	    }
	    else if(ESTADO_SIGUIENTE==ESTADO_SEMICERRADO)
	    {
	      ESTADO_SIGUIENTE=FUN_ESTADO_SEMICERRADO();
	    }
	    else if(ESTADO_SIGUIENTE==ESTADO_CERRADO)
	    {
	      ESTADO_SIGUIENTE=FUN_ESTADO_CERRADO();
	    }
	    else if(ESTADO_SIGUIENTE==ESTADO_INICIO)
	    {
	      ESTADO_SIGUIENTE=FUN_ESTADO_INICIO();
	    }
	    else {break;}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  htim15.Init.Prescaler = 64-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 20000-1;
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
  sConfigOC.Pulse = 500;
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
  sBreakDeadTimeConfig.BreakFilter = 0;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 LED_GREEN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//ESTA INTERRUPCION ENTRA CADA 0.01s
	  if (htim == &htim3)
	  {
	  static unsigned int Cont_Button_active = 0;	//Hace el conteo para el filtro en el flanco alto
	  static unsigned int Cont_Button_unactive = 0; //Hace el conteo para el filtro en el flanco bajo
	  static uint16_t Pulso=0;	//Lo que hace es avisar que tuvo un pulso alto para pasar al siguiente pulso
	  static uint16_t CAMBIO=0;	//ES USADO PARA DAR UN TIEMPO DE ESPERA ENTRE CADA PULSO

	  	if (HAL_GPIO_ReadPin(GPIOA,BUTTON_IN)==0)
	    {
	    /* Entrara aqui cuando se precione el push button
	     * El contador comenzara a contar, cuando pase el tiempo de HUMBRAL_TIME
	     * se considerara al push button como activo, cuando se deje de pulsar
	     * el contador se reiniciara*/
	    	SENAL_STATE=0;
	    	Cont_Button_active++;
	    	if(Cont_Button_active >= HUMBRAL_TIME)
	    	{
				if((Cont_Button_unactive>=10)&&(Cont_Button_unactive<=100))
				{
		/* Cuando se precione por segunda vez el push button y que tenga un tiempo
		 * en el esto bajo mayor a 20 y menor a 100, entonces entrara aqui
		 * y devolvienod
		 * */
					CAMBIO=2;
					Pulso=HIGH;
					Cont_Button_active = 0;
					Cont_Button_unactive=0;
				}else
				{
		/* Cuando se precione por primera vez el push button, entrara aqui
		 * Para dar a entender que se ha precionado y darle tiempo a la segunda pulsacion
		 * se le asigna a la variable Pulso un HIGH.
		 *
		 * Reseteamos los contadores de alto y bajo.*/
					//SENAL_STATE=0;
					CAMBIO=1;
					Pulso=HIGH;
					Cont_Button_active = 0;
					Cont_Button_unactive=0;
				}
	    	}

	    }else if(Pulso==HIGH){
	    /* Cuando dejo de pulsar el push button, entro aqui
	     * Se resetea el contador en alto y comienza a contar el contador en bajo*/
	    		Cont_Button_active = 0;
	    		Cont_Button_unactive++;

	    		if(Cont_Button_unactive>130)
				{
	    /* Cuando no se vuelve a pulsar el push button entra
	     * devuelve el valor de CAMBIO y resetea todos los valores*/
	    			SENAL_STATE=CAMBIO;
					Pulso=LOW;
					Cont_Button_unactive=0;
				}
			}
	 }
}

//----------------------------------------Maquina de estado---------------------------------------------------//
uint16_t FUN_ESTADO_INICIO (void)
{
   ESTADO_ANTERIOR=ESTADO_ACTUAL;
   ESTADO_ACTUAL=ESTADO_INICIO;

//rutina de inicio completada

   for(;;)
   {
      /*Prueba de leds
       * for(byte i=0; i<3;i++){
        digitalWrite(LED, LOW);
        digitalWrite(LED1, LOW);
        delay(500);
        digitalWrite(LED, HIGH);
        digitalWrite(LED1, HIGH);
        delay(500);
        digitalWrite(LED, HIGH);
        digitalWrite(LED1, LOW);
        delay(500);
        digitalWrite(LED, LOW);
        digitalWrite(LED1, HIGH);
        delay(500);
      }
	   // ESTADO ABIERTO
	   			  SERVOS(1,0);
	   	  // ESTADO SEMI_CERRADO
	   			  SERVOS(0,0);
	   	  // ESTADO CERRADO
	   			  SERVOS(0,1);
	   	  // APUNTAR
	   			  SERVOS(1,1);*/
    if(INICIO_STATE==TRUE)
    {
      return ESTADO_ABIERTO;
    }

    //HAL_Delay(1);

   }
}

uint16_t FUN_ESTADO_ABIERTO (void)
{
   ESTADO_ANTERIOR=ESTADO_ACTUAL;
   ESTADO_ACTUAL=ESTADO_ABIERTO;
   //realizan las funciones de estado abierto
   SENAL_STATE=0;//Evita que cambie de estado
   HAL_GPIO_WritePin(GPIOA,BLUE_LED,LOW);
   HAL_GPIO_WritePin(GPIOA,RED_LED,LOW);

   HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
   SERVOS(1,0);
   HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
   HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);

for(;;){
    //retorno a semicerrado
     if(SENAL_STATE==1)
     {
      return ESTADO_SEMICERRADO;
     }
     //retorno cerrado
     if(SENAL_STATE==2)
     {
      return ESTADO_CERRADO;
     }
     HAL_Delay(1);
  }
}

uint16_t FUN_ESTADO_SEMICERRADO (void)
{
   ESTADO_ANTERIOR=ESTADO_ACTUAL;
   ESTADO_ACTUAL=ESTADO_SEMICERRADO;
  //realizan las funciones de estado semicerrado

   SENAL_STATE=0;//Evita que cambie de estado
   HAL_GPIO_WritePin(GPIOA,BLUE_LED,LOW);
   HAL_GPIO_WritePin(GPIOA,RED_LED,HIGH);

   HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
   SERVOS(0,0);
   HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
   HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);

for(;;){
//cierre
    //retorno a ABIERTO
     if(SENAL_STATE==1)
     {
      return ESTADO_ABIERTO;
     }
     //retorno cerrado
     if(SENAL_STATE==2)
     {
      return ESTADO_CERRADO;
     }
     HAL_Delay(1);
   }
}

uint16_t FUN_ESTADO_CERRADO (void)
{
   ESTADO_ANTERIOR=ESTADO_ACTUAL;
   ESTADO_ACTUAL=ESTADO_CERRADO;
  //realizan las funciones de estado semicerrado

   SENAL_STATE=0;//Evita que cambie de estado
   HAL_GPIO_WritePin(GPIOA,BLUE_LED,HIGH);
   HAL_GPIO_WritePin(GPIOA,RED_LED,LOW);

   HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
   SERVOS(0,1);
   HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
   HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);

for(;;){

    //retorno a ABIERTO
     if(SENAL_STATE==1)
     {
      return ESTADO_ABIERTO;
     }
     //retorno a SEMICERRADO
     if(SENAL_STATE==2)
     {
      return ESTADO_SEMICERRADO;
     }
     HAL_Delay(1);
  }
}

//-------------------------Servos-----------------------//
/* Si a servo1 o a servo2
 * se le coloca 1, se pondra en 180 grados
 * pero si se le coloca, 0 se pondra en 0 grados
 * Se le debe poner
 *
 * El servo1 es el TOP
 * El servo2 es el BUTTOM
 *
 * ESTADO ABIERTO = SERVOS(1,0);
 * ESTADO SEMI_CERRADO = SERVOS(0,0);
 * ESTADO CERRADO = SERVOS(0,1);
 * APUNTAR = SERVOS(1,1);
 *
 * */
void SERVOS(uint8_t servo1, uint8_t servo2)
{
//Servo1
	if((servo1==1) && (htim15.Instance->CCR1!=2499))
	{
		//0 a 180
		for(int i=500; i<2500;i+=2)
		  {
			 htim15.Instance->CCR1 = i;
			 HAL_Delay(2);
		  }
	} else if((servo1==0) && (htim15.Instance->CCR1!=499))
	{
		//180 a 0
		for(int j=2500; j>500;j-=2)
		  {
			 htim15.Instance->CCR1 = j;
			 HAL_Delay(2);
		  }
	}

//Servo2
	if((servo2==1) && (htim15.Instance->CCR2!=2499))
	{
		//0 a 180
		for(int k=500; k<2500;k+=2)
		  {
			 htim15.Instance->CCR2 = k;
			 HAL_Delay(2);
		  }
	} else if((servo2==0) && (htim15.Instance->CCR2!=499))
	{
		//180 a 0
		for(int l=2500; l>500;l-=2)
		  {
			 htim15.Instance->CCR2 = l;
			 HAL_Delay(2);
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
