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
// Definiera binära mönster för varje tillstånd
#define INIT_PATTERN      0b11111 // s_init
#define CARS_GO_PATTERN   0b01100 // s_cars_go
#define PUSHED_WAIT_PATTERN 0b01100 // s_pushed_wait
#define CARS_STOPPIN_PATTERN 0b01010 // s_cars_stoppin
#define WALK_GETREADY_PATTERN 0b01001 // s_walk_getready
#define WALK_PATTERN      0b10001 // s_walk
#define WALK_STOPPIN_PATTERN 0b01001 // s_walk_stoppin
#define CAR_GETREADY_PATTERN 0b01011 // s_car_ready
#define ALL_RED				0b01001
#define EVQ_SIZE 10

enum event {
    ev_none = 0,        // Ingen händelse, standardvärde
    ev_button_push,     // När knappen trycks
    ev_state_timeout,   // Timeout-händelse
	ev_error	= -99
};
enum state {
	s_init,
    s_cars_go,        // Tillstånd där bilarna kör
	s_pushed_wait,
    s_cars_stoppin,
    s_walk_getready,
    s_walk,
	s_walk_stoppin,
	s_cars_getready,
	s_all_red
};

enum event evq[EVQ_SIZE];
int evq_count	= 0;
int evq_front_ix = 0;   // Pekare till den första händelsen i kön
int evq_rear_ix = 0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t ticks_left_in_state = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int is_button_pressed() {

	// Kontrollera knappens tillstånd
	return (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) ? 1 : 0;

}

void set_traffic_lights(enum state s) {
	uint16_t pattern;
	switch (s) {
	case s_init:
		// Initiera alla lampor (eller tänd dem som krävs för initiering)
		pattern = INIT_PATTERN;
		break;
	case s_all_red:
		pattern = ALL_RED;
		break;
	case s_cars_go:
		pattern = CARS_GO_PATTERN;
		break;
	case s_pushed_wait:
		pattern = PUSHED_WAIT_PATTERN;
		break;
	case s_cars_stoppin:
		pattern = CARS_STOPPIN_PATTERN;
		break;
	case s_walk_getready:
		pattern = WALK_GETREADY_PATTERN;
		break;
	case s_walk:
		pattern = WALK_PATTERN;
		break;
	case s_walk_stoppin:
		pattern = WALK_STOPPIN_PATTERN;
		break;
	case s_cars_getready:
		pattern = CAR_GETREADY_PATTERN;
		break;
	default:
		pattern = 0b00000; // Ingen lampor tända
		break;
	}
	// Ställ in lysdioderna enligt det valda mönstret
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,
			(pattern & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PA8
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,
			(pattern & (1 << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PA9
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,
			(pattern & (1 << 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PC7
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,
			(pattern & (1 << 3)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PB6
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,
			(pattern & (1 << 4)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PA7
}

void push_button_light_on() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // Sätter PB10 till hög för att tända dioden
}

void push_button_light_off() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // Sätter PB10 till låg för att släcka dioden
}
void evq_init(void) {
	for (int i = 0; i < EVQ_SIZE; i++) {
		evq[i] = ev_error;  // Fyll hela kön med felhändelse
	}
}

// if queue is full, ignore e
void evq_push_back(enum event e) {
// if queue is full, ignore e
	if (evq_count < EVQ_SIZE) {
		evq[evq_rear_ix] = e;
		evq_rear_ix++;
		evq_rear_ix %= EVQ_SIZE;
		evq_count++;
	}
}
enum event evq_pop_front() {
enum event e = ev_none;
if (evq_count > 0) {
	e = evq[evq_front_ix];
	evq[evq_front_ix] = ev_error; // detect stupidity
	evq_front_ix++;
	evq_front_ix %= EVQ_SIZE;
	evq_count--;
}
return e;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == B1_Pin) {  // B1_Pin refererar till den GPIO-pin B1 är kopplad till
        evq_push_back(ev_button_push);  // Lägg till ett knapptryck-event i kön
    }
}

int systick_count = 0;
void my_systick_handler()
{
	/*systick_count++;
	if (systick_count == 1000)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		systick_count = 0;
		evq_push_back(ev_state_timeout);
	}*/
	if(ticks_left_in_state > 0)
	{
		ticks_left_in_state --;
		if(ticks_left_in_state == 0)
		{
			evq_push_back(ev_state_timeout);
		}
	}
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
	evq_init();  // Initiera event-kön
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	enum state st = s_init;
	enum event ev = ev_none;
	//uint32_t curr_tick = 0;
	//uint32_t last_tick = HAL_GetTick();
	//uint32_t passed_time = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	//int last_press;
	//int curr_press;
	while (1) {
	/*
		curr_press = is_button_pressed();
		curr_tick = HAL_GetTick();
		passed_time = curr_tick - last_tick;
		ev = ev_none;
		last_tick = curr_tick;
		if (curr_press && !last_press )
		{
			evq_push_back(ev_button_push);
			//ev = ev_button_push;
		}
		last_press = curr_press;

		if (ticks_left_in_state > 0) {
			if (passed_time >= ticks_left_in_state) {
				ticks_left_in_state = 0;
				evq_push_back(ev_state_timeout);
				//ev = ev_state_timeout;
			} else {
				ticks_left_in_state -= passed_time;
			}
		}
*/
		ev = evq_pop_front();

		switch (st) {
		case s_init:
			set_traffic_lights(s_init);
			if (ev == ev_button_push) {
				st = s_all_red;
				ticks_left_in_state = 2000;
				set_traffic_lights(s_all_red);// Ingen timeout här
			}
			//last_tick = HAL_GetTick(); // Uppdatera tidpunkt
			break;
		case s_all_red:
			// All red lights state
			if (ev == ev_state_timeout) {
				set_traffic_lights(s_cars_go);
				ticks_left_in_state = 2000;  // T.ex. 2 sekunder i s_all_red
				st = s_cars_go;
				//last_tick = HAL_GetTick(); // Uppdatera tidpunkt
			}
			break;
		case s_cars_go:

			if (ev == ev_button_push) {
				push_button_light_on();
				ticks_left_in_state += 4000;
				set_traffic_lights(s_pushed_wait);
				st = s_pushed_wait;
			}
		case s_pushed_wait:
			// Wait state after button press
			if (ev == ev_state_timeout) {
				set_traffic_lights(s_cars_stoppin);
				ticks_left_in_state = 5000;
				st = s_cars_stoppin;
				//last_tick = HAL_GetTick(); // Uppdatera tidpunkt
			}
			break;
		case s_cars_stoppin:
			if (ev == ev_state_timeout) {
				set_traffic_lights(s_walk_getready);
				ticks_left_in_state = 3000; // T.ex. 3 sekunder i s_walk_getready
				st = s_walk_getready;
				//last_tick = HAL_GetTick(); // Uppdatera tidpunkt
			}
			break;
		case s_walk_getready:
			// Red for all (prepare for walk state)
			if (ev == ev_state_timeout) {
				set_traffic_lights(s_walk);
				ticks_left_in_state = 5000;  // T.ex. 5 sekunder för walk
				st = s_walk;
				push_button_light_off();
				//last_tick = HAL_GetTick(); // Uppdatera tidpunkt
			}
			break;
		case s_walk:
			if (ev == ev_state_timeout) {
				set_traffic_lights(s_walk_stoppin);
				ticks_left_in_state = 2000; // T.ex. 2 sekunder för cars_getready
				 st = s_walk_stoppin;
				 //last_tick = HAL_GetTick(); // Uppdatera tidpunkt
			}
			break;
		case s_walk_stoppin:
			// Yellow light for pedestrians
			if (ev == ev_state_timeout) {
				set_traffic_lights(s_cars_getready);
				ticks_left_in_state = 1000; // T.ex. 2 sekunder för cars_getready
				st = s_cars_getready;
				//last_tick = HAL_GetTick(); // Uppdatera tidpunkt
			}
			break;

		case s_cars_getready:
			// Prepare cars for green light
			if (ev == ev_state_timeout) {
				set_traffic_lights(s_cars_go);
				st = s_cars_go;
				//last_tick = HAL_GetTick(); // Uppdatera tidpunkt
			}
			break;
		default:
			return 0;
			break;
		}

		//last_press = curr_press;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA7 PA8 PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
	while (1) {
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
