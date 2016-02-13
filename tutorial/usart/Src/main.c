/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#define bcd2bin(x)        (((x) & 0x0f) + ((x) >> 4) * 10)
#define bin2bcd(x)        ((((x) / 10) << 4) + (x) % 10)

#define DS3234_REG_SECONDS      0x00
#define DS3234_REG_MINUTES      0x01
#define DS3234_REG_HOURS        0x02
#define DS3234_REG_DAY          0x03
#define DS3234_REG_DATE         0x04
#define DS3234_REG_MONTH        0x05
#define DS3234_REG_YEAR         0x06
#define DS3234_REG_CENTURY      (1 << 7) /* Bit 7 of the Month register */

#define DS3234_REG_CONTROL      0x0E
#define DS3234_REG_CONT_STAT    0x0F

#define RXBUFFERSIZE  33
uint8_t aTxBuffer[] = "www.aisekne.com/f4discofree - 0\r\n";
uint8_t halfsecTicks = 0;

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask01Handle;
osThreadId myTask04Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask01(void const * argument);
void StartTask04(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

struct rtc_time {
	uint8_t tm_sec;
	uint8_t tm_min;
	uint8_t tm_hour;
	uint8_t tm_mday;
	uint8_t tm_mon;
	uint8_t tm_year;
	uint8_t tm_wday;
	uint8_t tm_yday;
	uint8_t tm_isdst;
} now_t = { 06, 57, 22, 13, 2, 16, 6, 0, 0 };

uint8_t CODE11G[] =
		{ 0xE7, 0x44, 0xAD, 0xCD, 0x4E, 0xCB, 0xEB, 0x45, 0xEF, 0xCF };

void vfd_write_display(uint8_t addr, uint8_t digit, uint8_t flag) {
	addr += 0xC0;
	flag |= CODE11G[digit];
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &addr, 1, 1000);
	HAL_SPI_Transmit(&hspi1, &flag, 1, 1000);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

void vfd_write_display_flag(uint8_t addr, uint8_t flag) {
	addr += 0xC0;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &addr, 1, 1000);
	HAL_SPI_Transmit(&hspi1, &flag, 1, 1000);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

void vfd_display() {

	vfd_write_display(0x14, now_t.tm_sec % 10, 0x00);
	vfd_write_display(0x12, now_t.tm_sec / 10, 0x10);
	vfd_write_display(0x10, now_t.tm_min % 10, 0x00);
	vfd_write_display(0x0E, now_t.tm_min / 10, 0x10);
	vfd_write_display(0x0C, now_t.tm_hour % 10, 0x00);
	vfd_write_display(0x0A, now_t.tm_hour / 10, 0x00);
	vfd_write_display(0x08, now_t.tm_mday % 10, 0x00);
	vfd_write_display_flag(0x07, 0x01);
	vfd_write_display(0x06, now_t.tm_mday / 10, 0x00);
	vfd_write_display(0x04, now_t.tm_mon % 10, 0x00);
	vfd_write_display(0x02, now_t.tm_mon / 10, 0x00);

}

void ds3234_set_reg(uint8_t addr, uint8_t val) {
	addr |= 0x80;
	HAL_GPIO_WritePin(DS3234_CS_GPIO_Port, DS3234_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 1000);
	HAL_SPI_Receive(&hspi2, &val, 1, 1000);
	HAL_GPIO_WritePin(DS3234_CS_GPIO_Port, DS3234_CS_Pin, GPIO_PIN_SET);
}

void ds3234_get_reg(uint8_t addr, uint8_t* val) {
	addr &= 0x7F;
	HAL_GPIO_WritePin(DS3234_CS_GPIO_Port, DS3234_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 1000);
	HAL_SPI_Receive(&hspi2, val, 1, 1000);
	HAL_GPIO_WritePin(DS3234_CS_GPIO_Port, DS3234_CS_Pin, GPIO_PIN_SET);
}

void ds3234_read_time() {
	/* Seconds, Minutes, Hours, Day, Date, Month, Year */
	ds3234_get_reg(DS3234_REG_SECONDS, &now_t.tm_sec);
	now_t.tm_sec = bcd2bin(now_t.tm_sec);
	ds3234_get_reg(DS3234_REG_MINUTES, &now_t.tm_min);
	now_t.tm_min = bcd2bin(now_t.tm_min);
	ds3234_get_reg(DS3234_REG_HOURS, &now_t.tm_hour);
	now_t.tm_hour = bcd2bin(now_t.tm_hour & 0x3f);
	ds3234_get_reg(DS3234_REG_DAY, &now_t.tm_wday);
	now_t.tm_wday = bcd2bin(now_t.tm_wday) - 1; /* 0 = Sun */
	ds3234_get_reg(DS3234_REG_DATE, &now_t.tm_mday);
	now_t.tm_mday = bcd2bin(now_t.tm_mday);
	ds3234_get_reg(DS3234_REG_MONTH, &now_t.tm_mon);
	now_t.tm_mon = bcd2bin(now_t.tm_mon & 0x1f) - 1; /* 0 = Jan */
	ds3234_get_reg(DS3234_REG_YEAR, &now_t.tm_year);
	now_t.tm_year = bcd2bin(now_t.tm_year & 0xff) + 100; /* Assume 20YY */
}

void ds3234_set_time() {
	ds3234_set_reg(DS3234_REG_SECONDS, bin2bcd(now_t.tm_sec));
	ds3234_set_reg(DS3234_REG_MINUTES, bin2bcd(now_t.tm_min));
	ds3234_set_reg(DS3234_REG_HOURS, bin2bcd(now_t.tm_hour) & 0x3f);

	/* 0 = Sun */
	ds3234_set_reg(DS3234_REG_DAY, bin2bcd(now_t.tm_wday + 1));
	ds3234_set_reg(DS3234_REG_DATE, bin2bcd(now_t.tm_mday));

	/* 0 = Jan */
	ds3234_set_reg(DS3234_REG_MONTH, bin2bcd(now_t.tm_mon + 1));

	/* Assume 20YY although we just want to make sure not to go negative. */
	if (now_t.tm_year > 100)
		now_t.tm_year -= 100;

	ds3234_set_reg(DS3234_REG_YEAR, bin2bcd(now_t.tm_year));
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();

	/* USER CODE BEGIN 2 */

	uint8_t init_bytes[] = { 0x07, 0x40, 0xC0, 0x01, 0x00, 0x8C, 0x8E, 0x00 };

	for (int i = 0; i < 2; i++) {
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*) (&init_bytes[i]), 1, 1000);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	}

	for (int i = 0; i < 0x16; i++) {
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*) (&init_bytes[2]), 1, 1000);
		HAL_SPI_Transmit(&hspi1, (uint8_t*) (&init_bytes[4]), 1, 1000);
		init_bytes[2]++;
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	}

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) (&init_bytes[5]), 1, 1000);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

//	ds3234_set_time();
	vfd_display();

	ds3234_set_reg(DS3234_REG_CONTROL, 0x00);

//	vfd_write_display_flag(0x03, 0x04);
//	vfd_write_display_flag(0x0F, 0x03);

	while (1) {
	};

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of myTask02 */
	osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 128);
	myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

	/* definition and creation of myTask03 */
	osThreadDef(myTask03, StartTask03, osPriorityNormal, 0, 128);
	myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

	/* definition and creation of myTask01 */
	osThreadDef(myTask01, StartTask01, osPriorityNormal, 0, 128);
	myTask01Handle = osThreadCreate(osThread(myTask01), NULL);

	/* definition and creation of myTask04 */
	osThreadDef(myTask04, StartTask04, osPriorityNormal, 0, 128);
	myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	char i = 0;

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);

		aTxBuffer[30] = ((i++) % 10) + '0';
		HAL_UART_Transmit(&huart3, aTxBuffer, RXBUFFERSIZE, 5000);
		HAL_Delay(1000);
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	__PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* SPI1 init function */
void MX_SPI1_Init(void) {

	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
	hspi1.Init.CRCPolynomial = 10;
	HAL_SPI_Init(&hspi1);

}

/* SPI2 init function */
void MX_SPI2_Init(void) {

	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
	hspi2.Init.CRCPolynomial = 10;
	HAL_SPI_Init(&hspi2);

}

/* USART3 init function */
void MX_USART3_UART_Init(void) {

	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart3);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__GPIOF_CLK_ENABLE()
	;
	__GPIOH_CLK_ENABLE()
	;
	__GPIOA_CLK_ENABLE()
	;
	__GPIOB_CLK_ENABLE()
	;
	__GPIOI_CLK_ENABLE()
	;
	__GPIOC_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DS3234_CS_GPIO_Port, DS3234_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
	GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI_CS_Pin */
	GPIO_InitStruct.Pin = SPI_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ONEHZ_EXTI_Pin */
	GPIO_InitStruct.Pin = ONEHZ_EXTI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ONEHZ_EXTI_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DS3234_CS_Pin */
	GPIO_InitStruct.Pin = DS3234_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(DS3234_CS_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	return;
	halfsecTicks++;
	ds3234_read_time();
	vfd_display();
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument) {

	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(10);
	}
	/* USER CODE END 5 */
}

/* StartTask02 function */
void StartTask02(void const * argument) {
	/* USER CODE BEGIN StartTask02 */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		osDelay(200);
	}
	/* USER CODE END StartTask02 */
}

/* StartTask03 function */
void StartTask03(void const * argument) {
	/* USER CODE BEGIN StartTask03 */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		osDelay(300);
	}
	/* USER CODE END StartTask03 */
}

/* StartTask01 function */
void StartTask01(void const * argument) {
	/* USER CODE BEGIN StartTask01 */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		osDelay(100);
	}
	/* USER CODE END StartTask01 */
}

/* StartTask04 function */
void StartTask04(void const * argument) {
	/* USER CODE BEGIN StartTask04 */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		osDelay(400);
	}
	/* USER CODE END StartTask04 */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
