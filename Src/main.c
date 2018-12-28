
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "display.h"
#include "accelerometer_define.h"
#include "giroscope_define.h"
#include <stdbool.h>
#include <math.h>
#include "spi_1.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define DEBOUNCE_TIME 100

static uint32_t lastPress = 0;
enum {ACCEL, MAGNET, GYRO, NUM_SENSOR} volatile type = ACCEL;
uint8_t a_config[] = {0x27, 0, 0, 0x80};
uint8_t m_config[] = {0x10, 0x20, 0};
uint8_t g_config[] = {L3GX_CTRL_REG1 | 0x40, 0xF, 0, 0, 0xA0};
int16_t a_data[3]={0};
uint8_t g_buf[7]={0};
int16_t * g_data = (uint16_t*)(g_buf + 1);
struct {float data[3]; float length;} max_val[NUM_SENSOR];
volatile bool acc_conf = false;
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void EXTI0_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
	if (HAL_GetTick() - lastPress < DEBOUNCE_TIME)
		return;
	lastPress = HAL_GetTick();
}

void HAL_SYSTICK_Callback(void)
{
    if (HAL_GetTick() - lastPress == DEBOUNCE_TIME &&
	HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
	type = (type + 1) % NUM_SENSOR;
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
	// We finished setting up Accel
	acc_conf = true;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	char str[100];
	float acc_g[3];
	if (type == ACCEL)
		for (int i = 0; i < 3; i++)
			acc_g[i] = (a_data[i] >> 4) / 1024.;
	else {
		acc_g[0] = ((int16_t) __REV16(a_data[0])) / 1100.;
		acc_g[1] = ((int16_t) __REV16(a_data[2])) / 1100.;
		acc_g[2] = ((int16_t) __REV16(a_data[1])) / 1100.;
	}
	float acc_mod = sqrt(acc_g[0]* acc_g[0] + acc_g[1]*acc_g[1] + acc_g[2]*acc_g[2]);
	float angle =  (atan2(acc_g[1],acc_g[0]) * 180) / 3.1415926535;
	if (acc_mod > max_val[type].length) {
		for (int i = 0; i < 3; i++)
			max_val[type].data[i] = acc_g[i];
		max_val[type].length = acc_mod;
	}
	display_set_write_position(0, 0);
	if (type == ACCEL)
		display_write("--áëóåìåòïíåôò--");
	else
		display_write("--íáçî¶ôïíåôò--");
	display_set_write_position(3, 0);
	display_write("íÁËÓÉÍÁÌØÎ¦ ÚÎÁÞÅÎÎÑ:");
	for (int i = 0; i < 3; i++) {
		display_set_write_position(1, 28 * i);
		snprintf(str, 100, "%6.3f%c", acc_g[i], type == ACCEL? 'g' : 'G');
		display_write(str);
	}
	for (int i = 0; i < 3; i++) {
		display_set_write_position(4, 28 * i);
		snprintf(str, 100, "%6.3f%c", max_val[type].data[i], type == ACCEL? 'g' : 'G');
		display_write(str);
	}
	display_set_write_position(2, 0);
	snprintf(str, 100, "%8.4f%c %8.4fœ", acc_mod, type == ACCEL? 'g' : 'G', angle);
	display_write(str);
	display_set_write_position(5, 0);
	snprintf(str, 100, "%8.4f%c", max_val[type].length, type == ACCEL? 'g' : 'G');
	display_write(str);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef * hspi) {
	display_set_write_position(0, 0);
	for (int i = 0; i < 84; i++)
		display_putletter('-');
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi) {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	char str[100];
	float gyro_g[3] = { g_data[0] * 70 / 95, g_data[1] * 70 / 95, g_data[2] * 70 / 95 };
	float gyro_mod = sqrt(gyro_g[0]* gyro_g[0] + gyro_g[1]*gyro_g[1] + gyro_g[2]*gyro_g[2]);
	if (gyro_mod > max_val[GYRO].length) {
		for (int i = 0; i < 3; i++)
			max_val[GYRO].data[i] = gyro_g[i];
		max_val[GYRO].length = gyro_mod;
	}
	display_set_write_position(0, 0);
	display_write("--  ç¶òïóëïð  --");
	display_set_write_position(3, 0);
	display_write("íÁËÓÉÍÁÌØÎ¦ ÚÎÁÞÅÎÎÑ:");
	for (int i = 0; i < 3; i++) {
		display_set_write_position(1, 28 * i);
		snprintf(str, 100, "%7.3f", gyro_g[i] / 1000);
		display_write(str);
	}
	for (int i = 0; i < 3; i++) {
		display_set_write_position(4, 28 * i);
		snprintf(str, 100, "%7.2f", max_val[GYRO].data[i] / 1000);
		display_write(str);
	}
	display_set_write_position(2, 0);
	snprintf(str, 100, "%10.5f", gyro_mod / 1000);
	display_write(str);

	display_set_write_position(5, 0);
	snprintf(str, 100, "%10.5f", max_val[GYRO].length / 1000);
	display_write(str);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi) {
	HAL_SPI_RxCpltCallback(hspi);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef conf = {
	  .Pin = GPIO_PIN_0,
	  .Mode = GPIO_MODE_IT_RISING,
	  .Pull = GPIO_PULLDOWN,
	  .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_Init(GPIOA, &conf);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 4, 0);


//  __HAL_RCC_GPIOD_CLK_ENABLE();

//  GPIO_InitStruct.Pin = GPIO_PIN_15;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  display_init();
  HAL_I2C_Mem_Write_IT(&hi2c1, ACC_ADDRESS<<1, CTRL_REG1_A | 0x80, 1, a_config, 4);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_SPI_Transmit_IT(&hspi1, g_config, 5);
  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_SR_BSY) || !__HAL_SPI_GET_FLAG(&hspi1, SPI_SR_TXE));
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  while (!acc_conf)
	  __WFI();
  acc_conf = false;
  HAL_I2C_Mem_Write_IT(&hi2c1, MAG_ADDRESS<<1, CRA_REG_M, 1, m_config, 3);
  while (!acc_conf)
	  __WFI();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  display_clear();

	  switch (type) {
	  case ACCEL:
		  HAL_I2C_Mem_Read_IT(&hi2c1, ACC_ADDRESS<<1, OUT_X_L_A | 0x80, 1, (uint8_t *) a_data, 6);
		  break;
	  case MAGNET:
		  HAL_I2C_Mem_Read_IT(&hi2c1, MAG_ADDRESS<<1, OUT_X_H_M, 1, (uint8_t *) a_data, 6);
		  break;
	  case GYRO:
		  g_buf[0] = L3GX_OUT_X_L | 0xC0;
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, g_buf, g_buf, 7);
		  break;
	  }
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

	  HAL_Delay(1);
	  while (HAL_GetTick() % 1000)
		  __WFI();
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
