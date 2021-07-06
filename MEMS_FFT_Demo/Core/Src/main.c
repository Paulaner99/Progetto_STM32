//* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* GRUPPO 28 -> TOMMASO UGOLINI; RICCARDO PAOLINI; GIULIA KODRIC'; GIACOMO PETTA.
 *
 * 		PROGETTO 2
 *
 * ACCENNO ALLE SCELTE PROGETTUALI:
 * 		ABBIAMO DECISO DI FAR 'BLINKARE' IL LED ROSSO QUANDO LA TRASMISSIONE NON È ATTIVA.
 * 		INOLTRE QUANDO INTERROMPIAMO LA TRASMISSIONE METTIAMO IN PAUSA IL DMA
 * 		PER RISPARMIARE ENERGIA OLTRE AD INTERROMPERE L'ACCENSIONE DEI LED.
 * 		ABBIAMO ANCHE PENSATO DI INTRODURRE UNA DOPPIA SOGLIA AL POSTO DELLA SOGLIA SINGOLA
 * 		PER RIDURRE IL 'FLICKERING' DEI LED.
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "i2s.h"
#include "pdm2pcm.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4_discovery_lis3dsh.h"
#include <stdio.h>

#include <string.h>

#include <math.h>
#include "stm32f4xx.h"
#include "arm_math.h"
#include "math_helper.h"
#include "arm_const_structs.h"

#define FFT_SIZE 1024

//#define STREAM_RAW_DATA
//#define STREAM_FFT

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define __FPU_PRESENT             1U       /*!< FPU present                                   */
/* ----------------------------------------------------------------------
 * Defines for each of the tests performed
 * ------------------------------------------------------------------- */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

typedef enum {STATE_IDLE,STATE_HALF,STATE_FULL} i2cState;

int16_t data_col[FFT_SIZE*2];
int counter_samples = 0;		//used as index for the array data_col
uint16_t txBuf[128];
uint16_t pdmRxBuf[128];
uint16_t MidBuffer[16];
uint8_t txstate = 0;
i2cState rxstate = 0;

// variables for the mean of each slice of the magnitude
float mean[4];

// transfer data from data to data_col
void append_data(uint16_t data)
{
	data_col[counter_samples++] = data;
}

float fft_in_buf[FFT_SIZE*2];		//FFT_SIZE=1024 -> size=2048
float signal_spectrum[FFT_SIZE/2];	//FFT_SIZE=1024 -> size=512
float fft_mag_out_buf[FFT_SIZE*2];	//FFT_SIZE=1024 -> size=2048

// UART variables
uint8_t dataReceived = 0;	//Indicates when data received and read from UART
uint8_t chRX = 0;	// contain character read from UART
char to_send[MAX_S];
uint8_t n_to_send=0;

// variable to indicate state
uint8_t streamActive = OFF;

// variables for FFT
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	MX_DMA_Init();
	MX_TIM3_Init();
	MX_USART2_UART_Init();
	MX_I2S2_Init();
	MX_CRC_Init();
	MX_PDM2PCM_Init();
	/* USER CODE BEGIN 2 */

	//Configuring systick
	if (HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq)) > 0U){	//systick configuration
		return HAL_ERROR;
	}

	//Enabling UART Receiver Not Empty interrupt
	LL_USART_EnableIT_RXNE(USART2);
	//Enabling UART
	LL_USART_Enable(USART2);
	//Enabling microphone via I2S and using DMA to transfer data
	HAL_I2S_Receive_DMA(&hi2s2, &pdmRxBuf[0],64);
	HAL_I2S_DMAPause(&hi2s2);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		//if i receive data from UART
		if (dataReceived == 1){
			dataReceived = 0;

			//if the received character is equal to what we expect
			if (chRX == START_CHAR) {
				streamActive = 1 - streamActive;	//toggling the value of streamActive

				printf("Stream Toggle\r\n");
			}
			else
				printf("Wrong command!\nPress 's' to start...\r\n");

			if(!streamActive){
				//Reset active LEDs
				HAL_I2S_DMAPause(&hi2s2);
				LL_GPIO_ResetOutputPin(GPIOD,LL_GPIO_PIN_15);
				LL_GPIO_ResetOutputPin(GPIOD,LL_GPIO_PIN_12);
				LL_GPIO_ResetOutputPin(GPIOD,LL_GPIO_PIN_13);
				LL_GPIO_ResetOutputPin(GPIOD,LL_GPIO_PIN_14);
			}

			if(streamActive)
				HAL_I2S_DMAResume(&hi2s2);
		}


		if(streamActive){

			// Initializing variables
			for(int i = 0; i < FFT_SIZE*2; i++){
				data_col[i] = 0;
				fft_mag_out_buf[i] = 0;
				fft_in_buf[i] = 0;

				switch (rxstate)
				{
				case STATE_HALF:
					PDM_Filter(&pdmRxBuf[0],&MidBuffer[0], &PDM1_filter_handler); 		//filtering
					for (int i=0; i<16;i++)
						append_data(MidBuffer[i]);	//transferring data from MidBuffer to data_col
					rxstate = STATE_IDLE;	//setting state to idle
					break;
				case STATE_FULL:
					PDM_Filter(&pdmRxBuf[64],&MidBuffer[0], &PDM1_filter_handler);	//filtering
					for (int i=0; i<16;i++)
						append_data(MidBuffer[i]); //transferring data from MidBuffer to data_col
					rxstate = STATE_IDLE;	//setting state to idle
					break;
				case STATE_IDLE:
					//do some task...
					break;
				}

				if(counter_samples >= FFT_SIZE){
					/* Copy data to preserve buffer content */
					for(int inx_data = 0; inx_data<FFT_SIZE; inx_data++)
						fft_in_buf[inx_data] = (float)data_col[inx_data];

					/* Reset index of the buffer accumulating PCM samples */
					counter_samples = 0;

					///// FFT
					arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_in_buf, ifftFlag, doBitReverse);

					///// MAGNITUDE
					arm_cmplx_mag_f32(fft_in_buf, fft_mag_out_buf, FFT_SIZE);

					/* Just keep the useful part of the spectrum (FFT points/2) */
					for(int inx_data = 0; inx_data<FFT_SIZE/2; inx_data++)
						signal_spectrum[inx_data] = fft_mag_out_buf[inx_data];

					/* Do your processing here */

					// data are already in signal_spectrum of size 512 float

					// dividing it into 4 parts means: 128 values for each part

					//mean for part 1
					arm_mean_f32(&signal_spectrum[0], MAG_SLICE_SIZE, &mean[FIRST_SLICE]);
					//mean for part 2
					arm_mean_f32(&signal_spectrum[MAG_SLICE_SIZE - 1], MAG_SLICE_SIZE, &mean[SECOND_SLICE]);
					//mean for part 3
					arm_mean_f32(&signal_spectrum[(MAG_SLICE_SIZE * 2) - 1], MAG_SLICE_SIZE, &mean[THIRD_SLICE]);
					//mean for part 4
					arm_mean_f32(&signal_spectrum[(MAG_SLICE_SIZE * 3) - 1], MAG_SLICE_SIZE, &mean[FOURTH_SLICE]);

					// now we have the means for every slice of the magnified spectrum in the array mean[]

					// data are ready to be transmitted

					// transmit via UART

					//using this function to put in the string to_send the message to send via UART
					sprintf(to_send,"MEAN_1:%2.2f\tMEAN_2:%2.2f\tMEAN_3:%2.2f\tMEAN_4:%2.2f\r\n", mean[FIRST_SLICE], mean[SECOND_SLICE], mean[THIRD_SLICE], mean[FOURTH_SLICE]);
					//calculating length of the string to send via UART
					n_to_send=(uint8_t)strlen(to_send);

					LL_USART_EnableIT_TC(USART2); //enableTX interrupt

					// BLUE_LED ON if mean[FIRST_SLICE] higher than HIGH THRESHOLD
					if(mean[FIRST_SLICE] > H_THRESHOLD)
						LL_GPIO_SetOutputPin(GPIOD,LL_GPIO_PIN_15);
					// BLUE_LED OFF if mean[FIRST_SLICE] lower than LOW THRESHOLD
					else if(mean[FIRST_SLICE] < L_THRESHOLD)
						LL_GPIO_ResetOutputPin(GPIOD,LL_GPIO_PIN_15);

					// GREEN_LED ON if mean[SECOND_SLICE] higher than HIGH THRESHOLD
					if(mean[SECOND_SLICE] > H_THRESHOLD)
						LL_GPIO_SetOutputPin(GPIOD,LL_GPIO_PIN_12);
					// GREEN_LED OFF if mean[SECOND_SLICE] lower than LOW THRESHOLD
					else if(mean[SECOND_SLICE] < L_THRESHOLD)
						LL_GPIO_ResetOutputPin(GPIOD,LL_GPIO_PIN_12);

					// ORANGE_LED ON if mean[THIRD_SLICE] higher than HIGH THRESHOLD
					if(mean[THIRD_SLICE] > H_THRESHOLD)
						LL_GPIO_SetOutputPin(GPIOD,LL_GPIO_PIN_13);
					// ORANGE_LED OFF if mean[THIRD_SLICE] lower than LOW THRESHOLD
					else if(mean[THIRD_SLICE] < L_THRESHOLD)
						LL_GPIO_ResetOutputPin(GPIOD,LL_GPIO_PIN_13);

					// RED_LED ON if mean[FOURTH_SLICE] higher than HIGH THRESHOLD
					if(mean[FOURTH_SLICE] > H_THRESHOLD)
						LL_GPIO_SetOutputPin(GPIOD,LL_GPIO_PIN_14);
					// RED_LED OFF if mean[FOURTH_SLICE] lower than LOW THRESHOLD
					else if(mean[FOURTH_SLICE] < L_THRESHOLD)
						LL_GPIO_ResetOutputPin(GPIOD,LL_GPIO_PIN_14);

				}
			}



#ifdef STREAM_RAW_DATA
			for(int inx_data = 0; inx_data<FFT_SIZE; inx_data++)
			{
				printf("%d\r\n", data_col[inx_data]);

			}
#endif

#ifdef STREAM_FFT
			for(int inx_data = 0; inx_data<FFT_SIZE/2; inx_data++)
			{
				printf("%2.2f\r\n", fft_mag_out_buf[inx_data]);
			}
#endif

#ifdef STREAM_RAW_DATA
			while(1);
#endif

#ifdef STREAM_FFT
			while(1);
#endif

		}

	}

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
	while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
	{
	}
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while(LL_RCC_HSE_IsReady() != 1)
	{

	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 336, LL_RCC_PLLP_DIV_2);
	LL_RCC_PLLI2S_ConfigDomain_I2S(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLI2SM_DIV_8, 192, LL_RCC_PLLI2SR_DIV_2);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while(LL_RCC_PLL_IsReady() != 1)
	{

	}
	LL_RCC_PLLI2S_Enable();

	/* Wait till PLL is ready */
	while(LL_RCC_PLLI2S_IsReady() != 1)
	{

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{

	}
	LL_SetSystemCoreClock(168000000);

	/* Update the time base */
	if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
	{
		Error_Handler();
	}
	LL_RCC_SetI2SClockSource(LL_RCC_I2S1_CLKSOURCE_PLLI2S);
}

/* USER CODE BEGIN 4 */

/*
 * This function finds the maximum value in an array and returns its index in the array.
 * @param 	a[]		array to find maximum
 * @param 	n 		size of the array
 * @return 	index 	array index of maximum value in array
 */

int __io_putchar(int ch)
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	LL_USART_TransmitData8(USART2, (uint8_t)ch);

	/* Loop until the end of transmission */
	while (LL_USART_IsActiveFlag_TC(USART2) == 0)
	{}

	return ch;
}


void HAL_I2S_RxHalfCpltCallback (I2S_HandleTypeDef *hi2s) {
	rxstate = STATE_HALF;
}

void HAL_I2S_RxCpltCallback (I2S_HandleTypeDef *hi2s) {
	rxstate = STATE_FULL;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
