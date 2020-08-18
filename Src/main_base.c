/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BNO080.h"
#include "Quaternion.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <inttypes.h>
#include <Math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//void PositionControlA();
uint16_t PositionControl(double error);
void get_IMU_Value();
void normal_set();
void Rrotation_set();
void Lrotation_set();
void back_set();
void Solenoid_init();
void Rstep();
void Lstep();
void Allstep();
void Stop();
int __io_putchar(int ch) {
	ITM_SendChar(ch);
	return (ch);
}
int _write(int file, char *ptr, int len) {
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		__io_putchar(*ptr++);
	}
	return len;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int32_t EncoderCount3 = 0;
int32_t EncoderCount4 = 0;
float q[4], quatRadianAccuracy, errorRd, errorPd;
char Mode = 'I';
unsigned char MotorFlag = 0, TimEvent = 0;
double error_previous = 0, Ltime = 0.01, I_ctl, Control;
typedef struct _Gain {
	double Kp;
	double Ki;
	double Kd;
} Gain;
int initialPosFlag = 0, TargetDeg = 35, TargetEnc = 36, RotDirection = 0,
		stepcount = 9, DegTrig = 0;
uint32_t light = 400, TimCount = 0;
Gain LeftMotor = { 80, 5, 0.5 };
Gain RightMotor = { 1, 0.03, 0.005 };
uint32_t IC1Value = 0, IC2Value = 0, IC3Value = 0, IC4Value = 0;
float DutyCycle = 0;
uint8_t stepwalk = 0, Ble[3], b = 0;
uint16_t adcval[8], MotorSpeed = 8000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_SPI2_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART1_UART_Init();
	MX_TIM7_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_ADC1_Init();
	MX_TIM9_Init();
	MX_TIM13_Init();
	MX_TIM11_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */

	//MotorStart
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	//ServoStart
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	//UltraStart
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 84);
	HAL_TIM_Base_Start_IT(&htim7);
	/*HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
	 HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
	 HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
	 HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);*/
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
	//HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_ALL);
	BNO080_Initialization();
	BNO080_enableRotationVector(2500);
	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 300);
	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 300);
	//LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_4);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	HAL_ADC_Start_DMA(&hadc1, adcval, 8);
	__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 84);
	normal_set();
	Solenoid_init();
	HAL_UART_Receive_IT(&huart1, Ble, 3);
	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_4);
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 360); //축 해제
	HAL_Delay(2000);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t current_tick = HAL_GetTick();
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		while (HAL_GetTick() - current_tick < 10)
			;
		get_IMU_Value();
		//if (HAL_UART_Receive(&huart1, Ble, 3, 2) == HAL_OK) stepwalk = Ble[0] - 48;

		current_tick = HAL_GetTick();

		//모터위치수신
		EncoderCount3 = TIM3->CNT;	//R
		EncoderCount4 = TIM4->CNT;	//L
		switch (Mode) {
		//초기화 모드
		case 'I':
			if (MotorFlag == 0) {
				LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_4);

				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 1500); //left 초기화
				if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14) == GPIO_PIN_SET) { //L근접센서 도달
					MotorFlag = 1;
					__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0); //left 초기화
					LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_8);
					LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_7);
					if (initialPosFlag == 0) {

						initialPosFlag = 1;
						TIM4->CNT = 0; //R
						EncoderCount4 = 0;
						__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 1500); //right 초기화
					}
				}
			} else if (MotorFlag == 1) { //좌측 근접센서 도달

				if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13) == GPIO_PIN_SET) { //R근접센서 도달
					if (initialPosFlag == 1) {
						initialPosFlag = 2;
						TIM3->CNT = 0; //R
						EncoderCount3 = 0;
						MotorFlag = 2;
					}
				}
			} else if (MotorFlag == 2) {
				if (EncoderCount3 >= TargetEnc) {
					if (!TimEvent) {
						__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
						Stop();
						TimCount = 0;
						TimEvent = 1;
					}
					//normal_set();
					Allstep();

					//servo up
					if (TimCount >= 1000) {
						__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 500); //축 해제
						//I_ctl = 0;
						//error_previous = 0;
						MotorFlag = 0;
						TIM3->CNT = 0;
						EncoderCount3 = 0;
						TIM4->CNT = 0;
						EncoderCount4 = 0;
						initialPosFlag = 0;
						TimEvent = 0;
						Mode = 'G';
						LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_4);
					}
					break;
				} else {
					//PositionControl(EncoderCount3, TargetDeg);
					__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 1500);
				}
			}
			break;

			//저전압 모드
		case 'L':

			break;

			//Go
		case 'G':
			//Vaccum Pad Control, Sync Control
			if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13) == GPIO_PIN_SET
					|| HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14) == GPIO_PIN_SET) { //R인식 -> 좌측 전진

				if (stepwalk > stepcount) {
					LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_4);
					__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 360); //축 가압
					if (TimCount >= 400) {
						stepwalk = 0;
						if (Ble[0] == 115)
							Mode = 'S';
						else
							Mode = 'T';
						LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_2); //FL
						LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_1); //FR
						LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_0); //ML
						LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_12); //MR
						LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_11); //BL
						LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_10); //BR
						TimEvent = 0;
						switch (RotDirection) {
						case 0:
							Rrotation_set();
							RotDirection = 1;
							stepcount = 9;
							break;
						case 1:
							Lrotation_set();
							RotDirection = 2;
							stepcount = 9;
							break;
						}

					}
					break;
				} else if (!initialPosFlag) {
					initialPosFlag = 1;
					stepwalk++;
					Stop();
					Allstep();
					if (!TimEvent) {
						TimCount = 0;
						TimEvent = 1;
					}
				}
				if (TimCount >= 160) {
					normal_set();
					if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13) == GPIO_PIN_SET)
						Lstep();
					else if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14)
							== GPIO_PIN_SET)
						Rstep();
					TimEvent = 0;
				}
			} else {
				LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_4);
				initialPosFlag = 0;
			}
			//Motor Speed
			if (EncoderCount3 - EncoderCount4 < -2) {
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, MotorSpeed); //R
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0); //L

			} else if (EncoderCount3 - EncoderCount4 > 2) {
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, MotorSpeed * 0.7);

			} else {
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, MotorSpeed);
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, MotorSpeed * 0.7);
			}
			break;

		case 'T':
			//body turn
			if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13) == GPIO_PIN_SET
					|| HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14) == GPIO_PIN_SET) {
				if (!initialPosFlag) {
					initialPosFlag = 1;
					stepwalk++;
				}
			} else
				initialPosFlag = 0;
			if (stepwalk > 9) { //normal 3
				stepwalk = 0;
				Stop();
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
				initialPosFlag = 0;
				//I_ctl = 0;
				//error_previous = 0;
				//Control = 300;
				normal_set();
				Mode = 'I';
				break;
			} else {
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1,
						(5000 - stepwalk * 400));
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2,
						(5000 - stepwalk * 400) * 0.7);
			}

			break;
		case 'S':
			Stop();
			Allstep();
			LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_4);

			break;
		}

//값 수신 코드
//LL_USART_TransmitData8(USART1,&a);
//HAL_UART_Transmit(&huart1,&a,1,10);
//printf("%" PRId32, IC2Value);
//printf("  %" PRId32 ",%" PRId32, TIM4->CNT, TIM3->CNT);
//printf(" %f,%f,%f\n", BNO080_Roll, BNO080_Pitch, BNO080_Yaw);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* USART1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USER CODE BEGIN 4 */
uint16_t PositionControl(double error) {		//우측
	double P_ctl = LeftMotor.Kp * error;
	I_ctl += LeftMotor.Ki * error * Ltime;
	double D_ctl = LeftMotor.Kd * (error - error_previous) / (Ltime);
	Control = P_ctl + I_ctl + D_ctl;
	error_previous = error;
	return (uint16_t) fmin(Control, 8000);
	/*if (Control > 0) {
	 LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_5); //R
	 LL_GPIO_ResetOutputPin(GPIOG, GPIO_PIN_6);
	 __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, fmin(Control, 8000));
	 } else {
	 LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_6);
	 LL_GPIO_ResetOutputPin(GPIOG, GPIO_PIN_5);
	 __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, fmin(-Control, 8000));
	 }*/

}
void get_IMU_Value() {
	if (BNO080_dataAvailable() == 1) {
		q[0] = BNO080_getQuatI();
		q[1] = BNO080_getQuatJ();
		q[2] = BNO080_getQuatK();
		q[3] = BNO080_getQuatReal();
		quatRadianAccuracy = BNO080_getQuatRadianAccuracy();
		Quaternion_Update(&q[0]);
	}
}
void normal_set() {
	LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_5); //R
	LL_GPIO_ResetOutputPin(GPIOG, GPIO_PIN_6);
	LL_GPIO_ResetOutputPin(GPIOG, GPIO_PIN_7); //L
	LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_8);
}
void Lrotation_set() {
	LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_5); //R
	LL_GPIO_ResetOutputPin(GPIOG, GPIO_PIN_6);
	LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_7); //L
	LL_GPIO_ResetOutputPin(GPIOG, GPIO_PIN_8);
}
void Rrotation_set() {
	LL_GPIO_ResetOutputPin(GPIOG, GPIO_PIN_5); //R
	LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_6);
	LL_GPIO_ResetOutputPin(GPIOG, GPIO_PIN_7); //L
	LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_8);
}
void back_set() {
	LL_GPIO_ResetOutputPin(GPIOG, GPIO_PIN_5); //R
	LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_6);
	LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_7); //L
	LL_GPIO_ResetOutputPin(GPIOG, GPIO_PIN_8);
}
void Stop() {
	LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_5); //R
	LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_6);
	LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_7); //L
	LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_8);
}
void Solenoid_init() {
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_2); //FL
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_1); //FR
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_0); //ML
	LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_12); //MR
	LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_11); //BL
	LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_10); //BR
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_4); //M

}
void Rstep() {
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_2); //FL
	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_1); //FR
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_0); //ML
	LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_12); //MR
	LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_11); //BL
	LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_10); //BR
}
void Lstep() {
	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_2); //FL
	LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_1); //FR
	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_0); //ML
	LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_12); //MR
	LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_11); //BL
	LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_10); //BR
}
void Allstep() {
	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_2); //FL
	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_1); //FR
	LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_0); //ML
	LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_12); //MR
	LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_11); //BL
	LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_10); //BR
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			TIM1->CNT = 0;
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			IC1Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			IC2Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		}
	} else if (htim->Instance == TIM2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			TIM2->CNT = 0;
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			IC3Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			IC4Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		}
	}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM7) {
		if (TimCount >= 65535) {
			TimCount = 65535;
		} else
			TimCount++;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		HAL_UART_Receive_IT(&huart1, Ble, 3);
		stepwalk = Ble[0] - 48;
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
