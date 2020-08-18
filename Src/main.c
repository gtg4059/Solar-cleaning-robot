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
#include "stm32f4xx.h"
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
void normal_set();
void Rrotation_set();
void Lrotation_set();
void back_set();
void Solenoid_init();
void Rstep();
void Lstep();
void Allstep();
void Stop();
int _write(int file, unsigned char* p, int len) {
	//HAL_UART_Transmit(&huart1, p, len, 10);
	int i;
	for (i = 0; i < len; i++)
		ITM_SendChar(*p++);
	return len;
}
/*int __io_putchar(int ch) {
 ITM_SendChar(ch);
 return (ch);
 }
 int _write(int file, char *ptr, int len) {
 int DataIdx;
 for (DataIdx = 0; DataIdx < len; DataIdx++) {
 __io_putchar(*ptr++);
 }
 return len;
 }*/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int32_t EncoderCount3 = 0;
int32_t EncoderCount4 = 0;

char Mode = 'S';
float q[4], quatRadianAccuracy;
unsigned char MotorFlag = 0, TimEvent = 0;
double error_previous = 0, Ltime = 0.01, I_ctl, Control;
typedef struct _Gain {
	double Kp;
	double Ki;
	double Kd;
} Gain;
int initialPosFlag = 0, TargetDeg = 35, TargetEnc = 35, RotDirection = 0,
		stepcount = 9, DegTrig = 0;
uint32_t light = 400, TimCount = 0;
Gain LeftMotor = { 80, 5, 0.5 };
Gain RightMotor = { 1, 0.03, 0.005 };
uint32_t IC1Value = 0, IC2Value = 0, IC3Value = 0, IC4Value = 0;
float DutyCycle = 0;
uint8_t stepfront = 0, Ble[1], b = 0, LRflag = 0, dirsave = 0, Serialavailable =
		0, LRTrig = 0;
;
;
uint16_t adcval[8], MotorSpeed = 7000, imu_tick = 0;
float adcbuffer[8], adcvaltrace[8];
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
	MX_TIM7_Init();
	MX_TIM1_Init();
	MX_ADC1_Init();
	MX_TIM9_Init();
	MX_TIM13_Init();
	MX_TIM11_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	//Motor Initiate
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	//Servo Initiate
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	//Ultra Initiate
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
	//Ultra Trigger Signal
	__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 84);
	// Internal Timer Initiate
	HAL_TIM_Base_Start_IT(&htim7);
	// Encoder Initiate
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
	HAL_ADC_Start_DMA(&hadc1, adcval, 8);
	HAL_UART_Receive_IT(&huart1, Ble, 1);
	normal_set();
	Solenoid_init();
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 320); //축 해제
	//IMU Initiate
	//BNO080_GPIO_SPI_Initialization();
	//BNO080_Initialization();
	//BNO080_enableRotationVector(500);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	uint32_t current_tick = HAL_GetTick();
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		while (HAL_GetTick() - current_tick < 10)
			;
		current_tick = HAL_GetTick();
		/*LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_4); //M
		 LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_2); //FL
		 HAL_Delay(4000);
		 LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_2); //FL
		 LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_1); //FR
		 HAL_Delay(4000);
		 LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_1); //FR
		 LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_0); //ML
		 HAL_Delay(4000);
		 LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_0); //ML
		 LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_12); //MR
		 HAL_Delay(4000);
		 LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_12); //MR
		 LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_11); //BL
		 HAL_Delay(4000);
		 LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_11); //BL
		 LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_10); //BR
		 HAL_Delay(2000);
		 LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_10); //BR
		 LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_4); //M
		 HAL_Delay(4000);*/
		/*if (BNO080_dataAvailable() == 1) {
		 q[0] = BNO080_getQuatI();
		 q[1] = BNO080_getQuatJ();
		 q[2] = BNO080_getQuatK();
		 q[3] = BNO080_getQuatReal();
		 quatRadianAccuracy = BNO080_getQuatRadianAccuracy();
		 Quaternion_Update(&q[0]);
		 }*/
		//모터위치수신
		EncoderCount3 = TIM3->CNT;	//R
		EncoderCount4 = TIM4->CNT;	//L

		//초기화, 전진, 회전, 정지모드 전환
		switch (Mode) {
		//초기화 모드
		case 'I':
			if (MotorFlag == 0) {
				normal_set();
				Solenoid_init();
				LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_4);
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 5000); //left 초기화
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
				if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14) == GPIO_PIN_SET) { //L근접센서 도달
					MotorFlag = 1;
					__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0); //left 초기화
					LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_8);
					LL_GPIO_SetOutputPin(GPIOG, GPIO_PIN_7);
					if (initialPosFlag == 0) {

						initialPosFlag = 1;
						TIM4->CNT = 0; //R
						EncoderCount4 = 0;
						__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 5000); //right 초기화
					}
					break;
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
				break;
			} else if (MotorFlag == 2) {
				if (EncoderCount3 >= TargetEnc) { //엔코더 특정 지점 인식시 활성화
					if (!TimEvent) {
						__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
						__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
						Stop();
						TimCount = 0;
						TimEvent = 1;
					}
					//normal_set();
					Allstep();

					//servo up
					if (TimCount >= 600) { //내부타이머(종료 후 항상 초기화 필요)
						__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 500); //축 해제
						TIM3->CNT = 0;
						EncoderCount3 = 0;
						TIM4->CNT = 0;
						EncoderCount4 = 0;
						initialPosFlag = 0;
						TimEvent = 0;
						LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_4);
					}
					break;
				} else {
					__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 5000);
				}
			}
			break;
			//전진 모드
		case 'G':
			//근접센서 인식 ->잠시 멈추고 공압 전환 후 전진
			if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13) == GPIO_PIN_SET
					|| HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14) == GPIO_PIN_SET) {
				if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13) == GPIO_PIN_SET)
					LRTrig = 1; //R근접
				else if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14) == GPIO_PIN_SET)
					LRTrig = 0; //L근접

				if (!initialPosFlag) {
					if (adcvaltrace[0] > -30)
						stepfront = 1; //Lstep flow
					else if (adcvaltrace[1] > -30)
						stepfront = 4; //Rstep flow
					initialPosFlag = 1;
					Stop();
					Allstep();
					if (stepfront == 1) {
						LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_2); //FL
						stepfront++;
					} else if (stepfront == 2) {
						LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_0); //ML
						stepfront++;
					} else if (stepfront == 3) {
						LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_11); //BL
						stepfront = 0;
					} else if (stepfront == 4) {
						LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_1); //FR
						stepfront++;
					} else if (stepfront == 5) {
						LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_12); //MR
						stepfront++;
					} else if (stepfront == 6) {
						LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_10); //BR
						stepfront = 0;
					}
					if (!TimEvent) {
						TimCount = 0;
						TimEvent = 1;
					}
				}

				if (TimCount >= 800) {
					normal_set();
					if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13) == GPIO_PIN_SET) //R근접
						Lstep();
					else if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14)
							== GPIO_PIN_SET) //L근접
						Rstep();
					TimEvent = 0;
					initialPosFlag = 0;
				}
			} else {
				normal_set();
				LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_4);
			}
			//전진 Motor Speed 조절
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
			if (TimCount > 500) {
				//모든다리 공압 대기압으로
				LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_2); //FL
				LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_1); //FR
				LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_0); //ML
				LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_12); //MR
				LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_11); //BL
				LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_10); //BR
				switch (dirsave) {
				case 76: //좌회전
					Lrotation_set();
					break;
				case 82: //우회전
					Rrotation_set();
					break;
				}
				if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13) == GPIO_PIN_SET
						|| HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14)
								== GPIO_PIN_SET) { //인식시 1회씩 실행
					if (!initialPosFlag) {
						initialPosFlag = 1;
						//stepwalk++;
					}
				} else
					initialPosFlag = 0;
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 4000);
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 4000 * 0.7);
			} else {
				LL_GPIO_SetOutputPin(GPIOD, GPIO_PIN_4); // 가운데 다리 진공
				__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 320); //가운데 서보모터 내림
				if (LRTrig)
					Lstep(); //R근접
				else
					Rstep();
				Stop(); //전진 정지
			}
			break;
		case 'J':
			if (adcvaltrace[0] > -30) {
				Lstep();
				LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_2); //FL
			} else if (adcvaltrace[1] > -30) {
				Rstep();
				LL_GPIO_ResetOutputPin(GPIOD, GPIO_PIN_1); //FR
			}

			break;
		case 'S':
			Stop();
			break;
		}
		for (int i = 0; i < 8; i++) {
			adcbuffer[i] = adcval[i]; //((adcval[i]/4096-0.94)/0.018);
			adcvaltrace[i] = ((adcbuffer[i] / 4096 - 0.94) / 0.018);
		}
	}
	//adc = ((adcval[0] / 4096-0.94)/0.018);

//값 수신 코드
//LL_USART_TransmitData8(USART1,&a);
//HAL_UART_Transmit(&huart1,&a,1,10);
//printf("%" PRId32, IC2Value);
//printf("  %" PRId32 ",%" PRId32, TIM4->CNT, TIM3->CNT);
//printf(" %f,%f,%f\n", BNO080_Roll, BNO080_Pitch, BNO080_Yaw);

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

uint8_t serial_statue = 0, serial_mode = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		HAL_UART_Receive_IT(&huart1, Ble, 1);
		if (serial_statue == 1) {
			if (Ble[0] == 115) 	//s
				serial_mode = 1;
			else if (Ble[0] == 73) 	//I
				serial_mode = 2;
			else if (Ble[0] == 71) 	//G
				serial_mode = 3;
			else if (Ble[0] == 76 || Ble[0] == 82) {  //L, R
				dirsave = Ble[0];
				serial_mode = 4;
			} else if (Ble[0] == (uint8_t) 0x72) 	//r
				serial_mode = 5;
			else if (Ble[0] == (uint8_t) 0x03) {  		//etx end

				switch (serial_mode) {

				case 1:
					Mode = 'S';
					break;
				case 2:
					TimCount = 0;
					MotorFlag = 0;
					initialPosFlag = 0;
					Mode = 'I';
					break;
				case 3:
					TimCount = 0;
					Mode = 'G';
					break;
				case 4:
					TimCount = 0;
					Mode = 'T';

					break;
				case 5:

					printf("R:%d;P:%d;FL:%d;FR:%d;ML:%d;MR:%d;BL:%d;BR:%d;\n",
							(int) (BNO080_Roll), (int) (BNO080_Pitch),
							(int) ((adcval[0] / 4096 - 0.94) / 0.018),
							(int) ((adcval[1] / 4096 - 0.94) / 0.018),
							(int) ((adcval[2] / 4096 - 0.94) / 0.018),
							(int) ((adcval[3] / 4096 - 0.94) / 0.018),
							(int) ((adcval[4] / 4096 - 0.94) / 0.018),
							(int) ((adcval[5] / 4096 - 0.94) / 0.018));

					break;
				}
				serial_mode = 0;
				serial_statue = 0;
				//Ble[0] = 0x00;

			} else if (Ble[0] == (uint8_t) 0x02) {		//serial restart

				serial_mode = 0;
				Ble[0] = 0x00;
			}
		}
		if (Ble[0] == (uint8_t) 0x02) { 				//stx start
			serial_statue = 1;
		}
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
