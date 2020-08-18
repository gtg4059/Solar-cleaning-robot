/* USER CODE BEGIN Header */
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "function.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define swt HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define drivespeed 200
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */


/* Uart 1번에 대한 Printf를 통한 시리얼 통신 활성화---------------------------------------*/
int _write(int file, unsigned char* p, int len)
{
	HAL_UART_Transmit(&huart1, p, len, 10);
	return len;
}

/* millis 반환 전역변수 생성---------------------------------------*/
volatile unsigned long millis;

/* 제어 주기 설정---------------------------------------*/
uint16_t MCU_TIMER_start;
#define MCU_CONTROL_RATE  50   //ms
void timer_loop(unsigned int mcu_ms)
{
  while((uint16_t)millis-MCU_TIMER_start <= mcu_ms);
  MCU_TIMER_start = (uint16_t)millis;
}


/* DC모터 및 엔코더 제어 함수 및 변수---------------------------------------*/
int32_t encoderPos1=0;
int32_t encoderPos2=0;


void doMotorL(int dir, int ccr){
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,MIN(ccr,400));
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,!dir);	//INB
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,dir);	//INA
}

void doMotorR(int dir, int ccr){
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,MIN(ccr,400));
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,dir);	//INB
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,!dir);	//INA
}



void move(int ccr1, int ccr2){
	doMotorL((ccr1>=0)? 1:0,MIN(abs(ccr1),400));
	doMotorR((ccr2>=0)? 1:0,MIN(abs(ccr2),400));
}

float Kparead(int adc)
{
	float meas_v=(3.3*adc)/4096;
	float real_v=(meas_v/3.3)*5;
	float kPa=((real_v/5)-0.92)/0.007652;
	return kPa;
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float roll=0;
float pitch=0;
float yaw=0;
float rolloffset=0;
float pitchoffset=-1.2;
float yawoffset=0;

int ccr1=0;
int ccr2=0;

int pump=0;

uint8_t rx_data;
uint16_t adcval[2];

int vacuum1=0;
int vacuum2=0;

int mode=0;


float rKp = 50;
float rKi = 0;
float rKd = 0;
//직진 PID 게인
float tKp = 50;
float tKi = 0;
float tKd = 0;

//PID Loop time


float targetDeg = 0;
float control = 0;

int forwardspeed=0;
int verticalspeed=0;

uint32_t ICValue=0;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float q[4];
	float quatRadianAccuracy;
	int flag=0;
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
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_PWM_Start_IT(&htim10,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_4);
  HAL_ADC_Start_DMA(&hadc1,&adcval[0],2);
  HAL_UART_Receive_IT(&huart1,&rx_data,1);
  BNO080_Initialization();
  BNO080_enableRotationVector(10000);

  TIM4->CNT=30000;
  TIM5->CNT=30000;
  /*
  HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);*/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  timer_loop (MCU_CONTROL_RATE);


	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,pump);
	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,pump);


	  distance = (((float)(340 * ICValue) / 10000) / 2);
	  if(swt==0){
		  mode=0;
		  flag=0;
	  }
	  if((swt==1)&&(flag==0))
	  {
		  mode=1;
		  submode=0;
		  flag=1;
	  }
	  if(BNO080_dataAvailable()==1)
	  {
		  q[0]=BNO080_getQuatI();
		  q[1]=BNO080_getQuatJ();
		  q[2]=BNO080_getQuatK();
		  q[3]=BNO080_getQuatReal();
		  quatRadianAccuracy=BNO080_getQuatRadianAccuracy();
		  Quaternion_Update(&q[0]);
		  roll=BNO080_Roll-rolloffset;
		  pitch=BNO080_Pitch-pitchoffset;
		  yaw=BNO080_Yaw-yawoffset;
	  }

	  vacuum1=(int)Kparead(adcval[0]);
	  vacuum2=(int)Kparead(adcval[1]);

	  if(mode==0)	//stop mode
	  {
		  if(submode==0)
		  {
			  move(ccr1,ccr2);
			  //fedgecnt(6);
		  }
		  else if(submode==1)
		  {
			  move(-150,-150);
			  cntdelay(2500);
		  }
		  else if(submode==2)
		  {
			  move(0,0);
			  submode=0;
			  ccr1=0;
			  ccr2=0;
		  }
	  }
	  else if(mode==1)	//initialize mode
	  	  {
		  	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,1);
		  	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,1);
		  	  move(200,200);
	  	  }
	  /*
	  else if(mode==1)	//initialize mode
	  {
		  if(submode==0)
		  {
			  control=pid(tKp,tKi,tKd,roll,targetDeg);
			  move(drivespeed-(int)(control),drivespeed+(int)(control));
			  fedgecnt(6);
		  }
		  else if(submode==1)
		  {
			  move(-drivespeed,-drivespeed);
			  cntdelay(2500);
		  }
		  else if(submode==2)
		  {
			  move(0,0);
			  ccr1=0;
			  ccr2=0;
			  submode=0;
			  mode=0;
		  }

	  }*/
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

  /**Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
		{
			TIM2->CNT=0;
		}
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)
		{
			ICValue=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
		}

	}
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM7)
	{
		millis++;
		encoderPos1=(TIM4->CNT)-30000;
		encoderPos2=-(TIM5->CNT)+30000;
	}
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_0)	//PD0 인터럽트 발생
	{
		doEncoderB();
	}
	if(GPIO_Pin==GPIO_PIN_1)	//PD1 인터럽트 발생
	{
		doEncoderA();
	}
	if(GPIO_Pin==GPIO_PIN_2)	//PD2 인터럽트 발생
	{
		doEncoderD();
	}
	if(GPIO_Pin==GPIO_PIN_3)	//PD3 인터럽트 발생
	{
		doEncoderC();
	}
}
uint8_t buf[20];
int serial_statue=0;
int serial_count=0;


int serial_mode=0;

int d2=0;
int d3=0;



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		HAL_UART_Receive_IT(&huart1,&rx_data,1);
		if(serial_statue==1)
		{
			if(rx_data==(uint8_t)0x4D)	//Mode Change	HEX 'M'
			{
				serial_count=0;
				serial_mode=1;
			}
			else if(rx_data==(uint8_t)0x56)	//Vacuum Change   HEX 'V'
			{
				serial_count=0;
				serial_mode=2;
			}
			else if(rx_data==(uint8_t)0x44)	//Motor1 Change   HEX 'D' Left Motor speed
			{
				serial_count=0;
				serial_mode=3;
			}
			else if(rx_data==(uint8_t)0x45)	//Motor1 Change   HEX 'E' Right Motor speed
			{
				serial_count=0;
				serial_mode=4;
			}
			else if(rx_data==(uint8_t)0x52)	//Request Statue Data   HEX 'R'
			{
				serial_count=0;
				serial_mode=5;
			}
			else if(rx_data==(uint8_t)0x03)  		//etx end
			{

				int data=atoi(buf);
				switch(serial_mode)
				{
					case 1 :
						mode=data;
						break;
					case 2 :
						pump=data;
						break;
					case 3 :
						ccr1=data;
						break;
					case 4 :
						ccr2=data;
						break;
					case 5 :
						printf("R:%d;P:%d;M:%d;S:%d;LV:%d;RV:%d;D:%d;T:%d;\n",(int)(BNO080_Roll),(int)(BNO080_Pitch),mode,submode,vacuum1,vacuum2,distance,millis);
						break;
				}


				serial_mode=0;
				serial_count=0;
				serial_statue=0;


				for(int i=0;i<sizeof(buf);i++) buf[i]=0x00;
			}
			else if(rx_data==(uint8_t)0x02)		//serial restart
			{
				if(serial_count!=0)
				{
					serial_count=0;
					serial_mode=0;
					for(int i=0;i<sizeof(buf);i++) buf[i]=0x00;
				}
			}
			else if(serial_mode!=0)
			{
				Append(buf,rx_data);
				serial_count++;
			}
		}
		if(rx_data==(uint8_t)0x02) 				//stx start
		{
			serial_statue=1;
		}

	}
}


void Insert(char *ar, int idx, char ch)
{
    memmove(ar + idx + 1, ar + idx, strlen(ar) - idx + 1);
    ar[idx] = ch;
}

void Append(char *ar, char ch)
{
    Insert(ar, strlen(ar), ch);
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
