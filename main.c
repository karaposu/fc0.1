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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define BNO055_I2C_ADDR1                (0x28)
#define BNO055_I2C_ADDR2                (0x29)
#define BNO055_CALIB_STAT_ADDR        (0X35)
#define BNO055_OPR_MODE_ADDR        (0X3D)  //111101
#define BNO055_OPERATION_MODE_NDOF        (0X0C)

#define BNO055_I2C_ADDR3                (0x50)
#define BNO055_I2C_ADDR4                (0x52)
char device_active=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t buf_rec[1]={0};

float gx,gy,gz,heigth;
struct bno055_euler_float_t eulerAngles;

 int16_t euler_h,euler_p,euler_r;
u8 debug=5;
uint32_t adc_value=0;
uint8_t temp = 3;
uint8_t range;
uint8_t  g_usart1_rx_buf[9];
//uint8_t	 i2cBuf[8];

uint8_t	 RecBuf[8];
uint16_t	lidar_distance;
  uint8_t un_decoded_data[13];
 uint8_t counter ,buffrec[5],cout_doku;
  uint8_t Rx_indx, Rx_data[2], TBuffer[150],Transfer_cplt ,Rx_Buffer[30] ;


float old_axis_error_gx=0;
float current_axis_error_gx=0;
float old_axis_error_gy=0;
float current_axis_error_gy=0;
float old_axis_error_gz=0;
float current_axis_error_gz=0;
float old_axis_error_heigth=0;
float current_axis_error_heigth=0;

 struct bno055_euler_float_t eulerAngles;
 
 uint16_t  thr=0;
uint16_t  min_thr=950;
uint16_t  midlvl_thr=1200;
uint16_t effect_a ,effect_b,effect_c,effect_d; 








extern ADC_HandleTypeDef hadc1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
  //I2C_HandleTypeDef  hi2c;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Init(&hadc1);
  HAL_ADC_Start_IT(&hadc1); 		HAL_Delay(10);
	
	
	HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_1);
HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_2);
HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_3);
HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_4);


		
		 TIM3->CCR1=min_thr;    //  PA6
		  TIM3->CCR2=min_thr;    //PA7
		 TIM3->CCR3=min_thr;   //PB0
		  TIM3->CCR4=min_thr;     //PB1
	
	
	
	
	
	
	
	//HAL_TIM_Base_Start_IT(&htim1);
 struct bno055_t bno055;
	
	
	HAL_UART_Receive_DMA(&huart2, g_usart1_rx_buf, 9);

__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); //UART_IT_IDLE
	
lidar_distance=	TF_RX_Proc(g_usart1_rx_buf, 9);
	
	 if  (mpu_initialise(&bno055)==1    )  {HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_12 ,GPIO_PIN_SET);}			
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 
	 		thr=min_thr;
	 
  while (1)
  {  
		
	//	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
		//HAL_Delay(500);
		

	//	lidar_distance=	TF_RX_Proc(g_usart1_rx_buf, 9);
		  process_gyro();
		  bno055_convert_float_euler_hpr_deg(&eulerAngles);
	gx=	eulerAngles.h; gy=	eulerAngles.p ; gz=	eulerAngles.r; heigth= lidar_distance;
		
//	effect_a=	pid_calculate( gx  ,0, 1,0 ,0 , 0 ,  old_axis_error_gx  , current_axis_error_gx); old_axis_error_gx=current_axis_error_gx;
//	effect_b=		pid_calculate( gy  ,0, 1,0 ,0 , 0 ,  old_axis_error_gy  , current_axis_error_gy); old_axis_error_gy=current_axis_error_gy;
//	effect_c=	pid_calculate( gz  ,0, 1,0 ,0 , 0 ,  old_axis_error_gz  , current_axis_error_gz); old_axis_error_gz=current_axis_error_gz;
//	effect_d=		pid_calculate( heigth  ,50, 1,0 ,0 , 0 ,  old_axis_error_heigth  , current_axis_error_heigth); old_axis_error_heigth=current_axis_error_heigth;
//		
		
		
		

		
		
		 TIM3->CCR1=thr   -effect_b - effect_c  ;    //  PA6
		 TIM3->CCR2=thr    +effect_c-effect_a  ;    //PA7
		 TIM3->CCR3=thr			+effect_c+effect_a				;   //PB0
		 TIM3->CCR4=thr			+effect_b-effect_c		;     //PB1
		HAL_Delay(10);
		
		
		
		
		
		
		
		
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
