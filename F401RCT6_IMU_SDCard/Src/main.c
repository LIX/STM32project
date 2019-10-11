
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "MPU9250_RegisterMap.h"
#include "string.h"
#include "SparkFunMPU9250-DMP.h"
#include "sd_diskio.h"
#include "ff.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t I2C_data_ready;
long gyro_bias[3]={0xB1,0xDE,0xFFFFFFFB};
long accel_bias[3]={0x2D, 0x152, 0xFFFFFFA4};
RTC_TimeTypeDef rtc_time;
RTC_DateTypeDef rtc_date;
volatile uint16_t Vbat;
uint32_t Vbat_tick;
char str_time[80];
char str_imu[128];
FIL MyFile;
uint32_t SD_dected;
uint32_t SDData_ready;
HAL_SD_CardInfoTypeDef fff;

uint32_t firsttime;
uint8_t filename[17];

uint32_t _1s_counter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void Vbat_monitor();
extern "C" {void rtc_set(uint8_t* , uint32_t );}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void update_str_time()
{
	  HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
	  sprintf(str_time,"%04d%02d%02d%02d%02d%02d",2000+rtc_date.Year, rtc_date.Month, rtc_date.Date, rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds);	
}

void SD_DataWrite()
{
 uint32_t wbytes; /* File write counts */
	if (HAL_GetTick()-firsttime>=300000)	//300s
	{
		firsttime=HAL_GetTick();
		update_str_time();
		memcpy(filename, &(str_time[0]), 12);
	}
	
	
//	if (Vbat>2172)	//电压不足时判断为断电 停止写入
//	{
	
		 if(f_open(&MyFile, (TCHAR*)&filename, FA_WRITE | FA_OPEN_APPEND) == FR_OK)
		 {
			 uint8_t tmp[103]={0};
			 memcpy(tmp, &(str_time[8]), 6);
			 tmp[6]='.';
			 uint32_t t=HAL_GetTick();
			 tmp[7]='0'+(t/100%10);
			 tmp[8]='0'+(t/10%10);
			 tmp[9]=':';
			 memcpy(&(tmp[10]), str_imu, 0x5D);
			 //tmp[103]='\0';
			 
			 
			 if (f_write(&MyFile, tmp, 103, (UINT *)&wbytes)==FR_OK)
			 {
				 //f_sync(&MyFile);
				 //f_printf(&MyFile, (TCHAR*)tmp);
				 HAL_GPIO_TogglePin(led_SYS_state_GPIO_Port, led_SYS_state_Pin);
			 }
		//		f_write(&MyFile, &(str_time[8]), 7, (UINT *)&wbytes);
		//	 
		//	 f_write(&MyFile, ":\t", sizeof(":\t"), (UINT *)&wbytes);
		//	 f_write(&MyFile, str_imu, 0x5D, (UINT *)&wbytes);
		//	 f_write(&MyFile, "\n", sizeof("\n"), (UINT *)&wbytes);
			
		 }
		  
		 f_close(&MyFile);
//	 }
	//HAL_GPIO_WritePin(led_SYS_state_GPIO_Port, led_SYS_state_Pin, GPIO_PIN_SET);
// if(f_open(&MyFile, "STM32.TXT", FA_OPEN_ALWAYS | FA_WRITE) == FR_OK)
// {
//		 f_lseek(&MyFile, MyFile.fsize);
//		//f_write(&MyFile, str_time, 15, (UINT *)&wbytes);
//		f_write(&MyFile, str_time, (UINT)strlen(str_time), (UINT *)&wbytes);
//		 f_write(&MyFile, ktext, sizeof(ktext), (UINT *)&wbytes) ;
// }
// 	 f_close(&MyFile);

}



//每次复位后模拟一次插拔
void USB_soft_unplug()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
                        
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 
  GPIO_PIN_RESET);                                            
  HAL_Delay(220);
  //先把PA12拉低再释放，利用D+模拟USB的拔插动作   
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
}

void Vbat_monitor()
{
  
  if (Vbat>=2358)	//3.8V/2/3.3V*4096
	  HAL_GPIO_WritePin(led_bat_GPIO_Port, led_bat_Pin, GPIO_PIN_RESET);
  else if ( (Vbat>=2234) && (Vbat<2358) )	//3.6V-3.8V
  {
	  if (Vbat_tick>=500)
	  {
		  Vbat_tick=0;
		  HAL_GPIO_TogglePin(led_bat_GPIO_Port, led_bat_Pin);
	  }
  }
  else
  {
	  if (Vbat_tick>=100)
	  {
		  Vbat_tick=0;
		  HAL_GPIO_TogglePin(led_bat_GPIO_Port, led_bat_Pin);
	  }
  }
}

uint32_t chararrayToint(uint8_t* p, uint32_t len)
{
	uint32_t tmp=0, mult = 1;
	while (len--)
	{
		tmp+=(p[len]-'0')*mult;
		mult*=10;
	}
	return tmp;
}

void rtc_set(uint8_t* Buf, uint32_t len)
{
//	if (len==19)
//	{
		uint8_t* p=Buf;
		if((p[0]=='R') && (p[1]=='T') && (p[2]=='C') && (p[3]==':') && (p[12]=='/'))
		{
			rtc_date.Year=chararrayToint(&p[4], 4)-2000;
			rtc_date.Month=chararrayToint(&p[8], 2);
			rtc_date.Date=chararrayToint(&p[10], 2);
			HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
			rtc_time.Hours=chararrayToint(&p[13], 2);
			rtc_time.Minutes=chararrayToint(&p[15], 2);
			rtc_time.Seconds=chararrayToint(&p[17], 2);
			HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
		}
		else
		{
			uint8_t str[]="RTC formatv should be: RTC:20190705/151350\r\n";
			CDC_Transmit_FS(str, (uint16_t)sizeof(str));
		}
//	}
	
//  rtc_time.Hours=1;
//  rtc_time.Minutes=19;
//  rtc_time.Seconds=0;
//  HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
//  rtc_date.Date=5;
//  rtc_date.Month=7;
//  rtc_date.WeekDay=5;
//  rtc_date.Year=19;
//  HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
}

MPU9250_DMP imu;

void printIMUData(void)
{  
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
	
	
//  float accelX = imu.calcAccel(imu.ax);
//  float accelY = imu.calcAccel(imu.ay);
//  float accelZ = imu.calcAccel(imu.az);
//  float accelX = imu.calcAccel(imu.ax-accel_bias[0]);
//  float accelY = imu.calcAccel(imu.ay-accel_bias[1]);
//  float accelZ = imu.calcAccel(imu.az-accel_bias[2]);
  float gyroX = imu.calcGyro(imu.gx - gyro_bias[0]);
  float gyroY = imu.calcGyro(imu.gy - gyro_bias[1]);
  float gyroZ = imu.calcGyro(imu.gz - gyro_bias[2]);
  float accelX = imu.calcAccel(imu.ax)-accel_bias[0]/16384.f;
  float accelY = imu.calcAccel(imu.ay)-accel_bias[1]/16384.f;
  float accelZ = imu.calcAccel(imu.az)-accel_bias[2]/16384.f;
//  float gyroX = imu.calcGyro(imu.gx )-gyro_bias[0]/(32768.f/250);
//  float gyroY = imu.calcGyro(imu.gy )-gyro_bias[1]/(32768.f/250);
//  float gyroZ = imu.calcGyro(imu.gz )-gyro_bias[2]/(32768.f/250);
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);
//  

////	buf[0]=4;
////	void * p=&accelY;
////	
////	buf[1]=(*(uint32_t*)p)>>24;
////	buf[2]=(*(uint32_t*)p)>>16;
////	buf[3]=(*(uint32_t*)p)>>8;
////	buf[4]=(*(uint32_t*)p)&0xff;
////	SEND_BUF(buf);
//printf("imu.gx:%d gyro_bias[0]:%d\r\n",imu.gx, gyro_bias[0]);
//  printf("Accel: %5.3f,%5.3f,%5.3f;",accelX,accelY,accelZ);
//  printf("Gyro: %5.3f,%5.3f,%5.3f;",gyroX,gyroY,gyroZ);
//  printf("Mag: %.3f,%.3f,%.3f",magX,magY,magZ);
////  printf("Time: %lu ms",imu.time);
//  printf("\r\n");
	  
	update_str_time();
	  //CDC_Transmit_FS((uint8_t*)str_time, (uint16_t)sizeof(str_time));

	
	
	sprintf(str_imu,"Accel:%7.3f,%7.3f,%7.3f;  Gyro: %7.3f,%7.3f,%7.3f;  Mag:%7.3f,%7.3f,%7.3f\r\n",accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magX,magY,magZ);
	//sprintf(str_imu,"Accel: %7.3f,%7.3f,%7.3f;Gyro: %7.3f,%7.3f,%7.3f\r\n",accelX,accelY,accelZ,gyroX,gyroY,gyroZ);
	CDC_Transmit_FS((uint8_t*)str_imu, (uint16_t)strlen(str_imu));
	SDData_ready=1;
	
	

//	nrf24_packed.count=24;
//	nrf24_packed.acc_int16[0]=imu.ax;
//	nrf24_packed.acc_int16[1]=imu.ay;
//	nrf24_packed.acc_int16[2]=imu.az;
//	nrf24_packed.gyro_int16[0]=imu.gx;
//	nrf24_packed.gyro_int16[1]=imu.gy;
//	nrf24_packed.gyro_int16[2]=imu.gz;
//	nrf24_packed.mag_q30[0]=imu.mx;
//	nrf24_packed.mag_q30[1]=imu.my;
//	nrf24_packed.mag_q30[2]=imu.mz;
//	SEND_BUF((uint8_t *)&nrf24_packed);
}

void I2CSetup()
{
  uint8_t pdata[10]={0};
	HAL_I2C_Mem_Read(&hi2c2, ((0x68<<1)), MPU9250_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, pdata, 1, 1000);
//  while (pdata[0]!=0X71)
//  {
//	  //uint8_t str[]="unable to communicate with MPU9250\n";
//	  //CDC_Transmit_FS(str, sizeof(str));
//	  //printf("unable to communicate with MPU9250\n");
//	  HAL_GPIO_TogglePin(led_SYS_state_GPIO_Port,led_SYS_state_Pin);
//	  HAL_I2C_Mem_Read(&hi2c2, ((0x68<<1)), MPU9250_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, pdata, 1, 1000);
//	  HAL_Delay(1000);
//  }

  mpu_run_6500_self_test(gyro_bias, accel_bias,0);  
//  gyro_bias[3]={0xA0,0xF2,0xFFFFFFEC};
//  accel_bias[3]={0xE2, 0x101, 0xFFFFFE55};
//  HAL_Delay(100);

  imu.begin();
  
  
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(250); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(4); // Set accel to +/-2g
  imu.setSampleRate(100); // Set accel/gyro sample rate to 4Hz
  imu.setLPF(98);
  imu.setCompassSampleRate(100); // Set mag rate to 4Hz

  // Use enableInterrupt() to configure the MPU-9250's 
  // interrupt output as a "data ready" indicator.
  imu.enableInterrupt(1);

  // The interrupt level can either be active-high or low.
  // Configure as active-low, since we'll be using the pin's
  // internal pull-up resistor.
  // Options are INT_ACTIVE_LOW or INT_ACTIVE_HIGH
  imu.setIntLevel(INT_ACTIVE_LOW);

  // The interrupt can be set to latch until data has
  // been read, or to work as a 50us pulse.
  // Use latching method -- we'll read from the sensor
  // as soon as we see the pin go LOW.
  // Options are INT_LATCHED or INT_50US_PULSE
  imu.setIntLatched(INT_50US_PULSE);
}

void I2CLoop()
{
	  imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
	  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  printIMUData();
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_12)
	{
		;
		I2C_data_ready=1;
		
		
	}
}

uint8_t readddd;
void HAL_SYSTICK_Callback(void)
{
	static int i=0;
	i++;
	if (i%200==0)
	{
		readddd=1;
		
	}
	Vbat_tick++;
	
	if(SDData_ready==1)
	{
		SD_DataWrite();
		SDData_ready=0;
	}
	
	Vbat=HAL_ADC_GetValue(&hadc1);
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
	USB_soft_unplug();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
	I2CSetup();
	firsttime = HAL_GetTick();		//写入SD卡前读取时间与这个变量相减，判断是否要新建文件
	update_str_time();
	filename[15]='t';
	filename[14]='x';
	filename[13]='t';
	filename[12]='.';
	memcpy(filename, &(str_time[0]), 12);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		Vbat_monitor();
	  
	if (I2C_data_ready==1)
	{
		//HAL_GPIO_TogglePin(led_SYS_state_GPIO_Port,led_SYS_state_Pin);
		I2CLoop();
		I2C_data_ready=0;
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 2;

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
