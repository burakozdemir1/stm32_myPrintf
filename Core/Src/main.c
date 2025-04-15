/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdarg.h> // for create printMessage function
#include <string.h>
#include <stdio.h>

#include "bootloader_command_code.h"
#include "bootloader_command_app.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 char myMessage[] = "Bootloader Try Project!\n";
 /*
  char *myMessage lı ifadelerde size kullanırken sizeof değil strlen kullan

  */
// #define APPLICATION_ADDRESS_1 (uint32_t)0x08008000
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000
#define BL_RX_DATA_LENGTH		   300
//#define TX_TIMEOUT 1000

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

 uint8_t bootloader_rx_data[BL_RX_DATA_LENGTH];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void intToStr(uint8_t *p_str, uint32_t intnum) {
//    uint32_t i, divider = 1000000000, pos = 0, status = 0;
//
//    for (i = 0; i < 10; i++) {
//        p_str[pos++] = (intnum / divider) + 48;
//
//        intnum = intnum % divider;
//        divider /= 10;
//        if ((p_str[pos - 1] == '0') & (status == 0)) {
//            pos = 0;
//        } else {
//            status++;
//        }
//    }
}

void serialPutString(uint8_t *p_string) {
//    uint16_t length = 0;
//
//    while (p_string[length] != '\0') {
//        length++;
//    }
//    HAL_UART_Transmit(&huart2, p_string, length, TX_TIMEOUT);
//}
//HAL_StatusTypeDef serialPutByte(uint8_t param) {
//    return HAL_UART_Transmit(&huart2, &param, 1, TX_TIMEOUT);
}

void goToApps(uint32_t applicationAddress) {
//    uint32_t JumpAddress;
//    typedef void (*pFunction)(void);
//    pFunction JumpToApplication;
//
//    if (((*(__IO uint32_t *)applicationAddress) & 0x2FFE0000) == 0x20000000) {
//        serialPutString("Going to User Application\r\n");
//        // JumpAddress = *(__IO uint32_t *)(APPLICATION_ADDRESS + 4);
//        JumpAddress = *(__IO uint32_t *)(applicationAddress + 4);
//        JumpToApplication = (pFunction)JumpAddress;
//        /* Initialize user application's Stack Pointer */
//        __set_MSP(*(__IO uint32_t *)applicationAddress);
//        JumpToApplication();
//    }
}

void printMessage(char *format,...)
{
	char commingMessage[100];
	va_list vaList;

	va_start(vaList,format);
	vsprintf(commingMessage,format,vaList);

	HAL_UART_Transmit(&huart2, (uint8_t*)commingMessage, strlen(commingMessage), HAL_MAX_DELAY);
	va_end(vaList);

// #include <stdarg.h> // remember!!!

}

void bootloader_uart_data_read(void)
{
//	uint8_t bl_rx_length; // gelen verinin uzunluğunu alacağız
//
//	while(1) // bootloader ıma ait tüm işleri yaptırıp ana koda atlayacağım
//	{
//		memset(bootloader_rx_data, 0, BL_RX_DATA_LENGTH);
//
//		HAL_UART_Receive(&huart1, bootloader_rx_data, 1, HAL_MAX_DELAY); // (length to follow ) gelecek komutun uzunluğunu verecek ve 0.indekse yazılacak
//
//		bl_rx_length = bootloader_rx_data[0]; // boyutu verdik
//
//		HAL_UART_Receive(&huart1, bootloader_rx_data+1, bl_rx_length, HAL_MAX_DELAY);
//		/*
//		 	 bootloader_rx_data+1 -> sebebi,0 indekste size var çünkü
//		 	 gelen komutların paketleri şöyleydi;
//		 	 	 length to follow
//		 	 	 command code
//		 	 	 crc
//		 	 	 1.indekse size dan sonraki verileri yazacağım yani ondan sonra command code gelir onu tutacağım
//		 */
//
//		switch(bootloader_rx_data[1])
//		{
//			case BL_GET_VER:
//				bootloader_get_ver_cmd(bootloader_rx_data);
//				break;
//			default:
//				break;
//
//		}
//
//	}

		uint8_t bl_rx_length = 0;

		while(1)
		{
			memset(bootloader_rx_data, 0, BL_RX_DATA_LENGTH);

			HAL_UART_Receive(&huart1, bootloader_rx_data, 1, HAL_MAX_DELAY);

			bl_rx_length = bootloader_rx_data[0];

			HAL_UART_Receive(&huart1, &bootloader_rx_data[1], bl_rx_length, HAL_MAX_DELAY);

			switch(bootloader_rx_data[1])
			{
				case BL_GET_VER:
					bootloader_get_ver_cmd(bootloader_rx_data);
				break;
				case BL_GET_HELP:
					bootloader_get_help_cmd(bootloader_rx_data);
					break;
				case BL_GET_CID:
					bootloader_get_cid_cmd(bootloader_rx_data);
					break;
				case BL_GET_RDP_STATUS:
					bootloader_get_rdp_cmd(bootloader_rx_data);
					break;
				case BL_GO_TO_ADDR:
					bootloader_go_to_addr_cmd(bootloader_rx_data);
					break;
				case BL_FLASH_ERASE:
					bootloader_flash_erase_cmd(bootloader_rx_data);
					break;
				case BL_MEM_WRITE:
					bootloader_mem_write_cmd(bootloader_rx_data);
					break;
				case BL_EN_RW_PROTECT:
					bootloader_enable_read_write_protect_cmd(bootloader_rx_data);
					break;
				case BL_READ_SECTOR_P_STATUS:
					bootloader_read_sector_protection_status_cmd(bootloader_rx_data);
					break;
				case BL_DIS_R_W_PROTECT:
					bootloader_disable_read_write_protect_cmd(bootloader_rx_data);
					break;
			}
		}




}
static volatile uint32_t resetValue;
void bootloader_jump_to_user_application(void)
{
	void (*bootloader_application_reset_handler)(void); // reset fonksiyonunun adresini tutacak
	printMessage("BL DEBUG MSG:Bootloader atlama fonksiyonu cagirildi\n");
	uint32_t mspValue = *(volatile uint32_t *) FLASH_SECTOR2_BASE_ADDRESS ; // msp değerini taban adresten çekip içindeki değeri aldık
	printMessage("BL DEBUG MSG:MSP Degeri %#x\n",mspValue);
	__set_MSP(mspValue) ; // msp nin yeni değerini set ettik

	 resetValue = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS+4) ; // hatırla,msp den sonra reset handler in değeri geliyordu iki adres arasında 4 fark oluyordu
	printMessage("BL DEBUG MSG:Reset Degeri %#x\n",resetValue);

	bootloader_application_reset_handler = (void*) resetValue ; // yukarıdaki fonksiyona reset handlerin bulunduğu fonksiyonun adresini verdik,bu fonk onun reset h. ın adresini gösterir

	bootloader_application_reset_handler(); //reset handler çalışır
	printMessage("BL DEBUG MSG:oldu %#x\n",resetValue);


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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

 if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_SET)
 {
	 printMessage("BL DEBUG MSG: Butona Basildi,Bootloader'a Gidiliyor\n");

	 bootloader_uart_data_read();
 }
 else
 {
	 printMessage("BL DEBUG MSG: Butona Basilmadi!\n");

	 bootloader_jump_to_user_application();

	// goToApps(APPLICATION_ADDRESS_1);  // going to User App 1
 }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  printMessage("Project %d\n!",1);
//	  HAL_Delay(500);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
