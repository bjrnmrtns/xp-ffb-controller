/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "tusb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define HID_BINTERVAL 0x01 // 1 = 1000hz, 2 = 500hz, 3 = 333hz 4 = 250hz, 5 = 200hz 6 = 166hz, 7 = 125hz...
#define USB_HID_FFB_REPORT_DESC_SIZE 1229//1378
#define USB_STRING_DESC_BUF_SIZE 32
#define USBD_VID     0x1209
#define USBD_PID     0xFFB0
const tusb_desc_device_t usb_devdesc_ffboard_composite =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USBD_VID,
    .idProduct          = USBD_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

const uint16_t usb_dev_desc_langId = 0x0409;
const uint8_t	usb_dev_desc_manufacturer[] = "Open_FFBoard";
const uint8_t usb_dev_desc_product[] = "FFBoard";
const uint8_t* usb_dev_desc_interfaces[] = { "FFBoard CDC", "FFBoard HID" };

uint16_t _desc_str[USB_STRING_DESC_BUF_SIZE];

/*const uint8_t usb_cdc_conf[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, 2, 0, (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN), TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

  // 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(0, 4, 0x82, 8, 0x01, 0x81, 64),
};
*/
// Composite CDC and HID
const uint8_t usb_cdc_hid_conf[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, 3, 0, (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_HID_INOUT_DESC_LEN), TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

  // 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(0, 4, 0x82, 8, 0x01, 0x81, 64),

  // HID Descriptor. EP 83 and 2
  TUD_HID_INOUT_DESCRIPTOR(2, 5, HID_ITF_PROTOCOL_NONE, USB_HID_FFB_REPORT_DESC_SIZE, 0x83, 0x02, 64, HID_BINTERVAL),
};


uint8_t const * tud_hid_descriptor_report_cb(uint8_t itf){
	char buffer[] = "tud_hid_descriptoor_report_cb\r\n";
	HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
}

void tud_cdc_rx_cb(uint8_t itf){
	char buffer[] = "tud_cdc_rx_cb\r\n";
	HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
}

void tud_cdc_tx_complete_cb(uint8_t itf){
	char buffer[] = "tud_cdc_tx_complete_cb\r\n";
	HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize){
	char buf[] = "tud_hid_set_report_cb\r\n";
	HAL_UART_Transmit(&huart1, &buf[0], strlen(buf), 10);
}

uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type,uint8_t* buffer, uint16_t reqlen){
	char buf[] = "tud_hid_get_report_cb\r\n";
	HAL_UART_Transmit(&huart1, &buf[0], strlen(buf), 10);
}

void tud_hid_report_complete_cb(uint8_t itf, uint8_t const* report, uint8_t len){
	char buffer[] = "tud_hid_report_complete_cb\r\n";
	HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
}

uint8_t const * tud_descriptor_device_cb(void)
{
	char buffer[] = "tud_descriptor_device\r\n";
	HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
	return (uint8_t const *)&usb_devdesc_ffboard_composite;
}
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
	char buffer[] = "tud_descriptor_configuration_cb\r\n";
	HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
	return usb_cdc_hid_conf;
}

void ascii_to_utf16(uint16_t* dest, const uint8_t* src) {
    int chr_count = strlen(src);
    if ( chr_count > 31 ) chr_count = 31;

    // Convert ASCII string into UTF-16
    for(uint8_t i=0; i<chr_count; i++)
    {
      dest[i+1] = src[i];
    }
    dest[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	char buffer[] = "tud_descriptor_string_cb\r\n";
	HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
	uint16_t chr_count = 0;
	if (index == 0) // Language
	{
		_desc_str[1] = usb_dev_desc_langId;
		chr_count = 1;
	}else{
		if(index == usb_devdesc_ffboard_composite.iSerialNumber){
			const uint8_t buf[4];
			const int size = snprintf(&buf, 4, "%d%d%d", HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
			ascii_to_utf16(_desc_str, buf);
		}else if(index == usb_devdesc_ffboard_composite.iProduct){
			ascii_to_utf16(_desc_str, usb_dev_desc_product);
		}else if(index == usb_devdesc_ffboard_composite.iManufacturer){
			ascii_to_utf16(_desc_str, usb_dev_desc_manufacturer);
		}else if(index > 3 && (index - 4 < 3)){
			ascii_to_utf16(_desc_str, usb_dev_desc_interfaces[index - 4]);
		}else{
			return NULL;
		}
	}
	return _desc_str;
}
void tud_mount_cb(void)
{
	// start all stuff because usb is started
	char buffer[] = "tud_mount_cb\r\n";
	HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);

}
void tud_umount_cb(void)
{
	// stop all stuff because usb is essentially stopped
	char buffer[] = "tud_umount_cb\r\n";
	HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
}
void tud_suspend_cb(bool remote_wakeup_en)
{
	// stop all stuff because usb is essentially stopped
	char buffer[] = "tud_suspend_cb\r\n";
	HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);

}
void tud_resume_cb(void)
{
	// start all stuff because usb is started
	char buffer[] = "tud_resume_cb\r\n";
	HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
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
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  if(!tusb_init()) {
	  Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char buffer[] = "before start\r\n";
  HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
  while (1)
  {
	tud_task();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

