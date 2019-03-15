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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern USBD_HandleTypeDef hUsbDeviceFS;
#define BUTTON_REPORT_SIZE 8
#define INTERVAL 20

enum holdrelease {HOLD=0, RELEASE=1};
enum specbutton {LCTRL=0x00, LSHIFT=0x02, LALT=0x04, LMETA=0x08, RCTRL=0x10, RSHIFT=0x20, RALT=0x40, RMETA=0x80};
enum buttons1 {ENTER=0x28,ESC,BACKSPACE,TAB,SPACE,MINUS,EQUAL,LEFTBRACE,RIGHTBRACE,BACKSLASH,HASHTILDE,SEMICOLON,
APOSTROPHE,GRAVE,COMMA,DOT,SLASH,CAPSLOCK,F1,F2,F3,F4,F5,F6,F7,F8,F9,F10,F11,F12,SYSRQ,SCROLLLOCK,PAUSE,INSERT,
HOME,PAGEUP,DELETE,END,PAGEDOWN,RIGHT,LEFT,DOWN,UP,NUMLOCK};

uint8_t button_report[BUTTON_REPORT_SIZE] = {0};


#define CLICK_REPORT_SIZE 5
uint8_t click_report[CLICK_REPORT_SIZE] = {0};


void HID_CursorMove(int8_t x, int8_t y)
{
  // [1] - x -right, [2] - y - down
  click_report[1] = x;
  click_report[2] = y;
  USBD_HID_SendReport(&hUsbDeviceFS, click_report, CLICK_REPORT_SIZE);
}

uint8_t HID_GetScanCode(char button)
{
  uint8_t code = 0x00;
  
  if (button >= 0x61 && button <= 0x7A)
  {
    code = button - 93;
  }
  else
  if (button >= 0x31 && button <= 0x39)
  {
    code = button - 19;
  }
  else
  if (button == 0x30)
    code = 0x27;
  else 
  if (button == ' ')
    code = 0x2C;
  return code;
}

void HID_SendSpecButtonData(uint8_t scancode, uint8_t release)
{
  button_report[0] = scancode;
  USBD_HID_SendReport(&hUsbDeviceFS, button_report, BUTTON_REPORT_SIZE);
  HAL_Delay(INTERVAL); 
  button_report[0] = 0;
  if (release)
  {
    USBD_HID_SendReport(&hUsbDeviceFS, button_report, BUTTON_REPORT_SIZE);
    HAL_Delay(INTERVAL);
  }
}

void HID_SendButtonData(uint8_t scancode, uint8_t release)
{
  button_report[2] = scancode;
  USBD_HID_SendReport(&hUsbDeviceFS, button_report, BUTTON_REPORT_SIZE);
  HAL_Delay(INTERVAL); 
  button_report[2] = 0;
  if (release)
  {
    USBD_HID_SendReport(&hUsbDeviceFS, button_report, BUTTON_REPORT_SIZE);
    HAL_Delay(INTERVAL);
  }
}

void HID_SendCombo(uint8_t scancodespec, uint8_t button)
{
  button_report[0] = scancodespec;
  button_report[2] = HID_GetScanCode(button);
  USBD_HID_SendReport(&hUsbDeviceFS, button_report, BUTTON_REPORT_SIZE);
  HAL_Delay(INTERVAL); 
  button_report[0] = 0;
  button_report[2] = 0;
  USBD_HID_SendReport(&hUsbDeviceFS, button_report, BUTTON_REPORT_SIZE);
  HAL_Delay(INTERVAL);
}

void HID_WriteString(uint8_t * str)
{
  uint8_t scancode = 0;
  while (*str)
  {
    scancode = HID_GetScanCode(str[0]);
    HID_SendButtonData(scancode, RELEASE);
    str++;
  }
}

#define STEPSIZE 10
#define DELAYSTEPS 15
#define ITERATIONS 5

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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  /* USER CODE END 2 */
 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef USE_MOUSE
    
    for (int i = 0; i < ITERATIONS; i++)
    {
      HID_CursorMove(STEPSIZE,STEPSIZE);
      HAL_Delay(DELAYSTEPS);
    }
    for (int i = 0; i < ITERATIONS; i++)
    {
      HID_CursorMove(-STEPSIZE,STEPSIZE);
      HAL_Delay(DELAYSTEPS);
    }
    for (int i = 0; i < ITERATIONS; i++)
    {
      HID_CursorMove(-STEPSIZE,-STEPSIZE);
      HAL_Delay(DELAYSTEPS);
    }
    for (int i = 0; i < ITERATIONS; i++)
    {
      HID_CursorMove(STEPSIZE,-STEPSIZE);
      HAL_Delay(DELAYSTEPS);
    }
#else
    HID_SendCombo(LMETA, 'r');
    HID_WriteString("notepad");
    HID_SendButtonData(ENTER,RELEASE);
    HAL_Delay(100);
    HID_WriteString("nu privet   kak dela  tebya vzlomali asdasdasdasdasds");
    HAL_Delay(100);
#endif
    
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
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
