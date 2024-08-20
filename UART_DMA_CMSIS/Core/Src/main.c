/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PCLK1_FREQUENCY 64000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define RXBUF_SIZE 8
char rxBuffer[RXBUF_SIZE];

#define BUFFER_SIZE 5
char txBuffer[BUFFER_SIZE] = {'1', '2', '3', '4', '\0'};
char *p = txBuffer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void configure_uart_CMSIS(uint32_t baud_rate)
{
	USART1->CR1 &= ~USART_CR1_UE;  // Tắt UART để cấu hình

	// Tính toán giá trị BRR
	uint32_t brr_value = (PCLK1_FREQUENCY + (baud_rate / 2)) / baud_rate; // Làm tròn lên
	USART1->BRR = brr_value;  // Cấu hình Baud Rate

	// Cấu hình các thông số khác (Data Width, Stop Bits, Parity, HW Flow Control)
	USART1->CR1 &= ~(USART_CR1_M | USART_CR1_PCE);  // 8-bit data, no parity
	USART1->CR2 &= ~USART_CR2_STOP;  // 1 stop bit

	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;  // Enable transmitter and receiver
	USART1->CR1 |= USART_CR1_UE;  // Kích hoạt UART
}

void configure_dma_CMSIS(void)
{
	// Bước 1: Bật clock cho DMA1
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// Bước 2: Tắt DMA Channel 4
	DMA1_Channel4->CCR &= ~DMA_CCR_EN;

	// Bước 3: Cấu hình DMA cho UART1 TX
	// Hướng truy�?n dữ liệu: Memory to Peripheral
	DMA1_Channel4->CCR |= DMA_CCR_DIR; // Memory-to-Peripheral

	// �?ặt địa chỉ bộ nhớ
	DMA1_Channel4->CMAR = (uint32_t)txBuffer;

	// �?ặt địa chỉ ngoại vi (UART1 DR)
	DMA1_Channel4->CPAR = (uint32_t)&USART1->DR;

	// �?ặt kích thước dữ liệu (Buffer size)
	DMA1_Channel4->CNDTR = BUFFER_SIZE;

	// Cấu hình mức ưu tiên kênh DMA
	DMA1_Channel4->CCR |= DMA_CCR_PL; // High priority

	// Cấu hình chế độ hoạt động DMA (Normal Mode)
	DMA1_Channel4->CCR &= ~DMA_CCR_CIRC; // Normal mode

	// Cấu hình gia tăng địa chỉ bộ nhớ
	DMA1_Channel4->CCR |= DMA_CCR_MINC; // Memory increment mode

	// Cấu hình kích thước dữ liệu (Byte)
	DMA1_Channel4->CCR &= ~DMA_CCR_MSIZE; // Memory size = 8-bit
	DMA1_Channel4->CCR &= ~DMA_CCR_PSIZE; // Peripheral size = 8-bit

	// Cấu hình địa chỉ ngoại vi không gia tăng
	DMA1_Channel4->CCR &= ~DMA_CCR_PINC; // Peripheral no increment


	// Bước 2: Tắt DMA Channel 5
	DMA1_Channel5->CCR &= ~DMA_CCR_EN;

	// Bước 3: Cấu hình DMA cho UART1 RX
	// Hướng truyền dữ liệu: Peripheral to Memory
	DMA1_Channel5->CCR &= ~DMA_CCR_DIR; // Peripheral-to-Memory

	// Đặt địa chỉ bộ nhớ
	DMA1_Channel5->CMAR = (uint32_t)rxBuffer;

	// Đặt địa chỉ ngoại vi (UART1 DR)
	DMA1_Channel5->CPAR = (uint32_t)&USART1->DR;

	// Đặt kích thước dữ liệu (Buffer size)
	DMA1_Channel5->CNDTR = RXBUF_SIZE;

	// Cấu hình mức ưu tiên kênh DMA
	DMA1_Channel5->CCR |= DMA_CCR_PL; // High priority

	// Cấu hình chế độ hoạt động DMA (Normal Mode)
	DMA1_Channel5->CCR &= ~DMA_CCR_CIRC; // Normal mode

	// Cấu hình gia tăng địa chỉ bộ nhớ
	DMA1_Channel5->CCR |= DMA_CCR_MINC; // Memory increment mode

	// Cấu hình kích thước dữ liệu (Byte)
	DMA1_Channel5->CCR &= ~DMA_CCR_MSIZE; // Memory size = 8-bit
	DMA1_Channel5->CCR &= ~DMA_CCR_PSIZE; // Peripheral size = 8-bit

	// Cấu hình địa chỉ ngoại vi không gia tăng
	DMA1_Channel5->CCR &= ~DMA_CCR_PINC; // Peripheral no increment
}

// Cấu hình NVIC cho DMA1 Channel 5
void configure_nvic(void) {
    // Bước 1: Kích hoạt ngắt DMA1 Channel 5 trong NVIC
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    // Bước 2: Cấu hình mức ưu tiên cho ngắt DMA1 Channel 5 (nếu cần)
    NVIC_SetPriority(DMA1_Channel5_IRQn, 1); // Thiết lập mức ưu tiên
}

uint8_t received_count = 0;

// Xử lý ngắt DMA1 Channel 5
void DMA1_Channel5_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF5) {
        // Xử lý ngắt truyền dữ liệu hoàn tất (TC flag)


		while (received_count < RXBUF_SIZE)
		{
			// Chờ cho đến khi có dữ liệu đến (RXNE flag)
			while (!(USART1->SR & USART_SR_RXNE))
			{
				// Có thể thêm timeout ở đây nếu cần
			}

			// Đọc dữ liệu từ thanh ghi dữ liệu USART và lưu vào buffer
			rxBuffer[received_count] = USART1->DR;

			// Tăng chỉ số số byte đã nhận
			received_count++;
		}
        // Xóa cờ ngắt truyền dữ liệu hoàn tất
        DMA1->IFCR |= DMA_IFCR_CTCIF5;

        // Thực hiện các xử lý cần thiết sau khi nhận dữ liệu xong
    }
}

void start_transmission(void)
{
    // Bước 1: Bật DMA yêu cầu cho truy�?n dữ liệu UART
    USART1->CR3 |= USART_CR3_DMAT;

    // Bước 2: Kích hoạt DMA Channel để bắt đầu truy�?n dữ liệu
    DMA1_Channel4->CCR |= DMA_CCR_EN;
}

void start_receive(void)
{
	// Bước 3: Bật DMA yêu cầu cho nhận dữ liệu UART
	USART1->CR3 |= USART_CR3_DMAR;

	// Kích hoạt DMA Channel
	DMA1_Channel5->CCR |= DMA_CCR_EN;
}

void transmit_data(void)
{
    while (*p)
    {
        // Ch�? cho thanh ghi dữ liệu sẵn sàng để truy�?n (TXE flag)
        while (!(USART1->SR & USART_SR_TXE))
        {
            // Có thể thêm timeout ở đây nếu cần
        }

        // Truy�?n dữ liệu
        USART1->DR = *p++;

        // Ch�? cho việc truy�?n dữ liệu hoàn tất (TC flag)
        while (!(USART1->SR & USART_SR_TC))
        {
            // Có thể thêm timeout ở đây nếu cần
        }
    }
}

//void receive_data(void)
//{
//    uint8_t received_count = 0;
//
//    while (received_count < RXBUF_SIZE)
//    {
//        // Chờ cho đến khi có dữ liệu đến (RXNE flag)
//        while (!(USART1->SR & USART_SR_RXNE))
//        {
//        	// Có thể thêm timeout ở đây nếu cần
//        }
//
//        // Đọc dữ liệu từ thanh ghi dữ liệu USART và lưu vào buffer
//        rxBuffer[received_count] = USART1->DR;
//
//        // Tăng chỉ số số byte đã nhận
//        received_count++;
//    }
//}
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

  /* USER CODE BEGIN SysInit */
//  MX_GPIO_Init();
  configure_uart_CMSIS(115200);
  configure_dma_CMSIS();
  configure_nvic();

  // Bắt đầu truyền dữ liệu
  start_transmission();
  start_receive();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

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
