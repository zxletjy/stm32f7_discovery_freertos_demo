#include "stm32f7xx_hal.h"
#include "stm32746g_discovery_lcd.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LCD_FRAME_BUFFER          SDRAM_DEVICE_ADDR
/* Private macro -------------------------------------------------------------*/
#define TASK_LED_STACK_SIZE configMINIMAL_STACK_SIZE			//任务堆栈大小
#define TASK_LCD_STACK_SIZE configMINIMAL_STACK_SIZE			//任务堆栈大小
#define TASK_INIT_STACK_SIZE configMINIMAL_STACK_SIZE			//任务堆栈大小
#define TASK_COMM_STACK_SIZE configMINIMAL_STACK_SIZE			//任务堆栈大小

#define USARTx                           USART6
#define USARTx_CLK_ENABLE()              __USART6_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART6_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART6_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_6
#define USARTx_TX_GPIO_PORT              GPIOC
#define USARTx_TX_AF                     GPIO_AF8_USART6
#define USARTx_RX_PIN                    GPIO_PIN_7
#define USARTx_RX_GPIO_PORT              GPIOC
#define USARTx_RX_AF                     GPIO_AF8_USART6

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_STREAM              DMA2_Stream6
#define USARTx_RX_DMA_STREAM              DMA2_Stream1
#define USARTx_TX_DMA_CHANNEL             DMA_CHANNEL_5
#define USARTx_RX_DMA_CHANNEL             DMA_CHANNEL_5


/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA2_Stream6_IRQn
#define USARTx_DMA_RX_IRQn                DMA2_Stream1_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA2_Stream6_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA2_Stream1_IRQHandler

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART6_IRQn
#define USARTx_IRQHandler                USART6_IRQHandler
UART_HandleTypeDef UartHandle;
enum
{
	TASK_LED_PRI = 1,
	TASK_LCD_PRI,
	TASK_COMM_PRI,
	TASK_INIT_PRI
};
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
void Task_LED(void *p);
void Task_LCD(void *p);
void Task_COMM(void *p);
void Task_INIT(void *p);
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
	
  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();  
  
  /* Configure the system clock to 216 MHz */
  SystemClock_Config();

	xTaskCreate(Task_INIT, "INIT", TASK_INIT_STACK_SIZE, (void*)0, TASK_INIT_PRI, NULL);
  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  for(;;);
}
void Task_INIT(void *p)
{
	xTaskCreate(Task_LCD, "LCD", TASK_LCD_STACK_SIZE, (void*)0, TASK_LCD_PRI, NULL);
	xTaskCreate(Task_LED, "LED", TASK_LED_STACK_SIZE, (void*)0, TASK_LED_PRI, NULL);
	xTaskCreate(Task_COMM, "COMM", TASK_COMM_STACK_SIZE, (void*)0, TASK_COMM_PRI, NULL);
	vTaskDelete(NULL);
}
void Task_COMM(void *p)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;
	/* Buffer used for transmission */
	uint8_t TxBuffer[] = "what the fuck";
	
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();


  /* Enable USARTx clock */
  USARTx_CLK_ENABLE(); 
  
	/* Enable DMA clock */
  DMAx_CLK_ENABLE();
	
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;

  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Alternate = USARTx_RX_AF;

  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
	
	/*##-3- Configure the DMA ##################################################*/
  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = USARTx_TX_DMA_STREAM;
  hdma_tx.Init.Channel             = USARTx_TX_DMA_CHANNEL;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;

  HAL_DMA_Init(&hdma_tx);

  /* Associate the initialized DMA handle to the UART handle */
  __HAL_LINKDMA(&UartHandle, hdmatx, hdma_tx);

  /* Configure the DMA handler for reception process */
  hdma_rx.Instance                 = USARTx_RX_DMA_STREAM;
  hdma_rx.Init.Channel             = USARTx_RX_DMA_CHANNEL;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode                = DMA_NORMAL;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;

  HAL_DMA_Init(&hdma_rx);

  /* Associate the initialized DMA handle to the the UART handle */
  __HAL_LINKDMA(&UartHandle, hdmarx, hdma_rx);
    
  /*##-4- Configure the NVIC for DMA #########################################*/
  /* NVIC configuration for DMA transfer complete interrupt (USART6_TX) */
  HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);
    
  /* NVIC configuration for DMA transfer complete interrupt (USART6_RX) */
  HAL_NVIC_SetPriority(USARTx_DMA_RX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USARTx_DMA_RX_IRQn);
  
  /* NVIC for USART, to catch the TX complete */
  HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);
	
	UartHandle.Instance        = USARTx;
	UartHandle.Init.BaudRate = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_UART_DeInit(&UartHandle);
	HAL_UART_Init(&UartHandle);
	
	for(;;)
	{
		HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)TxBuffer, sizeof(TxBuffer) / sizeof(*(TxBuffer)));
		//HAL_UART_Transmit(&UartHandle, (uint8_t*)TxBuffer, sizeof(TxBuffer) / sizeof(*(TxBuffer)), 5000);
		vTaskDelay(1000);
	}
}
void Task_LCD(void *p)
{
	BSP_LCD_Init();
	/* Initialize the LCD Layers */
  BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FRAME_BUFFER);
	/* Set LCD Foreground Layer  */
  BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);

  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

  /* Clear the LCD */
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

  /* Display LCD messages */
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"STM32F746G FreeRTOS Demo", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"What the fuck !!!", CENTER_MODE);
	vTaskDelete(NULL);
}
/**
  * @brief  TASK_LED
  * @param  argument: Not used
  * @retval None
  */
void Task_LED(void *p)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	__HAL_RCC_GPIOI_CLK_ENABLE();
	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.Pin = GPIO_PIN_1;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOI, &GPIO_InitStructure);
  for(;;)
  {
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_SET);
		vTaskDelay(100);
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);
		vTaskDelay(100);
  }
}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);

}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
	while(1)
	{
		
	}
}
void vApplicationIdleHook (void)
{

}
/**
  * @brief  This function handles DMA interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA  
  *         used for USART data transmission     
  */
void USARTx_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmarx);
}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA  
  *         used for USART data reception    
  */
void USARTx_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmatx);
}


/**
  * @brief  This function handles UART interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA  
  *         used for USART data transmission     
  */
void USARTx_IRQHandler(void)
{
  HAL_UART_IRQHandler(&UartHandle);
}

