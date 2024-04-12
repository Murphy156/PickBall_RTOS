#include "main.h"
#include "cmsis_os.h"
#include "core_delay.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "retarget.h"

/*** FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"


/***
*************************************************************************
*                             函数声明
*************************************************************************
*/
static void AppTaskCreate(void); /*** 用于创建任务 */

static void LED_Task(void* pvParameters);         /*** LED_Task任务实现*/
static void KEY_Task(void* pvParameters);         /*** KEY_Task任务实现*/

static void BSP_Init(void); /*** 用于初始化板载相关资源*/


void SystemClock_Config(void);
//void MX_FREERTOS_Init(void);

/*****************************************************************
  * @brief  主函数
  * @param  无
  * @retval 无
  * @note   第一步：开发板硬件初始化
            第二步：创建APP应用任务
            第三步：启动FreeRTOS，开始多任务调度
  ****************************************************************/
int main(void)
{
    BaseType_t xReturn = pdPASS;

    /** 开发板硬件初始化 */
    BSP_Init();

    /** 创建AppTaskCreate任务 */
    xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* 任务入口函数 */
                          (const char*    )"AppTaskCreate",/* 任务名字 */
                          (uint16_t       )512,  /* 任务栈大小 */
                          (void*          )NULL,/* 任务入口函数参数 */
                          (UBaseType_t    )1, /* 任务的优先级 */
                          (TaskHandle_t*  )&AppTaskCreate_Handle);

    /*** 启动任务调度 */
    if (pdPASS == xReturn)
        vTaskStartScheduler(); /*** 启动任务，开启调度 */
    else
        return -1;

    while (1);    /*** 正常不会执行到这里 */

}

/***********************************************************************
  * @ 函数名  ： AppTaskCreate
  * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
  * @ 参数    ： 无
  * @ 返回值  ： 无
  **********************************************************************/
static void AppTaskCreate(void)
{
    BaseType_t xReturn = pdPASS; /*** 定义一个创建信息返回值，默认为pdPASS */

    taskENTER_CRITICAL(); /*** 进入临界区 */

    /** 数字越大，优先级越大*/
    /*** 创建LED_Task任务 */
    xReturn = xTaskCreate((TaskFunction_t)  LED_Task  ,/** 任务入口函数 */
                          (const char*   ) "LED_Task" ,/** 任务名字*/
                          (uint16_t      )  512       ,/** 任务栈大小*/
                          (void*         )  NULL      ,/** 任务入口函数参数*/
                          (UBaseType_t   )  2         ,/** 任务的优先级 */
                          (TaskHandle_t* )  &LED_Task_Handle);/** 任务控制块指针*/
    if(pdPASS == xReturn)
    {

    }

    /*** 创建KEY_Task任务 */
    xReturn =  xTaskCreate((TaskFunction_t ) KEY_Task   ,/** 任务入口函数 */
                           (const char*    ) "KEY_Task" ,/** 任务名字*/
                           (uint16_t       ) 512        ,/** 任务栈大小*/
                           (void*          ) NULL       ,/** 任务入口函数参数*/
                           (UBaseType_t    ) 1          ,/** 任务的优先级*/
                           (TaskHandle_t*  ) &KEY_Task_Handle);   /** 任务控制块指针*/
    if(pdPASS == xReturn)
    {

    }


    vTaskDelete(AppTaskCreate_Handle);

    taskEXIT_CRITICAL();
}

/**********************************************************************
  * @ 函数名  ： LED_Task
  * @ 功能说明： LED_Task任务主体
  * @ 参数    ：
  * @ 返回值  ： 无
  ********************************************************************/
static void LED_Task(void* parameter)
{
    while (1)
    {
        LED1_TOGGLE
        vTaskDelay(500);   /* 延时500个tick */

        LED2_TOGGLE
        vTaskDelay(500);   /* 延时500个tick */
    }
}

/**********************************************************************
  * @ 函数名  ： KEY_Task
  * @ 功能说明： KEY_Task任务主体
  * @ 参数    ：
  * @ 返回值  ： 无
  ********************************************************************/
static void KEY_Task(void* parameter)
{
    while (1)
    {
        if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON )
        {
            LED3_TOGGLE
        }
        if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON )
        {
            LED4_TOGGLE
        }
        if( Key_Scan(KEY3_GPIO_PORT,KEY3_PIN) == KEY_ON )
        {
            LED5_TOGGLE
        }
        if( Key_Scan(KEY4_GPIO_PORT,KEY4_PIN) == KEY_ON )
        {
            LED3_TOGGLE
        }
        if( Key_Scan(KEY5_GPIO_PORT,KEY5_PIN) == KEY_ON )
        {
            LED4_TOGGLE
        }
        vTaskDelay(100);/** 延时20个tick */
    }
}

/***********************************************************************
  * @ 函数名  ： BSP_Init
  * @ 功能说明： 板级外设初始化，所有板子上的初始化均可放在这个函数里面
  * @ 参数    ：
  * @ 返回值  ： 无
  *********************************************************************/
static void BSP_Init(void)
{
    HAL_Init();

    SystemClock_Config();

    /* 初始化SysTick */
    HAL_SYSTICK_Config( HAL_RCC_GetSysClockFreq() / configTICK_RATE_HZ );

    /* 配置优先级分组为4 */
    HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );

    /** LED 灯初始化 */
    LED_GPIO_Config();

    /** 初始化按键GPIO */
    Key_GPIO_Config();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
static void MX_NVIC_Init(void)
{
//    HAL_NVIC_SetPriority(DEBUG_USART_IRQ, 6, 0);
//    HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ );
//
//    HAL_NVIC_SetPriority(ADC_DMA_IRQ, 1, 0);
//    HAL_NVIC_EnableIRQ(ADC_DMA_IRQ);
//
//    HAL_NVIC_SetPriority(BASIC_TIM_IRQn, 3, 0);
//    HAL_NVIC_EnableIRQ(BASIC_TIM_IRQn);
//
//    HAL_NVIC_SetPriority(BASIC_TIM5_IRQn, 2, 0);
//    HAL_NVIC_EnableIRQ(BASIC_TIM5_IRQn);
//
//    HAL_NVIC_SetPriority(BLE_UARTx_IRQ, 0, 0);
//    HAL_NVIC_EnableIRQ(BLE_UARTx_IRQ);
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
