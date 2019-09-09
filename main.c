#define HSE_VALUE    ((uint32_t)8000000)

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"

/*******************************************************************/

#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "croutine.h"
#include "task.h"
#include "queue.h"
 
/*******************************************************************/


#include <math.h>

#include "main.h"
#include "pcd8544.h"

#include "text.h"

__IO uint16_t uhADCxConvertedValue = 0;
__IO uint32_t uwADCxConvertedVoltage = 0;
static void ADC_Config(void);

void vDisplayTask(void *par){

  const TickType_t xDelay = 200 / portTICK_PERIOD_MS;  /* Block for 200ms. */
  int y;

  while(1){
      lcd8544_shift_left(1);
      y = (uhADCxConvertedValue*48) >> 12;
      lcd8544_putpix(80,(unsigned char) y,1);
//      lcd8544_dec(uhADCxConvertedValue, 4, 0, 24, 1);
      lcd8544_refresh();
      vTaskDelay( xDelay );
  }
  vTaskDelete(NULL);
}

int main(void)
{
	SystemInit();
  SysTick_Config(SystemCoreClock/10000); // запуск systick (10 мкс)

  lcd8544_init(); // запуск модуля LCD
  lcd8544_clear();

  ADC_Config();  /* ADC configuration */
  ADC_SoftwareStartConv(ADCx);  /* Start ADC Software Conversion */ 

  xTaskCreate(vDisplayTask,"DisplayTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
  vTaskStartScheduler();

}

/**
  * @brief  ADC3 channel07 with DMA configuration
  * @note   This function Configure the ADC peripheral  
            1) Enable peripheral clocks
            2) DMA2_Stream0 channel2 configuration
            3) Configure ADC Channel1 pin as analog input
            4) Configure ADC3 Channel1 
  * @param  None
  * @retval None
  */
static void ADC_Config(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADCx, DMA and GPIO clocks ****************************************/ 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_AHB1PeriphClockCmd(ADCx_CHANNEL_GPIO_CLK, ENABLE);  
  RCC_APB2PeriphClockCmd(ADCx_CLK, ENABLE);
  


  /* DMA2 Stream0 channel2 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_CHANNELx;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADCx_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&uhADCxConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA_STREAMx, &DMA_InitStructure);
  DMA_Cmd(DMA_STREAMx, ENABLE);

  /* Configure ADC3 Channel7 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIO_PORT, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADCx, &ADC_InitStructure);

  /* ADC3 regular channel7 configuration **************************************/
  ADC_RegularChannelConfig(ADCx, ADC_CHANNEL, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADCx, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADCx, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADCx, ENABLE);
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
 
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*******************************************************************/
void vApplicationIdleHook( void )
{
}
 
 
 
/*******************************************************************/
void vApplicationMallocFailedHook( void )
{
    for( ;; );
}
 
 
 
/*******************************************************************/
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;
 
    for( ;; );
}
 
 
 
/*******************************************************************/
void vApplicationTickHook( void )
{
}
 
 
 
/*******************************************************************/