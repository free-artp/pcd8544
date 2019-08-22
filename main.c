#define HSE_VALUE    ((uint32_t)8000000)

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#include "pcd8544.h"

#include "text.h"

// Переменные и структуры
volatile unsigned int delay_val;        // счетчик для delay_ms()
volatile unsigned int counter1000;      // счетчик интервалов (такт 10 мкс)

// прерывание SysTick - отсчет 10 мкс интервалов
void SysTick_Handler(void) {
	if (delay_val>0) delay_val--;
	counter1000++;
}


// реализация задержки на delay_value милисекунд
void delay_ms(unsigned int delay_value) {
	delay_val=delay_value*10;
	while (delay_val!=0);
}

// реализация задержки на delay_value в * 10 мкс
void delay_10us(unsigned int delay_value) {
	delay_val=delay_value;
	while (delay_val!=0);
}

// const unsigned char str1[]="NOKIA 5110";
// const unsigned char str2[]="СТАРТОВАЛ";
// const unsigned char str3[]="РУССКИЕ БУКВЫ!";

int main(void)
{
	SystemInit();

    SysTick_Config(SystemCoreClock/10000); // запуск systick (10 мкс)

    lcd8544_init(); // запуск модуля LCD


    unsigned char y=3;
    signed char sy=1;
    while(1)
    {

        lcd8544_clear();

        lcd8544_rect(0,0,83,47,1);

        lcd8544_putstr(13,y,str1,1);
        lcd8544_putstr(15,y+10,str2,0);

        lcd8544_putstr(2,y+21,str3,1);

        lcd8544_refresh();

        delay_ms(200);
        y=y+sy;
        if ((y==3) || (y==18)) sy=-sy;
    }
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