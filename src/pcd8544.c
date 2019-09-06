#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_dma.h"
#include "misc.h"

//#include "../main.h"         // из модуля импортируется вызов delay_ms(unsigned int ms); - задержка в милисекундах
#include "pcd8544.h"
#include "font6x8.h"	  // шрифт

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define	delay_ms(x)	(vTaskDelay( x / portTICK_PERIOD_MS))


unsigned char lcd8544_buff[84*6]; // буфер дисплея

// прерывание по окончанию передачи буфера, использовать с настройкой NVIC и однократной передачей
void SPI2_DMA_TX_IRQHandler() {
  if (DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4)) {
    DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET) //;  - artp убрал ;
	{

	}
	 DMA_Cmd(DMA1_Stream4, DISABLE);
    LCD_CS1;			// СS=1 - закончили сеанс работы с дисплеем
  }
}


// отправка данных\команд на дисплей
void lcd8544_senddata(unsigned char data) {

#ifdef LCD_TO_SPI2
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2, data);
#else
	unsigned char i;
	for(i=0; i<8; i++) {
		if (data & 0x80) LCD_MOSI1;
		            else LCD_MOSI0;
		data = data<<1;
		LCD_SCK0;
		LCD_SCK1;
	}
#endif
}



// очистка дисплея
void lcd8544_refresh(void) {
#ifdef LCD_SEL_AUTO
	LCD_CS0;            // СS=0 - начали сеанс работы с дисплеем
#endif

	LCD_DC0;            // передача комманд
	lcd8544_senddata(0x40); // установка курсора в позицию Y=0; X=0
	lcd8544_senddata(0x80);

#ifdef LCD_TO_SPI2
	while(SPI2->SR & SPI_SR_BSY);
#endif

	LCD_DC1;            // передача данных

#ifndef LCD_SPI2_DMA // режим DMA не используется
	unsigned char y, x;

	for (y=0;y<6;y++) for (x=0;x<84;x++) lcd8544_senddata(lcd8544_buff[y*84+x]);
    #ifdef LCD_TO_SPI2
    	 while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
    #endif
    #ifdef LCD_SEL_AUTO
	   LCD_CS1;			// СS=1 - закончили сеанс работы с дисплеем
    #endif

#else // режим DMA
    	 DMA_Cmd(DMA1_Stream4, ENABLE);
#endif
}



// очистка дисплея
void lcd8544_clear(void) {
   unsigned char x,y;
   for (y=0;y<6;y++) for(x=0;x<84;x++) lcd8544_buff[y*84+x]=0;
}



// Инициализация
void lcd8544_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(LCD_AHB1_GPIO, ENABLE); // тактирование GPIOB

#ifdef LCD_TO_SPI2  // конфигурация аппаратного SPI2

	SPI_InitTypeDef  SPI_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);  // тактирование SPI2

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);   // SPI2_CLK  (Pin_13)
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);   // SPI2_MOSI (Pin_15)

	GPIO_InitStructure.GPIO_Pin 			= GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode 			= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed 			= GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType 			= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  			= GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SPI_DeInit(SPI2); // сбрасываем настройки SPI2 перед заданием конфигурации
	SPI_InitStructure.SPI_Mode        		= SPI_Mode_Master;
	SPI_InitStructure.SPI_Direction   		= SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_DataSize 			= SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL 				= SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA 				= SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS 				= SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI2_BaudRatePrescaler;
	SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;

	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);

#ifdef LCD_SPI2_DMA // использование DMA

	DMA_InitTypeDef DMA_InitStructure; // Переменная для структуры DMA

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); // Подаем такты на DMA1
	DMA_DeInit(DMA1_Stream0); // Очищаем конфигурацию в регистрах для Stream0
	DMA_StructInit(&DMA_InitStructure); // Инициализируем структуру значениями по умолчанию

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SPI2->DR); // Адрес регистра SPI2 куда писать данные
	DMA_InitStructure.DMA_Channel = DMA_Channel_0; // Канал DMA
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; // Направление копирования данных ОЗУ->Периферия
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable | DMA_PeripheralInc_Disable;  // В ОЗУ не 1 байт данных потому инкремент при работе
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&lcd8544_buff[0];
	DMA_InitStructure.DMA_BufferSize = 504;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;   // однократный режим
							  // DMA_Mode_Circular; // циклический режим

	DMA_Init(DMA1_Stream4, &DMA_InitStructure); // Инициализируем DMA

 	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE); // включаем SPI2 на работу через DMA

 	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE); // включаем прерывание по окончанию передачи данных

 	NVIC_InitTypeDef NVIC_InitStructure;
 	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 	NVIC_Init(&NVIC_InitStructure);


#endif

	GPIO_InitStructure.GPIO_Pin  			=(1<<LCD_CSE_PIN) | (1<<LCD_A0_PIN) | (1<<LCD_RST_PIN);

#else //  конфигурация для софт SPI

#ifndef	LCD_SOFT_RESET
    GPIO_InitStructure.GPIO_Pin  			=(1<<LCD_CSE_PIN) | (1<<LCD_A0_PIN) | (1<<LCD_SCK_PIN) | (1<<LCD_RST_PIN) | (1<<LCD_SDA_PIN);
#else
    GPIO_InitStructure.GPIO_Pin  			=(1<<LCD_CSE_PIN) | (1<<LCD_A0_PIN) | (1<<LCD_SCK_PIN) | (1<<LCD_SDA_PIN);
#endif

    LCD_SCK0;
#endif

    GPIO_InitStructure.GPIO_Mode 			=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType			=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 			=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed			=GPIO_Speed_100MHz;
	GPIO_Init(LCD_GPIO, &GPIO_InitStructure);


	LCD_CS0;            // CS=0  - начали сеанс работы с дисплеем
	LCD_DC0;			// A0=0  - переход в командный режим

	// сброс дисплея
	LCD_RST0;                   // RST=0
    delay_ms(1);				// пауза
	LCD_RST1;                   // RST=1
	delay_ms(LCD_RST_DLY);		// пауза

	// инициализация дисплея
	lcd8544_senddata(0x21);      // переход в расширенный режим
	lcd8544_senddata(0xC1);

	lcd8544_senddata(0x06);		// температурный коэффициент, от 4 до 7

	lcd8544_senddata(0x13);		// Bias 0b0001 0xxx - работает как контрастность

	lcd8544_senddata(0x20); 	// переход в обычный режим
	lcd8544_senddata(0b1100);	// 0b1100 - normal mode
		                        // 0b1101 - invert mode
		                        // 0b1001 - полностью засвеченный экран
		                        // 0b1000 - чистый экран


#ifdef LCD_TO_SPI2
	while(SPI2->SR & SPI_SR_BSY);
#endif

#ifdef LCD_SEL_AUTO
	LCD_CS1;
#endif
}


/*
Дисплей имеет размер 84х48 пикселей. Информация выводится вертикальными блоками высотой 8 пикселей,
значения которых определяются значениями бит в выводимом байте. Младший бит кодирует верхний пиксель.
Таким образом, графическая область дисплея представлена в виде шести строк, каждая по 8 пикселей в высоту.

Команды 01000yyy и 1xxxxxxx определяют координаты курсора - строку и позицию, в которых будет отображены следующие 8 бит данных.
После того как байт выведен, курсор смещается на соседнюю позицию.

unsigned char lcd8544_buff[84*6]; // буфер дисплея

buffer			0	1	2
y			x-	0	1	2	3	4	5
0	младший бит	.	.
1				.	.
2				.	.
3				.	.
4				.	.
5				.	.
6				.	.
7	старший бит	.	.
	-----------------------
	buffer		84	85	86
			x-	0	1	2	3	4	5
8	младший бит	.	.
9				.	.
10				.	.
11				.	.
12				.	.
13				.	.
14				.	.
15	старший бит	.	.

*/

// вывод пиксела
void lcd8544_putpix(unsigned char x, unsigned char y, unsigned char mode) {
	if ((x>84) || (y>47)) return;

	unsigned int adr=(y>>3)*84+x;
	unsigned char data=(1<<(y&0x07));

	if (mode) lcd8544_buff[adr]|=data;
   else lcd8544_buff[adr]&=~data;
}

void lcd8544_shift_left(unsigned char dx){
	unsigned int addr,y;
	for (y=0; y < 6; y++ ) {
		for (addr = y*84 + dx; addr < y*84 + 83; addr++) {
			lcd8544_buff[addr-dx] = lcd8544_buff[addr];
			lcd8544_buff[addr] = 0;
		}
	}
}

// процедура рисования линии
void lcd8544_line(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, unsigned char mode) {
signed char   dx, dy, sx, sy;
unsigned char  x,  y, mdx, mdy, l;


  dx=x2-x1; dy=y2-y1;

  if (dx>=0) { mdx=dx; sx=1; } else { mdx=x1-x2; sx=-1; }
  if (dy>=0) { mdy=dy; sy=1; } else { mdy=y1-y2; sy=-1; }

  x=x1; y=y1;

  if (mdx>=mdy) {
     l=mdx;
     while (l>0) {
         if (dy>0) { y=y1+mdy*(x-x1)/mdx; }
            else { y=y1-mdy*(x-x1)/mdx; }
         lcd8544_putpix(x,y,mode);
         x=x+sx;
         l--;
     }
  } else {
     l=mdy;
     while (l>0) {
        if (dy>0) { x=x1+((mdx*(y-y1))/mdy); }
          else { x=x1+((mdx*(y1-y))/mdy); }
        lcd8544_putpix(x,y,mode);
        y=y+sy;
        l--;
     }
  }
  lcd8544_putpix(x2, y2, mode);

}



// рисование прямоугольника (не заполненного)
void lcd8544_rect(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, unsigned char mode) {
	lcd8544_line(x1,y1, x2,y1, mode);
	lcd8544_line(x1,y2, x2,y2, mode);
	lcd8544_line(x1,y1, x1,y2, mode);
	lcd8544_line(x2,y1, x2,y2, mode);
}



// печать десятичного числа
void lcd8544_dec(unsigned int numb, unsigned char dcount, unsigned char x, unsigned char y, unsigned char mode) {
	unsigned int divid=10000;
	unsigned char i;

	for (i=5; i!=0; i--) {

		unsigned char res=numb/divid;

		if (i<=dcount) {
			lcd8544_putchar(x, y, res+'0', mode);
			x=x+6;
		}

		numb%=divid;
		divid/=10;
	}
}



// вывод символа на экран по координатам
void lcd8544_putchar(unsigned char px, unsigned char py, unsigned char ch, unsigned char mode) {
	const unsigned char *fontpointer;


	if (ch<127) {	// знакогенератор латиницы
		fontpointer=NewFontLAT; ch=ch-32; }
		else	{	// знакогенератор русских букв
		fontpointer=NewFontRUS; ch=ch-192;}

    unsigned char lcd_YP=7- (py & 0x07);    // битовая позиция символа в байте
    unsigned char lcd_YC=(py & 0xF8)>>3; 	// байтовая позиция символа на экране
    unsigned char x;
	for (x=0; x<6; x++) {

		unsigned char temp=*(fontpointer+ch*6+x);

		if (mode!=0) {
			temp=~temp;
			if (py>0) lcd8544_putpix(px, py-1, 1);	// если печать в режиме инверсии - сверху отчертим линию
		}

		temp&=0x7F;

		lcd8544_buff[lcd_YC*84+px]=lcd8544_buff[lcd_YC*84+px] | (temp<<(7-lcd_YP)); 	// печать верхней части символа

	    if (lcd_YP<7) lcd8544_buff[(lcd_YC+1)*84+px]=lcd8544_buff[(lcd_YC+1)*84+px] | (temp>>(lcd_YP+1)); 	// печать нижней части символа

		px++;
		if (px>83) return;
	}
}



// линия левее символа для печати в инверсе
void lcd8544_leftchline(unsigned char x, unsigned char y) {
	if (x>0) lcd8544_line(x-1, y-1, x-1, y+6, 1);
}


// вывод строки
void lcd8544_putstr(unsigned char x, unsigned char y, const unsigned char str[], unsigned char mode) {
	if (mode) lcd8544_leftchline(x, y);
	while (*str!=0) {
		lcd8544_putchar(x, y, *str, mode);
		x=x+6;
		str++;
	}
}
