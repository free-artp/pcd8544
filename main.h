#ifndef __MAIN_H
#define __MAIN_H

void delay_ms(unsigned int delay_value);

// #define ADCx                     ADC3
// #define ADC_CHANNEL              ADC_Channel_8
// #define ADCx_CLK                 RCC_APB2Periph_ADC3
// #define ADCx_CHANNEL_GPIO_CLK    RCC_AHB1Periph_GPIOF
// #define GPIO_PIN                 GPIO_Pin_10
// #define GPIO_PORT                GPIOF
// #define DMA_CHANNELx             DMA_Channel_2
// #define DMA_STREAMx              DMA2_Stream0
// #define ADCx_DR_ADDRESS          ((uint32_t)0x4001224C)


#define ADCx                     ADC3
#define ADC_CHANNEL              ADC_Channel_1
#define ADCx_CLK                 RCC_APB2Periph_ADC3
#define ADCx_CHANNEL_GPIO_CLK    RCC_AHB1Periph_GPIOA
#define GPIO_PIN                 GPIO_Pin_1
#define GPIO_PORT                GPIOA
#define DMA_CHANNELx             DMA_Channel_2
#define DMA_STREAMx              DMA2_Stream0
#define ADCx_DR_ADDRESS          ((uint32_t)0x4001224C)



#endif /* __MAIN_H */
