#ifndef _H_PERIPHERAL_
#define _H_PERIPHERAL_

#include "stm32f407xx.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "FreeRTOSConfig.h"

/*
 * Clock configuration parameters
 */
#define RCC_SOURCE              LL_RCC_PLLSOURCE_HSE
#define RCC_PLLM                LL_RCC_PLLM_DIV_8
#define RCC_PLLN                336
#define RCC_PLLR                LL_RCC_PLLP_DIV_2
#define RCC_SYS_SOURCE          LL_RCC_SYS_CLKSOURCE_PLL
#define RCC_AHB_PRESCALER       LL_RCC_SYSCLK_DIV_1
#define RCC_APB1_PRESCALER      LL_RCC_APB1_DIV_4
#define RCC_APB2_PRESCALER      LL_RCC_APB2_DIV_2

/*
 * Terminal configuration
 * UART configuration
 */
#define TERM_USART                      USART1
#define TERM_USART_IRQN                 USART1_IRQn
#define TERM_USART_IRQN_PRIORITY        1
#define TERM_USART_PERIPH_PRESCALER     2
#define TERM_USART_BAUDRATE             15200
#define TERM_USART_DATA_WIDTH           LL_USART_DATAWIDTH_8B
#define TERM_USART_HAWDWARE_FLOAT_CNTRL LL_USART_HWCONTROL_NONE
#define TERM_USART_OVERSAMPL            LL_USART_OVERSAMPLING_16
#define TERM_USART_PARITY               LL_USART_PARITY_NONE
#define TERM_USART_STOPBITS             LL_USART_STOPBITS_1
#define TERM_USART_TRANSFER_DIRECTION   LL_USART_DIRECTION_TX_RX

/*
 * Terminal configuration
 * DMA configuration
 */
#define TERM_DMA                        DMA2
#define TERM_DMA_CHANNEL                LL_DMA_CHANNEL_4
#define TERM_DMA_STREAM                 LL_DMA_STREAM_2
#define TERM_DMA_STREAM_IRQN            DMA2_Stream2_IRQn
#define TERM_DMA_STREAM_IRQN_PRIORITY   (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1)
#define TERM_DMA_DIRECTION              LL_DMA_DIRECTION_PERIPH_TO_MEMORY
#define TERM_DMA_BUFFER_SIZE            256
#define TERM_DMA_MEM_INC_MODE           LL_DMA_MEMORY_INCREMENT
#define TERM_DMA_SRC_ADDR               (uint32_t)&(TERM_USART)->DR

#endif