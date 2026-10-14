/* Minimal HAL model for host regression tests; never included by firmware. */
#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef struct { uint32_t CR1, CCMR1, ARR, CCR[2], CNT, DIER, SR; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; unsigned active[2]; } TIM_HandleTypeDef;
typedef struct { unsigned unused; } UART_HandleTypeDef;
extern TIM_TypeDef fake_tim1, fake_tim4, fake_tim16;
#define TIM16 (&fake_tim16)
#define TIM4 (&fake_tim4)
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define TIM_CR1_OPM 8U
#define TIM_CR1_CEN 1U
#define TIM_CCMR1_OC1M 0x70U
#define TIM_OCMODE_PWM1 0x60U
#define TIM_OCMODE_PWM2 0x70U
#define TIM_IT_UPDATE 1U
#define TIM_FLAG_UPDATE 1U
#define TIM_EVENTSOURCE_UPDATE 1U
#define TIM1_UP_TIM16_IRQn 0
#define HAL_TIM_CHANNEL_STATE_BUSY 1
#define TIM_CHANNEL_STATE_GET(t,c) ((t)->active[c])
#define SET_BIT(r,b) ((r) |= (b))
#define CLEAR_BIT(r,b) ((r) &= ~(b))
#define MODIFY_REG(r,m,v) ((r) = ((r) & ~(m)) | (v))
#define __HAL_TIM_SET_COUNTER(t,v) ((t)->Instance->CNT = (v))
#define __HAL_TIM_SET_AUTORELOAD(t,v) ((t)->Instance->ARR = (v))
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->Instance->ARR)
#define __HAL_TIM_GET_COUNTER(t) ((t)->Instance->CNT)
#define __HAL_TIM_GET_COMPARE(t,c) ((t)->Instance->CCR[c])
#define __HAL_TIM_GET_FLAG(t,b) ((t)->Instance->SR & (b))
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->Instance->CCR[c] = (v))
#define __HAL_TIM_DISABLE_IT(t,b) CLEAR_BIT((t)->Instance->DIER,b)
#define __HAL_TIM_ENABLE_IT(t,b) SET_BIT((t)->Instance->DIER,b)
#define __HAL_TIM_CLEAR_FLAG(t,b) CLEAR_BIT((t)->Instance->SR,b)
#define GPIOA 0
#define GPIOB 1
#define GPIOC 2
#define GPIOF 5
#define GPIO_PIN_1 (1U<<1)
#define GPIO_PIN_9 (1U<<9)
#define GPIO_PIN_3 (1U<<3)
#define GPIO_PIN_4 (1U<<4)
#define GPIO_PIN_10 (1U<<10)
#define GPIO_PIN_11 (1U<<11)
#define GPIO_PIN_14 (1U<<14)
#define GPIO_PIN_15 (1U<<15)
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define UART_CLEAR_OREF 1
#define UART_CLEAR_NEF 2
#define UART_CLEAR_FEF 4
#define UART_CLEAR_PEF 8
#define UART_RXDATA_FLUSH_REQUEST 1
#define __HAL_UART_CLEAR_FLAG(p,f) ((void)0)
#define __HAL_UART_SEND_REQ(p,f) ((void)0)
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(m) ((void)(m))
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
void HAL_NVIC_SetPriority(int irq, int preempt, int sub);
void HAL_NVIC_EnableIRQ(int irq);
void HAL_NVIC_ClearPendingIRQ(int irq);
void HAL_GPIO_WritePin(int port, uint16_t pin, int value);
int HAL_GPIO_ReadPin(int port, uint16_t pin);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *t, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *t, uint32_t event);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *p, uint8_t *data, uint16_t n);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *p, uint8_t *data, uint16_t n);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *p);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *p);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *p);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *p);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *t);
#endif
