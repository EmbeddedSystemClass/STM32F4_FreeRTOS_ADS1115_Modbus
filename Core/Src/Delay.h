#include "stdint.h"
#include "stm32f4xx_hal.h"
void delay_init(uint8_t SYSCLK);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);
void TM_Delay_Init(void);
void TM_DelayMicros(uint32_t micros);
void TM_DelayMillis(uint32_t millis);
