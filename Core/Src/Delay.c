
#include "Delay.h"

static uint8_t  fac_us=0;//us延時倍乘數
static uint16_t fac_ms=0;//ms延時倍乘數
//初始化延遲函式
//SYSTICK的時鐘固定為HCLK時鐘的1/8
//SYSCLK:系統時鐘
void delay_init(uint8_t SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;//bit2清空,選擇外部時鐘  HCLK/8
	fac_us=SYSCLK/8;
	fac_ms=(uint16_t)fac_us*1000;
}

void delay_ms(uint16_t nms)
{
	uint32_t temp;
	SysTick->LOAD=(uint32_t)nms*fac_ms;//時間載入(SysTick->LOAD為24bit)
	SysTick->VAL =0x00;           //清空計數器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;          //開始倒數
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待時間到達
	SysTick->CTRL=0x00;       //關閉計數器
	SysTick->VAL =0X00;       //清空計數器
}


//延時nus
//nus為要延時的us數.
void delay_us(uint32_t nus)
{
	uint32_t temp;
	SysTick->LOAD=nus*fac_us; //時間載入
	SysTick->VAL=0x00;        //清空計數器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;      //開始倒數
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待時間到達
	SysTick->CTRL=0x00;       //關閉計數器
	SysTick->VAL =0X00;       //清空計數器
}

uint32_t multiplier;

void TM_Delay_Init(void) {
	/* While loop takes 4 cycles */
	/* For 1 us delay, we need to divide with 4M */
	multiplier = 122000000 / 4000000;
}

void TM_DelayMicros(uint32_t micros) {
	/* Multiply micros with multipler */
	/* Substract 10 */
	micros = micros * multiplier - 10;
	/* 4 cycles for one loop */
	while (micros--);
}

void TM_DelayMillis(uint32_t millis) {
	/* Multiply millis with multipler */
	/* Substract 10 */
	millis = 1000 * millis * multiplier - 10;
	/* 4 cycles for one loop */
	while (millis--);
}
