/*
 * ads1115.c
 *
 *  Created on: Sep 24, 2020
 *      Author: user
 */
#include "main.h"
#include "Delay.h"

extern I2C_HandleTypeDef hi2c1;

void Read_I2C(void)
{
	unsigned char TX_buffer[1];
	unsigned char RX_buffer[1];
	unsigned char i2c_addr = 0x90;

	TX_buffer[0] = 0x55;
	HAL_I2C_Master_Transmit(&hi2c1, i2c_addr, (uint8_t *)TX_buffer, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, i2c_addr+1, RX_buffer, 1, 100);
	delay_ms(1);
}

float Read_ads1115(void)
{

}
