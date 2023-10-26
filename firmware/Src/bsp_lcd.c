/*
 * lcd_bsp.c
 *
 *  Created on: Oct 24, 2023
 *      Author: fernando
 */

#include <bsp_lcd.h>
#include <stm32f429xx.h>
#include <unistd.h>
#include <reg_util.h>

GPIO_TypeDef *ltdcPorts[] = {
		LCD_DATA_R2_PORT,
		LCD_DATA_R3_PORT,
		LCD_DATA_R4_PORT,
		LCD_DATA_R5_PORT,
		LCD_DATA_R6_PORT,
		LCD_DATA_R7_PORT,
		LCD_DATA_G2_PORT,
		LCD_DATA_G3_PORT,
		LCD_DATA_G4_PORT,
		LCD_DATA_G5_PORT,
		LCD_DATA_G6_PORT,
		LCD_DATA_G7_PORT,
		LCD_DATA_B2_PORT,
		LCD_DATA_B3_PORT,
		LCD_DATA_B4_PORT,
		LCD_DATA_B5_PORT,
		LCD_DATA_B6_PORT,
		LCD_DATA_B7_PORT,
		LCD_VSYNC_PORT,
		LCD_HSYNC_PORT,
		LCD_DOTCLK_PORT,
		LCD_DE_PORT
};


const uint8_t ltdcPins[]={
		LCD_DATA_R2_PIN,
		LCD_DATA_R3_PIN,
		LCD_DATA_R4_PIN,
		LCD_DATA_R5_PIN,
		LCD_DATA_R6_PIN,
		LCD_DATA_R7_PIN,
		LCD_DATA_G2_PIN,
		LCD_DATA_G3_PIN,
		LCD_DATA_G4_PIN,
		LCD_DATA_G5_PIN,
		LCD_DATA_G6_PIN,
		LCD_DATA_G7_PIN,
		LCD_DATA_B2_PIN,
		LCD_DATA_B3_PIN,
		LCD_DATA_B4_PIN,
		LCD_DATA_B5_PIN,
		LCD_DATA_B6_PIN,
		LCD_DATA_B7_PIN,
		LCD_VSYNC_PIN,
		LCD_HSYNC_PIN,
		LCD_DOTCLK_PIN,
		LCD_DE_PIN
};

const uint8_t ltdcPinsArraySize = sizeof(ltdcPins)/sizeof(ltdcPins[0]);

static void _initLtdcPins()
{
	RCC_TypeDef *rcc = RCC;
	REG_SET_VAL(rcc->AHB1ENR, 0x1FF, 0x1FF, RCC_AHB1ENR_GPIOAEN_Pos);
	for(uint8_t i = 0; i < ltdcPinsArraySize; i++)
	{
		REG_SET_VAL(ltdcPorts[i]->MODER, 2U, 3U,(ltdcPins[i] * 2U));
		REG_CLR_BIT(ltdcPorts[i]->OTYPER, ltdcPins[i]);
		REG_SET_VAL(ltdcPorts[i]->OSPEEDR, 2U, 3U, (ltdcPins[i] * 2U));
		if(i < 8)
		{
			REG_SET_VAL(ltdcPorts[i]->AFR[0], 14U, 0xFU, ltdcPins[i] * 4U);
		}
		else
		{
			REG_SET_VAL(ltdcPorts[i]->AFR[1], 14U, 0xFU, (ltdcPins[i] % 8U) * 4U);
		}
	}

}


void BSP_initDisplay(){
	_initLtdcPins();
}



