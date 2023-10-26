/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include <reg_util.h>
#include <stm32f429xx.h>
#include <bsp_lcd.h>

void systemClockSetup()
{
	RCC_TypeDef *rcc = RCC; //RCC base address
	FLASH_TypeDef *flash = FLASH;
	PWR_TypeDef *pwr = PWR;

	//configure wait state for flash
	REG_SET_VAL(flash->ACR, FLASH_ACR_LATENCY_5WS, 0xFUL, FLASH_ACR_LATENCY_Pos);

	//configure overdrive to achieve 180Mhz
	REG_SET_BIT(rcc->APB1ENR, RCC_APB1ENR_PWREN_Pos);
	REG_SET_VAL(pwr->CR, 0x03, 0x3UL, PWR_CR_VOS_Pos);
	REG_SET_BIT(pwr->CR, PWR_CR_ODEN_Pos);
	while(! REG_READ_BIT(pwr->CSR, PWR_CSR_ODRDY_Pos));
	REG_SET_BIT(pwr->CR, PWR_CR_ODSWEN_Pos);

	//Configure PLL
	//VCO should between 1and 2Mhz
	//write 8 (8Mhz HSE/8 = 2Mhz), mask 3F to clear, position zero
	REG_SET_VAL(rcc->PLLCFGR,0x8U,0x3FU,RCC_PLLCFGR_PLLM_Pos);
	REG_SET_VAL(rcc->PLLCFGR,180U,0x1FFU,RCC_PLLCFGR_PLLN_Pos);
	REG_SET_VAL(rcc->PLLCFGR,0U,3U,RCC_PLLCFGR_PLLP_Pos);

	//Display dotclock PLLSAI
	REG_SET_VAL(rcc->PLLCFGR,50U,0x1FFUL,RCC_PLLSAICFGR_PLLSAIN_Pos);
	REG_SET_VAL(rcc->PLLCFGR,2U,0x7UL,RCC_PLLSAICFGR_PLLSAIR_Pos);
	REG_SET_VAL(rcc->DCKCFGR,8U,0x3UL,RCC_DCKCFGR_PLLSAIDIVR_Pos);

	//enable PLLSAI
	REG_SET_BIT(rcc->CR, RCC_CR_PLLSAION_Pos);
	while(!REG_READ_BIT(rcc->CR, RCC_CR_PLLSAIRDY_Pos));

	//Configure AHB, APB1 and APB2
	REG_SET_VAL(rcc->CFGR, RCC_CFGR_HPRE_DIV1, 0xFUL, RCC_CFGR_HPRE_Pos); //no clock div, out 180Mhz
	REG_SET_VAL(rcc->CFGR, RCC_CFGR_PPRE1_DIV4, 0x7UL, RCC_CFGR_PPRE1_Pos); //APB1 45Mhz
	REG_SET_VAL(rcc->CFGR, RCC_CFGR_PPRE2_DIV2, 0x7UL, RCC_CFGR_PPRE2_Pos); //APB2 90Mhz


	//enable PLL and wait
	REG_SET_BIT(rcc->CR, RCC_CR_PLLON_Pos);
	while(!REG_READ_BIT(rcc->CR, RCC_CR_PLLRDY_Pos));

	//set PLL as sysclock
	REG_SET_VAL(rcc->CFGR, RCC_CFGR_SW_PLL, 0x3UL, RCC_CFGR_SW_Pos);
	while(!((REG_READ_VAL(rcc->CFGR, 0x3UL, RCC_CFGR_SWS_Pos))== RCC_CFGR_SW_PLL));

}


void ltdcSetup()
{
	RCC_TypeDef *rcc = RCC;
	LTDC_TypeDef *ltdc = LTDC;
	REG_SET_BIT(rcc->APB2ENR, RCC_APB2ENR_LTDCEN_Pos);
	//width in pixclock
	REG_SET_VAL(ltdc->SSCR, BSP_LCD_HSYNC_WIDTH-1, 0xFFFU, LTDC_SSCR_HSW_Pos);
	REG_SET_VAL(ltdc->SSCR, BSP_LCD_VSYNC_WIDTH-1, 0x7FFU, LTDC_SSCR_VSH_Pos);
	//back porch
	REG_SET_VAL(ltdc->BPCR, BSP_LCD_HSYNC_WIDTH+BSP_LCD_HSYNC_BP-1,0xFFFU,LTDC_BPCR_AHBP_Pos);
	REG_SET_VAL(ltdc->BPCR, BSP_LCD_VSYNC_WIDTH+BSP_LCD_VSYNC_BP-1,0x7FFU,LTDC_BPCR_AVBP_Pos);
	//active width
	REG_SET_VAL(ltdc->AWCR, BSP_LCD_HSYNC_WIDTH+BSP_LCD_HSYNC_BP+BSP_LCD_HSYNC_ADD-1, 0xFFFU,LTDC_AWCR_AAH_Pos);
	REG_SET_VAL(ltdc->AWCR, BSP_LCD_VSYNC_WIDTH+BSP_LCD_VSYNC_BP+BSP_LCD_VSYNC_ADD-1, 0x7FFU,LTDC_AWCR_AAW_Pos);
	//total width
	REG_SET_VAL(ltdc->TWCR, BSP_LCD_HSYNC_WIDTH+BSP_LCD_HSYNC_BP+BSP_LCD_HSYNC_ADD+BSP_LCD_HSYNC_FP-1, 0xFFFU,LTDC_TWCR_TOTALH_Pos);
	REG_SET_VAL(ltdc->TWCR, BSP_LCD_VSYNC_WIDTH+BSP_LCD_VSYNC_BP+BSP_LCD_VSYNC_ADD+BSP_LCD_VSYNC_FP-1, 0x7FFU,LTDC_TWCR_TOTALW_Pos);

	//background color
	REG_SET_VAL(ltdc->BCCR, 0xFF, 0xFFUL, LTDC_BCCR_BCRED_Pos);
	REG_SET_VAL(ltdc->BCCR, 0, 0xFFUL, LTDC_BCCR_BCBLUE_Pos);
	REG_SET_VAL(ltdc->BCCR, 0, 0xFFUL, LTDC_BCCR_BCGREEN_Pos);

	// set polarization


	//enable peripheral
	REG_SET_BIT(ltdc->GCR, LTDC_GCR_LTDCEN_Pos);
}


int main(void)
{
	systemClockSetup();
	ltdcSetup();
    /* Loop forever */
	for(;;);
}