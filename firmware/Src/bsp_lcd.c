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

#define SPI						SPI5
#define LCD_SCL_PIN				GPIO_PIN_7
#define LCD_SCL_PORT			GPIOF
#define LCD_SDA_PIN				GPIO_PIN_9
#define LCD_SDA_PORT			GPIOF
#define LCD_RESX_PIN			GPIO_PIN_7
#define LCD_RESX_PORT			GPIOA
#define LCD_CSX_PIN				GPIO_PIN_2
#define LCD_CSX_PORT			GPIOC
#define LCD_DCX_PIN			    GPIO_PIN_13
#define LCD_DCX_PORT		    GPIOD

#define LCD_RESX_HIGH()				REG_SET_BIT(LCD_RESX_PORT->ODR,LCD_RESX_PIN)
#define LCD_RESX_LOW()				REG_CLR_BIT(LCD_RESX_PORT->ODR,LCD_RESX_PIN)

#define LCD_CSX_HIGH()				REG_SET_BIT(LCD_CSX_PORT->ODR,LCD_CSX_PIN)
#define LCD_CSX_LOW()				REG_CLR_BIT(LCD_CSX_PORT->ODR,LCD_CSX_PIN)

#define LCD_DCX_HIGH()				REG_SET_BIT(LCD_DCX_PORT->ODR,LCD_DCX_PIN)
#define LCD_DCX_LOW()				REG_CLR_BIT(LCD_DCX_PORT->ODR,LCD_DCX_PIN)


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

static void _initLtdc()
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

static void _initSpiPins(){
	RCC_TypeDef *pRCC = RCC;
	GPIO_TypeDef *pGPIOA = GPIOA;
	GPIO_TypeDef *pGPIOC = GPIOC;
	GPIO_TypeDef *pGPIOD = GPIOD;
	GPIO_TypeDef *pGPIOF = GPIOF;

	/* Enable the clock for GPIOA,C,D,F peripherals */
	REG_SET_BIT(pRCC->AHB1ENR,RCC_AHB1ENR_GPIOAEN_Pos);
	REG_SET_BIT(pRCC->AHB1ENR,RCC_AHB1ENR_GPIOCEN_Pos);
	REG_SET_BIT(pRCC->AHB1ENR,RCC_AHB1ENR_GPIODEN_Pos);
	REG_SET_BIT(pRCC->AHB1ENR,RCC_AHB1ENR_GPIOFEN_Pos);

	//RESX
	REG_SET_VAL(pGPIOA->MODER,0x1U,0x3,(LCD_RESX_PIN * 2U)); 		/*mode*/
	REG_CLR_BIT(pGPIOA->OTYPER,LCD_RESX_PIN); 						/*Output type*/
	REG_SET_VAL(pGPIOA->OSPEEDR,2U,0x3U,(LCD_RESX_PIN * 2U)); 		/*speed*/


	//CSX
	REG_SET_VAL(pGPIOC->MODER,0x1U,0x3,(LCD_CSX_PIN * 2U)); 		/*mode*/
	REG_CLR_BIT(pGPIOC->OTYPER,LCD_CSX_PIN); 						/*Output type*/
	REG_SET_VAL(pGPIOC->OSPEEDR,2U,0x3U,(LCD_CSX_PIN * 2U)); 		/*speed*/

	//D/CX
	REG_SET_VAL(pGPIOD->MODER,0x1U,0x3,(LCD_DCX_PIN * 2U)); 		/*mode*/
	REG_CLR_BIT(pGPIOD->OTYPER,LCD_DCX_PIN); 					/*Output type*/
	REG_SET_VAL(pGPIOD->OSPEEDR,2U,0x3U,(LCD_DCX_PIN * 2U)); 		/*speed*/

	//SPI_CLK(PF7)
	REG_SET_VAL(pGPIOF->MODER,2U,0x3U,(LCD_SCL_PIN * 2U));
	REG_CLR_BIT(pGPIOF->OTYPER,LCD_SCL_PIN);
	REG_SET_VAL(pGPIOF->OSPEEDR,2U,0x3U,(LCD_SCL_PIN * 2U));
	REG_SET_VAL(pGPIOF->AFR[0],5U,0xFU,(LCD_SCL_PIN * 4U));

	//SPI_SDA(PF9)
	REG_SET_VAL(pGPIOF->MODER,2U,0x3U,(LCD_SDA_PIN * 2U));
	REG_CLR_BIT(pGPIOF->OTYPER,LCD_SDA_PIN);
	REG_SET_VAL(pGPIOF->OSPEEDR,2U,0x3U,(LCD_SDA_PIN * 2U));
	REG_SET_VAL(pGPIOF->AFR[1],5U,0xFU,((LCD_SDA_PIN % 8) * 4U));

	//CSX = HIGH
	REG_SET_BIT(pGPIOC->ODR,LCD_CSX_PIN);
	//RESX = HIGH
	REG_SET_BIT(pGPIOA->ODR,LCD_RESX_PIN);
	//D/CX = HIGH
	REG_SET_BIT(pGPIOD->ODR,LCD_DCX_PIN);

}

static void _initSpi(void)
{
	SPI_TypeDef *pSPI = SPI;
	RCC_TypeDef *pRCC = RCC;

	REG_SET_BIT(pRCC->APB2ENR,RCC_APB2ENR_SPI5EN_Pos);

	REG_SET_BIT(pSPI->CR1,SPI_CR1_MSTR_Pos); 		/*Controller mode*/
	REG_SET_BIT(pSPI->CR1,SPI_CR1_BIDIMODE_Pos);    /* BIDI mode enable*/
	REG_SET_BIT(pSPI->CR1,SPI_CR1_BIDIOE_Pos);      /* Tx only*/
	REG_SET_BIT(pSPI->CR1,SPI_CR1_BIDIOE_Pos);		/* Tx only */
	REG_CLR_BIT(pSPI->CR1,SPI_CR1_DFF_Pos);			/* DFF = 8bits */
	REG_SET_BIT(pSPI->CR1,SPI_CR1_SSM_Pos);			/* SSM enable */
	REG_SET_BIT(pSPI->CR1,SPI_CR1_SSI_Pos);			/* SSI enable */
	REG_CLR_BIT(pSPI->CR1,SPI_CR1_LSBFIRST_Pos);     /* Send msb first */
	REG_SET_VAL(pSPI->CR1,0x3U,0x7U,SPI_CR1_BR_Pos); /* SPI clck = 90MHz/16 ==> 5.625 MHz */
	REG_CLR_BIT(pSPI->CR1,SPI_CR1_CPOL_Pos); 		 /* CPOL = 0 */
	REG_CLR_BIT(pSPI->CR1,SPI_CR1_CPHA_Pos); 		 /* CPHA = 0 */
	REG_CLR_BIT(pSPI->CR2,SPI_CR2_FRF_Pos);			 /* SPI Motorola frame format*/
	REG_SET_BIT(pSPI->CR1,SPI_CR1_SPE_Pos);
}


static void _resetLcd(void)
{
	LCD_RESX_LOW();
	for(uint32_t i = 0 ; i<(0xFFFFU * 10U);i++);
	LCD_RESX_HIGH();
	for(uint32_t i = 0 ; i<(0xFFFFU * 10U);i++);
}


static void _writeLcdCmd(uint8_t cmd)
{
	SPI_TypeDef *pSPI = SPI;
	LCD_CSX_LOW();
	LCD_DCX_LOW(); //DCX = 0 , for command
	while(!REG_READ_BIT(pSPI->SR,SPI_SR_TXE_Pos));
	REG_WRITE(pSPI->DR,cmd);
	while(!REG_READ_BIT(pSPI->SR,SPI_SR_TXE_Pos));
	while(REG_READ_BIT(pSPI->SR,SPI_SR_BSY_Pos));
	LCD_DCX_HIGH();
	LCD_CSX_HIGH();

}

static void _writeLcdData(uint8_t *buffer,uint32_t len)
{
	SPI_TypeDef *pSPI = SPI;
	for(uint32_t i = 0 ; i < len ;i++){
		LCD_CSX_LOW();
		while(!REG_READ_BIT(pSPI->SR,SPI_SR_TXE_Pos));
		REG_WRITE(pSPI->DR,buffer[i]);
		while(!REG_READ_BIT(pSPI->SR,SPI_SR_TXE_Pos));
		while(REG_READ_BIT(pSPI->SR,SPI_SR_BSY_Pos));
		LCD_CSX_HIGH();
	}
}

static void _configLcd(void)
{
	uint8_t params[15];
	_writeLcdCmd(ILI9341_SWRESET);
	_writeLcdCmd(ILI9341_POWERB);
	params[0] = 0x00;
	params[1] = 0xD9;
	params[2] = 0x30;
	_writeLcdData(params, 3);

	_writeLcdCmd(ILI9341_POWER_SEQ);
	params[0]= 0x64;
	params[1]= 0x03;
	params[2]= 0X12;
	params[3]= 0X81;
	_writeLcdData(params, 4);

	_writeLcdCmd(ILI9341_DTCA);
	params[0]= 0x85;
	params[1]= 0x10;
	params[2]= 0x7A;
	_writeLcdData(params, 3);

	_writeLcdCmd(ILI9341_POWERA);
	params[0]= 0x39;
	params[1]= 0x2C;
	params[2]= 0x00;
	params[3]= 0x34;
	params[4]= 0x02;
	_writeLcdData(params, 5);

	_writeLcdCmd(ILI9341_PRC);
	params[0]= 0x20;
	_writeLcdData(params, 1);

	_writeLcdCmd(ILI9341_DTCB);
	params[0]= 0x00;
	params[1]= 0x00;
	_writeLcdData(params, 2);

	_writeLcdCmd(ILI9341_POWER1);
	params[0]= 0x1B;
	_writeLcdData(params, 1);

	_writeLcdCmd(ILI9341_POWER2);
	params[0]= 0x12;
	_writeLcdData(params, 1);

	_writeLcdCmd(ILI9341_VCOM1);
	params[0]= 0x08;
	params[1]= 0x26;
	_writeLcdData(params, 2);

	_writeLcdCmd(ILI9341_VCOM2);
	params[0]= 0XB7;
	_writeLcdData(params, 1);


	_writeLcdCmd(ILI9341_PIXEL_FORMAT);
	params[0]= 0x55; //select RGB565
	_writeLcdData(params, 1);

	_writeLcdCmd(ILI9341_FRMCTR1);
	params[0]= 0x00;
	params[1]= 0x1B;//frame rate = 70
	_writeLcdData(params, 2);

	_writeLcdCmd(ILI9341_DFC);    // Display Function Control
	params[0]= 0x0A;
	params[1]= 0xA2;
	_writeLcdData(params, 2);

	_writeLcdCmd(ILI9341_3GAMMA_EN);    // 3Gamma Function Disable
	params[0]= 0x02;
	_writeLcdData(params, 1);

	_writeLcdCmd(ILI9341_GAMMA);
	params[0]= 0x01;
	_writeLcdData(params, 1);

	_writeLcdCmd(ILI9341_PGAMMA);    //Set Gamma
	params[0]= 0x0F;
	params[1]= 0x1D;
	params[2]= 0x1A;
	params[3]= 0x0A;
	params[4]= 0x0D;
	params[5]= 0x07;
	params[6]= 0x49;
	params[7]= 0X66;
	params[8]= 0x3B;
	params[9]= 0x07;
	params[10]= 0x11;
	params[11]= 0x01;
	params[12]= 0x09;
	params[13]= 0x05;
	params[14]= 0x04;
	_writeLcdData(params, 15);

	_writeLcdCmd(ILI9341_NGAMMA);
	params[0]= 0x00;
	params[1]= 0x18;
	params[2]= 0x1D;
	params[3]= 0x02;
	params[4]= 0x0F;
	params[5]= 0x04;
	params[6]= 0x36;
	params[7]= 0x13;
	params[8]= 0x4C;
	params[9]= 0x07;
	params[10]= 0x13;
	params[11]= 0x0F;
	params[12]= 0x2E;
	params[13]= 0x2F;
	params[14]= 0x05;
	_writeLcdData(params, 15);

	_writeLcdCmd(ILI9341_RGB_INTERFACE);
	params[0] = 0xC2; //Data is fetched during falling edge of DOTCLK
	_writeLcdData(params, 1);

	_writeLcdCmd(ILI9341_INTERFACE);
	params[0] = 0x00;
	params[1] = 0x00;
	params[2] = 0x06;
	_writeLcdData(params, 3);

	_writeLcdCmd(ILI9341_SLEEP_OUT); //Exit Sleep
	delay_50ms();
	delay_50ms();
	_writeLcdCmd(ILI9341_DISPLAY_ON); //display on

}






void BSP_initDisplay(){
	_initSpiPins();
	_initSpi();
	_resetLcd();
	_configLcd();
	_initLtdcPins();
	_initLtdc();
}



