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

#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left


/* Level 1 Commands */
#define ILI9341_SWRESET             0x01U   /* Software Reset */
#define ILI9341_READ_DISPLAY_ID     0x04U   /* Read display identification information */
#define ILI9341_RDDST               0x09U   /* Read Display Status */
#define ILI9341_RDDPM               0x0AU   /* Read Display Power Mode */
#define ILI9341_RDDMADCTL           0x0BU   /* Read Display MADCTL */
#define ILI9341_RDDCOLMOD           0x0CU   /* Read Display Pixel Format */
#define ILI9341_RDDIM               0x0DU   /* Read Display Image Format */
#define ILI9341_RDDSM               0x0EU   /* Read Display Signal Mode */
#define ILI9341_RDDSDR              0x0FU   /* Read Display Self-Diagnostic Result */
#define ILI9341_SPLIN               0x10U   /* Enter Sleep Mode */
#define ILI9341_SLEEP_OUT           0x11U   /* Sleep out register */
#define ILI9341_PTLON               0x12U   /* Partial Mode ON */
#define ILI9341_NORMAL_MODE_ON      0x13U   /* Normal Display Mode ON */
#define ILI9341_DINVOFF             0x20U   /* Display Inversion OFF */
#define ILI9341_DINVON              0x21U   /* Display Inversion ON */
#define ILI9341_GAMMA               0x26U   /* Gamma register */
#define ILI9341_DISPLAY_OFF         0x28U   /* Display off register */
#define ILI9341_DISPLAY_ON          0x29U   /* Display on register */
#define ILI9341_CASET               0x2AU   /* Colomn address register */
#define ILI9341_RASET               0x2BU   /* Page address register */
#define ILI9341_GRAM                0x2CU   /* GRAM register */
#define ILI9341_RGBSET              0x2DU   /* Color SET */
#define ILI9341_RAMRD               0x2EU   /* Memory Read */
#define ILI9341_PLTAR               0x30U   /* Partial Area */
#define ILI9341_VSCRDEF             0x33U   /* Vertical Scrolling Definition */
#define ILI9341_TEOFF               0x34U   /* Tearing Effect Line OFF */
#define ILI9341_TEON                0x35U   /* Tearing Effect Line ON */
#define ILI9341_MAC                 0x36U   /* Memory Access Control register*/
#define ILI9341_VSCRSADD            0x37U   /* Vertical Scrolling Start Address */
#define ILI9341_IDMOFF              0x38U   /* Idle Mode OFF */
#define ILI9341_IDMON               0x39U   /* Idle Mode ON */
#define ILI9341_PIXEL_FORMAT        0x3AU   /* Pixel Format register */
#define ILI9341_WRITE_MEM_CONTINUE  0x3CU   /* Write Memory Continue */
#define ILI9341_READ_MEM_CONTINUE   0x3EU   /* Read Memory Continue */
#define ILI9341_SET_TEAR_SCANLINE   0x44U   /* Set Tear Scanline */
#define ILI9341_GET_SCANLINE        0x45U   /* Get Scanline */
#define ILI9341_WDB                 0x51U   /* Write Brightness Display register */
#define ILI9341_RDDISBV             0x52U   /* Read Display Brightness */
#define ILI9341_WCD                 0x53U   /* Write Control Display register*/
#define ILI9341_RDCTRLD             0x54U   /* Read CTRL Display */
#define ILI9341_WRCABC              0x55U   /* Write Content Adaptive Brightness Control */
#define ILI9341_RDCABC              0x56U   /* Read Content Adaptive Brightness Control */
#define ILI9341_WRITE_CABC          0x5EU   /* Write CABC Minimum Brightness */
#define ILI9341_READ_CABC           0x5FU   /* Read CABC Minimum Brightness */
#define ILI9341_READ_ID1            0xDAU   /* Read ID1 */
#define ILI9341_READ_ID2            0xDBU   /* Read ID2 */
#define ILI9341_READ_ID3            0xDCU   /* Read ID3 */

/* Level 2 Commands */
#define ILI9341_RGB_INTERFACE       0xB0U   /* RGB Interface Signal Control */
#define ILI9341_FRMCTR1             0xB1U   /* Frame Rate Control (In Normal Mode) */
#define ILI9341_FRMCTR2             0xB2U   /* Frame Rate Control (In Idle Mode) */
#define ILI9341_FRMCTR3             0xB3U   /* Frame Rate Control (In Partial Mode) */
#define ILI9341_INVTR               0xB4U   /* Display Inversion Control */
#define ILI9341_BPC                 0xB5U   /* Blanking Porch Control register */
#define ILI9341_DFC                 0xB6U   /* Display Function Control register */
#define ILI9341_ETMOD               0xB7U   /* Entry Mode Set */
#define ILI9341_BACKLIGHT1          0xB8U   /* Backlight Control 1 */
#define ILI9341_BACKLIGHT2          0xB9U   /* Backlight Control 2 */
#define ILI9341_BACKLIGHT3          0xBAU   /* Backlight Control 3 */
#define ILI9341_BACKLIGHT4          0xBBU   /* Backlight Control 4 */
#define ILI9341_BACKLIGHT5          0xBCU   /* Backlight Control 5 */
#define ILI9341_BACKLIGHT7          0xBEU   /* Backlight Control 7 */
#define ILI9341_BACKLIGHT8          0xBFU   /* Backlight Control 8 */
#define ILI9341_POWER1              0xC0U   /* Power Control 1 register */
#define ILI9341_POWER2              0xC1U   /* Power Control 2 register */
#define ILI9341_VCOM1               0xC5U   /* VCOM Control 1 register */
#define ILI9341_VCOM2               0xC7U   /* VCOM Control 2 register */
#define ILI9341_NVMWR               0xD0U   /* NV Memory Write */
#define ILI9341_NVMPKEY             0xD1U   /* NV Memory Protection Key */
#define ILI9341_RDNVM               0xD2U   /* NV Memory Status Read */
#define ILI9341_READ_ID4            0xD3U   /* Read ID4 */
#define ILI9341_PGAMMA              0xE0U   /* Positive Gamma Correction register */
#define ILI9341_NGAMMA              0xE1U   /* Negative Gamma Correction register */
#define ILI9341_DGAMCTRL1           0xE2U   /* Digital Gamma Control 1 */
#define ILI9341_DGAMCTRL2           0xE3U   /* Digital Gamma Control 2 */
#define ILI9341_INTERFACE           0xF6U   /* Interface control register */

/* Extend register commands */
#define ILI9341_POWERA               0xCBU   /* Power control A register */
#define ILI9341_POWERB               0xCFU   /* Power control B register */
#define ILI9341_DTCA                 0xE8U   /* Driver timing control A */
#define ILI9341_DTCB                 0xEAU   /* Driver timing control B */
#define ILI9341_POWER_SEQ            0xEDU   /* Power on sequence register */
#define ILI9341_3GAMMA_EN            0xF2U   /* 3 Gamma enable register */
#define ILI9341_PRC                  0xF7U   /* Pump ratio control register */


GPIO_TypeDef *ltdc_io_ports[] = {
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


const uint8_t ltdc_pins[]={
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

const uint8_t total_ltdc_pins = sizeof(ltdc_pins)/sizeof(ltdc_pins[0]);

void delay_50ms(void){
	for(uint32_t i = 0 ; i<(0xFFFFU * 10U);i++);
}


static void _initLtdcPins()
{
	REG_SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOAEN_Pos);
	REG_SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOBEN_Pos);
	REG_SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOCEN_Pos);
	REG_SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN_Pos);
	REG_SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOGEN_Pos);
	REG_SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOFEN_Pos);

	for(int i = 0 ; i < total_ltdc_pins ;i++){
		REG_SET_VAL(ltdc_io_ports[i]->MODER,2U,0x3U,(ltdc_pins[i] * 2U));
		REG_CLR_BIT(ltdc_io_ports[i]->OTYPER,ltdc_pins[i]);
		REG_SET_VAL(ltdc_io_ports[i]->OSPEEDR,2U,0x3U,(ltdc_pins[i] * 2U));
		if(ltdc_pins[i] < 8)
			REG_SET_VAL(ltdc_io_ports[i]->AFR[0],14U,0xFU,(ltdc_pins[i] * 4U));
		else
			REG_SET_VAL(ltdc_io_ports[i]->AFR[1],14U,0xFU,((ltdc_pins[i] % 8) * 4U));
	}

}

static void _initLtdc()
{
	LTDC_TypeDef *pLTDC = LTDC;

	REG_SET_BIT(RCC->APB2ENR,RCC_APB2ENR_LTDCEN_Pos);

	//Configure horizontal synchronization timings
	REG_SET_VAL(pLTDC->SSCR,(BSP_LCD_HSW-1),0xFFFU,LTDC_SSCR_HSW_Pos);
	REG_SET_VAL(pLTDC->BPCR,(BSP_LCD_HSW+BSP_LCD_HBP-1),0xFFFU,LTDC_BPCR_AHBP_Pos);
	REG_SET_VAL(pLTDC->AWCR,(BSP_LCD_HSW+BSP_LCD_HBP+BSP_LCD_ACTIVE_WIDTH-1),0xFFFU,LTDC_AWCR_AAW_Pos);
	uint32_t total_width = BSP_LCD_HSW+BSP_LCD_HBP+BSP_LCD_ACTIVE_WIDTH+BSP_LCD_HFP-1;
	REG_SET_VAL(pLTDC->TWCR,total_width,0xFFFU,LTDC_TWCR_TOTALW_Pos);

	//configure the vertical synchronization timings
	REG_SET_VAL(pLTDC->SSCR,(BSP_LCD_VSW-1),0x7FFU,LTDC_SSCR_VSH_Pos);
	REG_SET_VAL(pLTDC->BPCR,(BSP_LCD_VSW+BSP_LCD_VBP-1),0x7FFU,LTDC_BPCR_AVBP_Pos);
	REG_SET_VAL(pLTDC->AWCR,(BSP_LCD_VSW+BSP_LCD_VBP+BSP_LCD_HEIGHT-1),0x7FFU,LTDC_AWCR_AAH_Pos);
	uint32_t total_height = BSP_LCD_VSW+BSP_LCD_VBP+BSP_LCD_HEIGHT+BSP_LCD_VFP-1;
	REG_SET_VAL(pLTDC->TWCR,total_height,0x7FFU,LTDC_TWCR_TOTALH_Pos);

	//Configure the background color(BLUE)
	REG_SET_VAL(pLTDC->BCCR,0x0000FFU,0xFFFFFFU,LTDC_BCCR_BCBLUE_Pos);

	//default polarity for hsync, vsync, ltdc_clk, DE
	//TODO

	//enable the LTDC peripheral
	REG_SET_BIT(pLTDC->GCR,LTDC_GCR_LTDCEN_Pos);
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

void BSP_LCD_Set_Orientation(int orientation)
{
	uint8_t params[4];

	if(orientation == LANDSCAPE){

		_writeLcdCmd(ILI9341_RASET); //page address set
		params[0]= 0x00;
		params[1]= 0x00;
		params[2]= 0x00;
		params[3]= 0xf0; //240 rows = 0xf0
		_writeLcdData(params, 4);

		_writeLcdCmd(ILI9341_CASET);
		params[0]= 0x00;
		params[1]= 0x00;
		params[2]= 0x01;
		params[3]= 0x40; //320 columns = 0x140
		_writeLcdData(params, 4);

		params[0] = MADCTL_MV | MADCTL_MY | MADCTL_BGR; /*Memory Access Control <Landscape setting>*/
	}else if(orientation == PORTRAIT){

		_writeLcdCmd(ILI9341_RASET); //page address set
		params[0]= 0x00;
		params[1]= 0x00;
		params[2]= 0x01;
		params[3]= 0x40; //320 rows = 0x140
		_writeLcdData(params, 4);

		_writeLcdCmd(ILI9341_CASET);
		params[0]= 0x00;
		params[1]= 0x00;
		params[2]= 0x00;
		params[3]= 0xf0; //240 columns = 0xf0
		_writeLcdData(params, 4);

		params[0] = MADCTL_MY| MADCTL_MX| MADCTL_BGR;  /* Memory Access Control <portrait setting> */
	}

	_writeLcdCmd(ILI9341_MAC);    // Memory Access Control command
	_writeLcdData(params, 1);
}


void BSP_initDisplay(){
	_initSpiPins();
	_initSpi();
	_resetLcd();
	_configLcd();
	BSP_LCD_Set_Orientation(PORTRAIT);
	_initLtdcPins();
	_initLtdc();
}



