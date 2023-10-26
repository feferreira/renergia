#ifndef LCD_BSP_H
#define LCD_BSP_H
#include <stm32f429xx.h>

#define BSP_LCD_HSYNC_WIDTH 10
#define BSP_LCD_HSYNC_BP 20
#define BSP_LCD_HSYNC_ADD 240
#define BSP_LCD_HSYNC_FP 10

#define BSP_LCD_VSYNC_WIDTH 2 //line
#define BSP_LCD_VSYNC_BP 2 //line
#define BSP_LCD_VSYNC_ADD 320
#define BSP_LCD_VSYNC_FP 4 //line




#define GPIO_PIN_0 0U
#define GPIO_PIN_1 1U
#define GPIO_PIN_2 2U
#define GPIO_PIN_3 3U
#define GPIO_PIN_4 4U
#define GPIO_PIN_5 5U
#define GPIO_PIN_6 6U
#define GPIO_PIN_7 7U
#define GPIO_PIN_8 8U
#define GPIO_PIN_9 9U
#define GPIO_PIN_10 10U
#define GPIO_PIN_11 11U
#define GPIO_PIN_12 12U
#define GPIO_PIN_13 13U
#define GPIO_PIN_14 14U
#define GPIO_PIN_15 15U

//VSYNC
#define LCD_VSYNC_PIN GPIO_PIN_4
#define LCD_VSYNC_PORT GPIOA

//HSYNC
#define LCD_HSYNC_PIN GPIO_PIN_6
#define LCD_HSYNC_PORT GPIOC

//DOTCLOCK
#define LCD_DOTCLK_PIN GPIO_PIN_7
#define LCD_DOTCLK_PORT GPIOG

//DE
#define LCD_DE_PIN GPIO_PIN_10
#define LCD_DE_PORT GPIOF

//DATA_R
#define LCD_DATA_R2_PIN GPIO_PIN_10
#define LCD_DATA_R2_PORT GPIOC
#define LCD_DATA_R3_PIN GPIO_PIN_0
#define LCD_DATA_R3_PORT GPIOB
#define LCD_DATA_R4_PIN GPIO_PIN_11
#define LCD_DATA_R4_PORT GPIOA
#define LCD_DATA_R5_PIN GPIO_PIN_12
#define LCD_DATA_R5_PORT GPIOA
#define LCD_DATA_R6_PIN GPIO_PIN_1
#define LCD_DATA_R6_PORT GPIOB
#define LCD_DATA_R7_PIN GPIO_PIN_6
#define LCD_DATA_R7_PORT GPIOG

//DATA_G
#define LCD_DATA_G2_PIN GPIO_PIN_6
#define LCD_DATA_G2_PORT GPIOA
#define LCD_DATA_G3_PIN GPIO_PIN_10
#define LCD_DATA_G3_PORT GPIOG
#define LCD_DATA_G4_PIN GPIO_PIN_10
#define LCD_DATA_G4_PORT GPIOB
#define LCD_DATA_G5_PIN GPIO_PIN_11
#define LCD_DATA_G5_PORT GPIOB
#define LCD_DATA_G6_PIN GPIO_PIN_7
#define LCD_DATA_G6_PORT GPIOC
#define LCD_DATA_G7_PIN GPIO_PIN_3
#define LCD_DATA_G7_PORT GPIOD

//DATA_B
#define LCD_DATA_B2_PIN GPIO_PIN_6
#define LCD_DATA_B2_PORT GPIOD
#define LCD_DATA_B3_PIN GPIO_PIN_11
#define LCD_DATA_B3_PORT GPIOG
#define LCD_DATA_B4_PIN GPIO_PIN_12
#define LCD_DATA_B4_PORT GPIOG
#define LCD_DATA_B5_PIN GPIO_PIN_3
#define LCD_DATA_B5_PORT GPIOA
#define LCD_DATA_B6_PIN GPIO_PIN_8
#define LCD_DATA_B6_PORT GPIOB
#define LCD_DATA_B7_PIN GPIO_PIN_9
#define LCD_DATA_B7_PORT GPIOB


void BSP_initDisplay();

#endif