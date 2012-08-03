/*
 * rk29_m911.h
 *
 * Overview:  
 *
 * Copyright (c) 2011, YiFang Digital
 *
 * Version:  1.0
 * Created:  02/22/2011 03:36:04 PM
 * Author:  zqqu <zqqu@yifangdigital.com>
 * Company:  YiFang Digital
 * History:
 *
 * 
 */

// IRDA
#define BU92747GUW_RESET_PIN         RK29_PIN3_PD4// INVALID_GPIO //
#define BU92747GUW_RESET_MUX_NAME    GPIO3D4_HOSTWRN_NAME//NULL //
#define BU92747GUW_RESET_MUX_MODE    GPIO3H_GPIO3D4//NULL //

#define BU92747GUW_PWDN_PIN          RK29_PIN3_PD3//RK29_PIN5_PA7 //
#define BU92747GUW_PWDN_MUX_NAME     GPIO3D3_HOSTRDN_NAME//GPIO5A7_HSADCDATA2_NAME //
#define BU92747GUW_PWDN_MUX_MODE     GPIO3H_GPIO3D3//GPIO5L_GPIO5A7  //











//TOUCH
#define TOUCH_SCREEN_STANDBY_PIN          INVALID_GPIO    //VIP RK29_PIN6_PD1
#define TOUCH_SCREEN_STANDBY_VALUE        GPIO_HIGH
#define TOUCH_SCREEN_DISPLAY_PIN          INVALID_GPIO
#define TOUCH_SCREEN_DISPLAY_VALUE        GPIO_HIGH

/*****************************************************************************************
 * lcd  devices
 * author: zyw@rock-chips.com
 *****************************************************************************************/
//#ifdef  CONFIG_LCD_TD043MGEA1
#define LCD_TXD_PIN          INVALID_GPIO
#define LCD_CLK_PIN          INVALID_GPIO
#define LCD_CS_PIN           INVALID_GPIO
/*****************************************************************************************
* frame buffe  devices
* author: zyw@rock-chips.com
*****************************************************************************************/
#define FB_ID                       0
#define FB_DISPLAY_ON_PIN           RK29_PIN6_PD0   //VIP RK29_PIN6_PD1 //INVALID_GPIO // RK29_PIN6_PD0
#define FB_LCD_STANDBY_PIN          RK29_PIN6_PD1   //VIP RK29_PIN6_PD0 // RK29_PIN6_PD1
#define FB_LCD_CABC_EN_PIN          RK29_PIN6_PD2   //VIP INVALID_GPIO // RK29_PIN6_PD2
#define FB_MCU_FMK_PIN              INVALID_GPIO

#define FB_DISPLAY_ON_VALUE         GPIO_HIGH
#define FB_LCD_STANDBY_VALUE        GPIO_HIGH


//Gsensor
#define MMA8452_INT_PIN   RK29_PIN0_PA3

/**********************************************************************************************
 *
 *									WIFI/BT	
 *
 *********************************************************************************************/
//#define GPIO_WIFI_POWER       RK29_PIN6_PC0
#define GPIO_WIFI_POWER         RK29_PIN5_PD6
#define GPIO_WIFI_RESET         RK29_PIN6_PC0

//#define RK29SDK_WIFI_BT_GPIO_POWER_N       RK29_PIN5_PD6
//#define RK29SDK_WIFI_GPIO_RESET_N          RK29_PIN6_PC0
//#define RK29SDK_BT_GPIO_RESET_N            RK29_PIN6_PC4

#define RK29SDK_WIFI_SDIO_CARD_DETECT_N    RK29_PIN1_PD6

#define RK29SDK_WIFI_BT_GPIO_POWER_N       RK29_PIN5_PD6
#define RK29SDK_WIFI_GPIO_RESET_N          RK29_PIN6_PC0
#define RK29SDK_BT_GPIO_RESET_N            RK29_PIN6_PC4

#define BT_GPIO_POWER           RK29_PIN5_PD6
#define BT_GPIO_RESET          	RK29_PIN6_PC4
#define BT_GPIO_WAKE_UP         RK29_PIN6_PC5
#define BT_GPIO_WAKE_UP_HOST    //RK2818_PIN_PA7

/***************************************************
 *
 *				    LCD  
 *
 **************************************************/
//#define LCD_TXD_PIN          INVALID_GPIO
//#define LCD_CLK_PIN          INVALID_GPIO
//#define LCD_CS_PIN           INVALID_GPIO
//
//#define FB_ID                       0
//#define FB_DISPLAY_ON_PIN           RK29_PIN6_PD1
//#define FB_LCD_STANDBY_PIN          RK29_PIN6_PD0
//#define FB_LCD_CABC_EN_PIN          INVALID_GPIO
//#define FB_MCU_FMK_PIN              INVALID_GPIO

//#define FB_DISPLAY_ON_VALUE         GPIO_HIGH
//#define FB_LCD_STANDBY_VALUE        GPIO_HIGH

/* Base */
//#define OUT_TYPE		SCREEN_RGB
//#define OUT_FACE		OUT_P888
//#define OUT_CLK			 40000000
//#define LCDC_ACLK       150000000     //29 lcdc axi DMA 撞楕

/* Timing */
//#define H_PW			1
//#define H_BP			46
//#define H_VD			800
//#define H_FP			210

//#define V_PW			3
//#define V_BP			23
//#define V_VD			600
//#define V_FP			12

/* Other */
//#define DCLK_POL		0
//#define SWAP_RB			0

/***************************************************
 *
 *				     BACKLIGHG
 *
 **************************************************/
#define PWM_ID            0
#define PWM_MUX_NAME      GPIO1B5_PWM0_NAME
#define PWM_MUX_MODE      GPIO1L_PWM0
#define PWM_MUX_MODE_GPIO GPIO1L_GPIO1B5
#define PWM_EFFECT_VALUE  0
#define PWM_GPIO		RK29_PIN1_PB5

/**the value of MIN_BACKLIGHT_SCALE must be between 0~10*/
#define MIN_BACKLIGHT_SCALE	12



//#define LCD_DISP_ON_PIN

//#ifdef  LCD_DISP_ON_PIN
//#define BL_EN_MUX_NAME    GPIOF34_UART3_SEL_NAME
//#define BL_EN_MUX_MODE    IOMUXB_GPIO1_B34
//
//#define BL_EN_PIN         GPIO0L_GPIO0A5
//#define BL_EN_VALUE       GPIO_HIGH
//#endif

/***************************************************
 *
 *                      BATTERY 
 *
 **************************************************/
#define DC_DET_EFFECTIVE		1
#define CHG_OK_EFFECTIVE		1
#define GPIO_DC_DET			RK29_PIN4_PA1
#define GPIO_CHG_OK			RK29_PIN4_PA3
#define ADC_ADD_VALUE		1
#define ADC_CLI_VALUE		50
#define CHARGE_FULL_GATE 		4150

//This parameter is for new battery driver//
#define	TIMER_MS_COUNTS		            50	//timers length(ms)

#define	SLOPE_SECOND_COUNTS	            15	//time interval(s) for computing voltage slope
#define	DISCHARGE_MIN_SECOND	        60	//minimum time interval for discharging 1% battery
#define	CHARGE_MIN_SECOND	            90	//minimum time interval for charging 1% battery
#define	CHARGE_MID_SECOND	            160	//time interval for charging 1% battery when battery capacity over 80%
#define	CHARGE_MAX_SECOND	            220	//max time interval for charging 1% battery
#define CHARGE_FULL_DELAY_TIMES         10  //delay time when charging FULL
#define USBCHARGE_IDENTIFY_TIMES        5   //time for identifying USB and Charge
#define STABLE_SECOND					8  //check ok µçÆ½»á»Î¶¯¡£¡£
#define SHUTDOWN_SECOND					20
#define SPEEDLOSE_SECOND                120 //play game rapid down

#define	NUM_VOLTAGE_SAMPLE	            ((SLOPE_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)	//samling numbers
#define	NUM_DISCHARGE_MIN_SAMPLE	    ((DISCHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	
#define	NUM_CHARGE_MIN_SAMPLE	        ((CHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define	NUM_CHARGE_MID_SAMPLE	        ((CHARGE_MID_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define	NUM_CHARGE_MAX_SAMPLE	        ((CHARGE_MAX_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define NUM_CHARGE_FULL_DELAY_TIMES     ((CHARGE_FULL_DELAY_TIMES * 1000) / TIMER_MS_COUNTS)	
#define NUM_USBCHARGE_IDENTIFY_TIMES    ((USBCHARGE_IDENTIFY_TIMES * 1000) / TIMER_MS_COUNTS)	
#define NUM_STABLE_SAMPLE				((STABLE_SECOND * 1000) / TIMER_MS_COUNTS)
#define NUM_SHUTD0WN_SAMPLE             ((SHUTDOWN_SECOND * 1000) / TIMER_MS_COUNTS)
#define NUM_SPEEDLOSE_SAMPLE  			((SPEEDLOSE_SECOND * 1000) / TIMER_MS_COUNTS)

#define BAT_2V5_VALUE	        2500
#define BATT_MAX_VOL_VALUE	    4190	//voltage of FULL battery
#define	BATT_ZERO_VOL_VALUE     3500	//voltage when poweroff
#define BATT_NOMAL_VOL_VALUE    3800
#define SHUTDOWNVOLTAGE			3400
//define  divider resistors for ADC sampling, units as K
#define BAT_PULL_UP_R           549
#define BAT_PULL_DOWN_R         200


/*Inizio Sezione per Mediacom 810c ****************
 *
 *                    USB
 *
 **************************************************/
#define GPIO_USB_INT			 RK29_PIN0_PA0
#define MASS_STORAGE_NAME "M-MP810C"
#define MASS_STORAGE_PRODUCT ""
#define USB_PRODUCT_ID			0x2910
#define ADB_PRODUCT_ID			0x0c02
#define VENDOR_ID				0x0bb4
#define ADB_PRODUCT_NAME		"rk2918"
#define ADB_MANUFACTURE_NAME	"RockChip"



