/*
 * rk29_M-MP850i.h
 *
 * Overview:  
 *
 * Copyright (c) 2012, ...
 *
 * Version:  1.0
 * Created:  ...
 * Author:   ...
 * Company:  ...
 * History:
 *
 * 
 */

// IRDA
#define BU92747GUW_RESET_PIN         INVALID_GPIO //RK29_PIN3_PD4// INVALID_GPIO //
#define BU92747GUW_RESET_MUX_NAME    INVALID_GPIO //GPIO3D4_HOSTWRN_NAME//NULL //
#define BU92747GUW_RESET_MUX_MODE    INVALID_GPIO //GPIO3H_GPIO3D4//NULL //

#define BU92747GUW_PWDN_PIN          INVALID_GPIO //RK29_PIN3_PD3//RK29_PIN5_PA7 //
#define BU92747GUW_PWDN_MUX_NAME     INVALID_GPIO //GPIO3D3_HOSTRDN_NAME//GPIO5A7_HSADCDATA2_NAME //
#define BU92747GUW_PWDN_MUX_MODE     INVALID_GPIO //GPIO3H_GPIO3D3//GPIO5L_GPIO5A7  //





/**********************************************************************************************
 *
 *									TOUCHSCREEN
 *
 *********************************************************************************************/
#define TOUCH_SCREEN_STANDBY_PIN		INVALID_GPIO    //VIP RK29_PIN6_PD1
#define TOUCH_SCREEN_STANDBY_VALUE		GPIO_HIGH
#define TOUCH_SCREEN_DISPLAY_PIN		INVALID_GPIO
#define TOUCH_SCREEN_DISPLAY_VALUE		GPIO_HIGH

/**********************************************************************************************
 *
 *										TOUCH PANEL
 *
 *********************************************************************************************/
#define DEBOUNCE_REPTIME  				3
#define USE_TP_KEY						1
#define TOUCH_POWER_PIN					RK29_PIN6_PB0
#define TOUCH_RESET_PIN					RK29_PIN6_PC3
#define TOUCH_INT_PIN					RK29_PIN0_PA2
#define TOUCH_USE_I2C2					1
#define TOUCH_KEY_LED					INVALID_GPIO //RK29_PIN6_PA6

#define	TP_USE_WAKEUP_PIN

#define FT5X0X_I2C_SPEED 				100*1000

#define SCREEN_BOUNDARY_ADJUST_VALUE 	10

//#define TOUCH_POWER_PIN				RK29_PIN6_PB0
//#define TOUCH_RESET_PIN				RK29_PIN6_PC3
//#define TOUCH_INT_PIN					RK29_PIN0_PA2
//#define TOUCH_USE_I2C2				1
//#define TOUCH_KEY_LED					RK29_PIN6_PA6
//#define TOUCHKEY_ON_SCREEN

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

#define RK29SDK_WIFI_SDIO_CARD_DETECT_N    INVALID_GPIO //RK29_PIN1_PD6

#define RK29SDK_WIFI_BT_GPIO_POWER_N       INVALID_GPIO //RK29_PIN5_PD6
#define RK29SDK_WIFI_GPIO_RESET_N          INVALID_GPIO //RK29_PIN6_PC0
#define RK29SDK_BT_GPIO_RESET_N            INVALID_GPIO //RK29_PIN6_PC4

#define BT_GPIO_POWER           INVALID_GPIO //RK29_PIN5_PD6
#define BT_GPIO_RESET          	INVALID_GPIO //RK29_PIN6_PC4
#define BT_GPIO_WAKE_UP         INVALID_GPIO //RK29_PIN6_PC5
#define BT_GPIO_WAKE_UP_HOST    //RK2818_PIN_PA7

#define IOMUX_BT_GPIO_POWER     rk29_mux_api_set(GPIO5D6_SDMMC1PWREN_NAME, GPIO5H_GPIO5D6);
#define IOMUX_BT_GPIO_WAKE_UP_HOST() //rk2818_mux_api_set(GPIOA7_FLASHCS3_SEL_NAME,0);

#define BT_WAKE_LOCK_TIMEOUT    10 //s

#define WIFI_EXT_POWER         	RK29_PIN5_PD6

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
#define H_VD			1024
//#define H_FP			210

//#define V_PW			3
//#define V_BP			23
#define V_VD			768
//#define V_FP			12

/* Other */
//#define DCLK_POL		0
//#define SWAP_RB			0

/***************************************************
 *
 *				     BACKLIGHG
 *
 **************************************************/
#define PWM_ID				0
#define PWM_MUX_NAME		GPIO1B5_PWM0_NAME
#define PWM_MUX_MODE		GPIO1L_PWM0
#define PWM_MUX_MODE_GPIO	GPIO1L_GPIO1B5
#define PWM_EFFECT_VALUE	0
#define PWM_GPIO			RK29_PIN1_PB5

/**the value of MIN_BACKLIGHT_SCALE must be between 0~10*/
#define MIN_BACKLIGHT_SCALE	12

/**********************************************************************************************
 *
 *							PWM VOLTAGE REGULATOR
 *
 *********************************************************************************************/
#define REGULATOR_PWM_ID					2
#define REGULATOR_PWM_MUX_NAME      		GPIO2A3_SDMMC0WRITEPRT_PWM2_NAME
#define REGULATOR_PWM_MUX_MODE				GPIO2L_PWM2
#define REGULATOR_PWM_MUX_MODE_GPIO			GPIO2L_GPIO2A3
#define REGULATOR_PWM_GPIO					RK29_PIN2_PA3

#define TPS65910_HOST_IRQ					RK29_PIN0_PA1 //test

/**********************************************************************************************
 *
 *										RTC
 *
 *********************************************************************************************/
#define GPIO_RTC_INT			 RK29_PIN0_PA1

/***************************************************
 *
 *                      AUDIO
 *
 **************************************************/
//#define RTL5631_HP_HIGH
#define GPIO_SPK_CON			RK29_PIN6_PB6
//#define RT5631_DEF_VOL					0xc1
//#define RT5631_DEF_VOL_SPK				0xc8
#define RT5631_ADC_GAIN				0x0006
#define RT5631_DEF_VOL_ADC		0x6600
#define RT5631_DEF_HP_EQ				NORMAL
//#define NO_OFF_SPK_WHILE_REC

#define RT5631_ENABLE_ALC_DAC 1
#define RT5631_ALC_DAC_FUNC_ENA 1	//ALC functio for DAC
#define RT5631_SPK_TIMER	0	//if enable this, MUST enable RT5631_EQ_FUNC_ENA first!

#define RT5631_DEF_VOL					0xd4   //0xd4 -30dB 0xc0 0dB
#define RT5631_DEF_VOL_SPK			0xc4
/**********************************************************************************************
 *
 *										GSENSOR
 *
 *********************************************************************************************/
#define MMA8452_INT_PIN			RK29_PIN0_PA3

/**********************************************************************************************
 *
 *										USB
 *
 *********************************************************************************************/
#define GPIO_USB_INT			    RK29_PIN0_PA0
#define MASS_STORAGE_NAME			"MP850i"
#define MASS_STORAGE_PRODUCT		""
#define USB_PRODUCT_ID				0x2910
#define ADB_PRODUCT_ID				0x0c02
#define VENDOR_ID					0x0bb4
#define ADB_PRODUCT_NAME			"rk2918"
#define ADB_MANUFACTURE_NAME		"RockChip"

/**********************************************************************************************
 *
 *										HDMI
 *
 *********************************************************************************************/
#define ANX7150_ATTACHED_BUS1
#define GPIO_HDMI_DET				RK29_PIN1_PD7
#define	GPIO_ANX7150_RST			RK29_PIN2_PC7
#define ANX7150_RST_MUX_NAME		GPIO2C7_SPI1RXD_NAME
#define ANX7150_RST_MUX_MODE		GPIO2H_GPIO2C7

/**********************************************************************************************
 *
 *										SDMMC
 *
 *********************************************************************************************/
#define SDMMC_POWER_PIN				RK29_PIN5_PD5
#define SDMMC_DET_PIN				RK29_PIN2_PA2


/**********************************************************************************************
 *
 *									BATTERY
 *
 *********************************************************************************************/
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
#define BAT_PULL_UP_R           1300
#define BAT_PULL_DOWN_R         475


/***************************************************
 *
 *                  CAMERA SENSOR
 *
 **************************************************/
#define CONFIG_SENSOR_0			RK29_CAM_SENSOR_BCAM		/* back camera sensor */
#define CONFIG_SENSOR_IIC_ADDR_0	0xef

/*

#ifdef CONFIG_VIDEO_RK29
#define CONFIG_SENSOR_0 RK29_CAM_SENSOR_GT2005_BACK  // back camera sensor
#define CONFIG_SENSOR_IIC_ADDR_0 	    0x78
#define CONFIG_SENSOR_IIC_ADAPTER_ID_0    1
#define CONFIG_SENSOR_ORIENTATION_0       90
#define CONFIG_SENSOR_POWER_PIN_0         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_0         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_0       RK29_PIN6_PB7
#define CONFIG_SENSOR_FALSH_PIN_0         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_0 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_0 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_0 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_0 RK29_CAM_FLASHACTIVE_L
#define CONFIG_SENSOR_QCIF_FPS_FIXED_0      15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_0      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_0       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_0       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_0      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_0      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_0      30000



#define CONFIG_SENSOR_1 RK29_CAM_SENSOR_GC0308	//
#define CONFIG_SENSOR_IIC_ADDR_1 	    0x6C
#define CONFIG_SENSOR_IIC_ADAPTER_ID_1    1
#define CONFIG_SENSOR_ORIENTATION_1       270
#define CONFIG_SENSOR_POWER_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_1       RK29_PIN5_PD7
#define CONFIG_SENSOR_FALSH_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_1 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_1 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_1 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_1 RK29_CAM_FLASHACTIVE_L
#define NT99250_X_OFFSET	2
#define CONFIG_SENSOR_QCIF_FPS_FIXED_1      15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_1      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_1       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_1       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_1      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_1      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_1      30000
*/
