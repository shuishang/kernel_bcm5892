/*****************************************************************************
* Copyright 2003 - 2008 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/


/*
*
*   panel_lgp_LB040Q03.c
*
*   This file contains code to initialize the LG-Phillips LB040Q03 panel
*   for use by an LCD controller.
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <cfg_global.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/version.h>

#include <linux/broadcom/hw_cfg.h>
#include <linux/broadcom/gpio.h>

#include <asm/io.h>

/*
 * ---- Public Variables ------------------------------------------------- 
 * ---- Private Constants and Types -------------------------------------- 
 */

static char gBanner[] __initdata = KERN_INFO "LG-Phillips LB040Q03 Panel: 0.1\n";

#define LCD_CMD_END             0
#define LCD_CMD_SET_REG         1
#define LCD_CMD_WAIT_USEC       2
#define LCD_CMD_WAIT_MSEC       3

typedef struct
{
    uint8_t     cmd;
    uint8_t     reg;
    uint16_t    val;

} lcd_cmd_t;

#define LCD_END_SEQUENCE            { LCD_CMD_END }
#define LCD_SET_REG( reg,  val )    { LCD_CMD_SET_REG, (reg), (val) }
#define LCD_WAIT_USEC(usec)         { LCD_CMD_WAIT_USEC, 0, (usec) }
#define LCD_WAIT_MSEC(msec)         { LCD_CMD_WAIT_MSEC, 0, (msec) }

/* Definitions for the LCD Panel */

#define DEVICE_ID  0x70   /*'011100xx'b where xx is 'RS:R/W' */

/*
 * R/W = 0 write, 1 read 
 * RS = 0 idx register write or SR read, 1 instruction write   
 */

#define FIRST_BYTE_R	( DEVICE_ID | 0x00 ) 	/* first byte for the index register setting,	RS=0, R/W=0 */
#define FIRST_BYTE_INST ( DEVICE_ID | 0x02 )    /* first byte for writting instruction,	        RS=1, R/W=0 */

/* ---- Private Variables ------------------------------------------------ */

static  const lcd_cmd_t   gPowerOnSequence[] =
{
    LCD_WAIT_MSEC( 20 ),

    /*
     * Power On reset / Display Off state 
  *   LCD_SET_REG( 0x0a, 0x0000 ), /*	GON=0, POC=0 */
  *   LCD_SET_REG( 0x0a, 0x2000 ), /*	GXEN=1 */
  */

    LCD_WAIT_MSEC( 20 ),

    /* Power Supply Setting 1 */
    LCD_SET_REG( 0x0a, 0x6000 ), /*	EXM=1 */
    LCD_SET_REG( 0x0b, 0x0000 ), /*	VCOMG=0, VDV, VCM */

    /* Power Supply Setting 2 */
    LCD_SET_REG( 0x0a, 0x6155 ), /*	HSM/SAP/AP */
    
    LCD_WAIT_MSEC( 70 ),   /* More than 50 msec */
    
    /* Instruction Setting */
    LCD_SET_REG(  1, 0x409D ), 	
    LCD_SET_REG(  2, 0x0204 ),	
    LCD_SET_REG(  3, 0x0100 ),	
    LCD_SET_REG(  4, 0x4008 ),	
    LCD_SET_REG(  5, 0x4013 ),	
    LCD_SET_REG(  6, 0x0000 ),	
    LCD_SET_REG(  7, 0x0023 ),	
    LCD_SET_REG(  8, 0x0021 ),	
    LCD_SET_REG(  9, 0x0800 ),	
    LCD_SET_REG( 10, 0x0210 ),	/* 0x0210 ?? */
    LCD_SET_REG( 11, 0x0003 ),	
    LCD_SET_REG( 12, 0x100E ),	
    LCD_SET_REG( 13, 0x1D06 ),	
    LCD_SET_REG( 14, 0x0707 ),	
    LCD_SET_REG( 15, 0x0003 ),	
    LCD_SET_REG( 16, 0x0407 ),	
    LCD_SET_REG( 17, 0x0006 ),	
    LCD_SET_REG( 18, 0x0007 ),	
    LCD_SET_REG( 19, 0x0300 ),	
    
    LCD_WAIT_MSEC( 70 ),   /* More than 50 msec */
    
    /* Display On Sequence */

    LCD_SET_REG( 0x0a, 0x6B55), /* GON=1 */
    LCD_SET_REG(    5, 0x5093), /* POC=1 */

    LCD_END_SEQUENCE
};

static  const lcd_cmd_t   gPowerOffSequence[] =
{
    LCD_SET_REG( 0x0a, 0x6B55),
    LCD_SET_REG( 0x05, 0x5093),

    LCD_SET_REG( 0x0B, 0x0000),

    LCD_WAIT_MSEC( 1 ), /* At least 1 msec */

    LCD_SET_REG( 0x0A, 0x4000 ),

    LCD_END_SEQUENCE
};

/*
 * ---- Private Function Prototypes -------------------------------------- 
 * ---- Functions -------------------------------------------------------- 
 */

static void WriteSpi24( uint32_t val24 )
{
    int         bitIndex;

    /* Start the Transfer */

    gpio_set_value( HW_CFG_LCD_SPI_CS, 0 );
    udelay( 1 );

    /* Write out 24 bits for the Device ID, RS bit, R/W bit, and register index */

    for ( bitIndex = 23; bitIndex >= 0; bitIndex-- )
    {
        gpio_set_value( HW_CFG_LCD_SPI_CLK, 0 );
        gpio_set_value( HW_CFG_LCD_SPI_DATA, ( val24 >> bitIndex  ) & 1 );
        udelay( 2 );
        gpio_set_value( HW_CFG_LCD_SPI_CLK, 1 );
        udelay( 2 );
    }
    gpio_set_value( HW_CFG_LCD_SPI_CLK, 0 );
    udelay( 2 );
    gpio_set_value( HW_CFG_LCD_SPI_CLK, 1 );
    gpio_set_value( HW_CFG_LCD_SPI_DATA, 1 );
    udelay( 1 );
    gpio_set_value( HW_CFG_LCD_SPI_CS, 1 );
    udelay( 4 );

} /* WriteSpi24 */

/****************************************************************************
*
*  SetSpiRegister
*
*     Writes a vlue into a register.
*
****************************************************************************/

static void SetSpiRegister( uint8_t reg, uint16_t val )
{
    WriteSpi24(( FIRST_BYTE_R    << 16 ) | reg );
    WriteSpi24(( FIRST_BYTE_INST << 16 ) | val );
    
} /* SetSpiRegister */

/****************************************************************************
*
*  SendCommandSequence
*
*     Sends a sequence of commands to the LCD panel
*
****************************************************************************/

static void SendCommandSequence( const lcd_cmd_t *seq )
{
    const   lcd_cmd_t   *cmd = seq;

    while ( cmd->cmd != LCD_CMD_END )
    {
        switch ( cmd->cmd )
        {
            case LCD_CMD_SET_REG:
            {
                SetSpiRegister( cmd->reg, cmd->val );
                break;
            }

            case LCD_CMD_WAIT_USEC:
            {
                udelay( cmd->val );
                break;
            }

            case LCD_CMD_WAIT_MSEC:
            {
                mdelay( cmd->val );
                break;
            }
        }
        cmd++;
    }

} /* SendCommandSequence */

/****************************************************************************
*
*  PowerOnPanel
*
*     Powers on the LCD Panel
*
****************************************************************************/

static void PowerOnPanel( void )
{
    /* Turn on the voltage converters on the LCD Daughtercard */

    gpio_set_value( HW_CFG_LCD_BUF_ENABLE, 0 );
    mdelay( 10 );

    /* Turn on the 3.3v power */

    gpio_set_value( HW_CFG_LCD_POWER_ENABLE, 0 );
    udelay( 15 );

    /* Take it out of Reset */

    gpio_set_value( HW_CFG_LCD_RESET, 1 );
    udelay( 15 );

    /* Send the initialization sequence */

    SendCommandSequence( gPowerOnSequence );

} /* PowerOnPanel */

/****************************************************************************
*
*  PowerOffPanel
*
*     Powers off the LCD Panel
*
****************************************************************************/

static void PowerOffPanel( void )
{
    SendCommandSequence( gPowerOffSequence );

    /* Turn off the 3.3v supply */

    gpio_set_value( HW_CFG_LCD_POWER_ENABLE, 1 );

    mdelay( 10 );

    gpio_set_value( HW_CFG_LCD_BUF_ENABLE, 1 );

} /* PowerOffPanel */

/****************************************************************************
*
*  panel_init_gpio_output
*
*     Initialization function called when the module is loaded.
*
****************************************************************************/

static int panel_init_gpio_output( unsigned gpio, int value, char *label )
{
    int     rc;

    if (( rc = gpio_request( gpio, label )) < 0 )
    {
        printk( KERN_ERR "display-lgp-LB040Q03: Unable to request gpio pin %d for '%s'\n", gpio, label );
        return rc;
    }
    gpio_direction_output( gpio, value );

    return 0;
}

/****************************************************************************
*
*  panel_init
*
*     Initialization function called when the module is loaded.
*
****************************************************************************/

#define INIT_GPIO( gpio, value, label ) \
    if (( rc = panel_init_gpio_output( gpio, value, label )) < 0 ) \
    { \
        return rc; \
    }

static int __init panel_init( void )
{
    int     rc;

    printk( gBanner );

    /* Configure the direction and initial value of the various GPIO pins */

    /* The HW_CFG_LCD_RESET line is connected to the RESET lin on the LB040Q03 */

    INIT_GPIO( HW_CFG_LCD_RESET, 0, "LCD Reset" );

    /*
     * The HWFG_LCD_POWER_ENABLE line connects to a MOSFET on the LCD daughtercard. 
     * It's an active low signal which is used to control the VDD line being fed 
     * to the panel 
     */

    INIT_GPIO( HW_CFG_LCD_POWER_ENABLE, 1, "LCD Power Enable" );

    /*
     * The HW_CFG_LCD_BUF_ENABLE line is an active low signal which enables 
     * the voltage translation buffers on the LCD daughtercard. 
     */
    
    INIT_GPIO( HW_CFG_LCD_BUF_ENABLE, 1, "LCD Buffer Enable" );

    /* The HW_CFG_SPI_CS, CLK & DATA connects to the /CS, SCL< and SDI signals on the panel. */

    INIT_GPIO( HW_CFG_LCD_SPI_CS, 1, "LCD SPI-Chip-Select" );
    INIT_GPIO( HW_CFG_LCD_SPI_CLK, 1, "LCD SPI-Clock" );
    INIT_GPIO( HW_CFG_LCD_SPI_DATA, 1,  "LCD SPI-Data" );

    PowerOnPanel();

   return 0;

} /* panel_init */

/****************************************************************************
*
*  panel_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
****************************************************************************/

static void __exit panel_exit( void )
{
    PowerOffPanel();

} /* panel_exit */

/****************************************************************************/

module_init(panel_init);
module_exit(panel_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("LG-Phillips LB040Q03 Panel Initialization");
MODULE_LICENSE("GPL");

