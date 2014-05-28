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
*****************************************************************************
*
*  reg_gpio.h
*
*  PURPOSE:
*
*     This file contains definitions for the GPIO registers.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( __ASM_ARCH_REG_GPIO_H )
#define __ASM_ARCH_REG_GPIO_H

/* ---- Include Files ---------------------------------------------------- */

#include <asm/types.h>
#include <mach/hardware.h>
#include <linux/broadcom/gpio_types.h>
#include <linux/spinlock.h>

#include "gpio.h"

#include "bcm5892_reg.h"
#include "irqs.h"

HW_EXTERN_SPINLOCK(Gpio)

#define BCM5892_GPIO_DEBUG_EN 0

#if BCM5892_GPIO_DEBUG_EN
	#define BCM5892_GPIO_DEBUG(fmt, args...) printk(KERN_INFO fmt, ## args)
#else
    #define BCM5892_GPIO_DEBUG(fmt, args...)
#endif


/* GPIO group base, from bcm5892_reg.h */
/* There are 5 GPIO groups in 5892 */
#define	HW_GPIO0_PHY_BASE	GIO0_REG_BASE_ADDR
#define	HW_GPIO1_PHY_BASE	GIO1_REG_BASE_ADDR
#define	HW_GPIO2_PHY_BASE	GIO2_REG_BASE_ADDR
#define	HW_GPIO3_PHY_BASE	GIO3_REG_BASE_ADDR
#define	HW_GPIO4_PHY_BASE	GIO4_REG_BASE_ADDR


#define HW_GPIO0_NUM_PIN    23
#define HW_GPIO1_NUM_PIN    32
#define HW_GPIO2_NUM_PIN    30
#define HW_GPIO3_NUM_PIN    19
#define HW_GPIO4_NUM_PIN    10 /* If we change any of these number, need to also change ARCH_NR_GPIOS defined in gpio.h */

#define HW_GPIO0_PIN_MAX    HW_GPIO0_NUM_PIN                       /* 23 */
#define HW_GPIO1_PIN_MAX    (HW_GPIO0_PIN_MAX + HW_GPIO1_NUM_PIN)  /* 55 */
#define HW_GPIO2_PIN_MAX    (HW_GPIO1_PIN_MAX + HW_GPIO2_NUM_PIN)  /* 85 */
#define HW_GPIO3_PIN_MAX    (HW_GPIO2_PIN_MAX + HW_GPIO3_NUM_PIN)  /* 104 */
#define HW_GPIO4_PIN_MAX    (HW_GPIO3_PIN_MAX + HW_GPIO4_NUM_PIN)  /* 114 */


/* Maps PIN to GPIO group base register address */
#define HW_GPIO_PIN_TO_GROUPBASE(pin)    ((pin < HW_GPIO0_PIN_MAX) ? HW_GPIO0_PHY_BASE : \
                                              ((pin < HW_GPIO1_PIN_MAX) ? HW_GPIO1_PHY_BASE : \
                                              ((pin < HW_GPIO2_PIN_MAX) ? HW_GPIO2_PHY_BASE : \
                                              ((pin < HW_GPIO3_PIN_MAX) ? HW_GPIO3_PHY_BASE : \
                                              HW_GPIO4_PHY_BASE))))

/* Maps PIN to bit number offset of the GPIO group it belongs to */
#define HW_GPIO_PIN_TO_BIT_OFFSET(pin)   ((pin < HW_GPIO0_PIN_MAX) ? pin : \
                                         ((pin < HW_GPIO1_PIN_MAX) ? (pin - HW_GPIO0_PIN_MAX) : \
                                         ((pin < HW_GPIO2_PIN_MAX) ? (pin - HW_GPIO1_PIN_MAX) : \
                                         ((pin < HW_GPIO3_PIN_MAX) ? (pin - HW_GPIO2_PIN_MAX) : \
                                         (pin - HW_GPIO3_PIN_MAX)))))

/* Maps GPIO Group number to group base address */
#define HW_GPIO_NUM_TO_GROUPBASE(nGrp)   ((nGrp == 0) ? HW_GPIO0_PHY_BASE : \
                                              ((nGrp == 1) ? HW_GPIO1_PHY_BASE : \
                                              ((nGrp == 2) ? HW_GPIO2_PHY_BASE : \
                                              ((nGrp == 3) ? HW_GPIO3_PHY_BASE : \
                                              HW_GPIO4_PHY_BASE))))

#define HW_GPIO_SUBGROUP_SHIFT  8 /* subgroup is left shift 8 bits, [10:8] */
#define HW_GPIO_SUBGROUP_NONE   8 /* 0x...800 means no subgroup */



/* ---- Constants and Types ---------------------------------------------- */

#define DBG_DEFAULT_LEVEL 1

/* --------------------------------------------------------------------------
** This defines the public GPIO API defined in /include/linux/broadcom/gpio.h.
*/

typedef enum
{
    GPIO_PIN_TYPE_INPUT                   = 0x00,
    GPIO_PIN_TYPE_INPUT_WITH_INTERRUPT    = 0x01,
    GPIO_PIN_TYPE_OUTPUT                  = 0x02,
    GPIO_PIN_TYPE_ALTERNATIVE_FUNC0       = 0x03,
    GPIO_PIN_TYPE_ALTERNATIVE_FUNC1       = 0x04,
    GPIO_PIN_TYPE_ALTERNATIVE_DISABLE     = 0x05, /* change from alternative function back to GPIO */
} GPIO_PIN_TYPE;


#define gpio_get_pin_type  reg_gpio_iotr_get_pin_type
#define gpio_set_pin_type  reg_gpio_iotr_set_pin_type
#define gpio_set_pin_val   reg_gpio_set_pin
#define gpio_get_pin_val   reg_gpio_get_pin

/* SubGroup defines */

/* For our internal define, we use:
   [31:28] is group number (0-4). We add 1 to Group defines so it's not 0, otherwise confuses with normal GPIO pin number
   [27:23] is the first subgroup number (0-7). Still add 1. Note this is 5 bits
   [22:18] is the second subgroup number (0-7). Still add 1
   We have 2 subgroup numbers for one Aux function, since one AUX function can occupy 2 subgroups
   In the future if 3 subgroups are needed we can use the next 5-bits.
   Note in the case of 2 subgroups, the smallest one goes first, like 0&1, 0 goes for field1 and 1 goes for field2
*/
#define GPIO_GROUP(n)             ((n+1) << 28)
#define GPIO_SUBGROUP_FIELD1(n)   ((n+1) << 23)
#define GPIO_SUBGROUP_FIELD2(n)   ((n+1) << 18)

#define GPIO_AUX_TO_GROUP(n)       ((n>>28) - 1)
#define GPIO_AUX_TO_SUBGROUP1(n)   (((n>>23) & 0xF) - 1) /* values at 0 - 7 */
#define GPIO_AUX_TO_SUBGROUP2(n)   (((n>>18) & 0xF) - 1)

#define GPIO_AUX_IS_SUBGROUP2_EXIST(n) ((n>>18) & 0xF) /* do not minus 1 so 0 means there's no subgroup2 */

#define ALL_BITS  0xFFFFFFFF

#if ARCH_NR_GPIOS > GPIO_GROUP(0)
#error "Too much GPIOs"
#endif

typedef enum /* It's ok to have defines have same value */
{
	GPIO_AUX_TS         = ( GPIO_GROUP(0) | GPIO_SUBGROUP_FIELD1(0) ), /* Group 0, subgroup 0 */
	GPIO_AUX_SPI0       = ( GPIO_GROUP(0) | GPIO_SUBGROUP_FIELD1(1) ), /* Group 0, subgroup 1 */
	GPIO_AUX_SPI1       = ( GPIO_GROUP(0) | GPIO_SUBGROUP_FIELD1(2) ), /* Group 0, subgroup 2 */
	GPIO_AUX_SCI0       = ( GPIO_GROUP(0) | GPIO_SUBGROUP_FIELD1(3) ), /* Group 0, subgroup 3 */
	GPIO_AUX_I2C0       = ( GPIO_GROUP(0) | GPIO_SUBGROUP_FIELD1(3) ), /* Group 0, subgroup 3, AUX=1 */
	GPIO_AUX_UART0      = ( GPIO_GROUP(0) | GPIO_SUBGROUP_FIELD1(4) ), /* Group 0, subgroup 4 */

	GPIO_AUX_EINT0      = ( GPIO_GROUP(1) | GPIO_SUBGROUP_FIELD1(0) ), /* Group 1, subgroup 0 */
	GPIO_AUX_EINT1      = ( GPIO_GROUP(1) | GPIO_SUBGROUP_FIELD1(1) ), /* Group 1, subgroup 1 */
	GPIO_AUX_EINT2      = ( GPIO_GROUP(1) | GPIO_SUBGROUP_FIELD1(2) ), /* Group 1, subgroup 2 */
	GPIO_AUX_EINT3      = ( GPIO_GROUP(1) | GPIO_SUBGROUP_FIELD1(3) ), /* Group 1, subgroup 3 */
	GPIO_AUX_EINT4_7    = ( GPIO_GROUP(1) | GPIO_SUBGROUP_FIELD1(4) ), /* Group 1, subgroup 4 */
	GPIO_AUX_EINT8_15   = ( GPIO_GROUP(1) | GPIO_SUBGROUP_FIELD1(5) ), /* Group 1, subgroup 5 */
	GPIO_AUX_EINT16_19  = ( GPIO_GROUP(1) | GPIO_SUBGROUP_FIELD1(6) ), /* Group 1, subgroup 6 */
	GPIO_AUX_EINT20_23  = ( GPIO_GROUP(1) | GPIO_SUBGROUP_FIELD1(7) ), /* Group 1, subgroup 7 */

	GPIO_AUX_SMC        = ( GPIO_GROUP(2) | GPIO_SUBGROUP_FIELD1(0) ), /* Group 2, subgroup 0,   AUX=1 */
	GPIO_AUX_SPI3       = ( GPIO_GROUP(2) | GPIO_SUBGROUP_FIELD1(1) ), /* Group 2, subgroup 1,   AUX=1 */
	GPIO_AUX_LCD        = ( GPIO_GROUP(2) | GPIO_SUBGROUP_FIELD1(0) | GPIO_SUBGROUP_FIELD2(1) ), /* Group 2, subgroup 0&1, AUX=0 */

	GPIO_AUX_PWM        = ( GPIO_GROUP(3) | GPIO_SUBGROUP_FIELD1(0) ), /* Group 3, subgroup 0,   AUX=0 */
	GPIO_AUX_LED        = ( GPIO_GROUP(3) | GPIO_SUBGROUP_FIELD1(0) ), /* Group 3, subgroup 0,   AUX=1 */
	GPIO_AUX_I2C1       = ( GPIO_GROUP(3) | GPIO_SUBGROUP_FIELD1(1) ), /* Group 3, subgroup 1 */
	GPIO_AUX_I2S        = ( GPIO_GROUP(3) | GPIO_SUBGROUP_FIELD1(2) ), /* Group 3, subgroup 2 */
	GPIO_AUX_UART2      = ( GPIO_GROUP(3) | GPIO_SUBGROUP_FIELD1(3) | GPIO_SUBGROUP_FIELD2(4) ), /* Group 3, subgroup 3&4, AUX=0 */
	GPIO_AUX_IRDA       = ( GPIO_GROUP(3) | GPIO_SUBGROUP_FIELD1(3) ), /* Group 3, subgroup 3,   AUX=1 */
	GPIO_AUX_UART3      = ( GPIO_GROUP(3) | GPIO_SUBGROUP_FIELD1(5) ), /* Group 3, subgroup 5,   AUX=0 */
	GPIO_AUX_D1W        = ( GPIO_GROUP(3) | GPIO_SUBGROUP_FIELD1(5) ), /* Group 3, subgroup 5,   AUX=1 */

	GPIO_AUX_PRINTER    = ( GPIO_GROUP(4) | GPIO_SUBGROUP_FIELD1(0) | GPIO_SUBGROUP_FIELD2(1) ), /* Group 4, subgroup 0&1 */
	GPIO_AUX_SPI4       = ( GPIO_GROUP(4) | GPIO_SUBGROUP_FIELD1(0) ), /* Group 4, subgroup 0 */
} GPIO_SUBGROUP_TYPE;

/*
** End of public API
--------------------------------------------------------------------------*/

/* Register offsets */
#define REGOFFSET_GPIO_IOTR         0x000 /* GPIO Data in register */
#define REGOFFSET_GPIO_DOUT         0x004 /* GPIO Data out register */
#define REGOFFSET_GPIO_EN           0x008 /* GPIO driver enable register */
#define REGOFFSET_GPIO_INT_TYPE     0x00C /* GPIO Interrupt type register */
#define REGOFFSET_GPIO_INT_DE       0x010 /* GPIO Dual edge select register */
#define REGOFFSET_GPIO_INT_EDGE     0x014 /* GPIO Interrupt edge select register */
#define REGOFFSET_GPIO_INT_MSK      0x018 /* GPIO Interrupt mask register, 1 means INT allowed */
#define REGOFFSET_GPIO_INT_STAT     0x01C /* GPIO Interrupt Status register */
#define REGOFFSET_GPIO_INT_MSTAT    0x020 /* GPIO Interrupt masked status register */
#define REGOFFSET_GPIO_INT_CLR      0x024 /* GPIO Interrupt clear register */
#define REGOFFSET_GPIO_AUX_SEL      0x028 /* GPIO Auxiliary interface select register */
#define REGOFFSET_GPIO_SCR          0x02C /* GPIO security control register */
#define REGOFFSET_GPIO_INIT_VAL     0x030 /* GPIO Initial value register (unused) */
#define REGOFFSET_GPIO_PAD_RES      0x034 /* GPIO IO Programmable resistor direction */
#define REGOFFSET_GPIO_RES_EN       0x038 /* GPIO IO Programmable resister enable */
#define REGOFFSET_GPIO_TestInput    0x03C /* 28 Bit word sent to the GPIN[$CELL:0] bus */
#define REGOFFSET_GPIO_TestOutput   0x040 /* 28 Bit word from the GPOUT[$CELL:0] bus */
#define REGOFFSET_GPIO_TestEnable   0x044 /* 28 bit enable signal to drive GP_TestInput to the GP_DIN */
#define REGOFFSET_GPIO_HYSEN_SW     0x058 /* When set this bit enables the Schmitt-trigger functionality option of the IO pad. */
#define REGOFFSET_GPIO_SLEW_SW      0x05C /* This register provides the GPIO IO pad slew controllability. Default is 1’b1. */
#define REGOFFSET_GPIO_DRV_SEL0_SW  0x060 /* used to specify the drive strength parameter of the corresponding GPIO pad. */
#define REGOFFSET_GPIO_DRV_SEL1_SW  0x064 /* used to specify the drive strength parameter of the corresponding GPIO pad. */
#define REGOFFSET_GPIO_DRV_SEL2_SW  0x068 /* used to specify the drive strength parameter of the corresponding GPIO pad. */
#define REGOFFSET_GPIO_AUX01_SEL    0x06C /* Mux select for each GPIO bit to select between two functional auxiliary inputs. */
#define REGOFFSET_GPIO_INT_POLARITY 0x070 /* Polarity definition for the GPIO interrupt detection logic. */
#define REGOFFSET_GPIO_GROUP0       0x074 /* Specifies the GPIOs which belong to GRP0. */

/* Get GPIO register address from pin number. The PIN maps to GPIO register base first */
/* No subgroup is used, e.g. subgroup field is 0x800 */
#define REG_GPIO_FROM_PIN(nPin, nRegOffset)    \
                          __REG32 (IO_ADDRESS (HW_GPIO_PIN_TO_GROUPBASE(nPin) + (HW_GPIO_SUBGROUP_NONE << HW_GPIO_SUBGROUP_SHIFT) + nRegOffset))

/* Get PIO register address from group & subgroup number */
#define REG_GPIO_FROM_SUBGROUP(nGrp, nSubGrp, nRegOffset)  \
                          __REG32 (IO_ADDRESS (HW_GPIO_NUM_TO_GROUPBASE(nGrp) + (nSubGrp << HW_GPIO_SUBGROUP_SHIFT) + nRegOffset))

/* Get PIO register address from aux number (enum above)*/
#define REG_1ST_GPIO_FROM_AUX(auxNum, nRegOffset)  \
	__REG32 (IO_ADDRESS (HW_GPIO_NUM_TO_GROUPBASE(GPIO_AUX_TO_GROUP(auxNum)) + (GPIO_AUX_TO_SUBGROUP1(auxNum) << HW_GPIO_SUBGROUP_SHIFT) + nRegOffset))



static inline int reg_gpio_iotr_get_pin_type( int pin )
{
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));

	if (REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_AUX_SEL) & nBitMask)
	{
		/* AUX bit set, now check AUX01 bit */
		if (REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_AUX_SEL) & nBitMask)
			return GPIO_PIN_TYPE_ALTERNATIVE_FUNC1;
		else
			return GPIO_PIN_TYPE_ALTERNATIVE_FUNC0;
	}
	else
	{
		/* Not AUX, now check Input/Output */
		if (REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_EN) & nBitMask)
			return GPIO_PIN_TYPE_OUTPUT;
		else
			return GPIO_PIN_TYPE_INPUT; /* TODO: INPUT_WITH_INTERRUPT */
	}

}

static inline void reg_gpio_iotr_set_pin_type( int pin, GPIO_PIN_TYPE pinType )
{
    unsigned long flags;
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin)); /* this is not for pin number indicates AUX since that will be too big */


	BCM5892_GPIO_DEBUG ("pintype=%d, bitmask=%x\n", pinType, nBitMask);

    HW_IRQ_SAVE(Gpio, flags);

    switch(pinType)
    {
		case GPIO_PIN_TYPE_INPUT:
        case GPIO_PIN_TYPE_INPUT_WITH_INTERRUPT:
        case GPIO_PIN_TYPE_OUTPUT:
             /* Clear AUX bit */
             REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_AUX_SEL) &= ~nBitMask;

             /* Set input/output */
             if (pinType == GPIO_PIN_TYPE_OUTPUT)
             {
                 REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_EN) |= nBitMask;
			 }
			 else
			 {
                 REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_EN) &= ~nBitMask;
			 }

			 break;

        case GPIO_PIN_TYPE_ALTERNATIVE_FUNC0:
             if (pin < ARCH_NR_GPIOS)
             {
                 /* Set AUX bit */
                 REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_AUX_SEL) |= nBitMask;

                 /* Clear AUX01 bit */
    			 REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_AUX01_SEL) &= ~nBitMask;
			 }
			 else
			 {
				 BCM5892_GPIO_DEBUG ("aux0, grp=%x, %x, %x\n", GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), GPIO_AUX_TO_SUBGROUP2(pin));

				 /* 5892 specific subgroup */
				 REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_AUX_SEL)   |= ALL_BITS;
				 REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_AUX01_SEL) &= ~ALL_BITS;

				 if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin))
				 {
					 BCM5892_GPIO_DEBUG ("aux0, subgroup field2 exist\n");

					 REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_AUX_SEL)   |= ALL_BITS;
					 REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_AUX01_SEL) &= ~ALL_BITS;
				 }
			 }

			 break;

        case GPIO_PIN_TYPE_ALTERNATIVE_FUNC1:
             if (pin < ARCH_NR_GPIOS)
             {
				 /* Set AUX bit */
				 REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_AUX_SEL) |= nBitMask;

				 /* Set AUX01 bit */
				 REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_AUX01_SEL) |= nBitMask;
			 }
			 else
			 {
				 BCM5892_GPIO_DEBUG ("aux1, grp=%x, %x, %x\n", GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), GPIO_AUX_TO_SUBGROUP2(pin));

				 /* 5892 specific subgroup */
				 REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_AUX_SEL)   |= ALL_BITS;
				 REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_AUX01_SEL) |= ALL_BITS;

				 if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin))
				 {
					 BCM5892_GPIO_DEBUG ("aux1, subgroup field2 exist\n");

					 REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_AUX_SEL)   |= ALL_BITS;
					 REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_AUX01_SEL) |= ALL_BITS;
				 }
			 }

			 break;

		case GPIO_PIN_TYPE_ALTERNATIVE_DISABLE:
             if (pin < ARCH_NR_GPIOS)
             {
                 /* Clear AUX bit */
                 REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_AUX_SEL) &= ~nBitMask;
			 }
			 else
			 {
				 BCM5892_GPIO_DEBUG ("clear aux, grp=%x, %x, %x\n", GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), GPIO_AUX_TO_SUBGROUP2(pin));

				 /* 5892 specific subgroup */
				 REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_AUX_SEL)   &= ~ALL_BITS;

				 if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin))
				 {
					 BCM5892_GPIO_DEBUG ("clear aux, subgroup field2 exist\n");

					 REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_AUX_SEL)   &= ~ALL_BITS;
				 }
			 }

			 break;

	}

    HW_IRQ_RESTORE(Gpio, flags);
}

static inline void reg_gpio_set_pin( int pin, int val )
{
    unsigned long flags;
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));

    HW_IRQ_SAVE( Gpio, flags );
    if ( val == 0 )
    {
        /* Set the pin to zero */
	    REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_DOUT) &= ~nBitMask;
    }
    else
    {
        /* Set the pin to 1 */
	    REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_DOUT) |= nBitMask;
    }
    HW_IRQ_RESTORE( Gpio, flags );
}

static inline int reg_gpio_get_pin_output( int pin )
{
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));

    return ( REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_DOUT) & nBitMask ) != 0;
}


static inline int reg_gpio_get_pin( int pin )
{
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));

    return ( REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_DOUT) & nBitMask ) != 0;
}


static inline int reg_gpio_is_interrupt_enable( int pin )
{
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));

    return ( REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_INT_MSK) & nBitMask ) != 0;
}

static inline void reg_gpio_enable_interrupt( int pin )
{
    unsigned long flags;
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));

    HW_IRQ_SAVE( Gpio, flags );
    REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_INT_MSK) |= nBitMask ;
    HW_IRQ_RESTORE( Gpio, flags );
}


static inline void reg_gpio_disable_interrupt( int pin )
{
    unsigned long flags;
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));

    HW_IRQ_SAVE( Gpio, flags );
    REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_INT_MSK) &= ~nBitMask ;
    HW_IRQ_RESTORE( Gpio, flags );
}


static inline void reg_gpio_clear_interrupt( int pin )
{
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));
    REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_INT_CLR) |= nBitMask ;
}


static inline int reg_gpio_get_interrupt_status( int pin )
{
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));
    return ( REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_INT_STAT) & nBitMask );
}

static inline int reg_gpio_itr_get_interrupt_type( int pin )
{
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));
    return ( REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_INT_EDGE) & nBitMask );
}

static inline void reg_gpio_itr_set_interrupt_type( int pin, GPIO_INTERRUPT_TYPE interruptType )
{
   unsigned long flags;
   unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));

   HW_IRQ_SAVE( Gpio, flags );
   REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_INT_EDGE) |= nBitMask; /* TODO: INTERRUPT_TYPE */
   HW_IRQ_RESTORE( Gpio, flags );
}



static inline void reg_gpio_set_pull_up_down( int pin, int val )
{
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));

    if (pin < ARCH_NR_GPIOS) {
    	if( val )
        	REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_PAD_RES) |= nBitMask; /* Set pull up */
    	else
        	REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_PAD_RES) &= ~nBitMask; /* Set pull down */
    } else {
	/* 5892 specific subgroup */
	if (val) {
		REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_PAD_RES)   |= ALL_BITS;

		if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin)) {
			BCM5892_GPIO_DEBUG ("aux1, subgroup field2 exist\n");

			REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_PAD_RES)   |= ALL_BITS;
		}
	} else {
		REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_PAD_RES)   &= ~ALL_BITS;

		if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin)) {
			BCM5892_GPIO_DEBUG ("aux1, subgroup field2 exist\n");

			REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_PAD_RES)   &= ~ALL_BITS;
		}
	}
    }
}


static inline void reg_gpio_set_pull_up_down_enable( int pin )
{
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));

    if (pin < ARCH_NR_GPIOS) {
    	REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_RES_EN) |= nBitMask; /* Enable PAD resister */
    } else {
	/* 5892 specific subgroup */
	REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_RES_EN)   |= ALL_BITS;

	if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin)) {
		BCM5892_GPIO_DEBUG ("aux1, subgroup field2 exist\n");

		REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_RES_EN)   |= ALL_BITS;
	}
    }
}

static inline void reg_gpio_set_pull_up_down_disable( int pin )
{
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));

    if (pin < ARCH_NR_GPIOS) {
    	REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_RES_EN) &= ~nBitMask; /* Enable PAD resister */
    } else {
	/* 5892 specific subgroup */
	REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_RES_EN)   &= ~ALL_BITS;

	if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin)) {
		BCM5892_GPIO_DEBUG ("aux1, subgroup field2 exist\n");

		REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_RES_EN)   &= ~ALL_BITS;
	}
    }

}

typedef enum
{
    GPIO_DRV_STRENGTH_4mA		= 0x04,
    GPIO_DRV_STRENGTH_6mA		= 0x06,
    GPIO_DRV_STRENGTH_8mA		= 0x08
} GPIO_DRV_STRENGTH;

static inline void reg_gpio_set_drv_strength( int pin, GPIO_DRV_STRENGTH val )
{
    unsigned int  nBitMask = (1 << HW_GPIO_PIN_TO_BIT_OFFSET(pin));

    if (pin < ARCH_NR_GPIOS) {
	switch(val) {
	case GPIO_DRV_STRENGTH_4mA:
        	REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_DRV_SEL0_SW) &= ~nBitMask; /* 0 */
        	REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_DRV_SEL1_SW) |= nBitMask; /* 1  */
        	REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_DRV_SEL2_SW) &= ~nBitMask; /* 0 */
		break;
	case GPIO_DRV_STRENGTH_6mA:
        	REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_DRV_SEL0_SW) &= ~nBitMask; /* 0 */
        	REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_DRV_SEL1_SW) &= ~nBitMask; /* 0 */
        	REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_DRV_SEL2_SW) |= nBitMask; /* 1  */
		break;
	case GPIO_DRV_STRENGTH_8mA:
        	REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_DRV_SEL0_SW) |= nBitMask; /* 1 */
        	REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_DRV_SEL1_SW) &= ~nBitMask; /* 0 */
        	REG_GPIO_FROM_PIN (pin, REGOFFSET_GPIO_DRV_SEL2_SW) |= nBitMask; /* 1  */
		break;
	}
    } else {
	/* 5892 specific subgroup */
	switch(val) {
	case GPIO_DRV_STRENGTH_4mA:

		REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_DRV_SEL0_SW)   &= ~ALL_BITS;
		if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin)) {
			BCM5892_GPIO_DEBUG ("aux1, subgroup field2 exist\n");

			REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_DRV_SEL0_SW)   &= ~ALL_BITS;
		}

		REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_DRV_SEL1_SW)   |= ALL_BITS;
		if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin)) {
			BCM5892_GPIO_DEBUG ("aux1, subgroup field2 exist\n");

			REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_DRV_SEL1_SW)   |= ALL_BITS;
		}

		REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_DRV_SEL2_SW)   &= ~ALL_BITS;
		if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin)) {
			BCM5892_GPIO_DEBUG ("aux1, subgroup field2 exist\n");

			REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_DRV_SEL2_SW)   &= ~ALL_BITS;
		}
		break;

	case GPIO_DRV_STRENGTH_6mA:

		REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_DRV_SEL0_SW)   &= ~ALL_BITS;
		if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin)) {
			BCM5892_GPIO_DEBUG ("aux1, subgroup field2 exist\n");

			REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_DRV_SEL0_SW)   &= ~ALL_BITS;
		}

		REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_DRV_SEL1_SW)   &= ~ALL_BITS;
		if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin)) {
			BCM5892_GPIO_DEBUG ("aux1, subgroup field2 exist\n");

			REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_DRV_SEL1_SW)   &= ~ALL_BITS;
		}

		REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_DRV_SEL2_SW)   |= ALL_BITS;
		if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin)) {
			BCM5892_GPIO_DEBUG ("aux1, subgroup field2 exist\n");

			REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_DRV_SEL2_SW)   |= ALL_BITS;
		}
		break;
	case GPIO_DRV_STRENGTH_8mA:

		REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_DRV_SEL0_SW)   |= ALL_BITS;
		if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin)) {
			BCM5892_GPIO_DEBUG ("aux1, subgroup field2 exist\n");

			REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_DRV_SEL0_SW)   |= ALL_BITS;
		}

		REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_DRV_SEL1_SW)   &= ~ALL_BITS;
		if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin)) {
			BCM5892_GPIO_DEBUG ("aux1, subgroup field2 exist\n");

			REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_DRV_SEL1_SW)   &= ~ALL_BITS;
		}

		REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP1(pin), REGOFFSET_GPIO_DRV_SEL2_SW)   |= ALL_BITS;
		if (GPIO_AUX_IS_SUBGROUP2_EXIST(pin)) {
			BCM5892_GPIO_DEBUG ("aux1, subgroup field2 exist\n");

			REG_GPIO_FROM_SUBGROUP(GPIO_AUX_TO_GROUP(pin), GPIO_AUX_TO_SUBGROUP2(pin), REGOFFSET_GPIO_DRV_SEL2_SW)   |= ALL_BITS;
		}
		break;
	}


    }


}




/* This function is added only for compile, the function implementation will need to be revisited
   This function is called from gpiolib gpiolib_dbg_show()
*/
static inline int gpio_to_irq(int pin)
{
	/* only group 1 can take external interrupt (?) */
	if ((pin < HW_GPIO0_PIN_MAX) || (pin >= HW_GPIO1_PIN_MAX)) return 0;

	if (pin ==  HW_GPIO0_NUM_PIN)     return IRQ_OEXTGRP0;
	if (pin == (HW_GPIO0_NUM_PIN+1))  return IRQ_OEXTGRP1;
	if (pin == (HW_GPIO0_NUM_PIN+2))  return IRQ_OEXTGRP2;
	if (pin == (HW_GPIO0_NUM_PIN+3))  return IRQ_OEXTGRP3;

	/* now in reverse order */
	if (pin >= (HW_GPIO0_NUM_PIN+24)) return IRQ_OEXTGRP7; /* gpio 24-31 in group1 */
	if (pin >= (HW_GPIO0_NUM_PIN+16)) return IRQ_OEXTGRP6; /* gpio 16-23 */
	if (pin >= (HW_GPIO0_NUM_PIN+ 8)) return IRQ_OEXTGRP5; /* gpio 08-15 */
	if (pin >= (HW_GPIO0_NUM_PIN+ 4)) return IRQ_OEXTGRP4; /* gpio 04-07 */

	return 1;
}

#endif  /* __ASM_ARCH_REG_GPIO_H */
