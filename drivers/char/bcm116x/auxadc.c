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
*  auxadc.c
*
*  PURPOSE:
*
*     This implements the driver for the auxiliary ADC.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/fs.h>

#include <mach/reg_auxadc.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>

#include <linux/broadcom/bcm_major.h>

/*
 * ---- Public Variables ------------------------------------------------- 
 * ---- Private Constants and Types -------------------------------------- 
 */

#if 0
#   define AUXADC_DEBUG(x) printk x
#else
#   define AUXADC_DEBUG(x)
#endif

/* ---- Private Variables ------------------------------------------------ */

static char banner[] __initdata = KERN_INFO "Auxiliary ADC Driver: 0.02\n";
DECLARE_WAIT_QUEUE_HEAD( gAdcReadyQueue );
static struct semaphore gSem;

/* ---- Private Function Prototypes -------------------------------------- */

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  auxadc_open
*
***************************************************************************/

static int auxadc_open( struct inode *inode, struct file *file )
{
    AUXADC_DEBUG(( "auxadc_open called\n" ));

    return 0;

} /* auxadc_open */


/****************************************************************************
 *
 *  auxadc_access
 *
 ****************************************************************************/
int auxadc_access( int regID )
{
    int rc;
    u16 adcOutput;

    /* start the appropriate channel based on minor device number */
    switch ( regID )
    {
        case 1:
            REG_AUXADC_CR = REG_AUXADC_CR_AMUX1 | REG_AUXADC_CR_START;
            break;

        case 2:
            REG_AUXADC_CR = REG_AUXADC_CR_AMUX2 | REG_AUXADC_CR_START;
            break;

        case 3:
            REG_AUXADC_CR = REG_AUXADC_CR_AMUX3 | REG_AUXADC_CR_START;
            break;

        case 4:
            REG_AUXADC_CR = REG_AUXADC_CR_AMUX4 | REG_AUXADC_CR_START;
            break;

        case 5:
            REG_AUXADC_CR = REG_AUXADC_CR_AMUX5 | REG_AUXADC_CR_START;
            break;

        default:
            AUXADC_DEBUG(( "auxadc - error in minor device number\n" ));
            return -1;
    }

    /* wait for 10 ms for conversion to complete */
    rc = interruptible_sleep_on_timeout( &gAdcReadyQueue, 10 * HZ / 1000 );

    /* read the ADC, and then let other processes use it */
    if ( REG_AUXADC_DR & REG_AUXADC_DR_RDY )
    {
        /* output ready, read result */
        adcOutput = REG_AUXADC_DR & REG_AUXADC_DR_AUXDMASK;
    }
    else
    {
        /* output not ready, return 0 */
        return -1;
    }
    AUXADC_DEBUG(( "auxadc - adc output was %d\n", adcOutput ));
    return (int)adcOutput;
}


/****************************************************************************
*
*  auxadc_read
*
***************************************************************************/

static ssize_t auxadc_read( struct file *file, char *buffer, size_t count, loff_t *ppos )
{
    u16 adcOutput;
    int rc;

    AUXADC_DEBUG(( "auxadc_read called, major = %d, minor = %d\n", MAJOR( file->f_dentry->d_inode->i_rdev ), MINOR( file->f_dentry->d_inode->i_rdev )));

    if ( count == 0 )
    {
        return 0;
    }
    /* ensure only one channel is accessed at a time */
    if( down_interruptible( &gSem ) )
    {
        return -ERESTARTSYS;
    }

    rc = auxadc_access( MINOR( file->f_dentry->d_inode->i_rdev ) );

    /* release interrupt */
    up( &gSem );
    if( rc < 0 )
    {
       return rc;
    }

    adcOutput = (u16)rc;

    if ( copy_to_user( &buffer[0], &adcOutput, sizeof(u16)) != 0 )
    {
        return -EFAULT;
    }

    return sizeof(u16);

} /* auxadc_read */

/****************************************************************************
*
*  auxadc_release
*
***************************************************************************/

static int auxadc_release( struct inode *inode, struct file *file )
{
    AUXADC_DEBUG(( "auxadc_release called\n" ));
    return 0;

} /* auxadc_release */

/****************************************************************************
*
*  auxadc_write
*
***************************************************************************/

static ssize_t auxadc_write( struct file *file, const char *buffer, size_t count, loff_t *ppos )
{
    return -EINVAL;

} /* auxadc_write */

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations auxadc_fops =
{
    owner:      THIS_MODULE,
    open:       auxadc_open,
    release:    auxadc_release,
    read:       auxadc_read,
    write:      auxadc_write,
};

/****************************************************************************
*
*  auxadc_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

static int __init auxadc_init( void )
{
    int rc;

    AUXADC_DEBUG(( "auxadc_init called\n" ));
    printk( banner );

    if (( rc = register_chrdev( BCM_AUXADC_MAJOR, "auxadc", &auxadc_fops )) < 0 )
    {
        printk( KERN_WARNING "auxadc: register_chrdev failed for major %d\n", BCM_AUXADC_MAJOR ); 
        return rc;
    }

    /* initialize semaphore for ADC access control */
    sema_init(&gSem, 1);

    return 0;

} /* auxadc_init */

/****************************************************************************
*
*  auxadc_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit auxadc_exit( void )
{
    AUXADC_DEBUG(( "auxadc_exit called\n" ));

} /* auxadc_exit */

/****************************************************************************/

/* Changed from module_init to fs_initcall so that AUXADC driver 
 * is loaded before the any of the PMU drivers is loaded. PMU drivers 
 * were also changed to fs_initcall so that they are loaded before 
 * VC02 and USB drivers are loaded. THis was done because the host has to
 * read the PMU interrupts in time (< 8sec) or else the PMU
 * timeout timer (of 8sec) could expire causing the phone to shut off.
 * This was observed in cases where a battery was removed and then re inserted.
 * This action would cause a LOWBAT interrupt generated and the host has 8sec
 * to clear it before PMU goes into standby mode. If VC02 driver was loaded 
 * before PMU driver, the PMU driver was getting loaded well past 8sec window
 */

fs_initcall(auxadc_init);
module_exit(auxadc_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Auxiliary ADC Driver");
MODULE_LICENSE("GPL v2");

EXPORT_SYMBOL (auxadc_access);

