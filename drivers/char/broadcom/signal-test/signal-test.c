/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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

/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <asm/ioctls.h>
#include <linux/broadcom/bcm_major.h>

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

#define SIG_TEST_UP         _IO('S', 0x01 )
#define SIG_TEST_DOWN       _IO('S', 0x02 )
#define SIG_TEST_DOWN_INT   _IO('S', 0x03 )

/* ---- Private Variables ------------------------------------------------ */

struct semaphore    gSem;

/* ---- Private Function Prototypes -------------------------------------- */

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*   Called in response to an ioctl request
*
***************************************************************************/

static int sig_ioctl( struct inode *inode, struct file *file_id, unsigned int cmd, unsigned long arg )
{
    int rc = 0;

    printk( "%s: called\n", __func__ );

    switch ( cmd )
    {
        case SIG_TEST_UP:
        {
            printk( "%s: SIG_TEST_UP about to call up\n", __func__ );
            up( &gSem );
            printk( "%s: SIG_TEST_UP done\n", __func__ );
            break;
        }

        case SIG_TEST_DOWN:
        {
            printk( "%s: SIG_TEST_DOWN about to call down\n", __func__ );
            down( &gSem );
            printk( "%s: SIG_TEST_DOWN done\n", __func__ );
            break;
        }

        case SIG_TEST_DOWN_INT:
        {
            printk( "%s: SIG_TEST_DOWN_INT about to call down_interruptible\n", __func__ );
            rc = down_interruptible( &gSem );
            if ( rc == -EINTR )
            {
                printk( "%s: down_interruptible interrupted: returning -ERESTARTSYS\n", __func__ );
                rc = -ERESTARTSYS;
            }
            printk( "%s: SIG_TEST_DOWN_INT done\n", __func__ );
            break;
        }

        default:
        {
            printk( "%s: Unrecognized cmd: 0x%08x\n", __func__, cmd );
            rc = -ENOTTY;
        }
    }

    printk( "%s: returning %d\n", __func__, rc );
    return rc;
}

static struct file_operations sig_file_ops =
{
    .ioctl  = sig_ioctl,
};

/****************************************************************************
*
*   Registers a driver which can have ioctl calls made
*
***************************************************************************/

static int __init sig_test_init( void )
{
    int rc;

    sema_init( &gSem, 0 );

    if (( rc = register_chrdev( BCM_VC02_MAJOR, "sig-test", &sig_file_ops )) < 0 )
    {
        printk( KERN_ERR "register_chrdev failed: %d\n", rc );
        return rc;
    }

    return 0;
}

/****************************************************************************
*
*   Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit sig_test_exit( void )
{
}

/****************************************************************************/

module_init( sig_test_init );
module_exit( sig_test_exit );

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION( "Broadcom Signal Test" );
MODULE_LICENSE( "GPL" );
MODULE_VERSION( "1.0" );

