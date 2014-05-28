/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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
*  dls.c
*
*  PURPOSE:
*
*     This implements the polyringer wavetable aligned memory dls driver.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <mach/dma.h>
#include <linux/dma-mapping.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>

#include <asm/uaccess.h>

#include <linux/broadcom/bcm_major.h>

/*
 * ---- Public Variables ------------------------------------------------- 
 * ---- Private Constants and Types -------------------------------------- 
 */

#define DLS_DRIVER_NAME "dls"
#define ALIGN_SIZE  ((unsigned int)(0x40000))           /* 256 kbyte */
#define ALIGN_MASK  ((unsigned int)(ALIGN_SIZE-1))      /* 0x3ffff */

/*
 * ---- Private Function Prototypes -------------------------------------- 
 * ---- Private Variables ------------------------------------------------ 
 */
static dma_addr_t physp_base;
static dma_addr_t physp;    /* must be aligned */
static void * virtp_base;
static void * virtp;
static char banner[] __initdata = KERN_INFO "DLS Driver: 1.00 (built on "__DATE__" "__TIME__")\n";

/* ---- Functions -------------------------------------------------------- */
/****************************************************************************
*
*  dls_write
*
***************************************************************************/

static ssize_t dls_write( struct file *file, const char *buffer, size_t count, loff_t *ppos )
{
	if (copy_from_user(virtp, buffer, count))
	{
		printk("dls_write: failed to copy user to kernel space\n");
		return -EFAULT;
	}
	/*printk("dls_write: copied %u bytes to virtp %p, physp 0x%x\n", count, virtp, physp); */
    virtp += count;
    physp += count;
    return count;
}

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations dls_fops =
{
    owner:      THIS_MODULE,
    write:      dls_write,
};

/****************************************************************************
*
*  dls_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/
static int __init dls_init( void )
{
    int rc;

    printk( banner );
    if ( !( virtp = dma_alloc_coherent( NULL, ALIGN_SIZE, &physp, GFP_KERNEL )))
    {
        printk( KERN_ERR "kernel: Cannot allocate shared memory page\n" );
        return -ENOMEM;
    }
    if (( physp & ALIGN_MASK ) != 0 )
    {
        panic( "dls align(): We didn't get an aligned buffer" );
        return -ENOMEM;
    }
	physp_base = physp;
	virtp_base = virtp;
    /*printk( "dma_alloc_coherent size=0x%x, physp = 0x%x\n", (unsigned int)ALIGN_SIZE, physp ); */

    if (( rc = register_chrdev( BCM_DLS_MAJOR, "dls", &dls_fops )) < 0 )
    {
        printk( KERN_ERR "dls_init: register_chrdev failed for major %d\n", BCM_DLS_MAJOR );
        return rc;
    }
	return 0;
}
/****************************************************************************
*
*  dls_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit dls_exit( void )
{
    /*printk("dls_exit() called\n"); */
    if (virtp != NULL)
    {
        dma_free_coherent(NULL, ALIGN_SIZE, virtp_base, physp_base);
    }
}
/****************************************************************************
*
*  dls_virtptr
*
*  Called to get virtual pointer to aligned memory
*
***************************************************************************/
void *dls_virtptr(void)
{
	/*printk("dls_virtptr() returns 0x%x\n", virtp_base); */
    return virtp_base;
}
/****************************************************************************
*
*  dls_physptr
*
*  Called to get pointer to aligned memory
*
***************************************************************************/
dma_addr_t dls_physptr(void)
{
	/*printk("dls_physptr() returns 0x%x\n", physp_base); */
    return physp_base;
}
/****************************************************************************
*
*  dls_size
*
*  Called to get consumed size of aligned memory in bytes
*
***************************************************************************/
unsigned int dls_size(void)
{
	unsigned int size = (unsigned int)physp - (unsigned int)physp_base;
	/*printk("dls_size() returns 0x%x\n", size); */
    return size;
}

module_init(dls_init);
module_exit(dls_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("DLS Driver");
MODULE_LICENSE("GPL v2");
	
EXPORT_SYMBOL (dls_physptr);
EXPORT_SYMBOL (dls_virtptr);
EXPORT_SYMBOL (dls_size);

