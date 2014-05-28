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
*  Description: BCMRING ethernet switch driver.
*  
*  The ethernet switch driver allows the user to access the switch register
*  interface on a BCMRING device.  These registers allow configuration of the
*  three port switch to control features such as VLAN, forwarding rules, etc.
*/ 

#include <linux/fs.h>
#include <linux/sockios.h>
#include <linux/platform_device.h>
#include <linux/if.h>
#include <asm/uaccess.h>

#include <linux/broadcom/bcm_major.h>

/* BCMRING Device private ioctl calls */
#define SIOC_BCMRING_PRIVATE    SIOCDEVPRIVATE

typedef struct
{
   int cmd;             /* Command to run */
   void __user  *arg;   /* Pointer to the argument structure */
} ESW_IOCTL_ARGS;

/* Buffer size for temporarily holding ioctl args in kernel space */
#define BUF_SIZE     1024

typedef int (*CMD_FUNC)( unsigned int, void *, size_t );

static CMD_FUNC cmdFunc = NULL;

void esw_cmd_register( CMD_FUNC cmd )
{
   cmdFunc = cmd;
}

EXPORT_SYMBOL( esw_cmd_register );

/*
*  Character driver support
*/

static int esw_ioctl( struct inode *inode, struct file *filp,
                      unsigned int cmd, unsigned long arg )
{
   int rc;
   struct ifreq *ifr;
   ESW_IOCTL_ARGS *datap;
   unsigned int ethCmd;
   size_t len;
   void *bufp;

   /* The network interface can define its own ioctl commands to modify network configurations.
   *  The ioctl implementation for sockets recognizes 16 commands as private to the interface:
   *  SIOCDEVPRIVATE through SIOCDEVPRIVATE+15.  To call a custom ioctl function, user will supply
   *  a cmd parameter and an argument structure.
   *
   *  To expand beyond 16 custom ioctls, all Broadcom-specific network ioctls will be accessed through
   *  a single cmd SIOC_BCMRING_PRIVATE, which is defined in the range of SIOCDEVPRIVATE through
   *  SIOCDEVPRIVATE+15.  Another cmd type will be embedded into the argument structure (cmd field in
   *  ESW_IOCTL_ARGS to specify the custom function to call.  This will allow unlimited number of
   *  custom functions.
   */

   if( cmd != SIOC_BCMRING_PRIVATE )
   {
      return( -EOPNOTSUPP );
   }

   if( !cmdFunc )
   {
      printk( "Switch command function has not been registered\n" );
      return( -EFAULT );
   }

   rc = 0;

   ifr = (struct ifreq *)arg;
   datap = (ESW_IOCTL_ARGS *) &ifr->ifr_ifru;
   ethCmd = datap->cmd;
   len = _IOC_SIZE( ethCmd );

   if( len > BUF_SIZE )
   {
      printk( "Ioctl argument length too large: %ibytes (maximum is %ibytes)\n",
              len, BUF_SIZE );
      return( -ENOMEM );
   }

   bufp = (void *)kmalloc( BUF_SIZE, GFP_KERNEL );
   if( !bufp )
   {
      printk( "Unable to allocate buffer\n" );
      return( -ENOMEM );
   }

   if( (ethCmd & IOC_IN) && (len > 0) )
   {
      /* Write operation */
      if( copy_from_user( bufp, datap->arg, len ) ) return ( -EFAULT );
   }

   rc = (*cmdFunc)( _IOC_NR( ethCmd ), bufp, len );

   if( (ethCmd & IOC_OUT) && (len > 0) )
   {
      /* Read operation */
      if( copy_to_user( datap->arg, bufp, len ) ) return( -EFAULT );
   }

   kfree( bufp );
   
   return( rc );
}

static struct file_operations esw_fops =
{
   owner: THIS_MODULE,
   ioctl: esw_ioctl,
};


/*
*  Platform driver support
*/

static int esw_probe( struct platform_device *pdev )
{
   if( register_chrdev( BCM_ESW_MAJOR, "esw", &esw_fops ) < 0 )
   {
      printk( KERN_ERR "ESW: register_chrdev failed for major %u\n",
              BCM_ESW_MAJOR );
      return( -EFAULT );
   }

   return( 0 );
}

static int esw_remove( struct platform_device *pdev )
{
   return( 0 );
}

#ifdef CONFIG_PM
static int esw_suspend( struct platform_device *pdev, pm_message_t mesg )
{
   return( 0 );
}

static int esw_resume( struct platform_device *pdev )
{
   return( 0 );
}
#else
#define esw_suspend   NULL
#define esw_resume    NULL
#endif

static struct platform_driver esw_driver = {
   .driver = {
      .name = "bcmring-esw",
      .owner = THIS_MODULE,
   },
   .probe = esw_probe,
   .remove = esw_remove,
   .suspend = esw_suspend,
   .resume = esw_resume,
};

static int __init esw_init(void)
{
   return( platform_driver_register( &esw_driver ) );
}

static void __exit esw_exit(void)
{
   platform_driver_unregister( &esw_driver );
}

module_init( esw_init );
module_exit( esw_exit );

MODULE_AUTHOR( "Broadcom" );
MODULE_DESCRIPTION( "Ethernet Switch (ESW) Driver" );
MODULE_LICENSE( "GPL" );
