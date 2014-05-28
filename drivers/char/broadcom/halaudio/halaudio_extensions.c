/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
/**
*
*  @file    halaudio_extensions.c
*
*  @brief   HAL Audio extension layer. Only platform specific
*           extensions should be added to this file.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#include <linux/module.h>              /* For EXPORT_SYMBOL */
#include <linux/kernel.h>
#include <linux/sched.h>               /* For schedule_timeout */
#include <linux/semaphore.h>           /* For protecting accesses to gPMOps */
#include <linux/platform_device.h>     /* For Linux platform bus */

#ifdef CONFIG_BCM_BSC
#include <linux/broadcom/bsc.h>        /* Board Specific Configurations */
#endif

#include <linux/broadcom/halaudio.h>   /* HALAUDIO API */
#include <linux/broadcom/halaudio_cfg.h>
#include <linux/broadcom/halaudio_extensions_cfg.h>
#include <linux/broadcom/gpio.h>       /* GPIO register accesses */

extern int speakerInit( int extSpkrGpio, int onBoardSpkrGpio );
extern void speakerExit( void );

/* ---- Private Constants and Types -------------------------------------- */

/* ---- Private Function Prototypes -------------------------------------- */
static int setGain( HALAUDIO_BLOCK block, int db );

/* ---- Private Variables ------------------------------------------------ */
static HALAUDIO_EXTENSIONS_CFG   gCfg;
static HALAUDIO_DEV_CFG          gHalaudioDev;

static const HALAUDIO_EXTENSIONS_OPS gOps =
{
   .setgain    = setGain,  /* gain set */
};

static int spkrPaEnabled = 0;

/* ---- Public Variables ------------------------------------------------- */

/* ---- Functions -------------------------------------------------------- */
/**
*  Platform specific gain set routine. The following implementation
*  sets a GPIO to enable an external op-amp for a particular block
*
*  @return
*     0     - Success
*     -ve   - error code
*/
static int setGain(
   HALAUDIO_BLOCK block,            /**< (in) HAL block ID */
   int            db                /**< (in) gain in db */
)
{
   int               err;
   HALAUDIO_HDL      hdl;
   HALAUDIO_CODEC    cid;
   HALAUDIO_BLOCK    spkr_blk;

   if ( gCfg.ext_spkr_pwdn_gpio >= 0 )
   {
      hdl = halAudioAllocateClient();
      if ( hdl == NULL )
      {
         return -ENOMEM;
      }

      err = halAudioQueryCodecByName( hdl, gHalaudioDev.info.codec_name, &cid );
      if ( err )
      {
         halAudioFreeClient( hdl );
         return err;
      }

      spkr_blk = HALAUDIO_BLOCK_ANA_ID( cid, gHalaudioDev.info.spkr_hwsel, HALAUDIO_DIR_DAC );

      halAudioFreeClient( hdl );

      if ( block == spkr_blk )
      {
         if ( db > HALAUDIO_GAIN_MUTE )
         {
            if ( !spkrPaEnabled )
            {
               /* enable external speaker op-amp if no line-out speaker detected */
               if ( ( gCfg.line_out_det_gpio < 0 ) ||
                    ( gpio_get_value( gCfg.line_out_det_gpio ) != gCfg.line_out_det_gpio_inserted ) )
               {
                  gpio_set_value( gCfg.ext_spkr_pwdn_gpio, 1 );
               }
               spkrPaEnabled = 1;
            }
         }
         else
         {
            gpio_set_value( gCfg.ext_spkr_pwdn_gpio, 0 );
            spkrPaEnabled = 0;
         }
      }
   }


   return 0;
}


/***************************************************************************/
/**
*  Extensions support constructor
*/
static __init int halaudio_extensions_probe( struct platform_device *pdev )
{
   printk( KERN_INFO "HAL Audio platform support installed. Built %s %s\n",
         __DATE__, __TIME__ );

   if ( pdev->dev.platform_data == NULL )
   {
      printk( KERN_ERR "%s: Failed to obtain platform data\n", __FUNCTION__ );
      return -ENODEV;
   }

   memcpy( &gCfg, pdev->dev.platform_data, sizeof( gCfg ));

   if ( ( gCfg.ext_spkr_pwdn_gpio >= 0 ) || ( gCfg.line_out_det_gpio >= 0 ) )
   {
      int err;

      if ( strlen( gCfg.dev_name ) == 0 )
      {
         printk( KERN_ERR "%s: Name of the device that requires GPIO support cannot be found\n", __FUNCTION__ );
         return -ENODEV;
      }
#ifdef CONFIG_BCM_BSC
      err = bsc_query( gCfg.dev_name, &gHalaudioDev, sizeof( gHalaudioDev ));
      if ( err )
      {
         printk( KERN_ERR "%s: Failed returning from bsc_query name=%s\n", __FUNCTION__, gCfg.dev_name );
         return err;
      }
#else
      printk( KERN_ERR "%s: Missing the required Linux BSC driver\n", __FUNCTION__ );
      return -EPERM;
#endif

      if ( gCfg.ext_spkr_pwdn_gpio >= 0 )
      {
         err = gpio_request( gCfg.ext_spkr_pwdn_gpio, "External speaker op-amp" );
         if ( err )
         {
            printk( KERN_ERR "%s: Failed to request GPIO pin %i\n", __FUNCTION__, gCfg.ext_spkr_pwdn_gpio );
            return err;
         }

         /* Initially shutdown op-amp */
         gpio_direction_output( gCfg.ext_spkr_pwdn_gpio, 1 );

         gpio_free( gCfg.ext_spkr_pwdn_gpio );
      }

      if ( gCfg.line_out_det_gpio >= 0 )
      {
         speakerInit( gCfg.line_out_det_gpio, gCfg.ext_spkr_pwdn_gpio );
      }
   }

   /* Setup up platform sepcific extensions */
   halAudioSetExtensionsOps( &gOps );

   return 0;
}

/***************************************************************************/
/**
*  Extensions support destructor
*/
static int halaudio_extensions_remove( struct platform_device *pdev )
{
   if ( gCfg.ext_spkr_pwdn_gpio >= 0 )
   {
      gpio_free( gCfg.ext_spkr_pwdn_gpio );
   }

   if ( gCfg.line_out_det_gpio >= 0 )
   {
      free_irq( gpio_to_irq( gCfg.line_out_det_gpio ), NULL );
      gpio_free( gCfg.line_out_det_gpio );

      speakerExit();
   }

   return 0;
}

/* Platform bus driver */
static struct platform_driver halaudio_extensions_driver =
{
   .driver     =
   {
      .name    = "bcm-halaudio-extensions",
      .owner   = THIS_MODULE,
   },
   .probe = halaudio_extensions_probe,
   .remove = halaudio_extensions_remove,
};

static __init int halaudio_extensions_init( void )
{
   return platform_driver_register( &halaudio_extensions_driver );
}

static void __exit halaudio_extensions_exit( void )
{
   platform_driver_unregister( &halaudio_extensions_driver );
}

module_init( halaudio_extensions_init );
module_exit( halaudio_extensions_exit );
