/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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
*  @file    halaudio_external_ak4642.c
*
*  @brief   HAL Audio External routines for AK4642 codec.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <linux/proc_fs.h>          /* For /proc/halAudioExternalAK4642 */
#include <linux/sched.h>            /* For schedule_timeout */
#include <linux/platform_device.h>  /* For platform bus */
#define USE_BCM_GPIO 1
#include <linux/broadcom/gpio.h>    /* GPIO register accesses */
#include <linux/broadcom/halaudio.h>
#include <linux/broadcom/halaudio_external_ak4642.h>

#ifdef CONFIG_BCM_HALAUDIO_EXTERNAL_AK4642_I2C_TEST
#include <linux/sysctl.h>           /* sysctl interface */
#include <linux/timer.h>            /* timer used to control test freq */
#include <asm/atomic.h>             /* atomic variables */
#endif

#include <linux/broadcom/bcmring/gpio_defs.h>
#include <mach/csp/gpiomux.h>

#include "ak4642_regs.h"            /* AK4642 registers */
#include "ext_codec_ak4642_i2c.h"   /* I2C driver for AK4642 */

/* ---- Public Variables ------------------------------------------------- */

/**
 * @addtogroup HAL
 * @{
 */

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */
#define EXTAUDIO_PROC_NAME          "halAudioExternalAK4642"

/* ---- Private Variables ------------------------------------------------ */

/* Default register settings for AK4642 on power up */
static const unsigned char ext_codec_ak4642_DefaultRegs[AK4642_MAX_REGS] =
{
 0,0,1,0,2,0,0,0,0xE1,0xE1,0x18,0,0xE1,0x18,0x11,0x08,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#if 0
/* Mask to indicate power register mapping */
static unsigned char ext_codec_ak4642_ShadowPowerRegsMask[AK4642_MAX_REGS] =
{
   (AK4642_REG_PM1_PMBP | AK4642_REG_PM1_PMSPK | AK4642_REG_PM1_PMDAC),
   (AK4642_REG_PM2_HPMTN | AK4642_REG_PM2_PMHPL | AK4642_REG_PM2_PMHPR),
   (AK4642_REG_SS1_SPPSN | AK4642_REG_SS1_DACS | AK4642_REG_SS1_PMMP),
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   ( AK4642_REG_MC4_DACH ),
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
#endif

static unsigned char ext_codec_ak4642_ShadowRegs[AK4642_MAX_REGS];      /* Shadow registers for AK4642 registers */
static int halUpdateAK4642regs = 0;

static HALAUDIO_EXTERNAL_AK4642_CFG ak4642Cfg;

/* ---- Private Function Prototypes -------------------------------------- */
#ifdef CONFIG_BCM_HALAUDIO_EXTERNAL_AK4642_I2C_TEST
/* For AK4642 I2C stress test */
static void ak4642TestSysCtlInit( void );
static void ak4642TestSysCtlExit( void );
#endif

static int halAudioExternalAK4642ReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data );
static int halAudioExternalAK4642_MicSel( HALAUDIO_HWSEL hwsel );
/* Function read registers of AK4642 (use shadow regs to speed access) */
static inline unsigned char halReadReg( AK4642_REG reg )
{
   return(ext_codec_ak4642_ShadowRegs[reg]);
}

/* Will update the shadow registers and write to i2c if flag enabled */
static void halSetRegVal( AK4642_REG reg, unsigned char val )
{
   if (ext_codec_ak4642_ShadowRegs[reg] != val)
   {
      /* Update the shadow register with new value but check on flag to write to i2c */
      ext_codec_ak4642_ShadowRegs[reg] = val;
      if ( halUpdateAK4642regs )
      {
#if 0
         /* Do not allow for power pins to rise when in interrupt only mode */
         if (gPowerLevel == HAL_AUDIO_POWER_INTERRUPTS_ONLY)
         {
            val = val & ~(ext_codec_ak4642_ShadowPowerRegsMask[reg]);
         }
#endif
         ext_codec_ak4642_i2c_write( reg, val );
      }
   }
}

static inline unsigned char halGetRegBits( AK4642_REG reg, unsigned char val )
{
   return (halReadReg( reg ) & val);
}

static inline void halSetRegBits( AK4642_REG reg, unsigned char val )
{
   halSetRegVal( reg, halReadReg(reg) | val );
}

static inline void halClearRegBits( AK4642_REG reg, unsigned char val )
{
   halSetRegVal( reg, halReadReg(reg) & ~val );
}

/* Intended to force a value regardless of current shadow register value.
 * Will not save to shadow register.
 */
static inline void halForceSetRegVal( AK4642_REG reg, unsigned char val )
{
   ext_codec_ak4642_i2c_write( reg, val );
}

static inline void halForceSetRegBits( AK4642_REG reg, unsigned char val )
{
   halForceSetRegVal( reg, halReadReg(reg) | val );
}

static inline void halForceClearRegBits( AK4642_REG reg, unsigned char val )
{
   halForceSetRegVal( reg, halReadReg(reg) & ~val );
}

#if 0
static inline void halSetPowerRegVal( AK4642_REG reg, unsigned char val )
{
   /* Only allow values that are in the shadow registers */
   unsigned char allowVal = val & halReadReg(reg);
   if (allowVal)
   {
      allowVal |= ext_codec_ak4642_i2c_read(reg);
      ext_codec_ak4642_i2c_write( reg, allowVal );
   }
}

static inline void halSpkrPowerUp( void )
{
   /* Do not allow power up on digital mode.  Sleep mode will not write to I2C but will track changes */
   /*if (gPowerLevel != HAL_AUDIO_POWER_INTERRUPTS_ONLY) */
   {
      halSetRegVal( AK4642_REG_SS1, AK4642_REG_SS1_DACS );
      halSetRegVal( AK4642_REG_MC4, AK4642_REG_MC4_DACH );

      /* We power up the handset and headset speakers, the DAC, and the sidetone (Beep) */
      halSetRegVal( AK4642_REG_PM1, AK4642_REG_PM1_PMDAC | AK4642_REG_PM1_PMSPK | AK4642_REG_PM1_PMBP );

      halSetRegVal( AK4642_REG_PM2, AK4642_REG_PM2_PMHPL );
      halSetRegVal( AK4642_REG_PM2, AK4642_REG_PM2_PMHPR );

      /* Set the speaker and headphone mute during power up */
      halSetRegVal( AK4642_REG_PM2, AK4642_REG_PM2_HPMTN );
      halSetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_SPPSN );
   }
}

static inline void halSpkrPowerDown( void )
{
   /* Set the speaker to power save mode and mute the headphone speaker */
   halForceClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_SPPSN );
   halForceClearRegBits( AK4642_REG_PM2, AK4642_REG_PM2_HPMTN );

   /* Verify if the headphone or the ear speaker is powered or we skip waiting for the fall time from Vdd to ground */
   if ( (halReadReg(AK4642_REG_PM1) & AK4642_REG_PM1_PMSPK)    ||
         (halReadReg(AK4642_REG_PM2) & AK4642_REG_PM2_PMHPL)   ||
         (halReadReg(AK4642_REG_PM2) & AK4642_REG_PM2_PMHPR))
   {
      /* Delay after entering power save mode.  Reduces pop noise.  */
      set_current_state(TASK_INTERRUPTIBLE);
      schedule_timeout(50);
   }

   /* We now power down both headset and handset speakers as well as remove paths to DAC */
   halForceClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_DACS );
   halForceClearRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMDAC | AK4642_REG_PM1_PMSPK | AK4642_REG_PM1_PMBP );
   halForceClearRegBits( AK4642_REG_PM2, AK4642_REG_PM2_PMHPL | AK4642_REG_PM2_PMHPR );
   halForceClearRegBits( AK4642_REG_MC4, AK4642_REG_MC4_DACH );
}

static inline void halMicPowerUp( void )
{
   /* FIXME: do I really need this routine */
   /* if ( gPowerLevel != HAL_AUDIO_POWER_INTERRUPTS_ONLY ) */
   {
      /* Power up mic power for biasing */
      halSetRegVal( AK4642_REG_SS1, AK4642_REG_SS1_PMMP );

      /* Power up mic amp to work-around AK4642 silicon problem
       * where the speaker path is also powered off if the mic is off.
       */
      halAudioExternalAK4642_MicSel( halgMicActive );
   }
}

static inline void halMicPowerDown( void )
{
   halForceClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_PMMP );

   /* Clear PMADL and PMADR bits */
   halClearRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMADL );
   halClearRegBits( AK4642_REG_PM3, AK4642_REG_PM3_PMADR );
}
#endif

/* Clocks must be supplied when ADC and/or DAC are powered up.
 * The following control sequence is supplied by the AK4642 data sheet
 * 1. DIF1-0, FS3-0 and PLL3-0 set (Addrs: 04H and 05H)
 * 2. VCOM powered up (Addr:00H, D6)
 * 3. PMPLL bit set (Addr:01H, D0)
 */
static const int ext_codec_ak4642_reg_init_seq[AK4642_MAX_REGS] =
{
   4,5,0,1,
   31,30,29,28,27,26,25,24,23,22,21,20,
   19,18,17,16,15,14,13,12,11,10,
   9,8,7,6,3,2
};

/**
* This routine is called to setup registers after codec reset
*/
static inline void halLoadShadowRegs( void )
{
   int i, reg;
   for ( i = 0; i < AK4642_MAX_REGS; i++ )
   {
      reg = ext_codec_ak4642_reg_init_seq[i];

      /* Minimize I2C writes. Only write regs that are different from defaults */
      if ( ext_codec_ak4642_ShadowRegs[reg] != ext_codec_ak4642_DefaultRegs[reg] )
      {
         /* Mask out power bits. Blocks will be powered up later when appropriate.
          * This is an important step for initializing the clocks. Read comment
          * above for ext_codec_ak4642_reg_init_seq[].
          */
         unsigned char value = ext_codec_ak4642_ShadowRegs[reg]; /* & ~(ext_codec_ak4642_ShadowPowerRegsMask[reg]); */

         halForceSetRegVal( reg, value );
/*         printk("REG:0x%x VAL:0x%x 0x%x\n", reg, ext_codec_ak4642_ShadowRegs[reg], ext_codec_ak4642_i2c_read( reg ) ); */
      }
      else
      {
/*         printk("DEFREG:0x%x VAL:0x%x 0x%x\n", reg, ext_codec_ak4642_ShadowRegs[reg], ext_codec_ak4642_i2c_read( reg ) ); */
      }
   }
}

/***************************************************************************/
/**
*  Eanble power to hardware
*
*  @return  nothing
*/
void halAudioExternalAK4642_PowerEnable( int enable )
{
   if ( enable == 0 )
   {
#if 0
      /* Ensure proper order of power down */
      halSpkrPowerDown();
      halMicPowerDown();
#endif

      /* Do not allow the I2C to be written from here on */
      halUpdateAK4642regs = 0;
   }
   else
   {
      /* Allow the I2C to be written from here on */
      halUpdateAK4642regs = 1;

      /* Ensure proper order of power up occurs before loading shadow registers */
      /*halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_SPPSN ); */
      halLoadShadowRegs();

#if 0
      /* Ensure proper order of power up */
      halMicPowerUp();
      halSpkrPowerUp();
#endif
   }
}
EXPORT_SYMBOL( halAudioExternalAK4642_PowerEnable );

static short calculateInputDB( unsigned char gain )
{
   short db;

   /* rounding done */
   db = ((((int)gain - 145)*3*2)/8 + 1)/2;

   return(db);
}

static unsigned char calculateInputGain( short db )
{
   if (db > 36)
   {
      /* Maximum is +36 dB */
      db = 36;
   }
   if (db < -54)
   {
      /*
       * Minimum is -54 dB 
       * Mute 
       */
      return(0x00);
   }
   /* rounding done */
   return( (unsigned char)(((db*8*2)/3 + 1)/2 + 145) );
}

static short calculateOutputDB( unsigned char vol )
{
   short db;

   db = 12 - ((int)vol/2) ;

   return(db);
}

static unsigned char calculateOutputGain( short db )
{
   if (db > 12)
   {
      /* Maximum is +12 dB */
      db = 12;
   }
   if (db < -115)
   {
      /*
       * Minimum is -115 dB 
       * Mute 
       */
      return(0xFF);
   }
   return( (unsigned char)((12 - db) * 2) );
}

/* ---- Functions -------------------------------------------------------- */

/***************************************************************************/
/**
*  External audio block int routine. Alloc resources, initialize hardware.
*
*  @return  0 on success, otherwise failure
*/
int halAudioExternalAK4642_init( void )
{
   int error = 0;

   /* reserve GPIO */
   if ( ak4642Cfg.gpio_codec_pdn >= 0 )
   {
      gpiomux_rc_e gpiorc = gpiomux_request( ak4642Cfg.gpio_codec_pdn, chipcHw_GPIO_FUNCTION_GPIO, "AK4642 Reset" );
      if ( gpiorc != gpiomux_rc_SUCCESS )
      {
         printk( KERN_ERR "AK4642: Failed to request GPIO pin rc=%u\n", gpiorc );
         return -1;
      }
      /* Configure external codec GPIO as an output */
      gpio_direction_output( ak4642Cfg.gpio_codec_pdn, 0 );
   }

   /* Put external codec in reset for at least tPD=150ns */
   halAudioExternalAK4642_CodecReset( 1 );

   /* Take it out of reset */
   halAudioExternalAK4642_CodecReset( 0 );

   /* Initialze the shadow registers to the default values they come up in */
   memcpy( ext_codec_ak4642_ShadowRegs, ext_codec_ak4642_DefaultRegs, sizeof(ext_codec_ak4642_DefaultRegs)/sizeof(ext_codec_ak4642_DefaultRegs[0]) );

   /* Initialize the external codec i2c interface, AK4642 must be initialized first */
   error = ext_codec_ak4642_ic_init();
   if (error != 0)
   {
      printk( KERN_ERR "halAudioExternal: Failed to initialize AK4642\n" );
      return( error );
   }

   if ( ak4642Cfg.gpio_codec_pdn >= 0 )
   {
      /* Put external codec in reset until enabled */
      gpio_set_value( ak4642Cfg.gpio_codec_pdn, 0 );
   }

   /* Configure all codec registers: ak4642 PLL slave mode with BICK as reference */

   /* Set PMVCM bit of reg 00 */
   halSetRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMVCM );

   /* Set PMPLL bit of reg 01 for PLL Slave Mode 1 */
   halSetRegBits( AK4642_REG_PM2, AK4642_REG_PM2_PMPLL );

   /* Clear default mic gain to 0dB reg 02 */
   halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_MGAIN0 );

   /* Set clocking options in Mode Control 1 register, reg 04:
    * PLL Mode with BICK as reference (32fs)
    * Audio interface format is Tx/Rx I2S compatible.
    */
   halSetRegVal( AK4642_REG_MC1, 0x23 );

   /* 16 kHz sampling, reg 05 */
   halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_8KHZ );

   /* Set the Lch and Rch Input Digital Volume to 0 dB */
   halSetRegVal( AK4642_REG_IVL, 0x91 );
   halSetRegVal( AK4642_REG_IVR, 0x91 );

   /* Independent gain control for Lch and Rch */
   halClearRegBits( AK4642_REG_MC3, AK4642_REG_DVOLC );

   /* Configure for differential microphone input */
   halSetRegBits( AK4642_REG_PM3, AK4642_REG_PM3_MDIF1 | AK4642_REG_PM3_MDIF2 );

   /* Create debug proc entries */
   create_proc_read_entry( EXTAUDIO_PROC_NAME, 0, NULL, halAudioExternalAK4642ReadProc, NULL );

#ifdef CONFIG_BCM_HALAUDIO_EXTERNAL_AK4642_I2C_TEST
   ak4642TestSysCtlInit();
#endif

   return error;
}
EXPORT_SYMBOL( halAudioExternalAK4642_init );

/***************************************************************************/
/**
*  External audio block exit routine. Frees resources.
*
*  @return  0 on success, otherwise failure
*/
int halAudioExternalAK4642_exit( void )
{
#ifdef CONFIG_BCM_HALAUDIO_EXTERNAL_AK4642_I2C_TEST
   ak4642TestSysCtlExit();
#endif

   if ( ak4642Cfg.gpio_codec_pdn >= 0 )
   {
      gpiomux_free( ak4642Cfg.gpio_codec_pdn );
   }
   remove_proc_entry( EXTAUDIO_PROC_NAME, NULL );
   ext_codec_ak4642_ic_release();
   return 0;
}
EXPORT_SYMBOL( halAudioExternalAK4642_exit );

/***************************************************************************/
/**
*  halAudioExternalCustom_CodecReset - put the codec in reset or not
*
*  @return nothing
*/
void halAudioExternalAK4642_CodecReset( int reset )
{
   (void) reset;

   if ( ak4642Cfg.gpio_codec_pdn >= 0 )
   {
      if ( reset )
      {
         /* Put external codec in reset */
         if ( gpio_get_value( ak4642Cfg.gpio_codec_pdn ) != 0 )
         {
            gpio_set_value( ak4642Cfg.gpio_codec_pdn, 0 );

            set_current_state( TASK_INTERRUPTIBLE );  
            schedule_timeout( 1 );
         }
      }
      else
      {
         /* Take it out of reset */
         if ( gpio_get_value( ak4642Cfg.gpio_codec_pdn ) == 0 )
         {
            gpio_set_value( ak4642Cfg.gpio_codec_pdn, 1 );

            set_current_state( TASK_INTERRUPTIBLE );
            schedule_timeout( 1 );
         }
      }
   }
}
EXPORT_SYMBOL( halAudioExternalAK4642_CodecReset );

/***************************************************************************/
/**
*  HAL audio gain set routine used to set hardware gains for the chosen
*  block.  Setting the gain setting to Mute/Sleep powers down the block,
*  setting the gain to a valid setting powers up the block.
*
*  @return  0 on success, otherwise failure
*/
int halAudioExternalAK4642_GainSetAnalogHardware(
   int             db,        /**< (i) Gain in dB */
   HALAUDIO_DIR    dir,       /**< (i) Selects direction */
   HALAUDIO_HWSEL  hwsel      /**< (i) Selects one of two PGA blocks */
)
{
   int error = 0;

   if ( dir != HALAUDIO_DIR_ADC && dir != HALAUDIO_DIR_DAC )
   {
      return -EINVAL;
   }

   if ( dir == HALAUDIO_DIR_ADC )
   {
      unsigned int mgain1 = 0;
      unsigned int mgain0 = 0;

      switch ( db )
      {
         case 0:
         {
            mgain1 = 0;
            mgain0 = 0;
         }
         break;
         case 20:
         {
            mgain1 = 0;
            mgain0 = 1;
         }
         break;
         case 26:
         {
            mgain1 = 1;
            mgain0 = 0;
         }
         break;
         case 32:
         {
            mgain1 = 1;
            mgain0 = 1;
         }
         break;
         default:
         {
            if (db > HALAUDIO_GAIN_MUTE)
            {
               /* Invalid gain setting */
               error = -2;
            }
            else
            {
               /* Power down setting */
            }
         }
         break;
      }
      if ( !error )
      {
         if ( db > HALAUDIO_GAIN_MUTE )
         {
            /* Write mgain bits to registers */
            if ( mgain0 )
            {
               halSetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_MGAIN0 );
            }
            else
            {
               halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_MGAIN0 );
            }
            if ( mgain1 )
            {
               halSetRegBits( AK4642_REG_SS2, AK4642_REG_SS2_MGAIN1 );
            }
            else
            {
               halClearRegBits( AK4642_REG_SS2, AK4642_REG_SS2_MGAIN1 );
            }

            halSetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_PMMP );

            /* Write ivol setting  */
            halAudioExternalAK4642_MicSel( hwsel );
         }
         else
         {
            if (   halReadReg(AK4642_REG_SS1) & AK4642_REG_SS1_PMMP )
            {
               /* Power down the block */
               halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_PMMP );
#if 0
               /* Warning: Do not affect the PMADL and PMADR bits, because
                * doing so seems to also bring down the speaker path. This
                * appears to be a silicon issue with the AK4642. The work-
                * around is when audio is fully powered, either PMADL or
                * PMADR bits will be set even though the mic is set to
                * sleep.
                */
               /* Clear PMADL and PMADR bits */
               halClearRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMADL );
               halClearRegBits( AK4642_REG_PM3, AK4642_REG_PM3_PMADR );
#endif
            }
         }
      }

   }
   else if ( dir == HALAUDIO_DIR_DAC )
   {
      if ( hwsel == HALAUDIO_HWSEL_A )         /* primary handset speaker */
      {
         unsigned char spkg = 0;

         switch ( db )
         {
            case 4:
            {
               /* Set Speaker gain to 4.43 dB */
               spkg = AK4642_REG_SS2_SPKG_4_43DB;
            }
            break;
            case 6:
            {
               /* Set Speaker gain to 6.43 dB */
               spkg = AK4642_REG_SS2_SPKG_6_43DB;
            }
            break;
            case 10:
            {
               /* Set Speaker gain to 10.65 dB */
               spkg = AK4642_REG_SS2_SPKG_10_65DB;
            }
            break;
            case 12:
            {
               /* Set Speaker gain to 12.65 dB */
               spkg = AK4642_REG_SS2_SPKG_12_65DB;
            }
            break;
            default:
            {
               if (db > HALAUDIO_GAIN_MUTE)
               {
                  /* Invalid gain setting */
                  error = -2;
               }
               else
               {
                  /* Power down setting */
               }
            }
            break;
         }

         if ( !error )
         {
            if (db > HALAUDIO_GAIN_MUTE)
            {
               /* Set SPKG1-0 bits */
               halSetRegVal( AK4642_REG_SS2, (halReadReg( AK4642_REG_SS2 ) & ~AK4642_REG_SS2_SPKG_MASK) | spkg );
               /* Connect the speaker to the DAC */
               halSetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_DACS );

               /*halgSpkActive = block; */

               /*If speaker is unpowered, follow proper power up procedures*/
               if (!(halReadReg(AK4642_REG_SS1) & AK4642_REG_PM1_PMSPK) )
               {
                  halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_SPPSN );

                  /* We power up the handset and headset speakers, the DAC, and the sidetone (Beep) */
                  halSetRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMDAC | AK4642_REG_PM1_PMSPK | AK4642_REG_PM1_PMBP );
                  halSetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_SPPSN );
               }
            }
            else
            {
               /* Power save mode of speaker amp */

               halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_SPPSN);
#if 0
               /* Delay to allow fall in output */
               if (  gPowerLevel != HAL_AUDIO_POWER_INTERRUPTS_ONLY &&
                     (halReadReg(AK4642_REG_PM1) & AK4642_REG_PM1_PMSPK) != 0 &&
                     halgSpkActive == HAL_AUDIO_EAR_SPKR )
               {
                  set_current_state(TASK_INTERRUPTIBLE);
                  schedule_timeout(50);
               }
#endif
               halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_DACS );

               if (db == HALAUDIO_GAIN_MUTE)
               {
                  /* Clear PMDAC bit of reg 00 (turn off DAC if aux_spk also not powered) */
                  if ( (halReadReg(AK4642_REG_PM2) & AK4642_REG_PM2_PMHPR) == 0 )
                  {
                     /*
                      * Clear PMDAC bit of reg 00 
                      * Clear PMSPK bit of reg 00 
                      */
                     halClearRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMDAC | AK4642_REG_PM1_PMSPK );
                  }
                  else
                  {
                     /* Just Clear PMSPK bit of reg 00 */
                     halClearRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMSPK );
                  }
               }
            }
         }
      }
      else if ( hwsel == HALAUDIO_HWSEL_B )       /* Auxiliary speaker */
      {
         switch( db )
         {
            case 0:
            case 3:
            {
               /* Connect DAC output to headphone amp */
               halSetRegBits( AK4642_REG_MC4, AK4642_REG_MC4_DACH );

               if (db)
               {
                  /*
                   * Set HPG bit 
                   * Set Headphone gain to +3.6 dB 
                   */
                  halSetRegBits( AK4642_REG_PM3, AK4642_REG_PM3_HPG );
               }
               else
               {
                  /*
                   * Clear HPG bit 
                   * Set Headphone gain to 0 dB 
                   */
                  halClearRegBits( AK4642_REG_PM3, AK4642_REG_PM3_HPG );
               }

               /*
                * Power up the block 
                * Set PMDAC bit of reg 00 
                */
               /*if (gPowerLevel == HAL_AUDIO_POWER_RESUME_ALL) */
               {
                  halSetRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMDAC );

#if 1
                  /* Stereo mode */
                  {
                     /* Set PMHPL and PMHPR bit of reg 01 */
                     halSetRegBits( AK4642_REG_PM2, AK4642_REG_PM2_PMHPR | AK4642_REG_PM2_PMHPL );
                  }
#else
                  /* mono mode */
                  {
                     /* Clear PMHPL bit of reg 01 */
                     halClearRegBits( AK4642_REG_PM2, AK4642_REG_PM2_PMHPL );

                     /* Set PMHPR bit of reg 01 */
                     halSetRegBits( AK4642_REG_PM2, AK4642_REG_PM2_PMHPR );
                  }
#endif
                  /* Set HPMTN bit of reg 01 (Normal - unmute) */
                  halSetRegBits(AK4642_REG_PM2, AK4642_REG_PM2_HPMTN);
               }
            }
            break;

            case HALAUDIO_GAIN_MUTE:
            {
               /*
                * Power down the block 
                * Clear HPMTN bit of reg 01 (MUTE) 
                */
               halClearRegBits(AK4642_REG_PM2, AK4642_REG_PM2_HPMTN);
#if 0
               /*Delay to allow fall in output */
               if (  gPowerLevel != HAL_AUDIO_POWER_INTERRUPTS_ONLY  &&
                     (halReadReg(AK4642_REG_PM2) & (AK4642_REG_PM2_PMHPR | AK4642_REG_PM2_PMHPL)) != 0 &&
                      halgSpkActive == HAL_AUDIO_AUX_SPKR
                  )
               {
                  set_current_state(TASK_INTERRUPTIBLE);
                  schedule_timeout(50);
               }
#endif
               /* Clear PMHPR bit of reg 01 (turn off headphone block) */
               halClearRegBits( AK4642_REG_PM2, AK4642_REG_PM2_PMHPR | AK4642_REG_PM2_PMHPL );
               if ( (halReadReg(AK4642_REG_PM1) & (AK4642_REG_PM1_PMSPK)) == 0 )
               {
                  /* Clear PMDAC bit of reg 00 (turn off DAC if ear_spk also not powered) */
                  halClearRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMDAC );
               }
               /* Disconnect DAC output to headphone amp */
               halClearRegBits( AK4642_REG_MC4, AK4642_REG_MC4_DACH );
            }
            break;

            default:
            {
               /* Invalid gain setting */
               error = -3;
            }
            break;
         }
      }
      else
      {
         error = -EINVAL;
      }
   }
   else
   {
      error = -EINVAL;
   }
   return error;
}
EXPORT_SYMBOL( halAudioExternalAK4642_GainSetAnalogHardware );

/***************************************************************************/
/**
*  HAL Get Analog Hardware gain information.
*
*  @return  0 on success, otherwise failure
*/
int halAudioExternalAK4642_GainGetAnalogHardware(
   HALAUDIO_GAIN_INFO  *info,       /*<< (o) Ptr to gain info structure */
   HALAUDIO_DIR         dir,        /*<< (i) Direction path */
   HALAUDIO_HWSEL       hwsel       /*<< (i) Hardware mux selection */
)
{
   if ( dir != HALAUDIO_DIR_ADC && dir != HALAUDIO_DIR_DAC )
   {
      return -EINVAL;
   }

   if ( dir == HALAUDIO_DIR_DAC )
   {
      if ( hwsel == HALAUDIO_HWSEL_A )         /* primary handset speaker */
      {
         if ( !halGetRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMSPK ) )
         {
            info->currentdb     = HALAUDIO_GAIN_MUTE;
         }
         else
         {
            unsigned char regValue = halGetRegBits( AK4642_REG_SS2, AK4642_REG_SS2_SPKG_MASK );

            if ( regValue == AK4642_REG_SS2_SPKG_4_43DB )
               info->currentdb  = 4;
            else if ( regValue == AK4642_REG_SS2_SPKG_6_43DB )
               info->currentdb  = 6;
            else if ( regValue == AK4642_REG_SS2_SPKG_10_65DB )
               info->currentdb  = 10;
            else if ( regValue == AK4642_REG_SS2_SPKG_12_65DB )
               info->currentdb  = 12;
         }

         info->mindb            = 4;
         info->maxdb            = 12;
         info->range_type       = HALAUDIO_RANGETYPE_LIST;

         info->range.list.num   = 4;
         info->range.list.db[0] = 4;
         info->range.list.db[1] = 6;
         info->range.list.db[2] = 10;
         info->range.list.db[3] = 12;
      }
      else
      {
         if ( !halGetRegBits( AK4642_REG_PM2, AK4642_REG_PM2_HPMTN ) )
         {
            info->currentdb    = HALAUDIO_GAIN_MUTE;
         }
         else
         {
            if ( halGetRegBits( AK4642_REG_PM3, AK4642_REG_PM3_HPG ) )
               info->currentdb = 3;
            else
               info->currentdb = 0;
         }

         info->mindb            = 0;
         info->maxdb            = 3;
         info->range_type       = HALAUDIO_RANGETYPE_LIST;

         info->range.list.num   = 2;
         info->range.list.db[0] = 0;
         info->range.list.db[1] = 3;
      }
   }
   else
   {
      /* ADC */

      /* hwsel not used */

      if ( !halGetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_PMMP ) )
      {
         info->currentdb     = HALAUDIO_GAIN_MUTE;
      }
      else
      {
         unsigned char mgain0, mgain1;

         mgain0 = halGetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_MGAIN0 );
         mgain1 = halGetRegBits( AK4642_REG_SS2, AK4642_REG_SS2_MGAIN1 );

         if ( !mgain1 && !mgain0 )
            info->currentdb  = 0;
         else if ( !mgain1 && mgain0 )
            info->currentdb  = 20;
         else if ( mgain1 && !mgain0 )
            info->currentdb  = 26;
         else if ( mgain1 && mgain0 )
            info->currentdb  = 32;
      }

      info->mindb            = 0;
      info->maxdb            = 32;
      info->range_type       = HALAUDIO_RANGETYPE_LIST;

      info->range.list.num   = 4;
      info->range.list.db[0] = 0;
      info->range.list.db[1] = 20;
      info->range.list.db[2] = 26;
      info->range.list.db[3] = 32;
   }

   return 0;
}
EXPORT_SYMBOL( halAudioExternalAK4642_GainGetAnalogHardware );

/***************************************************************************/
/**
*  HAL Set Digital Hardware gain setting.
*
*  @return  0 on success, otherwise failure
*/
int halAudioExternalAK4642_GainSetDigitalHardware(
   int             db,        /**< (i) Gain in dB */
   HALAUDIO_DIR    dir        /**< (i) Selects direction */
)
{
   int error = 0;

   if ( dir == HALAUDIO_DIR_ADC )
   {
      halSetRegVal( AK4642_REG_IVL, calculateInputGain(db) );
      halSetRegVal( AK4642_REG_IVR, calculateInputGain(db) );
   }
   else if ( dir == HALAUDIO_DIR_DAC )
   {
      halSetRegVal( AK4642_REG_DVL, calculateOutputGain(db) );
      halSetRegVal( AK4642_REG_DVR, calculateOutputGain(db) );
   }
   else
   {
      error = -EINVAL;
   }

   return error;
}
EXPORT_SYMBOL( halAudioExternalAK4642_GainSetDigitalHardware );

/***************************************************************************/
/**
*  HAL Get Digital Hardware gain setting.
*
*  @return  0 on success, otherwise failure
*/
int halAudioExternalAK4642_GainGetDigitalHardware(
   HALAUDIO_GAIN_INFO  *info,       /*<< (o) Ptr to gain info structure */
   HALAUDIO_DIR         dir         /*<< (i) Direction path */
)
{
   unsigned char regValue;

   if ( dir == HALAUDIO_DIR_ADC )
   {
      info->mindb             = -54;
      info->maxdb             = 36;
      info->range_type        = HALAUDIO_RANGETYPE_FIXED_STEPSIZE;
      info->range.fixed_step  = 1;

      /* Assume L/R is configured with the same gain */
      regValue = halReadReg( AK4642_REG_IVL );

      if( regValue == 0 )
         info->currentdb = HALAUDIO_GAIN_MUTE;
      else
         info->currentdb = calculateInputDB( regValue );
   }
   else if ( dir == HALAUDIO_DIR_DAC )
   {
      info->mindb             = -115;
      info->maxdb             = 12;
      info->range_type        = HALAUDIO_RANGETYPE_FIXED_STEPSIZE;
      info->range.fixed_step  = 1;

      /* Assume L/R is configured with the same gain */
      regValue = halReadReg( AK4642_REG_DVL );

      if( regValue == 0xFF )
         info->currentdb = HALAUDIO_GAIN_MUTE;
      else
         info->currentdb = calculateOutputDB( regValue );
   }

   return 0;
}
EXPORT_SYMBOL( halAudioExternalAK4642_GainGetDigitalHardware );

/***************************************************************************/
/**
*  Set sidetone gain. Mute may be set with db=HALAUDIO_GAIN_MUTE.
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remarks
*/
int halAudioExternalAK4642_SidetoneSet(
   int      db                      /**< (i)    Gain in db */
)
{
   if ( db <= HALAUDIO_GAIN_MUTE )
   {
      /*
       * Turn off sidetone 
       * Clear BEEPS bit of reg 02 
       */
      halClearRegBits( AK4642_REG_SS1, AK4642_REG_SS1_BEEPS );
   }
   else
   {
      /* Turn on sidetone */
      halSetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_BEEPS );
   }

   return 0;
}
EXPORT_SYMBOL( halAudioExternalAK4642_SidetoneSet );

/***************************************************************************/
/**
*  Query sidetone gain info. Current gain and other gain information
*  are returned.
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remarks
*/
#define HALAUDIO_EXTERNAL_AK4642_SIDETONE_GAIN    0
int halAudioExternalAK4642_SidetoneGet(
   HALAUDIO_GAIN_INFO  *info        /*<< (o) Ptr to gain info structure */
)
{
   if( !halGetRegBits( AK4642_REG_SS1, AK4642_REG_SS1_BEEPS ) )
      info->currentdb      = HALAUDIO_GAIN_MUTE;
   else
      info->currentdb      = HALAUDIO_EXTERNAL_AK4642_SIDETONE_GAIN;

   info->mindb             = HALAUDIO_EXTERNAL_AK4642_SIDETONE_GAIN;
   info->maxdb             = HALAUDIO_EXTERNAL_AK4642_SIDETONE_GAIN;
   info->range_type        = HALAUDIO_RANGETYPE_FIXED_STEPSIZE;
   info->range.fixed_step  = 0;

   return 0;
}
EXPORT_SYMBOL( halAudioExternalAK4642_SidetoneGet );

/***************************************************************************/
/**
*  HAL microphone select routine.
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternalAK4642_MicSel(
   HALAUDIO_HWSEL hwsel      /**< (i) select mic input */
)
{
   int error = 0;

   if ( hwsel == HALAUDIO_HWSEL_A )
   {
      /* Select channel 1 PMADL=1 PMADR=0 */
      halSetRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMADL );
      halClearRegBits( AK4642_REG_PM3, AK4642_REG_PM3_PMADR );
   }
   else if ( hwsel == HALAUDIO_HWSEL_B )
   {
      /* Select channel 2 PMADL=0 PMADR=1 */
      halClearRegBits( AK4642_REG_PM1, AK4642_REG_PM1_PMADL );
      halSetRegBits( AK4642_REG_PM3, AK4642_REG_PM3_PMADR );
   }
   else
   {
      error = -EINVAL;
   }

   return error;
}

/***************************************************************************/
/**
*  HAL audio Set frequency
*
*  @return  0 on success, otherwise failure
*/
int halAudioExternalAK4642_SetFrequency( int freqHz )
{
   int result = 0;

   /*
   ** Set sampling rate frequency of hardware
   */
   switch ( freqHz )
   {
      case 8000:
      {
         halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_8KHZ );
      }
      break;

      case 16000:
      {
         halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_16KHZ );
      }
      break;

      case 22050:
      {
         halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_22_05KHZ );
      }
      break;

      case 24000:
      {
         halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_24KHZ );
      }
      break;

      case 32000:
      {
         halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_32KHZ );
      }
      break;

      case 44100:
      {
         halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_44_1KHZ );
      }
      break;

      case 48000:
      {
         halSetRegVal( AK4642_REG_MC2, AK4642_REG_MC2_48KHZ );
      }
      break;

      default:
      {
         result = -EINVAL;
      }
      break;
   }
   return result;
}
EXPORT_SYMBOL( halAudioExternalAK4642_SetFrequency );

/***************************************************************************/
/**
*  Proc read callback function
*
*  @return  Number of characters to print
*/
static int halAudioExternalAK4642ReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   int len = 0;
   int i;

   (void) start; (void) offset; (void) count; (void) data; /* avoid compiler warning */                         /* Cache volatile buffers before printing */

   len += sprintf( buf+len, "AK4642 SHADOW REGISTERS:\n" );
   for (i = 0; i < 16; i++)
   {
      len += sprintf( buf+len, "0x%2.2x ", halReadReg(i) );
   }
   len += sprintf( buf+len, "\n" );
   for (i = 16; i < 32; i++)
   {
      len += sprintf( buf+len, "0x%2.2x ", halReadReg(i) );
   }
   len += sprintf( buf+len, "\n" );

   len += sprintf( buf+len, "AK4642 REGISTERS:\n" );
   if ( ak4642Cfg.gpio_codec_pdn >= 0 && gpio_get_value( ak4642Cfg.gpio_codec_pdn ) == 0)
   {
      len += sprintf( buf+len, "AK4642 in reset\n" );
   }
   else
   {
      for (i = 0; i < 16; i++)
      {
         len += sprintf( buf+len, "0x%2.2x ", ext_codec_ak4642_i2c_read(i) );
      }
      len += sprintf( buf+len, "\n" );
      for (i = 16; i < 32; i++)
      {
         len += sprintf( buf+len, "0x%2.2x ", ext_codec_ak4642_i2c_read(i) );
      }
      len += sprintf( buf+len, "\n" );
   }

   *eof = 1;
   return len+1;
}

#ifdef CONFIG_BCM_HALAUDIO_EXTERNAL_AK4642_I2C_TEST
/***************************************************************************/
/**
*  Debug sysctl interface data variables used to stress test
*  reading and writing AK4642 register over I2C.
*/

static int ak4642_read_errors;         /* Number of read errors */
static int ak4642_write_errors;        /* Number of write errors */
static int ak4642_test_count;          /* Number of r/w tests executed */
static int ak4642_rw_per_tick;         /* Number of JIFFIES between tests */

/* Copy of registers used for read/write compare */
static unsigned int ak4642_regs_copy[AK4642_MAX_REGS];

static struct timer_list ak4642_timer; /* Timer control variables */
static atomic_t ak4642_stop_timer;

static DECLARE_COMPLETION( gThreadDone );
static int quitNow;
static struct semaphore gI2cTestSem;

static int i2cTestThread( void *arg );

static int proc_do_rwI2CPerTick(ctl_table *table, int write, struct file *filp,
      void __user *buffer, size_t *lenp, loff_t *ppos);

#define CTL_TABLE_INT(varStr,var) \
      procname: varStr,\
      data: &var,\
      maxlen: sizeof(int),\
      mode: 0644,\
      proc_handler: &proc_dointvec,

#define CTL_TABLE_INT_ARRAY(varStr,var) \
      procname: varStr,\
      data: var,\
      maxlen: sizeof(var),\
      mode: 0644,\
      proc_handler: &proc_dointvec,

static struct ctl_table gSysCtlAk4642[] =
{
   { CTL_TABLE_INT("rd_errs", ak4642_read_errors) },
   { CTL_TABLE_INT("wr_errs", ak4642_write_errors) },
   { CTL_TABLE_INT("test_count", ak4642_test_count) },
   { CTL_TABLE_INT_ARRAY("regs", ak4642_regs_copy) },
   {
      procname: "rwtest_per_tick",
      data: &ak4642_rw_per_tick,
      maxlen: sizeof(int),
      mode: 0644,
      proc_handler: &proc_do_rwI2CPerTick,
   },
   {}
};
static struct ctl_table gSysCtl[] =
{
   /* Hard-code a sysctl ID of 250 */
   {
      .procname   = "ak4642_i2c_test",
      .mode       = 0555,
      .child      = gSysCtlAk4642,
   },
   {}
};
static struct ctl_table_header *gSysCtlHeader;

/***************************************************************************/
/**
*  Timer expiry routine used to read and write registers over I2C. The timer
*  expires every ak4642_rw_per_tick jiffies.
*/
static void ak4642RWTimedOut( unsigned long data )
{
   (void) data;

   /* Signal thread to run */
   up( &gI2cTestSem );

   /* Re-trigger timer */
   if ( !atomic_read( &ak4642_stop_timer ))
   {
      ak4642_timer.expires = jiffies + ak4642_rw_per_tick;
      add_timer( &ak4642_timer );
   }
}

/***************************************************************************/
/**
*  Cleanup debug timer
*/
static void ak4642DelTimer( void )
{
   if ( !atomic_read( &ak4642_stop_timer ))
   {
      atomic_set( &ak4642_stop_timer, 1 );
      del_timer( &ak4642_timer );   /* Force removal of timer */
   }
}

/***************************************************************************/
/**
*  Sysctl method for starting and stopping I2C test
*/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_do_rwI2CPerTick(ctl_table *table, int write,
      void __user *buffer, size_t *lenp, loff_t *ppos)
#else
static int proc_do_rwI2CPerTick(ctl_table *table, int write, struct file *filp,
      void __user *buffer, size_t *lenp, loff_t *ppos)
#endif
{
   int rc;

   /* Process integer operation */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
   rc = proc_dointvec( table, write, buffer, lenp, ppos );
#else
   rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );
#endif

   if ( write )
   {
      /* Configure timer to read AK4642 registers over I2C */

      if ( ak4642_rw_per_tick == 0 )
      {
         ak4642DelTimer();
      }
      else
      {
         /* Launch timer if not already running */
         if ( atomic_read( &ak4642_stop_timer ))
         {
            int i;

            /* Take AK4642 out of reset */
            halAudioExternalAK4642_CodecReset( 0 );

            /* Get copy of registers as basis for test comparison */
            for ( i = 0; i < AK4642_MAX_REGS; i++ )
            {
               ak4642_regs_copy[i] = ext_codec_ak4642_i2c_read( i );
            }

            init_timer( &ak4642_timer );
            ak4642_timer.function = ak4642RWTimedOut;
            ak4642_timer.data = 0;
            ak4642_timer.expires = jiffies + ak4642_rw_per_tick;
            atomic_set( &ak4642_stop_timer, 0 );
            add_timer( &ak4642_timer );
         }
      }
   }
   return rc;
}

/***************************************************************************/
/**
*  I2C test thread
*
*  @return  0 on success, otherwise failure
*/
static int i2cTestThread( void *arg )
{
   (void)arg;                       /* Unused */

   daemonize( "i2ctest" );

   while ( 1 )
   {
      unsigned int regval;
      int i;
      int err;

      err = down_interruptible( &gI2cTestSem );
      if ( err || quitNow )
      {
         /* Task is interrupted by signal or quitting */
         break;
      }

      ak4642_test_count++;

      /* Read all AK4642 registers and check for correct data */
      for ( i = 0; i < AK4642_MAX_REGS; i++ )
      {
         regval = ext_codec_ak4642_i2c_read( i );
         if ( regval != ak4642_regs_copy[i] )
         {
            ak4642_read_errors++;
            /*printk( KERN_ERR "AK4642 I2C Test: reg=%i read 0x%x expect 0x%x\n", i, regval, ak4642_regs_copy[i] ); */
         }
      }

#define AK4642_TEST_REG       0x09     /* Lch Input Vol control */
#define AK4642_TEST_VALUE     0x5a

      /* Test write */
      ext_codec_ak4642_i2c_write( AK4642_TEST_REG, AK4642_TEST_VALUE );
      regval = ext_codec_ak4642_i2c_read( AK4642_TEST_REG );
      if ( regval != AK4642_TEST_VALUE )
      {
         ak4642_write_errors++;
      }
      /* Restore register */
      ext_codec_ak4642_i2c_write( AK4642_TEST_REG, ak4642_regs_copy[AK4642_TEST_REG] );
   }

   complete_and_exit( &gThreadDone, 0 );
   return 0;
}

/***************************************************************************/
/**
*  Initialize SysCtl.
*
*  @return  none
*
*  @remarks
*/
static void ak4642TestSysCtlInit( void )
{
   int result;

   quitNow = 0;

   gSysCtlHeader = register_sysctl_table( gSysCtl );

   init_completion( &gThreadDone );
   sema_init( &gI2cTestSem, 0 );

   result = kernel_thread( i2cTestThread, NULL, CLONE_KERNEL );
   if ( result < 0 )
   {
      printk( KERN_ERR "AK4642: Error failed to create test thread\n" );
   }

   /* Timer initially stopped */
   atomic_set( &ak4642_stop_timer, 1 );
}

/***************************************************************************/
/**
*  Cleanup SysCtl.
*
*  @return  none
*
*  @remarks
*/
static void ak4642TestSysCtlExit( void )
{
   ak4642DelTimer();

   quitNow = 1;
   up( &gI2cTestSem );
   wait_for_completion( &gThreadDone );

   /* Put AK4642 in reset */
   halAudioExternalAK4642_CodecReset( 1 );

   unregister_sysctl_table( gSysCtlHeader );
}
#endif   /* CONFIG_BCM_HALAUDIO_EXTERNAL_AK4642_I2C_TEST */

static int ak4642_probe( struct platform_device *pdev )
{
   printk( KERN_INFO "HAL Audio External AK4642 Driver: 1.0. Built %s %s\n", __DATE__, __TIME__ );

   /* Grab platform configuration */
   if ( pdev->dev.platform_data == NULL )
   {
      printk( KERN_ERR "%s: missing platform_data\n",  __FUNCTION__ );
      return -ENODEV;
   }
   memcpy( &ak4642Cfg, pdev->dev.platform_data, sizeof( ak4642Cfg ));

   printk( KERN_INFO "AK4642 Codec PDN gpio=%d\n", ak4642Cfg.gpio_codec_pdn );

   return 0;
}

static int ak4642_remove( struct platform_device *pdev )
{
   return 0;
}

static struct platform_driver ak4642_driver =
{
   .driver =
   {
      .name = "bcm-halaudio-ak4642",
      .owner = THIS_MODULE,
   },
   .probe = ak4642_probe,
   .remove = ak4642_remove,
};

/***************************************************************************/
/**
*  Driver initialization called when module loaded by kernel
*
*  @return
*     0              Success
*     -ve            Error code
*/
static int __init ak4642_init( void )
{
   return platform_driver_register( &ak4642_driver );
}

/***************************************************************************/
/**
*  Driver destructor routine.
*/
static void __exit ak4642_exit( void )
{
   platform_driver_unregister( &ak4642_driver );
}

module_init( ak4642_init );
module_exit( ak4642_exit );

MODULE_AUTHOR( "Broadcom" );
MODULE_DESCRIPTION( "HAL Audio External AK4642 Driver" );
MODULE_VERSION( "1.0" );
MODULE_LICENSE( "GPL" );

/** @} */
