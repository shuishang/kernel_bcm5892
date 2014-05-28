/*****************************************************************************
* Copyright 2008 - 2009 Broadcom Corporation.  All rights reserved.
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
*  halaudio_i2s.c
*
*  PURPOSE:
*
*     This file contains the I2S driver routines.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------- */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <linux/errno.h>
#include <linux/proc_fs.h>                   /* For /proc/audio */
#include <linux/sysctl.h>
#include <linux/device.h>
#include <linux/dmapool.h>                   /* Linux DMA API */
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/broadcom/halaudio.h>
#include <linux/broadcom/halaudio_lib.h>
#include <linux/broadcom/amxr.h>
#include <linux/broadcom/halaudio_external_ak4642.h> /* AK4642 external codec API */
#include <linux/clk.h>

#include <mach/dma.h>
#include <mach/csp/i2sHw_reg.h>
#include <mach/csp/chipcHw_inline.h>
#include <mach/csp/gpiomux.h>
#include <csp/i2sHw.h>

#include "i2s_drv.h"
#include "dma_priv.h"

/* ---- Public Variables ------------------------------------- */
/* ---- Private Constants and Types -------------------------- */

/* Hardware definitions (should be in i2sHw_reg.h) */
#define I2S_FIFO_SIZE                     512   /* In bytes */

/* DMA constants */
#define I2S_MAX_DMA_BUFFER_SIZE_BYTES     (2*480) /* 5ms 48kHz stereo frames */
#define I2S_DMA_ALIGN_IN_BYTES            8     /* 64-bit alignment */
#define I2S_DMA_ALLOC_CROSS_RESTRICT      1024  /* Buffers cannot cross 1kb boundary */

/* Default sampling frequency configurations */
#define I2S_DEFAULT_SAMP_FREQ             16000 /* In Hz */
#define I2S_DEFAULT_FRAME_PERIOD          5000  /* In usec */
#define I2S_DEFAULT_FRAME_SIZE            160   /* In bytes, cannot exceed MAX_DMA_BUFFER_SIZE_BYTES */
#define I2S_DEFAULT_MIXER_FREQ            HAL_MIXER_SIGTYPE_16K /* Must be consistent with I2S_DEFAULT_SAMP_FREQ */
#define I2S_SAMP_WIDTH                    2     /* Each sample in bytes */

/* Priming */
#define I2S_MAX_PRIME_SIZE_BYTES          (I2S_MAX_DMA_BUFFER_SIZE_BYTES * 2)
#define I2S_DEFAULT_PRIME_PERIOD          5000  /* In usec, cannot be more than I2S_DEFAULT_FRAME_PERIOD */
#define I2S_DEFAULT_PRIME_SIZE_BYTES      2*I2S_DEFAULT_FRAME_SIZE   /* Stereo */
#define I2S_MAX_PRIME_PERIOD              I2S_DEFAULT_FRAME_PERIOD   /* Do not change */

#if (I2S_DEFAULT_PRIME_PERIOD > I2S_MAX_PRIME_PERIOD)
#error Default prime period cannot exceed I2S_MAX_PRIME_PERIOD
#endif

/* I2S physical bi-directional DMA FIFO addresses */
#define I2S_CHAN0_DMAFIFO_PHYS_ADDR       (MM_ADDR_IO_I2S0 + i2sHw_I2S0_DMA_FIFO_OFFSET)
#define I2S_CHAN1_DMAFIFO_PHYS_ADDR       (MM_ADDR_IO_I2S1 + i2sHw_I2S1_DMA_FIFO_OFFSET)

/* CSX Data stucture */
struct i2s_csx_data
{
   CSX_IO_POINT_FNCS    csx_ops;
   void                *priv;
};

/* I2S error statistics */
struct i2s_ch_errs
{
   unsigned int         dma_egr;          /* DMA egress errors */
   unsigned int         dma_igr;          /* DMA ingress erros */
   unsigned int         dma_sync;         /* DMA egress sync errors */
};

/* I2S configuration and parameters */
struct i2s_media
{
   unsigned int         num_chans;     /* Number of channels. 1 - mono and 2 - stereo   */
   unsigned int         frame_size;    /* Frame Size in bytes                           */

   struct dma_data_buf  buf[2];        /* Double buffer                                 */
   struct dma_cfg       dma;           /* DMA configuration                             */

   /* Debug facilities */
   int                  ramp;          /* Sysctl: Ramp generation                       */
   unsigned short       rampseed;      /* Ramp seed                                     */
   unsigned int         isrcount;      /* ISR counter                                   */
};

/* I2S information structure */
struct i2s_info
{
   int                  initialized;
   atomic_t             running;          /* Flag indicating channels are active                   */
   atomic_t             prepared;         /* Flag indicating channels are prepared to be enabled   */
   struct dma_data_buf  zero;             /* DMA scratch buffer used for priming */

   unsigned int         samp_freq;        /* Sampling frequency in Hz                  */
   unsigned int         frame_period;     /* Frame period in usec. Debug only          */
   unsigned int         egr_prime;        /* Egress priming in bytes                   */

   /* I2S media */
   struct i2s_media     igr;              /* Ingress Media information                 */
   struct i2s_media     egr;              /* Egress Media information                  */

   /* Write state */
   HALAUDIO_WRITE       write;            /* Write state                               */

   /* ISR status */
   atomic_t             queued_pkts_egr;  /* Num of egress packets awaiting to be DMA'd. Should not exceed 2. When 0, means DMA is IDLE */
   int                  active_idx;       /* Index to active buffer in ingress and egress double buffers */
   struct i2s_ch_errs   errs;             /* Channel errors                                              */

   /* Debug facilities */
   int                  loop_ig2eg;       /* Sysctl: Ingress to egress loopback           */
   int                  loop_eg2ig;       /* Sysctl: Egress to ingress loopback           */
   int                  ramp_check;       /* Sysctl: Validate ramp in hw eg2ig loopback   */
   int                  ramp_check_delay; /* Sysctl: Delay between igr and egr ramps      */
   int                  ramp_check_errs;  /* Sysctl: Number of glitches in received ramp  */
   HALAUDIO_SINECTL     sinectl;

   /* Mixer facilities */
   AMXR_PORT_ID         mixer_port;       /* Mixer port handle for channel                */

   /* CSX data */
   struct i2s_csx_data  csx_data[HALAUDIO_NUM_CSX_POINTS]; /* Array of CSX data structures */
};

struct i2s_sample_rate
{
   int                  samp_freq;        /* Sampling frequency in Hz */
   i2sHw_SAMPLERATE     i2s_rate;         /* i2s_rate enumeration */
};

/* Debug trace */
#define I2S_ENABLE_LOG           0
#if I2S_ENABLE_LOG
#include <linux/broadcom/knllog.h>   /* for debugging */
#define I2SLOG                   KNLLOG
#else
#define I2SLOG(c,d...)
#endif

/* ---- Private Variables ------------------------------------ */

static short i2sNo = 0;
module_param(i2sNo, short, 0); MODULE_PARM_DESC(verbose, "I2S Interface Select");

static short useAK4642 = 1;
module_param(useAK4642, short, 0); MODULE_PARM_DESC(verbose, "Enable external AK4642 Support");

static short nogpio = 0;
module_param(nogpio, short, 0); MODULE_PARM_DESC(verbose, "Disable GPIO allocation for debugging");

static struct i2s_info gI2s =
{
   .samp_freq     = I2S_DEFAULT_SAMP_FREQ,
   .frame_period  = I2S_DEFAULT_FRAME_PERIOD,
   .igr =
   {
      .num_chans  = 1,
      .frame_size = I2S_DEFAULT_FRAME_SIZE,
   },
   .egr =
   {
      .num_chans  = 2,
      .frame_size = 2*I2S_DEFAULT_FRAME_SIZE,
   },
   .egr_prime     = I2S_DEFAULT_PRIME_SIZE_BYTES,
};

static struct dma_pool *gDmaPool;      /* DMA memory pool */

/**
* Static sysctl data structures
*/
#define CTL_TABLE_INT(varStr,var) \
      procname: varStr,\
      data: var,\
      maxlen: sizeof(int),\
      mode: 0644,\
      proc_handler: &proc_dointvec,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_doEg2IgLoopback( ctl_table *table, int write,
      void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_doSineGen( ctl_table *table, int write,
      void __user *buffer, size_t *lenp, loff_t *ppos );
#else
static int proc_doEg2IgLoopback( ctl_table *table, int write, struct file *filp,
      void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_doSineGen( ctl_table *table, int write, struct file *filp,
      void __user *buffer, size_t *lenp, loff_t *ppos );
#endif

static struct ctl_table gSysCtlI2S[] =
{
   {
      .procname      = "loope2i",
      .data          = &gI2s.loop_eg2ig,
      .maxlen        = sizeof(int),
      .mode          = 0644,
      .proc_handler  = &proc_doEg2IgLoopback,
      .extra1        = &gI2s,
   },
   {
      .procname      = "sine_freq",
      .data          = &gI2s.sinectl.freq,
      .maxlen        = sizeof(int),
      .mode          = 0644,
      .proc_handler  = &proc_doSineGen,
      .extra1        = &gI2s,
   },
   { CTL_TABLE_INT( "loopi2e",     &gI2s.loop_ig2eg) },
   { CTL_TABLE_INT( "ramp_igr",    &gI2s.igr.ramp ) },
   { CTL_TABLE_INT( "ramp_egr",    &gI2s.egr.ramp ) },
   { CTL_TABLE_INT( "ramp_check",  &gI2s.ramp_check ) },
   {}
};

static const char *gSysCtlName[] =
{
   "i2s0",
   "i2s1"
};

static struct ctl_table gSysCtl[] =
{
    {
      .mode       = 0555,
      .child      = gSysCtlI2S,
    },
    {}
};
static struct ctl_table_header *gSysCtlHeader;

/* Installed callback. Called when all ingress processing has completed */
static HALAUDIO_IF_FRAME_ELAPSED gI2sElapsedCb;
static void                     *gI2sUserData;

/* HAL Audio interface handle */
static HALAUDIO_IF_HDL           gInterfHandle;

/* I2S sampling frequency rate map, sorted from low to high */
struct i2s_sample_rate gI2s_samp_freq[] =
{
   {
      .i2s_rate   = i2sHw_SAMPLERATE_8,
      .samp_freq  = 8000,
   },
   {
      .i2s_rate   = i2sHw_SAMPLERATE_16,
      .samp_freq  = 16000,
   },
   {
      .i2s_rate   = i2sHw_SAMPLERATE_2205,
      .samp_freq  = 22050,
   },
   {
      .i2s_rate   = i2sHw_SAMPLERATE_32,
      .samp_freq  = 32000,
   },
   {
      .i2s_rate   = i2sHw_SAMPLERATE_4410,
      .samp_freq  = 44100,
   },
   {
      .i2s_rate   = i2sHw_SAMPLERATE_48,
      .samp_freq  = 48000,
   },
};

/* I2S clocks */

static struct clk *gI2s0_clk;
static struct clk *gI2s1_clk;

/* APM clocks */
static struct clk *gApm100_clk;  /* Child clock */
static struct clk *gApm200_clk;  /* Parent clock */

/* ---- Private Function Prototypes -------------------------- */

static int  i2sInit( HALAUDIO_IF_FRAME_ELAPSED cb, void *data );
static int  i2sExit( void );
static int  i2sPrepare( void );
static int  i2sEnable( void );
static int  i2sDisable( void );
static int  i2sPmShutdown( void );
static int  i2sPmResume( void );
static int  i2sSetFreq( int chno, int freqHz );
static int  i2sGetFreq( int chno, int *freqHz );
static int  i2sCodecInfo( int chno, HALAUDIO_CODEC_INFO *codec_info );
static int  i2sCsxSet( int chno, HALAUDIO_CSX_POINT_ID point, const CSX_IO_POINT_FNCS *fncp, void *data );
static int  i2sWrite( int chno, int bytes, const char *audiobuf, HALAUDIO_CODEC_IORW_CB usercb, void *userdata );
static int  i2sAnaPowerDown( int powerdn );
static int  i2sAnaGainSet( int chno, int db, HALAUDIO_DIR dir, HALAUDIO_HWSEL hwsel );
static int  i2sAnaGainGet( int chno, HALAUDIO_GAIN_INFO *info, HALAUDIO_DIR dir, HALAUDIO_HWSEL hwsel );
static int  i2sDigGainSet( int chno, int db, HALAUDIO_DIR dir );
static int  i2sDigGainGet( int chno, HALAUDIO_GAIN_INFO *info, HALAUDIO_DIR dir );
static int  i2sSidetoneGainSet( int chno, int db );
static int  i2sSidetoneGainGet( int chno, HALAUDIO_GAIN_INFO *info );

static int  i2sDmaInit( void );
static int  i2sDmaTerm( void );
static void i2sDmaIngressHandler( DMA_Device_t dev, int reason, void *data );
static void i2sDmaEgressHandler( DMA_Device_t dev, int reason, void *data );
static void i2sDmaEgressDoTransfer( void );
static void i2sProcInit( void );
static void i2sProcTerm( void );
static int  i2sWriteTerm( void );

static int  i2sReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data );

static int  i2sMixerPortsRegister( void );
static int  i2sMixerPortsDeregister( void );

static i2sHw_SAMPLERATE getSampRate( int samp_freq );

/* HAL Audio I2S interface operations */
static HALAUDIO_IF_OPS halaudio_i2s_ops __initdata =
{
   .init                = i2sInit,
   .exit                = i2sExit,
   .enable              = i2sEnable,
   .disable             = i2sDisable,
   .prepare             = i2sPrepare,
   .ana_powerdn         = i2sAnaPowerDown,
   .pm_shutdown         = i2sPmShutdown,
   .pm_resume           = i2sPmResume,
   .codec_ops           =
   {
      .setfreq          = i2sSetFreq,
      .getfreq          = i2sGetFreq,
      .setana           = i2sAnaGainSet,
      .getana           = i2sAnaGainGet,
      .setdig           = i2sDigGainSet,
      .getdig           = i2sDigGainGet,
      .setsidetone      = i2sSidetoneGainSet,
      .getsidetone      = i2sSidetoneGainGet,
      /*.read             = i2sRead, */
      .write            = i2sWrite,
      .info             = i2sCodecInfo,
      .setcsx           = i2sCsxSet,
   },
};

/* ---- Functions -------------------------------------------- */

/***************************************************************************/
/**
*  I2S initialization
*
*  @return
*     0        I2S channel initialized successfully
*     -ENOMEM  Insufficient memory for DMA audio buffers
*     Other    Other errors
*/
static int i2sInit(
   HALAUDIO_IF_FRAME_ELAPSED  cb,   /*<< (i) Callback to call when one frame elapses */
   void                      *data  /*<< (i) User data to pass to callback */
)
{
   gpiomux_rc_e   gpiorc;
   int            err;

   atomic_set( &gI2s.running, 0 );
   atomic_set( &gI2s.prepared, 0 );

   /* Setup the APM clocks via clock framework */
   gApm200_clk = clk_get( NULL, "APM200" );
   if ( IS_ERR( gApm200_clk ))
   {
      printk( KERN_ERR "%s: Unable to find APM200 clock err=%li\n",
            __FUNCTION__, PTR_ERR( gApm200_clk ));
      return PTR_ERR( gApm200_clk );
   }

   gApm100_clk = clk_get( NULL, "APM100" );
   if ( IS_ERR( gApm100_clk ))
   {
      printk( KERN_ERR "%s: Unable to find APM100 clock err=%li\n",
            __FUNCTION__, PTR_ERR( gApm100_clk ));
      clk_put( gApm200_clk );
      return PTR_ERR( gApm100_clk );
   }

   err = clk_set_rate( gApm200_clk, 200000000 );
   if ( err )
   {
      goto exit_release_clocks;
   }

   err = clk_set_rate( gApm100_clk, 100000000 );
   if ( err )
   {
      goto exit_release_clocks;
   }

   /* Enable bus interfaces */
   chipcHw_audioChannelEnable( chipcHw_REG_AUDIO_CHANNEL_ENABLE_NTP_CLOCK | chipcHw_REG_AUDIO_CHANNEL_ENABLE_APM_CLOCK );
   chipcHw_audioChannelEnable( chipcHw_REG_AUDIO_CHANNEL_ENABLE_PCM0_CLOCK | chipcHw_REG_AUDIO_CHANNEL_ENABLE_PCM1_CLOCK );
   chipcHw_busInterfaceClockEnable( chipcHw_REG_BUS_CLOCK_I2S0 );
   chipcHw_busInterfaceClockEnable( chipcHw_REG_BUS_CLOCK_I2S1 );

   if ( i2sNo == 0 )
   {
      /* Initialize interface specific settings */
      gI2s.igr.dma.device     = DMA_DEVICE_I2S0_DEV_TO_MEM;
      gI2s.igr.dma.fifo_addr  = I2S_CHAN0_DMAFIFO_PHYS_ADDR;
      gI2s.egr.dma.device     = DMA_DEVICE_I2S0_MEM_TO_DEV;
      gI2s.egr.dma.fifo_addr  = I2S_CHAN0_DMAFIFO_PHYS_ADDR;

      if ( !nogpio )
      {
         /* Request GPIO MUX groups */
         gpiorc = gpiomux_requestGroup( gpiomux_group_i2s0, "I2S0" );
         if ( gpiorc != gpiomux_rc_SUCCESS )
         {
            printk( KERN_ERR "%s: failed to request I2S0 gpio group err=%u\n", __FUNCTION__, gpiorc );
            err = -EBUSY;
            goto exit_release_clocks;
         }
      }
   }
   else
   {
      /* Initialize interface specific settings */
      gI2s.igr.dma.device     = DMA_DEVICE_I2S1_DEV_TO_MEM;
      gI2s.igr.dma.fifo_addr  = I2S_CHAN1_DMAFIFO_PHYS_ADDR;
      gI2s.egr.dma.device     = DMA_DEVICE_I2S1_MEM_TO_DEV;
      gI2s.egr.dma.fifo_addr  = I2S_CHAN1_DMAFIFO_PHYS_ADDR;

      if ( !nogpio )
      {
         /* Request GPIO MUX groups */
         gpiorc = gpiomux_requestGroup( gpiomux_group_i2s1, "I2S1" );
         if ( gpiorc != gpiomux_rc_SUCCESS )
         {
            printk( KERN_ERR "%s: failed to request I2S1 gpio group err=%u\n", __FUNCTION__, gpiorc );
            err = -EBUSY;
            goto exit_release_clocks;
         }
      }
   }

   /* Initialize I2S hardware block */
   err = I2sHw_Init( i2sNo );
   if ( err )
   {
      printk( KERN_ERR "I2S [Interf=%u]: Failed to initialize I2S hardware err=%i\n", i2sNo, err );
      goto exit_release_clocks;
   }

   /* Setup the I2S clocks via clock framework */
   gI2s0_clk = clk_get( NULL, "I2S0" );
   if ( IS_ERR( gI2s0_clk ))
   {
      printk( KERN_ERR "%s: Unable to find I2S0 clock err=%li\n",
            __FUNCTION__, PTR_ERR( gI2s0_clk ));
      err = PTR_ERR( gI2s0_clk );
      goto exit_release_clocks;
   }

   gI2s1_clk = clk_get( NULL, "I2S1" );
   if ( IS_ERR( gI2s1_clk ))
   {
      printk( KERN_ERR "%s: Unable to find I2S1 clock err=%li\n",
            __FUNCTION__, PTR_ERR( gI2s1_clk ));
      clk_put( gI2s0_clk );
      err = PTR_ERR( gI2s1_clk );
      goto exit_release_clocks;
   }

   if ( i2sNo == 0 )
   {
      /* configure chipc to use PCM clock for i2s */
      /* set chipc to use PCM output */
      clk_set_rate( gI2s0_clk, 25000000 );
      /* Enable clock */
      err = clk_enable( gI2s0_clk );
      if ( err )
      {
         printk( KERN_ERR "I2S: Failed to enable I2S0 clock\n" );
         goto exit_release_clocks_i2s;
      }
   }
   else
   {
      /* set chipc to use PCM output */
      clk_set_rate( gI2s1_clk, 25000000 );
      /* Enable clock */
      err = clk_enable( gI2s1_clk );
      if ( err )
      {
         printk( KERN_ERR "I2S: Failed to enable I2S1 clock\n" );
         goto exit_release_clocks_i2s;
      }
   }

   err = i2sDmaInit();
   if ( err != 0 )
   {
      printk( KERN_ERR "I2S [Interf=%u]: failed to initialize DMA resources err=%i\n", i2sNo, err );
      goto exit_disable_clocks;
   }

   err = i2sMixerPortsRegister();
   if ( err != 0 )
   {
      printk( KERN_ERR "I2S [Interf=%u]: failed to register mixer ports err=%i\n", i2sNo, err );
      goto exit_full_cleanup;
   }

   if ( useAK4642 )
   {
      /* Initialize AK4642 */
      err = halAudioExternalAK4642_init();
      if ( err != 0 )
      {
         printk( KERN_ERR "I2S [Interf=%u]: failed to initialize AK4642 err=%i\n", i2sNo, err );
         goto exit_full_cleanup;
      }
   }

   i2sProcInit();

   /* Save HAL Audio specifics */
   gI2sElapsedCb = cb;
   gI2sUserData  = data;

   gI2s.initialized = 1;

   return 0;

exit_full_cleanup:   /* Cleanup DMA and I2S hardware */
   i2sExit();
   return err;

exit_disable_clocks:
   if ( i2sNo == 0 )
   {
      clk_disable( gI2s0_clk );
   }
   else
   {
      clk_disable( gI2s1_clk );
   }

exit_release_clocks_i2s:
   clk_put( gI2s0_clk );
   clk_put( gI2s1_clk );

exit_release_clocks:
   clk_put( gApm100_clk );
   clk_put( gApm200_clk );

   return err;
}

/***************************************************************************/
/**
*  Cleanup I2S resources
*
*  @return  Nothing.
*
*  @remark
*/
static int i2sExit( void )
{
   int          rc, error = 0;
   gpiomux_rc_e gpiorc;

   i2sProcTerm();

   rc = i2sMixerPortsDeregister();
   if ( rc )
   {
      printk( KERN_ERR "I2S [Interf=%u]: failed to cleanup mixer ports\n", i2sNo );
      error = rc;
   }

   rc = i2sDisable();
   if ( rc != 0 )
   {
      printk( KERN_ERR "I2S [Interf=%u]: failed to disable I2S hardware\n", i2sNo );
      error = rc;
   }

   rc = i2sDmaTerm();
   if ( rc != 0 )
   {
      printk( KERN_ERR "I2S [Interf=%u]: failed to cleanup DMA resources\n", i2sNo );
      error = rc;
   }

   i2sWriteTerm();

   rc = I2sHw_Exit( i2sNo );
   if ( rc != 0 )
   {
      printk( KERN_ERR "I2S [Interf=%u]: failed to cleanup I2S hardware\n", i2sNo );
      error = rc;
   }

   if ( useAK4642 )
   {
      rc = halAudioExternalAK4642_exit();
      if  ( rc != 0 )
      {
         printk( KERN_ERR "I2S [Interf=%u]: failed to exit AK4642\n", i2sNo );
         error = rc;
      }
   }

   if ( !nogpio )
   {
      if ( i2sNo == 0 )
      {
         /* Free GPIO MUX groups */
         gpiorc = gpiomux_freeGroup( gpiomux_group_i2s0 );
         if ( gpiorc != gpiomux_rc_SUCCESS )
         {
            printk( KERN_ERR "I2S: failed to free I2S0 GPIO MUX group rc=%i\n", gpiorc );
            error = gpiorc;
         }
      }
      else
      {
         /* Free GPIO MUX groups */
         gpiorc = gpiomux_freeGroup( gpiomux_group_i2s1 );
         if ( gpiorc != gpiomux_rc_SUCCESS )
         {
            printk( KERN_ERR "I2S: failed to free I2S1 GPIO MUX group rc=%i\n", gpiorc );
            error = gpiorc;
         }
      }
   }

   gI2s.initialized = 0;

   if ( i2sNo == 0 )
   {
      clk_disable( gI2s0_clk );
   }
   else
   {
      clk_disable( gI2s1_clk );
   }

   clk_put( gI2s0_clk );
   clk_put( gI2s1_clk );
   clk_put( gApm100_clk );
   clk_put( gApm200_clk );

   return error;
}

/***************************************************************************/
/**
*  I2S retrieve codec info
*
*  @return
*     0        I2S codec infor retrieved succesfully
*     -ve      Error code
*/
static int i2sCodecInfo( int chno, HALAUDIO_CODEC_INFO *codec_info )
{
   int i;

   (void) chno;

   memset( codec_info, 0, sizeof(*codec_info) );

   for ( i = 0; i < sizeof(gI2s_samp_freq)/sizeof(gI2s_samp_freq[0]) && i < HALAUDIO_MAX_NUM_FREQ_SETTINGS; i++ )
   {
      codec_info->freqs.freq[i] = gI2s_samp_freq[i].samp_freq;
      codec_info->freqs.num++;
   }

   codec_info->sample_width = I2S_SAMP_WIDTH;        /* sample size width in bytes */
   codec_info->read_format  = HALAUDIO_FMT_S16_LE;   /* Read format */
   codec_info->write_format = HALAUDIO_FMT_S16_LE;   /* Write format */

   /* Following are AK4642 specifics */
   codec_info->channels_tx  = 1;                     /* # of channels for tx (mic), 1=mono, 2=stereo, etc. */
   codec_info->channels_rx  = 2;                     /* # of channels for rx (spk), 1=mono, 2=stereo, etc. */
   codec_info->mics         = 2;                     /* # of mic paths. 0 means no ADC */
   codec_info->spkrs        = 2;                     /* # of speaker paths. 0 means no DAC */
   codec_info->bulk_delay   = -1;                    /* Echo bulk delay in samples, -1 = not calibrated */

#if 0
   codec_info->equlen_tx  = 0;                     /* max # of tx equalizer coeffs, 0 = equ unsupported */
   codec_info->equlen_rx  = 0;                     /* max # of rx equalizer coeffs, 0 = equ unsupported */
   codec_info->parent_id  = 0;                     /* Parent interface id */
#endif

   sprintf( codec_info->name, "I2S CH%i", i2sNo );

   return 0;
}

/***************************************************************************/
/**
*  Set up CSX points for capture and injection. Refer to CSX documentation
*  for more informatin.
*
*  @return
*     0        Success
*     -ve      Error code
*/
static int i2sCsxSet(
   int                        chno, /*<< (i) Codec channel number */
   HALAUDIO_CSX_POINT_ID      point,/*<< (i) Point ID to install the CSX point */
   const CSX_IO_POINT_FNCS   *fncp, /*<< (i) Ptr to CSX callbacks */
   void                      *data  /*<< (i) User data to pass back to callbacks */
)
{
   struct i2s_info  *infop;
   unsigned long     flags;
   int err = 0;

   if ( chno >= 1 )
   {
      return -EINVAL;
   }

   infop = &gI2s;

   local_irq_save( flags );
   switch( point )
   {
      case HALAUDIO_CSX_POINT_ADC:
      {
         memcpy( &infop->csx_data[HALAUDIO_CSX_POINT_ADC].csx_ops,
                 fncp,
                 sizeof(infop->csx_data[HALAUDIO_CSX_POINT_ADC].csx_ops) );

         infop->csx_data[HALAUDIO_CSX_POINT_ADC].priv = data;
         break;
      }
      case HALAUDIO_CSX_POINT_DAC:
      {
         memcpy( &infop->csx_data[HALAUDIO_CSX_POINT_DAC].csx_ops,
                 fncp,
                 sizeof(infop->csx_data[HALAUDIO_CSX_POINT_DAC].csx_ops) );

         infop->csx_data[HALAUDIO_CSX_POINT_DAC].priv = data;
         break;
      }
      default:
      {
         err = -EINVAL;
      }
   }
   local_irq_restore( flags );

   return err;
}

/***************************************************************************/
/**
*  Prepare I2S channels to start: allocate DMA channels, configure channel
*  parameters, prime egress DMA, start ingress DMA.
*
*  @remarks       This routine may block.
*
*  @return
*     0           - success
*     -ENOMEM     - insufficient memory
*     -EBUSY      - cannot allocate DMA channels
*     -ve         - other error codes
*/
static int i2sPrepare( void )
{
   int                  rc;
   i2sHw_Config         i2sconfig;

   if ( atomic_read( &gI2s.running ) || atomic_read( &gI2s.prepared ))
   {
      return -EBUSY; /* Already running or prepared */
   }

   /* Index is pre-incremented before use. Set to second buffer to start */
   gI2s.active_idx    = 1;
   gI2s.igr.isrcount  = 0;
   gI2s.egr.isrcount  = 0;
   memset( &gI2s.errs, 0, sizeof(gI2s.errs) );

   /* Install DMA handlers */
   dma_set_device_handler( gI2s.egr.dma.device, i2sDmaEgressHandler, NULL /* userData */ );
   dma_set_device_handler( gI2s.igr.dma.device, i2sDmaIngressHandler, NULL /* userData */ );

   /* FIXME: Soft reset the I2S channel to ensure FIFO entirely flushed.
    *        As a consequence, hardware parameters setup previously may have to
    *        be setup again.
    */
   I2sHw_SoftReset( i2sNo );

   i2sconfig.rxDMASize        = i2sHw_DMASIZE_16;
   i2sconfig.txDMASize        = i2sHw_DMASIZE_32;
   i2sconfig.interfaceMode    = i2sHW_MODE_TDM_MASTER;
   i2sconfig.txAudioMode      = i2sHw_AUDIO_MODE_STEREO;
   i2sconfig.rxAudioMode      = i2sHw_AUDIO_MODE_MONO;
   i2sconfig.extOrStd         = i2sHw_PROTOCOL_STANDARD;
   i2sconfig.recordMode       = i2sHw_RECORDING_MODE_SINGLE;
   i2sconfig.recordCh         = i2sHw_RECORDING_CHANNEL_LEFT;
   i2sconfig.frameSigMode     = i2sHw_FRAME_SIGNAL_MODE_TXONLY;
   i2sconfig.txAudioChMode    = i2sHw_AUDIO_TX_START_LEFT;

   /* I2S channel interface configuration */
   rc = I2sHw_SetConfig( i2sNo, &i2sconfig );
   if ( rc )
   {
      printk( KERN_ERR "I2S [Interf=%u]: Failed to set interface configuration rc=%i\n", i2sNo, rc );
      return rc;
   }

   /* I2S channel sample rate configuration */
   rc = I2sHw_SetSamplingRate( i2sNo, getSampRate( gI2s.samp_freq ), i2sHw_DIR_TXRX );
   if ( rc )
   {
      printk( KERN_ERR "I2S [Interf=%u]: Failed to set sample rate rc=%i\n", i2sNo, rc );
      return rc;
   }

   rc = I2sHw_FlushFifo( i2sNo, i2sHw_DIR_TXRX );
   if ( rc )
   {
      printk( KERN_ERR "I2S [Interf=%u]: Failed to flush FIFO rc=%i\n", i2sNo, rc );
      return rc;
   }

   rc = I2sHw_EnableFifo( i2sNo, 1, i2sHw_DIR_TXRX );
   if ( rc )
   {
      printk( KERN_ERR "I2S [Interf=%u]: Failed to enable FIFO rc=%i\n", i2sNo, rc );
      return rc;
   }

   rc = I2sHw_EnableDma( i2sNo, 1, i2sHw_DIR_TXRX );
   if ( rc )
   {
      printk( KERN_ERR "I2S [Interf=%u]: Failed to enable I2S DMA interface rc=%i\n", i2sNo, rc );
      return rc;
   }

   /* Request egress DMA channel */
   gI2s.egr.dma.handle = dma_alloc_channel( gI2s.egr.dma.device );
   if ( gI2s.egr.dma.handle < 0 )
   {
      printk( KERN_ERR "I2S [Interf=%u]: failed to get MTP DMA channel\n", i2sNo );
      rc = -EBUSY;
      goto cleanup_dma_channels;
   }

   /* Clear egress samples */
   memset( gI2s.egr.buf[0].virt, 0, gI2s.egr.frame_size );
   memset( gI2s.egr.buf[1].virt, 0, gI2s.egr.frame_size );

   /* Prime egress */
   atomic_set( &gI2s.queued_pkts_egr, 1 );
   rc = dma_transfer_to_device( gI2s.egr.dma.handle,
         gI2s.zero.phys, gI2s.egr.dma.fifo_addr, gI2s.egr.frame_size + gI2s.egr_prime );
   if ( rc != 0 )
   {
      printk( KERN_ERR "I2S [Interf=%u]: egress prime DMA failed\n", i2sNo );
      goto cleanup_dma_channels;
   }

   /* Request ingress DMA channel */
   gI2s.igr.dma.handle = dma_alloc_channel( gI2s.igr.dma.device );
   if ( gI2s.igr.dma.handle < 0 )
   {
      printk( KERN_ERR "I2S [Interf=%u]: failed to get PTM DMA channel\n", i2sNo );
      rc = -EBUSY;
      goto cleanup_dma_channels;
   }

   /* Allocate ingress double buffer DMA descriptors */
   rc = dma_alloc_double_dst_descriptors( gI2s.igr.dma.handle,
         gI2s.igr.dma.fifo_addr, gI2s.igr.buf[0].phys, gI2s.igr.buf[1].phys,
         gI2s.igr.frame_size );
   if ( rc != 2 )
   {
      printk( KERN_ERR "I2S [Interf=%u]: %d ingress descriptors are allocated, instead of 2.\n",
            i2sNo, rc );
      goto cleanup_dma_channels;
   }

   /* Start ingress DMA channel. DMA does not actually start until I2S channel is enabled. */
   rc = dma_start_transfer( gI2s.igr.dma.handle, 2 * gI2s.igr.frame_size );
   if ( rc != 0 )
   {
      printk( KERN_ERR "I2S [Interf=%u]: Failed to start ingress DMA\n", i2sNo );
      goto cleanup_dma_channels;
   }

   /* FIXME: setup addition hardware parameters which were reset by soft reset */
   I2sHw_EnableLoopback( i2sNo, gI2s.loop_eg2ig );

func_exit:

   if ( useAK4642 )
   {
      /* Take AK4642 out of reset and restore registers */
      halAudioExternalAK4642_CodecReset( 0 );
      halAudioExternalAK4642_PowerEnable( 1 );
   }

   if ( rc == 0 )
   {
      atomic_set( &gI2s.prepared, 1 );
   }

   return rc;

cleanup_dma_channels:

   /* Free DMA channels that may have been allocated */
   dma_set_device_handler( gI2s.egr.dma.device, NULL, NULL );

   if ( gI2s.egr.dma.handle >= 0 )
   {
      dma_free_channel( gI2s.egr.dma.handle );
      gI2s.egr.dma.handle = DMA_INVALID_HANDLE;
   }

   dma_set_device_handler( gI2s.igr.dma.device, NULL, NULL );

   if ( gI2s.igr.dma.handle >= 0 )
   {
      dma_free_channel( gI2s.igr.dma.handle );
      gI2s.igr.dma.handle = DMA_INVALID_HANDLE;
   }

   goto func_exit;
}

/***************************************************************************/
/**
*  Enable I2S interface
*
*  @return
*     0     success
*     -1    failed to enable I2S channels
*
*  @remark
*/
static int i2sEnable( void )
{
   int rc;

   if ( !atomic_read( &gI2s.prepared ))
   {
      return -EPERM;
   }

   if ( atomic_read( &gI2s.running ))
   {
      return -EBUSY; /* Already enabled */
   }

   rc = I2sHw_EnableInterface( i2sNo, 1, i2sHw_DIR_TXRX );
   if ( rc == 0 )
   {
      atomic_set( &gI2s.running, 1 );
   }

   return rc;
}

/***************************************************************************/
/**
*  Disable I2S interface
*
*  @return
*     0     success
*     -1    failed to enable I2S channels
*
*  @remark
*/
static int i2sDisable()
{
   int rc = 0;
   int err = 0;

   if ( !atomic_read( &gI2s.prepared ))
   {
      return 0; /* Nothing to disable, has not been prepared */
   }

   atomic_set( &gI2s.running, 0 );
   atomic_set( &gI2s.queued_pkts_egr, 0 );

   /* Stop egress DMA, I2S block, and then ingress DMA */
   rc = dma_stop_transfer( gI2s.egr.dma.handle );
   if ( rc != 0 )
   {
      printk( KERN_ERR "I2S [Interf=%u]: failed to stop egress DMA\n", i2sNo );
      err = rc;
   }
   dma_set_device_handler( gI2s.egr.dma.device, NULL, NULL );

   I2sHw_EnableInterface( i2sNo, 0, i2sHw_DIR_TXRX );

   rc = dma_stop_transfer( gI2s.igr.dma.handle );
   if ( rc != 0 )
   {
      printk( KERN_ERR "I2S [Interf=%u]: failed to stop egress DMA\n", i2sNo );
      err = rc;
   }
   dma_set_device_handler( gI2s.igr.dma.device, NULL, NULL );

   dma_free_channel( gI2s.egr.dma.handle );
   gI2s.egr.dma.handle = DMA_INVALID_HANDLE;
   dma_free_channel( gI2s.igr.dma.handle );
   gI2s.igr.dma.handle = DMA_INVALID_HANDLE;

   if ( useAK4642 )
   {
      /* Power down AK4642 and put it in reset */
      halAudioExternalAK4642_PowerEnable( 0 );
      halAudioExternalAK4642_CodecReset( 1 );
   }

   /* Flush write buffers */
   halAudioWriteFlush( &gI2s.write );

   atomic_set( &gI2s.prepared, 0 );

   return err;
}

/***************************************************************************/
/**
*  Select sampling frequency for all channels
*
*  @return
*     0        Success
*     -ENODEV  I2S has not be initiialized
*     -EBUSY   I2S is currently running
*     -EINVAL  Invalid channel number
*
*  @remarks
*     This routine needs the I2S block disabled before it can proceed
*     because it must reset the DMA chains and configure the I2S registers.
*/
static int i2sSetFreq(
   int chno,                        /**< (i) channel number 0-2 */
   int freqHz                       /**< (i) sampling frequency in Hz */
)
{
   int i, validhz, rc = 0;
   unsigned int frame_size, frame_period, egr_prime;

   (void) chno;

   if ( !gI2s.initialized )
   {
      return -ENODEV;
   }

   if ( atomic_read( &gI2s.running ) || atomic_read( &gI2s.prepared ))
   {
      /* Cannot change sampling frequency if currently running or already prepared */
      return -EBUSY;
   }

   /* Validate sampling frequency is supported */
   validhz = 0;
   for ( i = 0; i < sizeof(gI2s_samp_freq)/sizeof(gI2s_samp_freq[0]); i++ )
   {
      if ( freqHz == gI2s_samp_freq[i].samp_freq )
      {
         validhz = 1;
         break;
      }
   }
   if ( !validhz )
   {
      return -EINVAL;
   }

   /* Calculate frame size and period based on selected sampling frequency.
    * The frame period is recalculated to account for rounding errors with
    * 22.05 kHz and 44.1 kHz sampling rates.
    */
   frame_size   = (freqHz * I2S_DEFAULT_FRAME_PERIOD * I2S_SAMP_WIDTH) / 1000000; /* in bytes */
   frame_period = (frame_size * (1000000/I2S_SAMP_WIDTH)) / freqHz;               /* in usec */

   /* Calculate priming. If priming size is larger than fifo size, then the driver
    * must adjust the amount of priming to allow egress dma interrupts to
    * lag ingress dma ingress by a minimum period. This is required to guarantee
    * that all ingress and egress signal processing are complete before samples
    * are sent to the hardware.
    */
   egr_prime    = (gI2s.egr.num_chans * freqHz * I2S_DEFAULT_PRIME_PERIOD * I2S_SAMP_WIDTH) / 1000000;

   gI2s.samp_freq      = freqHz;
   gI2s.frame_period   = frame_period;
   gI2s.egr_prime      = egr_prime;
   gI2s.igr.frame_size = gI2s.igr.num_chans * frame_size;
   gI2s.egr.frame_size = gI2s.egr.num_chans * frame_size;

   if ( useAK4642 )
   {
      rc = halAudioExternalAK4642_SetFrequency( freqHz );
      if ( rc )
      {
         printk( KERN_ERR "I2S [Interf=%u]: Failed to set sampling frequency for AK4642 rc=%i\n", i2sNo, rc );
      }
   }

   rc = amxrSetPortFreq( gI2s.mixer_port, freqHz, frame_size, freqHz, frame_size );
   if ( rc )
   {
      printk( KERN_ERR "I2S [Interf=%u]: Failed to set sampling frequency in audio mixer rc=%i\n", i2sNo, rc );
   }

   halAudioSineConfig( &gI2s.sinectl, gI2s.sinectl.freq, freqHz );

   return rc;
}

/***************************************************************************/
/**
*  Retrieve sampling frequency information for a codec channel.
*
*  @return
*     0        Success
*     -ve      Error code
*/
static int i2sGetFreq(
   int   chno,                      /*<< (i) Codec channel number */
   int  *freqhz                     /*<< (o) Ptr to sampling frequency in Hz */
)
{
   (void) chno;

   *freqhz = gI2s.samp_freq;

   return 0;
}

/***************************************************************************/
/**
*  Power down analog hardware for codec channel
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remarks
*     This method can choose to operate on one or all of the channels
*     in the interface at a time. To operate on all of the channels,
*     use the HALAuDIO_IF_ALL_CODECS.
*/
static int i2sAnaPowerDown(
   int powerdn                      /*<< (i) 1 to power down, 0 to power up */
)
{
   if ( useAK4642 )
   {
      if ( powerdn )
      {
         /* Power down AK4642 and put it in reset */
         halAudioExternalAK4642_PowerEnable( 0 );
         halAudioExternalAK4642_CodecReset( 1 );
      }
      else
      {
         /* Take AK4642 out of reset and restore registers */
         halAudioExternalAK4642_CodecReset( 0 );
         halAudioExternalAK4642_PowerEnable( 1 );
      }
   }

   return 0;
}

/***************************************************************************/
/**
*  Disable clocks to enable lower-power state.  Shutdown is only permitted
*  if the interface is already disabled.
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remarks
*/
static int i2sPmShutdown( void )
{
   if ( atomic_read( &gI2s.running ) )
   {
      return -EBUSY;
   }

   if ( i2sNo == 0 )
   {
      clk_disable( gI2s0_clk );
   }
   else
   {
      clk_disable( gI2s1_clk );
   }

   return 0;
}

/***************************************************************************/
/**
*  Re-enable clocks to resume after shutdown.
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remarks
*/
static int i2sPmResume( void )
{
   if ( i2sNo == 0 )
   {
      return clk_enable( gI2s0_clk );
   }
   else
   {
      return clk_enable( gI2s1_clk );
   }
}

/***************************************************************************/
/**
*  Set codec analog gain
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remarks
*/
static int i2sAnaGainSet(
   int             chno,      /**< (i)    I2S channel index */
   int             db,        /**< (i)    dB value to set gain */
   HALAUDIO_DIR    dir,       /**< (i)    Audio direction */
   HALAUDIO_HWSEL  hwsel      /**< (i)    HW mux selection */
)
{
   int rc = 0;

   (void) chno;  /* Unused */

   if ( useAK4642 )
   {
      rc = halAudioExternalAK4642_GainSetAnalogHardware( db, dir, hwsel );
   }

   return rc;
}

/***************************************************************************/
/**
*  Get codec analog gain
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remarks
*/
static int i2sAnaGainGet(
   int                  chno,       /*<< (i) codec channel number */
   HALAUDIO_GAIN_INFO  *info,       /*<< (o) Ptr to gain info structure */
   HALAUDIO_DIR         dir,        /*<< (i) Direction path */
   HALAUDIO_HWSEL       hwsel       /*<< (i) Hardware mux selection */
)
{
   int rc = 0;

   (void) chno;  /* Unused */

   if ( useAK4642 )
   {
      rc = halAudioExternalAK4642_GainGetAnalogHardware( info, dir, hwsel );
   }

   return rc;
}

/***************************************************************************/
/**
*  Set codec digital gain
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remarks
*/
static int i2sDigGainSet(
   int             chno,      /**< (i)    I2S channel index */
   int             db,        /**< (i)    dB value to set gain */
   HALAUDIO_DIR    dir        /**< (i)    Audio direction */
)
{
   int rc = 0;

   (void)chno;  /* Unused */

   if ( useAK4642 )
   {
      rc = halAudioExternalAK4642_GainSetDigitalHardware( db, dir );
   }

   return rc;
}

/***************************************************************************/
/**
*  Get codec digital gain
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remarks
*/
static int i2sDigGainGet(
   int                  chno,       /**< (i) I2S channel index */
   HALAUDIO_GAIN_INFO  *info,       /*<< (o) Ptr to gain info structure */
   HALAUDIO_DIR         dir         /*<< (i) Direction path */
)
{
   int rc = 0;

   (void)chno;  /* Unused */

   if ( useAK4642 )
   {
      rc = halAudioExternalAK4642_GainGetDigitalHardware( info, dir );
   }

   return rc;
}

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
static int i2sSidetoneGainSet(
   int      chno,                   /**< (i)    Channel index */
   int      db                      /**< (i)    Gain in db */
)
{
   int rc = 0;

   (void) chno;   /* Unused */

   if ( useAK4642 )
   {
      rc = halAudioExternalAK4642_SidetoneSet( db );
   }

   return rc;
}

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
static int i2sSidetoneGainGet(
   int                  chno,       /*<< (i) codec channel number */
   HALAUDIO_GAIN_INFO  *info        /*<< (o) Ptr to gain info structure */
)
{
   int rc = 0;

   (void) chno;   /* Unused */

   if ( useAK4642 )
   {
      rc = halAudioExternalAK4642_SidetoneGet( info );
   }

   return rc;
}

/***************************************************************************/
/**
*  Setup DMA memory resources
*
*  @return
*     0        - success
*     -ENOMEM  - failed to allocate memory
*/
static int i2sDmaInit( void )
{
   int rc = 0;

   /* Create DMA buffer pool */
   gDmaPool = dma_pool_create( "I2S DMA memory pool", NULL,
         I2S_MAX_DMA_BUFFER_SIZE_BYTES, I2S_DMA_ALIGN_IN_BYTES,
         I2S_DMA_ALLOC_CROSS_RESTRICT );
   if ( gDmaPool == NULL )
   {
      printk( KERN_ERR "I2S: failed to allocate DMA buffer pool\n" );
      return -ENOMEM;
   }

   gI2s.igr.buf[0].virt = dma_pool_alloc( gDmaPool, GFP_KERNEL, &gI2s.igr.buf[0].phys );
   gI2s.igr.buf[1].virt = dma_pool_alloc( gDmaPool, GFP_KERNEL, &gI2s.igr.buf[1].phys );
   gI2s.egr.buf[0].virt = dma_pool_alloc( gDmaPool, GFP_KERNEL, &gI2s.egr.buf[0].phys );
   gI2s.egr.buf[1].virt = dma_pool_alloc( gDmaPool, GFP_KERNEL, &gI2s.egr.buf[1].phys );

   if ( !gI2s.igr.buf[0].virt || !gI2s.igr.buf[1].virt || !gI2s.egr.buf[0].virt || !gI2s.egr.buf[1].virt )
   {
      printk( KERN_ERR "I2S [Interf=%u]: failed to allocate DMA audio buffers\n", i2sNo );
      rc = -ENOMEM;
      goto cleanup_exit;
   }

   /* Allocate buffer of 0's for egress priming. */
   gI2s.zero.virt = dma_alloc_writecombine( NULL, I2S_MAX_PRIME_SIZE_BYTES,
            &gI2s.zero.phys, GFP_KERNEL );
   if ( gI2s.zero.virt == NULL )
   {
      rc = -ENOMEM;
      goto cleanup_exit;
   }
   memset( gI2s.zero.virt, 0, I2S_MAX_PRIME_SIZE_BYTES );

   return 0;

cleanup_exit:
   i2sDmaTerm();
   return rc;
}

/***************************************************************************/
/**
*  Cleanup DMA memory resources
*
*  @return
*/
static int i2sDmaTerm( void )
{
   int rc = 0;

   if ( gI2s.igr.buf[0].virt )
   {
      dma_pool_free( gDmaPool, gI2s.igr.buf[0].virt, gI2s.igr.buf[0].phys );
      gI2s.igr.buf[0].virt = NULL;
   }
   if ( gI2s.igr.buf[1].virt )
   {
      dma_pool_free( gDmaPool, gI2s.igr.buf[1].virt, gI2s.igr.buf[1].phys );
      gI2s.igr.buf[1].virt = NULL;
   }
   if ( gI2s.egr.buf[0].virt )
   {
      dma_pool_free( gDmaPool, gI2s.egr.buf[0].virt, gI2s.egr.buf[0].phys );
      gI2s.egr.buf[0].virt  = NULL;
   }
   if ( gI2s.egr.buf[1].virt )
   {
      dma_pool_free( gDmaPool, gI2s.egr.buf[1].virt, gI2s.egr.buf[1].phys );
      gI2s.egr.buf[1].virt  = NULL;
   }
   dma_pool_destroy( gDmaPool );

   if ( gI2s.zero.virt )
   {
      dma_free_writecombine( NULL, I2S_MAX_PRIME_SIZE_BYTES,
            gI2s.zero.virt, gI2s.zero.phys );
      gI2s.zero.virt = NULL;
   }

   return rc;
}

/***************************************************************************/
/**
*  I2S ingress DMA handler that services all ingress DMA channels. This
*  handler is controlled by the master ingress channel.
*
*  @return  none
*
*  @remarks
*/
static void i2sDmaIngressHandler(
   DMA_Device_t   dev,           /**< (i) Device that triggered callback */
   int            reason,        /**< (i) Reason for interrupt */
   void          *data           /**< (i) User data pointer */
)
{
   struct i2s_media *pM = &gI2s.igr;
   struct i2s_info  *pInfo = &gI2s;
   unsigned short   *ingressp;
   int               frame_size;

   (void) dev;    /* unused */

   I2SLOG( "%u: dev=%u reason=0x%x", pM->isrcount, (unsigned)dev, (unsigned)reason );

   if ( reason != DMA_HANDLER_REASON_BLOCK_COMPLETE )
   {
      gI2s.errs.dma_igr++;
      return;
   }

   /* Process ingress */
   pM->isrcount++;

   /* Point to buffer index with actual samples */
   gI2s.active_idx = (gI2s.active_idx + 1) & 1;
   ingressp        = pM->buf[gI2s.active_idx].virt;
   frame_size      = pM->frame_size;

   if ( gI2s.ramp_check )
   {
      /* Verify against data from 2 frames ago */
      halAudioCompareData( ingressp, gI2s.egr.buf[gI2s.active_idx].virt, frame_size/sizeof(int16_t), &gI2s.ramp_check_errs, &gI2s.ramp_check_delay );
   }
   if ( pM->ramp )
   {
      halAudioGenerateRamp( ingressp, &pM->rampseed, frame_size/sizeof(int16_t), 1 /* mono */ );
   }

   if ( pInfo->csx_data[HALAUDIO_CSX_POINT_ADC].csx_ops.csxCallback )
   {
      pInfo->csx_data[HALAUDIO_CSX_POINT_ADC].csx_ops.csxCallback( (char *)ingressp, frame_size, pInfo->csx_data[HALAUDIO_CSX_POINT_ADC].priv );
   }

   /* Perform callback action. All ingress processing have completed. */
   if ( gI2sElapsedCb )
   {
      (*gI2sElapsedCb)( gI2sUserData );
   }

   I2SLOG( "end" );
}

/***************************************************************************/
/**
*  I2S egress DMA interrupt handler
*
*  @return  none
*/
static void i2sDmaEgressHandler(
   DMA_Device_t   dev,           /**< (i) Device that triggered callback */
   int            reason,        /**< (i) Reason for interrupt */
   void          *data           /**< (i) User data pointer */
)
{
   struct i2s_media *pM = &gI2s.egr;

   (void) dev;    /* unused */

   I2SLOG( "dev=%u reason=%i isr=%u pkts=%i", dev, reason, pM->isrcount, atomic_read( &gI2s.queued_pkts_egr ));
   pM->isrcount++;

   if ( reason != DMA_HANDLER_REASON_TRANSFER_COMPLETE )
   {
      printk( KERN_ERR "I2S [Interf=%u]: egress dma error reason=%i\n", i2sNo, reason );
      gI2s.errs.dma_egr++;
      return;
   }

   atomic_dec( &gI2s.queued_pkts_egr ); /* DMA completed */

   if ( atomic_read( &gI2s.queued_pkts_egr ) > 0 )
   {
      /* More egress packets awaiting to be DMA'd. Start another transfer */
      i2sDmaEgressDoTransfer();
   }
   I2SLOG( "end dev=%u", dev );
}

/***************************************************************************/
/**
*  Helper routine to do the egress DMA transfer for an APM channel
*
*  @return  None
*
*  @remarks
*     It is expected that egress DMA transfers only occur after ingress
*     DMA transfers have completed.
*
*/
static void i2sDmaEgressDoTransfer( void )
{
   struct i2s_media *pM = &gI2s.egr;
   struct i2s_info  *pInfo = &gI2s;
   void             *egressp;
   int               rc;
   int               frame_size;

   /* Check that egress ISR is in sync with ingress ISR */
   if ( pM->isrcount != gI2s.igr.isrcount )
   {
      I2SLOG( "I2S [Interf=%i] egress sync err: egr=%u igr=%u",i2sNo, pM->isrcount, gI2s.igr.isrcount );
      gI2s.errs.dma_sync++;
   }

   egressp     = pM->buf[gI2s.active_idx].virt;
   frame_size  = pM->frame_size;

   /* Write request */
   halAudioWriteService( &gI2s.write, egressp, frame_size );

   if ( pM->ramp )
   {
      halAudioGenerateRamp( egressp, &pM->rampseed, frame_size/sizeof(int16_t), pM->num_chans );
   }
   else if ( gI2s.sinectl.freq )
   {
      halAudioSine( egressp, &gI2s.sinectl, frame_size/2, pM->num_chans );
   }
   else if ( gI2s.loop_ig2eg )
   {
      /* software loopback ingress to egress; Make sure channel configuration is the same first. */
      if ( pM->num_chans == gI2s.igr.num_chans )
      {
         memcpy( egressp, gI2s.igr.buf[gI2s.active_idx].virt, frame_size );
      }
   }

   if ( pInfo->csx_data[HALAUDIO_CSX_POINT_DAC].csx_ops.csxCallback )
   {
      pInfo->csx_data[HALAUDIO_CSX_POINT_DAC].csx_ops.csxCallback( egressp, frame_size, pInfo->csx_data[HALAUDIO_CSX_POINT_DAC].priv );
   }

   /* DMA egress samples */
   rc = dma_transfer_to_device( pM->dma.handle,
         pM->buf[gI2s.active_idx].phys, pM->dma.fifo_addr, frame_size );
   if ( rc )
   {
      gI2s.errs.dma_egr++;
   }
}

/***************************************************************************/
/**
*  Map sampling frequency in Hz to I2S rate
*
*  @return
*     i2sHw_SAMPLERATE  - on success a valid rate
*     -1                - on error
*/
static i2sHw_SAMPLERATE getSampRate( int samp_freq )
{
   int i;

   for ( i = 0; i < sizeof(gI2s_samp_freq)/sizeof(gI2s_samp_freq[0]); i++ )
   {
      if ( samp_freq <= gI2s_samp_freq[i].samp_freq )
      {
         return gI2s_samp_freq[i].i2s_rate;
      }
   }

   /* If all else fails, return a sane rate */
   return i2sHw_SAMPLERATE_8;
}

/***************************************************************************/
/**
*  Write method used to directly write samples to a channel's DAC buffers.
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remarks
*/
static int i2sWrite(
   int                     chno,       /**< (i) channel index */
   int                     bytes,      /**< (i) Number of bytes to write */
   const char             *audiobuf,   /**< (i) Pointer to audio samples */
   HALAUDIO_CODEC_IORW_CB  usercb,     /**< (i) User callback to request for more data */
   void                   *userdata    /**< (i) User data */
)
{
   return halAudioWriteRequest( &gI2s.write, bytes, audiobuf, usercb, userdata );
}

/***************************************************************************/
/**
*  Cleanup I2S write memory resources
*
*  @return
*/
static int i2sWriteTerm( void )
{
   return halAudioWriteFree( &gI2s.write );
}

/***************************************************************************/
/**
*  Sysctl callback to handle egress to ingress loopback
*
*  @return  0 success, otherwise failure
*/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_doEg2IgLoopback( ctl_table *table, int write,
      void __user *buffer, size_t *lenp, loff_t *ppos )
#else
static int proc_doEg2IgLoopback( ctl_table *table, int write, struct file *filp,
      void __user *buffer, size_t *lenp, loff_t *ppos )
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
      I2sHw_EnableLoopback( i2sNo, gI2s.loop_eg2ig );
   }
   return rc;
}

/***************************************************************************/
/**
*  Sysctl callback to handle sine generation test
*
*  @return  0 success, otherwise failure
*/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_doSineGen( ctl_table *table, int write,
      void __user *buffer, size_t *lenp, loff_t *ppos )
#else
static int proc_doSineGen( ctl_table *table, int write, struct file *filp,
      void __user *buffer, size_t *lenp, loff_t *ppos )
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
      /* Skip 0 hz */
      if ( gI2s.sinectl.freq )
      {
         halAudioSineConfig( &gI2s.sinectl, gI2s.sinectl.freq, gI2s.samp_freq );
      }
   }
   return rc;
}

/***************************************************************************/
/**
*  Proc files initialization
*
*  @return  Nothing
*/
static void i2sProcInit( void )
{
   char procName[10];

   sprintf( procName, "i2s%u", i2sNo );

   create_proc_read_entry( procName, 0, NULL, i2sReadProc, NULL );

   gSysCtl[0].procname = gSysCtlName[i2sNo];
   gSysCtlHeader = register_sysctl_table( gSysCtl );
}

/***************************************************************************/
/**
*  Proc files termination
*
*  @return  Nothing
*/
static void i2sProcTerm( void )
{
   char procName[10];

   sprintf( procName, "i2s%u", i2sNo );

   remove_proc_entry( procName, NULL );

   if ( gSysCtlHeader )
   {
      unregister_sysctl_table( gSysCtlHeader );
   }
}

/***************************************************************************/
/**
*  Proc read callback function
*
*  @return  Number of characters to print
*/
static int i2sReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   int len = 0;
   char igbuf[I2S_MAX_DMA_BUFFER_SIZE_BYTES];
   char egbuf[I2S_MAX_DMA_BUFFER_SIZE_BYTES];
   unsigned long flags;

   (void) start; (void) offset; (void) count;      /* avoid compiler warning */

   memset( igbuf, 0, sizeof(igbuf) );
   memset( egbuf, 0, sizeof(egbuf) );

   /* Cache data in critical section */
   local_irq_save( flags );
   memcpy( igbuf, gI2s.igr.buf[gI2s.active_idx].virt, gI2s.igr.frame_size );
   memcpy( egbuf, gI2s.egr.buf[gI2s.active_idx].virt, gI2s.egr.frame_size );
   local_irq_restore( flags );

   len += sprintf( buf+len, "      I2S CH%i @ %i Hz, %i us\n",
         i2sNo, gI2s.samp_freq, gI2s.frame_period );
   len += sprintf( buf+len, "Igr (%i bytes):\n", gI2s.igr.frame_size );
   halAudioPrintMemory( buf, &len, igbuf, gI2s.igr.frame_size/sizeof(int16_t), 1 /* HEX_FORMAT */, 10 /* line length */,
         2 /* word width */, 0 /* print_addr */, 0 /* addr */ );

   len += sprintf( buf+len, "Egr (%i bytes):\n", gI2s.egr.frame_size );
   halAudioPrintMemory( buf, &len, egbuf, gI2s.egr.frame_size/sizeof(int16_t), 1 /* HEX_FORMAT */, 10 /* line length */,
         2 /* word width */, 0 /* print_addr */, 0 /* addr */ );

   /* Error report and other information */
   len += sprintf( buf+len, "Irqs:        igress=%u egress=%u\n", gI2s.igr.isrcount, gI2s.egr.isrcount );
   len += sprintf( buf+len, "DMA errors:  igress=%i egress=%i sync=%i\n", gI2s.errs.dma_igr, gI2s.errs.dma_egr, gI2s.errs.dma_sync );

   if ( gI2s.ramp_check )
   {
      len += sprintf( buf+len, "Ramp check:  delay=%i errors=%i\n", gI2s.ramp_check_delay, gI2s.ramp_check_errs );
   }

   /* Channel parameters */
   len += sprintf( buf+len, "Priming:     samples=%i\n", gI2s.egr_prime/sizeof(int16_t) );

   len += sprintf( buf+len, "Ingress DMA: device=0x%x fifo=0x%x handle=0x%x\n",
         gI2s.igr.dma.device, gI2s.igr.dma.fifo_addr, gI2s.igr.dma.handle );
   len += sprintf( buf+len, "Egress DMA:  device=0x%x fifo=0x%x handle=0x%x\n",
         gI2s.egr.dma.device, gI2s.egr.dma.fifo_addr, gI2s.egr.dma.handle );
   len += sprintf( buf+len, "Buffer Igr:  [0] 0x%.8x (0x%.8x phy)\n"
                            "             [1] 0x%.8x (0x%.8x phy)\n",
         (unsigned int)gI2s.igr.buf[0].virt, gI2s.igr.buf[0].phys, (unsigned int)gI2s.igr.buf[1].virt, gI2s.igr.buf[1].phys );
   len += sprintf( buf+len, "Buffer Egr:  [0] 0x%.8x (0x%.8x phy)\n", (unsigned int)gI2s.egr.buf[gI2s.active_idx].virt, gI2s.egr.buf[gI2s.active_idx].phys );
   len += sprintf( buf+len, "Debug:       running=%i prepared=%i\n", atomic_read( &gI2s.running ), atomic_read( &gI2s.prepared ));


   *eof = 1;
   return len+1;
}

/***************************************************************************/
/**
*  I2S mixer callback for outgoing data (i.e. ingress)
*
*  @return
*     NULL     - non-matching frame size or non-existent buffer
*     ptr      - pointer to ingress buffer
*/
static int16_t *i2sMixerCb_IngressGetPtr(
   int   numBytes,            /**< (i) frame size in bytes */
   void *privdata             /**< (i) User data           */
)
{
   int16_t *ptr;

   (void) privdata;

   ptr = NULL;
   if ( numBytes == gI2s.igr.frame_size )
   {
      ptr = (int16_t*)gI2s.igr.buf[gI2s.active_idx].virt;
   }

   I2SLOG( "Interf=%i numBytes=%i ptr=0x%x", i2sNo, numBytes, ptr );

   return ptr;
}

/***************************************************************************/
/**
*  I2S mixer callback for incoming data (i.e. egress)
*
*  @return
*     NULL     - non-matching frame size or non-existent buffer
*     ptr      - pointer to egress buffer
*/
static int16_t *i2sMixerCb_EgressGetPtr(
   int   numBytes,            /**< (i) frame size in bytes */
   void *privdata             /**< (i) mixer port index  */
)
{
   int16_t *ptr;

   (void) privdata;

   ptr = NULL;
   if ( numBytes == gI2s.egr.frame_size )
   {
      ptr = (int16_t*)gI2s.egr.buf[gI2s.active_idx].virt;
   }

   I2SLOG( "Interf=%i numBytes=%i ptr=0x%x", i2sNo, numBytes, ptr );

   return ptr;
}

/***************************************************************************/
/**
*  I2S mixer callback to indicate that the egress data has been deposited.
*
*  @return     None
*
*  @remark
*     This callback is used as a trigger to DMA more data to the DAC, if
*     appropriate.
*/
static void i2sMixerCb_EgressDone(
   int   numBytes,            /**< (i) frame size in bytes */
   void *privdata             /**< (i) mixer port index  */
)
{
   (void) privdata;

   I2SLOG( "I2S [ch %i] pkts=%i", i2sNo, atomic_read( &gI2s.queued_pkts_egr ));

   /* Take action only when I2S channels are actually running */
   if ( atomic_read( &gI2s.running ))
   {
      atomic_inc( &gI2s.queued_pkts_egr ); /* new packet */

      /* Exactly 1 packet awaits, thus DMA was idle. Start a new transfer right away. */
      if ( atomic_read( &gI2s.queued_pkts_egr ) == 1 )
      {
         i2sDmaEgressDoTransfer();
      }
   }
}

/***************************************************************************/
/**
*  I2s mixer callback to flush the egress buffers when the last destination
*  connection is removed.
*
*  @return     None
*/
static void i2sMixerCb_EgressFlush(
   void *privdata             /*<< (i) private data */
)
{
   unsigned long flags;

   (void) privdata;

   /* Clear double buffers */
   local_irq_save( flags );
   memset( gI2s.egr.buf[0].virt, 0, I2S_MAX_DMA_BUFFER_SIZE_BYTES );
   memset( gI2s.egr.buf[1].virt, 0, I2S_MAX_DMA_BUFFER_SIZE_BYTES );
   local_irq_restore( flags );
}

/***************************************************************************/
/**
*  I2S mixer port registration
*
*  @return  None
*/
static int i2sMixerPortsRegister( void )
{
   AMXR_PORT_CB   cb;
   AMXR_PORT_ID   portid;
   char           name[32];
   int            err;

   memset( &cb, 0, sizeof(cb) );

   cb.getsrc         = i2sMixerCb_IngressGetPtr;
   cb.getdst         = i2sMixerCb_EgressGetPtr;
   cb.dstdone        = i2sMixerCb_EgressDone;
   cb.dstcnxsremoved = i2sMixerCb_EgressFlush;

   sprintf( name, "halaudio.i2s%i", i2sNo );

   err = amxrCreatePort( name, &cb, NULL /* privdata */,
         gI2s.samp_freq, gI2s.egr.num_chans, gI2s.egr.frame_size,
         gI2s.samp_freq, gI2s.igr.num_chans, gI2s.igr.frame_size,
         &portid );
   if ( err )
   {
      return err;
   }

   gI2s.mixer_port = portid;

   return 0;
}

/***************************************************************************/
/**
*  I2S mixer port de-registration
*
*  @return  None
*/
static int i2sMixerPortsDeregister( void )
{
   int err, rc;

   rc = 0;
   err = amxrRemovePort( gI2s.mixer_port );
   if ( err )
   {
      printk( KERN_ERR "I2S [Interf=%d]: failed to deregister mixer port\n", i2sNo );
      rc = err;
   }
   gI2s.mixer_port = NULL;  /* invalidate handle */

   return rc;
}

/***************************************************************************/
/**
*  Driver initialization called when module loaded by kernel
*
*  @return
*     0              Success
*     -ve            Error code
*/
static int __init i2s_init( void )
{
   char name[10];
   int  err;

   printk( KERN_INFO "HAL Audio I2S Driver: 1.0. Built %s %s\n", __DATE__, __TIME__ );

   sprintf( name, "I2S%u", i2sNo );

   if ( nogpio )
   {
      useAK4642 = 0; /* override AK4642 use */
   }

   err = halAudioAddInterface( &halaudio_i2s_ops, 1, name,
         I2S_DEFAULT_FRAME_PERIOD, 1 /* synchronize */, &gInterfHandle );
   if ( err )
   {
      printk( KERN_ERR "I2S: failed to install the audio interface %d!\n", err );
      return err;
   }

   return 0;
}

/***************************************************************************/
/**
*  Driver destructor routine. Frees all resources
*/
static void __exit i2s_exit( void )
{
   halAudioDelInterface( gInterfHandle );
}

module_init( i2s_init );
module_exit( i2s_exit );

MODULE_AUTHOR( "Broadcom" );
MODULE_DESCRIPTION( "HAL Audio BCMRING I2S Low Level Driver" );
MODULE_LICENSE( "GPL v2" );

