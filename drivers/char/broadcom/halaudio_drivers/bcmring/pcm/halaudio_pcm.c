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
*  halaudio_pcm.c
*
*  PURPOSE:
*
*     This file contains the PCM driver routines.
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
#include <linux/broadcom/knllog.h>           /* for debugging */
#include <linux/sysctl.h>
#include <linux/device.h>
#include <linux/dmapool.h>                   /* Linux DMA API */
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/broadcom/halaudio.h>
#include <linux/broadcom/halaudio_lib.h>
#include <linux/broadcom/amxr.h>
#include <linux/broadcom/halaudio_pcm_platform.h>
#include <linux/broadcom/gpio.h>                /* GPIO register accesses */
#include <linux/clk.h>

#include <mach/dma.h>
#include <mach/csp/pcmHw_reg.h>
#include <mach/csp/chipcHw_inline.h>
#include <mach/csp/cap.h>
#include <mach/csp/gpiomux.h>

#include <csp/pcmHw.h>

#include "pcm_drv.h"
#include "dma_priv.h"

/* ---- Public Variables ------------------------------------- */
/* ---- Private Constants and Types -------------------------- */

/* Two hardware PCM channels, known also as "PCM interfaces,"
 * are supported. These channels are mapped into HAL Audio channels.
 */
#define PCM_MAX_NUM_HW_CHANNELS           2       /* Do not change */

/* Hardware PCM time slots are configured based on bit clock
 * frequency, frame synchronization rate and sample bit width. The
 * number of physical time slots is calculated by the following
 * formula:
 *
 * num_slots = bit_clock / fs_rate / bit_width
 *
 * For example, using the maximum bit clock of 8192 kHz, frame sync
 * rate of 8 kHz and bit width of 8 bits, the total number of time slots
 * equals 128. Of the 128 time slots, a maximum of 8 may be selected.
 * These 8 time slots are referred to as "slot channels."
 *
 * The PCM DMA interface always presents data for the 8 slot channels
 * in an interleaved fashion if all of them are active or not.
 *
 * This driver assumes that the active slot channels are adjacent
 * to each other and start at the first slot channel. For example,
 * if two slot channels are active, they would take up spots
 * 0 and 1 out of eight spots.
 */
#define PCM_MAX_BIT_CLOCK_FREQ            8192000  /* In Hz. Do not change */
#define PCM_MAX_NUM_SLOT_CHANNELS         8        /* Do not change */
#define PCM_MAX_SAMPLING_RATE             16000    /* in Hz - do not change */

/* PCM channel default configuration */
#define PCM_DEFAULT_FRAME_PERIOD          5000     /* In usec - do not change */
#define PCM_DEFAULT_INTERCHS              1        /* interleaved data: 1 for mono, 2 for stereo, etc */
#define PCM_DEFAULT_SLOT_CHANS_USED       1        /* slot channels used, max 8 */
#define PCM_DEFAULT_BCLK_FREQ_KHZ         1024     /* bit clock in kHz */
#define PCM_DEFAULT_SAMP_WIDTH            2        /* sample byte width */
#define PCM_DEFAULT_SAMP_FREQ             8000     /* In Hz  */

#define CALCFRAMESZ(freq,per,sampwidth)   ((freq) * (per) * (sampwidth) / 1000000 ) /* in bytes */
#define PCM_DEFAULT_FRAME_SIZE            CALCFRAMESZ( PCM_DEFAULT_SAMP_FREQ, PCM_DEFAULT_FRAME_PERIOD, PCM_DEFAULT_SAMP_WIDTH ) /* do not change */
#define PCM_MAX_FRAME_SIZE                CALCFRAMESZ( PCM_MAX_SAMPLING_RATE, PCM_DEFAULT_FRAME_PERIOD, PCM_DEFAULT_SAMP_WIDTH ) /* do not change */

/* PCM DMA defintions */
#define PCM_DEFAULT_DMA_FRAME_SIZE        (PCM_MAX_NUM_SLOT_CHANNELS * PCM_DEFAULT_FRAME_SIZE)
#define PCM_MAX_DMA_FRAME_SIZE            (PCM_MAX_NUM_SLOT_CHANNELS * PCM_MAX_FRAME_SIZE)

/* Allocate a DMA buffer to accommodate a 5ms frame at max sampling rate */
#define PCM_MAX_DMA_BUFFER_SIZE_BYTES     PCM_MAX_DMA_FRAME_SIZE
#define PCM_DMA_ALIGN_IN_BYTES            8        /* DMA buffers should be 64-bit aligned for efficiency */
#define PCM_DMA_ALLOC_CROSS_RESTRICT      0

/* Egress Priming configuration */
#define PCM_DEFAULT_PRIME_PERIOD          PCM_DEFAULT_FRAME_PERIOD
#define PCM_DEFAULT_PRIME_SIZE_BYTES      PCM_DEFAULT_DMA_FRAME_SIZE
#define PCM_MAX_PRIME_BUFFER_SIZE_BYTES   (PCM_MAX_DMA_FRAME_SIZE * 2)

/* CSX Data stucture */
struct pcm_csx_data
{
   CSX_IO_POINT_FNCS    csx_ops;
   void                *priv;
};

/* PCM error statistics */
struct pcm_ch_errs
{
   unsigned int         dma_egr;          /* DMA egress errors */
   unsigned int         dma_igr;          /* DMA ingress erros */
   unsigned int         dma_sync;         /* DMA egress sync errors */
};

/* PCM channel configuration and parameters */
struct pcm_ch_cfg
{
   unsigned int         ch;               /* PCM HAL Audio channel number */
   unsigned int         interchs;         /* Number of interleaved channels: 1 for mono, 2 for stereo, etc */
   unsigned int         slotassigned[PCM_MAX_NUM_SLOT_CHANNELS]; /* Assigned time slot, -1 means unused */
   unsigned int         slotchansused;    /* Number of used slot channels */

   /* DMA buffers containing interleaved data */
   struct dma_cfg       dma_igr;          /* Ingress (Peripheral to memory) DMA config */
   struct dma_cfg       dma_egr;          /* Egress  (Memory to peripheral) DMA config */
   struct dma_data_buf  buf_igr[2];       /* Ingress (tx) double buffer */
   struct dma_data_buf  buf_egr[2];       /* Egress (rx) double buffer */

   void                *igrdatap;         /* De-interleaved ingress buffer */
   void                *egrdatap;         /* De-interleaved egress buffer */

   /* Channel configuration */
   unsigned int         samp_freq;        /* Sampling frequency in Hz */
   unsigned int         frame_period;     /* Frame period in usec. Debug only  */
   unsigned int         dma_frame_size;   /* DMA Frame size in bytes. Used in ISR */
   unsigned int         frame_size;       /* Frame size in bytes. Used in ISR */
   unsigned int         dma_prime_egr;    /* Egress priming in bytes for DMA frames */
   pcmHw_Config         pcmhw_cfg;        /* Hardware configuration */

   HALAUDIO_EQU         equ_igr;          /* Ingress equalizer parameters */
   HALAUDIO_EQU         equ_egr;          /* Egress equalizer parameters */

   /* Write state */
   HALAUDIO_WRITE       write;            /* Write state */

   /* ISR Status */
   int                  active_idx;       /* Index to active buffer for ingress double buffers */
   int                  active_idx_eg;    /* Index to active buffer for egress double buffers */
   struct pcm_ch_errs   errs;             /* Channel errors */

   /* Debug facilities */
   int                  loop_ig2eg;       /* Sysctl: Ingress to egress loopback */
   int                  loop_eg2ig;       /* Sysctl: Egress to ingress loopback */
   int                  ramp_igr;         /* Sysctl: Ingress ramp generation */
   int                  ramp_egr;         /* Sysctl: Egress ramp generation */
   int                  ramp_check;       /* Sysctl: Validate ramp in hw eg2ig loopback */
   int                  ramp_check_delta; /* Sysctl: Delta between igr and egr ramps */
   int                  ramp_check_errs;  /* Sysctl: Number of glitches in received ramp */
   uint16_t             rampseed_igr;     /* Ingress ramp seed */
   uint16_t             rampseed_egr;     /* Egress ramp seed */
   HALAUDIO_SINECTL     sinectl;          /* Sine generation state */
   unsigned int         isrcount_igr;     /* Ingress ISR counter */
   unsigned int         isrcount_egr;     /* Egress ISR counter */

   /* Mixer facilities */
   AMXR_PORT_ID         mixer_port;       /* Mixer port handle for channel */

   /* CSX data */
   struct pcm_csx_data  csx_data[HALAUDIO_NUM_CSX_POINTS]; /* Array of CSX data structures */
};

/* PCM channel information structure */
struct pcm_info
{
   int                  initialized;
   atomic_t             running;          /* Flag indicating channels are active */
   atomic_t             prepared;         /* Flag indicating channels are prepared to be enabled */
   struct dma_data_buf  zero;             /* DMA scratch buffer used for priming */
   struct pcm_ch_cfg    ch[PCM_MAX_NUM_HW_CHANNELS];
};

/* Debug trace */
#define PCM_ENABLE_LOG           0
#if PCM_ENABLE_LOG
#define PCMLOG                   KNLLOG
#else
#define PCMLOG(c,d...)
#endif

/* ---- Private Variables ------------------------------------ */

static short halDacFiltHist[HALAUDIO_EQU_COEFS_MAX_NUM];
static short halAdcFiltHist[HALAUDIO_EQU_COEFS_MAX_NUM];

static HALAUDIO_PCM_PLATFORM_INFO gPcmPlatformInfo;

static struct pcm_info gPcm =
{
   .ch =
   {
      [0] =
      {
         .ch            = 0,
         .slotchansused = PCM_DEFAULT_SLOT_CHANS_USED,
         .slotassigned  = { 0, -1, -1, -1, -1, -1, -1, -1 },
         .interchs      = PCM_DEFAULT_INTERCHS,
         .dma_igr       =
         {
            .device     = DMA_DEVICE_APM_PCM0_DEV_TO_MEM,
            .fifo_addr  = (MM_ADDR_IO_APM + PCM0_DEV_TO_MEM_FIFO_OFFSET),
         },
         .dma_egr       =
         {
            .device     = DMA_DEVICE_APM_PCM0_MEM_TO_DEV,
            .fifo_addr  = (MM_ADDR_IO_APM + PCM0_MEM_TO_DEV_FIFO_OFFSET),
         },
         .samp_freq     = PCM_DEFAULT_SAMP_FREQ,
         .frame_period  = PCM_DEFAULT_FRAME_PERIOD,
         .frame_size    = PCM_DEFAULT_FRAME_SIZE,
         .dma_frame_size= PCM_DEFAULT_DMA_FRAME_SIZE,
         .dma_prime_egr = PCM_DEFAULT_PRIME_SIZE_BYTES,
         .pcmhw_cfg     =
         {
            .dataDelayCycles = 1,
            .fsEdgeTrig      = 1,
            .fsActiveLow     = 0,
            .clkInverted     = 1,
            .byteOrder       = 0,
            .clkFreqkHz      = PCM_DEFAULT_BCLK_FREQ_KHZ,
            .fsMode          = pcmHw_FRAME_SYNC_SHORT,
            .mode            = pcmHW_MODE_TDM_MASTER,
         },
      },
      [1] =
      {
         .ch            = 1,
         .slotchansused = PCM_DEFAULT_SLOT_CHANS_USED,
         .slotassigned  = { 0, -1, -1, -1, -1, -1, -1, -1 },
         .interchs      = PCM_DEFAULT_INTERCHS,
         .dma_igr       =
         {
            .device     = DMA_DEVICE_APM_PCM1_DEV_TO_MEM,
            .fifo_addr  = (MM_ADDR_IO_APM + PCM1_DEV_TO_MEM_FIFO_OFFSET),
         },
         .dma_egr       =
         {
            .device     = DMA_DEVICE_APM_PCM1_MEM_TO_DEV,
            .fifo_addr  = (MM_ADDR_IO_APM + PCM1_MEM_TO_DEV_FIFO_OFFSET),
         },
         .samp_freq     = PCM_DEFAULT_SAMP_FREQ,
         .frame_period  = PCM_DEFAULT_FRAME_PERIOD,
         .frame_size    = PCM_DEFAULT_FRAME_SIZE,
         .dma_frame_size= PCM_DEFAULT_DMA_FRAME_SIZE,
         .dma_prime_egr = PCM_DEFAULT_PRIME_SIZE_BYTES,
         .pcmhw_cfg     =
         {
            .dataDelayCycles = 1,
            .fsEdgeTrig      = 1,
            .fsActiveLow     = 0,
            .clkInverted     = 1,
            .byteOrder       = 0,
            .clkFreqkHz      = PCM_DEFAULT_BCLK_FREQ_KHZ,
            .fsMode          = pcmHw_FRAME_SYNC_SHORT,
            .mode            = pcmHW_MODE_TDM_MASTER,
         },
      },
   }
};

/* DMA memory pool */
static struct dma_pool          *gDmaPool;

/* Installed callback. Called when all ingress processing has completed */
static HALAUDIO_IF_FRAME_ELAPSED gPcmElapsedCb;
static void                     *gPcmUserData;

/* Reference counter used to determine when all ingress interrupt processing
 * have completed
 */
static int                       gPcmRefCount;

/* HAL Audio interface handle */
static HALAUDIO_IF_HDL           gInterfHandle;

/*
 * Private functions
 */
static void pcm_platform_exit( HALAUDIO_PCM_PLATFORM_INFO *info );

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

#define PCM_CHAN_SYS_CTL( CHAN )                    \
   {                                                \
      .procname      = "loophwe2i"#CHAN,            \
      .data          = &gPcm.ch[CHAN].loop_eg2ig,   \
      .maxlen        = sizeof(int),                 \
      .mode          = 0644,                        \
      .proc_handler  = &proc_doEg2IgLoopback,       \
      .extra1        = &gPcm.ch[CHAN],              \
   },                                               \
   {                                                \
      .procname      = "sine_freq"#CHAN,            \
      .data          = &gPcm.ch[CHAN].sinectl.freq, \
      .maxlen        = sizeof(int),                 \
      .mode          = 0644,                        \
      .proc_handler  = &proc_doSineGen,             \
      .extra1        = &gPcm.ch[CHAN],              \
   },                                               \
   { CTL_TABLE_INT( "loopi2e"#CHAN,    &gPcm.ch[CHAN].loop_ig2eg) }, \
   { CTL_TABLE_INT( "ramp_igr"#CHAN,   &gPcm.ch[CHAN].ramp_igr ) },  \
   { CTL_TABLE_INT( "ramp_egr"#CHAN,   &gPcm.ch[CHAN].ramp_egr ) },  \
   { CTL_TABLE_INT( "ramp_check"#CHAN, &gPcm.ch[CHAN].ramp_check ) }

static struct ctl_table gSysCtlPCM[] =
{
   PCM_CHAN_SYS_CTL( 0 ),
   PCM_CHAN_SYS_CTL( 1 ),
   {}
};

static struct ctl_table gSysCtl[] =
{
    {
      .procname   = "pcm",
      .mode       = 0555,
      .child      = gSysCtlPCM,
    },
    {}
};

static struct ctl_table_header *gSysCtlHeader;
static struct proc_dir_entry   *gProcDir;

/* APM clocks */
static struct clk *gApm100_clk;  /* Child clock */
static struct clk *gApm200_clk;  /* Parent clock */

/* Number of currently supported PCM channels */
static unsigned int gPcmNumChans;

/* ---- Private Function Prototypes -------------------------- */

static int  pcmInit( HALAUDIO_IF_FRAME_ELAPSED cb, void *data );
static int  pcmExit( void );
static int  pcmPrepare( void );
static int  pcmEnable( void );
static int  pcmDisable( void );
static int  pcmPmShutdown( void );
static int  pcmPmResume( void );
static int  pcmSetFreq( int chno, int freqHz );
static int  pcmGetFreq( int chno, int *freqHz );
static int  pcmEquParmSet( int chno, HALAUDIO_DIR dir, const HALAUDIO_EQU *equ );
static int  pcmEquParmGet( int chno, HALAUDIO_DIR dir, HALAUDIO_EQU *equ );
static int  pcmCodecInfo( int chno, HALAUDIO_CODEC_INFO *codec_info );
static int  pcmCsxSet( int chno, HALAUDIO_CSX_POINT_ID point, const CSX_IO_POINT_FNCS *fncp, void *data );
static int  pcmWrite( int chno, int bytes, const char *audiobuf, HALAUDIO_CODEC_IORW_CB usercb, void *userdata );

static int  pcmDmaInit( void );
static int  pcmDmaTerm( void );
static void pcmDmaIngressHandler( DMA_Device_t dev, int reason, void *data );
static void pcmDmaEgressHandler( DMA_Device_t dev, int reason, void *data );
static void pcmProcInit( void );
static void pcmProcTerm( void );
static int  pcmReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data );
static int  pcmWriteTerm( void );
static int  pcmDisableNoCheck( void );

static int  pcmMixerPortsRegister( void );
static int  pcmMixerPortsDeregister( void );
static int  pcmMixerSetFreq( AMXR_PORT_ID portid, int freq, int bytes );

static void interleave_data( uint16_t *dmabufp, const uint16_t *srcp, int dma_fsize, int slots_used );
static void extract_data( uint16_t *dstp, const uint16_t *dmabufp, int dst_fsize, int slots_used );

/* HAL Audio PCM interface operations
 */
static HALAUDIO_IF_OPS halaudio_pcm_ops __initdata =
{
   .init                = pcmInit,
   .exit                = pcmExit,
   .prepare             = pcmPrepare,
   .enable              = pcmEnable,
   .disable             = pcmDisable,
   .pm_shutdown         = pcmPmShutdown,
   .pm_resume           = pcmPmResume,
   .codec_ops           =
   {
      .setfreq          = pcmSetFreq,
      .getfreq          = pcmGetFreq,
      .setequ           = pcmEquParmSet,
      .getequ           = pcmEquParmGet,
/*      .read             = pcmRead, */
      .write            = pcmWrite,
      .info             = pcmCodecInfo,
      .setcsx           = pcmCsxSet,
   },
};

/* Supported sampling frequencies. The list is sorted from lowest to highest.
 */
static unsigned int gPcmFreqs[] =
{
   8000,
   16000
};

/* ---- Functions -------------------------------------------- */

/***************************************************************************/
/**
*  PCM retrieve codec info
*
*  @return
*     0        PCM codec infor retrieved succesfully
*     -ve      Error code
*/
static int pcmCodecInfo( int chno, HALAUDIO_CODEC_INFO *codec_info )
{
   struct pcm_ch_cfg *ch;
   int                i;

   ch = &gPcm.ch[chno];

   memset( codec_info, 0, sizeof(*codec_info) );

   for ( i = 0; i < sizeof(gPcmFreqs)/sizeof(gPcmFreqs[0]) && i < HALAUDIO_MAX_NUM_FREQ_SETTINGS; i++ )
   {
      codec_info->freqs.freq[i] = gPcmFreqs[i];
      codec_info->freqs.num++;
   }

   codec_info->channels_tx  = ch->interchs;          /* # of channels for tx (mic), 1=mono, 2=stereo, etc. */
   codec_info->channels_rx  = ch->interchs;          /* # of channels for rx (spk), 1=mono, 2=stereo, etc. */
   codec_info->equlen_tx = HALAUDIO_EQU_COEFS_MAX_NUM; /* equalizer available */
   codec_info->equlen_rx = HALAUDIO_EQU_COEFS_MAX_NUM; /* equalizer available */
   codec_info->sample_width = PCM_DEFAULT_SAMP_WIDTH;/* sample size width in bytes */
   codec_info->read_format  = HALAUDIO_FMT_S16_LE;   /* Read format */
   codec_info->write_format = HALAUDIO_FMT_S16_LE;   /* Write format */
   codec_info->mics         = 1;                     /* # of mic paths. 0 means no ADC */
   codec_info->spkrs        = 1;                     /* # of speaker paths. 0 means no DAC */
   codec_info->bulk_delay   = -1;                    /* Echo bulk delay in samples, -1 = not calibrated */

   sprintf( codec_info->name, "PCM CH%i", ch->ch );

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
static int pcmCsxSet(
   int                        chno, /*<< (i) Codec channel number */
   HALAUDIO_CSX_POINT_ID      point,/*<< (i) Point ID to install the CSX point */
   const CSX_IO_POINT_FNCS   *fncp, /*<< (i) Ptr to CSX callbacks */
   void                      *data  /*<< (i) User data to pass back to callbacks */
)
{
   struct pcm_ch_cfg *ch;
   unsigned long      flags;
   int err = 0;

   ch = &gPcm.ch[chno];

   local_irq_save( flags );
   switch( point )
   {
      case HALAUDIO_CSX_POINT_ADC:
      {
         memcpy( &ch->csx_data[HALAUDIO_CSX_POINT_ADC].csx_ops,
                 fncp,
                 sizeof(ch->csx_data[HALAUDIO_CSX_POINT_ADC].csx_ops) );

         ch->csx_data[HALAUDIO_CSX_POINT_ADC].priv = data;
         break;
      }
      case HALAUDIO_CSX_POINT_DAC:
      {
         memcpy( &ch->csx_data[HALAUDIO_CSX_POINT_DAC].csx_ops,
                 fncp,
                 sizeof(ch->csx_data[HALAUDIO_CSX_POINT_DAC].csx_ops) );

         ch->csx_data[HALAUDIO_CSX_POINT_DAC].priv = data;
         break;
      }
      default:
      {
         err = -EINVAL;
      }
   }
   local_irq_restore( flags );

   return 0;
}

/***************************************************************************/
/**
*  PCM initialization
*
*  @return
*     0        PCM channel initialized successfully
*     -ENOMEM  Insufficient memory for DMA audio buffers
*     Other    Other errors
*/
int pcmInit(
   HALAUDIO_IF_FRAME_ELAPSED  cb,   /*<< (i) Callback to call when one frame elapses */
   void                      *data  /*<< (i) User data to pass to callback */
)
{
   int                         err, i;
   HALAUDIO_PCM_PLATFORM_INFO *info = &gPcmPlatformInfo;

   atomic_set( &gPcm.running, 0 );
   atomic_set( &gPcm.prepared, 0 );

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

   err = clk_enable( gApm100_clk );
   if ( err )
   {
      printk( KERN_ERR "%s: Failed to enable APM100 clock\n", __FUNCTION__ );
      goto exit_release_clocks;
   }

   /* Enable bus interfaces */
   chipcHw_audioChannelEnable( chipcHw_REG_AUDIO_CHANNEL_ENABLE_NTP_CLOCK | chipcHw_REG_AUDIO_CHANNEL_ENABLE_APM_CLOCK );
   chipcHw_audioChannelEnable( chipcHw_REG_AUDIO_CHANNEL_ENABLE_PCM0_CLOCK | chipcHw_REG_AUDIO_CHANNEL_ENABLE_PCM1_CLOCK );

   chipcHw_softReset( chipcHw_REG_SOFT_RESET_PCM );

   /* Initialize PCM hardware block */
   for ( i = info->channel_select; i < (info->channel_select + info->channels); i++ )
   {
      err = PcmHw_init( i );
      if ( err )
      {
         printk( KERN_ERR "%s: PCM [ch=%u]: Failed to initialize PCM err=%i\n", __FUNCTION__, i, err );
         goto exit_disable_clocks;
      }
   }

   err = pcmDmaInit();
   if ( err )
   {
      printk( KERN_ERR "%s: failed to initialize DMA resources err=%i\n", __FUNCTION__, err );
      goto cleanup_alloc_buffers;
   }

   err = pcmMixerPortsRegister();
   if ( err )
   {
      printk( KERN_ERR "%s: failed to register mixer ports err=%i\n", __FUNCTION__, err );
      goto cleanup_alloc_buffers;
   }

   pcmProcInit();

   /* Save HAL Audio specifics */
   gPcmElapsedCb = cb;
   gPcmUserData  = data;

   gPcm.initialized = 1;

   return 0;

cleanup_alloc_buffers:  /* Full cleanup */
   pcmExit();
   return err;

exit_disable_clocks:
   clk_disable( gApm100_clk );

exit_release_clocks:
   clk_put( gApm100_clk );
   clk_put( gApm200_clk );

   return err;
}

/***************************************************************************/
/**
*  Cleanup PCM resources
*
*  @return  Nothing.
*
*  @remark
*/
static int pcmExit( void )
{
   int                         rc, i, error = 0;
   HALAUDIO_PCM_PLATFORM_INFO *info = &gPcmPlatformInfo;

   pcmProcTerm();

   rc = pcmMixerPortsDeregister();
   if ( rc )
   {
      printk( KERN_ERR "%s: failed to cleanup mixer ports\n", __FUNCTION__ );
      error = rc;
   }

   rc = pcmDisable();
   if ( rc != 0 )
   {
      printk( KERN_ERR "%s: failed to disable PCM hardware\n", __FUNCTION__ );
      error = rc;
   }

   rc = pcmDmaTerm();
   if ( rc != 0 )
   {
      printk( KERN_ERR "%s: failed to cleanup DMA resources\n", __FUNCTION__ );
      error = rc;
   }

   pcmWriteTerm();

   for ( i = info->channel_select; i < (info->channel_select + info->channels); i++ )
   {
      rc = PcmHw_exit( i );
      if ( rc != 0 )
      {
         printk( KERN_ERR "%s: failed to cleanup PCM hardware [ch=%u]\n", __FUNCTION__, i );
         error = rc;
      }
   }

   gPcm.initialized = 0;

   clk_disable( gApm100_clk );
   clk_put( gApm100_clk );
   clk_put( gApm200_clk );

   return error;
}

/***************************************************************************/
/**
*  Prepare PCM channels to start: allocate DMA channels, configure channel
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
static int pcmPrepare( void )
{
   int                         rc, i, j;
   struct pcm_ch_cfg          *ch;
   HALAUDIO_PCM_PLATFORM_INFO *info = &gPcmPlatformInfo;

   if ( atomic_read( &gPcm.running ) || atomic_read( &gPcm.prepared ))
   {
      return -EBUSY; /* Already running or prepared */
   }

   ch = &gPcm.ch[info->channel_select];
   for ( i = info->channel_select; i < (info->channel_select + info->channels); i++, ch++ )
   {
      /* Index is pre-incremented before use. Set to second buffer to start */
      ch->active_idx    = 1;
      ch->active_idx_eg = 1;
      ch->isrcount_igr  = 0;
      ch->isrcount_egr  = 0;
      memset( &ch->errs, 0, sizeof(ch->errs) );

      /* Install DMA handlers */
      dma_set_device_handler( ch->dma_egr.device, pcmDmaEgressHandler, ch );
      dma_set_device_handler( ch->dma_igr.device, pcmDmaIngressHandler, ch );

      /* PCM channel interface configuration */
      rc = PcmHw_ConfigSet( ch->ch, &ch->pcmhw_cfg );
      if ( rc )
      {
         printk( KERN_ERR "%s: [ch=%u] Failed to set PCM config rc=%i\n",
               __FUNCTION__, ch->ch, rc );
         goto cleanup_dma_channels;
      }

      /* PCM channel sampling frequency */
      rc = PcmHw_FsFreqSet( ch->ch, ch->samp_freq );
      if ( rc )
      {
         printk( KERN_ERR "%s: [ch=%u] Failed to set frame sync freq rc=%i\n",
               __FUNCTION__, ch->ch, rc );
         goto cleanup_dma_channels;
      }

      /* Set data size */
      rc = PcmHw_DataSizeSet( ch->ch, PCM_DEFAULT_SAMP_WIDTH );
      if ( rc )
      {
         printk( KERN_ERR "%s: [ch=%u] Failed to set data size rc=%i\n",
               __FUNCTION__, ch->ch, rc );
         goto cleanup_dma_channels;
      }

      /* Request egress DMA channel */
      ch->dma_egr.handle = dma_alloc_channel( ch->dma_egr.device );
      if ( ch->dma_egr.handle < 0 )
      {
         printk( KERN_ERR "%s: [ch=%u] failed to get MTP DMA channel\n",
               __FUNCTION__, ch->ch );
         rc = -EBUSY;
         goto cleanup_dma_channels;
      }

      /* Clear egress samples */
      memset( ch->buf_egr[0].virt, 0, ch->dma_frame_size );
      memset( ch->buf_egr[1].virt, 0, ch->dma_frame_size );

      /* Allocate egress double buffer DMA descriptors */
      rc = dma_alloc_double_src_descriptors( ch->dma_egr.handle,
            ch->buf_egr[0].phys, ch->buf_egr[1].phys,
            ch->dma_egr.fifo_addr, ch->dma_frame_size );
      if ( rc != 2 )
      {
         printk( KERN_ERR "%s: [ch=%u] %d descs allocated instead of 2.\n",
               __FUNCTION__, ch->ch, rc );
         goto cleanup_dma_channels;
      }

      /* Start egress DMA channel to begin priming */
      rc = dma_start_transfer( ch->dma_egr.handle, 2 * ch->dma_frame_size );
      if ( rc != 0 )
      {
         printk( KERN_ERR "%s: [ch=%u] Failed to start egress DMA\n",
               __FUNCTION__, ch->ch );
         goto cleanup_dma_channels;
      }

      /* Request ingress DMA channel */
      ch->dma_igr.handle = dma_alloc_channel( ch->dma_igr.device );
      if ( ch->dma_igr.handle < 0 )
      {
         printk( KERN_ERR "%s: [ch=%u] failed to get PTM DMA channel\n",
               __FUNCTION__, ch->ch );
         rc = -EBUSY;
         goto cleanup_dma_channels;
      }

      /* Allocate ingress double buffer DMA descriptors */
      rc = dma_alloc_double_dst_descriptors( ch->dma_igr.handle,
            ch->dma_igr.fifo_addr, ch->buf_igr[0].phys, ch->buf_igr[1].phys,
            ch->dma_frame_size );
      if ( rc != 2 )
      {
         printk( KERN_ERR "%s: [ch=%u] %d descs allocated instead of 2.\n",
               __FUNCTION__, ch->ch, rc );
         goto cleanup_dma_channels;
      }

      /* Start ingress DMA channel. DMA does not actually start
       * until PCM channel is enabled.
       */
      rc = dma_start_transfer( ch->dma_igr.handle, 2 * ch->dma_frame_size );
      if ( rc != 0 )
      {
         printk( KERN_ERR "%s: [ch=%u] Failed to start ingress DMA\n",
               __FUNCTION__, ch->ch );
         goto cleanup_dma_channels;
      }

      for ( j = 0; j < PCM_MAX_NUM_SLOT_CHANNELS; j++ )
      {
         /* Allocate timeslot to slot channel */
         if ( ch->slotassigned[j] >= 0 )
         {
            rc = PcmHw_TimeSlotAlloc( ch->ch, j, ch->slotassigned[j] );
            if ( rc )
            {
               printk( KERN_ERR "%s: [ch=%u] Failed to allocate time slot %i -> slot channel %i rc=%i\n",
                     __FUNCTION__, ch->ch, ch->slotassigned[j], j, rc );
               goto cleanup_dma_channels;
            }
         }
      }
   }

   gPcmRefCount = 0;
   atomic_set( &gPcm.prepared, 1 );
   return 0;

cleanup_dma_channels:
   pcmDisableNoCheck();
   return rc;
}

/***************************************************************************/
/**
*  Enable PCM channel
*
*  @return
*     0     success
*     -1    failed to enable PCM channels
*
*  @remarks
*     This routine cannot block. pcmPrepare must be called first to
*     prepare the channels.
*/
static int pcmEnable( void )
{
   struct pcm_ch_cfg          *ch;
   int                         rc, i;
   HALAUDIO_PCM_PLATFORM_INFO *info = &gPcmPlatformInfo;

   if ( !atomic_read( &gPcm.prepared ))
   {
      return -EPERM;
   }

   if ( atomic_read( &gPcm.running ))
   {
      return -EBUSY; /* Already enabled */
   }

   ch = &gPcm.ch[info->channel_select];
   for ( i = info->channel_select; i < (info->channel_select + info->channels); i++, ch++ )
   {
      rc = PcmHw_InterfaceEnable( ch->ch, 1 );
      if ( rc != 0 )
      {
         printk( KERN_ERR "%s: [ch=%u] failed to enable interface\n",
               __FUNCTION__, ch->ch );
         return rc;
      }

      /* A mask is used in to select slot channels. The assumption is
       * that the active slot channels are adjacent and start at slot channel 0.
       */
      rc = PcmHw_ChEnable( ch->ch, (uint8_t)((2 << ch->slotchansused) - 1),
            pcmHw_DIR_TXRX, 1 );
      if ( rc != 0 )
      {
         printk( KERN_ERR "%s: [ch=%u] failed to enable channels\n",
               __FUNCTION__, ch->ch );
         return rc;
      }
   }

   atomic_set( &gPcm.running, 1 );

   return 0;
}

/***************************************************************************/
/**
*  Helper routine to disable PCM channel, but with no state checks
*
*  @return
*     0     success
*     -1    failed to enable PCM channels
*
*  @remark
*/
static int pcmDisableNoCheck( void )
{
   struct pcm_ch_cfg          *ch;
   int                         rc, i, j;
   int                         err;
   HALAUDIO_PCM_PLATFORM_INFO *info = &gPcmPlatformInfo;

   atomic_set( &gPcm.running, 0 );

   err = 0;
   ch  = &gPcm.ch[info->channel_select];
   for ( i = info->channel_select; i < (info->channel_select + info->channels); i++, ch++ )
   {
      /* Stop egress DMA, PCM block, and then ingress DMA */

      rc = dma_stop_transfer( ch->dma_egr.handle );
      if ( rc != 0 )
      {
         printk( KERN_ERR "%s: [ch=%u] failed to stop egress DMA\n",
               __FUNCTION__, ch->ch );
         err = rc;
      }

      PcmHw_ChEnable( ch->ch, (uint8_t)((2 << ch->slotchansused) - 1),
            pcmHw_DIR_TXRX, 0 );

      PcmHw_InterfaceEnable( ch->ch, 0 );

      rc = dma_stop_transfer( ch->dma_igr.handle );
      if ( rc != 0 )
      {
         printk( KERN_ERR "%s: [ch=%u] failed to stop egress DMA\n",
               __FUNCTION__, ch->ch );
         err = rc;
      }

      dma_set_device_handler( ch->dma_egr.device, NULL, NULL );
      dma_set_device_handler( ch->dma_igr.device, NULL, NULL );

      if ( ch->dma_egr.handle >= 0 )
      {
         dma_free_channel( ch->dma_egr.handle );
         ch->dma_egr.handle = DMA_INVALID_HANDLE;
      }
      if ( ch->dma_igr.handle >= 0 )
      {
         dma_free_channel( ch->dma_igr.handle );
         ch->dma_igr.handle = DMA_INVALID_HANDLE;
      }

      for ( j = 0; j < PCM_MAX_NUM_SLOT_CHANNELS; j++ )
      {
         /* Deallocate time slots */
         if ( ch->slotassigned[j] >= 0 )
         {
            rc = PcmHw_TimeSlotDeAlloc( ch->ch, ch->slotassigned[j] );
            if ( rc )
            {
               printk( KERN_ERR "%s: [ch=%u] Failed to deallocate timeslot %i rc=%i\n",
                     __FUNCTION__, ch->ch, ch->slotassigned[j], rc );
               err = rc;
            }
         }
      }

      /* Flush write buffers */
      halAudioWriteFlush( &ch->write );
   }

   atomic_set( &gPcm.prepared, 0 );

   return err;
}

/***************************************************************************/
/**
*  Disable PCM channel
*
*  @return
*     0     success
*     -1    failed to enable PCM channels
*
*  @remark
*/
static int pcmDisable( void )
{

   if ( !atomic_read( &gPcm.prepared ))
   {
      return 0; /* Nothing to disable, has not been prepared */
   }

   return pcmDisableNoCheck();
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
static int pcmPmShutdown( void )
{
   if ( atomic_read( &gPcm.running ))
   {
      return -EBUSY;
   }

   clk_disable( gApm100_clk );

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
static int pcmPmResume( void )
{
   return clk_enable( gApm100_clk );
}

/***************************************************************************/
/**
*  Select sampling frequency for all channels
*
*  @return
*     0        Success
*     -ENODEV  PCM has not be initiialized
*     -EBUSY   PCM is currently running
*     -EINVAL  Invalid channel number
*
*  @remarks
*     This routine needs the PCM block disabled before it can proceed
*     because it must reset the DMA chains and configure the PCM registers.
*/
static int pcmSetFreq(
   int chno,                        /**< (i) channel number 0-2 */
   int freqHz                       /**< (i) sampling frequency in Hz */
)
{
   struct pcm_ch_cfg *ch;
   int                i, validhz, rc = 0;
   unsigned int       frame_size, frame_period, dma_prime_egr;

   if ( !gPcm.initialized )
   {
      return -ENODEV;
   }

   if ( atomic_read( &gPcm.running ) || atomic_read( &gPcm.prepared ))
   {
      /* Cannot change sampling frequency if currently running or already prepared */
      return -EBUSY;
   }

   /* Validate sampling frequency is supported */
   validhz = 0;
   for ( i = 0; i < ARRAY_SIZE( gPcmFreqs ); i++ )
   {
      if ( freqHz == gPcmFreqs[i] )
      {
         validhz = 1;
         break;
      }
   }
   if ( !validhz )
   {
      return -EINVAL;
   }

   ch = &gPcm.ch[chno];

   /* Calculate frame size and period based on selected sampling frequency.
    */
   frame_size     = (freqHz * PCM_DEFAULT_FRAME_PERIOD * PCM_DEFAULT_SAMP_WIDTH) / 1000000; /* in bytes */
   frame_period   = (frame_size * (1000000/PCM_DEFAULT_SAMP_WIDTH)) / freqHz;               /* in usec */
   dma_prime_egr  = (freqHz * PCM_DEFAULT_PRIME_PERIOD * PCM_DEFAULT_SAMP_WIDTH) / 1000000;

   ch->frame_period     = frame_period;
   ch->samp_freq        = freqHz;
   ch->frame_size       = frame_size;
   ch->dma_frame_size   = frame_size * PCM_MAX_NUM_SLOT_CHANNELS; /* DMA frame size */
   ch->dma_prime_egr    = dma_prime_egr * PCM_MAX_NUM_SLOT_CHANNELS;  /* DMA frame size */

   /* PCM channel frame sync frequency */
   rc = PcmHw_FsFreqSet( ch->ch, freqHz );
   if ( rc )
   {
      printk( KERN_ERR "%s: [ch=%u] Failed to set frame sync frequency rc=%i\n", __FUNCTION__, ch->ch, rc );
   }

   pcmMixerSetFreq( ch->mixer_port, freqHz, frame_size );

   /* Update sine generation state */
   halAudioSineConfig( &ch->sinectl, ch->sinectl.freq, ch->samp_freq );

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
static int pcmGetFreq(
   int   chno,                      /*<< (i) Codec channel number */
   int  *freqhz                     /*<< (o) Ptr to sampling frequency in Hz */
)
{
   struct pcm_ch_cfg *ch;

   ch       = &gPcm.ch[chno];
   *freqhz  = ch->samp_freq;

   return 0;
}

/***************************************************************************/
/**
*  Set equalizer parameters such as filter coefficients, filter length,
*  and other parameters.
*
*  @return
*     0        Success
*     -ve      Error code
*/
static int pcmEquParmSet(
   int                  chno,       /*<< (i) Codec channel number */
   HALAUDIO_DIR         dir,        /*<< (i) Direction path */
   const HALAUDIO_EQU  *equ         /*<< (i) Ptr to equalizer parameters */
)
{
   HALAUDIO_EQU      *saved_equ;
   struct pcm_ch_cfg *ch;

   ch = &gPcm.ch[chno];

   if ( dir == HALAUDIO_DIR_ADC )
   {
      saved_equ   = &ch->equ_igr;
   }
   else
   {
      saved_equ   = &ch->equ_egr;
   }

   /* Save equalizer parameters */
   memcpy( saved_equ, equ, sizeof(*saved_equ) );

   return 0;
}

/***************************************************************************/
/**
*  Query equalizer parameters such as filter coefficients, filter length,
*  and other parameters.
*
*  @return
*     0        Success
*     -ve      Error code
*/
static int pcmEquParmGet(
   int                  chno,       /*<< (i) Codec channel number */
   HALAUDIO_DIR         dir,        /*<< (i) Direction path */
   HALAUDIO_EQU        *equ         /*<< (0) Ptr to equalizer parameters */
)
{
   struct pcm_ch_cfg *ch;
   HALAUDIO_EQU      *saved_equ;

   ch = &gPcm.ch[chno];

   if ( dir == HALAUDIO_DIR_ADC )
   {
      saved_equ   = &ch->equ_igr;
   }
   else
   {
      saved_equ   = &ch->equ_egr;
   }

   memcpy( equ, saved_equ, sizeof(*equ) );
   return 0;
}

/***************************************************************************/
/**
*  Setup DMA memory resources
*
*  @return
*     0        - success
*     -ENOMEM  - failed to allocate memory
*/
static int pcmDmaInit( void )
{
   int                         i;
   struct pcm_ch_cfg          *ch;
   int                         rc = 0;
   HALAUDIO_PCM_PLATFORM_INFO *info = &gPcmPlatformInfo;

   /* Create DMA buffer pool */
   gDmaPool = dma_pool_create( "PCM DMA memory pool", NULL,
         PCM_MAX_DMA_BUFFER_SIZE_BYTES, PCM_DMA_ALIGN_IN_BYTES,
         PCM_DMA_ALLOC_CROSS_RESTRICT );
   if ( gDmaPool == NULL )
   {
      printk( KERN_ERR "%s: failed to allocate DMA buffer pool\n", __FUNCTION__ );
      return -ENOMEM;
   }

   ch = &gPcm.ch[info->channel_select];
   for ( i = info->channel_select; i < (info->channel_select + info->channels); i++, ch++ )
   {
      /* Allocate ingress and egress audio buffers */
      ch->buf_igr[0].virt = dma_pool_alloc( gDmaPool, GFP_KERNEL, &ch->buf_igr[0].phys );
      ch->buf_igr[1].virt = dma_pool_alloc( gDmaPool, GFP_KERNEL, &ch->buf_igr[1].phys );
      ch->buf_egr[0].virt = dma_pool_alloc( gDmaPool, GFP_KERNEL, &ch->buf_egr[0].phys );
      ch->buf_egr[1].virt = dma_pool_alloc( gDmaPool, GFP_KERNEL, &ch->buf_egr[1].phys );

      if ( !ch->buf_igr[0].virt || !ch->buf_igr[1].virt || !ch->buf_egr[0].virt || !ch->buf_egr[1].virt )
      {
         printk( KERN_ERR "%s: [CH %u] failed to allocate DMA audio buffers\n", __FUNCTION__, ch->ch );
         rc = -ENOMEM;
         goto cleanup_exit;
      }

      /* Allocate ingress/egress buffers for de-interleaved data processing */
      ch->igrdatap = vmalloc( PCM_MAX_FRAME_SIZE );
      if ( !ch->igrdatap )
      {
         printk( KERN_ERR "PCM [CH %u]: Failed to allocated ingress buffer rc=%i\n", ch->ch, rc );
         rc = -ENOMEM;
         goto cleanup_exit;
      }
      memset( ch->igrdatap, 0, PCM_MAX_FRAME_SIZE );

      ch->egrdatap = vmalloc( PCM_MAX_FRAME_SIZE );
      if ( !ch->egrdatap )
      {
         printk( KERN_ERR "PCM [CH %u]: Failed to allocated egress buffer rc=%i\n", ch->ch, rc );
         rc = -ENOMEM;
         goto cleanup_exit;
      }
      memset( ch->egrdatap, 0, PCM_MAX_FRAME_SIZE );
   }

   gPcm.zero.virt = dma_alloc_writecombine( NULL, PCM_MAX_PRIME_BUFFER_SIZE_BYTES,
            &gPcm.zero.phys, GFP_KERNEL );
   if ( gPcm.zero.virt == NULL )
   {
      rc = -ENOMEM;
      goto cleanup_exit;
   }
   memset( gPcm.zero.virt, 0, PCM_MAX_PRIME_BUFFER_SIZE_BYTES );

   return rc;

cleanup_exit:
   pcmDmaTerm();
   return rc;
}

/***************************************************************************/
/**
*  Cleanup DMA memory resources
*
*  @return
*/
static int pcmDmaTerm( void )
{
   int                i;
   struct pcm_ch_cfg *ch;

   ch = gPcm.ch;
   for ( i = 0; i < gPcmNumChans; i++, ch++ )
   {
      if ( ch->buf_igr[0].virt )
      {
         dma_pool_free( gDmaPool, ch->buf_igr[0].virt, ch->buf_igr[0].phys );
         ch->buf_igr[0].virt = NULL;
      }
      if ( ch->buf_igr[1].virt )
      {
         dma_pool_free( gDmaPool, ch->buf_igr[1].virt, ch->buf_igr[1].phys );
         ch->buf_igr[1].virt = NULL;
      }
      if ( ch->buf_egr[0].virt )
      {
         dma_pool_free( gDmaPool, ch->buf_egr[0].virt, ch->buf_egr[0].phys );
         ch->buf_egr[0].virt = NULL;
      }
      if ( ch->buf_egr[1].virt )
      {
         dma_pool_free( gDmaPool, ch->buf_egr[1].virt, ch->buf_egr[1].phys );
         ch->buf_egr[1].virt = NULL;
      }
      if ( ch->igrdatap )
      {
         vfree( ch->igrdatap );
         ch->igrdatap = NULL;
      }
      if ( ch->egrdatap )
      {
         vfree( ch->egrdatap );
         ch->egrdatap = NULL;
      }
   }

   dma_pool_destroy( gDmaPool );

   if ( gPcm.zero.virt )
   {
      dma_free_writecombine( NULL, PCM_MAX_PRIME_BUFFER_SIZE_BYTES,
            gPcm.zero.virt, gPcm.zero.phys );
      gPcm.zero.virt = NULL;
   }

   return 0;
}

/***************************************************************************/
/**
*  Helper routine to interleave data prior to DMA
*
*  @return  None
*
*  @remarks
*
*/
static void interleave_data(
   uint16_t         *dmabufp,       /*<< (o) Ptr to store interleaved data */
   const uint16_t   *srcp,          /*<< (i) Ptr to source buffer */
   int               dma_fsize,     /*<< (i) DMA buffer frame size in bytes */
   int               slots_used     /*<< (i) Number of slots to deposit */
)
{
   int i, j;
   int dst_inc, dst_samples;

   /* Perform the interleave. Each slot channel FIFO is 32-bit wide and thus
    * two 16-bit words are packed together for the interleave. If the width
    * of a sample is 8 bits, then 4 bytes are packed together.
    */
   dst_inc     = PCM_MAX_NUM_SLOT_CHANNELS * 2;
   dst_samples = dma_fsize/sizeof(uint16_t);

   for ( i = 0; i < dst_samples; i += dst_inc )
   {
      for ( j = 0; j < slots_used; j++ )
      {
         dmabufp[2*j]   = *srcp++;
         dmabufp[2*j+1] = *srcp++;
      }
      dmabufp += dst_inc;
   }
}

/***************************************************************************/
/**
*  Helper routine to extract slot channel data from an interleaved PCM DMA
*  buffer.
*
*  @return  None
*
*  @remarks
*/
static void extract_data(
   uint16_t         *dstp,          /*<< (o) Ptr to store extracted data */
   const uint16_t   *dmabufp,       /*<< (i) Ptr to interleaved DMA buffer */
   int               dst_fsize,     /*<< (i) Dst buffer frame size in bytes */
   int               slots_used     /*<< (i) Number of slots to extract */
)
{
   int i, j;
   int dst_inc, dst_samples;

   /* De-interleave the data. Each slot channel DMA FIFO is 32-bits wide.
    * Thus for 16-bit samples, pairs of samples are interleaved between
    * slot channels.
    */
   dst_inc     = 2 * slots_used;
   dst_samples = dst_fsize/sizeof(uint16_t);

   for ( i = 0; i < dst_samples; i += dst_inc )
   {
      for ( j = 0; j < slots_used; j++ )
      {
         *dstp++ = dmabufp[2*j];
         *dstp++ = dmabufp[2*j+1];
      }
      dmabufp += (PCM_MAX_NUM_SLOT_CHANNELS * 2);
   }
}

/***************************************************************************/
/**
*  PCM ingress DMA handler that services all ingress DMA channels. This
*  handler is controlled by the master ingress channel.
*
*  @return  none
*
*  @remarks
*/
static void pcmDmaIngressHandler(
   DMA_Device_t   dev,           /**< (i) Device that triggered callback */
   int            reason,        /**< (i) Reason for interrupt */
   void          *data           /**< (i) User data pointer */
)
{
   struct pcm_ch_cfg          *ch;
   uint16_t                   *rawdatap;  /* Raw pointer to DMA data */
   int                         frame_size;
   HALAUDIO_PCM_PLATFORM_INFO *info = &gPcmPlatformInfo;

   (void) dev;    /* unused */

   ch = data;
   PCMLOG( "%u: ch=%i", ch->isrcount_igr, ch->ch );

   if ( reason != DMA_HANDLER_REASON_BLOCK_COMPLETE )
   {
      ch->errs.dma_igr++;
      return;
   }

   ch->isrcount_igr++;

   /* Point to buffer index with actual samples */
   ch->active_idx = ch->active_idx ^ 1;

   rawdatap    = ch->buf_igr[ch->active_idx].virt;
   frame_size  = ch->frame_size;

   /* Extract data from raw DMA buffer */
   extract_data( ch->igrdatap, rawdatap, frame_size, ch->slotchansused );

   /* put through software equalizer */
   if ( ch->equ_igr.len )
   {
      halAudioEquProcess( ch->igrdatap, ch->equ_igr.coeffs, halAdcFiltHist, 
            ch->equ_igr.len, ch->frame_size/2 /* 16-bit samples */ );
   }

   if ( ch->ramp_check )
   {
      /* Verify against data from 2 frames ago */
      halAudioCompareData( rawdatap, ch->buf_egr[ch->active_idx_eg].virt,
            ch->dma_frame_size/sizeof(uint16_t), &ch->ramp_check_errs, &ch->ramp_check_delta );
   }
   if ( ch->ramp_igr )
   {
      halAudioGenerateRamp( ch->igrdatap, &ch->rampseed_igr, frame_size/sizeof(uint16_t), ch->interchs );
   }

   if ( ch->csx_data[HALAUDIO_CSX_POINT_ADC].csx_ops.csxCallback )
   {
      ch->csx_data[HALAUDIO_CSX_POINT_ADC].csx_ops.csxCallback( ch->igrdatap, frame_size, ch->csx_data[HALAUDIO_CSX_POINT_ADC].priv );
   }

   /* Trigger callback when all ingress processing have completed.
    *
    * Simple reference counter used. If PCM channels are both slaved to external
    * devices, there is potential for miscounting if the external clocks
    * glitches or drifts.
    */
   gPcmRefCount++;
   if ( gPcmRefCount >= info->channels )
   {
      if ( gPcmElapsedCb )
      {
         (*gPcmElapsedCb)( gPcmUserData );
      }
      gPcmRefCount = 0;
   }

   /*PCMLOG( "end" ); */
}

/***************************************************************************/
/**
*  PCM egress DMA interrupt handler
*
*  @return  none
*/
static void pcmDmaEgressHandler(
   DMA_Device_t   dev,           /**< (i) Device that triggered callback */
   int            reason,        /**< (i) Reason for interrupt */
   void          *data           /**< (i) User data pointer */
)
{
   struct pcm_ch_cfg *ch;
   int frame_size;

   ch = data;
   PCMLOG( "dev=%u reason=%i isr=%u", dev, reason, ch->isrcount_egr );
   ch->isrcount_egr++;

   if ( reason != DMA_HANDLER_REASON_BLOCK_COMPLETE )
   {
      ch->errs.dma_egr++;
      return;
   }

   /* Point to buffer index with actual samples */
   ch->active_idx_eg = ch->active_idx_eg ^ 1;

   frame_size  = ch->frame_size;

   /* put through software equalizer */
   if ( ch->equ_egr.len )
   {
      halAudioEquProcess( ch->egrdatap, ch->equ_egr.coeffs, halDacFiltHist, 
            ch->equ_egr.len, ch->frame_size/2 /* 16-bit samples */ );
   }

   /* Write request */
   halAudioWriteService( &ch->write, ch->egrdatap, frame_size );

   if ( ch->ramp_egr )
   {
      halAudioGenerateRamp( ch->egrdatap, &ch->rampseed_egr, frame_size/sizeof(uint16_t), ch->interchs );
   }
   else if ( ch->sinectl.freq )
   {
      halAudioSine( ch->egrdatap, &ch->sinectl, frame_size/sizeof(uint16_t), ch->interchs );
   }
   else if ( ch->loop_ig2eg )
   {
      /* software loopback ingress to egress */
      memcpy( ch->egrdatap, ch->igrdatap, frame_size );
   }

   if ( ch->csx_data[HALAUDIO_CSX_POINT_DAC].csx_ops.csxCallback )
   {
      ch->csx_data[HALAUDIO_CSX_POINT_DAC].csx_ops.csxCallback( (char *)ch->egrdatap, frame_size, ch->csx_data[HALAUDIO_CSX_POINT_DAC].priv );
   }

   /* Interleave data for DMA */
   interleave_data( ch->buf_egr[ch->active_idx_eg].virt, ch->egrdatap, ch->dma_frame_size, ch->slotchansused );

   /* PCMLOG( "end dev=%u", dev ); */
}

/***************************************************************************/
/**
*  Write method used to directly write samples to a channel's egress buffers.
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remarks
*/
static int pcmWrite(
   int                     chno,       /**< (i) channel index */
   int                     bytes,      /**< (i) Number of bytes to write */
   const char             *audiobuf,   /**< (i) Pointer to audio samples */
   HALAUDIO_CODEC_IORW_CB  usercb,     /**< (i) User callback to request for more data */
   void                   *userdata    /**< (i) User data */
)
{
   struct pcm_ch_cfg *ch = &gPcm.ch[chno];
   return halAudioWriteRequest( &ch->write, bytes, audiobuf, usercb, userdata );
}

/***************************************************************************/
/**
*  Cleanup PCM write memory resources
*
*  @return
*/
static int pcmWriteTerm( void )
{
   struct pcm_ch_cfg *ch;
   int                i;

   ch = gPcm.ch;
   for ( i = 0; i < gPcmNumChans; i++, ch++ )
   {
      halAudioWriteFree( &ch->write );
   }

   return 0;
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
      struct pcm_ch_cfg *ch;

      ch = table->extra1;
      PcmHw_LoopbackEnable( ch->ch, ch->loop_eg2ig );
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
   struct pcm_ch_cfg *ch;
   int                rc;

   /* Process integer operation */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
   rc = proc_dointvec( table, write, buffer, lenp, ppos );
#else
   rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );
#endif

   if ( write )
   {
      ch = table->extra1;
      halAudioSineConfig( &ch->sinectl, ch->sinectl.freq, ch->samp_freq );
   }
   return rc;
}

/***************************************************************************/
/**
*  Proc files initialization
*
*  @return  Nothing
*/
static void pcmProcInit( void )
{
   HALAUDIO_PCM_PLATFORM_INFO *info = &gPcmPlatformInfo;

   /* procfs directory and files */
   gProcDir = proc_mkdir( "pcm", NULL );
   if ( gProcDir == NULL )
   {
      printk( KERN_ERR "%s: Failed to create directory for proc entries\n", __FUNCTION__ );
      return;
   }

   if ( info->channels == 1 )
   {
      char              name[10];
      int               i, length;
      struct ctl_table *tbp;
      char              refc;

      sprintf( name, "pcm%i", info->channel_select );
      create_proc_read_entry( name, 0, gProcDir, pcmReadProc, &gPcm.ch[info->channel_select] );

      /* Prune sysctl table of unused channel */
      tbp   = gSysCtlPCM;
      refc  = '0' + info->channel_select;
      i     = 0;
      while ( tbp->procname )
      {
         length = strlen( tbp->procname );
         if ( tbp->procname[length-1] != refc )
         {
            /* Squish out the entry */
            memmove( tbp, tbp+1, (ARRAY_SIZE(gSysCtlPCM) - i - 1) * sizeof(gSysCtlPCM[0]) );
         }
         else
         {
            tbp++;
            i++;
         }
      }
   }
   else
   {
      /* Both channels supported */
      create_proc_read_entry( "pcm0", 0, gProcDir, pcmReadProc, &gPcm.ch[0] );
      create_proc_read_entry( "pcm1", 0, gProcDir, pcmReadProc, &gPcm.ch[1] );
   }

   gSysCtlHeader = register_sysctl_table( gSysCtl );
}

/***************************************************************************/
/**
*  Proc files termination
*
*  @return  Nothing
*/
static void pcmProcTerm( void )
{
   HALAUDIO_PCM_PLATFORM_INFO *info = &gPcmPlatformInfo;

   if ( info->channels == 1 )
   {
      char name[10];
      sprintf( name, "pcm%i", info->channel_select );
      remove_proc_entry( name, gProcDir );
   }
   else
   {
      remove_proc_entry( "pcm0", gProcDir );
      remove_proc_entry( "pcm0", gProcDir );
   }
   remove_proc_entry( "pcm", NULL );

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
static int pcmReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   int                  len = 0;
   int8_t               igbuf[PCM_MAX_DMA_BUFFER_SIZE_BYTES];
   int8_t               egbuf[PCM_MAX_DMA_BUFFER_SIZE_BYTES];
   struct pcm_ch_cfg   *ch;
   unsigned long        flags;

   (void) start; (void) offset; (void) count;      /* avoid compiler warning */

   ch = data;
   memset( igbuf, 0, sizeof(igbuf) );
   memset( egbuf, 0, sizeof(egbuf) );

   /* Cache data in critical section */
   local_irq_save( flags );
   memcpy( igbuf, ch->igrdatap, ch->frame_size );
   memcpy( egbuf, ch->egrdatap, ch->frame_size );
   local_irq_restore( flags );

   len += sprintf( buf+len, "      PCM CH%i @ %i Hz, %i bytes, %i us\n",
         ch->ch, ch->samp_freq, ch->frame_size, ch->frame_period );
   len += sprintf( buf+len, "Igr:\n" );
   halAudioPrintMemory( buf, &len, igbuf, ch->frame_size/sizeof(uint16_t), 1 /* HEX_FORMAT */, 10 /* line length */,
         2 /* word width */, 0 /* print_addr */, 0 /* addr */ );

   len += sprintf( buf+len, "Egr:\n" );
   halAudioPrintMemory( buf, &len, egbuf, ch->frame_size/sizeof(uint16_t), 1 /* HEX_FORMAT */, 10 /* line length */,
         2 /* word width */, 0 /* print_addr */, 0 /* addr */ );

   /* Error report and other information */
   len += sprintf( buf+len, "Irqs:        igress=%u egress=%u\n", ch->isrcount_igr, ch->isrcount_egr );
   len += sprintf( buf+len, "DMA errors:  igress=%i egress=%i sync=%i\n", ch->errs.dma_igr, ch->errs.dma_egr, ch->errs.dma_sync );

   if ( ch->ramp_check )
   {
      len += sprintf( buf+len, "Ramp check:  delay=%i errors=%i\n", ch->ramp_check_delta, ch->ramp_check_errs );
   }

   /* Channel parameters */
   len += sprintf( buf+len, "DMA:         prime_bytes=%i frame_bytes=%i\n", ch->dma_prime_egr, ch->dma_frame_size );

   len += sprintf( buf+len, "Ingress DMA: device=0x%x fifo=0x%x handle=0x%x\n",
         ch->dma_igr.device, ch->dma_igr.fifo_addr, ch->dma_igr.handle );
   len += sprintf( buf+len, "Egress DMA:  device=0x%x fifo=0x%x handle=0x%x\n",
         ch->dma_egr.device, ch->dma_egr.fifo_addr, ch->dma_egr.handle );
   len += sprintf( buf+len, "Buffer Igr:  [0] 0x%.8x (0x%.8x phy)\n"
                            "             [1] 0x%.8x (0x%.8x phy)\n",
         (unsigned int)ch->buf_igr[0].virt, ch->buf_igr[0].phys, (unsigned int)ch->buf_igr[1].virt, ch->buf_igr[1].phys );
   len += sprintf( buf+len, "Buffer Egr:  [0] 0x%.8x (0x%.8x phy)\n"
                            "             [1] 0x%.8x (0x%.8x phy)\n",
         (unsigned int)ch->buf_egr[0].virt, ch->buf_egr[0].phys, (unsigned int)ch->buf_egr[1].virt, ch->buf_egr[1].phys );
   len += sprintf( buf+len, "Debug:       running=%i prepared=%i\n", atomic_read( &gPcm.running ), atomic_read( &gPcm.prepared ));

   *eof = 1;
   return len+1;
}

/***************************************************************************/
/**
*  Set frequency for mixer port
*
*  @return  None
*/
static int pcmMixerSetFreq(
   AMXR_PORT_ID   portid,     /**< (i) mixer port handle */
   int            freq,       /**< (i) sampling frequency to set */
   int            bytes       /**< (i) period size in bytes */
)
{
   return amxrSetPortFreq( portid, freq, bytes, freq, bytes );
}

/***************************************************************************/
/**
*  PCM mixer callback for outgoing data (i.e. ingress)
*
*  @return
*     NULL     - non-matching frame size or non-existent buffer
*     ptr      - pointer to ingress buffer
*/
static int16_t *pcmMixerCb_IngressGetPtr(
   int   numBytes,            /**< (i) frame size in bytes */
   void *privdata             /**< (i) User data           */
)
{
   struct pcm_ch_cfg *ch;
   int16_t           *ptr;

   /* Always pass valid pointer to ingress buffer */
   ch  = (struct pcm_ch_cfg *)privdata;
   ptr = ch->igrdatap;

   PCMLOG( "ch=%i numBytes=%i ptr=0x%x", ch->ch, numBytes, ptr );

   return ptr;
}

/***************************************************************************/
/**
*  PCM mixer callback for incoming data (i.e. egress)
*
*  @return
*     NULL     - non-matching frame size or non-existent buffer
*     ptr      - pointer to egress buffer
*/
static int16_t *pcmMixerCb_EgressGetPtr(
   int   numBytes,            /**< (i) frame size in bytes */
   void *privdata             /**< (i) mixer port index  */
)
{
   struct pcm_ch_cfg *ch;
   int16_t           *ptr;

   ch     = (struct pcm_ch_cfg *)privdata;
   ptr    = NULL;

   if ( numBytes == ch->frame_size )
   {
      ptr = ch->egrdatap;
   }

   PCMLOG( "ch=%i numBytes=%i ptr=0x%x", ch->ch, numBytes, ptr );

   return ptr;
}

/***************************************************************************/
/**
*  PCM mixer callback to flush the egress buffers when the last destination
*  connection is removed.
*
*  @return     None
*/
static void pcmMixerCb_EgressFlush(
   void *privdata             /*<< (i) private data */
)
{
   struct pcm_ch_cfg *ch;

   ch = (struct pcm_ch_cfg *)privdata;

   PCMLOG( "PCM [ch %i]", ch->ch );

   memset( ch->egrdatap, 0, PCM_MAX_FRAME_SIZE );
}

/***************************************************************************/
/**
*  PCM mixer port registration
*
*  @return  None
*/
static int pcmMixerPortsRegister( void )
{
   struct pcm_ch_cfg          *ch;
   int                         i, err;
   AMXR_PORT_CB                cb;
   AMXR_PORT_ID                portid;
   char                        name[32];
   HALAUDIO_PCM_PLATFORM_INFO *info = &gPcmPlatformInfo;

   ch = &gPcm.ch[info->channel_select];
   for ( i = info->channel_select; i < (info->channel_select + info->channels); i++, ch++ )
   {
      memset( &cb, 0, sizeof(cb) );
      cb.getsrc         = pcmMixerCb_IngressGetPtr;
      cb.getdst         = pcmMixerCb_EgressGetPtr;
      cb.dstcnxsremoved = pcmMixerCb_EgressFlush;

      sprintf( name, "halaudio.pcm%i", ch->ch );

      err = amxrCreatePort( name, &cb, ch /* privdata */,
            PCM_DEFAULT_SAMP_FREQ, ch->interchs, PCM_DEFAULT_FRAME_SIZE,
            PCM_DEFAULT_SAMP_FREQ, ch->interchs, PCM_DEFAULT_FRAME_SIZE,
            &portid );
      if ( err )
      {
         return err;
      }

      ch->mixer_port = portid;
   }

   return 0;
}

/***************************************************************************/
/**
*  PCM mixer port de-registration
*
*  @return  None
*/
static int pcmMixerPortsDeregister( void )
{
   struct pcm_ch_cfg          *ch;
   int                         i, err, rc;
   HALAUDIO_PCM_PLATFORM_INFO *info = &gPcmPlatformInfo;

   rc = 0;
   ch = &gPcm.ch[info->channel_select];
   for ( i = info->channel_select; i < (info->channel_select + info->channels); i++, ch++ )
   {
      err = amxrRemovePort( ch->mixer_port );
      if ( err )
      {
         printk( KERN_ERR "%s: PCM [ch=%d]: failed to deregister mixer port\n", __FUNCTION__, ch->ch );
         rc = err;
      }
      ch->mixer_port = NULL;  /* invalidate handle */
   }

   return rc;
}

/***************************************************************************/
/**
*  BT module reset helper routine
*/
static void reset_bt( HALAUDIO_PCM_BT_GPIO *gpio, int reset )
{
   if ( reset )
   {
      /* Force reset of BT module */
      gpio_set_value( gpio->rst_b, 0 );
      set_current_state( TASK_INTERRUPTIBLE );
      schedule_timeout( HZ * 100 / 1000 );

      if ( gpio->vreg_ctl >= 0 )
      {
         gpio_set_value( gpio->vreg_ctl, 0 );
         set_current_state( TASK_INTERRUPTIBLE );
         schedule_timeout( HZ * 100 / 1000 );
      }
   }
   else
   {
      /* Take BT module out of reset */
      if ( gpio->vreg_ctl >= 0 )
      {
         gpio_set_value( gpio->vreg_ctl, 1 );
         set_current_state( TASK_INTERRUPTIBLE );
         schedule_timeout( HZ * 100 / 1000 );
      }

      gpio_set_value( gpio->wake, 1 );
      set_current_state( TASK_INTERRUPTIBLE );
      schedule_timeout( HZ * 100 / 1000 );

      gpio_set_value( gpio->rst_b, 1 );
   }
}

/***************************************************************************/
/**
*  Platform support constructor
*/
static int pcm_platform_init( HALAUDIO_PCM_PLATFORM_INFO *info )
{
   gpiomux_rc_e   gpiorc;
   int            err;

   /* Initialize GPIOs for PCM Channel 0 */
   if (( info->channels == 2 )
   ||  ( info->channels == 1 && info->channel_select == 0 ))
   {
      gpiorc = gpiomux_requestGroup( gpiomux_group_pcm0, "PCM0" );
      if ( gpiorc != gpiomux_rc_SUCCESS )
      {
         printk( KERN_ERR "%s: failed to request PCM0 gpio group rc=%u\n", __FUNCTION__, gpiorc );
         err = -EBUSY;
         goto cleanup_and_exit;
      }

      if ( info->bt_gpio.rst_b >= 0 )
      {
         /* Following GPIO code required for BT UART communication */
         gpiorc = gpiomux_requestGroup( gpiomux_group_uart1_fc, "BTUART" );
         if ( gpiorc != gpiomux_rc_SUCCESS )
         {
            printk( KERN_ERR "%s: failed to request BT gpio group rc=%u\n", __FUNCTION__, gpiorc );
            err = -EBUSY;
            goto cleanup_and_exit;
         }

         /* Request pin to enable power to BT module */
         err = gpio_request( info->bt_gpio.rst_b, "BT_RST_B" );
         if ( err )
         {
            printk( KERN_ERR "%s: failed to request BT_RST_B gpio pin rc=%u\n", __FUNCTION__, err );
            err = -EBUSY;
            goto cleanup_and_exit;
         }

         /* Request pin for BT VREG CTL if required */
         if ( info->bt_gpio.vreg_ctl >= 0 )
         {
            err = gpio_request( info->bt_gpio.vreg_ctl, "BT_VREG_CTL" );
            if ( gpiorc != gpiomux_rc_SUCCESS )
            {
               printk( KERN_ERR "%s: failed to request BT_VREG_CTL gpio pin rc=%u\n", __FUNCTION__, gpiorc );
               err = -EBUSY;
               goto cleanup_and_exit;
            }
         }

         err = gpio_request( info->bt_gpio.wake, "BT_WAKE" );
         if ( gpiorc != gpiomux_rc_SUCCESS )
         {
            printk( KERN_ERR "%s: failed to request BT_WAKE gpio pin rc=%u\n", __FUNCTION__, gpiorc );
            err = -EBUSY;
            goto cleanup_and_exit;
         }

         gpio_direction_output( info->bt_gpio.wake, 0 );
         gpio_direction_output( info->bt_gpio.rst_b, 0 );

         if ( info->bt_gpio.vreg_ctl >= 0 )
         {
            gpio_direction_output( info->bt_gpio.vreg_ctl, 0 );
         }

         /* Force reset of BT module */
         reset_bt( &info->bt_gpio, 1 );

         /* Take BT module out of reset */
         reset_bt( &info->bt_gpio, 0 );
      }
   }

   /* Initialize GPIOs for PCM Channel 1 */
   if (( info->channels == 2 )
   ||  ( info->channels == 1 && info->channel_select == 1 ))
   {
      gpiorc = gpiomux_requestGroup( gpiomux_group_pcm1, "PCM1" );
      if ( gpiorc != gpiomux_rc_SUCCESS )
      {
         printk( KERN_ERR "%s: failed to request PCM1 gpio group rc=%u\n", __FUNCTION__, gpiorc );
         err = -EBUSY;
         goto cleanup_and_exit;
      }
   }

   return 0;

cleanup_and_exit:
   pcm_platform_exit( info );
   return err;
}

/***************************************************************************/
/**
*  Platform support destructor
*/
static void pcm_platform_exit( HALAUDIO_PCM_PLATFORM_INFO *info )
{
   gpiomux_rc_e   gpiorc;

   if (( info->channels == 2 )
   ||  ( info->channels == 1 && info->channel_select == 0 ))
   {
      gpiorc = gpiomux_freeGroup( gpiomux_group_pcm0 );
      if ( gpiorc != gpiomux_rc_SUCCESS )
      {
         printk( KERN_ERR "%s: failed to free PCM0 GPIO MUX group rc=%i\n", __FUNCTION__, gpiorc );
      }

      if ( info->bt_gpio.rst_b >= 0 )
      {
         gpiorc = gpiomux_freeGroup( gpiomux_group_uart1_fc );
         if ( gpiorc != gpiomux_rc_SUCCESS )
         {
            printk( KERN_ERR "%s: failed to free BTUART GPIO MUX group rc=%i\n", __FUNCTION__, gpiorc );
         }

         reset_bt( &info->bt_gpio, 1 );

         gpio_free( info->bt_gpio.rst_b );
         if ( info->bt_gpio.vreg_ctl >= 0 )
         {
            gpio_free( info->bt_gpio.vreg_ctl );
         }
         gpio_free( info->bt_gpio.wake );
      }
   }

   if (( info->channels == 2 )
   ||  ( info->channels == 1 && info->channel_select == 1 ))
   {
      gpiorc = gpiomux_freeGroup( gpiomux_group_pcm1 );
      if ( gpiorc != gpiomux_rc_SUCCESS )
      {
         printk( KERN_ERR "%s: failed to free PCM1 GPIO MUX group rc=%i\n", __FUNCTION__, gpiorc );
      }
   }
}

static int __init pcm_probe( struct platform_device *pdev )
{
   int err;
   HALAUDIO_PCM_PLATFORM_INFO *info;

   printk( KERN_INFO "HAL Audio PCM Driver: 1.0. Built %s %s\n", __DATE__, __TIME__ );

   gPcmNumChans = CAP_PCM_MAX_NUM_HW_CHANNELS;

   /* Grab platform configuration */
   if ( pdev->dev.platform_data == NULL )
   {
      printk( KERN_ERR "%s: missing platform_data\n",  __FUNCTION__ );
      return -ENODEV;
   }
   memcpy( &gPcmPlatformInfo, pdev->dev.platform_data, sizeof( gPcmPlatformInfo ));
   info = &gPcmPlatformInfo;

   printk( KERN_INFO "channels=%d channel_select=%d\n", info->channels, info->channel_select );
   printk( KERN_INFO "BT GPIOs: RST_B=%d VREG_CTL=%d WAKE=%d\n", info->bt_gpio.rst_b, info->bt_gpio.vreg_ctl, info->bt_gpio.wake );

   /* Validate device parameters */
   if ( info->channel_select >= gPcmNumChans
   ||   info->channels > gPcmNumChans 
   ||   info->channels == 0 )
   {
      printk( KERN_ERR "%s: invalid parameters - channel_select=%i channels=%i\n",
            __FUNCTION__, info->channel_select, info->channels );
      return -EINVAL;
   }

   if ( info->channels == gPcmNumChans )
   {
      info->channel_select = 0;
   }

   err = pcm_platform_init( info );
   if ( err )
   {
      printk( KERN_ERR "%s: failed to initialize the PCM platform %d!\n", __FUNCTION__, err );
      return err;
   }

   err = halAudioAddInterface( &halaudio_pcm_ops, info->channels, "PCM",
         PCM_DEFAULT_FRAME_PERIOD, 1 /* synchronize */, &gInterfHandle );
   if ( err )
   {
      printk( KERN_ERR "%s: failed to install the audio interface %d!\n", __FUNCTION__, err );
      goto err_platform_exit;
   }

   return 0;

err_platform_exit:
   pcm_platform_exit( info );

   return err;
}

static int __exit pcm_remove( struct platform_device *pdev )
{
   HALAUDIO_PCM_PLATFORM_INFO *info = &gPcmPlatformInfo;

   halAudioDelInterface( gInterfHandle );
   pcm_platform_exit( info );

   return 0;
}

static struct platform_driver pcm_driver =
{
   .driver =
   {
      .name = "bcm-halaudio-pcm",
      .owner = THIS_MODULE,
   },
   .probe = pcm_probe,
   .remove = pcm_remove,
};

/***************************************************************************/
/**
*  Driver initialization called when module loaded by kernel
*
*  @return
*     0              Success
*     -ve            Error code
*/
static int __init pcm_init( void )
{
   return platform_driver_register( &pcm_driver );
}

/***************************************************************************/
/**
*  Driver destructor routine.
*/
static void __exit pcm_exit( void )
{
   platform_driver_unregister( &pcm_driver );
}

module_init( pcm_init );
module_exit( pcm_exit );

MODULE_AUTHOR( "Broadcom" );
MODULE_DESCRIPTION( "HAL Audio BCMRING PCM Driver" );
MODULE_LICENSE( "GPL" );
