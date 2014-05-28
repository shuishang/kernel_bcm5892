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
*  @file    apm_drv.c
*
*  @brief   APM driver implementation for BCMRING chip family
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>                   /* For /proc/audio */
#include <linux/sysctl.h>
#include <linux/device.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>                 /* For request_irq */
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/clk.h>

#include <linux/broadcom/knllog.h>           /* For debugging */
#include <linux/broadcom/halaudio.h>
#include <linux/broadcom/halaudio_lib.h>
#include <linux/broadcom/amxr.h>

#include <mach/irqs.h>                   /* For request_irq */
#include <mach/csp/apmHw_reg.h>
#include <mach/csp/chipcHw_inline.h>
#include <csp/apmHw.h>
#include <mach/csp/cap.h>

#include "apm_drv.h"
#include "dma_priv.h"

/**
 * @addtogroup HALAUDIO
 * @{
 */

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
#define APM_MAX_NUM_CHANS                 3     /* Maximum 3 APM channels */
#define APM_MAX_EGR_CHANS                 2     /* Supports 2 egress APM channels */
#define APM_LAST_IGR_CHAN_IDX             (APM_MAX_NUM_CHANS-1)

/* Hardware definitions (should be apmHw_reg.h) */
#define APM_HW_FIFO_SIZE_BYTES            128   /* Holds 64 16-bit samples */

/* DMA constants */
#define APM_MAX_DMA_BUFFER_SIZE_BYTES     480   /* 5ms 48kHz mono frames */
#define APM_DMA_ALIGN_IN_BYTES            8     /* 64-bit alignment */
#define APM_DMA_ALLOC_CROSS_RESTRICT      1024  /* Buffers cannot cross 1kb boundary */

/* Default sampling frequency configurations */
#define APM_DEFAULT_SAMP_FREQ             16000 /* In Hz */
#define APM_DEFAULT_FRAME_PERIOD          5000  /* In usec */
#define APM_DEFAULT_FRAME_SIZE            160   /* In bytes */
#define APM_SAMP_WIDTH                    2     /* Each sample in bytes */

/* Priming */
#define APM_MAX_PRIME_SIZE_BYTES          (APM_MAX_DMA_BUFFER_SIZE_BYTES * 2)
#define APM_DEFAULT_PRIME_PERIOD          5000  /* In usec, cannot be more than APM_DEFAULT_FRAME_PERIOD */
#define APM_DEFAULT_PRIME_SIZE_BYTES      APM_DEFAULT_FRAME_SIZE
#define APM_MAX_PRIME_PERIOD              APM_DEFAULT_FRAME_PERIOD   /* Do not change */
#define APM_DEFAULT_IGEG_LAG_PERIOD       1500  /* In usec, needed if priming exceeds fifo size */

#if (APM_DEFAULT_PRIME_PERIOD > APM_MAX_PRIME_PERIOD)
#error Default prime period cannot exceed APM_MAX_PRIME_PERIOD
#endif

/* Physical addresses of APM channel fifos */
#define APM_CHAN0_PTM_FIFO_PHYSADDR       (MM_ADDR_IO_APM + apmHw_CODEC0_DEV_TO_MEM_FIFO_OFFSET)
#define APM_CHAN0_MTP_FIFO_PHYSADDR       (MM_ADDR_IO_APM + apmHw_CODEC0_MEM_TO_DEV_FIFO_OFFSET)
#define APM_CHAN1_PTM_FIFO_PHYSADDR       (MM_ADDR_IO_APM + apmHw_CODEC1_DEV_TO_MEM_FIFO_OFFSET)
#define APM_CHAN1_MTP_FIFO_PHYSADDR       (MM_ADDR_IO_APM + apmHw_CODEC1_MEM_TO_DEV_FIFO_OFFSET)
#define APM_CHAN2_PTM_FIFO_PHYSADDR       (MM_ADDR_IO_APM + apmHw_CODEC2_DEV_TO_MEM_FIFO_OFFSET)

/* Error checking interrupts */
#define APM_INT_MASKS_ERRORS              ( APM_INT_MASK_DAC_UNDERFLOW_A | APM_INT_MASK_DAC_UNDERFLOW_B | \
                                            APM_INT_MASK_ADC_OVERFLOW_A  | APM_INT_MASK_ADC_OVERFLOW_B  | \
                                            APM_INT_MASK_ADC_OVERFLOW_C )

#define APM_DEFAULT_DAC_POWERUP_RAMPTIME         40  /* In ms */
#define APM_DEFAULT_DAC_POWERDN_RAMPTIME         40  /* In ms */
#define APM_DEFAULT_DAC_POWERDN_BLOCKING_DELAY   (APM_DEFAULT_DAC_POWERDN_RAMPTIME+60) /* In ms */

/* Synchronization check. If the sampling frequency is a multiple of 8
 * then it can be synchronized with HAL Audio
 */
#define APM_SYNC_FREQ( freq )             (((freq) % 8 ) == 0)

/* CSX Data stucture */
struct apm_csx_data
{
   CSX_IO_POINT_FNCS    csx_ops;
   void                *priv;
};

/* APM fifo error statistics */
struct apm_ch_errs
{
   unsigned int         dma_egr;          /* DMA egress errors */
   unsigned int         dma_igr;          /* DMA ingress erros */
   unsigned int         dma_sync;         /* DMA egress sync errors */
   unsigned int         dac_underflow;    /* DAC fifo underflow errors */
   unsigned int         adc_overflow;     /* ADC fifo overflow errors */
};

/* APM gain state */
struct apm_gain
{
   int                  dig_gain;         /* Digital gain in dB */
   int                  ana_gain[APM_MAX_MUX_POSITIONS]; /* Analog gain in dB per mux position */
};

/* APM sidetone state */
struct apm_sidetone
{
   int                  db;               /* Gain in dB */
   int                  enable;           /* Enable flag */
};

/* APM channel configuration and parameters */
struct apm_ch_cfg
{
   /* Channel configuration */
   unsigned int         ch;               /* Channel number */
   int                  no_egr;           /* Flag to indicate the channel has only ingress support */
   struct dma_cfg       dma_igr;          /* Ingress (Peripheral to memory) DMA config */
   struct dma_cfg       dma_egr;          /* Egress  (Memory to peripheral) DMA config */
   struct dma_data_buf  buf_igr[2];       /* Ingress (tx) double buffer */
   struct dma_data_buf  buf_egr[2];       /* Egress (rx) double buffer */
   unsigned int         samp_freq;        /* Sampling frequency in Hz */
   unsigned int         frame_period;     /* Frame period in usec. Debug only  */
   unsigned int         frame_size;       /* Frame size in bytes. Used in ISR */
   unsigned int         prime_egr;        /* Egress priming in bytes */

   /* Hardware block states */
   struct apm_gain      mic;              /* Active microphone gain settings */
   struct apm_gain      spk;              /* Active speaker gain settings */
   struct apm_sidetone  sidetone;         /* Active sidetone settings */
   HALAUDIO_EQU         equ_igr;          /* Ingress equalizer parameters */
   HALAUDIO_EQU         equ_egr;          /* Egress equalizer parameters */

   /* Write state */
   HALAUDIO_WRITE       write;            /* Write state */

   /* Read state */

   /* ISR status */
   atomic_t             queued_pkts_egr;  /* Num of egress packets awaiting to be DMA'd. Should not exceed 2. When 0, means DMA is idle */
   int                  active_idx;       /* Index to active buffer in ingress and egress double buffers */
   struct apm_ch_errs   errs;             /* Channel errors */

   /* Debug facilities */
   int                  loop_ig2eg;       /* Sysctl: Ingress to egress loopback */
   int                  loop_eg2ig;       /* Sysctl: Egress to ingress loopback */
   int                  loophw_ig2eg;     /* Sysctl: Ingress to egress loopback in HW */
   int                  loophw_eg2ig;     /* Sysctl: Ingress to egress loopback in HW */
   int                  ramp_igr;         /* Sysctl: Ingress ramp generation */
   int                  ramp_egr;         /* Sysctl: Egress ramp generation */
   int                  ramp_check;       /* Sysctl: Validate ramp in hw eg2ig loopback */
   int                  ramp_check_delay; /* Sysctl: Delay between igr and egr ramps */
   int                  ramp_check_errs;  /* Sysctl: Number of glitches in received ramp */
   unsigned short       rampseed_igr;     /* Ingress ramp seed */
   unsigned short       rampseed_egr;     /* Egress ramp seed */
   unsigned int         isrcount_igr;     /* Ingress ISR counter */
   unsigned int         isrcount_egr;     /* Egress ISR counter */
   HALAUDIO_SINECTL     sinectl;

   /* Mixer facilities */
   AMXR_PORT_ID         mixer_port;       /* Mixer port handle for channel */

   /* CSX data */
   struct apm_csx_data  csx_data[HALAUDIO_NUM_CSX_POINTS]; /* Array of CSX data structures */
};

/* APM information structure */
struct apm_info
{
   int                  initialized;
   atomic_t             running;          /* Flag indicating the APM channels are active */
   atomic_t             prepared;         /* Flag indicating the APM channels are prepared to be enabled */
   struct dma_data_buf  zero;             /* DMA scratch buffer used for priming */
   struct apm_ch_cfg    ch[APM_MAX_NUM_CHANS];
   int                  micdet[APM_MICDET_MAX_CHANS];
};

#define APM_PROCDIR_NAME                  "bcmapm"
#define APM0_PROC_NAME                    "apm0"
#define APM1_PROC_NAME                    "apm1"
#define APM2_PROC_NAME                    "apm2"

#define APM_LOG_ENABLED                   0
#if APM_LOG_ENABLED
#define APM_LOG                           KNLLOG
#else
#define APM_LOG(c,args...)
#endif

#define APM_PROFILING_ENABLED             1
#if APM_PROFILING_ENABLED && defined(CONFIG_BCM_KNLLOG_SUPPORT)
#define APM_PROFILING(c,args...)          \
   if ( gKnllogIrqSchedEnable && KNLLOG_PROFILING ) { \
      KNLLOG(c, ## args); \
   }
#else
#define APM_PROFILING(c,args...)
#endif

/* ---- Private Variables ------------------------------------------------ */

#define APM_DEFAULT_CONFIGS \
   .samp_freq     = APM_DEFAULT_SAMP_FREQ, \
   .frame_period  = APM_DEFAULT_FRAME_PERIOD, \
   .frame_size    = APM_DEFAULT_FRAME_SIZE, \
   .prime_egr     = APM_DEFAULT_PRIME_SIZE_BYTES,

/*
* APM Information block The information block holds the static configuration
* of the APM channels. There are 2.5 channels in the APM block: 2 ADC/DAC
* pairs, and one single ADC named A, B, and C respectively. Core configuration
* information includes: channel ID, DMA configuration, sampling frequency,
* and other information.
*/
static struct apm_info  gApm =
{
   .ch =
   {
      [0] =
      {
         .ch            = 0,                 /* APM Channel A */
         .dma_igr       =
         {
            .device     = DMA_DEVICE_APM_CODEC_A_DEV_TO_MEM,
            .fifo_addr  = APM_CHAN0_PTM_FIFO_PHYSADDR,
         },
         .dma_egr       =
         {
            .device     = DMA_DEVICE_APM_CODEC_A_MEM_TO_DEV,
            .fifo_addr  = APM_CHAN0_MTP_FIFO_PHYSADDR,
         },
         APM_DEFAULT_CONFIGS
      },
      [1] =
      {
         .ch            = 1,                 /* APM Channel B */
         .dma_igr       =
         {
            .device     = DMA_DEVICE_APM_CODEC_B_DEV_TO_MEM,
            .fifo_addr  = APM_CHAN1_PTM_FIFO_PHYSADDR,
         },
         .dma_egr       =
         {
            .device     = DMA_DEVICE_APM_CODEC_B_MEM_TO_DEV,
            .fifo_addr  = APM_CHAN1_MTP_FIFO_PHYSADDR,
         },
         APM_DEFAULT_CONFIGS
      },
      [2] =
      {
         .ch            = 2,                 /* APM Channel C */
         .no_egr        = 1,                 /* Only support ingress */
         .dma_igr       =
         {
            .device     = DMA_DEVICE_APM_CODEC_C_DEV_TO_MEM,
            .fifo_addr  = APM_CHAN2_PTM_FIFO_PHYSADDR,
         },
         APM_DEFAULT_CONFIGS
      },
   }
};

static struct dma_pool    *gDmaPool;      /* DMA buffer pool */

/**
* Static sysctl data structures
*/
#define CTL_TABLE_INT(varStr,var) \
      procname: varStr,\
      data: var,\
      maxlen: sizeof(int),\
      mode: 0644,\
      proc_handler: &proc_dointvec,

static struct ctl_table_header *gSysCtlHeader;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_doApmLoop( ctl_table *table, int write,
      void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_doSineGen( ctl_table *table, int write,
      void __user *buffer, size_t *lenp, loff_t *ppos );
#else
static int proc_doApmLoop( ctl_table *table, int write, struct file *filp,
      void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_doSineGen( ctl_table *table, int write, struct file *filp,
      void __user *buffer, size_t *lenp, loff_t *ppos );
#endif

#define APM_CHAN_SYS_CTL( CH ) \
   { \
      .procname      = "loophwi2e"#CH, \
      .data          = &gApm.ch[CH].loophw_ig2eg, \
      .maxlen        = sizeof(int), \
      .mode          = 0644, \
      .proc_handler  = &proc_doApmLoop, \
      .extra1        = &gApm.ch[CH], \
      .extra2        = (void *)apmHw_LOOPBACK_MODE_TXRX, \
   }, \
   { \
      .procname      = "loophwe2i"#CH, \
      .data          = &gApm.ch[CH].loophw_eg2ig, \
      .maxlen        = sizeof(int), \
      .mode          = 0644, \
      .proc_handler  = &proc_doApmLoop, \
      .extra1        = &gApm.ch[CH], \
      .extra2        = (void *)apmHw_LOOPBACK_MODE_DAC_ADC, \
   }, \
   { CTL_TABLE_INT( "loopi2e"#CH, &gApm.ch[CH].loop_ig2eg) }, \
   { \
      .procname      = "loope2i"#CH,  /* Still hardware loopback */ \
      .data          = &gApm.ch[CH].loop_eg2ig, \
      .maxlen        = sizeof(int), \
      .mode          = 0644, \
      .proc_handler  = &proc_doApmLoop, \
      .extra1        = &gApm.ch[CH], \
      .extra2        = (void *)apmHw_LOOPBACK_MODE_NETWORK, \
   }, \
   { \
      .procname      = "sine_freq"#CH, \
      .data          = &gApm.ch[CH].sinectl.freq, \
      .maxlen        = sizeof(int), \
      .mode          = 0644, \
      .proc_handler  = &proc_doSineGen, \
      .extra1        = &gApm.ch[CH], \
   }, \
   { CTL_TABLE_INT( "ramp_igr"#CH,    &gApm.ch[CH].ramp_igr ) }, \
   { CTL_TABLE_INT( "ramp_egr"#CH,    &gApm.ch[CH].ramp_egr ) }, \
   { CTL_TABLE_INT( "ramp_check"#CH,  &gApm.ch[CH].ramp_check ) }

static struct ctl_table gSysCtlApm[] =
{
   APM_CHAN_SYS_CTL( 0 ),
   APM_CHAN_SYS_CTL( 1 ),
   { CTL_TABLE_INT( "ramp_igr2", &gApm.ch[2].ramp_igr ) },
   {}
};

static struct ctl_table gSysCtl[] =
{
   {
      .procname   = "bcmapm",
      .mode       = 0555,
      .child      = gSysCtlApm
   },
   {}
};
static struct ctl_table_header  *gSysCtlHeader;

/* Static procfs dir entry */
static struct proc_dir_entry    *gProcDir;

/* Installed callback. Called when all ingress processing has completed */
static HALAUDIO_IF_FRAME_ELAPSED gApmElapsedCb;
static void                     *gApmUserData;

/* Reference counter used to determine when all ingress interrupt processing
 * have completed
 */
static int                       gApmRefCount;

/* Number of synchronized channels */
static int                       gApmSyncChans;

/* HAL Audio interface handle */
static HALAUDIO_IF_HDL           gInterfHandle;

/* Translate Hal Audio directions to APM directions */
static apmHw_DIR halDirToApmDirTable[3] =
{
   [HALAUDIO_DIR_ADC]  = apmHw_DIR_TX,
   [HALAUDIO_DIR_DAC]  = apmHw_DIR_RX,
   [HALAUDIO_DIR_BOTH] = apmHw_DIR_TXRX,
};

/* Supported sampling frequencies. Do not support 22.05kHz and 44.1kHz yet due to
 * limitations of the audio mixer.
 *
 * The list is sorted from lowest to highest.
 */
static unsigned int gApmFreqs[] =
{
   8000,
   16000,
#if !defined(CONFIG_BCM_HALAUDIO_APM_DISABLE_22_44_KHZ)
	/* config option can be removed once 16 kHz and 22.05/44.1 kHz resampler used */
   22050,
#endif
   24000,
   32000,
   40000,
#if !defined(CONFIG_BCM_HALAUDIO_APM_DISABLE_22_44_KHZ)
   44100,
#endif
   48000,
};

/* Q12 gain map min and max ranges */
#define MAX_Q12_GAIN     18
#define MIN_Q12_GAIN     -50

/**
 * dB to 16-bit Q12 multiplier map. Used in setting software gains.
 * Ranges from -50dB to +18db (4096 = 0dB)
 */
static short q12GainMap[] =
{
   13, 15, 16, 18, 21, 23, 26, 29, 33, 37,
   41, 46, 52, 58, 65, 73, 82, 92, 103, 115,
   130, 145, 163, 183, 205, 230, 258, 290, 325, 365,
   410, 460, 516, 579, 649, 728, 817, 917, 1029, 1154,
   1295, 1453, 1631, 1830, 2053, 2303, 2584, 2900, 3254, 3651,
   4096, 4596, 5157, 5786, 6492, 7284, 8173, 9170, 10289, 11544,
   12953, 14533, 16306, 18296, 20529, 23034, 25844, 28997, 32536
};

/* Microphone detection callbacks */
static struct apm_mic_det_ops gApmMicDet;

/* APM clocks */
static struct clk *gApm100_clk;  /* Child clock */
static struct clk *gApm200_clk;  /* Parent clock */

/* Number of currently supported APM channels */
static unsigned int gApmNumChans;

/* Equalizer coefficients scratch buffer and mutex */
static int32_t gEquCoeffs[HALAUDIO_EQU_COEFS_MAX_NUM];   
static struct semaphore gEquCoeffsMutex;

/* ---- Private Function Prototypes -------------------------------------- */
static int  apmInit( HALAUDIO_IF_FRAME_ELAPSED cb, void *data );
static int  apmExit( void );
static int  apmPrepare( void );
static int  apmEnable( void );
static int  apmDisable( void );
static int  apmAnaPowerDown( int powerdn );
static int  apmPmShutdown( void );
static int  apmPmResume( void );
static int  apmSetFreq( int chno, int freqHz );
static int  apmGetFreq( int chno, int *freqhz );
static int  apmAnaGainSet( int chno, int db, HALAUDIO_DIR dir, HALAUDIO_HWSEL hwsel );
static int  apmAnaGainGet( int chno, HALAUDIO_GAIN_INFO *info, HALAUDIO_DIR dir, HALAUDIO_HWSEL hwsel );
static int  apmDigGainSet( int chno, int db, HALAUDIO_DIR dir );
static int  apmDigGainGet( int chno, HALAUDIO_GAIN_INFO *info, HALAUDIO_DIR dir );
static int  apmSidetoneGainSet( int chno, int db );
static int  apmSidetoneGainGet( int chno, HALAUDIO_GAIN_INFO *info );
static int  apmEquParmSet( int chno, HALAUDIO_DIR dir, const HALAUDIO_EQU *equ );
static int  apmEquParmGet( int chno, HALAUDIO_DIR dir, HALAUDIO_EQU *equ );
static int  apmWrite( int chno, int bytes, const char *audiobuf, HALAUDIO_CODEC_IORW_CB usercb, void *userdata );
static int  apmCodecInfo( int chno, HALAUDIO_CODEC_INFO *codec_info );
static int  apmCsxSet( int chno, HALAUDIO_CSX_POINT_ID point, const CSX_IO_POINT_FNCS *fncp, void *data );

static int  apmDmaInit( void );
static int  apmDmaTerm( void );
static void apmDmaIngressHandler( DMA_Device_t dev, int reason, void *data );
static void apmDmaEgressHandler( DMA_Device_t dev, int reason, void *data );
static void apmDmaEgressDoTransfer( struct apm_ch_cfg *ch );
static void apmProcInit( void );
static void apmProcTerm( void );
static int  apmReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data );
static void apmResetGainBlocks( void );
static int  apmWriteTerm( void );

static int  apmMixerPortsRegister( void );
static int  apmMixerPortsDeregister( void );
static int  apmMixerSetFreq( AMXR_PORT_ID portid, int freq, int bytes );

static int  apmEquInit( void );

static irqreturn_t apm_isr( int irq, void *dev_id );

/* ---- Functions -------------------------------------------------------- */

/* HAL Audio APM interface operations */
static HALAUDIO_IF_OPS halaudio_apm_ops __initdata =
{
   .init                = apmInit,
   .exit                = apmExit,
   .prepare             = apmPrepare,
   .enable              = apmEnable,
   .disable             = apmDisable,
   .ana_powerdn         = apmAnaPowerDown,
   .pm_shutdown         = apmPmShutdown,
   .pm_resume           = apmPmResume,
   .codec_ops           =
   {
      .setfreq          = apmSetFreq,
      .getfreq          = apmGetFreq,
      .setana           = apmAnaGainSet,
      .getana           = apmAnaGainGet,
      .setdig           = apmDigGainSet,
      .getdig           = apmDigGainGet,
      .setsidetone      = apmSidetoneGainSet,
      .getsidetone      = apmSidetoneGainGet,
      .setequ           = apmEquParmSet,
      .getequ           = apmEquParmGet,
      /*.read             = apmRead, */
      .write            = apmWrite,
      .info             = apmCodecInfo,
      .setcsx           = apmCsxSet,
   },
};

/***************************************************************************/
/**
*  Translate Hal Audio directions to APM directions
*
*  @return
*     +ve number  - APM direction code
*     -1          - invalid Hal Audio direction
*/
inline static int halDirToApmDir(
   HALAUDIO_DIR   dir               /**< (i)    Audio direction */
)
{
   if ( dir > HALAUDIO_DIR_BOTH )
   {
      return -1;
   }

   return halDirToApmDirTable[dir];
}

/***************************************************************************/
/**
*  Translate Hal Audio hardware mux selection to APM Switch position (A/B)
*
*  @return
*     +ve number  - APM switch position code
*     -1          - invalid Hal Audio mux selection
*/
inline static int halHwselToApmsw(
   HALAUDIO_HWSEL hwsel             /**< (i)    HW mux selection between A and B */
)
{
   int sw = -1;;

   if ( hwsel == HALAUDIO_HWSEL_A )
   {
      sw = apmHw_SWITCH_A;
   }
   else if ( hwsel == HALAUDIO_HWSEL_B )
   {
      sw = apmHw_SWITCH_B;
   }

   return sw;
}

/***************************************************************************/
/**
*  Translate db to Q12 linear gain value. The db value will also be
*  range limited
*
*  @return
*     +ve number  - linear gain value
*/
inline static short dbToLinearQ12(
   int     *db                      /**< (io) Ptr to gain in db */
)
{
   int dbval = *db;

   if ( dbval > MAX_Q12_GAIN )
   {
      *db = MAX_Q12_GAIN;
      return 0x7fff;        /* maximum gain */
   }

   if ( dbval < MIN_Q12_GAIN )
   {
      *db = HALAUDIO_GAIN_MUTE;
      return 0;             /* mute */
   }

   /* Map db to mapped linear value */
   return q12GainMap[dbval - MIN_Q12_GAIN];
}

/***************************************************************************/
/**
*  Translate linear Q12 gain value to db
*
*  @return
*     +ve number  - linear gain value
*/
inline static int linearQ12ToDb(
   int      lingain           /**< (i)    linear gain value */
)
{
   int i;

   for ( i = 0; i < sizeof(q12GainMap)/sizeof(q12GainMap[0]); i++ )
   {
      if ( lingain < q12GainMap[i] )
      {
         break;
      }
   }

   /* Adjust offset to determine dB value */
   i += MIN_Q12_GAIN;

   if ( i > MAX_Q12_GAIN )
   {
      i = MAX_Q12_GAIN;
   }

   return i;
}

/***************************************************************************/
/**
*  Initialize APM block
*/
static int apmInit(
   HALAUDIO_IF_FRAME_ELAPSED  cb,   /*<< (i) Callback to call when one frame elapses */
   void                      *data  /*<< (i) User data to pass to callback */
)
{
   int err;

   atomic_set( &gApm.running, 0 );
   atomic_set( &gApm.prepared, 0 );

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
      printk( KERN_ERR "%s: Failed to set freq for APM200 clock\n", __FUNCTION__ );
      goto exit_release_clocks;
   }

   err = clk_set_rate( gApm100_clk, 100000000 );
   if ( err )
   {
      printk( KERN_ERR "%s: Failed to set freq for APM100 clock\n", __FUNCTION__ );
      goto exit_release_clocks;
   }

   err = clk_enable( gApm100_clk );
   if ( err )
   {
      printk( KERN_ERR "%s: Failed to enable APM100 clock\n", __FUNCTION__ );
      goto exit_release_clocks;
   }

   /* Enable hardware audio channels */
   chipcHw_audioChannelEnable( chipcHw_REG_AUDIO_CHANNEL_ENABLE_NTP_CLOCK |
         chipcHw_REG_AUDIO_CHANNEL_ENABLE_APM_CLOCK );
   chipcHw_audioChannelEnable( chipcHw_REG_AUDIO_CHANNEL_ENABLE_ALL |
         (cap_isPresent(CAP_APM, 0)?chipcHw_REG_AUDIO_CHANNEL_ENABLE_A:0) |
         (cap_isPresent(CAP_APM, 1)?chipcHw_REG_AUDIO_CHANNEL_ENABLE_B:0) |
         (cap_isPresent(CAP_APM, 2)?chipcHw_REG_AUDIO_CHANNEL_ENABLE_C:0) );

   /* Ensure all settings cleared before starting */
   chipcHw_softReset( chipcHw_REG_SOFT_RESET_APM );

   /* Initialize APM hardware block */
   err = ApmHw_init();
   if ( err != 0 )
   {
      printk( KERN_ERR "APM: hardware intialization failed err=%i\n", err );
      goto exit_disable_clocks;
   }

   /* Initialize APM DMA resources */
   err = apmDmaInit();
   if ( err != 0 )
   {
      printk( KERN_ERR "APM: failed to initialize DMA resources err=%i\n", err );
      goto exit_disable_clocks;
   }

   apmProcInit();

   err = apmMixerPortsRegister();
   if ( err != 0 )
   {
      printk( KERN_ERR "%s: failed to create mixer ports err=%i\n", __FUNCTION__, err );
      goto exit_disable_clocks;
   }

   gApmElapsedCb  = cb;
   gApmUserData   = data;

   err = request_irq( IRQ_APM, apm_isr, IRQF_DISABLED, "apm", NULL);
   if ( err != 0 )
   {
      printk( KERN_ERR "APM: failed to initialize APM ISR err=%i\n", err );
      goto exit_disable_clocks;
   }

   apmResetGainBlocks();
    
   /* Initialize equalizers */ 
   err = apmEquInit(); 
   if ( err != 0 ) 
   { 
      printk( KERN_ERR "APM: failed to initialize Equalizer err=%i\n", err ); 
      goto exit_disable_clocks; 
   } 
   
   gApm.initialized = 1;

   return 0;

exit_disable_clocks:
   clk_disable( gApm100_clk );

exit_release_clocks:
   clk_put( gApm100_clk );
   clk_put( gApm200_clk );

   return err;
}

/***************************************************************************/
/**
*  Terminate APM driver
*/
static int apmExit( void )
{
   int error, rc;

   error = 0;

   rc = apmMixerPortsDeregister();
   if ( rc )
   {
      printk( KERN_ERR "%s: failed to cleanup mixer ports\n", __FUNCTION__ );
      error = rc;
   }

   apmProcTerm();

   rc = apmDisable();

   rc = apmDmaTerm();
   if ( rc != 0 )
   {
      printk( KERN_ERR "APM: failed to cleanup DMA resources\n" );
      error = rc;
   }

   apmWriteTerm();

   free_irq( IRQ_APM, NULL );

   rc = ApmHw_exit();
   if ( rc )
   {
      printk( KERN_ERR "APM: failed to cleanup APM hardware resources\n" );
      error = rc;
   }

   gApm.initialized = 0;

   clk_disable( gApm100_clk );

   clk_put( gApm100_clk );
   clk_put( gApm200_clk );

   return error;
}

/***************************************************************************/
/**
*  Prepare APM channel to start: allocate DMA channels, configure APM
*  parameters, prime egress DMA, start ingress DMA.
*
*  @remark        This routine may block.
*
*  @return
*     0           - success
*     -ENOMEM     - insufficient memory
*     -EBUSY      - cannot allocate DMA channels
*     -ve         - other error codes
*/
static int apmPrepare( void )
{
   int                  rc, i;
   struct apm_ch_cfg   *ch;

   if ( atomic_read( &gApm.running ) || atomic_read( &gApm.prepared ))
   {
      return -EBUSY; /* Already running or prepared */
   }

   ch = gApm.ch;
   for ( i = 0; i < gApmNumChans; i++, ch++ )
   {
      /* Index is pre-incremented before use. Set to second buffer to start */
      ch->active_idx    = 1;
      ch->isrcount_igr  = 0;
      ch->isrcount_egr  = 0;
      memset( &ch->errs, 0, sizeof(ch->errs) );

      ApmHw_SampleFreqSet( ch->ch, ch->samp_freq );

      dma_set_device_handler( ch->dma_igr.device, apmDmaIngressHandler,
            ch /* userData */ );

      /* Request egress DMA channel, if supported */
      if ( !ch->no_egr )
      {
         dma_set_device_handler( ch->dma_egr.device, apmDmaEgressHandler,
               ch /* userData */ );

         ch->dma_egr.handle = dma_alloc_channel( ch->dma_egr.device );
         if ( ch->dma_egr.handle < 0 )
         {
            printk( KERN_ERR "APM: [CH %u] failed to get MTP DMA channel\n", ch->ch );
            rc = -EBUSY;
            goto cleanup_dma_channels;
         }

         /* Clear egress samples */
         memset( ch->buf_egr[0].virt, 0, ch->frame_size );
         memset( ch->buf_egr[1].virt, 0, ch->frame_size );

         /* Prime egress */
         atomic_set( &ch->queued_pkts_egr, 1 );
         rc = dma_transfer_to_device( ch->dma_egr.handle,
               gApm.zero.phys, ch->dma_egr.fifo_addr, ch->frame_size + ch->prime_egr );
         if ( rc != 0 )
         {
            printk( KERN_ERR "APM: [CH %u] egress prime DMA failed\n", ch->ch );
            goto cleanup_dma_channels;
         }
      }

      /* Request ingress DMA channel */
      ch->dma_igr.handle = dma_alloc_channel( ch->dma_igr.device );
      if ( ch->dma_igr.handle < 0 )
      {
         printk( KERN_ERR "APM: [CH %u] failed to get PTM DMA channel\n", ch->ch );
         rc = -EBUSY;
         goto cleanup_dma_channels;
      }

      /* Allocate ingress double buffer DMA descriptors */
      rc = dma_alloc_double_dst_descriptors( ch->dma_igr.handle,
            ch->dma_igr.fifo_addr, ch->buf_igr[0].phys, ch->buf_igr[1].phys,
            ch->frame_size );
      if ( rc != 2 )
      {
         printk( KERN_ERR "APM: [CH %u] %d ingress descriptors are allocated, instead of 2.\n",
               ch->ch, rc );
         goto cleanup_dma_channels;
      }

      /* Start ingress DMA channel. DMA does not actually start until APM channel is enabled. */
      rc = dma_start_transfer( ch->dma_igr.handle, 2 * ch->frame_size );
      if ( rc != 0 )
      {
         printk( KERN_ERR "APM: [CH %u] Failed to start ingress DMA\n", ch->ch );
         goto cleanup_dma_channels;
      }
   }

   gApmRefCount = 0;
   atomic_set( &gApm.prepared, 1 );
   return 0;

cleanup_dma_channels:

   /* Free DMA channels that may have been allocated */
   ch = gApm.ch;
   for ( i = 0; i < gApmNumChans; i++, ch++ )
   {
      if ( !ch->no_egr )
      {
         dma_set_device_handler( ch->dma_egr.device, NULL, NULL );

         if ( ch->dma_egr.handle >= 0 )
         {
            dma_free_channel( ch->dma_egr.handle );
            ch->dma_egr.handle = DMA_INVALID_HANDLE;
         }
      }

      dma_set_device_handler( ch->dma_igr.device, NULL, NULL );

      if ( ch->dma_igr.handle >= 0 )
      {
         dma_free_channel( ch->dma_igr.handle );
         ch->dma_igr.handle = DMA_INVALID_HANDLE;
      }
   }
   return rc;
}

/***************************************************************************/
/**
*  Enable APM channels
*
*  @remark    This routine cannot block. apmPrepare must be called first to
*             prepare the channels.
*
*  @return
*     0        - success
*     -EPERM   - APM channels have not been prepared
*     -ve      - failure return code
*/
static int apmEnable( void )
{
   int rc;

   if ( !atomic_read( &gApm.prepared ))
   {
      return -EPERM;
   }

   if ( atomic_read( &gApm.running ))
   {
      return 0;   /* Already enabled */
   }

   rc = ApmHw_ChEnable( apmHw_ALL_CH );

   /* Enable interrupts to detect error conditions */
   ApmHw_InterruptEnable( 1, APM_INT_MASKS_ERRORS );

   if ( rc == 0 )
   {
      atomic_set( &gApm.running, 1 );
   }

   return rc;
}

/***************************************************************************/
/**
*  Disable APM channel(s) within an interface. Interrupts and resources
*  associated with the codec channel(s) will be released.
*
*  @return
*     0        Success
*     -ve      Error code
*/
static int apmDisable( void )
{
   int                rc;
   int                i, err;
   struct apm_ch_cfg *ch;

   if ( !atomic_read( &gApm.prepared ))
   {
      return 0; /* Nothing to disable, has not been prepared */
   }

   atomic_set( &gApm.running, 0 );

   /* Mask APM interrupts to avoid false detecting errors during disable */
   ApmHw_InterruptEnable( 0, APM_INT_MASKS_ERRORS );

   /* stop egress DMA, APM block, and then ingress DMA */
   err = 0;
   ch  = gApm.ch;
   for ( i = 0; i < gApmNumChans; i++, ch++ )
   {
      if ( !ch->no_egr )
      {
         atomic_set( &ch->queued_pkts_egr, 0 );
         rc = dma_stop_transfer( ch->dma_egr.handle );
         if ( rc != 0 )
         {
            printk( KERN_ERR "APM: [CH %u] failed to stop egress DMA\n", ch->ch );
            err = rc;
         }
         dma_set_device_handler( ch->dma_egr.device, NULL, NULL );
      }
   }

   ApmHw_ChDisable( apmHw_ALL_CH );

   ch = gApm.ch;
   for ( i = 0; i < gApmNumChans; i++, ch++ )
   {
      rc = dma_stop_transfer( ch->dma_igr.handle );
      if ( rc != 0 )
      {
         printk( KERN_ERR "APM: [CH %u] failed to stop egress DMA\n", ch->ch );
         err = rc;
      }
      dma_set_device_handler( ch->dma_igr.device, NULL, NULL );
   }

   /* Free DMA channels */
   ch = gApm.ch;
   for ( i = 0; i < gApmNumChans; i++, ch++ )
   {
      if ( !ch->no_egr )
      {
         dma_free_channel( ch->dma_egr.handle );
         ch->dma_egr.handle = DMA_INVALID_HANDLE;
      }
      dma_free_channel( ch->dma_igr.handle );
      ch->dma_igr.handle = DMA_INVALID_HANDLE;
   }

   /* Flush write buffers */
   ch = gApm.ch;
   for ( i = 0; i < gApmNumChans; i++, ch++ )
   {
      halAudioWriteFlush( &ch->write );
   }

   atomic_set( &gApm.prepared, 0 );

   return err;
}

/***************************************************************************/
/**
*  Power down analog hardware for codec channel(s)
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remark
*     This method can choose to operate on one or all of the channels
*     in the interface at a time. To operate on all of the channels,
*     use the HALAuDIO_IF_ALL_CODECS.
*/
static int apmDacRampDown( int powerdn )
{
   struct apm_ch_cfg *ch;
   int i, err;

   err = 0;
   ch  = gApm.ch;
   for ( i = 0; i < gApmNumChans; i++, ch++ )
   {
      if ( !ch->no_egr )
      {
         err |= ApmHw_AmpDacRampDown( ch->ch, apmHw_SWITCH_A, apmHw_DAC_RAMP_SHAPE_COSINE, APM_DEFAULT_DAC_POWERDN_RAMPTIME, powerdn );
         err |= ApmHw_AmpDacRampDown( ch->ch, apmHw_SWITCH_B, apmHw_DAC_RAMP_SHAPE_COSINE, APM_DEFAULT_DAC_POWERDN_RAMPTIME, powerdn );
      }
   }

   return err;
}

/***************************************************************************/
/**
*  Power down analog hardware for codec channel(s)
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remark
*     This method can choose to operate on one or all of the channels
*     in the interface at a time. To operate on all of the channels,
*     use the HALAuDIO_IF_ALL_CODECS.
*/
static int apmAnaPowerDown(
   int powerdn                      /*<< (i) 1 to power down, 0 to power up */
)
{
   struct apm_ch_cfg *ch;
   int                err, rc;
   int                i;

   ch  = gApm.ch;
   rc = 0;
   if ( powerdn )
   {
      /* Initiate DAC ramp down prior to power down to avoid sudden glitch.
       * Delay is necessary to allow ramp down to proceed.
       */
      err = apmDacRampDown( powerdn );
      if ( err )
      {
         rc = err;
      }
      set_current_state( TASK_INTERRUPTIBLE );
      schedule_timeout( HZ * APM_DEFAULT_DAC_POWERDN_BLOCKING_DELAY / 1000 );
   }

   ch  = gApm.ch;
   err = 0;
   for ( i = 0; i < gApmNumChans; i++, ch++ )
   {
      /* Power up/down microphone only if not mute. If mute it is already
       * powered down and thus not need to be powered up.
       */
      if ( ch->mic.ana_gain[0] > HALAUDIO_GAIN_MUTE )
      {
         err |= ApmHw_AmpSetPowerDown( ch->ch, apmHw_DIR_TX, powerdn );
      }
      
      if ( !ch->no_egr )
      {
         /* Power up/down speaker */
         err |= ApmHw_AmpSetPowerDown( ch->ch, apmHw_DIR_RX, powerdn );
      }

      /* Microphone bias power up/down. Mic bias position A cannot be powered
       * up/down arbitrarily because mic insertion detection is tied to this
       * circuitry. When mic insertion detection is enabled, it cannot power
       * down the associated mic bias, thus logic has been added to guard 
       * against this.
       */
      if ( ch->ch < APM_MICDET_MAX_CHANS )
      {
         if (( !gApm.micdet[ch->ch] && powerdn ) || !powerdn )
         {
            err |= ApmHw_SetMicBias( ch->ch, apmHw_SWITCH_A, !powerdn );
         }
      }
      err |= ApmHw_SetMicBias( ch->ch, apmHw_SWITCH_B, !powerdn );
   }
   if ( err )
   {
      rc = err;
   }

   ch  = gApm.ch;
   if ( !powerdn )
   {
      /* Initiate DAC ramp up following power up */
      err = apmDacRampDown( powerdn );
      if ( err )
      {
         rc = err;
      }
   }

   return rc;
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
*  @remark
*/
static int apmPmShutdown( void )
{
   struct apm_ch_cfg *ch;
   int                i;

   ch  = gApm.ch;
   if ( atomic_read( &gApm.running ))
   {
      return -EBUSY;
   }

   /* Explicitly remove the mic bias associated with mic detection channel
    * to reduce power consumption. This bias has not been disabled earlier 
    * because mic detection requires the bias to be active.
    */
   for ( i = 0; i < APM_MICDET_MAX_CHANS; i++, ch++ )
   {
      ApmHw_SetMicBias( ch->ch, apmHw_SWITCH_A, 0 );
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
*  @remark
*/
static int apmPmResume( void )
{
   struct apm_ch_cfg *ch;
   int                i;
   int                retVal;
   ch  = gApm.ch;

   retVal = clk_enable( gApm100_clk );

   /* Re-enable mic bias on channels that have mic detection enabled */
   for ( i = 0; i < APM_MICDET_MAX_CHANS; i++, ch++ )
   {
      if ( gApm.micdet[ch->ch] )
      {
         ApmHw_SetMicBias( ch->ch, apmHw_SWITCH_A, 1 );
      }
   }

   return retVal;
}

/***************************************************************************/
/**
*  Enable microphone detection
*/
int apmEnableMicDetect(
   enum apm_micdet_chan ch,         /**< (i) Channel */
   int                  enable      /**< (i) 1 to enable, 0 to disable */
)
{
   if ( ch >= APM_MICDET_MAX_CHANS )
   {
      return -EINVAL;
   }

   gApm.micdet[ch] = enable;

   /* Setup interrupt mask */
   ApmHw_InterruptEnable( enable, (ch == APM_MICDET_A) ?
         APM_INT_MASK_MIC_A_IN : APM_INT_MASK_MIC_B_IN );

   if ( enable )
   {
      /* Enable mic bias for detection. Do nothing when disabling
       * mic detection
       */
      ApmHw_SetMicBias( ch, apmHw_SWITCH_A, 1 );
   }

   return 0;
}

/***************************************************************************/
/**
*  Select sampling frequency for all channels
*
*  @return
*     0        Success
*     -ENODEV  APM has not be initiialized
*     -EBUSY   APM is currently running
*     -EINVAL  Invalid channel number
*
*  @remark
*     This routine needs the channel disabled before it can proceed
*     because it must reset the DMA chains and configure the APM registers.
*/
static int apmSetFreq(
   int chno,                        /**< (i) channel number 0-2 */
   int freqHz                       /**< (i) sampling frequency in Hz */
)
{
   int                rc;
   struct apm_ch_cfg *ch;
   unsigned int       frame_size, frame_period, prime_egr;
   int                i, validhz;
   int                old_samp_freq;

   if ( !gApm.initialized )
   {
      return -ENODEV;
   }

   if ( atomic_read( &gApm.running ) || atomic_read( &gApm.prepared ))
   {
      /* Cannot change sampling frequency if currently running or already prepared */
      return -EBUSY;
   }

   /* Validate sampling frequency is supported */
   validhz = 0;
   for ( i = 0; i < sizeof(gApmFreqs)/sizeof(gApmFreqs[0]); i++ )
   {
      if ( freqHz == gApmFreqs[i] )
      {
         validhz = 1;
         break;
      }
   }
   if ( !validhz )
   {
      return -EINVAL;
   }

   ch = &gApm.ch[chno];

   /* Calculate frame size and period based on selected sampling frequency.
    * The frame period is recalculated to account for rounding errors with
    * 22.05 kHz and 44.1 kHz sampling rates.
    */
   frame_size   = (freqHz * APM_DEFAULT_FRAME_PERIOD * APM_SAMP_WIDTH) / 1000000; /* in bytes */

   /* Frame size must be multiples of 16 bytes to match with DMA bursts */
   frame_size   = frame_size & ~0xf;

   frame_period = (frame_size * (1000000/APM_SAMP_WIDTH)) / freqHz;               /* in usec */
   prime_egr    = frame_size;

   rc = apmMixerSetFreq( ch->mixer_port, freqHz, frame_size );
   if ( rc == 0 )
   {
      old_samp_freq = ch->samp_freq;

      ch->frame_period  = frame_period;
      ch->frame_size    = frame_size;
      ch->samp_freq     = freqHz;
      ch->prime_egr     = prime_egr;

      /* Update sine generation state */
      halAudioSineConfig( &ch->sinectl, ch->sinectl.freq, ch->samp_freq );

      /* Update synchronized channel count if there is a change */
      if ( APM_SYNC_FREQ( old_samp_freq ) != APM_SYNC_FREQ( freqHz ))
      {
         if ( APM_SYNC_FREQ( freqHz ))
         {
            gApmSyncChans++;
         }
         else
         {
            gApmSyncChans--;
         }
      }

      rc = halAudioSetSyncFlag( gInterfHandle, gApmSyncChans > 0 );

      APM_LOG( "samp_freq=%i frame_size=%i frame_period=%i prime_egr=%i",
            ch->samp_freq, ch->frame_size, ch->frame_period, ch->prime_egr );
   }

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
static int apmGetFreq(
   int   chno,                      /*<< (i) Codec channel number */
   int  *freqhz                     /*<< (o) Ptr to sampling frequency in Hz */
)
{
   struct apm_ch_cfg *ch;

   ch       = &gApm.ch[chno];
   *freqhz  = ch->samp_freq;

   return 0;
}

/***************************************************************************/
/**
*  Set hardware analog PGA gains. Each channel consists of up to four
*  PGA blocks, two per direction (ingress and egress).
*
*  @return
*     0        - success
*     -ve      - failure code
*/
static int apmAnaGainSet(
   int             chno,      /**< (i)    Channel index */
   int             db,        /**< (i)    dB value to set gain */
   HALAUDIO_DIR    dir,       /**< (i)    Audio direction */
   HALAUDIO_HWSEL  hwsel      /**< (i)    HW mux selection between A and B */
)
{
   int                apmdir, sw, error;
   struct apm_ch_cfg *ch;

   ch       = &gApm.ch[chno];
   apmdir   = halDirToApmDir( dir );
   sw       = halHwselToApmsw( hwsel );

   if ( apmdir < 0 || sw < 0 )
   {
      return -EINVAL;
   }

   if ( chno >= gApmNumChans )
   {
      return -EINVAL;
   }

   error = ApmHw_PGAGainSet( ch->ch, sw, apmdir, db );


   /* Select mic input or speaker output */
   if ( !error )
   {
      if ( dir == HALAUDIO_DIR_ADC )
      {
         /* Use ADC power to implement microphone mute */
         error = ApmHw_AmpSetPowerDown( ch->ch, apmdir, db <= HALAUDIO_GAIN_MUTE);

         if ( !error )
         {
            error = ApmHW_SetTxMux( ch->ch, sw );
         }
      }
      else if ( dir == HALAUDIO_DIR_DAC )
      {
         apmHw_SPK_MODE mode;
         mode = ( db > HALAUDIO_GAIN_MUTE ) ? apmHw_SPK_MODE_MUTE_NONE : apmHw_SPK_MODE_MUTE_BOTH;
         error = ApmHw_SetSpkMode( ch->ch, sw, mode );
      }
   }

   /* Save state for context restore */
   if ( !error )
   {
      if ( dir == HALAUDIO_DIR_ADC )
      {
         /* Mic positions are mutually exclusive */
         ch->mic.ana_gain[0] = db;
         ch->mic.ana_gain[1] = db;
      }
      else
      {
         /* Map mux selection to array index */
         int gain_index  = ( hwsel == HALAUDIO_HWSEL_A ) ? 0 : 1;
         ch->spk.ana_gain[gain_index] = db;
      }
   }

   return error;
}

/***************************************************************************/
/**
*  Retrieve analog gains for a codec channel.
*
*  @return
*     0        Success
*     -ve      Error code
*/
static int apmAnaGainGet(
   int                  chno,       /*<< (i) codec channel number */
   HALAUDIO_GAIN_INFO  *info,       /*<< (o) Ptr to gain info structure */
   HALAUDIO_DIR         dir,        /*<< (i) Direction path */
   HALAUDIO_HWSEL       hwsel       /*<< (i) Hardware mux selection */
)
{
   struct apm_ch_cfg *ch;
   int                apmdir, sw, error;
   int                pgaGain;

   ch       = &gApm.ch[chno];
   apmdir   = halDirToApmDir( dir );
   sw       = halHwselToApmsw( hwsel );

   if ( apmdir < 0 || sw < 0 )
   {
      return -EINVAL;
   }

   error = ApmHw_PGAGainGet( ch->ch, sw, apmdir, &pgaGain );
   if ( error )
   {
      return -EINVAL;
   }
   info->currentdb = pgaGain;

   if ( dir == HALAUDIO_DIR_DAC )
   {
      apmHw_SPK_MODE mode;

      if ( ch->no_egr )
      {
         /* Egress/DAC channel unsupported */
         return -EINVAL;
      }

      /* Check if speaker direction is muted */
      error = ApmHw_GetSpkMode( ch->ch, sw, &mode );
      if ( error )
      {
         return -EINVAL;
      }
      if ( mode == apmHw_SPK_MODE_MUTE_BOTH )
      {
         info->currentdb = HALAUDIO_GAIN_MUTE;
      }

      info->mindb             = -21;
      info->maxdb             = 0;
      info->range_type        = HALAUDIO_RANGETYPE_FIXED_STEPSIZE;
      info->range.fixed_step  = 3;   /* 3dB increments */
   }
   else
   {
      int j, *gainp, numgains;

      /* Microphone gain settings */

      gainp    = &info->range.list.db[0];
      numgains = 0;

      /* 0 to 21 db in 3db increments */
      for ( j = 0; j < 21 && numgains < HALAUDIO_DB_SETTINGS_MAX_NUM; j += 3 )
      {
         *gainp = j;
         gainp++;
         numgains++;
      }

      /* 21 to 42 dB in 1db increments */
      for ( j = 21; j <= 42 && numgains < HALAUDIO_DB_SETTINGS_MAX_NUM; j++ )
      {
         *gainp = j;
         gainp++;
         numgains++;
      }
      info->mindb          = 0;
      info->maxdb          = 42;
      info->range_type     = HALAUDIO_RANGETYPE_LIST;
      info->range.list.num = numgains;

      if ( ApmHw_AmpGetPowerDown( ch->ch, apmHw_DIR_TX ))
      {
         info->currentdb = HALAUDIO_GAIN_MUTE;
      }
   }

   return 0;
}

/***************************************************************************/
/**
*  Set hardware digital gains. Each channel consists of up to two
*  gains, one per direction (ingress and egress).
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remark
*/
static int apmDigGainSet(
   int             chno,      /**< (i)    Channel index */
   int             db,        /**< (i)    dB value to set gain */
   HALAUDIO_DIR    dir        /**< (i)    Audio direction */
)
{
   struct apm_ch_cfg *ch;
   int                apmdir;
   short              lin_gain;
   int                error;

   apmdir = halDirToApmDir( dir );
   if ( apmdir < 0 )
   {
      return -EINVAL;
   }

   ch = &gApm.ch[chno];

   /* Use hardware digital gain block */
   lin_gain = dbToLinearQ12( &db );

   error = ApmHW_DigitalGainSet( ch->ch, apmdir, lin_gain );

   /* Save state for context restore */
   if ( !error )
   {
      if ( dir == HALAUDIO_DIR_ADC )
      {
         ch->mic.dig_gain   = db;
      }
      else
      {
         ch->spk.dig_gain   = db;
      }
   }

   return error;
}

/***************************************************************************/
/**
*  Retrieve digital gains for a codec channel. Valid gain range excludes
*  mute.
*
*  @return
*     0        Success
*     -ve      Error code
*/
static int apmDigGainGet(
   int                  chno,       /*<< (i) codec channel number */
   HALAUDIO_GAIN_INFO  *info,       /*<< (o) Ptr to gain info structure */
   HALAUDIO_DIR         dir         /*<< (i) Direction path */
)
{
   struct apm_ch_cfg *ch;

   info->mindb             = MIN_Q12_GAIN;
   info->maxdb             = MAX_Q12_GAIN;
   info->range_type        = HALAUDIO_RANGETYPE_FIXED_STEPSIZE;
   info->range.fixed_step  = 1;  /* 1dB increments */

   ch = &gApm.ch[chno];
   if ( dir == HALAUDIO_DIR_DAC )
   {
      if ( ch->no_egr )
      {
         /* Egress/DAC channel unsupported */
         return -EINVAL;
      }

      info->currentdb = ch->spk.dig_gain;
   }
   else
   {
      info->currentdb = ch->mic.dig_gain;
   }

   return 0;
}

/***************************************************************************/
/**
*  Set sidetone gain. Mute may be set with db=HALAUDIO_GAIN_MUTE.
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remark
*/
static int apmSidetoneGainSet(
   int      chno,                   /**< (i)    Channel index */
   int      db                      /**< (i)    Gain in db */
)
{
   struct apm_ch_cfg *ch;
   short              lin_gain;
   int                error;

   ch       = &gApm.ch[chno];
   lin_gain = dbToLinearQ12( &db );

   error = ApmHW_SidetoneVolumeSet( ch->ch, lin_gain );

   if ( !error )
   {
      error = ApmHW_SidetoneEnable( ch->ch, db > HALAUDIO_GAIN_MUTE );
   }

   /* Save state for context restore */
   if ( !error )
   {
      ch->sidetone.db   = db;
   }

   return error;
}

/***************************************************************************/
/**
*  Query sidetone gain info. Current gain and other gain information
*  are returned.
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remark
*/
static int apmSidetoneGainGet(
   int                  chno,       /*<< (i) codec channel number */
   HALAUDIO_GAIN_INFO  *info        /*<< (o) Ptr to gain info structure */
)
{
   struct apm_ch_cfg *ch;

   ch = &gApm.ch[chno];

   info->currentdb         = ch->sidetone.db;
   info->mindb             = MIN_Q12_GAIN;
   info->maxdb             = 0;
   info->range_type        = HALAUDIO_RANGETYPE_FIXED_STEPSIZE;
   info->range.fixed_step  = 1;  /* 1dB increments */

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
static int apmEquParmSet(
   int                  chno,       /*<< (i) Codec channel number */
   HALAUDIO_DIR         dir,        /*<< (i) Direction path */
   const HALAUDIO_EQU  *equ         /*<< (i) Ptr to equalizer parameters */
)
{
   apmHw_COEFF        select;
   int                err;
   HALAUDIO_EQU      *saved_equ;
   struct apm_ch_cfg *ch;

   ch = &gApm.ch[chno];

   if ( ch->no_egr && dir == HALAUDIO_DIR_DAC )
   {
      return -EINVAL;
   }

   if ( dir == HALAUDIO_DIR_ADC )
   {
      select      = apmHw_COEFF_TX_EQ;
      saved_equ   = &ch->equ_igr;
   }
   else
   {
      select      = apmHw_COEFF_RX_EQ;
      saved_equ   = &ch->equ_egr;
   }

   down( &gEquCoeffsMutex );

   memset( gEquCoeffs, 0, sizeof(gEquCoeffs) );
   
   if ( equ->len > 0 )
   {
      memcpy( gEquCoeffs, equ->coeffs, sizeof(equ->coeffs[0]) * equ->len );
   }
   else
   {
       /* An impulse response */
      gEquCoeffs[0] = 0x7FFF;
   }

   err = ApmHw_WriteCoeff( ch->ch, select, gEquCoeffs, HALAUDIO_EQU_COEFS_MAX_NUM ); 
   if ( !err )
   {
      /* Save equalizer parameters */
      memcpy( saved_equ, equ, sizeof(*saved_equ) );
   }

   up( &gEquCoeffsMutex );

   return err;
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
static int apmEquParmGet(
   int                  chno,       /*<< (i) Codec channel number */
   HALAUDIO_DIR         dir,        /*<< (i) Direction path */
   HALAUDIO_EQU        *equ         /*<< (0) Ptr to equalizer parameters */
)
{
   struct apm_ch_cfg *ch;
   HALAUDIO_EQU      *saved_equ;

   ch = &gApm.ch[chno];

   if ( ch->no_egr && dir == HALAUDIO_DIR_DAC )
   {
      return -EINVAL;
   }

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
*  Write method used to directly write samples to a channel's DAC buffers.
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remark
*/
static int apmWrite(
   int                     chno,       /**< (i) channel index */
   int                     bytes,      /**< (i) Number of bytes to write */
   const char             *audiobuf,   /**< (i) Pointer to audio samples */
   HALAUDIO_CODEC_IORW_CB  usercb,     /**< (i) User callback to request for more data */
   void                   *userdata    /**< (i) User data */
)
{
   struct apm_ch_cfg *ch;

   ch = &gApm.ch[chno];
   if ( ch->no_egr )
   {
      return -EPERM;
   }

   return halAudioWriteRequest( &ch->write, bytes, audiobuf, usercb, userdata );
}

/***************************************************************************/
/**
*  Query codec channel information
*
*  @return
*     0        Success
*     -ve      Error code
*
*  @remark
*/
static int  apmCodecInfo(
   int                  chno,       /*<< (i) Codec channel number */
   HALAUDIO_CODEC_INFO *info        /*<< (o) Ptr to codec info structure */
)
{
   struct apm_ch_cfg *ch;
   int                i;

   if ( chno >= gApmNumChans )
   {
      return -EINVAL;
   }

   ch = &gApm.ch[chno];

   memset( info, 0, sizeof(*info) );

   for ( i = 0; i < sizeof(gApmFreqs)/sizeof(gApmFreqs[0]) && i < HALAUDIO_MAX_NUM_FREQ_SETTINGS; i++ )
   {
      info->freqs.freq[i] = gApmFreqs[i];
      info->freqs.num++;
   }

   info->channels_tx    = 1;  /* mono */
   info->equlen_tx      = apmHw_REG_CHA_EQ_CONFIG_EQ_COEFF_COUNT_MAX;
   info->sample_width   = APM_SAMP_WIDTH;
   info->mics           = APM_MAX_MUX_POSITIONS;
   info->bulk_delay     = -1; /* FIXME: Need a way to calibrate */
   info->read_format    = HALAUDIO_FMT_S16_LE;
   info->write_format   = HALAUDIO_FMT_S16_LE;

   if ( !ch->no_egr )
   {
      info->channels_rx = 1;  /* mono */
      info->equlen_rx   = apmHw_REG_CHA_EQ_CONFIG_EQ_COEFF_COUNT_MAX;
      info->spkrs       = APM_MAX_MUX_POSITIONS;
   }

   sprintf( info->name, "APM CH%i", ch->ch );

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
static int apmCsxSet(
   int                        chno, /*<< (i) Codec channel number */
   HALAUDIO_CSX_POINT_ID      point,/*<< (i) Point ID to install the CSX point */
   const CSX_IO_POINT_FNCS   *fncp, /*<< (i) Ptr to CSX callbacks */
   void                      *data  /*<< (i) User data to pass back to callbacks */
)
{
   struct apm_ch_cfg *ch;
   unsigned long      flags;
   int err = 0;

   if ( chno >= gApmNumChans )
   {
      return -EINVAL;
   }

   ch = &gApm.ch[chno];

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

   return err;
}

/***************************************************************************/
/**
*  APM helper routine to initialize gain block states
*/
static void apmResetGainBlocks( void )
{
   int                  chno;
   struct apm_ch_cfg   *ch;

   ch = gApm.ch;
   for ( chno = 0; chno < gApmNumChans; chno++, ch++ )
   {
      ch->mic.dig_gain     = HALAUDIO_GAIN_MUTE;
      ch->mic.ana_gain[0]  = HALAUDIO_GAIN_MUTE;
      ch->mic.ana_gain[1]  = HALAUDIO_GAIN_MUTE;
      ch->spk.dig_gain     = HALAUDIO_GAIN_MUTE;
      ch->spk.ana_gain[0]  = HALAUDIO_GAIN_MUTE;
      ch->spk.ana_gain[1]  = HALAUDIO_GAIN_MUTE;
      ch->sidetone.db      = HALAUDIO_GAIN_MUTE;

      apmDigGainSet( ch->ch, ch->mic.dig_gain, HALAUDIO_DIR_ADC );
      apmDigGainSet( ch->ch, ch->spk.dig_gain, HALAUDIO_DIR_DAC );
      apmAnaGainSet( ch->ch, ch->mic.ana_gain[0], HALAUDIO_DIR_ADC, HALAUDIO_HWSEL_A );
      apmAnaGainSet( ch->ch, ch->spk.ana_gain[0], HALAUDIO_DIR_DAC, HALAUDIO_HWSEL_A );
      apmAnaGainSet( ch->ch, ch->spk.ana_gain[1], HALAUDIO_DIR_DAC, HALAUDIO_HWSEL_B );
      apmSidetoneGainSet( ch->ch, ch->sidetone.db );
   }
}

/***************************************************************************/
/**
*  Setup APM DMA memory resources
*
*  @return
*     0        - success
*     -ENOMEM  - failed to allocate memory
*/
static int apmDmaInit( void )
{
   int                i;
   struct apm_ch_cfg *ch;
   int                rc = 0;

   /* Create DMA buffer pool */
   gDmaPool = dma_pool_create( "APM DMA memory pool", NULL,
         APM_MAX_DMA_BUFFER_SIZE_BYTES, APM_DMA_ALIGN_IN_BYTES,
         APM_DMA_ALLOC_CROSS_RESTRICT );
   if ( gDmaPool == NULL )
   {
      printk( KERN_ERR "APM: failed to allocate DMA buffer pool\n" );
      return -ENOMEM;
   }

   ch = gApm.ch;
   for ( i = 0; i < gApmNumChans; i++, ch++ )
   {
      /* Allocate ingress and egress audio buffers. Note that egress buffer
       * is allocated regardless whether the egress direction is supported or not.
       * This allows mixer to connect to a non-existent egress channel with a valid
       * buffer.
       */
      ch->buf_igr[0].virt = dma_pool_alloc( gDmaPool, GFP_KERNEL, &ch->buf_igr[0].phys );
      ch->buf_igr[1].virt = dma_pool_alloc( gDmaPool, GFP_KERNEL, &ch->buf_igr[1].phys );
      ch->buf_egr[0].virt = dma_pool_alloc( gDmaPool, GFP_KERNEL, &ch->buf_egr[0].phys );
      ch->buf_egr[1].virt = dma_pool_alloc( gDmaPool, GFP_KERNEL, &ch->buf_egr[1].phys );

      if ( !ch->buf_igr[0].virt || !ch->buf_igr[1].virt || !ch->buf_egr[0].virt || !ch->buf_egr[1].virt )
      {
         printk( KERN_ERR "APM: [CH %u] failed to allocate DMA audio buffers\n", ch->ch );
         rc = -ENOMEM;
         goto cleanup_exit;
      }
   }

   gApm.zero.virt = dma_alloc_writecombine( NULL, APM_MAX_PRIME_SIZE_BYTES,
            &gApm.zero.phys, GFP_KERNEL );
   if ( gApm.zero.virt == NULL )
   {
      rc = -ENOMEM;
      goto cleanup_exit;
   }
   memset( gApm.zero.virt, 0, APM_MAX_PRIME_SIZE_BYTES );

   return 0;

cleanup_exit:
   apmDmaTerm();
   return rc;
}

/***************************************************************************/
/**
*  Cleanup APM DMA memory resources
*
*  @return
*/
static int apmDmaTerm( void )
{
   int                i;
   struct apm_ch_cfg *ch;
   int                rc = 0;

   ch = gApm.ch;

   for ( i = 0; i < gApmNumChans; i++, ch++ )
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
   }
   dma_pool_destroy( gDmaPool );

   if ( gApm.zero.virt )
   {
      dma_free_writecombine( NULL, APM_MAX_PRIME_SIZE_BYTES,
            gApm.zero.virt, gApm.zero.phys );
      gApm.zero.virt = NULL;
   }

   return rc;
}

/***************************************************************************/
/**
*  Cleanup APM write memory resources
*
*  @return
*/
static int apmWriteTerm( void )
{
   struct apm_ch_cfg *ch;
   int                i;

   ch = gApm.ch;
   for ( i = 0; i < gApmNumChans; i++, ch++ )
   {
      halAudioWriteFree( &ch->write );
   }

   return 0;
}

/***************************************************************************/
/**
*  APM ingress DMA interrupt handler that services ingress DMA's for all the
*  APM channels.
*
*  @return  None
*/
static void apmDmaIngressHandler(
   DMA_Device_t   dev,           /**< (i) Device that triggered callback */
   int            reason,        /**< (i) Reason for interrupt */
   void          *data           /**< (i) User data pointer */
)
{
   struct apm_ch_cfg *ch;
   unsigned short    *ingressp;

   (void) dev;    /* unused */

   ch = data;
   APM_LOG( "isr=%u ch=%i", ch->isrcount_igr, ch->ch );
   APM_PROFILING( "in  [ch=%i]", ch->ch );

   if ( reason != DMA_HANDLER_REASON_BLOCK_COMPLETE )
   {
      ch->errs.dma_igr++;
      return;
   }

   /* Point to buffer index with actual samples */
   ch->active_idx = (ch->active_idx + 1) & 1;
   ingressp       = ch->buf_igr[ch->active_idx].virt;

   ch->isrcount_igr++;

   if ( ch->ramp_check )
   {
      /* Verify against data from 2 frames ago */
      halAudioCompareData( ingressp, ch->buf_egr[ch->active_idx].virt, ch->frame_size/2, &ch->ramp_check_errs, &ch->ramp_check_delay );
   }
   if ( ch->ramp_igr )
   {
      halAudioGenerateRamp( ingressp, &ch->rampseed_igr, ch->frame_size/2, 1 /* mono */ );
   }

   if ( ch->csx_data[HALAUDIO_CSX_POINT_ADC].csx_ops.csxCallback )
   {
      ch->csx_data[HALAUDIO_CSX_POINT_ADC].csx_ops.csxCallback( (char *)ingressp, ch->frame_size, ch->csx_data[HALAUDIO_CSX_POINT_ADC].priv );
   }

   /* Call callback when all ingress processing have completed. Only channels
    * running at sampling frequencies which are wholly divisable by 8 should 
    * call the callback 
    */
   if ( APM_SYNC_FREQ( ch->samp_freq ))
   {
      gApmRefCount++;
      if ( gApmRefCount >= gApmSyncChans )
      {
         if ( gApmElapsedCb )
         {
            (*gApmElapsedCb)( gApmUserData );
         }
         gApmRefCount = 0;
      }
   }
   else
   {
      amxrServiceUnsyncPort( ch->mixer_port );
   }

   APM_PROFILING( "out [ch=%i]", ch->ch );
   APM_LOG( "end" );
}

/***************************************************************************/
/**
*  APM egress DMA interrupt handler
*
*  @return  None
*/
static void apmDmaEgressHandler(
   DMA_Device_t   dev,           /**< (i) Device that triggered callback */
   int            reason,        /**< (i) Reason for interrupt */
   void          *data           /**< (i) User data pointer */
)
{
   struct apm_ch_cfg *ch;

   (void) dev;    /* unused */

   ch = data;
   APM_LOG( "dev=%u reason=%i isr=%u pkts=%i", dev, reason, ch->isrcount_egr, atomic_read( &ch->queued_pkts_egr ));
   APM_PROFILING( "in  [ch=%i]", ch->ch );
   ch->isrcount_egr++;

   if ( reason != DMA_HANDLER_REASON_TRANSFER_COMPLETE )
   {
      ch->errs.dma_egr++;
      return;
   }

   atomic_dec( &ch->queued_pkts_egr ); /* DMA completed */

   if ( atomic_read( &ch->queued_pkts_egr ) > 0 )
   {
      /* More egress packets awaiting to be DMA'd. Start another transfer */
      apmDmaEgressDoTransfer( ch );
   }
   APM_PROFILING( "out [ch=%i]", ch->ch );
   APM_LOG( "end dev=%u", dev );
}

/***************************************************************************/
/**
*  Helper routine to do the egress DMA transfer for an APM channel
*
*  @return  None
*
*  @remark
*     It is expected that egress DMA transfers only occur after ingress
*     DMA transfers have completed.
*
*/
static void apmDmaEgressDoTransfer(
   struct apm_ch_cfg *ch         /** (io) Ptr to APM channel */
)
{
   void *egressp;
   int   rc;
   int   frame_size;

   /* Check that egress ISR is in sync with ingress ISR */
   if ( ch->isrcount_egr != ch->isrcount_igr )
   {
      APM_LOG( "egress sync err: egr=%u igr=%u", ch->isrcount_egr, ch->isrcount_igr );
      ch->errs.dma_sync++;
   }

   egressp     = ch->buf_egr[ch->active_idx].virt;
   frame_size  = ch->frame_size;

   /* Service write requests */
   halAudioWriteService( &ch->write, egressp, frame_size );

   /* Service test and debug facilities */
   if ( ch->ramp_egr )
   {
      halAudioGenerateRamp( egressp, &ch->rampseed_egr, frame_size/2, 1 /* mono */ );
   }
   else if ( ch->sinectl.freq )
   {
      halAudioSine( egressp, &ch->sinectl, frame_size/2, 1 /* mono */ );
   }
   else if ( ch->loop_ig2eg )
   {
      /* software loopback ingress to egress */
      memcpy( egressp, ch->buf_igr[ch->active_idx].virt, frame_size );
   }

   if ( ch->csx_data[HALAUDIO_CSX_POINT_DAC].csx_ops.csxCallback )
   {
      ch->csx_data[HALAUDIO_CSX_POINT_DAC].csx_ops.csxCallback( egressp, frame_size, ch->csx_data[HALAUDIO_CSX_POINT_DAC].priv );
   }

   /* DMA egress samples */
   rc = dma_transfer_to_device( ch->dma_egr.handle,
         ch->buf_egr[ch->active_idx].phys, ch->dma_egr.fifo_addr, frame_size );
   if ( rc )
   {
      ch->errs.dma_egr++;
   }
}

/***************************************************************************/
/**
*  APM proc files initialization
*
*  @return  Nothing
*/
static void apmProcInit( void )
{
   gProcDir = proc_mkdir( APM_PROCDIR_NAME, NULL );
   create_proc_read_entry( APM0_PROC_NAME, 0, gProcDir, apmReadProc, &gApm.ch[0] );
   if ( gApmNumChans >= 2 )
   {
      create_proc_read_entry( APM1_PROC_NAME, 0, gProcDir, apmReadProc, &gApm.ch[1] );
   }
   if ( gApmNumChans == 3 )
   {
      create_proc_read_entry( APM2_PROC_NAME, 0, gProcDir, apmReadProc, &gApm.ch[2] );
   }

   gSysCtlHeader = register_sysctl_table( gSysCtl );
}

/***************************************************************************/
/**
*  APM proc files termination
*
*  @return  Nothing
*/
static void apmProcTerm( void )
{
   remove_proc_entry( APM0_PROC_NAME, gProcDir );
   if ( gApmNumChans >= 2 )
   {
      remove_proc_entry( APM1_PROC_NAME, gProcDir );
   }
   if ( gApmNumChans == 3 )
   {
      remove_proc_entry( APM2_PROC_NAME, gProcDir );
   }
   remove_proc_entry( APM_PROCDIR_NAME, NULL );

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
static int apmReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   int len = 0;
   int16_t igbuf[APM_MAX_DMA_BUFFER_SIZE_BYTES/2];
   int16_t egbuf[APM_MAX_DMA_BUFFER_SIZE_BYTES/2];
   struct apm_ch_cfg *ch;
   unsigned long flags;

   (void) start; (void) offset; (void) count;      /* avoid compiler warning */

   ch = data;
   memset( igbuf, 0, sizeof(igbuf) );
   memset( egbuf, 0, sizeof(egbuf) );

   /* Cache data in critical section */
   local_irq_save( flags );
   memcpy( igbuf, ch->buf_igr[ch->active_idx].virt, ch->frame_size );
   memcpy( egbuf, ch->buf_egr[ch->active_idx].virt, ch->frame_size );
   local_irq_restore( flags );

   len += sprintf( buf+len, "      APM CH%i @ %i Hz, %i bytes, %i us\n",
         ch->ch, ch->samp_freq, ch->frame_size, ch->frame_period );
   len += sprintf( buf+len, "Igr:\n" );
   halAudioPrintMemory( buf, &len, igbuf, ch->frame_size/sizeof(short), 1 /* in hex */, 10 /* line length */,
         2 /* word width */, 0 /* print_addr */, 0 /* addr */ );

   /* Skip egress data if not supported */
   if ( !ch->no_egr )
   {
      len += sprintf( buf+len, "Egr:\n" );
      halAudioPrintMemory( buf, &len, egbuf, ch->frame_size/sizeof(short), 1 /* in hex */, 10 /* line length */,
         2 /* word width */, 0 /* print_addr */, 0 /* addr */ );
   }

   /* Error report and other information */
   if ( !ch->no_egr )
   {
      len += sprintf( buf+len, "Irqs:        igress=%u egress=%u\n", ch->isrcount_igr, ch->isrcount_egr );
      len += sprintf( buf+len, "DMA errors:  igress=%i egress=%i sync=%i\n", ch->errs.dma_igr, ch->errs.dma_egr, ch->errs.dma_sync );
      len += sprintf( buf+len, "APM errors:  dac_underflow=%i adc_overflow=%i\n", ch->errs.dac_underflow, ch->errs.adc_overflow );
   }
   else
   {
      len += sprintf( buf+len, "Irqs:        igress=%u\n", ch->isrcount_igr );
      len += sprintf( buf+len, "DMA errors:  igress=%i\n", ch->errs.dma_igr );
      len += sprintf( buf+len, "APM errors:  adc_overflow=%i\n", ch->errs.adc_overflow );
   }
   if ( ch->ramp_check )
   {
      len += sprintf( buf+len, "Ramp check:  delay=%i errors=%i\n", ch->ramp_check_delay, ch->ramp_check_errs );
   }

   /* Channel parameters */
   if ( !ch->no_egr )
      len += sprintf( buf+len, "Priming:     samples=%i\n", ch->prime_egr/sizeof(short) );

   len += sprintf( buf+len, "Ingress DMA: device=0x%x fifo=0x%x handle=0x%x\n",
         ch->dma_igr.device, ch->dma_igr.fifo_addr, ch->dma_igr.handle );
   if ( !ch->no_egr )
   {
      len += sprintf( buf+len, "Egress DMA:  device=0x%x fifo=0x%x handle=0x%x\n",
            ch->dma_egr.device, ch->dma_egr.fifo_addr, ch->dma_egr.handle );
   }
   len += sprintf( buf+len, "Buffer Igr:  [0] 0x%.8lx (0x%.8x phy)\n"
                            "             [1] 0x%.8lx (0x%.8x phy)\n",
         (unsigned long)ch->buf_igr[0].virt, ch->buf_igr[0].phys, (unsigned long)ch->buf_igr[1].virt, ch->buf_igr[1].phys );
   if ( !ch->no_egr )
   {
      len += sprintf( buf+len, "Buffer Egr:  [0] 0x%.8lx (0x%.8x phy)\n"
                               "             [1] 0x%.8lx (0x%.8x phy)\n",
            (unsigned long)ch->buf_egr[0].virt, ch->buf_egr[0].phys, (unsigned long)ch->buf_egr[1].virt, ch->buf_egr[1].phys );
   }
   len += sprintf( buf+len, "Debug:       running=%i prepared=%i sync_chans=%i\n", 
         atomic_read( &gApm.running ), atomic_read( &gApm.prepared ), gApmSyncChans );

   *eof = 1;
   return len+1;
}

/***************************************************************************/
/**
*  Sysctl callback to handle APM channel loopback
*
*  @return  0 success, otherwise failure
*/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_doApmLoop( ctl_table *table, int write,
      void __user *buffer, size_t *lenp, loff_t *ppos )
#else
static int proc_doApmLoop( ctl_table *table, int write, struct file *filp,
      void __user *buffer, size_t *lenp, loff_t *ppos )
#endif
{
   int                  rc;
   struct apm_ch_cfg   *ch;
   apmHw_LOOPBACK_MODE  mode;
   int                 *loop;

   /* Process integer operation */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
   rc = proc_dointvec( table, write, buffer, lenp, ppos );
#else
   rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );
#endif

   if ( write )
   {
      loop  = table->data;
      ch    = table->extra1;
      mode  = (apmHw_LOOPBACK_MODE)table->extra2;

      if ( *loop )
      {
         ApmHw_LoopBackModeSet( ch->ch, mode );
      }
      else
      {
         ApmHw_LoopBackModeSet( ch->ch, apmHw_LOOPBACK_MODE_NONE );
      }
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
   struct apm_ch_cfg *ch;
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
*  APM ISR for debug use only to detect DMA under and over runs
*
*  @return  irqreturn_t
*/
static irqreturn_t apm_isr(
   int irq,             /**< IRQ number */
   void *dev_id         /**< user data */
)
{
   apmHw_IsrMask status;
   struct apm_ch_cfg *ch;

   ApmHw_InterruptStatusGet( &status );
   ApmHw_InterruptClear( status );

   ch = gApm.ch;

   if ( status & APM_INT_MASK_DAC_UNDERFLOW_A )
   {
      ch[0].errs.dac_underflow++;

#if 0
      /* Skip expected underflow during startup */
      if ( ch[0].errs.dac_underflow > 1 )
      {
         printk( KERN_ERR "APM: DAC A fifo underflow status=0x%x\n", status );
      }
#endif
   }

   if ( status & APM_INT_MASK_DAC_UNDERFLOW_B )
   {
      ch[1].errs.dac_underflow++;

#if 0
      /* Skip expected underflow during startup */
      if ( ch[1].errs.dac_underflow > 1 )
      {
         printk( KERN_ERR "APM: DAC B fifo underflow status=0x%x\n", status );
      }
#endif
   }

   if ( status & APM_INT_MASK_ADC_OVERFLOW_A )
   {
      ch[0].errs.adc_overflow++;
      /*printk( KERN_ERR "APM: ADC A fifo overflow status=0x%x\n", status ); */
   }

   if ( status & APM_INT_MASK_ADC_OVERFLOW_B )
   {
      ch[1].errs.adc_overflow++;
      /*printk( KERN_ERR "APM: ADC B fifo overflow status=0x%x\n", status ); */
   }

   if ( status & APM_INT_MASK_ADC_OVERFLOW_C )
   {
      ch[2].errs.adc_overflow++;
      /*printk( KERN_ERR "APM: ADC C fifo overflow status=0x%x\n", status ); */
   }


   if ( status & APM_INT_MASK_MIC_A_IN )
   {
      if ( gApmMicDet.mic_in )
      {
         gApmMicDet.mic_in( APM_MICDET_A );
      }
   }

   if ( status & APM_INT_MASK_MIC_A_ON )
   {
      if ( gApmMicDet.mic_on )
      {
         gApmMicDet.mic_on( APM_MICDET_A );
      }
   }

   if ( status & APM_INT_MASK_MIC_B_IN )
   {
      if ( gApmMicDet.mic_in )
      {
         gApmMicDet.mic_in( APM_MICDET_B );
      }
   }

   if ( status & APM_INT_MASK_MIC_B_ON )
   {
      if ( gApmMicDet.mic_on )
      {
         gApmMicDet.mic_on( APM_MICDET_B );
      }
   }

   return IRQ_HANDLED;
}

/***************************************************************************/
/**
*  APM mixer callback for outgoing data (i.e. ingress)
*
*  @return
*     NULL     - non-matching frame size or non-existent buffer
*     ptr      - pointer to ingress buffer
*/
static int16_t *apmMixerCb_IngressGetPtr(
   int   numBytes,            /**< (i) frame size in bytes */
   void *privdata             /**< (i) private data */
)
{
   struct apm_ch_cfg *ch;
   int16_t           *ptr;

   ch  = (struct apm_ch_cfg *)privdata;
   ptr = NULL;
   if ( numBytes == ch->frame_size )
   {
      ptr = ch->buf_igr[ch->active_idx].virt;
   }

   /*APM_LOG( "numBytes=%i ptr=0x%lx", numBytes, (unsigned long)ptr ); */

   return ptr;
}

/***************************************************************************/
/**
*  APM mixer callback for incoming data (i.e. egress)
*
*  @return
*     NULL     - non-matching frame size or non-existent buffer
*     ptr      - pointer to egress buffer
*/
static int16_t *apmMixerCb_EgressGetPtr(
   int   numBytes,            /**< (i) frame size in bytes */
   void *privdata             /**< (i) private data */
)
{
   struct apm_ch_cfg *ch;
   int16_t           *ptr;

   ch  = (struct apm_ch_cfg *)privdata;
   ptr = NULL;
   if ( numBytes == ch->frame_size )
   {
      ptr = ch->buf_egr[ch->active_idx].virt;
   }

   /*APM_LOG( "numBytes=%i ptr=0x%lx", numBytes, (unsigned long)ptr ); */

   return ptr;
}

/***************************************************************************/
/**
*  APM mixer callback to indicate that the egress data has been deposited.
*
*  @return     None
*
*  @remark
*     This callback is used as a trigger to DMA more data to the DAC, if
*     appropriate.
*/
static void apmMixerCb_EgressDone(
   int   numBytes,            /**< (i) frame size in bytes */
   void *privdata             /**< (i) private data */
)
{
   struct apm_ch_cfg *ch;

   ch  = (struct apm_ch_cfg *)privdata;

   /*APM_LOG( "APM [ch %i] pkts=%i bytes=%i running=%i", ch->ch, atomic_read( &ch->queued_pkts_egr ), numBytes, atomic_read( &gApm.running )); */

   /* Take action only when APM channels are actually running */
   if ( atomic_read( &gApm.running ))
   {
      /* new packet arrived */
      atomic_inc( &ch->queued_pkts_egr );

      /* Exactly 1 packet awaits, thus DMA was idle. Start a new transfer right away. */
      if ( atomic_read( &ch->queued_pkts_egr ) == 1 )
      {
         apmDmaEgressDoTransfer( ch );
      }
   }
}

/***************************************************************************/
/**
*  APM mixer callback to flush the egress buffers when the last destination
*  connection is removed.
*
*  @return     None
*/
static void apmMixerCb_EgressFlush(
   void *privdata             /*<< (i) private data */
)
{
   struct apm_ch_cfg *ch;
   unsigned long flags;

   ch  = (struct apm_ch_cfg *)privdata;

   /*APM_LOG( "APM [ch %i]", ch->ch ); */

   /* Clear double buffers */
   local_irq_save( flags );
   memset( ch->buf_egr[0].virt, 0, APM_MAX_DMA_BUFFER_SIZE_BYTES );
   memset( ch->buf_egr[1].virt, 0, APM_MAX_DMA_BUFFER_SIZE_BYTES );
   local_irq_restore( flags );
}

/***************************************************************************/
/**
*  APM mixer port registration
*
*  @return  None
*/
static int apmMixerPortsRegister( void )
{
   struct apm_ch_cfg   *ch;
   int                  i, err;
   AMXR_PORT_CB         cb;
   AMXR_PORT_ID         portid;
   char                 name[32];

   ch = gApm.ch;
   for ( i = 0; i < gApmNumChans; i++, ch++ )
   {
      memset( &cb, 0, sizeof(cb) );

      cb.getsrc      = apmMixerCb_IngressGetPtr;
      if ( !ch->no_egr )
      {
         cb.getdst         = apmMixerCb_EgressGetPtr;
         cb.dstdone        = apmMixerCb_EgressDone;
         cb.dstcnxsremoved = apmMixerCb_EgressFlush;
      }

      sprintf( name, "halaudio.apm%i", ch->ch );

      err = amxrCreatePort( name, &cb, ch /* privdata */,
            ch->no_egr ? 0 : APM_DEFAULT_SAMP_FREQ, 
            ch->no_egr ? 0 : 1 /* mono */, 
            ch->no_egr ? 0 : APM_DEFAULT_FRAME_SIZE,
            APM_DEFAULT_SAMP_FREQ, 1 /* mono src */, APM_DEFAULT_FRAME_SIZE,
            &portid );
      if ( err )
      {
         printk( KERN_ERR "%s: failed to create mixer port %i err=%i\n", __FUNCTION__, i, err );
         return err;
      }

      ch->mixer_port = portid;
   }
   return 0;
}

/***************************************************************************/
/**
*  APM mixer port de-registration
*
*  @return  None
*/
static int apmMixerPortsDeregister( void )
{
   struct apm_ch_cfg  *ch;
   int                 i, err, rc;

   ch = gApm.ch;
   rc = 0;
   for ( i = 0; i < gApmNumChans; i++, ch++ )
   {
      err = amxrRemovePort( ch->mixer_port );
      if ( err )
      {
         printk( KERN_ERR "APM [ch=%d]: failed to deregister mixer port err=%i\n", ch->ch, err );
         rc = err;
      }
      ch->mixer_port = NULL;  /* invalidate handle */
   }

   return rc;
}

/***************************************************************************/
/**
*  Set frequency for mixer port
*
*  @return  None
*/
static int apmMixerSetFreq(
   AMXR_PORT_ID   portid,     /**< (i) mixer port handle */
   int            freq,       /**< (i) sampling frequency to set */
   int            bytes       /**< (i) period in bytes */
)
{
   return amxrSetPortFreq( portid, freq, bytes, freq, bytes );
}

/***************************************************************************/
/**
*  Set microphone detection callback operations.
*/
void apmSetMicDetectOps(
   APM_MIC_DET_OPS *ops       /*<< (i) Ptr to callbacks. If NULL, reset callbacks */
)
{
   unsigned long flags;
   if ( ops )
   {
      local_irq_save( flags );
      memcpy( &gApmMicDet, ops, sizeof(gApmMicDet) );
      local_irq_restore( flags );
   }
   else
   {
      local_irq_save( flags );
      memset( &gApmMicDet, 0, sizeof(gApmMicDet) );
      local_irq_restore( flags );
   }
}

/***************************************************************************/
/**
*  Hardware equalizers are enabled by default to avoid having to ever
*  reset equalizer configuration. Resetting equalizer configuration causes
*  timing glitches which when accumulated may cause synchronization problems 
*  between APM channels. A "disabled" equalizer is equivalent to configuring 
*  with an impulse response.
*/
static int apmEquInit( void )
{
   struct apm_ch_cfg *ch;
   int i;

   init_MUTEX( &gEquCoeffsMutex );
   memset( gEquCoeffs, 0, sizeof(gEquCoeffs) );

   /* Impulse response */
   gEquCoeffs[0] = 0x7FFF;

   ch = gApm.ch;
   for ( i = 0; i < gApmNumChans; i++, ch++ )
   {
      ApmHw_WriteCoeff( ch->ch, apmHw_COEFF_TX_EQ, gEquCoeffs, HALAUDIO_EQU_COEFS_MAX_NUM );
      ApmHW_EquEnable( ch->ch, apmHw_DIR_TX, 1 /* enable */ );

      if ( !ch->no_egr )
      {
         ApmHw_WriteCoeff( ch->ch, apmHw_COEFF_RX_EQ, gEquCoeffs, HALAUDIO_EQU_COEFS_MAX_NUM );
         ApmHW_EquEnable( ch->ch, apmHw_DIR_RX, 1 /* enable */ );
      }
   }

   return 0;
}

/***************************************************************************/
/**
*  Driver initialization called when module loaded by kernel
*
*  @return
*     0              Success
*     -ve            Error code
*/
static int __init halaudio_init( void )
{
   int err;

   printk( KERN_INFO "HAL Audio APM Driver: 1.0. Built %s %s\n", __DATE__, __TIME__ );

   gApmNumChans   = CAP_APM_MAX_NUM_CHANS;
   gApmSyncChans  = gApmNumChans;

   err = halAudioAddInterface( &halaudio_apm_ops, gApmNumChans, "APM",
         APM_DEFAULT_FRAME_PERIOD, 1 /* synchronize */, &gInterfHandle );
   if ( err )
   {
      printk( KERN_ERR "APM: failed to install the audio interface %d!\n", err );
      return err;
   }

   err = apmHeadsetInit();
   if ( err )
   {
      return err;
   }

   return 0;
}

/***************************************************************************/
/**
*  Driver destructor routine. Frees all resources
*/
static void __exit halaudio_exit( void )
{
   halAudioDelInterface( gInterfHandle );

   apmHeadsetExit();
}

module_init( halaudio_init );
module_exit( halaudio_exit );

MODULE_AUTHOR( "Broadcom" );
MODULE_DESCRIPTION( "HAL Audio BCMRING APM Driver" );
MODULE_LICENSE( "GPL v2" );

/** @} */
