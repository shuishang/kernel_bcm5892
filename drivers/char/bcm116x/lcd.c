/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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
*  lcd.c
*
*  PURPOSE:
*
*   This implements the LCD driver.
*
*   In particular, it's designed to drive the HYeLCD TG176220-T003DA
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/string.h>
#include <linux/module.h>

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/interrupt.h>

#include <linux/version.h>
#include <linux/types.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/kernel_stat.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/dma-mapping.h>
#include <mach/dma.h>

#include <asm/irq.h>
#include <mach/reg_lcd.h>
#include <mach/reg_gpio.h>
#include <mach/reg_irq.h>
#include <mach/reg_dma.h>
#include <mach/reg_sys.h>
#include <linux/broadcom/regaccess.h>
#if defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
#include <linux/broadcom/idle_prof.h>
#endif
#include <linux/broadcom/cpu_sleep.h>
#include <asm/mach/irq.h>
#include <asm/io.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/hw_cfg.h>
#include <linux/broadcom/lcd.h>
#include <linux/broadcom/PowerManager.h>
#include <linux/broadcom/lcd_backlight.h>


/*
 * ---- Public Variables ------------------------------------------------- 
 * ---- Private Constants and Types -------------------------------------- 
 */

/*
 * Attributes of the LCD
 */
#define LCD_WIDTH               176
#define LCD_HEIGHT              220
#define LCD_BITS_PER_PIXEL      16
#define LCD_FRAME_BUFFER_BYTES  ( LCD_WIDTH * LCD_HEIGHT * LCD_BITS_PER_PIXEL / 8 )

#define ALIGN_UP(value, alignment) (((value)+((alignment)-1)) & ~((alignment)-1))

#define BITS_TO_UINT32S( bits )  ( ALIGN_UP( bits, 32 ) / 32 )
#define BITS_TO_BYTES( bits )    ( BITS_TO_UINT32S( bits ) * sizeof( uint32_t ))

/*
 * The + 1 is so that we can be guaranteed to be able to mark the imaginary 
 * row just beyond the bottom of the display as clean. 
 */

uint32_t gDirtyRowBits[ BITS_TO_UINT32S( LCD_HEIGHT + 1 )];

#define  DIRTY_WORD_IDX( row )   ( (row) >> 5 )             /* Divide by 32 */
#define  DIRTY_WORD_MASK( row )  ( 1 << ( (row) & 0x1f ))   /* Module 32 */

#define  IS_ROW_DIRTY( bits, row )   ( (((uint32_t *)bits)[ DIRTY_WORD_IDX( row ) ] &   DIRTY_WORD_MASK( row )) != 0 )
#define  MARK_ROW_DIRTY( bits, row )   (((uint32_t *)bits)[ DIRTY_WORD_IDX( row ) ] |=  DIRTY_WORD_MASK( row ))
#define  MARK_ROW_CLEAN( bits, row )   (((uint32_t *)bits)[ DIRTY_WORD_IDX( row ) ] &= ~DIRTY_WORD_MASK( row ))


#define USE_DMA     1       /* 1 = use DMA to LCD */
#if 0
#   define LCD_PUTS(str)           printk( "%s: %s", __FUNCTION__, str )
#   define LCD_DEBUG(fmt, args...) printk( "%s: " fmt, __FUNCTION__ , ## args )
#else
#   define LCD_PUTS(str)
#   define LCD_DEBUG(fmt, args...)
#endif

#if 0
#   define LCD_DEBUG_REG(fmt, args...) printk( "%s: " fmt, __FUNCTION__ , ## args )
#else
#   define LCD_DEBUG_REG(fmt, args...)
#endif


/*
 * Definitions for the LCD initialization sequence
 */

#define LCD_CMD_END             0
#define LCD_CMD_SET_REG         1
#define LCD_CMD_WAIT_USEC       2
#define LCD_CMD_WAIT_MSEC       3
#define LCD_CMD_LINE_CALIBRATE  4

typedef struct
{
    u8      cmd;
    u8      reg;
    u8      val;
    u8      pad;

} lcd_cmd_t;

#define LCD_END_SEQUENCE            { LCD_CMD_END }
#define LCD_SET_REG( reg,  val )    { LCD_CMD_SET_REG, (reg), (val) }
#define LCD_WAIT_USEC(usec)         { LCD_CMD_WAIT_USEC, 0, (usec) }
#define LCD_WAIT_MSEC(msec)         { LCD_CMD_WAIT_MSEC, 0, (msec) }

#define LCD_SET_REG_WAIT_USEC( reg,  val, usec ) \
    LCD_SET_REG( (reg), (val) ),                 \
    LCD_WAIT_USEC( usec )

#define LCD_SET_REG_WAIT_MSEC( reg,  val, msec ) \
    LCD_SET_REG( (reg), (val) ),                 \
    LCD_WAIT_MSEC( msec )

static inline void lcd_write_cmd( u8 reg, u8 val )
{
    u32 cmd = ( (u32)(reg) << 8 ) | (u32)(val);

    LCD_DEBUG_REG( "0x%04x\n", ( (u32)(reg) << 8 ) | (u32)(val)); \

    REG_LCD_CMDR = cmd;
}


#define LCD_INIT_TEST_TIME()
#define LCD_START_TEST_TIME()
#define LCD_STOP_TEST_TIME()

#define RED_MASK    0xF800
#define GREEN_MASK  0x07C0
#define BLUE_MASK   0x001F

#define RED_SHIFT   11
#define GREEN_SHIFT 5
#define BLUE_SHIFT  0

#define RGB_VAL( r, g, b )  (( (r) << RED_SHIFT ) | ( (g) << GREEN_SHIFT ) | ( (b) << BLUE_SHIFT ))

#define DESIRED_DMA_CHANNEL 1   /* desired DMA channel: 1 is second highest priority */

#define DMA_CONTROL                             \
    ( REG_DMA_CHAN_CTL_SRC_INCR             \
    | REG_DMA_CHAN_CTL_DEST_WIDTH_32        \
    | REG_DMA_CHAN_CTL_SRC_WIDTH_32         \
    | REG_DMA_CHAN_CTL_DEST_BURST_SIZE_4    \
    | REG_DMA_CHAN_CTL_SRC_BURST_SIZE_64    \
    | (LCD_WIDTH/2))

#define DMA_CONFIG                                  \
    ( REG_DMA_CHAN_CFG_TC_INT_ENABLE            \
    | REG_DMA_CHAN_CFG_ERROR_INT_ENABLE         \
    | REG_DMA_CHAN_CFG_FLOW_CTL_MEM_TO_PRF_DMA  \
    | (REG_DMA_PERIPHERAL_LCD << REG_DMA_CHAN_CFG_DEST_PERIPHERAL_SHIFT) \
    | REG_DMA_CHAN_CFG_ENABLE)

#define BYTES_PER_PIXEL     2       /* always 16 bits per pixel, so two bytes */
#define BYTES_PER_LLI       16      /* number of bytes per Linked List Item (LLI) */

/* swap from big endian to little endian for 32 bits */
#define SWAP(val) \
 ((uint32_t)( \
  (((uint32_t)(val) & (uint32_t)0x000000ffUL) << 24) | \
  (((uint32_t)(val) & (uint32_t)0x0000ff00UL) <<  8) | \
  (((uint32_t)(val) & (uint32_t)0x00ff0000UL) >>  8) | \
  (((uint32_t)(val) & (uint32_t)0xff000000UL) >> 24) ))

#if USE_DMA
typedef struct
{
    size_t      sizeInBytes;
    void       *virtPtr;
    dma_addr_t  physPtr;

} LCD_DmaBuffer_t;

/* definition of DMA Linked List Item (LLI) */
typedef struct
{
    u32     source;     /* source address */
    u32     dest;       /* dest address */
    u32     link;       /* link to next LLI */
    u32     control;    /* control word */

} LCD_DmaLLI_t;

/* structure for restoring overwritten LLI information */
typedef struct
{
    int     valid;      /* flag to show data is valid */
    int     row;        /* row to store */
    u32     link;       /* link register */
    u32     control;    /* control register */

} LCD_DmaRestore_t;

static int gDmaChannel;
static LCD_DmaBuffer_t  gDmaLinkedList;
static LCD_DmaRestore_t gDmaSavedLli;
#endif

typedef struct
{
    size_t      sizeInBytes;
    void       *virtPtr;
    dma_addr_t  physPtr;

} LCD_FrameBuffer_t;

LCD_FrameBuffer_t   gFrameBuffer;
int                 gFrameBufferAllocated = 0;

/* ---- Private Variables ------------------------------------------------ */

static char gBanner[] __initdata = KERN_INFO "BCM116x LCD Driver: 0.02\n";

/*
 * The following sequence is used to initialize the controller for the first time.
 */

#define LCD_REG_X_POS   6
#define LCD_REG_Y_POS   7

static lcd_cmd_t   gInitCmd[] =
{
    LCD_SET_REG(   3,    1 ),   /* Reset the uPD161621 */
    LCD_SET_REG(  34,    1 ),   /* Reset the uPD161644 */

    LCD_WAIT_USEC( 250 ),

    /* The following setup the gamma curve correction power supply circuit */

    LCD_SET_REG(  97,    0 ),
    LCD_SET_REG(  98, 0x09 ),
    LCD_SET_REG(  99, 0x34 ),
    LCD_SET_REG( 100, 0x30 ),
    LCD_SET_REG( 102, 0x00 ),
    LCD_SET_REG( 103, 0x09 ),
    LCD_SET_REG( 104, 0x06 ),
    LCD_SET_REG( 105, 0x24 ),

    LCD_SET_REG(  13,    1 ), /* Select 220 gate outputs */
    LCD_SET_REG(  40,    0 ), /* Pre-charge direction */
    LCD_SET_REG(  41,    0 ), /* Pre-charge discharge */
    LCD_SET_REG(  80, 0x40 ), /* VCOM control register */
    LCD_SET_REG(  82, 0x37 ), /* Output stage capacity */
    LCD_SET_REG(  83, 0x00 ), /* gamma reference voltage generator */
    LCD_SET_REG(  84, 0x00 ), /* VCOM control register */
    LCD_SET_REG( 114, 0x12 ), /* Operational Amplifier */
    LCD_SET_REG( 121, 0x20 ), /* pre-charge period */

    { LCD_CMD_LINE_CALIBRATE },

    LCD_SET_REG(   9,  LCD_WIDTH - 1 ),  /* Max value of X reg in window access mode */
    LCD_SET_REG(   8,    0 ),            /* Min value of X reg in window access mode */
    LCD_SET_REG(  11,  LCD_HEIGHT - 1 ), /* Max value of Y reg in window access mode */
    LCD_SET_REG(  10,    0 ),            /* Min value of Y reg in window access mode */
    LCD_SET_REG(   5, 0x10 ),            /* Sets Window Access Mode */

    LCD_SET_REG( LCD_REG_X_POS, 0 ), /* X ddrees of Display RAM */
    LCD_SET_REG( LCD_REG_Y_POS, 0 ), /* Y address of Display RAM */

    LCD_SET_REG_WAIT_USEC(  25, 0x16, 250 ), /* Power Supply Control Register 2 */
    LCD_SET_REG_WAIT_USEC(  26, 0x25, 250 ), /* Power Supply Control Register 3 */
    LCD_SET_REG_WAIT_USEC(  27, 0x6A, 250 ), /* Power Supply Control Register 4 */
    LCD_SET_REG_WAIT_USEC(  28, 0x00, 250 ), /* Power Supply Control Register 5 */
    LCD_SET_REG_WAIT_USEC(  29, 0x07, 250 ), /* Power Supply Control Register 6 */
    LCD_SET_REG_WAIT_USEC(  30, 0x03, 250 ), /* Power Supply Control Register 7 */
    LCD_SET_REG_WAIT_USEC(  31, 0x00, 250 ), /* Power Supply Control Register 8 */
    LCD_SET_REG_WAIT_USEC(  32, 0x00, 250 ), /* Power Supply Control Register 9 */
    LCD_SET_REG_WAIT_USEC(  33, 0x2F, 250 ), /* Power Supply Control Register 10 */
    LCD_SET_REG_WAIT_USEC(  24, 0x5E, 250 ), /* Power Supply Control Register 1 */
    LCD_SET_REG_WAIT_USEC(  27, 0x6B, 250 ), /* Power Supply Control Register 4 */
    LCD_SET_REG(  24, 0x5F ), /* Power Supply Control Register 1 */
    LCD_WAIT_MSEC( 200 ),
    LCD_SET_REG(  55, 0x01 ), /* Gate Scan operation select */
    LCD_SET_REG(  59, 0x01 ), /* GOE1 output control */
    LCD_WAIT_MSEC( 17 ),
    LCD_SET_REG(   0, 0x20 ), /* Control Register 1 */

    LCD_END_SEQUENCE
};

static lcd_cmd_t    gPwrOffCmd[] =
{
    LCD_SET_REG_WAIT_MSEC(  0, 0xA0, 17 ),
    LCD_SET_REG_WAIT_MSEC(  0, 0xA8, 17 ),  /* go to standby, amplifiers off */
    LCD_SET_REG_WAIT_USEC( 24, 0x5E, 250 ), /* turn off internal DC/DC converter */
    LCD_SET_REG( 1, 1 ), /* oscillator stop */

    LCD_END_SEQUENCE
};

static  int     gInitialized = 0;
spinlock_t      gLcdUpdateLock = SPIN_LOCK_UNLOCKED;
static  long    gUpdateThreadPid = 0;
struct  completion gUpdateExited;
struct  semaphore gUpdateSem;
struct  semaphore gDmaSem;
LCD_DirtyRows_t gDirtyRows = { LCD_HEIGHT, 0 };

#if defined( CONFIG_BCM_LCD_PERF )

static  int                 gLcdPerf        = 1;
static  int                 gLcdPerfFreq    = 50;
static  struct timer_list   gLcdPerfTimer;

typedef struct
{
#if !defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
    unsigned int    jiffies;
    unsigned int    user;
    unsigned int    nice;
    unsigned int    system;
    unsigned int    busy;
#else
    idle_handle_t   idle_handle;
    u32 idle;
    u32 total;
#endif
} LcdPerfData;

static  LcdPerfData                 gLcdPerfCurrData;
#if !defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
static  LcdPerfData                 gLcdPerfPrevData;
#endif
static  unsigned int                gLcdPerfPeakBusy = 0;

static  struct ctl_table_header    *gSysCtlHeader;

static int lcd_proc_sysctl_perf(ctl_table *table, int write, struct file *filp,
             void __user *buffer, size_t *lenp, loff_t *ppos );

static struct ctl_table gSysCtlLcd[] = {
   {
      .ctl_name         = BCM_SYSCTL_LCD_PERF,
      .procname         = "perf",
      .data             = &gLcdPerf,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &lcd_proc_sysctl_perf
   },
   {
      .ctl_name         = BCM_SYSCTL_LCD_PERF_FREQ,
      .procname         = "perf_freq",
      .data             = &gLcdPerfFreq,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &proc_dointvec
   },
   {
      .ctl_name         = BCM_SYSCTL_LCD_PERF_PEAKBUSY,
      .procname         = "perf_peak_busy",
      .data             = &gLcdPerfPeakBusy,
      .maxlen           = sizeof( int ),
      .mode             = 0644,
      .proc_handler     = &proc_dointvec
   },
   {}
};
static struct ctl_table gSysCtl[] = {
   {
      .ctl_name         = CTL_BCM_LCD,
      .procname         = "lcd",
      .mode             = 0555,
      .child            = gSysCtlLcd
   },
   {}
};

#endif  /* CONFIG_BCM_LCD_PERF */

static int lcd_pm_update (PM_CompPowerLevel compPowerLevel, PM_PowerLevel sysPowerLevel);

static PM_Comp_Ops_t lcd_pm_ops =
{
   update_power_level: &lcd_pm_update,
};

/* ---- Private Function Prototypes -------------------------------------- */

static void     lcd_color_test( int num );

#if defined( CONFIG_BCM_LCD_PERF )
static void     lcd_draw_perf( void );
static void     lcd_clear_perf( void );
static void     lcd_perf_timer( unsigned long dummy );
#endif

static void     lcd_enable_sub_backlight( int level );
static void     lcd_enable_cs( int level );
static void     lcd_exit( void );
static void     lcd_fill_rect( LCD_Rect_t *r );
static int      lcd_init( void );
static int      lcd_pwr_on( void );
static void     lcd_init_all( void );
static void     lcd_init_controller( void );
static void     lcd_pwr_off_controller( void );
static int      lcd_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg );
static int      lcd_mmap( struct file *file, struct vm_area_struct * vma );
static int      lcd_open( struct inode *inode, struct file *file );
static void     lcd_print_data( unsigned int x, unsigned int y, unsigned int numPixels );
static void     lcd_print_regs( unsigned char startReg, unsigned char endReg );
static int      lcd_release( struct inode *inode, struct file *file );
static void     lcd_reset_controller( int level );
static void     lcd_scope_timeout( int level );
static int      lcd_send_cmd_sequence( lcd_cmd_t *cmdSeq );
#if USE_DMA
static void     lcd_start_dma( unsigned int top, unsigned int bottom );
static          irqreturn_t lcd_dma_isr( void *unused );
static int      lcd_dmaCreateList( void );
#endif

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations lcd_fops =
{
    owner:      THIS_MODULE,
    ioctl:      lcd_ioctl,
    mmap:       lcd_mmap,
    open:       lcd_open,
    release:    lcd_release,
};

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  lcd_color_test
*
*   Sends some predefined pattern to the LCD
*
***************************************************************************/

/* static */
void lcd_color_test( int num )
{
    unsigned int    x;
    unsigned int    y;
    LCD_DirtyRows_t dirtyRows;
    u16            *fbPtr;

    LCD_DEBUG( "%d\n", num );

    dirtyRows.top = 0;
    dirtyRows.bottom = LCD_HEIGHT - 1;

    fbPtr = gFrameBuffer.virtPtr;

    switch ( num )
    {
        case 0: /* Clear to black */
        {

            for ( y = 0; y < LCD_HEIGHT; y++ )
            {
                for ( x = 0; x < LCD_WIDTH; x++ )
                {
                    *fbPtr++ = 0;
                }
            }
            break;
        }

        case 1: /* Color Wedges */
        {
    #define R   RGB_VAL( 1, 0, 0 )
    #define G   RGB_VAL( 0, 2, 0 ) /* We want 32 values, so we double the green value */
    #define B   RGB_VAL( 0, 0, 1 )

            typedef struct
            {
                unsigned int numRows;
                unsigned int incr;

            } pattern_t;

            pattern_t   pattern[] = {{ 30, R },   { 30, G },   { 30, B },
                                     { 30, R|G }, { 30, R|B }, { 30, G|B },
                                     { 30, R|G|B },
                                     { 10, 0 }, { 0 }};

            pattern_t   *p = &pattern[ 0 ];

            for ( y = 0; y < LCD_HEIGHT; p++ )
            {
                unsigned int r;

                for ( r = 0; r < p->numRows; r++, y++ )
                {
                    u16 color = 0;
                    int col1, col2;

                    for ( col1 = 0; col1 < 8; col1++ )
                    {
                        *fbPtr++ = 0;
                    }
                    for ( col1 = 0; col1 < 32; col1++ )
                    {
                        /* 5 * 32 = 160 */

                        for ( col2 = 0; col2 < 5; col2++ )
                        {
                            *fbPtr++ = color;
                        }

                        color += p->incr;
                    }
                    for ( col1 = 0; col1 < 8; col1++ )
                    {
                        *fbPtr++ = 0;
                    }
                }
            }
            break;
        }

        case 3: /* Bit Test */
        {
            /*
             * Here we go directly to the interface rather than through the 
             * frame buffer so we can test all 18 bits. 
             */

            lcd_write_cmd( LCD_REG_X_POS, 0 );
            lcd_write_cmd( LCD_REG_Y_POS, 0 );

            REG_LCD_DATR = 0;

            for ( x = 0; x < 18; x++ )
            {
                REG_LCD_DATR = ( 1 << x );
            }

            for ( x = 0; x < 18; x++ )
            {
                REG_LCD_DATR = ~( 1 << x );
            }

            REG_LCD_DATR = 0xFFFFFFFF;
            return;
        }

        case 4: /* Dump Some memory */
        {
            printk( "fbPtr = 0x%08lx\n", (long)fbPtr );

            for ( y = 0; y < 4; y++ )
            {
                printk( "Line: %d ", y );

                for ( x = 0; x < 14; x++ )
                {
                    printk( "%04x ", fbPtr[x] );
                }
                fbPtr += LCD_WIDTH;
                printk( "\n" );
            }
            break;
        }
    }

    lcd_dirty_rows( &dirtyRows );

} /* lcd_color_test */

/****************************************************************************
*
*  lcd_copyarea
*
*   Marks the rows in the copyarea dirty so they can be transferred later 
*
***************************************************************************/
void lcd_copyarea( const struct fb_copyarea *area )
{
    int row;
    unsigned long flags;

    LCD_DEBUG( "sx=%u, sy=%u, dx=%u, dy=%u, width=%u, height=%u\n",
               area->sx, area->sy, area->dx, area->dy,
               area->width, area->height );

    spin_lock_irqsave( &gLcdUpdateLock, flags );

    for ( row = area->dy; row <= area->dy + area->height - 1; row++ )
    {
        /*MARK_ROW_DIRTY( gDirtyRowBits, row ); */
    }

    spin_unlock_irqrestore( &gLcdUpdateLock, flags );
} /* lcd_copyarea */

/****************************************************************************
*
*  lcd_fillrect
*
*   Marks the rows in the fillrect dirty so they can be transferred later 
*
***************************************************************************/
void lcd_fillrect( LCD_FillRectColor_t *rect_c )
{
    int row;
    unsigned long flags;

    LCD_DEBUG( "dx=%u, dy=%u, width=%u, height=%u, rawColor=%u\n",
               rect_c->dx, rect_c->dy, rect_c->width, rect_c->height,
               rect_c->rawColor );

    spin_lock_irqsave( &gLcdUpdateLock, flags );

    for ( row = rect_c->dy; row <= rect_c->dy + rect_c->height - 1; row++ )
    {
        /*MARK_ROW_DIRTY( gDirtyRowBits, row ); */
    }

    spin_unlock_irqrestore( &gLcdUpdateLock, flags );
} /* lcd_fillrect */

/****************************************************************************
*
*  lcd_dirty_rows
*
*   Marks the indicated rows as dirty and arranges for them to be transferred
*   to the LCD.
*
***************************************************************************/

/* static */
void lcd_dirty_rows( LCD_DirtyRows_t *dirtyRows )
{
    int                 row;
    unsigned long       flags;

    LCD_DEBUG( "top = %u,  bottom = %u\n", dirtyRows->top, dirtyRows->bottom );

    if (( dirtyRows->top > dirtyRows->bottom )
    ||  ( dirtyRows->bottom >= LCD_HEIGHT ))
    {
        LCD_DEBUG( "invalid params - ignoring\n" );
        LCD_DEBUG( "top = %u,  bottom = %u\n", dirtyRows->top, dirtyRows->bottom );

        return;
    }

    spin_lock_irqsave( &gLcdUpdateLock, flags );

    for ( row = dirtyRows->top; row <= dirtyRows->bottom; row++ )
    {
        MARK_ROW_DIRTY( gDirtyRowBits, row );
    }
    up( &gUpdateSem );
    spin_unlock_irqrestore( &gLcdUpdateLock, flags );

} /* lcd_dirty_rows */

/****************************************************************************
*
*  lcd_dirty_row_bits
*
*   Marks the indicated rows as dirty and arranges for them to be transferred
*   to the LCD.
*
***************************************************************************/

void lcd_dirty_row_bits( LCD_DirtyRowBits_t *dirtyRowBits )
{
    int             i;
    int             numWords;
    unsigned long   flags;

    /* Mark all of the indicated rows as dirty, and then update the videocore. */
    
    if ( dirtyRowBits->numRows > LCD_HEIGHT )
    {
        dirtyRowBits->numRows = LCD_HEIGHT;
    }
    numWords = BITS_TO_UINT32S( dirtyRowBits->numRows );
    
    spin_lock_irqsave( &gLcdUpdateLock, flags );
    for ( i = 0; i < numWords; i++ ) 
    {
        gDirtyRowBits[ i ] |= dirtyRowBits->bits[ i ];
    }
    up( &gUpdateSem );
    spin_unlock_irqrestore( &gLcdUpdateLock, flags );

} /* lcd_dirty_row_bits */

/****************************************************************************
*
*  lcd_update_thread
*
*   Worker thread to transfer data to the LCD.
*
***************************************************************************/
static
int lcd_update_thread(void *data)
{
    uint32_t dirtyRowBits[ BITS_TO_UINT32S( LCD_HEIGHT + 1 )];

    /* This thread doesn't need any user-level access,
     * so get rid of all our resources
     */
    daemonize("lcdUpdate");

    /* Run until signal received */
    while ( 1 )
    {
        if ( down_interruptible( &gUpdateSem ) == 0 )
        {
            int             row;
            int             dirty;
            unsigned long   flags;
            LCD_DirtyRows_t dirtyRows = { 0, 0 };

            if ( !gInitialized )
            {
                /* Nothing we can do - ignore the update request. */

                continue;
            }

            spin_lock_irqsave( &gLcdUpdateLock, flags );
            memcpy( dirtyRowBits, gDirtyRowBits, sizeof( dirtyRowBits ));
            memset( gDirtyRowBits, 0, sizeof( gDirtyRowBits ));
            spin_unlock_irqrestore( &gLcdUpdateLock, flags );


            /*
             * We need to update the display. Since we can only DMA contiguous 
             * regions of bytes, we walk through the bits and figure out the 
             * contiguous regions 
             */

            /*
             * Mark the "imaginary row" which beyond the bottom as clean. 
             * This will force a dirty section that includes the bottom line 
             * to be updated. 
             */

            MARK_ROW_CLEAN( dirtyRowBits, LCD_HEIGHT );
            dirty = 0;

            for ( row = 0; row <= LCD_HEIGHT; row++ ) 
            {
                if ( IS_ROW_DIRTY( dirtyRowBits, row ))
                {
                    if ( !dirty )
                    {
                        /* This is the first row in a region of dirty rows */

                        dirty = 1;
                        dirtyRows.top = row;
                    }
                }
                else
                {
                    if ( dirty )
                    {
                        /*
                         * The previous row is the last row in a region of 
                         * dirty rows. 
                         */

                        dirty = 0;
                        dirtyRows.bottom = row - 1;

#if USE_DMA
                        if ( down_interruptible( &gDmaSem ) == 0 )
                        {
                            lcd_write_cmd( LCD_REG_X_POS, 0 );
                            lcd_write_cmd( LCD_REG_Y_POS, dirtyRows.top );

                            /* gDmaSem will be released when the DMA completes. */

                            lcd_start_dma( dirtyRows.top, dirtyRows.bottom );
                        }
                        else
                        {
                            break;
                        }
#else
                        {
                            unsigned int    row;
                            unsigned int    col;
                            u16            *p;

                            lcd_write_cmd( LCD_REG_X_POS, 0 );
                            lcd_write_cmd( LCD_REG_Y_POS, dirtyRows.top );
                            p = gFrameBuffer.virtPtr;
                            p += ( dirtyRows.top * LCD_WIDTH );

                            for ( row = dirtyRows.top; row <= dirtyRows.bottom; row++ )
                            {
                                for ( col = 0; col < LCD_WIDTH; col++ )
                                {
                                    REG_LCD_DATR = *p++;
                                }
                                /* yield to other threads every 20 rows */
                                if ((row % 20) == 20)
                                    yield();
                            }
                        }
#endif
                    }
                }
            }
        }
        else
        {
            /* We've been signalled. */

            break;
        }
    }

    complete_and_exit(&gUpdateExited, 0);
} /* lcd_update_thread */


/****************************************************************************
*
*  lcd_draw_perf
*
*   Draws the CPU performance data at the bottom of the LCD
*
***************************************************************************/

#if defined( CONFIG_BCM_LCD_PERF )

/* static */
void lcd_draw_perf( void )
{
    unsigned int    total;
    unsigned int    accum;
    unsigned int    pos;
    unsigned int    end;
    u16            *fbPtr;
    LCD_DirtyRows_t dirtyRows;

    /* Put a black line in for separation */

    fbPtr = gFrameBuffer.virtPtr;
    fbPtr += LCD_WIDTH * ( LCD_HEIGHT - 2 );
    for ( pos = 0; pos < LCD_WIDTH; pos++ )
    {
        *fbPtr++ = 0;
    }

    /*
     * Now that we have the data, figure out the delta, and draw a line 
     * along the bottom 
     */

#if !defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
    total = gLcdPerfCurrData.jiffies - gLcdPerfPrevData.jiffies;
    accum = gLcdPerfCurrData.system  - gLcdPerfPrevData.system;
#else
    total = gLcdPerfCurrData.total;
    accum = total - gLcdPerfCurrData.idle;
#endif

    end = accum * LCD_WIDTH / total;

    for ( pos = 0; ( pos <= end ) && ( pos < LCD_WIDTH ); pos++ )
    {
        *fbPtr++ = RGB_VAL( 31, 0, 0 );  /* Red */
    }

#if !defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
    accum += ( gLcdPerfCurrData.user - gLcdPerfPrevData.user );
    end = accum * LCD_WIDTH / total;

    for ( ; ( pos <= end ) && ( pos < LCD_WIDTH ); pos++ )
    {
        *fbPtr++ = RGB_VAL( 0, 63, 0 );  /* Green */
    }

    accum += ( gLcdPerfCurrData.nice - gLcdPerfPrevData.nice );
    end = accum * LCD_WIDTH / total;

    for ( ; ( pos <= end ) && ( pos < LCD_WIDTH ); pos++ )
    {
        *fbPtr++ = RGB_VAL( 31, 63, 16 );  /* Yellow (100% R, 100% G, 50% B) */
    }
#endif

    if ( accum > gLcdPerfPeakBusy )
    {
        gLcdPerfPeakBusy = accum;
    }
    end = gLcdPerfPeakBusy * LCD_WIDTH / total;

    for ( ; ( pos <= end ) && ( pos < LCD_WIDTH ); pos++ )
    {
        *fbPtr++ = RGB_VAL( 0, 0, 31 );  /* Blue */
    }

    for ( ; pos < LCD_WIDTH; pos++ )
    {
        *fbPtr++ = RGB_VAL( 8, 16, 8 );  /* Dark Gray (25% R, 25% G, 25% B) */
    }

    dirtyRows.top = LCD_HEIGHT - 2;
    dirtyRows.bottom = LCD_HEIGHT - 1;
    lcd_dirty_rows( &dirtyRows );

} /* lcd_draw_perf */

#endif /* CONFIG_BCM_LCD_PERF */

/****************************************************************************
*
*  lcd_clear_perf
*
*   Clears the CPU performance data at the bottom of the LCD
*
***************************************************************************/

#if defined( CONFIG_BCM_LCD_PERF )

/* static */
void lcd_clear_perf( void )
{
    unsigned int    pos;
    u16            *fbPtr;
    LCD_DirtyRows_t dirtyRows;

    /* Put 2 black lines to clear performance meter */

    fbPtr = gFrameBuffer.virtPtr;
    fbPtr += LCD_WIDTH * ( LCD_HEIGHT - 2 );
    for ( pos = 0; pos < (2 * LCD_WIDTH); pos++ )
    {
        *fbPtr++ = 0;
    }

    dirtyRows.top = LCD_HEIGHT - 2;
    dirtyRows.bottom = LCD_HEIGHT - 1;
    lcd_dirty_rows( &dirtyRows );

} /* lcd_clear_perf */

#endif /* CONFIG_BCM_LCD_PERF */


/****************************************************************************
*
*  lcd_enable_sub_backlight
*
*   Sets the LCD_BL_EN_M signal to 'level'.
*
***************************************************************************/

/* static */
void lcd_enable_sub_backlight( int level )
{
    LCD_DEBUG( "%d\n", level );

    /* LCD on phone platform does not have sub LCD */

} /* lcd_enable_sub_backlight */

/****************************************************************************
*
*  lcd_enable_cs
*
*   Enables the chip select line for the LCD
*
***************************************************************************/

/* static */
void lcd_enable_cs( int level )
{
    LCD_DEBUG( "%d\n", level );

    /* LCD on phone platform does not have sub LCD */

} /* lcd_enable_cs */

/****************************************************************************
*
*  lcd_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

/* static */
void __exit lcd_exit( void )
{
    LCD_PUTS( "called\n" );

    lcd_backlight_deinit();

#if defined( CONFIG_BCM_LCD_PERF )

    del_timer( &gLcdPerfTimer );

    if ( gSysCtlHeader != NULL )
    {
        unregister_sysctl_table( gSysCtlHeader );
    }

#endif

    if (gUpdateThreadPid >= 0)
    {
        kill_proc_info(SIGTERM, SEND_SIG_PRIV, gUpdateThreadPid);
        wait_for_completion(&gUpdateExited);
    }

} /* lcd_exit */

/****************************************************************************
*
*  lcd_fill_rect
*
*     Debugging routine that fills a rectangle with a color
*
***************************************************************************/

/* static */
void lcd_fill_rect( LCD_Rect_t *r )
{
    unsigned int    y;
    unsigned int    y_end = r->top + r->height;
    unsigned int    col;

    LCD_DEBUG( "x=%u y=%u w=%u h=%u color=0x%04x\n",
               r->left, r->top, r->width, r->height, r->color );

    for ( y = r->top; y < y_end; y++ )
    {
        lcd_write_cmd( LCD_REG_X_POS, r->left );
        lcd_write_cmd( LCD_REG_Y_POS, y );

        for ( col = 0; col < r->width; col++ )
        {
            REG_LCD_DATR = r->color;
        }
    }

} /* lcd_fill_rect */


/****************************************************************************
*
*  lcd_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

static
int __init lcd_init( void )
{
    int rc;

    LCD_PUTS( "called\n" );

    printk( gBanner );

    /* Register our device with Linux */
    if (( rc = register_chrdev( BCM_LCD_MAJOR, "lcd", &lcd_fops )) < 0 )
    {
        printk( KERN_WARNING "lcd: register_chrdev failed for major %d\n", BCM_LCD_MAJOR );
        return rc;
    }

    /*
     * Allocate memory for the framebuffer. Our LCD is fixed at 176 x 220 x 16 bits/pixel 
     * Only do this if the memory has not been allocated by lcd_get_framebuffer_addr(). 
     */

    if (!gFrameBufferAllocated)
    {
        gFrameBuffer.sizeInBytes = LCD_FRAME_BUFFER_BYTES;

        /*
         * dma_alloc_writecombine allocates uncached, buffered memory, that is 
         * io_remappable 
         */

        gFrameBuffer.virtPtr = dma_alloc_writecombine( NULL, gFrameBuffer.sizeInBytes,
                                                 &gFrameBuffer.physPtr, GFP_KERNEL );

        LCD_DEBUG( "virtPtr = 0x%lx, physPtr = 0x%lx\n", (long)gFrameBuffer.virtPtr, (long)gFrameBuffer.physPtr );
        memset( gFrameBuffer.virtPtr, 0, gFrameBuffer.sizeInBytes );

        if (( gFrameBuffer.physPtr & ~PAGE_MASK ) != 0 )
        {
            panic( "lcd_init: We didn't get a page aligned buffer" );
            return -ENOMEM;
        }

        if ( gFrameBuffer.virtPtr == NULL )
        {
            return -ENOMEM;
        }
        gFrameBufferAllocated = 1;
    }

#if defined( CONFIG_BCM_LCD_PERF )

    gSysCtlHeader = register_sysctl_table( gSysCtl );
    init_timer( &gLcdPerfTimer );
    gLcdPerfTimer.function = lcd_perf_timer;
#endif /* CONFIG_BCM_LCD_PERF */

#if USE_DMA
    /* create dma descriptors */
    if ( lcd_dmaCreateList() != 0 )
    {
        LCD_DEBUG("LCD DBG_ERROR - Failed to create DMA linked list\n");
        return -ENOMEM;
    }

    /* request DMA channel and DMA irq, and enable dma irq */
    gDmaChannel = DESIRED_DMA_CHANNEL;
    if ( dma_request_chan( gDmaChannel, "LCD" ) != 0 )
    {
        /* couldn't get desired DMA channel, take whatever is available */
        if ( dma_request_avail_chan( &gDmaChannel, "LCD" ) != 0 )
        {
            LCD_DEBUG("LCD DBG_ERROR - Failed to get DMA channel\n");
            return -EINVAL;
        }
    }

    if ( dma_request_irq( gDmaChannel, lcd_dma_isr, 0 ) != 0 )
    {
        LCD_DEBUG("LCD DBG_ERROR - Failed to get dma irq\n");
        return -EINVAL;
    }

    gDmaSavedLli.valid = 0;
    sema_init(&gDmaSem, 1);
    dma_enable_irq( gDmaChannel );
#endif

    /* Create update thread */
    sema_init(&gUpdateSem, 0);
    init_completion(&gUpdateExited);
    gUpdateThreadPid = kernel_thread(lcd_update_thread, 0, 0);

    /* Register our device with the Power Manager */
    if ((rc = pm_register_component(PM_COMP_LCD, &lcd_pm_ops)) < 0)
    {
       printk("lcd: failed to register with power manager\n");
       return rc;
    }

    return 0;
}

/****************************************************************************
*
*  lcd_set_power
*
*     Called by backlight driver to turn power on/off
*
***************************************************************************/
void lcd_set_power( int onOff )
{
    lcd_pm_update( onOff ? PM_COMP_PWR_ON : PM_COMP_PWR_OFF, 0 );
}

/****************************************************************************
*
*  lcd_pm_update
*
*     Called by power manager to update component power level
*
***************************************************************************/
static
int lcd_pm_update(PM_CompPowerLevel compPowerLevel, PM_PowerLevel sysPowerLevel)
{
   static PM_CompPowerLevel powerLevel = PM_COMP_PWR_OFF;

   /* Nothing to do if power level did not change */
   if (compPowerLevel == powerLevel)
      return 0;

   /* Save new power level */
   powerLevel = compPowerLevel;
   switch (powerLevel)
   {
      case PM_COMP_PWR_OFF:
      case PM_COMP_PWR_STANDBY:
         lcd_pwr_off_controller();
         break;
      case PM_COMP_PWR_ON:
         lcd_pwr_on();
         break;
   }

   return 0;
}


/****************************************************************************
*
*  lcd_pwr_on
*
*     Power on controller
*
***************************************************************************/
static
int lcd_pwr_on ( void )
{
   static int firstInit = 1;
   unsigned int    row;
   unsigned int    col;
   u16            *p;

   LCD_PUTS( "called\n" );

   if (firstInit)
   {
      /* Initialize our side of the controller hardware */
      /* At 78 MHz, 1 AHB cycle = 12.8 ns */
      /*
       * According to the HYeFB data sheet: 
       * Setup = 0 ns for reads or write 
       * Pulse = 120 ns for reads and 140 ns for write (10 & 11 AHB cycles) 
       * Hold  = 250 - Pulse = 130 ns for read and 110 ns for write (11 & 10 AHB cycles) 
       */

      /*
       * TODO should this be set for the worst case, i.e. AHB is 104 MHz? 
       * depends on whether we ever run CPU=104 AHB=104 case 
       */
      REG_LCD_RTR = REG_LCD_RTR_VAL( 0, 10, 11 );
      REG_LCD_WTR = REG_LCD_WTR_VAL( 0, 11, 10 );

      REG_LCD_CR = REG_LCD_CR_ENABLE_16_TO_18_EXPANSION;

      /* Configure the various GPIO pins */

      reg_gpio_iotr_set_pin_type( HW_GPIO_LCD_RESET, GPIO_IOTR_GPIO_OUTPUT );
      lcd_backlight_init();

      LCD_INIT_TEST_TIME();
   }

   /* Initialize controller */
   lcd_init_all();

   if (firstInit)
   {
      /*  Write the initial splash screen to the LCD before turning on the backlight. */
      lcd_write_cmd( LCD_REG_X_POS, 0 );
      lcd_write_cmd( LCD_REG_Y_POS, 1 );
      p = gFrameBuffer.virtPtr;
      p += ( 1 * LCD_WIDTH );

      for ( row = 1; row <= LCD_HEIGHT-1; row++ )
      {
          for ( col = 0; col < LCD_WIDTH; col++ )
          {
              REG_LCD_DATR = *p++;
          }
      }
      lcd_backlight_enable( 15 );
   }
   firstInit = 0;

#if defined( CONFIG_BCM_LCD_PERF )
   if (gLcdPerf)
   {
#if defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
      arch_init_idle_profile(&gLcdPerfCurrData.idle_handle);
#endif
      gLcdPerfTimer.expires = jiffies + gLcdPerfFreq;
      add_timer( &gLcdPerfTimer );
   }

#endif /* CONFIG_BCM_LCD_PERF */

   return 0;

} /* lcd_pwr_on */

/****************************************************************************
*
*  lcd_init_controller
*
*   Initializes the LCD Controller. This is required after resetting it.
*
***************************************************************************/

/* static */
void lcd_init_all( void )
{
    LCD_PUTS( "called\n" );

    if (!gInitialized)
    {
        gInitialized = 1;

        lcd_reset_controller( 0 );
        udelay( 10 );
        lcd_reset_controller( 1 );
        udelay( 10 );

        lcd_enable_cs( 0 );

        lcd_init_controller();
    }

} /* lcd_init_all */

/****************************************************************************
*
*  lcd_init_controller
*
*   Initializes the LCD Controller. This is required after resetting it.
*
***************************************************************************/

/* static */
void lcd_init_controller( void )
{
    LCD_PUTS( "called\n" );

    lcd_send_cmd_sequence( gInitCmd );

} /* lcd_init_controller */

/****************************************************************************
*
*  lcd_pwr_off_controller
*
*   Power off the LCD controller.
*
***************************************************************************/

/* static */
void lcd_pwr_off_controller( void )
{
    LCD_PUTS( "called\n" );

    lcd_send_cmd_sequence( gPwrOffCmd );

    gInitialized = 0;

} /* lcd_pwr_off_controller */

/****************************************************************************
*
*  lcd_ioctl
*
***************************************************************************/

/* static */
int lcd_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
    int err = 0;

    /* LCD_DEBUG( "type: '%c' cmd: 0x%x\n", _IOC_TYPE( cmd ), _IOC_NR( cmd )); */

    switch ( cmd )
    {
        case LCD_IOCTL_RESET:
        {
            lcd_reset_controller( (int)arg );
            break;
        }

        case LCD_IOCTL_ENABLE_BACKLIGHT:
        {
            lcd_backlight_enable( (int)arg );
            break;
        }

        case LCD_IOCTL_ENABLE_SUB_BACKLIGHT:
        {
            lcd_enable_sub_backlight( (int)arg );
            break;
        }

        case LCD_IOCTL_ENABLE_CS:
        {
            lcd_enable_cs( (int)arg );
            break;
        }

        case LCD_IOCTL_SCOPE_TIMEOUT:
        {
            lcd_scope_timeout( (int)arg );
            break;
        }

        case LCD_IOCTL_INIT:
        {
            lcd_init_controller();
            break;
        }

        case LCD_IOCTL_INIT_ALL:
        {
            lcd_init_all();
            break;
        }

        case LCD_IOCTL_SETUP:
        {
            REG_LCD_RTR &= ~REG_LCD_RTR_SETUP_MASK;
            REG_LCD_RTR |= ( arg << REG_LCD_RTR_SETUP_SHIFT );
            REG_LCD_WTR &= ~REG_LCD_WTR_SETUP_MASK;
            REG_LCD_WTR |= ( arg << REG_LCD_WTR_SETUP_SHIFT );
            break;
        }

        case LCD_IOCTL_HOLD:
        {
            REG_LCD_RTR &= ~REG_LCD_RTR_HOLD_MASK;
            REG_LCD_RTR |= ( arg << REG_LCD_RTR_HOLD_SHIFT );
            REG_LCD_WTR &= ~REG_LCD_WTR_HOLD_MASK;
            REG_LCD_WTR |= ( arg << REG_LCD_WTR_HOLD_SHIFT );
            break;
        }

        case LCD_IOCTL_PULSE:
        {
            REG_LCD_RTR &= ~REG_LCD_RTR_PULSE_MASK;
            REG_LCD_RTR |= ( arg << REG_LCD_RTR_PULSE_SHIFT );
            REG_LCD_WTR &= ~REG_LCD_WTR_PULSE_MASK;
            REG_LCD_WTR |= ( arg << REG_LCD_WTR_PULSE_SHIFT );
            break;
        }

        case LCD_IOCTL_REG:
        {
            LCD_Reg_t r;

            if ( copy_from_user( &r, (LCD_Reg_t *)arg, sizeof( r )) != 0 )
            {
                return -EFAULT;
            }

            lcd_write_cmd( r.reg, r.val );
            break;
        }

        case LCD_IOCTL_RECT:
        {
            LCD_Rect_t r;

            if ( copy_from_user( &r, (LCD_Rect_t *)arg, sizeof( r )) != 0 )
            {
                return -EFAULT;
            }

            lcd_fill_rect( &r );
            break;
        }

        case LCD_IOCTL_COLOR_TEST:
        {
            lcd_color_test( (int)arg );
            break;
        }

        case LCD_IOCTL_DIRTY_ROWS:
        {
            LCD_DirtyRows_t dirtyRows;

            if ( copy_from_user( &dirtyRows, (LCD_DirtyRows_t *)arg, sizeof( dirtyRows )) != 0 )
            {
                return -EFAULT;
            }

#if defined( CONFIG_BCM_LCD_PERF )
            if ( gLcdPerf )
            {
                if ( dirtyRows.bottom > LCD_HEIGHT - 3 )
                {
                    if ( dirtyRows.top > LCD_HEIGHT - 3 )
                    {
                        return 0;
                    }
                    dirtyRows.bottom = LCD_HEIGHT - 3;
                }
            }
#endif

            lcd_dirty_rows( &dirtyRows );
            break;
        }

        case LCD_IOCTL_DIRTY_ROW_BITS:
        {
           err = lcd_ioctl_dirty_row_bits( arg );
           break;
        }

        case LCD_IOCTL_PRINT_REGS:
        {
            LCD_PrintRegs_t  prReg;

            if ( copy_from_user( &prReg, (LCD_PrintRegs_t *)arg, sizeof( prReg )) != 0 )
            {
                return -EFAULT;
            }

            lcd_print_regs( prReg.startReg, prReg.endReg );
            break;
        }

        case LCD_IOCTL_PRINT_DATA:
        {
            LCD_PrintData_t  prData;

            if ( copy_from_user( &prData, (LCD_PrintData_t *)arg, sizeof( prData )) != 0 )
            {
                return -EFAULT;
            }

            lcd_print_data( prData.x, prData.y, prData.numPixels );
            break;
        }

        case LCD_IOCTL_PWR_OFF:
        {
            lcd_pwr_off_controller();
            break;
        }

        case LCD_IOCTL_INFO:
        {
            LCD_Info_t lcdInfo;

            lcd_get_info( &lcdInfo );
            err = copy_to_user( (void *)arg, &lcdInfo, sizeof( LCD_Info_t ));

            break;
        }


        default:
        {
            LCD_DEBUG( "Unrecognized ioctl: '0x%x'\n", cmd );
            return -ENOTTY;
        }
    }

    return ( err );

} /* lcd_ioctl */

/****************************************************************************
*
*  Helper function for dealing with the LCD_IOCTL_DIRTY_ROW_BITS ioctl.
*
*  This function is used both from lcdfb.c and this file.
*
***************************************************************************/

int lcd_ioctl_dirty_row_bits( unsigned long arg )
{
   int                err = 0;
   LCD_DirtyRowBits_t dirtyRowBits;
   uint32_t           bitsData[ BITS_TO_UINT32S( LCD_HEIGHT + 1 )];

   if ( copy_from_user( &dirtyRowBits, (LCD_DirtyRowBits_t *)arg, sizeof( dirtyRowBits )) != 0 )
   {
      return -EFAULT;
   }

   if ( dirtyRowBits.numRows == 0 )
   {
      return 0;
   }

   if ( dirtyRowBits.numRows > LCD_HEIGHT )
   {
       dirtyRowBits.numRows = LCD_HEIGHT;
   }

   if ( copy_from_user( bitsData, dirtyRowBits.bits, BITS_TO_BYTES( dirtyRowBits.numRows )) != 0 )
   {
      err = -EFAULT;
   }
   else
   {
      dirtyRowBits.bits = bitsData;

#if defined( CONFIG_BCM_LCD_PERF )
      if ( gLcdPerf )
      {
          MARK_ROW_CLEAN( dirtyRowBits.bits, LCD_HEIGHT - 1 );
          MARK_ROW_CLEAN( dirtyRowBits.bits, LCD_HEIGHT - 2 );
      }
#endif
      lcd_dirty_row_bits( &dirtyRowBits );
   }

   return err;

} /* lcd_ioctl_dirty_row_bits */

/****************************************************************************
*
*  lcd_get_info
*
*
*
***************************************************************************/

/*static */
void lcd_get_info( LCD_Info_t *lcdInfo )
{
    lcdInfo->bitsPerPixel  = LCD_BITS_PER_PIXEL;
    lcdInfo->height        = LCD_HEIGHT;
    lcdInfo->width         = LCD_WIDTH;
}


/****************************************************************************
*
*  lcd_mmap
*
*   Note that the bulk of this code came from the fb_mmap routine found in
*   drivers/video/fbmem.c
*
***************************************************************************/

/* static */
int lcd_mmap( struct file *file, struct vm_area_struct * vma )
{
    unsigned long   offset;
    unsigned long   start;
    unsigned long   len;

    /*
     * vma->vm_start    is the start of the memory region, in user space 
     * vma->vm_end      is one byte beyond the end of the memory region, in user space 
     * vma->vm_pgoff    is the offset (in pages) within the vm_start to vm_end region 
     */

    LCD_PUTS( "called\n" );

    if ( vma->vm_pgoff > ( ~0UL >> PAGE_SHIFT ))
    {
        LCD_DEBUG( "vm_pgoff is out of range\n" );

        /* the value is outside our memory range */

        return -EINVAL;
    }

    /* Convert offset into a byte offset, rather than a page offset */

    offset = vma->vm_pgoff << PAGE_SHIFT;

    start = (unsigned long)gFrameBuffer.physPtr; /* already page-aligned */

    len = PAGE_ALIGN( start + gFrameBuffer.sizeInBytes );

    if ( offset > len )
    {
        LCD_DEBUG( "offset is too large, offset = %lu, len = %lu\n", offset, len );

        /* The pointer requested by the user isn't inside of our frame buffer */

        return -EINVAL;
    }

    vma->vm_page_prot = pgprot_writecombine( vma->vm_page_prot );

    offset += start;

    vma->vm_pgoff = offset >> PAGE_SHIFT;

    if ( 0 != io_remap_pfn_range( vma,
                                  vma->vm_start,
                                  offset >> PAGE_SHIFT,
                                  vma->vm_end - vma->vm_start,
                                  vma->vm_page_prot ))
    {
        LCD_DEBUG( "remap_page_range failed\n" );

        return -EAGAIN;
    }

    return 0;

} /* lcd_mmap */

/****************************************************************************
*
*  lcd_open
*
***************************************************************************/

/* static */
int lcd_open( struct inode *inode, struct file *file )
{
/*    int rc; */

    /* LCD_DEBUG( "major = %d, minor = %d\n", MAJOR( inode->i_rdev ),  MINOR( inode->i_rdev )); */

    return 0;

} /* lcd_open */

/****************************************************************************
*
*  lcd_perf_timer
*
***************************************************************************/

#if defined( CONFIG_BCM_LCD_PERF )

/* static */
void lcd_perf_timer( unsigned long dummy )
{
    (void)dummy;

    if ( gLcdPerf )
    {
#if !defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
        gLcdPerfPrevData = gLcdPerfCurrData;

        gLcdPerfCurrData.nice = kstat_this_cpu.cpustat.nice;
        gLcdPerfCurrData.user = kstat_this_cpu.cpustat.user;
        gLcdPerfCurrData.system = kstat_this_cpu.cpustat.system +
            kstat_this_cpu.cpustat.softirq +
            kstat_this_cpu.cpustat.irq +
            kstat_this_cpu.cpustat.steal +
            kstat_this_cpu.cpustat.iowait;
        gLcdPerfCurrData.jiffies = jiffies;
#else
        arch_get_idle_profile( &gLcdPerfCurrData.idle_handle, &gLcdPerfCurrData.idle, &gLcdPerfCurrData.total );
#endif

        lcd_draw_perf();

        gLcdPerfTimer.expires = jiffies + gLcdPerfFreq + 1;
        add_timer( &gLcdPerfTimer );
    }
    else
    {
        lcd_clear_perf();
    }

} /* lcd_perf_timer */

/****************************************************************************
*
*  lcd_proc_sysctl_perf
*
*   Handler for lcd perf sysctl
*
***************************************************************************/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int lcd_proc_sysctl_perf(ctl_table *table, int write,
             void __user *buffer, size_t *lenp, loff_t *ppos )
#else
static int lcd_proc_sysctl_perf(ctl_table *table, int write, struct file *filp,
             void __user *buffer, size_t *lenp, loff_t *ppos )
#endif
{
    int rc;

    if ( !table || !table->data )
        return -EINVAL;

    if ( write )
    {
        int lastLcdPerf;

        lastLcdPerf = gLcdPerf;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
        rc = proc_dointvec( table, write, buffer, lenp, ppos );
#else
        rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );
#endif

        if (rc < 0)
            return rc;

        /* Start timer if enabling.  The timer will be stopped in the timer function
         * if disabling.
         */
        if (gLcdPerf != lastLcdPerf)
        {
            if (gLcdPerf)
            {
#if defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )
                arch_init_idle_profile(&gLcdPerfCurrData.idle_handle);
#endif
                /* Use mod-timer to avoid starting a timer twice */
                mod_timer( &gLcdPerfTimer, jiffies + gLcdPerfFreq );
            }
        }

        return rc;
    }
    else
    {
        /* nothing special for read, just use generic int handler */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
        return proc_dointvec( table, write, buffer, lenp, ppos );
#else
        return proc_dointvec( table, write, filp, buffer, lenp, ppos );
#endif
    }
}

#endif /* CONFIG_BCM_LCD_PERF */

/****************************************************************************
*
*  lcd_print_data
*
***************************************************************************/

/* static */
void lcd_print_data( unsigned int x, unsigned int y, unsigned int numPixels )
{
    unsigned int pixNum;
    unsigned int val;

    if ( x >= LCD_WIDTH )
    {
        printk( "lcd_print_data: Expecting x to be between 0 and %d, found: %d\n",
                LCD_WIDTH - 1, x );
        return;
    }

    if ( y >= LCD_HEIGHT )
    {
        printk( "lcd_print_data: Expecting y to be between 0 and %d, found: %d\n",
                LCD_HEIGHT - 1, y );
        return;
    }

    if (( numPixels <= 0 ) || ((( y * LCD_WIDTH ) + x + numPixels ) > LCD_FRAME_BUFFER_BYTES ))
    {
        printk( "lcd_print_data: Expecting numPixels to be between 1 and %d, found: %d\n",
                LCD_FRAME_BUFFER_BYTES - ( y * LCD_WIDTH ) - x, numPixels );
        return;
    }

/* When we read the LCD, we get back 18 bits */

#define RED_VAL( val )      ( ((val) >> 12 ) & 0x3f )
#define GREEN_VAL( val )    ( ((val) >>  6 ) & 0x3f )
#define BLUE_VAL( val )     ( ((val)       ) & 0x3f )

    lcd_write_cmd( LCD_REG_X_POS, x );
    lcd_write_cmd( LCD_REG_Y_POS, y );

    val = REG_LCD_DATR; /* Dummy read (required) */

    for ( pixNum = 0; pixNum < numPixels; pixNum++ )
    {
        val = REG_LCD_DATR;

        printk( "LCD Data %3u = 0x%04x R:%d G:%d B:%d\n",
                pixNum, val, RED_VAL( val ), GREEN_VAL( val ), BLUE_VAL( val ));
    }

} /* lcd_print_regs */

/****************************************************************************
*
*  lcd_print_regs
*
***************************************************************************/

/* static */
void lcd_print_regs( unsigned char startReg, unsigned char endReg )
{
    unsigned char reg;

    /*
     * Doh - This function doesn't really work properly, since there isn't 
     * anyway for us to tell the controller which register to read. 
     */
    /*
     * All we're really reading is the values retained by the capacitance of 
     * the traces. 
     */

    for ( reg = startReg; reg <= endReg; reg++ )
    {
        unsigned int val;

        val = REG_LCD_CMDR;

        printk( "LCD Reg %3u 0x%02x = %3u, 0x%02x\n", reg, reg, val & 0xFF, val );
    }

} /* lcd_print_regs */

/****************************************************************************
*
*  lcd_release
*
***************************************************************************/

/* static */
int lcd_release( struct inode *inode, struct file *file )
{
    /* LCD_PUTS( "called\n" ); */

    return 0;

} /* lcd_release */

/****************************************************************************
*
*  lcd_reset_controller
*
*   Resets the controller for use.
*
***************************************************************************/

/* static */
void lcd_reset_controller( int level )
{
    LCD_DEBUG( "%d\n", level );

    reg_gpio_set_pin( HW_GPIO_LCD_RESET, level );

} /* lcd_reset_controller */

/****************************************************************************
*
*  lcd_scope_timeout
*
*   Debugging routine to help measuring timeouts on an oscilloscope.
*
***************************************************************************/

/* static */
void lcd_scope_timeout( int level )
{
    /*
     * This routine exists so that the timeouts used during the 
     * initialization sequence can be checked on an oscilloscope. 
     */

    /* To use, make sure that LCD_T */

    static lcd_cmd_t   gWait17[] =
    {
        LCD_WAIT_MSEC( 17 ),

        LCD_END_SEQUENCE
    };

    static lcd_cmd_t   gWait200[] =
    {
        LCD_WAIT_MSEC( 200 ),

        LCD_END_SEQUENCE
    };

    static lcd_cmd_t   gWait250[] =
    {
        LCD_WAIT_USEC( 250 ),

        LCD_END_SEQUENCE
    };


    int i;

    LCD_DEBUG( "%d\n", level );

    if ( level == 0 )
    {
        for ( i = 0; i < 10; i++ )
        {
            lcd_send_cmd_sequence( gWait250 );

            udelay( 10 );
        }
    }
    else
    if ( level == 1 )
    {
        for ( i = 0; i < 10; i++ )
        {
            lcd_send_cmd_sequence( gWait17 );

            mdelay( 1 );
        }
    }
    else
    if ( level == 2 )
    {
        for ( i = 0; i < 10; i++ )
        {
            lcd_send_cmd_sequence( gWait200 );

            mdelay( 10 );
        }
    }

} /* lcd_scope_timeout */

/****************************************************************************
*
*  lcd_send_cmd_sequence
*
*   Sends a sequence of commands to the LCD device
*
***************************************************************************/

/* static */
int lcd_send_cmd_sequence( lcd_cmd_t *cmdSeq )
{
    lcd_cmd_t  *cmd = cmdSeq;

    while ( cmd->cmd != LCD_CMD_END )
    {
        switch ( cmd->cmd )
        {
            case LCD_CMD_SET_REG:
            {
                lcd_write_cmd( cmd->reg, cmd->val );
                break;
            }

            case LCD_CMD_WAIT_USEC:
            {
                unsigned long delayTime = cmd->val;

                LCD_DEBUG_REG( "Waiting for %lu uSec\n", delayTime );

                LCD_START_TEST_TIME();

                udelay( delayTime );

                LCD_STOP_TEST_TIME();
                break;
            }

            case LCD_CMD_WAIT_MSEC:
            {
                unsigned long delayTime = cmd->val;

                LCD_DEBUG_REG( "Waiting for %lu mSec\n", delayTime );

                LCD_START_TEST_TIME();

                mdelay( delayTime );

                LCD_STOP_TEST_TIME();
                break;
            }

            case LCD_CMD_LINE_CALIBRATE:
            {
                unsigned long flags;

                local_irq_save( flags );

                LCD_DEBUG_REG( "%s", "Calibrating Line\n" );

                /*
                 * NOTE: We write directly to the register so that the 
                 *       calibration timing won't be affected. 
                 * Adding debug prints modifies the timing enough that the 
                 * LCD has a noticable flicker. 
                 */

                REG_LCD_CMDR = 0x2D01;  /* Calibration register - Start */

                udelay( 75 );

                REG_LCD_CMDR = 0x2D00;  /* Calibration register - Stop */

                local_irq_restore( flags );
                break;
            }
        }

        cmd++;
    }

    return 0;

} /* lcd_send_cmd_sequence */

/****************************************************************************
*
*  lcd_get_framebuffer_addr
*
*   Gets the address of the frame buffer
*
***************************************************************************/

void *lcd_get_framebuffer_addr( int *frame_size, dma_addr_t *dma_addr )
{
    /*
     *  Check if we have been initialized yet.  If not, another driver wants 
     *  access to our framebuffer before we have been inited.  In this case, 
     *  allocate the framebuffer now to avoid the other driver failing. 
     */
    if (!gFrameBufferAllocated)
    {
        gFrameBuffer.sizeInBytes = LCD_FRAME_BUFFER_BYTES;

        /*
         * dma_alloc_writecombine allocates uncached, buffered memory, that is 
         * io_remappable 
         */

        gFrameBuffer.virtPtr = dma_alloc_writecombine( NULL, gFrameBuffer.sizeInBytes,
                                                 &gFrameBuffer.physPtr, GFP_KERNEL );

        LCD_DEBUG( "virtPtr = 0x%lx, physPtr = 0x%lx\n", (long)gFrameBuffer.virtPtr, (long)gFrameBuffer.physPtr );
        memset( gFrameBuffer.virtPtr, 0, gFrameBuffer.sizeInBytes );

        if (( gFrameBuffer.physPtr & ~PAGE_MASK ) != 0 )
        {
            panic( "lcd_init: We didn't get a page aligned buffer" );
            return NULL;
        }

        if ( gFrameBuffer.virtPtr == NULL )
        {
            return NULL;
        }
        gFrameBufferAllocated = 1;
    }

    if (dma_addr)
    {
        *dma_addr = gFrameBuffer.physPtr;
    }
    if (frame_size)
    {
        *frame_size = gFrameBuffer.sizeInBytes;
    }
    return gFrameBuffer.virtPtr;
} /* lcd_get_framebuffer_addr */

#if USE_DMA
/****************************************************************************
*
*  lcd_start_dma
*
*   Initiates a DMA transfer of data from the frame buffer to the LCD
*
*   top = index of first row
*   bottom = index of last row
*
***************************************************************************/

static void lcd_start_dma( unsigned int top, unsigned int bottom )
{
    LCD_DmaLLI_t *linkedList;
    LCD_DmaLLI_t *startp;

    /* bounds checking to avoid writing out of bounds */
    if (( top >= 0 ) && ( top < LCD_HEIGHT ) && ( bottom >= 0 ) && ( bottom < LCD_HEIGHT ))
    {
        /* need a delay here to ensure previous commands to LCD controller are complete */
        udelay( 1 );

        /* enable DMA for the LCD controller */
        REG_LCD_CR |= (REG_LCD_CR_ENABLE_DMA | REG_LCD_CR_ENABLE_DMA_WORDSWAP);

        /* get pointers to linked list and start LLI */
        linkedList = (LCD_DmaLLI_t *)gDmaLinkedList.virtPtr;
        startp = &linkedList[ top ];

        /* save LLI info for the end LLI link and control registers */
        gDmaSavedLli.row = bottom;
        gDmaSavedLli.link = linkedList[ bottom ].link;
        gDmaSavedLli.control = linkedList[ bottom ].control;
        gDmaSavedLli.valid = 1;

        /* now make the end LLI the end of the list */
        linkedList[ bottom ].link = 0;
        linkedList[ bottom ].control = SWAP( DMA_CONTROL | REG_DMA_CHAN_CTL_TC_INT_ENABLE );

        /* start the transfer */
        dma_setup_chan( gDmaChannel, SWAP(startp->source), SWAP(startp->dest),
                SWAP(startp->link), SWAP(startp->control), DMA_CONFIG );
    }
    else
    {
        /* no DMA processing to be done, release the DMA semaphore */
        up( &gDmaSem );
    }

} /* lcd_start_dma */

/****************************************************************************
*
*  lcd_dma_isr
*
*  This isr is triggered when a memory to LCD transfer has completed.
*
***************************************************************************/
static irqreturn_t lcd_dma_isr( void *unused )
{
    LCD_DmaLLI_t *linkedList;

    /* disable DMA mode in LCD controller */
    REG_LCD_CR &= ~(REG_LCD_CR_ENABLE_DMA | REG_LCD_CR_ENABLE_DMA_WORDSWAP);

    /* restore the LLI at the end of the list */
    if ( gDmaSavedLli.valid )
    {
        linkedList = (LCD_DmaLLI_t *)gDmaLinkedList.virtPtr;
        linkedList[ gDmaSavedLli.row ].link = gDmaSavedLli.link;
        linkedList[ gDmaSavedLli.row ].control = gDmaSavedLli.control;
        gDmaSavedLli.valid = 0;
    }

    /* show that DMA to LCD is complete */
    up( &gDmaSem );

    return IRQ_HANDLED;
}

/****************************************************************************
*
*  lcd_dmaCreateList
*
*  Create the linked lists of DMA descriptors
*
***************************************************************************/
static int lcd_dmaCreateList( void )
{
    int i;
    LCD_DmaLLI_t *list;

    /* allocate memory for DMA linked list (1 LLI per LCD row) */
    gDmaLinkedList.sizeInBytes = LCD_HEIGHT * BYTES_PER_LLI;
    gDmaLinkedList.virtPtr = dma_alloc_coherent( NULL, gDmaLinkedList.sizeInBytes,
            &gDmaLinkedList.physPtr, GFP_KERNEL );

    if ( gDmaLinkedList.virtPtr == NULL )
    {
        return -ENOMEM;
    }

    list = (LCD_DmaLLI_t *)gDmaLinkedList.virtPtr;

    /* The LLI data must be stored little endian when DMA is configured for big
     * endian.
     */

    /* create all but the last LLI */
    for (i = 0; i < LCD_HEIGHT - 1; i++ )
    {
        list[i].source = SWAP( gFrameBuffer.physPtr + LCD_WIDTH * BYTES_PER_PIXEL * i );
        list[i].dest = SWAP( REG_LCD_DATR_PADDR );
        list[i].link = SWAP( gDmaLinkedList.physPtr + BYTES_PER_LLI * (i + 1) );
        list[i].control = SWAP( DMA_CONTROL );
    }

    /* create the last LLI */
    list[i].source = SWAP( gFrameBuffer.physPtr + LCD_WIDTH * BYTES_PER_PIXEL * i );
    list[i].dest = SWAP( REG_LCD_DATR_PADDR );
    list[i].link = 0;
    list[i].control = SWAP( DMA_CONTROL | REG_DMA_CHAN_CTL_TC_INT_ENABLE );

    return 0;
}
#endif


/******************************************************************************
** FUNCTION:   lcd_is_dirty_row_update_supported
**
** PURPOSE:    Indicates if LCD driver supports dirty-row logic.
**
** PARAMETERS: None.
**
** RETURNS:    True (1) or False (0).
**
** NOTE:
******************************************************************************/
int lcd_is_dirty_row_update_supported( void )
{
   return ( 1 );
}


/* --------------------------------------------------------------------------
** Stubs.
*/

int lcd_is_display_regions_supported( void )
{
   return ( 0 );
}



/****************************************************************************/

module_init(lcd_init);
module_exit(lcd_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM116x LCD Driver");
MODULE_LICENSE("GPL v2");

