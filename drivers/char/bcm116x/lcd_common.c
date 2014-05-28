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




/****************************************************************************
*
*  lcd.c
*
*  PURPOSE:
*    This implements the code to use a Broadcom LCD host interface.
*
*  NOTES:
*    Uses device minor number to select panel:  0==main 1==sub
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

#include <asm/byteorder.h>
#include <asm/irq.h>
#include <mach/reg_lcd.h>
#include <linux/broadcom/gpio.h>
#include <mach/reg_irq.h>
#include <mach/reg_dma.h>
#include <mach/reg_sys.h>
#include <linux/broadcom/regaccess.h>
#if defined(CONFIG_BCM_IDLE_PROFILER_SUPPORT)
#include <linux/broadcom/idle_prof.h>
#endif
#include <linux/broadcom/cpu_sleep.h>
#include <asm/mach/irq.h>
#include <asm/io.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/hw.h>
#include <linux/broadcom/lcd.h>
#include <linux/broadcom/PowerManager.h>
#include <linux/broadcom/lcd_backlight.h>

#include <cfg_global.h>

#include "lcd_common.h"



/*
 * ---- Public Variables ------------------------------------------------- 
 * ---- Private Constants and Types -------------------------------------- 
 */

static char gBanner[] __initdata = KERN_INFO "lcd: Broadcom LCD Driver: 0.01";


/*
 * globals to:	communicate with the update thread 
 * 		control access to LCD registers 
 */
/*		manage DMA channel */
static int     gInitialized = 0;
static spinlock_t      gLcdUpdateLock = SPIN_LOCK_UNLOCKED;
static long    gUpdateThreadPid = 0;
static struct  completion gUpdateExited;
static struct  semaphore gUpdateSem;
#if USE_DMA
static struct  semaphore gDmaSem;
#endif


/* forward func declarations */
static void     lcd_exit(void);
static int      lcd_init(void);
static void 	lcd_iocr_init(void);
static int      lcd_pwr_on(void);
static void     lcd_init_all(void);
static void     lcd_pwr_off_controller(void);
static int      lcd_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg);
static int      lcd_mmap(struct file *file, struct vm_area_struct * vma);
static int      lcd_open(struct inode *inode, struct file *file);
static int      lcd_release(struct inode *inode, struct file *file);
static void     lcd_reset_controller(int level);

void lcd_display_test(LCD_dev_info_t *dev);
void lcd_display_rect(LCD_dev_info_t *dev,LCD_Rect_t *r);

#if USE_DMA
static void     lcd_start_dma(LCD_dev_info_t *dev,
			unsigned int top, unsigned int bottom);
static          irqreturn_t lcd_dma_isr(void *unused);
static int      lcd_dmaCreateList(LCD_dev_info_t *dev);

static int gDmaChannel;
static DMA_Restore_t gDmaSavedLli;
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

static void
lcd_dump_regs(void)
{
    printk("LCDC_CMD : 0x%08x\n", REG_LCD_CMDR);
    printk("LCDC_DATA: 0x%08x\n", REG_LCD_DATR);
    printk("LCDC_WTR : 0x%08x\n", REG_LCD_WTR);
    printk("LCDC_RTR : 0x%08x\n", REG_LCD_RTR);
    printk("LCDC_CR  : 0x%08x\n", REG_LCD_CR);
    printk("LCDC_SR  : 0x%08x\n", REG_LCD_SR);
    printk("IOCR0    : 0x%08x\n", REG_SYS_IOCR0);
    printk("IOCR2    : 0x%08x\n", REG_SYS_IOCR2);
    printk("GPIO_TR1 : 0x%08x\n", REG_GPIO_IOTR1);

#if USE_DMA
    printk("DMA_CFG  : 0x%08x\n", REG_DMA_CONFIG);
    printk("DMA_SYNC : 0x%08x\n", REG_DMA_SYNC);
    printk("DMA_1_CFG: 0x%08x\n", REG_DMA_CHAN_1_CONFIG);
    printk("DMA_1_CTL: 0x%08x\n", REG_DMA_CHAN_1_CONTROL);
    printk("DMA_1_SRC: 0x%08x\n", REG_DMA_CHAN_1_SRC_ADDR);
    printk("DMA_1_DST: 0x%08x\n", REG_DMA_CHAN_1_DEST_ADDR);
    printk("DMA_1_LNK: 0x%08x\n", REG_DMA_CHAN_1_LINK);
#endif
}

void
lcd_select_panel(LCD_panel_t panel)
{
    if (panel == LCD_sub_panel)
        REG_LCD_CR |= REG_LCD_CR_SELCD;
    else
        REG_LCD_CR &= ~REG_LCD_CR_SELCD;

	LCD_DEBUG("panel=%d REG_LCD_CR=0x%08X\n", panel, REG_LCD_CR);
	
}

void
lcd_write_cmd(uint32_t cmd)
{
    while (REG_LCD_SR & REG_LCD_SR_FIFO_FULL)
	udelay(0);

    REG_LCD_CMDR = cmd;
}

void
lcd_write_data(uint32_t data)
{
    while (REG_LCD_SR & REG_LCD_SR_FIFO_FULL)
	udelay(0);

    REG_LCD_DATR = data;
}

void
lcd_write_param(uint32_t cmd)
{
    extern LCD_Bus_t LCD_Bus;

    if (LCD_Bus == LCD_18BIT)
	  {
        REG_LCD_CR &= ~REG_LCD_CR_ENABLE_16_TO_18_EXPANSION;
        WRITE_LCD_DATA(cmd);
		udelay(0);
        REG_LCD_CR |= REG_LCD_CR_ENABLE_16_TO_18_EXPANSION;
    }
    else
        WRITE_LCD_DATA(cmd);
}

/****************************************************************************
*
*  lcd_dirty_rows
*
*   Marks the indicated rows as dirty and arranges for them to be transferred
*   to the LCD.
*
***************************************************************************/

static void
lcd_dev_dirty_rows(LCD_dev_info_t *dev, LCD_DirtyRows_t *dirtyRows)
{
    unsigned long flags;

    LCD_DEBUG("top = %u,  bottom = %u\n", dirtyRows->top, dirtyRows->bottom);

    if ((dirtyRows->top > dirtyRows->bottom) ||
            (dirtyRows->bottom >= dev->height))
    {
        LCD_DEBUG("invalid dirty-rows params - ignoring\n");
        LCD_DEBUG("top = %u,  bottom = %u\n",
		dirtyRows->top, dirtyRows->bottom);

        return;
    }

    /* Mark dirty rows */
    spin_lock_irqsave(&gLcdUpdateLock, flags);

    if (dirtyRows->top < dev->dirty_rows.top)
        dev->dirty_rows.top = dirtyRows->top;

    if (dirtyRows->bottom > dev->dirty_rows.bottom)
        dev->dirty_rows.bottom = dirtyRows->bottom;

    LCD_PUTS("updateSem");
    up(&gUpdateSem);
    spin_unlock_irqrestore(&gLcdUpdateLock, flags);
    LCD_PUTS("done");
}                               /* lcd_dirty_rows */

void
lcd_dirty_rows(LCD_DirtyRows_t *dirtyRows)
{
    lcd_dev_dirty_rows(&LCD_device[LCD_main_panel], dirtyRows);
}


#if !USE_DMA

/* only to be called by lcd_update_thread() below!! */
static void
lcd_send_data(uint16_t *p, int len)
{
    int i;

    if (REG_LCD_CR & REG_LCD_CR_ENABLE_8_BIT_INTF)   /* 8 bit bus */
    {
	for (i = 0; i < len; i++)
	{
	    WRITE_LCD_DATA(p[i] >> 8);
	    WRITE_LCD_DATA(p[i]);

	    /* yield to other threads every N rows */
	    if ((i & 0xF) == 0xF)
		yield();
	}
    }
    else
    {
	for (i = 0; i < len; i++)
	{
	    WRITE_LCD_DATA(p[i]);

	    /* yield to other threads every X rows */
	    if ((i & 0xF) == 0xF)
		yield();
	}
    }
}

#endif /* !USE_DMA */

/****************************************************************************
*
*  lcd_update_thread
*
*   Worker thread to transfer data to the LCD.
*
***************************************************************************/
static int
lcd_update_thread(void *data)
{
    /* This thread doesn't need any user-level access, * so get rid of all
       our resources */
    daemonize("lcdUpdate");
    LCD_PUTS("enter");

    /* Run until signal received */
    while (down_interruptible(&gUpdateSem) == 0)
	  {
		int d;
		unsigned long flags;
		LCD_DirtyRows_t dirtyRows;
		
		LCD_PUTS("wakeup");
		
		/* check all panels here since they share registers */
		for (d = 0; d < LCD_num_panels; d++)
		  {
			LCD_dev_info_t *dev = &LCD_device[d];
			
#if USE_DMA
			/* grab this here so we only try to DMA to one panel at a time */
			if (down_interruptible(&gDmaSem) != 0)
			  break;
#endif
			
			/* Get dirty rows and reset global dirty row list */
			spin_lock_irqsave(&gLcdUpdateLock, flags);
			dirtyRows = dev->dirty_rows;
			dev->dirty_rows.top = dev->height;
			dev->dirty_rows.bottom = 0;
			spin_unlock_irqrestore(&gLcdUpdateLock, flags);
			
			/*
    * Transfer the rows to LCD 
			 * (actually, transfer all the rows as there does not seem 
			 * to be any LCD command for a partial update) 
			 */
			if (gInitialized && (dirtyRows.bottom >= dirtyRows.top))
			  {
				LCD_PUTS("refresh");
				
				/* refresh the panel */
				lcd_select_panel((LCD_panel_t)d);
				lcd_setup_for_data(dev);
				
#if USE_DMA
				/* start the DMA transfer */
				/*lcd_start_dma(&dev->top, dirtyRows.bottom); */
				lcd_start_dma(dev, 0, dev->height - 1);
#else
				lcd_send_data(dev->frame_buffer.virtPtr,
							  dev->width * dev->height);
#endif
			  }
			else
			  {
#if USE_DMA
				/* no DMA processing to be done, release the semaphore */
				up(&gDmaSem);
#endif
			  }
		  }
		
		LCD_PUTS("wait");
	  }
	
    complete_and_exit(&gUpdateExited, 0);
}                               /* lcd_update_thread */



/****************************************************************************
*
*  lcd_enable_sub_backlight
*
*   Sets the LCD_BL_EN_M signal to 'level'.
*
***************************************************************************/

/* static */
void lcd_enable_sub_backlight(int level)
{
    LCD_DEBUG("%d\n", level);

    /* TODO - no separate sub-panel backlight control? */

} /* lcd_enable_sub_backlight */

/****************************************************************************
*
*  lcd_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

/* static */
void __exit
lcd_exit(void)
{
    LCD_PUTS("enter");

    lcd_backlight_enable(0);

    if (gUpdateThreadPid >= 0)
    {
   	    kill_proc_info(SIGTERM, SEND_SIG_PRIV,gUpdateThreadPid);
	    wait_for_completion(&gUpdateExited);
    }

    gpio_free( LCD_Reset_GPIO );
    gpio_free( LCD_Backlight_GPIO );

} /* lcd_exit */


static int
lcd_alloc_fb(LCD_dev_info_t *dev)
{
    if (dev->frame_buffer.virtPtr != NULL)
        return 0;

    /*
     * dma_alloc_writecombine allocates uncached, buffered memory, that is 
     * io_remappable 
     */
    dev->frame_buffer.sizeInBytes = dev->width * dev->height *
	    dev->bits_per_pixel / 8;

    dev->frame_buffer.virtPtr = dma_alloc_writecombine(NULL,
            dev->frame_buffer.sizeInBytes, &dev->frame_buffer.physPtr,
	    GFP_KERNEL);

    LCD_DEBUG("size=%#x virtPtr = 0x%lx, physPtr = 0x%lx\n",
	    dev->frame_buffer.sizeInBytes,
	    (long)dev->frame_buffer.virtPtr, (long)dev->frame_buffer.physPtr);

    if (dev->frame_buffer.virtPtr == NULL)
	return -ENOMEM;

    if ((dev->frame_buffer.physPtr & ~PAGE_MASK) != 0)
    {
        panic("lcd_init: We didn't get a page aligned buffer");
        return -ENOMEM;
    }

    memset(dev->frame_buffer.virtPtr, 0, dev->frame_buffer.sizeInBytes);
    return 0;
}

/****************************************************************************
*
*  lcd_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

static int __init
lcd_init(void)
{
    int rc;
    int i;

    LCD_PUTS("enter");
    printk("%s for %s\n", gBanner, LCD_panel_name);

    /* Register our device with Linux */
    if ((rc = register_chrdev(BCM_LCD_MAJOR, "lcd", &lcd_fops)) < 0)
    {
        printk(KERN_WARNING "lcd: register_chrdev failed for major %d\n",
		BCM_LCD_MAJOR);
        return rc;
    }

    /* Allocate memory for the framebuffers and DMA. */
    for (i = 0; i < LCD_num_panels; i++)
    {
	LCD_dev_info_t *dev = &LCD_device[i];

	rc = lcd_alloc_fb(dev);

	if (rc)
	    return rc;

#if USE_DMA
	if (lcd_dmaCreateList(dev) != 0)
	{
	    LCD_DEBUG("LCD DBG_ERROR - Failed to create DMA linked list\n");
	    return -ENOMEM;
	}
#endif
    }

#if USE_DMA
    /* request DMA channel and DMA irq, and enable dma irq */
    gDmaChannel = DESIRED_DMA_CHANNEL;

    if (dma_request_chan(gDmaChannel, "LCD") != 0)
    {
        /* couldn't get desired DMA channel, take whatever is available */
        if (dma_request_avail_chan(&gDmaChannel, "LCD") != 0)
        {
            LCD_DEBUG("LCD DBG_ERROR - Failed to get DMA channel\n");
            return -EINVAL;
        }
    }

    if (dma_request_irq(gDmaChannel, lcd_dma_isr, 0) != 0)
    {
        LCD_DEBUG("LCD DBG_ERROR - Failed to get dma irq\n");
        return -EINVAL;
    }

    sema_init(&gDmaSem, 1);

    if (dma_enable_irq(gDmaChannel) != 0)
    {
	LCD_DEBUG("cannot enable DMA IRQ\n");
	return -EINVAL;
    }
#endif	/* USE_DMA */

    /* Create update thread */
    sema_init(&gUpdateSem, 0);
    init_completion(&gUpdateExited);

    gpio_request( LCD_Reset_GPIO, "LCD Reset" );
    gpio_request( LCD_Backlight_GPIO, "LCD Backlight" );


    lcd_pwr_on();

    /* launch thread */
    gUpdateThreadPid = kernel_thread(lcd_update_thread, 0, 0);

#if 0
    /* Register our device with the Power Manager */
    if ((rc = pm_register_component(PM_COMP_LCD, &lcd_pm_ops)) < 0)
    {
        printk("lcd: failed to register with power manager\n");
        return rc;
    }
#endif

    gInitialized = 1;
    LCD_PUTS("done");
    return 0;
}


#if 0
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

#endif

/**
*  This function sets up the IOCR registers for LCD, based on Baseband Chip.
*
*/
static void
lcd_iocr_init(void)
{
    LCD_Intf_t intf_type = LCD_Intf;

#ifdef XCFG_BCM21351
    LCD_Bus_t bus = LCD_Bus;
    uint32_t val = 0;
    uint32_t mask = 0;

    LCD_PUTS("enter");
    /*
     * --------Set IOCR0 for Lcd interface 
     *    29:28   LCD     LCD/SPI/SD2/ARM9 
     *            0       LCD interface pin select 
     *            1       MSPRO/SPI 
     *            2       SD2 on LCDD0-9 
     *            3       ARM9 TRACE/SPI 
     *    10      UART RX/TX on LCDD16/17 UARTB/LCDD 
     *            0       LCDD16/17 
     *            1       UBRX/UBTX on LCDD16/17 
     *    2               1 M68   Select M68 interface on LCD 
     */

    /* LCD Interface */
    if ((intf_type == LCD_Z80) || (intf_type == LCD_M68))
    {
        mask |= ((3 << 28) | (1 << 2));
        val = (0 << 28);
        if (intf_type == LCD_M68)
        {
            val |= (1 << 2);
        }
    }
    /* LCD D16/17 select */
    if (bus == LCD_18BIT)
    {
        mask |= (1 << 10);
    }

    /* Set IOCR0 for LCD */
    regaccess_and_bits(&REG_SYS_IOCR0, ~mask);
    regaccess_or_bits(&REG_SYS_IOCR0, val);

    /* --------Set IOCR1 for Lcd interface */

    /* --------Set IOCR2 for Lcd interface */

#if 0
    /*
     * --------Set IOCR3 for Lcd interface 
     *    3       LCD_DIS  
     *            0       normal operation 
     *            1       LCD pads output disabled, pull down. 
     */
    mask = 0;
    val = 0;
    /* LCD DIS set for normal operation */
    mask = (1 << 3);
    val = 0;
    /* Set IOCR3 for LCD */
    *(volatile uint32_t *)REG_SYS_IOCR3 &= ~mask;
    *(volatile uint32_t *)REG_SYS_IOCR3 |= val;
#endif

#if 0
    /*
     * --------Set IOCR4 for Lcd interface 
     *  30              LCD slew         
     * 			Slew control on LCD pads (Default = 0) 
     *  23:21   LCD drive        
     * 			Control drive strength of LCD pads (Default = 101) 
     */
    mask = 0;
    val = 0;
    /* LCD Slew */
    mask |= (1 << 30);
    val |= (0 << 30);
    /* LCD Drive (default value 101b) */
    mask |= (7 << 21);
    val |= (5 << 21);
    /* Set IOCR4 for LCD */
    *(volatile uint32_t *)REG_SYS_IOCR4 &= ~mask;
    *(volatile uint32_t *)REG_SYS_IOCR4 |= val;
#endif

#else
    LCD_Volt_t volt = LCD_Volt;

    LCD_PUTS("enter");
    /*
     * --------Set IOCR0 for Lcd interface 
     *    29:28   LCD     LCD/SPI/SD2/ARM9 
     *            0       LCD Z80 interface 
     *            1       LCD M68 interface 
     *            2       SPI interface 
     *            3       ARM9 TRACE 
     * LCD Interface 
     */
    regaccess_and_bits(&REG_SYS_IOCR0, ~(3 << 28));
    regaccess_or_bits(&REG_SYS_IOCR0, (intf_type << 28));

    /*
     * --------Set IOCR2 for Lcd interface 
     *    30      LCD Drive Voltage Select         
     *            0       1.8v 
     *            1       3.0v 
     * LCD voltage in IOCR2 
     */
    regaccess_and_bits(&REG_SYS_IOCR2, ~(1 << 30));
    regaccess_or_bits(&REG_SYS_IOCR2, volt << 30);

    mdelay(100);
#endif
}



/****************************************************************************
*
*  lcd_pwr_on
*
*     Power on controller
*
***************************************************************************/
static int
lcd_pwr_on(void)
{
    LCD_PUTS("enter");

    if (!gInitialized)
    {
	/* first init IOCR registers for LCD I/O */
	lcd_iocr_init();

        /* Configure the GPIO pins */
	LCD_PUTS("gpio");
        gpio_direction_output( LCD_Reset_GPIO, 1 );
        gpio_direction_output( LCD_Backlight_GPIO, 1 );

        /* Initialize our side of the controller hardware */
	LCD_PUTS("R/WTR");
	REG_LCD_RTR = LCD_Read_Timing;
	REG_LCD_WTR = LCD_Write_Timing;

	/* put control register into a known state - also selects main panel */
        REG_LCD_CR = 0;

	if (LCD_Bus == LCD_18BIT)
	    REG_LCD_CR |= REG_LCD_CR_ENABLE_16_TO_18_EXPANSION;
    }

    /* Initialize controller */
    lcd_init_all();
    return 0;
}                               /* lcd_pwr_on */

static void
lcd_init_all(void)
{
    LCD_PUTS("enter");

    if (gInitialized)
	return;

    /* drop then raise the RESET line */
    lcd_reset_controller(0);
    mdelay(100);
    lcd_reset_controller(1);
    mdelay(10);

    lcd_init_panels();

} /* lcd_init_all */

/****************************************************************************
*
*  lcd_pwr_off_controller
*
*   Power off the LCD controller.
*
***************************************************************************/

static void
lcd_pwr_off_controller(void)
{
    LCD_PUTS("enter");

    /* TODO */

} /* lcd_pwr_off_controller */

/****************************************************************************
*
*  lcd_ioctl - TODO - lots of stuff needs to be filled in
*
***************************************************************************/

static int
lcd_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
    int err = 0;
    LCD_dev_info_t *dev = (LCD_dev_info_t*)file->private_data;

    LCD_DEBUG("dev: %d  type: '%c' cmd: 0x%x\n", dev->panel,
	    _IOC_TYPE(cmd), _IOC_NR(cmd));
    
    switch (cmd)
    {
    case LCD_IOCTL_RESET:
	lcd_reset_controller((int)arg);
	break;

    case LCD_IOCTL_ENABLE_BACKLIGHT:
	lcd_backlight_enable((int)arg);
	break;

    case LCD_IOCTL_ENABLE_SUB_BACKLIGHT:
	lcd_enable_sub_backlight((int)arg);
	break;

    case LCD_IOCTL_ENABLE_CS:
	break;

    case LCD_IOCTL_SCOPE_TIMEOUT:
	break;

    case LCD_IOCTL_INIT:
	  lcd_init_panels();
	  break;

    case LCD_IOCTL_INIT_ALL:
	  lcd_init_all();
	  break;
	  
    case LCD_IOCTL_SETUP:
	break;

    case LCD_IOCTL_HOLD:
	break;

    case LCD_IOCTL_PULSE:
	break;

    case LCD_IOCTL_REG:
    {
	LCD_Reg_t r;

	if (copy_from_user(&r, (LCD_Reg_t *)arg, sizeof(r)) != 0)
	{
	    return -EFAULT;
	}

	/*lcd_write_cmd(r.reg, r.val); */
	break;
    }

    case LCD_IOCTL_RECT:
    {
        LCD_Rect_t r;

	if (copy_from_user(&r, (LCD_Rect_t *)arg, sizeof(r)) != 0)
	{
	    return -EFAULT;
	}

        lcd_display_rect(dev,&r);

	break;
    }

    case LCD_IOCTL_COLOR_TEST:
	break;

    case LCD_IOCTL_DIRTY_ROWS:
    {
	LCD_DirtyRows_t dirtyRows;

	if (copy_from_user(&dirtyRows, (LCD_DirtyRows_t *)arg,
		sizeof dirtyRows) != 0)
	{
	    return -EFAULT;
	}

	lcd_dev_dirty_rows(dev, &dirtyRows);
	break;
    }

	  case LCD_IOCTL_PRINT_REGS:
		lcd_dump_regs();
		break;
		
    case LCD_IOCTL_PRINT_DATA:
	break;

    case LCD_IOCTL_PWR_OFF:
	lcd_pwr_off_controller();
	break;

    case LCD_IOCTL_INFO:
	  LCD_DEBUG("cmd=LCD_IOCTL_INFO arg=0x%08lX", arg);
	  {		
		LCD_Info_t lcdInfo;
		
		lcd_get_info(&lcdInfo);
		err = copy_to_user((void *)arg, &lcdInfo, sizeof(LCD_Info_t));
		
		break;
	  }


    default:
	LCD_DEBUG("Unrecognized ioctl: '0x%x'\n", cmd);
	return -ENOTTY;
    }

    return (err);

} /* lcd_ioctl */


/* LCD driver hook for direct DMA CAM->LCD - used by camera_tc.c only */
void
lcd_cam_hook(int action)
{
    LCD_DEBUG("action=%d\n", action);

    switch (action)
    {
    case 0:
	REG_LCD_CR &= ~(REG_LCD_CR_ENABLE_DMA | REG_LCD_CR_ENABLE_DMA_WORDSWAP);
	break;

    case 1:
	lcd_select_panel(LCD_main_panel);
	lcd_setup_for_data(&LCD_device[LCD_main_panel]);

#ifdef __LITTLE_ENDIAN
        REG_LCD_CR |= (REG_LCD_CR_ENABLE_DMA);
#else
        REG_LCD_CR |= (REG_LCD_CR_ENABLE_DMA | REG_LCD_CR_ENABLE_DMA_WORDSWAP);
#endif
	break;
    }
}


/****************************************************************************
*
*  lcd_get_info
*
*
*
***************************************************************************/

void
lcd_get_info(LCD_Info_t *lcdInfo)
{
    LCD_dev_info_t *dev = &LCD_device[LCD_main_panel];

    lcdInfo->bitsPerPixel  = dev->bits_per_pixel;
    lcdInfo->height        = dev->height;
    lcdInfo->width         = dev->width;
}


/****************************************************************************
*
*  lcd_mmap
*
*   Note that the bulk of this code came from the fb_mmap routine found in
*   drivers/video/fbmem.c
*
***************************************************************************/

static int
lcd_mmap(struct file *file, struct vm_area_struct * vma)
{
    unsigned long   offset;
    unsigned long   start;
    unsigned long   len;
    LCD_dev_info_t *dev;
    LCD_FrameBuffer_t *fb;

    /*
     * vma->vm_start    is the start of the memory region, in user space 
     * vma->vm_end      is one byte beyond the end of the memory region, in user space 
     * vma->vm_pgoff    is the offset (in pages) within the vm_start to vm_end region 
     */

    LCD_PUTS("enter");

    if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
    {
        printk("lcd: vm_pgoff is out of range\n");
        return -EINVAL;
    }

    if (file == NULL || file->private_data == NULL)
    {
        printk("lcd: bad file pointer or LCD-device pointer\n");
        return -EINVAL;
    }

    dev = (LCD_dev_info_t*)file->private_data;
    fb = &dev->frame_buffer;

    /* Convert offset into a byte offset, rather than a page offset */
    offset = vma->vm_pgoff << PAGE_SHIFT;
    start = (unsigned long)fb->physPtr; /* already page-aligned */
    len = PAGE_ALIGN(start + fb->sizeInBytes);

    if (offset > len)
    {
        /* The pointer requested by the user isn't inside of our frame buffer */
        LCD_DEBUG("offset is too large, offset = %lu, len = %lu\n",
		offset, len);
        return -EINVAL;
    }

    vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
    offset += start;
    vma->vm_pgoff = offset >> PAGE_SHIFT;

    if (0 != io_remap_pfn_range(vma,
                                  vma->vm_start,
                                  offset >> PAGE_SHIFT,
                                  vma->vm_end - vma->vm_start,
                                  vma->vm_page_prot))
    {
        LCD_DEBUG("remap_page_range failed\n");
        return -EAGAIN;
    }

    return 0;

} /* lcd_mmap */

/****************************************************************************
*
*  lcd_open
*
***************************************************************************/

static int
lcd_open(struct inode *inode, struct file *file)
{
    int minor = iminor(inode);

    LCD_DEBUG("major = %d, minor = %d\n", imajor(inode),  minor);

    /* minor number must match values for LCD_panel_t */
    if (minor < 0 || minor >= LCD_num_panels)
    {
        printk("lcd: bad minor number %d; range is 0..%d\n",
		minor, LCD_num_panels - 1);
        return -EINVAL;
    }

    /* set our private pointer to the correct LCD_dev_info_t */
    file->private_data = (void*)&LCD_device[minor];

    /*
     * XXX - hack to see if LCD update task works 
	/ *    lcd_display_test((LCD_dev_info_t*)file->private_data); 
	/ */
    /*lcd_dump_regs(); */

    return 0;

} /* lcd_open */

/****************************************************************************
*
*  lcd_release
*
***************************************************************************/

static int
lcd_release(struct inode *inode, struct file *file)
{
    LCD_PUTS("enter");

    return 0;

} /* lcd_release */

/****************************************************************************
*
*  lcd_reset_controller
*
*   Resets the controller for use.
*
***************************************************************************/

static void
lcd_reset_controller(int level)
{
    LCD_DEBUG("%d\n", level);

    gpio_set_value(LCD_Reset_GPIO, level);

} /* lcd_reset_controller */


/****************************************************************************
*
*  lcd_get_framebuffer_addr
*
*   Gets the address of the primary frame buffer
*
***************************************************************************/

void *
lcd_get_framebuffer_addr(int *frame_size, dma_addr_t *dma_addr)
{
    int rc;
    LCD_dev_info_t *dev = &LCD_device[LCD_main_panel];

    /*
     *  Check if we have been initialized yet.  If not, another driver wants 
     *  access to our framebuffer before we have been inited.  In this case, 
     *  allocate the framebuffer now to avoid the other driver failing. 
     */
    /*	(lcd_alloc_fb() takes care not to reinitialize itself.) */

    rc = lcd_alloc_fb(dev);

    if (rc)
	return NULL;

    if (dma_addr)
        *dma_addr = dev->frame_buffer.physPtr;

    if (frame_size)
        *frame_size = dev->frame_buffer.sizeInBytes;

    return dev->frame_buffer.virtPtr;

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

static void
lcd_start_dma(LCD_dev_info_t *dev, unsigned int top, unsigned int bottom)
{
    DMA_LLI_t *linkedList;
    DMA_LLI_t *startp;

    LCD_DEBUG("enter; top=%d bot=%d \n", top, bottom);

    /* bounds checking to avoid writing out of bounds */
    if ((top >= 0) && (top < dev->height) &&
	    (bottom >= 0) && (bottom < dev->height))
    {
        /* need a delay here to ensure previous commands to LCD controller
           are complete */
        mdelay(1);

        /* enable DMA for the LCD controller */
#ifdef __LITTLE_ENDIAN
        REG_LCD_CR |= (REG_LCD_CR_ENABLE_DMA);
#else
        REG_LCD_CR |= (REG_LCD_CR_ENABLE_DMA | REG_LCD_CR_ENABLE_DMA_WORDSWAP);
#endif
        /* get pointers to linked list and start LLI */
        linkedList = (DMA_LLI_t *)dev->dma_linked_list.virtPtr;
        startp = &linkedList[top];

        /* save LLI info for the end LLI link and control registers */
        gDmaSavedLli.list = linkedList;
        gDmaSavedLli.row = bottom;
        gDmaSavedLli.link = linkedList[bottom].link;
        gDmaSavedLli.control = linkedList[bottom].control;
        gDmaSavedLli.valid = 1;

        /* now make the end LLI the end of the list */
        linkedList[bottom].link = 0;
        linkedList[bottom].control =
		DMA_SWAP((dev->width * BYTES_PER_PIXEL / DMA_DIVIDE_WIDTH) |
			DMA_CONTROL | REG_DMA_CHAN_CTL_TC_INT_ENABLE);

        /* start the transfer */
		LCD_DEBUG("ch=%d src=0x%08X dst=0x%08X link=0x%08X ctrl=0x%08X cfg=0x%08X\n",
				  gDmaChannel, DMA_SWAP(startp->source), DMA_SWAP(startp->dest),
				  DMA_SWAP(startp->link), DMA_SWAP(startp->control), DMA_CONFIG);
		
        dma_setup_chan(gDmaChannel, DMA_SWAP(startp->source), DMA_SWAP(startp->dest),
                DMA_SWAP(startp->link), DMA_SWAP(startp->control), DMA_CONFIG);
		
		LCD_PUTS("DMA started");
    }
    else
    {
        /* no DMA processing to be done, release the DMA semaphore */
        up(&gDmaSem);
	LCD_PUTS("no DMA");
    }
}                               /* lcd_start_dma */

/****************************************************************************
*
*  lcd_dma_isr
*
*  This isr is triggered when a memory to LCD transfer has completed.
*
***************************************************************************/
static irqreturn_t
lcd_dma_isr(void *unused)
{
    DMA_LLI_t *linkedList;

    LCD_PUTS("enter");
#if 1
    /* disable DMA mode in LCD controller */
#ifdef __LITTLE_ENDIAN
    REG_LCD_CR &= ~(REG_LCD_CR_ENABLE_DMA);
#else
    REG_LCD_CR &= ~(REG_LCD_CR_ENABLE_DMA | REG_LCD_CR_ENABLE_DMA_WORDSWAP);
#endif
#endif
    /* restore the LLI at the end of the list */
    if (gDmaSavedLli.valid)
    {
        linkedList = gDmaSavedLli.list;
        linkedList[gDmaSavedLli.row].link = gDmaSavedLli.link;
        linkedList[gDmaSavedLli.row].control = gDmaSavedLli.control;
        gDmaSavedLli.valid = 0;
    }
    /* show that DMA to LCD is complete */
    up(&gDmaSem);

    LCD_PUTS("handled");
    return IRQ_HANDLED;
}

/****************************************************************************
*
*  lcd_dmaCreateList
*
*  Create the linked lists of DMA descriptors
*
***************************************************************************/
static int
lcd_dmaCreateList(LCD_dev_info_t *dev)
{
    int i;
    int width;
    DMA_LLI_t *list;

    LCD_PUTS("enter");

    /* allocate memory for DMA linked list (1 LLI per LCD row) */
    dev->dma_linked_list.sizeInBytes = dev->height * DMA_BYTES_PER_LLI;
    dev->dma_linked_list.virtPtr = dma_alloc_coherent(NULL,
            dev->dma_linked_list.sizeInBytes,
	    &dev->dma_linked_list.physPtr, GFP_KERNEL);

    LCD_DEBUG("virtPtr = 0x%lx, physPtr = 0x%lx\n",
	    (long)dev->dma_linked_list.virtPtr,
	    (long)dev->dma_linked_list.physPtr);

    if (dev->dma_linked_list.virtPtr == NULL)
    {
	LCD_DEBUG("cannot allocate DMA-safe memory for LLIs");
        return -ENOMEM;
    }

    list = (DMA_LLI_t *)dev->dma_linked_list.virtPtr;
    width = dev->width * BYTES_PER_PIXEL;

    /* The LLI data must be stored little endian when DMA is configured
       for big endian. */

    /* create all but the last LLI */
    for (i = 0; i < dev->height - 2; i++)
    {
        list[i].source = DMA_SWAP(dev->frame_buffer.physPtr + width * i);
        list[i].dest = DMA_SWAP(REG_LCD_DATR_PADDR);
        list[i].link = DMA_SWAP(dev->dma_linked_list.physPtr +
		DMA_BYTES_PER_LLI * (i + 1));
        list[i].control = DMA_SWAP((width / DMA_DIVIDE_WIDTH) | DMA_CONTROL);
    }

    /* create the last LLI */
    list[i].source = DMA_SWAP(dev->frame_buffer.physPtr + width * i);
    list[i].dest = DMA_SWAP(REG_LCD_DATR_PADDR);
    list[i].link = 0;
    list[i].control = DMA_SWAP((width / DMA_DIVIDE_WIDTH) |
	    DMA_CONTROL | REG_DMA_CHAN_CTL_TC_INT_ENABLE);

    gDmaSavedLli.valid = 0;

    LCD_PUTS("done");
    return 0;
}
#endif	/* USE_DMA */


/* --------------------------------------------------------------------------
** Stubs.
*/

int
lcd_is_dirty_row_update_supported(void)
{
   return (0);
}

int
lcd_is_display_regions_supported(void)
{
    return 0;
}

void
lcd_backlight_enable(LCD_BACKLIGHT_LEVEL level)
{
#if LCD_Backlight_GPIO != 17
    gpio_set_value(LCD_Backlight_GPIO, level ? 1 : 0);
#endif
}


/* XXX - temp hacks */
void
lcd_display_test(LCD_dev_info_t *dev)
{
    int i, j;
    uint16_t *fb;
    LCD_DirtyRows_t dirtyRows;

    fb = dev->frame_buffer.virtPtr;
	
    for (i = 50; i < 100; i++)
	  for (j = 50; j < 100; j++)
	    if (i < j)
		  fb[i * dev->width + j] = LCD_COLOR_YELLOW >> 1;
	
    dirtyRows.top = 50;
    dirtyRows.bottom = 100;
    lcd_dev_dirty_rows(dev, &dirtyRows);
}

void
lcd_display_rect(LCD_dev_info_t *dev, LCD_Rect_t *r)
{
    int i, j;
    uint16_t *fb;
    LCD_DirtyRows_t dirtyRows;

    fb = dev->frame_buffer.virtPtr;

    if(r->top < 0)    
        r->top = 0;
    if(r->top > dev->height) 
        r->top = dev->height;
    if(r->left < 0)   
        r->left = 0;
    if(r->left > dev->width)
        r->left = dev->width;

    for (i = r->top; i < (r->top + r->height); i++)
	for (j = r->left; j < (r->left + r->width); j++)
            fb[i * dev->width + j] = r->color;

    dirtyRows.top = r->top;
    dirtyRows.bottom = r->top + r->height;

    if(dirtyRows.bottom >= dev->height)
        dirtyRows.bottom -= 1;

    lcd_dev_dirty_rows(dev, &dirtyRows);
}


/****************************************************************************/

module_init(lcd_init);
module_exit(lcd_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom L2F50219P00 LCD Driver");
MODULE_LICENSE("GPL v2");

