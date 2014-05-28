/* drivers/rtc/rtc-bcmring.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * BCMRING RTC Driver
*/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/clk.h>

#include <csp/rtcHw.h>
#include <mach/csp/rtcHw_inline.h>
#include <mach/csp/cap.h>

static unsigned int epoch = 1970;
static DEFINE_SPINLOCK( bcmring_rtc_lock );
static struct clk *bbl_clk;

/* IRQ Handlers */

/*
 * RTC IRQ hanlder. This routine is invoked when a RTC oneshot timer completes
 */
static irqreturn_t
rtc_alm_isr( int irq, void *data )
{
   struct rtc_device *rdev = data;
   uint32_t intstatus;

   intstatus = rtcHw_readReg( rtchw_INTERRUPT_STATUS_ADDR );
   pr_debug( "%s: interrupted intstatus=0x%x\n", __func__, intstatus );
   /* Disable alarm interrupts because they are oneshot */
   rtcHw_disableOneshotInt();
   rtcHw_clearOneshotTimerInterrupt();
   if ( intstatus & rtchw_CMD_ONESHOT_INTERRUPT_STATUS )
   {
      pr_debug( "%s: oneshot interrupted\n", __func__ );
      rtc_update_irq( rdev, 1, RTC_AF | RTC_IRQF );
   }
   return IRQ_HANDLED;
}

/*
 * RTC IRQ hanlder. This routine is invoked when periodic interrupts occur
 */
static irqreturn_t
rtc_per_isr( int irq, void *data )
{
   struct rtc_device *rdev = data;
   uint32_t intstatus;

   intstatus = rtcHw_readReg( rtchw_INTERRUPT_STATUS_ADDR );
   rtcHw_clearPeriodicTimerInterrupt();
   pr_debug( "%s: interrupted intstatus=0x%x\n", __func__, intstatus );
   if ( intstatus & rtchw_CMD_PERIODIC_INTERRUPT_STATUS )
   {
      pr_debug( "%s: periodic interrupted\n", __func__ );
      rtc_update_irq( rdev, 1, RTC_PF | RTC_IRQF );
   }
   return IRQ_HANDLED;
}

/* Update control registers */
static void
bcmring_rtc_setaie( int to )
{
   pr_debug( "%s: aie=%d\n", __func__, to );

   if ( to )
   {
      rtcHw_enableOneshotInt();
   }
   else
   {
      rtcHw_disableOneshotInt();
   }
}

static int
bcmring_rtc_setpie( struct device *dev, int enabled )
{
   pr_debug( "%s: pie=%d\n", __func__, enabled );

   spin_lock_irq( &bcmring_rtc_lock );

   if ( enabled )
   {
      rtcHw_startPeriodicTimer(); /* enables the interrupt */
   }
   else
   {
      rtcHw_stopPeriodicTimer();  /* disables the interrupt */
   }
   spin_unlock_irq( &bcmring_rtc_lock );

   return 0;
}

static int
bcmring_rtc_setfreq( struct device *dev, int freq )
{
   rtcHw_INTERVAL_e interval;

   pr_debug( "%s: freq=%d\n", __func__, freq );

   switch ( freq )
   {
      case 1:
         interval = rtcHw_INTERVAL_1000ms;
         break;
      case 2:
         interval = rtcHw_INTERVAL_500ms;
         break;
      case 4:
         interval = rtcHw_INTERVAL_250ms;
         break;
      case 8:
         interval = rtcHw_INTERVAL_125ms;
         break;
      default:
         pr_debug( "%s: BAD freq=%d\n", __func__, freq );
         return -EINVAL;
   }

   pr_debug( "%s: OKAY freq=%d interval=%d\n", __func__, freq, interval );

   spin_lock_irq( &bcmring_rtc_lock );
   rtcHw_setPeriodicTimerInterval( interval );
   spin_unlock_irq( &bcmring_rtc_lock );

   return 0;
}

static int
bcmring_rtc_getfreq( struct device *dev, int *freq )
{
   rtcHw_INTERVAL_e interval;

   spin_lock_irq( &bcmring_rtc_lock );
   interval = rtcHw_readReg( rtchw_PERIODIC_TIMER_ADDR );
   spin_unlock_irq( &bcmring_rtc_lock );

   switch ( interval )
   {
      case rtcHw_INTERVAL_125ms:
         *freq = 8;
         break;
      case rtcHw_INTERVAL_250ms:
         *freq = 4;
         break;
      case rtcHw_INTERVAL_500ms:
         *freq = 2;
         break;
      case rtcHw_INTERVAL_1000ms:
         *freq = 1;
         break;
      default:
         *freq = 0xffffffff; /* invalid */
         pr_debug( "%s: Bad interval=%d\n", __func__, interval );
         return -EINVAL;
   }

   pr_debug( "%s: interval=%d, freq=%d\n", __func__, interval, *freq );

   return 0;
}

/* Time read/write */

static int
bcmring_rtc_gettime( struct device *dev, struct rtc_time *rtc_tm )
{
   unsigned int epoch_sec, elapsed_sec;

   epoch_sec = mktime( epoch, 1, 1, 0, 0, 0 );
   elapsed_sec = rtcHw_getTime();

   pr_debug( "%s: epoch_sec=%u, elapsed_sec=%u\n", __func__, epoch_sec, elapsed_sec );
   rtc_time_to_tm( epoch_sec + elapsed_sec, rtc_tm );

   pr_debug( "read time 0x%02x.0x%02x.0x%02x 0x%02x/0x%02x/0x%02x\n",
             rtc_tm->tm_year, rtc_tm->tm_mon, rtc_tm->tm_mday, rtc_tm->tm_hour, rtc_tm->tm_min, rtc_tm->tm_sec );

   return 0;
}

static int
bcmring_rtc_settime( struct device *dev, struct rtc_time *time )
{
   unsigned int epoch_sec, current_sec;

   epoch_sec = mktime( epoch, 1, 1, 0, 0, 0 );
   current_sec = mktime( time->tm_year + 1900, time->tm_mon + 1, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec );

   rtcHw_setTime( current_sec - epoch_sec );

   pr_debug( "%s: current_sec=%u, epoch_sec=%u\n", __func__, current_sec, epoch_sec );

   pr_debug( "set time %02d.%02d.%02d %02d/%02d/%02d\n", time->tm_year, time->tm_mon, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec );

   return 0;
}

static int
bcmring_rtc_getalarm( struct device *dev, struct rtc_wkalrm *alrm )
{
   unsigned int epoch_sec, elapsed_sec, alarm_elapsed_sec;
   rtcHw_TIME_t alm_reg_secs;
   struct rtc_time *alm_tm = &alrm->time;
   alrm->enabled = rtcHw_isOneshotEnabled();
   alrm->pending = ( rtcHw_readReg( rtchw_INTERRUPT_STATUS_ADDR ) & rtchw_CMD_ONESHOT_INTERRUPT_STATUS ) ? 1 : 0;

   epoch_sec = mktime( epoch, 1, 1, 0, 0, 0 );
   elapsed_sec = rtcHw_getTime();

   alm_reg_secs = rtcHw_getOneshotTimer();

   /* Handle carry over */
   if ((elapsed_sec & 0x0ffff) > alm_reg_secs)
   {
      elapsed_sec += 0x10000;
   }
   elapsed_sec &= ~0xffff; /* clear lower 16 bits for 16-bit alarm match register below */
   alarm_elapsed_sec = elapsed_sec + alm_reg_secs;
   pr_debug( "%s: epoch_sec=%u, elapsed_sec=%u, alm_reg_secs=%u=0x%x, alarm_elapsed_sec=%u=0x%x\n",
             __func__, epoch_sec, elapsed_sec, alm_reg_secs, alm_reg_secs, alarm_elapsed_sec, alarm_elapsed_sec );

   rtc_time_to_tm( epoch_sec + alarm_elapsed_sec, alm_tm );
   pr_debug( "read alarm %02x %02x.%02x.%02x %02x/%02x/%02x\n",
             alrm->enabled, alm_tm->tm_year, alm_tm->tm_mon, alm_tm->tm_mday, alm_tm->tm_hour, alm_tm->tm_min, alm_tm->tm_sec );

   return 0;
}

static int
bcmring_rtc_setalarm( struct device *dev, struct rtc_wkalrm *alrm )
{
   unsigned int epoch_sec, elapsed_sec;
   struct rtc_time *time = &alrm->time;
   rtcHw_TIME_t alm_secs;

   pr_debug( "%s: %d, %02x/%02x/%02x %02x.%02x.%02x\n",
             __func__, alrm->enabled, time->tm_mday & 0xff, time->tm_mon & 0xff, time->tm_year & 0xff, time->tm_hour & 0xff, time->tm_min & 0xff,
             time->tm_sec );

   epoch_sec = mktime( epoch, 1, 1, 0, 0, 0 );
   elapsed_sec = rtcHw_getTime();
   alm_secs = mktime( time->tm_year + 1900, time->tm_mon + 1, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec );

   pr_debug( "%s: epoch_sec=%u, elapsed_sec=%u, alm_secs=%u\n", __func__, epoch_sec, elapsed_sec, alm_secs );

   rtcHw_disableOneshotInt();
   rtcHw_clearOneshotTimerInterrupt();

   rtcHw_setOneshotTimer( alm_secs );

   if (alrm->enabled) {
      rtcHw_enableOneshotInt();
   }


   return 0;
}

static int
bcmring_rtc_proc( struct device *dev, struct seq_file *seq )
{
   seq_printf( seq, "\nperiodic timer: 0x%x\n", rtcHw_readReg( rtchw_PERIODIC_TIMER_ADDR ) );
   seq_printf( seq, "match register: 0x%x\n", rtcHw_readReg( rtchw_MATCH_REGISTER_ADDR ) );
   seq_printf( seq, "rtc register: 0x%x\n", rtcHw_readReg( rtchw_RTC_REGISTER_ADDR ) );
   seq_printf( seq, "clear intr register: 0x%x\n", rtcHw_readReg( rtchw_CLEAR_INTR_ADDR ) );
   seq_printf( seq, "current time register: 0x%x\n", rtcHw_readReg( rtchw_CURRENT_TIME_ADDR ) );
   seq_printf( seq, "intr status register: 0x%x\n", rtcHw_readReg( rtchw_INTERRUPT_STATUS_ADDR ) );
   seq_printf( seq, "control addr register: 0x%x\n", rtcHw_readReg( rtchw_CONTROL_ADDR ) );
   return 0;
}

static int
bcmring_rtc_ioctl( struct device *dev, unsigned int cmd, unsigned long arg )
{
   switch ( cmd )
   {
      case RTC_AIE_OFF:
         spin_lock_irq( &bcmring_rtc_lock );
         bcmring_rtc_setaie( 0 );
         spin_unlock_irq( &bcmring_rtc_lock );
         return 0;
      case RTC_AIE_ON:
         spin_lock_irq( &bcmring_rtc_lock );
         bcmring_rtc_setaie( 1 );
         spin_unlock_irq( &bcmring_rtc_lock );
         return 0;
      case RTC_PIE_OFF:
         spin_lock_irq( &bcmring_rtc_lock );
         rtcHw_stopPeriodicTimer();
         spin_unlock_irq( &bcmring_rtc_lock );
         return 0;
      case RTC_PIE_ON:
         spin_lock_irq( &bcmring_rtc_lock );
         rtcHw_startPeriodicTimer();
         spin_unlock_irq( &bcmring_rtc_lock );
         return 0;
      case RTC_IRQP_READ:
      {
         int freq;
         int ret = bcmring_rtc_getfreq( dev, &freq );
         if ( ret != 0 )
         {
            return ret;
         }
         return put_user( freq, ( unsigned long * ) arg );
      }
      case RTC_IRQP_SET:
         return bcmring_rtc_setfreq( dev, ( int ) arg );
   }
   return -ENOIOCTLCMD;
}

static void
bcmring_rtc_release( struct device *dev )
{
   bcmring_rtc_setaie( 0 );
   bcmring_rtc_setpie( dev, 0 );
}

static const struct rtc_class_ops bcmring_rtcops = {
   .ioctl = bcmring_rtc_ioctl,
   .release = bcmring_rtc_release,
   .read_time = bcmring_rtc_gettime,
   .set_time = bcmring_rtc_settime,
   .read_alarm = bcmring_rtc_getalarm,
   .set_alarm = bcmring_rtc_setalarm,
   .irq_set_freq = bcmring_rtc_setfreq,
   .irq_set_state = bcmring_rtc_setpie,
   .proc = bcmring_rtc_proc,
};

static void
bcmring_rtc_enable( struct platform_device *pdev, int en )
{
   if ( !en )
   {
      rtcHw_stopTimer();
   }
   else
   {
      rtcHw_startTimer();
   }
}

static int __exit
bcmring_rtc_remove( struct platform_device *dev )
{
   struct rtc_device *rtc = platform_get_drvdata( dev );

   device_init_wakeup( &dev->dev, 0 );

   platform_set_drvdata( dev, NULL );

   bcmring_rtc_setpie( &dev->dev, 0 );
   bcmring_rtc_setaie( 0 );

   free_irq( IRQ_RTC1, rtc );
   rtc_device_unregister( rtc );
   
   clk_disable(bbl_clk);
   clk_put(bbl_clk);

   return 0;
}

static int __init
bcmring_rtc_probe( struct platform_device *pdev )
{
   int ret;
   struct rtc_device *rtc;

   pr_debug( "%s: probe=%p\n", __func__, pdev );

   if (cap_isPresent(CAP_BBL, 0) != CAP_PRESENT) {
      printk(KERN_WARNING "RTC is not supported\n");
      return -ENODEV;
   }

   bbl_clk = clk_get(NULL, "BBL");
   ret = IS_ERR(bbl_clk);
   if (ret)
   {
      printk(KERN_ERR "RTC: Unable to find BBL clock\n");
      return ret;
   }

   ret = clk_enable(bbl_clk);
   if (ret)
   {
      printk(KERN_ERR "RTC: Failed to enable BBL clock\n");
      goto err_clk_put;
   }

   bcmring_rtc_enable( pdev, 1 );

   bcmring_rtc_setfreq( &pdev->dev, 1 );

   device_init_wakeup( &pdev->dev, 1 );
   rtcHw_disableOneshotInt();
   rtcHw_stopPeriodicTimer();
   rtcHw_clearOneshotTimerInterrupt();
   rtcHw_clearPeriodicTimerInterrupt();
   rtc = rtc_device_register( "bcmring", &pdev->dev, &bcmring_rtcops, THIS_MODULE );

   if ( IS_ERR( rtc ) )
   {
      ret = PTR_ERR( rtc );
      pr_debug( "cannot attach rtc\n" );
      goto err_device_unregister;
   }

   rtc->max_user_freq = 128;

   ret = request_irq( IRQ_RTC1, rtc_alm_isr, 0, "bcmring-rtc-alm", rtc );
   if ( ret < 0 )
   {
      pr_debug( "IRQ%d error %d\n", IRQ_RTC1, ret );
      goto err_device_unregister;
   }
   ret = request_irq( IRQ_RTC0, rtc_per_isr, 0, "bcmring-rtc-per", rtc );
   if ( ret < 0 )
   {
      pr_debug( "IRQ%d error %d\n", IRQ_RTC0, ret );
      goto err_free_irq1;
   }
   platform_set_drvdata( pdev, rtc );
   return 0;

 err_free_irq1:
   free_irq( IRQ_RTC1, pdev );

 err_device_unregister:
   /*bcmring_rtc_enable (pdev, 0); */
   rtc_device_unregister( rtc );

   clk_disable(bbl_clk);
 err_clk_put:
   clk_put(bbl_clk);
   bbl_clk = NULL;

   return ret;
}

#ifdef CONFIG_PM

/* RTC Power management control */

static int period_cnt;

static int
bcmring_rtc_suspend( struct platform_device *pdev, pm_message_t state )
{
   period_cnt = rtcHw_readReg( rtchw_PERIODIC_TIMER_ADDR );
   return 0;
}

static int
bcmring_rtc_resume( struct platform_device *pdev )
{
   rtcHw_writeReg( rtchw_PERIODIC_TIMER_ADDR, period_cnt );
   return 0;
}
#else
#define bcmring_rtc_suspend NULL
#define bcmring_rtc_resume  NULL
#endif

static struct platform_driver bcmring_rtcdrv = {
   .remove = __exit_p( bcmring_rtc_remove ),
   .suspend = bcmring_rtc_suspend,
   .resume = bcmring_rtc_resume,
   .driver = {
              .name = "bcmring-rtc0",
              .owner = THIS_MODULE,
              },
};

static char __initdata banner[] = "BCMRING RTC, (c) 2009 Broadcom Corporation\n";

static int __init
bcmring_rtc_init( void )
{
   printk( banner );
   return platform_driver_probe( &bcmring_rtcdrv, bcmring_rtc_probe );
}

static void __exit
bcmring_rtc_exit( void )
{
   platform_driver_unregister( &bcmring_rtcdrv );
}

module_init( bcmring_rtc_init );
module_exit( bcmring_rtc_exit );

MODULE_DESCRIPTION( "Broadcom BCMRING RTC Driver" );
MODULE_AUTHOR( "Broadcom Corporation" );
MODULE_LICENSE( "GPL" );
MODULE_ALIAS( "platform:bcmring-rtc" );
