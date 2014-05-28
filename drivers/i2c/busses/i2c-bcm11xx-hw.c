/*****************************************************************************
* Copyright 2006 - 2010 Broadcom Corporation.  All rights reserved.
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





#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/semaphore.h>

#include <mach/reg_i2c.h>

#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/timer.h>

#if defined( CONFIG_ARCH_BCMRING )
#   include <csp/i2cHw.h>
#   include <csp/pcmHw.h>
#   include <mach/csp/gpiomux.h>
#endif

/* ----- global variables ---------------------------------------------	*/

#define I2C_POLL_COMMAND_DONE 1

/* Debug logging */
#ifdef DEBUG
#undef DEBUG
#endif
#define DEBUG 1

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_DATA2	0x20

#if defined( CONFIG_ARCH_BCM2153 ) || defined( CONFIG_ARCH_BCMRING )
#define ADD_TURNAROUND_DELAY    1
#else
#define ADD_TURNAROUND_DELAY    0
#endif

#define I2C_ISR_MASK_ALL    ( REG_I2C_ISR_SES_DONE | REG_I2C_ISR_I2CERR | REG_I2C_ISR_TXFIFOEMPTY | REG_I2C_ISR_NOACK )

#if ADD_TURNAROUND_DELAY

#define BSC_TIM_P_VAL_BIT_SHIFT 3

#if defined( CONFIG_ARCH_BCMRING )
#define BSC_MASTER_CLK_FREQ   I2CHW_REG_MASTER_CLK_FREQ
#else
/* Assumption of fixed 13 MHz clock signal */
#define BSC_MASTER_CLK_FREQ   13000000UL
#endif

#endif

#define DBG_DEFAULT_LEVEL  (DBG_ERROR)
static int i2c_debug_level = DBG_DEFAULT_LEVEL;

#if DEBUG
#   define I2C_DEBUG(level,fmt,args...) do { if (level & i2c_debug_level) printk( "%s: " fmt, __FUNCTION__, ##args ); } while (0)
#else
#   define I2C_DEBUG(level,fmt,args...)
#endif

static unsigned int i2c_errors = 0;
static int          i2c_last_error = 0;
static unsigned int i2c_max_irq_wait = 0;
static unsigned int i2c_max_cmdbusy_cnt = 0;
static unsigned int i2c_max_cmdbusy_clk = 0;

static struct ctl_table gSysCtlLocal[] = {
   {
      .ctl_name      = 1,
      .procname      = "level",
      .data          = &i2c_debug_level,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 2,
      .procname      = "errors",
      .data          = &i2c_errors,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 3,
      .procname      = "max_irq_wait",
      .data          = &i2c_max_irq_wait,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 4,
      .procname      = "max_cmdbusy_cnt",
      .data          = &i2c_max_cmdbusy_cnt,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 5,
      .procname      = "max_cmdbusy_clk",
      .data          = &i2c_max_cmdbusy_clk,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {}
};
static struct ctl_table gSysCtl[] = {
   {
      .ctl_name      = CTL_BCM_I2C,
      .procname      = "i2c",
      .mode          = 0555,
      .child         = gSysCtlLocal
   },
   {}
};

struct bcm11xx_i2c {
	u32 interrupt;
#if !I2C_POLL_COMMAND_DONE
	wait_queue_head_t queue;
#endif
	struct i2c_adapter adap;
   struct ctl_table_header *sysCtlHeader; /* sysctl table */
};

static struct bcm11xx_i2c *i2c = NULL;
static struct clk *i2c_clk;
static struct semaphore i2c_lock;

#if ADD_TURNAROUND_DELAY

static unsigned long i2c_bus_clk_period = 0; /* in micro seconds */

/* Turn-around delay (2 bus clocks time) after write / read needed for BCM2153 BSC */
#define TURN_AROUND_DELAY	(i2c_bus_clk_period * 2)

/*compute the clock period required for the delay between i2c transactions */
static unsigned long bcm11xx_i2c_bus_clk_period(void)
{
   u8 tim, div, p;

   tim = REG_I2C_TIM;
   div = tim & REG_I2C_TIM_DIVMSK;
	p = (tim & REG_I2C_TIM_PMSK) >> BSC_TIM_P_VAL_BIT_SHIFT;

   I2C_DEBUG(DBG_INFO, " tim[%d]\t div[%d]\t p[%d]\n", tim, div, p);

	div = 1 << (4 - div);
	p = 2 * (p + 1) + 1 + 2 * (p + 1) + 2;

	return ((div * p * 1000000 + (BSC_MASTER_CLK_FREQ >> 1) - 1) / BSC_MASTER_CLK_FREQ);
}
#endif

static int bcm11xx_check_cmdbusy( void )
{
   int i = 0;
   timer_tick_count_t   clk = timer_get_tick_count();
   timer_tick_count_t   clk2;

   if ( REG_I2C_ISR & REG_I2C_ISR_CMD_BUSY )
   {
      while ( REG_I2C_ISR & REG_I2C_ISR_CMD_BUSY )		/*wait for I2C CMD not busy */
      {
         if (i > 100000)
         {
            i2c_errors++;
            i2c_last_error = -ETIMEDOUT;
            return -ETIMEDOUT;
         }
         i++;
      }
   }

   clk2 = timer_get_tick_count();
   clk = clk2 - clk;
   if (i > i2c_max_cmdbusy_cnt)
   {
      i2c_max_cmdbusy_cnt = i;
   }
   if (clk > i2c_max_cmdbusy_clk)
   {
      i2c_max_cmdbusy_clk = clk;
   }
   return 0;
}

#if !I2C_POLL_COMMAND_DONE
static irqreturn_t bcm11xx_i2c_isr(int irq, void *dev_id)
{
    if (REG_I2C_ISR)
    {
        /* Read again to allow register to stabilise */
	i2c->interrupt = REG_I2C_ISR;

        REG_I2C_ISR = i2c->interrupt; /* clear interrupts */

        wake_up_interruptible(&i2c->queue);
    }
    return IRQ_HANDLED;
}

static int bcm11xx_wait_interrupt( void )
   int res = 0;
   timer_tick_count_t clk;

   clk = timer_get_tick_count();
   res = wait_event_interruptible_timeout(i2c->queue, i2c->interrupt, HZ);
   clk = timer_get_tick_count() - clk;
   if (clk > i2c_max_irq_wait)
   {
      i2c_max_irq_wait = clk;
   }

   if (res < 0)
   {
      I2C_DEBUG(DBG_ERROR, "interrupted\n");
   }
   else if (!i2c->interrupt)
   {
      I2C_DEBUG(DBG_ERROR, "wait timed out, %d\n", clk);      
      i2c_errors++;
      res = -ETIMEDOUT;
   }
   else
   {
      res = i2c->interrupt;
   }

   i2c->interrupt = 0;

   return res;
}

#else

static int bcm11xx_wait_interrupt( void )
{
   /* wait for I2C Controller Interrupt */
   unsigned int i = 0;
   timer_tick_count_t clk;
   timer_tick_count_t clk2;
   int isr;

   clk = timer_get_tick_count();
   while (( REG_I2C_ISR & I2C_ISR_MASK_ALL ) == 0 )
   {
      i++;
      if (i == (unsigned int)(-1))
      {
         I2C_DEBUG(DBG_ERROR, "wait timed out, %d\n", clk);
         i2c_last_error = -ETIMEDOUT;
         return -ETIMEDOUT;
      }
   }

   clk2 = timer_get_tick_count();
   clk = clk2 - clk;
   if (clk > i2c_max_irq_wait)
   {
      i2c_max_irq_wait = clk;
   }

	isr = REG_I2C_ISR;
	REG_I2C_ISR = isr;

    return isr;
}
#endif

static int bcm11xx_wait_sesdone( int clear_cmd )
{
   int ret;
   u8 isr;
   u8 cs = REG_I2C_CS;

   ret = bcm11xx_wait_interrupt();
   if (ret < 0)
	{
      I2C_DEBUG(DBG_ERROR, "wait interrupt timed out %d cs = 0x%02X\n", ret, cs);
      i2c_errors++;

      /* Clear command */
      if (clear_cmd)
      {
         cs = REG_I2C_CS;
         REG_I2C_CS = (cs & ~REG_I2C_CS_CMDMASK);
      }
      return ret;
   }

   /* Get status from interrupt status register */
   isr = ret;

   /* Clear command */
   cs = REG_I2C_CS;
   if (clear_cmd)
   {
      REG_I2C_CS = (cs & ~REG_I2C_CS_CMDMASK);
   }

	if( isr & REG_I2C_ISR_I2CERR )
	{
      I2C_DEBUG(DBG_ERROR, "bus error detected\n");
      i2c_errors++;
      i2c_last_error = -EIO;
		return -EIO;
	}

   if( isr & REG_I2C_ISR_NOACK )
   {
      I2C_DEBUG(DBG_ERROR, "no ack\n");
      i2c_errors++;
   }

   if( !(isr & REG_I2C_ISR_SES_DONE) )
   {
      I2C_DEBUG(DBG_ERROR, "ses done timedout\n");
      i2c_errors++;
      i2c_last_error = -EIO;
      return -EIO;
   }

	return (cs & REG_I2C_CS_ACK)?0:1;
}

static int bcm11xx_i2c_start( void )
{
   u8 cs;
   int i = 0;
   I2C_DEBUG(DBG_TRACE,"\n");

#if ADD_TURNAROUND_DELAY
   udelay(TURN_AROUND_DELAY);
#endif

   /*
    * Wait for the I2C bus to go idle (SDA = SCL = 1).
    * However, we're the master, so we only need to wait for a few milliseconds for this.
    */

   while (( (cs = REG_I2C_CS) & (REG_I2C_CS_SDA | REG_I2C_CS_SCL) ) != (REG_I2C_CS_SDA | REG_I2C_CS_SCL))
   {
      i2c_errors++;
      I2C_DEBUG(DBG_ERROR, "waiting for cs = 0x%2x [i = %d]\n", cs, i);
      REG_I2C_CS = 0;
      udelay(80); 
      REG_I2C_CS = REG_I2C_CS_SCL;
      udelay(80);
      REG_I2C_CS = REG_I2C_CS_SDA | REG_I2C_CS_SCL;
      udelay(80);
      REG_I2C_CS = REG_I2C_CS_SDA | REG_I2C_CS_SCL | REG_I2C_CS_EN;
      udelay(100);
 
      msleep( 2 );

      if ( ++i >= 16 )
      {
          I2C_DEBUG( DBG_ERROR, "bus appears to be hung - bailing\n" );
          i2c_last_error = -EIO;
          return -EIO;
      }
   }

   /* Wait for controller to be done with last command */
	if ( bcm11xx_check_cmdbusy() < 0 )
   {
      I2C_DEBUG(DBG_ERROR, "cmdbusy always high\n");
      i2c_last_error = -EIO;
      return -EIO;
   }

   /* Send normal start condition */
   REG_I2C_CS = REG_I2C_CS_SDA | REG_I2C_CS_SCL | REG_I2C_CS_CMDSTART | REG_I2C_CS_EN;

   /* Wait for command to be done */
   if ( bcm11xx_wait_sesdone(1) < 0)
   {
      I2C_DEBUG(DBG_ERROR, "sesdone timed out\n");
      i2c_last_error = -ETIMEDOUT;
      return -ETIMEDOUT;
   }

   return 0;
}

static int bcm11xx_i2c_repstart( void )
{
   I2C_DEBUG(DBG_TRACE,"\n");

   /* Wait for controller to be done with last command */
   if ( bcm11xx_check_cmdbusy() < 0 )
   {
      I2C_DEBUG(DBG_ERROR, "cmdbusy always high\n");
      i2c_last_error = -EIO;
      return -EIO;
   }

   /* Send repeated start condition */
   REG_I2C_CS = REG_I2C_CS_SDA | REG_I2C_CS_SCL | REG_I2C_CS_CMDRESTART | REG_I2C_CS_EN;

   /* Wait for command to be done */
   if ( bcm11xx_wait_sesdone(1) < 0)
   {
      I2C_DEBUG(DBG_ERROR, "sesdone timed out\n");
      i2c_last_error = -ETIMEDOUT;
      return -ETIMEDOUT;
   }

   return 0;
}

static int bcm11xx_i2c_stop( void )
{
   u8 cs = REG_I2C_CS;

   I2C_DEBUG(DBG_TRACE,"\n");

   /* Wait for controller to be done with last command */
   if ( bcm11xx_check_cmdbusy() < 0 )
   {
      I2C_DEBUG(DBG_ERROR, "cmdbusy always high\n");
      i2c_last_error = -EIO;
      return -EIO;
   }

   /* Send stop condition */
   REG_I2C_CS = REG_I2C_CS_SDA | REG_I2C_CS_SCL | REG_I2C_CS_CMDSTOP | REG_I2C_CS_EN;

   /* Wait for command to be done */
   if ( bcm11xx_wait_sesdone(1) < 0)
   {
      I2C_DEBUG(DBG_ERROR, "sesdone timed out, cs = 0x%02x\n", cs);
      i2c_last_error = -ETIMEDOUT;
      return -ETIMEDOUT;
   }

   return 0;
}

/* send a byte without start cond., look for arbitration,
   check ackn. from slave */
/* returns:
 * 1 if the device acknowledged
 * 0 if the device did not ack
 * -ETIMEDOUT if an error occurred (while raising the scl line)
 */
static int bcm11xx_i2c_outb(char c)
{
   int ack;

   I2C_DEBUG(DBG_TRACE,"0x%2x\n", c);

   udelay(5);

   /* Wait for controller to be done with last command */
   if ( bcm11xx_check_cmdbusy() < 0 )
   {
      I2C_DEBUG(DBG_ERROR, "cmdbusy always high\n");
      i2c_last_error = -EIO;
      return -EIO;
   }

   /* Send data */
   REG_I2C_CS = REG_I2C_CS_SDA | REG_I2C_CS_SCL | REG_I2C_CS_EN;

   REG_I2C_DAT = (u8)c;

   /* Wait for command to be done */
   if ( (ack = bcm11xx_wait_sesdone(0)) < 0)
   {
      I2C_DEBUG(DBG_ERROR, "sesdone timed out\n");
      i2c_last_error = -ETIMEDOUT;
      return -ETIMEDOUT;
   }

	return ack;
}


static int bcm11xx_i2c_inb(int no_ack)
{
   u8 cs = REG_I2C_CS;
   u8 data;

   I2C_DEBUG(DBG_TRACE,"%d\n", no_ack);

   /* Wait for controller to be done with last command */
   if ( bcm11xx_check_cmdbusy() < 0 )
   {
      I2C_DEBUG(DBG_ERROR, "cmdbusy always high\n");
      i2c_last_error = -EIO;
      return -EIO;
   }

   /* Initiate data read with ACK low */
   REG_I2C_CS = REG_I2C_CS_SDA | REG_I2C_CS_SCL | REG_I2C_CS_CMDREAD | REG_I2C_CS_EN | (no_ack?REG_I2C_CS_ACK:0);

   /* Wait for command to be done */
   if ( bcm11xx_wait_sesdone(1) < 0)
   {
      I2C_DEBUG(DBG_ERROR, "sesdone timed out, cs = 0x%02x\n", cs);
      i2c_last_error = -ETIMEDOUT;
      return -ETIMEDOUT;
   }

   /* Read data */
   data = REG_I2C_DAT;

   udelay(5);
   return (int)data;
}

/* try_address tries to contact a chip for a number of
 * times before it gives up.
 * return values:
 * 1 chip answered
 * 0 chip did not answer
 * -x transmission error
 */
static int bcm11xx_try_address(struct i2c_adapter *i2c_adap,
		       unsigned char addr, int retries)
{
	int i,ret = -1;

   I2C_DEBUG(DBG_TRACE,"0x%02x, %d\n", addr, retries);

	for (i=0;i<=retries;i++) {
		ret = bcm11xx_i2c_outb(addr);
		if (ret==1)
			break;	/* success! */
		bcm11xx_i2c_stop();
		udelay(5);
		if (i==retries)  /* no success */
			break;
		bcm11xx_i2c_start();
		udelay(100);
	}
   udelay(5);
	if (i)
   {
      I2C_DEBUG(DBG_INFO, "Used %d tries to %s client at 0x%02x : %s\n",
		    i+1, addr & 1 ? "read" : "write", addr>>1,
		    ret==1 ? "success" : ret==0 ? "no ack" : "failed, timeout?" );
   }
	return ret;
}

static int bcm11xx_sendbytes(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
	char c;
	const char *temp = msg->buf;
	int count = msg->len;
	unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;
	int retval;
	int wrcount=0;

	while (count > 0) {
		c = *temp;
      I2C_DEBUG(DBG_TRACE2, "writing %2.2X\n", c&0xff);
		retval = bcm11xx_i2c_outb(c);
		if ((retval>0) || (nak_ok && (retval==0)))  { /* ok or ignored NAK */
			count--;
			temp++;
			wrcount++;
		} else { /* arbitration or no acknowledge */
         I2C_DEBUG(DBG_ERROR, "error %d/%d.\n", wrcount, msg->len);
         i2c_errors++;
			bcm11xx_i2c_stop();
			return (retval<0)? retval : -EFAULT;
			        /* got a better one ?? */
		}
	}
	return wrcount;
}

static int bcm11xx_readbytes(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
	int inval;
	int rdcount=0;   	/* counts bytes read */
	char *temp = msg->buf;
	int count = msg->len;

	while (count > 0) {
		inval = bcm11xx_i2c_inb((msg->flags & I2C_M_NO_RD_ACK) || (count == 1));
		if (inval>=0) {
         I2C_DEBUG(DBG_TRACE2, "reading %2.2X\n", inval&0xff);
			*temp = inval;
			rdcount++;
		} else {   /* read timed out */
         i2c_errors++;
			I2C_DEBUG(DBG_ERROR,"timed out.\n");
			break;
		}

		temp++;
		count--;
	}
	return rdcount;
}

/* doAddress initiates the transfer by generating the start condition (in
 * try_address) and transmits the address in the necessary format to handle
 * reads, writes as well as 10bit-addresses.
 * returns:
 *  0 everything went okay, the chip ack'ed, or IGNORE_NAK flag was set
 * -x an error occurred (like: -EREMOTEIO if the device did not answer, or
 *	-ETIMEDOUT, for example if the lines are stuck...)
 */
static int bcm11xx_doAddress(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
	unsigned short flags = msg->flags;
	unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;

	unsigned char addr;
	int ret, retries;

	retries = nak_ok ? 0 : i2c_adap->retries;

	if ( (flags & I2C_M_TEN)  ) {
		/* a ten bit address */
		addr = 0xf0 | (( msg->addr >> 7) & 0x03);
      I2C_DEBUG(DBG_TRACE2, "addr: %d\n", addr);
		/* try extended address code...*/
		ret = bcm11xx_try_address(i2c_adap, addr, retries);
		if ((ret != 1) && !nak_ok)  {
			I2C_DEBUG(DBG_ERROR, "died at extended address code.\n");
         i2c_last_error = -EREMOTEIO;
			return -EREMOTEIO;
		}
		/* the remaining 8 bit address */
		ret = bcm11xx_i2c_outb(msg->addr & 0x7f);
		if ((ret != 1) && !nak_ok) {
			/* the chip did not ack / xmission error occurred */
			I2C_DEBUG(DBG_ERROR, "died at 2nd address code.\n");
         i2c_last_error = -EREMOTEIO;
			return -EREMOTEIO;
		}
		if ( flags & I2C_M_RD ) {
			ret = bcm11xx_i2c_repstart();
         if (ret < 0)
         {
            i2c_last_error = -EIO;
            return -EIO;
         }   
			/* okay, now switch into reading mode */
			addr |= 0x01;
			ret = bcm11xx_try_address(i2c_adap, addr, retries);
			if ((ret!=1) && !nak_ok) {
				I2C_DEBUG(DBG_ERROR, "died at extended address code.\n");
            i2c_last_error = -EREMOTEIO;
				return -EREMOTEIO;
			}
		}
	} else {		/* normal 7bit address	*/
		addr = ( msg->addr << 1 );
		if (flags & I2C_M_RD )
			addr |= 1;
		if (flags & I2C_M_REV_DIR_ADDR )
			addr ^= 1;
		ret = bcm11xx_try_address(i2c_adap, addr, retries);
		if ((ret!=1) && !nak_ok)
      {
         i2c_errors++;
         i2c_last_error = -EREMOTEIO;
			return -EREMOTEIO;
      }
	}

	return 0;
}

/* Master tranfer function */
static int bcm11xx_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[], int num)
{
   /* Based on i2c-algo-bit */
   struct i2c_msg *pmsg;

   int i,ret;
   unsigned short nak_ok;

   down(&i2c_lock);

   /* Send first start */
   ret = bcm11xx_i2c_start();
   if (ret < 0)
   {
      I2C_DEBUG(DBG_INFO, "bcm11xx_i2c_start() returned %d\n", ret);
      up(&i2c_lock);
      return ret;
   }

   /* Loop through all messages */
   for (i = 0; i < num; i++)
   {
      pmsg = &msgs[i];
      nak_ok = pmsg->flags & I2C_M_IGNORE_NAK;

      if (!(pmsg->flags & I2C_M_NOSTART))
      {
         /* Send repeated start only on subsequent messages */
         if (i)
         {
            ret = bcm11xx_i2c_repstart();
            if (ret < 0)
            {
               up(&i2c_lock);
               return ret;
            }
         }
         ret = bcm11xx_doAddress(i2c_adap, pmsg);
         if ((ret != 0) && !nak_ok) {
            I2C_DEBUG(DBG_INFO, "NAK from device addr %2.2x msg #%d\n",msgs[i].addr,i);
            up(&i2c_lock);
            return (ret<0) ? ret : -EREMOTEIO;
         }
      }
      if (pmsg->flags & I2C_M_RD ) {
         /* read bytes into buffer*/
         ret = bcm11xx_readbytes(i2c_adap, pmsg);
         I2C_DEBUG(DBG_INFO, "read %d bytes.\n",ret);
         if (ret < pmsg->len ) {
            up(&i2c_lock);
            return (ret<0)? ret : -EREMOTEIO;
         }
      } else {
         /* write bytes from buffer */
         ret = bcm11xx_sendbytes(i2c_adap, pmsg);
         I2C_DEBUG(DBG_INFO, "wrote %d bytes.\n",ret);
         if (ret < pmsg->len ) {
            up(&i2c_lock);
            return (ret<0) ? ret : -EREMOTEIO;
         }
      }
   }
	ret = bcm11xx_i2c_stop();
    up(&i2c_lock);
	return (ret < 0) ? ret : num;
}


/* Controller initialization function */
static void bcm11xx_i2c_enable_controller(struct bcm11xx_i2c *i2c)
{
   int rc;
   I2C_DEBUG(DBG_TRACE, "");

#if defined( CONFIG_ARCH_BCMRING )
   /* Enable I2C clock via clock framework */
   i2c_clk = clk_get( NULL, "I2C" );
   rc =  IS_ERR( i2c_clk );
   if ( rc )
   {
      printk( KERN_ERR "I2C: Unable to find I2C clock\n" );
   }

   rc = clk_set_rate( i2c_clk, I2CHW_REG_MASTER_CLK_FREQ );
   if (rc) {
      printk( KERN_ERR "I2C: Failed to set the I2C clock to %lu Hz\n",
         I2CHW_REG_MASTER_CLK_FREQ );
   }

   rc = clk_enable( i2c_clk );
   if ( rc )
   {
      printk( KERN_ERR "I2C: Failed to enable I2C clock\n" );
   }

   I2CHw_Init();
#endif

   /* The following lines were added so that calling this function will bring
    * the device out of i2c cmd busy lockup.
    */

   /* Set the I2C host controller reset bit 19 to high to reset the I2C host. */
#if defined( CONFIG_ARCH_BCMRING )   
   chipcHw_softResetEnable(chipcHw_REG_SOFT_RESET_I2CH);
   udelay(10);
   /* Set the I2C host controller reset bit 19 to low to bring the the I2C host *
    * out of reset.                                                             */
   chipcHw_softResetDisable(chipcHw_REG_SOFT_RESET_I2CH);
#endif   
   
   REG_I2C_CS = REG_I2C_CS_SDA | REG_I2C_CS_SCL;      /* reset controller (EN=0) */
   
   REG_I2C_RCM = 0;                                   /* disable CRC */
#if defined( CONFIG_ARCH_BCMRING )
#ifdef CONFIG_TOUCHSCREEN_CY8CTST120_I2C 
   /* Reduce the number of I2C lockups the driver has to handle by     *
    * slowing down the I2C clock frequency for the touch screen slave. */ 
   REG_I2C_TIM = REG_I2C_CLK_71K;                     /* select clock */
#else   
   REG_I2C_TIM = REG_I2C_CLK_93K;                     /* select clock */
#endif   
#else
   REG_I2C_TIM = REG_I2C_CLK_85K;                     /* select clock */
#endif
   REG_I2C_CLKEN = REG_I2C_CLKEN_CLKEN;               /* enable clock */
	REG_I2C_ISR = REG_I2C_ISR_SES_DONE |
                 REG_I2C_ISR_I2CERR |
                 REG_I2C_ISR_TXFIFOEMPTY |
                 REG_I2C_ISR_NOACK;                   /* clear interrupts */
   REG_I2C_IER = REG_I2C_IER_INT_EN |
                 REG_I2C_IER_ERRINT_EN |
                 REG_I2C_IER_NOACK_EN;                /* enable interrupts */
   REG_I2C_TOUT = REG_I2C_TOUT_EN | 0x7F;             /* initialize timeout */
   REG_I2C_FCR = REG_I2C_FCR_FLUSH;                   /* flush fifo */
   
   REG_I2C_CS = REG_I2C_CS_SDA | REG_I2C_CS_SCL | REG_I2C_CS_EN;  /* enable controller */
}

#if 0
/* The following function doesn't seem to be referenced */
static void reset_i2c_master(void)
{
   /* I2C bus is in bad shape. Resetting the slave did not seem to work. 
    * Try resetting the master.
    */
   bcm11xx_i2c_enable_controller(i2c);
}
#endif

/* BCM11xx I2C adaptor and algorithm definitions */
static u32 bcm11xx_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL |
	       I2C_FUNC_10BIT_ADDR | I2C_FUNC_PROTOCOL_MANGLING;
}

static struct i2c_algorithm bcm11xx_algo = {
	.master_xfer = bcm11xx_xfer,
	.functionality = bcm11xx_functionality,
};

static struct i2c_adapter bcm11xx_ops = {
	.owner = THIS_MODULE,
   .name ="bcm11xx-i2c",
	.id = I2C_HW_B_BCM11XX,
	.algo = &bcm11xx_algo,
	.timeout = 1,
	.retries = 1,
   .class = UINT_MAX,
};

/* Module initialization function */
static int i2c_bcm11xx_probe(struct platform_device *pdev)
{
   int rc = 0;


   if (i2c)
   {
      printk("BCM11xx i2c adapter module already initialized.\n");
      return 0;
   }
   
   printk("BCM11xx i2c adapter module\n");

   /* Allocate memory for control sturcture */
   if (!(i2c = kmalloc(sizeof(*i2c), GFP_KERNEL))) {
      return -ENOMEM;
   }
   memset(i2c, 0, sizeof(*i2c));

   i2c->sysCtlHeader = register_sysctl_table( gSysCtl );

#if defined( CONFIG_ARCH_BCMRING )
   {
      /* Setup GPIO MUX */
      gpiomux_rc_e gpiorc;

      gpiorc = gpiomux_requestGroup( gpiomux_group_i2ch, "I2C Host" );
      if ( gpiorc != gpiomux_rc_SUCCESS )
      {
         printk( KERN_ERR "%s i2c failed to request gpio group rc=%u\n", __FUNCTION__, gpiorc );
         return -EBUSY;
      }
   }
#endif

#if !I2C_POLL_COMMAND_DONE
   /* Initialize IRQ and wait queue */
   init_waitqueue_head(&i2c->queue);

   rc = request_irq(IRQ_I2C, bcm11xx_i2c_isr, IRQF_DISABLED, "bcm11xx-i2c", i2c);

   if (rc < 0)
   {
      printk("i2c-bcm11xx: %s failed to attach interrupt, rc = %d\n", __FUNCTION__, rc);
      goto fail_irq;
   }
#endif

   /* Enable controller */
   bcm11xx_i2c_enable_controller(i2c);

#if ADD_TURNAROUND_DELAY
	i2c_bus_clk_period = bcm11xx_i2c_bus_clk_period();
   printk(KERN_DEBUG "%s:%s(): i2c_bus_clk_period[%ld]\n", __FILE__, __FUNCTION__, i2c_bus_clk_period);
#endif

   /* Add I2C adaptor */
   i2c->adap = bcm11xx_ops;

   i2c_set_adapdata(&i2c->adap, i2c);
   
   if ((rc = i2c_add_numbered_adapter(&i2c->adap)) < 0)
   {
      printk("i2c-bcm11xx: %s failed to add adapter, rc = %d\n", __FUNCTION__, rc);
      goto fail_add;
   }

   platform_set_drvdata(pdev, i2c);
   return rc;

fail_add:

#if !I2C_POLL_COMMAND_DONE
   free_irq(IRQ_I2C, 0);
fail_irq:
#endif

   kfree(i2c);
   i2c = NULL;
   return rc;
}

/* Module exit function */
static int i2c_bcm11xx_remove(struct platform_device *pdev)
{
   if (!i2c)
      return 0;

   i2c_del_adapter(&i2c->adap);

#if !I2C_POLL_COMMAND_DONE
   free_irq(IRQ_I2C, i2c);
#endif

   /* unregister sysctl table */
   if ( i2c->sysCtlHeader != NULL )
   {
      unregister_sysctl_table( i2c->sysCtlHeader );
      i2c->sysCtlHeader = NULL;
   }

   kfree(i2c);
   i2c = NULL;

#if defined( CONFIG_ARCH_BCMRING )
   I2CHw_Exit();
   /* Disable I2C clock via clock framework */
   clk_disable( i2c_clk );
   clk_put( i2c_clk );
   /* free gpiomux group */
   gpiomux_freeGroup( gpiomux_group_i2ch );
#endif

   printk("BCM11xx i2c adapter module exited\n");

   return 0;
}

#ifdef CONFIG_PM
/* Power Management Suspend Callback */
static int i2c_bcm11xx_suspend(struct platform_device *pdev, pm_message_t mesg)
{
   down(&i2c_lock);
   clk_disable( i2c_clk );
   return 0;
}

/* Power Management Resume Callback */
static int i2c_bcm11xx_resume(struct platform_device *pdev)
{
   int rc;
   rc = clk_enable( i2c_clk );
   up(&i2c_lock);
   return rc;
}
#else
#define i2c_bcm11xx_suspend    NULL
#define i2c_bcm11xx_resume     NULL
#endif

static struct platform_driver i2c_bcm11xx_driver = {
   .driver = {
      .name = "bcm-i2c",
      .owner = THIS_MODULE,
   },
   .probe   = i2c_bcm11xx_probe,
   .remove  = i2c_bcm11xx_remove,
   .suspend = i2c_bcm11xx_suspend,
   .resume  = i2c_bcm11xx_resume,
};

int __init i2c_bcm11xx_init(void)
{
   init_MUTEX(&i2c_lock);
   return platform_driver_register(&i2c_bcm11xx_driver);
}

static void i2c_bcm11xx_exit( void )
{
   platform_driver_unregister(&i2c_bcm11xx_driver);
}

subsys_initcall( i2c_bcm11xx_init );
module_exit( i2c_bcm11xx_exit );

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("I2C hardware driver for BCM11xx");
MODULE_LICENSE("GPL");


