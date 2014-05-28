/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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
*  dma.c
*
*  PURPOSE:
*
*     This implements the BCM116X dma driver.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>

#include <mach/reg_dma.h>
#include <mach/reg_irq.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <mach/dma.h>

/*
 * ---- Public Variables ------------------------------------------------- 
 * ---- Private Constants and Types -------------------------------------- 
 */

#define DMA_DRIVER_NAME "dma"
#define DMA_MAX_CHANNELS REG_DMA_MAX_CHANNELS

/* Debug logging */
#ifdef DEBUG
#undef DEBUG
#endif
#define DEBUG 1

#define DBG_ERROR    0x01
#define DBG_INFO     0x02
#define DBG_TRACE    0x04
#define DBG_TRACE2	0x08
#define DBG_DATA     0x10
#define DBG_DATA2    0x20

#define DBG_DEFAULT_LEVEL	(DBG_ERROR)

#if DEBUG
#  define DMA_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
#  define DMA_DEBUG(level,x)
#endif

/* ---- Private Function Prototypes -------------------------------------- */

static irqreturn_t dma_isr(int irq, void *dev_id);

/* ---- Private Variables ------------------------------------------------ */

typedef struct dma_chan
{
   int lock;                    /* lock flag */
   const char name[64];         /* name of peripheral or memory */
     irqreturn_t(*handler) (void *);    /* presence implies registered */
   void *dev_id;                /* argument for handler */
   int called;                  /* Number of times interrupt called since bound */
   int dmaError;                /* DMA errors count */
}
dma_chan;

static struct dma_chan dma_state[DMA_MAX_CHANNELS];

static char banner[] __initdata = KERN_INFO "DMA Driver: 1.00 (built on " __DATE__ " " __TIME__ ")\n";

static int gLevel = DBG_DEFAULT_LEVEL;
static int gIsr = 0;
static int gIrqs = 0;

/* sysctl */
static struct ctl_table_header *gSysCtlHeader = NULL;

#define BCM_SYSCTL_DMA_STRUCT(ID)      {                 \
   {                                                     \
      .ctl_name      = BCM_SYSCTL_DMA_LOCK,              \
      .procname      = "lock",                           \
      .data          = &dma_state[ID].lock,              \
      .maxlen        = sizeof( int ),                    \
      .mode          = 0644,                             \
      .proc_handler  = &proc_dointvec                    \
   },                                                    \
   {                                                     \
      .ctl_name      = BCM_SYSCTL_DMA_NAME,              \
      .procname      = "name",                           \
      .data          = &dma_state[ID].name,              \
      .maxlen        = sizeof( dma_state[ID].name ),     \
      .mode          = 0644,                             \
      .proc_handler  = &proc_dostring                    \
   },                                                    \
   {                                                     \
      .ctl_name      = BCM_SYSCTL_DMA_CALLED,            \
      .procname      = "called",                         \
      .data          = &dma_state[ID].called,            \
      .maxlen        = sizeof( int ),                    \
      .mode          = 0644,                             \
      .proc_handler  = &proc_dointvec                    \
   },                                                    \
   {                                                     \
      .ctl_name      = BCM_SYSCTL_DMA_ERRORS,            \
      .procname      = "dmaError",                       \
      .data          = &dma_state[ID].dmaError,          \
      .maxlen        = sizeof( int ),                    \
      .mode          = 0644,                             \
      .proc_handler  = &proc_dointvec                    \
   },                                                    \
   {}                                                    \
}

static struct ctl_table gSysCtl0[] = BCM_SYSCTL_DMA_STRUCT(0);
static struct ctl_table gSysCtl1[] = BCM_SYSCTL_DMA_STRUCT(1);
static struct ctl_table gSysCtl2[] = BCM_SYSCTL_DMA_STRUCT(2);
static struct ctl_table gSysCtl3[] = BCM_SYSCTL_DMA_STRUCT(3);
static struct ctl_table gSysCtlLocal[] = {
   {
      .ctl_name      = BCM_SYSCTL_DMA_LEVEL,
      .procname      = "level",
      .data          = &gLevel,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = BCM_SYSCTL_DMA_ISR,
      .procname      = "isr",
      .data          = &gIsr,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = BCM_SYSCTL_DMA_IRQS,
      .procname      = "irqs",
      .data          = &gIrqs,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = BCM_SYSCTL_DMA_0,
      .procname      = "0",
      .mode          = 0555,
      .child         = gSysCtl0
   },
   {
      .ctl_name      = BCM_SYSCTL_DMA_1,
      .procname      = "1",
      .mode          = 0555,
      .child         = gSysCtl1
   },
   {
      .ctl_name      = BCM_SYSCTL_DMA_2,
      .procname      = "2",
      .mode          = 0555,
      .child         = gSysCtl2
   },
   {
      .ctl_name      = BCM_SYSCTL_DMA_3,
      .procname      = "3",
      .mode          = 0555,
      .child         = gSysCtl3
   },
   {}
};
static struct ctl_table gSysCtl[] = {
   {
      .ctl_name      = CTL_BCM_DMA,
      .procname      = DMA_DRIVER_NAME,
      .mode          = 0555,
      .child         = gSysCtlLocal
   },
   {}
};

/* ---- Functions -------------------------------------------------------- */
/*------------------------------------------------------------ */
/* Disable dma interrupts and clear interrupt bits for a channel. */
static inline void DMA_DISABLE_INT(int chan)
{
   REG_DMA_CHAN_CONFIG(chan) &= ~REG_DMA_CHAN_CTL_TC_INT_ENABLE;
   REG_DMA_INT_TC_CLEAR_CHAN(chan);
   REG_DMA_INT_ERROR_CLEAR_CHAN(chan);
}
static inline void DMA_ENABLE_INT(int chan)
{
   REG_DMA_CHAN_CONFIG(chan) |= REG_DMA_CHAN_CTL_TC_INT_ENABLE;
}
static inline void DMA_DISABLE_CORE(void)
{
   REG_DMA_CONFIG &= ~REG_DMA_CONFIG_ENABLED;   /* disable core */
}
static inline void DMA_ENABLE_CORE(void)
{
#ifndef __LITTLE_ENDIAN
	REG_DMA_CONFIG = REG_DMA_CONFIG_ENABLED | REG_DMA_CONFIG_BIG_ENDIAN; /* enable core and set big endian */
#else
	REG_DMA_CONFIG = (REG_DMA_CONFIG_ENABLED | REG_DMA_CONFIG_LITTLE_ENDIAN); /* enable core and set little endian */
#endif	
}
static inline int DMA_ACTIVE(int chan)
{
   return (REG_DMA_ENABLED_CHANNELS & (1 << chan));
}

/*------------------------------------------------------------ */
/*
 * Initialize the DMA channel. This is normally done by the 
 * dma isr but is also needed in polling mode. 
 */
void dma_init_chan(int chan)
{
   DMA_DEBUG(DBG_TRACE, ("dma_init_chan %d\n", chan));
   REG_DMA_CHAN_CONTROL(chan) = 0;
   REG_DMA_CHAN_CONFIG(chan) = 0;
   REG_DMA_INT_ERROR_CLEAR_CHAN(chan);
   REG_DMA_INT_TC_CLEAR_CHAN(chan);
}

/*------------------------------------------------------------ */
/* Configure the DMA channel. */
void dma_setup_chan(int chan, int srcaddr, int dstaddr, int link, int ctrl, int cfg)
{
   DMA_DEBUG(DBG_TRACE, ("dma_setup_chan channel %d srcaddr 0x%x dstaddr 0x%x link 0x%x ctrl 0x%x cfg 0x%x\n",
                         chan, srcaddr, dstaddr, link, ctrl, cfg));
   REG_DMA_CHAN_SRC_ADDR(chan) = srcaddr;
   REG_DMA_CHAN_DEST_ADDR(chan) = dstaddr;
   REG_DMA_CHAN_LINK(chan) = link;
   REG_DMA_CHAN_CONTROL(chan) = ctrl;
   REG_DMA_CHAN_CONFIG(chan) = cfg;
}

/*------------------------------------------------------------ */
/*
 * Wait till dma transaction complete. Busy wait if not using 
 * interrupt handler. Typically this is only for debug. 
 */
void dma_poll_chan(int chan)
{
   DMA_DEBUG(DBG_TRACE, ("dma_poll_chan enter 0x%x\n", REG_DMA_ENABLED_CHANNELS));
   while (DMA_ACTIVE(chan))
   {
      ;                         /* busy wait */
      DMA_DEBUG(DBG_TRACE, ("dma_poll_chan busy 0x%x\n", REG_DMA_ENABLED_CHANNELS));
   }
   REG_DMA_INT_TC_CLEAR_CHAN(chan);
   /*DMA_DISABLE_INT(chan); */
   DMA_DEBUG(DBG_TRACE, ("dma_poll_chan returns 0x%x\n", REG_DMA_ENABLED_CHANNELS));
}

/*------------------------------------------------------------ */
/* Initialize all channels to zero control/config, knock down ints. */
static void dma_init_all_chan(void)
{
   int chan;
   DMA_DEBUG(DBG_TRACE, ("dma_init_all_chan\n"));
   REG_DMA_ENABLED_CHANNELS = 0;
   for (chan = 0; chan < DMA_MAX_CHANNELS; chan++)
   {
      dma_init_chan(chan);
   }
}

/****************************************************************************
*
*  dma_cleanup
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/
static void dma_cleanup(void)
{
   DMA_DEBUG(DBG_TRACE, ("dma_cleanup()\n"));

   DMA_DISABLE_CORE();
   dma_init_all_chan();

   /* unregister sysctl table */
   if (gSysCtlHeader != NULL)
   {
      unregister_sysctl_table(gSysCtlHeader);
   }

   if (gIsr)
   {
      free_irq(IRQ_DMA, NULL);
      gIsr = 0;
   }
}

/****************************************************************************
*
*  dma_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/
static int __init dma_init(void)
{
   gLevel = DBG_DEFAULT_LEVEL;  /* Initialize debug level */

   DMA_DEBUG(DBG_INFO, ("dma_init called\n"));
   printk(banner);

   memset(&dma_state, 0, sizeof(dma_state));

   gSysCtlHeader = register_sysctl_table(gSysCtl);
   if (gSysCtlHeader == NULL)
   {
      goto fail;
   }

   DMA_ENABLE_CORE();
   dma_init_all_chan();

   return 0;

 fail:
   dma_cleanup();
   return -EINVAL;

}

/****************************************************************************
*
*  dma_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit dma_exit(void)
{
   DMA_DEBUG(DBG_INFO, ("dma_exit called\n"));

   dma_cleanup();
}

/****************************************************************************
*
*  dma_request_chan
*
*       Called to request exclusive access to a dma channel.
*
***************************************************************************/
int dma_request_chan(unsigned int chan, const char *name)
{
   DMA_DEBUG(DBG_TRACE, ("dma_request_chan %d %s\n", chan, name));
   if (chan >= DMA_MAX_CHANNELS)
      return -EINVAL;

   if (xchg(&dma_state[chan].lock, 1) != 0)
      return -EBUSY;

   strncpy((char *) dma_state[chan].name, name, sizeof(dma_state[chan].name));

   /* old flag was 0, now contains 1 to indicate busy */
   return 0;
}

/****************************************************************************
*
*  dma_request_avail_chan
*
*       Called to request exclusive access to any free dma channel.
*       Returns channel number in chanp if 0 return code.
*
***************************************************************************/
int dma_request_avail_chan(unsigned int *chanp, const char *name)
{
   int chan;
   unsigned long flags;

   DMA_DEBUG(DBG_TRACE, ("dma_request_avail_chan %s\n", name));
   /*
    * Allocate channels starting with highest number (higher number has lower
    * priority) so that devices that want to pick a particular channel with
    * dma_request_chan will still have highest priority channels available to choose from.
    */
   local_irq_save(flags);
   for (chan = DMA_MAX_CHANNELS - 1; chan >= 0; chan--)
   {
      if (!dma_state[chan].lock)
      {
         dma_state[chan].lock = 1;
         local_irq_restore(flags);
         strncpy((char *) dma_state[chan].name, name, sizeof(dma_state[chan].name));
         *chanp = chan;
         DMA_DEBUG(DBG_TRACE, ("dma_request_avail_chan for %s returns chan %d\n", name, chan));
         return 0;
      }
   }
   /* All channels busy. */
   local_irq_restore(flags);
   return -EBUSY;
}

/****************************************************************************
*
*  dma_free_chan
*
*       Called to free exclusive access to a dma channel.
*
***************************************************************************/
void dma_free_chan(unsigned int chan)
{
   DMA_DEBUG(DBG_TRACE, ("dma_free_chan %d\n", chan));
   if (chan >= DMA_MAX_CHANNELS)
   {
      DMA_DEBUG(DBG_INFO, ("Trying to free DMA channel %d\n", chan));
      return;
   }

   if (xchg(&dma_state[chan].lock, 0) == 0)
   {
      DMA_DEBUG(DBG_INFO, ("Trying to free already free DMA channel %d\n", chan));
      return;
   }
   /*strcpy((char *)dma_state[chan].name, "");  Leave last user name around for debug. */
}

/****************************************************************************
*
*  dma_request_irq
*
*       Called to register ISR on DMA chan, irq is not enabled until
*       dma_enable_irq is called.
*
***************************************************************************/
int dma_request_irq(int chan, irqreturn_t(*handler) (void *), void *dev_id)
{
   unsigned long flags;

   local_irq_save(flags);
   if (!gIsr)
   {
      /* Note: Calling request_irq also enables the IRQ. */
      int rc;
      
      rc = request_irq(IRQ_DMA, dma_isr, 
      IRQF_DISABLED,
      "dma", NULL);


      if (rc != 0)
      {
         local_irq_restore(flags);
         DMA_DEBUG(DBG_ERROR, ("DMA - Failed to register ISR.\n"));
         return rc;
      }
      gIsr = 1;
   }
   local_irq_restore(flags);

   if (dma_state[chan].handler != NULL)
   {
      DMA_DEBUG(DBG_INFO, ("DMA - ISR %d already registered, overriding ISR.\n", chan));
   }

   local_irq_save(flags);
   /*
    * Clear any pending interrupts and register interrupt handler.
    * Do not call dma_reset_chan() here since user likely will have
    * setup the control/config registers just before enabling interrupts.
    */
   DMA_DISABLE_INT(chan);
   dma_state[chan].handler = handler;   /* Register ISR */
   dma_state[chan].dev_id = dev_id;
   dma_state[chan].called = 0;
   local_irq_restore(flags);

   return 0;
}

/****************************************************************************
*
*  dma_free_irq
*
*       Called to free ISR on DMA chan
*
***************************************************************************/
int dma_free_irq(int chan)
{
   unsigned long flags;
   int handlers;
   int i;

   if (dma_state[chan].handler == NULL)
   {
      DMA_DEBUG(DBG_INFO, ("DMA - ISR %d not registered, cannot free.\n", chan));
      return -EINVAL;
   }

   local_irq_save(flags);

   /* Clear any pending interrupts */
   DMA_DISABLE_INT(chan);

   /* UnRegister ISR */
   dma_state[chan].handler = NULL;
   dma_state[chan].dev_id = NULL;

   handlers = 0;
   for (i = 0; i < DMA_MAX_CHANNELS; i++)
   {
      if (dma_state[i].handler != NULL)
      {
         handlers++;
      }
   }
   if (handlers == 0)
   {
      free_irq(IRQ_DMA, NULL);
      gIsr = 0;
   }

   local_irq_restore(flags);

   return 0;
}

/****************************************************************************
*
*  dma_enable_irq
*
*       Called to enable ISR on DMA chan
*
***************************************************************************/
int dma_enable_irq(int chan)
{
   unsigned long flags;
   if (dma_state[chan].handler == NULL)
   {
      DMA_DEBUG(DBG_INFO, ("DMA - ISR %d not registered, cannot enable.\n", chan));
      return -EINVAL;
   }
   local_irq_save(flags);
   DMA_ENABLE_INT(chan);
   local_irq_restore(flags);
   return 0;
}

/****************************************************************************
*
*  dma_disable_irq
*
*       Called to disable ISR on DMA chan
*
***************************************************************************/
int dma_disable_irq(int chan)
{
   unsigned long flags;
   if (dma_state[chan].handler == NULL)
   {
      DMA_DEBUG(DBG_INFO, ("DMA - ISR %d not registered, cannot disable.\n", chan));
      return -EINVAL;
   }
   local_irq_save(flags);
   DMA_DISABLE_INT(chan);
   local_irq_restore(flags);
   return 0;
}

/****************************************************************************
*
*  dma_isr
*
*       Global DMA ISR
*
***************************************************************************/
static irqreturn_t dma_isr(int irq, void *dev_id)
{
   int chan;

   (void) irq;
   (void) dev_id;

   REG_IRQ_ICR = REG_IRQ_ICR_DMACLR;    /* Clear the global interrupt source */

   DMA_DEBUG(DBG_TRACE2, ("DMA - dma_isr()\n"));

   gIrqs++;

   /* loop through 4 DMA chans, calling registered handlers for each active ISR */
   for (chan = 0; chan < DMA_MAX_CHANNELS; chan++)
   {
      if (REG_DMA_INT_STATUS_CHAN(chan))
      {
         if (REG_DMA_INT_ERROR_STATUS_CHAN(chan))
         {
            dma_state[chan].dmaError++;
            DMA_DEBUG(DBG_ERROR, ("Error on dma chan %d\n", chan));
         }
         /* clear the channel */
         dma_init_chan(chan);
         if (dma_state[chan].handler)
         {
            (*(dma_state[chan].handler)) (dma_state[chan].dev_id);      /* Call registered handler */
            dma_state[chan].called++;   /* Bump called counter */
         }
      }
   }
   return IRQ_HANDLED;
}

module_init(dma_init);
module_exit(dma_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("DMA Control Driver");
MODULE_LICENSE("GPL v2");

EXPORT_SYMBOL(dma_request_chan);
EXPORT_SYMBOL(dma_request_avail_chan);
EXPORT_SYMBOL(dma_free_chan);
EXPORT_SYMBOL(dma_request_irq);
EXPORT_SYMBOL(dma_free_irq);
EXPORT_SYMBOL(dma_enable_irq);
EXPORT_SYMBOL(dma_disable_irq);
EXPORT_SYMBOL(dma_init_chan);
EXPORT_SYMBOL(dma_setup_chan);
EXPORT_SYMBOL(dma_poll_chan);

