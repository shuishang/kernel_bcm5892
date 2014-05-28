/*****************************************************************************
*  Copyright 2001 - 2009 Broadcom Corporation.  All rights reserved.
*
*  Unless you and Broadcom execute a separate written software license
*  agreement governing use of this software, this software is licensed to you
*  under the terms of the GNU General Public License version 2, available at
*  http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
*  Notwithstanding the above, under no circumstances may you combine this
*  software in any way with any other Broadcom software provided under a
*  license other than the GPL, without Broadcom's express prior written
*  consent.
*
*****************************************************************************/

#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/broadcom/bcm_spu.h>

#include <asm/byteorder.h>
#include <linux/semaphore.h>

#include <csp/spuHw.h>
#include <mach/csp/spuHw_inline.h>
#include <mach/csp/cap.h>

#define DEFAULT_TX_DMA_DESCS 3

/* ---- Private Variables ------------------------------------ */
static struct semaphore spu_lock;
static atomic_t bus_is_probed;
static DMA_DescriptorRing_t tx_dma_ring;
static atomic_t deviceLock;

/* ---- Private Function Prototypes -------------------------- */
static void spu_dma_irq_tx(DMA_Device_t dev, int reason, void *data);
static void spu_dma_irq_rx(DMA_Device_t dev, int reason, void *data);

/* ---- Functions -------------------------------------------- */
/*****************************************************************************
* FUNCTION:   spu_dma_irq_tx
*
* PURPOSE:    IRQ handler for DMA in TX direction (Memory to SPU-M)
*
* PARAMETERS: dev - [IN] Dma device descriptor
*             reason - [IN] Unused
*             data - [IN] spu_dma_context struct
*
* RETURNS:    None
*****************************************************************************/
static void spu_dma_irq_tx(DMA_Device_t dev, int reason, void *data)
{
   spu_dma_context *spu_dma = (spu_dma_context *)data;
   (void)reason;   
   up(&spu_dma->dma_tx_lock);
}

/*****************************************************************************
* FUNCTION:   spu_dma_irq_rx
*
* PURPOSE:    IRQ handler for DMA in RX direction (SPU-M to Memory)
*
* PARAMETERS: dev - [IN] Dma device descriptor
*             reason - [IN] Unused
*             data - [IN] spu_dma_context struct
*
* RETURNS:    None
*****************************************************************************/
static void spu_dma_irq_rx(DMA_Device_t dev, int reason, void *data)
{
   spu_dma_context *spu_dma = (spu_dma_context *)data;
   (void)reason;
   up(&spu_dma->dma_rx_lock);
}

/*****************************************************************************
* FUNCTION:   spu_request
*
* PURPOSE:    Request for access to SPU-M.  Will initialize SPU-M if not done
*
* PARAMETERS: force_init - [IN] Force the SPU-M to initialize or reset
*
* RETURNS:    None
*****************************************************************************/
void spu_request(void)
{
   down(&spu_lock);
   atomic_set(&deviceLock, 1);
}

/*****************************************************************************
* FUNCTION:   spu_release
*
* PURPOSE:    Release access to SPU-M.
* 
* PARAMETERS: None
*
* RETURNS:    None
*****************************************************************************/
void spu_release( void )
{
   atomic_set(&deviceLock, 0);
   up(&spu_lock);
}

/*****************************************************************************
* FUNCTION:   spu_dma_context_init
*
* PURPOSE:    Initialize settings in the spu_dma_context struct
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*
* RETURNS:    Returns 0 if successful, negative value if failure
*****************************************************************************/
int spu_dma_context_init( spu_dma_context *spu_dma )
{
   int rc = 0;

   if (!atomic_read(&bus_is_probed))
      return -ENODEV;

   init_MUTEX_LOCKED( &spu_dma->dma_tx_lock);
   init_MUTEX_LOCKED( &spu_dma->dma_rx_lock);

   spu_dma->dma_tx_cfg.device = DMA_DEVICE_SPUM_MEM_TO_DEV;
   spu_dma->dma_tx_cfg.fifo_addr = MM_IO_VIRT_TO_PHYS(spuHw_getInputFifoAddress());
   
   spu_dma->dma_rx_cfg.device = DMA_DEVICE_SPUM_DEV_TO_MEM;
   spu_dma->dma_rx_cfg.fifo_addr = MM_IO_VIRT_TO_PHYS(spuHw_getOutputFifoAddress());

   return rc;   
}

/*****************************************************************************
* FUNCTION:   spu_dma_set_device_handlers
*
* PURPOSE:    Associates the DMA device handler to the spu_dma_context
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*
* RETURNS:    Returns 0 if successful, negative value if failure
*****************************************************************************/
int spu_dma_set_device_handlers( spu_dma_context *spu_dma )
{
   int rc;

   if (!atomic_read(&bus_is_probed))
      return -ENODEV;

   rc = dma_set_device_handler( spu_dma->dma_tx_cfg.device, spu_dma_irq_tx, spu_dma );
   if (rc != 0) 
   {
      printk("spu_dma_context_init: TX dma_set_device_handler failed\n");
      return (rc);
   }
   
   rc = dma_set_device_handler(spu_dma->dma_rx_cfg.device, spu_dma_irq_rx, spu_dma);
   if (rc != 0) 
   {
      printk("spu_dma_context_init: RX dma_set_device_handler failed\n");
      return (rc);
   }

   return 0;   
}

/*****************************************************************************
* FUNCTION:   spu_dma_alloc
*
* PURPOSE:    Allocate memory for DMA buffers
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*
* RETURNS:    Returns 0 if successful, negative value if failure
*****************************************************************************/
int spu_dma_alloc( spu_dma_context *spu_dma )
{
   int rc;

   if (!atomic_read(&bus_is_probed))
      return -ENODEV;

   spu_dma->crypto_cmd.virt = dma_alloc_writecombine(  NULL, SPU_DMA_CMD_BUFFER_LENGTH, &spu_dma->crypto_cmd.phys, GFP_KERNEL);
   if ( !spu_dma->crypto_cmd.virt )
   {
      printk("spu_dma_alloc: Unable to allocate crypto_cmd buffer\n");
      return (-ENOMEM);
   }
   
   spu_dma->crypto_in.virt = dma_alloc_writecombine( NULL, SPU_DMA_DATA_BUFFER_LENGTH, &spu_dma->crypto_in.phys, GFP_KERNEL);
   if ( !spu_dma->crypto_in.virt )
   {
      printk("spu_dma_alloc: Unable to allocate crypto_in buffer\n");
      rc = -ENOMEM;
      goto free_cmd_buf;
   }
   
   spu_dma->crypto_out.virt = dma_alloc_writecombine(  NULL, SPU_DMA_DATA_BUFFER_LENGTH, &spu_dma->crypto_out.phys, GFP_KERNEL);
   if ( !spu_dma->crypto_out.virt )
   {
      printk("spu_dma_alloc: Unable to allocate crypto_out buffer\n");
      rc = -ENOMEM;
      goto free_in_buf;
   }

   return 0;

free_in_buf:
   dma_free_writecombine( NULL, SPU_DMA_DATA_BUFFER_LENGTH,
         spu_dma->crypto_in.virt, spu_dma->crypto_in.phys );

free_cmd_buf:
   dma_free_writecombine( NULL, SPU_DMA_CMD_BUFFER_LENGTH,
         spu_dma->crypto_cmd.virt, spu_dma->crypto_cmd.phys );

   return rc;   
}

/*****************************************************************************
* FUNCTION:   spu_dma_dealloc
*
* PURPOSE:    Free DMA buffer memory
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*
* RETURNS:    Returns 0 if successful, negative value if failure
*****************************************************************************/
void spu_dma_dealloc( spu_dma_context *spu_dma )
{
   if (spu_dma->crypto_out.virt != NULL )
   {
      dma_free_writecombine( NULL, SPU_DMA_DATA_BUFFER_LENGTH, spu_dma->crypto_out.virt, spu_dma->crypto_out.phys);
   }
   spu_dma->crypto_out.phys = 0;

   if (spu_dma->crypto_in.virt != NULL )
   {
      dma_free_writecombine( NULL, SPU_DMA_DATA_BUFFER_LENGTH, spu_dma->crypto_in.virt, spu_dma->crypto_in.phys);
   }
   spu_dma->crypto_in.phys = 0;   
   
   if (spu_dma->crypto_cmd.virt != NULL )
   {
      dma_free_writecombine( NULL, SPU_DMA_CMD_BUFFER_LENGTH, spu_dma->crypto_cmd.virt, spu_dma->crypto_cmd.phys);
   }
   spu_dma->crypto_cmd.phys = 0;   
}

/*****************************************************************************
* FUNCTION:   spu_dma_reserve
*
* PURPOSE:    Reserve DMA channels for use
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*
* RETURNS:    None
*****************************************************************************/
int spu_dma_reserve(spu_dma_context *spu_dma)
{
   int rc;

   if (!atomic_read(&bus_is_probed))
      return -ENODEV;

   /* reserve TX/RX DMA channels */
   spu_dma->dma_tx_cfg.handle = dma_alloc_channel(spu_dma->dma_tx_cfg.device);
   if (spu_dma->dma_tx_cfg.handle < 0)
   {
      printk("spu_dma_reserve: TX dma_alloc_channel failed\n");
      rc = spu_dma->dma_tx_cfg.handle;
      return rc;
   }
   
   spu_dma->dma_rx_cfg.handle = dma_alloc_channel(spu_dma->dma_rx_cfg.device);
   if (spu_dma->dma_rx_cfg.handle < 0)
   {
      printk("spu_dma_reserve: RX dma_alloc_channel failed\n");
      rc = spu_dma->dma_rx_cfg.handle;
      goto free_tx_ch;
   }

   return 0;

free_tx_ch:
   dma_free_channel(spu_dma->dma_tx_cfg.handle);
   return rc;
}

/*****************************************************************************
* FUNCTION:   spu_dma_free
*
* PURPOSE:    Free acquired DMA channels
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*
* RETURNS:    None
*****************************************************************************/
void spu_dma_free(spu_dma_context *spu_dma)
{
   int rc;
   
   rc = dma_free_channel(spu_dma->dma_tx_cfg.handle);
   if (rc != 0)
   {
      printk("spu_dma_free: TX dma_free_channel failed\n");
   }
   
   rc = dma_free_channel(spu_dma->dma_rx_cfg.handle);
   if (rc != 0)
   {
      printk("spu_dma_free: RX dma_free_channel failed\n");
   }   
}

/*****************************************************************************
* FUNCTION:   spu_dma_config
*
* PURPOSE:    Configure DMA to setup for transfer
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*             cmd_len - [IN] length of command in bytes
*             in_len  - [IN] length of input data in bytes
*             out_len - [IN] length of expected output data in bytes
*
* RETURNS:    None
*****************************************************************************/
int spu_dma_config(spu_dma_context *spu_dma, uint32_t cmd_len, uint32_t in_len, uint32_t out_len)
{
   int rc;
   unsigned int desc_cnt; /* descriptor count */
   unsigned int desc_ring_size;

   if (!atomic_read(&bus_is_probed))
      return -ENODEV;

   if (!atomic_read(&deviceLock))
   {
      printk("spu_dma_config: deviceLock is NOT locked\n");
      return -EFAULT;
   }

   /*
    * Set up TX dma so it can transfer both the SPU command and data in
    * one shot
    */

   /* number of desc needed for SPU command */
   desc_cnt = dma_calculate_descriptor_count(spu_dma->dma_tx_cfg.device,
            spu_dma->crypto_cmd.phys, spu_dma->dma_tx_cfg.fifo_addr, cmd_len);
   if (desc_cnt < 0)
   {
      printk("spu_dma_config: dma_calculate_descriptor_count for command failed\n");
      rc = desc_cnt;
      return rc;
   }

   /* 32 bit align and add extra 4 bytes for status */
   in_len = (((in_len + 3) / sizeof(uint32_t)) * sizeof(uint32_t)) + 4;

   /* number of desc needed for SPU data */
   rc = dma_calculate_descriptor_count(spu_dma->dma_tx_cfg.device,
            spu_dma->crypto_in.phys, spu_dma->dma_tx_cfg.fifo_addr,
            in_len);
   if (rc < 0)
   {
      printk("spu_dma_config: dma_calculate_descriptor_count for data failed\n");
      return rc;
   }

   /* add desc counts (cmd + data) */
   desc_cnt += rc;

   /* calculate the descriptor size */
   desc_ring_size = dmacHw_descriptorLen(desc_cnt);

   if (desc_ring_size > tx_dma_ring.bytesAllocated)
   {
      rc = dma_alloc_descriptor_ring(&tx_dma_ring, desc_cnt);
      if (rc < 0)
      {
         printk("spu_dma_config: dma_alloc_descriptor_ring failed\n");
         return rc;
      }

      rc = dma_set_device_descriptor_ring(spu_dma->dma_tx_cfg.device,
         &tx_dma_ring);
      if (rc < 0)
      {
         printk("spu_dma_config: dma_set_device_descriptor_ring failed\n");
         return rc;
      }
   }
   else
   {
      dma_init_descriptor_ring(&tx_dma_ring, desc_cnt);
   }

   /* add desc for SPU command */
   rc = dma_add_descriptors(&tx_dma_ring,
         spu_dma->dma_tx_cfg.device,
         spu_dma->crypto_cmd.phys,
         spu_dma->dma_tx_cfg.fifo_addr,
         cmd_len);
   
   if (rc != 0) 
   {
      printk("spu_dma_config: dma_add_descriptors failed\n");
   }
   
   /* add desc for SPU data */
   rc = dma_add_descriptors(&tx_dma_ring,
         spu_dma->dma_tx_cfg.device,
         spu_dma->crypto_in.phys,
         spu_dma->dma_tx_cfg.fifo_addr,
         in_len);
   if (rc < 0) 
   {
      printk("spu_dma_config: dma_add_descriptors failed\n");
      return rc;
   }

   /* tell the DMA ready for transfer */
   rc = dma_start_transfer(spu_dma->dma_tx_cfg.handle, cmd_len + in_len);
   if (rc < 0)
   {
      printk("spu_dma_config: dma_start_transfer failed\n");
      return rc;
   }
   
   rc = dma_transfer_from_device(spu_dma->dma_rx_cfg.handle,
         spu_dma->dma_rx_cfg.fifo_addr, spu_dma->crypto_out.phys, out_len);
   if (rc < 0)
   {
      printk("spu_dma_config: dma_transfer_from_device failed\n");
      dma_stop_transfer(spu_dma->dma_tx_cfg.handle);
      return rc;
   }

   return 0;
}

/*****************************************************************************
* FUNCTION:   spu_dma_quick
*
* PURPOSE:    Perform immediate DMA operations if SPU command/input/output is less than 4KB
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*             cmd_len - [IN] length of command in bytes
*             in_len  - [IN] length of input data in bytes
*             out_len - [IN] length of expected output data in bytes
*
* RETURNS:    0 : on success
*****************************************************************************/
int spu_dma_quick(spu_dma_context *spu_dma, uint32_t cmd_len, uint32_t in_len, uint32_t out_len)
{

   if (!atomic_read(&bus_is_probed))
   {
      return -ENODEV;
   }
   /* Process comamnd */
   if (dma_quick_transfer (spu_dma->dma_tx_cfg.handle, spu_dma->crypto_cmd.phys, spu_dma->dma_tx_cfg.fifo_addr, cmd_len) == -1)
   {
      return -4;
   }
   /* Process output */
   if (dma_quick_transfer (spu_dma->dma_rx_cfg.handle, spu_dma->dma_rx_cfg.fifo_addr, spu_dma->crypto_out.phys, out_len) == -1)
   {
      return -3;
   }
   /* ignite the SPU FIFO for DMA */
   spuHw_initiateDma();
   /* Process input */
   in_len = ( ((in_len + 3) / sizeof(uint32_t)) * sizeof(uint32_t)) + sizeof(uint32_t);
   if (dma_quick_transfer (spu_dma->dma_tx_cfg.handle, spu_dma->crypto_in.phys, spu_dma->dma_tx_cfg.fifo_addr, in_len) == -1)
   {
      return -4;
   }
   return 0;
}

/*****************************************************************************
* FUNCTION:   spu_dma_wait
*
* PURPOSE:    Wait for DMA transfer to complete
*
* PARAMETERS: spu_dma - [IN] spu_dma_context struct
*
* RETURNS:    None
*****************************************************************************/
void spu_dma_wait(spu_dma_context *spu_dma)
{
   down(&spu_dma->dma_tx_lock);
   down(&spu_dma->dma_rx_lock);   
}

EXPORT_SYMBOL(spu_request);
EXPORT_SYMBOL(spu_release);
EXPORT_SYMBOL(spu_dma_alloc);
EXPORT_SYMBOL(spu_dma_dealloc);
EXPORT_SYMBOL(spu_dma_context_init);
EXPORT_SYMBOL(spu_dma_set_device_handlers);
EXPORT_SYMBOL(spu_dma_reserve);
EXPORT_SYMBOL(spu_dma_free);
EXPORT_SYMBOL(spu_dma_config);
EXPORT_SYMBOL(spu_dma_quick);
EXPORT_SYMBOL(spu_dma_wait);

//extern int consistent_proc_init(void);

static int spu_probe(struct platform_device *pdev)
{
   int rc;

	init_MUTEX(&spu_lock);
   atomic_set(&bus_is_probed, 0);

   if (cap_isPresent(CAP_SPU, 0) != CAP_PRESENT) {
      printk(KERN_WARNING "SPU: Not supported\n");
      return -ENODEV;
   }

	/* Initialize SPU-M block */
   spuHw_initDevice();

#ifdef __LITTLE_ENDIAN
	/* Configure the device for little endian input and output */
   spuHw_configDevice ( spuHw_DEV_CONFIG_OPEN |
            spuHw_DEV_CONFIG_INPUT_LITTLE |
            spuHw_DEV_CONFIG_OUTPUT_LITTLE );
#else
	/* Configure the device for big endian input and output */
   spuHw_configDevice ( spuHw_DEV_CONFIG_OPEN );
#endif

   rc = dma_alloc_descriptor_ring(&tx_dma_ring, DEFAULT_TX_DMA_DESCS);
   if (rc < 0) {
      printk(KERN_ERR "SPU: dma_alloc_descriptor_ring failed\n");
      return rc;
   }

   rc = dma_set_device_descriptor_ring(DMA_DEVICE_SPUM_MEM_TO_DEV, &tx_dma_ring);
   if (rc < 0) {
      dma_free_descriptor_ring(&tx_dma_ring);
      printk(KERN_ERR "SPU: dma_set_device_descriptor_ring failed\n");
      return rc;
   }

   //consistent_proc_init();

   atomic_set(&deviceLock, 0);
   atomic_set(&bus_is_probed, 1);

   printk(KERN_INFO "SPU: driver probed. tx_ring virt=0x08%x phys=0x08%x\n",
         tx_dma_ring.virtAddr, tx_dma_ring.physAddr);

   return 0;
}

static int spu_remove(struct platform_device *pdev)
{
   atomic_set(&bus_is_probed, 0);
   spu_request();
   dma_free_descriptor_ring(&tx_dma_ring);
   spu_release();
   return 0;
}

#ifdef CONFIG_PM
static int spu_suspend(struct platform_device *pdev, pm_message_t state)
{
   down(&spu_lock);
   return 0;
}

static int spu_resume(struct platform_device *pdev)
{
   up(&spu_lock);
   return 0;
}
#else
#define spu_suspend   NULL
#define spu_resume    NULL
#endif

static struct platform_driver spu_driver = {
   .driver = {
      .name = "bcmring-spu",
      .owner = THIS_MODULE,
   },
   .probe = spu_probe,
   .remove = spu_remove,
   .suspend = spu_suspend,
   .resume = spu_resume,
};

static int __init spu_init(void)
{   
   return platform_driver_register(&spu_driver);
}

static void __exit spu_exit(void)
{
   platform_driver_unregister(&spu_driver);
}

#ifdef CONFIG_CRYPTO_DEV_BCM_SPU
/*
 * When compiled into the kernel, we need to force the SPU driver to be
 * initialized before device_initcall, where users of the SPU driver are being
 * initialized
 */
subsys_initcall(spu_init);
#else /* while compiled as a module */
module_init(spu_init);
module_exit(spu_exit);
#endif

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom BCMRING SPU Driver");
MODULE_LICENSE("GPL");
