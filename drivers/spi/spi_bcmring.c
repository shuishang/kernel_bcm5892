/*
 * Broadcom SPI Host controller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/amba/bus.h>
#include <linux/proc_fs.h>
#include <linux/completion.h>
#include <linux/broadcom/knllog.h>
#include <linux/version.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <asm/uaccess.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <asm/delay.h>
#include <asm/dma.h>
#include <mach/dma.h>

#include <linux/broadcom/bcmring/gpio_defs.h>
#include <mach/csp/gpiomux.h>

#include <mach/csp/spiHw_reg.h>
#include <mach/csp/mm_io.h>
#include <csp/spiHw.h>
#include <mach/spih.h>


#define SPI_TEST

#define DEFAULT_DELTA       100
#define DEFAULT_BAUD_RATE   9375000
#define DEFAULT_FRAME_SIZE  16

/* max number of frames per DMA session */
#define MAX_DMA_FRAME       128
#define DMA_BUF_SIZE        (MAX_DMA_FRAME * 2)
#define MAX_PROC_BUF_SIZE   256

/* TODO: check for SPI DMA alignment */
#define IS_DMA_ALIGNED(x)   (((u32)(x) & 0x07) == 0)

#define START_STATE         ((void*)0)
#define RUNNING_STATE       ((void*)1)
#define DONE_STATE          ((void*)2)
#define ERROR_STATE         ((void*)-1)

#define QUEUE_RUNNING       0
#define QUEUE_STOPPED       1

#ifdef SPI_TEST
struct test_cfg {
	u32 rate; /* baud rate */
	atomic_t delta_time; /* time btw transfer in ms */
	u32 frame_size; /* frame size (bits/frame) */
	atomic_t enable_dma; /* flag to enable/disable dma */
	atomic_t start; /* flag to start the dummy test thread */

	long id; /* thread ID */
	struct completion exit_lock; /* exit sync */
};
#endif

struct dma_cfg {
	struct semaphore tx_lock;
	struct semaphore rx_lock;
	DMA_Handle_t tx_handle;
	DMA_Handle_t rx_handle;
};

struct driver_data {
	/* Driver model hookup */
	struct platform_device *pdev;

   /* SPI framework hookup */
	struct spi_master *master;

	/* DMA configuration */
	struct dma_cfg dma_cfg;

	/* dummy DMA buffer */
	void *dummy_dma_buf;
	dma_addr_t dummy_dma_addr;

   atomic_t loopback_enable;

	/* Driver message queue */
	struct workqueue_struct *workqueue;
	struct work_struct pump_messages;
	struct list_head queue;
	int busy;
	int run;

	/* Current message transfer state info */
	struct spi_message* cur_msg;
	struct spi_transfer* cur_transfer;
	struct chip_data *cur_chip;
	void *tx;
	void *rx;
	dma_addr_t rx_dma;
	dma_addr_t tx_dma;
	u32 n_bytes;

#ifdef SPI_TEST
	struct test_cfg test_cfg; 
#endif
};

struct chip_data {
	u32 bits_per_word;
	u32 speed_hz;
	u32 enable_dma;
	void (*cs_control)(u32 command);
};

struct proc_dirs {
	struct proc_dir_entry *parent_dir;
#ifdef SPI_TEST
	struct proc_dir_entry *test_dir;
#endif
};

/*
 * Function Prototypes
 */
static int null_cs_assert(u8 cs_num);
static int null_cs_deassert(u8 cs_num);
static inline void dma_tx_transfer_complete(struct dma_cfg *dma);
static inline void dma_rx_transfer_complete(struct dma_cfg *dma);

static struct semaphore spi_lock;
static struct clk *spi_clk;

SPIHW_TRANSFER_FNCS cs_ctrl = {&null_cs_assert, &null_cs_deassert};

static struct proc_dirs spi_proc;

static int null_cs_assert(u8 cs_num)
{
	return 0;
}

static int null_cs_deassert(u8 cs_num)
{
	return 0;
}

static int flush(struct driver_data *drv_data)
{
	return loops_per_jiffy << 1;
}

static void cs_control(u32 command)
{
}

static void irq_tx_dma(DMA_Device_t dev, int reason, void *data)
{
	struct dma_cfg *dma = (struct dma_cfg *)data;
	
	dma_tx_transfer_complete(dma);
}

static void irq_rx_dma(DMA_Device_t dev, int reason, void *data)
{
	struct dma_cfg *dma = (struct dma_cfg *)data;
	
	dma_rx_transfer_complete(dma);
}

static irqreturn_t irq_spi(int irq, void *data)
{
	/*struct driver_data *drv_data = (struct driver_data *)data; */
	u8 irq_status;

	irq_status = SpiHw_GetMaskIntrStatus();
	SpiHw_IntrClear(irq_status);

	if (irq_status & SPIHW_REG_INTR_RT) {
		u16 read_buf;

		/* error condition, clean the RX FIFO */
		while (SpiHw_IsRxNotEmpty()) {
			SpiHw_ReadFifo(&read_buf, 1);
			while (SpiHw_IsBusy());
		}
   }

	return IRQ_HANDLED;
}

static int dma_txrx_init(struct dma_cfg *dma)
{
	int rc;

	/* init locks as LOCKED */
	init_MUTEX_LOCKED(&dma->tx_lock);
	init_MUTEX_LOCKED(&dma->rx_lock);

	/* register IRQ handlers */
	rc = dma_set_device_handler(DMA_DEVICE_SPIH_MEM_TO_DEV, irq_tx_dma, dma);
	if (rc != 0) {
		printk(KERN_ERR "SPI: dma_txrx_init: TX dma_set_device_handler failed "
				"with status=%d\n", rc);
		return rc;
	}
	rc = dma_set_device_handler(DMA_DEVICE_SPIH_DEV_TO_MEM, irq_rx_dma, dma);
	if (rc != 0) {
		printk(KERN_ERR "SPI: dma_txrx_init: RX dma_set_device_handler failed "
				"with status=%d\n", rc);
		return rc;
	}

	return 0;
}

static inline void dma_txrx_transfer(struct dma_cfg *dma, dma_addr_t tx_dma,
		dma_addr_t rx_dma, u32 num_bytes, u32 frame_size_bytes)
{
	int rc;
	dma_addr_t spi_dma;

	spi_dma = (dma_addr_t)SpiHw_DmaFifoAddr();
	spi_dma = MM_IO_VIRT_TO_PHYS(spi_dma);

   if (frame_size_bytes == 1) {
      DMA_gDeviceAttribute[DMA_DEVICE_SPIH_MEM_TO_DEV].config.srcMaxTransactionWidth =
         dmacHw_SRC_TRANSACTION_WIDTH_8;
      DMA_gDeviceAttribute[DMA_DEVICE_SPIH_DEV_TO_MEM].config.dstMaxTransactionWidth =
         dmacHw_DST_TRANSACTION_WIDTH_8;
      DMA_gDeviceAttribute[DMA_DEVICE_SPIH_MEM_TO_DEV].config.dstMaxTransactionWidth =
         dmacHw_DST_TRANSACTION_WIDTH_8;
      DMA_gDeviceAttribute[DMA_DEVICE_SPIH_DEV_TO_MEM].config.srcMaxTransactionWidth =
         dmacHw_SRC_TRANSACTION_WIDTH_8;
   } else if (frame_size_bytes == 2) {
      DMA_gDeviceAttribute[DMA_DEVICE_SPIH_MEM_TO_DEV].config.srcMaxTransactionWidth =
         dmacHw_SRC_TRANSACTION_WIDTH_16;
      DMA_gDeviceAttribute[DMA_DEVICE_SPIH_DEV_TO_MEM].config.dstMaxTransactionWidth =
         dmacHw_DST_TRANSACTION_WIDTH_16;
      DMA_gDeviceAttribute[DMA_DEVICE_SPIH_MEM_TO_DEV].config.dstMaxTransactionWidth =
         dmacHw_DST_TRANSACTION_WIDTH_16;
      DMA_gDeviceAttribute[DMA_DEVICE_SPIH_DEV_TO_MEM].config.srcMaxTransactionWidth =
         dmacHw_SRC_TRANSACTION_WIDTH_16;
   } else {
      printk(KERN_ERR "SPI: Invalid frame size = %u bytes\n",
            frame_size_bytes);
      goto dma_end;
   }

	/* request DMA channels */
	dma->tx_handle = dma_alloc_channel(DMA_DEVICE_SPIH_MEM_TO_DEV);
	if (dma->tx_handle < 0) {
		printk(KERN_ERR "SPI: dma_txrx_transfer: TX dma_alloc_channel "
				"failed\n");

		goto dma_end;
	}
	dma->rx_handle = dma_alloc_channel(DMA_DEVICE_SPIH_DEV_TO_MEM);
	if (dma->rx_handle < 0) {
		printk(KERN_ERR "SPI: dma_txrx_transfer: RX dma_alloc_channel "
				"failed\n");
		goto dma_end;
	}

	/* RX DMA */
	rc = dma_transfer_from_device(dma->rx_handle,
			spi_dma, rx_dma, num_bytes);
	if (rc != 0) {
		printk(KERN_ERR "SPI: dma_txrx_transfer: RX DMA failed\n");
		goto dma_end;
	}

	/* TX DMA */
	rc = dma_transfer_to_device(dma->tx_handle,
			tx_dma, spi_dma, num_bytes);
	if (rc != 0) {
		printk(KERN_ERR "SPI: dma_txrx_transfer: TX DMA failed\n");
		goto dma_end;
	}

	/* wait for TX DMA to complete */
	down(&dma->tx_lock);

	/* wait for RX DMA to complete */
	down(&dma->rx_lock);

	rc = dma_free_channel(dma->tx_handle);
	if (rc != 0) {
		printk(KERN_ERR "SPI: dma_txrx_transfer: TX dma_free_channel "
				"failed\n");
		goto dma_end;
	}

	rc = dma_free_channel(dma->rx_handle);
	if (rc != 0) {
		printk(KERN_ERR "SPI: dma_txrx_transfer: RX dma_free_channel "
				"failed\n");
		goto dma_end;
	}

dma_end:
	return;
}

static inline void dma_tx_transfer_complete(struct dma_cfg *dma)
{
	up(&dma->tx_lock);
}

static inline void dma_rx_transfer_complete(struct dma_cfg *dma)
{
	up(&dma->rx_lock);
}

/* caller already set message->status; dma and pio irqs are blocked */
static void giveback(struct driver_data *drv_data)
{
	struct spi_transfer* last_transfer;
	struct spi_message *msg;
	
	msg = drv_data->cur_msg;
	drv_data->cur_msg = NULL;
	drv_data->cur_transfer = NULL;
	drv_data->cur_chip = NULL;
	queue_work(drv_data->workqueue, &drv_data->pump_messages);

	last_transfer = list_entry(msg->transfers.prev,
			struct spi_transfer,
			transfer_list);
	
	msg->state = NULL;
	if (msg->complete)
		msg->complete(msg->context);
}

static void *next_transfer(struct driver_data *drv_data)
{
	struct spi_message *msg = drv_data->cur_msg;
	struct spi_transfer *trans = drv_data->cur_transfer;
	
	/* Move to next transfer */
	if (trans->transfer_list.next != &msg->transfers) {
		drv_data->cur_transfer =
			list_entry(trans->transfer_list.next,
					struct spi_transfer,
					transfer_list);
		return RUNNING_STATE;
	}
	else
		return DONE_STATE;
}

static int map_dma_buffers(struct driver_data *drv_data, u32 num_bytes,
		int is_dma_mapped)
{
	struct spi_message *msg = drv_data->cur_msg;
	struct device *dev = &msg->spi->dev;

	if (!IS_DMA_ALIGNED(drv_data->rx) || !IS_DMA_ALIGNED(drv_data->tx)) {
		printk(KERN_ERR "SPI: map_dma_buffers failed with unaligned DMA "
				"memory\n");
		return -EFAULT;
	}

	/* use dummy DMA buffer if TX buffer is NULL */
	if (drv_data->tx == NULL) {
		drv_data->tx = drv_data->dummy_dma_buf;
		memset(drv_data->tx, 0, num_bytes);
		dma_sync_single_for_device(dev, drv_data->dummy_dma_addr, num_bytes,
				DMA_TO_DEVICE);
		drv_data->tx_dma = dma_map_single(dev, drv_data->tx,
				num_bytes, DMA_TO_DEVICE);
	} else if (!is_dma_mapped || drv_data->tx != drv_data->dummy_dma_buf) {
		/* map the TX buffer */
		drv_data->tx_dma = dma_map_single(dev, drv_data->tx,
				num_bytes, DMA_TO_DEVICE);

		if (dma_mapping_error(dev, drv_data->tx_dma)) {
			printk(KERN_ERR "SPI: dma_map_single failed tx_dma=0x%x\n",
					drv_data->tx_dma);
			return -EFAULT;
		}
	}

	/* Modify setup if RX buffer is null */
	if (drv_data->rx == NULL) {
		drv_data->rx = drv_data->dummy_dma_buf;
	} else if (!is_dma_mapped || drv_data->rx != drv_data->dummy_dma_buf) {
		/* map the RX buffer */
		drv_data->rx_dma = dma_map_single(dev, drv_data->rx, num_bytes, 
				DMA_FROM_DEVICE);

		if (dma_mapping_error(dev, drv_data->rx_dma)) {
			printk(KERN_ERR "SPI: dma_map_single failed rx_dma=0x%x\n",
					drv_data->rx_dma);
			return -EFAULT;
		}
	}

	return 0;
}

static void unmap_dma_buffers(struct driver_data *drv_data, u32 num_bytes,
		int is_dma_mapped)
{
	struct device *dev = &drv_data->cur_msg->spi->dev;

	if (is_dma_mapped)
		return;

	if (drv_data->tx != drv_data->dummy_dma_buf) {
		dma_unmap_single(dev, drv_data->tx_dma,
					num_bytes, DMA_TO_DEVICE);
	}

	if (drv_data->rx != drv_data->dummy_dma_buf) {
		dma_unmap_single(dev, drv_data->rx_dma,
					num_bytes, DMA_FROM_DEVICE);
	}
}

static struct spi_transfer *configure_transfer(struct driver_data *drv_data)
{
	struct chip_data *chip = drv_data->cur_chip;
	struct spi_transfer *transfer = drv_data->cur_transfer;
	int rc;

	u8 bits = 0;
	u32 speed = 0;

	bits = chip->bits_per_word;
	speed = chip->speed_hz;

	if (transfer->tx_buf == NULL && transfer->rx_buf == NULL) {
		printk(KERN_ERR "SPI: configure_transfer failed: Both TX and RX "
				"buffers point to NULL\n");
		return NULL;
	}

	drv_data->tx = (void *)transfer->tx_buf;
	drv_data->rx = transfer->rx_buf;

	drv_data->tx_dma = transfer->tx_dma;
	drv_data->rx_dma = transfer->rx_dma;

	if (transfer->bits_per_word)
		bits = transfer->bits_per_word;
	
	if (transfer->speed_hz)
		speed = transfer->speed_hz;

	/* validate and configure frame size */
	bits = transfer->bits_per_word;
	if (bits >= 4 && bits <= 8) {
		drv_data->n_bytes = 1;
	} else if (bits > 8 && bits <= 16) {
		drv_data->n_bytes = 2;
	} else {
		printk(KERN_ERR "SPI: configure_transfer failed: bits_per_word=%u\n",
				bits);
		return NULL;
	}
	rc = SpiHw_SetFrameSize(bits);
	if (rc != 0) {
		printk(KERN_ERR "SPI: configure_transfer failed: SpiHw_SetFrameSize "
				"failed\n");
		return NULL;
	}
		
	/* configure bit rate */
	rc = SpiHw_SetBitRate(speed);
	if (rc != 0) {
		printk(KERN_ERR "SPI: configure_transfer failed: SpiHw_SetBitRate failed\n");
		return NULL;;
	}

	return transfer;
}

static void pump_transfers(unsigned long data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;
	struct chip_data *chip = NULL;

	/* Get current state information */
	message = drv_data->cur_msg;
	transfer = drv_data->cur_transfer;
	chip = drv_data->cur_chip;

	/* Handle for abort */
	if (message->state == ERROR_STATE) {
		goto transfer_fail;
	}

	/* Handle end of message */
	if (message->state == DONE_STATE) {
		goto message_done;
	}

	/* Delay if requested at end of transfer*/
	if (message->state == RUNNING_STATE) {
		previous = list_entry(transfer->transfer_list.prev,
				struct spi_transfer,
				transfer_list);
		if (previous->delay_usecs)
			udelay(previous->delay_usecs);
	}

	/* Setup the transfer state based on the type of transfer */
	if (flush(drv_data) == 0) {
		goto transfer_fail;
	}

   if (atomic_read(&drv_data->loopback_enable))
      SpiHw_LoopBackModeEnable();

	message->state = RUNNING_STATE;
	chip->cs_control(BCMRING_CS_ASSERT);

	do {
		unsigned int tot_frames;

		/* set up buffers, frame size, and bit rate for this transfer */
		transfer = configure_transfer(drv_data);

		if (transfer == NULL) {
			goto transfer_fail;
		}

		tot_frames = transfer->len / drv_data->n_bytes;

		if (chip->enable_dma) {
			unsigned int remain_frames = tot_frames;
			unsigned int cur_frames = 0;

			SpiHw_EnableTxDMA();
			SpiHw_EnableRxDMA();

			while (remain_frames > 0) {
				if (remain_frames > MAX_DMA_FRAME) {
					cur_frames = MAX_DMA_FRAME;
					remain_frames -= MAX_DMA_FRAME;
				} else {
					cur_frames = remain_frames;
					remain_frames = 0;
				}
				
				if (map_dma_buffers(drv_data, cur_frames * drv_data->n_bytes, 
							message->is_dma_mapped) != 0) {
						goto transfer_fail;
				}

				/* start DMA transfer, block until finish */
				dma_txrx_transfer(&drv_data->dma_cfg, drv_data->tx_dma,
						drv_data->rx_dma, cur_frames * drv_data->n_bytes,
                  drv_data->n_bytes);

				unmap_dma_buffers(drv_data, cur_frames * drv_data->n_bytes,
						message->is_dma_mapped);

				if (drv_data->tx != drv_data->dummy_dma_buf) {
					drv_data->tx = (u8 *)drv_data->tx + cur_frames *
						drv_data->n_bytes;
				}
				
				if (drv_data->rx != drv_data->dummy_dma_buf) {
					drv_data->rx = (u8 *)drv_data->rx + cur_frames *
						drv_data->n_bytes;
				}
			}

			SpiHw_DisableTxDMA();
			SpiHw_DisableRxDMA();

		} else { /* NOT using DMA */

			if (transfer->tx_buf == NULL) {
				tot_frames = SpiHw_ReadPoll(drv_data->rx, tot_frames);
			} else if (transfer->rx_buf == NULL) {
				tot_frames = SpiHw_WritePoll(drv_data->tx, tot_frames);
			} else {
				tot_frames = SpiHw_WriteReadPoll(drv_data->tx,
						drv_data->rx, tot_frames);
			}

			if (tot_frames != transfer->len / drv_data->n_bytes) {
				printk(KERN_ERR "SPI: transfer error: Frames to be processed = %u "
						"Frames processed = %u\n",
						transfer->len / drv_data->n_bytes, tot_frames);
			}
		}

		message->actual_length += transfer->len; 

	} while ((drv_data->cur_msg->state = next_transfer(drv_data))
			!= DONE_STATE);

	chip->cs_control(BCMRING_CS_DEASSERT);
   SpiHw_LoopBackModeDisable();

	goto message_done;

transfer_fail:
	message->status = -EIO;
	giveback(drv_data);
	return;

message_done:
	message->status = 0;
	giveback(drv_data);
	/* go on */
	return;
}

static void pump_messages(struct work_struct *work)
{
	struct driver_data *drv_data =
		container_of(work, struct driver_data, pump_messages);

	/* Lock queue and check for queue work */
	if (list_empty(&drv_data->queue) || drv_data->run == QUEUE_STOPPED) {
		drv_data->busy = 0;
		return;
	}

	/* Make sure we are not already running a message */
	if (drv_data->cur_msg) {
		return;
	}

	down(&spi_lock);

	/* Extract head of queue */
	drv_data->cur_msg = list_entry(drv_data->queue.next,
			struct spi_message, queue);
	list_del_init(&drv_data->cur_msg->queue);

	/* Initial message state*/
	drv_data->cur_msg->state = START_STATE;
	drv_data->cur_transfer = list_entry(drv_data->cur_msg->transfers.next,
			struct spi_transfer,
			transfer_list);

	/* Setup the SSP using the per chip configuration */
	drv_data->cur_chip = spi_get_ctldata(drv_data->cur_msg->spi);

	/* Mark as busy and launch transfers */
	pump_transfers((unsigned long)drv_data);

	drv_data->busy = 1;

	up(&spi_lock);
}

static int transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct driver_data *drv_data = spi_master_get_devdata(spi->master);
	
	if (drv_data->run == QUEUE_STOPPED) {
		return -ESHUTDOWN;
	}

	msg->actual_length = 0;
	msg->status = -EINPROGRESS;
	msg->state = START_STATE;
	
	list_add_tail(&msg->queue, &drv_data->queue);
	
	if (drv_data->run == QUEUE_RUNNING && !drv_data->busy)
		queue_work(drv_data->workqueue, &drv_data->pump_messages);
	
	return 0;
}

#define MODEBITS (SPI_MODE_0)

static int setup(struct spi_device *spi)
{
	struct driver_data *drv_data = spi_master_get_devdata(spi->master);
	struct bcmring_spi_chip *chip_info = NULL;
	struct chip_data *chip;
	u32 bits;

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;
	
	if (spi->mode & ~MODEBITS) {
		printk(KERN_ERR "SPI: setup failed: unsupported mode bits %x\n",
				spi->mode & ~MODEBITS);
		return -EINVAL;
	}

	/* Only destroy_queue (or use chip_info) on first setup */
	chip = spi_get_ctldata(spi);
	if (!chip) {
		chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
		if (!chip) {
			printk(KERN_ERR "SPI: setup failed: can't allocate chip data\n");
			return -ENOMEM;
		}
		/* TODO: cs_control will not work, should default to something */
		chip->cs_control = cs_control;
		chip->enable_dma = 0;
	}

	/*
	 * Protocol drivers may change the chip settings, so if chip_info exists,
	 * use it
	 */
	chip_info = spi->controller_data;

	if (chip_info) {
		if (chip_info->cs_control)
			chip->cs_control = chip_info->cs_control;
		chip->enable_dma = chip_info->enable_dma;
	}

	chip->speed_hz = spi->max_speed_hz;

	/* validate and configure frame size */
	bits = spi->bits_per_word;
	if (bits >= 4 && bits <= 8) {
		drv_data->n_bytes = 1;
	} else if (bits > 8 && bits <= 16) {
		drv_data->n_bytes = 2;
	} else {
		printk(KERN_ERR "SPI: setup failed: invalid bits_per_word=%u\n",
				bits);
		return -EINVAL;
	}
	chip->bits_per_word = bits;

	spi_set_ctldata(spi, chip);

	if (SpiHw_SetClkPhase(SPIHW_CLKPHASE_FIRSTEDGE) != 0) {
		printk(KERN_ERR "SPI: setup: SpiHw_SetClkPhase failed\n");
		return -EFAULT;
	}

	if (SpiHw_SetClkPolarity( SPIHW_CLKPOLARITY_LOWIDLE) != 0) {
		printk(KERN_ERR "SPI: setup: SpiHw_SetClkPolarity failed\n");
		return -EFAULT;
	}

	printk(KERN_NOTICE "SPI: setup: bits_per_word=%u speed_hz=%u\n",
			chip->bits_per_word,	chip->speed_hz);

	return 0;
}

static void cleanup(struct spi_device *spi)
{
	struct chip_data *chip = spi_get_ctldata((struct spi_device *)spi);
	
	kfree(chip);
}

static int init_queue(struct driver_data *drv_data)
{
	INIT_LIST_HEAD(&drv_data->queue);

	drv_data->run = QUEUE_STOPPED;
	drv_data->busy = 0;

	INIT_WORK(&drv_data->pump_messages, pump_messages);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
	drv_data->workqueue = create_singlethread_workqueue(drv_data->master->dev.init_name);
#else
	drv_data->workqueue = create_singlethread_workqueue(drv_data->master->dev.bus_id);
#endif
	if (drv_data->workqueue == NULL)
		return -EBUSY;
	
	return 0;
}

static int start_queue(struct driver_data *drv_data)
{
	if (drv_data->run == QUEUE_RUNNING || drv_data->busy) {
		return -EBUSY;
	}

	drv_data->run = QUEUE_RUNNING;
	drv_data->cur_msg = NULL;
	drv_data->cur_transfer = NULL;
	drv_data->cur_chip = NULL;

	queue_work(drv_data->workqueue, &drv_data->pump_messages);

	return 0;
}

static int stop_queue(struct driver_data *drv_data)
{
	unsigned limit = 500;
	int status = 0;

	/* This is a bit lame, but is optimized for the common execution path.
	 * A wait_queue on the drv_data->busy could be used, but then the common
	 * execution path (pump_messages) would be required to call wake_up or
	 * friends on every SPI message. Do this instead */
	drv_data->run = QUEUE_STOPPED;
	while (!list_empty(&drv_data->queue) && drv_data->busy && limit--) {
		msleep(10);
	}

	if (!list_empty(&drv_data->queue) || drv_data->busy)
		status = -EBUSY;

	return status;
}

static int destroy_queue(struct driver_data *drv_data)
{
	int status;

	status = stop_queue(drv_data);
	if (status != 0)
		return status;

	destroy_workqueue(drv_data->workqueue);

	return 0;
}

#ifdef SPI_TEST
static int
proc_test_rate_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
	unsigned int len = 0;
	u32 rate;

	if (off > 0)
		return 0;

	rate = SpiHw_GetBitRate();
	
	len += sprintf(buffer + len, "Current baud rate is %u bps\n", rate);

	return len;
}
#endif

#ifdef SPI_TEST
static int
proc_test_rate_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	struct test_cfg *cfg = &drv_data->test_cfg;
	u32 rate;
	int rc;
	unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

	if (count > MAX_PROC_BUF_SIZE)
		count = MAX_PROC_BUF_SIZE;
	
	if (copy_from_user(kernel_buffer, buffer, count)) {
		printk(KERN_WARNING "SPI: copy_from_user failed\n");
		return -EFAULT;
	}
	
	if (sscanf(kernel_buffer, "%u", &rate) != 1) {
		printk(KERN_WARNING "SPI: proc write syntax error\n");
		return count;
	}

	rc = SpiHw_SetBitRate(rate);
	cfg->rate = SpiHw_GetBitRate();

	printk(KERN_WARNING "Baud rate set at: %u\n", cfg->rate);

	return count;
}
#endif

#ifdef SPI_TEST
static int
proc_test_delta_time_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	struct test_cfg *cfg = &drv_data->test_cfg;
	unsigned int len = 0;

	if (off > 0)
		return 0;
	
	len += sprintf(buffer + len, "Current delta time is %u ms\n",
			atomic_read(&cfg->delta_time));

	return len;
}
#endif

#ifdef SPI_TEST
static int
proc_test_delta_time_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	struct test_cfg *cfg = &drv_data->test_cfg;
	u32 delta_time;
	unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

	if (count > MAX_PROC_BUF_SIZE)
		count = MAX_PROC_BUF_SIZE;
	
	if (copy_from_user(kernel_buffer, buffer, count)) {
		printk(KERN_WARNING "SPI: copy_from_user failed\n");
		return -EFAULT;
	}
	
	if (sscanf(kernel_buffer, "%u", &delta_time) != 1) {
		printk(KERN_WARNING "SPI: proc write syntax error\n");
		return count;
	}

	atomic_set(&cfg->delta_time, delta_time);

	return count;
}
#endif

#ifdef SPI_TEST
static int
proc_test_frame_size_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
	unsigned int len = 0;
	u32 frame_size;

	if (off > 0)
		return 0;

	frame_size = SpiHw_GetFrameSize();
	
	len += sprintf(buffer + len, "Current frame size is %u bits/frame\n",
			frame_size);

	return len;
}
#endif

#ifdef SPI_TEST
static int
proc_test_frame_size_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	struct test_cfg *cfg = &drv_data->test_cfg;
	u32 frame_size;
	int rc;
	unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

	if (count > MAX_PROC_BUF_SIZE)
		count = MAX_PROC_BUF_SIZE;
	
	if (copy_from_user(kernel_buffer, buffer, count)) {
		printk(KERN_WARNING "SPI: copy_from_user failed\n");
		return -EFAULT;
	}
	
	if (sscanf(kernel_buffer, "%u", &frame_size) != 1) {
		printk(KERN_WARNING "SPI: proc write syntax error\n");
		return count;
	}

	if (frame_size > 16 || frame_size < 4) {
		printk(KERN_ERR "SPI: Inalid frame size: %u\n", frame_size);
		return count;
	}

	rc = SpiHw_SetFrameSize(frame_size);
	cfg->frame_size = SpiHw_GetFrameSize();
	printk(KERN_WARNING "Frame size set at: %u\n", cfg->frame_size);

	return count;
}
#endif

#ifdef SPI_TEST
static int
proc_test_enable_dma_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	struct test_cfg *cfg = &drv_data->test_cfg;
	unsigned int len = 0;

	if (off > 0)
		return 0;
	
	len += sprintf(buffer + len, "DMA is %s\n",
			atomic_read(&cfg->enable_dma) ? "enabled" : "disabled");

	return len;
}
#endif

#ifdef SPI_TEST
static int
proc_test_enable_dma_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	struct test_cfg *cfg = &drv_data->test_cfg;
	u32 enable_dma;
	unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

	if (count > MAX_PROC_BUF_SIZE)
		count = MAX_PROC_BUF_SIZE;
	
	if (copy_from_user(kernel_buffer, buffer, count)) {
		printk(KERN_WARNING "SPI: copy_from_user failed\n");
		return -EFAULT;
	}
	
	if (sscanf(kernel_buffer, "%u", &enable_dma) != 1) {
		printk(KERN_WARNING "SPI: proc write syntax error\n");
		return count;
	}

	if (enable_dma) {
		atomic_set(&cfg->enable_dma, 1);
	} else {
		atomic_set(&cfg->enable_dma, 0);
	}

	return count;
}
#endif

#ifdef SPI_TEST
static int test_thread_main(void *data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	struct test_cfg *cfg = &drv_data->test_cfg;
	u16 read_buf;
	void *tx_buf = NULL;
	void *rx_buf = NULL;
	dma_addr_t tx_dma, rx_dma;
	u32 i;
	u32 frame_size;
   u32 n_bytes;

	daemonize("SPI_THREAD");
	printk(KERN_INFO "SPI: Entering Thread...");
	
	/* ensure the RX FIFO is clear */
	while (SpiHw_IsRxNotEmpty()) {
		SpiHw_ReadFifo(&read_buf, 1);
		while (SpiHw_IsBusy());
	}

	SpiHw_LoopBackModeEnable();

	tx_buf = dma_alloc_coherent(NULL, DMA_BUF_SIZE,
					&tx_dma, GFP_KERNEL);
	if (!tx_buf) {
		printk(KERN_ERR "SPI: cannot allocate TX buffer\n");
		goto test_thread_exit;
	}

	rx_buf = dma_alloc_coherent(NULL, DMA_BUF_SIZE,
					&rx_dma, GFP_KERNEL);
	if (!rx_buf) {
		printk(KERN_ERR "SPI: cannot allocate RX buffer\n");
		goto test_thread_exit;
	}

	while (1) {
      unsigned char *tx8 = tx_buf;
      unsigned char *rx8 = rx_buf;
      u16 *tx16 = tx_buf;
      u16 *rx16 = rx_buf;
      
		if (!atomic_read(&cfg->start))
			break;

      frame_size = SpiHw_GetFrameSize();
      n_bytes = frame_size > 8 ? 2 : 1;

		for (i = 0; i < MAX_DMA_FRAME; i++) {
         if (frame_size <= 8) {
            tx8[i] = i;
            rx8[i] = 0;
         } else {
            tx16[i] = 0xCE00 + i;
            rx16[i] = 0;
         }
		}

		KNLLOG("tstart [Data transfer on SPI]\n");
		if (atomic_read(&cfg->enable_dma)) {
			SpiHw_EnableTxDMA();
			SpiHw_EnableRxDMA();

			/* start DMA transfer, block until finish */
			dma_txrx_transfer(&drv_data->dma_cfg, tx_dma, rx_dma,
               MAX_DMA_FRAME * n_bytes, n_bytes);

			SpiHw_DisableTxDMA();
			SpiHw_DisableRxDMA();
		} else {
			SpiHw_WriteReadPoll(tx_buf, rx_buf, MAX_DMA_FRAME);
		}
		KNLLOG("tstop [Data transfer on SPI]\n");

      for (i = 0; i < MAX_DMA_FRAME; i++) {
         if (frame_size <= 8) {
            if (rx8[i] != (tx8[i] & ((1 << frame_size) - 1))) {
               printk(KERN_ERR "SPI: tx_buf[%u] = 0x%x does not match "
							"rx_buf[%u] = 0x%x\n", i,
							tx8[i] & ((1 << frame_size) -1), i, rx8[i]);
            }
         } else {
            if (rx16[i] != (tx16[i] & ((1 << frame_size) - 1))) {
               printk(KERN_ERR "SPI: tx_buf[%u] = 0x%x does not match "
							"rx_buf[%u] = 0x%x\n", i,
							tx16[i] & ((1 << frame_size) -1), i, rx16[i]);
            }
         }
      }

		if (atomic_read(&cfg->delta_time)) {
			msleep(atomic_read(&cfg->delta_time));
		}
	}

test_thread_exit:
	printk(KERN_INFO "SPI: Exiting Thread...\n");
	SpiHw_LoopBackModeDisable();
	atomic_set(&cfg->start, 0);
	if (tx_buf != NULL) {
		dma_free_coherent(NULL, DMA_BUF_SIZE, tx_buf, tx_dma);
		tx_buf = NULL;
	}
	if (rx_buf != NULL) {
		dma_free_coherent(NULL, DMA_BUF_SIZE, rx_buf, rx_dma);
		rx_buf = NULL;
	}
	complete_and_exit(&cfg->exit_lock, 0);
	return 0;
}
#endif

#ifdef SPI_TEST
static int test_thread_init(struct driver_data *drv_data)
{
	struct test_cfg *cfg = &drv_data->test_cfg;

	init_completion(&cfg->exit_lock); /* incomplete */

	/* don't use DMA by default */
	atomic_set(&cfg->enable_dma, 0);

	/* default delta time */
	atomic_set(&cfg->delta_time, DEFAULT_DELTA);

	SpiHw_SetFrameSize(DEFAULT_FRAME_SIZE);
	SpiHw_SetBitRate(DEFAULT_BAUD_RATE);
	
	return 0;
}
#endif

#ifdef SPI_TEST
static void test_thread_start(struct driver_data *drv_data)
{
	struct test_cfg *cfg = &drv_data->test_cfg;

	/* already started */
	if (atomic_read(&cfg->start)) {
		return;
	}

	/* mark exit_lock as incomplete */
	INIT_COMPLETION(cfg->exit_lock);
	atomic_set(&cfg->start, 1);
	cfg->id = kernel_thread(test_thread_main, (void *)drv_data, 0);
}
#endif

#ifdef SPI_TEST
static void test_thread_stop(struct driver_data *drv_data)
{
	struct test_cfg *cfg = &drv_data->test_cfg;

	atomic_set(&cfg->start, 0);
	
	if (cfg->id > 0) {
		kill_proc_info(SIGTERM, SEND_SIG_PRIV, cfg->id);
		wait_for_completion(&cfg->exit_lock);
	}

	cfg->id = 0;
}
#endif

static int
proc_loopback_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	unsigned int len = 0;

	if (off > 0)
		return 0;
	
	len += sprintf(buffer + len, "SPI loopback mode is %s\n",
			atomic_read(&drv_data->loopback_enable) ? "ON" : "OFF");

	return len;
}

static int
proc_loopback_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	u32 enable;
	unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

	if (count > MAX_PROC_BUF_SIZE)
		count = MAX_PROC_BUF_SIZE;
	
	if (copy_from_user(kernel_buffer, buffer, count)) {
		printk(KERN_WARNING "SPI: copy_from_user failed\n");
		return -EFAULT;
	}
	
	if (sscanf(kernel_buffer, "%u", &enable) != 1) {
		printk(KERN_WARNING "SPI: proc write syntax error\n");
		return count;
	}

	if (enable)
		atomic_set(&drv_data->loopback_enable, 1);
	else
			atomic_set(&drv_data->loopback_enable, 0);
	
	return count;
}

#ifdef SPI_TEST
static int
proc_test_start_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	struct test_cfg *cfg = &drv_data->test_cfg;
	unsigned int len = 0;

	if (off > 0)
		return 0;
	
	len += sprintf(buffer + len, "Test SPI thread is %s\n",
			atomic_read(&cfg->start) ? "running" : "NOT running");

	return len;
}
#endif

#ifdef SPI_TEST
static int
proc_test_start_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	u32 start;
	unsigned char kernel_buffer[MAX_PROC_BUF_SIZE];

	if (count > MAX_PROC_BUF_SIZE)
		count = MAX_PROC_BUF_SIZE;
	
	if (copy_from_user(kernel_buffer, buffer, count)) {
		printk(KERN_WARNING "SPI: copy_from_user failed\n");
		return -EFAULT;
	}
	
	if (sscanf(kernel_buffer, "%u", &start) != 1) {
		printk(KERN_WARNING "SPI: proc write syntax error\n");
		return count;
	}

	if (start) {
		test_thread_start(drv_data);
	} else {
		test_thread_stop(drv_data);
	}

	return count;
}
#endif

/*
 * Set up proc entries
 */
static int proc_setup(struct driver_data *drv_data)
{
   struct proc_dir_entry *proc_loopback;
#ifdef SPI_TEST
	struct proc_dir_entry *proc_test_rate;
	struct proc_dir_entry *proc_test_delta_time;
	struct proc_dir_entry *proc_test_frame_size;
	struct proc_dir_entry *proc_test_enable_dma;
	struct proc_dir_entry *proc_test_start;
#endif

	spi_proc.parent_dir = proc_mkdir("spih", NULL);

   proc_loopback = create_proc_entry("loopbackEnable", 0644,
			spi_proc.parent_dir);
	if (proc_loopback == NULL) {
		return -ENOMEM;
	}
	proc_loopback->read_proc = proc_loopback_read;
	proc_loopback->write_proc = proc_loopback_write;
	proc_loopback->data = drv_data;


#ifdef SPI_TEST
	spi_proc.test_dir = proc_mkdir("perfTest", spi_proc.parent_dir);
#endif

#ifdef SPI_TEST
	proc_test_rate = create_proc_entry("rate", 0644,
			spi_proc.test_dir);
	if (proc_test_rate == NULL) {
		return -ENOMEM;
	}
	proc_test_rate->read_proc = proc_test_rate_read;
	proc_test_rate->write_proc = proc_test_rate_write;
	proc_test_rate->data = drv_data;

	proc_test_delta_time = create_proc_entry("deltaTime", 0644,
			spi_proc.test_dir);
	if (proc_test_delta_time == NULL) {
		return -ENOMEM;
	}
	proc_test_delta_time->read_proc = proc_test_delta_time_read;
	proc_test_delta_time->write_proc = proc_test_delta_time_write;
	proc_test_delta_time->data = drv_data;

	proc_test_frame_size = create_proc_entry("frameSize", 0644,
			spi_proc.test_dir);
	if (proc_test_frame_size == NULL) {
		return -ENOMEM;
	}
	proc_test_frame_size->read_proc = proc_test_frame_size_read;
	proc_test_frame_size->write_proc = proc_test_frame_size_write;
	proc_test_frame_size->data = drv_data;

	proc_test_enable_dma = create_proc_entry("enableDMA", 0644,
			spi_proc.test_dir);
	if (proc_test_enable_dma == NULL) {
		return -ENOMEM;
	}
	proc_test_enable_dma->read_proc = proc_test_enable_dma_read;
	proc_test_enable_dma->write_proc = proc_test_enable_dma_write;
	proc_test_enable_dma->data = drv_data;

	proc_test_start = create_proc_entry("start", 0644,
			spi_proc.test_dir);
	if (proc_test_start == NULL) {
		return -ENOMEM;
	}
	proc_test_start->read_proc = proc_test_start_read;
	proc_test_start->write_proc = proc_test_start_write;
	proc_test_start->data = drv_data;
#endif

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)
static int bcmring_spi_probe(struct amba_device *pdev, struct amba_id *id)
#else
static int bcmring_spi_probe(struct amba_device *pdev, void *id)
#endif
{
	struct device *dev = &pdev->dev;
	struct bcmring_spi_master *platform_info;
	struct spi_master *master;
	struct driver_data *drv_data = 0;
	int status = 0;

	platform_info = dev->platform_data;
	if (!platform_info) {
		printk(KERN_ERR "SPI: cannot obtain platform data\n");
		return -EINVAL;
	}

   /* Enable the clock via clock framework */
   spi_clk = clk_get(NULL, "SPI");
   if (IS_ERR(spi_clk)) {
      printk(KERN_ERR "SPI: Unable to find SPI clock\n");
      return -EFAULT;
   }

   clk_set_rate(spi_clk, 50000000);
   status = clk_enable(spi_clk);
   if (status < 0) {
      printk(KERN_ERR "SPI: Unable to enable SPI clock\n");
      goto error_put_clk;
   }

   /* initialize the SPI deivce */
	status = SpiHw_Init(&cs_ctrl);
	if (status != 0) {
		printk(KERN_ERR "SPI: Init device failed\n");
		goto error_disable_clk;
	}

	status = amba_request_regions(pdev, NULL);
	if (status) {
		printk(KERN_ERR "SPI: cannot to reserve regs region\n");
      goto error_spi_exit;
		return status;
	}

	/* Allocate master with space for drv_data */
	master = spi_alloc_master(dev, sizeof(struct driver_data) + 16);
	if (!master) {
		printk(KERN_ERR "SPI: cannot alloc spi_master\n");
		goto error_free_region;
	}

	drv_data = spi_master_get_devdata(master);
	drv_data->master = master;

	/* TODO: find a way to determine bus num */
	master->bus_num = 0;
	master->num_chipselect = platform_info->num_chipselect;
	master->setup = setup;
	master->transfer = transfer;
	master->cleanup = cleanup;

   /* turn off loopback mode by default */
   atomic_set(&drv_data->loopback_enable, 0);

	/* dummy DMA buffer is used for throw-away TX and RX data */
	drv_data->dummy_dma_buf = dma_alloc_coherent(dev, DMA_BUF_SIZE,
					&drv_data->dummy_dma_addr, GFP_KERNEL);
	if (!drv_data->dummy_dma_buf) {
		printk(KERN_ERR "SPI: cannot allocate dummy DMA buffer\n");
		goto error_free_region;
	}

	status = dma_txrx_init(&drv_data->dma_cfg);
	if (status != 0) {
		printk(KERN_ERR "SPI: problem initializing DMA\n");
		goto error_free_dummy_dma;
	}

	/* Initial and start queue */
	status = init_queue(drv_data);
	if (status != 0) {
		printk(KERN_ERR "SPI: problem initializing queue\n");
		goto error_free_dummy_dma;
	}

	status = start_queue(drv_data);
	if (status != 0) {
		printk(KERN_ERR "SPI: problem starting queue\n");
		goto error_destroy_queue;
	}

	/* Register with the SPI framework */
	amba_set_drvdata(pdev, drv_data);

	status = spi_register_master(master);
	if (status != 0) {
		printk(KERN_ERR "SPI: problem registering spi master\n");
		goto error_destroy_queue;
	}

	status = request_irq(IRQ_SPIH, irq_spi, IRQF_SHARED, "SPIH", drv_data);
	if (status < 0) {
		printk(KERN_ERR "SPI: problem requesting IRQ\n");
		goto error_destroy_queue;
	}
	/* disable all interrupts except the Receive Timeout */
	SpiHw_IntrDisable(SPIHW_REG_INTR_ROR | SPIHW_REG_INTR_RT |
			SPIHW_REG_INTR_RX | SPIHW_REG_INTR_TX);
	SpiHw_IntrEnable(SPIHW_REG_INTR_RT);

	status = SpiHw_Enable();
	if (status != 0) {
		printk(KERN_ERR "SPI: SpiHw_Enable failed\n");
		goto error_free_irq;
	}

	status = proc_setup(drv_data);
	if (status != 0) {
		printk(KERN_ERR "SPI: problem creating proc entries\n");
		goto error_free_irq;
	}

#ifdef SPI_TEST
	test_thread_init(drv_data);
#endif

	return status;

error_free_irq:
	free_irq(IRQ_SPIH, 0);

error_destroy_queue:
	destroy_queue(drv_data);

error_free_dummy_dma:
	dma_free_coherent(dev, DMA_BUF_SIZE, drv_data->dummy_dma_buf,
			drv_data->dummy_dma_addr);
	drv_data->dummy_dma_buf = NULL;

error_free_region:
	amba_release_regions(pdev);

error_spi_exit:
   SpiHw_Exit();

error_disable_clk:
   clk_disable(spi_clk);

error_put_clk:
   clk_put(spi_clk);
	return status;
}

static int bcmring_spi_remove(struct amba_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct driver_data *drv_data = amba_get_drvdata(pdev);
	int status = 0;

	if (!drv_data)
		return 0;

	status = SpiHw_Disable();
	if (status != 0) {
		printk(KERN_ERR "SPI: bcmring_spi_remove: SpiHw_Disable failed\n");
		return -EFAULT;
	}

	free_irq(IRQ_SPIH, 0);

	/* Remove the queue */
	status = destroy_queue(drv_data);
	if (status != 0) {
		printk(KERN_ERR "SPI: bcmring_spi_remove: workqueue will not "
			"complete, message memory not freed\n");
		return status;
	}

	remove_proc_entry("spih", NULL);

	dma_free_coherent(dev, DMA_BUF_SIZE, drv_data->dummy_dma_buf,
			drv_data->dummy_dma_addr);
	drv_data->dummy_dma_buf = NULL;

	/* disconnect from the SPI framework */
	spi_unregister_master(drv_data->master);

	/* prevent double remove */
	amba_set_drvdata(pdev, NULL);

	amba_release_regions(pdev);

	SpiHw_Exit();
   clk_disable(spi_clk);
   clk_put(spi_clk);

	return 0;
}

static struct amba_id bcmring_id_table[] = {
	{
		.id	= 0x00041022,
		.mask	= 0x000fffff,
	},
	{0, 0},
};

#ifdef CONFIG_PM
static int bcmring_spi_suspend(struct amba_device *dev, pm_message_t state)
{
   (void)dev;

   down(&spi_lock);
   clk_disable(spi_clk);
   return 0;
}

static int bcmring_spi_resume(struct amba_device *dev)
{
   int status = 0;

   (void)dev;

   status = clk_enable(spi_clk);
   if (status < 0) {
       printk(KERN_ERR "bcmring_spi_resume: Unable to enable SPI clock\n");
   }

   up(&spi_lock);
   return 0;
}
#else
#define bcmring_spi_suspend   NULL
#define bcmring_spi_resume    NULL
#endif

static struct amba_driver bcmring_driver = {
	.drv = {
		.name   = "bcmring-spi",
	},
	.probe      = bcmring_spi_probe,
	.remove     = bcmring_spi_remove,
	.id_table   = bcmring_id_table,
    .suspend    = bcmring_spi_suspend,
    .resume     = bcmring_spi_resume,
};

static int __init bcmring_spi_init(void)
{
   init_MUTEX(&spi_lock);

   /* set GPIOs to SPI mode */
   gpiomux_requestGroup(gpiomux_group_spi, "SPI");

	return amba_driver_register(&bcmring_driver);
}
module_init(bcmring_spi_init);

static void __exit bcmring_spi_exit(void)
{
	amba_driver_unregister(&bcmring_driver);

   /* free the SPI GPIO pins */
   gpiomux_freeGroup(gpiomux_group_spi);
}
module_exit(bcmring_spi_exit);

MODULE_AUTHOR("Broadcom Corporation <@broadcom.com>");
MODULE_DESCRIPTION("SPIH Controller");
MODULE_LICENSE("GPL");
