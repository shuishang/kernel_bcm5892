/*****************************************************************************
* Copyright 2008 - 2011 Broadcom Corporation.  All rights reserved.
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


#include <asm/bug.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <mach/platform.h>
#include <linux/version.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <asm/cache.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <linux/ethtool.h>
#include <asm/processor.h>	/* Processor type for cache alignment. */
#include <linux/netdevice.h>
#include <mach/shm.h>
#include <mach/hardware.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>	/* for seq_file */
#include <linux/vmalloc.h>
#include <linux/socket.h>
#include <linux/syscalls.h>

#define BCM5892_MCAST_SUPPORT

#define MIN_DEBUG_LEVEL 0
#define MAX_DEBUG_LEVEL 3
static int cur_dbg_lvl = MIN_DEBUG_LEVEL;

#undef BCM5892_DEBUG
#undef BCM5892_DUMP_STAT_REGS 

#undef PDEBUG
#ifdef BCM5892_DEBUG
#define PDEBUG(lvl,fmt, args...) do { \
		if (lvl == cur_dbg_lvl) \
			printk( KERN_INFO "BCM5892_ETH: " fmt, ## args); \
	} while (0)
#else
#define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#define USE_MII_BUS_POINTER
#endif
#include "bcm589X_mac.h"	/* Our Macros, and Structure Definitions */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)
#	include <linux/platform_device.h>
#endif

#define DRV_VERSION     "1.02"

#define __raw_writeq(val, addr) do{\
					__raw_writel((u32)(0xFFFFFFFF & (val)), addr);\
					(val) >>= 32;\
					__raw_writel((u32)(0xFFFF & (val)), ((u32)addr)+4); \
				}while(0);

#define BYTE_SWAP(x) \
        ((uint32_t)( \
                (((uint32_t)(x) & (uint32_t)0x000000ff) << 24) | \
                (((uint32_t)(x) & (uint32_t)0x0000ff00) <<  8) | \
                (((uint32_t)(x) & (uint32_t)0x00ff0000) >>  8) | \
                (((uint32_t)(x) & (uint32_t)0xff000000) >> 24) ))

#define 	WAIT_FOR_PHY_READY_DELAY	1

/**********************************************************************
 *  The static data member declarations _Begin
 **********************************************************************/

/* Used for debugging */
struct {
	int intr;
	int intr_count;
	int intr_current;
	int intr_raw;
	int rx_overflow;
	int rx_poll;
	int rx_in_isr;
} trace;

static char bcm5892mac_string[] = "bcm5892-mac";
static char bcm5892mac_mdio_string[] = "bcm5892-mac-mdio";

static struct resource bcm5892mac_resources[] = {
	[0] = {
		.flags  = IORESOURCE_IRQ,
		.start  = IRQ_OEMAC,
	},
	[1] = {
		.flags  = IORESOURCE_MEM,
		.start  = START_ETH_CFG,
		.end    = END_ETH_CFG,
	},
	[2] = {
		.flags  = IORESOURCE_MEM,
		.start  = START_MMI_CFG,
		.end    = END_MMI_CFG,
	},
};

static struct proc_dir_entry *bcm5892_eth_root_dir ; // BCM5892  eth proc root directory


static int	bcm5892mac_start_tx(struct sk_buff*, struct net_device*);
static int	bcm5892mac_close(struct net_device*);
static int	bcm5892mac_open(struct net_device*);
static void	bcm5892mac_tx_timeout(struct net_device*);
static struct net_device_stats *bcm5892_get_stats(struct net_device *);
#ifdef BCM5892_MCAST_SUPPORT
static void	bcm5892mac_setmulti(struct bcm5892mac_softc*);
static void	bcm5892mac_set_rx_mode(struct net_device*);
#endif
static int  bcm5892mac_set_addr(struct net_device *dev, void *addr);
static int	bcm5892_mii_ioctl(struct net_device*, struct ifreq*, int);
static int	bcm5892mac_change_mtu(struct net_device*, int);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
static const struct net_device_ops bcm5892_netdev_ops = {
    .ndo_open               = bcm5892mac_open,
	.ndo_stop               = bcm5892mac_close,
	.ndo_get_stats          = bcm5892_get_stats,
	.ndo_start_xmit         = bcm5892mac_start_tx,
#if 0
	.ndo_set_multicast_list = bcm5892mac_setmulti,
#endif
	.ndo_set_mac_address    = bcm5892mac_set_addr,
	.ndo_tx_timeout         = bcm5892mac_tx_timeout,
	.ndo_do_ioctl           = bcm5892_mii_ioctl,
	.ndo_change_mtu			= bcm5892mac_change_mtu,
	.ndo_set_rx_mode		= bcm5892mac_set_rx_mode,
};
#endif

								
/**********************************************************************
 *  The static data member declarations _End
 **********************************************************************/


/**********************************************************************
 *  The function declarations _Begin
 **********************************************************************/
static int	bcm5892dma_initctx(struct bcm5892macdma*, struct bcm5892mac_softc*, int, int);
static void	bcm5892dma_uninitctx(struct bcm5892macdma*);
static void	bcm5892dma_start(struct bcm5892macdma*, int);
static void	bcm5892dma_channel_stop(struct bcm5892macdma*);
static void	bcm5892dma_align_skb(struct sk_buff*, int, int);
static int	bcm5892dma_add_rcvbuffer(struct bcm5892macdma*, struct sk_buff*);
static int	bcm5892dma_add_txbuffer(struct bcm5892macdma*, struct sk_buff*);
static void	bcm5892dma_emptyring(struct bcm5892macdma*);
static int	bcm5892dma_fillring(struct bcm5892macdma*);
static int	bcm5892dma_rx_process(struct bcm5892mac_softc*, struct bcm5892macdma*, int, int);
static void	bcm5892dma_tx_process(struct bcm5892mac_softc*, struct bcm5892macdma*, int);
static void	bcm5892mac_rx_overflow(struct bcm5892mac_softc *sc);

static int	bcm5892mac_initctx(struct bcm5892mac_softc*);
#if 0
static u64	bcm5892mac_addr2reg(unsigned char*);
#endif
static int	bcm5892mac_poll(struct napi_struct *napi, int budget);

#ifdef BCM5892_MCAST_SUPPORT
static u32 	bmc5892_partial_checksum(unsigned char *mc_addr,int len);
static void	bcm5892mac_promiscuous_mode(struct bcm5892mac_softc*, int);
#endif /*#ifdef BCM5892_MCAST_SUPPORT*/
static void	bcm5892mac_uninitctx(struct bcm5892mac_softc*);
static void 	bcm5892mac_channel_start(struct bcm5892mac_softc*);
static int	bcm5892mac_set_speed(struct bcm5892mac_softc*, enum bcm5892_speed);
static int	bcm5892mac_set_duplex(struct bcm5892mac_softc*,	enum bcm5892_duplex, enum bcm5892_fc);

#if 0
#ifdef BCM5892_MCAST_SUPPORT
static int	bcm5892mac_parse_xdigit(char);
#endif /*#ifdef BCM5892_MCAST_SUPPORT*/
#endif

static void	bcm5892mac_channel_stop(struct bcm5892mac_softc*);
static enum bcm5892_state
		bcm5892mac_set_channel_state(struct bcm5892mac_softc*, enum bcm5892_state);

static int	bcm5892_PHY_reset(struct bcm5892mac_softc *, u32);

static u16	bcm5892_mii_ctrl(struct bcm5892mac_softc*, u32, u32, u32, u16);
static int	bcm5892_mii_read(struct mii_bus*, int, int);
static int	bcm5892_mii_write(struct mii_bus*, int,	int, u16);
static void	bcm5892_mii_poll(struct net_device*);
static int	bcm5892_mdio_read(struct net_device*, int, int);
static void	bcm5892_mdio_write(struct net_device*, int, int, int);

static int	bcm5892_ethtool_get_settings(struct net_device*, struct ethtool_cmd *);
static int	bcm5892_ethtool_set_settings(struct net_device*, struct ethtool_cmd *);
static void	bcm5892_ethtool_get_drvinfo(struct net_device*, struct ethtool_drvinfo*);
static int	bcm5892_ethtool_get_regs_len(struct net_device*);
static void	bcm5892_ethtool_get_regs(struct net_device*, struct ethtool_regs*, void*);
static int	bcm5892_ethtool_nway_reset(struct net_device*);

static int	bcm5892mac_init(struct platform_device*);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,18) 
	static irqreturn_t	bcm5892mac_intr(int, void*, struct pt_regs*);
#else
	static irqreturn_t	bcm5892mac_intr(int, void*);
#endif /* #if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,18) */

static int	__init	bcm5892mac_probe(struct platform_device*);
static int	__exit	bcm5892mac_remove(struct platform_device*);
static int	__init	bcm5892mac_init_module(void);
static void	__exit	bcm5892mac_cleanup_module(void);

/* Functions related to PROC file system support */

static int get_debug_level(char *page, char **start, off_t off, int count, int *eof, void *data);
static int set_debug_level(struct file *file, const char *buffer, unsigned long count, void *data);
static int get_mac_cfg(char *page, char **start, off_t off, int count, int *eof, void *data);
static int get_miib_ctr(char *page, char **start, off_t off, int count, int *eof, void *data);
static int get_eth_ctrl(char *page, char **start, off_t off, int count, int *eof, void *data);
static int get_mii_reg(char *page, char **start, off_t off, int count, int *eof, void *data);
static void *tx_rng_seq_start(struct seq_file *s, loff_t *pos);
static void *tx_rng_seq_next(struct seq_file *s, void *v, loff_t *pos);
static void tx_rng_seq_stop(struct seq_file *s, void *v);
static int tx_rng_seq_show(struct seq_file *s, void *v);
static int tx_rng_open(struct inode *inode, struct file *file);
static int rx_rng_seq_show(struct seq_file *s, void *v);
static int rx_rng_open(struct inode *inode, struct file *file);
static int eth_mac_proc_create(struct net_device *dev);
static void eth_mac_proc_remove(void);

//static int set_proc_mac_cfg( struct file *file, const char __user *buffer,unsigned long count, void * data);

//static int get_proc_mac_cfg( char *page, char **start, off_t off,int count, int *eof, void *data);


/**********************************************************************
 *  The function declarations _End
 **********************************************************************/

/**********************************************************************
 *  The static data member declarations _Begin
 **********************************************************************/
static struct ethtool_ops bcm5892_ethtool_ops = 
{
	.get_settings = bcm5892_ethtool_get_settings,
	.set_settings = bcm5892_ethtool_set_settings,
	.get_drvinfo  = bcm5892_ethtool_get_drvinfo,
	.get_regs_len = bcm5892_ethtool_get_regs_len,
	.get_regs     = bcm5892_ethtool_get_regs,
	.get_link     = ethtool_op_get_link,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
	.set_ufo      = ethtool_op_set_ufo,
	.get_ufo      = ethtool_op_get_ufo,
#endif /* #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15) */
	.nway_reset   = bcm5892_ethtool_nway_reset
};

static struct seq_operations tx_rng_seq_ops = {
        .start  = tx_rng_seq_start,
        .next   = tx_rng_seq_next,
        .stop   = tx_rng_seq_stop,
        .show   = tx_rng_seq_show
};
static struct file_operations tx_rng_ops = {
        .owner  = THIS_MODULE,
        .open   = tx_rng_open,
        .read   = seq_read,
        .llseek = seq_lseek,
        .release= seq_release
};

static struct file_operations rx_rng_ops = {
        .owner = THIS_MODULE,
        .open  = rx_rng_open,
        .read  = seq_read,
        .llseek = seq_lseek,
        .release = seq_release
};



/**********************************************************************
 *  The static data member declarations _End
 **********************************************************************/

#if 0
#define NUMCHARS 8

static void 
data_dump(unsigned char *ptr, unsigned int len)
{
	unsigned int count;
	unsigned int i;
	unsigned int temp=0;
	unsigned char buff[NUMCHARS+1];
  
	if(NULL == ptr)	{
		PDEBUG("%s: WRONG INPUT\n", __FUNCTION__);
	}
	if (len == 0)
		len = 10;
	while (temp < len) {
		if (temp + NUMCHARS <= len)
			count = NUMCHARS;
		else
			count = len - temp;
		for (i = 0; i < count; i++) {
			printk(KERN_CRIT "%02x ", ptr[temp+i]);
			if ((ptr[temp+i] > 0x1f ) && (ptr[temp+i] < 0x7f))
				buff[i] = ptr[temp+i];
			else
				buff[i] = '.';
      		}
		for ( ; i < NUMCHARS; i++) {
			printk(KERN_CRIT "   ");
		}
		buff[count] = '\0';
		temp += count;
		printk(KERN_CRIT "\t %s\n", buff);
	}
}
#endif

#ifdef BCM5892_DUMP_STAT_REGS

static void
bcm5892_dump_stat_regs(struct bcm5892mac_softc *s)
{
        /* RX Stats */
        PDEBUG("RXFRMTOTAL : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_RXFRMTOTAL));
        PDEBUG("RXFRMGOOD  : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_RXFRMGOOD));
        PDEBUG("RXFRAG     : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_RXFRAG));
        PDEBUG("RXOVERRUN  : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_RXOVERRUN));
        PDEBUG("RXCRCALIGN : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_RXCRCALIGN));
        PDEBUG("RXUSIZE    : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_RXUSIZE)); 
        PDEBUG("RXCRC      : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_RXCRC));
        PDEBUG("RXALIGN    : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_RXALIGN));
        PDEBUG("RXCDERR    : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_RXCDERR));
        PDEBUG("RXFIFOSTAT : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_RXFIFOSTAT));

        /* TX Stats */
        PDEBUG("TXFRMTOTAL : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_TXFRMTOTAL));
        PDEBUG("TXFRMGOOD  : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_TXFRMGOOD));
        PDEBUG("TXFRAG     : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_TXFRAG));
        PDEBUG("TXUNDERRUN : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_TXUNDERRUN));
        PDEBUG("TXCOLTOTAL : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_TXCOLTOTAL));
        PDEBUG("TXCOL      : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_TX1COL));
        PDEBUG("TXMCOL     : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_TXMCOL));
        PDEBUG("TXEXCOL    : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_TXEXCOL));
        PDEBUG("TXLATE     : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_TXLATE));
        PDEBUG("TXDEFER    : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_TXDEFER));
        PDEBUG("TXCRS      : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_TXNOCRS));
        PDEBUG("TXFIFOSTAT : %d\n",__raw_readl(s->bcm5892_macbase + OFFSET_TXFIFOSTAT));
}

#endif

/**********************************************************************
 *  The function defiition to read MAC address from NAND device _Begin
 **********************************************************************/

/* Utility function. */
static unsigned char str2hexnum(unsigned char c)
{
        if(c >= '0' && c <= '9')
                return c - '0';
        if(c >= 'a' && c <= 'f')
                return c - 'a' + 10;
        if(c >= 'A' && c <= 'F')
                return c - 'A' + 10;
        return 0; /* foo */
}

/* Utility function. */
static void str2eaddr(unsigned char *ea, unsigned char *str)
{
        int i;

        for (i = 0; i < 6; i++) {
		unsigned char num;

		if((*str == '.') || (*str == ':'))
			str++;
		num = str2hexnum(*str++) << 4;
		num |= (str2hexnum(*str++));
		ea[i] = num;
        }
}


/**********************************************************************
 *  BCM5892_get _ethaddr(sc)
 *
 *  Read the MAC address from u-boot environment variable
 *  stored at NAND 2nd block and program to ethernet address
 *  register.
 *
 *  Input parameters:
 *	sc - struct bcm5892mac_softc (pointer to a MAC context)
 *
 *  Return value:
 *		   -1:  Success
 *		    0:  ERROR
 *********************************************************************/

static int
bcm5892_get_ethaddr( struct bcm5892mac_softc *sc)
{
	struct mtd_info *mtd = NULL;	
	char *ptr = NULL, *trbuf = NULL, ethaddr[6];
	uint8_t *rbuf = NULL;
	uint32_t hwaddr_lo = 0,hwaddr_hi = 0, i, len = 0;
	int rtn = 0;

	/* First clear previously configured MAC address
	 * in MAC register.
	 */

	printk(KERN_ERR "Inside get_ethaddr.\n");
	__raw_writel(0x0, sc->bcm5892_macbase + OFFSET_MACADDR0);
	__raw_writel(0x0, sc->bcm5892_macbase + OFFSET_MACADDR1);

	/* Get the handle to mtd device. */
	mtd = bcm5892_get_mtd_device_nm("NAND-block1");

	if (IS_ERR(mtd)) {
		PDEBUG("Unable to get mtd device.\n");
		return 0;
	}

	printk(KERN_ERR "after mtd.\n");
	/* Get buffer to read the u-boot environment variable. */
	rbuf = (uint8_t *)kmalloc(0x20000, GFP_KERNEL);
	if (rbuf == NULL) {
		PDEBUG("Fail to allocate memory.\n");
		bcm5892_put_mtd_device(mtd);
		return 0;
	}

	/* ReadNAND 2nd block */
	rtn = mtd->read(mtd, (unsigned long)0x60000, (uint32_t)0x20000, &len, rbuf);

	if ((rtn < 0) || (len < 0x20000)) {
		PDEBUG("MTD device read fail %d\n", rtn);
		bcm5892_put_mtd_device(mtd);
		kfree(rbuf);
		return 0;
	}

	printk(KERN_ERR "inside else.\n");
	trbuf = (char *)rbuf;

	/* Extract 'ethaddr' environment variable */
	while (trbuf <= (char *)(rbuf+0x20000)) {
		ptr = strstr((char *)trbuf, "ethaddr=");
		if (ptr != NULL) {
			str2eaddr(ethaddr, ptr + strlen("ethaddr="));
			for (i = 0; i < 4; i++) {
				hwaddr_lo = (hwaddr_lo << 0x08)|ethaddr[i];
			}
			for( i = 4; i < 6; i++) {
				hwaddr_hi = (hwaddr_hi << 0x08)|ethaddr[i];
			}
			__raw_writel(hwaddr_lo, sc->bcm5892_macbase + OFFSET_MACADDR0);
			__raw_writel(hwaddr_hi, sc->bcm5892_macbase + OFFSET_MACADDR1);
			printk(KERN_ERR "success return hi=0x%08x lo=0x%08x\n",
			       hwaddr_hi, hwaddr_lo);

			break;
		}
		len = strlen(trbuf);
		if (len) {
			trbuf += len + 1;
		}
		else {
			PDEBUG("MAC address not found.\n");
			bcm5892_put_mtd_device(mtd);
			kfree(rbuf);
			return 0;
		}
	}

	bcm5892_put_mtd_device(mtd);
	kfree(rbuf);
	printk(KERN_ERR" return.\n");
	return -1;
}


/**********************************************************************
 *  The function defiition to read MAC address from NAND device _End
 **********************************************************************/

/**********************************************************************
 *  The DMA function definitions _Begin
 **********************************************************************/

/**********************************************************************
 *  BCM5892dma_INITCTX(d,s,txrx,maxdescr)
 *
 *  Initialize a DMA channel context.
 *
 *  Input parameters:
 *	d - struct bcm5892macdma (DMA channel context)
 *	s - struct bcm5892mac_softc (pointer to a MAC context)
 *	txrx - Identifies DMA_TX or DMA_RX for channel direction
 *	maxdescr - number of descriptors
 *
 *  Return value:
 *		    0: Everything All Right
 *		not 0: ERROR
 **********************************************************************/

static int
bcm5892dma_initctx(
		   struct bcm5892macdma *d,
		   struct bcm5892mac_softc *s,
		   int txrx,
		   int maxdescr)
{
	int retVal = 0;
	u32 buf_size = 0;
	
	if (maxdescr % 2) {
		retVal = -EINVAL;
		PDEBUG("The Maximum Number of Descriptors should be and _EVEN_ Number");
		goto Exit;
	}

	/*
	 * Save away interesting stuff in the structure
	 */

 	d->bcm5892dma_eth = s;
 	d->bcm5892dma_txdir = txrx;

	/*
	 * Allocate memory for the ring
	 */

	d->bcm5892dma_maxdescr = maxdescr;
	buf_size = ALIGN64 + (sizeof(*d->bcm5892dma_dscrtable)) * (d->bcm5892dma_maxdescr);
 	d->bcm5892dma_dscrtable_unaligned = kcalloc(buf_size, sizeof(unsigned char), GFP_KERNEL);

	if (NULL == d->bcm5892dma_dscrtable_unaligned) {
		retVal = -ENOSPC;
		PDEBUG("%s() :LN%d: ERROR: Allocating Memory\n", __FUNCTION__, __LINE__);
		goto Exit;
	}	
 
	/*
	 * The descriptor table must be aligned to 64 Bytes Boundary.
	 * -The Spec.
	 */
	d->bcm5892dma_dscrtable = d->bcm5892dma_dscrtable_unaligned;
	if (0 != (((u32)(d->bcm5892dma_dscrtable)) % ALIGN64)) {
		d->bcm5892dma_dscrtable = (struct bcm5892dmadscr*)get_aligned(d->bcm5892dma_dscrtable_unaligned, ALIGN64);
	}

	d->bcm5892dma_dscrtable_end = d->bcm5892dma_dscrtable + d->bcm5892dma_maxdescr;
 	d->bcm5892dma_dscrtable_phys = virt_to_phys(d->bcm5892dma_dscrtable);

	/*
 	 * Allocate context table, No alignment requirement.
	 */
	d->bcm5892dma_ctxtable = kcalloc(d->bcm5892dma_maxdescr,
					 sizeof(*d->bcm5892dma_ctxtable),
					 GFP_KERNEL);
	if (NULL == d->bcm5892dma_ctxtable) {
		retVal = -ENOSPC;
		PDEBUG("%s() :LN%d: ERROR: Allocating Memory\n", __FUNCTION__, __LINE__);
		goto Exit;
	}

	return 0;
 Exit:
	if (NULL != d->bcm5892dma_ctxtable) {
		kfree(d->bcm5892dma_ctxtable);
		d->bcm5892dma_ctxtable = NULL;
	}

	if (NULL != d->bcm5892dma_dscrtable_unaligned) {
		kfree(d->bcm5892dma_dscrtable_unaligned);
		d->bcm5892dma_dscrtable_unaligned = NULL;
	}
	return retVal;
}

/**********************************************************************
 *  BCM5892dma_UNINITCTX(d)
 *
 *  Un-Initialize a DMA channel context.
 *
 *  Input parameters:
 *	d - struct bcm5892macdma (DMA channel context)
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/

static void
bcm5892dma_uninitctx(struct bcm5892macdma *d)
{
	if (d->bcm5892dma_dscrtable_unaligned) {
		kfree(d->bcm5892dma_dscrtable_unaligned);
		d->bcm5892dma_dscrtable_unaligned = d->bcm5892dma_dscrtable = NULL;
	}
	if (d->bcm5892dma_ctxtable) {
		kfree(d->bcm5892dma_ctxtable);
		d->bcm5892dma_ctxtable = NULL;
	}
	d->bcm5892dma_dscrtable_phys = 0;
}

/**********************************************************************
 *  BCM5892MACDMA_START(d, txrx)
 *
 *  Initialize the hardware registers for a DMA channel.
 *
 *  Input parameters:
 *         d - DMA channel to init (context must be previously init'd
 *         direction - DMA_RX or DMA_TX depending on what type of channel
 *
 *  Return value:
 *		    Nothing
 ***********************************************************************/
static void
bcm5892dma_start(struct bcm5892macdma *d, int direction)
{
	u32 water_mark = 0;
	u32 configVal = 0;
	u32 ringlen = 0;
	u32 regVal = 0;
        u32 rfbuffset = 0;

	/*
 	 * Initialize (Turn on?) the DMA channel
 	 */
	water_mark = (direction == DMA_RX) ? BCM5892_RFBWMRK : BCM5892_TFBWMRK;
	ringlen = d->bcm5892dma_maxdescr;
	configVal = (water_mark << 0x10)|ringlen;

 	__raw_writel(configVal, d->bcm5892dma_bcfg);

	/*
 	 * Initialize ring pointers
 	 */
	d->bcm5892dma_addptr = d->bcm5892dma_dscrtable;
	d->bcm5892dma_remptr = d->bcm5892dma_dscrtable;
	
	/*
	 * BufSize, if the DMA is RX, Ignore for TX
	 */ 
	regVal = (u32)(d->bcm5892dma_dscrtable_phys);
	__raw_writel(regVal, d->bcm5892dma_dscrbase);

	if (DMA_RX == direction) {
		rfbuffset = ((ENET_PACKET_SIZE / ALIGN64) + 1) << (BCM5892_RBUFFSZ_SHIFT);
		rfbuffset |= ( 1 << BCM5892_RBUFF_SET_BIT31 );
		PDEBUGG("Writing value 0x%08x on rbuffsz\n", rfbuffset);
		__raw_writel(rfbuffset,  d->bcm5892dma_buffsz);
		if (rfbuffset - __raw_readl(d->bcm5892dma_buffsz)) {
			PDEBUG("Read value 0x%08x on rbuffsz\n", __raw_readl(d->bcm5892dma_buffsz));
		}
	}
	__raw_writel(regVal, d->bcm5892dma_bdptr);
	/*__my_writel(__LINE__, regVal, d->bcm5892dma_bdptr);*/
	__raw_writel(regVal, d->bcm5892dma_swptr);
}

/**********************************************************************
 *  BCM5892DMA_STOP(d)
 *
 *  Initialize the hardware registers for a DMA channel.
 *
 *  Input parameters:
 *         d - DMA channel to init (context must be previously init'd)
 *
 *  Return value:
 *		    Nothing
 ********************************************************************* */

static
void bcm5892dma_channel_stop(struct bcm5892macdma *d)
{
	/*
	 * Turn off the DMA channel
	 */
 	__raw_writel(0, d->bcm5892dma_bcfg);
	__raw_writel(0, d->bcm5892dma_dscrtable);

	/*
	 * Zero ring pointers
	 */
	d->bcm5892dma_addptr = NULL;
	d->bcm5892dma_remptr = NULL;
}

/**********************************************************************
 *  BCM5892DMA_ALIGN_SKB(skb,power2,offset)
 *
 *  Align the allocated skbuf to a boundary specified by power2, that 
 *  done, put an offset given by offset.
 *
 *  Input parameters:
 *         skb - pointer to the newly allocated sk_buff
 *         power2 - Power (NonZero) of 2, to which the new alignment 
 *		    is required.
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/

static void
bcm5892dma_align_skb(
	struct sk_buff *skb, 
	int power2, 
	int offset)
{
	unsigned long addr;
	unsigned long newaddr;

	addr = (unsigned long) skb->data;
	newaddr = (unsigned long) get_aligned((void*)addr, power2);
	skb_reserve(skb, newaddr-addr+offset);
}

/**********************************************************************
 *  BCM5892DMA_ADD_RCVBUFFER(d,sb)
 *
 *  Add a buffer to the specified DMA channel.   For receive channels,
 *  this queues a buffer for inbound packets.
 *
 *  Input parameters:
 *         d - DMA channel descriptor
 *         sb - sk_buff to add, or NULL if we should allocate one
 *
 *  Return value:
 *		    0: if buffer added successfully
 *		not 0: ERROR
 **********************************************************************/

static int
bcm5892dma_add_rcvbuffer(struct bcm5892macdma* d,
			 struct sk_buff* sb)
{
	struct bcm5892dmadscr *dsc = NULL;
	struct bcm5892dmadscr *nextdsc = NULL;
	struct sk_buff *sb_new = NULL;

	/* get pointer to our current place in the ring */
	dsc = d->bcm5892dma_addptr;
	nextdsc = BCM5892DMA_NEXTBUF(d, bcm5892dma_addptr);

 	/*
 	 * figure out if the ring is full - if the next descriptor
 	 * is the same as the one that we're going to remove from
	 * the ring, the ring is full
	 */
	if (nextdsc == d->bcm5892dma_remptr) {
		PDEBUG("%s() :LN%d: ERROR: Allocating Memory\n", __FUNCTION__, __LINE__);
		return -ENOSPC;
	}

 	/*
	 * Allocate a sk_buff if we don't already have one.
 	 * If we do have an sk_buff, reset it so that it's empty.
	 *
	 * Note: sk_buffs don't seem to be guaranteed to have any sort
	 * of alignment when they are allocated.  Therefore, allocate enough
	 * extra space to make sure that:
	 *
	 *    1. the data does not start in the middle of a cache line.
	 *    2. The data does not end in the middle of a cache line
	 *    3. The buffer can be aligned such that the IP addresses are
	 *       naturally aligned.
	 *
	 * Now there seems to be a problem here. If we do not align for IP 
	 * address, the Kernel Network layer will ingest garbage, and the
	 * result will be a "slow" network at the best (or N/W failure at
	 * worst). _BUT_ if we align for IP address then, we violate the
	 * spec, which says that the buffer data write location should be
	 * 64(32?) aligned. Presently: NO Natural IP Address alignment.
	 *
	 * Remember, the SOCs MAC writes whole cache lines at a time,
	 * without reading the old contents first.  So, if the sk_buff's
	 * data portion starts in the middle of a cache line, the SOC
	 * DMA will trash the beginning (and ending) portions.
	 */
 	
	if (sb == NULL) {
		sb_new = dev_alloc_skb(ENET_PACKET_SIZE + ALIGN64 * 2);
		if (!sb_new) {
			PDEBUGG("%s: sk_buff allocation failed\n",
				d->bcm5892dma_eth->bcm5892_dev->name);
			return -ENOBUFS;
		}
 		bcm5892dma_align_skb(sb_new, ALIGN64, ETHER_ALIGN);
 	}
 	else {
		sb_new = sb;
		/*
		 * nothing special to reinit buffer, it's already aligned
		 * and sb->data already points to a good place.
		 */
 	}

	/*
	 * fill in the descriptor
	 */
	dsc->pData = virt_to_phys(sb_new->data);
    
	/*
	 * fill in the context
	 */
	d->bcm5892dma_ctxtable[dsc-d->bcm5892dma_dscrtable] = sb_new;

	/*
	 * point at next packet
	 */
	d->bcm5892dma_addptr = nextdsc;

	/*
	 * Give the buffer to the DMA Engine.
	 */
	__raw_writel((u32)virt_to_phys(dsc), d->bcm5892dma_swptr);
	
	return 0;
}

/**********************************************************************
 *  BCM5892DMA_ADD_TXBUFFER(d,sb)
 *
 *  Add a transmit buffer to the specified DMA channel, causing a
 *  transmit to start.
 *
 *  Input parameters:
 *         d - DMA channel descriptor
 *         sb - sk_buff to add
 * 
 *  Return value:
 *		    0: if buffer added successfully
 *		not 0: ERROR
 **********************************************************************/

static int
bcm5892dma_add_txbuffer(
	struct bcm5892macdma *d,
	struct sk_buff *sb)
{
	struct bcm5892dmadscr *currdsc = NULL;
	struct bcm5892dmadscr *nextdsc = NULL;
	u32 phys = 0;
	int length = 0;

	/* get pointer to our current place in the ring */
 	currdsc = d->bcm5892dma_addptr;
 	nextdsc = BCM5892DMA_NEXTBUF(d, bcm5892dma_addptr);

	/*
	 * figure out if the ring is full - if the next descriptor
	 * is the same as the one that we're going to remove from
	 * the ring, the ring is full
	 */
 	if (nextdsc == d->bcm5892dma_remptr) {
		PDEBUG("%s() :LN%d: ERROR: Allocating Memory, as the ring is FULL\n",
		       __FUNCTION__, __LINE__);
		return -ENOSPC;
	}

 	/*
	 * Under Linux, it's not necessary to copy/coalesce buffers
	 * like it is on NetBSD.  We think they're all contiguous,
	 * but that may not be true for GBE.
	 */
	length = sb->len;
	if (0 == length) {
		PDEBUG("ERROR: Data Length of zero is not acceptable");
		return -EINVAL;
	}

	/*
	 * fill in the descriptor.  Note that the number of cache
	 * blocks in the descriptor is the number of blocks
	 * *spanned*, so we need to add in the offset (if any)
	 * while doing the calculation.
	 */
	phys = virt_to_phys(sb->data);

	/*
	 * fill in the descriptor
	 */
	currdsc->flags = BCM5892_TX_BD_SOP | BCM5892_TX_BD_EOP | BCM5892_TX_BD_CAP | length;
	currdsc->pData = virt_to_phys(sb->data);
        
        dmac_flush_range(sb->data, sb->data+length);
	
	/*
	 * fill in the context
	 */
	d->bcm5892dma_ctxtable[currdsc - d->bcm5892dma_dscrtable] = sb;

	/*
	 * Give the buffer to the DMA Engine.
	 */
	nextdsc->flags = 0; //make sure the dummy packet is not valid

	__raw_writel((u32)(virt_to_phys(nextdsc)), d->bcm5892dma_swptr);

	/*
	 * point at next packet
	 */
	d->bcm5892dma_addptr = nextdsc;

	return 0;                                       /* Yeah, we did it */	
}

/**********************************************************************
 *  BCM5892DMA_EMPTYRING(d)
 *
 *  Free all allocated sk_buffs on the specified DMA channel;
 *
 *  Input parameters:
 *         d  - DMA channel
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/

static void
bcm5892dma_emptyring(struct bcm5892macdma *d)
{
	int idx;
	struct sk_buff *sb;
	
	if (NULL == d)
		return;

	for (idx = 0; idx < d->bcm5892dma_maxdescr; idx++) {
		sb = d->bcm5892dma_ctxtable[idx];
		if (sb)	{
			dev_kfree_skb(sb);
			d->bcm5892dma_ctxtable[idx] = NULL;
		}
	}
}

/**********************************************************************
 *  BCM5892DMA_FILLRING(d)
 *
 *  Fill the specified DMA channel (must be receive channel)
 *  with sk_buffs
 *
 *  Input parameters:
 *         d - DMA channel
 *
 *  Return value:
 *		    0: if buffer added successfully
 *		not 0: ERROR
 **********************************************************************/

static int 
bcm5892dma_fillring(struct bcm5892macdma *d)
{
	int idx;
	int retVal = 0;

	for (idx = 0; idx < BCM5892_MAX_RXDESCR - 1; idx++) {
		retVal = bcm5892dma_add_rcvbuffer(d, NULL);
		if (retVal != 0)
			break;
	}
	return retVal;
}

/**********************************************************************
 *  BCM5892DMA_RX_PROCESS(sc,d,work_to_do,poll)
 *
 *  Process "completed" receive buffers on the specified DMA channel.
 *
 *  Input parameters:
 *	sc - struct bcm5892mac_softc (pointer to a MAC context)
 *	d  - struct bcm5892macdma (DMA channel context)
 *    work_to_do - no. of packets to process before enabling interrupt
 *                 again (for NAPI)
 *    poll - polling status
 *
 *  Return value:
 *		    1: Done, nothing to recieve
 *		    0: ERROR
 **********************************************************************/

static int
bcm5892dma_rx_process(
		      struct bcm5892mac_softc *sc,
		      struct bcm5892macdma *d,
		      int work_to_do,
		      int poll)
{
	struct net_device *dev = sc->bcm5892_dev;

	int curidx = 0;
	int hwidx = 0;
	struct bcm5892dmadscr *dsc = NULL;
	struct sk_buff *sb = NULL;
	int len = 0;
	int work_done = 0;
	int dropped = 0;
 	u32 grs_val = 0;

	dev = dev;/*to do away with the warning*/

       	while (work_to_do-- >= 0) {
		unsigned long flags;

		/*
		 * figure out where we are (as an index) and where
		 * the hardware is (also as an index)
		 *
		 */
		spin_lock_irqsave(&sc->bcm5892_lock, flags);
		dsc = d->bcm5892dma_remptr;
		curidx = dsc - d->bcm5892dma_dscrtable;
		hwidx = ((__raw_readl(d->bcm5892dma_bdptr))
			 - (d->bcm5892dma_dscrtable_phys)) / 8;

		/*
		 * If they're the same, that means we've processed all
		 * of the descriptors up to (but not including) the one that
		 * the hardware is working on right now.
		 * 
		 * => After checking the index for equality also check RHLT[RAW_INTR].
		 * If RHLT==1, then do not exit thru done
		 */
		if (curidx == hwidx) {
			spin_unlock_irqrestore(&sc->bcm5892_lock, flags);
			break; // exit as the ring is empty
		}

#if 0
		/* Ring is completly full and GRS bit may be set. */
		if ((curidx + 0x8) == hwidx) {
			/* check if GRS bit  in ETH_CTRL is set or not.
			   If it is set then reset it */
			grs_val = __raw_readl(sc->bcm5892_ethctrl);
			if (grs_val & BCM5892_ETH_CTRL_GRS) {
				grs_val &= ~BCM5892_ETH_CTRL_GRS;
				__raw_writel(grs_val, sc->bcm5892_ethctrl);
			}
			/* Do not exit from loop as ring is full and not empty */
			spin_unlock_irqrestore(&sc->bcm5892_lock, flags);
			continue;
		}
#endif		
		/*
		 * Otherwise, get the packet's sk_buff ptr back
		 */
		sb = d->bcm5892dma_ctxtable[curidx];
		if (sb == NULL) {
			printk(KERN_ERR "%s: NULL sb in rx_process at idx %d hwidx %d\n",
			       bcm5892mac_string, curidx, hwidx);
		}

		d->bcm5892dma_ctxtable[curidx] = NULL;
  		len = d->bcm5892dma_dscrtable[curidx].flags & 0xFFFF;

		/*
		 * Check packet status.  If good, process it.
		 * If not, silently drop it and put it back on the
		 * receive ring.
		 */
		d->bcm5892dma_remptr = BCM5892DMA_NEXTBUF(d, bcm5892dma_remptr);
	     
		/* 
		 * Check for Inavlid packets
		 */
                if ((likely (!(dsc->flags & BCM5892_RX_BD_ER))) && (len > 61) ) {
			/*
			 * Add a new buffer to replace the old one.  If we fail
			 * to allocate a buffer, we're going to drop this
			 * packet and put it right back on the receive ring.
			 */

			if (unlikely (bcm5892dma_add_rcvbuffer(d, NULL) == -ENOBUFS)) {
				/*struct net_device_stats declared at latest version of
				  netdevice.h not in 2.6.14.2 version*/

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
				sc->bcm5892_stats.rx_dropped++;
#else
				dev->stats.rx_dropped++;
#endif /* #if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23) */
				/* re-add old buffer */
				bcm5892dma_add_rcvbuffer(d, sb);
				/* No point in continuing at the moment */
				PDEBUG("%s(): dropped packet (1)\n", __FUNCTION__);
				d->bcm5892dma_remptr = BCM5892DMA_NEXTBUF(d, bcm5892dma_remptr);
				spin_unlock_irqrestore(&sc->bcm5892_lock, flags);
				break;
			}
			else {
				/*
				 * Set length into the packet
				 */
 
				if (!sb) {
					spin_unlock_irqrestore(&sc->bcm5892_lock, flags);
					break;
				}
				skb_put(sb, len);

				/*
				 * Buffer has been replaced on the
				 * receive ring.  Pass the buffer to
				 * the kernel
				 */
				sb->protocol = eth_type_trans(sb, d->bcm5892dma_eth->bcm5892_dev);
				__cpuc_flush_kern_all();
				//dmac_flush_range(sb->data, sb->data + len);
				dropped = netif_rx(sb);
				if (NET_RX_DROP == dropped) {
					PDEBUGG("%s(): LINUX Network Core Drops Packet\n", __FUNCTION__);
					/*struct net_device_stats declared at latest 
					  version of netdevice.h not in 2.6.14.2 version, */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
					sc->bcm5892_stats.rx_dropped++;
#else
					dev->stats.rx_dropped++;
#endif /* #if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23) */

					spin_unlock_irqrestore(&sc->bcm5892_lock, flags);
					break;
				}
				else {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
					sc->bcm5892_stats.rx_bytes += len;
					sc->bcm5892_stats.rx_packets++;
#else
					dev->stats.rx_bytes += len;
					dev->stats.rx_packets++;
#endif /* #if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23) */
				}
			}
		}
		else {

                        if (likely ((dsc->flags & BCM5892_RX_BD_ER) && (len > 61))) {
				/*
				 * Received a Frame with CRC or Alignment error.
				 * Just drop it and put it back on the receive ring.
				 */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
				sc->bcm5892_stats.rx_frame_errors++;
#else
				dev->stats.rx_frame_errors++;
#endif /* #if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23) */
                        }
                        else {
				/*
				 * Packet was mangled somehow.  Just drop it and
				 * put it back on the receive ring.
				 */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
				sc->bcm5892_stats.rx_errors++;
#else
				dev->stats.rx_errors++;
#endif /* #if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23) */
				sc->runt = 1; 
			}
			bcm5892dma_add_rcvbuffer(d, sb);
		}
		spin_unlock_irqrestore(&sc->bcm5892_lock, flags);
		/*
		 * .. and advance to the next buffer.
		 */
		work_done++;
	}
#if 1
	/* Check for RX Overflow */
	if (__raw_readl(sc->bcm5892_intr_raw) & BCM5892_INTR_RAW_ROV)
		sc->rx_overflow = 1;

	if (sc->rx_overflow || sc->runt) {
		/* No need to lock interrupts here? */
		bcm5892mac_rx_overflow(sc);
		sc->runt = 0;
		sc->rx_overflow = 0;
	}

	/* If the RX side has stopped (GRS set), clear it
	 * to enable it the RX side to fill the newly cleared
	 * descriptor entries.
	 */
	grs_val = __raw_readl(sc->bcm5892_ethctrl);
	if (grs_val & BCM5892_ETH_CTRL_GRS) {
		grs_val &= ~BCM5892_ETH_CTRL_GRS;
		__raw_writel(grs_val, sc->bcm5892_ethctrl);
	}
#endif
	return work_done;
}

/**********************************************************************
*  BCM5892DMA_TX_PROCESS(sc,d,poll)
*
*  Process "completed" transmit buffers on the specified DMA channel.
*  It processes all of the packets on a given channel before
*  returning.
*
*  Input parameters:
*      sc - softc structure
*       d - DMA channel context
*    poll - polling state
*
*  Return value:
*		    Nothing
**********************************************************************/

static void
bcm5892dma_tx_process(struct bcm5892mac_softc *sc,
		      struct bcm5892macdma *d,
		      int poll)
{
	int curidx = 0;
	int hwidx = 0;
	struct sk_buff *sb = NULL;
	unsigned long flags = 0;
	int packets_handled = 0;
	struct net_device *dev = sc->bcm5892_dev;

	spin_lock_irqsave(&sc->bcm5892_lock, flags);
	if (d->bcm5892dma_remptr == d->bcm5892dma_addptr) {
//		PDEBUG("%s: remptr = 0x%08x, addptr = 0x%08x\n", __FUNCTION__, d->bcm5892dma_remptr, d->bcm5892dma_addptr);
//		PDEBUG("%s:Internal error! Trying to XMIT on empty ring\n", __FUNCTION__);
		goto end_unlock;
	}

	hwidx = (__raw_readl(d->bcm5892dma_bdptr) - d->bcm5892dma_dscrtable_phys)\
		/sizeof(*d->bcm5892dma_dscrtable);
	for (;;) {
		/*
		 * figure out where we are (as an index) and where
		 * the hardware is (also as an index)
		 */
		curidx = (d->bcm5892dma_remptr - d->bcm5892dma_dscrtable);

		/*
		 * If they're the same, that means we've processed all
		 * of the descriptors up to (but not including) the one that
		 * the hardware is working on right now.
		 */
		if (((curidx + 2) % BCM5892_MAX_TXDESCR) == hwidx)
			break;

		/*
		 * Otherwise, get the packet's sk_buff ptr back
		 */
		sb = d->bcm5892dma_ctxtable[curidx];
		d->bcm5892dma_ctxtable[curidx] = NULL;

		/*
		 * Stats
		 */
		if (sb != NULL) {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
			sc->bcm5892_stats.tx_bytes += sb->len;
			sc->bcm5892_stats.tx_packets++;
#else
			dev->stats.tx_bytes += sb->len;
			dev->stats.tx_packets++;
#endif
			/*
			 * for transmits, we just free buffers.
			 */
			dev_kfree_skb_irq(sb);

		}
		d->bcm5892dma_remptr = BCM5892DMA_NEXTBUF(d, bcm5892dma_remptr);
		packets_handled++;
		/* .. and advance to the next buffer.
		 */
	}

	/*
	 * Decide if we should wake up the protocol or not.
	 * Other drivers seem to do this when we reach a low
	 * watermark on the transmit queue.
	 */
	if (packets_handled) {
		netif_wake_queue(d->bcm5892dma_eth->bcm5892_dev);
	}

end_unlock:
	spin_unlock_irqrestore(&sc->bcm5892_lock, flags);
}
/**********************************************************************
 *  The DMA function definitions _End
 **********************************************************************/

/**********************************************************************
 *  The MAC function definitions _Begin
 **********************************************************************/

/**********************************************************************
 *  BCM5892MAC_INITCTX(s)
 *
 *  Initialize an Ethernet context structure - this is called
 *  once per MAC.  Memory is allocated here, so don't call it
 *  again from inside the ioctl routines that bring the interface
 *  up/down
 *
 *  Input parameters:
 *         s - bcm5892mac structure (Driver Context)
 *
 *  Return value:
 *         0
 **********************************************************************/

static int
bcm5892mac_initctx(struct bcm5892mac_softc *s)
{

	/*
	 * figure out the addresses of some ports
	 */
	/*s->bcm5892_macbase	= BCM5892_ETHERNET_MAC_BASE;*/
	s->bcm5892_ethctrl	= s->bcm5892_macbase + OFFSET_ETH_CTRL;	
	s->bcm5892_intrmask	= s->bcm5892_macbase + OFFSET_INTR_MASK;
	s->bcm5892_maskedintr	= s->bcm5892_macbase + OFFSET_INTR;
	s->bcm5892_intr_raw	= s->bcm5892_macbase + OFFSET_INTR_RAW;
	s->bcm5892_intr_clr	= s->bcm5892_macbase + OFFSET_INTR_CLR; 
	s->bcm5892_mcastaddrf0	= s->bcm5892_macbase + OFFSET_MCADDRF0;
	s->bcm5892_mcastaddrf1	= s->bcm5892_macbase + OFFSET_MCADDRF1;
	s->bcm5892_phyctrl	= s->bcm5892_macbase + OFFSET_PHYCTRL;
	s->bcm5892_rbufsize	= s->bcm5892_macbase + OFFSET_RBUFFSZ;
	s->bcm5892_rbase	= s->bcm5892_macbase + OFFSET_RBASE; 
	s->bcm5892_rcfg		= s->bcm5892_macbase + OFFSET_RBCFG;
	s->bcm5892_rbdptr	= s->bcm5892_macbase + OFFSET_RBDPTR;
	s->bcm5892_rswptr	= s->bcm5892_macbase + OFFSET_RSWPTR;
	s->bcm5892_tbase	= s->bcm5892_macbase + OFFSET_TBASE; 
	s->bcm5892_tcfg		= s->bcm5892_macbase + OFFSET_TBCFG;
	s->bcm5892_tbdptr	= s->bcm5892_macbase + OFFSET_TBDPTR; 
	s->bcm5892_tswptr	= s->bcm5892_macbase + OFFSET_TSWPTR;
	s->bcm5892_mac_bp	= s->bcm5892_macbase + OFFSET_MACBP;
	s->bcm5892_mac_cfg	= s->bcm5892_macbase + OFFSET_MACCFG; 
	s->bcm5892_macaddr0	= s->bcm5892_macbase + OFFSET_MACADDR0;
	s->bcm5892_macaddr1	= s->bcm5892_macbase + OFFSET_MACADDR1;
	s->bcm5892_maxfrm	= s->bcm5892_macbase + OFFSET_MAXFRM;
	s->bcm5892_pq		= s->bcm5892_macbase + OFFSET_MACPQ;
	s->bcm5892_rxfe		= s->bcm5892_macbase + OFFSET_MACRXFE;
	s->bcm5892_rxff		= s->bcm5892_macbase + OFFSET_MACRXFF;
	s->bcm5892_txfe		= s->bcm5892_macbase + OFFSET_MACTXFE;
	s->bcm5892_txff		= s->bcm5892_macbase + OFFSET_MACTXFF;
	s->bcm5892_macmode	= s->bcm5892_macbase + OFFSET_MACMODE;
	s->bcm5892_txipg	= s->bcm5892_macbase + OFFSET_TXIPG;
	s->bcm5892_txpctrl	= s->bcm5892_macbase + OFFSET_TXPCTRL;
	s->bcm5892_txfifof	= s->bcm5892_macbase + OFFSET_TXFIFOF;
	s->bcm5892_rxfifostat	= s->bcm5892_macbase + OFFSET_RXFIFOSTAT;
	s->bcm5892_txfifostat	= s->bcm5892_macbase + OFFSET_TXFIFOSTAT;

	s->bcm5892_miimgt	= s->bcm5892_mmibase + OFFSET_MIIMGT;
	s->bcm5892_miicmd	= s->bcm5892_mmibase + OFFSET_MIICMD;

	

	s->bcm5892_rxdma.bcm5892dma_buffsz	= s->bcm5892_rbufsize;
	s->bcm5892_rxdma.bcm5892dma_dscrbase	= s->bcm5892_rbase;
	s->bcm5892_rxdma.bcm5892dma_bcfg	= s->bcm5892_rcfg;
	s->bcm5892_rxdma.bcm5892dma_bdptr	= s->bcm5892_rbdptr;
	s->bcm5892_rxdma.bcm5892dma_swptr	= s->bcm5892_rswptr;

	s->bcm5892_txdma.bcm5892dma_dscrbase	= s->bcm5892_tbase;
	s->bcm5892_txdma.bcm5892dma_bcfg	= s->bcm5892_tcfg;
	s->bcm5892_txdma.bcm5892dma_bdptr	= s->bcm5892_tbdptr;
	s->bcm5892_txdma.bcm5892dma_swptr	= s->bcm5892_tswptr;
	
	/*
	 * Initialize the DMA channels.  Right now, only one per MAC is used
	 * Note: Only do this _once_, as it allocates memory from the kernel!
	 */

	if(bcm5892dma_initctx(&(s->bcm5892_txdma), s, DMA_TX, BCM5892_MAX_TXDESCR)) {
		PDEBUG("ERROR: Could not Initialize TX DMA\n");
		return -1;
	}

	if(bcm5892dma_initctx(&(s->bcm5892_rxdma), s, DMA_RX, BCM5892_MAX_RXDESCR)) {
		PDEBUG("ERROR: Could not Initialize RX DMA\n");
		bcm5892dma_uninitctx(&(s->bcm5892_txdma));
		return -1;
	}

	/*
	 * initial state is OFF
	 */
	__raw_writel(BCM5892_INTR_MASK_ALL, s->bcm5892_intrmask);
	s->current_state = bcm5892_state_off;
	return 0;
}

#if 0
/**********************************************************************
 *  BCM5892MAC_ADDR2REG(ptr)
 *
 *  Convert six bytes into the 64-bit register value that
 *  we typically write into the BCM5892MAC's address/mcast registers
 *
 *  Input parameters:
 *         ptr - pointer to 6 bytes
 *
 *  Return value:
 *		    Register value of 64 bits
 ********************************************************************* */

static u64
bcm5892mac_addr2reg(unsigned char *ptr)
{
	u64 reg = 0;

	ptr += 6;

	reg |= (u64) *(--ptr);
	reg <<= 8;
	reg |= (u64) *(--ptr);
	reg <<= 8;
	reg |= (u64) *(--ptr);
	reg <<= 8;
	reg |= (u64) *(--ptr);
	reg <<= 8;
	reg |= (u64) *(--ptr);
	reg <<= 8;
	reg |= (u64) *(--ptr);

	return reg;
}
#endif 

/**********************************************************************
 *  BCM5892MAC_CHANGE_MTU(dev, new_mtu)
 *
 *  Change the Maximum Transfer Unit.
 *
 *  Input parameters:
 *             _dev - The Network Device Context
 *          new_mtu - The new Maximum Trnasfer Unit
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/
static int
bcm5892mac_change_mtu(struct net_device *_dev, int new_mtu)
{
	if (new_mtu > ENET_PACKET_SIZE) {
		PDEBUG("ERROR: Invalid MTU Value %d\n", new_mtu);
		return -EINVAL;
	}
	_dev->mtu = new_mtu;
        return 0;
}

#ifdef BCM5892_MCAST_SUPPORT
/**********************************************************************
 *	bmc5892_partial_checksum(unsigned char *mc_addr,int len)
 *
 *  Do a "small" checksum calculation. This routine is used
 *  for calculating partial checksums on small pieces like
 *  headers, pseudoheaders, etc.  where we expect the header
 *  to already be in the cache (no prefetching or other
 *  fancy stuff needed).
 *  
 *  Input parameters:
 *      p - 16-bit aligned pointer
 *      len - length in bytes (must be multiple of 2)
 *      partial - 32-bit partial checksum
 *      
 *  Return value:
 *      new partial checksum
 ********************************************************************* */

static u32
bmc5892_partial_checksum(unsigned char *mc_addr,int len)
{    
	unsigned int crc_global,crc;
	unsigned int poly_global;
	unsigned int poly,i,x;
	unsigned char b, c, data,data_bytes;

	crc_global = 0xffffffff;
	poly_global = 0x04c11db6;
	crc = 0;

	data_bytes = len;

	for (x = 0; x < data_bytes; x++) {
		data = mc_addr[x];
		for (i = 0; i < 8; i = i + 1) {
			b = ((data & (1 << i)) >> i);
			c = ((crc_global & 0x80000000) >> 31);
			crc_global = crc_global << 1;
			if (c ^ b) {
				crc_global = crc_global ^ poly_global;
				crc_global = crc_global | 1;
			}
		}
		crc = crc_global;
		poly = poly_global;
	}
	return (crc);
}

/**********************************************************************
 *  BCM5892MAC_SETMULTI(sc)
 *
 *  Reprogram the multicast table into the hardware, given
 *  the list of multicasts associated with the interface
 *  structure.
 *
 *  Input parameters:
 *	sc - struct bcm5892mac_softc (Driver Software context)
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/

static void
bcm5892mac_setmulti(struct bcm5892mac_softc *sc)
{

	u32 reg32;
	int idx;
	struct dev_mc_list *mclist;
	struct net_device *dev = sc->bcm5892_dev;
        int i=0;
	u32 partial=0;
	u32 mc_index1=0;
	u32 mc_index0=0;

	/*
	 * Clear out entire multicast table.  We do this by nuking
	 * the entire hash table and all the direct matches except
	 * the first one, which is used for our station address
	 */
	//printk(KERN_INFO "Erasing Entire Multicast entries \n");
	__raw_writel(0, sc->bcm5892_mcastaddrf0);
	__raw_writel(0, sc->bcm5892_mcastaddrf1);

	idx=0;
        mclist = dev->mc_list;

	if (dev->flags & IFF_PROMISC) {
		printk(KERN_INFO "Promiscous mode:%d: bit set so enabling all",
		       dev->flags & IFF_PROMISC);
		reg32 = __raw_readl(sc->bcm5892_mac_cfg);
		reg32 |= BCM5892_MAC_CFG_PROM;
		__raw_writel(reg32, sc->bcm5892_mac_cfg);
		return;
	}
	if (dev->flags & IFF_ALLMULTI) {
		printk(KERN_INFO "All Multicast:%d: bit set so enabling all",
		       dev->flags & IFF_ALLMULTI);
		__raw_writel(0xFFFFFFFF, sc->bcm5892_mcastaddrf0);
		__raw_writel(0xFFFFFFFF, sc->bcm5892_mcastaddrf1);
		return;
	}
	if (dev->flags & IFF_MULTICAST) {
		mc_index1 = __raw_readl(sc->bcm5892_mcastaddrf1);
		mc_index0 = __raw_readl(sc->bcm5892_mcastaddrf0);

       		while (mclist) {
               		for (i = 0; i < mclist->dmi_addrlen; i++)
				partial=bmc5892_partial_checksum(&mclist->dmi_addr[0],
								 mclist->dmi_addrlen);
        		partial = partial & 0x0000003F;
			if (partial > 31)
				mc_index1 |= (((u32)1)<<(partial-32));
			else
				mc_index0 |= (((u32)1)<<(partial));
                	idx++;
                	mclist = mclist->next;
       		}
		__raw_writel(mc_index1, sc->bcm5892_mcastaddrf1);
		__raw_writel(mc_index0, sc->bcm5892_mcastaddrf0);
	}
}

#if 0

static void
bcm5892mac_setmulti(
	struct bcm5892mac_softc *sc
	)
{
//	u64 reg64;
	u32 reg32;
	int idx;
	struct dev_mc_list *mclist;
	struct net_device *dev = sc->bcm5892_dev;
// set mult merg
	int i=0;
	u32 partial;
	u32 mc_index1;
	u32 mc_index0;

	/*
	 * Clear out entire multicast table.  We do this by nuking
	 * the entire hash table and all the direct matches except
	 * the first one, which is used for our station address
	 */
	__raw_writel(0, sc->bcm5892_mcastaddrf0);
	__raw_writel(0, sc->bcm5892_mcastaddrf1);

/* STUB STATEMENTS*/	
#if 0
	/* Where is the Hash table base address in SPEC?? */
	for (idx = 0; idx < BCM5892_MAC_HASH_COUNT; idx++) {
		port = sc->macbase + OFFSET_MCHASH+(idx*sizeof(u64));
		__raw_writeq(0, port);
	}
#endif
	/*
	 * Clear the filter to say we don't want any multicasts.
	 */
	reg32 = __raw_readl(sc->bcm5892_mac_cfg);
	reg32 &= ~(BCM5892_MAC_CFG_PROM | BCM5892_MAC_CFG_ACFG);
	__raw_writel(reg32, sc->bcm5892_mac_cfg);
	if (dev->flags & IFF_ALLMULTI) {
		/*
		 * Enable ALL multicasts.  Do this by inverting the
		 * multicast enable bit.
		 */
		reg32 = __raw_readl(sc->bcm5892_mac_cfg);
		reg32 |= BCM5892_MAC_CFG_PROM;
//		__raw_writel(reg32, sc->bcm5892_mac_cfg); temporarily commented as multicasting not being used 
		return;
	}

	/*
	 * Progam new multicast entries.  For now, only use the
	 * perfect filter.  In the future we'll need to use the
	 * hash filter if the perfect filter overflows	
	 */

	/* XXX only using perfect filter for now, need to use hash
	 * XXX if the table overflows
	 */
	idx = 1;                /* skip station address */
	mclist = dev->mc_list;
	reg64 = bcm5892mac_addr2reg(mclist->dmi_addr);
	__raw_writeq(reg64, sc->bcm5892_mcastaddrf0);
}

 #endif 

/**********************************************************************
 *  BCM5892MAC_PROMISCUOUS_MODE(sc,onoff)
 *
 *  Turn on or off promiscuous mode
 *
 *  Input parameters:
 *		d - struct bcm5892mac_softc (Driver Software context)
 *	  onoff - 1 to turn on, 0 to turn off
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/

static void
bcm5892mac_promiscuous_mode(struct bcm5892mac_softc *sc, int onoff)
{
	u32 reg;
	volatile u32 mac_val_low=0x0; // used to hold lower 32 bits of mac address when setting up unicast mac address
	volatile u32 mac_val_hi=0x0;  // used to hold higher 32 bits of mac address when setting up unicast mac address

	if (sc->current_state != bcm5892_state_on)
		return;

	reg = __raw_readl(sc->bcm5892_mac_cfg);
	reg &= ~BCM5892_MAC_CFG_PROM;  
	if (onoff)
		reg |= BCM5892_MAC_CFG_PROM;
	else {
		mac_val_low = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACADDR0));	
		mac_val_hi  = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACADDR1));
	}
	__raw_writel(reg, sc->bcm5892_mac_cfg);
}

/**********************************************************************
 *  BCM5892MAC_SET_RX_MODE(dev)
 *
 *  Set the reception mode.
 *
 *  Input parameters:
 *              dev - The Network Device Context
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/

static void
bcm5892mac_set_rx_mode(
	struct net_device *dev
	)
{
	unsigned long flags = 0;
	struct bcm5892mac_softc *sc = netdev_priv(dev);

	spin_lock_irqsave(&sc->bcm5892_lock, flags);
//	if ((dev->flags ^ sc->bcm5892_devflags) & IFF_PROMISC)
//	{
		/*
		 * Promiscuous changed.
		 */
		if (dev->flags & IFF_PROMISC)
			bcm5892mac_promiscuous_mode(sc,1);
		else {
			bcm5892mac_promiscuous_mode(sc,0);
		}
//`	}
	spin_unlock_irqrestore(&sc->bcm5892_lock, flags);

	/*
	 * Program the multicasts.  Do this every time.
	 */
	bcm5892mac_setmulti(sc); 
}

#endif /* #ifdef BCM5892_MCAST_SUPPORT */

/*****************************************************************
 *	BCM5892MAC_SET_ADDR(struct net_device *dev, void *addr)
 *      Change MAC address thru ifconfig command  	
 *	
 * 	return int 
 ******************************************************************/
static int 
bcm5892mac_set_addr(struct net_device *dev, void *new_addr)
{
	struct bcm5892mac_softc *sc =  netdev_priv(dev);
 	int err = 0, i;
        u32 mac_reg,mac_reg0, mac_reg1;
        unsigned char eaddr[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};

      	dev = sc->bcm5892_dev;

        printk(KERN_ERR "going to get mac address from nand\n");
        err = bcm5892_get_ethaddr(sc);
        if (0 == err) {
                PDEBUG("Firmware has not provided MAC ADDRESS. Will be using hardcoded address.\n");
        }

	mac_reg = __raw_readl(sc->bcm5892_macbase + OFFSET_MACADDR0);
	mac_reg0 = BYTE_SWAP(mac_reg);
	mac_reg = __raw_readl(sc->bcm5892_macbase + OFFSET_MACADDR1);
	mac_reg1 = BYTE_SWAP(mac_reg);

        for (i = 0; i < 4; i++)	{
		eaddr[i] = (uint8_t) (mac_reg0 & 0xFF);
		//      printk(KERN_ERR "  0x%08x",eaddr[i]);
		mac_reg0 >>= 8;
	}
        mac_reg1 >>= 0x10;
        for (i = 4; i < 6; i++)	{
		eaddr[i] = (uint8_t)(mac_reg1 & 0xFF);
		//      printk(KERN_ERR "  0x%08x",eaddr[i]);
		mac_reg1 >>= 8;

	}
	memcpy(dev->dev_addr, eaddr, ETHER_ADDR_LEN);
                                             
	return 0;


	//	printk(KERN_NOTICE "Mac address setting feature not Supported in this version\n");
	//return 0;

	/*
	 *Mac address setting up via ifconfig  feature is unsupported
	 */

#if 0
	printk(KERN_INFO "new address= %s\n", *tmp_addr);
	if(!is_valid_ether_addr(saddr->sa_data))  return  -EINVAL;
	
	memcpy(dev->dev_addr, saddr->sa_data, dev->addr_len);
	printk(KERN_ERR "dev address= %s\n", dev->dev_addr);


	addr0 = dev->dev_addr[0] << 24 |
		dev->dev_addr[1] << 16 |
		dev->dev_addr[2] << 8  |
		dev->dev_addr[3];
	addr1 = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACADDR1));
 	addr1 &= ~0xffff;
	addr1 |= dev->dev_addr[4] << 8 | dev->dev_addr[5];
	printk(KERN_ERR "addr0= %x addr1= %x\n", addr0, addr1);
	__raw_writel(addr0, sc->bcm5892_macaddr0);	
	__raw_writel(addr1, sc->bcm5892_macaddr1);	
	addr0 = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACADDR0));	
	addr1  = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACADDR1));
	printk(KERN_ERR " Final  Hi mac =0x%08x lo mac = 0x%08x  \n",addr1, addr0);
	
	return 0;
#endif 
}


/**********************************************************************
 *  BCM5892MAC_UNINITCTX(sc)
 *
 *  Un-Initialize the Driver context.
 *
 *  Input parameters:
 *	sc - struct bcm5892mac_softc (Driver Software context)
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/

static void
bcm5892mac_uninitctx(struct bcm5892mac_softc *sc)
{
	bcm5892dma_uninitctx(&(sc->bcm5892_txdma));
	bcm5892dma_uninitctx(&(sc->bcm5892_rxdma));
}

/**********************************************************************
 *  BCM5892MAC_CHANNEL_START(s)
 *
 *  Start packet processing on this MAC.
 *
 *  Input parameters:
 *         sc - bc5892mac structure, the driver context
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/

static void bcm5892mac_channel_start(struct bcm5892mac_softc *sc)
{
//	u64 reg = (u64)0;
	int val = 0;
	int board_id = 0; // this value is used to store current board id
	/*
	 * Don't do this if running
	 */

	if (sc->current_state==bcm5892_state_on) {
		PDEBUG("%s: Channel ALREADY STARTED, Returning\n", __FUNCTION__);
		return;
	}

	/*
	 * 1. Put the ethernet MAC into reset state (Set MACCFG[SRST] = 1).
	 */
	val = __raw_readl(sc->bcm5892_mac_cfg);
	val |= BCM5892_MAC_CFG_SRST;
	__raw_writel(val, sc->bcm5892_mac_cfg);
	
	/*
	 * 2. Program the Maximum Frame Length.
	 */
	__raw_writel(BCM5892_MAC_MAX_FRM_DEFAULT_VAL, sc->bcm5892_maxfrm);

	/*
	 * 3. Program the Maximum Pause Qunta Length (Taking the reset val)
	 */
	__raw_writel(BCM5892_MAC_PQ_RESET_CASE_VAL, sc->bcm5892_pq);

	/*
	 * 4. The number of entries that must exist in the Rx FIFO 
	 *    to indicate that a new frame has arrived
	 */
	__raw_writel(BCM5892_MAC_RXFF_RECOMMENDED_VAL, sc->bcm5892_rxff);

	/*
	 * 5. The number of entries in the Transmit FIFO that will stop 
	 *    the Tx DMA from fetching new frame data.
	 */
	__raw_writel(BCM5892_MAC_TXFE_RECOMMENDED_VAL, sc->bcm5892_txfe);

	/*
	 * 6. The number of entries in the Transmit FIFO required before
	 *    the MAC will transmit a new frame.
	 */
	__raw_writel(BCM5892_MAC_TXFF_RECOMMENDED_VAL, sc->bcm5892_txff);

	/*
	 * 7. Programmable Inter-Packet Gap for Transmit Frames
	 */
	__raw_writel(BCM5892_MIN_TX_IPG_BEST_CASE, sc->bcm5892_txipg);

	/*
	 * 8. Clear out & Set the MAC ADDRESS
	 */
//	__raw_writel(0, sc->bcm5892_macaddr0);
//	__raw_writel(0, sc->bcm5892_macaddr1);

//	reg = bcm5892mac_addr2reg(sc->bcm5892_hwaddr);
//	__raw_writeq(reg, sc->bcm5892_macaddr0); /*<--Watch out for endianness issues!!!*/

	/* 9. disable Rx/Tx DMA */
	val = __raw_readl(sc->bcm5892_ethctrl);
	val |= (BCM5892_ETH_CTRL_GRS|BCM5892_ETH_CTRL_GTS);
	__raw_writel(val, sc->bcm5892_ethctrl);
	
	/*
	 * 10. Program the Rx DMA with: RING Base Addr, Len, Buf Size and
	 * enable the interrupt.
	 */
	bcm5892dma_start(&(sc->bcm5892_rxdma), DMA_RX);

	/*
	 * 11. Program the Tx DMA with: RING Base Addr, Len, Buf Size and
	 * enable the interrupt.
	 */
	bcm5892dma_start(&(sc->bcm5892_txdma), DMA_TX);

	/* 12. Fill receive ring */
	bcm5892dma_fillring(&(sc->bcm5892_rxdma));

	/*
	 * 13. Configure the speed, duplex, and flow control
	 */
	bcm5892mac_set_speed(sc, sc->current_speed);
	bcm5892mac_set_duplex(sc, sc->current_duplex, sc->current_fc);

	/* 14. Enable MAC Tx/Rx Path && Do Not Receive Everything */
	val = __raw_readl(sc->bcm5892_mac_cfg);
	val |= BCM5892_MAC_CFG_TXEN | BCM5892_MAC_CFG_RXEN;	/* Enable Tx/Rx Path */
	__raw_writel(val, sc->bcm5892_mac_cfg);

	/* 15. enable Rx/Tx DMA and MIB Stats */
	val = __raw_readl(sc->bcm5892_ethctrl);
	val &= ~BCM5892_ETH_CTRL_GRS;
	val |= BCM5892_ETH_CTRL_GTS | BCM5892_ETH_CTRL_MEN;
	__raw_writel(val, sc->bcm5892_ethctrl);

	/* 16.  Put PHY to Reset State, NORMAL OPS <IS IT Needed?> */
	val = __raw_readl(sc->bcm5892_phyctrl);
	val &= ~BCM5892_PHY_CTRL_EXT_PHY;

	/* clear EPHY low-power state */
	val &= ~(BCM5892_PHY_CTRL_PDB | BCM5892_PHY_CTRL_PDD | BCM5892_PHY_CTRL_PDP);
	__raw_writel(val, sc->bcm5892_phyctrl);

	/* 17. Enable interrupts */
        __raw_writel(BCM5892_INTR_MASK_RXF
		     | BCM5892_INTR_MASK_TXF
		     | BCM5892_INTR_MASK_ROV, sc->bcm5892_intrmask); 
	
	/*
	 * 20. Put the ethernet MAC out of reset state (Set MACCFG[SRST] = 0).
	 */
	val = __raw_readl(sc->bcm5892_mac_cfg);
	val &= (~BCM5892_MAC_CFG_SRST);
	__raw_writel(val, sc->bcm5892_mac_cfg);

	/* 17. We are on NOW */	
	sc->current_state = bcm5892_state_on;
	bcm5892mac_setmulti(sc);

	/* Check BOARD ID A0 or B0*/

	 board_id = __raw_readl(IO_ADDRESS(BCM5892_CHIPID_ADDR)); 
	 if (board_id != BCM5892_A0_ID) {
		 /*
		  * This code is added to take care of Rx Overflow Enable behaviour
		  * (MACCFG[12]) in B0 and production tape out.
		  * Following is the change description
		  *  Enable MACCFG[12] when the MAC is taken out of SW reset.
		  */
		 val = __raw_readl(sc->bcm5892_mac_cfg);
		 val |= (BCM5892_MAC_CFG_RxOFLO);
		 __raw_writel(val,sc->bcm5892_mac_cfg);
	 }
	 netif_start_queue(sc->bcm5892_dev);

	 PDEBUGG("%s: Channel STARTED\n", __FUNCTION__);
}

/**********************************************************************
 *  BCM5892MAC_SET_SPEED(s,speed)
 *
 *  Configure LAN speed for the specified MAC.
 *  Warning: must be called when MAC is off!
 *
 *  Input parameters:
 *	s - struct bcm5892mac_softc (pointer to a MAC context)
 *	speed - MAC Speed
 *
 *  Return value:
 *		    1: Success
 *		    0: Failure
 **********************************************************************/

static int
bcm5892mac_set_speed(struct bcm5892mac_softc *s, enum bcm5892_speed speed)
{
	u32 cfg = (u32)0;

	/*
	 * Save new current values
	 */
	s->current_speed = speed;
	if (s->current_state == bcm5892_state_on) {
		PDEBUG("New Speed Setting saved for(to be effective on) next restart\n");
		return 0;       /* save for next restart */
	}

	/*
	 * Read current register values
	 */
	cfg = __raw_readl(s->bcm5892_mac_cfg);

	/*
	 * Mask out the stuff we want to change
	 */
	cfg &= ~(0x03<<2); /*The speed is now effectively 10MBPS*/
	switch(speed) {
		case bcm5892_speed_10:
			cfg |= BCM5892_MAC_CFG_SPD_10;
			break;
		case bcm5892_speed_100:
			cfg |= BCM5892_MAC_CFG_SPD_100;
			break;
		default:
			PDEBUGG("ONLY 10/100 MBPS Allowed, for this driver NONE ELSE.\n");
			return 0;
	}
	__raw_writel(cfg, s->bcm5892_mac_cfg);
	return 1;
}

/**********************************************************************
 *  BCM5892MAC_SET_DUPLEX(s,duplex,fc)
 *
 *  Set Ethernet duplex and flow control options for this MAC
 *  Warning: must be called when MAC is off!
*
 *
 *  Input parameters:
 *		 s - struct bcm5892mac_softc (pointer to a MAC context)
 *      duplex - duplex setting (see enum bcm5892_duplex)
 *		fc - flow control setting (see enum bcm5892_fc)
 *
 *  Return value:
 *		    0: Success in changing mode
 *		not 0: ERROR
 **********************************************************************/

static int
bcm5892mac_set_duplex(
	struct bcm5892mac_softc *s,
	enum bcm5892_duplex duplex,
	enum bcm5892_fc fc
	)
{
	u32 cfg = (u32)0;

	/*
	 * Save new current values
	 */

	s->current_duplex = duplex;
	s->current_fc = fc;
	if (s->current_state == bcm5892_state_on) {
		PDEBUGG("New Duplex Setting saved for(to be effective on) next restart\n");
		return 0;       /* save for next restart */
	}
	/*
	 * Read current register values
	 */
	cfg = __raw_readl(s->bcm5892_mac_cfg);

	/*
	 * Mask off the stuff we're about to change
	 */
	cfg &= ~(BCM5892_MAC_CFG_HDEN);
	switch (duplex) {
		case bcm5892_duplex_half:
			cfg |= (BCM5892_MAC_CFG_HDEN);
			__raw_writel(cfg, s->bcm5892_mac_cfg);
			cfg = (BCM5892_MAC_BP_IPG_CFG|BCM5892_MAC_BP_BKEN);
		//	printk(KERN_ERR "HD:cfg=0x%08x\n",cfg);	
		//	cfg &=(~BCM5892_MAC_BP_BKEN);
	//		s->bcm5892_mac_bp=0x0;
	//		__raw_writel(cfg, s->bcm5892_mac_bp);
			//printk(KERN_ERR "HALF Duplex\n");
			break;

		case bcm5892_duplex_full:
			__raw_writel(cfg, s->bcm5892_mac_cfg);
			printk(KERN_ERR "FULL Duplex\n");
			break;

		default:
			PDEBUGG("Invalid duplex mode value\n");
			return 0;
	}
	return 1;
}

#if 0
#ifdef BCM5892_MCAST_SUPPORT
/**********************************************************************
 *  BCM5892MAC_PARSE_XDIGIT(str)
 *
 *  Parse a hex digit, returning its value
 *
 *  Input parameters:
 *         str - character
 *
 *  Return value:
 *		 hex value: Conversion Done
 *			-1: Invalid input.
 ********************************************************************* */
static int
bcm5892mac_parse_xdigit(
	char str
	)
{
	int digit;

	if ((str >= '0') && (str <= '9'))
		digit = str - '0';
	else if ((str >= 'a') && (str <= 'f'))
		digit = str - 'a' + 10;
	else if ((str >= 'A') && (str <= 'F'))
		digit = str - 'A' + 10;
	else
		return -1;
	return digit;
}
#endif /* #ifdef BCM5892_MCAST_SUPPORT */
#endif

/**********************************************************************
 *  BCM5892MAC_CHANNEL_STOP(s)
 *
 *  Stop packet processing on this MAC.
 *
 *  Input parameters:
 *	s - struct bcm5892mac_softc (pointer to a MAC context)
 *
 *  Return value:
 *         nothing
 **********************************************************************/
static void
bcm5892mac_channel_stop(struct bcm5892mac_softc *s)
{
	u32 val = 0;
	/* don't do this if already stopped */

	if (s->current_state == bcm5892_state_off) {
		PDEBUGG("%s: Channel ALREADY STOPPED, Returning\n", __FUNCTION__);
		return;
	}
	netif_stop_queue(s->bcm5892_dev);

	__raw_writel(0x01, s->bcm5892_txfifof);
	udelay(5);
	__raw_writel(0x00, s->bcm5892_txfifof);

	/* disable Rx/Tx DMA */
	val = __raw_readl(s->bcm5892_ethctrl);
	val |= (BCM5892_ETH_CTRL_GRS|BCM5892_ETH_CTRL_GTS);
	__raw_writel(val, s->bcm5892_ethctrl);

	/* don't accept any packets, disable all interrupts */
	__raw_writel(BCM5892_INTR_MASK_ALL, s->bcm5892_intrmask);

	/* disable MAC Tx/Rx Path */
	val = __raw_readl(s->bcm5892_mac_cfg);
	val &= (~(BCM5892_MAC_CFG_TXEN|BCM5892_MAC_CFG_RXEN));
	__raw_writel(val, s->bcm5892_mac_cfg);

	/* Turn off timers, if any */

	/* turn off receiver and transmitter */
	/* Put the whole UNIMAC Engine in S/W Reset */
	__raw_writel(BCM5892_MAC_CFG_SRST, s->bcm5892_mac_cfg);
	s->current_state = bcm5892_state_off;

	/*
	 * Stop DMA channels (rings should be ok now)
	 */
	bcm5892dma_channel_stop(&(s->bcm5892_rxdma));
	bcm5892dma_channel_stop(&(s->bcm5892_txdma));

	/* Empty the receive and transmit rings */
	bcm5892dma_emptyring(&(s->bcm5892_rxdma));
	bcm5892dma_emptyring(&(s->bcm5892_txdma));
 	mdelay(10);
}

/**********************************************************************
 *  BCM5892MAC_SET_CHANNEL_STATE(state)
 *
 *  Set the channel's state ON or OFF
 *
 *  Input parameters:
 *         sc - bcm5892mac structure (Driver Context)
 *         state - new state
 *
 *  Return value:
 *		    old State
 **********************************************************************/
static enum bcm5892_state
bcm5892mac_set_channel_state(struct bcm5892mac_softc *sc, enum bcm5892_state state)
{
	enum bcm5892_state oldstate = sc->current_state;

	/*
	 * If same as previous state, return
	 */

	if (state == oldstate) {
		return oldstate;
	}

	/*
	 * If new state is ON, turn channel on
	 */

	if (state == bcm5892_state_on) {
		bcm5892mac_channel_start(sc);
	}
	else {
		sc->current_state = bcm5892_state_off;
		bcm5892mac_channel_stop(sc);
	}

	/*
	 * Return previous state
	 */
	return oldstate;
}
/**********************************************************************
 *  The MAC function definitions _End
 **********************************************************************/

/**********************************************************************
 *  The MII (MDIO/PHY) Specific function definitions _Begin
 **********************************************************************/
static int
bcm5892_PHY_reset(
	struct	bcm5892mac_softc *sc,
	u32	phy_addr
	)
{
	struct net_device *dev;
	u32 val;
	int retVal = 0;

	dev = sc->bcm5892_dev;
	if ((0 == phy_addr)||(NULL == sc)) {
		PDEBUG("ERROR in %s(): Invalid INPUTS\n", __FUNCTION__);
		return -EINVAL;
	}

	/* enable MMI */
	val = 0xCD;	
	__raw_writel(val, sc->bcm5892_miimgt);
 
	/* Issue the reset */
	bcm5892_mdio_write(dev, phy_addr, MII_BMCR, 0x8000); 
	udelay(1);
	return retVal;
}


/**********************************************************************
 *  BCM5892_MII_PROBE(dev)
 *
 *  The Physical Layer 'MII Probing' Function.
 *
 *  Input parameters:
 *              dev - The Network Device Context
 *
 *  Return value:
 *		   (0-Success, Negative value-fail)
 **********************************************************************/
static int bcm5892_mii_probe( struct net_device *dev)
{
	struct bcm5892mac_softc *sc = netdev_priv(dev);
	struct phy_device *phy_dev = NULL;
	int retVal = 0, i = 0;

	for (i = 0; i < PHY_MAX_ADDR; i++) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#ifdef USE_MII_BUS_POINTER
		phy_dev = sc->mii_bus->phy_map[i];
#else
		phy_dev = sc->mii_bus.phy_map[i];
#endif
#else
		phy_dev = sc->mii_bus.phy_map[i];
#endif
		if (phy_dev) {
			PDEBUGG("\tThe PHY found at i=%d\n", i);
			break;
		}
	}
	if (!phy_dev) {
		PDEBUG("%s: no PHY found\n", dev->name);
		return -ENXIO;
        }  

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,20) 
	phy_dev = phy_connect(dev, phy_dev->dev.bus_id, &bcm5892_mii_poll, 0); 
#else
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,31) 
	phy_dev = phy_connect(dev, phy_dev->dev.bus_id, &bcm5892_mii_poll, 0, PHY_INTERFACE_MODE_MII); /*GMII??*/
#else
	phy_dev = phy_connect(dev, dev_name(&phy_dev->dev), &bcm5892_mii_poll, 0, PHY_INTERFACE_MODE_MII); /*GMII??*/
#endif /* LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,31) */
#endif /* #if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,20) */	

	if (IS_ERR(phy_dev)) {
		PDEBUG("%s: could not attach to PHY\n", dev->name);
		return PTR_ERR(phy_dev);
	}
	/* Remove any features not supported by the controller */
	phy_dev->supported &= SUPPORTED_10baseT_Half	|/*<-Sure about this*/
				SUPPORTED_10baseT_Full	|/*<-Sure about this*/
				SUPPORTED_100baseT_Half	|/*<-Sure about this*/
				SUPPORTED_100baseT_Full |/*<-Sure about this*/
				SUPPORTED_Autoneg	|/*<-Sure about this*/
				SUPPORTED_MII		|/*<-Sure about this*/
				SUPPORTED_Pause		|/*<-Sure about this*/
				SUPPORTED_Asym_Pause;	 /*<-Not sure about this*/

	phy_dev->advertising = phy_dev->supported;

	PDEBUG("%s: attached PHY driver [%s] (mii_bus:phy_addr=%s, irq=%d)\n",
		dev->name, phy_dev->drv->name,
		phy_dev->dev.bus_id, phy_dev->irq);

	sc->bcm5892phy_dev = phy_dev;
	return retVal;
}

/**********************************************************************
 *  BCM5892_MII_IOCTL(dev,rq,cmd)
 *
 *  The Physical Layer IOCTL handler Callback.
 *
 *  Input parameters:
 *              dev - The Network Device Context
 *	           rq - The ioctl code for this interface
 *              cmd - command data
 *
 *  Return value:
 *		   (0-Success, Negative value-fail)
 **********************************************************************/
static int
bcm5892_mii_ioctl(
	struct net_device *dev,
	struct ifreq *rq,
	int cmd
	)
{
	struct bcm5892mac_softc *sc = netdev_priv(dev);
	int retVal = 0;

	if (!netif_running(dev) || !sc->bcm5892phy_dev) {
		return -EINVAL;
	}

	retVal = phy_mii_ioctl(sc->bcm5892phy_dev, if_mii(rq), cmd);
	return retVal;
}

/**********************************************************************
 *  BCM5892_MII_CTRL(sc,phy_addr,dir,reg_addr,data)
 *
 *  Send some bits to the MII.  The bits to be sent are right-
 *  justified in the 'data' parameter.
 *
 *  Input parameters:
 *               sc - bcm5892mac structure (Driver Context)
 * 	     phy_addr - Address of PHY
 *              dir - read or write (direction of data movement)
 *         reg_addr - address to access
 *             data - data to send
 *  Return value:
 *		     0xffff: error occurred.
 *		Other Value: Value read
 **********************************************************************/
static u16 bcm5892_mii_ctrl( struct bcm5892mac_softc *sc,
				u32 phy_addr,
				u32 dir,
				u32 reg_addr,
				u16 data)
{
	u16 retVal	= (u16)(~0);
	u32 val		= (u32)0;
	u32 bsyVal	= (u32)0;
        u32 cmdVal = 0; 

        cmdVal = (BCM5892_MDIO_READ == dir) ? ((u32)(1<<29)) : ((u32)(1<<28));    

        val =   ((1<<30)       |       // Start
                 cmdVal        |       // command (read)
                 (phy_addr<<23)|       // phy addr (internal)
                 (reg_addr<<18)|       // reg addr 
                 (2<<16)       |       // Bus turn around
                 (data)                // read data (XXXXX)
		);

	do {
		bsyVal = __raw_readl(sc->bcm5892_miimgt); 
		udelay(BCM5892_PHY_POLL_USEC);
	} while((bsyVal & MMI_F_bsy_MASK) == MMI_F_bsy_MASK);

	__raw_writel(val, sc->bcm5892_miicmd);

	do {
		bsyVal = __raw_readl(sc->bcm5892_miimgt); 
		udelay(BCM5892_PHY_POLL_USEC);
	} while((bsyVal & MMI_F_bsy_MASK) == MMI_F_bsy_MASK);

	retVal = __raw_readl(sc->bcm5892_miicmd);
	return (0xFFFF & retVal);
}

/**********************************************************************
 *  BCM5892_MII_READ(bus, phy_addr, reg_addr)
 *
 *  Read a PHY register.
 *
 *  Input parameters:
 *            bus - MDIO bus handle
 *       phy_addr - PHY's address
 *       reg_addr - index of register to read
 *
 *  Return value:
 *		     0xffff: error occurred.
 *		Other Value: Value read
 **********************************************************************/
static int
bcm5892_mii_read(
	struct mii_bus *bus,
	int phy_addr,
	int reg_addr
	)
{
	struct bcm5892mac_softc *sc = (struct bcm5892mac_softc *)bus->priv;
	int retVal = 0;
	
	retVal = (int)bcm5892_mii_ctrl(sc, phy_addr, BCM5892_MDIO_READ, reg_addr, (u16)0);
	return retVal;
}

/**********************************************************************
 *  BCM5892_MII_WRITE(bus, phy_addr, regidx, regval)
 *
 *  Write a value to a PHY register.
 *
 *  Input parameters:
 *            bus - MDIO bus handle
 *       phy_addr - PHY's address
 *       reg_addr - index of register to read
 *         regval - data to write to register
 *
 *  Return value:
 *		     (0-Success, Negative value-fail)
 **********************************************************************/
static int
bcm5892_mii_write(
	struct mii_bus *bus,
	int phy_addr,
	int reg_addr,
	u16 regval
	)
{
	struct bcm5892mac_softc *sc = (struct bcm5892mac_softc *)bus->priv;
	int retVal = 0;
	
	retVal = (int)bcm5892_mii_ctrl(sc, phy_addr, BCM5892_MDIO_WRITE, reg_addr, regval);
	return retVal;
}
/**********************************************************************
 *  BCM5892_MII_POLL(dev)
 *
 *  The Media Intedpendent Interface Poll Callback
 *
 *  Input parameters:
 *         dev: The Network Device Context.
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/
static void
bcm5892_mii_poll(struct net_device *dev)
{
	struct bcm5892mac_softc *sc = netdev_priv(dev);
	struct phy_device *phy_dev = sc->bcm5892phy_dev;
	unsigned long flags = 0;
	enum bcm5892_fc fc = bcm5892_fc_none;
	int link_chg = 0, speed_chg = 0, duplex_chg = 0, pause_chg = 0, fc_chg = 0;

	link_chg	= (sc->current_link != phy_dev->link);
	speed_chg	= (sc->current_speed != phy_dev->speed);
	duplex_chg	= (sc->current_duplex != phy_dev->duplex);
	pause_chg	= (sc->current_pause != phy_dev->pause);

	if (!link_chg && !speed_chg && !duplex_chg && !pause_chg) {
		PDEBUGG("No Change");
		/* No change... */
		return;
	}
	PDEBUGG("%s:Change in link parameters!!\n", __FUNCTION__);
	printk(KERN_INFO "PHY %s: link status:%d Autoneg %d %dbase-%cD\n",
	       dev->name, phy_dev->link,phy_dev->autoneg,phy_dev->speed,
	       phy_dev->duplex == DUPLEX_FULL ? 'F' : 'H');

	if (!phy_dev->link) {
		if (link_chg) {
			sc->current_link = phy_dev->link;
			sc->current_speed = bcm5892_speed_none;
			sc->current_duplex = bcm5892_duplex_none;
			sc->current_fc = bcm5892_fc_disabled;
			sc->current_pause = -1;
			PDEBUG("%s: link unavailable\n", dev->name);
		}
		return;
	}

	if (phy_dev->duplex == DUPLEX_FULL) {
		fc = bcm5892_fc_disabled;
		if (phy_dev->pause) {
			fc = bcm5892_fc_frame;
		}
	}
	else {
		fc = bcm5892_fc_collision;
	}

	fc_chg = (sc->current_fc != fc);
	spin_lock_irqsave(&sc->bcm5892_lock, flags);
	sc->current_speed	= phy_dev->speed;
	sc->current_duplex	= phy_dev->duplex;
	sc->current_fc		= fc;
	sc->current_pause	= phy_dev->pause;
	sc->current_link	= phy_dev->link;

	if ((speed_chg || duplex_chg || fc_chg)) {
		/*
		 * something changed, restart the channel
		 */
		PDEBUGG("%s: restarting channel because PHY state changed\n", dev->name);
		bcm5892mac_channel_stop(sc);
		bcm5892mac_channel_start(sc);
		PDEBUGG("channel restart done\n");
	}
	spin_unlock_irqrestore(&sc->bcm5892_lock, flags);
	printk(KERN_INFO "CONTEXT %s: link available: %dbase-%cD\n", 
	       dev->name, sc->current_speed, 
	       sc->current_duplex == DUPLEX_FULL ? 'F' : 'H');
}

/**********************************************************************
 *  BCM5892_MDIO_READ(bus,phy_addr,reg_addr)
 *
 *  Read a PHY register.
 *
 *  Input parameters:
 *            dev - Network Device Context
 *       phy_addr - PHY's address
 *       reg_addr - index of register to read
 *
 *  Return value:
 *                   0xffff: error occurred.
 *              Other Value: Value read
 **********************************************************************/
static int
bcm5892_mdio_read(
	struct net_device *dev,
	int phy_addr,
	int reg_addr
	)
{
	int retVal = -1;

	PDEBUGG("\t%s()\n", __FUNCTION__);
	retVal = (int)bcm5892_mii_ctrl(netdev_priv(dev), phy_addr, BCM5892_MDIO_READ, reg_addr, 0);
	if(0xFFFF == retVal) {
		retVal = -1;
	}
	return retVal;
}

/**********************************************************************
 *  BCM5892_MDIO_WRITE(bus,phy_addr,reg_addr,data)
 *
 *  Write to value to a PHY register.
 *
 *  Input parameters:
 *            dev - Network Device Context
 *       phy_addr - PHY's address
 *       reg_addr - index of register to read
 *	     data - 
 *
 *  Return value:
 *                  Nothing
 **********************************************************************/
static void
bcm5892_mdio_write(
	struct net_device *dev,
	int phy_addr,
	int reg_addr,
	int data
	)
{
	PDEBUGG("\t%s()\n", __FUNCTION__);
//	printk(KERN_ERR "#####reg_addr *x data %x #####\n",reg_addr,data);
	bcm5892_mii_ctrl(netdev_priv(dev), phy_addr, BCM5892_MDIO_WRITE, reg_addr, data);
}

/**********************************************************************
 *  The MII (MDIO/PHY) Specific function definitions _End
 **********************************************************************/

/**********************************************************************
 *  The ETHTOOL OPS Specific function definitions _Begin
 **********************************************************************/
/**********************************************************************
 *  BCM5892_ETHTOOL_GET_SETTINGS(device, cmd)
 *
 *  Return link specific information.
 *
 *  Input parameters:
 *
 *              dev - The Network Device Context
 *              cmd - The ETHTOOL Command Context
 *
 *  Return value:
 *		    0: Everything All Right
 *		not 0: ERROR <Impossible/Could be FATAL>
 **********************************************************************/

static int bcm5892_ethtool_get_settings(
	struct net_device *dev, 
	struct ethtool_cmd *cmd
	)
{
	int retVal = 0;
	struct bcm5892mac_softc *sc = netdev_priv(dev);
	

	u32 support_speeds =   (SUPPORTED_10baseT_Half|
				SUPPORTED_10baseT_Full|
				SUPPORTED_100baseT_Half|
				SUPPORTED_100baseT_Full|
				SUPPORTED_1000baseT_Half|
				SUPPORTED_1000baseT_Full);

	u32 advertng_speeds =  (ADVERTISED_10baseT_Half|
				ADVERTISED_10baseT_Full|
				ADVERTISED_100baseT_Half|
				ADVERTISED_100baseT_Full|
				ADVERTISED_1000baseT_Half|
				ADVERTISED_1000baseT_Full);
	
	 retVal = mii_ethtool_gset(&sc->mii_info, cmd);

	if (0 != retVal) {
		PDEBUG("FATAL ERROR: Could not get Phy Device Settings, Return Value == %d\n", retVal);
		return retVal;
	}
	
	cmd->supported &= ~(support_speeds);
	cmd->supported |= SUPPORTED_10baseT_Half | 
			SUPPORTED_10baseT_Full  |
			SUPPORTED_100baseT_Half |
			SUPPORTED_100baseT_Full |
			SUPPORTED_Autoneg	|
			SUPPORTED_MII		|
			SUPPORTED_Pause 	|
			SUPPORTED_Asym_Pause; 
	cmd->advertising &= ~(advertng_speeds);
	cmd->advertising = 
			ADVERTISED_10baseT_Half  | 
			ADVERTISED_10baseT_Full  |
			ADVERTISED_100baseT_Half |
			ADVERTISED_100baseT_Full |
			ADVERTISED_Autoneg	  |
			ADVERTISED_MII		  |
			ADVERTISED_Pause 	  |
			ADVERTISED_Asym_Pause;
	cmd->port		= PORT_MII;
	cmd->transceiver	= XCVR_INTERNAL;
	//cmd->autoneg		= AUTONEG_ENABLE;

	if (0 == netif_carrier_ok(sc->bcm5892_dev)) {
		cmd->speed = -1;
		cmd->duplex = -1;
	}
	return 0;
}


/**********************************************************************
 *  BCM5892_ETHTOOL_SET_SETTINGS(device, cmd)
 *
 *  Sets different link parameters.
 *
 *  Input parameters:
 *
 *              dev - The Network Device Context
 *              cmd - The ETHTOOL Command Context
 *
 *  Return value:
 *		    0: Everything All Right
 *		not 0: ERROR 
 **********************************************************************/

static int bcm5892_ethtool_set_settings(
	struct net_device *dev, 
	struct ethtool_cmd *cmd
	)
{
	int retVal = -EINVAL;

	struct bcm5892mac_softc *sc = netdev_priv(dev);

	if((cmd->advertising & ADVERTISED_1000baseT_Half)||
           (cmd->advertising & ADVERTISED_1000baseT_Full)) {
		PDEBUG("Trying to Advertise Wrong Speed, are we??\n");
		goto Exit;
	}
	
	if((cmd->supported & SUPPORTED_1000baseT_Half)||
           (cmd->supported & SUPPORTED_1000baseT_Full)) {
		PDEBUG("Trying to Support Wrong Speed, are we??\n");
		goto Exit;
	}

	if(!((SPEED_10 == cmd->speed)||(SPEED_100 == cmd->speed))) {
		PDEBUG("Trying to Set Wrong Speed, are we??\n");
		goto Exit;
	}

	if(PORT_MII != cmd->port) {
		PDEBUG("Trying to Set Wrong Port Type, are we??\n");
		goto Exit;
	}
	

//	if(AUTONEG_ENABLE != cmd->autoneg)
//	{
//		PDEBUG("Trying to Disable Auto-Negotiation, are we??\n");
//		goto Exit;
//	}
	
	if(XCVR_INTERNAL != cmd->transceiver) {
		PDEBUG("Trying to Select (may be external??) some other XCeiver, are we??\n");
		goto Exit;
	}

	retVal = mii_ethtool_sset(&sc->mii_info, cmd);
	if(0 > retVal) {
		PDEBUG("Hmm! Some other \"Un-Recognised\' Error\n");
	}

Exit:
	return retVal;
}

/**********************************************************************
 *  BCM5892_ETHTOOL_GET_DRVINFO(dev,info)
 *
 *  Returns driver specific "text" information.
 *
 *  Input parameters:
 *              dev - The Network Device Context
 *             info - The ETHTOOL Information Context
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/

static void
bcm5892_ethtool_get_drvinfo(
	struct net_device *dev,
	struct ethtool_drvinfo *info
	)
{
	struct bcm5892mac_softc *sc     = netdev_priv(dev);
	struct platform_device *pldev   = NULL;
	struct resource *res		= NULL;
	struct resource *mmi_res	= NULL;


	strncpy(info->driver, bcm5892mac_string, sizeof(info->driver));
	strncpy(info->version, DRV_VERSION, sizeof(info->version));
	strncpy(info->fw_version, "", sizeof(info->fw_version));

	//if(NULL == sc->bcm5892_dev->class_dev.dev->parent/*sc->bcm5892_dev->dev.parent*/)
	if (NULL == sc->bcm5892_dev->dev.parent) {
		PDEBUG("The Parent type string is null. FATAL ERROR?\n");
		return;
	}

	//pldev = to_platform_device(sc->bcm5892_dev->class_dev.dev->parent/*bcm5892_dev->dev.parent*/);

	pldev = to_platform_device(sc->bcm5892_dev->dev.parent/*bcm5892_dev->dev.parent*/);

	if (NULL == pldev) {
		PDEBUG("The Platform device pointer is null. FATAL ERROR?\n");
		return;
	}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,31) 
	strncpy(info->bus_info, sc->bcm5892_dev->dev.parent->bus_id, sizeof(info->bus_info));
#else
	strncpy(info->bus_info, dev_name(sc->bcm5892_dev->dev.parent), sizeof(info->bus_info));
#endif
	res = bcm5892_platform_get_resource(pldev, IORESOURCE_MEM, IDX_IORES_MAC_MEM);
	mmi_res = bcm5892_platform_get_resource(pldev, IORESOURCE_MEM, IDX_IORES_MMI_MEM);

	if ((NULL == res)||(NULL == mmi_res)) {
		PDEBUG("The Platform resource pointer is null. FATAL ERROR?\n");
		return;
	}

	info->regdump_len = (res->end - res->start + 1 + mmi_res->end - mmi_res->start + 1); 
}


/**********************************************************************
 *  BCM5892_ETHTOOL_GET_REGS_LEN(dev)
 *
 *  Returns the length of all MAC+PHY Register length
 *
 *  Input parameters:
 *              dev - The Network Device Context
 *
 *  Return value:
 *		    0: ERROR
 *		not 0: Length of the registers
 **********************************************************************/

static int
bcm5892_ethtool_get_regs_len(struct net_device *dev)
{
	struct bcm5892mac_softc *sc     = netdev_priv(dev);
	struct platform_device *pldev   = NULL;
	struct resource *res		= NULL;
	struct resource *mmi_res	= NULL;
	int retVal = 0;
	
	if (NULL == sc->bcm5892_dev->dev.parent) {
		PDEBUG("The Parent type string is null. FATAL ERROR?\n");
		return 0;
	}

	pldev = to_platform_device(sc->bcm5892_dev->dev.parent);
	if (NULL == pldev) {
		PDEBUG("The Platform device pointer is null. FATAL ERROR?\n");
		return 0;
	}

	res = bcm5892_platform_get_resource(pldev, IORESOURCE_MEM, IDX_IORES_MAC_MEM);
	mmi_res = bcm5892_platform_get_resource(pldev, IORESOURCE_MEM, IDX_IORES_MMI_MEM);
	if ((NULL == res)||(NULL == mmi_res)) {
		PDEBUG("The Platform resource pointer is null. FATAL ERROR?\n");
		return 0;
	}
	retVal = (res->end - res->start + 1 + mmi_res->end - mmi_res->start + 1);
        return retVal;
}

/**********************************************************************
 *  BCM5892_ETHTOOL_GET_REGS(dev,regs,space)
 *
 *  Dumps the entire register space of BCM5892 MAC+PHY into the buffer.
 *
 *  Input parameters:
 *              dev - The Network Device Context
 *	       regs - Pointer to the structure with parameters given
 *		      by ethtool for dumping the registers.
 *            space - The input argumnet into which all the registers
 *		      are dumped.
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/

static void
bcm5892_ethtool_get_regs(
	struct net_device *dev,
	struct ethtool_regs *regs,
	void *space
	)
{
	struct bcm5892mac_softc *sc     = netdev_priv(dev);
	struct platform_device *pldev   = NULL;
	struct resource *res		= NULL;
	struct resource *mmi_res	= NULL;
	int i = 0, j = 0, offset = 0;
	u8 *reg_space = (u8 *) space;

	
	if (NULL == sc->bcm5892_dev->dev.parent) {
		PDEBUG("The Parent type string is null. FATAL ERROR?\n");
		return;
	}

	pldev = to_platform_device(sc->bcm5892_dev->dev.parent);
	if (NULL == pldev) {
		PDEBUG("The Platform device pointer is null. FATAL ERROR?\n");
		return;
	}

	res = bcm5892_platform_get_resource(pldev, IORESOURCE_MEM, IDX_IORES_MAC_MEM);
	mmi_res = bcm5892_platform_get_resource(pldev, IORESOURCE_MEM, IDX_IORES_MMI_MEM);
	
	if ((NULL == res)||(NULL == mmi_res)) {
		PDEBUG("The Platform resource pointer is null. FATAL ERROR?\n");
		return;
	}

	offset = j = res->end - res->start + 1;
	for (i = 0; i < j; i += 4) {
		memcpy((unsigned char*)(reg_space+i),
		       ((unsigned char*)(res->start) + i), 4);
	}
	j = mmi_res->end - mmi_res->start + 1;
	for (i = 0; i < j; i += 4) {
		memcpy((unsigned char*)(reg_space + offset + i),
		       ((unsigned char*)(mmi_res->start) + i), 4);
	}
	return;
}

/**********************************************************************
 *  BCM5892_ETHTOOL_NWAY_RESET(dev)
 *
 *  Restart the Autonegotiation for this nic.
 *
 *  Input parameters:
 *
 *              dev - The Network Device Context
 *
 *  Return value:
 *		    0: Everything All Right
 *		not 0: ERROR 
 **********************************************************************/

static int bcm5892_ethtool_nway_reset(struct net_device *dev)
{
	struct bcm5892mac_softc *sc = netdev_priv(dev);
	int bmcr = 0;
	int retVal = -EINVAL;

	/* if autoneg is off, it's an error */
	//bmcr = sc->mii_bus.read(&sc->mii_bus, sc->mii_bus.id, MII_BMCR);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#ifdef USE_MII_BUS_POINTER
	bmcr = sc->mii_bus->read(sc->mii_bus, (int)sc->mii_bus->id, MII_BMCR);
#else
	bmcr = sc->mii_bus.read(&sc->mii_bus, (int)sc->mii_bus.id, MII_BMCR);
#endif
#else
	bmcr = sc->mii_bus.read(&sc->mii_bus, (int)sc->mii_bus.id, MII_BMCR);
#endif

	if (bmcr & BMCR_ANENABLE) {
 		bmcr |= BMCR_ANRESTART;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#ifdef USE_MII_BUS_POINTER
		bmcr = sc->mii_bus->write(sc->mii_bus, (int)sc->mii_bus->id, MII_BMCR, bmcr);
#else
		bmcr = sc->mii_bus.write(&sc->mii_bus, (int)sc->mii_bus.id, MII_BMCR, bmcr);
#endif
#else
		bmcr = sc->mii_bus.write(&sc->mii_bus, (int)sc->mii_bus.id, MII_BMCR, bmcr);
#endif
		retVal = 0;
	}

	return retVal;
}

/**********************************************************************
 *  The ETHTOOL OPS Specific function definitions _End
 **********************************************************************/

/**********************************************************************
 *  The Kernel Specific function definitions _Begin
 **********************************************************************/

/**********************************************************************
 *  BCM5892MAC_START_TX(skb,dev)
 *
 *  Start output on the specified interface.  Basically, we
 *  queue as many buffers as we can until the ring fills up, or
 *  we run off the end of the queue, whichever comes first.
 *
 *  Input parameters:
 *           skb - socket buffer or sk_buff
 *           dev - Network Device context
 *
 *  Return value:
 *		    0: Transmission OK
 *		not 0: ERROR
 ***********************************************************************/
static int
bcm5892mac_start_tx(
	struct sk_buff *skb,
	struct net_device *dev
	)
{
	u32 val = 0;
	struct bcm5892mac_softc *sc = netdev_priv(dev);

	/* lock eth irq */
	spin_lock_irq (&sc->bcm5892_lock);

	val = __raw_readl(sc->bcm5892_ethctrl);
	if(0 == (BCM5892_ETH_CTRL_GTS & val)) {
		spin_unlock_irq (&sc->bcm5892_lock);
		return 1;
	}

	val = __raw_readl(sc->bcm5892_mac_cfg);
	val &= (~(BCM5892_MAC_CFG_TXEN)); /*Disable MAC Tx Path*/
	__raw_writel(val, sc->bcm5892_mac_cfg);
	__raw_writel((u32)(__raw_readl(sc->bcm5892_txdma.bcm5892dma_swptr)),
			sc->bcm5892_txdma.bcm5892dma_bdptr);
	/*
	 * Put the buffer on the transmit ring.  If we
	 * don't have room, stop the queue.
	 */
	if (bcm5892dma_add_txbuffer(&(sc->bcm5892_txdma), skb)) {
		/* XXX save skb that we could not send */
		netif_stop_queue(dev);
		spin_unlock_irq(&sc->bcm5892_lock);
		return 1;
	}
	dev->trans_start = jiffies;

	val = __raw_readl(sc->bcm5892_mac_cfg);
	val |= ((BCM5892_MAC_CFG_TXEN)); /*Enable MAC Tx Path*/
	__raw_writel(val, sc->bcm5892_mac_cfg);

	val = __raw_readl(sc->bcm5892_ethctrl);
	val &= (~(BCM5892_ETH_CTRL_GTS));
	__raw_writel(val, sc->bcm5892_ethctrl);

	PDEBUGG("\t%s:Spin Lock released\n", __FUNCTION__);
	spin_unlock_irq (&sc->bcm5892_lock);

	return 0;
}

/**********************************************************************
 *  BCM5892MAC_CLOSE(dev)
 *
 *  The Device Driver operations "close" callback.
 *
 *  Input parameters:
 *         dev: The network device context 
 *
 *  Return value:
 *		    0: All the time.
 **********************************************************************/
static int
bcm5892mac_close(
	struct net_device *dev
	)
{
	struct bcm5892mac_softc *sc = netdev_priv(dev);
	int retVal = 0;
	u32 val;
	unsigned long flags = 0;
	phy_stop(sc->bcm5892phy_dev);

	PDEBUG("%s: Shutting down ethercard\n", dev->name);

	napi_disable(&sc->napi);

	bcm5892mac_set_channel_state(sc, bcm5892_state_off);

	phy_disconnect(sc->bcm5892phy_dev);
	sc->bcm5892phy_dev = NULL;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#ifdef USE_MII_BUS_POINTER
	mdiobus_unregister(sc->mii_bus);
#else
	mdiobus_unregister(&sc->mii_bus);
#endif
#else
	mdiobus_unregister(&sc->mii_bus);
#endif

	/* Put EPHY in low-power state */
	val = __raw_readl(sc->bcm5892_phyctrl);
	val |= BCM5892_PHY_CTRL_PDB | BCM5892_PHY_CTRL_PDD | BCM5892_PHY_CTRL_PDP;
	__raw_writel(val, sc->bcm5892_phyctrl);

	/* Clear Any pending interrupt */

	spin_lock_irqsave(&sc->bcm5892_lock, flags);
	__raw_writel(BCM5892_INTR_UNMASK_ALL, sc->bcm5892_intr_clr);
	__raw_writel(0, sc->bcm5892_intr_clr);
	spin_unlock_irqrestore(&sc->bcm5892_lock, flags);

	free_irq(dev->irq, dev);
	bcm5892dma_emptyring(&(sc->bcm5892_txdma));
	bcm5892dma_emptyring(&(sc->bcm5892_rxdma));

	return retVal;
}

/**********************************************************************
 *  BCM5892MAC_OPEN(dev)
 *
 *  The Device Driver operations "open" callback.
 *
 *  Input parameters:
 *         dev: The network device context 
 *
 *  Return value:
 *		    0: Opening is Successful
 *		not 0: ERROR
 **********************************************************************/

static int
bcm5892mac_open(
	struct net_device *dev
	)
{
	struct bcm5892mac_softc *sc = netdev_priv(dev);
	int err = -1;
	u32 val, val1;

// u32 phy_rst_val = 0;
	if (NULL == sc) {
		PDEBUG("Wrong/NULL value of device Soft Context Pointer\n");
		goto out_err;
	}
	/*bcm5892_DMU_tset_settings(sc);*/

	/* Put EPHY in normal power state */ //Ram solution start
	val = __raw_readl(sc->bcm5892_phyctrl);
	val1 = ~(BCM5892_PHY_CTRL_PDB | BCM5892_PHY_CTRL_PDD | BCM5892_PHY_CTRL_PDP);
	val =  (val & val1);
	printk("bcm5892mac_open: val: %08x, val1 %08x\n", val, val1);
	__raw_writel(val, sc->bcm5892_phyctrl);  //sd
	printk("sd:in the open routine\n"); //Ram soultion end

	err = bcm5892_PHY_reset(sc, BCM5892_PHY_GIVEN);
	if (0xFFFF == err) {
		PDEBUG("ERROR: Could not Reset PHY\n");
		err = -1;
		goto out_err;
	}

	/*
	 * Probe PHY address
	 */
	/* new for kernel version 2.6.32.9. Added by sram */

	//Inlcuded 1 micro second delay for new EPhy initialization process
	printk("Before mdiobus_register: %08x, %08x\n", (unsigned int)sc, (unsigned int)&sc->mii_bus);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#ifdef USE_MII_BUS_POINTER
	err = mdiobus_register(sc->mii_bus);
#else
	err = mdiobus_register(&sc->mii_bus);
#endif
#else
	err = mdiobus_register(&sc->mii_bus);
#endif
	printk("After mdiobus_register: %08x,  %08x\n", (unsigned int)sc, (unsigned int)&sc->mii_bus);
	if (err) {
		PDEBUG("%s: unable to register MDIO bus\n", dev->name);
		goto out_unirq;
	}

	/*
	 * Attach to the PHY
	 */
	err = bcm5892_mii_probe(dev);
	if (err) {
		PDEBUG("ERROR: Could not probe PHY\n");
		goto out_unregister;
	}
	
	err = request_irq(dev->irq, &bcm5892mac_intr, IRQF_SHARED, dev->name, dev);
	if (err) {
		PDEBUG("%s: unable to get IRQ %d\n", dev->name,dev->irq);
		goto out_unregister;
	}

	/*
	 * Turn on the channel
	 */
	bcm5892mac_set_channel_state(sc, bcm5892_state_on);

	phy_start(sc->bcm5892phy_dev);

	napi_enable(&sc->napi);
	ssleep(WAIT_FOR_PHY_READY_DELAY);
	return 0;

out_unregister:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#ifdef USE_MII_BUS_POINTER
	mdiobus_unregister(sc->mii_bus);
#else
	mdiobus_unregister(&sc->mii_bus);
#endif
#else
	mdiobus_unregister(&sc->mii_bus);
#endif
out_unirq:
	free_irq(dev->irq, dev);
out_err:
	return err;
}

/**********************************************************************
 *  BCM5892MAC_TX_TIMEOUT(dev)
 *
 *  The transmission timeout callback.
 *
 *  Input parameters:
 *         dev: The network device context 
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/
static void
bcm5892mac_tx_timeout(struct net_device *dev)
{
	struct bcm5892mac_softc *sc = netdev_priv(dev);

	spin_lock_irq(&sc->bcm5892_lock);
	dev->trans_start = jiffies;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
       	sc->bcm5892_stats.tx_errors++;
#else
	dev->stats.tx_errors++;
#endif
	spin_unlock_irq(&sc->bcm5892_lock);
	PDEBUG("%s: Transmit timed out\n",dev->name);
}

/**********************************************************************
 *  BCM5892MAC_INIT(dev)
 *
 *  Initialize the Device
 *
 *  Input parameters:
 *
 *              dev -  Device Context
 *
 *  Return value:
 *		    0: Everything All Right
 *		not 0: ERROR 
 **********************************************************************/
static int
bcm5892mac_init( struct  platform_device *pldev
        )
{
        u64 ea_reg = (u64)0;
	u32 mac_reg,mac_reg0, mac_reg1;
        int i = 0;
        int err = 0;
        struct net_device *dev = NULL;
        struct bcm5892mac_softc *sc = NULL;
	int idx = 0;
        unsigned char eaddr[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};

	if (NULL == pldev) {
		PDEBUG("%s: Fatal Error: Invalid platform device pointer value\n", __FUNCTION__);
		return -EINVAL;
	}
	dev = dev_get_drvdata(&(pldev->dev));
	if (NULL == dev) {
		PDEBUG("%s: Fatal Error: Could not find the Network Device\n", __FUNCTION__);
		return -EINVAL;
	}
	sc = netdev_priv(dev);
	if (NULL == sc) {
		PDEBUG("%s: Fatal Error: Could not find the Device Context Pointer\n", __FUNCTION__);
		return -EINVAL;
	}

	/* Write all 1 to Multicast address filters */
	__raw_writel(0xFFFFFFFF, sc->bcm5892_macbase + OFFSET_MCADDRF0);
	__raw_writel(0xFFFFFFFF, sc->bcm5892_macbase + OFFSET_MCADDRF1);
	idx = pldev->id;

    sc->bcm5892_dev = dev;
	sc->sbe_idx = idx;	
	memset(&ea_reg, '\0', sizeof(u64));

	/*
	 * Read the ethernet address.  The firwmare may have left this programmed
	 * with the MAC address to use.
	 */
        mac_reg = __raw_readl(sc->bcm5892_macbase + OFFSET_MACADDR0);
        mac_reg0 = BYTE_SWAP(mac_reg);
        mac_reg = __raw_readl(sc->bcm5892_macbase + OFFSET_MACADDR1);
        mac_reg1 = BYTE_SWAP(mac_reg);

	/* If the MAC registers are non-zero, use the address they contain.
	   Otherwise, set them to the address held in eaddr[] */
	if (mac_reg0 || mac_reg1) {
		for (i = 0; i < 4; i++) {
			eaddr[i] = (uint8_t) (mac_reg0 & 0xFF);
			mac_reg0 >>= 8;
		}
		mac_reg1 >>= 0x10;
		for (i = 4; i < 6; i++) {
			eaddr[i] = (uint8_t)(mac_reg1 & 0xFF);
			mac_reg1 >>= 8;
		}
	}
	else {
		mac_reg0 = 0;
		for (i = 0; i < 4; i++)
			mac_reg0 = (mac_reg0 << 0x08) | eaddr[i];
	
		mac_reg1 = 0;
		for (i = 4; i < 6; i++)
			mac_reg1 = (mac_reg1 << 0x8) | eaddr[i];

	        __raw_writel(mac_reg0, sc->bcm5892_macbase + OFFSET_MACADDR0);
        	__raw_writel(mac_reg1, sc->bcm5892_macbase + OFFSET_MACADDR1);
	}

	printk(KERN_ERR "%s: MAC addr: %02x:%02x:%02x:%02x:%02x:%02x\n",
	       bcm5892mac_string,
	       eaddr[0], eaddr[1], eaddr[2], eaddr[3], eaddr[4], eaddr[5]);
	memcpy(dev->dev_addr, eaddr, ETHER_ADDR_LEN);

	/*************************************************************************/

        /*
         * Init packet size
         */
        sc->bcm5892_buffersize = ENET_PACKET_SIZE + SMP_CACHE_BYTES * 2 + ETHER_ALIGN;
	
	/*
	 * Initialize context (get pointers to registers and stuff), then
	 * allocate the memory for the descriptor tables.
	 */
	bcm5892mac_initctx(sc);

 	/*
	 * Set up Linux device callins
	 */

	spin_lock_init(&sc->bcm5892_lock);

 	dev->watchdog_timeo     = TX_TIMEOUT;
 	dev->irq                = pldev->resource[IDX_IORES_IRQ].start;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
	dev->netdev_ops = &bcm5892_netdev_ops;
#else
 	dev->open               = bcm5892mac_open;
	dev->hard_start_xmit    = bcm5892mac_start_tx;
	dev->stop               = bcm5892mac_close;
 	dev->do_ioctl           = bcm5892_mii_ioctl;
 	dev->tx_timeout         = bcm5892mac_tx_timeout;
    dev->get_stats          = bcm5892_get_stats;
 	dev->change_mtu         = bcm5892mac_change_mtu;
 	dev->set_rx_mode 	    = bcm5892mac_set_rx_mode;
	dev->set_mac_address 	= bcm5892mac_set_addr;
#endif

	netif_napi_add(dev, &sc->napi, bcm5892mac_poll, 16);

	SET_ETHTOOL_OPS(dev, &bcm5892_ethtool_ops);

 	err = register_netdev(dev);
 	if (err) {
         	PDEBUG("Unable to register netdev");
         	bcm5892mac_uninitctx(sc);
         	return err;
 	}
	PDEBUG("%s.%d: registered as %s\n", bcm5892mac_string, idx, dev->name);

 	/*
  	* Display Ethernet address (this is called during the config
  	* process so we need to finish off the config message that
  	* was being displayed)
  	*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#ifdef USE_MII_BUS_POINTER
 	sc->mii_bus->name        = bcm5892mac_mdio_string;
#else
 	sc->mii_bus.name        = bcm5892mac_mdio_string;
#endif
#else
 	sc->mii_bus.name        = bcm5892mac_mdio_string;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#ifdef USE_MII_BUS_POINTER
	snprintf(sc->mii_bus->id, MII_BUS_ID_SIZE, "%x", BCM5892_PHY_GIVEN);
#else
	snprintf(sc->mii_bus.id, MII_BUS_ID_SIZE, "%x", BCM5892_PHY_GIVEN);
#endif
#else
	snprintf(sc->mii_bus.id, MII_BUS_ID_SIZE, "%x", BCM5892_PHY_GIVEN);
#endif
#else
	sc->mii_bus->id = 2;
#endif
	sc->current_link   = -1;
	sc->current_speed  = bcm5892_speed_none;
	sc->current_duplex = bcm5892_duplex_none;
	sc->current_fc     = bcm5892_fc_disabled;
	sc->current_pause  = -1;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#ifdef USE_MII_BUS_POINTER
 	sc->mii_bus->priv        = sc;
 	sc->mii_bus->read        = bcm5892_mii_read;
 	sc->mii_bus->write       = bcm5892_mii_write;
 	sc->mii_bus->phy_mask	= ~(0x01 << (BCM5892_PHY_GIVEN));

	sc->mii_bus->irq = sc->phy_irq;
	for (i = 0; i < PHY_MAX_ADDR; ++i)
		sc->mii_bus->irq[i] = PHY_POLL;
 	sc->mii_bus->parent                 = &pldev->dev;
#else
 	sc->mii_bus.priv        = sc;
 	sc->mii_bus.read        = bcm5892_mii_read;
 	sc->mii_bus.write       = bcm5892_mii_write;
 	sc->mii_bus.phy_mask	= ~(0x01 << (BCM5892_PHY_GIVEN));

	sc->mii_bus.irq = sc->phy_irq;
	for (i = 0; i < PHY_MAX_ADDR; ++i)
		sc->mii_bus.irq[i] = PHY_POLL;
 	sc->mii_bus.parent                 = &pldev->dev;
#endif
#else
 	sc->mii_bus.priv        = sc;
 	sc->mii_bus.read        = bcm5892_mii_read;
 	sc->mii_bus.write       = bcm5892_mii_write;
 	sc->mii_bus.phy_mask	= ~(0x01 << (BCM5892_PHY_GIVEN));

	sc->mii_bus.irq = sc->phy_irq;
	for (i = 0; i < PHY_MAX_ADDR; ++i)
		sc->mii_bus.irq[i] = PHY_POLL;
 	sc->mii_bus.dev                 = &pldev->dev;
#endif
 	sc->mii_info.phy_id_mask        = 0x01 << (BCM5892_PHY_GIVEN  - 1);
 	sc->mii_info.reg_num_mask       = 0x1F;
 	sc->mii_info.phy_id        	= BCM5892_PHY_GIVEN;
 	sc->mii_info.dev                = dev; //sc->bcm5892_dev;
 	sc->mii_info.mdio_read          = bcm5892_mdio_read;
 	sc->mii_info.mdio_write         = bcm5892_mdio_write;
 	sc->mii_info.full_duplex       	= 1; /* is full duplex? */
 	sc->mii_info.force_media       	= 1; /* is autoneg. disabled? */
 	sc->mii_info.supports_gmii     	= 1; /* are GMII registers supported? */

	PDEBUG("Probe is completed - SUCCESS\n"); 
 	return err;
}


static int bcm5892mac_poll(struct napi_struct *napi, int budget)
{
	u32 mask;
	int work_done;
	struct bcm5892mac_softc *sc = container_of(napi, struct bcm5892mac_softc, napi);

	work_done = bcm5892dma_rx_process(sc, &sc->bcm5892_rxdma, budget, 1);
	bcm5892dma_tx_process(sc, &sc->bcm5892_txdma, 1);

	if (work_done < budget) {
#if 1
		napi_complete(napi);
#endif /* fix me */

		/* re-enable RX interrupts */
		mask = __raw_readl(sc->bcm5892_intrmask);
		__raw_writel(mask | BCM5892_INTR_RXF, sc->bcm5892_intrmask);
	}
	return work_done;
}


/*
  1. We have overflowed. Need to restart the mac FIFO controller via
  a soft reset of the the mac. Do the following:
  2.
  a) if ETH_CTRL[GRS = 0] then set GRS and wait for GRSC to be set.
  b) if [THLT != 0] then wait for THLT to be set.
  3. When these are set, the DMA has completed
  4. Soft reset the MAC. MACFG[SRST =1] then MACCFG[SRST = 0]
  5. Reenable the Rx DMA via  GRS = 0
  6. Reception from MAC should be completed.
  7. Clear GRSC and THLT  [raw]bits  via resetting them to 0
*/
static void bcm5892mac_rx_overflow(struct bcm5892mac_softc *sc)
{
	u32 intr_eth_val, intr_tmp_val;
	struct net_device *dev = sc->bcm5892_dev;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
       	sc->bcm5892_stats.rx_fifo_errors++;
#else
	dev->stats.rx_fifo_errors++;
#endif
	trace.intr = 0;
	if (!(__raw_readl(sc->bcm5892_ethctrl) & BCM5892_ETH_CTRL_GRS)) {
		trace.intr = 1;
		/* Still receiving, set GRS */
		intr_eth_val = __raw_readl(sc->bcm5892_ethctrl);
		intr_eth_val |= BCM5892_ETH_CTRL_GRS;
		__raw_writel(intr_eth_val, sc->bcm5892_ethctrl);

		/* Wait for GRSC */
		while (!(__raw_readl(sc->bcm5892_intr_raw) & BCM5892_INTR_RAW_GRSC))
			;
	}

	/* XXX - what happens if we lose the link here? */
	trace.intr = 2;
	if (!(__raw_readl(sc->bcm5892_ethctrl) & BCM5892_ETH_CTRL_GTS)) {
		trace.intr = 3;

		/* Still transmitting, wait for THLT */
		while (!(__raw_readl(sc->bcm5892_intr_raw) & BCM5892_INTR_RAW_THLT))
			;
	}
	/* RX and TX DMA both halted */
	trace.intr = 4;

	/* issue soft reset */
	intr_tmp_val = __raw_readl(sc->bcm5892_mac_cfg);
	intr_tmp_val |= BCM5892_MAC_CFG_SRST;
	__raw_writel(intr_tmp_val, sc->bcm5892_mac_cfg);

	/* clear GRS and GTS in eth_ctrl */
	intr_eth_val = __raw_readl(sc->bcm5892_ethctrl);
	intr_eth_val &= ~(BCM5892_ETH_CTRL_GRS | BCM5892_ETH_CTRL_GTS);
	__raw_writel(intr_eth_val, sc->bcm5892_ethctrl);

	/* release soft reset */
	intr_tmp_val = __raw_readl(sc->bcm5892_mac_cfg);
	intr_tmp_val &= ~(BCM5892_MAC_CFG_SRST);
	__raw_writel(intr_tmp_val, sc->bcm5892_mac_cfg);

	/* Clear GRSC and THLT [INTR_RAW] bits */
	intr_tmp_val = BCM5892_INTR_CLR_GRSC;
	intr_tmp_val |= BCM5892_INTR_CLR_RHLT;
	intr_tmp_val |= BCM5892_INTR_CLR_GTSC;
	intr_tmp_val |= BCM5892_INTR_CLR_THLT;
	intr_tmp_val |= BCM5892_INTR_CLR_ROV;
	__raw_writel(intr_tmp_val, sc->bcm5892_intr_clr);
	__raw_writel(0, sc->bcm5892_intr_clr);
}

/**********************************************************************
 *  BCM5892MAC_INTR(irq, dev_instance, pt_regs)
 *
 *  Interrupt handler for MAC interrupts
 *
 *  Input parameters:
 *         irq - The Interrupt Number dedicated to us.
 *         dev_nstance - The Current Device Instance (Device Context)
 *	   pt_regs - 
 *
 *  Return value:
 *	IRQ_RETVAL(1): Interrupt handling is Succesful
 *	IRQ_RETVAL(0): Interrupt handling is Not Succesful
 *         
 **********************************************************************/
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,18) 
static irqreturn_t 
bcm5892mac_intr(
	int irq,
	void *dev_instance,
	struct pt_regs* regs
	)
#else
static irqreturn_t 
bcm5892mac_intr(
	int irq,
	void *dev_instance
	)
#endif
{
	struct net_device *dev = (struct net_device *) dev_instance;
	struct bcm5892mac_softc *sc = netdev_priv(dev);
	u32 intr_current = 0;
	int handled = 0;
	u32 intr_raw, intrmask, maskedintr;

	intr_raw   = __raw_readl(sc->bcm5892_intr_raw);
	intrmask   = __raw_readl(sc->bcm5892_intrmask);
	maskedintr = __raw_readl(sc->bcm5892_maskedintr);

	/* CAUTION: Masked Intr is a big "misnomer" in the Eth SubSystem literature */
	intr_current = maskedintr; // Find out which interrupt has occured

	/* Clear all pending interrupts */
	__raw_writel(intr_current, sc->bcm5892_intr_clr);
	__raw_writel(0, sc->bcm5892_intr_clr);

	/* debugging trace */
	++trace.intr_count;
	trace.intr_current = intr_current;
	trace.intr_raw = intr_raw;
	handled = 1;

	/* Detect overflow */
	if (intr_raw & BCM5892_INTR_RAW_ROV)
		sc->rx_overflow = 1;
#if 1
	if (napi_schedule_prep(&sc->napi)) {
		/* Disable RX interrupts */
		u32 imask = __raw_readl(sc->bcm5892_intrmask);
		__raw_writel(imask & ~BCM5892_INTR_RXF, sc->bcm5892_intrmask);

		++trace.rx_poll;

		/* Schedule the polling routine */
		__napi_schedule(&sc->napi);
		/* Depend on the exit from poll to reenable intr */
	}
	else {
		++trace.rx_in_isr;

		/* may leave some packets behind */
		bcm5892dma_rx_process(sc, &sc->bcm5892_rxdma,
				      BCM5892_MAX_RXDESCR, 0);
		bcm5892dma_tx_process(sc, &sc->bcm5892_txdma, 0);
	}
#endif /* fix me */

	return IRQ_RETVAL(handled);
}
/**********************************************************************
 *  BCM5892MAC_PROBE(device)
 *
 *  The Platform Driver Probe function.
 *
 *  Input parameters:
 *         device: The Device Context
 *
 *  Return value:
 *		    0: Driver Probe is Succesful
 *		not 0: ERROR
 **********************************************************************/
static int __init bcm5892mac_probe(struct platform_device* pldev)
{
	struct net_device *dev		= NULL;
	struct bcm5892mac_softc *sc	= NULL;
	void __iomem *macbase		= NULL;
	void __iomem *mmibase		= NULL;
	struct resource *res		= NULL;
	struct resource *mmi_res	= NULL;
	int err				= 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	struct mii_bus *miib = NULL;
#endif

	/*Validation of platform device structure*/
	if (!pldev) {
        	PDEBUG("WRONG INPUT\nplatfrom_device ppointer should not be NULL.\n");
       	 	return -EINVAL;
	}

	res = bcm5892_platform_get_resource(pldev, IORESOURCE_MEM, IDX_IORES_MAC_MEM);
	if (NULL == res) {
		PDEBUG("ERROR: Could not get Platform Resource MAC Register Memory Resource\n");
		return -ENOMEM;
	}

	mmi_res = bcm5892_platform_get_resource(pldev, IORESOURCE_MEM, IDX_IORES_MMI_MEM);
	if (NULL == mmi_res) {
		PDEBUG("ERROR: Could not get Platform Resource MMI Register Memory Resource\n");
		return -ENOMEM;
	}

	if (!request_mem_region(res->start, (res->end - res->start + 1), pldev->name)) {
		PDEBUG("ERROR: Could not request mem region. In file %s, LN:%d\n",
		       __FILE__, __LINE__);
		return -ENOMEM;
	}

	if (!request_mem_region(mmi_res->start, (mmi_res->end - mmi_res->start + 1), pldev->name)) {
		PDEBUG("ERROR: Could not request mem region, for MMI Register Memory Locations. In file %s, LN:%d\n", __FILE__, __LINE__);
		goto Release_MAC;
	}

	macbase = ioremap_nocache(res->start, res->end - res->start + 1);
	if (NULL == macbase) {
		PDEBUG("Unable to map device registers\n");
		err = -ENOMEM;
		goto Release_MMI;
	}

	mmibase = ioremap_nocache(mmi_res->start, mmi_res->end - mmi_res->start + 1);
	if (NULL == mmibase) {
		PDEBUG("Unable to map device registers\n");
		err = -ENOMEM;
		goto Unmap_MAC;
	}
	dev = alloc_etherdev(sizeof(struct bcm5892mac_softc));
	if (!dev) {
		PDEBUG("Unable to allocate etherdev\n");
		err = -ENOMEM;
		goto Exit;
	}

	pldev->id = res->start;
	dev_set_drvdata(&(pldev->dev), dev);
	SET_NETDEV_DEV(dev, (&pldev->dev));
	sc = netdev_priv(dev);
	if (NULL == sc) {
		PDEBUG("FATAL ERROR: Could not find a pointer to Device Context\n");
		err = -1;
		goto Exit;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	miib = mdiobus_alloc();
#ifdef USE_MII_BUS_POINTER
	sc->mii_bus = miib;
	if (sc->mii_bus == NULL)
	{
		PDEBUG("Unable to allocate etherdev\n");
		err = -ENOMEM;
		goto Exit;
	}
#else
	sc->mii_bus = *miib;
#endif
#endif
	sc->bcm5892_macbase = macbase;
	sc->bcm5892_mmibase = mmibase;

	/* Get the MAC address from firmware and program 
         * into the mac address register.
	 */
//	err = bcm5892_get_ethaddr(sc);
#if 0
	if (0 == err) {
		PDEBUG("Firmware has not provided MAC ADDRESS. Will be using hardcoded address.\n");
	}
#endif

	err = bcm5892mac_init(pldev);
	if (err) {
		free_netdev(dev);
		dev = NULL;
		PDEBUG("Could not initialize the MAC\n");
		goto Exit;
	}

	eth_mac_proc_create(dev);	

	printk(KERN_ERR "%s: driver trace at %p\n", bcm5892mac_string, &trace);

	return 0;
Exit:
	iounmap(mmibase);
Unmap_MAC:
	iounmap(macbase);
Release_MMI:
	release_mem_region(mmi_res->start, (mmi_res->end - mmi_res->start + 1));
Release_MAC:
	release_mem_region(res->start, (res->end - res->start + 1));
	return err;
}


/**********************************************************************
 *  BCM5892MAC_REMOVE(device)
 *
 *  The Removal of Platform Device, and un-initialize the previously
 *  added MAC, and it's MEM Regions and Resources.
 *
 *  Input parameters:
 *         device: The Device Context
 *
 *  Return value:
 *		    0: Driver Entry is Succesfull
 **********************************************************************/
static int __exit bcm5892mac_remove(struct platform_device *pldev)
{
	struct net_device *dev = platform_get_drvdata(pldev);
	struct bcm5892mac_softc *sc = netdev_priv(dev);
	int retVal = 0;
	
	iounmap(sc->bcm5892_mmibase);
	iounmap(sc->bcm5892_macbase);
	unregister_netdev(dev);
	bcm5892mac_uninitctx(sc);
	release_mem_region(pldev->resource[IDX_IORES_MAC_MEM].start,
			   (pldev->resource[IDX_IORES_MAC_MEM].end
			    - pldev->resource[IDX_IORES_MAC_MEM].start + 1));
	release_mem_region(pldev->resource[IDX_IORES_MMI_MEM].start,
				 (pldev->resource[IDX_IORES_MMI_MEM].end
				  - pldev->resource[IDX_IORES_MMI_MEM].start + 1));
	free_netdev(dev);

	return retVal;
}

#ifdef CONFIG_PM
static int bcm5892mac_drv_suspend(struct platform_device *pdev, pm_message_t state)
{
#if 0
	struct net_device *dev = platform_get_drvdata(pdev);

	if (netif_running(dev))
		netif_device_detach(dev);
#endif
	int ret;
	char *filename = "/usr/sbin/ifdown";
	char *argv[] = {filename, "eth0", NULL};
 	char *envp[] = {"HOME=/",
 					 "TERM=linux",
 					 "PATH=/sbin:/usr/sbin:/bin:/usr/bin",
 					 NULL};

 	ret = kernel_execve(filename, argv, envp);
	return 0;
}

static int bcm5892mac_drv_resume(struct platform_device *pdev)
{
#if 0
	struct net_device *dev = platform_get_drvdata(pdev);

	if (netif_running(dev)) {
		netif_device_attach(dev);
	}
#endif
	int ret;
	char *filename = "/usr/sbin/ifup";
	char *argv[] = {filename, "eth0", NULL};
 	char *envp[] = {"HOME=/",
 					 "TERM=linux",
 					 "PATH=/sbin:/usr/sbin:/bin:/usr/bin",
 					 NULL};

	ret = kernel_execve(filename, argv, envp);
	return 0;
}
#else
#define bcm5892mac_drv_suspend NULL
#define bcm5892mac_drv_resume NULL
#endif

/**********************************************************************
 * This structure defines the methods to be called by a bus driver 
 * during the lifecycle of a device on that bus.
**********************************************************************/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)

	static struct platform_driver bcm5892mac_driver = 
	{
        	.probe = bcm5892mac_probe,
        	.remove = __exit_p(bcm5892mac_remove),
		.suspend = bcm5892mac_drv_suspend,
		.resume = bcm5892mac_drv_resume,
        	.driver =
        	{
                	.name = bcm5892mac_string,
        	},
	};
#else

	static struct device_driver bcm5892mac_driver =
	{
        	.name = bcm5892mac_string,
		.bus = &platform_bus_type,
		.probe = bcm5892mac_probe,
	};
#endif /* #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16) */


/**********************************************************************
 *   This function calls the device structure. 
 *   Input Parameter:	
 *		dev - pointer to the struct device
 **********************************************************************/

static void bcm5892mac_release (struct device *dev) {}


/**********************************************************************
 *  This structure defines the methods to be called by a platform device
 *  during the lifecycle of a device
**********************************************************************/

static struct platform_device bcm5892mac_pdev = {

        .name           = bcm5892mac_string,
        .id             = 0,
        .dev =  {
		.release        = bcm5892mac_release,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		.init_name = bcm5892mac_string,
#endif

                },

        .resource               = bcm5892mac_resources,
        .num_resources          = ARRAY_SIZE(bcm5892mac_resources),

};

/**********************************************************************
 *  BCM5892MAC_INIT_MODULE(VOID)
 *  BCM5892MAC_INIT_MODULE(VOID)
 *
 *  The Driver Entry Function
 *
 *  Input parameters:
 *         None
 *
 *  Return value:
 *		    0: Driver Entry is Succesful
 *		not 0: ERROR
 **********************************************************************/
static int __init
bcm5892mac_init_module(void)
{
	int err = -1;

	PDEBUGG("Entering Module Init");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)
	err = bcm5892_platform_driver_register(&bcm5892mac_driver);
#else
	err = driver_register(&bcm5892mac_driver);
#endif
	if (err) {
		PDEBUGG("ERROR module_init, could not bcm5892_platform_driver_register\n");
		PDEBUGG("Error Code = 0x%08x\n", err);
		return err;
	}

	err = bcm5892_platform_device_register(&bcm5892mac_pdev);
	if (err) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)
		bcm5892_platform_driver_unregister(&bcm5892mac_driver);
#else
		driver_unregister(&bcm5892mac_driver);
#endif
		PDEBUGG("ERROR IN module_init. Could not do bcm5892_platform_device_register\n");
		PDEBUGG("Error Code = 0x%08x\n", err);
		return err;
	}
	PDEBUGG("Exiting Module Init");
	return err;
}

/**********************************************************************
 *  BCM5892MAC_CLEANUP_MODULE(VOID)
 *
 *  The Driver Exit Function
 *
 *  Input parameters:
 *         None
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/
static void __exit
bcm5892mac_cleanup_module(void)
{
	PDEBUGG("Entering Module Exit");
	/*Unregister the Platform Device*/
	bcm5892_platform_device_unregister(&bcm5892mac_pdev);

	/* Unregister the driver*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)
	bcm5892_platform_driver_unregister(&bcm5892mac_driver);
#else
	driver_unregister(&bcm5892mac_driver);
#endif
	PDEBUGG("Exiting Module Exit");

	eth_mac_proc_remove(); 	
	return;
}

/**********************************************************************
 *  BCM5892MAC_GET_STATS(device)
 *
 *  The function passes the collected device statistics to "ifconfig"
 *  command issued at linux command prompt.
 *
 *  Input parameters:
 *         device: Net Device Context
 *
 *  Return value:
 *         devive stats : Net Device Statistics
 **********************************************************************/
static struct net_device_stats *bcm5892_get_stats(struct net_device *dev)
{
#ifdef BCM5892_DUMP_STAT_REGS
        bcm5892_dump_stat_regs(sc);
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
        return(&sc->bcm5892_stats);
#else
        return(&dev->stats);
#endif

}

static char* bcm5892_eth_proc_root="bcm5892_eth";

static int get_debug_level(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	unsigned int len=0;
	len += sprintf(page+len, "\n\n## Current Debug Level = 0x%x ##\n\n",cur_dbg_lvl);
	*eof = 1;
	return len;
}

static int set_debug_level(struct file *file, const char *buffer, unsigned long count, void *data)
{
	unsigned int 	len=1;
	unsigned char 	debug_buffer[2];
	int		dbg_lvl =0;

	//printk(KERN_INFO "count %x ## \n\n",(unsigned int) count);
	if (count != 2)
	{
		printk(KERN_ERR "Please pass (one:1) digit debug level value only \n");
		return -EINVAL;
	}

	// Last buffer byte will be LF or CR only
	if(copy_from_user(&debug_buffer[0], buffer, len)) 
	{
		printk(KERN_ERR "Problem in copying invalid user buffer \n");
		return -EFAULT;
	}

	debug_buffer[len]='\0'; // Only one byte value is available now
	if ( sscanf(debug_buffer,"%d",&dbg_lvl) != 1)
	{
		printk(KERN_ERR "\n##Invalid value :%s: is passed ##\n",debug_buffer);
		return -EINVAL;
	}
	if (!((dbg_lvl >=MIN_DEBUG_LEVEL) && (dbg_lvl <= MAX_DEBUG_LEVEL)))
	{
		printk(KERN_ERR "\n##Passed value :%d: is not in valid range %d-%d \n",dbg_lvl,MIN_DEBUG_LEVEL,MAX_DEBUG_LEVEL);
		return -EINVAL;
	}
	printk(KERN_INFO "\n##set_debug_level(): Old Debug Level 0x%x ##\n", cur_dbg_lvl);
	cur_dbg_lvl = dbg_lvl;
	printk(KERN_INFO "\n##set_debug_level(): New Debug Level 0x%x ##\n", cur_dbg_lvl);
	return count;
}

static int get_mac_cfg(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	volatile u32  mac_val=(u32)0;
	unsigned int len=0;

        len += sprintf(page+len,"\n ## Dump of ETHERNET MAC CONFIG RELATED REGISTERS offset %x count %x ##\n\n",
			(unsigned int)off,count);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACBP));
	len += sprintf(page+len, "MAC_BP register value =0x%08x\n",mac_val);
	
	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACCFG));
	len += sprintf(page+len, "MAC_CGF register value =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACADDR0));
	len += sprintf(page+len, "MAC_ADDR_LOW value =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACADDR1));
	len += sprintf(page+len, "MAC_ADDR_HIGH value =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MAXFRM));
	len += sprintf(page+len, "MAC_MAX_FRAME_LEN value =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACPQ));
	len += sprintf(page+len, "MAC_ETH_PAUSE_QUANT value =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACRXFE));
	len += sprintf(page+len, "MAC_RxFIFO ENTRY WMARK  value =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACRXFF));
	len += sprintf(page+len, "MAC_RxFIFO FRAME VALID value =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACTXFE));
	len += sprintf(page+len, "MAC_TxFIFO ENTRY WMARK value =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACTXFF));
	len += sprintf(page+len, "MAC_TxFIFO FRAME VALID THRESHOLD value =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_MACMODE));
	len += sprintf(page+len, "MAC_MODE STATUS  value =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXIPG));
	len += sprintf(page+len, "Tx MIN IPG LEN value =0x%08x\n",mac_val);
	
	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXPCTRL));
	len += sprintf(page+len, "Tx Pause Ctrl value =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXFIFOF));
	len += sprintf(page+len, "Tx FIFO Flush value =0x%08x\n",mac_val);
	
	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXFIFOSTAT));
	len += sprintf(page+len, "Rx FIFO Stat value =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXFIFOSTAT));
	len += sprintf(page+len, "Tx FIFO Stat value =0x%08x\n",mac_val);

        len += sprintf(page+len, "\n ## End of ETHERNET MAC CONFIG RELATED REGISTERS Len 0x%x##\n\n", len);
	*eof = 1;
 	return len;
}

static int get_miib_ctr(char *page, char **start, off_t off, int count,
			int *eof, void *data)
{
	volatile u32  mac_val=(u32)0;

	unsigned int len=0;

        len += sprintf(page+len,"\n ## Dump of ETHERNET MIIB CONTROL RELATED REGISTERS offset %x count %x ##\n\n",
			(unsigned int)off,count);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXOCTGOOD));
	len += sprintf(page+len, "Total Transmitted Octets in Good Packets =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXFRMGOOD));
	len += sprintf(page+len, "Total Transmitted Octets in Good Frames =0x%08x\n",mac_val);
 
	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXOCTTOTAL));
	len += sprintf(page+len, "Total of All Transmitted Octets =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXFRMTOTAL));
	len += sprintf(page+len, "Total of Transmitted Frames =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXBCASTGOOD));
	len += sprintf(page+len, "Total of Good Broadcast Frames Transmitteds =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXMCASTGOOD));
	len += sprintf(page+len, "Total of Good Multicast Frames Transmitted =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TX64));
	len += sprintf(page+len, "64B Length Frames Transmitted =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TX65_127));
	len += sprintf(page+len, "65B--127B Length Frames Transmitted =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TX128_255));
	len += sprintf(page+len, "128B--255B Length Frames Transmitted =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TX256_511));
	len += sprintf(page+len, "256B--511B Length Frames Transmitted =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TX512_1023));
	len += sprintf(page+len, "512B--1023B Length Frames Transmitted =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TX1024_MAX));
	len += sprintf(page+len, "1024B--MAXFRM Length Frames Transmitted =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXJABBER));
	len += sprintf(page+len, "MAXFRM with Bad CRC Transmitted =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXJUMBO));
	len += sprintf(page+len, "MAXFRM with Good CRC Transmitted  =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXFRAG));
	len += sprintf(page+len, "Frames < 64B with Bad CRC Transmitted  =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXUNDERRUN));
	len += sprintf(page+len, "Frames truncated due to FIFO underrun Transmitted  =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXCOLTOTAL));
	len += sprintf(page+len, "Collisions Seen by Transmitter  =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TX1COL));
	len += sprintf(page+len, "Frames Transmitted with only 1 Collision  =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXMCOL));
	len += sprintf(page+len, "Frames Transmitted with > 1 Collision  =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXEXCOL));
	len += sprintf(page+len, "Frames Aborted after Excessive [16] Collisions  =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXLATE));
	len += sprintf(page+len, "Frames that Experienced Late Collision =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXDEFER));
	len += sprintf(page+len, "Frame Transmission Delayed due to Busy Network =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXNOCRS));
	len += sprintf(page+len, "Loss of CRS when transmitting =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TXPAUSE));
	len += sprintf(page+len, "Total Pause Frames Transmitted =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXOCTGOOD));
	len += sprintf(page+len, "Total Received Octets in Good Packets =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXFRMGOOD));
	len += sprintf(page+len, "Total of Successfully Received Frames =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXOCTTOTAL));
	len += sprintf(page+len, "Total of All Received Octets =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXFRMTOTAL));
	len += sprintf(page+len, "Total of Received Frames =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXBCASTGOOD));
	len += sprintf(page+len, "Total of Good Broadcast Frames Received =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXMCASTGOOD));
	len += sprintf(page+len, "Total of Good Multicast Frames Received=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RX64));
	len += sprintf(page+len, "64B Length Frames Received=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RX65_127));
	len += sprintf(page+len, "65B--127B Length Frames Received=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RX128_255));
	len += sprintf(page+len, "128B--255B Length Frames Received=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RX256_511));
	len += sprintf(page+len, "256B--511B Length Frames Received=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RX512_1023));
	len += sprintf(page+len, "512B--1023B Length Frames Received=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RX1024_MAX));
	len += sprintf(page+len, "1024B--MAXFRM Length Frames Received=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXJABBER));
	len += sprintf(page+len, "Frames > MAXFRM with Bad CRC Received=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXJUMBO));
	len += sprintf(page+len, "Frames > MAXFRM with Good CRC Received=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXFRAG));
	len += sprintf(page+len, "Frames < 64B with Bad CRC Received=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXOVERRUN));
	len += sprintf(page+len, "Frames truncated due to FIFO overrun=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXCRCALIGN));
	len += sprintf(page+len, "Received Frames with Bad CRC or Alignment=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXUSIZE));
	len += sprintf(page+len, "Frames < 64B with Good CRC Received=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXCRC));
	len += sprintf(page+len, "Received Frames with Bad CRC but Even=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXALIGN));
	len += sprintf(page+len, "Received Frames with Bad CRC and Odd=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXCDERR));
	len += sprintf(page+len, "Receive Frames with Code Error Events=0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RXPAUSEFM));
	len += sprintf(page+len, "Control Frames (Non-Pause) Received=0x%08x\n",mac_val);

        len += sprintf(page+len, "\n ## End of ETHERNET MIIB CONTROL RELATED REGISTERS Len 0x%x##\n\n", len);
	*eof = 1;
 	return len;
}

static int get_eth_ctrl(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	volatile u32  mac_val=(u32)0;
	unsigned int len=0;

        len += sprintf(page+len,"\n ## Dump of ETHERNET CONTROL RELATED REGISTERS offset %x count %x ##\n\n",
			(unsigned int)off,count);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_ETH_CTRL));
	len += sprintf(page+len, "ETH_CTRL =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_INTR_MASK));
	len += sprintf(page+len, "INTERRUPT_MASK =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_INTR));
	len += sprintf(page+len, "MASKED_INTERRUPT =0x%08x\n",mac_val);

	mac_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_INTR_RAW));
	len += sprintf(page+len, "RAW_INTERRUPT =0x%08x\n",mac_val);

        len += sprintf(page+len, "\n ## End of ETHERNET CONTROL RELATED REGISTERS Len 0x%x##\n\n", len);
	*eof = 1;
 	return len;
}

static int get_mii_reg(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	unsigned int len=0;
        unsigned int index=0;
        volatile u32  mii_reg_value=(u32)0;
        unsigned char mii_reg_name[][80]={
                "MII Control Register",
                "MII Status register",
                "PHY ID High",
                "PHY ID Low",
                "Auto-Negotiation Advertisement Register",
                "Link Partner Ability Register",
                "Auto-Negotiation Expansion Register",
                "Next Page Register",
                "Link Partner Next Page",
                "Reserved",
                "Reserved",
                "Reserved",
                "Reserved",
                "Reserved",
                "Reserved",
                "Reserved",
                "100Base-X Auxiliary Control Register",
                "100Base-X Auxiliary Status Register",
                "100Base-X Receive Error Counter",
                "100Base-X False Carrier Counter",
                "100Base-X Disconnect Counter",
                "Reserved",
                "Reserved",
                "PTEST (Reserved)",
                "Auxiliary Control/Status Register",
                "Auxiliary Status Summary Register",
                "Interrupt",
                "Auxiliary Mode 2 Register",
                "10Base-T Auxiliary Error and General Status Register",
                "Auxiliary Mode Register",
                "Auxiliary Multi PHY Register",
                "Broadcom Test Register"
        };
	struct net_device *dev = data;
        struct bcm5892mac_softc *sc = netdev_priv(dev);
	
        len += sprintf(page+len,"\n## Dump of MII REGISTERS Offset 0x%x Count 0x%x ##\n\n",
			(unsigned int)off,count);
        for ( index=0; index < 32 ; index++)
        {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#ifdef USE_MII_BUS_POINTER
                mii_reg_value =bcm5892_mii_read(sc->mii_bus, (int)(2), index);
#else
                mii_reg_value =bcm5892_mii_read(&sc->mii_bus, (int)(2), index);
#endif
#else
                mii_reg_value =bcm5892_mii_read(&sc->mii_bus, (int)(2), index);
#endif
                len += sprintf(page+len, "0x%x:%s   =   0x%x \n",
			       index,mii_reg_name[index],mii_reg_value);
        }
        len += sprintf(page+len, "\n ## End of MII REGISTERS Len 0x%x ##\n\n",len);
	*eof = 1;
	return len;
}

// **********************************************
// Transmit Related proc implementation
// **********************************************

static void *tx_rng_seq_start(struct seq_file *s, loff_t *pos)
{
	static unsigned long counter = 0;
	int  tx_rng_val;
	int * val;

	/* beginning a new sequence ? */	
	if ( *pos == 0 )
	{	
		seq_printf(s, "\n## Dump of ETHERNET TRANSMIT RING RELATED REGISTERS %d %d \n",(int)s->count,(int) s->size);

		tx_rng_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TBASE));
		val = (int*)phys_to_virt(tx_rng_val);
		seq_printf(s, " TBASE:- Addr = 0x%08x val =0x%08x\n",tx_rng_val,*val);	

		seq_printf(s, " Dumping TX Ring  Address and Value \n");
		/* yes => return a non null value to begin the sequence */
		return &counter;
	}
	else
	{
		/* no => it's the end of the sequence, return end to stop reading */
		counter = 0;
		*pos = 0;
		return NULL;
	}
}

static void *tx_rng_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	unsigned long *tmp_counter = (unsigned long *)v;
	(*tmp_counter)++;
	(*pos)++;
	return NULL;
	//return tmp_counter;
}

static void tx_rng_seq_stop(struct seq_file *s, void *v)
{
	/* nothing to do, we use a static value in start() */
	int  tx_rng_val;
	int * val;

	tx_rng_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TBDPTR));
	val = (int*)phys_to_virt(tx_rng_val);
	seq_printf(s, " TBDPTR:-  = 0x%08x val = 0x%08x\n",tx_rng_val,*val);	
	
	tx_rng_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TSWPTR));
	val = (int*)phys_to_virt(tx_rng_val);
	seq_printf(s, " TSWPTR:-  = 0x%08x val = 0x%08x\n",tx_rng_val,*val);	

	tx_rng_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TBCFG));
	seq_printf(s, " TBCFG:-   = 0x%08x\n",tx_rng_val);	
	seq_printf(s, "\n## End of ETHERNET TRANSMIT RING RELATED REGISTERS %d %d \n",(int)s->count,(int) s->size);
}

static int tx_rng_seq_show(struct seq_file *s, void *v)
{
	//unsigned long  *spos = (unsigned long *) v;
	int i;
	int  tx_rng_val;
	volatile int * val;

	tx_rng_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_TBASE));

	for (i=0; i<256;i++)
	{
		val = (int*)phys_to_virt(tx_rng_val);
		seq_printf(s, " Offset %03d Addr = 0x%08x val1 = 0x%08x val2 = 0x%08x\n", 
				i, tx_rng_val, *(val), *(val+1) );
		// %d:%d :%d:\n", i,(int)s->count,(int)s->size,(int)*spos);
		tx_rng_val = (tx_rng_val+8);
	}
	return 0;
}


static int tx_rng_open(struct inode *inode, struct file *file)
{
	int status;
	size_t size = (4096 * 5);  // 4K space is enough for tx ring output
	char *buf = kmalloc(size, GFP_KERNEL);
	struct seq_file *m;
	if (!buf)
		return -ENOMEM;
	status = seq_open(file, &tx_rng_seq_ops);
	if (status == 0)
	{
		m = file->private_data;
		m->buf = buf;
		m->size = size;
	}
	return status;
}

// **********************************************
// Receive Related proc implementation
// **********************************************

static int rx_rng_seq_show(struct seq_file *s, void *v)
{
        int i ;
        int  rx_rng_val;
        volatile int * val;

	seq_printf(s, "\n## Dump of ETHERNET RECEIVE RING RELATED REGISTERS %d %d \n",(int)s->count,(int) s->size);
        rx_rng_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RBASE));
        val = (int*)phys_to_virt(rx_rng_val);
        seq_printf(s, " RBASE:- Addr = 0x%08x val =0x%08x\n",rx_rng_val,*val);
        seq_printf(s, " Dumping RX Ring  Address and Value \n");

        for (i=0;i<256;i++)
        {
                val = (int*)phys_to_virt(rx_rng_val);
		seq_printf(s, " Offset %03d Addr = 0x%08x val1 = 0x%08x val2 = 0x%08x\n", 
				i, rx_rng_val, *(val), *(val+1) );
                rx_rng_val = (rx_rng_val+8);
        }
        rx_rng_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RBDPTR));
        val = (int*)phys_to_virt(rx_rng_val);
        seq_printf(s, " RBDPTR:-  = 0x%08x val = 0x%08x\n",rx_rng_val,*val);

        rx_rng_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RSWPTR));
        val = (int*)phys_to_virt(rx_rng_val);
        seq_printf(s, " RSWPTR:-  = 0x%08x val = 0x%08x\n",rx_rng_val, *val);

        rx_rng_val = __raw_readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + OFFSET_RBCFG));
        seq_printf(s, " RBCFG:-   = 0x%08x\n",rx_rng_val);

	seq_printf(s, "\n## End of ETHERNET RECEIVE RING RELATED REGISTERS %d %d \n",(int)s->count,(int) s->size);
	return 0;
}

static int rx_rng_open(struct inode *inode, struct file *file)
{
        int status;
        size_t size = (4096 * 5);  // 4K space is enough for tx ring output
        char *buf = kmalloc(size, GFP_KERNEL);
        struct seq_file *m;
        if (!buf)
                return -ENOMEM;
        status = single_open(file, rx_rng_seq_show , NULL);
        if (status == 0)
        {
                m = file->private_data;
                m->buf = buf;
                m->size = size;
        }
        return status;
}

static int eth_mac_proc_create(struct net_device *dev )
{
	struct proc_dir_entry *dent, *ent;
	dent =	proc_mkdir(bcm5892_eth_proc_root,bcm5892_eth_root_dir);	
	if (dent) {
	 	ent = create_proc_entry("debug_level",S_IFREG|S_IRUGO, dent); 
		if (ent) {
			ent->read_proc  = get_debug_level;
			ent->write_proc = set_debug_level;
		}
	 	ent = create_proc_entry("mac_cfg",S_IFREG|S_IRUGO, dent); 
		if (ent) {
			ent->read_proc = get_mac_cfg;
		}
	 	ent = create_proc_entry("miib_ctr",S_IFREG|S_IRUGO, dent); 
		if (ent) {
			ent->read_proc = get_miib_ctr;
		}
		/* ******************************************************************************* */
		/* Output is more than 1 page(i.e. 4K=4096 bytes so special processing is required */
		/* ******************************************************************************* */
	 	ent = create_proc_entry("tx_rng",S_IFREG|S_IRUGO, dent); 
		if (ent) {
			ent->proc_fops = &tx_rng_ops;
		}
	 	ent = create_proc_entry("rx_rng",S_IFREG|S_IRUGO, dent); 
		if (ent) {
			ent->proc_fops = &rx_rng_ops;
		}
	 	ent = create_proc_entry("eth_ctrl",S_IFREG|S_IRUGO, dent); 
		if (ent) {
			ent->read_proc = get_eth_ctrl;
		}
	 	ent = create_proc_entry("mii_reg",S_IFREG|S_IRUGO, dent); 
		if (ent) {
			ent->read_proc = get_mii_reg;
			ent->data = dev;
		}
	}	
	return 0; 
}

static void eth_mac_proc_remove(void)
{
	remove_proc_entry(bcm5892_eth_proc_root,NULL);
	
//	vfree(cfg_str);
} 

/**********************************************************************
 *  The Kernel Specific function definitions _End
 **********************************************************************/


module_init(bcm5892mac_init_module);
module_exit(bcm5892mac_cleanup_module);

MODULE_DESCRIPTION("Broadcom 5892 Ethernet Driver");
MODULE_LICENSE("GPL");

