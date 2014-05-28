/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/mii.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/proc_fs.h>
#include <linux/kfifo.h>
#include <linux/sysctl.h>
#include <linux/reboot.h>

#include <linux/broadcom/bcm_sysctl.h>

#define ENABLE_NET_KNLLOG 0
#if ENABLE_NET_KNLLOG
#include <linux/broadcom/knllog.h>
#define NET_KNLLOG(fmt, args...) KNLLOG(fmt, ## args)
#else
#define NET_KNLLOG(fmt, args...)
#endif

#include <mach/csp/ethHw_dma.h>
#include <mach/csp/chipcHw_inline.h>
#include <mach/csp/cap.h>

#include <mach/irqs.h>
#include <mach/dma.h>
#include <mach/pm.h>

#include <asm/dma.h>

#include <csp/intcHw.h>
#include <csp/ethHw.h>
#include <csp/ethHw_mii.h>

/*--------------------------------------------------------------------------*/

/* Driver configuration (compile-time) */

/* Number of Rx descriptors */
#define DMA_RX_DESC_NUM_DEFAULT  64

/* 802.3as defines the max packet size to be 2000bytes and is rounded here to
*  a nice boundary
*/
#define DMA_RX_BUF_LEN           2048

/* At least 2*350 Tx descriptors are required (determined empirically) to
*  sustain ping attack.  Normally, DoS will be enabled, but this will handle
*  the worst possible case.
*/
#define DMA_TX_DESC_NUM_DEFAULT  (16 * DMA_RX_DESC_NUM_DEFAULT)

#define MARKER                   "------->"

/* This will be overridden if ethtool is not supported */
#define PHY_DEVICE_SUPPORT       1

/* All outgoing frames will be marked with the following traffic class */
#define TRAFFIC_CLASS            ETHHW_TC_HIGHEST

#define BCM_NET_MODULE_DESCRIPTION     "Broadcom BCMRING network driver"
#define BCM_NET_MODULE_VERSION         "0.2.0"

/*--------------------------------------------------------------------------*/

static const __devinitconst char banner[] =
   KERN_INFO BCM_NET_MODULE_DESCRIPTION " " BCM_NET_MODULE_VERSION "\n";

#ifndef CONFIG_NET_BCMRING_ETHTOOL
#undef PHY_DEVICE_SUPPORT
#define PHY_DEVICE_SUPPORT       0
#endif

#if 0
#define TXCFG_SIZE               ((sizeof(uint64_t) + sizeof(struct sk_buff *))
#else
/* Force onto 64-bit boundary */
#define TXCFG_SIZE               16
#endif
#define TXCFG_BUF(bp, i)         (uint64_t *)((char *)(bp) + (TXCFG_SIZE * (i)))
#define TXCFG_BUF_SIZE           (TXCFG_SIZE * DMA_TX_DESC_NUM)

#define RX_DESC(bp, i)           (ETHHW_DMA_DESC *)((char *)(bp) + ((i) * sizeof(ETHHW_DMA_DESC)))
#define TX_DESC                  RX_DESC

static struct workqueue_struct *ethStatusWorkQueuep;

static void ethStatusWork(struct work_struct *work);
DECLARE_DELAYED_WORK(ethStatusTask, ethStatusWork);
/* TODO: Needed for ethStatusWork, but should supply argument at run-time */
static struct net_device *localDevp;

#ifndef MARKER
#define MARKER
#endif

#define myPrintk(f, s...)        printk(MARKER "%s(): " f, __func__, ## s)
#define myPanic(f, s...)         panic(MARKER "%s(): " f, __func__, ## s)

/* TODO: Replace DMA_ESW_xxx with appropriate value from header file */
#define DMA_ESW_MODULE           1
#define DMA_ESW_CH_RX            2           /* PTM direction */
#define DMA_ESW_CH_TX            3           /* MTP direction */

#define DMA_BURST_SIZE           8
#define DMA_BLOCK_SIZE           256

#define IRQ_DMA_ESW_RX           IRQ_DMA1C0  /* PTM */
#define IRQ_DMA_ESW_TX           IRQ_DMA1C1  /* MTP */

#define PORT_PC                  ETHHW_PORT_0
#define PORT_LAN                 ETHHW_PORT_1

#define PHY_DEV_NUM              ETHHW_PORT_NUM

#define isDaMcast(bp)            ((bp)[0] == 0x01)

#define isVLANtagged(bp)         (((bp)[12] == 0x81) && \
                                  ((bp)[13] == 0x00))

#define TX_TIMEOUT               (5 * HZ)
#define MIN_UNTAGGED_FRAME_LEN   60
#define MIN_TAGGED_FRAME_LEN     64
#define PAD_BUF_SIZE             (MIN_TAGGED_FRAME_LEN + sizeof(struct sk_buff *))

#define BRCM_ENCAP_MODE_REMOVE         0  /* Remove Broadcom field */
#define BRCM_ENCAP_MODE_KEEP_SKIP      1  /* Keep Broadcom field, but bypass in Rx driver */
#define BRCM_ENCAP_MODE_KEEP_HANDLER   2  /* Keep Broadcom field, but bypass in packet handler */

static unsigned int brcmMode = BRCM_ENCAP_MODE_REMOVE;
module_param(brcmMode, uint, 0644);
MODULE_PARM_DESC(brcmMode, "Rx Broadcom type encapsulation mode");
#define BRCM_ENCAP_MODE    brcmMode

static unsigned int descRx = DMA_RX_DESC_NUM_DEFAULT;
module_param(descRx, uint, 0644);
MODULE_PARM_DESC(descRx, "Number of Rx descriptors");
#define DMA_RX_DESC_NUM    descRx 

static unsigned int descTx = DMA_TX_DESC_NUM_DEFAULT;
module_param(descTx, uint, 0644);
MODULE_PARM_DESC(descTx, "Number of Tx descriptors");
#define DMA_TX_DESC_NUM    descTx 

/* module parameters for rate limiting */
#define RATE_LIMIT_UDP_THRESHOLD_PPS_NORMAL_DEFAULT 8000
#define RATE_LIMIT_TCP_THRESHOLD_PPS_NORMAL_DEFAULT 8000
#define RATE_LIMIT_UDP_THRESHOLD_PPS_DOS_DEFAULT    800
#define RATE_LIMIT_TCP_THRESHOLD_PPS_DOS_DEFAULT    400

/* define the DOS detection period */
#define RATE_LIMIT_DOS_DET_PERIOD_MS_DEFAULT        60 /* 60 ms */

static unsigned int rLEnable = 0; /* disabled by default */
module_param(rLEnable, uint, 0644);
MODULE_PARM_DESC(rLEnable, "Rate Limit enable/disable");
#define RATE_LIMIT_ENABLE    rLEnable

static unsigned int udpThNormal = RATE_LIMIT_UDP_THRESHOLD_PPS_NORMAL_DEFAULT;
module_param(udpThNormal, uint, 0644);
MODULE_PARM_DESC(udpThNormal, "Rate Limit UDP normal threshold in PPS");
#define RATE_LIMIT_UDP_THRESHOLD_NORMAL_PPS  udpThNormal

static unsigned int tcpThNormal = RATE_LIMIT_TCP_THRESHOLD_PPS_NORMAL_DEFAULT;
module_param(tcpThNormal, uint, 0644);
MODULE_PARM_DESC(tcpThNormal, "Rate Limit TCP normal hreshold in PPS");
#define RATE_LIMIT_TCP_THRESHOLD_NORMAL_PPS  tcpThNormal

static unsigned int udpThDos = RATE_LIMIT_UDP_THRESHOLD_PPS_DOS_DEFAULT;
module_param(udpThDos, uint, 0644);
MODULE_PARM_DESC(udpThDos, "Rate Limit UDP Threshold during DOS in PPS");
#define RATE_LIMIT_UDP_THRESHOLD_DOS_PPS  udpThDos

static unsigned int tcpThDos = RATE_LIMIT_TCP_THRESHOLD_PPS_DOS_DEFAULT;
module_param(tcpThDos, uint, 0644);
MODULE_PARM_DESC(tcpThDos, "Rate Limit TCP Threshold during in PPS");
#define RATE_LIMIT_TCP_THRESHOLD_DOS_PPS  tcpThDos

static unsigned int dosDetPeriodMs = RATE_LIMIT_DOS_DET_PERIOD_MS_DEFAULT;
module_param(dosDetPeriodMs, uint, 0644);
MODULE_PARM_DESC(dosDetPeriodMs, "Rate Limit TCP Threshold during in PPS");
#define RATE_LIMIT_DOS_DET_PERIOD_MS dosDetPeriodMs

static int sysCtlStatusPoll;
static uint32_t sysCtlMsgLevel;
static unsigned int sysCtlTxFifoMax;
static unsigned int sysCtlRxFifoMax;
static unsigned int sysCtlRxDiscard;
static unsigned int sysCtlPmDisablePortPc;
static unsigned int sysCtlPmDisablePortLan;
static unsigned int sysCtlPmDisablePortInternal;

/* These rate limit threshold are in packet per jiffy */
static unsigned int sysCtlUdpRateLimitThreshold;
static unsigned int sysCtlTcpRateLimitThreshold;
static unsigned int sysCtlUdpRateLimitDropped;
static unsigned int sysCtlTcpRateLimitDropped;
static unsigned int sysCtlUdpRateLimitPassed;
static unsigned int sysCtlTcpRateLimitPassed;
static unsigned int sysCtlrLEnable;
static unsigned int sysCtlUdpThresholdNormal;
static unsigned int sysCtlUdpThresholdDos;
static unsigned int sysCtlTcpThresholdNormal;
static unsigned int sysCtlTcpThresholdDos;
static unsigned int sysCtlDosDetPeriod; /* in units of jiffies */

/* For packet base rate limiting - applied to UDP and TCP packets */
static unsigned int rateLimitUdpPacketCount = 0;
static unsigned int rateLimitTcpPacketCount = 0;
static unsigned long rateLimitUdpLastJiffies = 0;
static unsigned long rateLimitTcpLastJiffies = 0;

/* For DOS attack detection */
static unsigned int rateLimitConsecutiveJiffiesUdpPacketDropped = 0;
static unsigned long rateLimitDiscardedUdpPacketsWithinJiffy = 0;
static unsigned long rateLimitLastUdpPacketDropJiffies = 0;
static unsigned int rateLimitConsecutiveJiffiesTcpPacketDropped = 0;
static unsigned long rateLimitDiscardedTcpPacketsWithinJiffy = 0;
static unsigned long rateLimitLastTcpPacketDropJiffies = 0;

/* Threshold variables in units of packet per jiffy */
/* The spec calls for 120 packets per 15ms window, for both
   TCP and UDP packets. This translates to 8000 packets per
   second. If we use jiffy, which is 10ms, it would be 80 packets
   per 10ms window */
static unsigned int rateLimitUdpThresholdPacketsPerJiffyNormal = 0;
static unsigned int rateLimitTcpThresholdPacketsPerJiffyNormal = 0;
static unsigned int rateLimitUdpThresholdPacketsPerJiffyDos = 0;
static unsigned int rateLimitTcpThresholdPacketsPerJiffyDos = 0;
static unsigned int rateLimitUdpThresholdPacketsPerJiffy = 0;
static unsigned int rateLimitTcpThresholdPacketsPerJiffy = 0;
static unsigned int rateLimitDosDetPeriodInJiffies = 0;

#define RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY rateLimitUdpThresholdPacketsPerJiffy
#define RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY rateLimitTcpThresholdPacketsPerJiffy
#define RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY_DOS rateLimitUdpThresholdPacketsPerJiffyDos
#define RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY_DOS rateLimitTcpThresholdPacketsPerJiffyDos
#define RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY_NORMAL rateLimitUdpThresholdPacketsPerJiffyNormal
#define RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY_NORMAL rateLimitTcpThresholdPacketsPerJiffyNormal
#define RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES rateLimitDosDetPeriodInJiffies

#define CTL_TABLE_ENTRY(_ctl_name, _procname, _data, _mode) \
   .ctl_name = (_ctl_name), \
   .procname = (_procname), \
   .data = (void *)&(_data), \
   .maxlen = sizeof((_data)), \
   .mode = (_mode), \
   .proc_handler = &proc_dointvec

static struct ctl_table sysCtlEth[] = {
	{CTL_TABLE_ENTRY(1, "statusPoll", sysCtlStatusPoll, 0644)},
	{CTL_TABLE_ENTRY(2, "msgLevel", sysCtlMsgLevel, 0644)},
	{CTL_TABLE_ENTRY(3, "txFifoMax", sysCtlTxFifoMax, 0644)},
	{CTL_TABLE_ENTRY(4, "rxFifoMax", sysCtlRxFifoMax, 0644)},
	{CTL_TABLE_ENTRY(5, "pmDisablePortPc", sysCtlPmDisablePortPc, 0644)},
	{CTL_TABLE_ENTRY(6, "pmDisablePortLan", sysCtlPmDisablePortLan, 0644)},
	{CTL_TABLE_ENTRY(7, "pmDisablePortInternal", sysCtlPmDisablePortInternal, 0644)},
	{CTL_TABLE_ENTRY(8, "rxDiscard", sysCtlRxDiscard, 0644)},
	{CTL_TABLE_ENTRY(9, "udpThActivePpj", sysCtlUdpRateLimitThreshold, 0644)},
	{CTL_TABLE_ENTRY(10, "tcpThActivePpj", sysCtlTcpRateLimitThreshold, 0644)},
	{CTL_TABLE_ENTRY(11, "udpDropped", sysCtlUdpRateLimitDropped, 0644)},
	{CTL_TABLE_ENTRY(12, "tcpDropped", sysCtlTcpRateLimitDropped, 0644)},
	{CTL_TABLE_ENTRY(13, "udpPassed", sysCtlUdpRateLimitPassed, 0644)},
	{CTL_TABLE_ENTRY(14, "tcpPassed", sysCtlTcpRateLimitPassed, 0644)},
	{CTL_TABLE_ENTRY(15, "rLEnable", sysCtlrLEnable, 0644)},
	{CTL_TABLE_ENTRY(16, "udpThNormalPpj", sysCtlUdpThresholdNormal, 0644)},
	{CTL_TABLE_ENTRY(17, "udpThDosPpj", sysCtlUdpThresholdDos, 0644)},
	{CTL_TABLE_ENTRY(18, "tcpThNormalPpj", sysCtlTcpThresholdNormal, 0644)},
	{CTL_TABLE_ENTRY(19, "tcpThDosPpj", sysCtlTcpThresholdDos, 0644)},
	{CTL_TABLE_ENTRY(20, "dosDetPeriodJ", sysCtlDosDetPeriod, 0644)},
	{}
};

static struct ctl_table sysCtl[] = {
	{
	   .ctl_name = CTL_BCM_ETH,
	   .procname = "eth",
	   .mode = 0555,
	   .child = sysCtlEth
	},
	{}
};

static struct ctl_table_header *sysCtlHeaderp;

/* The message level is actually a mask for this driver */
#define MSG_LEVEL_OFF            0x000000000
#define MSG_LEVEL_RX             0x000000001
#define MSG_LEVEL_RX_RAW         0x000000002
#define MSG_LEVEL_RX_VERBOSE     0x000000004
#define MSG_LEVEL_TX             0x000000100
#define MSG_LEVEL_TX_RAW         0x000000200
#define MSG_LEVEL_TX_VERBOSE     0x000000400
#define MSG_LEVEL_STATUS_VERBOSE 0x000001000

#define MSG_LEVEL_RX_RAW_VERBOSE MSG_LEVEL_RX_VERBOSE
#define MSG_LEVEL_TX_RAW_VERBOSE MSG_LEVEL_TX_VERBOSE

#define STR_OCTETS_FORMAT        "%02x %02x %02x %02x %02x %02x %02x %02x"
#define STR_OCTETS_ARGS(bp)      (bp)[0], (bp)[1], (bp)[2], (bp)[3], (bp)[4], (bp)[5], (bp)[6], (bp)[7]
#define FRAME_DUMP(msgLevel, strp, bufp, len) \
if (sysCtlMsgLevel & (msgLevel)) { \
	printk("%s: len=%i, bufp=0x%08x\n", (strp), (len), (int)(bufp)); \
	if (sysCtlMsgLevel & msgLevel ## _VERBOSE) { \
		printk(STR_OCTETS_FORMAT " " STR_OCTETS_FORMAT " " \
		       STR_OCTETS_FORMAT " ... " STR_OCTETS_FORMAT "\n", \
		       STR_OCTETS_ARGS(&bufp[0]), STR_OCTETS_ARGS(&bufp[8]), \
		       STR_OCTETS_ARGS(&bufp[16]), STR_OCTETS_ARGS(&bufp[len - 8])); \
	} \
}

#define FRAME_DUMP_ALWAYS(strp, bufp, len) \
	printk("%s: len=%i, bufp=0x%08x\n", (strp), (len), (int)(bufp)); \
	printk(STR_OCTETS_FORMAT " " STR_OCTETS_FORMAT " " \
	       STR_OCTETS_FORMAT " ... " STR_OCTETS_FORMAT "\n", \
	       STR_OCTETS_ARGS(&bufp[0]), STR_OCTETS_ARGS(&bufp[8]), \
	       STR_OCTETS_ARGS(&bufp[16]), STR_OCTETS_ARGS(&bufp[len - 8])); \

#define msg_printk(msgLevel,s, ...) \
                                 if (sysCtlMsgLevel & (msgLevel)) printk(s, ##__VA_ARGS__)

#define SKB_BUF_HDR(bp)          (*(struct sk_buff **)((void *)(bp) - sizeof(struct sk_buff *)))

typedef struct {
	struct net_device *dev;
} IRQ_CONTEXT;

typedef struct {
	int descLen;
	void *descBufp;      /* Virtual address */
	dma_addr_t handle;   /* Physical address */
	int index;
	int tail;
	int count;
	unsigned int countAlloc;
	unsigned int countFree;
	int ringSize;
	IRQ_CONTEXT irqContext;
} DMA_DRV_PRIV;

typedef u64 mib_t;

typedef struct {
	/* NOTE: Do not change the case or the name of these structure members.
	*        The structure member name was chosen to simplify the coding later
	*        by creating macros.
	*/
	mib_t TxOctets;
	mib_t TxDropPkts;
	mib_t TxBroadcastPkts;
	mib_t TxMulticastPkts;
	mib_t TxUnicastPkts;
	mib_t TxCollisions;
	mib_t TxSingleCollision;
	mib_t TxMultipleCollision;
	mib_t TxDeferredTransmit;
	mib_t TxLateCollision;
	mib_t TxExcessiveCollision;
	mib_t TxFrameInDisc;
	mib_t TxPausePkts;
	mib_t RxOctets;
	mib_t RxUndersizePkts;
	mib_t RxPausePkts;
	mib_t Pkts64octets;
	mib_t Pkts65to127octets;
	mib_t Pkts128to255octets;
	mib_t Pkts256to511octets;
	mib_t Pkts512to1023octets;
	mib_t Pkts1024to1522octets;
	mib_t RxOversizePkts;
	mib_t RxJabbers;
	mib_t RxAlignmentErrors;
	mib_t RxFcsErrors;
	mib_t RxGoodOctets;
	mib_t RxDropPkts;
	mib_t RxUnicastPkts;
	mib_t RxMulticastPkts;
	mib_t RxBroadcastPkts;
	mib_t RxSaChanges;
	mib_t RxFragments;
	mib_t RxExcessSizeDisc;
	mib_t RxSymbolError;
} ETH_DRV_MIB;

typedef struct {
	char *padBufp;
	dma_addr_t padHandle;
	int phyPort;
	struct {
		int lan;
		int pc;
	} port;
	ETH_DRV_MIB mib;
} ETH_PRIV;

typedef struct {
	DMA_DRV_PRIV rx;
	DMA_DRV_PRIV tx;
	struct kfifo *txFifop;
	struct {
		char *bufp;
		int index;
	} txCfg;
} DMA_PRIV;

typedef struct {
	int portNum;                        /* Number of external ports available */
	int portNumExt;                     /* Number of external ports for external PHY */
	int phyAvailable[ETHHW_PORT_NUM];   /* Per-port PHY available */
	int gmiiAvailable[ETHHW_PORT_NUM];  /* Per-port GMII/1000Mbps available */
	int sgmiiAvailable[ETHHW_PORT_NUM]; /* Per-port SGMII available */
} CHIP_PRIV;

#define CHIP_PORTS(privp)     (privp)->chip.portNum

typedef struct {
	struct net_device_stats stat;
	struct mii_if_info mii_if;
	ETH_PRIV eth;
	DMA_PRIV dma;
	CHIP_PRIV chip;
#ifdef CONFIG_PM
	int wasRunning;
#endif
	struct napi_struct napi;
#if PHY_DEVICE_SUPPORT
	struct net_device *phyDev[PHY_DEV_NUM];
#endif
} BCMRING_PRIV;

static BCMRING_PRIV *procPrivp;

static void *allocFuncp(int len);
static void freeFunc(void *bufp);
static int dmaStart(DMA_PRIV *dmap);
static int dmaStop(DMA_PRIV *dmap);
static int ethStart(ETH_PRIV *ethp);
static int txClean(DMA_PRIV *dmap);
static void tx(BCMRING_PRIV *privp);
static void txWork(unsigned long data);
DECLARE_TASKLET(txTasklet, txWork, 0);
static BCMRING_PRIV *txPrivp;
static struct workqueue_struct *txWorkQueuep;

static int miiGet(struct net_device *dev, int phy_id, int location);
static void miiSet(struct net_device *dev, int phy_id, int location, int val);

static void ethIsrRx(DMA_Device_t dev, int reason, void *userData);
static void ethIsrTx(DMA_Device_t dev, int reason, void *userData);

static int brcmPacketTypeHandler(struct sk_buff *skb, struct net_device *dev,
                                 struct packet_type *ptype,
                                 struct net_device *orig_dev);

static int bcmring_net_open(struct net_device *dev);
static int bcmring_net_close(struct net_device *dev);
static int bcmring_net_do_ioctl(struct net_device *dev, struct ifreq *ifr,
                                int cmd);
static int bcmring_net_hard_start_xmit(struct sk_buff *skb,
                                       struct net_device *dev);
static int bcmring_net_poll(struct napi_struct *napi, int quota);
static void bcmring_net_set_multicast_list(struct net_device *dev);
static int bcmring_net_set_mac_address(struct net_device *dev, void *addr);
static struct net_device_stats *bcmring_net_get_stats(struct net_device *dev);
static void bcmring_net_tx_timeout(struct net_device *dev);
#ifdef CONFIG_NET_POLL_CONTROLLER
static void bcmring_net_poll_controller(struct net_device *dev);
#endif
static int bcmring_net_init(void);
static void bcmring_net_exit(void);
#ifdef CONFIG_PM
static int bcmring_net_notify_reboot(struct notifier_block *nb,
                                     unsigned long event, void *ptr);
static int bcmring_net_suspend(struct platform_device *pdev, 
                               pm_message_t state);
static int bcmring_net_resume(struct platform_device *pdev);
static struct notifier_block bcmring_net_notifier_reboot = {
   .notifier_call = bcmring_net_notify_reboot,
   .next          = NULL,
   .priority      = 0

};
#else
#define bcmring_net_suspend      NULL
#define bcmring_net_resume       NULL
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
static const struct net_device_ops bcmring_netdev_ops = {
	.ndo_open               = bcmring_net_open,
	.ndo_stop               = bcmring_net_close,
	.ndo_get_stats          = bcmring_net_get_stats,
	.ndo_start_xmit         = bcmring_net_hard_start_xmit,
	.ndo_set_multicast_list = bcmring_net_set_multicast_list,
	.ndo_set_mac_address    = bcmring_net_set_mac_address,
	.ndo_tx_timeout         = bcmring_net_tx_timeout,
	.ndo_do_ioctl           = bcmring_net_do_ioctl,
};

#if PHY_DEVICE_SUPPORT
static const struct net_device_ops phy_netdev_ops;
#endif
#endif

static struct packet_type brcmPacketType = {
	.type = cpu_to_be16(ETHHW_BRCM_TYPE),
	.func = brcmPacketTypeHandler,
};

static inline int ethMiiGet(int port, int addr)
{
	uint32_t val;

	ethHw_miiGet(port, 0, (uint32_t)addr, &val);

	return val;
}

static inline void ethMiiSet(int port, int addr, int data)
{
	ethHw_miiSet(port, 0, (uint32_t)addr, (uint32_t)data);
}

#ifdef CONFIG_NET_BCMRING_ETHTOOL
#include "bcmring_ethtool.c"
#endif

/*--------------------------------------------------------------------------*/

static void *allocFuncp(int len)
{
	struct sk_buff *skb;
	char *bufp;
	int offset;

	/* Allocate socket buffer */
	skb = dev_alloc_skb(len);
	if (likely(skb)) {
		/* Reserve space for a buffer header used to identify the sk_buff when
		*  freeing the socket buffer.  At the same time, ensure skb->data is on
		*  64bit alignment for DMA purposes.
		*/
		bufp = (char *)(((int)skb->data + sizeof(struct sk_buff *) + 7) & 0xfffffff8);
		offset = (int)bufp - (int)skb->data;

		skb_reserve(skb, offset);
		SKB_BUF_HDR(skb->data) = skb;
	} else {
		return NULL;
	}

	return (void *)skb->data;
}

static void freeFunc(void *bufp)
{
	struct sk_buff *skb;

	skb = SKB_BUF_HDR(bufp);
	if (skb) {
		/* Unmap data buffer */
		dma_unmap_single(NULL, (dma_addr_t) bufp, skb->len, DMA_FROM_DEVICE);

		/* Free socket buffer */
		dev_kfree_skb_any(skb);
	} else {
		/* Buffer is a zero padding buffer used to enforce the minimum ethernet
		*  frame size requirements.  This buffer can be used many times so it
		*  does need to be freed here.  It will be freed when the driver is
		*  closed in bcmring_net_close()
		*/
	}
}

static int dmaStart(DMA_PRIV *dmap)
{
	ethHw_dmaInit(DMA_ESW_MODULE);

	/* Start Rx DMA */

	ethHw_dmaDisable(DMA_ESW_MODULE, DMA_ESW_CH_RX);

	/* Configure Rx DMA */
	ethHw_dmaRxConfig(DMA_BURST_SIZE, DMA_BLOCK_SIZE);
	ethHw_dmaConfig(DMA_ESW_MODULE, DMA_ESW_CH_RX,
	                ETHHW_DMA_PTM_CTL_HI, ETHHW_DMA_PTM_CTL_LO,
	                ETHHW_DMA_PTM_CFG_HI, ETHHW_DMA_PTM_CFG_LO, ETHHW_DMA_PTM_SSTAT, 0);

	dmap->rx.count = 0;  /* Use count to keep track of Rx updates */
	dmap->rx.index = 0;
	dmap->rx.tail = 0;
	dmap->rx.descLen = DMA_RX_DESC_NUM * sizeof(ETHHW_DMA_DESC);
	dmap->rx.descBufp = (void *)dma_alloc_coherent(NULL, dmap->rx.descLen,
	                                               &dmap->rx.handle,
	                                               GFP_KERNEL);
	if (!dmap->rx.descBufp) {
		myPrintk(KERN_ERR "Failed to alloc Rx DMA descriptor memory\n");

		return -ENOMEM;
	}

	/* Configure Rx descriptors */
	{
		int i;
		uint8_t *bufp;
		ETHHW_DMA_DESC *descp;
		ETHHW_DMA_DESC *handlep;
		dma_addr_t dmaBuf;

		/* Remove compiler warning when DMA_RX_DESC_NUM = 0 */
		descp = NULL;

		for (i = 0; i < DMA_RX_DESC_NUM; i++) {
			descp = RX_DESC(dmap->rx.descBufp, i);
			handlep = RX_DESC(dmap->rx.handle, i);
			bufp = (uint8_t *)allocFuncp(DMA_RX_BUF_LEN);
			if (!bufp) {
				myPrintk(KERN_ERR "Failed to allocate Rx DMA buffer\n");
				dmaStop(dmap);
				return -ENOMEM;
			}

			/* Map data buffer to invalidate cache */
			dmaBuf = dma_map_single(NULL, bufp, DMA_RX_BUF_LEN,
			                        DMA_FROM_DEVICE);

			/* Rx config descriptor
			*  The config descriptor allows transfer to be purely 64-bit
			*  transactions, spanning an arbitrary number of descriptors,
			*  so information must be provided to define offsets and EOP
			*/
			ETHHW_DMA_DESC_CREATE(descp,
			                      ETHHW_DMA_PTM_FIFO_ADDR,
			                      dmaBuf,
			                      handlep,
			                      ETHHW_DMA_PTM_CTL_LO | ETHHW_DMA_CTL_INT_EN,   /* Enable interrupts per Rx descriptor */
			                      DMA_RX_BUF_LEN);

			/* Save virtual address of buffer */
			descp->rsvd = (uint32_t)bufp;
		}

		/* Wrap last descriptor back to beginning */
		ETHHW_DMA_DESC_WRAP(descp, (uint32_t)RX_DESC(dmap->rx.handle, 0));
	}

	dma_set_device_handler(DMA_DEVICE_ESW_DEV_TO_MEM,
	                       (DMA_DeviceHandler_t) ethIsrRx,
	                       (void *)&dmap->rx.irqContext);
	ethHw_dmaIrqClear(DMA_ESW_MODULE, DMA_ESW_CH_RX);
	ethHw_dmaIrqEnable(DMA_ESW_MODULE, DMA_ESW_CH_RX);

	/* Start Tx DMA */

	/* Allocate buffer for Tx config */
	dmap->txCfg.index = 0;
	dmap->txCfg.bufp = (char *)kmalloc(TXCFG_BUF_SIZE, GFP_KERNEL);
	if (!dmap->txCfg.bufp) {
		myPrintk(KERN_ERR "Failed to alloc Tx config buffer memory\n");
		dmaStop(dmap);
		return -ENOMEM;
	}

	ethHw_dmaDisable(DMA_ESW_MODULE, DMA_ESW_CH_TX);

	/* Configure Tx DMA */
	ethHw_dmaTxConfig(DMA_BURST_SIZE);
	ethHw_dmaConfig(DMA_ESW_MODULE, DMA_ESW_CH_TX,
	                ETHHW_DMA_MTP_CTL_HI, ETHHW_DMA_MTP_CTL_LO,
	                ETHHW_DMA_MTP_CFG_HI, ETHHW_DMA_MTP_CFG_LO, 0, ETHHW_DMA_MTP_DSTAT);

	dmap->tx.count = dmap->tx.ringSize;    /* Use count to keep track of queue level */
	dmap->tx.countAlloc = 0;
	dmap->tx.countFree = 0;
	dmap->tx.index = 0;
	dmap->tx.tail = 0;
	dmap->tx.descLen = DMA_TX_DESC_NUM * sizeof(ETHHW_DMA_DESC);
	dmap->tx.descBufp = (void *)dma_alloc_coherent(NULL, dmap->tx.descLen,
	                                               &dmap->tx.handle,
	                                               GFP_KERNEL);
	if (!dmap->tx.descBufp) {
		myPrintk(KERN_ERR "Failed to alloc Tx DMA descriptor memory\n");
		dmaStop(dmap);
		return -ENOMEM;
	}

	dma_set_device_handler(DMA_DEVICE_ESW_MEM_TO_DEV,
	                       (DMA_DeviceHandler_t) ethIsrTx,
	                       (void *)&dmap->tx.irqContext);
	ethHw_dmaIrqClear(DMA_ESW_MODULE, DMA_ESW_CH_TX);
	ethHw_dmaIrqDisable(DMA_ESW_MODULE, DMA_ESW_CH_TX);

	/* TODO: Do not enable the RX DMA until the everything is ready */

	return 0;
}

static int dmaStop(DMA_PRIV *dmap)
{
	/* Stop Rx DMA */

	/* Disable Rx DMA */
	ethHw_dmaDisable(DMA_ESW_MODULE, DMA_ESW_CH_RX);

	/* Free pending Rx buffers */
	{
		int i;
		ETHHW_DMA_DESC *descp;
		char *bufp;

		for (i = 0; i < DMA_RX_DESC_NUM; i++) {
			descp = RX_DESC(dmap->rx.descBufp, i);
			bufp = (char *)descp->rsvd;
			freeFunc(bufp);
		}
	}

	/* Free memory used for descriptor ring */
	dma_free_coherent(NULL, dmap->rx.descLen, dmap->rx.descBufp,
	                  dmap->rx.handle);

	/* Stop Tx DMA */

	/* Disable Tx DMA */
	ethHw_dmaDisable(DMA_ESW_MODULE, DMA_ESW_CH_TX);

	/* Free pending Tx buffers */
	txClean(dmap);

	/* Free memory used for descriptor ring */
	dma_free_coherent(NULL, dmap->tx.descLen, dmap->tx.descBufp,
	                  dmap->tx.handle);
	kfree(dmap->txCfg.bufp);

	return 0;
}

static int ethStart(ETH_PRIV *ethp)
{
	int rc;
	int reg;

	rc = ethHw_Init();
	if (rc) {
		myPrintk(KERN_ERR "Ethernet initialization failed (rc=%i)\n", rc);
	}
	else {
		myPrintk(KERN_WARNING "Ethernet initialization successful (rc=%i)\n", rc);
	}

	/* Disable forwarding to internal port (this must be done before
	*  forwarding is enabled on the external ports)
	*/
	ethHw_macEnableSet(ETHHW_PORT_INT, 0, 0);

	/* STP not used so put external ports in forwarding state */
	/* TODO:  If STP support is required, this state control will need to be
	*         moved to the STP application
	*/
	ethHw_stpStateSet(ETHHW_PORT_0, ETHHW_STP_STATE_FORWARDING);
	ethHw_stpStateSet(ETHHW_PORT_1, ETHHW_STP_STATE_FORWARDING);

	/* Assign ports */
	ethp->port.lan = PORT_LAN;
	ethp->port.pc = PORT_PC;

	ethp->phyPort = ethp->port.lan;

	/*Set LRE Access Register to enable 
	 *LRE Register Set for LAN port
	*/
	reg = ethMiiGet(ethp->port.lan, 14);
	if(!reg & 0x0001)
	{
		reg = ethMiiGet(ethp->port.lan, 14);
		reg |= 0x0004;
		ethMiiSet(ethp->port.lan, 14, reg);
	}

	/*Enable WireSpeed mode for LAN port*/
	ethMiiSet(ethp->port.lan, 0x18, 0x7007);
	reg = ethMiiGet(ethp->port.lan, 0x18);
	reg |= 0x8017;
	ethMiiSet(ethp->port.lan, 0x18, reg);

	/*Set LRE Access Register to enable 
	 *LRE Register Set for PC port
	*/
	reg = ethMiiGet(ethp->port.pc, 14);
	if(!reg & 0x0001)
	{
		reg = ethMiiGet(ethp->port.pc, 14);
		reg |= 0x0004;
		ethMiiSet(ethp->port.pc, 14, reg);
	}

	/*Enable WireSpeed mode for PC port*/
	ethMiiSet(ethp->port.pc, 0x18, 0x7007);
	reg = ethMiiGet(ethp->port.pc, 0x18);
	reg |= 0x8017;
	ethMiiSet(ethp->port.pc, 0x18, reg);
	
	/* Disable forwarding to internal port */
	ethHw_impEnableSet(0);

	/* Clear MIB */
	memset(&ethp->mib, 0, sizeof(ethp->mib));

	return 0;
}

static int miiGet(struct net_device *dev, int phy_id, int location)
{
	uint32_t val;
	BCMRING_PRIV *privp;

	privp = netdev_priv(dev);

	ethHw_miiGet(phy_id, 0, (uint32_t) location, &val);

	return val;
}

static void miiSet(struct net_device *dev, int phy_id, int location, int val)
{
	BCMRING_PRIV *privp;

	privp = netdev_priv(dev);

	ethHw_miiSet(phy_id, 0, (uint32_t) location, (uint32_t) val);
}

static void ethIsrRx(DMA_Device_t dev, int reason, void *userData)
{
	IRQ_CONTEXT *contextp;
	BCMRING_PRIV *privp;

	(void)reason;

	if (unlikely(dev != DMA_DEVICE_ESW_DEV_TO_MEM)) {
		myPrintk(KERN_ERR "Invalid device.  Received %d, expected %d.\n", dev,
		         DMA_DEVICE_ESW_DEV_TO_MEM);
	}

	contextp = (IRQ_CONTEXT *)userData;

	/* Disable and acknowledge Rx DMA interrupt */
	ethHw_dmaIrqDisable(DMA_ESW_MODULE, DMA_ESW_CH_RX);
	ethHw_dmaIrqClear(DMA_ESW_MODULE, DMA_ESW_CH_RX);

	/* Rx frame is available, so schedule poll using NAPI */
	privp = netdev_priv(contextp->dev);
	napi_schedule(&privp->napi);
}

static void ethIsrTx(DMA_Device_t dev, int reason, void *userData)
{
	IRQ_CONTEXT *contextp;
	BCMRING_PRIV *privp;

	(void)reason;
	(void)userData;

	if (unlikely(dev != DMA_DEVICE_ESW_MEM_TO_DEV)) {
		myPrintk(KERN_ERR "Invalid device.  Received %d, expected %d.\n", dev,
		         DMA_DEVICE_ESW_DEV_TO_MEM);
	}

	/* Acknowledge Tx DMA interrupt */
	ethHw_dmaIrqClear(DMA_ESW_MODULE, DMA_ESW_CH_TX);

	contextp = (IRQ_CONTEXT *)userData;
	privp = netdev_priv(contextp->dev);

	if (ethHw_dmaIsBusy(DMA_ESW_MODULE, DMA_ESW_CH_TX)) {
		/* Tx DMA transfer in progress, so just leave frame in Tx FIFO.  Another
		*  attempt to send the frame will be made when the DMA interrupt signals
		*  the completion of the current DMA transfer
		*/
	} else {
		tasklet_schedule(&txTasklet);
   }
}

static inline __be16 brcmPacketType_eth_type_trans(struct sk_buff *skb)
{
	__be16 type;

	/* This function is modelled after eth_type_trans() */

	type = *((__be16 *)skb->data);

	/* Move data pointer to start of payload */
	skb_pull(skb, sizeof( __be16 ));

	/* Determine ethernet type.  Please refer to eth_type_trans() for an
	*  explaination on the logic below
	*/
	if (ntohs(type) < 1536) {
		if (type == 0xFFFF)
			type = htons(ETH_P_802_3);
		else
			type = htons(ETH_P_802_2);
	}

	return( type );
}

static inline void __brcmPacketTypeHandler(struct sk_buff *skb)
{
	/* Adjust skb to start of the true ethernet type field */
	skb_pull(skb, ETHHW_BRCM_HDR_LEN - sizeof( __be16 ));

	/* Update ethernet type field in skb */
	skb->protocol = brcmPacketType_eth_type_trans(skb);
}

static int brcmPacketTypeHandler(struct sk_buff *skb, struct net_device *dev,
                                 struct packet_type *ptype,
                                 struct net_device *orig_dev)
{
	/* The intent of the Broadcom ethernet type is to allow a raw socket
	*  application to control the egress port for tx/send() frames and/or
	*  determine the ingress port for rx/recv() frames while maintaining
	*  original compatibility
	*/

	(void)dev;
	(void)ptype;
	(void)orig_dev;

	__brcmPacketTypeHandler(skb);

	/* Re-queue packet to IP stack */
	netif_rx(skb);

	return NET_RX_SUCCESS;
}

static int bcmring_net_open(struct net_device *dev)
{
	BCMRING_PRIV *privp;
	ETH_PRIV *ethp;
	char *padBufp;
	int rc;

	sysCtlStatusPoll = 1000; 
	sysCtlMsgLevel = MSG_LEVEL_OFF;
	sysCtlTxFifoMax = 0;
	sysCtlRxFifoMax = 0;
	sysCtlRxDiscard = 0;
	sysCtlPmDisablePortPc = 1;
	sysCtlPmDisablePortLan = 0;
	sysCtlPmDisablePortInternal = 0;
	sysCtlHeaderp = register_sysctl_table(sysCtl);
	sysCtlUdpRateLimitThreshold = RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY;
	sysCtlTcpRateLimitThreshold = RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY;
	sysCtlUdpThresholdNormal = RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY_NORMAL;
	sysCtlUdpThresholdDos = RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY_DOS;
	sysCtlTcpThresholdNormal = RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY_NORMAL;
	sysCtlTcpThresholdDos = RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY_DOS;
	sysCtlDosDetPeriod = RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES;
	sysCtlUdpRateLimitPassed = 0;
	sysCtlTcpRateLimitPassed = 0;
	sysCtlUdpRateLimitDropped = 0;
	sysCtlTcpRateLimitDropped = 0;
	sysCtlrLEnable = RATE_LIMIT_ENABLE;

	privp = netdev_priv(dev);
	ethp = &privp->eth;
	txPrivp = privp;

	/* Create Tx FIFO (enough to hold DMA_TX_DESC_NUM amount of pointers */
	privp->dma.txFifop = kfifo_alloc(DMA_TX_DESC_NUM * 4, GFP_KERNEL, NULL);
	if (!privp->dma.txFifop) {
		myPrintk(KERN_ERR "Failed to create Tx FIFO\n");
		return -ENOMEM;
	}

	/* Allocate buffer for zero padding Tx frames to meet minimum frame length
	*  requirements.
	*/
	padBufp = (char *)dma_alloc_coherent(NULL, PAD_BUF_SIZE,
	                                     &ethp->padHandle, GFP_KERNEL);
	if (!padBufp) {
		myPrintk(KERN_ERR "Failed to alloc Tx pad buffer memory\n");
		kfifo_free(privp->dma.txFifop);
		return -ENOMEM;
	}

	/* Initialize DMA parameters */
	privp->dma.rx.ringSize = DMA_RX_DESC_NUM;
	privp->dma.tx.ringSize = DMA_TX_DESC_NUM;

	privp->dma.rx.irqContext.dev = dev;
	privp->dma.tx.irqContext.dev = dev;

	/* Start DMA block */
	rc = dmaStart(&privp->dma);
	if (rc) {
		myPrintk(KERN_ERR "Failed to start DMA block\n");
		kfifo_free(privp->dma.txFifop);
		dma_free_coherent(NULL, PAD_BUF_SIZE, padBufp, ethp->padHandle);

		return rc;
	}

	/* Treat zero padding Tx buffer like a socket buffer to handle the case
	*  when the driver tries to free the padding buffer in freeFunc()
	*/
	ethp->padBufp = padBufp + sizeof(struct sk_buff *);
	memset(ethp->padBufp, 0, MIN_TAGGED_FRAME_LEN);
	SKB_BUF_HDR(ethp->padBufp) = (struct sk_buff *)NULL;

	/* Create new workqueue/thread */
	txWorkQueuep = create_workqueue("ethTx");
	ethStatusWorkQueuep = create_workqueue("ethStatus");
	queue_delayed_work(ethStatusWorkQueuep, &ethStatusTask, 0);

	napi_enable(&privp->napi);

	if (CHIP_PORTS(privp) == 1) {
		/* Single port configuration */

		/* Power-up the PHY */
		ethHw_phyPowerdownSet(privp->eth.phyPort, 0);
	} else {
		/* Non-single port configuration */

		/* The PHYs cannot be powered-down since the external ports may still be
		*  bridging network traffic
		*/

		/* TODO: Define other power saving operations for this configuration,
		*        for example, if only there is only one port has a link, perhaps
		*        keep PHYs enabled and disable MAC and switch clock.
		*/
	}

	if (BRCM_ENCAP_MODE == BRCM_ENCAP_MODE_KEEP_HANDLER) {
		/* Add packet handler for Broadcom raw socket packet type */
		dev_add_pack( &brcmPacketType );
	}

	netif_start_queue(dev);

	/* Check if network cable is connected and link is up to configure initial
	*  state
	*/
	if ( ethMiiGet( privp->eth.phyPort, ETHHW_MII_EXT_STATUS ) &
	     ETHHW_MII_EXT_STATUS_LINK_MASK ) {
		netif_carrier_on(localDevp);
	} else {
		netif_carrier_off(localDevp);
	}

	/* Enable forwarding to internal port */
	ethHw_impEnableSet(1);
	ethHw_macEnableSet(ETHHW_PORT_INT, 1, 1);

	/* TODO: Enable Rx DMA (Tx DMA is enabled when a Tx frame is ready to send) */
	ethHw_dmaEnable(DMA_ESW_MODULE, DMA_ESW_CH_RX, RX_DESC(privp->dma.rx.handle, 0));

	return 0;
}

static int bcmring_net_close(struct net_device *dev)
{
	BCMRING_PRIV *privp;
	ETH_PRIV *ethp;

	netif_stop_queue(dev);

	if (BRCM_ENCAP_MODE == BRCM_ENCAP_MODE_KEEP_HANDLER) {
		/* Remove packet handler for Broadcom raw socket packet type */
		dev_remove_pack( &brcmPacketType );
	}

	privp = netdev_priv(dev);
	ethp = &privp->eth;

	napi_disable(&privp->napi);

	/* Disable and free interrupts */
	ethHw_dmaIrqDisable(DMA_ESW_MODULE, DMA_ESW_CH_RX);
	ethHw_dmaIrqDisable(DMA_ESW_MODULE, DMA_ESW_CH_TX);

	/* Stop DMA and ethernet */
	dmaStop(&privp->dma);

	/* Disable forwarding to internal port */
	ethHw_macEnableSet(ETHHW_PORT_INT, 0, 0);
	ethHw_impEnableSet(0);

	/* Free Tx FIFO */
	kfifo_free(privp->dma.txFifop);

	/* Unmap and free buffer for zero padding Tx frames */
	dma_unmap_single(NULL, (dma_addr_t) ethp->padBufp, MIN_TAGGED_FRAME_LEN,
	                 DMA_FROM_DEVICE);
	dma_free_coherent(NULL, PAD_BUF_SIZE,
	                  ethp->padBufp - sizeof(struct sk_buff *),
	                  ethp->padHandle);

	if (sysCtlHeaderp) {
		unregister_sysctl_table(sysCtlHeaderp);
	}

	/* Flush and destroy workqueue/thread */
	flush_workqueue(txWorkQueuep);
	destroy_workqueue(txWorkQueuep);

	cancel_delayed_work(&ethStatusTask);
	flush_workqueue(ethStatusWorkQueuep);
	destroy_workqueue(ethStatusWorkQueuep);

	if (CHIP_PORTS(privp) == 1) {
		/* Power-down the PHY */
		ethHw_phyPowerdownSet(privp->eth.phyPort, 1);
	} else {
		/* Non-single port configuration */

		/* The PHYs cannot be powered-down since the external ports may still be
		*  bridging network traffic
		*/

		/* TODO: Define other power saving operations for this configuration,
		*        for example, if only there is only one port has a link, perhaps
		*        keep PHYs enabled and disable MAC and switch clock.
		*/
	}

	return 0;
}

static int bcmring_net_do_ioctl(struct net_device *dev, struct ifreq *ifr,
                                int cmd)
{
	int rc;
	BCMRING_PRIV *privp;

	privp = netdev_priv(dev);
	rc = 0;

	switch (cmd) {
	case SIOCGMIIPHY:
		{
			struct mii_ioctl_data *datap;
			datap = if_mii(ifr);
			datap->phy_id = privp->eth.phyPort;
		}
		break;

	case SIOCGMIIREG:
		{
			struct mii_ioctl_data *datap;
			datap = if_mii(ifr);
			datap->val_out = (__u16)ethMiiGet(datap->phy_id, datap->reg_num);
		}
		break;

	case SIOCSMIIREG:
		{
			struct mii_ioctl_data *datap;
			datap = if_mii(ifr);
			ethMiiSet(datap->phy_id, datap->reg_num, (int)datap->val_in);
		}
		break;

	default:
		{
			return -EOPNOTSUPP;
		}
	}

	return rc;
}

static int txClean(DMA_PRIV *dmap)
{
	ETHHW_DMA_DESC *descp;

	NET_KNLLOG("[start net txClean]");

	/* Update Tx DMA to free socket buffers from previous Tx */
	while (dmap->tx.countFree != dmap->tx.countAlloc) {
		/* Tail should contain the start of completed Tx transfers.  For each
		*  of these completed transfers, free the associated buffers
		*/

		descp = TX_DESC(dmap->tx.descBufp, dmap->tx.tail);
		if (ETHHW_DMA_MTP_TRANSFER_DONE(descp)) {
			freeFunc((void *)descp->rsvd);
			dmap->tx.tail++;
			dmap->tx.countFree++;
			dmap->tx.count++;
		} else {
			/* Last Tx still in progress */
         NET_KNLLOG("[stop net txClean]");
			return 0;
		}
	}

	/* Last Tx completed, so start from beginning of Tx descriptor chain */
	dmap->tx.tail = 0;
	dmap->tx.index = 0;
	dmap->txCfg.index = 0;

	NET_KNLLOG("[stop net txClean]");
	return 1;
}

static void txWork(unsigned long data)
{
	if (txClean(&txPrivp->dma)) {
		NET_KNLLOG("[start net tx]");
		tx(txPrivp);
		NET_KNLLOG("[stop net tx]");
	} else {
		/* Defer Tx until last Tx DMA transaction has completed */
	}
}

static void ethStatusWork(struct work_struct *work)
{
	static int portPrev = -1;
	static uint32_t impSpeedPrev = -1;

	BCMRING_PRIV *privp;
	int port;
	int up;
	int change;
	uint32_t impSpeed;

	(void)work;

	/* Check link state of LAN port */
	privp = netdev_priv(localDevp);
	port = privp->eth.phyPort;

	impSpeed = ethHw_portSpeed(port);

	if ((impSpeed != impSpeedPrev) || (port != portPrev)) {
		msg_printk(MSG_LEVEL_STATUS_VERBOSE,
		           "Changing internal port Rx speed to %i\n", impSpeed);
		ethHw_impSpeedSet(impSpeed, 1000);
		impSpeedPrev = impSpeed;
		portPrev = port;
	}

	change = ethHw_portLinkStatusChange(port);

	if (change || (port != portPrev)) {
		portPrev = port;

		/* Link status has changed */
		up = ethHw_portLinkStatus(port);

		if (up || (port != portPrev)) {
			msg_printk(MSG_LEVEL_STATUS_VERBOSE, "link up\n");
			netif_carrier_on(localDevp);
		} else {
			msg_printk(MSG_LEVEL_STATUS_VERBOSE, "link down\n");
			netif_carrier_off(localDevp);
		}
		portPrev = port;
	}

	queue_delayed_work(ethStatusWorkQueuep, &ethStatusTask,
	                   msecs_to_jiffies(sysCtlStatusPoll));
}

static void tx(BCMRING_PRIV *privp)
{
	char *bufp;
	int len;
	dma_addr_t dmaBuf;
	uint64_t *cfgBufp;
	int dmaLen;
	struct sk_buff *skb;
	ETHHW_DMA_DESC *descp;
	ETHHW_DMA_DESC *handlep;
	size_t fifoLen;
	int portMask;
	int offset;
	int tc;
	int minLen;

	descp = NULL;

	/* Build descriptor chain */
	while ((fifoLen = __kfifo_len(privp->dma.txFifop)) > 0) {
		/* Keep track of worst-case Tx FIFO usage */
		len = fifoLen / sizeof(struct sk_buff *);
		if (len > sysCtlTxFifoMax)
			sysCtlTxFifoMax = len;

		/* Get data from Tx FIFO */
		kfifo_get(privp->dma.txFifop, (unsigned char *)&skb,
		          sizeof(struct sk_buff *));
		bufp = skb->data;
		len = skb->len;

		if( isVLANtagged(bufp) ) {
			minLen = MIN_TAGGED_FRAME_LEN;
		} else {
			minLen = MIN_UNTAGGED_FRAME_LEN;
		}

		/* Adjust length to meet minimum size requirements */
		if (len < minLen) {
			int padLen;

			padLen = minLen - len + ETHHW_BRCM_HDR_LEN;

			/* Allocate additional space with zero padding */
			if (unlikely(skb_pad(skb, padLen)) != 0) {
				/* skb is freed by skb_pad() on an error */
				myPrintk(KERN_WARNING "Unable to pad socket buffer to meet "
				         "minimum length requirements\n");

				continue;
			}

			/* Update buffer and length */
			bufp = skb->data;
			len = minLen;
		}

		/* Process buffer (frame may require a Broadcom field to be inserted) */
		if (unlikely(skb_headroom(skb) < ETHHW_BRCM_HDR_LEN)) {
			/* This case should not happen since we initialized the net device to
			*  increase dev->hard_header_len to account for the Broadcom field
			*/
			dev_kfree_skb_any(skb);
			myPrintk(KERN_ERR "Tx function requires %i bytes of headroom in "
			         "socket buffer\n", ETHHW_BRCM_HDR_LEN);

			continue;
		}

		FRAME_DUMP(MSG_LEVEL_TX, "Tx", bufp, len);

		/* Allow switch ARL to make forwarding decision */
		portMask = -1;

		if (isDaMcast(bufp)) {
			/* Force all multicast frames from the CPU to both external ports.
			*  Some protocols such as EAPOL and LLDP are terminated at each hop
			*  so they will be discarded by the switch unless we format the frame
			*  to bypass the switch ARL forwarding rules
			*/
			portMask = ETHHW_PORT_MASK(ETHHW_PORT_1) |
			           ETHHW_PORT_MASK(ETHHW_PORT_0);
		}

		/* Format the frame according to ASIC requirements and adjust buffer */
		if ((bufp[12] == 0x81) && (bufp[13] == 0x00) ) {
			/* Frame is 802.1pQ tagged so match traffic class with PCP */
			tc = (bufp[14] >> 5) & 0x7;
		}
		else {
			/* Use default traffic class */
			tc = TRAFFIC_CLASS;
		}
		offset = ethHw_txFrameFormat(&bufp, &len, portMask, tc, ETHHW_TE_FOLLOW);
		skb_push(skb, offset);

		FRAME_DUMP(MSG_LEVEL_TX_RAW, "Tx Raw", bufp, len);

		/* Insert buffer header used to identify the sk_buff when freeing the
		*  socket buffer
		*/
		SKB_BUF_HDR(bufp) = skb;

		/* Get Tx config buffer and configure */
		cfgBufp = TXCFG_BUF(privp->dma.txCfg.bufp, privp->dma.txCfg.index);

		privp->dma.txCfg.index++;

#if TXCFG_SIZE == 16
		/* Keep on 64-bit boundary */
		cfgBufp = (uint64_t *)((uint8_t *)cfgBufp + 8);
#else
		cfgBufp = (uint64_t *)((uint8_t *)cfgBufp + sizeof(struct sk_buff *));
#endif
		SKB_BUF_HDR(cfgBufp) = (struct sk_buff *)NULL;

		*cfgBufp = ETHHW_DMA_CFG_OFFSET(bufp, len) | ETHHW_DMA_CFG_EOP_MASK;

		/* Map Tx config buffer to update external memory */
		dmaBuf = dma_map_single(NULL, cfgBufp, sizeof(uint64_t),
		                        DMA_TO_DEVICE);
		dmaLen = ETHHW_DMA_MTP_TRANSACTION_SIZE(dmaBuf, sizeof(uint64_t));

		/* Send config to Tx DMA */
		descp = TX_DESC(privp->dma.tx.descBufp, privp->dma.tx.index);
		handlep = TX_DESC(privp->dma.tx.handle, privp->dma.tx.index);
		privp->dma.tx.index++;

		/* Tx config descriptor
		*  The config descriptor allows transfer to be purely 64-bit
		*  transactions, spanning an arbitrary number of descriptors,
		*  so information must be provided to define offsets and EOP
		*/
		ETHHW_DMA_DESC_CREATE(descp,
		                      dmaBuf,
		                      ETHHW_DMA_MTP_FIFO_ADDR_CFG,
		                      handlep,
		                      ETHHW_DMA_MTP_CTL_LO,
		                      dmaLen);

		/* Save virtual address of buffer */
		descp->rsvd = (uint32_t) cfgBufp;

		/* Map data buffer to update external memory */
		dmaBuf = dma_map_single(NULL, bufp, len, DMA_TO_DEVICE);

		/* Make length a multiple of 64-bits and put data buffer on 64-bit boundary */
		dmaLen = ETHHW_DMA_MTP_TRANSACTION_SIZE(dmaBuf, len);
		dmaBuf &= 0xfffffff8;

		/* Send data to Tx DMA */
		descp = TX_DESC(privp->dma.tx.descBufp, privp->dma.tx.index);
		handlep = TX_DESC(privp->dma.tx.handle, privp->dma.tx.index);
		privp->dma.tx.index++;

		/* Tx config descriptor
		*  The config descriptor allows transfer to be purely 64-bit
		*  transactions, spanning an arbitrary number of descriptors,
		*  so information must be provided to define offsets and EOP
		*/
		ETHHW_DMA_DESC_CREATE(descp,
		                      dmaBuf,
		                      ETHHW_DMA_MTP_FIFO_ADDR_DATA,
		                      handlep,
		                      ETHHW_DMA_MTP_CTL_LO,
		                      dmaLen);

		/* Save virtual address of buffer */
		descp->rsvd = (uint32_t) bufp;

		privp->stat.tx_packets++;
		privp->stat.tx_bytes += (len - ETHHW_BRCM_HDR_LEN);

		privp->dma.tx.countAlloc += 2;
		privp->dma.tx.count -= 2;
	}

	/* Start Tx DMA transfer */
	if (descp) {
		ETHHW_DMA_DESC_TX_DONE(descp);
		handlep = TX_DESC(privp->dma.tx.handle, 0);
		ethHw_dmaIrqEnable(DMA_ESW_MODULE, DMA_ESW_CH_TX);
		ethHw_dmaEnable(DMA_ESW_MODULE, DMA_ESW_CH_TX, handlep);
	}
}

static int bcmring_net_hard_start_xmit(struct sk_buff *skb,
                                       struct net_device *dev)
{
	BCMRING_PRIV *privp;
	int len;
	privp = netdev_priv(dev);

	len = sizeof(struct sk_buff *);
	if (__kfifo_put(privp->dma.txFifop, (unsigned char *)&skb, len) != len) {
		/* Not enough space in Tx DMA to send frame */
		myPrintk(KERN_INFO "Ethernet DMA Tx FIFO full\n");
		netif_stop_queue(dev);

		return NETDEV_TX_BUSY;
	}

	dev->trans_start = jiffies;

	if (ethHw_dmaIsBusy(DMA_ESW_MODULE, DMA_ESW_CH_TX)) {
		/* Tx DMA transfer in progress, so just leave frame in Tx FIFO.  Another
		*  attempt to send the frame will be made when the DMA interrupt signals
		*  the completion of the current DMA transfer
		*/
	} else {
      tasklet_schedule(&txTasklet);
   }

	return NETDEV_TX_OK;
}

static int bcmring_net_poll(struct napi_struct *napi, int budget)
{
	BCMRING_PRIV *privp;
	struct sk_buff *skb;
	char *bufp;
	char *newBufp;
	int offset;
	int len;
	int quota;
	int rxCount;
	ETHHW_DMA_DESC *descp;
	dma_addr_t dmaBuf;

	struct net_device *dev;
	privp = container_of(napi, BCMRING_PRIV, napi);
	dev = privp->mii_if.dev;
	quota = budget;
	rxCount = 0;

	while (rxCount < quota) {
		/* Get next frame from Rx DMA */
		descp = RX_DESC(privp->dma.rx.descBufp, privp->dma.rx.index);
		if (ETHHW_DMA_PTM_TRANSFER_DONE(descp)) {
			bufp = (char *)descp->rsvd;
			len = ETHHW_DMA_BUF_LEN(descp);

			/* Update descriptor with new buffer */
			newBufp = (uint8_t *)allocFuncp(DMA_RX_BUF_LEN);
			if (!newBufp) {
				myPrintk(KERN_ERR "Failed to allocate Rx DMA buffer\n");
				break;
			}

			/* Map data buffer to invalidate cache */
			dmaBuf = dma_map_single(NULL, newBufp, DMA_RX_BUF_LEN,
			                        DMA_FROM_DEVICE);

			/* Update descriptor */
			ETHHW_DMA_DESC_RX_UPDATE(descp, dmaBuf, DMA_RX_BUF_LEN);

			/* Save virtual address of buffer */
			descp->rsvd = (uint32_t) newBufp;

			/* Go to next descriptor */
			if (++privp->dma.rx.index >= DMA_RX_DESC_NUM) {
				privp->dma.rx.index = 0;
			}
		} else {
			/* No frame available */

			/* See if sync-up is required */
			{
				ETHHW_DMA_DESC *dp;
				int index;
				int sync;

				sync = 0;
				index = privp->dma.rx.index;
				if( ++index >= DMA_RX_DESC_NUM )
				{
					index = 0;
				}

				while( index != privp->dma.rx.index )
				{
					dp = RX_DESC(privp->dma.rx.descBufp, index);
					if( ETHHW_DMA_PTM_TRANSFER_DONE( dp ) )
					{
						/* Sync-up required */
						sync = 1;
						break;
					}

					if( ++index >= DMA_RX_DESC_NUM )
					{
						index = 0;
					}
				}

				if( unlikely( sync ) )
				{
					privp->dma.rx.index = index;
					continue;
				}
			}

			break;
		}

		/* Unmap buffer */
		dma_unmap_single(NULL, (dma_addr_t) bufp, len, DMA_FROM_DEVICE);

		/* Update Rx DMA to reallocate a buffer for the descriptor ring */
		privp->dma.rx.count++;

		/* Get socket buffer information from Rx DMA buffer */
		skb = SKB_BUF_HDR(bufp);

		FRAME_DUMP(MSG_LEVEL_RX_RAW, "Rx Raw", bufp, len);

		/* Format the frame according to ASIC requirements and adjust buffer */
		if (BRCM_ENCAP_MODE == BRCM_ENCAP_MODE_REMOVE) {
			offset = ethHw_rxFrameFormat(&bufp, &len);
		} else {
			offset = ethHw_rxFrameFormatRaw(&bufp, &len);
		}
		skb_reserve(skb, offset);

		FRAME_DUMP(MSG_LEVEL_RX, "Rx", bufp, len);

		if (unlikely(len < MIN_UNTAGGED_FRAME_LEN)) {
			/* Frame size is invalid, so discard */
			FRAME_DUMP_ALWAYS( KERN_WARNING "Invalid frame size received\n", bufp, len);
			dev_kfree_skb_any(skb);

			privp->stat.rx_dropped++;

			continue;
		}

	 /* Check number of UDP and TCP packets received within last period */
     /* If it exceeds the maximum, discard packets  */
	 /* Checking: Ethernet type = 0x0800 (IP), IP version = 4 . If all checks out, then Protocol = 0x11 (UDP)  or 0x06 (TCP) */
     if ( RATE_LIMIT_ENABLE && (bufp[12] == 0x08) && (bufp[13] == 0x00) && ((bufp[14] & 0xf0) == 0x40) )  /* If IPv4  */
     {
	    if (bufp[23] == 0x11) /* UDP packet */
		{
           if (rateLimitUdpLastJiffies != jiffies)
           {
               /* DOS detection - Enabled if RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES is non-zero*/
               if (RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES)
               {
                   /* reset consecutive jiffies if there was no UDP packet dropped in last jiffy. */
                   if (!rateLimitDiscardedUdpPacketsWithinJiffy)
                   {
                       rateLimitConsecutiveJiffiesUdpPacketDropped = 0;
                   }

                   if (rateLimitConsecutiveJiffiesUdpPacketDropped >= RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES)
                   {
                       RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY = RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY_DOS;
                       sysCtlUdpRateLimitThreshold = RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY;
                   }
                   else
                   {
                       RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY = RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY_NORMAL;
                       sysCtlUdpRateLimitThreshold = RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY;
                   }

                   /* reset discarded packet count for next jiffy */
                   rateLimitDiscardedUdpPacketsWithinJiffy = 0;
               }

               rateLimitUdpLastJiffies = jiffies;
               rateLimitUdpPacketCount = 0;
           }

           if (rateLimitUdpPacketCount < RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY)
		   {
               rateLimitUdpPacketCount++;
			   sysCtlUdpRateLimitPassed++;
		   }
           else /* drop the UDP packet */
           {
               dev_kfree_skb_any(skb);
               privp->stat.rx_dropped++;
			   sysCtlUdpRateLimitDropped++;

               /* DOS detection - Enabled if RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES is non-zero*/
               if (RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES)
               {
                   rateLimitDiscardedUdpPacketsWithinJiffy++;
                   if (rateLimitLastUdpPacketDropJiffies != jiffies)
                   {
                       rateLimitLastUdpPacketDropJiffies = jiffies;
                       if (rateLimitConsecutiveJiffiesUdpPacketDropped < RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES)
                       {
                           /* Don't go higher than needed to avoid potential value wrap around */
                           rateLimitConsecutiveJiffiesUdpPacketDropped++;
                       }
                   }
               }

               continue;
           }
        }
        else if ((bufp[23] & 0x06 ) == 0x06 ) /* TCP packet */
        {
           if (rateLimitTcpLastJiffies != jiffies)
           {
               /* DOS detection - Enabled if RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES is non-zero*/
               if (RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES)
               {
                  /* reset consecutive jiffies if there was no TCP packet dropped in last jiffy */
                   if (!rateLimitDiscardedTcpPacketsWithinJiffy)
                   {
                       rateLimitConsecutiveJiffiesTcpPacketDropped = 0;
                   }

                   if (rateLimitConsecutiveJiffiesTcpPacketDropped >= RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES)
                   {
                       RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY = RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY_DOS;
                       sysCtlTcpRateLimitThreshold = RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY;
                   }
                   else
                   {
                       RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY = RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY_NORMAL;
                       sysCtlTcpRateLimitThreshold = RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY;
                   }

                   /* reset discarded packet count for next jiffy */
                   rateLimitDiscardedTcpPacketsWithinJiffy = 0;
               }

               rateLimitTcpLastJiffies = jiffies;
               rateLimitTcpPacketCount = 0;
           }
           if (rateLimitTcpPacketCount < RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY)
           {
               rateLimitTcpPacketCount++;
			   sysCtlTcpRateLimitPassed++;
		   }
           else /* drop the TCP packet */
           {
               dev_kfree_skb_any(skb);
               privp->stat.rx_dropped++;
			   sysCtlTcpRateLimitDropped++;

               /* DOS detection - Enabled if RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES is non-zero*/
               if (RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES)
               {
                   rateLimitDiscardedTcpPacketsWithinJiffy++;
                   if (rateLimitLastTcpPacketDropJiffies != jiffies)
                   {
                       rateLimitLastTcpPacketDropJiffies = jiffies;
                       if (rateLimitConsecutiveJiffiesTcpPacketDropped < RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES)
                       {
                          /* Don't go higher than needed to avoid potential value wrap around */
                           rateLimitConsecutiveJiffiesTcpPacketDropped++;
                       }
                   }
               }

               continue;
           }
        }
     }

		/* Update remainder of socket buffer information */
		skb_put(skb, len);
		skb->dev = dev;
		skb->protocol = eth_type_trans(skb, dev);

		if (BRCM_ENCAP_MODE == BRCM_ENCAP_MODE_KEEP_SKIP) {
			__brcmPacketTypeHandler(skb);
		}

		skb->ip_summed = CHECKSUM_NONE;
#if 0
		if (unlikely(sysCtlRxDiscard)) {
			/* Discard for debugging purposes */
			dev_kfree_skb_any(skb);
			continue;
		}
#endif

		/* Forward socket buffer to kernel for processing by IP stack */
		netif_receive_skb(skb);

		/* Update counts and stats */
		rxCount++;

		privp->stat.rx_packets++;
		if (BRCM_ENCAP_MODE == BRCM_ENCAP_MODE_REMOVE) {
			privp->stat.rx_bytes += len;
		} else {
			/* Report length of Rx data that was received over the wire, so
			*  account for Broadcom type field
			*/
			privp->stat.rx_bytes += (len - ETHHW_BRCM_HDR_LEN);
		}
	}

	if (rxCount > sysCtlRxFifoMax)
		sysCtlRxFifoMax = rxCount;

	if (likely(rxCount < quota))
	{
		/* Notify kernel that no more Rx frames are available */
		napi_complete(napi);

		/* Acknowledge and enable Rx DMA interrupt */
		ethHw_dmaIrqClear(DMA_ESW_MODULE, DMA_ESW_CH_RX);
		ethHw_dmaIrqEnable(DMA_ESW_MODULE, DMA_ESW_CH_RX);
	}

	return rxCount;
}

static void bcmring_net_set_multicast_list(struct net_device *dev)
{
	/* TODO:  Add multicast list support */
}

static int bcmring_net_set_mac_address(struct net_device *dev, void *addr)
{
	BCMRING_PRIV *privp;
	unsigned char *macp;

	privp = netdev_priv(dev);

	macp = ((struct sockaddr *)addr)->sa_data;
	ethHw_arlEntrySet(macp, 0, ETHHW_PORT_INT, 0, 1, 1);

	memcpy(dev->dev_addr, macp, dev->addr_len);

	return 0;
}

static struct net_device_stats *bcmring_net_get_stats(struct net_device *dev)
{
	BCMRING_PRIV *privp;
	struct net_device_stats *statp;

	privp = netdev_priv(dev);
	statp = &privp->stat;

	return statp;
}

static void bcmring_net_tx_timeout(struct net_device *dev)
{
	BCMRING_PRIV *privp;

	privp = netdev_priv(dev);

	netif_wake_queue(dev);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void bcmring_net_poll_controller(struct net_device *dev)
{
	BCMRING_PRIV *privp;

	privp = netdev_priv(dev);

	/* TODO:  Add polling support */
}
#endif

#if PHY_DEVICE_SUPPORT
static int bcmring_net_create_phy_if(int index, struct net_device **dev)
{
	struct net_device *phyDev;
	BCMRING_PRIV *privp;
	int rc;
	char name[] = "phy_";

	if (index > 9) {
		myPrintk(KERN_ERR "Can only support upto 9 PHY devices\n");
		return -EFAULT;
	}
	
	phyDev = alloc_etherdev(sizeof(BCMRING_PRIV));
	if (!phyDev) {
		myPrintk(KERN_ERR "Failed to alloc PHY device\n");
		return -ENOMEM;
	}

	privp = netdev_priv(phyDev);

	/* Map port directly to supplied index */
	privp->eth.phyPort = index;

	/* Build device name, ie phy0, phy1, etc */
	name[3] = '0' + index;
	strcpy(phyDev->name, name);

	/* Assign device ops */
	phyDev->ethtool_ops = &bcmring_ethtool_ops;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
	/* Not needed, but added to prevent kernel warnings in console */
	phyDev->netdev_ops = &phy_netdev_ops;
#endif

	rc = register_netdev(phyDev);
	if (rc) {
		myPrintk(KERN_ERR "Failed to register PHY device\n");
		free_netdev(phyDev);
		return rc;
	}

	*dev = phyDev;

	return 0;
}
#endif

static int bcmring_net_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	BCMRING_PRIV *privp;
	int rc;
	int reg;
	int i;

	myPrintk(KERN_INFO "bcmring_net configuration: brcmMode=%i, descRx=%i, descTx=%i\n",
	       brcmMode, descRx, descTx);

	/* Initialize resources required by the driver */
	dev = alloc_etherdev(sizeof(BCMRING_PRIV));
	if (!dev) {
		myPrintk(KERN_ERR "Failed to alloc device\n");

		return -ENOMEM;
	}

	privp = netdev_priv(dev);
	platform_set_drvdata(pdev, dev);

	/* Create copy of driver handle for proc interface */
	procPrivp = privp;
	localDevp = dev;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
	dev->netdev_ops = &bcmring_netdev_ops;
#else
	dev->get_stats = bcmring_net_get_stats;
	dev->open = bcmring_net_open;
	dev->stop = bcmring_net_close;
	dev->hard_start_xmit = bcmring_net_hard_start_xmit;
	dev->set_mac_address = bcmring_net_set_mac_address;
	dev->do_ioctl = bcmring_net_do_ioctl;
	dev->tx_timeout = bcmring_net_tx_timeout;
#endif
	dev->watchdog_timeo = TX_TIMEOUT;
	netif_napi_add(dev, &privp->napi, bcmring_net_poll, 64);
#ifdef CONFIG_NET_BCMRING_ETHTOOL
	dev->ethtool_ops = &bcmring_ethtool_ops;
#endif
#ifdef CONFIG_NET_POLL_CONTROLLER
	dev->poll_controller = bcmring_net_poll_controller;
#endif
	/* Assign Rx interrupt even though both Rx and Tx interrupts are required.
	*  This is for informational purposes only
	*/
	dev->irq = IRQ_DMA_ESW_RX;
	dev->hard_header_len += ETHHW_BRCM_HDR_LEN;
	dev->features &= ~(NETIF_F_SG | NETIF_F_FRAGLIST);
	dev->tx_queue_len = DMA_TX_DESC_NUM;

	rc = register_netdev(dev);
	if (rc) {
		myPrintk(KERN_ERR "Failed to register device\n");
		free_netdev(dev);

		return rc;
	}

	/* Start ethernet block.  This should be done as early as possible to allow
	*  the switch to begin bridging frames on the external ports
	*/
	rc = ethStart(&privp->eth);
	if (rc) {
		myPrintk(KERN_ERR "Failed to start ethernet block\n");
		free_netdev(dev);

		return rc;
	}

	/* Clear stats */
	memset(&privp->stat, 0, sizeof(privp->stat));

	/* Initialize MII support */
	privp->mii_if.phy_id = privp->eth.phyPort;
	privp->mii_if.phy_id_mask = ETHHW_PORT_NUM - 1;    /* ETHHW_PORT_NUM must be 2^x */
	privp->mii_if.reg_num_mask = 32 - 1;               /* Number of MII registers (32 per spec) */
	privp->mii_if.dev = dev;
	privp->mii_if.mdio_read = miiGet;
	privp->mii_if.mdio_write = miiSet;

	reg = ethMiiGet(privp->mii_if.phy_id, MII_ADVERTISE);
	privp->mii_if.advertising = reg;

	reg = ethMiiGet(privp->mii_if.phy_id, 24);
	privp->mii_if.full_duplex = ((reg & 0x0001) ? 1 : 0);

	reg = ethMiiGet(privp->mii_if.phy_id, MII_BMCR);
	privp->mii_if.force_media = ((reg & BMCR_ANENABLE) ? 0 : 1);

	/* Get chip configuration */
	privp->chip.portNum = 0;
	privp->chip.portNumExt = 0;

	for (i = ETHHW_PORT_MIN; i <= ETHHW_PORT_MAX; i++) {
		privp->chip.phyAvailable[i] = cap_isPresent(CAP_ETH_PHY, i);
		privp->chip.gmiiAvailable[i] = cap_isPresent(CAP_ETH_GMII, i);
		privp->chip.sgmiiAvailable[i] = cap_isPresent(CAP_ETH_SGMII, i);

		if (privp->chip.phyAvailable[i] == CAP_PRESENT)
			privp->chip.portNum++;

		if (privp->chip.sgmiiAvailable[i] == CAP_PRESENT)
			privp->chip.portNumExt++;
	}

	if (privp->chip.portNum == 0) {
		myPrintk(KERN_ERR "No ethernet ports available\n");
		return -EFAULT;
	}

#if PHY_DEVICE_SUPPORT
	for (i=0; i<PHY_DEV_NUM; i++) {
		privp->phyDev[i] = NULL;
	}

	if (privp->chip.portNumExt > 1) {
		int num;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
		/* Clear structure since it will not be used by the /phy devices */
		memset((void *)&phy_netdev_ops, 0, sizeof(struct net_device_ops));
#endif

		/* At least two external ports exist so create /phy devices
		*  for ethtool.  This will give the user the ability to configure
		*  each external port with ethtool.  The mapping is as follows:
		*
		*  phy0 - ethernet port 0
		*  phy1 - ethernet port 1
		*  eth0 - PHY port (typically the LAN port, but is user defined)
		*/
		if (privp->chip.portNumExt > ETHHW_PORT_NUM) {
			myPrintk(KERN_WARNING "Detected %i ports, but only upto %i are "
			         "supported", privp->chip.portNumExt, ETHHW_PORT_NUM);
			num = ETHHW_PORT_NUM;
		}
		else
		{
			num = privp->chip.portNumExt;
		}

			for (i=0; i<num; i++) {
				bcmring_net_create_phy_if(i, &privp->phyDev[i]);
			}
	}
#endif

	return 0;
}

static int bcmring_net_remove(struct platform_device *pdev)
{
	struct net_device *dev;

	dev = platform_get_drvdata(pdev);

#if PHY_DEVICE_SUPPORT
	{
		BCMRING_PRIV *privp;
		int i;

		privp = netdev_priv(dev);

		if (privp->chip.portNumExt > 1) {
			for (i=0; i<ETHHW_PORT_NUM; i++) {
				if (privp->phyDev[i]) {
					unregister_netdev(privp->phyDev[i]);
					free_netdev(privp->phyDev[i]);
				}
			}
		}
	}
#endif

	unregister_netdev(dev);
	free_netdev(dev);

	return 0;
}

static void bcmring_net_shutdown(struct platform_device *pdev)
{
	(void)pdev;

	/* Disable forwarding to CPU from internal port */
	ethHw_macEnableSet(ETHHW_PORT_INT, 0, 0);
	ethHw_impEnableSet(0);
}

#ifdef CONFIG_PM
static int bcmring_net_notify_reboot(struct notifier_block *nb,
                                     unsigned long event, void *ptr)
{
	(void)nb;
	(void)ptr;

	switch (event) {
	case SYS_DOWN:
	case SYS_HALT:
	case SYS_POWER_OFF:
		{
			/* Disable forwarding to CPU from internal port */
			ethHw_macEnableSet(ETHHW_PORT_INT, 0, 0);
			ethHw_impEnableSet(0);
		}
		break;

	default:
		{
		}
		break;
	}
	return NOTIFY_DONE;
}

static int bcmring_net_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct net_device *dev;
	BCMRING_PRIV *privp;
	int net_alive;
	int disable_lan, disable_pc, disable_int;

	dev = platform_get_drvdata(pdev);
	privp = netdev_priv(dev);

	/* Backup current device state */
	privp->wasRunning = netif_running(dev);

	if (privp->wasRunning) {
		/* Shutdown network i/f */
		netif_device_detach(dev);

		/* Pause any existing work */
		cancel_delayed_work(&ethStatusTask);
		flush_workqueue(ethStatusWorkQueuep);
	}

	net_alive   = (pm_net_get_state() == pm_state_alive);
	disable_lan = net_alive ? sysCtlPmDisablePortLan : 1;
	disable_pc  = net_alive ? sysCtlPmDisablePortPc : 1;
	disable_int = net_alive ? sysCtlPmDisablePortInternal : 1;

	/* Disable MAC Rx */
	if (disable_lan) {
		ethHw_macEnableSet(PORT_LAN, 0, 1);
	}
	if ((CHIP_PORTS(privp) > 1) && disable_pc) {
		ethHw_macEnableSet(PORT_PC, 0, 1);
	}

	/* Let Tx DMA drain */
	if (privp->wasRunning) {
		while (__kfifo_len(privp->dma.txFifop) != 0)
		;
	}

	/* Disable MAC Tx as well */
	if (disable_lan) {
		ethHw_macEnableSet(PORT_LAN, 0, 0);
	}
	if ((CHIP_PORTS(privp) > 1) && disable_pc) {
		ethHw_macEnableSet(PORT_PC, 0, 0);
	}

	/* Disable internal port */
	if (disable_int) {
		ethHw_macEnableSet(ETHHW_PORT_INT, 0, 0);
	}

	/* Power-down PHY(s) */
	if (disable_lan) {
		ethHw_phyPowerdownSet(PORT_LAN, 1);
	}
	if ((CHIP_PORTS(privp) > 1) && disable_pc) {
		ethHw_phyPowerdownSet(PORT_PC, 1);
	}

	/* Disable switch clock if PHYs are powered down */
	if ((disable_lan && (CHIP_PORTS(privp) == 1)) ||
	    (disable_lan && disable_pc && (CHIP_PORTS(privp) > 1))) {
		chipcHw_setClockDisable(chipcHw_CLOCK_ESW);
		chipcHw_busInterfaceClockDisable(chipcHw_REG_BUS_CLOCK_ESW);
	}

	return 0;
}

static int bcmring_net_resume(struct platform_device *pdev)
{
	struct net_device *dev;
	BCMRING_PRIV *privp;
	int net_alive;
	int disable_lan, disable_pc, disable_int;

	dev = platform_get_drvdata(pdev);
	privp = netdev_priv(dev);

	net_alive   = (pm_net_get_state() == pm_state_alive);
	disable_lan = net_alive ? sysCtlPmDisablePortLan : 1;
	disable_pc  = net_alive ? sysCtlPmDisablePortPc : 1;
	disable_int = net_alive ? sysCtlPmDisablePortInternal : 1;

	/* Enable switch clock if necessary */
	if ((disable_lan && (CHIP_PORTS(privp) == 1)) ||
	    (disable_lan && disable_pc && (CHIP_PORTS(privp) > 1))) {
		chipcHw_setClockEnable(chipcHw_CLOCK_ESW);
		chipcHw_busInterfaceClockEnable(chipcHw_REG_BUS_CLOCK_ESW);
	}

	/* Power-up the PHY(s) */
	if (disable_lan) {
		ethHw_phyPowerdownSet(PORT_LAN, 0);
	}
	if ((CHIP_PORTS(privp) > 1) && disable_pc) {
	ethHw_phyPowerdownSet(PORT_PC, 0);
	}

	/* Enable internal port */
	if (disable_int) {
		ethHw_macEnableSet(ETHHW_PORT_INT, 1, 1);
	}

	/* Enable external port */
	if (disable_lan) {
		ethHw_macEnableSet(PORT_LAN, 1, 1);
	}
	if ((CHIP_PORTS(privp) > 1) && disable_pc) {
		ethHw_macEnableSet(PORT_PC, 1, 1);
	}

	if (privp->wasRunning) {
		/* Resume any existing work */
		queue_delayed_work(ethStatusWorkQueuep, &ethStatusTask, 0);

		/* Restore network i/f */
		netif_device_attach(dev);
	}

	return 0;
}
#endif

/*--------------------------------------------------------------------------*/

static struct platform_driver bcmring_net_driver = {
	.driver = {
	            .name = "bcmring-net",
	            .owner = THIS_MODULE,
	          },
	.probe = bcmring_net_probe,
	.remove = bcmring_net_remove,
	.shutdown = bcmring_net_shutdown,
	.suspend = bcmring_net_suspend,
	.resume = bcmring_net_resume,
};

static int __init bcmring_net_init(void)
{
	myPrintk(KERN_INFO "%s\n", banner);
#ifdef CONFIG_PM
	register_reboot_notifier(&bcmring_net_notifier_reboot);
#endif
	/* Calculate the rate limits in units of packet per jiffy. Just do that once */
	RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY_NORMAL = RATE_LIMIT_UDP_THRESHOLD_NORMAL_PPS / HZ;
	RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY_NORMAL = RATE_LIMIT_TCP_THRESHOLD_NORMAL_PPS / HZ;
	RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY_DOS = RATE_LIMIT_UDP_THRESHOLD_DOS_PPS / HZ;
	RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY_DOS = RATE_LIMIT_TCP_THRESHOLD_DOS_PPS / HZ;
	RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY = RATE_LIMIT_UDP_PACKET_THRESHOLD_PER_JIFFY_NORMAL;
	RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY = RATE_LIMIT_TCP_PACKET_THRESHOLD_PER_JIFFY_NORMAL;
	RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES = msecs_to_jiffies(RATE_LIMIT_DOS_DET_PERIOD_MS);

	/* Print the rate limiting module params so we know they are correct */
	myPrintk(KERN_INFO "bcmring_net => RL: Packet based rate limit enable = %d\n", RATE_LIMIT_ENABLE);
	if ( RATE_LIMIT_ENABLE )
	{
		myPrintk(KERN_INFO "bcmring_net => RL: Normal UDP threshold = %d PPS\n", RATE_LIMIT_UDP_THRESHOLD_NORMAL_PPS);
		myPrintk(KERN_INFO "bcmring_net => RL: Normal TCP threshold = %d PPS\n", RATE_LIMIT_TCP_THRESHOLD_NORMAL_PPS);
		myPrintk(KERN_INFO "bcmring_net => RL: DOS UDP threshold = %d PPS\n", RATE_LIMIT_UDP_THRESHOLD_DOS_PPS);
		myPrintk(KERN_INFO "bcmring_net => RL: DOS TCP threshold = %d PPS\n", RATE_LIMIT_TCP_THRESHOLD_DOS_PPS);
		myPrintk(KERN_INFO "bcmring_net => RL: DOS Det period = %d ms | %d jiffies \n",
               RATE_LIMIT_DOS_DET_PERIOD_MS, RATE_LIMIT_DOS_DET_PERIOD_IN_JIFFIES);
	}

	return platform_driver_register(&bcmring_net_driver);
}

static void __exit bcmring_net_exit(void)
{
#ifdef CONFIG_PM
	unregister_reboot_notifier(&bcmring_net_notifier_reboot);
#endif
	platform_driver_unregister(&bcmring_net_driver);
}

module_init(bcmring_net_init);
module_exit(bcmring_net_exit);

MODULE_DESCRIPTION(BCM_NET_MODULE_DESCRIPTION);
MODULE_LICENSE("GPL");
MODULE_VERSION(BCM_NET_MODULE_VERSION);
