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

#ifndef CONFIG_NET_BCMRING_ETHTOOL
/* Ideally, bcmring_ethtool.c is compiled separately and linked with
*  bcmring_net.c, however, the build env will create a separate module.  For
*  now, this file is simply included in bcmring_net.c
*/
#error This file is meant to be included by network wrapper file such as bcmring_net.c
#endif

#include <linux/ethtool.h>

/* The way in which ethtool dumps the register values is not easy for a
*  developer to work with.  This custom dump will print the register contents
*  from within the ethtool hook function.
*/
#define GET_REGS_CUSTOM       0

/* Maximum duration for PHY identification (in seconds)
*/
#define PHYS_ID_DURATION_MAX  30

/* Extensions to linux/mii.h */

/* Extended mode status register */
#ifndef EMSR_1000HALF
#define EMSR_1000HALF         0x0100
#endif
#ifndef EMSR_1000FULL
#define EMSR_1000FULL         0x0200
#endif

typedef char ETHTOOL_STAT_STR[ETH_GSTRING_LEN];

static const ETHTOOL_STAT_STR ethtoolStrStat[] =
{
   /* The order of the strings should not be changed since other code depends
   *  on this specific order (same as bcmring_ethtool_get_ethtool_stats())
   */

   /* TODO:  Since /phy stats can be displayed, consider adding collision
   *         counters
   */

   "Octets    <eth>|<phy>->Rx|Rx",
   "DropPkts               Rx|Rx",
   "Q0Pkt                  Rx|Tx",
   "BroadcastPkts          Rx|Rx",
   "MulticastPkts          Rx|Rx",
   "UnicastPkts            Rx|Rx",
   "PausePkts              Rx|Rx",
   "Q1Pkt                  Rx|Tx",
   "Q2Pkt                  Rx|Tx",
   "Q3Pkt                  Rx|Tx",
   "Q4Pkt                  Rx|Tx",
   "Q5Pkt                  Rx|Tx",
   "Octets                 Tx|Tx",
   "UndersizePkts          Tx|Rx",
   "PausePkts              Tx|Tx",
   "Pkts64Octets           Tx|Rx",
   "Pkts65To127Octets      Tx|Rx",
   "Pkts128To255Octets     Tx|Rx",
   "Pkts256To511Octets     Tx|Rx",
   "Pkts512To1023Octets    Tx|Rx",
   "Pkts1024ToMaxPktOctets Tx|Rx",
   "GoodOctets             Tx|Tx",
   "DropPkts               Tx|Tx",
   "UnicastPkts            Tx|Tx",
   "MulticastPkts          Tx|Tx",
   "BroadcastPkts          Tx|Tx",
   "SaChanges              Tx|Rx",
   "JumboPktCount          Tx|Rx",
   "InRangeErrorCount      Tx|Rx",
   "OutRangeErrorCount     Tx|Rx",
   "Discard                Tx|Rx",
};


static const ETHTOOL_STAT_STR ethtoolStrTest[] =
{
   "Register Access         ",
   "Internal Memory         ",
   "PHY Loopback (local)    ",
};


static u32 alwaysEnabled( struct net_device *dev );
#if 0
static u32 alwaysDisabled( struct net_device *dev );
#endif

static int bcmring_ethtool_get_settings( struct net_device *dev,
                                         struct ethtool_cmd *cmd );
static int bcmring_ethtool_set_settings( struct net_device *dev,
                                         struct ethtool_cmd *cmd );
static void bcmring_ethtool_get_drvinfo( struct net_device *dev,
                                         struct ethtool_drvinfo *info );
static int bcmring_ethtool_get_regs_len( struct net_device *dev );
static void bcmring_ethtool_get_regs( struct net_device *dev,
                                      struct ethtool_regs *regs, void *p );
static u32 bcmring_ethtool_get_msglevel( struct net_device *dev );
static void bcmring_ethtool_set_msglevel( struct net_device *dev, u32 level );
static int bcmring_ethtool_nway_reset( struct net_device *dev );
static int bcmring_ethtool_get_link( struct net_device *dev );
static void bcmring_ethtool_get_ringparam( struct net_device *dev,
                                           struct ethtool_ringparam *ering );
static int bcmring_ethtool_set_ringparam( struct net_device *dev,
                                          struct ethtool_ringparam *ering );
static void bcmring_ethtool_get_pauseparam( struct net_device *dev,
                                            struct ethtool_pauseparam *epause );
static int bcmring_ethtool_set_pauseparam( struct net_device *dev,
                                           struct ethtool_pauseparam *epause );
static int bcmring_ethtool_get_test_count( struct net_device *dev );
static void bcmring_ethtool_self_test( struct net_device *dev,
                                       struct ethtool_test *etest,
                                       u64 *data );
static void bcmring_ethtool_get_strings( struct net_device *dev,
                                         u32 strSet, u8 *buf );
static int bcmring_ethtool_phys_id( struct net_device *dev, u32 data );
static int bcmring_ethtool_get_stats_count( struct net_device *dev );
static void bcmring_ethtool_get_ethtool_stats( struct net_device *dev,
                                               struct ethtool_stats *estats,
                                               u64 *tmp_stats );

static struct ethtool_ops bcmring_ethtool_ops =
{
   .get_settings        = bcmring_ethtool_get_settings,
   .set_settings        = bcmring_ethtool_set_settings,
   .get_drvinfo         = bcmring_ethtool_get_drvinfo,
   .get_regs_len        = bcmring_ethtool_get_regs_len,
   .get_regs            = bcmring_ethtool_get_regs,
   .get_wol             = NULL,        /* Not supported */
   .set_wol             = NULL,        /* Not supported */
   .get_msglevel        = bcmring_ethtool_get_msglevel,
   .set_msglevel        = bcmring_ethtool_set_msglevel,
   .nway_reset          = bcmring_ethtool_nway_reset,
   .get_link            = bcmring_ethtool_get_link,
   .get_eeprom_len      = NULL,        /* Not supported */
   .get_eeprom          = NULL,        /* Not supported */
   .set_eeprom          = NULL,        /* Not supported */
   .get_coalesce        = NULL,        /* Not supported */
   .set_coalesce        = NULL,        /* Not supported */
   .get_ringparam       = bcmring_ethtool_get_ringparam,
   .set_ringparam       = bcmring_ethtool_set_ringparam,
   .get_pauseparam      = bcmring_ethtool_get_pauseparam,
   .set_pauseparam      = bcmring_ethtool_set_pauseparam,
   .get_rx_csum         = alwaysEnabled,
   .set_rx_csum         = NULL,        /* Not supported */
   .get_tx_csum         = alwaysEnabled,
   .set_tx_csum         = NULL,        /* Not supported */
   .get_sg              = NULL,        /* Not supported */
   .set_sg              = NULL,        /* Not supported */
   .get_tso             = NULL,        /* Not supported */
   .set_tso             = NULL,        /* Not supported */
   .self_test_count     = bcmring_ethtool_get_test_count,
   .self_test           = bcmring_ethtool_self_test,
   .get_strings         = bcmring_ethtool_get_strings,
   .phys_id             = bcmring_ethtool_phys_id,
   .get_stats_count     = bcmring_ethtool_get_stats_count,
   .get_ethtool_stats   = bcmring_ethtool_get_ethtool_stats,
   .begin               = NULL,        /* Not supported */
   .complete            = NULL,        /* Not supported */
   .get_ufo             = ethtool_op_get_ufo,
   .set_ufo             = ethtool_op_set_ufo,
};


/* TODO: Add proper register defs */
#define REG_LED_MODE_MAP_0       *((uint16_t *)MM_IO_PHYS_TO_VIRT( 0x304000c0 ))
#define REG_LED_MODE_MAP_1       *((uint16_t *)MM_IO_PHYS_TO_VIRT( 0x304000d0 ))

typedef enum
{
   PHY_MODE_INTERNAL,
   PHY_MODE_EXTERNAL,
} PHY_MODE_t;

#define ethHw_phy_mode_get(n)    PHY_MODE_INTERNAL
#define ethHw_phy_mode_set(n,t)  /* Do nothing for now */


static u32 alwaysEnabled( struct net_device *dev )
{
   (void)dev;

   return( 1 );
}


#if 0
static u32 alwaysDisabled( struct net_device *dev )
{
   (void)dev;

   return( 0 );
}
#endif


static int bcmring_ethtool_get_settings( struct net_device *dev,
                                         struct ethtool_cmd *cmd )
{
   BCMRING_PRIV *privp;
   ETH_PRIV *ethp;
   int reg;

   privp = netdev_priv( dev );
   ethp = &privp->eth;

   reg = ethMiiGet( ethp->phyPort, MII_BMSR );
   cmd->supported = 0;
   cmd->supported |= ((reg & BMSR_10HALF) ? SUPPORTED_10baseT_Half : 0);
   cmd->supported |= ((reg & BMSR_10FULL) ? SUPPORTED_10baseT_Full : 0);
   cmd->supported |= ((reg & BMSR_100HALF) ? SUPPORTED_100baseT_Half : 0);
   cmd->supported |= ((reg & BMSR_100FULL) ? SUPPORTED_100baseT_Full : 0);
   cmd->supported |= ((reg & BMSR_ANEGCAPABLE) ? SUPPORTED_Autoneg : 0);
   cmd->supported |= SUPPORTED_Autoneg;
   cmd->supported |= SUPPORTED_TP;
   cmd->supported |= (SUPPORTED_Pause | SUPPORTED_Asym_Pause);

   if( privp->chip.gmiiAvailable[ethp->phyPort] )
   {
      if( reg & BMSR_ESTATEN )
      {
         reg = ethMiiGet( ethp->phyPort, MII_ESTATUS );
         cmd->supported |= ((reg & ESTATUS_1000_THALF) ? SUPPORTED_1000baseT_Half : 0);
         cmd->supported |= ((reg & ESTATUS_1000_TFULL) ? SUPPORTED_1000baseT_Full : 0);
      }
   }

   reg = ethMiiGet( ethp->phyPort, MII_BMCR );
   cmd->autoneg = ((reg & BMCR_ANENABLE) ? AUTONEG_ENABLE : AUTONEG_DISABLE);
   if( cmd->autoneg )
   {
      struct
      {
         int speed;
         int duplex;
      } link[] = { { SPEED_10, DUPLEX_HALF },   /* [0] No HCD */
                   { SPEED_10, DUPLEX_HALF },
                   { SPEED_10, DUPLEX_FULL },
                   { SPEED_100, DUPLEX_HALF },
                   { SPEED_100, DUPLEX_HALF },
                   { SPEED_100, DUPLEX_FULL },
                   { SPEED_1000, DUPLEX_HALF },
                   { SPEED_1000, DUPLEX_FULL }
                 };

      /* Autonegotiation is enabled, so report settings */
      /* TODO: Current link state is reported in vendor-specific MII
      *        registers, so this code may not work for non-Broadcom PHYs
      */
      reg = ethMiiGet( ethp->phyPort, 0x19 );
      cmd->speed = link[(reg >> 8) & 0x0007].speed;
      cmd->duplex = link[(reg >> 8) & 0x0007].duplex;
   }
   else
   {
      int speed[] = { SPEED_10, SPEED_100, SPEED_1000, -1 };

      /* Autonegotiation is disabled, so report forced settings */
      cmd->speed = speed[((reg & BMCR_SPEED100 ) ? 1 : 0) |
                         ((reg & BMCR_SPEED1000) ? 2 : 0)];
      cmd->duplex = ((reg & BMCR_FULLDPLX) ? DUPLEX_FULL : DUPLEX_HALF);
   }

   reg = ethMiiGet( ethp->phyPort, MII_ADVERTISE );
   cmd->advertising = 0;
   cmd->advertising |= ((reg & ADVERTISE_10HALF) ? ADVERTISED_10baseT_Half : 0);
   cmd->advertising |= ((reg & ADVERTISE_10FULL) ? ADVERTISED_10baseT_Full : 0);
   cmd->advertising |= ((reg & ADVERTISE_100HALF) ? ADVERTISED_100baseT_Half : 0);
   cmd->advertising |= ((reg & ADVERTISE_100FULL) ? ADVERTISED_100baseT_Full : 0);
   cmd->advertising |= ((reg & ADVERTISE_PAUSE_CAP) ? ADVERTISED_Pause : 0);
   cmd->advertising |= ((reg & ADVERTISE_PAUSE_ASYM) ? ADVERTISED_Asym_Pause : 0);

   cmd->advertising |= ADVERTISED_TP;

   reg = ethMiiGet( ethp->phyPort, MII_CTRL1000 );
   cmd->advertising |= ((reg & ADVERTISE_1000HALF) ? ADVERTISED_1000baseT_Half : 0);
   cmd->advertising |= ((reg & ADVERTISE_1000FULL) ? ADVERTISED_1000baseT_Full : 0);
   cmd->advertising |= ((cmd->autoneg == AUTONEG_ENABLE) ? ADVERTISED_Autoneg : 0);

   /* Treat phy_address as physical port number */
   cmd->phy_address = ethp->phyPort;

   cmd->transceiver = ((ethHw_phy_mode_get( UNIT_NUM ) == PHY_MODE_INTERNAL) ?
                       XCVR_INTERNAL : XCVR_EXTERNAL);

   /* Hard-coded configuration */
   cmd->port = PORT_TP;
   cmd->maxtxpkt = 0;
   cmd->maxrxpkt = 0;

   return( 0 );
}


static int bcmring_ethtool_set_settings( struct net_device *dev,
                                         struct ethtool_cmd *cmd )
{
   BCMRING_PRIV *privp;
   ETH_PRIV *ethp;
   int reg;

   privp = netdev_priv( dev );
   ethp = &privp->eth;

   /* Override LAN port */
   ethp->phyPort = cmd->phy_address;

   /* Get currrent advertised link settings */
   reg = ethMiiGet( ethp->phyPort, MII_ADVERTISE );

   /* Default to advertise nothing */
   reg &= ~(ADVERTISE_10FULL    | ADVERTISE_10HALF |
            ADVERTISE_100FULL   | ADVERTISE_100HALF |
            ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);

   reg |= ((cmd->advertising & ADVERTISED_10baseT_Half)  ? ADVERTISE_10HALF     : 0);
   reg |= ((cmd->advertising & ADVERTISED_10baseT_Full)  ? ADVERTISE_10FULL     : 0);
   reg |= ((cmd->advertising & ADVERTISED_100baseT_Half) ? ADVERTISE_100HALF    : 0);
   reg |= ((cmd->advertising & ADVERTISED_100baseT_Full) ? ADVERTISE_100FULL    : 0);
   reg |= ((cmd->advertising & ADVERTISED_Pause)         ? ADVERTISE_PAUSE_CAP  : 0);
   reg |= ((cmd->advertising & ADVERTISED_Asym_Pause)    ? ADVERTISE_PAUSE_ASYM : 0);

   /* Set advertised link settings */
   ethMiiSet( ethp->phyPort, MII_ADVERTISE, reg );

   /* Get currrent extended advertised link settings */
   reg = ethMiiGet( ethp->phyPort, MII_CTRL1000 );

   /* Default to advertise nothing */
   reg &= ~(ADVERTISE_1000FULL | ADVERTISE_1000HALF);

   /* Set advertised link settings */
   /* NOTE:  Allow user to advertise 1000Mbps support even though it may not
   *         supported.
   */
   reg |= ((cmd->advertising & ADVERTISED_1000baseT_Half) ? ADVERTISE_1000HALF : 0);
   reg |= ((cmd->advertising & ADVERTISED_1000baseT_Full) ? ADVERTISE_1000FULL : 0);

   /* Set extended advertised link settings */
   ethMiiSet( ethp->phyPort, MII_CTRL1000, reg );

   /* Get current autonegotiation and forced state */
   reg = ethMiiGet( ethp->phyPort, MII_BMCR );

   /* Default to:
   *     autoneg = disabled
   *     dulpex  = half
   *     speed   = 10Mbps
   */
   reg &= ~(BMCR_ANENABLE | BMCR_FULLDPLX | BMCR_SPEED100 | BMCR_SPEED1000);

   reg |= ((cmd->autoneg == AUTONEG_ENABLE) ? BMCR_ANENABLE : 0);

   if( cmd->speed == SPEED_100 )
   {
      reg |= BMCR_SPEED100;
   }
   else if( cmd->speed == SPEED_1000 )
   {
      /* NOTE:  Allow user to force 1000Mbps support even though it may not
      *         supported.
      */
      reg |= BMCR_SPEED1000;
   }

   reg |= ((cmd->duplex == DUPLEX_FULL) ? BMCR_FULLDPLX : 0);

   /* Set autonegotiation and forced state */
   ethMiiSet( ethp->phyPort, MII_BMCR, reg );

   /* Set PHY type */
   if( (cmd->transceiver == XCVR_EXTERNAL) && (privp->chip.portNumExt == 0) )
   {
      /* Attempting to set external PHY, but not supported */
   }
   else
   {
      /* TODO:  Although the chip supports both internal and external PHYs,
      *         the hardware may only support one mode.  Perhaps add some
      *         smarts to see if the PHY is connected
      */
      ethHw_phy_mode_set( UNIT_NUM, ((cmd->transceiver == XCVR_INTERNAL) ?
                          PHY_MODE_INTERNAL : PHY_MODE_EXTERNAL) );
   }

   /* Read-only settings:
   *     cmd->supported
   */

   /* Unsupported settings:
   *     cmd->port
   *     cmd->maxtxpkt
   *     cmd->maxrxpkt
   */
   if( cmd->port != PORT_TP )
   {
      printk( "Unable to change port type.  Only twisted-pair is supported\n" );
   }

   if( (cmd->maxtxpkt != 0) || (cmd->maxrxpkt != 0) )
   {
      printk( "Unable to change packet interrupts.  This setting is not supported\n" );
   }

   /* NOTE: User is responsible for restarting autonegotiation with
   *        "ethool -r"
   */

   return( 0 );
}


static void bcmring_ethtool_get_drvinfo( struct net_device *dev,
                                         struct ethtool_drvinfo *info )
{
   BCMRING_PRIV *privp;

   privp = netdev_priv( dev );

   strcpy( info->driver, BCM_NET_MODULE_DESCRIPTION );
   strcpy( info->version, BCM_NET_MODULE_VERSION );
#ifndef SVNVERSION
#define SVNVERSION "?"
#endif

   strcpy( info->fw_version, "SVN Build " SVNVERSION );
#if 0
   /* TODO:  Add support for external PHY */
   if( ethlExtPhyIsEnabled( privp->eth.drvp ) )
   {
      strcpy( info->bus_info, "Internal bus, external PHY (SGMII)" );
   }
   else
#endif
   {
      strcpy( info->bus_info, "Internal bus, internal PHY (GMII)" );
   }
}


typedef struct
{
   uint32_t start;
   uint32_t end;
} ADDR_RANGE;

static ADDR_RANGE regRange[] =
{
   { 0x30400000, 0x304003f0 },
   { 0x30400800, 0x30400850 },
   { 0x30401000, 0x30401280 },
   { 0x30402000, 0x304020c0 },
   { 0x30402800, 0x30402c18 },
   { 0x30410000, 0x304107f8 },
   { 0x30410800, 0x30410ff8 },
   { 0x30414000, 0x304147f8 },
   { 0x304a0000, 0x304a0090 },
};


#if GET_REGS_CUSTOM
static const char *regRangeStr[] =
{
   "Control",
   "Status",
   "Management",
   "ARL Access",
   "ARL Vtbl Access",
   "External Port 0 MIB",
   "External Port 1 MIB",
   "IMP Port MIB",
   "IMP port and dual gphy misc.",
};


static void dumpRegs( uint32_t start, uint32_t end, char *descp )
{
   uint32_t addr;

   /* NOTE:  The register space only has valid registers every second 32-bit
   *         word 
   */

   /*      "0x00000000: 00000000 00000000 00000000 00000000\n" */
   printk( "\n%s:\n", descp );
   printk( "             +0x00    +0x08    +0x10    +0x18" );

   for( addr=start; addr<=end; addr+=8 )
   {
      if( (addr % 0x20) == 0 )
      {
         printk( "\n0x%08x: ", addr );
      }

      printk( "%08x ", *((uint32_t *)MM_IO_PHYS_TO_VIRT( addr )) );
   }

   printk( "\n" );
}
#endif


static int bcmring_ethtool_get_regs_len( struct net_device *dev )
{
#if GET_REGS_CUSTOM
   return( 0 );
#else
   return( 16 * 1024 );
#endif
}


static void bcmring_ethtool_get_regs( struct net_device *dev,
                                      struct ethtool_regs *regs, void *p )
{
   register int i;
   uint32_t *bufp;

   regs->version = 0;
   regs->len = 0;

   bufp = (uint32_t *)p;

   for( i=0; i<(sizeof( regRange ) / sizeof( regRange[0] )); i++ )
   {
#if GET_REGS_CUSTOM
      dumpRegs( regRange[i].start, regRange[i].end, regRangeStr[i] );
#else
      register int addr;

      /* NOTE:  The register space only has valid registers every second 32-bit
      *         word 
      */

      for( addr=regRange[i].start; addr<=regRange[i].end; addr+=(2 * sizeof( uint32_t )) )
      {
         *bufp++ = *((uint32_t *)MM_IO_PHYS_TO_VIRT( addr ));
         regs->len += sizeof( uint32_t );
      }
#endif
   }
}


static u32 bcmring_ethtool_get_msglevel( struct net_device *dev )
{
   BCMRING_PRIV *privp;

   privp = netdev_priv( dev );

   /* Share the sysctl message level */
   return( sysCtlMsgLevel );
}


static void bcmring_ethtool_set_msglevel( struct net_device *dev, u32 level )
{
   BCMRING_PRIV *privp;

   privp = netdev_priv( dev );

   /* Share the sysctl message level */
   sysCtlMsgLevel = level;
}


static int bcmring_ethtool_nway_reset( struct net_device *dev )
{
   BCMRING_PRIV *privp;
   ETH_PRIV *ethp;
   int reg;

   privp = netdev_priv( dev );
   ethp = &privp->eth;

   reg = ethMiiGet( ethp->phyPort, MII_BMCR );
   ethMiiSet( ethp->phyPort, MII_BMCR, reg | BMCR_ANRESTART );

   return( 0 );
}


static int bcmring_ethtool_get_link( struct net_device *dev )
{
   BCMRING_PRIV *privp;
   ETH_PRIV *ethp;
   int reg;

   privp = netdev_priv( dev );
   ethp = &privp->eth;

   reg = ethMiiGet( ethp->phyPort, ETHHW_MII_EXT_STATUS );

   return( (reg & ETHHW_MII_EXT_STATUS_LINK_MASK) ? 1 : 0 );
}


static void bcmring_ethtool_get_ringparam( struct net_device *dev,
                                           struct ethtool_ringparam *ering )
{
   BCMRING_PRIV *privp;

   privp = netdev_priv( dev );

   ering->rx_max_pending = privp->dma.rx.ringSize;
   ering->rx_mini_max_pending = 0;
   ering->rx_jumbo_max_pending = 0;
   ering->tx_max_pending = privp->dma.tx.ringSize;
   /* TODO:  No easy way to determine rx_pending without traversing the
   *         descriptor ring
   */
   ering->rx_pending = 0;
   ering->rx_mini_pending = 0;
   ering->rx_jumbo_pending = 0;
   ering->tx_pending = kfifo_len( privp->dma.txFifop ) / sizeof( struct sk_buff * );
}


static int bcmring_ethtool_set_ringparam( struct net_device *dev,
                                          struct ethtool_ringparam *ering )
{
   (void)dev;
   (void)ering;

   /* TODO:  Not required, but may add later for debugging purposes */

   return( -EOPNOTSUPP );
}


static void bcmring_ethtool_get_pauseparam( struct net_device *dev,
                                            struct ethtool_pauseparam *epause )
{
   BCMRING_PRIV *privp;
   ETH_PRIV *ethp;
   int reg;
   int pause;
   int asm_dir;

   privp = netdev_priv( dev );
   ethp = &privp->eth;

   reg = ethMiiGet( ethp->phyPort, MII_ADVERTISE );

   pause = reg & ADVERTISE_PAUSE_CAP;
   asm_dir = reg & ADVERTISE_PAUSE_ASYM;

   if( pause && !asm_dir )
   {
      /* Symmetric pause */
      epause->rx_pause = 1;
      epause->tx_pause = 1;
   }
   else if( asm_dir )
   {
      /* Asymmetric pause */

      /* Per 802.3 37.2.4.2, when ASM_DIR = 1:
      *     PAUSE = 1 advertises receiver but no transmitter
      *     PAUSE = 0 advertises transmitter but no receiver
      */
      if( reg & ADVERTISE_PAUSE_CAP )
      {
         epause->rx_pause = 1;
         epause->tx_pause = 0;
      }
      else
      {
         epause->rx_pause = 0;
         epause->tx_pause = 1;
      }
   }
   else
   {
      /* No pause */
      epause->rx_pause = 0;
      epause->tx_pause = 0;
   }

   reg = ethMiiGet( ethp->phyPort, MII_BMCR );
   epause->autoneg = ((reg & BMCR_ANENABLE) ? 1 : 0);
}


static int bcmring_ethtool_set_pauseparam( struct net_device *dev,
                                           struct ethtool_pauseparam *epause )
{
   BCMRING_PRIV *privp;
   ETH_PRIV *ethp;
   int reg;

   privp = netdev_priv( dev );
   ethp = &privp->eth;

   reg = ethMiiGet( ethp->phyPort, MII_ADVERTISE );

   /* Start with no pause support */
   reg &= ~(ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);

   if( epause->rx_pause && epause->tx_pause )
   {
      /* Symmetric pause */
      reg |= ADVERTISE_PAUSE_CAP;
   }
   else if( epause->rx_pause ^ epause->tx_pause )
   {
      /* Asymmetric pause */
      reg |= ADVERTISE_PAUSE_ASYM;

      /* Per 802.3 37.2.4.2, when ASM_DIR = 1:
      *     PAUSE = 1 advertises receiver but no transmitter
      *     PAUSE = 0 advertises transmitter but no receiver
      */
      if( epause->rx_pause )
      {
         reg |= ADVERTISE_PAUSE_CAP;
      }
   }

   ethMiiSet( ethp->phyPort, MII_ADVERTISE, reg );

   reg = ethMiiGet( ethp->phyPort, MII_BMCR );

   if( epause->autoneg )
   {
      reg |= BMCR_ANENABLE;
   }
   else
   {
      reg &= ~BMCR_ANENABLE;
   }

   ethMiiSet( ethp->phyPort, MII_BMCR, reg );

   /* NOTE: User is responsible for restarting autonegotiation with
   *        "ethtool -r"
   */

   return( 0 );
}


static int bcmring_ethtool_get_test_count( struct net_device *dev )
{
   (void)dev;

   return( sizeof( ethtoolStrTest ) / sizeof( ethtoolStrTest[0] ) );
}


static void bcmring_ethtool_self_test( struct net_device *dev,
                                       struct ethtool_test *etest,
                                       u64 *data )
{
   register int i;

   etest->len = bcmring_ethtool_get_test_count( dev );

   if( etest->flags & ETH_TEST_FL_OFFLINE )
   {
      /* TODO: Run test while interface is offline */
   }

   for( i=0; i<etest->len; i++ )
   {
#if 0
      /* TODO: Go through list of self tests */
      if( !(*data++ = TODO_add_test_function_array()) )
#else
      /* Temporary return codes for testing purposes */
      *data++ = 100 + i;
#endif
      {
         etest->flags |= ETH_TEST_FL_FAILED;
      }
   }

   if( etest->flags & ETH_TEST_FL_OFFLINE )
   {
      /* TODO: Restore original state before taking interface offline */
   }
}


static void bcmring_ethtool_get_strings( struct net_device *dev,
                                         u32 strSet, u8 *buf )
{
   switch( strSet )
   {
      case ETH_SS_STATS:
      {
         memcpy( buf, ethtoolStrStat, sizeof( ethtoolStrStat ) );
      }
      break;

      case ETH_SS_TEST:
      {
         memcpy( buf, ethtoolStrTest, sizeof( ethtoolStrTest ) );
      }
      break;

      default:
      {
         myPrintk( KERN_WARNING "Unsupported string set %d\n", strSet );
      }
      break;
   }
}


static int bcmring_ethtool_phys_id( struct net_device *dev, u32 seconds )
{
   BCMRING_PRIV *privp;
   ETH_PRIV *ethp;
   volatile uint16_t ledModeMap0;
   volatile uint16_t ledModeMap1;

   /* Set maximum duration */
   if( seconds > PHYS_ID_DURATION_MAX )
   {
      seconds = PHYS_ID_DURATION_MAX;
   }

   privp = netdev_priv( dev );
   ethp = &privp->eth;

   /* Backup original LED configuration */
   ledModeMap0 = REG_LED_MODE_MAP_0;
   ledModeMap1 = REG_LED_MODE_MAP_1;

   /* Blink only the LAN port */
   REG_LED_MODE_MAP_0 = 0;
   REG_LED_MODE_MAP_1 = 1 << ethp->phyPort;

   msleep_interruptible( seconds * 1000 );

   /* Restore original LED configuration */
   REG_LED_MODE_MAP_0 = ledModeMap0;
   REG_LED_MODE_MAP_1 = ledModeMap1;

   return( 0 );
}


static int bcmring_ethtool_get_stats_count( struct net_device *dev )
{
   return( sizeof( ethtoolStrStat ) / sizeof( ethtoolStrStat[0] ) );
}


static void bcmring_ethtool_get_ethtool_stats( struct net_device *dev,
                                               struct ethtool_stats *estats,
                                               u64 *stats )
{
   BCMRING_PRIV *privp;
   ETH_PRIV *ethp;

   privp = netdev_priv( dev );
   ethp = &privp->eth;

   if( strncmp( dev->name, "phy", 3 ) == 0 )
   {
      /* /phy device, so report external port MIB */

#define GET_MIB(m)      *stats++ = ethHw_mib ## m ( ethp->phyPort )

      GET_MIB( RxOctets );                      /* RxOctets */
      GET_MIB( RxDropPkts );                    /* RxDropPkts */
      GET_MIB( TxQ0Pkt );                       /* TxQ0Pkt */
      GET_MIB( RxBroadcastPkts );               /* RxBroadcastPkts */
      GET_MIB( RxMulticastPkts );               /* RxMulticastPkts */
      GET_MIB( RxUnicastPkts );                 /* RxUnicastPkts */
      GET_MIB( RxPausePkts );                   /* RxPausePkts */
      GET_MIB( TxQ1Pkt );                       /* TxQ1Pkt */
      GET_MIB( TxQ2Pkt );                       /* TxQ2Pkt */
      GET_MIB( TxQ3Pkt );                       /* TxQ3Pkt */
      GET_MIB( TxQ4Pkt );                       /* TxQ4Pkt */
      GET_MIB( TxQ5Pkt );                       /* TxQ5Pkt */
      GET_MIB( TxOctets );                      /* TxOctets */
      GET_MIB( RxUndersizePkts );               /* RxUndersizePkts */
      GET_MIB( TxPausePkts );                   /* TxPausePkts */
      GET_MIB( Pkts64Octets );                  /* RxPkts64Octets */
      GET_MIB( Pkts65To127Octets );             /* RxPkts65To127Octets */
      GET_MIB( Pkts128To255Octets );            /* RxPkts128To255Octets */
      GET_MIB( Pkts256To511Octets );            /* RxPkts256To511Octets */
      GET_MIB( Pkts512To1023Octets );           /* RxPkts512To1023Octets */
      GET_MIB( Pkts1024ToMaxPktOctets );        /* RxPkts1024ToMaxPktOctets */
      GET_MIB( TxOctets );                      /* TxGoodOctets */
      GET_MIB( TxDropPkts );                    /* TxDropPkts */
      GET_MIB( TxUnicastPkts );                 /* TxUnicastPkts */
      GET_MIB( TxMulticastPkts );               /* TxMulticastPkts */
      GET_MIB( TxBroadcastPkts );               /* TxBroadcastPkts */
      GET_MIB( RxSaChanges );                   /* RxSaChanges */
      GET_MIB( JumboPktCount );                 /* RxJumboPktCount */
      GET_MIB( InRangeErrorCount );             /* RxInRangeErrorCount */
      GET_MIB( OutRangeErrorCount );            /* RxOutRangeErrorCount */
      GET_MIB( RxDiscard );                     /* RxDiscard */
   }
   else
   {
      /* The order of the following function calls must match the order of
      *  ethtoolStrStat[].  Note that Rx/Tx from the NIC perspective is actually
      *  Tx/Rx from the internal port and Rx/Tx from the LAN port.
      */

#define GET_MIB_IMP(m)     *stats++ = ethHw_mib ## m ( ETHHW_PORT_INT )
#define GET_MIB_LAN(m)     *stats++ = ethHw_mib ## m ( ethp->phyPort )

      GET_MIB_IMP( TxOctets );                  /* RxOctets */
      GET_MIB_IMP( TxDropPkts );                /* RxDropPkts */
      GET_MIB_IMP( TxQ0Pkt );                   /* RxQ0Pkt */
      GET_MIB_IMP( TxBroadcastPkts );           /* RxBroadcastPkts */
      GET_MIB_IMP( TxMulticastPkts );           /* RxMulticastPkts */
      GET_MIB_IMP( TxUnicastPkts );             /* RxUnicastPkts */
      GET_MIB_LAN( RxPausePkts );               /* RxPausePkts */
      GET_MIB_IMP( TxQ1Pkt );                   /* RxQ1Pkt */
      GET_MIB_IMP( TxQ2Pkt );                   /* RxQ2Pkt */
      GET_MIB_IMP( TxQ3Pkt );                   /* RxQ3Pkt */
      GET_MIB_IMP( TxQ4Pkt );                   /* RxQ4Pkt */
      GET_MIB_IMP( TxQ5Pkt );                   /* RxQ5Pkt */
      GET_MIB_IMP( RxOctets );                  /* TxOctets */
      GET_MIB_IMP( RxUndersizePkts );           /* TxUndersizePkts */
      GET_MIB_LAN( TxPausePkts );               /* TxPausePkts */
      GET_MIB_IMP( Pkts64Octets );              /* TxPkts64Octets */
      GET_MIB_IMP( Pkts65To127Octets );         /* TxPkts65To127Octets */
      GET_MIB_IMP( Pkts128To255Octets );        /* TxPkts128To255Octets */
      GET_MIB_IMP( Pkts256To511Octets );        /* TxPkts256To511Octets */
      GET_MIB_IMP( Pkts512To1023Octets );       /* TxPkts512To1023Octets */
      GET_MIB_IMP( Pkts1024ToMaxPktOctets );    /* TxPkts1024ToMaxPktOctets */
      GET_MIB_IMP( RxGoodOctets );              /* TxGoodOctets */
      GET_MIB_IMP( RxDropPkts );                /* TxDropPkts */
      GET_MIB_IMP( RxUnicastPkts );             /* TxUnicastPkts */
      GET_MIB_IMP( RxMulticastPkts );           /* TxMulticastPkts */
      GET_MIB_IMP( RxBroadcastPkts );           /* TxBroadcastPkts */
      GET_MIB_IMP( RxSaChanges );               /* TxSaChanges */
      GET_MIB_IMP( JumboPktCount );             /* TxJumboPktCount */
      GET_MIB_IMP( InRangeErrorCount );         /* TxInRangeErrorCount */
      GET_MIB_IMP( OutRangeErrorCount );        /* TxOutRangeErrorCount */
      GET_MIB_IMP( RxDiscard );                 /* TxDiscard */
   }
}
