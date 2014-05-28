/*****************************************************************************
* Copyright 2009 Broadcom Corporation.  All rights reserved.
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

#include <linux/types.h>
#include <linux/device.h>
#include <linux/amba/bus.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/mfd/ds1wm.h>
#include <linux/string.h>
#include <linux/version.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/div64.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>

#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/i2c.h>
#include <mach/dma.h>
#include <mach/system.h>
#include <mach/pmb.h>
#include <mach/dmu.h>
#include <mach/reg_gpio.h>

#include <linux/broadcom/sdio_platform.h>
#include <linux/broadcom/bootmemheap.h>

#include <linux/broadcom/mmdma.h>
#include <mmdma_settings.h>

#ifdef CONFIG_VDEC
#include <vdec_settings.h>
#endif

HW_DECLARE_SPINLOCK(Gpio)
#define BCM5892_ETHERNET_MAC_BASE       0x00086000

#if defined( CONFIG_DEBUG_SPINLOCK )
    EXPORT_SYMBOL(gGpioRegLock);
#endif

#define IO_DESC(pa, sz) { .virtual = IO_ADDRESS(pa), \
                          .pfn = __phys_to_pfn(pa), \
                          .length = (sz), \
                          .type = MT_DEVICE }


/* Virtual mapping for devices */
static struct map_desc bcm5892_io_desc[] __initdata = {
	IO_DESC(PMB_REG_BASE_ADDR,SZ_4K),
	IO_DESC(SMU_REG_BASE_ADDR,SZ_16M),
	IO_DESC(PKA_REG_BASE_ADDR,SZ_4K),
	IO_DESC(RNG_REG_BASE_ADDR,SZ_4K),
	IO_DESC(NVM_REG_BASE_ADDR,SZ_4K),
	IO_DESC(SPL_REG_BASE_ADDR,SZ_4K),
	IO_DESC(DMU_REG_BASE_ADDR,SZ_4K),
	IO_DESC(CFG_REG_BASE_ADDR,SZ_4K),
	IO_DESC(BBL0_REG_BASE_ADDR,SZ_4K),
	IO_DESC(BBL0_REG_BASE_ADDR,SZ_4K),
	IO_DESC(VIC0_REG_BASE_ADDR,SZ_4K),
	IO_DESC(VIC1_REG_BASE_ADDR,SZ_4K),
	IO_DESC(VIC2_REG_BASE_ADDR,SZ_4K),
	IO_DESC(SBM_REG_BASE_ADDR,SZ_4K),
	IO_DESC(DMA0_REG_BASE_ADDR,SZ_4K),
	IO_DESC(DMA1_REG_BASE_ADDR,SZ_4K),
	IO_DESC(ETH_REG_BASE_ADDR,SZ_4K),
	IO_DESC(SDM1_REG_BASE_ADDR,SZ_4K),
	IO_DESC(SDM0_REG_BASE_ADDR,SZ_4K),
	IO_DESC(LCD_REG_BASE_ADDR,SZ_4K),
	IO_DESC(DEC_REG_BASE_ADDR,SZ_4K),
	IO_DESC(MEM_REG_BASE_ADDR,SZ_4K),
	IO_DESC(USB0_REG_BASE_ADDR,SZ_4K),
	IO_DESC(USBWC_REG_BASE_ADDR,SZ_4K),
	IO_DESC(USB1_REG_BASE_ADDR,SZ_4K),
	IO_DESC(USB2_REG_BASE_ADDR,SZ_4K),
	IO_DESC(TIM0_REG_BASE_ADDR,SZ_4K),
	IO_DESC(TIM1_REG_BASE_ADDR,SZ_4K),
	IO_DESC(TIM2_REG_BASE_ADDR,SZ_4K),
	IO_DESC(TIM3_REG_BASE_ADDR,SZ_4K),
	IO_DESC(WDT_REG_BASE_ADDR,SZ_4K),
	IO_DESC(SCI0_REG_BASE_ADDR,SZ_4K),
	IO_DESC(SCI1_REG_BASE_ADDR,SZ_4K),
	IO_DESC(ADC0_REG_BASE_ADDR,SZ_4K),
	IO_DESC(ADC1_REG_BASE_ADDR,SZ_4K),
	IO_DESC(MSR_REG_BASE_ADDR,SZ_4K),
	IO_DESC(I2C0_REG_BASE_ADDR,SZ_4K),
	IO_DESC(SPI0_REG_BASE_ADDR,SZ_4K),
	IO_DESC(SPI1_REG_BASE_ADDR,SZ_256K),
	IO_DESC(GIO0_REG_BASE_ADDR,SZ_4K),
	IO_DESC(GIO1_REG_BASE_ADDR,SZ_4K),
	IO_DESC(URT0_REG_BASE_ADDR,SZ_4K),
	IO_DESC(URT1_REG_BASE_ADDR,SZ_4K),
	IO_DESC(URT2_REG_BASE_ADDR,SZ_4K),
	IO_DESC(URT3_REG_BASE_ADDR,SZ_1K),
	IO_DESC(I2C1_REG_BASE_ADDR,SZ_1K),
	IO_DESC(SPI2_REG_BASE_ADDR,SZ_1K),
	IO_DESC(PWM_REG_BASE_ADDR,SZ_1K),
	IO_DESC(GIO3_REG_BASE_ADDR,SZ_4K),
	IO_DESC(I2S_REG_BASE_ADDR,SZ_4K),
	IO_DESC(DAC_REG_BASE_ADDR,SZ_1K),
	IO_DESC(UMC_REG_BASE_ADDR,SZ_4K),
	IO_DESC(TPB_REG_BASE_ADDR,SZ_4K),
	IO_DESC(TST_REG_BASE_ADDR,SZ_4K),
	IO_DESC(SPI3_REG_BASE_ADDR,SZ_4K),
	IO_DESC(GIO2_REG_BASE_ADDR,SZ_4K),
	IO_DESC(GIO4_REG_BASE_ADDR ,SZ_4K),
	IO_DESC(SMC_REG_BASE_ADDR,SZ_4K),
	IO_DESC(MMI_REG_BASE_ADDR,SZ_4K),
	IO_DESC(START_SCRATCH+0x3b000,SZ_16K), /* map scratch mem used by TDM3 as IO, for now */
};


/* AMBA devices */
/* UART0 */
static struct amba_device uart0_device = {
	.dev		= {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		.bus_id = ":amba:uart0",
#else
	.init_name = ":amba:uart0",
#endif
	},
	.res		= {
		.start	= URT0_REG_BASE_ADDR,
		.end	= END_URT0,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { IRQ_OUART0, NO_IRQ },
	.periphid	= 0x41011,
};

/* UART1 is used for GDB */
static struct amba_device uart1_device = {
	.dev		= {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		.bus_id	= ":amba:uart1",
#else
	.init_name = ":amba:uart1",
#endif
	},
	.res		= {
		.start	= URT1_REG_BASE_ADDR,
		.end	= END_URT1,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { IRQ_OUART1, NO_IRQ },
	.periphid	= 0x41011,
};

/* UART2 */
static struct amba_device uart2_device = {
	.dev            = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		.bus_id = ":amba:uart2",
#else
	.init_name = ":amba:uart2",
#endif
	},
	.res            = {
		.start  = URT2_REG_BASE_ADDR,
		.end    = END_URT2,
		.flags  = IORESOURCE_MEM,
	},
	.irq            = { IRQ_OUART2, NO_IRQ },
	.periphid       = 0x41011,
};

/* UART3 */
static struct amba_device uart3_device = {
	.dev		= {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		.bus_id	= ":amba:uart3",
#else
	.init_name = ":amba:uart3",
#endif
	},
	.res		= {
		.start	= URT3_REG_BASE_ADDR,
		.end	= END_URT3,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { IRQ_OUART3, NO_IRQ },
	.periphid	= 0x41011,
};


/* Watchdog Timer*/
static struct amba_device open_wdt_device = {
	.dev		= {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		.bus_id	= ":amba:wdt",
#else
	.init_name = ":amba:wdt",
#endif
	},
	.res		= {
		.start	= WDT_REG_BASE_ADDR,
		.end	= END_WDT,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { IRQ_OWDOG_TIMOUT, NO_IRQ },
	.periphid	= 0x41805,
};

static struct amba_device umc_nand_device = {
	.dev		= {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		.bus_id	= ":amba:umc-nand",
#else
	.init_name = ":amba:umc-nand",
#endif
	},
	.res		= {
		.start	= START_NAND_CS1,
		.end	= END_NAND_CS1,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { NO_IRQ, NO_IRQ },
	.periphid	= 0x41353,
};

/* PL022 SPI device 0 */
static struct amba_device spi0_amba_device = {
	.dev = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		.bus_id = "spi0",
#else
	.init_name = "spi0",
#endif
	},
	.res = {
		.start	= SPI0_REG_BASE_ADDR,
		.end	= END_SPI0,
		.flags	= IORESOURCE_MEM,
	},

	.irq		= { IRQ_OSPI0, NO_IRQ },
	.periphid	= 0x41022,
};

/* PL022 SPI device 1 */
static struct amba_device spi1_amba_device = {
	.dev = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		.bus_id = "spi1",
#else
	.init_name = "spi1",
#endif
	},
	.res = {
		.start	= SPI1_REG_BASE_ADDR,
		.end	= END_SPI1,
		.flags	= IORESOURCE_MEM,
	},

	.irq		= { IRQ_OSPI1, NO_IRQ },
	.periphid	= 0x41022,
};

/* PL022 SPI device 3 */
static struct amba_device spi3_amba_device = {
	.dev = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		.bus_id = "spi3",
#else
	.init_name = "spi3",
#endif
	},
	.res = {
		.start	= SPI3_REG_BASE_ADDR,
		.end	= END_SPI3,
		.flags	= IORESOURCE_MEM,
	},

	.irq		= { IRQ_OSPI3, NO_IRQ },
	.periphid	= 0x41022,
	};

/* PL022 SPI device 4 */
static struct amba_device spi4_amba_device = {
	.dev = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		.bus_id = "spi4",
#else
	.init_name = "spi4",
#endif
	},
	.res = {
		.start	= START_TPB_SPI,
		.end	= END_TPB_SPI,
		.flags	= IORESOURCE_MEM,
	},

	.irq		= { IRQ_OTPBSPI4, NO_IRQ },
	.periphid	= 0x41022,
};

static struct bcm5892_i2c_platform_data i2c_params_slow = {
	.bus_freq_khz = 100,
	.bus_delay_us = 0,
};

static struct bcm5892_i2c_platform_data i2c_params_fast = {
	.bus_freq_khz = 400,
	.bus_delay_us = 0,
};

/* BSC / I2C device 0 */
static struct amba_device i2c0_amba_device = {
	.dev		= {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		.bus_id	= "i2c0",
#else
	.init_name = "i2c0",
#endif
		.platform_data = &i2c_params_fast,
	},
	.res		= {
		.start	= I2C0_REG_BASE_ADDR,
		.end	= END_I2C0,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { IRQ_OBSC0, NO_IRQ }, /* if not given an IRQ, the driver will poll */
	.periphid	= 0x12c00,  /* last two digits: i2c bus ID */
};

/* BSC / I2C device 1 */
static struct amba_device i2c1_amba_device = {
	.dev		= {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		.bus_id	= "i2c1",
#else
	.init_name = "i2c1",
#endif
		.platform_data = &i2c_params_slow,
	},
	.res		= {
		.start	= I2C1_REG_BASE_ADDR,
		.end	= END_I2C1,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { IRQ_OBSC1, NO_IRQ }, /* if not given an IRQ, the driver will poll */
	.periphid	= 0x12c01,  /* last two digits: i2c bus ID */
};

/* ADC */
static struct amba_device adc_amba_device = {
	.dev		= {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		.bus_id	= "adc",
#else
	.init_name = "adc",
#endif
	},
	.res		= {
		.start	= START_ADC0,
		.end	= END_ADC1,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { NO_IRQ, NO_IRQ },
	.periphid	= 0xadc00,  /* bogus ID for the ADC */
};

/* DMA */
static struct amba_device dma_amba_device = {
	.dev		= {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		.bus_id	= "dma",
#else
	.init_name = "dma",
#endif
	},
	.res		= {
		.start	= START_DMA0_CFG,
		.end	= END_DMA0_CFG,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { IRQ_OODMA, NO_IRQ },
	.periphid	= 0x41081,
};


static struct amba_device *amba_devs[] __initdata = {
	&uart0_device,
	&uart1_device,
	&uart2_device,
	&uart3_device,
	&open_wdt_device,
	&umc_nand_device,
	&spi0_amba_device,
	&spi1_amba_device,
	&spi3_amba_device,
	&spi4_amba_device,
	&i2c0_amba_device,
	&i2c1_amba_device,
	&adc_amba_device,
	&dma_amba_device
};

static struct platform_device rtc_dev = {
	.name		= "bcm5892-rtc",
	.id		= -1,
};

/* d1w interface */
static void ds1wm_enable(struct platform_device *pdev)
{
	reg_gpio_iotr_set_pin_type(GPIO_AUX_D1W,
				    GPIO_PIN_TYPE_ALTERNATIVE_FUNC1);
}

static struct ds1wm_platform_data ds1wm_platform_data = {
	.bus_shift	= 2,
	.enable		= ds1wm_enable,
};

static struct resource ds1wm_resources[] = {
	[0] = {
		.start	= D1W_REG_BASE_ADDR,
		.end	= END_D1W,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_OD1W,
		.end	= IRQ_OD1W,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device ds1wm = {
	.name		=	"ds1wm",
	.id		=	-1,
	.dev		=	{
		.platform_data	= &ds1wm_platform_data,
	},
	.num_resources  = ARRAY_SIZE(ds1wm_resources),
  	.resource	= ds1wm_resources,
};

/* sdm0 */
static struct sdio_platform_cfg sdm0_platform_data = {
   .devtype = SDIO_DEV_TYPE_SDMMC,
};
static struct resource sdm0_resources[] = {
	[0] = {
		.start	= SDM0_REG_BASE_ADDR,
		.end	= END_SDM0_CFG,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_OSDIOMMCCEATA4,
		.end	= IRQ_OSDIOMMCCEATA4,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device sdm0 = {
	.name		=	"bcm-sdio",
	.id		=	0, /* Use sdm0 */
	.dev		=	{
		.platform_data	= &sdm0_platform_data,
	},
	.num_resources  = ARRAY_SIZE(sdm0_resources),
  	.resource	= sdm0_resources,
};

/* sdm1 */
static struct sdio_platform_cfg sdm1_platform_data = {
   .devtype = SDIO_DEV_TYPE_WIFI,
};
static struct resource sdm1_resources[] = {
	[0] = {
		.start	= SDM1_REG_BASE_ADDR,
		.end	= END_SDM1_CFG,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_OSDIOMMCCEATA8,
		.end	= IRQ_OSDIOMMCCEATA8,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device sdm1 = {
	.name		=	"bcm-sdio",
	.id		=	1, /* Use sdm1 */
	.dev		=	{
		.platform_data	= &sdm1_platform_data,
	},
	.num_resources  = ARRAY_SIZE(sdm1_resources),
  	.resource	= sdm1_resources,
};

/* XXX This bus information is BCM95892-specific */
struct i2c_board_info example_i2c0_bus_info[] =  {
	{
		I2C_BOARD_INFO("24c128", 0x51),  /* AT24c128 has same commands as Microchip 24LC128 */
	},
	{
		I2C_BOARD_INFO("wm8750", 0x1A),  /* external audio codec chip */
	},
};

struct i2c_board_info example_i2c1_bus_info[] =  {
	{
		I2C_BOARD_INFO("24c128", 0x53),  /* AT24c128 has same commands as Microchip 24LC128 */
	},
	{
		I2C_BOARD_INFO("ALC5623_codec", 0x1A),  /* external audio codec chip */
	},
#if 1 /* this is when A0 pin is high */
	{
		I2C_BOARD_INFO("coupler_bank0", 0x26),  /* smart card coupler chip NXP8026 Bank0 */
	},
	{
		I2C_BOARD_INFO("coupler_bank1reg0", 0x22),  /* smart card coupler chip NXP8026 Bank1 Register 0 */
	},
	{
		I2C_BOARD_INFO("coupler_bank1reg1", 0x23),  /* smart card coupler chip NXP8026 Bank1 Register 1 */
	},
#else /* this is when A0 pin is low */
	{
		I2C_BOARD_INFO("coupler_bank0", 0x24),  /* smart card coupler chip NXP8026 Bank0 */
	},
	{
		I2C_BOARD_INFO("coupler_bank1reg0", 0x20),  /* smart card coupler chip NXP8026 Bank1 Register 0 */
	},
	{
		I2C_BOARD_INFO("coupler_bank1reg1", 0x21),  /* smart card coupler chip NXP8026 Bank1 Register 1 */
	},
#endif
};


struct spi_board_info example_spi_bus_info[4] = {
	{
		.modalias = "mtd_dataflash",
		.max_speed_hz = 20000000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = 0,
	},
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000,
		.bus_num = 1,
		.chip_select = 0,
		.mode = 0,
	},
	{
		.modalias = "mtd_dataflash",
		.max_speed_hz = 20000000,
		.bus_num = 3,
		.chip_select = 0,
		.mode = 0,
	},
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000,
		.bus_num = 4,
		.chip_select = 0,
		.mode = 0,
	},
};

extern void bcm5892_init_irq(void);
extern void sys_timer_init(void);

static void __init bcm5892_map_io(void)
{
	iotable_init(bcm5892_io_desc, ARRAY_SIZE(bcm5892_io_desc));
}

void bcm5892_cpu_reset(void)
{
	uint32_t reg, val;

	reg = DMU_R_dmu_sw_rst_MEMADDR;
	val = (1 << DMU_SWRST);
	call_secure_api(CLS_DMU_REG_WR_ID, 3, reg, val, 0);
}


#ifdef CONFIG_VDEC
#ifdef HW_CFG_VDEC
static struct vdec_cfg board_vdec_cfg = HW_CFG_VDEC;
#else
#error "Please define HW_CFG_VDEC in hw_cfg/vdec_settings.h."
#endif
static size_t board_bootmemheap_calc_vdec_mem(void)
{
   return board_vdec_cfg.memsize;
}
#endif

#ifdef HW_CFG_MMDMA_MEM_TABLE
#define board_mmdma_mem_table bcm5892_mmdma_mem_table
static unsigned int board_mmdma_mem_table[] = HW_CFG_MMDMA_MEM_TABLE;
#endif

#define board_mmdma_hw_cfg bcm5892_mmdma_param
static struct mmdma_hw_cfg board_mmdma_hw_cfg;

#define board_mmdma_device bcm5892_mmdma_device
static struct platform_device board_mmdma_device = {
   .name = "bcm5892-mmdma",
   .id = -1,
   .dev = {
      .platform_data = &board_mmdma_hw_cfg,
   },
};

#define BCM5892_PA_PM_MISC (0x60000000)
static struct resource bcm5892_pm_misc[] = {
	{
		.start = BCM5892_PA_PM_MISC,
		.end   = BCM5892_PA_PM_MISC + 0x10,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device bcm5892_device_pm_misc = {
	.name		  = "bcm5892-pm-misc",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(bcm5892_pm_misc),
	.resource	  = bcm5892_pm_misc,
};

#define board_bootmemheap_calc_mmdma bcm5892_bootmemheap_calc_mmdma
static size_t board_bootmemheap_calc_mmdma(void)
{
   size_t mmdma_size = 0;

#ifdef HW_CFG_MMDMA_MEM_TABLE
       unsigned int i;

       for (i = 0; i < ARRAY_SIZE(board_mmdma_mem_table); i++) {
          mmdma_size += board_mmdma_mem_table[i];
       }
#endif

   printk(KERN_INFO "%s: Reserving memory %u bytes for MMDMA\n", __func__, mmdma_size);

   return mmdma_size;
}


/****************************************************************************
*
*   Called from the customize_machine function in arch/arm/kernel/setup.c
*
*   The customize_machine function is tagged as an arch_initcall
*   (see include/linux/init.h for the order that the various init sections
*   are called in.
*
*****************************************************************************/
//static void __init bcm5892_init_machine( void )
void __init bcm5892_init_machine( void )
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(amba_devs); i++) {
		struct amba_device *d = amba_devs[i];
		amba_device_register(d, &iomem_resource);
	}
	platform_device_register(&rtc_dev);
	platform_device_register(&ds1wm);

	printk(KERN_NOTICE "SDIO0 init\n");
#if 0
	cls_dmu_block_enable_rst(DMU_SDM0_PWR_ENABLE);
#else
	printk(KERN_NOTICE  "SDM0 reset skip\n");
#endif
	platform_device_register(&sdm0);

	printk(KERN_NOTICE "SDIO1 init\n");
	reg_gpio_iotr_set_pin_type(GPIO_AUX_PRINTER, GPIO_PIN_TYPE_ALTERNATIVE_FUNC1);
	reg_gpio_set_drv_strength(GPIO_AUX_PRINTER, GPIO_DRV_STRENGTH_6mA);
	reg_gpio_set_pull_up_down(GPIO_AUX_PRINTER, 0);
#if 0
	cls_dmu_block_enable_rst(DMU_SDM1_PWR_ENABLE);
#else
	printk(KERN_NOTICE  "SDM1 reset skip\n");
#endif
	platform_device_register(&sdm1);

	platform_device_register(&bcm5892_device_pm_misc);

#if 0
	/* Initialize the PMB for context switching. */
	pmb_init();
#endif

	/* LCD (set pins to 6mA) */
	writel (0, IO_ADDRESS(GIO2_R_GRPF2_DRV_SEL0_SW_MEMADDR));
	writel (0, IO_ADDRESS(GIO2_R_GRPF2_DRV_SEL1_SW_MEMADDR));
	writel (0x3fffffff, IO_ADDRESS(GIO2_R_GRPF2_DRV_SEL2_SW_MEMADDR));

	/* SDB (set pins to 4mA) */
	/* Note: Only SDB GPIO pins need to be programmed to 4mA. SDA and
	   dedicated SDB pins default to 4mA.
	*/
	writel (0x1e0, IO_ADDRESS(GIO4_R_GRPF4_DRV_SEL0_SW_MEMADDR));
	writel (0x21f, IO_ADDRESS(GIO4_R_GRPF4_DRV_SEL1_SW_MEMADDR));
	writel (0, IO_ADDRESS(GIO4_R_GRPF4_DRV_SEL2_SW_MEMADDR));

	/* I2S (set pins to 4mA) */
	writel (0x7fc3f, IO_ADDRESS(GIO3_R_GRPF3_DRV_SEL0_SW_MEMADDR));
	writel (0x3c0, IO_ADDRESS(GIO3_R_GRPF3_DRV_SEL1_SW_MEMADDR));
	writel (0, IO_ADDRESS(GIO3_R_GRPF3_DRV_SEL2_SW_MEMADDR));

	spi_register_board_info(example_spi_bus_info, ARRAY_SIZE(example_spi_bus_info));

	i2c_register_board_info(0, example_i2c0_bus_info, ARRAY_SIZE(example_i2c0_bus_info));
	i2c_register_board_info(1, example_i2c1_bus_info, ARRAY_SIZE(example_i2c1_bus_info));

	/* enable SW ASIC reset */
	call_secure_api(CLS_DMU_REG_WR_ID, 3,  DMU_R_dmu_rst_enable_MEMADDR, 1, 1);

#if 0
extern irqreturn_t pend_sec_irq_handler(int, void *);
	/* register a handler for secure interrupts */
	ret = request_irq(IRQ_PEND_SECURE, pend_sec_irq_handler, IRQF_DISABLED,
		    "pending-secure" , NULL);
	if (ret)
		printk(KERN_ERR "pending-secure: IRQ already in use!\n");
#endif

#ifdef HW_CFG_MMDMA_MEM_TABLE
   board_mmdma_hw_cfg.mem_table = &board_mmdma_mem_table[0];
   board_mmdma_hw_cfg.mem_cnt = ARRAY_SIZE(board_mmdma_mem_table);
   board_mmdma_hw_cfg.tot_mem = board_bootmemheap_calc_mmdma();
#else
   board_mmdma_hw_cfg.mem_table = NULL;
   board_mmdma_hw_cfg.mem_cnt = 0;
   board_mmdma_hw_cfg.tot_mem = 0;
#endif

   platform_device_register(&board_mmdma_device);
}

/****************************************************************************
*
*   Called from setup_arch (in arch/arm/kernel/setup.c) to fixup any tags
*   passed in by the boot loader.
*
*****************************************************************************/

static void __init bcm5892_fixup
(
	struct machine_desc *desc,
	struct tag *t,
	char **cmdline,
	struct meminfo *mi
)
{
   /* Setup bootmem heap calculation pointers */
#ifdef CONFIG_VDEC
   bootmemheap_calc_vdec_mem = board_bootmemheap_calc_vdec_mem;
#endif

   bootmemheap_calc_mmdma = board_bootmemheap_calc_mmdma;
}

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

          for(i = 0; i < 6; i++) {
                  unsigned char num;

                  if((*str == '.') || (*str == ':'))
                          str++;
                  num = str2hexnum(*str++) << 4;
                  num |= (str2hexnum(*str++));
                  ea[i] = num;
          }
  }

static int __init ethaddr_setup(char *macaddr)
{
	volatile u32 hwaddr_lo=0x0; /* used to hold lower 32 bits of mac address when setting up unicast mac address */
        volatile u32 hwaddr_hi=0x0;  /* used to hold higher 32 bits of mac address when setting up unicast mac address */
	int i=0;
	char ethaddr[6];

	if (ethaddr == NULL) {
		printk(KERN_ERR"No MAC address!\n");
		return 1;
	}

	str2eaddr(ethaddr, macaddr);

        printk(KERN_NOTICE "ETHADDR command line: %s\n", ethaddr);

/*
 *	mac_val_low = readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + 0x040c));
 *        mac_val_hi  = readl(IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + 0x0410));
 */

        for(i = 0; i < 4; i++) {
            hwaddr_lo = (hwaddr_lo << 0x08)|ethaddr[i];
		printk(KERN_ERR"hwaddr_lo=0x%x ethaddr=%x\n",hwaddr_lo,ethaddr[i]);
        }
        for(i = 4; i < 6; i++) {
            hwaddr_hi = (hwaddr_hi << 0x08)|ethaddr[i];
		printk(KERN_ERR"hwaddr_hi=0x%x ethaddr=%x\n",hwaddr_hi,ethaddr[i]);
        }

        writel(hwaddr_lo, IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + 0x040c));
        writel(hwaddr_hi, IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + 0x0410));

	printk(KERN_NOTICE "loaddr=0x%08x loaddr_val=0x%08x",IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + 0x040c),hwaddr_lo);
	printk(KERN_NOTICE "hiaddr=0x%08x hiaddr_val=0x%08x",IO_ADDRESS(BCM5892_ETHERNET_MAC_BASE + 0x0410),hwaddr_hi);
        return 1;
}

__setup("ethaddr=", ethaddr_setup);


/****************************************************************************
*
*   Timer related information. It is called from
*   time_init (in arch/arm/kernel/time.c).
*
*****************************************************************************/

static struct sys_timer bcm5892_timer = {
  	.init		= sys_timer_init,
};


/****************************************************************************
*
*   bootmemheap setup. Used to be in timer.c
*   This change is needed due to kernel upgrade from 2.6.27.18 to 2.6.32.9
*
*****************************************************************************/

extern int bootmemheap_setup(void);

static int __init bcm5892_bootmemheap_setup(char *str)
{
        /*
         * The bootmem setup fails in bcmring_fixup() and early_initcall sections.
         * It works as a __setup command line parameter, or here. There are no
         * other logical points exposed to do this setup. It must be done after the
         * MMU is setup, but before the regular kernel allocation is done.
         * So this isn't architecturally a good place to call a bootmem heap
         * setup function, but there is no other alternative at this time.
         * Putting it as a kernel command line argument isn't good since it
         * impacts our and customer bootstraps.
         */
        return bootmemheap_setup();
}

__setup("bootmemheap", bcm5892_bootmemheap_setup);


/****************************************************************************
*
*   Machine Description
*
*****************************************************************************/

MACHINE_START(BCM5892, "Broadcom BCM5892 Chip")
/* Maintainer: Broadcom Corporation */
	.phys_io	= BCM5892_PERIPH_BASE,
	.io_pg_offst	= IO_ADDRESS(BCM5892_PERIPH_BASE), 
  /*	.boot_params	= (BCM5892_SRAM_BASE + 0x100), */
	.boot_params	= (START_DDR + 0x100),
	.fixup		= bcm5892_fixup,
	.map_io		= bcm5892_map_io,
	.init_irq	= bcm5892_init_irq,
	.timer		= &bcm5892_timer,
	.init_machine	= bcm5892_init_machine
MACHINE_END
