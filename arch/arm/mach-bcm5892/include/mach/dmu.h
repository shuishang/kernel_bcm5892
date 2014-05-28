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

#ifndef __DMU_H_
#define __DMU_H_

#include <mach/bcm5892_reg.h>

/*pwd_blk1 register */
#define DMU_ADC0_PWR_ENABLE 	DMU_F_dmu_pwd_adc0_R	
#define DMU_ADC1_PWR_ENABLE	DMU_F_dmu_pwd_adc1_R	
#define DMU_BBL_PWR_ENABLE	DMU_F_dmu_pwd_bbl_R	
#define DMU_CFG_PWR_ENABLE	DMU_F_dmu_pwd_cfg_R	
#define DMU_D1W_PWR_ENABLE	DMU_F_dmu_pwd_d1w_R
#define DMU_DAC_PWR_ENABLE	DMU_F_dmu_pwd_dac_R	
#define DMU_DDR_PWR_ENABLE 	DMU_F_dmu_pwd_ddr_R	
#define DMU_DEC_PWR_ENABLE 	DMU_F_dmu_pwd_dec_R	
#define DMU_ETH_PWR_ENABLE	DMU_F_dmu_pwd_eth_R	
#define DMU_ETM_PWR_ENABLE	DMU_F_dmu_pwd_etm_R	
#define DMU_GIO0_PWR_ENABLE	DMU_F_dmu_pwd_gio0_R	
#define DMU_GIO1_PWR_ENABLE	DMU_F_dmu_pwd_gio1_R	
#define DMU_GIO2_PWR_ENABLE	DMU_F_dmu_pwd_gio2_R	
#define DMU_GIO3_PWR_ENABLE	DMU_F_dmu_pwd_gio3_R	
#define DMU_GIO4_PWR_ENABLE	DMU_F_dmu_pwd_gio4_R	
#define DMU_I2C0_PWR_ENABLE	DMU_F_dmu_pwd_i2c0_R	
#define DMU_I2C1_PWR_ENABLE	DMU_F_dmu_pwd_i2c1_R	
#define DMU_I2S_PWR_ENABLE	DMU_F_dmu_pwd_i2s_R	
#define DMU_LCD_PWR_ENABLE	DMU_F_dmu_pwd_lcd_R	
#define DMU_MEM_PWR_ENABLE	DMU_F_dmu_pwd_mem_R	
#define DMU_MMI_PWR_ENABLE	DMU_F_dmu_pwd_mmi_R	
#define DMU_MSR_PWR_ENABLE	DMU_F_dmu_pwd_msr_R
#define DMU_NVM_PWR_ENABLE	DMU_F_dmu_pwd_nvm_R	
#define DMU_ODMA_PWR_ENABLE	DMU_F_dmu_pwd_odma_R	
#define DMU_PBX_PWR_ENABLE	DMU_F_dmu_pwd_pbx_R	
#define DMU_PBY_PWR_ENABLE	DMU_F_dmu_pwd_pby_R	
#define DMU_PBZ_PWR_ENABLE	DMU_F_dmu_pwd_pbz_R	
#define DMU_PKA_PWR_ENABLE	DMU_F_dmu_pwd_pka_R
#define DMU_PWM_PWR_ENABLE	DMU_F_dmu_pwd_pwm_R	
#define DMU_RNG_PWR_ENABLE    	DMU_F_dmu_pwd_rng_R	
#define DMU_ROM_PWR_ENABLE    	DMU_F_dmu_pwd_rom_R
#define DMU_SCI0_PWR_ENABLE    	DMU_F_dmu_pwd_sci0_R

/*pwd_blk2 register */
#define DMU_SCI1_PWR_ENABLE  	DMU_F_dmu_pwd_sci1_R+32 
#define DMU_SDM0_PWR_ENABLE   	DMU_F_dmu_pwd_sdm0_R+32
#define DMU_SDM1_PWR_ENABLE   	DMU_F_dmu_pwd_sdm1_R+32
#define DMU_SDMA_PWR_ENABLE   	DMU_F_dmu_pwd_sdma_R+32
#define DMU_SMC_PWR_ENABLE    	DMU_F_dmu_pwd_smc_R+32
#define DMU_SMU_PWR_ENABLE    	DMU_F_dmu_pwd_smu_R+32
#define DMU_SPI0_PWR_ENABLE  	DMU_F_dmu_pwd_spi0_R+32 
#define DMU_SPI1_PWR_ENABLE  	DMU_F_dmu_pwd_spi1_R+32 
#define DMU_SPI2_PWR_ENABLE  	DMU_F_dmu_pwd_spi2_R+32 
#define DMU_SPI3_PWR_ENABLE  	DMU_F_dmu_pwd_spi3_R+32 
#define DMU_SPL_PWR_ENABLE   	DMU_F_dmu_pwd_spl_R+32 
#define DMU_TIM0_PWR_ENABLE  	DMU_F_dmu_pwd_tim0_R+32 
#define DMU_TIM1_PWR_ENABLE  	DMU_F_dmu_pwd_tim1_R+32 
#define DMU_TIM2_PWR_ENABLE  	DMU_F_dmu_pwd_tim2_R+32 
#define DMU_TIM3_PWR_ENABLE  	DMU_F_dmu_pwd_tim3_R+32 
#define DMU_TIM4_PWR_ENABLE  	DMU_F_dmu_pwd_tim4_R+32 
#define DMU_TPB_PWR_ENABLE   	DMU_F_dmu_pwd_tpb_R+32 
#define DMU_UMC_PWR_ENABLE   	DMU_F_dmu_pwd_umc_R+32 
#define DMU_URT0_PWR_ENABLE  	DMU_F_dmu_pwd_urt0_R+32 
#define DMU_URT1_PWR_ENABLE  	DMU_F_dmu_pwd_urt1_R+32 
#define DMU_URT2_PWR_ENABLE  	DMU_F_dmu_pwd_urt2_R+32 
#define DMU_URT3_PWR_ENABLE 	DMU_F_dmu_pwd_urt3_R+32 
#define DMU_USB0_PWR_ENABLE  	DMU_F_dmu_pwd_usb0_R+32 
#define DMU_USB1_PWR_ENABLE  	DMU_F_dmu_pwd_usb1_R+32 
#define DMU_USB2_PWR_ENABLE  	DMU_F_dmu_pwd_usb2_R+32 
#define DMU_WDT_PWR_ENABLE   	DMU_F_dmu_pwd_wdt_R+32 


/*rst_blk1 register */
#define DMU_ADC0_RST_ENABLE	DMU_F_dmu_rst_adc0_R	
#define DMU_ADC1_RST_ENABLE	DMU_F_dmu_rst_adc1_R	
#define DMU_BBL_RST_ENABLE	DMU_F_dmu_rst_bbl_R	
#define DMU_CFG_RST_ENABLE	DMU_F_dmu_rst_cfg_R	
#define DMU_D1W_RST_ENABLE	DMU_F_dmu_rst_d1w_R	
#define DMU_DAC_RST_ENABLE	DMU_F_dmu_rst_dac_R	
#define DMU_DDR_RST_ENABLE	DMU_F_dmu_rst_ddr_R	
#define DMU_ETH_RST_ENABLE	DMU_F_dmu_rst_eth_R	
#define DMU_ETM_RST_ENABLE	DMU_F_dmu_rst_etm_R	
#define DMU_GIO0_RST_ENABLE	DMU_F_dmu_rst_gio0_R	
#define DMU_GIO1_RST_ENABLE	DMU_F_dmu_rst_gio1_R	
#define DMU_GIO2_RST_ENABLE	DMU_F_dmu_rst_gio2_R	
#define DMU_GIO3_RST_ENABLE	DMU_F_dmu_rst_gio3_R	
#define DMU_GIO4_RST_ENABLE	DMU_F_dmu_rst_gio4_R	
#define DMU_I2C0_RST_ENABLE	DMU_F_dmu_rst_i2c0_R	
#define DMU_I2C1_RST_ENABLE	DMU_F_dmu_rst_i2c1_R	
#define DMU_I2S_RST_ENABLE	DMU_F_dmu_rst_i2s_R	
#define DMU_LCD_RST_ENABLE	DMU_F_dmu_rst_lcd_R	
#define DMU_MEM_RST_ENABLE	DMU_F_dmu_rst_mem_R	
#define DMU_MMI_RST_ENABLE	DMU_F_dmu_rst_mmi_R	
#define DMU_MSR_RST_ENABLE	DMU_F_dmu_rst_msr_R	
#define DMU_NVM_RST_ENABLE	DMU_F_dmu_rst_nvm_R	
#define DMU_ODMA_RST_ENABLE	DMU_F_dmu_rst_odma_R	
#define DMU_PBX_RST_ENABLE	DMU_F_dmu_rst_pbx_R	
#define DMU_PBY_RST_ENABLE	DMU_F_dmu_rst_pby_R	
#define DMU_PBZ_RST_ENABLE	DMU_F_dmu_rst_pbz_R	
#define DMU_PKA_RST_ENABLE	DMU_F_dmu_rst_pka_R	
#define DMU_PWM_RST_ENABLE	DMU_F_dmu_rst_pwm_R	
#define DMU_DEC_RST_ENABLE	DMU_F_dmu_rst_dec_R	
#define DMU_RNG_RST_ENABLE    	DMU_F_dmu_rst_rng_R	
#define DMU_ROM_RST_ENABLE    	DMU_F_dmu_rst_rom_R	
#define DMU_SCI0_RST_ENABLE    	DMU_F_dmu_rst_sci0_R

/*rst_blk2 register */
#define DMU_SCI1_RST_ENABLE  	DMU_F_dmu_rst_sci1_R+32 
#define DMU_SDM0_RST_ENABLE   	DMU_F_dmu_rst_sdm0_R+32	
#define DMU_SDM1_RST_ENABLE  	DMU_F_dmu_rst_sdm1_R+32 
#define DMU_SDMA_RST_ENABLE  	DMU_F_dmu_rst_sdma_R+32 
#define DMU_SMC_RST_ENABLE   	DMU_F_dmu_rst_smc_R+32 
#define DMU_SMU_RST_ENABLE   	DMU_F_dmu_rst_smu_R+32 
#define DMU_SPI0_RST_ENABLE  	DMU_F_dmu_rst_spi0_R+32 
#define DMU_SPI1_RST_ENABLE  	DMU_F_dmu_rst_spi1_R+32 
#define DMU_SPI2_RST_ENABLE  	DMU_F_dmu_rst_spi2_R+32 
#define DMU_SPI3_RST_ENABLE  	DMU_F_dmu_rst_spi3_R+32 
#define DMU_SPL_RST_ENABLE   	DMU_F_dmu_rst_spl_R+32
#define DMU_TIM0_RST_ENABLE  	DMU_F_dmu_rst_tim0_R+32 
#define DMU_TIM1_RST_ENABLE  	DMU_F_dmu_rst_tim1_R+32
#define DMU_TIM2_RST_ENABLE  	DMU_F_dmu_rst_tim2_R+32 
#define DMU_TIM3_RST_ENABLE  	DMU_F_dmu_rst_tim3_R+32 
#define DMU_TIM4_RST_ENABLE  	DMU_F_dmu_rst_tim4_R+32 
#define DMU_TPB_RST_ENABLE   	DMU_F_dmu_rst_tpb_R+32
#define DMU_UMC_RST_ENABLE   	DMU_F_dmu_rst_umc_R+32 
#define DMU_URT0_RST_ENABLE  	DMU_F_dmu_rst_urt0_R+32
#define DMU_URT1_RST_ENABLE  	DMU_F_dmu_rst_urt1_R+32 
#define DMU_URT2_RST_ENABLE  	DMU_F_dmu_rst_urt2_R+32 
#define DMU_URT3_RST_ENABLE  	DMU_F_dmu_rst_urt3_R+32 
#define DMU_USB0_RST_ENABLE  	DMU_F_dmu_rst_usb0_R+32
#define DMU_USB1_RST_ENABLE  	DMU_F_dmu_rst_usb1_R+32 
#define DMU_USB2_RST_ENABLE  	DMU_F_dmu_rst_usb2_R+32 
#define DMU_WDT_RST_ENABLE   	DMU_F_dmu_rst_wdt_R+32 

uint32_t cls_dmu_block_enable_rst(uint32_t enable);
uint32_t cls_dmu_block_enable(uint32_t enable);
void cls_dmu_block_disable(uint32_t enable);


#endif  /* DMU_H */
