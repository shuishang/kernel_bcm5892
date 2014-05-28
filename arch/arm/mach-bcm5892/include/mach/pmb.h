/*****************************************************************************
*  Copyright 2008 Broadcom Corporation.  All rights reserved.
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

typedef enum pmb_mbox_ids {
	SET_BIG_ENDIAN_ID = 0,
	DUMMY_OPEN_FUNC_ID,
	DUMMY_SECURE_FUNC_ID,
	TEMPCTL_ID,
	SMAU_TEST_ID,
	CLS_CRYPTO_INIT_ID,
	CLS_BULK_CIPHER_INIT_ID,
	CLS_BULK_CIPHER_START_ID,
	CLS_BULK_CIPHER_DMA_START_ID,
	CLS_BULK_CIPHER_ALL_ID,
	CLS_GET_HMAC_SHA1_HASH,
	CLS_GET_HMAC_SHA256_HASH,
	CLS_FIPS_SELFTEST_AES_CBC_ID,
	CLS_FIPS_SELFTEST_HMAC_SHA1_ID,
	CLS_FIPS_SELFTEST_HMAC_SHA256_ID,
	CLS_SMU_MEM_INIT_ID,
	CLS_SMU_CONFIG_WIN_ID,
	CLS_DSA_SIGN_ID,
	CLS_DSA_VERIFY_ID,
	/*	CLS_FIPS_SELFTEST_DSA_VERIFY, */
	CLS_RTC_SETTIME_ID,
	CLS_RTC_SETALRM_ID,
	CLS_RTC_GETALRM_ID,
	CLS_RTC_AIE_ON_OFF_ID,
	CLS_RTC_SETPER_ID,	
	CLS_RTC_GETPER_ID,
	CLS_RTC_PIE_ON_OFF_ID,
	CLS_SECURE_INT_ID,
	CLS_DMU_PLL_RESYNC,
	CLS_DMU_PWD,
	CLS_DMU_CPUCLK_SEL,
	CLS_DMU_PLL_ENABLE,
	CLS_DMU_PLL_DISABLE,
	CLS_SPL_LF_MONITOR,
	CLS_DMU_REG_WR_ID,
	CLS_DMU_BLOCK_RESET,
    CLS_RSA_MOD_EXP_ID,
    CLS_RSA_MOD_EXP_CRT_ID,
    CLS_RSA_KEY_GEN_ID,
    OPEN_API_TABLE_INIT,
	CLS_DDR_SELF_RERRESH_ENABLE,

	SCL_TEST_ID,
	INVALID_ID,
} PMB_MBOX_ID;

extern uint32_t (*call_secure_api)(uint32_t mb_id, uint32_t mb_argc, ...) ;
/*
#ifdef CONFIG_IMP_CALL_SECURE_API
extern uint32_t call_secure_api(uint32_t, uint32_t, ...);
#endif 
*/


