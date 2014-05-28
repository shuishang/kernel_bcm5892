/*****************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
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
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/crypto.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <asm/byteorder.h>

#include <linux/broadcom/bcm_spu.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>

#include <csp/spuHw.h>
#include <mach/csp/spuHw_inline.h>
#include <mach/csp/cap.h>
#include <mach/csp/rngHw_inline.h>

#define IV_LEN                    12
#define END_MARK_LEN              4
#define ICV_LEN                   16
#define KEY_LEN                   16
#define MAX_AAD_LEN               64
#define MAX_PAYLOAD_LEN           (32 * 1024)
#define AES_BLK_SIZE              16
#define AES_BLK_MASK              (AES_BLK_SIZE - 1)
#define AES_BLK_ALIGNED_LEN(x)    ((x + AES_BLK_MASK) & (~AES_BLK_MASK))    

/* flag to turn on debug prints */
static volatile int gDbg = 0;
#define SPU_TEST_DBG(format, args...) \
   do { if (gDbg) printk(KERN_WARNING format, ## args); } while (0)

#define SPU_TEST_PRINT(format, args...) \
   do { printk(KERN_INFO format, ## args); } while (0)

#define MAX_PROC_BUF_SIZE    256
#define PROC_PARENT_DIR      "spu_aes_gcm_test"
#define PROC_ENTRY_DEBUG     "debug"
#define PROC_ENTRY_RUN       "runTest"

struct proc_dir
{
   struct proc_dir_entry *parent;
};

struct proc_dir gProc;
static spu_dma_context gcm_crypto_dma;

static u8 key[KEY_LEN];
static u8 aad[MAX_AAD_LEN];

static const u8 endmark[END_MARK_LEN] =
{
   0x00, 0x00, 0x00, 0x01
};

static u8 iv[IV_LEN + END_MARK_LEN];

static u8 enc_icv[ICV_LEN];
static u8 dec_icv[ICV_LEN];

static u8 enc_payload[MAX_PAYLOAD_LEN];
static u8 enc_out[MAX_AAD_LEN + MAX_PAYLOAD_LEN];
static u8 dec_in[MAX_AAD_LEN + MAX_PAYLOAD_LEN];
static u8 dec_out[MAX_AAD_LEN + MAX_PAYLOAD_LEN];

static int aes_gcm_encrypt_test(unsigned int random, unsigned int aad_len, unsigned int payload_len)
{
   unsigned int i, status, aad_aligned_len, payload_aligned_len;
   u8 *icv, *buf8p;
   int ret = 0;
   size_t out_len;
   int cmd_len_bytes, output_len_bytes;
   u32 *cmd_buf, *in_buf, *out_buf, *buf32p; 
   spuHw_CONTEXT_t spu_context;

   cmd_buf = gcm_crypto_dma.crypto_cmd.virt; 
   in_buf = gcm_crypto_dma.crypto_in.virt;
   out_buf = gcm_crypto_dma.crypto_out.virt;

   /* both aad and payload need to be aligned to 128-bit */
   aad_aligned_len = AES_BLK_ALIGNED_LEN(aad_len);
   payload_aligned_len = AES_BLK_ALIGNED_LEN(payload_len);

   if (random)
   {
      /* randomly generate key, aad, payload, and iv */
      memset(key, 0, sizeof(key));
      buf32p = (u32 *)key;
      for (i = 0; i < KEY_LEN / 4; i++)
         buf32p[i] = rngHw_getRandomNumber();

      memset(aad, 0, sizeof(aad));
      buf32p = (u32 *)aad;
      for (i = 0; i < aad_len / 4; i++)
         buf32p[i] = rngHw_getRandomNumber();

      memset(enc_payload, 0, sizeof(enc_payload));
      buf32p = (u32 *)enc_payload;
      for (i = 0; i < payload_len / 4; i++)
         buf32p[i] = rngHw_getRandomNumber();

      memset(iv, 0, sizeof(iv));
      buf32p = (u32 *)iv;
      for (i = 0; i < IV_LEN / 4; i++)
         buf32p[i] = rngHw_getRandomNumber();
      /* copy over special end mark */
      memcpy(&iv[IV_LEN], endmark, END_MARK_LEN);
   }
   else
   {
      memset(key, 0, sizeof(key));
      buf32p = (u32 *)key;
      for (i = 0; i < KEY_LEN / 4; i++)
         buf32p[i] = i;

      memset(aad, 0, sizeof(aad));
      buf32p = (u32 *)aad;
      for (i = 0; i < aad_len / 4; i++)
         buf32p[i] = i;

      memset(iv, 0, sizeof(iv));
      buf32p = (u32 *)iv;
      for (i = 0; i < IV_LEN / 4; i++)
         buf32p[i] = i;
      /* copy over special end mark */
      memcpy(&iv[IV_LEN], endmark, END_MARK_LEN);

      memset(enc_payload, 0, sizeof(enc_payload));
      buf32p = (u32 *)enc_payload;
      for (i = 0; i < payload_len / 4; i++)
         buf32p[i] = i;
   }

   /* clear DMA data buffers */
   memset(cmd_buf, 0, SPU_DMA_CMD_BUFFER_LENGTH);
   memset(in_buf, 0, SPU_DMA_DATA_BUFFER_LENGTH);
   memset(out_buf, 0, SPU_DMA_DATA_BUFFER_LENGTH);

   /* copy over the data to the input buffer */
   buf8p = (u8 *)in_buf;
   memcpy(buf8p, aad, aad_aligned_len);
   buf8p += aad_aligned_len;
   memcpy(buf8p, enc_payload, payload_aligned_len);

   memset((void*)&spu_context, 0, sizeof(spu_context));

   if (payload_len)
   {
      spu_context.cryptoAlgo = spuHw_CRYPTO_ALGO_AES;
      spu_context.cryptoMode = spuHw_CRYPTO_MODE_GCM;
      spu_context.cryptoType = spuHw_CRYPTO_TYPE_AES_K128;
   }
   
   spu_context.authAlgo = spuHw_AUTH_ALGO_AES;
   spu_context.authMode = spuHw_AUTH_MODE_GCM;
   spu_context.authType = spuHw_AUTH_TYPE_AES_K128;
   spu_context.authOrder = spuHw_AUTH_ORDER_FIRST;
   
   spu_context.dataAttribute.aesGcmAadLength = aad_len;
   spu_context.dataAttribute.cryptoOffset = aad_aligned_len;
   spu_context.dataAttribute.cryptoLength = payload_len;
   spu_context.dataAttribute.dataLength = aad_aligned_len + payload_aligned_len;
   spu_context.dataAttribute.macOffset = 0;
   spu_context.dataAttribute.macLength = (payload_len)? (aad_aligned_len + payload_len):(aad_len);

   spu_context.contextId = 0x12345678;
   spu_context.operation = spuHw_CRYPTO_OPERATION_ENCRYPTION;
   spu_context.keyType = spuHw_KEY_OPEN;
   spu_context.cryptoKey = key;
   spu_context.cryptoKeyLen = sizeof(key) / sizeof(u32);

   spu_context.icvLen = ICV_LEN / sizeof(u32);
   spu_context.authKeyLen = 0;
   
   spu_context.initVector = iv;
   spu_context.initVectorLen = sizeof(iv) / sizeof(u32);

   SPU_TEST_PRINT("Encryption parameters:\n");
   SPU_TEST_PRINT("aad_len=%d aad_aligned_len=%d, key_len=%d, iv_len=%d icv_len=%d payload_len=%d payload_aligned_len=%d\n", aad_len, aad_aligned_len, sizeof(key), sizeof(iv), ICV_LEN, payload_len, payload_aligned_len);
   
   SPU_TEST_DBG("Test vectors before encryption:\n");
   
   SPU_TEST_DBG("Key:\n");
   buf32p = (u32 *)key;
   for (i = 0; i < KEY_LEN / 4; i++)
      SPU_TEST_DBG("key[%d] 0x%08x\n", i, buf32p[i]);

   SPU_TEST_DBG("128-bit aligned AAD (zero padded):\n");
   buf32p = (u32 *)aad;
   for (i = 0; i < aad_aligned_len / 4; i++)
      SPU_TEST_DBG("aad[%d] 0x%08x\n", i, buf32p[i]);

   SPU_TEST_DBG("IV (padded with 4-byte special end mark):\n");
   buf32p = (u32 *)iv;
   for (i = 0; i < sizeof(iv) / 4; i++)
      SPU_TEST_DBG("iv[%d] 0x%08x\n", i, buf32p[i]);

   SPU_TEST_DBG("128-bit aligned payload (zero padded):\n");
   buf32p = (u32 *)enc_payload;
   for (i = 0; i < payload_aligned_len / 4; i++)
      SPU_TEST_DBG("payload[%d] 0x%08x\n", i, buf32p[i]);

   spu_request();
   
   ret = spu_dma_set_device_handlers(&gcm_crypto_dma);
   if (ret)
   {
      printk(KERN_ERR "spu_dma_set_device_handlers failed\n");
      spu_release();
      return ret;
   }

   cmd_len_bytes = spuHw_createCryptoCommand(&spu_context, cmd_buf);
   SPU_TEST_PRINT("Command length is %d\n", cmd_len_bytes);

   output_len_bytes = spuHw_OUTPUT_HEADER_LEN + 
                      spu_context.dataAttribute.dataLength + 
                      ICV_LEN +
                      spuHw_OUTPUT_STATUS_LEN;

   ret = spuHw_setPacketLength(cmd_len_bytes + spu_context.dataAttribute.dataLength, output_len_bytes);


   /* reserve channels for DMA transfer */
   spu_dma_reserve(&gcm_crypto_dma);

   /* configure the SPU DMA */   
   spu_dma_config(&gcm_crypto_dma, cmd_len_bytes, spu_context.dataAttribute.dataLength, output_len_bytes);

   /* ignite DMA transfers */
   spuHw_initiateDma();

   /* wait for DMA transfer to complete */
   spu_dma_wait(&gcm_crypto_dma);
   
   /* free aquired DMA channels */
   spu_dma_free(&gcm_crypto_dma);

   status = spuHw_getCryptoStatus((spuHw_PACKET_t)out_buf, output_len_bytes);
   if (status != spuHw_CRYPTO_STATUS_SUCCESS)
   {
      printk(KERN_ERR "spuHw_getCryptoStatus failed\n");
      spu_release();
      return -EIO;
   }

   SPU_TEST_DBG("Results after encryption:\n");

   /* get ICV */
   icv = spuHw_getAuthenticationResult(out_buf);
   memcpy(enc_icv, icv, ICV_LEN);
   
   SPU_TEST_DBG("ICV:\n");
   buf32p = (u32 *)enc_icv;
   for (i = 0; i < ICV_LEN / 4; i++)
      SPU_TEST_DBG("icv[%d] 0x%08x\n", i, buf32p[i]);
   
   buf32p = (uint32_t*)spuHw_getData((spuHw_PACKET_t)out_buf, &out_len);
   memcpy(enc_out, buf32p, spu_context.dataAttribute.dataLength);

   /* make sure AAD does not change */
   ret = memcmp(buf32p, aad, aad_aligned_len);
   if (ret)
   {
      printk(KERN_ERR "AAD is changed after encryption\n");
   }

   SPU_TEST_DBG("128-bit aligned AAD (zero padded):\n");
   for (i = 0; i < aad_aligned_len / 4; i++)
      SPU_TEST_DBG("aad[%d] 0x%08x\n", i, buf32p[i]);

   /* advance to the data section */
   buf32p += aad_aligned_len / 4;
   SPU_TEST_DBG("128-bit aligned encrypted payload:\n");
   for (i = 0; i < payload_aligned_len / 4; i++)
      SPU_TEST_DBG("encrypted_payload[%d] 0x%08x\n", i, buf32p[i]);

   spu_release();

   SPU_TEST_PRINT("Encryption done\n");

   return spu_context.dataAttribute.dataLength;
}

static int aes_gcm_decrypt_test(unsigned int aad_len, unsigned int payload_len)
{
   unsigned int i, status, aad_aligned_len, payload_aligned_len;
   u8 *icv, *buf8p;
   int ret = 0;
   size_t out_len;
   int cmd_len_bytes, output_len_bytes;
   u32 *cmd_buf, *in_buf, *out_buf, *buf32p; 
   spuHw_CONTEXT_t spu_context;

   cmd_buf = gcm_crypto_dma.crypto_cmd.virt; 
   in_buf = gcm_crypto_dma.crypto_in.virt;
   out_buf = gcm_crypto_dma.crypto_out.virt;

   /* both aad and payload need to be aligned to 128-bit */
   aad_aligned_len = AES_BLK_ALIGNED_LEN(aad_len);
   payload_aligned_len = AES_BLK_ALIGNED_LEN(payload_len);

   /* clear DMA data buffers */
   memset(cmd_buf, 0, SPU_DMA_CMD_BUFFER_LENGTH);
   memset(in_buf, 0, SPU_DMA_DATA_BUFFER_LENGTH);
   memset(out_buf, 0, SPU_DMA_DATA_BUFFER_LENGTH);

   /* copy over the data to the input buffer */
   buf8p = (u8 *)in_buf;
   memcpy(buf8p, dec_in, aad_aligned_len + payload_aligned_len);

   memset((void*)&spu_context, 0, sizeof(spu_context));

   if (payload_len)
   {
      spu_context.cryptoAlgo = spuHw_CRYPTO_ALGO_AES;
      spu_context.cryptoMode = spuHw_CRYPTO_MODE_GCM;
      spu_context.cryptoType = spuHw_CRYPTO_TYPE_AES_K128;
   }
   
   spu_context.authAlgo = spuHw_AUTH_ALGO_AES;
   spu_context.authMode = spuHw_AUTH_MODE_GCM;
   spu_context.authType = spuHw_AUTH_TYPE_AES_K128;
   spu_context.authOrder = spuHw_AUTH_ORDER_FIRST;
   
   spu_context.dataAttribute.aesGcmAadLength = aad_len;
   spu_context.dataAttribute.cryptoOffset = aad_aligned_len;
   spu_context.dataAttribute.cryptoLength = payload_len;
   spu_context.dataAttribute.dataLength = aad_aligned_len + payload_aligned_len;
   spu_context.dataAttribute.macOffset = 0;
   spu_context.dataAttribute.macLength = (payload_len)? (aad_aligned_len + payload_len):(aad_len);

   spu_context.contextId = 0x12345678;
   spu_context.operation = spuHw_CRYPTO_OPERATION_DECRYPTION;
   spu_context.keyType = spuHw_KEY_OPEN;
   spu_context.cryptoKey = key;
   spu_context.cryptoKeyLen = sizeof(key) / sizeof(u32);

   spu_context.icvLen = ICV_LEN / sizeof(u32);
   spu_context.authKeyLen = 0;
   
   spu_context.initVector = iv;
   spu_context.initVectorLen = sizeof(iv) / sizeof(u32);

   SPU_TEST_PRINT("Decryption parameters:\n");
   SPU_TEST_PRINT("aad_len=%d aad_aligned_len=%d, key_len=%d, iv_len=%d icv_len=%d payload_len=%d payload_aligned_len=%d\n", aad_len, aad_aligned_len, sizeof(key), sizeof(iv), ICV_LEN, payload_len, payload_aligned_len);
   
   spu_request();
   
   ret = spu_dma_set_device_handlers(&gcm_crypto_dma);
   if (ret)
   {
      printk(KERN_ERR "spu_dma_set_device_handlers failed\n");
      spu_release();
      return ret;
   }

   cmd_len_bytes = spuHw_createCryptoCommand(&spu_context, cmd_buf);
   SPU_TEST_PRINT("Command length is %d\n", cmd_len_bytes);

   output_len_bytes = spuHw_OUTPUT_HEADER_LEN + 
                      spu_context.dataAttribute.dataLength + 
                      ICV_LEN +
                      spuHw_OUTPUT_STATUS_LEN;

   ret = spuHw_setPacketLength(cmd_len_bytes + spu_context.dataAttribute.dataLength, output_len_bytes);


   /* reserve channels for DMA transfer */
   spu_dma_reserve(&gcm_crypto_dma);

   /* configure the SPU DMA */   
   spu_dma_config(&gcm_crypto_dma, cmd_len_bytes, spu_context.dataAttribute.dataLength, output_len_bytes);

   /* ignite DMA transfers */
   spuHw_initiateDma();

   /* wait for DMA transfer to complete */
   spu_dma_wait(&gcm_crypto_dma);
   
   /* free aquired DMA channels */
   spu_dma_free(&gcm_crypto_dma);

   status = spuHw_getCryptoStatus((spuHw_PACKET_t)out_buf, output_len_bytes);
   if (status != spuHw_CRYPTO_STATUS_SUCCESS)
   {
      printk(KERN_ERR "spuHw_getCryptoStatus failed\n");
      spu_release();
      return -EIO;
   }

   SPU_TEST_DBG("Results after decryption:\n");

   /* get ICV */
   icv = spuHw_getAuthenticationResult(out_buf);
   memcpy(dec_icv, icv, ICV_LEN);

   ret = memcmp(enc_icv, dec_icv, ICV_LEN);
   if (ret)
   {
      printk(KERN_ERR "ICV check failed\n");
   }
   
   SPU_TEST_DBG("ICV:\n");
   buf32p = (u32 *)dec_icv;
   for (i = 0; i < ICV_LEN / 4; i++)
      SPU_TEST_DBG("icv[%d] 0x%08x\n", i, buf32p[i]);
   
   buf32p = (uint32_t*)spuHw_getData((spuHw_PACKET_t)out_buf, &out_len);
   memcpy(dec_out, buf32p, spu_context.dataAttribute.dataLength);

   /* make sure AAD does not change */
   ret = memcmp(buf32p, aad, aad_aligned_len);
   if (ret)
   {
      printk(KERN_ERR "AAD is changed after decryption\n");
   }

   SPU_TEST_DBG("128-bit aligned AAD (zero padded):\n");
   for (i = 0; i < aad_aligned_len / 4; i++)
      SPU_TEST_DBG("aad[%d] 0x%08x\n", i, buf32p[i]);

   /* advance to the data section */
   buf32p += aad_aligned_len / 4;

   /* make sure decrypted payload matches original payload */
   ret = memcmp(buf32p, enc_payload, payload_len);
   if (ret)
   {
      printk(KERN_ERR "Decrypted payload does not match original payload\n");
   }

   SPU_TEST_DBG("128-bit aligned decrypted payload:\n");
   for (i = 0; i < payload_aligned_len / 4; i++)
      SPU_TEST_DBG("decrypted_payload[%d] 0x%08x\n", i, buf32p[i]);

   spu_release();

   SPU_TEST_PRINT("Decryption done\n");

   return 0;
}

static int
proc_debug_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
   int rc;
   unsigned int debug;
   unsigned char kbuf[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   rc = copy_from_user(kbuf, buffer, count);
   if (rc)
   {
      printk(KERN_ERR "copy_from_user failed status=%d", rc);
      return -EFAULT;
   }

   if (sscanf(kbuf, "%u", &debug) != 1)
   {
      printk(KERN_ERR "echo <debug> > /proc/%s/%s\n",
            PROC_PARENT_DIR, PROC_ENTRY_DEBUG);
      return count;
   }

   if (debug)
      gDbg = 1;
   else
      gDbg = 0;

   return count;
}

static int
proc_debug_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Debug print is %s\n", gDbg ? "enabled" : "disabled");
   
   return len;
}

static int
proc_run_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
   int rc, enc_out_len;
   unsigned int random, aad_len, payload_len;
   unsigned char kbuf[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   rc = copy_from_user(kbuf, buffer, count);
   if (rc)
   {
      printk(KERN_ERR "copy_from_user failed status=%d", rc);
      return -EFAULT;
   }

   if (sscanf(kbuf, "%u %u %u", &random, &aad_len, &payload_len) != 3)
   {
      printk(KERN_ERR "echo <random> <aad_len> <payload_len> > /proc/%s/%s\n",
            PROC_PARENT_DIR, PROC_ENTRY_RUN);
      return count;
   }

   if (aad_len > MAX_AAD_LEN)
   {
      printk(KERN_ERR "Maximum AAD length is %u\n", MAX_AAD_LEN);
      return count;
   }

   if (payload_len > MAX_PAYLOAD_LEN)
   {
      printk(KERN_ERR "Maximum payload length is %u\n", MAX_PAYLOAD_LEN);
      return count;
   }

   enc_out_len = aes_gcm_encrypt_test(random, aad_len, payload_len);
   if (enc_out_len <= 0)
   {
      printk(KERN_ERR "Encryption test failed\n");
      return count;
   }

   /* copy over encrypted data for decryption */
   memcpy(dec_in, enc_out, enc_out_len);

   rc = aes_gcm_decrypt_test(aad_len, payload_len);
   if (rc)
   {
      printk(KERN_ERR "Decryption test failed\n");
      return count;
   }

   return count;
}

static int proc_init(void)
{
   int rc;
   struct proc_dir *proc = &gProc;
   struct proc_dir_entry *proc_debug;
   struct proc_dir_entry *proc_run;

   proc->parent = proc_mkdir(PROC_PARENT_DIR, NULL);

   proc_debug = create_proc_entry(PROC_ENTRY_DEBUG, 0644, proc->parent);
   if (proc_debug == NULL)
   {
      rc = -ENOMEM;
      goto err_del_parent;
   }
   proc_debug->read_proc = proc_debug_read;
   proc_debug->write_proc = proc_debug_write;
   proc_debug->data = NULL;

   proc_run = create_proc_entry(PROC_ENTRY_RUN, 0644, proc->parent);
   if (proc_run == NULL)
   {
      rc = -ENOMEM;
      goto err_del_debug;
   }
   proc_run->read_proc = NULL;
   proc_run->write_proc = proc_run_write;
   proc_run->data = NULL;

   return 0;

err_del_debug:
   remove_proc_entry(PROC_ENTRY_DEBUG, proc->parent);

err_del_parent:
   remove_proc_entry(PROC_PARENT_DIR, NULL);

   return rc;
}

static int proc_term(void)
{
   struct proc_dir *proc = &gProc;

   remove_proc_entry(PROC_ENTRY_RUN, proc->parent);
   remove_proc_entry(PROC_ENTRY_DEBUG, proc->parent);
   remove_proc_entry(PROC_PARENT_DIR, NULL);

   return 0;
}

static int spu_test_init(void)
{
   int ret;
   
   /* initialize SPU DMA context settings */
   ret = spu_dma_context_init(&gcm_crypto_dma);
   if (ret)
   {
      printk(KERN_ERR "spu_dma_context_init failed\n");
      return ret;
   }   
  
   /* SPU DMA buffer allocation */
   ret = spu_dma_alloc(&gcm_crypto_dma);
   if (ret)
   {
      printk(KERN_ERR "spu_dma_alloc failed\n");
      return ret;
   }

   ret = proc_init();
   if (ret)
   {
      printk(KERN_ERR "proc_init failed\n");
      goto free_spu_dma;
   }

   SPU_TEST_PRINT("SPU AES GCM driver initialized\n");

   return 0;

free_spu_dma:
   spu_dma_dealloc(&gcm_crypto_dma);

   return ret;
}

static void spu_test_exit(void)
{
   proc_term();
   spu_dma_dealloc(&gcm_crypto_dma);
}

module_init(spu_test_init);
module_exit(spu_test_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom SPU AES GCM Test Driver");
MODULE_LICENSE("GPL");
