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

/* ---- Include Files ---------------------------------------------------- */
#include <linux/module.h>

#include <linux/broadcom/gos/std/arpa/inet.h>
#include <linux/string.h>

#include <linux/time.h>
#include <linux/hrtimer.h>

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */
/* ---- Private Function Prototypes -------------------------------------- */
uint32_t htonl_kernel(uint32_t hl);
uint16_t htons_kernel(uint16_t hs);
uint32_t ntohl_kernel(uint32_t nl);
uint16_t ntohs_kernel(uint16_t ns);

void *memset_kernel(void *s, int c, size_t n);
char *strcpy_kernel(char *dst,const char *src);
int strncmp_kernel(const char *src1, const char *src2, size_t s);
void * memcpy_kernel(void *dst, const void *src, size_t n);

int atoi_kernel(const char * str);

int clock_gettime_kernel(clockid_t clk_id, struct timespec *tp);
int usleep_kernel(useconds_t useconds);

/* ---- Functions -------------------------------------------------------- */
uint32_t htonl_kernel(uint32_t hl)
{
   return ___htonl(hl);
}
EXPORT_SYMBOL(htonl_kernel);

uint16_t htons_kernel(uint16_t hs)
{
   return ___htons(hs);
}
EXPORT_SYMBOL(htons_kernel);

uint32_t ntohl_kernel(uint32_t nl)
{
   return ___ntohl(nl);
}
EXPORT_SYMBOL(ntohl_kernel);

uint16_t ntohs_kernel(uint16_t ns)
{
   return ___ntohs(ns);
}
EXPORT_SYMBOL(ntohs_kernel);

void* memset_kernel(void *s, int c, size_t n)
{
   return memset(s, c, n);
}
EXPORT_SYMBOL(memset_kernel);

char *strcpy_kernel(char *dst,const char *src)
{
   return strcpy(dst, src);
}
EXPORT_SYMBOL(strcpy_kernel);

char *strncpy_kernel(char *dst,const char *src, size_t s)
{
   return strncpy(dst, src, s);
}
EXPORT_SYMBOL(strncpy_kernel);

int strcmp_kernel(const char *src1, const char *src2)
{
   return strcmp(src1, src2);
}
EXPORT_SYMBOL(strcmp_kernel);

int strncmp_kernel(const char *src1, const char *src2, size_t s)
{
   return strncmp(src1, src2, s);
}
EXPORT_SYMBOL(strncmp_kernel);

int strlen_kernel(const char *s)
{
   return strlen(s);
}
EXPORT_SYMBOL(strlen_kernel);

void* memcpy_kernel(void *dst, const void *src, size_t n)
{
   return memcpy(dst, src, n);
}
EXPORT_SYMBOL(memcpy_kernel);

int memcmp_kernel(const void *src1,const void *src2, size_t s)
{
   return memcmp(src1, src2, s);
}
EXPORT_SYMBOL(memcmp_kernel);

void* memmove_kernel(void *to, const void *from, size_t s)
{
   return memmove(to, from, s);
}
EXPORT_SYMBOL(memmove_kernel);

int atoi_kernel(const char * str)
{
   return (int)(simple_strtol(str, NULL, 0));
}
EXPORT_SYMBOL(atoi_kernel);

int clock_gettime_kernel(clockid_t clk_id, struct timespec *tp)
{
   switch (clk_id)
   {
      case CLOCK_REALTIME:
         {
            ktime_get_real_ts(tp);
         }
         break;
      case CLOCK_MONOTONIC:
         {
            ktime_get_ts(tp);
         }
         break;
      default:
         return -EINVAL;
   }
   return 0;
}
EXPORT_SYMBOL(clock_gettime_kernel);

int usleep_kernel(useconds_t useconds)
{
   struct timespec tu;
   
   if(useconds < 1000000)
   {
      tu.tv_sec = 0;
      tu.tv_nsec = useconds * 1000;
   }
   else
   {
      tu.tv_sec = useconds / 1000000;
      tu.tv_nsec = useconds % 1000000 * 1000;
   }
   return (int)hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
}
EXPORT_SYMBOL(usleep_kernel);

