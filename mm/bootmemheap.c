/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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

/*
*
*****************************************************************************
*
*  bootmemheap.c
*
*  PURPOSE:
*  A simple API for allocating and holding regions and never freeing them.
*  This is to avoid fragmentation of a	large pool, which typically contains
*  regions such as framebuffers,	video decoder memory pools, etc.
*
*	The allocation also contains an identifier string. This is recorded
*	against the memory allocated. Since free's are not done, a subsequent
*	alloc with the same string will return the same memory again. This would
*	happen if a module was unloaded and reloaded, for example.
*
*  NOTES:
*
*****************************************************************************/
/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/broadcom/bootmemheap.h>
#include <linux/ioport.h>
#include <asm/uaccess.h>
#include <cfg_global.h>

#define PROCNAME "bootmemheap"
#define MAX_IDS 	128			  /* Maximum number of allocations from the big pool */
#define MAX_IDLEN  32			  /* Maximum mem identifier string length */


typedef struct
{
	char name[MAX_IDLEN];		  /* Name of memory region or owner */
	void *address;					  /* Allocated memory address */
	size_t size;					  /* Size of allocation */
	size_t alignedsize;			  /* Size rouned up to next page size */
}
heap_t;

static unsigned int heap_pages = 0;
static caddr_t heap_base = 0;
static caddr_t heap_next;
static caddr_t heap_end;
static struct resource mem_resource = {
	.name = PROCNAME,
	.start = 0,
	.end = 0,
	.flags = IORESOURCE_MEM | IORESOURCE_BUSY
};

/* Declare storage for maximum unique allocations */
static heap_t gObj[MAX_IDS];	  /* Memory info for all allocations */
static int gNumAllocs = 0;		  /* Number of allocations */
static int gSetupCalled = 0;	  /* Whether setup function called or not */
DECLARE_MUTEX(sem);

static int read_proc(char *page, char **start, off_t off, int count, int *eof,
							void *data);
static int write_proc(struct file *file, const char __user * buffer,
							 unsigned long count, void *data);


/*
 * Calculate LCD Framebuffer size required. Note: This
 * needs to move into individual driver code in future
 * for better flexibility. Can't #ifdef based on the
 * .config options since modules will likely be involved
 * and code may not be loaded. Also can't do it in __earlyinit
 * since this is too late for setup time. May have to generate
 * the numbers from a makefile script or other tool in future.
 * Another option is to do coarse allocation (aka heap) based
 * on basic configurations and not be so picky about exact fits.
 * For example, we could always allocate based on frame sizes
 * for largest displays.
 */

#if defined( CFG_GLOBAL_LCD_SIZE )
#define CALL_LCD_HEAP_SIZE  1
#else
#define CALL_LCD_HEAP_SIZE  0
#endif

#if CALL_LCD_HEAP_SIZE
size_t (*bootmemheap_calc_fb_mem)( void ) = NULL;
static unsigned int LcdHeapSize(void)
{
    size_t  fbsize = 0;

    if (bootmemheap_calc_fb_mem)
        fbsize = bootmemheap_calc_fb_mem();

    /* Round up to an even page size */

	fbsize = PAGE_SIZE * ((fbsize + PAGE_SIZE - 1) / PAGE_SIZE);
	printk(KERN_INFO "%s: framebuffer rounded up to page size = %u = 0x%x bytes\n",
			 __func__, fbsize, fbsize);
	return fbsize;
}
#endif

#ifdef CONFIG_VDEC
size_t (*bootmemheap_calc_vdec_mem)( void ) = NULL;
EXPORT_SYMBOL(bootmemheap_calc_vdec_mem);
/*
 * Calculate VDEC heap size required. Note: This
 * needs to move into individual lcd driver code in future
 * for better flexibility.
 */
static unsigned int VdecHeapSize(void)
{
   unsigned int vdecsize = 0;
   if (bootmemheap_calc_vdec_mem)
      vdecsize = bootmemheap_calc_vdec_mem();
   vdecsize = PAGE_SIZE * ((vdecsize + PAGE_SIZE - 1) / PAGE_SIZE);
	printk(KERN_INFO "%s: vdec heapsize = 0x%x bytes\n", __func__, vdecsize);
	return vdecsize;
}
#endif

size_t (*bootmemheap_calc_mmdma)( void ) = NULL;
static unsigned int mmDmaHeapSize(void)
{
   size_t mmdmasize = 0;

   if (bootmemheap_calc_mmdma)
      mmdmasize = bootmemheap_calc_mmdma();

    /* Round up to an even page size */
	mmdmasize = PAGE_SIZE * ((mmdmasize + PAGE_SIZE - 1) / PAGE_SIZE);
	printk(KERN_INFO "%s: mmdma heap size = %u = 0x%x bytes\n",
			 __func__, mmdmasize, mmdmasize);
	return mmdmasize;
}

/*
 * Setup the bootmem. This is called from core.c, in the bcmring_timer_init()
 * function, since this is a callout that works and is at the right place
 * in the init sequence. This avoids us having to use a __setup cmd line
 * parameter in the boot2 bootstrap code.
 */
int __init bootmemheap_setup(void)
{
	unsigned int heapsize = 0;

	gSetupCalled = 1;

  	/*
	 * FIXME: the numbers that come from the 2 video drivers
	 * and the VDEC driver need to be available at setup time
	 * which means they need to be compile time constants
	 * rather than dynamically registered.
	 */
#if CALL_LCD_HEAP_SIZE
	heapsize += LcdHeapSize();
#endif

#ifdef  CONFIG_VDEC
	heapsize += VdecHeapSize();
#endif

   heapsize += mmDmaHeapSize();

	heap_pages = heapsize / PAGE_SIZE;
	printk(KERN_INFO "%s: total heapsize = 0x%x bytes = %u pages\n", __func__, heapsize,
			 heap_pages);

	if (heap_pages == 0)
	{
		printk(KERN_INFO "%s: nothing configured to allocate\n", __func__);
		return 0;
	}
	/* Alloc the memory */
	heap_base = alloc_bootmem_low_pages(heap_pages << PAGE_SHIFT);
	if (!heap_base)
	{
		printk(KERN_CRIT "%s: not enough memory for %d pages\n", __func__,
				 heap_pages);
		return -ENOMEM;
	}
	printk(KERN_INFO "%s: Allocated %d pages at 0x%p.\n", __func__, heap_pages,
			 heap_base);
	heap_next = heap_base;
	heap_end = heap_base + heap_pages * PAGE_SIZE;

	/* register the resource */
	mem_resource.start = (unsigned long) heap_base;
	mem_resource.end = mem_resource.start + (heap_pages << PAGE_SHIFT);
	request_resource(&iomem_resource, &mem_resource);
	return 0;
}

/* Initialize proc entries and memory allocation information */
static int __init bootmemheap_init(void)
{
	struct proc_dir_entry *res;
	res = create_proc_entry(PROCNAME, 0444, NULL);
	if (!res)
	{
		return -ENOMEM;
	}
	res->read_proc = read_proc;
	res->write_proc = write_proc;
	memset(gObj, 0, sizeof(gObj));

	if (!gSetupCalled)
	{
		printk(KERN_ERR "%s: Error - bootmemheap_setup() not called\n", __func__);
		return -1;
	}
	if (heap_pages == 0)
	{
		printk(KERN_WARNING "%s: size=0. Check CFG_GLOBAL_LCD_SIZE and hw_cfg settings.\n",
				 __func__);
	}
	return 0;
}

__initcall(bootmemheap_init);


/*
 * Allocate from bootmemheap pool using identifier string and requested size.
 * This memory is contiguous and DMA'able. Future: create unmapped guard
 * pages between each region to catch trampling.
 */
void *bootmemheap_alloc(char *memid_str, size_t size)
{
	heap_t *objp;
	unsigned int i;
	unsigned int pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;
	unsigned int remaining = heap_end - heap_next;

	if (!gSetupCalled)
	{
		printk(KERN_ERR "%s: Error - bootmemheap_setup() not called\n", __func__);
		return NULL;
	}

	if (!size || !memid_str)
	{
		printk(KERN_ERR "%s: size (%d) or memid_str (\'%s\')must be non-NULL\n", __func__, size, memid_str);
		return NULL;
	}
	if (down_interruptible(&sem))
	{
		printk(KERN_ERR "%s: down_interruptible failed\n", __func__);
		return NULL;
	}

	for (i = 0, objp = &gObj[0]; i < gNumAllocs; i++, objp++)
	{
		if (strcmp(memid_str, objp->name) == 0)
		{
			if (size > objp->alignedsize)
			{
				printk(KERN_ERR
						 "%s: \'%s\' re-requesting larger size than available 0x%x > 0x%x\n",
						 __func__, objp->name, size, objp->alignedsize);
				up(&sem);
				return NULL;
			}
			printk(KERN_INFO "%s:  Successfully re-requested \'%s\'\n", __func__,
					 objp->name);
			objp->size = size;	  /* Can request larger size if it doesn't exceed next page boundary */
			up(&sem);
			return objp->address;
		}
	}
	if ((pages * PAGE_SIZE) > remaining)
	{
		printk(KERN_ERR "%s: Error - Heap exhausted for \'%s\' with size 0x%x\n",
				 __func__, memid_str, size);
		return NULL;
	}
	if (gNumAllocs >= MAX_IDS)
	{
		printk(KERN_ERR "%s: Error - Number of Allocs (%d) >= MAX_IDS (%d)\n", __func__, gNumAllocs, MAX_IDS);
		return NULL;
	}
	objp = &gObj[gNumAllocs];
	objp->address = heap_next;
	heap_next += (pages * PAGE_SIZE);
	if (objp->address != NULL)
	{
		strncpy(objp->name, memid_str, sizeof(objp->name));
		objp->size = size;
		objp->alignedsize = pages * PAGE_SIZE;
		gNumAllocs++;
		printk(KERN_INFO "%s: \'%s\' allocated 0x%x bytes at 0x%p\n", __func__,
				 memid_str, size, objp->address);
	}
	up(&sem);
	return (objp->address);
}

/* /proc/bootmemheap read function */
static int read_proc(char *page, char **start, off_t off, int count, int *eof,
							void *data)
{
	unsigned int i;
	unsigned int len = 0;
	heap_t *objp;

	len +=
		sprintf(page + len,
				  "\nHeapSize  0x%08x\n"
				  "HeapUsed  0x%08x\n"
				  "HeapFree  0x%08x\n\n"
				  "HeapBase  0x%p\n"
				  "HeapEnd   0x%p\n" "HeapNext  0x%p\n\n", heap_end - heap_base,
				  heap_next - heap_base, heap_end - heap_next, heap_base, heap_end,
				  heap_next);
	len +=
		sprintf(page + len,
				  "Slot     Address_Range       Size_Dec    Size_Hex   Owner\n");
	for (i = 0, objp = &gObj[0]; i < gNumAllocs; i++, objp++)
	{
		len +=
			sprintf(page + len, "%-4d 0x%p-0x%p   %8u  0x%08x   %-16s\n", i,
					  objp->address, objp->address + objp->size - 1, objp->size,
					  objp->size, objp->name);
	}
	return len;
}

/* /proc/bootmemheap write function */
static int write_proc(struct file *file, const char __user * buffer,
							 unsigned long count, void *data)
{
	int rc;
	unsigned int testAllocSize;
	char testName[MAX_IDLEN];
	unsigned char kbuf[256];

	testAllocSize = 0;
	testName[0] = '\0';
	rc = copy_from_user(kbuf, buffer, count);
	if (rc)
	{
		printk(KERN_ERR "copy_from_user failed status=%d", rc);
		return -EFAULT;
	}
	if ((sscanf(kbuf, "%s %x", testName, &testAllocSize) != 2)
		 || !(testAllocSize && testName[0]))
	{
		printk(KERN_ERR "echo <name hexsize> > /proc/%s\n", PROCNAME);
		return count;
	}
	if (bootmemheap_alloc(testName, testAllocSize) == NULL)
	{
		printk(KERN_ERR "\'%s\' failed to alloc 0x%x bytes\n", testName,
				 testAllocSize);
	}
	return count;
}

EXPORT_SYMBOL(bootmemheap_alloc);
