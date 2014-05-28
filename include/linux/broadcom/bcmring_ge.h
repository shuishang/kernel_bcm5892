/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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
 * Description: Public header of the Graphic Engine (GE) driver. The Graphic
 * Engine can perform hardware-accelerated operations including alpha
 * blending, ratsering, and color filling
 */ 

#ifndef _BCMRING_GE_H
#define _BCMRING_GE_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/broadcom/knllog.h>
#else
#include <stdint.h>
#endif

#ifdef __KERNEL__
#define GE_LOG_ENABLED    0
#if GE_LOG_ENABLED
#define GE_LOG            KNLLOG
#else
#define GE_LOG(c,args...)
#endif
#endif

/* max width */
#define GE_MAX_WIDTH     1024

/* max height */
#define GE_MAX_HEIGHT    1024

/*
 * Version of the GE
 */
enum ge_version {
   GE_VERSION_2, /* GEv2 */
   GE_VERSION_3, /* GEv3 */
};

/*
 * Supported color format
 *
 * NOTE: DO NOT modify the enum value or order since it's used as array index
 * for internal lookup tables
 */
enum ge_color_format {
   /* RGB color space */
   GE_COLOR_URGB888 = 0, /* unpacked RGB, 32 bits */
   GE_COLOR_ARGB888, /* ARGB, 32 bits */
   GE_COLOR_PRGB888, /* packed RGB, 24 bits */
   GE_COLOR_RRGB565, /* RGB 565, 16 bits */

   /* YUV color space, only supported in the alpha blending operation */
   GE_COLOR_YUV422I, /* YUV422 Interleave */
   GE_COLOR_YUV420P, /* YUV420 Planar */
   GE_COLOR_YUV420S, /* YUV420 Semi Planar */

   GE_COLOR_MAX,
};

/*
 * Endianess of the pixel
 *
 * NOTE: DO NOT modify the enum value or order since it's used as array index
 * for internal lookup tables
 *
 */
enum ge_endianness {
   GE_ENDIAN_LITTLE = 0,
   GE_ENDIAN_BIG,
   GE_ENDIAN_WCE,

   GE_ENDIAN_MAX,
};

/*
 * Sync/Async operating mode
 */
enum ge_mode {
   GE_MODE_SYNC = 0, /* synchronous mode, may block */
   GE_MODE_ASYNC /* asynchronous mode, return immediately */
};

/*
 * Current status of the operation
 */
enum ge_status {
   /* used as Linux return value, need to be >= 0 */
   GE_STATUS_PENDING = 1, /* pending to be executed */
   GE_STATUS_RUNNING, /* operation is running */
   GE_STATUS_DONE /* operation is done */
};

/*
 * Destination alpha channel option when SRC2 (and DST) is URGB888 or ARGB888
 *
 * NOTE: DO NOT modify the enum value or order since it's used as array index
 * for internal lookup tables
 */
enum ge_dst_alpha_byte {
   /*
    * Keep the result of the operations in the alpha channel byte location of
    * ARGB888 or the dummy byte location of URGB888
    */
   GE_DST_ALPHA_ROPOUT = 0,

   /*
    * Keep the alpha channel in the destination untouched
    */
   GE_DST_ALPHA_UNTOUCH,

   /*
    * Use the global alpha to overwrite the embedded alpha in the
    * destination image
    */
   GE_DST_ALPHA_GLOBAL,

   /*
    * Weight the embedded alpha in the destination by following formula:
    * DST_ALPHA = A +  (1 - A) * alpha (DST)
    */
   GE_DST_ALPHA_WEIGHTED1,

   /*
    * When SRC is URGB888, DST is ARGB888:
    * DST_ALPHA = (1-global_alpha)*alpha(DST)
    *
    * When SRC is ARGB888, DST is ARGB888:
    * DST_ALPHA = (1-(alpha(SRC))*alpha(DST)
    *
    * When DST is URGB888:
    * DST_ALPHA = DummyByte(DST);
    */
   GE_DST_ALPHA_WEIGHTED2,
   GE_DST_ALPHA_MAX,
};

/*
 * Alpha blending parameters
 */
struct ge_alpha {
   /* destination alpha channel option */
   enum ge_dst_alpha_byte dst_alpha_ch;

   /* use global alpha (source needs to be URGB888) */
   unsigned int use_global_alpha;

   /* global alpha [1 255] */
   unsigned char global_alpha; 
   
   /* enable dither */
   unsigned int dither_enable;
   
   /* enable rounding */
   unsigned int round_enable;

   /* flag to indicate if SRC1 pixel has premultiplied alpha */
   unsigned int src1_premult;

   /* flag to indicate if SRC2 pixel has premultiplied alpha */
   unsigned int src2_premult;

   /* enable pipelined color fill in SRC1 */
   unsigned int color_fill_enable;

   /*
    * Color key, used when 'color_fill_enable' is set to 1
    */
   uint32_t color;
};

/*
 * Supported raster operations
 *
 * NOTE: DO NOT modify the enum value or order since it's used as array index
 * for internal lookup tables
 */
enum ge_raster_op {
   GE_RASTER_BLACK = 0, /* 0 */
   GE_RASTER_NS1_AND_NS2, /* ~SRC1 & ~SRC2 */
   GE_RASTER_NS1_AND_S2, /* ~SRC1 & SRC2 */
   GE_RASTER_NS1, /* ~SRC1 */
   GE_RASTER_S1_AND_NS2, /* SRC1 & ~SRC2 */
   GE_RASTER_NS2, /* ~SRC2 */
   GE_RASTER_S1_XOR_S2, /* SRC1 ^ SRC2 */
   GE_RASTER_NS1_OR_NS2, /* ~SRC1 | ~SRC2 */
   GE_RASTER_S1_AND_S2, /* SRC1 & SRC2 */
   GE_RASTER_S1_XNOR_S2, /* SRC1 ~^ SRC2 */
   GE_RASTER_S2, /* SRC2 */
   GE_RASTER_NS1_OR_S2, /* ~SRC1 | SRC2 */
   GE_RASTER_S1, /* SRC1, copyarea */
   GE_RASTER_S1_OR_NS2, /* SRC1 | ~SRC2 */
   GE_RASTER_S1_OR_S2, /* SRC1 | SRC2 */
   GE_RASTER_WHITE, /* 1 */

   GE_RASTER_MAX,
};

/*
 * Raster operation parameters
 */
struct ge_raster {
   /* raster operations */
   enum ge_raster_op op;

   /* enable pipelined color fill in SRC1 */
   unsigned int color_fill_enable;

   /*
    * To enable transparency
    *
    * If the format of SRC1 data is not same as the destination, the SRC1 data
    * will do color expansion before color key transparent operation. After
    * done the color expansion, the expanded SRC1 data is compared with the
    * 'color' parameter. If they are matched, the graphic engine writes back
    * the original DST data. If they are not matched, the graphic engine writes
    * raster operation output to overwrite the original DST data in memory
    */
   unsigned int trans_enable;

   /*
    * To enable transparency operation on the alpha channel
    */
   unsigned int trans_alpha_ch_enable;

   /*
    * Color key, used when 'color_fill_enable' or 'trans_enable' is set to 1
    */
   uint32_t color;

   /*
    * The following parameters are for manipulation of the destination alpha
    * channel
    */

   /* destination alpha channel option */
   enum ge_dst_alpha_byte dst_alpha_ch;

   /* use global alpha (source needs to be URGB888) */
   unsigned int use_global_alpha;

   /* global alpha [1 255] */
   unsigned char global_alpha; 
   
   /* enable dither */
   unsigned int dither_enable;
   
   /* enable rounding */
   unsigned int round_enable;
};

/*
 * Data structure that conatins the information of the GE
 */
struct ge_info {
   enum ge_version version; /* version of the GE */
};

/*
 * Data buffer
 */
struct ge_buf {
   void *ptr; /* virtual address */
   uint32_t addr; /* physical address */
   unsigned int len; /* buffer size in bytes */
};

/*
 * Supported Graphic Engine operations
 */
enum ge_operation {
   GE_OP_ALPHA_BLEND, /* the name is quite obvious */
   GE_OP_RASTER, /* BitBlt using the raster operator */
   GE_OP_FILL_COLOR, /* fill a beautiful color */
   GE_OP_NOP, /* used for sync up all previous async operations */
   GE_OP_INVALID
};

/*
 * GE parameter
 *
 * Limitations:
 * 1. Source 2 and Destination need to have the same color format and
 *    endianness
 */
struct ge_param {
   enum ge_mode mode; /* operating mode */

   enum ge_operation operation; /* desired GE operation */
   union {
      struct ge_alpha alpha; /* alpha blending parameters */
      uint32_t color; /* color pixel information for color fill */
      struct ge_raster raster; /* raster operations */
   } option;

   enum ge_color_format src1_format; /* color format of src1 */
   enum ge_color_format src2_dst_format; /* color format of src2 and dst */

   enum ge_endianness src1_endian; /* endianess of src1 */
   enum ge_endianness src2_dst_endian; /* endianess of src2 and dst */

   unsigned int s1x; /* X offset of the 1st src (pixels) */
   unsigned int s1y; /* Y offset of the 1st src (pixels) */
   unsigned int s2x; /* X offset of the 2nd src (pixels) */
   unsigned int s2y; /* Y offset of the 2nd src (pixels) */
   unsigned int dx; /* X offset of the dst (pixels) */
   unsigned int dy; /* Y offset of the dst (pixels) */

   unsigned int width; /* width in pixels (for YUV in bytes) */
   unsigned int height; /* height in pixels (for YUV in bytes) */

   unsigned int src1_pitch; /* pixels per line of src1 (for YUV in bytes) */
   unsigned int src2_dst_pitch; /* pixels per line of src2 and dst (for YUV in bytes) */

   uint32_t src1_addr; /* SRC1 buffer address (physical) */
   uint32_t src2_addr; /* SRC2 buffer address (physical) */
   uint32_t dst_addr; /* DST buffer address (physical) */

   /*
    * SRC1 address (physical) for Chroma Cb in YUV420 planar format or
    * Chroma CbCr in YUV420 semi-planar format
    */
   uint32_t src1_uaddr;

   /* SRC1 address (physical) for Chroma Cr in YUV420 planar format */
   uint32_t src1_vaddr;

   uint32_t src2_uaddr;
   uint32_t src2_vaddr;
   uint32_t dst_uaddr;
   uint32_t dst_vaddr;
};

/*
 * IOCTLs for user applications
 */

#define GE_MAGIC               'G'
#define GE_CMD_INFO_GET        0x01
#define GE_CMD_ENGAGE          0x02
#define GE_CMD_STATUS_GET      0x03

/*
 * Get the GE version
 */
#define GE_IOCTL_INFO_GET      _IOR(GE_MAGIC, GE_CMD_INFO_GET, struct ge_info)

/*
 * In asynchronous mode, upon successful return, a handle (>= 0) will be
 * returned. The user can then use this handle to check if the operation has
 * completed (calling GE_IOCTL_STATUS_GET)
 */
#define GE_IOCTL_ENGAGE        _IOW(GE_MAGIC, GE_CMD_ENGAGE, struct ge_param)

/*
 * Used in asynchronous mode only. An operation handle obtained from
 * GE_IOCTL_ENGAGE is submitted. Upon successful return, enum ge_status will
 * be returned
 */
#define GE_IOCTL_STATUS_GET    _IOW(GE_MAGIC, GE_CMD_STATUS_GET, int)

#ifdef __KERNEL__
/*
 * Engage the Graphic Engine. Can be either synchronous or asynchronous
 */
extern int ge_engage(struct ge_param *param);
#endif /* __KERNEL__ */

#endif /* _BCMRING_GE_H */
