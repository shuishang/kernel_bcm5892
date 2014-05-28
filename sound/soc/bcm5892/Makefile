#===================================================
# Makefile for ALSA SOC BRCM BCM5892
#===================================================
#
# to use DMA instead of FIFO
EXTRA_CFLAGS := -DBCM5892_ASOC_DMA=1
#
# Platform Support
#
snd-bcm5892-asoc-pcm-objs := bcm5892_asoc_pcm.o bcm5892_asoc_datahandling.o
snd-bcm5892-asoc-i2s-objs := bcm5892_asoc_i2s.o
#
obj-$(CONFIG_SND_BCM5892_SOC) += snd-bcm5892-asoc-pcm.o
obj-$(CONFIG_SND_BCM5892_SOC) += snd-bcm5892-asoc-i2s.o 

#
# Machine Support
#
# Internal DAC
snd-bcm5892-asoc-card-internal-objs  := bcm5892_asoc_card_internal.o
snd-bcm5892-asoc-codec-internal-objs := bcm5892_asoc_codec_internal.o
#
obj-$(CONFIG_SND_BCM5892_SOC) += snd-bcm5892-asoc-card-internal.o
obj-$(CONFIG_SND_BCM5892_SOC) += snd-bcm5892-asoc-codec-internal.o
#
# External Codec
snd-bcm5892-asoc-card-wm8750-objs  := bcm5892_asoc_card_wm8750.o
#
obj-$(CONFIG_SND_BCM5892_SOC) += snd-bcm5892-asoc-card-wm8750.o
