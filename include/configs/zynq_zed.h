/*
 * (C) Copyright 2012 Xilinx
 *
 * Configuration for Zynq Evaluation and Development Board - ZedBoard
 * See zynq_common.h for Zynq common configs
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_ZYNQ_ZED_H
#define __CONFIG_ZYNQ_ZED_H

#define PHYS_SDRAM_1_SIZE (512 * 1024 * 1024)

#define CONFIG_ZYNQ_SERIAL_UART1
#define CONFIG_ZYNQ_GEM0
#define CONFIG_ZYNQ_GEM_PHY_ADDR0	0

#define CONFIG_ZYNQ_RCE

#define CONFIG_ZYNQ_LOAD_FPGA

#define CONFIG_SYS_NO_FLASH

#define CONFIG_ZYNQ_SDHCI0

#define CONFIG_MMC
#define CONFIG_SYS_MMC_ENV_DEV 0  

#define CONFIG_ENV_IS_IN_FAT      
#define CONFIG_FAT_WRITE          

#define FAT_ENV_INTERFACE "mmc"   
#define FAT_ENV_DEVICE     0
#define FAT_ENV_PART       1
#define FAT_ENV_FILE      "uboot.env"

/* populate BSI group/cluster with environment values */
#define CONFIG_BSI_ENV

#include <configs/zynq_common.h>

#endif /* __CONFIG_ZYNQ_ZED_H */
