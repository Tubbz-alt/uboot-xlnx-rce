/*
 *
 * Configuration for Zynq RCE Board
 * See zynq_common.h for Zynq common configs
 *
 */

#ifndef __CONFIG_ZYNQ_RCE_H
#define __CONFIG_ZYNQ_RCE_H

#define PHYS_SDRAM_1_SIZE (512 * 1024 * 1024)

#define CONFIG_CPU_FREQ_HZ		800000000

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

#include <configs/zynq_common.h>

#endif /* __CONFIG_ZYNQ_RCE_H */
