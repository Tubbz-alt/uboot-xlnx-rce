/*
 * (C) Copyright 2012 Michal Simek <monstr@monstr.eu>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <netdev.h>
#include <zynqpl.h>
#include <fpga.h>
#include <version.h>
#include <asm/arch/hardware.h>
#include <asm/arch/sys_proto.h>

#ifdef CONFIG_ZYNQ_RCE
#include "rce.h"
#endif

#define MMC_DEV_NUM        0

DECLARE_GLOBAL_DATA_PTR;

/* Bootmode setting values */
#define BOOT_MODES_MASK    0x0000000F
#define QSPI_MODE          0x00000001
#define NOR_FLASH_MODE     0x00000002
#define NAND_FLASH_MODE    0x00000004
#define SD_MODE            0x00000005
#define JTAG_MODE          0x00000000

#define MAX_ROOTSTR_LEN    64                                                         
#define MAX_IPSTR_LEN      64                                                         
#define MAX_BOOTARGS_LEN   256                                                        
#define MAX_STRBUF_LEN     MAX_BOOTARGS_LEN+MAX_ROOTSTR_LEN+MAX_IPSTR_LEN             

#ifdef CONFIG_FPGA
Xilinx_desc fpga;

/* It can be done differently */
Xilinx_desc fpga010 = XILINX_XC7Z010_DESC(0x10);
Xilinx_desc fpga020 = XILINX_XC7Z020_DESC(0x20);
Xilinx_desc fpga030 = XILINX_XC7Z030_DESC(0x30);
Xilinx_desc fpga045 = XILINX_XC7Z045_DESC(0x45);
Xilinx_desc fpga100 = XILINX_XC7Z100_DESC(0x100);
#endif

#ifdef CONFIG_ZYNQ_LOAD_FPGA
#define FPGA_DEV_NUM       0
void set_fpga_freq(void);
#endif

#ifdef CONFIG_ZYNQ_RCE
#define GROUP_NAME_SIZE    32
const char uboot_version[] __attribute__ ((aligned(4))) = UBOOT_GIT_TAG;
const char dat_version[]   __attribute__ ((aligned(4))) = DAT_SVN_REV;
int fpga_loaded = 0;
int bsi_ready   = 0;
int configure_bsi(void);
int board_load_fpga(void);
#endif

void abort_boot(void)
  {
  setenv("bootdelay", "-1");
  main_loop();  
  }
  
int board_init(void)
{
#ifdef CONFIG_FPGA
	u32 idcode;

	idcode = zynq_slcr_get_idcode();

	switch (idcode) {
	case XILINX_ZYNQ_7010:
		fpga = fpga010;
		break;
	case XILINX_ZYNQ_7020:
		fpga = fpga020;
		break;
	case XILINX_ZYNQ_7030:
		fpga = fpga030;
		break;
	case XILINX_ZYNQ_7045:
		fpga = fpga045;
		break;
	case XILINX_ZYNQ_7100:
		fpga = fpga100;
		break;
	}
#endif

	/* temporary hack to clear pending irqs before Linux as it
	 * will hang Linux
	 */
	writel(0x26d, 0xe0001014);

	/* temporary hack to take USB out of reset til the is fixed
	 * in Linux
	 */
	writel(0x80, 0xe000a204);
	writel(0x80, 0xe000a208);
	writel(0x80, 0xe000a040);
	writel(0x00, 0xe000a040);
	writel(0x80, 0xe000a040);

#ifdef CONFIG_FPGA
	fpga_init();
	fpga_add(fpga_xilinx, &fpga);
#endif

	return 0;
}

int board_late_init(void)
  {
  /* load the fpga fabric */
  board_load_fpga();
    
  /* configure the bsi */
  return configure_bsi();
  }

int board_load_fpga(void)
{
    int ret = 0;

#ifdef CONFIG_FPGA
#ifdef CONFIG_ZYNQ_LOAD_FPGA
    cmd_tbl_t *bcmd;
    ulong size;
    char *filesize;
    char * argv[6] = { "fatload", "mmc", "0:1", FPGA_LOAD_ADDR_STR, FPGA_BIT_FILE, NULL };
	unsigned long time;    
    char      *tmp;
    uint32_t  loadbit = 0;
      
    /* 
     * Update fpga clock frequencies prior to
     * loading the fpga fabric.
     */
    set_fpga_freq();

    /* locate bitstream load option in environment */
    tmp = getenv("loadbit");
    if (tmp != NULL)      
      loadbit = simple_strtoul(tmp, NULL, 16);
      
    if(!loadbit)
      {
      /* use default bitstream included in boot.bin */
      fpga_loaded = 1;
      return 0;      
      }
    
	/* Locate the fatload command */
	bcmd = find_cmd("fatload");
	if (!bcmd) {
		printf("%s: Error - 'fatload' command not present.\n",__func__);
		abort_boot();
	}

    /* 
     * Load the bistream file from the 
     * FAT filesystem into memory.
     */
	if (do_fat_fsload(bcmd, 0, 5, argv) != 0)
      {
        printf("%s: error loading bitstream file %s\n",__func__,argv[4]);
		abort_boot();
      }

    /* 
     * After loading, the filesize environment variable
     * is populated with the file size.
     */
    filesize = getenv("filesize");
	if (NULL == filesize)
      {
      printf("%s: filesize not in environment!\n",__func__);
      abort_boot();
      }

    /* convert file size string into a longword */
    size = simple_strtoul(filesize, NULL, 16);    

    /* reset FPGA logic */
	zynq_slcr_unlock();

	/* Disable AXI interface by asserting FPGA resets */
	writel(0xF, &slcr_base->fpga_rst_ctrl);

	/* Set Level Shifters DT618760 */
	writel(0xF, &slcr_base->lvl_shftr_en);

	/* Enable AXI interface by de-asserting FPGA resets */
	writel(0x0, &slcr_base->fpga_rst_ctrl);

	zynq_slcr_lock();

    /*
     * Load FPGA fabric with the bitstream.
     */
	time = get_timer(0);
    
    ret = fpga_loadbitstream(FPGA_DEV_NUM, (char *)FPGA_LOAD_ADDR, (size_t)size);

	time = get_timer(time);
    
    if (ret != FPGA_SUCCESS)
      {
      printf("%s: error loading fpga bitstream!\n",__func__);      
      abort_boot();
      }
    
    fpga_loaded = 1;

	printf("%d bitstream bytes loaded in %lu ms", (int)size, time);
	if (time > 0) {
		puts(" (");
		print_size(size / time * 1000, "/s");
		puts(")");
	}
	puts("\n");
    
#endif /* CONFIG_ZYNQ_LOAD_FPGA */
#endif /* CONFIG_FPGA */

	return ret;
}

#ifdef CONFIG_CMD_NET
int board_eth_init(bd_t *bis)
{
	u32 ret = 0;

#ifdef CONFIG_XILINX_AXIEMAC
	ret |= xilinx_axiemac_initialize(bis, XILINX_AXIEMAC_BASEADDR,
						XILINX_AXIDMA_BASEADDR);
#endif
#ifdef CONFIG_XILINX_EMACLITE
	u32 txpp = 0;
	u32 rxpp = 0;
# ifdef CONFIG_XILINX_EMACLITE_TX_PING_PONG
	txpp = 1;
# endif
# ifdef CONFIG_XILINX_EMACLITE_RX_PING_PONG
	rxpp = 1;
# endif
	ret |= xilinx_emaclite_initialize(bis, XILINX_EMACLITE_BASEADDR,
			txpp, rxpp);
#endif

#if defined(CONFIG_ZYNQ_GEM)
# if defined(CONFIG_ZYNQ_GEM0)
	ret |= zynq_gem_initialize(bis, ZYNQ_GEM_BASEADDR0,
						CONFIG_ZYNQ_GEM_PHY_ADDR0, 0);
# endif
# if defined(CONFIG_ZYNQ_GEM1)
	ret |= zynq_gem_initialize(bis, ZYNQ_GEM_BASEADDR1,
						CONFIG_ZYNQ_GEM_PHY_ADDR1, 0);
# endif
#endif

	return ret;
}
#endif

#ifdef CONFIG_CMD_MMC
int board_mmc_init(bd_t *bd)
{
	int ret = 0;

#if defined(CONFIG_ZYNQ_SDHCI)
# if defined(CONFIG_ZYNQ_SDHCI0)
	ret = zynq_sdhci_init(ZYNQ_SDHCI_BASEADDR0);
# endif
# if defined(CONFIG_ZYNQ_SDHCI1)
	ret |= zynq_sdhci_init(ZYNQ_SDHCI_BASEADDR1);
# endif
#endif
	return ret;
}
#endif

int dram_init(void)
{
	gd->ram_size = CONFIG_SYS_SDRAM_SIZE;

	zynq_ddrc_init();

	return 0;
}

int set_bootargs(void)
{
  unsigned ip;
  unsigned nm;
  unsigned gw;
  char src[MAX_STRBUF_LEN];
  char dst[MAX_STRBUF_LEN];
  char *bootdefs;
  char ipstr[16];
  char gwstr[16];
  char nmstr[16];
  
  memset(ipstr,0,16);
  memset(gwstr,0,16);
  memset(nmstr,0,16);

  /* get static ip info from bsi */
  int status = rce_bsi_ipinfo(&ip,&gw,&nm);
  if(status)
    {
    printf("%s: error getting bsi ip info - %d\n",__func__,status);
    abort_boot();
    }
  
  /* convert to host order */
  ip = ntohl(ip);
  if(!ip) ip = 0xc0a800fe; // 192.168.0.254
  /* do not assign broadcast address */
  if((ip & 0xff) == 0xff)
    ip -= 1;
  ip -= rce_bsi_slot()-1;
  gw = ntohl(gw);
  if(!gw) gw = 0xc0a80001; // 192.168.0.1
  nm = ntohl(nm);
  if(!nm) nm = 0xffff0000; // 255.255.0.0
      
  if(ip)
    sprintf(ipstr,"%d.%d.%d.%d",(int)(ip>>24)&0xff,(int)(ip>>16)&0xff,(int)(ip>>8)&0xff,(int)((ip>>0)&0xff));
  else
    sprintf(ipstr,"%s","");
  
  if(gw)
    sprintf(gwstr,"%d.%d.%d.%d",(int)(gw>>24)&0xff,(int)(gw>>16)&0xff,(int)(gw>>8)&0xff,(int)(gw>>0)&0xff);  
  else
    sprintf(gwstr,"%s","");

  if(nm)
    sprintf(nmstr,"%d.%d.%d.%d",(int)(nm>>24)&0xff,(int)(nm>>16)&0xff,(int)(nm>>8)&0xff,(int)(nm>>0)&0xff);
  else
    sprintf(nmstr,"%s","");
    
  sprintf(src," ip=%s::%s:%s::eth0",ipstr,gwstr,nmstr);
  
  /* concatenate default bootargs from environment with ip info */
  bootdefs = getenv("bootdefs");
  if(bootdefs)
    {
    int bootlen = strlen(bootdefs);
    strncpy(dst,bootdefs,(bootlen<MAX_BOOTARGS_LEN) ? bootlen : MAX_BOOTARGS_LEN);
    strcat(dst,src);
    }
  
  /* add bootargs to environment */
  setenv("bootargs",dst);
  
  printf("Net:   bsi ip %s::%s:%s::eth0",ipstr,gwstr,nmstr);
  printf("\n");
  
  return 0;
}

int set_rootfs(void)
{
  char buf[MAX_STRBUF_LEN];
  char *args;
  char *mode;
  char *fs;
  
  /* select the correct root fs bootarg for the current boot mode */
  mode = getenv("modeboot");
  if(!mode)
    {
    printf("No modeboot in env\n");
    abort_boot();
    }

  args = getenv("bootargs");
  if(!args)
    {
    printf("No bootargs in env\n");
    abort_boot();
    }

  if(!strcmp(mode,"sdboot_rdisk"))
    fs = getenv("ramdisk_rootfs");
  else
    fs = getenv("sd_rootfs");

  if(!fs)
    {
    printf("No rootfs in env\n");
    abort_boot();
    }
  
  /* ensure storage space for rootfs arg string */  
  int len = strlen(fs);
  if(len > (MAX_ROOTSTR_LEN-strlen(" root=")))
    {
    printf("rootfs exceeds max len of %d\n",MAX_ROOTSTR_LEN-strlen(" root="));
    abort_boot();
    }

  /* concatenate default bootargs from environment with root fs arg */
  if(strlen(args) > (MAX_BOOTARGS_LEN-len))
    {
    printf("bootargs+rootfs exceeds max len of %d\n",MAX_BOOTARGS_LEN);
    abort_boot();
    }

  sprintf(buf,"%s root=%s",args,fs);
  
  /* update bootargs in environment */
  setenv("bootargs",buf);
  
  return 0;
}
  
void show_boot_progress(int val)
  {    
  if (val == BOOTSTAGE_ID_CHECK_BOOT_OS)
    {
    if(!fpga_loaded || !bsi_ready)
      {
      printf("%s: aborting auto-boot - firmware not loaded\n",__func__);
      abort_boot();
      }

    if(rce_is_dtm() && rce_is_dhcp())
      set_rootfs();
    
    rce_bsi_status(BSI_BOOT_RESPONSE_OS_HANDOFF);
    }
  }

#ifdef CONFIG_FPGA
#ifdef CONFIG_ZYNQ_LOAD_FPGA
void set_fpga_freq(void)
  {
  char  env[32];
  char *tmp;
  int i;
  uint32_t freq = 0;

  /* locate any FPGA clock frequencies in environment */
  
  for (i=0; i<NUM_FPGA_CLKS; i++)
    {
    sprintf(env,"fpga%d_clk",i);
    tmp = getenv(env);
    if (tmp != NULL)
      {      
      /* convert frequency string into a longword */
      freq = simple_strtoul(tmp, NULL, 10);
      
      /* update the clock frequency */
      rce_fpga_clock(i,freq);
      }
    }
  }
#endif /* CONFIG_ZYNQ_LOAD_FPGA */
#endif /* CONFIG_FPGA */

#ifdef CONFIG_ZYNQ_RCE
int configure_bsi(void)
  {
  union {
    uint8_t  u8[8];
    uint64_t u64;
  } mac,macBsi;
  uint32_t phy = 0;
  char *tmp;
  int ret = 0;
  
#ifdef CONFIG_BSI_ENV
  char group[GROUP_NAME_SIZE];
  uint32_t cluster = 1;
  uint32_t bay = 1;
  uint32_t element = 1;
  int i;

  /*
   * Set the BSI cluster configuration using
   * the environment variables.
   * This is for testing purposes only.
   */
  tmp = getenv("bsi_group");
  if (tmp != NULL)
    {
    int len = strlen(tmp);
    memset(group,0,GROUP_NAME_SIZE);
    if(len > GROUP_NAME_SIZE)
      len = GROUP_NAME_SIZE;
    for(i=0; i<len; i++)
      group[i] = *tmp++;

    tmp = getenv("bsi_cluster");
	if (tmp != NULL)
      {
      cluster = simple_strtoul(tmp, NULL, 16);
      }

    tmp = getenv("bsi_bay");
	if (tmp != NULL)
      {
      bay = simple_strtoul(tmp, NULL, 16);
      }

    tmp = getenv("bsi_element");
	if (tmp != NULL)
      {
      element = simple_strtoul(tmp, NULL, 16);
      }

    rce_bsi_group(group);
    rce_bsi_cluster(cluster,bay,element);
    }
#endif /* CONFIG_BSI_ENV */

  /*
   * Get the MAC address using
   * the ethaddr environment variable.
   */

  mac.u64 = 0;
  eth_getenv_enetaddr("ethaddr",mac.u8);

  /*
   * Get the PHY configuration using
   * the phycfg environment variable.
   */
  tmp = getenv("phycfg");
  if (tmp == NULL)
    {
    printf("phycfg not in environment!\nNet:   ");
    }
  else
    {     
    phy = simple_strtoul(tmp, NULL, 16);
    }

  ret = rce_init(mac.u64,phy);
  if (!ret) bsi_ready = 1;
  else 
    {
    printf("%s: error initializing bsi - %d!\n",__func__,ret);
    abort_boot();
    }
      
  /* update MAC address in environment */
  macBsi.u64 = rce_mac();
  uint8_t ethaddr[32]; 
  sprintf((char *)ethaddr,"%02x:%02x:%02x:%02x:%02x:%02x",macBsi.u8[0],macBsi.u8[1],macBsi.u8[2],macBsi.u8[3],macBsi.u8[4],macBsi.u8[5]);
  setenv("ethaddr",(char*)ethaddr);
  
  if(rce_is_dtm() && rce_is_dhcp()) set_bootargs();
  
  rce_uboot_version(uboot_version,strlen(uboot_version));
  rce_dat_version(dat_version,strlen(dat_version));
  
  return ret;
  }
#endif /* CONFIG_ZYNQ_RCE */
