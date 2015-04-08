/*
**  Package:
**	
**
**  Abstract:
**      
**
**  Author:
**      Sergio Maldonado, SLAC (smaldona@slac.stanford.edu)
**
**  Creation Date:
**	    000 - July 19, 2013
**
**  Revision History:
**	    None.
**
** --
*/

#include <stdint.h>
#include "rce.h"
#include <config.h>

#include "map/Lookup.h"
#include "map/MapAxi.h"
#include "bsi/Bsi_Cfg.h"
#include "bsi/Bsi.h"
#include "boot/cm.h"

/* GPIO settings */
#define GPIO_DIRM0_ADDR                 0xE000A204
#define GPIO_OEN0_ADDR                  0xE000A208
#define GPIO_BANK0_WRITE_ADDR           0xE000A040
#define GPIO_BANK0_READ_ADDR            0xE000A060
#define GPIO_CFG_VAL                    0x00004000
#define GPIO_DTM_MASK                   (1 << 23)

/* SLCR FPGA clock registers */
#define SLCR_FPGA_CLK_CTL_BASE          0xF8000170
#define SLCR_FPGA_CLK_CTL_SIZE          0x10
#define SLCR_FPGA_CLK_DIV_MASK          0x3f00
#define IO_PLL_CLK_FREQ                 1000000000 /* 1000 Mhz */

/* Ethernet phy configuration register offset */
#define ETH_PHYCFG_OFFSET               13

#define BSI_INSERT(b,s,v)  ((v << (b##_V_##s)) & (b##_M_##s))

static unsigned _isDtm = 0;

/* 
 * Add externs for missing symbols provided in common.h.
 * Cannot include common.h due to confict errors caused by including datCode.hh.
 */
extern int printf(const char *fmt, ...)
		__attribute__ ((format (__printf__, 1, 2)));

extern int sprintf(char *buf, const char *fmt, ...)
		__attribute__ ((format (__printf__, 2, 3)));
        
extern unsigned long get_timer (unsigned long base);

extern void * memset(void *,int,int);

extern void udelay(int usecs);
  
/*
** ++
**
**
** --
*/

int bsi_init(Bsi bsi, uint64_t mac, uint32_t phy)
  {
  int i;
  union {
    uint32_t u32[2];
    uint8_t  u8[8];
    uint64_t u64;
  } macNet, macBsi;

  uint8_t ethaddr[32];  

  uint32_t *axi = (uint32_t*)LookupAxiFw();
  if(!axi) return -1;

  uint32_t fwphy = axi[ETH_PHYCFG_OFFSET];
  if(!fwphy) fwphy = phy;
  
  /* update BSI with eFUSE and device DNA */
  uint32_t efuse = *(uint32_t*)((uint32_t)axi+AXI_EFUSE_OFFSET);
  uint64_t dna = ((uint64_t)(*(uint32_t*)((uint32_t)axi+AXI_DNA0_OFFSET))<<32);
  dna |= (uint64_t)(*(uint32_t*)((uint32_t)axi+AXI_DNA1_OFFSET));

  BsiWrite32(bsi,BSI_EFUSE_OFFSET,efuse);
  BsiWrite64(bsi,BSI_DEVICE_DNA_OFFSET,dna);  
  
  /* if present, use eFUSE to calculate MAC address, otherwise use passed in argument */
  if(efuse)
    {
    uint32_t offset = (efuse >> 14) & 0x3;
    macNet.u64 = BsiMacTable[offset] | (efuse & 0x3FFF);
    macBsi.u64 = macNet.u64;
    sprintf((char *)ethaddr,"eFUSE mac %02x:%02x:%02x:%02x:%02x:%02x",macBsi.u8[5],macBsi.u8[4],macBsi.u8[3],macBsi.u8[2],macBsi.u8[1],macBsi.u8[0]);
    }
  else
    {
    if(!mac)
      {
      printf("%s: no valid mac address in environment!\n",__func__);      
      return -1;
      }
    macNet.u64 = mac;
    macBsi.u64 = 0;
    /* Swap the mac from network order to little endian */
    for (i=0; i<6; i++)
      macBsi.u8[i] = macNet.u8[5-i];
    sprintf((char *)ethaddr,"mac %02x:%02x:%02x:%02x:%02x:%02x",macNet.u8[0],macNet.u8[1],macNet.u8[2],macNet.u8[3],macNet.u8[4],macNet.u8[5]);
    }
  
  /* Initialize the BSI */
  BsiWrite32(bsi,BSI_BOOT_RESPONSE_OFFSET,BSI_BOOT_RESPONSE_NOT_BOOTED);
  BsiSetup(bsi, fwphy, macBsi.u64);
  printf("Net:   %s\n",ethaddr);

  return 0;
  }

/*
** ++
**
**
** --
*/

int gpio_init(void)
  {
  uint32_t gpio;
  int dtm = 0;

  /* Read the GPIO bank 0 value */
  gpio = *(uint32_t *)GPIO_BANK0_READ_ADDR;
    
  /* Check dtm bit to determine if this platform is a dtm */
  if (!(gpio & GPIO_DTM_MASK))
    dtm = 1;

  /* Configure the BSI ready GPIO pin as an output */
  *(uint32_t *)GPIO_DIRM0_ADDR = GPIO_CFG_VAL;

  /* Assert the BSI ready pin */
  *(uint32_t *)GPIO_BANK0_WRITE_ADDR = gpio | GPIO_CFG_VAL;

  /* 
   * Enable GPIO BSI ready pin output.
   * This tells the IPMI Controller that the
   * BSI parameters are ready for reading.
   */
  *(uint32_t *)GPIO_OEN0_ADDR  = GPIO_CFG_VAL;
  
  return dtm;
}

/*
** ++
**
**
** --
*/

int rce_init(uint64_t mac, uint32_t phy)
{
  Bsi bsi = 0;
#ifndef CONFIG_BSI_ENV  
  unsigned long time;
#endif

  bsi = LookupBsi();
  if (!bsi) return -1;
  
  /* write ipmi parameters to the bsi */
  int ret = bsi_init(bsi,mac,phy);
  if(ret != 0) return -1;

#ifndef CONFIG_BSI_ENV
  /* initialize the gpio, signal ipmi, and get dtm flag */
  _isDtm = gpio_init();
#endif

#ifndef CONFIG_BSI_ENV
  /* cm init must be executed after the ipmi has been signaled */
  if (_isDtm)
    {
    BsiWrite32(bsi,BSI_BOOT_RESPONSE_OFFSET,BSI_BOOT_RESPONSE_CM_INIT);
	time = get_timer(0);
    cm_net_init(bsi);
	time = get_timer(time);
	printf("Net:   cm_net_init completed in %lu ms\n", time);
    }
#endif

  BsiWrite32(bsi,BSI_BOOT_RESPONSE_OFFSET,BSI_BOOT_RESPONSE_RCE_READY);
  
  return 0;
}

/*
** ++
**
**
** --
*/

int rce_isdtm(void)
  {
  return _isDtm;
  }


/*
** ++
**
**
** --
*/

void rce_bsi_status(uint32_t status)
{
  Bsi bsi = LookupBsi();
  
  if (!bsi) return;
  
  BsiWrite32(bsi,BSI_BOOT_RESPONSE_OFFSET,status);    
}

/*
** ++
**
**
** --
*/

void rce_bsi_group(const char *buffer)
  {
  Bsi bsi = LookupBsi();

  if (!bsi) return;
  
  BsiWriteGroup(bsi,(char *)buffer);  
  }
  
/*
** ++
**
**
** --
*/

void rce_bsi_cluster(uint32_t cluster, uint32_t bay, uint32_t element)
  {
  Bsi bsi = LookupBsi();
  uint32_t value = (BSI_INSERT(BSI_CLUSTER_ADDR, CLUSTER, cluster) |
                    BSI_INSERT(BSI_CLUSTER_ADDR, BAY,     bay)  |
                    BSI_INSERT(BSI_CLUSTER_ADDR, ELEMENT, element));

  if (!bsi) return;
  
  BsiWrite32(bsi,BSI_CLUSTER_ADDR_OFFSET, value);
  }  

/*
** ++
**
**
** --
*/

int rce_bsi_slot(void)
  {
  Bsi bsi = LookupBsi();
  if (!bsi) return -1;
  
  return BSI_CLUSTER_FROM_CLUSTER_ADDR(BsiRead32(bsi,BSI_CLUSTER_ADDR_OFFSET));
  }
  
/*
** ++
**
**
** --
*/

int rce_bsi_ipinfo(unsigned *ip, unsigned *gw, unsigned *nm)
  {
  Bsi bsi = LookupBsi();
  if (!bsi) return -1;
    
  *ip = (unsigned)BsiRead32(bsi, BSI_CLUSTER_IP_INFO_OFFSET+3);
  *nm = (unsigned)BsiRead32(bsi, BSI_CLUSTER_IP_INFO_OFFSET+4);
  *gw = (unsigned)BsiRead32(bsi, BSI_CLUSTER_IP_INFO_OFFSET+5);
  
  return 0;
  }

/*
** ++
**
**
** --
*/

void rce_uboot_version(const char *buffer, uint32_t len)
  {
  Bsi bsi = LookupBsi();
  unsigned* val = (unsigned*)buffer;
  unsigned  idx = 0;
  
  if (!bsi) return;
  
  do {
    BsiWrite32(bsi,BSI_UBOOT_VERSION_OFFSET+idx++,*val++);
     }
  while ((idx < BSI_UBOOT_VERSION_SIZE) && ((idx*4) < len));
  }  

/*
** ++
**
**
** --
*/

void rce_dat_version(const char *buffer, uint32_t len)
  {
  Bsi bsi = LookupBsi();
  unsigned* val = (unsigned*)buffer;
  unsigned  idx = 0;
  
  if (!bsi) return;
  
  do {
    BsiWrite32(bsi,BSI_DAT_VERSION_OFFSET+idx++,*val++);
     }
  while ((idx < BSI_DAT_VERSION_SIZE) && ((idx*4) < len));
  }  

/*
** ++
**
**
** --
*/

void rce_fpga_clock(uint32_t clk, uint32_t freq)
  {
  uint32_t reg;
  uint32_t val;
  uint32_t div;
  
  if (clk >= NUM_FPGA_CLKS)
    return;
    
  if (!freq || (freq > IO_PLL_CLK_FREQ))
    return;
        
  reg = SLCR_FPGA_CLK_CTL_BASE+(clk*SLCR_FPGA_CLK_CTL_SIZE);
  
  /* read the control register */
  val = *(volatile uint32_t *)reg;
  
  /* calculate divisor0 */
  div = ((IO_PLL_CLK_FREQ/freq) << 8) & SLCR_FPGA_CLK_DIV_MASK;
  val = (val & ~SLCR_FPGA_CLK_DIV_MASK) | div;
  
  /* write the control register */
  *(volatile uint32_t *)reg = val;
  }

/*
** ++
**
**
** --
*/

uint64_t rce_mac(void)
  {
  Bsi bsi = LookupBsi();
  return BsiRead64(bsi,BSI_MAC_ADDR_OFFSET);  
  }
