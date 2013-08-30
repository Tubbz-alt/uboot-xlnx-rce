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

#include "datCode.hh"
#include "rce.h"

#include DAT_PUBLIC(tool,    map,  Lookup.h)
#include DAT_PUBLIC(tool,    map,  MapAxi.h)
#include DAT_PUBLIC(tool,    map,  MapOcm.h)
#include DAT_PUBLIC(service, cmb,  BSI_cfg.h)
#include DAT_PUBLIC(service, cmb,  Bsi.h)
#include DAT_PUBLIC(cm,      boot, cm.h)

/* GPIO settings */
#define GPIO_DIRM0_ADDR                 0xE000A204
#define GPIO_OEN0_ADDR                  0xE000A208
#define GPIO_BANK0_WRITE_ADDR           0xE000A040
#define GPIO_BANK0_READ_ADDR            0xE000A060
#define GPIO_CFG_VAL                    0x00004000

/* SLCR FPGA clock registers */
#define SLCR_FPGA_CLK_CTL_BASE          0xF8000170
#define SLCR_FPGA_CLK_CTL_SIZE          0x10
#define SLCR_FPGA_CLK_DIV_MASK          0x3f00
#define IO_PLL_CLK_FREQ                 1000000000 /* 1000 Mhz */

#define BSI_INSERT(b,s,v)  ((v << (b##_V_##s)) & (b##_M_##s))

/* 
 * Add externs for missing symbols provided in common.h.
 * Cannot include common.h due to errors caused by including datCode.hh.
 */
extern int printf(const char *fmt, ...)
		__attribute__ ((format (__printf__, 1, 2)));

extern int sprintf(char *buf, const char *fmt, ...)
		__attribute__ ((format (__printf__, 2, 3)));
        
extern unsigned long get_timer (unsigned long base);

extern void * memset(void *,int,int);
  
/*
** ++
**
**
** --
*/

int axi_init(Bsi bsi, Ocm ocm, Axi axi)
  {
  BsiWrite32(bsi,BSI_BOOT_RESPONSE_OFFSET,BSI_BOOT_RESPONSE_AXI_INIT); 
  
  /* set the dma base address for all fifos */
  *(uint32_t *)(axi + AXI_MEM_CHAN_BASE_ADDR) = ocm + OCM_FIFO_READ_OFFSET;

  /* disable all interrupts */
  *(uint32_t *)(axi + AXI_INTR_ENAB_OFFSET) = 0;

  /* disable all fifo toggling */
  *(uint32_t *)(axi + AXI_FIFO_TOGGLE_OFFSET) = 0;

  /* configure the cache policy for AXI transactions */
  *(uint32_t *)(axi + AXI_CACHE_CFG_OFFSET) = 0xf;
  
  /* configure the bsi fifo, both sides */
  *(uint32_t *)(axi + AXI_FIFO_CFG_OFFSET + (AXI_BSI_FIFO_ID*8))      = AXI_BSI_FIFO_ID;    
  *(uint32_t *)(axi + AXI_FIFO_CFG_OFFSET + ((AXI_BSI_FIFO_ID*8)+4))  = AXI_BSI_FIFO_ID;  

  /* enable all fifos */
  *(uint32_t *)(axi + AXI_FIFO_ENABLE_OFFSET) = 0x1fffff;
  
  return 0;
  }

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
  
  macNet.u64 = mac;
  macBsi.u64 = 0;
  
  /* Swap the mac from network order to little endian */
  for (i=0; i<6; i++)
    macBsi.u8[i] = macNet.u8[5-i];

  /* Initialize the BSI */
  BsiWrite32(bsi,BSI_BOOT_RESPONSE_OFFSET,BSI_BOOT_RESPONSE_BSI_INIT);
  BsiSetup(bsi, phy, macBsi.u64);
  sprintf((char *)ethaddr,"%02x:%02x:%02x:%02x:%02x:%02x",macNet.u8[0],macNet.u8[1],macNet.u8[2],macNet.u8[3],macNet.u8[4],macNet.u8[5]);
  printf("Net:   bsi mac %s\n",ethaddr);

  return 0;
  }

/*
** ++
**
**
** --
*/

int ocm_init(Bsi bsi, Ocm ocm)
  {    
  BsiWrite32(bsi,BSI_BOOT_RESPONSE_OFFSET,BSI_BOOT_RESPONSE_OCM_INIT);
    
  /* Initialize OCM space */
  memset((void *)(ocm + OCM_FIFO_READ_OFFSET),0x80,OCM_FIFO_READ_SIZE);
  
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

    /* Read the GPIO bank 0 value */
  gpio = *(uint32_t *)GPIO_BANK0_READ_ADDR;

  /* Configure the GPIO pin as an output */
  *(uint32_t *)GPIO_DIRM0_ADDR = GPIO_CFG_VAL;

  /* Assert the pin */
  *(uint32_t *)GPIO_BANK0_WRITE_ADDR = gpio | GPIO_CFG_VAL;

  /* 
   * Enable GPIO pin output.
   * This tells the IPMI Controller that the
   * BSI parameters are ready for reading.
   */
  *(uint32_t *)GPIO_OEN0_ADDR  = GPIO_CFG_VAL;
  
  return 0;
}

/*
** ++
**
**
** --
*/

void rce_init(uint64_t mac, uint32_t phy, uint32_t dtm)
{
  Axi axi = 0;
  Bsi bsi = 0;
  Ocm ocm = 0;
  unsigned long time;

  bsi = LookupBsi();
  if (bsi)
    {
    BsiWrite32(bsi,BSI_BOOT_RESPONSE_OFFSET,BSI_BOOT_RESPONSE_NOT_BOOTED);
    ocm = LookupOcm();
    if (ocm)
      {
      if (!ocm_init(bsi,ocm))
        {      
        axi = LookupAxi();
        if (axi)
          {
          if (!axi_init(bsi,ocm,axi))
            bsi_init(bsi,mac,phy);
          }
        }
      }
    
    /* initialize the gpio and signal ipmi */
    gpio_init();

    /* cm init must be executed after the ipmi has been signaled */
    if (axi && ocm && dtm)    
      {
      BsiWrite32(bsi,BSI_BOOT_RESPONSE_OFFSET,BSI_BOOT_RESPONSE_CM_INIT);
	  time = get_timer(0);
      cm_net_init(axi,ocm);
	  time = get_timer(time);
	  printf("Net:   cm_net_init completed in %lu ms\n", time);
      }
    }
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
