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

#ifndef RCE_H
#define RCE_H

/* BSI boot status */
#define BSI_BOOT_RESPONSE_NOT_BOOTED    255
#define BSI_BOOT_RESPONSE_OCM_INIT      250
#define BSI_BOOT_RESPONSE_AXI_INIT      246
#define BSI_BOOT_RESPONSE_CM_INIT       242
#define BSI_BOOT_RESPONSE_RCE_READY     238
#define BSI_BOOT_RESPONSE_OS_HANDOFF    234

#define NUM_FPGA_CLKS                   4

int  rce_init(uint64_t mac, uint32_t phy);
int  rce_is_dtm(void);
int  rce_bsi_ipinfo(unsigned *ip, unsigned *gw, unsigned *nm);
void rce_bsi_status(uint32_t status);
int  rce_bsi_slot(void);
void rce_bsi_group(const char *buffer);
void rce_bsi_cluster(uint32_t slot, uint32_t cmb, uint32_t element);
void rce_fpga_clock(uint32_t clk, uint32_t freq);
void rce_uboot_version(const char *buffer, uint32_t len);
void rce_dat_version(const char *buffer, uint32_t len);
uint64_t rce_mac(void);

#endif /* RCE_H */
