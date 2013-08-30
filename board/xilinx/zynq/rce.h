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
#define BSI_BOOT_RESPONSE_NOT_BOOTED    0x000000FF
#define BSI_BOOT_RESPONSE_OCM_INIT      0x000000FE
#define BSI_BOOT_RESPONSE_AXI_INIT      0x000000FD
#define BSI_BOOT_RESPONSE_BSI_INIT      0x000000FC
#define BSI_BOOT_RESPONSE_CM_INIT       0x000000FB
#define BSI_BOOT_RESPONSE_OS_HANDOFF    0x000000FA

#define NUM_FPGA_CLKS                   4

void rce_init(uint64_t mac, uint32_t phy, uint32_t dtm);
void rce_bsi_status(uint32_t status);
void rce_bsi_group(const char *buffer);
void rce_bsi_cluster(uint32_t slot, uint32_t cmb, uint32_t element);
void rce_fpga_clock(uint32_t clk, uint32_t freq);
#endif /* RCE_H */
