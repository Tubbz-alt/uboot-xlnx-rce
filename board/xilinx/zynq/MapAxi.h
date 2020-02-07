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

#ifndef MAPAXI_H
#define MAPAXI_H

#define AXI0_MAP_SIZE                 0x00040000 /* 256K */
#define AXI1_MAP_SIZE                 0x00040000 /* 256K */

/* the following offsets apply only to AXI1 starting at 0x80000000 */
#define AXI_FWBITVER_OFFSET          0x0        /* firmware bitfile version */
#define AXI_FWMODVER_OFFSET          0x8        /* firmware module version */
#define AXI_CLKSEL0_OFFSET           0x10       /* reference clock 0 freq select */
#define AXI_CLKSEL1_OFFSET           0x14       /* reference clock 0 freq select */
#define AXI_DNA0_OFFSET              0x20       /* device DNA upper bits */
#define AXI_DNA1_OFFSET              0x24       /* device DNA lower bits */
#define AXI_EFUSE_OFFSET             0x30       /* EFuse register */
#define AXI_FWBUILD_OFFSET           0x1000     /* firmware build user and timestamp string */
#define AXI_FWBUILD_MEM_SIZE         0x100      /* size in bytes of firmware build string */

#endif /* MAPAXI_H */
