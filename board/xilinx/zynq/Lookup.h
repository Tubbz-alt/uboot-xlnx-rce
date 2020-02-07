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

#ifndef TOOL_MAP_LOOKUP_H
#define TOOL_MAP_LOOKUP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef uint32_t Axi;
typedef uint32_t Ocm;
typedef uint32_t Gic;

/* BSI uses word instead of byte offsets */
typedef volatile uint32_t *Bsi;

//Axi LookupAxi0(void);
//Axi LookupAxi1(void);
//Axi LookupAxiSocket(void);
Axi LookupAxiFw(void) { return (Axi)0x80000000; }
Bsi LookupBsi(void) { return (Bsi)0x84000000; }
//Ocm LookupOcm(void);
//Gic LookupGic(void);

//uint32_t LookupRegion(uint32_t region, uint32_t offset);

//int TeardownAxi0(Axi axi);
//int TeardownAxi1(Axi axi);
//int TeardownAxiSocket(Axi axi);
int TeardownAxiFw(Axi axi) { return 0;}
int TeardownBsi(Bsi bsi) { return 0;}
//int TeardownOcm(Ocm ocm);
//int TeardownGic(Gic gic);

//int TeardownRegion(uint32_t addr, uint32_t offset);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LOOKUP_H */
