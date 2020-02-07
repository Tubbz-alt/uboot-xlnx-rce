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

#ifndef BSI_INL_H



//#include "map/Lookup.h"
#include "Bsi_Cfg.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*
** ++
**
**
** --
*/

static inline uint32_t BsiRead32(Bsi bsi, Offset offset)
  {
  return bsi[offset];
  }
  
/*
** ++
**
**
** --
*/
 
static inline uint64_t BsiRead64(Bsi bsi, Offset offset)
  {
  union
    {
    uint64_t u64;
    uint32_t u32[2];
    } val = { 0 };
 
  val.u32[0] = bsi[offset];
  val.u32[1] = bsi[offset+1];
  return val.u64;
  }

/*
** ++
**
**
** --
*/
 

static inline void BsiWrite32(Bsi bsi, Offset offset, uint32_t val)
  {
  bsi[offset] = val;
  }

/*
** ++
**
**
** --
*/ 

static inline void BsiWrite64(Bsi bsi, Offset offset, uint64_t val)
  {
  union
    {
    uint64_t u64;
    uint32_t u32[2];
    } value = { val };
 
  bsi[offset]   = value.u32[0];
  bsi[offset+1] = value.u32[1];
  }

/*
** ++
**
**
** --
*/

static inline void BsiSetup(Bsi bsi, uint32_t phy, uint64_t mac)
  {
  BsiWrite32(bsi,BSI_NETWORK_PHY_TYPE_OFFSET, phy);
  BsiWrite64(bsi,BSI_MAC_ADDR_OFFSET, mac);
  }

/*
** ++
**
**
** --
*/

static inline const char* BsiReadGroup(Bsi bsi, char value[BSI_GROUP_NAME_SIZE * sizeof(unsigned)])
  {
  unsigned* val = (unsigned*)value;
  unsigned  idx = 0;
  do {
    *val++ = BsiRead32(bsi,BSI_CLUSTER_GROUP_NAME_OFFSET+idx);
  }
  while (++idx < BSI_GROUP_NAME_SIZE - 1);
  return value;
  }

/*
** ++
**
**
** --
*/

static inline void BsiWriteGroup(Bsi bsi, char value[BSI_GROUP_NAME_SIZE * sizeof(unsigned)])
  {
  unsigned* val = (unsigned*)value;
  unsigned  idx = 0;
  do {
    BsiWrite32(bsi,BSI_CLUSTER_GROUP_NAME_OFFSET+idx,*val++);
  }
  while (++idx < BSI_GROUP_NAME_SIZE - 1);
  }

#ifdef __cplusplus
}
#endif /* __cplusplus */
  
#endif
