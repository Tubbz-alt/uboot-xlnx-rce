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

#ifndef BSI_H
#define BSI_H

#include "Lookup.h"
#include "Bsi_Cfg.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Bit offsets for fields in the 32-bit BSI FIFO entry */
#define BSI_FIFO_ADDR_OFFSET         0
#define BSI_FIFO_EMPTY_OFFSET        16

/* Bit masks for fields in the 32-bit BSI FIFO entry */
#define BSI_FIFO_ADDR_MASK           0x0000FFFF
#define BSI_FIFO_EMPTY_MASK          0x00000001

#define BSI_FIFO_GET_ADDR(a)   \
    (uint32_t)((a >> BSI_FIFO_ADDR_OFFSET)    & BSI_FIFO_ADDR_MASK)

/* valid == !empty */
#define BSI_FIFO_GET_VALID(a)  \
    (uint32_t)((a >> BSI_FIFO_EMPTY_OFFSET)  & BSI_FIFO_EMPTY_MASK)

typedef uint32_t Offset;

static uint32_t BsiRead32(Bsi bsi, Offset offset);
static uint64_t BsiRead64(Bsi bsi, Offset offset);

static void BsiWrite32(Bsi bsi, Offset offset, uint32_t val);
static void BsiWrite64(Bsi bsi, Offset offset, uint64_t val);

static void BsiSetup(Bsi bsi, uint32_t phy, uint64_t mac);

static const char* BsiReadGroup(Bsi bsi, char value[BSI_GROUP_NAME_SIZE * sizeof(unsigned)]);

static void BsiWriteGroup(Bsi bsi, char value[BSI_GROUP_NAME_SIZE * sizeof(unsigned)]);

// Include the code to be inlined
#include "Bsi-inl.h"

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* BSI_H */
