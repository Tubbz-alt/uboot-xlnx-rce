// -*-Mode: C;-*-
/*
@file Bsi_Cfg.h
@verbatim
                               Copyright 2018
                                    by
                       The Board of Trustees of the
                    Leland Stanford Junior University.
                           All rights reserved.
@endverbatim

@par Facility:


@par Abstract:
This file contains the masks, sizes and offsets for extracting fields
from words in the BSI (BootStrap Interface.)

@author
Sergio Maldonado <smaldona@slac.stanford.edu>

@par Date created:
2018/08/28

@par Credits:
SLAC
*/

#ifndef BSI_CFG_H
#define BSI_CFG_H

#define BSI_CLUSTER_CFG_VERSION_2       2
#define BSI_CLUSTER_CFG_VERSION_3       3

#ifndef USE_BSI_V2
#include "Bsi_Cfg_v3.h"
#else
#include "Bsi_Cfg_v2.h"
#endif

#endif /* BSI_CFG_H */
