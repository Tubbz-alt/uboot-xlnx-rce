#
# (C) Copyright 2000-2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk

# include the DAT build configuration
datroot  := $(DAT_ROOT)
datbuild := $(datroot)/build
dattgt   := arm-eabi-rceCA9-opt
datsvn   := '"$(shell LC_ALL=C svnversion -n $(datroot))"'
gitrev   :=  "$(shell git rev-parse --short HEAD)"
gittag   := '"$(shell git describe --tags $(gitrev))"'

ZYNQCFLAGS  := -I$(datroot) -DDAT_BOARD_SUBDIR=rce -DDAT_GENERATION_SUBDIR=gen3 -DDAT_SVN_REV=$(datsvn) -DUBOOT_GIT_TAG=$(gittag)
CFLAGS      += $(ZYNQCFLAGS)
CPPFLAGS    += $(ZYNQCFLAGS) # needed for depend

LIB	  = $(obj)lib$(BOARD).o

COBJS-y	 := rce.o board.o

COBJS	 := $(sort $(COBJS-y))

ZYNQOBJS := $(datbuild)/tool/lib/$(dattgt)/libmap.a

#this library is only required for targets that do not configure the bsi using the environment
ifndef CONFIG_BSI_ENV
ZYNQOBJS += $(datbuild)/cm/lib/$(dattgt)/libboot.a $(datbuild)/tool/lib/$(dattgt)/libport.a
endif

SRCS	 := $(COBJS:.o=.c)
OBJS	 := $(addprefix $(obj),$(COBJS))
OBJS     += $(ZYNQOBJS)

$(LIB):	$(obj).depend $(OBJS)
	$(call cmd_link_o_target, $(OBJS))

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
