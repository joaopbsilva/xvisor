/**
 * Copyright (c) 2012 Sukanto Ghosh.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * @file arm_plat.h
 * @author Sukanto Ghosh (sukantoghosh@gmail.com)
 * @brief ARM platform configuration
 */
#ifndef _ARM_PLAT_H__
#define _ARM_PLAT_H__

#define CT_CA9X4_MPIC			(0xF8F00000)

#define A9_MPCORE_SCU			(CT_CA9X4_MPIC + 0x0000)
#define A9_MPCORE_GIC_CPU		(CT_CA9X4_MPIC + 0x0100)
#define A9_MPCORE_GIT			(CT_CA9X4_MPIC + 0x0200)
#define A9_MPCORE_TWD			(CT_CA9X4_MPIC + 0x0600)
#define A9_MPCORE_GIC_DIST		(CT_CA9X4_MPIC + 0x1000)

#define IRQ_CA9X4_GIC_START		27
#define NR_IRQS_CA9X4			96
#define NR_GIC_CA9X4			1

#define IMX_PA_AIPS1		0x02000000
#define IMX_PA_AIPS2		0x02100000
#define IMX_PA_EIM		0x08000000


#define IMX_NOR			(IMX_PA_EIM)
#define ZYNQ_UART0		(0xE0000000)
#define IMX_IOMUX		(IMX_PA_AIPS1 + 0x000e0000)
#define ZYNQ_PRIVATE_TIMER		(0xF8F00600)//ARM LOCAL TIMER(Private timer)...Já está em cima
#define V2M_SYSREGS		(IMX_PA_AIPS1 + 0x000F0000)
#define ZYNQ_UART1		(0xE0001000)

#define IRQ_PRIVATE_TIMER		(29)

/*
 * Defines required by common code
 */
#define ARM_PLAT_SPIN_ADDR	(V2M_SYS_FLAGS)//Não

#endif
