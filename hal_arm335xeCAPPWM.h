//--------------------------------------------------------------------------
// Description: hal_arm335xQEP.h
// HAL module to implement quadrature decoding using the ARM335x eQEP
// Module
//
// Author(s): Russell Gower
// License: GNU GPL Version 2.0 or (at your option) any later version.
//
// Major Changes:
// 2014-Nov    Russell Gower
//             Initial implementation, based on encoderc.c by John Kasunich
//--------------------------------------------------------------------------
// This file is part of LinuxCNC HAL
//
// Copyright (C) 2014  Russell Gower
//                     <russell AT thegowers DOT me DOT uk>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
// 02110-1301, USA.
//
// THE AUTHORS OF THIS PROGRAM ACCEPT ABSOLUTELY NO LIABILITY FOR
// ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
// TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
// harming persons must have provisions for completely removing power
// from all motors, etc, before persons enter any danger area.  All
// machinery must be designed to comply with local and national safety
// codes, and the authors of this software can not, and do not, take
// any responsibility for such compliance.
//
// This code was written as part of the LinuxCNC project.  For more
// information, go to www.linuxcnc.org.
//-------------------------------------------------------------------------

#ifndef _hal_arm335xCAPPWM_H_
#define _hal_arm335xCAPPWM_H_

#define IOMEMLEN (8*1024)


typedef struct {
    char *name;
    int addr;
} devices_t;

typedef struct {
    __u32   IDVER;
    __u32   SYSCONFIG;
    __u32   CLKCONFIG;
    __u32   CLKSTATUS;
} pwmss_reg_t;

typedef struct {
	__u32	TSCTR;				//0x0		Time-stamp counter register
	__u32	CTRPHS				//0x4		Counter phase offset value register
	__u32	CAP1				//0x8		Capture 1 register
	__u32	CAP2				//0xC		Capture 2 register
	__u32	CAP3				//0x10		Capture 3 register
	__u32	CAP4				//0x14		Capture 4 register
	__u32	RESERVED[4]			//0x18		Reserved memory space
	__u16	ECCTL1				//0x28		Capture control register 1
	__u16	ECCTL2				//0x2A		Capture control register 2
	__u16	ECEINT				//0x2C		Capture interrupt enable register
	__u16	ECFLG				//0x2E		Capture interrupt flag register
	__u16	ECCLR				//0x30		Capture interrupt clear register
	__u16	ECFRC				//0x32		Capture interrupt force register
	__u32	RESERVED_2[10]		//0x34		Reserved memory space
	__u32	REVID				//0x5C		Revision ID register


} eCAP_reg_t;


typedef struct {
    char 					*name;
    pwmss_reg_t volatile	*pwmss_reg;   /* Pointer to PWMSS hardware registers */
    ePWM_reg_t volatile		*ePWM_reg;  	/* Pointer to ePWM hardware registers */

	hal_float_t				*dcA;
	hal_float_t				*dcB;
	hal_bit_t				*enA_in;
	hal_bit_t				*enB_in;
	hal_float_t				*scale_in;
	hal_bit_t				*dirA_out;
	hal_bit_t				*dirB_out;
//	hal_u32_t				*dirApin;
//	hal_u32_t				*dirBpin;
	
	hal_float_t				scale;
	hal_float_t				oldScale;
	hal_float_t				scaled_dcA;
	hal_float_t				scaled_dcB;
	hal_float_t				old_scaled_dcA;
	hal_float_t				old_scaled_dcB;
	hal_bit_t				enA;
	hal_bit_t				enB;
	hal_bit_t				oldEnA;
	hal_bit_t				oldEnB;
	hal_float_t				min_dc;
	hal_float_t				max_dc;
	hal_bit_t				dirA;
	hal_bit_t				dirB;
	hal_bit_t				oldDirA;
	hal_bit_t				oldDirB;
	hal_u32_t				period;
	hal_float_t				resolution;
	hal_bit_t				outputType;
} eCAP_t;

// system tick 100MHz
#define SYSCLKOUT  1e8
#define SYSCLKOUT_INV 1e-8

#endif

