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

#ifndef _hal_arm335xPWM_H_
#define _hal_arm335xPWM_H_

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
	__u16	TBCTL;		//0x0		Time base control register
	__u16	TBSTS;		//0x2		TIme base status register
	__u16	TBPHSHR;	//0x4		HRPWM phase register
	__u16	TBPHS;		//0x6		Time base phase register
	__u16	TBCNT;		//0x8		Time base counter register
	__u16	TBPRD;		//0xA		Time base period register
	__u16	RESERVED;	//0xC		Reserved memory space, DO NOT MODIFY
	__u16	CMPCTL;		//0xE		Counter compare control register
	__u16	CMPAHR;		//0x10		HRPWM counter compare A register
	__u16	CMPA;		//0x12		Counter compare A register
	__u16	CMPB;		//0x14		Counter compare B register
	__u16	AQCTLA;		//0x16		Action qualifier control register for output A
	__u16	AQCTLB;		//0x18		Action qualifier control register for output B
	__u16	AQSFRC;		//0x1A		Action qualifier software force register
	__u16	AQCSFRC;	//0x1C		Action qualifier continuous software force register
	__u16	DBCTL;		//0x1E		Dead band generator control register
	__u16	DBRED;		//0x20		Dead band generator rising edge delay count register
	__u16	DBFED;		//0x22		Dead band generator falling edge delay count register

} ePWM_reg_t;


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
} ePWM_t;

// system tick 100MHz
#define SYSCLKOUT  1e8
#define SYSCLKOUT_INV 1e-8

#endif

