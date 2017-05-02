/*--------------------------------------------------------------------------
// Description: hal_arm335xQEP.c
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
//------------------------------------------------------------------------*/

/* Use config_module.h instead of config.h so we can use RTAPI_INC_LIST_H */
#include "config_module.h"
#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "hal.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>

/* Include any other necessary header files */
#include "hal_arm335xPWM.h"


/* this probably should be an ARM335x define */
#if !defined(TARGET_PLATFORM_BEAGLEBONE)
#error "This driver is for the beaglebone platform only"
#endif

#if !defined(BUILD_SYS_USER_DSO)
#error "This driver is for usermode threads only"
#endif

/* Module information */
#define MODNAME "hal_arm335xPWM"
MODULE_AUTHOR("Josh Lane");
MODULE_DESCRIPTION("PWMSS HAL driver for ARM335x");
MODULE_LICENSE("GPL");

#define MAX_PWM 3
char *pwmgens[MAX_PWM] = {0,};
RTAPI_MP_ARRAY_STRING(pwmgens, MAX_PWM, "name of ePWM modules");
int frequency = 0;
RTAPI_MP_INT(frequency, "frequency of PWM modules (1Hz - 25kHz)");
float minDC = 0.0;
RTAPI_MP_FLOAT(minDC, "min allowable duty cycle (0.0% - 100.0%");
float maxDC = 100.0;
RTAPI_MP_FLOAT(maxDC, "max allowable duty cycle (0.0% - 100.0%");

/* Globals */
const devices_t devices[] = {
    {"ePWM0", 0x48300000},
    {"ePWM1", 0x48302000},
    {"ePWM2", 0x48304000},
    {NULL,-1}
};

#define TBCTL_CTRMODE_UP        0x0
#define TBCTL_CTRMODE_DOWN      0x1
#define TBCTL_CTRMODE_UPDOWN    0x2
#define TBCTL_CTRMODE_FREEZE    0x3

static const char *modname = MODNAME;
static int comp_id;

static ePWM_t *ePWM_array; /* pointer to array of ePWM_t structs in
                                    shmem, 1 per pwmgen */
static int howmany;
////////static hal_u32_t timebase;
/*---------------------
 Function prototypes
---------------------*/
static int export_ePWM(ePWM_t *addr);
static int setup_ePWM(ePWM_t *ePWM);
static void update(void *arg, long period);
void disable_ePWM(ePWM_t *ePWM);

/*---------------------
 INIT and EXIT CODE
---------------------*/

int rtapi_app_main(void)
{
    int n, retval, i;
    ePWM_t *ePWM;
    /* test for number of channels */
    for (howmany=0; howmany < MAX_PWM && pwmgens[howmany]; howmany++) ;

    if(howmany <= 0)  {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: ERROR: invalid number of pwmgens: %d\n", modname, howmany);
        return -1;
    }

    /* have good config info, connect to the HAL */
    comp_id = hal_init(modname);
    if(comp_id < 0 ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n",modname);
        return -1;
    }

    /* allocate shared memory for ePWM data */
    ePWM_array = hal_malloc(howmany * sizeof(ePWM_t));
    if (ePWM_array ==  0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: ERROR: hal_malloc() failed\n",modname);
        hal_exit(comp_id);
        return -1;
    }

/////////////    timebase = 0;

    /* setup and export all the variables for each ePWM device */
    for (n = 0; n < howmany; n++) {
        ePWM = &(ePWM_array[n]);
        /* make sure it's a valid device */
        for(i=0; devices[i].name; i++) {
            retval = strcmp(pwmgens[n], devices[i].name);
            if (retval == 0 ) {	//input pwmgen name matches ePWM0, ePWM1, or ePWM2
                ePWM->name = devices[i].name;
                int fd = open("/dev/mem", O_RDWR);

                ePWM->pwmss_reg = mmap(0, IOMEMLEN, PROT_READ | PROT_WRITE, MAP_SHARED,
                    fd, devices[i].addr);

                ePWM->ePWM_reg = (void*) ePWM->pwmss_reg + 0x200;		//change 0x200 to ePWM_OFFSET

                close(fd);

                if(ePWM->pwmss_reg == MAP_FAILED) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                        "%s: ERROR: mmap failed %s\n", modname, ePWM->name);
                    return -1;
                }
                rtapi_print("memmapped %s to %p and %p\n",ePWM->name,ePWM->pwmss_reg,ePWM->ePWM_reg);
                setup_ePWM(ePWM);
                break;
            }
        }

        if(retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: unknown device %s\n",
                modname, pwmgens[n]);
            return -1;
        }
    }

    /* export functions */
    retval = hal_export_funct("ePWM.update", update,
        ePWM_array, 1, 0, comp_id);
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: ERROR: function export failed\n",modname);
        hal_exit(comp_id);
        return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO,
        "%s: installed %d ePWM generators\n", modname, howmany);
    retval = hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{

	int n, retval, i;
    ePWM_t *ePWM;
	
	for(i = 0; i < howmany; i++){
		ePWM = &(ePWM_array[n]);
		disable_ePWM(ePWM);
	}
    hal_exit(comp_id);
}

/*---------------------
 Realtime functions
---------------------*/

static void update(void *arg, long period)
{

}


/*---------------------
 Local functions
---------------------*/
static int setup_ePWM(ePWM_t *ePWM)
{
	/* export pins to hal */
	export_pwmgen(ePWM);
	/* initiatlize members of ePWM struct */
    *(ePWM->dutyA) = 28.0;
	*(ePWM->dutyB) = 63.0;
	*(ePWM->enAout) = false;
	*(ePWM->enBout) = false;

	int param_error = 1;
	if (frequency < 0 )
	    param_error = 0;

    if (param_error == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
					"%s: ERROR: PWM frequency not valid\n",modname);
				hal_exit(comp_id);
		return -1;
	}

	/* compute neccessary TBPRD */
	float Cyclens =0.0f ;
	float Divisor =0;
	int i , j ;
	const float CLKDIV_div[] = {1.0 ,2.0 ,4.0 ,8.0 ,16.0 ,32.0 , 64.0 , 128.0};
	const float HSPCLKDIV_div[] ={1.0 ,2.0 ,4.0 ,6.0 ,8.0 ,10.0 , 12.0 , 14.0};
	int NearCLKDIV =0;
	int NearHSPCLKDIV =0;
	int NearTBPRD =4000;
#if 0
	Cyclens = 1000000000.0f / frequency ; /* 10^9 / HZ , comput time per cycle (ns) */


	Divisor =  (Cyclens / 655350.0f) ;	/* am335x provide (128*14) divider , and per TBPRD means 10 ns when divider /1 ,
						 * and max TBPRD is 65535 , so , the max cycle is 128*14* 65535 *10ns
						 */

	if(Divisor > (128 * 14)) {
		rtapi_print_msg(RTAPI_MSG_ERR,
					"%s: ERROR: PWM frequency out of bounds\n",modname);
				hal_exit(comp_id);
		return -1;
	}
	else {
		/* using Exhaustive Attack method */
		for(i = 0 ; i < 8 ; i ++) {
			for(j = 0 ; j < 8 ; j ++) {
				if((CLKDIV_div[i] * HSPCLKDIV_div[j]) < (CLKDIV_div[NearCLKDIV] * HSPCLKDIV_div[NearHSPCLKDIV]) &&
				  ((CLKDIV_div[i] * HSPCLKDIV_div[j]) > Divisor)) {
					NearCLKDIV = i ;
					NearHSPCLKDIV = j ;
				}
			}
		}

		NearTBPRD = (Cyclens / (10.0 *CLKDIV_div[NearCLKDIV] *HSPCLKDIV_div[NearHSPCLKDIV])) ;

		/* setting clock diver and freeze time base */
		ePWM->ePWM_reg->TBCTL = TBCTL_CTRMODE_FREEZE | (NearCLKDIV << 10) | (NearHSPCLKDIV << 7);

		ePWM->ePWM_reg->TBPRD = (unsigned short)NearTBPRD;

		/* reset time base counter */
		ePWM->ePWM_reg->TBCNT = 0;
	}
#endif

	/* setting clock diver and freeze time base */
	//ePWM->ePWM_reg->TBCTL = TBCTL_CTRMODE_FREEZE | (NearCLKDIV << 10) | (NearHSPCLKDIV << 7);
	ePWM->ePWM_reg->TBCTL = TBCTL_CTRMODE_FREEZE | (NearCLKDIV << 10) | (NearHSPCLKDIV << 7);

	/*  setting duty A and duty B */
	//ePWM->ePWM_reg->CMPB = (unsigned short)((float)NearTBPRD * (*(ePWM->dutyB)));
	ePWM->ePWM_reg->CMPB = (unsigned short)(1025);

	//ePWM->ePWM_reg->CMPA = (unsigned short)((float)NearTBPRD * (*(ePWM->dutyA)));
	ePWM->ePWM_reg->CMPA = (unsigned short)(3321);

	//ePWM->ePWM_reg->TBPRD = (unsigned short)NearTBPRD;
	ePWM->ePWM_reg->TBPRD = (unsigned short)(4000);

	/* reset time base counter */
	ePWM->ePWM_reg->TBCNT = 0;
		
	ePWM->ePWM_reg->AQCTLA = 0x2 | ( 0x3 << 4);
		
	ePWM->ePWM_reg->AQCTLB = 0x2 | ( 0x3 << 8);

    ePWM->ePWM_reg->TBCTL &= ~0x3;

    rtapi_print("%s: REVID = %#x\n",modname, ePWM->ePWM_reg->QREVID);
    return 0;
}

static int export_pwmgen(ePWM_t *ePWM)
{
    if (hal_pin_bit_newf(HAL_IO, &(ePWM->enAout), comp_id, "%s.A-out-enable", ePWM->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting A-out-enable\n");
        return -1;
    }
    if (hal_pin_bit_newf(HAL_IO, &(ePWM->enBout), comp_id, "%s.B-out-enable", ePWM->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting B-out-enable\n");
        return -1;
    }
    if (hal_pin_float_newf(HAL_IO, &(ePWM->dutyA), comp_id, "%s.Channel-A-duty-cycle", ePWM->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting Channel-A-duty-cycle\n");
        return -1;
    }
	if (hal_pin_float_newf(HAL_IO, &(ePWM->dutyB), comp_id, "%s.Channel-B-duty-cycle", ePWM->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting Channel-B-duty-cycle\n");
        return -1;
    }
	if (hal_pin_float_newf(HAL_IO, &(ePWM->scale), comp_id, "%s.Duty-cycle-scale", ePWM->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting Duty-cycle-scale\n");
        return -1;
    }


    return 0;
}

void disable_ePWM(ePWM_t *ePWM)
{
    ePWM->ePWM_reg->TBCTL |= 0x3;

	ePWM->ePWM_reg->AQCTLA = 0x1 | ( 0x3 << 4);
		
	ePWM->ePWM_reg->AQCTLB = 0x1 | ( 0x3 << 8);

	ePWM->ePWM_reg->TBCNT = 0;
}

