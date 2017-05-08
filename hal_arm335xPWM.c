/*--------------------------------------------------------------------------
// Description: hal_arm335xPWM.c
// 
//
// Author(s): Josh Lane
// License: GNU GPL Version 2.0 or (at your option) any later version.
//
// Major Changes:
//
//
//--------------------------------------------------------------------------
// This file is part of LinuxCNC HAL
//
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

/* CONSTANTS AND MACROS */
#define MAX_PWM 3
//TBCTL Counter Modes
#define TBCTL_CTRMODE_UP        0x0
#define TBCTL_CTRMODE_DOWN      0x1
#define TBCTL_CTRMODE_UPDOWN    0x2
#define TBCTL_CTRMODE_FREEZE    0x3

#define CHANNEL_A	0
#define CHANNEL_B	1

/* HAL Module Parameters */
char *pwmgens[MAX_PWM] = {0,};
RTAPI_MP_ARRAY_STRING(pwmgens, MAX_PWM, "name of ePWM modules");
int frequency = 0;
RTAPI_MP_INT(frequency, "frequency of PWM modules (1Hz - 25kHz)");
int minDC = 0;
RTAPI_MP_INT(minDC, "min allowable duty cycle (0% - 100%)");
int maxDC = 100;
RTAPI_MP_INT(maxDC, "max allowable duty cycle (0% - 100%)");
int type = 0;
RTAPI_MP_INT(type, "output type. 0: unidirectional, 1: bidirectional");

/* Global Variables */
const devices_t devices[] = {
    {"ePWM0", 0x48300000},
    {"ePWM1", 0x48302000},
    {"ePWM2", 0x48304000},
    {NULL, -1}
};
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
void disable_channel(ePWM_t *ePWM, int channel);
void enable_channel(ePWM_t *ePWM, int channel);
void set_channel_dc(ePWM_t *ePWM, int channel);

/*---------------------
 INIT and EXIT CODE
---------------------*/

int rtapi_app_main(void)
{
    int n, retval, i;
    ePWM_t *ePWM;
	
	//Check for valid module parameters
	int param_error = 1;
	if (frequency < 0 )
	    param_error = 0;
	if (minDC < 0 || minDC > maxDC || maxDC > 100)
		param_error = 0;

    if (param_error == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
					"%s: ERROR: PWM frequency not valid\n",modname);
				hal_exit(comp_id);
		return -1;
	}
	
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

/*	int n;
    ePWM_t *ePWM;
	
	for(n = 0; n < howmany; n++){
		ePWM = &(ePWM_array[n]);
		disable_ePWM(ePWM);
	}*/
    hal_exit(comp_id);
}

/*---------------------
 Realtime functions
---------------------*/

static void update(void *arg, long period)
{
	hal_s32_t     i;
	ePWM_t *ePWM;
	
    ePWM = arg;
	
	for(i = 0; i < howmany; i++)
	{
		ePWM->scale = (*(ePWM->scale_in));
		ePWM->enA = (*(ePWM->enA_in));
		ePWM->enB = (*(ePWM->enB_in));
		
		//check if scale parameter has changed
		if(ePWM->scale != ePWM->oldScale) //scale parameter has changed, validate new scale value
		{
			
			if (ePWM->scale < 1e-20) && (ePWM->scale > -1e-20)) // value too small, divide by zero is a bad thing
			{
				ePWM->scale = 1.0;
			}
		}
		
		//calculate scaled duty cycle values
		ePWM->scaled_dcA = (*(ePWM->dcA)) / ePWM->scale; //scale dcA
		ePWM->scaled_dcB = (*(ePWM->dcB)) / ePWM->scale; //scale dcB
		
		//clamp dc values if output is unidirectional
		if(ePWM->outputType = 0)
		{
			if(ePWM->scaled_dcA < 0.0)
				ePWM->scaled_dcA = 0.0;
				
			if(ePWM->scaled_dcB < 0.0)
				ePWM->scaled_dcB = 0.0;
		}
		
		//check if either duty cycle is below the resolution of the PWM
		if(ePWM->scaled_dcA < ePWM->resolution && ePWM->scaled_dcA > -ePWM->resolution) //dcA is smaller than the resolution of the PWM counters and will be treated as 0
		{
			ePWM->scaled_dcA = 0.0;
			ePWM->enA = 0;
		}
		
		if(ePWM->scaled_dcB < ePWM->resolution && ePWM->scaled_dcB > -ePWM->resolution) //dcB is smaller than the resolution of the PWM counters and will be treated as 0
		{
			ePWM->scaled_dcB = 0.0;
			ePWM->enB = 0;
		}
		
		//channel A output is to be disabled
		if(!ePWM->enA)
		{
			ePWM->scaled_dcA = 0.0;
			if(ePWM->oldEnA) //channel A output is currently enabled; else the channel is already disabled so do nothing
				disable_channel(ePWM, CHANNEL_A);
		}
		else //channel A output is to be enabled
		{
			//set PWM direction
			if(ePWM->scaled_dcA < 0.0)
			{
				ePWM->dirA = 1;
				ePWM->scaled_dcA = -ePWM->scaled_dcA;
			}
			else
			{
				ePWM->dirA = 0;
			}
			
			/* limit the duty cycle */
			if(ePWM->scaled_dcA > ePWM->max_dc) 
			{
				ePWM->scaled_dcA = ePWM->max_dc;
			}
			else if(ePWM->scaled_dcA < ePWM->min_dc)
			{
				ePWM->scaled_dcA = ePWM->min_dc;
			}
			//check if scaled_dcA has changed and update registers accordingly
			if(ePWM->scaled_dcA != ePWM->old_scaled_dcA)
				set_channel_dc(ePWM, CHANNEL_A);
			//check if the A direction output has changed and update accordingly
			if(ePWM->outputType = 1)
				if(ePWM->dirA != ePWM->oldDirA)
					setDir();	//replace with real way
			//check if channel A output is currently disabled and update registers accordingly
			if(!ePWM->oldEnA && ePWM->enA)
				enable_channel(ePWM, CHANNEL_A);
		}
		
		//channel B output is to be disabled
		if(!ePWM->enB)
		{
			ePWM->scaled_dcB = 0.0;
			if(ePWM->oldEnB) //channel A output is currently enabled; else the channel is already disabled so do nothing
				disable_channel(ePWM, CHANNEL_B);
		}
		else //channel B output is to be enabled
		{
			//set PWM direction
			if(ePWM->scaled_dcB < 0.0)
			{
				ePWM->dirB = 1;
				ePWM->scaled_dcB = -ePWM->scaled_dcB;
			}
			else
			{
				ePWM->dirB = 0;
			}
			
			/* limit the duty cycle */
			if(ePWM->scaled_dcB > ePWM->max_dc) 
			{
				ePWM->scaled_dcB = ePWM->max_dc;
			}
			else if(ePWM->scaled_dcB < ePWM->min_dc)
			{
				ePWM->scaled_dcB = ePWM->min_dc;
			}
			//check if scaled_dcB has changed and update registers accordingly
			if(ePWM->scaled_dcB != ePWM->old_scaled_dcB)
				set_channel_dc(ePWM, CHANNEL_B);
			//check if the B direction output has changed and update accordingly
			if(ePWM->dirB != ePWM->oldDirB)
				setDir();	//replace with real way
			//check if channel B output is currently disabled and update registers accordingly
			if(!ePWM->oldEnB && ePWM->enB)
				enable_channel(ePWM, CHANNEL_B);
		}
		
	#if 0	
		if(*(ePWM->dcA) < (float)minDC)
			*(ePWM->dcA) = (float)minDC;
		else if(*(ePWM->dcA) > (float)maxDC)
			*(ePWM->dutyA) = (float)maxDC;
		
		if(*(ePWM->dutyB) < (float)minDC)
			*(ePWM->dutyB) = (float)minDC;
		else if(*(ePWM->dutyB) > (float)maxDC)
			*(ePWM->dutyB) = (float)maxDC;
		
		/* update channel A */
		if(*(ePWM->enAout) && *(ePWM->dutyA))	//channel A is enabled and dc is non zero
		{
			if(!ePWM->oldEnA || !ePWM->oldDutyA)	//channel A was previously disabled
			{
				enable_channel(ePWM, CHANNEL_A);//enable PWM channel A
			}
			else	//channel A is already enabled so just set duty cycle
			{
				if(*(ePWM->dutyA) != ePWM->oldDutyA)
					set_channel_dc(ePWM, CHANNEL_A); //set duty cycle for channel A
			}
		}
		else	//channel A should be disabled
		{
			if(ePWM->oldEnA && ePWM->oldDutyA)	//channel A was previously enabled
			{
				disable_channel(ePWM, CHANNEL_A);//disable PWM channel A
				
			} //else do nothing, channel A is already disabled and should stay disabled
		}
		/* update channel B */
		if(*(ePWM->enBout) && *(ePWM->dutyB))	//channel B is enabled and dc is non zero
		{
			if(!ePWM->oldEnB || !ePWM->oldDutyB)	//channel B was previously disabled
			{
				enable_channel(ePWM, CHANNEL_B);//enable PWM channel B
			}
			else	//channel B is already enabled so just set duty cycle
			{
				if(*(ePWM->dutyB) != ePWM->oldDutyB)
					set_channel_dc(ePWM, CHANNEL_B); //set duty cycle for channel B
			}
		}
		else	//channel B should be disabled
		{
			if(ePWM->oldEnB && ePWM->oldDutyB)	//channel B was previously enabled
			{
				disable_channel(ePWM, CHANNEL_B);//disable PWM channel B
				
			} //else do nothing, channel B is already disabled and should stay disabled
		}
	#endif
		
		ePWM->oldEnA = ePWM->enA;
		ePWM->old_scaled_dcA = ePWM->scaled_dcA;
		ePWM->oldDirA = ePWM->dirA;
		ePWM->oldEnB = ePWM->enB;
		ePWM->old_scaled_dcB = ePWM->scaled_dcB;
		ePWM->oldDirB = ePWM->dirB;
		
		ePWM++;
	}
}


/*---------------------
 Local functions
---------------------*/
static int setup_ePWM(ePWM_t *ePWM)
{
	/* export pins to hal */
	export_ePWM(ePWM);
	
	/* initialize members of ePWM struct */
	ePWM->scale = 100.0f;
	ePWM->scaled_dcA = 0.0f;
	ePWM->scaled_dcB = 0.0f;
	ePWM->old_scaled_dcA = 0.0f;
	ePWM->old_scaled_dcB = 0.0f;
	ePWM->enA = false;
	ePWM->enB = false;
	ePWM->oldEnA = false;
	ePWM->oldEnB = false;
	ePWM->min_dc = (float)minDC / 100.0f;
	ePWM->max_dc = (float)maxDC / 100.0f;
	ePWM->dirA = 0;
	ePWM->dirB = 0;
	ePWM->oldDirA = 0;
	ePWM->oldDirB = 0;
	ePWM->period = 4000; //equates to 25kHz
	ePWM->resolution = 0.00025; //equates to 25kHz
	ePWM->outputType = type;
	

	/* compute TBPRD and Clock dividers for desired PWM frequency */
	float Cyclens =0.0f ;
	float Divisor =0.0f;
	int i , j ;
	const float CLKDIV_div[] = {1.0 ,2.0 ,4.0 ,8.0 ,16.0 ,32.0 , 64.0 , 128.0};
	const float HSPCLKDIV_div[] ={1.0 ,2.0 ,4.0 ,6.0 ,8.0 ,10.0 , 12.0 , 14.0};
	int NearCLKDIV =7;
	int NearHSPCLKDIV =7;
	//int NearTBPRD =0;

	Cyclens = 1000000000.0f / frequency ; /* 10^9 / HZ , comput time per cycle (ns) */

	Divisor =  (Cyclens / 655350.0f) ;	/* am335x provide (128*14) divider , and per TBPRD means 10 ns when divider /1 ,
						 * and max TBPRD is 65535 , so , the max cycle is 128*14* 65535 *10ns */

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

		//NearTBPRD = (Cyclens / (10.0 *CLKDIV_div[NearCLKDIV] *HSPCLKDIV_div[NearHSPCLKDIV])) ;
		ePWM->period = (Cyclens / (10.0 *CLKDIV_div[NearCLKDIV] *HSPCLKDIV_div[NearHSPCLKDIV])) ;
		ePWM->resolution = 1.0f/(float)ePWM->period;
	}

	/* setting clock driver and freeze time base */
	ePWM->ePWM_reg->TBCTL = TBCTL_CTRMODE_FREEZE | (NearCLKDIV << 10) | (NearHSPCLKDIV << 7);
	
	ePWM->ePWM_reg->TBPRD = (unsigned short)ePWM->period;
	
	/* reset time base counter */
	ePWM->ePWM_reg->TBCNT = 0;

	/*  setting duty A and duty B, PWM channel outputs initially disabled */
	ePWM->ePWM_reg->CMPB = (unsigned short)((float)ePWM->period * ePWM->dcB_scaled);

	ePWM->ePWM_reg->CMPA = (unsigned short)((float)ePWM->period * ePWM->dcA_scaled);
	
	/* make sure PWM A and B outputs are disabled */
	ePWM->ePWM_reg->AQCTLA = 0x1 | ( 0x0 << 4);
    		
    ePWM->ePWM_reg->AQCTLB = 0x1 | ( 0x0 << 8);

	/* start the period counter even though the outputs are disabled */
    ePWM->ePWM_reg->TBCTL &= ~0x3;
	
    return 0;
}

static int export_ePWM(ePWM_t *ePWM)
{
    if (hal_pin_bit_newf(HAL_IN, &(ePWM->enA_in), comp_id, "%s.en_A", ePWM->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting A-out-enable\n");
        return -1;
    }
    if (hal_pin_bit_newf(HAL_IN, &(ePWM->enB_in), comp_id, "%s.en_B", ePWM->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting B-out-enable\n");
        return -1;
    }
    if (hal_pin_float_newf(HAL_IN, &(ePWM->dcA), comp_id, "%s.dc_A", ePWM->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting Channel-A-duty-cycle\n");
        return -1;
    }
    if (hal_pin_float_newf(HAL_IN, &(ePWM->dcB), comp_id, "%s.dc_B", ePWM->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting Channel-B-duty-cycle\n");
        return -1;
    }
    if (hal_pin_float_newf(HAL_IN, &(ePWM->scale_in), comp_id, "%s.dc_scale", ePWM->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting Duty-cycle-scale\n");
        return -1;
    }
    if (hal_pin_u32_newf(HAL_IN, &(ePWM->dirApin), comp_id, "%s.dir_A_pin", ePWM->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting dirApin\n");
        return -1;
    }
    if (hal_pin_u32_newf(HAL_IN, &(ePWM->dirBpin), comp_id, "%s.dir_B_pin", ePWM->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Error exporting dirBpin\n");
        return -1;
    }

    return 0;
}

void disable_ePWM(ePWM_t *ePWM)
{
    ePWM->ePWM_reg->TBCTL |= 0x3;
    
    ePWM->ePWM_reg->AQCTLA = 0x1 | ( 0x0 << 4);
    		
    ePWM->ePWM_reg->AQCTLB = 0x1 | ( 0x0 << 8);

    ePWM->ePWM_reg->TBCNT = 0;
}

void disable_channel(ePWM_t *ePWM, int channel)
{
    //ePWM->ePWM_reg->TBCTL |= 0x3;
    if(channel == CHANNEL_A)
	{
        ePWM->ePWM_reg->AQCTLA = 0x1 | ( 0x0 << 4);
		//*(ePWM->dutyA) = *(ePWM->minDC);
	}
    else if(channel == CHANNEL_B)	
	{
        ePWM->ePWM_reg->AQCTLB = 0x1 | ( 0x0 << 8);
		//*(ePWM->dutyB) = *(ePWM->minDC);
	}

    //ePWM->ePWM_reg->TBCNT = 0;
}

void enable_channel(ePWM_t *ePWM, int channel)
{
    //ePWM->ePWM_reg->TBCTL |= 0x3;
	//set_channel_dc(ePWM, channel);
	
    if(channel == CHANNEL_A){
        ePWM->ePWM_reg->AQCTLA = 0x2 | ( 0x3 << 4);
    }else if(channel == CHANNEL_B){
        ePWM->ePWM_reg->AQCTLB = 0x2 | ( 0x3 << 8);
    }
    //ePWM->ePWM_reg->TBCNT = 0;
}

void set_channel_dc(ePWM_t *ePWM, int channel)
{
	//int NearTBPRD = 4000;

    if(channel == CHANNEL_A){
		//ePWM->dcA_scaled = *(ePWM->dutyA) / (100.0f * *(ePWM->scale));
		unsigned short CMPA_val = (unsigned short)((float)ePWM->period * ePWM->scaled_dcA);
		if(CMPA_val < 1)
		{
			disable_channel(ePWM, channel);
			ePWM->enA = 0;
		}
        ePWM->ePWM_reg->CMPA = CMPA_val;
    }
	else if(channel == CHANNEL_B){
		//ePWM->dcB_scaled = *(ePWM->dutyB) / (100.0f * *(ePWM->scale));
		unsigned short CMPB_val = (unsigned short)((float)ePWM->period * ePWM->scaled_dcB);
		if(CMPB_val == 0)
		{
			disable_channel(ePWM, channel);
			ePWM->enA = 0;
		}
        ePWM->ePWM_reg->CMPB = CMPB_val;
    }
}

