
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

#include "datatypes.h"


/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef MATLAB_MEX_FILE

#include <math.h>
#include "vesc_usb.h"
#include "vesc_usb.cpp"
#include "buffer.h"
#include "buffer.c"
#include "crc.h"
#include "crc.c"
#include "datatypes.h"

#endif
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void sf_vesc_usb_Start_wrapper(real_T *xD,
			void **pW,
			const uint8_T *vesc_dev, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef MATLAB_MEX_FILE

pW[0] = new vescUSB;
((vescUSB *) pW[0]) -> comm_init(vesc_dev);

#endif
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void sf_vesc_usb_Outputs_wrapper(mc_values *slx_values,
			mc_configuration *slx_mcconf,
			boolean_T *slx_mcconf_read,
			const real_T *xD,
			void **pW,
			const uint8_T *vesc_dev, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef MATLAB_MEX_FILE
// xD[0]: mode state
// default: idle
// 1: on chip position pid
// 2: read mcconfig
// 3: write mcconfig
// 4: on chip current control

((vescUSB *)pW[0])->receive_packet(); // parse 

switch ((unsigned int)xD[0]) {
default: // idle, PID, current
	slx_values[0] = ((vescUSB *)pW[0])->values;
	break;
case 2: // read mcconfig
	slx_mcconf_read[0] = ((vescUSB *)pW[0])->mcconf_read; // check to see if new mcconfig has been read; output to slx
	if (slx_mcconf_read[0]) {
		slx_mcconf[0] = ((vescUSB *)pW[0])->mcconf;
	}
	break;
case 3: // write mcconfig
	((vescUSB *)pW[0])->bldc_interface_set_mcconf(&(((vescUSB *)pW[0])->mcconf)); // wrote
	slx_mcconf[0] = ((vescUSB *)pW[0])->mcconf; // read new mcconfig to slx
	((vescUSB *)pW[0])->mcconf_read = false; // reset mcconfig read flag
	break;
}

//slx_state[0] = xD[0];

#endif
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Updates function
 *
 */
void sf_vesc_usb_Update_wrapper(const uint32_T *slx_mode,
			const real_T *slx_pos_cmd,
			const real_T *slx_current_cmd,
			const real_T *slx_pos_pid_gains,
			const real_T *slx_curr_lims,
			mc_values *slx_values,
			mc_configuration *slx_mcconf,
			boolean_T *slx_mcconf_read,
			real_T *xD,
			void **pW,
			const uint8_T *vesc_dev, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef MATLAB_MEX_FILE
// xD[0]: mode state
// default: idle
// 1: on chip position pid
// 2: read mcconfig
// 3: write mcconfig
// 4: on chip current control

switch ((unsigned int)xD[0]) {
default: // idle
	((vescUSB *)pW[0])->bldc_interface_set_current(0); // zero current
	((vescUSB *)pW[0])->bldc_interface_get_values(); // request values
	switch (slx_mode[0]) { // main state machine switch
	case 1: // pid
		xD[0] = 1;
		break;
	case 2: // read mcconfig
		xD[0] = 2;
		break;
	case 3: // write mcconfig
		if (((vescUSB *)pW[0])->mcconf_read) { // requires old values to be read
			// assign new user values
			((vescUSB *)pW[0])->mcconf.p_pid_kp = slx_pos_pid_gains[0];
			((vescUSB *)pW[0])->mcconf.p_pid_ki = slx_pos_pid_gains[1];
			((vescUSB *)pW[0])->mcconf.p_pid_kd = slx_pos_pid_gains[2];
			((vescUSB *)pW[0])->mcconf.p_pid_kd_filter = slx_pos_pid_gains[3];
			((vescUSB *)pW[0])->mcconf.p_pid_ang_div = slx_pos_pid_gains[4];
			((vescUSB *)pW[0])->mcconf.l_current_max = slx_curr_lims[0];
			((vescUSB *)pW[0])->mcconf.l_current_min = slx_curr_lims[1];
			((vescUSB *)pW[0])->mcconf.l_in_current_max = slx_curr_lims[2];
			((vescUSB *)pW[0])->mcconf.l_in_current_min = slx_curr_lims[3];
			((vescUSB *)pW[0])->mcconf.l_abs_current_max = slx_curr_lims[4];
			xD[0] = 3;
		}
		break;
	case 4: // current mode
		xD[0] = 4;
	default:
		break;
	}
	break;
case 1: // on chip pid
	if (slx_mode[0] != 1) {
		xD[0] = 0;
		((vescUSB *)pW[0])->bldc_interface_set_current(0); // zero current on exit
	}
	else {
		((vescUSB *)pW[0])->bldc_interface_set_pos(slx_pos_cmd[0]); // write new des pos
		((vescUSB *)pW[0])->bldc_interface_get_values(); // request values
	}
	break;

case 2: // read mcconfig
	if (slx_mode[0] != 2) {
		xD[0] = 0;
	}
	else {
		((vescUSB *)pW[0])->bldc_interface_get_mcconf();
		//((vescUSB *) pW[0]) -> mcconf_read = false;
	}
	break;
case 3: // write mcconfig
	// write sequence; always once only
	xD[0] = 0; // return to idle
	break;

case 4: // current mode
	if (slx_mode[0] != 4) {
		xD[0] = 0;
		((vescUSB *)pW[0])->bldc_interface_set_current(0); // zero current on exit
	}
	else {
		((vescUSB *)pW[0])->bldc_interface_set_current(slx_current_cmd[0]); // write new current cmd
		((vescUSB *)pW[0])->bldc_interface_get_values(); // request values
	}	
	break;

}


#endif
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Terminate function
 *
 */
void sf_vesc_usb_Terminate_wrapper(real_T *xD,
			void **pW,
			const uint8_T *vesc_dev, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef MATLAB_MEX_FILE

for (int j = 0; j < 10; j++){
    // repeat j times
    ((vescUSB *) pW[0]) -> comm_close();
}

delete pW[0];

#endif
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}

