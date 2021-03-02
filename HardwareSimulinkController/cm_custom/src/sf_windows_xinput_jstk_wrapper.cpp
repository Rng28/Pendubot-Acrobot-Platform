
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



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <windows.h>
#include <xinput.h>
#pragma comment(lib, "XInput.lib")
//#pragma comment(lib, "xinput9_1_0.lib")

XINPUT_STATE state;
XINPUT_VIBRATION vibration;
DWORD dwResult;
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
extern "C" void sf_windows_xinput_jstk_Start_wrapper(const uint8_T *jstkID, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Start code goes here.
 */

ZeroMemory( &state, sizeof(XINPUT_STATE) );
ZeroMemory( &vibration, sizeof(XINPUT_VIBRATION) );
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
extern "C" void sf_windows_xinput_jstk_Outputs_wrapper(const real32_T *wLeftMotorSpeed,
			const real32_T *wRightMotorSpeed,
			uint16_T *wButtons,
			real32_T *bLeftTrigger,
			real32_T *bRightTrigger,
			real32_T *sThumbLX,
			real32_T *sThumbLY,
			real32_T *sThumbRX,
			real32_T *sThumbRY,
			boolean_T *jstkConnected,
			const uint8_T *jstkID, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
dwResult = XInputGetState(jstkID[0], &state );

if( dwResult == ERROR_SUCCESS )
{
    // Controller is connected
    jstkConnected[0] = 1;
    wButtons[0] = state.Gamepad.wButtons;
    bLeftTrigger[0] = (float) state.Gamepad.bLeftTrigger / 255.0;
    bRightTrigger[0] = (float) state.Gamepad.bRightTrigger / 255.0;
    sThumbLX[0] = ((float) state.Gamepad.sThumbLX + 32768.0) * 2.0 / 65535.0  - 1.0;
    sThumbLY[0] = ((float) state.Gamepad.sThumbLY + 32768.0) * 2.0 / 65535.0 - 1.0;
    sThumbRX[0] = ((float) state.Gamepad.sThumbRX + 32768.0) * 2.0 / 65535.0 - 1.0;
    sThumbRY[0] = ((float) state.Gamepad.sThumbRY + 32768.0) * 2.0 / 65535.0 - 1.0;
    
    vibration.wLeftMotorSpeed = (unsigned short) (wLeftMotorSpeed[0] * 65535); // use any value between 0-65535 here
    vibration.wRightMotorSpeed = (unsigned short) (wRightMotorSpeed[0] * 65535); // use any value between 0-65535 here
    XInputSetState( jstkID[0], &vibration );
    
}
else
{
    // Controller is not connected
    jstkConnected[0] = 0;
}
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


