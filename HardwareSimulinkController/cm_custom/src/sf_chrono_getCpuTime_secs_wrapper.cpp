
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
#include<chrono>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void sf_chrono_getCpuTime_secs_Outputs_wrapper(real_T *slx_secs)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
using namespace std::chrono;
   
static high_resolution_clock::time_point currentStamp;
static high_resolution_clock::time_point startStamp;
static bool _init;

if (!_init)
{
    startStamp = std::chrono::high_resolution_clock::now();
    _init = true;
}

currentStamp = std::chrono::high_resolution_clock::now();
auto elapsed = currentStamp - startStamp;

long long elapsed_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
slx_secs[0] = 0.000001 * elapsed_microseconds;
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


