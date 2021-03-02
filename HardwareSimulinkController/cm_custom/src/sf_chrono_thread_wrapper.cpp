
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
#include <chrono>
#include <thread>

auto loop_start = std::chrono::high_resolution_clock::now();
auto loop_end = std::chrono::high_resolution_clock::now();
static auto thread_start = loop_start;
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
 * Start function
 *
 */
extern "C" void sf_chrono_thread_Start_wrapper(const real_T *smplt, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Start code goes here.
 */

/*

DWORD dwError, dwPriClass, dwThreadPri;


if(!SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS))
{
dwError = GetLastError();
printf("Failed to set desired priority class.");
}
if(!SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL))
{
dwError = GetLastError();
printf("Failed to set desired thread priority.");
}
 *
 */
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
extern "C" void sf_chrono_thread_Outputs_wrapper(real_T *sys_clock_s,
			real_T *loop_time_left_s,
			const real_T *smplt, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
// payload above

loop_end = std::chrono::high_resolution_clock::now();
sys_clock_s[0] = std::chrono::duration<double>(loop_end - thread_start).count();

// sleep for
loop_time_left_s[0] = smplt[0] - std::chrono::duration<double>(loop_end - loop_start).count();

if (loop_time_left_s[0] > 0)
{
    unsigned loop_time_left_us = (unsigned) (loop_time_left_s[0]*1000000.0);
    std::this_thread::sleep_for(std::chrono::microseconds((unsigned) loop_time_left_us));
    loop_start = loop_end + std::chrono::microseconds((unsigned) loop_time_left_us);
    
}
else
{
    loop_start = loop_end;
}
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


