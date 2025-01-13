#include "pid.h"

#define POSITION_PID 1 //位置式
#define DELTA_PID  2   //增量式
#define PID_MODE POSITION_PID

static void abs_limit(float *x,int32_t limit)
{
	if(*x > limit)
		*x = limit;
	if(*x < -limit)
		*x = -limit;
}

static void pid_init(pid_t *pid,float p,float i,float d,int32_t max_out,int32_t integral_limit)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxout = max_out;
	pid->integral_limit = integral_limit;
	pid->output_deadband = 5;
}

static void pid_reset(pid_t *pid,float p,float i,float d)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	
	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->out  = 0;
}

float pid_calc(pid_t *pid,float get,float set)
{
	pid->get = get;
	pid->set = set;
	pid->error[NOW_ERR] = set - get;
	
	#if (PID_MODE == POSITION_PID)
		pid->pout = pid->kp * pid->error[NOW_ERR];
		pid->iout += pid->ki * pid->error[NOW_ERR];
		pid->dout = pid->kd * (pid->error[NOW_ERR] - pid->error[LAST_ERR]);
		
		abs_limit(&(pid->iout),pid->integral_limit);
		pid->out = pid->pout + pid->iout + pid->dout;
		abs_limit(&(pid->out),pid->maxout);
	#elif (PID_MODE == DELTA_PID)
		pid->pout = pid->kp * (pid->error[NOW_ERR] - pid->error[LAST_ERR]);
		pid->iout = pid->ki * pid->error[NOW_ERR];
		pid->dout = pid->kd * (pid->error[NOW_ERR] * pid->error[LAST_ERR] + pid->error[LLAST_ERR]);
		
		pid->out += pid->pout + pid->iout + pid->dout;
		abs_limit(&(pid->out), pid->maxout);
	#endif
  
	pid->error[LLAST_ERR] = pid->error[LAST_ERR];
	pid->error[LAST_ERR]  = pid->error[NOW_ERR];
  
    return pid->out;
}	

float fuzzy_pid_calc(pid_t *pid, float get, float set)
{
	pid->get = get;
	pid->set = set;
	pid->error[NOW_ERR] = set - get;
	
	#if (PID_MODE == POSITION_PID)
		pid->pout = pid->kp * pid->error[NOW_ERR];
		pid->iout += pid->ki * pid->error[NOW_ERR];
		pid->dout = pid->kd * (pid->error[NOW_ERR] - pid->error[LAST_ERR]);
		
		abs_limit(&(pid->iout), pid->integral_limit);
		pid->out = pid->pout + pid->iout + pid->dout;
		abs_limit(&(pid->out), pid->maxout);

	#elif (PID_MODE == DELTA_PID)
		pid->pout = pid->kp * (pid->error[NOW_ERR] - pid->error[LAST_ERR]);
		pid->iout = pid->ki * pid->error[NOW_ERR];
		pid->dout = pid->kd * (pid->error[NOW_ERR] * pid->error[LAST_ERR] + pid->error[LLAST_ERR]);
		
		pid->out += pid->pout + pid->iout + pid->dout;
		abs_limit(&(pid->out), pid->maxout);

	#endif
		pid->error[LLAST_ERR] = pid->error[LAST_ERR];
		pid->error[LAST_ERR]  = pid->error[NOW_ERR];
  
    return pid->out;
}


void PID_Struct_Init(pid_t *pid,float p,float i,float d,int32_t max_out,int32_t integral_limit,INIT_STATUS init_status)
{
	if(init_status == INIT)//用于初始化
	{
		pid->f_pid_init = pid_init;
		pid->f_pid_reset = pid_reset;
		
		pid->f_pid_init(pid,p,i,d,max_out,integral_limit);
		pid->f_pid_reset(pid,p,i,d);
	}
	else									 //用于debug
	{
		pid->f_pid_init = pid_init;
		pid->f_pid_init(pid,p,i,d,max_out,integral_limit);
	}
}
