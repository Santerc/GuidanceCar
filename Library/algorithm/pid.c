//
// Created by 59794 on 2024/12/23.
//
/* Includes -------------------------------------------------------------------*/
#include "pid.h"

/* Functions ------------------------------------------------------------------*/
/**
 * @brief 	xx
 * @param 	None
 * @retval	None
 * @note	None
 */

float limit(float num, float max, float min){
  if(num > max){
    return max;
  }
  if(num < min){
    return min;
  }
  return num;
}

void PID_ParamInit(PID_PIDParamTypeDef* pparam, float kp, float ki, float kd, float sum_max, float output_max) {
    pparam->kp = kp;
    pparam->ki = ki;
    pparam->kd = kd;
    pparam->sum_max = sum_max;
    pparam->output_max = output_max;
		pparam->err_max = 10000;
}
void PID_Set_Slope(PID_PIDParamTypeDef* pparam,float err_max){
		pparam->err_max = err_max;
}
void PID_Clear_Slope(PID_PIDParamTypeDef* pparam){
		pparam->err_max = 10000;
}
void PID_ArrayParamInit(PID_PIDParamTypeDef* pparam,float param[5]){
	PID_ParamInit(pparam,param[0],param[1],param[2],param[3],param[4]);
}

float PID_GetRef(PID_PIDTypeDef* pid) {
    return pid->ref;
}

void PID_SetRef(PID_PIDTypeDef* pid, float ref) {
    pid->ref = ref;
}

void PID_AddRef(PID_PIDTypeDef* pid, float inc) {
    pid->ref += inc;
}


float PID_GetFdb(PID_PIDTypeDef* pid) {
    return pid->fdb;
}

void PID_SetFdb(PID_PIDTypeDef* pid, float fdb) {
    pid->fdb = fdb;
}

float PID_GetOutput(PID_PIDTypeDef* pid) {
    return pid->output;
}

void PID_SetOutput(PID_PIDTypeDef* pid,float value){
	pid->output = value;
}

void PID_ClearData(PID_PIDTypeDef* pid) {
	pid->ref    = 0;
	pid->fdb    = 0;
	pid->err 	= 0;
	pid->err_last = 0;
	pid->sum    = 0;
	pid->output = 0;
}

void PID_Calc(PID_PIDTypeDef* pid, PID_PIDParamTypeDef* pparam) {
	float dError,Error;

	Error = limit(pid->ref - pid->fdb, pparam->err_max,-pparam->err_max);		//计算误差

	pid->sum = pid->sum + Error;		//累积误差
	pid->err_last = pid->err;			//更新结构体历史误差
	pid->err = Error;					//更新结构体误差
	dError = pid->err - pid->err_last;	//计算微分

	//积分限幅
	pid->sum = limit(pid->sum, pparam->sum_max,-pparam->sum_max);

	//计算输出
	pid->output = pparam->kp * Error + pparam->ki * pid->sum + pparam->kd * dError;

	//输出限幅
	pid->output = limit(pid->output, pparam->output_max, -pparam->output_max);
}

float PID_GetAntiWindupError( float error,PID_AntiWindupParamTypeDef * antiparam){
	if (error < antiparam->dec_start ){
		return error;
	}
	else if(error <antiparam->dec_end){
		return error * (error - antiparam->dec_start)/(antiparam->dec_end - antiparam->dec_start);
	}
	return 0;
}

void PID_CalcPIDAntiWindup(PID_PIDTypeDef* pid, PID_PIDParamTypeDef* param,PID_AntiWindupParamTypeDef * antiparam){
    float dError,Error;
    Error = pid->ref - pid->fdb;
    // 计算积分
    pid->sum = pid->sum + PID_GetAntiWindupError(Error,antiparam);
    // 积分限幅
    pid->sum = limit(pid->sum, param->sum_max, -param->sum_max);
    // 计算差分
    pid->err_last = pid->err;
    pid->err = Error;
    dError = pid->err - pid->err_last;
    // 计算结果
    pid->output = param->kp * Error + param->ki * pid->sum + param->kd * dError;
    // 输出限幅
	pid->output = limit(pid->output,param->output_max,-param->output_max);
}

//void PID_CalcDelta(PID_PIDTypeDef* pid, PID_PIDParamTypeDef* param,low_pass_t * filter){
//	float Error,dError,ddError;
//	Error = pid->ref - pid->fdb;
//	// 计算差分
//	pid->err[2] = pid->err[1];
//	pid->err[1] = pid->err[0];
//	pid->err[0] = Error;

//	dError =low_pass_filter(pid->err[0] - pid->err[1],filter) ;
//	ddError = pid->err[0] - 2.0f * pid->err[1] + pid->err[2];
//	// 计算积分
//	pid->sum = Error;
//	// 积分限幅
//    pid->sum = limit(pid->sum, param->sum_max, -param->sum_max);
//	// 计算结果
//	pid->output += param->kp * dError + param->ki * Error + param->kd * ddError;
//	// 输出限幅
//	pid->output = limit(pid->output,param->output_max,-param->output_max);
//}
//
//void PID_calcPartDiff(PID_PIDTypeDef* pid, PID_PIDParamTypeDef* param, low_pass_t* filter){
//	float dError,Error;
//    Error = pid->ref - pid->fdb;
//    // 计算积分
//    pid->sum = pid->sum + Error;
//    // 积分限幅
//    pid->sum = limit(pid->sum, param->sum_max, -param->sum_max);
//    // 计算差分
//    pid->err_last = pid->err;
//    pid->err = Error;
//	dError =low_pass_filter(pid->err - pid->err_last,filter) ;
//	// 计算结果
//    pid->output = param->kp * Error + param->ki * pid->sum + param->kd * dError;
//    // 输出限幅
//	pid->output = limit(pid->output,param->output_max,-param->output_max);
//}