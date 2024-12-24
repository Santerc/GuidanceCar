#ifndef __PID_LIB_H
#define __PID_LIB_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes -------------------------------------------------------------------*/
#include "stddef.h"
#include "string.h"
/* define ----------------------------------------------------------------------*/

/* Typedef --------------------------------------------------------------------*/
typedef struct {
    float ref;			//期望值
    float fdb;			//反馈值
    float err;			//误差值
		float err_last;		//历史误差
    float sum;			//积分值
    float output;		//输出值
}
PID_PIDTypeDef;			//PID数据结构体

typedef struct {
    float kp;			//比例参数
    float ki;			//积分参数
    float kd;			//微分参数
		float err_max;    //误差上限
    float sum_max;		//积分上限
    float output_max;	//输出上限
}
PID_PIDParamTypeDef;	//PID参数结构体

typedef struct
{
	float dec_start;
	float dec_end;
}PID_AntiWindupParamTypeDef;
/* Variables ------------------------------------------------------------------*/

/* Functions ------------------------------------------------------------------*/
void PID_ParamInit(PID_PIDParamTypeDef* pparam, float kp, float ki, float kd, float sum_max, float output_max);
void PID_Set_Slope(PID_PIDParamTypeDef* pparam,float err_max);
void PID_Clear_Slope(PID_PIDParamTypeDef* pparam);
void PID_ArrayParamInit(	PID_PIDParamTypeDef* pparam,float param[5]);
void PID_ClearData(	PID_PIDTypeDef* pid);
void PID_Calc(	PID_PIDTypeDef* pid, PID_PIDParamTypeDef* pparam);

float PID_GetRef(		PID_PIDTypeDef* pid) ;
float PID_GetOutput(	PID_PIDTypeDef* pid);
float PID_GetFdb(		PID_PIDTypeDef* pid);
void PID_SetFdb(	PID_PIDTypeDef* pid, float fdb);
void PID_SetRef(	PID_PIDTypeDef* pid, float ref) ;
void PID_SetOutput(	PID_PIDTypeDef* pid, float value);
void PID_AddRef(	PID_PIDTypeDef* pid, float inc) ;
#ifdef __cplusplus
}
#endif
#endif
/************************ COPYRIGHT BIT DreamChaser *****END OF FILE****/
