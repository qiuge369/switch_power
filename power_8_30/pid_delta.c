/*
 * pid_delta.c
 *
 *  Created on: 2015年7月5日
 *      Author: xiaoyao
 */
#include "pid_delta.h"

////////////////////////////////////////////
//   PID参数初始化
//////////////////////////////
void PidDeltaInit(PID_DELTA *pp, double setPoint, double min, double max,
		double kp, double ki, double kd) {
	pp->Proportion = kp;
	pp->Integral = ki;
	pp->Derivative = kd;
	pp->LastError = 0;
	pp->PreError = 0;
	pp->setPoint = setPoint;
	pp->min_result = min;
	pp->max_result = max;
}
;
///////////////////////////////////////////
//增量式PID算法程序
//input:PID 的控制常数和和误差值
//Return:误差增量templ
////////////////////////////////////////
double PidDeltaCal(PID_DELTA *pp, double ThisPoint) //增量式PID算法（需要控制的不是控制量的绝对值，而是控制量的增量）
{
	double ThisError;
	double pError, dError, iError;
	double templ;

	ThisError = pp->setPoint - ThisPoint;
	pError = ThisError - pp->LastError;
	iError = ThisError;
	dError = ThisError - 2 * (pp->LastError) + pp->PreError;
	//增量计算
		templ = (pp->Proportion * pError + pp->Integral * iError
				+ pp->Derivative * dError);  //增量
	if (templ > pp->max_result || templ < pp->min_result) {
		pp->PreError = pp->LastError;
		pp->LastError = ThisError;
		return (templ > pp->max_result) ? pp->max_result : pp->min_result;
	} else {	//符合条件更新误差
		//存储误差用于下次计算
		pp->PreError = pp->LastError;
		pp->LastError = ThisError;
		return templ;
	}
}

//动态调整pid
void adjust_pid(PID_DELTA *pp, double kp, double ki, double kd) {
	pp->Proportion = kp;
	pp->Integral = ki;
	pp->Derivative = kd;
}

void adjust_pid_setPoint(PID_DELTA *pp, double newSetPoint)
{
	pp->setPoint = newSetPoint;
}

//动态调整限制范围
void adjust_pid_limit(PID_DELTA *pp, double min, double max) {
	pp->min_result = min;
	pp->max_result = max;
}

