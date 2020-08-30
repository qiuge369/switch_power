/*
 * pid_delta.h
 *
 *  Created on: 2015年1月20日
 *      Author: acer
 */

#ifndef PID_DELTA_H_
#define PID_DELTA_H_

extern double p_para;
extern double i_para;
extern double d_para;

////////////////////////////////////////////////////
//    定义PID参数结构体
/////////////////////////////////////////////////
typedef struct PID_DELTA {                   //结构体定义
	double setPoint;                       //设定值
	double Proportion;                     //比例系数
	double Integral;                       //积分系数
	double Derivative;                     //微分系数
	double LastError;                      //Error[-1] 前一节拍误差
	double PreError;                       //Error[-2] 前两拍误差

	double min_result;
	double max_result;
} PID_DELTA;

extern void PidDeltaInit(PID_DELTA *pp, double setPoint, double min, double max,
		double kp, double ki, double kd);
extern double PidDeltaCal(PID_DELTA *pp, double ThisPoint);
extern void adjust_pid(PID_DELTA *pp, double kp, double ki, double kd);
extern void adjust_pid_limit(PID_DELTA *pp, double min, double max);
extern void adjust_pid_setPoint(PID_DELTA *pp, double newSetPoint);

#endif /* PID_DELTA_H_ */
