/*
 * pid_delta.h
 *
 *  Created on: 2015��1��20��
 *      Author: acer
 */

#ifndef PID_DELTA_H_
#define PID_DELTA_H_

extern double p_para;
extern double i_para;
extern double d_para;

////////////////////////////////////////////////////
//    ����PID�����ṹ��
/////////////////////////////////////////////////
typedef struct PID_DELTA {                   //�ṹ�嶨��
	int setPoint;                       //�趨ֵ
	double Proportion;                     //����ϵ��
	double Integral;                       //����ϵ��
	double Derivative;                     //΢��ϵ��
	double LastError;                      //Error[-1] ǰһ�������
	double PreError;                       //Error[-2] ǰ�������

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
