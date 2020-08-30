/*
 * pid_delta.c
 *
 *  Created on: 2015��7��5��
 *      Author: xiaoyao
 */
#include "pid_delta.h"

////////////////////////////////////////////
//   PID������ʼ��
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
//����ʽPID�㷨����
//input:PID �Ŀ��Ƴ����ͺ����ֵ
//Return:�������templ
////////////////////////////////////////
double PidDeltaCal(PID_DELTA *pp, double ThisPoint) //����ʽPID�㷨����Ҫ���ƵĲ��ǿ������ľ���ֵ�����ǿ�������������
{
	double ThisError;
	double pError, dError, iError;
	double templ;

	ThisError = pp->setPoint - ThisPoint;
	pError = ThisError - pp->LastError;
	iError = ThisError;
	dError = ThisError - 2 * (pp->LastError) + pp->PreError;
	//��������
		templ = (pp->Proportion * pError + pp->Integral * iError
				+ pp->Derivative * dError);  //����
	if (templ > pp->max_result || templ < pp->min_result) {
		pp->PreError = pp->LastError;
		pp->LastError = ThisError;
		return (templ > pp->max_result) ? pp->max_result : pp->min_result;
	} else {	//���������������
		//�洢��������´μ���
		pp->PreError = pp->LastError;
		pp->LastError = ThisError;
		return templ;
	}
}

//��̬����pid
void adjust_pid(PID_DELTA *pp, double kp, double ki, double kd) {
	pp->Proportion = kp;
	pp->Integral = ki;
	pp->Derivative = kd;
}

void adjust_pid_setPoint(PID_DELTA *pp, double newSetPoint)
{
	pp->setPoint = newSetPoint;
}

//��̬�������Ʒ�Χ
void adjust_pid_limit(PID_DELTA *pp, double min, double max) {
	pp->min_result = min;
	pp->max_result = max;
}

