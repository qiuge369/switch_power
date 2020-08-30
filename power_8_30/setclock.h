/*
 * setclock.h
 *
 *  Created on: 2018Äê3ÔÂ10ÈÕ
 *      Author: pc
 */

#ifndef SETCLOCK_H_
#define SETCLOCK_H_
#include <msp430f6638.h>
#include <stdint.h>


#define PMM_STATUS_OK     0
#define PMM_STATUS_ERROR  1
#define _HAL_PMM_SVMLE  (SVMLE)
#define _HAL_PMM_SVSLE  (SVSLE)
#define _HAL_PMM_SVSFP  (SVSLFP)
#define _HAL_PMM_SVMFP  (SVMLFP)


void SetClock_MCLK12MHZ_SMCLK24MHZ_ACLK32_768K(void);




#endif /* SETCLOCK_H_ */
