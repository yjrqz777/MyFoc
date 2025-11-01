#ifndef EM_SVPWM_H_
#define EM_SVPWM_H_

#include "data_type.h"


#define	EM_PWM_PERIOD_CYCLES	8500
#define EM_SQRT3FACTOR			56756
#define T_SQRT3					12470


void EmSvpwm(int16 u_alpha, int16 u_beta, uint16 *ta, uint16 *tb, uint16 *tc, uint16 *sector);

#endif

