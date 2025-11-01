#include "data_type.h"
#include "em_svpwm.h"

void EmSvpwm(int16 u_alpha, int16 u_beta, uint16 *ta, uint16 *tb, uint16 *tc, uint16 *sector)
{
	int32 u_alpha_p;
	int32 u_beta_p;
	int32 x;
	int32 y;
	int32 z;
	int32 ts;

	ts = EM_PWM_PERIOD_CYCLES;

	/* xyz cacalate */
	u_alpha_p = u_alpha * T_SQRT3;
	u_beta_p = u_beta * ts;

	x = u_beta_p;
	y = (u_alpha_p + u_beta_p) / 2;
	z = (-u_alpha_p + u_beta_p) / 2;
	
	/* tabc caculate */
	if(y < 0)
	{
	    if(z < 0)
	    {
	        (*sector) = 5;
	        (*tc) = ts/4 + (-y-z)/131072;
	        (*ta) = (*tc) + y/65536;
	        (*tb) = (*ta) + z/65536;
	    }
	    else
	    {
	        if(x < 0)
	        {
	            (*sector) = 4;
	            (*tc) = ts/4 + (z-x)/131072;
	            (*tb) = (*tc) + x/65536;
	            (*ta) = (*tb) - z/65536;
	        }
	        else
	        {
	            (*sector) = 3;
	            (*tb) = ts/4 + (x-y)/131072;
	            (*tc) = (*tb) - x/65536;
	            (*ta) = (*tc) + y/65536;
	        }
	    }
	}
	else
	{
	    if(z > 0)
	    {
	        (*sector) = 2;
	        (*tb) = ts/4 + (z+y)/131072;
	        (*ta) = (*tb) - z/65536;
	        (*tc) = (*ta) - y/65536;
	    }
	    else
	    {
	        if(x < 0)
	        {
	            (*sector) = 6;
	            (*ta) = ts/4 + (y-x)/131072;
	            (*tc) = (*ta) - y/65536;
	            (*tb) = (*tc) + x/65536;
	        }
			else
			{
	            (*sector) = 1;
	            (*ta) = ts/4 + (x-z)/131072;
	            (*tb) = (*ta) + z/65536;
	            (*tc) = (*tb) - x/65536;                        
			}
	    }
	}
	
}

