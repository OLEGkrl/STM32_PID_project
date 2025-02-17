

#ifndef PID_STM32_H_
#define PID_STM32_H_
#include"math.h"

#define DELTA_T 9

typedef struct{
		float Kp;
		float Ki;
		float Kd;
		float dt;
		float set;
		float cur;
		float err, preVerr;
		float P, D, I;
		int out;
		float deltaT;
		int cur_arr;
		int flagKi;
		int mode;


} PID ;
void PID_setVal(PID *,float);
int PID_get(PID *);
void PID_insertSens(PID *,float);
void PID_setPID(PID *,float, float, float);
void PID_setPID_t(PID *,float, float, float,float);
void PID_setPID_t_m(PID *,float, float, float,float,int);
void PID_updateKFS(PID * PID_t,float p, float i, float d);
void PID_setP(PID*,float);
void PID_setI(PID *,float);
void PID_setD(PID *,float);
void PID_insertPWM(PID * PID_t,int );
void PID_update(PID *);

#endif /* PID_STM32_H_ */
