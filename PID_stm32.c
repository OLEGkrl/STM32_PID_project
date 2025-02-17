#include "PID_stm32.h"
/*
PID::PID() {
	Kp = 0;
	Ki = 0;
	Kd = 0;
	P = 0;
	D = 0;
	I = 0;
	err = 0;
}

PID::PID(float p, float i, float d) {
	Kp = p;
	Ki = i;
	Kd = d;
	P = 0;
	D = 0;
	I = 0;
	err = 0;
}
PID::PID(float p, float i, float d, float t) {
	Kp = p;
	Ki = i;
	Kd = d;
	dt = t;
	P = 0;
	D = 0;
	I = 0;
	err = 0;
}
*/

void PID_setVal(PID * PID_t,float v) {
	PID_t ->set = v;
	PID_t->deltaT =DELTA_T;
	//PID_update(PID_t);
}

void PID_insertSens(PID * PID_t,float s) {
	PID_t ->cur = s;
	PID_update(PID_t);

}
void PID_insertPWM(PID * PID_t,int ar) { // MAX val of PWM
	PID_t ->cur_arr = ar;
}
void PID_updateKFS(PID * PID_t,float p, float i, float d) {
	PID_t ->Kp = p;
	PID_t ->Ki = i;
	PID_t ->Kd = d;

}
void PID_setPID(PID * PID_t,float p, float i, float d) {
	PID_t ->Kp = p;
	PID_t ->Ki = i;
	PID_t ->Kd = d;
	PID_t ->P = 0;
	PID_t ->D = 0;
	PID_t ->I = 0;
	PID_t ->err = 0;
}
void PID_setPID_t(PID * PID_t,float p, float i, float d, float t) {
	PID_t ->Kp = p;
	PID_t ->Ki = i;
	PID_t ->Kd = d;
	PID_t ->Kp_ = p;
	PID_t ->Ki_ = i;
	PID_t ->Kd_ = d;
	PID_t -> dt =t;
	PID_t -> flagKi =0;
	PID_t ->P = 0;
	PID_t ->D = 0;
	PID_t ->I = 0;
	PID_t ->err = 0;

}
void PID_setPID_t_m(PID * PID_t,float p, float i, float d, float t,int m) {
	PID_t ->Kp = p;
	PID_t ->Ki = i;
	PID_t ->Kd = d;
	PID_t ->Kp_ = p;
	PID_t ->Ki_ = i;
	PID_t ->Kd_ = d;
	PID_t -> dt =t;
	PID_t -> flagKi =0;
	PID_t ->P = 0;
	PID_t ->D = 0;
	PID_t ->I = 0;
	PID_t ->err = 0;
	PID_t->mode =m;
}
void PID_setP(PID * PID_t,float n) {
	PID_t ->Kp = n;
}
void PID_setI(PID * PID_t,float n) {
	PID_t ->Ki = n;
	PID_t ->I = 0;
}
void PID_setD(PID * PID_t,float n) {
	PID_t ->Kd = n;
}

void PID_update(PID * PID_t) {
	PID_t -> P = PID_t ->set - PID_t ->cur;

	PID_t ->err = PID_t ->set - PID_t ->cur;
	PID_t ->D = (PID_t ->err - PID_t ->preVerr) / PID_t ->dt;
	PID_t ->preVerr = PID_t ->err;
	switch(PID_t->mode){
	case 0:
		PID_t ->I += (PID_t ->Ki) * PID_t ->err;
		PID_t ->out = (int)((PID_t ->Kp) * PID_t ->P +  PID_t ->I + PID_t ->Kd * PID_t ->D);
		break;
	case 1:
		if(fabsf(PID_t->err) < PID_t->deltaT){
			PID_t->flagKi =1;
		}
		if( (fabsf(PID_t->err) > PID_t->deltaT) && PID_t->flagKi ==1){ // || PID_t -> err < -1
				PID_t->flagKi =0;
		}
		if(PID_t->flagKi ==1 ){
			PID_t ->I += (PID_t ->Ki) * PID_t ->err;
			PID_t ->out = (int)((PID_t ->Kp) * PID_t ->P +  PID_t ->I + PID_t ->Kd * PID_t ->D);
		}else{
			PID_t ->out = (int)((PID_t ->Kp) * PID_t ->P  );
		}
		break;
	case 2:
		if(fabsf(PID_t->err) < PID_t->deltaT){
				PID_t->flagKi =1;
		}
		if( (fabsf(PID_t->err) > PID_t->deltaT) && PID_t->flagKi ==1){
				PID_t->flagKi =0;
		}
			if(PID_t->flagKi ==1 ){
				PID_t ->I += (PID_t ->Ki) * PID_t ->err;
		}
		if( PID_t->err > PID_t ->deltaT){
			PID_t->out = PID_t -> cur_arr+1;
		}else{
			PID_t ->out = (int)((PID_t ->Kp) * PID_t ->P +  PID_t ->I + PID_t ->Kd * PID_t ->D);
		}
		break;
	case 3:
			if(fabsf(PID_t->err) < PID_t->deltaT){
					PID_t->flagKi =1;
			}
			if( (fabsf(PID_t->err) > PID_t->deltaT) && PID_t->flagKi ==1){
					PID_t->flagKi =0;
			}
			if(PID_t->flagKi ==1 ){
					PID_t ->I += (PID_t ->Ki) * PID_t ->err;
			}else{
				PID_t ->I += (PID_t ->Ki /2) * PID_t ->err;
			}
//		PID_t ->I += (PID_t ->Ki / 2) * PID_t ->err;
		if( PID_t->err > PID_t ->deltaT +1){
				PID_t->out = PID_t -> cur_arr+1;
			}else{
				PID_t ->out = (int)((PID_t ->Kp) * PID_t ->P +  PID_t ->I + PID_t ->Kd * PID_t ->D);
			}
			break;
	}


	//PID_t ->I += (PID_t ->Ki) * PID_t ->err;

//
//	}else{

//	}
}

int PID_get(PID * PID_t) {
	if(PID_t-> out <0){
			return 0;
	}
	else if (PID_t ->out > PID_t-> cur_arr){
	//PID_t->I=0;
		return PID_t->cur_arr+1;
	}
	return PID_t ->out;

}

