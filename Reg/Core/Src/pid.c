/*
 * pid.c
 *
 *  Created on: Feb 3, 2022
 *      Author: trcho
 */


#include "main.h"
#include <math.h>
#include <structures.h>
#include <funksjoner.h>
#include <string.h>
#include <stdlib.h>
#include <testfunksjoner.h>
//#include <variabler.h>
#include <variabler_ext.h>





/* Variabler -------------------------------------------------- */






/* Funksjoner -------------------------------------------------- */
void stamp_regulator(void){
	spid.e = spid.yr - stamp_m;													// Avvik
	spid.fbf = param.stamp_pid.a * spid.fbfs + param.stamp_pid.b*stamp_m;		// filtrert måledata

	spid.up = param.stamp_pid.kp*spid.e;												// Proporsjonalbidrag
	spid.ui = spid.uis + param.stamp_pid.ki * param.stamp_pid.ts * (spid.e+spid.es)/2; 	// integratorbidrag
	spid.ud = -param.stamp_pid.kd * (spid.fbf-spid.fbfs) / param.stamp_pid.ts;			// derivatorbidrag

	if (spid.ui > param.stamp_pid.ui_maks){												// Integratorbegrensning
		spid.ui = param.stamp_pid.ui_maks;	}
	else if (spid.ui < param.stamp_pid.ui_min){
		spid.ui = param.stamp_pid.ui_min;	}

	spid.ut = spid.up+spid.ui+spid.ud;													// Totalbidrag
	if (spid.ut > param.stamp_pid.ut_maks){												// Totalbidragsbegrensning
		spid.ut = param.stamp_pid.ut_maks;}
	else if (spid.ut < param.stamp_pid.ut_min){
		spid.ut = param.stamp_pid.ut_min;	}

	spid.es = spid.e;																	// Oppdaterer forrige avvik
	spid.fbfs = spid.fbf;																// Oppdaterer forrige filtrerte måling
	spid.uis = spid.ui;																	// Oppdaterer forrige integratorbidrag

	float prosent_paadrag = kg_til_paadrag(spid.ut);									// Konverterer kg thrust til prosent pådrag
	stampbidrag.vhf = prosent_paadrag;
	stampbidrag.vvf = prosent_paadrag;
	prosent_paadrag = kg_til_paadrag(-spid.ut);											// Konverterer for invers rettet thrustere
	stampbidrag.vhb = prosent_paadrag;
	stampbidrag.vvb = prosent_paadrag;
}


void rull_regulator(void){
	rpid.e = rpid.yr - rull_m;
	rpid.fbf = param.rull_pid.a * rpid.fbfs + param.rull_pid.b*rull_m;

	rpid.up = param.rull_pid.kp*rpid.e;
	rpid.ui = rpid.uis + param.rull_pid.ki * param.rull_pid.ts * (rpid.e+rpid.es) /2;
	rpid.ud = -param.rull_pid.kd * (rpid.fbf-rpid.fbfs) / param.rull_pid.ts ;

	if (rpid.ui > param.rull_pid.ui_maks){
		rpid.ui = param.rull_pid.ui_maks;	}
	else if (rpid.ui < param.rull_pid.ui_min){
		rpid.ui = param.rull_pid.ui_min;	}

	rpid.ut = rpid.up+rpid.ui+rpid.ud;
	if (rpid.ut > param.rull_pid.ut_maks){
		rpid.ut = param.rull_pid.ut_maks;}
	else if (rpid.ut < param.rull_pid.ut_min){
		rpid.ut = param.rull_pid.ut_min;	}

	rpid.es = rpid.e;
	rpid.fbfs = rpid.fbf;
	rpid.uis = rpid.ui;

	float prosent_paadrag = kg_til_paadrag(rpid.ut);
	rullbidrag.vhf = prosent_paadrag;
	rullbidrag.vhb = prosent_paadrag;
	prosent_paadrag = kg_til_paadrag(-rpid.ut);
	rullbidrag.vvb = prosent_paadrag;
	rullbidrag.vvf = prosent_paadrag;


}

void hiv_regulator(void){

	hpid.e = hpid.yr - hiv_m;
	hpid.fbf = param.hiv_pid.a * hpid.fbfs + param.hiv_pid.b*hiv_m;

	hpid.up = param.hiv_pid.kp*hpid.e;
	hpid.ui = hpid.uis + param.hiv_pid.ki * param.hiv_pid.ts * (hpid.e+hpid.es) /2;
	hpid.ud = -param.hiv_pid.kd * (hpid.fbf-hpid.fbfs) / param.hiv_pid.ts;

	if (hpid.ui > param.hiv_pid.ui_maks){
		hpid.ui = param.hiv_pid.ui_maks;	}
	else if (hpid.ui < param.hiv_pid.ui_min){
		hpid.ui = param.hiv_pid.ui_min;	}

	hpid.ut = hpid.up+hpid.ui+hpid.ud;
	if (hpid.ut > param.hiv_pid.ut_maks){
		hpid.ut = param.hiv_pid.ut_maks;}
	else if (hpid.ut < param.hiv_pid.ut_min){
		hpid.ut = param.hiv_pid.ut_min;	}

	hpid.es = hpid.e;
	hpid.fbfs = hpid.fbf;
	hpid.uis = hpid.ui;

	float prosent_paadrag = kg_til_paadrag(hpid.ut);
	hivbidrag.vhf = prosent_paadrag;
	hivbidrag.vhb = prosent_paadrag;
	hivbidrag.vvb = prosent_paadrag;
	hivbidrag.vvf = prosent_paadrag;
}

void gir_regulator(void){

	gpid.e = gpid.yr - gir_mf;
	gpid.fbf = param.gir_pid.a * gpid.fbfs + param.gir_pid.b*gir_m;

	gpid.up = param.gir_pid.kp*gpid.e;
	gpid.ui = gpid.uis + param.gir_pid.ki * param.gir_pid.ts * (gpid.e+gpid.es) /2;
	gpid.ud = -param.gir_pid.kd * (gpid.fbf-gpid.fbfs) / param.gir_pid.ts;

	if (gpid.ui > param.gir_pid.ui_maks){
		gpid.ui = param.gir_pid.ui_maks;	}
	else if (gpid.ui < param.gir_pid.ui_min){
		gpid.ui = param.gir_pid.ui_min;	}

	gpid.ut = gpid.up+gpid.ui+gpid.ud;
	if (gpid.ut > param.gir_pid.ut_maks){
		gpid.ut = param.gir_pid.ut_maks;}
	else if (gpid.ut < param.gir_pid.ut_min){
		gpid.ut = param.gir_pid.ut_min;	}

	gpid.es = gpid.e;
	gpid.fbfs = gpid.fbf;
	gpid.uis = gpid.ui;

	float prosent_paadrag = kg_til_paadrag(gpid.ut);
	girbidrag.hhf = prosent_paadrag;
	girbidrag.hhb = -prosent_paadrag;
	girbidrag.hvb = prosent_paadrag;
	girbidrag.hvf = -prosent_paadrag;
}

float pid(pid_struct *pid, float state, float ref){
	float e = ref - state;
	float fstate = pid->fstate_old*pid->a + state*pid->b; //filter on state to reduce jerk

	float p = pid->Kp * e;
	float i = pid->i_old + pid->Ki*(e+pid->e_old)*TSTEP /2;
	float d = pid->Kd * (fstate - pid->fstate_old) /TSTEP;

	if (i > pid->umax){i = pid->umax;}
	else if (i < pid->umin){i = pid->umin;}

	float u = p + i - d;
	if (u > pid->umax){u = pid->umax;}
	else if (u < pid->umin){u = pid->umin;}

	pid->e_old = e; pid->i_old = i; pid->fstate_old = fstate;

	return u;
}

float smc(smc_struct *smc, float state, float ref){
	float e = state - ref;
	float e_dt = (e - smc->e_old)/TSTEP;
	float e_int =smc->e_int_old +  (smc->e_old+e)*TSTEP/2;
	float ref_dt = (ref - smc->ref_old)/TSTEP;
	float ref_dt_dt = (ref_dt - smc->ref_dt_old)/TSTEP;

	float s = e_dt + smc->c1*e + smc->c2*e_int;

	float acc = calc_acceleration_smc(&smc, &model_states);
	float u = - (s)/(fabs(s)+smc->epsilon) * (smc->g_inv * fabs(smc->c1 *e_dt + smc->c2*e + acc - ref_dt_dt)  + smc->b0);

	smc->e_old = e;
	smc->e_int_old = e_int;
	smc->ref_old = ref;
	smc->ref_dt_old = ref_dt;

	if (u > smc->umax){u = smc->umax;}
	else if (u < smc->umin){u = smc->umin;}

	return u;
}

void lqr(lqr_struct *lqr, state_struct *state, NED_eta *ref, NED_eta *thrust){
	float e_z = - ref->z + state->z;
	float e_z_int = lqr->e_z_int_old + (e_z + lqr->e_z_old)/2 *TSTEP;

	float e_phi = - ref->phi + state->phi;
	float e_phi_int = lqr->e_phi_int_old + (e_phi + lqr->e_phi_old)/2 *TSTEP;

	float e_theta = - ref->theta + state->theta;
	float e_theta_int = lqr->e_theta_int_old + (e_theta + lqr->e_theta_old)/2 *TSTEP;

	float x[9] = {state->w, state->p, state->q, e_z, e_phi, e_theta, e_z_int, e_phi_int, e_theta_int};

	for (int i = 0; i < lqr->K.rows; i++) {
		float result = 0.0;
		for (int j = 0; j < lqr->K.cols; j++) {
			result += lqr->K.data[i][j] * x[j];
		}
		// insert result as the i-th element in &thrust
		*((float *)thrust + i) = result;
	}
	thrust->phi += 4.91*cos(ref->theta) * sin(ref->phi);
	thrust->theta += 4.9*sin(ref->theta);

	lqr->e_z_old = e_z;
	lqr->e_phi_old = e_phi;
	lqr->e_theta_old = e_theta;

	lqr->e_z_int_old = e_z_int;
	lqr->e_phi_int_old = e_phi_int;
	lqr->e_theta_int_old = e_theta_int;
}



/*
% f(x) = (p1*x^3 + p2*x^2 + p3*x + p4) / (x + q1)
% Coefficients (with 95% confidence bounds):
%        p1 =  -4.439e-06  (-6.606e-06, -2.272e-06)
%        p2 =     0.04196  (0.03473, 0.04919)
%        p3 =       17.17  (11.08, 23.26)
%        p4 =      0.2886  (-60.69, 61.26)
%        q1 =       19.56  (-7.809, 46.92)
%      f(x) = (p1*x^3 + p2*x^2 + p3*x + p4) / (x + q1)
% Coefficients (with 95% confidence bounds):
%        p1 =  -3.394e-06  (-4.396e-06, -2.391e-06)
%        p2 =     0.03539  (0.03129, 0.03949)
%        p3 =       18.06  (13.95, 22.18)
%        p4 =      0.2909  (-50.84, 51.42)
%        q1 =       22.05  (1.823, 42.27)
*/
