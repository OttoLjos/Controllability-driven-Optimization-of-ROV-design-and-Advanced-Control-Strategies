/*
 * verdier.c
 *
 *  Created on: Mar 28, 2022
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
#include <matrix_operations.h>

	uint8_t c;


void oppstartsverdier(void){
	param.demping.gir = 0.7;
	param.demping.hiv = 1.0;
	param.demping.rull = 1.0;
	param.demping.stamp = 1.0;
	param.demping.klype = 0.35;
	param.demping.vri = 0.25;
	param.demping.teleskop = 0.3;
	param.demping.pfa = 0.95;
	param.demping.pfb = 1 - param.demping.pfa;
	gain = 4;
	styreretning = 0.0;
	forrige_bryterstatus = 0;
	brytertrykk = 0;
	float HR_rad[8] = {0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI, -3*M_PI/4, -M_PI/2, -M_PI/4};
	memcpy(&HR_kompass_rad, &HR_rad, sizeof(kompass));
	memcpy(&thruster_PWM, 		&oppstart_PWM, sizeof(thruster_sett));
	memcpy(&manipulator_PWM, 	&oppstart_PWM, sizeof(manipulator_sett));
	memset(&styrebidrag,0, sizeof(thruster_sett));
	memset(&totalbidrag,0, sizeof(thruster_sett));
	memset(&stampbidrag,0, sizeof(thruster_sett));
	memset(&rullbidrag, 0, sizeof(thruster_sett));
	memset(&hivbidrag, 	0, sizeof(thruster_sett));
	mk_modell.a = 0.0024;
	mk_modell.b = 1;

	spid.yr = 0.0;
	rpid.yr = 0.0;
	hpid.yr = 0.0;

	param.hiv_pid.a = 0.85;
	param.hiv_pid.b = 1-param.hiv_pid.a;
	param.hiv_pid.dz = 0;
	param.hiv_pid.kp = 1.75;
	param.hiv_pid.ki = 0.1;
	param.hiv_pid.kd = 0.64;
	param.hiv_pid.ts = 0.05;
	param.hiv_pid.ui_min = -0.8;
	param.hiv_pid.ui_maks = 0.8;
	param.hiv_pid.ut_min = -0.8;
	param.hiv_pid.ut_maks = 0.8;

	param.rull_pid.a = 0.95;
	param.rull_pid.b = 1-param.rull_pid.a;
	param.rull_pid.dz = 0;
	param.rull_pid.kp = 0.011;
	param.rull_pid.ki = 0.004; // 0.287;
	param.rull_pid.kd = 0.0087;
	param.rull_pid.ts = 0.05;
	param.rull_pid.ui_min = -0.5;
	param.rull_pid.ui_maks = 0.5;
	param.rull_pid.ut_min = -0.5;
	param.rull_pid.ut_maks = 0.5;

	param.stamp_pid.a = 0.95;
	param.stamp_pid.b = 1-param.stamp_pid.a;
	param.stamp_pid.dz = 0;
	param.stamp_pid.kp = 0.0255;
	param.stamp_pid.ki = 0.008; //0.353;
	param.stamp_pid.kd = 0.0211;
	param.stamp_pid.ts = 0.05;
	param.stamp_pid.ui_min = -0.6;
	param.stamp_pid.ui_maks = 0.6;
	param.stamp_pid.ut_min = -0.6;
	param.stamp_pid.ut_maks = 0.6;

	param.gir_pid.a = 0.95;
	param.gir_pid.b = 1-param.stamp_pid.a;
	param.gir_pid.dz = 0;
	param.gir_pid.kp = 0.0255;
	param.gir_pid.ki = 0.008; //0.353;
	param.gir_pid.kd = 0.0211;
	param.gir_pid.ts = 0.05;
	param.gir_pid.ui_min = -0.4;
	param.gir_pid.ui_maks = 0.4;
	param.gir_pid.ut_min = -0.4;
	param.gir_pid.ut_maks = 0.4;


	// Master control:
	float matrixData[8 * 6] = {
	     0.3536, 	0.3536, 	0.0000, 	 0.0000, 	 0.0000, 	 0.8081,
	    -0.3536, 	0.3536, 	0.0000, 	 0.0000, 	 0.0000, 	-0.8081,
	    -0.3536,   -0.3536, 	0.0000, 	 0.0000, 	 0.0000,	 0.8081,
	     0.3536,   -0.3536, 	0.0000, 	 0.0000, 	 0.0000, 	-0.8081,
	     0.0000, 	0.0000, 	0.2500, 	-0.8929, 	-1.0870, 	 0.0000,
	     0.0000, 	0.0000, 	0.2500, 	-0.8929, 	 1.0870, 	 0.0000,
	     0.0000, 	0.0000, 	0.2500, 	 0.8929, 	 1.0870, 	 0.0000,
	     0.0000, 	0.0000, 	0.2500, 	 0.8929, 	-1.0870,	 0.0000
	};
	assign_matrix_data(&Tmat, 8, 6, matrixData);

	step_test stdstep;
	stdstep.step_start = 5000;
	stdstep.step_stop = 20000;
	stdstep.ref_rate = 0.025;
	stdstep.sine_trajectory = 0;

	memcpy(&heave_step.heave, &stdstep, sizeof(step_test));
	memcpy(&heave_step.roll, &stdstep, sizeof(step_test));
	memcpy(&heave_step.pitch, &stdstep, sizeof(step_test));
	heave_step.test_length = 25000;

	memcpy(&overlap_steps, &heave_step, sizeof(step_sequence));
	memcpy(&sep_steps, &heave_step, sizeof(step_sequence));
	memcpy(&roll_step, &heave_step, sizeof(step_sequence));
	memcpy(&pitch_step, &heave_step, sizeof(step_sequence));

	sep_steps.heave.step_size  		= heave_step.heave.step_size 		= -0.5;
	sep_steps.roll.step_size		= roll_step.roll.step_size 			= M_PI/12;
	sep_steps.pitch.step_size	 	= pitch_step.pitch.step_size 		= M_PI/12;

	sep_steps.roll.step_start 	= 35000;
	sep_steps.roll.step_stop 	= 50000;
	sep_steps.pitch.step_start 	= 65000;
	sep_steps.pitch.step_stop 	= 80000;
	sep_steps.test_length 		= 95000;

	overlap_steps.roll.step_start 	= 8500;
	overlap_steps.roll.step_stop 	= 18500;
	overlap_steps.pitch.step_start 	= 12000;
	overlap_steps.pitch.step_stop 	= 22000;
	overlap_steps.test_length 		= 30000;

	stdstep.sine_trajectory = 1;
	stdstep.amp1 			= 0.25;
	stdstep.freq1 			= 2*M_PI/16;
	stdstep.amp2 			= 0.0; // 0.25;
	stdstep.freq2 			= 0.0; // 2*M_PI/8;
	memcpy(&sep_sine.heave, &stdstep, sizeof(step_test));
	memcpy(&sep_sine.roll, &stdstep, sizeof(step_test));
	memcpy(&sep_sine.pitch, &stdstep, sizeof(step_test));
	memcpy(&overlap_sine, &sep_sine, sizeof(step_sequence));
	sep_sine.heave.step_stop 	= 37000;
	sep_sine.roll.amp1 			= M_PI/6;
	sep_sine.roll.step_start 	= 40000;
	sep_sine.roll.step_stop 	= 72000;
	sep_sine.pitch.amp1 		= M_PI/4;
	sep_sine.pitch.step_start 	= 75000;
	sep_sine.pitch.step_stop 	= 107000;
	sep_sine.test_length 		= 110000;

	overlap_sine.heave.step_stop 	= 45000;
	overlap_sine.roll.step_start 	= 10000;
	overlap_sine.roll.step_stop 	= 45000;
	overlap_sine.pitch.step_start 	= 15000;
	overlap_sine.pitch.step_stop 	= 45000;
	overlap_sine.test_length 		= 50000;
	memset(&step_counter, 0, sizeof(step_timers));

	//	pid_struct pid_heave, pid_roll, pid_pitch;
	//	lqr_struct lqr_reg;
	//	smc_struct smc_heave, smc_roll, smc_pitch

	// Initialize pid_struct instance
	pid_struct pid;
	pid.reg_state = 3;
	pid.Kp = 480.0;
	pid.Ki = 30.0;
	pid.Kd = 450.0;
	pid.a = 0.1577;
	pid.b = 1-pid.a;
	pid.umax = 138;
	pid.umin = -138;
	memcpy(&pid_heave, 	&pid, sizeof(pid_struct));

	pid.reg_state = 4;
	pid.Kp = 12.0;
	pid.Ki = 10.0;
	pid.Kd = 20.0;
	pid.a = 0.2544;
	pid.b = 1-pid.a;
//	pid.umax = 44;
//	pid.umin = -44;
	memcpy(&pid_roll, 	&pid, sizeof(pid_struct));

	pid.reg_state = 5;
	pid.Kp = 16.0;
	pid.Ki = 12.0;
	pid.Kd = 20.0;
	pid.a = 0.2301;
	pid.b = 1-pid.a;
//	pid.umax = 36;
//	pid.umin = -36;
	memcpy(&pid_pitch, 	&pid, sizeof(pid_struct));

	// PID_z =     [179.0554   , 5.8977    , 519.8582  , 0.1577];
	// PID_phi =   [32.1605    , 29.7081   , 7.6614    , 0.2544];
	// PID_theta = [66.4578    , 43.9984   , 21.3043   , 0.2301];

	// Initialize lqr_struct instance 
	// float Kdata[6*9];
	float Kdata[6*9] =  {
		0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0,
		-321.0, 0, 0, -489.0, 0, 0, -42.0, 0, 0,
		0, -2.2, 0, 0, -9.0, 0, 0, -6.0, 0,
		0, 0, -3.2, 0, 0, -9.0, 0, 0, -5.8,
		0, 0, 0, 0, 0, 0, 0, 0, 0
	};
	assign_matrix_data(&lqr_reg.K, 6, 9, Kdata);

	// Initialize smc_struct instance 
	// smc_struct smc;
	smc_heave.reg_state = 3;
	smc_heave.epsilon = 0.2;
	smc_heave.b0 = 36.0;
	smc_heave.c1 = 1.6;
	smc_heave.c2 = 0.10;
	smc_heave.umax = 138;
	smc_heave.umin = -138;
	smc_heave.g_inv = 124.1027;
	// memcpy(&smc_heave, 	&smc, sizeof(smc_struct));
	
	smc_roll.reg_state = 4;
	smc_roll.epsilon = 0.4;
	smc_roll.b0 = 12.0;
	smc_roll.c1 = 1.0;
	smc_roll.c2 = 0.8;
	smc_roll.umax = 138;//44;
	smc_roll.umin = -138;//44;
	smc_roll.g_inv = 1.0287;
	// memcpy(&smc_roll, 	&smc, sizeof(smc_struct));

	smc_pitch.reg_state = 5;
	smc_pitch.epsilon = 0.4;
	smc_pitch.b0 = 15.0;
	smc_pitch.c1 = 1.0;
	smc_pitch.c2 = 1.0;
	smc_pitch.umax = 138;//36;
	smc_pitch.umin = -138;//36;
	smc_pitch.g_inv = 2.1733;
	// memcpy(&smc_pitch, 	&smc, sizeof(smc_struct));

	// New Control Law
	// F_z =  -z_d*(6.11*abs(z_d)*(cos(phi)^2*cos(theta)^2)-sin(phi)^2-0.33*sin(theta)^2+sin(phi)^2*sin(theta)^2 +1)
	// F_phi  = -(sin(phi)*(3.63-0.66*cos(theta)^2+theta_d*cos(phi)*sin(theta)*(2.18+1.3*abs(theta_d)))/cos(theta) -2.66*phi_*abs(phi_d)
	// F_theta = -cos(phi)*(3.1*sin(theta)+theta_d*cos(phi)*(2.16+1.27*abs(theta_d)))

	// Old control law
	// zeta =      [0, 0, 3.0, 8.47, 8.83,0];
	// epsilon =   [0, 0, 1.80, 0.44, 0.42,0];
	// rho =       [0, 0, 1.90, 3.03, 2.85,0];
	// c1 =        [0, 0, 1.00, 3.99, 2.39,0];
	// c2 =        [0, 0, 0.10, 0.53, 0.64,0];
	// [Minv_Ds, Minv_Dm, Minv_G, Binv]
	// Heave: [-0.2267, -1.417, 0, 124.1027]
	// Roll: [-3.154, -0.5218, -3.98, 1.0287]
	// Pitch: [-2.17, -0.7095, -1.884, 2.1733]
}

void reset_pid_values(pid_struct *pid){
	pid->e_old = 0;
	pid->i_old = 0;
	pid->fstate_old = 0;
}

void reset_lqr_values(lqr_struct *lqr){
	lqr->e_z_old = 0;
	lqr->e_phi_old = 0;
	lqr->e_theta_old = 0;
	lqr->e_z_int_old = 0;
	lqr->e_phi_int_old = 0;
	lqr->e_theta_int_old = 0;
}

void reset_smc_values(smc_struct *smc){
	smc->ref_old = 0;
	smc->ref_dt_old = 0;
	smc->e_old = 0;
	smc->e_int_old = 0;
}

void oppdater_parameter(uint32_t param_id, float parameter){
	switch(param_id){
	case 1:
		if(!esc_reset){
			esc_reset = 1;
		}
		break;
	case 2:
		flagg.hpid = flagg.rpid = flagg.spid = flagg.gpid = flagg.hiv_pause = flagg.gir_pause = flagg.rull_pause = flagg.stamp_pause = 0;
		memset(&hivbidrag, 0, sizeof(thruster_sett_float));
		memset(&rullbidrag, 0, sizeof(thruster_sett_float));
		memset(&stampbidrag, 0, sizeof(thruster_sett_float));
		memset(&girbidrag, 0, sizeof(thruster_sett_float));
		break;
	case 3:
		if(flagg.hiv_pause){flagg.hiv_pause = 0;}
		else{flagg.hpid = !flagg.hpid;}
		if(!flagg.hpid){memcpy(&hivbidrag, &null8, sizeof(thruster_sett_float));}
		hpid.yr = hiv_m;
		hpid.es = hpid.fbfs = hpid.uds = hpid.uis = 0;
		break;
	case 4:
		if(flagg.rull_pause){flagg.rull_pause = 0;}
		else{flagg.rpid = !flagg.rpid;}
		if(!flagg.rpid){memcpy(&rullbidrag, &null8, sizeof(thruster_sett_float));}
		rpid.es = rpid.fbfs = rpid.uds = rpid.uis = 0;
		break;
	case 5:
		if(flagg.stamp_pause){flagg.stamp_pause = 0;}
		else{flagg.spid = !flagg.spid;}
		if(!flagg.spid){memcpy(&stampbidrag, &null8, sizeof(thruster_sett_float));}
		spid.es = spid.fbfs = spid.uds = spid.uis = 0;
		break;
	case 6:
		if(flagg.gir_pause){flagg.gir_pause = 0;}
		else{flagg.gpid = !flagg.gpid;}
		if(!flagg.gpid){memcpy(&girbidrag, &null8, sizeof(thruster_sett_float));}
		gpid.yr = gir_m;
		gpid.es = gpid.fbfs = gpid.uds = gpid.uis = 0;
		break;
	case 10:
		param.demping.gir = (float) parameter;			break;
	case 11:
		param.demping.hiv = (float) parameter;			break;
	case 12:
		param.demping.rull = (float) parameter;			break;
	case 13:
		param.demping.stamp = (float) parameter;		break;
	case 14:
		param.demping.teleskop = (float) parameter;		break;
	case 15:
		param.demping.vri = (float) parameter;			break;
	case 16:
		param.demping.klype = (float) parameter;		break;
	case 17:
		param.demping.pfa = (float) parameter;
		param.demping.pfb = 1 - (float) parameter;		break;
	case 30:
		param.hiv_pid.kp = (float) parameter;			break;
	case 31:
		param.hiv_pid.ki = (float) parameter;			break;
	case 32:
		param.hiv_pid.kd = (float) parameter;			break;
	case 33:
		param.hiv_pid.ui_min = (float) -parameter;				// begrensning av kg pådrag pr thruster
		param.hiv_pid.ui_maks = (float) parameter;		break;
	case 34:
		param.hiv_pid.ut_min = (float) -parameter;				// begrensning av kg pådrag pr thruster
		param.hiv_pid.ut_maks = (float) parameter;		break;
	case 35:
		hpid.yr += (float) parameter;					break; 	// dybdeendring i meter
	case 36:
		param.hiv_pid.a = parameter;							// filterparameter for derivatorledd
		param.hiv_pid.b = 1-param.hiv_pid.a;			break;
	case 40:
		param.rull_pid.kp = (float) parameter;			break;
	case 41:
		param.rull_pid.ki = (float) parameter;			break;
	case 42:
		param.rull_pid.kd = (float) parameter;			break;
	case 43:
		param.rull_pid.ui_min = (float) -parameter;				// begrensning av kg pådrag pr thruster
		param.rull_pid.ui_maks = (float) parameter;		break;
	case 44:
		param.rull_pid.ut_min = (float) -parameter;				// begrensning av kg pådrag pr thruster
		param.rull_pid.ut_maks = (float) parameter;		break;
	case 45:
		rpid.yr = (float) parameter;					break; 	// rullvinkel-referanse i grader
	case 46:
		param.rull_pid.a = parameter;							// filterparameter for derivatorledd
		param.rull_pid.b = 1-param.rull_pid.a;			break;
	case 50:
		param.stamp_pid.kp = (float) parameter;			break;
	case 51:
		param.stamp_pid.ki = (float) parameter;			break;
	case 52:
		param.stamp_pid.kd = (float) parameter;			break;
	case 53:
		param.stamp_pid.ui_min = (float) -parameter;			// begrensning av kg pådrag pr thruster
		param.stamp_pid.ui_maks = (float) parameter;	break;
	case 54:
		param.stamp_pid.ut_min = (float) -parameter;			// begrensning av kg pådrag pr thruster
		param.stamp_pid.ut_maks = (float) parameter;	break;
	case 55:
		spid.yr = (float) parameter;					break; 	// stampvinkel-referanse i grader
	case 56:
		param.stamp_pid.a = parameter;							// filterparameter for derivatorledd
		param.stamp_pid.b = 1-param.stamp_pid.a;		break;
	case 60:
		param.gir_pid.kp = (float) parameter;			break;
	case 61:
		param.gir_pid.ki = (float) parameter;			break;
	case 62:
		param.gir_pid.kd = (float) parameter;			break;
	case 63:
		param.gir_pid.ui_min = (float) -parameter;				// begrensning av kg pådrag pr thruster
		param.gir_pid.ui_maks = (float) parameter;		break;
	case 64:
		param.gir_pid.ut_min = (float) -parameter;				// begrensning av kg pådrag pr thruster
		param.gir_pid.ut_maks = (float) parameter;		break;
	case 65:
		gpid.yr = (float) parameter;					break; 	// girvinkel-referanse i grader
	case 66:
		param.gir_pid.a = parameter;							// filterparameter for derivatorledd
		param.gir_pid.b = 1-param.gir_pid.a;			break;
	case 80:
		flagg.test_thrustere = 1;
		thruster_ID = (uint8_t) parameter;				break;
	case 90:
		mk_modell.a = (float) parameter;				break;
	case 91:
		mk_modell.b = (float) parameter;				break;

		// Master control
	case 100:
		// Set control status.
		// 0: off, 1: PID, 2: LQR, 3: SMC
		c =(uint8_t) parameter;
		if (( c==1 )||( c==2 )||( c==3 )||( c==4 )){
			memset(&ctrl_ref, 0, sizeof(NED_eta));
			ctrl_ref.z = model_states.z;

			reset_pid_values(&pid_heave); reset_pid_values(&pid_roll); reset_pid_values(&pid_pitch);
			reset_lqr_values(&lqr_reg);
			reset_smc_values(&smc_heave); reset_smc_values(&smc_roll); reset_smc_values(&smc_pitch);
//			test_thrustere = 1;
			controller_status = c;}
		else {
			controller_status = 0;}
		break;

		// set reference value for controller
	case 110:
		// Surge
		ctrl_ref.x = parameter;
		break;
	case 111:
		// Sway
		ctrl_ref.y = parameter;
		break;
	case 112:
		// Heave
		ctrl_ref.z = parameter;
		break;
	case 113:
		// Roll
		ctrl_ref.phi = parameter;
		break;
	case 114:
		// Pitch
		ctrl_ref.theta = parameter;
		break;
	case 115:
		// Yaw
		ctrl_ref.psi = parameter;
		break;
	// Step response sequences
	case 120:
		c =(uint8_t) parameter;
		// Step in heave
		if 		(c==1) 	{memcpy(&active_step, &heave_step, sizeof(step_sequence));}
		else if (c==2)	{memcpy(&active_step, &roll_step,  sizeof(step_sequence));}
		else if (c==3)	{memcpy(&active_step, &pitch_step, sizeof(step_sequence));}
		else if (c==4)	{memcpy(&active_step, &sep_steps,  sizeof(step_sequence));}
		else if (c==5)	{memcpy(&active_step, &overlap_steps,  sizeof(step_sequence));}
		else if (c==6)	{memcpy(&active_step, &sep_sine,  sizeof(step_sequence));}
		else if (c==7)	{memcpy(&active_step, &overlap_sine,  sizeof(step_sequence));}
		else 			{break;}
		memcpy(&active_step.old_ref, &ctrl_ref, sizeof(NED_eta));
//		active_step.old_ref = ctrl_ref;
		step_counter.seq_time = step_counter.incr_timer = 0;
		step_counter.TIMER_STATUS = 1;
		active_step.SEQ_STATUS = 1;

		break;
	case 131:
		smc_heave.b0 = parameter;
		reset_smc_values(&smc_heave);
		break;
	case 132:
		smc_heave.epsilon = parameter;
		reset_smc_values(&smc_heave);
		break;
	case 133:
		smc_heave.c1 = parameter;
		reset_smc_values(&smc_heave);
		break;
	case 134:
		smc_heave.c2 = parameter;
		reset_smc_values(&smc_heave);
		break;
	case 141:
		smc_roll.b0 = parameter;
		reset_smc_values(&smc_roll);
		break;
	case 142:
		smc_roll.epsilon = parameter;
		reset_smc_values(&smc_roll);
		break;
	case 143:
		smc_roll.c1 = parameter;
		reset_smc_values(&smc_roll);
		break;
	case 144:
		smc_roll.c2 = parameter;
		reset_smc_values(&smc_roll);
		break;
	case 151:
		smc_pitch.b0 = parameter;
		reset_smc_values(&smc_pitch);
		break;
	case 152:
		smc_pitch.epsilon = parameter;
		reset_smc_values(&smc_pitch);
		break;
	case 153:
		smc_pitch.c1 = parameter;
		reset_smc_values(&smc_pitch);
		break;
	case 154:
		smc_pitch.c2 = parameter;
		reset_smc_values(&smc_pitch);
		break;
	case 190:
		test_thrustere = 1;
		break;
	case 191:
		test_thrustere = 0;
		break;
	}
}




