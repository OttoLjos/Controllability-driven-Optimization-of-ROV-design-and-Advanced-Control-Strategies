/*
 * structures.h
 *
 *  Created on: Jan 20, 2022
 *      Author: trcho
 */

#ifndef INC_STRUCTURES_H_
#define INC_STRUCTURES_H_

#include "main.h"
//#include <variabler.h>


//typedef struct {
//	innhold;
//} navn;

typedef struct {
	int8_t jag; 		// x-retning
	int8_t svai;		// y-retning
	int8_t hiv;			// z-retning
	int8_t gir;			// rotasjon om z
	int8_t manipulator;	// manipulator '0b reset on grab release cw ccw ut inn'
	int8_t rull;		// rotasjon om x
	int8_t stamp;		// rotasjon om y
	int8_t throttling;	// paadragsbegrensning
} styrestruct;

typedef struct{
	uint8_t flagg;
} flagg_8b;

typedef struct {
	float a; 						// stigningsrate
	float b; 						// konstantledd
}  motor_karakteristikk_modell;		// 1. ordens modell for kompensering av motorkarakteristikken
									// siden propellen er mer effektiv en retning.

typedef struct {
	float ph;		// skalering for pådrag horisontalt
	float pv;		// skalering for pådrag vertikalt
	float hhf;		// HHF proporsjonalt med makspådrag
	float hhb;		// HHB proporsjonalt med makspådrag
	float hvb;		// HVB proporsjonalt med makspådrag
	float hvf;		// HVF proporsjonalt med makspådrag
	float vhf;		// VHF proporsjonalt med makspådrag
	float vhb;		// VHB proporsjonalt med makspådrag
	float vvb;		// VVB proporsjonalt med makspådrag
	float vvf;		// VVF proporsjonalt med makspådrag
} thruster_skalering;		// thrusterskalering


typedef struct {
	uint32_t param_id;
	float parameter;
} can_param;

typedef struct{
	float hiv;
	float gir;
	float rull;
	float stamp;
	float teleskop;
	float vri;
	float klype;
	float pfa; 			//pådragsfilter a
	float pfb;			//pådragsfilter 1-a
} styredemping;

typedef struct{
	float kp;			// Proporsjonalforsterkning
	float ki;			// Integratorforsterkning
	float kd;			// Derivatorforsterkning
	float ts;			// tidskonstant
	float dz;			// deadzone
	float a;			// IIR Filterkoeffisient
	float b; 			// IIR Invers filterkoeffisient (1-a)
	float ui_maks;		// Integratorbegrensning tak
	float ui_min;		// Integratorbegrensning gulv
	float ut_maks;		// Totalpådragsbegrensning tak
	float ut_min;		// Totalpådragsbegrensning gulv
} pid_param;

typedef struct{
	float yr;			// Inngangsreferanse
	float e;			// Avvik (error)
	float es;			// Forrige avvik
	float up;			// Proporsjonalpådrag
	float ui;			// Integratorpådrag
	float uis;			// Forrige Integratorpådrag
	float ud;			// Derivatorpådrag
	float uds;			// Forrige Derivatorpådrag
	float fbf;			// Feedback Filtrert (Måling filtrert i IIR)
	float fbfs;			// Forrige Feedback Filtrert
	float ut;			// Totalpådrag up+ui+ud
} pid_variabler;

typedef struct{
	styredemping demping;
	pid_param rull_pid;
	pid_param hiv_pid;
	pid_param stamp_pid;
	pid_param gir_pid;
} parameter_struct;

//typedef struct {
//	int16_t u_dot;
//	int16_t v_dot;
//	int16_t w_dot;
//	int16_t z;
//	int16_t phi;
//	int16_t theta;
//	int16_t psi;
//	int16_t u_dot_old;
//	int16_t v_dot_old;
//	int16_t w_dot_old;
//	int16_t z_old;
//	int16_t phi_old;
//	int16_t theta_old;
//	int16_t psi_old;
//} sensordata_struct;

//typedef struct {
//	float x;
//	float y;
//	float z;
//	float phi;
//	float theta;
//	float psi;
//	float u;
//	float v;
//	float w;
//	float p;
//	float q;
//	float r;
//} state_struct;

typedef struct {
	int16_t u_dot, v_dot, w_dot, z, phi, theta, psi, u_dot_old, v_dot_old, w_dot_old, z_old, phi_old, theta_old, psi_old;
} sensordata_struct;

typedef struct {
	float x,y,z,phi,theta,psi,u,v,w,p,q,r;
} state_struct;

typedef struct {
    int rows;
    int cols;
    float **data;  // Pointer to matrix data
} Matrix;

typedef struct {
	float x, y, z, phi, theta, psi;
} NED_eta;

typedef struct {
	uint32_t seq_time, incr_timer;
	uint8_t TIMER_STATUS;
} step_timers;

typedef struct {
	uint8_t sine_trajectory;
	uint32_t step_start, step_stop;
	float step_size, ref_rate, freq1, amp1, freq2, amp2; // per 100ms
} step_test;

typedef struct {
	uint8_t SEQ_STATUS;
	uint32_t test_length;
	step_test heave, roll, pitch;
	NED_eta active_ref, old_ref;
} step_sequence;

typedef struct {
	float Kp, Ki, Kd, a, b, umax, umin;
	float u, e_old, i_old, fstate_old;
	uint8_t reg_state;
}pid_struct;

typedef struct {
	float e_z_old, e_phi_old, e_theta_old;
	float e_z_int_old, e_phi_int_old, e_theta_int_old;
	Matrix K;
} lqr_struct;

typedef struct {
	float c1, c2, epsilon, b0, g_inv, umin, umax;
	float ref_old, ref_dt_old, e_old, e_int_old;
	uint8_t reg_state;
}smc_struct;



typedef struct {
	int16_t hiv;	// dybdeposisjon i cm
	int16_t rull;	// rullvinkel i tidels grader
	int16_t stamp;	// stampvinkel i tidels grader
	int16_t gir;	// girvinkel
} gyrostruct;

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t heading;
} imustruct;

typedef struct {
	uint8_t regulering;	// reguleringsflagg
	uint8_t hiv_pause;	// reguleringspause når dybden endres
	uint8_t rull_pause; // reguleringspause når rull endres
	uint8_t stamp_pause;// reguleringspause når stamp endres
	uint8_t gir_pause; 	// reguleringspause når gir endres
	uint8_t spid;		// stamp-reguleringsflagg
	uint8_t rpid;		// rull-reguleringsflagg
	uint8_t hpid;		// hiv-reguleringsflagg
	uint8_t gpid;		// gir-reguleringsflagg
	uint8_t can_69;		// Autonom styrekommandoer
	uint8_t can_70;		// Mottatt styredata over CAN
	uint8_t can_71;		// Mottatt ny verdi for parameter
	uint8_t can_80;		// Mottatt Gyrodata fra sensornode
	uint8_t can_81;		// Mottatt akselerasjonsdata fra sensornoden
	uint8_t can_82;
	uint8_t can_90;		// Mottatt strømdata
	uint8_t can_91;
	uint8_t manipulator_null;
	uint8_t test_thrustere; // testfunksjon for thrustere
	uint8_t testfunksjon;	// testfunksjon kjører
}FLAGG_struct;

typedef struct {
	int16_t hhf;
	int16_t hhb;
	int16_t hvb;
	int16_t hvf;
	int16_t vhf;
	int16_t vhb;
	int16_t vvb;
	int16_t vvf;
} thruster_sett;

typedef struct {
	int8_t hhf;
	int8_t hhb;
	int8_t hvb;
	int8_t hvf;
	int8_t vhf;
	int8_t vhb;
	int8_t vvb;
	int8_t vvf;
} thruster_datastruct;

typedef struct {
	float hhf;
	float hhb;
	float hvb;
	float hvf;
	float vhf;
	float vhb;
	float vvb;
	float vvf;
} thruster_sett_float;

typedef struct {
	int16_t teleskop;
	int16_t vri;
	int16_t klype;
} manipulator_sett;

typedef struct {
	float N;
	float NE;
	float E;
	float SE;
	float S;
	float SW;
	float W;
	float NW;
} kompass;

typedef struct {
	uint16_t thruster_12v;
	uint16_t manipulator_12v;
	uint16_t elektronikk_5v;
	uint16_t tom;	
} effekt_struct;

#endif /* INC_STRUCTURES_H_ */
