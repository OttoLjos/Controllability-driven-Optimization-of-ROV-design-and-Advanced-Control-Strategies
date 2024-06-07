/*
 * variabler.h
 *
 *  Created on: Mar 22, 2022
 *      Author: trcho
 */

#ifndef INC_VARIABLER_EXT_H_
#define INC_VARIABLER_EXT_H_

#include <main.h>
#include "structures.h"
//#include <variabler.h>

extern uint8_t test_thrustere;
extern int32_t null8[8];
extern uint16_t oppstart_PWM[8];

extern FLAGG_struct flagg;
extern uint8_t hiv_FLAGG;
//
extern uint8_t teller_reg_hiv, teller_reg_rull, teller_reg_stamp, teller_reg_gir;
extern uint8_t teller_bryter;
extern uint16_t teller_diode;
extern uint16_t teller_test_thruster;
extern uint16_t teller_idle;
extern uint8_t test_reg;
extern uint32_t teller_test_reg;
extern int8_t esc_status_avprell;
extern uint8_t esc_hiv, esc_rull, esc_stamp, esc_gir, esc_reset;
extern uint16_t teller_esc_status;

extern uint8_t thruster_ID;
extern uint8_t forrige_bryterstatus, brytertrykk;

extern uint8_t led_status;

extern volatile float styreretning, maks_paadrag, skalering, gir, hiv, stamp, rull, gain;
extern volatile float hiv_m, rull_m, stamp_m, gir_m, gir_mf, power12vf;
extern volatile uint32_t sum_paadrag;
extern styrestruct styremelding, autonomstyring;
extern thruster_sett thruster_PWM;
extern thruster_datastruct thrusterdata;
extern manipulator_sett manipulator_PWM, manipulator_filter;
extern gyrostruct gyrodata;
extern imustruct aksdata, magnodata;
extern kompass HR_kompass_rad;
extern thruster_sett_float totalbidrag, styrebidrag, stampbidrag, rullbidrag, hivbidrag, girbidrag, styrefilter;
extern effekt_struct effekt_forbruk;

extern thruster_sett_float ctrl_contribution, active_thrust;
extern sensordata_struct sensordata;
extern state_struct model_states;
extern NED_eta ctrl_ref, thrust_NED, thrust_BFF;
extern pid_struct pid_heave, pid_roll, pid_pitch;
extern lqr_struct lqr_reg;
extern smc_struct smc_heave, smc_roll, smc_pitch;
extern uint8_t controller_status;
extern Matrix Tmat;
extern step_timers step_counter;
extern step_sequence sep_steps, overlap_steps, heave_step, roll_step, pitch_step, active_step, sep_sine, overlap_sine;


extern parameter_struct param;

extern pid_variabler spid, rpid, hpid, gpid;

extern thruster_skalering t_skalering;		// thrusterskalering


extern can_param ny_param;

extern motor_karakteristikk_modell mk_modell;

#endif /* INC_VARIABLER_EXT_H_ */
