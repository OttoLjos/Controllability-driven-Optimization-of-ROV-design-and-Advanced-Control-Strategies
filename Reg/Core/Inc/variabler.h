/*
 * variable_decl.h
 *
 *  Created on: Mar 22, 2022
 *      Author: trcho
 */

#ifndef INC_VARIABLER_H_
#define INC_VARIABLER_H_

#include <main.h>
#include "structures.h"

uint8_t test_thrustere;
int32_t null8[8];// = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000};
uint16_t oppstart_PWM[8] = {NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL};

 FLAGG_struct flagg;
 uint8_t hiv_FLAGG;
//
 uint8_t teller_reg_hiv, teller_reg_rull, teller_reg_stamp, teller_reg_gir;
 uint8_t teller_bryter;
//	uint8_t teller_bryter = 0;
//	uint8_t teller_reg_hiv = 0;
 uint16_t teller_diode;
 uint16_t teller_test_thruster;
 uint16_t teller_idle;
 uint8_t test_reg;
 uint32_t teller_test_reg;
 int8_t esc_status_avprell;
 uint8_t esc_hiv, esc_rull, esc_stamp, esc_gir, esc_reset;
 uint16_t teller_esc_status;

 uint8_t thruster_ID;
 uint8_t forrige_bryterstatus, brytertrykk;

 uint8_t led_status;

 volatile float styreretning, maks_paadrag, skalering, gir, hiv, stamp, rull, gain;
 volatile float hiv_m, rull_m, stamp_m, gir_m, gir_mf, power12vf;
 volatile uint32_t sum_paadrag;
 styrestruct styremelding, autonomstyring;
 thruster_sett thruster_PWM;
 thruster_datastruct thrusterdata;
 manipulator_sett manipulator_PWM, manipulator_filter;
 gyrostruct gyrodata;
 imustruct aksdata, magnodata;
 kompass HR_kompass_rad;
 thruster_sett_float totalbidrag, styrebidrag, stampbidrag, rullbidrag, hivbidrag, girbidrag, styrefilter;
 effekt_struct effekt_forbruk;

 thruster_sett_float ctrl_contribution, active_thrust;
 sensordata_struct sensordata;
 state_struct model_states;
 NED_eta ctrl_ref, thrust_NED, thrust_BFF;
 pid_struct pid_heave, pid_roll, pid_pitch;
 lqr_struct lqr_reg;
 smc_struct smc_heave, smc_roll, smc_pitch;
 uint8_t controller_status;
 Matrix Tmat;
 step_timers step_counter;
 step_sequence sep_steps, overlap_steps, heave_step, roll_step, pitch_step, active_step, sep_sine, overlap_sine;



 parameter_struct param;

 pid_variabler spid, rpid, hpid, gpid;

can_param ny_param;
thruster_skalering t_skalering;
motor_karakteristikk_modell mk_modell;


#endif /* INC_VARIABLER_H_ */
