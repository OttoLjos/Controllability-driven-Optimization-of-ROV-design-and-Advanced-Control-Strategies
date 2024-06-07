/*
 * funksjoner.h
 *
 *  Created on: Jan 18, 2022
 *      Author: trcho
 */

#ifndef INC_FUNKSJONER_H_
#define INC_FUNKSJONER_H_

/* Includes --------------------------------------------------- */


/* Variables -------------------------------------------------- */

//extern volatile float styreretning;

/* Private funksjonsprototyper -------------------------------- */

void skriv_thruster_PWM(void);
void skriv_manipulator_PWM(void);
void oppdater_LED_moenster(void);
void sjekk_brytertrykk(void);
void behandle_paadrag(void);
void oppdater_styrebidrag(void);
void oppdater_ledlys(void);
void oppstartsverdier(void);
void oppdater_parameter(uint32_t param_id, float parameter);
void manipulator_styring(void);
float kg_til_paadrag(float kg_thrust);
void skaler_paadrag(void);
void thruster_retning_korreksjon(void);
void effekt_kontroll(void);
void paadrags_filter(void);
void kombiner_styring(void);
void ESC_status(void);
void add_contribution(thruster_sett_float *result, thruster_sett_float *added);
void NED2BFF(NED_eta *ned, NED_eta *BFF, const state_struct *state);
void BFF2ThrustPercent(const NED_eta *BFF, thruster_sett_float *thrust);
float thrust2percent(float thrust);
float calc_acceleration_smc(smc_struct *smc, state_struct *states);
void step_machine(const step_test *step, float *c_ref, float old_ref);
void thrust_filtering(thruster_sett_float *thrust, thruster_sett_float *active);

/* Private funksjonsdeklarasjoner ----------------------------- */


/* Includes ----------------------------------------- */

#endif /* INC_FUNKSJONER_H_ */


