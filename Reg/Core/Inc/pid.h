/*
 * pid.h
 *
 *  Created on: Feb 3, 2022
 *      Author: trcho
 */

#ifndef INC_PID_H_
#define INC_PID_H_

/* Includes --------------------------------------------------- */

/* Variables -------------------------------------------------- */

extern volatile float styreretning;

/* Private funksjonsprototyper -------------------------------- */

void stamp_regulator(void);
void rull_regulator(void);
void hiv_regulator(void);
void gir_regulator(void);
float pid(pid_struct *pid, float state, float ref);
void lqr(lqr_struct *lqr, state_struct *state, NED_eta *ref, NED_eta *thrust);
float smc(smc_struct *smc, float state, float ref);




/* Private funksjonsdeklarasjoner ----------------------------- */

#endif /* INC_PID_H_ */
