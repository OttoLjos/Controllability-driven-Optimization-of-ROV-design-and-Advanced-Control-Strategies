/*
 * verdier.h
 *
 *  Created on: Mar 28, 2022
 *      Author: trcho
 */

#ifndef INC_VERDIER_H_
#define INC_VERDIER_H_


/* Includes --------------------------------------------------- */



/* Private funksjonsprototyper -------------------------------- */

void oppstartsverdier(void);
void oppdater_parameter(uint32_t param_id, float parameter);
void reset_pid_values(pid_struct *pid);
void reset_lqr_values(lqr_struct *lqr);
void reset_smc_values(smc_struct *smc);


/* ------------------------------------------------------------ */

#endif /* INC_VERDIER_H_ */
