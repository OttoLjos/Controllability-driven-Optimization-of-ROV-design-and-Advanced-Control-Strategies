/*
 * testing.h
 *
 *  Created on: Feb 23, 2022
 *      Author: trcho
 */

#ifndef INC_TESTFUNKSJONER_H_
#define INC_TESTFUNKSJONER_H_

/* Includes --------------------------------------------------- */
#include <structures.h>

/* Variables -------------------------------------------------- */

//extern styrestruct test_sm; // styremelding for testing
//extern uint8_t test_dioder_toggle;


/* Private funksjonsprototyper -------------------------------- */


void test_led_dioder(void);
void test_tpwm_pinner(void);
void test_mpwm_pinner(void);
void test_styremeldinger(void);
void test_thruster(uint32_t thrusterID);
#endif /* INC_TESTFUNKSJONER_H_ */
