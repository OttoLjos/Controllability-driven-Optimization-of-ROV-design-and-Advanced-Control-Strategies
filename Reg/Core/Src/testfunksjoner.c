/*
 * testing.c
 *
 *  Created on: Feb 23, 2022
 *      Author: trcho
 */


#include "main.h"
#include <math.h>
#include <structures.h>
#include <funksjoner.h>
#include <string.h>
#include <stdlib.h>
//#include <testfunksjoner.h>
//#include <variabler.h>
#include <variabler_ext.h>


/* Variabler -------------------------------------------------- */
uint8_t test_teller = 0;
uint16_t led_pinner[9] = {LED_HHF, LED_HHB, LED_HVB, LED_HVF, LED_VHF, LED_VHB, LED_VVB, LED_VVF, 0x0000};

thruster_sett thr_test;

int8_t xt[8] = {0, 50, 0, 75, 100, -75, -50, 30};
int8_t yt[8] = {0, 0, 50, 75, 100, 75, 25, 70};
int8_t zt[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int8_t gt[8] = {0, 0, 0, 50, -50, 0, 0, 0};

int8_t mt[8] = {0, 65, 66, 68, 72, 80, 96, 112};



/* Funksjoner -------------------------------------------------- */
void test_led_dioder(void){
	GPIOE->ODR = led_pinner[test_teller];
	test_teller++;
	if (test_teller >= 9){
		test_teller = 0;
	}
}

void test_tpwm_pinner(void){
	thr_test.hhf = 1499 + test_teller*50;
	thr_test.hhb = 1499 + test_teller*50;
	thr_test.hvb = 1499 + test_teller*50;
	thr_test.hvf = 1499 + test_teller*50;
	thr_test.vhf = 1499 + test_teller*50;
	thr_test.vhb = 1499 + test_teller*50;
	thr_test.vvb = 1499 + test_teller*50;
	thr_test.vvf = 1499 + test_teller*50;

	TIM2->CCR1 = thr_test.hhf;  		// HHF
	TIM2->CCR2 = thr_test.hhb;  		// HHB
	TIM2->CCR3 = thr_test.hvb;  		// HVB
	TIM2->CCR4 = thr_test.hvf;  		// HVF
	TIM3->CCR1 = thr_test.vhf;  		// VHF
	TIM3->CCR2 = thr_test.vhb;  		// VHB
	TIM3->CCR3 = thr_test.vvb;  		// VVB
	TIM3->CCR4 = thr_test.vvf;  		// VVF


	test_teller++;
	if (test_teller >= 8){
		test_teller = 0;
	}
}

void test_mpwm_pinner(void){
	int16_t manipulator_pwm = test_teller*50 + 1499;

	TIM4->CCR1 = manipulator_pwm;	// Teleskop
	TIM4->CCR2 = manipulator_pwm;	// Rull
	TIM4->CCR3 = manipulator_pwm;	// Klype


	test_teller++;
	if (test_teller >= 8){
		test_teller = 0;
	}
}

void test_styremeldinger(void){
	styremelding.jag = 0;	//xt[test_teller];
	styremelding.svai = 0;	//yt[test_teller];
	styremelding.hiv = 0; 	// zt[test_teller];
	styremelding.gir = 0;	//gt[test_teller];
	styremelding.rull = 0;
	styremelding.stamp = 0;
	styremelding.throttling = 0;
	styremelding.manipulator = mt[test_teller];

//	oppdater_styrebidrag();
//	skriv_paadrag();
	manipulator_styring();

	test_teller++;
	if (test_teller >= 8){
		test_teller = 0;
	}
}


void test_thruster(uint32_t thrusterID){
	flagg.testfunksjon = 1;
	if(thrusterID == 1){ thruster_PWM.hhf = 1540;	}
	else if(thrusterID == 2){ thruster_PWM.hhb = 1540;	}
	else if(thrusterID == 3){ thruster_PWM.hvb = 1540;	}
	else if(thrusterID == 4){ thruster_PWM.hvf = 1540;	}
	else if(thrusterID == 5){ thruster_PWM.vhf = 1540;	}
	else if(thrusterID == 6){ thruster_PWM.vhb = 1540;	}
	else if(thrusterID == 7){ thruster_PWM.vvb = 1540;	}
	else if(thrusterID == 8){ thruster_PWM.vvf = 1540;	}

	teller_test_thruster = 0;
	skriv_thruster_PWM();
}

