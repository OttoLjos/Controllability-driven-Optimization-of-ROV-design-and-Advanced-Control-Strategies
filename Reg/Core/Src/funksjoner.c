/*
 * funksjoner.c
 *
 *  Created on: Jan 20, 2022
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
#include <verdier.h>


/* Variabler -------------------------------------------------- */



/* Funksjoner -------------------------------------------------- */

void behandle_paadrag(void){
//	paadrags_filter();


	memset(&totalbidrag, 0, sizeof(totalbidrag));
	add_contribution(&totalbidrag, &styrebidrag);
	if(controller_status){
		add_contribution(&totalbidrag, &ctrl_contribution);
	}
	if(flagg.spid){  add_contribution(&totalbidrag, &stampbidrag); }	// PID regulering av stampvinkelen
	if(flagg.rpid){  add_contribution(&totalbidrag, &rullbidrag);  }	// PID regulering av rullvinkelen
	if(flagg.hpid){  add_contribution(&totalbidrag, &hivbidrag);   }	// PID regulering av hivposisjon
	// if(flagg.gpid){  add_contribution(&totalbidrag, &girbidrag);   }	// PID regulering av girvinkel

	thruster_retning_korreksjon();
	skaler_paadrag();
	effekt_kontroll();
	thrust_filtering(&totalbidrag ,&active_thrust);

	thruster_PWM.hhf = (int16_t) (NEUTRAL + gain*active_thrust.hhf);
	thruster_PWM.hhb = (int16_t) (NEUTRAL + gain*active_thrust.hhb);
	thruster_PWM.hvb = (int16_t) (NEUTRAL + gain*active_thrust.hvb);
	thruster_PWM.hvf = (int16_t) (NEUTRAL + gain*active_thrust.hvf);
	thruster_PWM.vhf = (int16_t) (NEUTRAL + gain*active_thrust.vhf);
	thruster_PWM.vhb = (int16_t) (NEUTRAL + gain*active_thrust.vhb);
	thruster_PWM.vvb = (int16_t) (NEUTRAL + gain*active_thrust.vvb);
	thruster_PWM.vvf = (int16_t) (NEUTRAL + gain*active_thrust.vvf);

	oppdater_ledlys();
	skriv_thruster_PWM();


	thrusterdata.hhf = (int8_t) (gain*active_thrust.hhf/4);
	thrusterdata.hhb = (int8_t) (gain*active_thrust.hhb/4);
	thrusterdata.hvb = (int8_t) (gain*active_thrust.hvb/4);
	thrusterdata.hvf = (int8_t) (gain*active_thrust.hvf/4);
	thrusterdata.vhf = (int8_t) (gain*active_thrust.vhf/4);
	thrusterdata.vhb = (int8_t) (gain*active_thrust.vhb/4);
	thrusterdata.vvb = (int8_t) (gain*active_thrust.vvb/4);
	thrusterdata.vvf = (int8_t) (gain*active_thrust.vvf/4);

	ESC_status();

	

}

void oppdater_styrebidrag(void){
	// Regner ut argument og modulus av jag og svai
	styreretning = atan2(styremelding.svai, styremelding.jag);
	maks_paadrag = sqrt(styremelding.jag*styremelding.jag + styremelding.svai*styremelding.svai);

	// Utregning av pådragsnivå til hver horisontale thruster
	styrebidrag.hhf = cos(HR_kompass_rad.NW - styreretning)*maks_paadrag;
	styrebidrag.hhb = cos(HR_kompass_rad.SW - styreretning)*maks_paadrag;
	styrebidrag.hvb = - styrebidrag.hhf; 			// thrusteren er orientert 180 grader på hhf
	styrebidrag.hvf = - styrebidrag.hhb; 			// thrusteren er orientert 180 grader på hhb


	// Skalering i tilfelle gir-pådrag som kan akkumulere pådraget over "100%"
	if (styremelding.gir){
		if(flagg.gpid){flagg.gir_pause = 1;}
		flagg.gpid = 0;
		teller_reg_gir = 0;
		gir = styremelding.gir * param.demping.gir;
		styrebidrag.hhf -= gir;
		styrebidrag.hhb += gir;
		styrebidrag.hvb -= gir;
		styrebidrag.hvf += gir;
	}

	// Bidrag til vertikale thrustere i tilfelle hiv
	if (styremelding.hiv){
		ctrl_ref.z = model_states.z;
		if(flagg.hpid){flagg.hiv_pause = 1;}
		flagg.hpid = 0;
		teller_reg_hiv = 0;
		hiv = styremelding.hiv * param.demping.hiv;
		hiv = -hiv; // invertert styring av opp og ned
		styrebidrag.vhf = hiv;
		styrebidrag.vhb = hiv;
		styrebidrag.vvb = hiv;
		styrebidrag.vvf = hiv;
	}
	else{
		styrebidrag.vhf = 0;
		styrebidrag.vhb = 0;
		styrebidrag.vvb = 0;
		styrebidrag.vvf = 0;
	}

	// Pådrag for manuell stamp
	if(styremelding.stamp){
		if(flagg.spid){flagg.stamp_pause = 1;}
		flagg.spid = 0;
		teller_reg_stamp = 0;
		stamp = styremelding.stamp * param.demping.stamp;
		styrebidrag.vhf += stamp;
		styrebidrag.vhb -= stamp;
		styrebidrag.vvb -= stamp;
		styrebidrag.vvf += stamp;
	}

	// Pådrag for manuell stamp
	if(styremelding.rull){
		if(flagg.rpid){flagg.rull_pause = 1;}
		flagg.rpid = 0;
		teller_reg_rull = 0;
		rull = styremelding.rull * param.demping.rull;
		styrebidrag.vhf += rull;
		styrebidrag.vhb += rull;
		styrebidrag.vvb -= rull;
		styrebidrag.vvf -= rull;
	}

	// Pådragsbegresning fra topside
	if (styremelding.throttling){gain = (100-styremelding.throttling)/25;}
	else{						 gain = 4;}

}


void paadrags_filter(void){
	styrebidrag.hhf = styrefilter.hhf * param.demping.pfa + param.demping.pfb * styrebidrag.hhf;
	styrebidrag.hhb = styrefilter.hhb * param.demping.pfa + param.demping.pfb * styrebidrag.hhb;
	styrebidrag.hvb = styrefilter.hvb * param.demping.pfa + param.demping.pfb * styrebidrag.hvb;
	styrebidrag.hvf = styrefilter.hvf * param.demping.pfa + param.demping.pfb * styrebidrag.hvf;
	styrebidrag.vhf = styrefilter.vhf * param.demping.pfa + param.demping.pfb * styrebidrag.vhf;
	styrebidrag.vhb = styrefilter.vhb * param.demping.pfa + param.demping.pfb * styrebidrag.vhb;
	styrebidrag.vvb = styrefilter.vvb * param.demping.pfa + param.demping.pfb * styrebidrag.vvb;
	styrebidrag.vvf = styrefilter.vvf * param.demping.pfa + param.demping.pfb * styrebidrag.vvf;

	styrefilter.hhf = styrebidrag.hhf;
	styrefilter.hhb = styrebidrag.hhb;
	styrefilter.hvb = styrebidrag.hvb;
	styrefilter.hvf = styrebidrag.hvf;
	styrefilter.vhf = styrebidrag.vhf;
	styrefilter.vhb = styrebidrag.vhb;
	styrefilter.vvb = styrebidrag.vvb;
	styrefilter.vvf = styrebidrag.vvf;
}

void thrust_filtering(thruster_sett_float *thrust, thruster_sett_float *active){
	float step = 5.0;
	if ( fabs(thrust->hhf - active->hhf) >= step ){ active->hhf += copysign(step ,(thrust->hhf - active->hhf));}
	else{active->hhf = thrust->hhf;}
	if ( fabs(thrust->hhb - active->hhb) >= step ){ active->hhb += copysign(step ,(thrust->hhb - active->hhb));}
	else{active->hhb = thrust->hhb;}
	if ( fabs(thrust->hvb - active->hvb) >= step ){ active->hvb += copysign(step ,(thrust->hvb - active->hvb));}
	else{active->hvb = thrust->hvb;}
	if ( fabs(thrust->hvf - active->hvf) >= step ){ active->hvf += copysign(step ,(thrust->hvf - active->hvf));}
	else{active->hvf = thrust->hvf;}
	if ( fabs(thrust->vhf - active->vhf) >= step ){ active->vhf += copysign(step ,(thrust->vhf - active->vhf));}
	else{active->vhf = thrust->vhf;}
	if ( fabs(thrust->vhb - active->vhb) >= step ){ active->vhb += copysign(step ,(thrust->vhb - active->vhb));}
	else{active->vhb = thrust->vhb;}
	if ( fabs(thrust->vvb - active->vvb) >= step ){ active->vvb += copysign(step ,(thrust->vvb - active->vvb));}
	else{active->vvb = thrust->vvb;}
	if ( fabs(thrust->vvf - active->vvf) >= step ){ active->vvf += copysign(step ,(thrust->vvf - active->vvf));}
	else{active->vvf = thrust->vvf;}
}

void manipulator_styring(void){
	uint8_t COM = styremelding.manipulator; // 
	if ((COM & MAN_KLYPE) && controller_status) {
		test_thrustere = 0;

		if (active_step.SEQ_STATUS){
			memcpy(&ctrl_ref, &active_step.old_ref, sizeof(NED_eta));
			active_step.SEQ_STATUS = 0;
			memset(&step_counter, 0, sizeof(step_timers));
		}

		controller_status = 0;
		memset(&ctrl_ref, 0, sizeof(NED_eta));
		ctrl_ref.z = model_states.z;
		reset_pid_values(&pid_heave); reset_pid_values(&pid_roll); reset_pid_values(&pid_pitch);
		reset_lqr_values(&lqr_reg);
		reset_smc_values(&smc_heave); reset_smc_values(&smc_roll); reset_smc_values(&smc_pitch);
		memcpy(&thruster_PWM, &oppstart_PWM, sizeof(thruster_sett));
		skriv_thruster_PWM();
	}

	if (COM & MAN_ON){
	//		if(COM & MAN_RESET){		// reset funksjon (ikke impl)
	//			; //utføre reset
	//		}
		if(COM & MAN_KLYPE){			//  manipulatorkloen aktiv
			if(COM & MAN_KLYPE_LUKKE){	manipulator_PWM.klype = NEUTRAL-(param.demping.klype*400);}	// Lukke kloa
			else{						manipulator_PWM.klype = NEUTRAL+(param.demping.klype*400);}	// Åpne kloa
			manipulator_PWM.vri = 		NEUTRAL;
			manipulator_PWM.teleskop = 	NEUTRAL;
		}
		else if(COM & MAN_VRI){			// manipulator rotering aktiv
			if(COM & MAN_VRI_CCW){		manipulator_PWM.vri = NEUTRAL-(param.demping.vri*400);}		// kloa roterer mot klokka
			else{						manipulator_PWM.vri = NEUTRAL+(param.demping.vri*400);}		// kloa roterer med klokka
			manipulator_PWM.klype =		NEUTRAL;
			manipulator_PWM.teleskop = 	NEUTRAL;
		}
		else if(COM & MAN_TELESKOP){	// manipulator rotering aktiv
			if(COM & MAN_TELESKOP_UT){	manipulator_PWM.teleskop = NEUTRAL+(param.demping.teleskop*400);}		// manipulatoren går ut
			else{						manipulator_PWM.teleskop = NEUTRAL-(param.demping.teleskop*400);}		// manipulatoren går inn
			manipulator_PWM.klype =		NEUTRAL;
			manipulator_PWM.vri = 		NEUTRAL;
		}
		else{
			manipulator_PWM.klype =		NEUTRAL;
			manipulator_PWM.vri = 		NEUTRAL;
			manipulator_PWM.teleskop =	NEUTRAL;
			flagg.manipulator_null = 1;
		}
	}
	else{
		memcpy(&manipulator_PWM, &oppstart_PWM, sizeof(manipulator_sett));
	}
	skriv_manipulator_PWM();

}

void thruster_retning_korreksjon(void){  // nedskalering mtp thrusterkarakteristikk
	if (totalbidrag.hhf < -10){	totalbidrag.hhf = totalbidrag.hhf / (mk_modell.b + fabs(totalbidrag.hhf)*mk_modell.a);	}
	if (totalbidrag.hhb < -10){	totalbidrag.hhb = totalbidrag.hhb / (mk_modell.b + fabs(totalbidrag.hhb)*mk_modell.a);	}
	if (totalbidrag.hvb < -10){	totalbidrag.hvb = totalbidrag.hvb / (mk_modell.b + fabs(totalbidrag.hvb)*mk_modell.a);	}
	if (totalbidrag.hvf < -10){	totalbidrag.hvf = totalbidrag.hvf / (mk_modell.b + fabs(totalbidrag.hvf)*mk_modell.a);	}

	if (totalbidrag.vhf > 10){	totalbidrag.vhf = totalbidrag.vhf/ (mk_modell.b + totalbidrag.vhf*mk_modell.a);	}
	if (totalbidrag.vhb > 10){	totalbidrag.vhb = totalbidrag.vhb/ (mk_modell.b + totalbidrag.vhb*mk_modell.a);	}
	if (totalbidrag.vvb > 10){	totalbidrag.vvb = totalbidrag.vvb/ (mk_modell.b + totalbidrag.vvb*mk_modell.a);	}
	if (totalbidrag.vvf > 10){	totalbidrag.vvf = totalbidrag.vvf/ (mk_modell.b + totalbidrag.vvf*mk_modell.a);	}
}

void skaler_paadrag(void){
	t_skalering.ph = 1.0;
	t_skalering.pv = 1.0;
	t_skalering.hhf = 1.0;
	t_skalering.hhb = 1.0;
	t_skalering.hvb = 1.0;
	t_skalering.hvf = 1.0;
	t_skalering.vhf = 1.0;
	t_skalering.vhb = 1.0;
	t_skalering.vvb = 1.0;
	t_skalering.vvf = 1.0;


	if(fabs(totalbidrag.hhf)>100){	t_skalering.hhf = 100.0 / fabs(totalbidrag.hhf);	}
	if(fabs(totalbidrag.hhb)>100){	t_skalering.hhb = 100.0 / fabs(totalbidrag.hhb);	}
	if(fabs(totalbidrag.hvb)>100){	t_skalering.hvb = 100.0 / fabs(totalbidrag.hvb);	}
	if(fabs(totalbidrag.hvf)>100){	t_skalering.hvf = 100.0 / fabs(totalbidrag.hvf);	}
	if(fabs(totalbidrag.vhf)>100){	t_skalering.vhf = 100.0 / fabs(totalbidrag.vhf);	}
	if(fabs(totalbidrag.vhb)>100){	t_skalering.vhb = 100.0 / fabs(totalbidrag.vhb);	}
	if(fabs(totalbidrag.vvb)>100){	t_skalering.vvb = 100.0 / fabs(totalbidrag.vvb);	}
	if(fabs(totalbidrag.vvf)>100){	t_skalering.vvf = 100.0 / fabs(totalbidrag.vvf);	}

	if(t_skalering.ph > t_skalering.hhf){			t_skalering.ph = t_skalering.hhf;		}
	if(t_skalering.ph > t_skalering.hhb){			t_skalering.ph = t_skalering.hhb;		}
	if(t_skalering.ph > t_skalering.hvb){			t_skalering.ph = t_skalering.hvb;		}
	if(t_skalering.ph > t_skalering.hvf){			t_skalering.ph = t_skalering.hvf;		}
	if(t_skalering.pv > t_skalering.vhf){			t_skalering.pv = t_skalering.vhf;		}
	if(t_skalering.pv > t_skalering.vhb){			t_skalering.pv = t_skalering.vhb;		}
	if(t_skalering.pv > t_skalering.vvb){			t_skalering.pv = t_skalering.vvb;		}
	if(t_skalering.pv > t_skalering.vvf){			t_skalering.pv = t_skalering.vvf;		}

	if(t_skalering.ph < 1.0){
		totalbidrag.hhf *= t_skalering.ph;
		totalbidrag.hhb *= t_skalering.ph;
		totalbidrag.hvb *= t_skalering.ph;
		totalbidrag.hvf *= t_skalering.ph;
	}
	if(t_skalering.pv < 1.0){
		totalbidrag.vhf *= t_skalering.pv;
		totalbidrag.vhb *= t_skalering.pv;
		totalbidrag.vvb *= t_skalering.pv;
		totalbidrag.vvf *= t_skalering.pv;
	}
}

void skriv_thruster_PWM(void){		// Skriver ny pulsbredde for PWM-signalet til hver respektive Compare Register.
	TIM2->CCR1 = thruster_PWM.hhf;  		// HHF
	TIM2->CCR2 = thruster_PWM.hhb;  		// HHB
	TIM2->CCR3 = thruster_PWM.hvb;  		// HVB
	TIM2->CCR4 = thruster_PWM.hvf;  		// HVF
	TIM3->CCR1 = thruster_PWM.vhf;  		// VHF
	TIM3->CCR2 = thruster_PWM.vhb;  		// VHB
	TIM3->CCR3 = thruster_PWM.vvb;  		// VVB
	TIM3->CCR4 = thruster_PWM.vvf;  		// VVF
}

void skriv_manipulator_PWM(void){	// Skriver ny pulsbredde for PWM-signalet til hver respektive Compare Register.

	if(flagg.manipulator_null){
		manipulator_PWM.teleskop = NEUTRAL;
		manipulator_PWM.vri = NEUTRAL;
		manipulator_PWM.klype = NEUTRAL;
		flagg.manipulator_null = 0;
	}
	else{
		manipulator_PWM.teleskop = manipulator_filter.teleskop * param.demping.pfa + param.demping.pfb * manipulator_PWM.teleskop;
		manipulator_PWM.vri = manipulator_filter.vri * param.demping.pfa + param.demping.pfb * manipulator_PWM.vri;
		manipulator_PWM.klype = manipulator_filter.klype * param.demping.pfa + param.demping.pfb * manipulator_PWM.klype;
	}

	TIM4->CCR1 = manipulator_PWM.teleskop;	// Teleskop
	TIM4->CCR2 = manipulator_PWM.vri;	// Rull
	TIM4->CCR3 = manipulator_PWM.klype;	// Klype

	manipulator_filter.teleskop = manipulator_PWM.teleskop;
	manipulator_filter.vri = manipulator_PWM.vri;
	manipulator_filter.klype = manipulator_PWM.klype;
}

void effekt_kontroll(void){
	if(effekt_forbruk.thruster_12v > 10000){
		gain *= 0.8;
	}
}

void ESC_status(void){
	sum_paadrag = abs(thrusterdata.hhf) + abs(thrusterdata.hhb) + abs(thrusterdata.hvb) + abs(thrusterdata.hvf)
			+ abs(thrusterdata.vhf) + abs(thrusterdata.vhb) + abs(thrusterdata.vvb) + abs(thrusterdata.vvf);
	if((sum_paadrag > 300) && (effekt_forbruk.thruster_12v < 50) && (effekt_forbruk.thruster_12v != 0)){esc_status_avprell++;}
	else{esc_status_avprell = 0;}
	if(esc_status_avprell >= 24){
		esc_status_avprell = 0;
		esc_reset = 1;
	}
	if(esc_status_avprell < 0){esc_status_avprell = 0;}
}

void sjekk_brytertrykk(void){
	if (GPIOA->IDR & GPIO_PIN_0 ) { 			// Sjekker om bryteren er trykket inn
       if(!forrige_bryterstatus) { 				// Var bryteren trykket inn sist kontrollsjekk
    	   forrige_bryterstatus = 1;
    	   brytertrykk = 1;      				// Nytt brytertrykk registrert
       }
	}
    else {                 						// Hvis bryteren ikke er trykket inn
    	 forrige_bryterstatus = 0; 				// Bryterstatus settes til 0.
    }
}


void oppdater_ledlys(void){
	if((thruster_PWM.hhf > 1520) || ((thruster_PWM.hhf < 1480) && led_status)){
			GPIOE->ODR |= LED_HHF;	}
	else{	GPIOE->ODR &= ~LED_HHF;	}
	if((thruster_PWM.hhb > 1520) || ((thruster_PWM.hhb < 1480) && led_status)){
			GPIOE->ODR |= LED_HHB;	}
	else{	GPIOE->ODR &= ~LED_HHB;	}
	if((thruster_PWM.hvb > 1520) || ((thruster_PWM.hvb < 1480) && led_status)){
			GPIOE->ODR |= LED_HVB;	}
	else{	GPIOE->ODR &= ~LED_HVB;	}
	if((thruster_PWM.hvf > 1520) || ((thruster_PWM.hvf < 1480) && led_status)){
			GPIOE->ODR |= LED_HVF;	}
	else{	GPIOE->ODR &= ~LED_HVF;	}
	if((thruster_PWM.vhf > 1520) || ((thruster_PWM.vhf < 1480) && led_status)){
			GPIOE->ODR |= LED_VHF;	}
	else{	GPIOE->ODR &= ~LED_VHF;	}
	if((thruster_PWM.vhb > 1520) || ((thruster_PWM.vhb < 1480) && led_status)){
			GPIOE->ODR |= LED_VHB;	}
	else{	GPIOE->ODR &= ~LED_VHB;	}
	if((thruster_PWM.vvb > 1520) || ((thruster_PWM.vvb < 1480) && led_status)){
			GPIOE->ODR |= LED_VVB;	}
	else{	GPIOE->ODR &= ~LED_VVB;	}
	if((thruster_PWM.vvf > 1520) || ((thruster_PWM.vvf < 1480) && led_status)){
			GPIOE->ODR |= LED_VVF;	}
	else{	GPIOE->ODR &= ~LED_VVF;	}
}

float kg_til_paadrag(float kg_thrust){
	float kg_float = (float) (fabs(kg_thrust));
	float paadrag = (24.44*kg_float*kg_float+39.94*kg_float + 0.0)/(kg_float+0.21); // + 0.52
	if(kg_thrust<0.0){
		return -paadrag;	}
	else {
		return paadrag;		}
}

void kombiner_styring(void){
	if((styremelding.jag + autonomstyring.jag) > 100){ styremelding.jag = 100;}
	else if((styremelding.jag + autonomstyring.jag) < -100){ styremelding.jag = -100;}
	else{styremelding.jag += autonomstyring.jag;}

	if((styremelding.svai + autonomstyring.svai) > 100){ styremelding.svai = 100;}
	else if((styremelding.svai + autonomstyring.svai) < -100){ styremelding.svai = -100;}
	else{styremelding.svai += autonomstyring.svai;}

	if((styremelding.hiv + autonomstyring.hiv) > 100){ styremelding.hiv = 100;}
	else if((styremelding.hiv + autonomstyring.hiv) < -100){ styremelding.hiv = -100;}
	else{styremelding.hiv += autonomstyring.hiv;}

	if((styremelding.gir + autonomstyring.gir) > 100){ styremelding.gir = 100;}
	else if((styremelding.gir + autonomstyring.gir) < -100){ styremelding.gir = -100;}
	else{styremelding.gir += autonomstyring.gir;}

}

/*
%      f(x) = (p1*x^2 + p2*x + p3) / (x + q1)
% Coefficients (with 95% confidence bounds):
%        p1 =       24.44  (17.37, 31.51)
%        p2 =       39.94  (11.68, 68.2)
%        p3 =      0.5184  (-1.446, 2.482)
%        q1 =      0.2079  (-0.1737, 0.5895)
*/


void add_contribution(thruster_sett_float *result, thruster_sett_float *added) {
    result->hhf += added->hhf;
    result->hhb += added->hhb;
    result->hvb += added->hvb;
    result->hvf += added->hvf;
    result->vhf += added->vhf;
    result->vhb += added->vhb;
    result->vvb += added->vvb;
    result->vvf += added->vvf;

	memset(added, 0, sizeof(thruster_sett_float));
}

void NED2BFF(NED_eta *NED, NED_eta *BFF, const state_struct *state){
	// relies on x,y and psi in NED are 0
	BFF->x = -NED->z *sin(state->theta);
	BFF->y = NED->z * cos(state->theta) * sin(state->phi);
	BFF->z = NED->z * cos(state->phi) * cos(state->theta);
	BFF->phi = NED->phi;
	BFF->theta = NED->theta * cos(state->phi);
	BFF->psi = -NED->theta * sin(state->phi);

	memset(NED, 0, sizeof(NED_eta));
}

void BFF2ThrustPercent(const NED_eta *BFF, thruster_sett_float *thrust) {
//	float x[6];
//	float x[6] = {BFF->x,BFF->y,BFF->z,BFF->phi,BFF->theta,BFF->psi};
//	memcpy(&x, &BFF, sizeof(NED_eta));
	float result;
	float result_in_percent[8];
	for (int i = 0; i < Tmat.rows; i++) {
		result = Tmat.data[i][0]*BFF->x + Tmat.data[i][1]*BFF->y + Tmat.data[i][2]*BFF->z + Tmat.data[i][3]*BFF->phi + Tmat.data[i][4]*BFF->theta + Tmat.data[i][5]*BFF->psi;
//		result = 0.0;
//		for (int j = 0; j < Tmat.cols; j++) {
//			result += Tmat.data[i][j] * x[j];
//		}
		// insert result as the i-th element in &thrust
		result_in_percent[i] = thrust2percent(result);
//		*((float *)thrust + i) = thrust2percent(result);
	}
	thrust->hhf = result_in_percent[0];
	thrust->hhb = result_in_percent[1];
	thrust->hvb = result_in_percent[2];
	thrust->hvf = result_in_percent[3];
	thrust->vhf = result_in_percent[4];
	thrust->vhb = result_in_percent[5];
	thrust->vvb = result_in_percent[6];
	thrust->vvf = result_in_percent[7];
//	if((thrust->hhf == 0)&&(thrust->hhb == 0)&&(thrust->hvb == 0)&&(thrust->hvf == 0)&&(thrust->vhf == 0)&&(thrust->vhb == 0)&&(thrust->vvb == 0)&&(thrust->vvf == 0)){
//		test_thrustere = 1;
//	}
//	else {
//		test_thrustere = 0;
//	}
}

float thrust2percent(float thrust){
	float ftrust = fabs(thrust)/9.81;
	float percent = (24.44*ftrust*ftrust+39.94*ftrust + 0.0)/(ftrust+0.21); // + 0.52
	return copysign(percent, thrust);
}

// New Control Law
//-z_d * (cos(theta)^2 * (6.11 * abs(z_d) * cos(phi)^2 - sin(phi)^2) - 0.33 * sin(theta)^2 + 1)
// F_z =  -z_d*(6.11*abs(z_d)*(cos(phi)^2*cos(theta)^2)-sin(phi)^2-0.33*sin(theta)^2+sin(phi)^2*sin(theta)^2 +1)
// F_phi  = -(sin(phi)*(3.63-0.66*cos(theta)^2+theta_d*cos(phi)*sin(theta)*(2.18+1.3*abs(theta_d)))/cos(theta) -2.66*phi_d*abs(phi_d)
// F_theta = -cos(phi)*(3.1*sin(theta)+theta_d*cos(phi)*(2.16+1.27*abs(theta_d)))
float calc_acceleration_smc(smc_struct *smc, state_struct *states){
	float result = 0.0;
	float s4 = sin(states->phi);
	float s5 = sin(states->theta);
	float c4 = cos(states->phi);
	float c5 = cos(states->theta);
	switch(smc->reg_state){
	case 1: // x
//		result = 0.0;
		break;
	case 2: // y
//		result = 0.0;
		break;
	case 3:  // z
		result = - states->w * (c5*c5 * (6.11 * fabs(states->w)*c4*c4 - s4*s4) - 0.33*s5*s5 + 1);
		break;
	case 4: // phi
		result = - (s4*(3.63 - 0.66*c5*c5 + states->q*c4*s5*(2.18+1.3*fabs(states->q))) / c5) - 2.66*states->p*fabs(states->p);
		break;
	case 5: // theta
		result = -c4*(3.1*s5 + states->q*c4*(2.16+1.27*fabs(states->q)));
		break;
	case 6: // psi
//		result = 0.0;
		break;
	}

	return result;
}

void step_machine(const step_test *step, float *c_ref, float old_ref){
	float step_ref = 0.0;
	if (step_counter.seq_time < step->step_start){

		return;
	}

	if (step->sine_trajectory){
		step_ref = old_ref + step->amp1*sin((step_counter.seq_time-step->step_start)*step->freq1*0.001) + step->amp2*sin((step_counter.seq_time-step->step_start*0.001)*step->freq2);
	}
	else {
		step_ref = old_ref + step->step_size;
	}

	float delta_ref = 0.0;
	if (step_counter.seq_time < step->step_stop){
		if (step_counter.incr_timer >= 10){
			delta_ref = step_ref - *c_ref;
			if(fabs(delta_ref) > step->ref_rate){ 	*c_ref += copysign(step->ref_rate, delta_ref);	}
			else{									*c_ref += delta_ref;	}
		}
	}
	else {
		if (step_counter.incr_timer >= 10){
			delta_ref = old_ref - *c_ref ;
			if(fabs(delta_ref) > step->ref_rate){ 	*c_ref += copysign(step->ref_rate, delta_ref);	}
			else{									*c_ref += delta_ref;	}
		}
	}
	return;
}
//-Tau3*sin(theta)
//Tau3*cos(theta)*sin(phi)
//Tau3*cos(phi)*cos(theta)
//               Tau4
//      Tau5*cos(phi)
//     -Tau5*sin(phi)
