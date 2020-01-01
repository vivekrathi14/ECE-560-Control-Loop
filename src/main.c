/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include <stdint.h>

#include "gpio_defs.h"
#include "debug.h"

#include "timers.h"
#include "delay.h"
#include "LEDS.h"

#include "HBLED.h"
#include "FX.h"

volatile int g_enable_control=1;
volatile int g_set_current=0;
volatile int g_flash_current_mA = FLASH_CURRENT_MA;

volatile int measured_current;
volatile int period = PWM_PERIOD;
volatile int16_t g_duty_cycle=0;  // global to give debugger access
volatile int error;
enum {OpenLoop, BangBang, Incremental, Proportional, PID, PID_FX}
	control_mode=DEF_CONTROL_MODE;

int32_t pGain_8 = PGAIN_8; // proportional gain numerator scaled by 2^8
volatile int g_enable_flash=1;

typedef struct {
	float dState; // Last position input
	float iState; // Integrator state
	float iMax, iMin; // Maximum and minimum allowable integrator state
	float iGain, // integral gain
				pGain, // proportional gain
				dGain; // derivative gain
} SPid;

typedef struct {
	FX16_16 dState; // Last position input
	FX16_16 iState; // Integrator state
	FX16_16 iMax, iMin; // Maximum and minimum allowable integrator state
	FX16_16 iGain, // integral gain
					pGain, // proportional gain
					dGain; // derivative gain
} SPidFX;


SPid plantPID = {0, // dState
	0, // iState
	LIM_DUTY_CYCLE, // iMax
	-LIM_DUTY_CYCLE, // iMin
	I_GAIN_FL, // iGain
	P_GAIN_FL, // pGain
	D_GAIN_FL  // dGain
};

SPidFX plantPID_FX = {FL_TO_FX(0), // dState
	FL_TO_FX(0), // iState
	FL_TO_FX(LIM_DUTY_CYCLE), // iMax
	FL_TO_FX(-LIM_DUTY_CYCLE), // iMin
	I_GAIN_FX, // iGain
	P_GAIN_FX, // pGain
	D_GAIN_FX  // dGain
};


float UpdatePID(SPid * pid, float error, float position){
	float pTerm, dTerm, iTerm;

	// calculate the proportional term
	pTerm = pid->pGain * error;
	// calculate the integral state with appropriate limiting
	pid->iState += error;
	if (pid->iState > pid->iMax) 
		pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) 
		pid->iState = pid->iMin;
	iTerm = pid->iGain * pid->iState; // calculate the integral term
	dTerm = pid->dGain * (position - pid->dState);
	pid->dState = position;

	return pTerm + iTerm - dTerm;
}

FX16_16 UpdatePID_FX(SPidFX * pid, FX16_16 error_FX, FX16_16 position_FX){
	FX16_16 pTerm, dTerm, iTerm, diff, ret_val;

	// calculate the proportional term
	pTerm = Multiply_FX(pid->pGain, error_FX);

	// calculate the integral state with appropriate limiting
	pid->iState = Add_FX(pid->iState, error_FX);
	if (pid->iState > pid->iMax) 
		pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) 
		pid->iState = pid->iMin;
	
	iTerm = Multiply_FX(pid->iGain, pid->iState); // calculate the integral term
	diff = Subtract_FX(position_FX, pid->dState);
	dTerm = Multiply_FX(pid->dGain, diff);
	pid->dState = position_FX;

	ret_val = Add_FX(pTerm, iTerm);
	ret_val = Subtract_FX(ret_val, dTerm);
	return ret_val;
}

void Control_HBLED(void) {
	uint16_t res;
	FX16_16 change_FX, error_FX;
	
	FPTB->PSOR = MASK(DBG_CONTROLLER);
	
#if USE_ADC_INTERRUPT
	// already completed conversion, so don't wait
#else
	while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK))
		; // wait until end of conversion
#endif
	res = ADC0->R[0];

	measured_current = (res*1500)>>16; // Extra Credit: Make this code work: V_REF_MV*MA_SCALING_FACTOR)/(ADC_FULL_SCALE*R_SENSE)

	switch (control_mode) {
		case OpenLoop:
				// don't do anything!
			break;
		case BangBang:
			if (measured_current < g_set_current)
				g_duty_cycle = LIM_DUTY_CYCLE;
			else
				g_duty_cycle = 0;
			break;
		case Incremental:
			if (measured_current < g_set_current)
				g_duty_cycle += INC_STEP;
			else
				g_duty_cycle -= INC_STEP;
			break;
		case Proportional:
			g_duty_cycle += (pGain_8*(g_set_current - measured_current))/256; //  - 1;
		break;
		case PID:
			g_duty_cycle += UpdatePID(&plantPID, g_set_current - measured_current, measured_current);
			break;
		case PID_FX:
			error_FX = INT_TO_FX(g_set_current - measured_current);
			change_FX = UpdatePID_FX(&plantPID_FX, error_FX, INT_TO_FX(measured_current));
			g_duty_cycle += FX_TO_INT(change_FX);
		break;
		default:
			break;
	}
	
	// Update PWM controller with duty cycle
	if (g_duty_cycle < 0)
		g_duty_cycle = 0;
	else if (g_duty_cycle > LIM_DUTY_CYCLE)
		g_duty_cycle = LIM_DUTY_CYCLE;
	PWM_Set_Value(TPM0, PWM_HBLED_CHANNEL, g_duty_cycle);
	FPTB->PCOR = MASK(DBG_CONTROLLER);
}

#if USE_ADC_INTERRUPT
void ADC0_IRQHandler() {
	FPTB->PSOR = MASK(DBG_IRQ_ADC);
	Control_HBLED();
	FPTB->PCOR = MASK(DBG_IRQ_ADC);
}
#endif

void Set_DAC(unsigned int code) {
	// Force 16-bit write to DAC
	uint16_t * dac0dat = (uint16_t *)&(DAC0->DAT[0].DATL);
	*dac0dat = (uint16_t) code;
}

void Set_DAC_mA(unsigned int current) {
	unsigned int code = MA_TO_DAC_CODE(current);
	// Force 16-bit write to DAC
	uint16_t * dac0dat = (uint16_t *)&(DAC0->DAT[0].DATL);
	*dac0dat = (uint16_t) code;
}

void Init_DAC(void) {
  // Enable clock to DAC and Port E
	SIM->SCGC6 |= SIM_SCGC6_DAC0_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// Select analog for pin
	PORTE->PCR[DAC_POS] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[DAC_POS] |= PORT_PCR_MUX(0);	
		
	// Disable buffer mode
	DAC0->C1 = 0;
	DAC0->C2 = 0;
	
	// Enable DAC, select VDDA as reference voltage
	DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK;
	Set_DAC(0);
}

void Init_ADC(void) {
	// Configure ADC to read Ch 8 (FPTB 0)
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
	ADC0->CFG1 = 0x0C; // 16 bit
	//	ADC0->CFG2 = ADC_CFG2_ADLSTS(3);
	ADC0->SC2 = ADC_SC2_REFSEL(0);

#if USE_ADC_HW_TRIGGER
	// Enable hardware triggering of ADC
	ADC0->SC2 |= ADC_SC2_ADTRG(1);
	// Select triggering by TPM0 Overflow
	SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8) | SIM_SOPT7_ADC0ALTTRGEN_MASK;
	// Select input channel 
	ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
	ADC0->SC1[0] |= ADC_SC1_ADCH(ADC_SENSE_CHANNEL);
#endif

#if USE_ADC_INTERRUPT 
	// enable ADC interrupt
	ADC0->SC1[0] |= ADC_SC1_AIEN(1);

	// Configure NVIC for ADC interrupt
	NVIC_SetPriority(ADC0_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(ADC0_IRQn); 
	NVIC_EnableIRQ(ADC0_IRQn);	
#endif
}

void Update_Set_Current(void) {
	static int delay=FLASH_PERIOD;

	if (g_enable_flash){
		delay--;
#if ENABLE_PRE_FLASH
		if (delay==1) {
			FPTB->PSOR = MASK(DBG_LED_ON);
			Set_DAC_mA(g_flash_current_mA/4);
			g_set_current = g_flash_current_mA/4;
			} else 
#endif
		if (delay==0) {
			delay=FLASH_PERIOD;
			FPTB->PSOR = MASK(DBG_LED_ON);
			Set_DAC_mA(g_flash_current_mA);
			g_set_current = g_flash_current_mA;
		} else {
			FPTB->PCOR = MASK(DBG_LED_ON);
			Set_DAC_mA(0);
			g_set_current = 0;
		}
	}
}
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	
	Init_Debug_Signals();
	Init_DAC();
	Init_ADC();
	Init_RGB_LEDs();

	
	// Configure driver for buck converter
	// Set up PTE31 to use for SMPS with TPM0 Ch 4
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[31]  &= PORT_PCR_MUX(7);
	PORTE->PCR[31]  |= PORT_PCR_MUX(3);
	PWM_Init(TPM0, PWM_HBLED_CHANNEL, PWM_PERIOD, 40);
	
	Control_RGB_LEDs(1,1,0);
	Delay(100);
	Control_RGB_LEDs(0,0,1);	
	
	// Configure flash timer
	Init_PIT(123456);
	Start_PIT();	
	
	while (1) {
#if USE_ASYNC_SAMPLING
		// Start conversion
		ADC0->SC1[0] = ADC_SC1_AIEN(1) | ADC_SENSE_CHANNEL;
		Control_HBLED(); // Blocks until ADC done, then updates control
#endif
		// else do nothing but wait for interrupts
	}
}



