#include "timers.h"
#include <MKL25Z4.h>
#include "HBLED.h"
#include "GPIO_defs.h"
#include "debug.h"
extern void Update_Set_Current(void);

void PWM_Init(TPM_Type * TPM, uint8_t channel_num, uint16_t period, uint16_t duty)
{
	//turn on clock to TPM 
	switch ((int) TPM) {
		case (int) TPM0:
			SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
			break;
		case (int) TPM1:
			SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
			break;
		case (int) TPM2:
			SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
			break;
		default:
			break;
	}
	//set clock source for tpm
	SIM->SOPT2 |= (SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK);

	//load the counter and mod
	TPM->MOD = period;
		
	//set channel to center-aligned low-true PWM
	TPM->CONTROLS[channel_num].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;

	//set TPM to up-down and divide by 1 prescaler and clock mode
	TPM->SC = (TPM_SC_CPWMS_MASK | TPM_SC_PS(0));
	
	//set trigger mode and keep running in debug mode
	TPM->CONF |= TPM_CONF_TRGSEL(0xA) | TPM_CONF_DBGMODE(3);

	// Set initial duty cycle
	TPM->CONTROLS[channel_num].CnV = duty;
		
#if USE_TPM0_INTERRUPT // if using interrupt 
	TPM0->SC |= TPM_SC_TOIE_MASK;

	// Configure NVIC 
	NVIC_SetPriority(TPM0_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(TPM0_IRQn); 
	NVIC_EnableIRQ(TPM0_IRQn);	
#endif

	// Start the timer counting
	TPM->SC |= TPM_SC_CMOD(1);
}

void PWM_Set_Value(TPM_Type * TPM, uint8_t channel_num, uint16_t value) {
	TPM->CONTROLS[channel_num].CnV = value;
}
extern void Control_HBLED(void);

void TPM0_IRQHandler() {
	static uint32_t control_divider = SW_CTL_FREQ_DIV_FACTOR;
	
	FPTB->PSOR = MASK(DBG_IRQ_TPM);
	//clear pending IRQ flag
	TPM0->SC |= TPM_SC_TOF_MASK; 

	control_divider--;
	if (control_divider == 0) {
		control_divider = SW_CTL_FREQ_DIV_FACTOR;
		// Start conversion
		ADC0->SC1[0] = ADC_SC1_AIEN(1) | ADC_SENSE_CHANNEL;
		
	#if USE_ADC_INTERRUPT
		// can return immediately
	#else
		// Call control function, which will wait for ADC coco
		Control_HBLED();
	#endif
	}
	FPTB->PCOR = MASK(DBG_IRQ_TPM);
}


void Init_PIT(unsigned period) {
	// Enable clock to PIT module
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	// Disable clocks for configuration, 
	PIT->MCR |= PIT_MCR_MDIS_MASK;
	
	// Initialize PIT0 to count down from argument 
	PIT->CHANNEL[0].LDVAL = PIT_LDVAL_TSV(period);

	// No chaining
	PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_CHN_MASK;

	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;

#if 1 // generate interrupts
	NVIC_SetPriority(PIT_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(PIT_IRQn); 
	NVIC_EnableIRQ(PIT_IRQn);	
#endif
}

void Start_PIT(void) {
// Enable counter
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
	PIT->MCR &= ~PIT_MCR_MDIS_MASK;

}

void Stop_PIT(void) {
// Disable counter
	PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK;
}

void PIT_IRQHandler() {
	FPTB->PSOR = MASK(DBG_IRQ_PIT);
	// check to see which channel triggered interrupt 
	if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {
		// clear status flag for timer channel 0
		PIT->CHANNEL[0].TFLG &= PIT_TFLG_TIF_MASK;
		// Do ISR work
		Update_Set_Current();
	} else if (PIT->CHANNEL[1].TFLG & PIT_TFLG_TIF_MASK) {
		// clear status flag for timer channel 1
		PIT->CHANNEL[1].TFLG &= PIT_TFLG_TIF_MASK;
	} 
	FPTB->PCOR = MASK(DBG_IRQ_PIT);
}

// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
