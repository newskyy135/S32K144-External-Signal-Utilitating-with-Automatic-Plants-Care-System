#include "device_registers.h"           // Device header
#include "clocks_and_modes.h"
#include "ADC.h"
#include <math.h>


char data=0;
void LPIT0_init (uint32_t delay);

int idle_counter = 0;           /* main loop idle counter */
int lpit0_ch0_flag_counter = 0; /* LPIT0 chan 0 timeout counter */

unsigned int FND_DATA[10]={0x3F, 0x06,0x5b,0x4F,0x66,0x6D,0x7C,0x07,0x7F,0x67}; 
unsigned int FND_SEL[5]={0x2000,0x4000,0x8000,0x10000};
unsigned int j=0; /*FND select pin index */
unsigned int num=0,num0=0,num1=0,num2=0,num3=0;
unsigned int Dtime = 0; /* Delay Time Setting Variable*/
unsigned int External_PIN=0; /* External_PIN:SW External input Assignment */

void PORT_init (void)
{
	PCC->PCCn[PCC_PORTD_INDEX ]|=PCC_PCCn_CGC_MASK;
	PCC->PCCn[PCC_PORTE_INDEX ]|=PCC_PCCn_CGC_MASK;
	PCC->PCCn[PCC_PORTC_INDEX]|=PCC_PCCn_CGC_MASK;   /* Enable clock for PORTC */ 
	
	PTD->PDDR |= 1 << 0 | 1 << 1 | 1 << 2 | 1 << 3 | 1 << 4 | 1 << 5 | 1 << 6 | 1 << 7 | 1 << 11;
	PTE->PDDR |= 1<<13|1<<14|1<<15|1<<16 ;

	PTC->PDDR &= ~(1<<12); /* Port C12 Port Input set, value '0'*/ 
	PTC->PDDR &= ~(1<<13); /* Port C13 Port Input set, value '0'*/ 
	
	PORTC->PCR[12] |= PORT_PCR_MUX(1); // Port C12 mux = GPIO 
	PORTC->PCR[12] |=(10<<16); // Port C12 IRQC : interrupt on Falling-edge
	// PORTC_13 pin GPIO and Falling-edge Set 
	PORTC->PCR[13] |= PORT_PCR_MUX(1); // Port C13 mux = GPIO 
	PORTC->PCR[13] |=(10<<16); // Port C13 IRQC : interrupt on Falling-edge
	
	PORTD->PCR[0] = PORT_PCR_MUX(1);
	PORTD->PCR[1] = PORT_PCR_MUX(1);
	PORTD->PCR[2] = PORT_PCR_MUX(1);
	PORTD->PCR[3] = PORT_PCR_MUX(1);
	PORTD->PCR[4] = PORT_PCR_MUX(1);
	PORTD->PCR[5] = PORT_PCR_MUX(1);
	PORTD->PCR[6] = PORT_PCR_MUX(1);
	PORTD->PCR[7] = PORT_PCR_MUX(1);
	PORTD->PCR[11] = PORT_PCR_MUX(1);
	
	PORTE->PCR[13] = PORT_PCR_MUX(1);
	PORTE->PCR[14] = PORT_PCR_MUX(1);
	PORTE->PCR[15] = PORT_PCR_MUX(1);
	PORTE->PCR[16] = PORT_PCR_MUX(1);
}

void NVIC_init_IRQs(void)
{ 
	S32_NVIC->ICPR[1] |= 1<<(61%32); // Clear any pending IRQ61 
	S32_NVIC->ISER[1] |= 1<<(61%32); // Enable IRQ61 
	S32_NVIC->IP[61] =0xB; //Priority 11 of 15 
}

void WDOG_disable (void){
  WDOG->CNT=0xD928C520;     /* Unlock watchdog */
  WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value */
  WDOG->CS = 0x00002100;    /* Disable watchdog */
}

void delay_ms (uint32_t ms){
	 LPIT0_init(ms);           /* Initialize PIT0 for 1 second timeout  */
   while (0 == (LPIT0->MSR & LPIT_MSR_TIF0_MASK)) {} /* Wait for LPIT0 CH0 Flag */
               lpit0_ch0_flag_counter++;         /* Increment LPIT0 timeout counter */
               LPIT0->MSR |= LPIT_MSR_TIF0_MASK; /* Clear LPIT0 timer flag 0 */
}

void LPIT0_init (uint32_t delay) 
{ 
	uint32_t timeout; /*!
 	* LPIT Clocking: 
	* ============================== 
	*/ 
	PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);    /* Clock Src = 6 (SPLL2_DIV2_CLK)*/ 
	PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clk to LPIT0 regs */
	/*! 
	* LPIT Initialization: 
	*/ 
	LPIT0->MCR |= LPIT_MCR_M_CEN_MASK;  /* DBG_EN-0: Timer chans stop in Debug mode */ 
		/* DOZE_EN=0: Timer chans are stopped in DOZE mode */ 
		/* SW_RST=0: SW reset does not reset timer chans, regs */ 
		/* M_CEN=1: enable module clk (allows writing other LPIT0 regs) */

	timeout=delay* 40000; 
	LPIT0->TMR[0].TVAL = timeout;      /* Chan 0 Timeout period: 40M clocks */ 
	LPIT0->TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK; 
		/* T_EN=1: Timer channel is enabled */
		/* CHAIN=0: channel chaining is disabled */ 
		/* MODE=0: 32 periodic counter mode */ 
		/* TSOT=0: Timer decrements immediately based on restart */ 
		/* TSOI=0: Timer does not stop after timeout */ 
		/* TROT=0 Timer will not reload on trigger */ 
		/* TRG_SRC=0: External trigger soruce */ 
		/* TRG_SEL=0: Timer chan 0 trigger source is selected*/
}


void FND_Control(int num) {
	int j = 0;
	int num0, num1, num2, num3;
	
		num3=(num/1000)%10;
		num2=(num/100)%10;
		num1=(num/10)%10;
		num0=(num/1)%10;
	
		PTE->PSOR = FND_SEL[j];
		PTD->PCOR = FND_DATA[num3];
		delay_ms(Dtime/300);
		PTD->PSOR = 0xff;
		PTE->PCOR = 0xfffff;
		j++;
	
		PTE->PSOR = FND_SEL[j];
		PTD->PCOR = FND_DATA[num2];
		delay_ms(Dtime/300);
		PTD->PSOR = 0xff;
		PTE->PCOR = 0xfffff;
		j++;
		
		PTE->PSOR = FND_SEL[j];
		PTD->PCOR = FND_DATA[num1];
		delay_ms(Dtime/300);
		PTD->PSOR = 0xff;
		PTE->PCOR = 0xfffff;
		j++;
		
		PTE->PSOR = FND_SEL[j];
		PTD->PCOR = FND_DATA[num0];
		delay_ms(Dtime/300);
		PTD->PSOR = 0xff;
		PTE->PCOR = 0xfffff;
		j=0;
}

int main(void)
{
	uint32_t adcResultInMv = 0;
	uint32_t adcResultInMv_pre = 0;
	
	WDOG_disable();        /* Disable WDGO*/
	PORT_init();
  SOSC_init_8MHz();      /* Initialize system oscilator for 8 MHz xtal */
  SPLL_init_160MHz();    /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
  NormalRUNmode_80MHz(); /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */
	ADC_init();
	NVIC_init_IRQs(); /*Interrupt Pending, Endable, Priority Set*/
	Dtime = 500; // Delay Reset Value
	

	/*PTD->PCOR &= 0xFFF;
	PTD->PSOR |=0x800;
	delay_ms(Dtime*10000);*/
	
	/*PTB->PCOR &= 0xFFFFF;
	PTB->PSOR |=0x10000;
	delay_ms(Dtime*10000);*/
	
	/*for (;;) {
		convertAdcChan(0);

		while(adc_complete()==0){}

		adcResultInMv = read_adc_chx();

		if (abs(adcResultInMv_pre - adcResultInMv) > 3) {
			FND_Control(adcResultInMv);
			adcResultInMv_pre = adcResultInMv;
		}
		else
			FND_Control(adcResultInMv_pre);
		
		PTD->PCOR &= 0xFFF;
		PTD->PSOR |=0x800;
		delay_ms(Dtime*1000);
		
		}*/
		
		/*for (;;) {
		convertAdcChan(0);

		while(adc_complete()==0){}

		adcResultInMv = read_adc_chx();

		if (abs(adcResultInMv_pre - adcResultInMv) > 10)
		{
			FND_Control(adcResultInMv);
			adcResultInMv_pre = adcResultInMv;
			
			if (adcResultInMv > 3800)
			{
				PTB->PCOR |= 0xFFFFF;
				PTB->PSOR |=0x10000;
				delay_ms(Dtime/100);
			}
			else
				PTB->PCOR |= 0xFFFFF;
		}
		else
		{
			FND_Control(adcResultInMv_pre);
			
			if (adcResultInMv_pre > 3800)
			{
				PTB->PCOR |= 0xFFFFF;
				PTB->PSOR |=0x10000;
				delay_ms(Dtime/100);
			}
			else
				PTB->PCOR |= 0xFFFFF;
		}
	}*/
		
	for (;;) {
		convertAdcChan(0);

		while(adc_complete()==0){}

		adcResultInMv = read_adc_chx();

		if (adcResultInMv > 3800)
			{
				PTD->PCOR &= 0xFFF;
				PTD->PSOR |=0x800;
				delay_ms(Dtime*10);
			}
		else{
				PTD->PCOR |= 0xFFF;
				delay_ms(Dtime*10);
		}
	}
	
	}
	

void PORTC_IRQHandler(void){
	PORTC->PCR[12] &= ~(0x01000000); // Port Control Register ISF bit '0' set
	
	uint32_t adcResultInMv = 0;
	uint32_t adcResultInMv_pre = 0;

	ADC_init();
			
	adcResultInMv = read_adc_chx();
		
	for (int i = 0;i<1000;i++) {
		convertAdcChan(0);

		while(adc_complete()==0){}

		adcResultInMv = read_adc_chx();

		if (abs(adcResultInMv_pre - adcResultInMv) > 10)
		{
			FND_Control(adcResultInMv);
			adcResultInMv_pre = adcResultInMv;
			
		}
		else
			FND_Control(adcResultInMv_pre);
		
	}
	
			PORTC->PCR[12] |= 0x01000000;
}