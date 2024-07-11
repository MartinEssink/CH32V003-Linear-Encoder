#include "ch32v003fun.h"
#include <stdio.h>

#define N_SIN 120
#define N_ADC 60
#define N_SIN_OFFSET 20

int32_t quad_values[4];
volatile uint8_t *adc_buffer_pt;
int32_t x,y,sxy,dxy;
int32_t pos = 0;
int8_t angle;


uint8_t sindata[] = {
128,134,141,147,154,160,167,173,179,185, //0
191,197,202,208,213,218,222,227,231,234, //30
238,241,244,247,249,251,252,253,254,255, //60
255,255,254,253,252,251,249,247,244,241, //90
238,234,231,227,222,218,213,208,202,197,
191,185,179,173,167,160,154,147,141,134,
128,121,114,108,101,95 ,88 ,82 ,76 ,70 ,
64 ,58 ,53 ,47 ,42 ,37 ,33 ,28 ,24 ,21 ,
17 ,14 ,11 ,8  ,6  ,4  ,3  ,2  ,1  ,0  ,
0  ,0  ,1  ,2  ,3  ,4  ,6  ,8  ,11 ,14 ,
17 ,21 ,24 ,28 ,33 ,37 ,42 ,47 ,53 ,58 ,
64 ,70 ,76 ,82 ,88 ,95 ,101,108,114,121,

128,134,141,147,154,160,167,173,179,185,
191,197,202,208,213,218,222,227,231,234,
238,241,244,247,249,251,252,253,254,255,
255,255,254,253,252,251,249,247,244,241,
238,234,231,227,222,218,213,208,202,197,
191,185,179,173,167,160,154,147,141,134,
};

volatile uint8_t adc_buffer[N_ADC] = {0};

// This interrupt is triggered once the ADC buffer is full
void DMA1_Channel1_IRQHandler(void) __attribute__((interrupt));
void DMA1_Channel1_IRQHandler(){
    if(DMA1->INTFR & DMA1_FLAG_TC1) {

		adc_buffer_pt = adc_buffer;

		// We do not do a convolution with a sine wave, but a square wave.
		for(uint8_t i=0; i<4; i++){
			quad_values[i] = 0;
			for(uint8_t j=0; j<15; j++)	quad_values[i] += *adc_buffer_pt++;
		}

		// We extract the sine and cosine equivalent 
		y = (quad_values[0] + quad_values[1] - quad_values[2] - quad_values[3]); // "Sin"
		x = (quad_values[0] - quad_values[1] - quad_values[2] + quad_values[3]); // "Cos"

		sxy = x+y;
		dxy = x-y;
		
		// Approximate the angle. Here we use a linear approximation
		// The angle is between -127 and 127
		if(x > 0){
			if(y>0)	angle =  32-(32*dxy)/(sxy); 
			else	angle = -32+(32*sxy)/(dxy); 
		}else{
			if(y>0)	angle =  96+(32*sxy)/(dxy); 
			else	angle = -96-(32*dxy)/(sxy);  
		}

		// The movement is filtered to avoid jumping an entire cycle
		pos += (int8_t)(angle-(int8_t)pos)/4;


        DMA1->INTFCR = DMA_CTCIF1;
    }
}


int main(){
	SystemInit();

	// Enable AFIO, GPIOC, GPIOD and TIM1
	RCC->APB2PCENR |= RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1;

	// Enable DMA
	RCC->AHBPCENR = RCC_AHBPeriph_SRAM | RCC_AHBPeriph_DMA1;

	// Remap TIM1
	// CH1-PC4,CH1N-PC3,CH2-PC7,CH2N-PC2,CH3-PC5,CH3N-PC6
	AFIO->PCFR1 |= AFIO_PCFR1_TIM1_REMAP_FULLREMAP;


	//// ADC Setup

	// DMA1 CH1 Setup
	// This channel is triggered by the ADC and writes ADC samples to memory
	DMA1_Channel1->CNTR = N_ADC;
	DMA1_Channel1->MADDR = (uint32_t)adc_buffer;
	DMA1_Channel1->PADDR = (uint32_t)&ADC1->RDATAR; 
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	DMA1_Channel1->CFGR = 
		0 |    				                 // PERIPHERAL2MEM
		DMA_CFGR1_PL |                       // High priority.
		0 |               					 // 8-bit memory (We use only the LSB)
		DMA_CFGR1_PSIZE_0 |                  // 16-bit peripheral
		DMA_CFGR1_MINC |                     // Increase memory.
		DMA_CFGR1_CIRC |                     // Circular mode.
		DMA_CFGR1_TCIE | 					 // Interrupt for a full buffer
		DMA_CFGR1_EN;                        // Enable

	// Reset pin PD4, to Analog In
    GPIOD->CFGLR &= ~(0xf<<(4*4));	// CNF = 00: Analog, MODE = 00: Input

    // Reset the ADC to init all regs
    RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
    RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

    // ADCCLK = 24 MHz => RCC_ADCPRE divide by 2
    RCC->CFGR0 &= ~RCC_ADCPRE;  // Clear out the bis in case they were set
    RCC->CFGR0 |= RCC_ADCPRE_DIV2;	// set it to 010xx for /2.

    // Keep CALVOL register with initial value
    ADC1->CTLR2 = ADC_ADON | ADC_DMA | ADC_EXTTRIG | ADC_ExternalTrigConv_T1_TRGO;

    // Possible times: 0->3,1->9,2->15,3->30,4->43,5->57,6->73,7->241 cycles
	// This time (241*2) is larger than the timer period. We sample at half the rate.
    ADC1->SAMPTR2 = 7/*3 cycles*/ << (3/*offset per channel*/ * 7/*channel*/);

    // Set sequencer to channel 2 only
    ADC1->RSQR3 = 7;

    // Calibrate
    ADC1->CTLR2 |= ADC_RSTCAL;
    while(ADC1->CTLR2 & ADC_RSTCAL){}
    ADC1->CTLR2 |= ADC_CAL;
    while(ADC1->CTLR2 & ADC_CAL){}

	// SINE Setup
	// Setup the DMA channels for the 6 sine waves
	// DMA1CH2
	DMA1_Channel2->CNTR = N_SIN;
	DMA1_Channel2->MADDR = (uint32_t)sindata;
	DMA1_Channel2->PADDR = (uint32_t)&TIM1->CH1CVR; // This is the output register for out buffer.
	DMA1_Channel2->CFGR = 
		DMA_CFGR1_DIR |                      // MEM2PERIPHERAL
		DMA_CFGR1_PL |                       // High priority.
		0 |                                  // 8-bit memory
		DMA_CFGR1_PSIZE_1 |                  // 32-bit peripheral (0b10)
		DMA_CFGR1_MINC |                     // Increase memory.
		DMA_CFGR1_CIRC |                     // Circular mode.
		DMA_CFGR1_EN;                        // Enable

	// DMA1CH3
	DMA1_Channel3->CNTR = N_SIN;
	DMA1_Channel3->MADDR = (uint32_t)sindata+N_SIN_OFFSET;
	DMA1_Channel3->PADDR = (uint32_t)&TIM1->CH2CVR; // This is the output register for out buffer.
	DMA1_Channel3->CFGR = 
		DMA_CFGR1_DIR |                      // MEM2PERIPHERAL
		DMA_CFGR1_PL |                       // High priority.
		0 |                                  // 8-bit memory
		DMA_CFGR1_PSIZE_1 |                  // 32-bit peripheral
		DMA_CFGR1_MINC |                     // Increase memory.
		DMA_CFGR1_CIRC |                     // Circular mode.
		DMA_CFGR1_EN;                        // Enable

	// DMA1CH6
	DMA1_Channel6->CNTR = N_SIN;
	DMA1_Channel6->MADDR = (uint32_t)sindata+N_SIN_OFFSET*2;
	DMA1_Channel6->PADDR = (uint32_t)&TIM1->CH3CVR; // This is the output register for out buffer.
	DMA1_Channel6->CFGR = 
		DMA_CFGR1_DIR |                      // MEM2PERIPHERAL
		DMA_CFGR1_PL |                       // High priority.
		0 |                                  // 8-bit memory
		DMA_CFGR1_PSIZE_1 |                  // 32-bit peripheral
		DMA_CFGR1_MINC |                     // Increase memory.
		DMA_CFGR1_CIRC |                     // Circular mode.
		DMA_CFGR1_EN;                        // Enable

	// Configure PC3-7
	GPIOC->CFGLR &= ~(
		(0xf<<(4*3)) |
		(0xf<<(4*4)) |
		(0xf<<(4*5)) |
		(0xf<<(4*6)) |
		(0xf<<(4*7))
	);
	GPIOC->CFGLR |= (
		(GPIO_Speed_2MHz | GPIO_CNF_OUT_PP_AF)<<(4*3) |
		(GPIO_Speed_2MHz | GPIO_CNF_OUT_PP_AF)<<(4*4) |
		(GPIO_Speed_2MHz | GPIO_CNF_OUT_PP_AF)<<(4*6) |
		(GPIO_Speed_2MHz | GPIO_CNF_OUT_PP_AF)<<(4*5) |
		(GPIO_Speed_2MHz | GPIO_CNF_OUT_PP_AF)<<(4*7)
	);
	// GPIOC->CFGLR &= 0x00000FFF;
	// GPIOC->CFGLR |= 0x99999000;

	//Configure PD2
	GPIOD->CFGLR &= ~(0xf<<(4*2));
	GPIOD->CFGLR |= (GPIO_Speed_2MHz | GPIO_CNF_OUT_PP_AF)<<(4*2);

	// Reset TIM1 to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;
		
	// Prescaler 
	TIM1->PSC = 0x0000;
	
	// Auto Reload - sets period
	TIM1->ATRLR = 255;
	
	// Reload immediately, trigger DMA
	TIM1->SWEVGR = TIM_UG | TIM_TG; 

	// Enable CH1, CH1N, ... CH3N
	TIM1->CCER |= TIM_CC1E | TIM_CC1NE | TIM_CC2E | TIM_CC2NE | TIM_CC3E | TIM_CC3NE;
		
	// CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
	TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1 | TIM_OC2M_2 | TIM_OC2M_1;
	TIM1->CHCTLR2 |= TIM_OC3M_2 | TIM_OC3M_1;
	
	// Enable TIM1 outputs
	TIM1->BDTR |= TIM_MOE;

	// Enable DMA Interrupt
	TIM1->DMAINTENR = TIM_CC1DE | TIM_CC2DE | TIM_CC3DE | TIM_UIE;

    // TRGO source from Update
	TIM1->CTLR2 = TIM_MMS_1;

	// Enable TIM1
	TIM1->CTLR1 |= TIM_CEN;


	Delay_Ms( 100 );
	int32_t pos0 = pos;

	while(1){
		Delay_Ms( 100 );
		printf("%10ld \r", pos-pos0);
	}
}