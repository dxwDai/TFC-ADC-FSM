//***************************************************************************
// * 
// * Created on:  01 11 2016
// * Author:      Xuweu Dai  (xuewu.dai at northumbria.ac.uk)
// *
// * File:        Example Programme ADC at KL25Z
// *
// * This program converts the analog input from ADC channel 13 (PTB3) 
// * using software trigger continuously.
// * PTB3 is connect to the potentiometer POT1 at the TFC-Shield board.
// * When sdjusting POT1, the voltage of PTB3 varies from 0 volts to 3.3 volts.
// * The value of this voltage is used to control the tri-color LEDs. 
// * When the potentiometer is turned, the LEDs colour changes.
// * At the same time, the result of the each A/D conversionn is also sent to 
// * the PC and through the UART0 and displayed at the PC's terminal windows.
// *
// * Copyright:   (C) 2016 Northumbria University, Newcastle upon Tyne, UK.
// *
// * This program is free software: you can redistribute it and/or modify
// * it under the terms of the GNU Lesser General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// 
// * This program is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// * GNU Lesser General Public License for more details.
// *
// %              _
// %  \/\ /\ /   /  * _  '
// % _/\ \/\/ __/__.'(_|_|_
// **************************************************************************/


#include "MKL25Z4.h"
#include "stdio.h"
#include "TFC_ADCCamera.h"





// #define ADC_SC1_ADCH_MASK      0x1Fu   
// #define ADC_SC1_ADCH(x)     (((uint32_t)(x))&ADC_SC1_ADCH_MASK)

/*
enum FSMADC            // Defines an enumeration type    
{
    init=-1,        
	  ADC_STATE_IDLE=0,
    ADC_STATE_CAPTURE_POT_0 = 1,     
	  ADC_STATE_CAPTURE_POT_1 =2,
	  ADC_STATE_CAPTURE_BATTERY_LEVEL=3,
    ADC_STATE_CAPTURE_LINE_SCAN = 4
} adcState;
*/

#define ADC_STATE_INIT							-1
#define ADC_STATE_IDLE							0
#define ADC_STATE_CAPTURE_POT_0			        1
#define ADC_STATE_CAPTURE_POT_1			        2
#define ADC_STATE_CAPTURE_BATTERY_LEVEL			3
#define ADC_STATE_CAPTURE_LINE_SCAN		      4

static uint8_t 	CurrentADC_State =	ADC_STATE_INIT;	

volatile uint16_t PotADC_Value[2];
volatile uint16_t BatSenseADC_Value;

volatile uint8_t CurrentLineScanPixel = 0;

volatile uint8_t cameraImageReady = 0;

short unsigned int imageData[2][128]; // a ping-pong buffer


void camInit(void)
{  
	//Initial the IO port
	// SI (PTD7) output, CLK (PTE1) output
	// AO (PTD5) input
	// SIM->SCGC5 |= (0x1<<10); // enable clock to PTB
	SIM->SCGC5 |=(0x1<<12 | 0x1<<13); 
  /* enable clock to Port D, E */
	
    PORTD->PCR[7] = 0x100;     /* make PTD7 pin as GPIO */
    PTD->PDDR |= (0x1<<7);     /* make PTD7 as output pin */
	
    PORTE->PCR[1] = 0x100;     /* make PTE1 pin as GPIO */
    PTE->PDDR |= (0x1<<1);     /* make PTE1 as output pin */
	
		// PORTD->PCR[5] = 0x100;     /* make PTD5 pin as GPIO */
		// PTD->PDDR &= ~(0x1<<5);    /* make PTD5 as input pin */
		PORTD->PCR[5] = 0; // PTD5.MUX[10 9 8]=000, analog input
	
	
	  SIM->SCGC6 |= 0x08000000;   // enable clock to ADC0 ; 0x8000000u
	  //  SIM_SCGC6 |= (SIM_SCGC6_ADC0_MASK);
    
		
// Configure ADC as it will be used, but because ADC_SC1_ADCH is 31,
    // the ADC will be inactive.  Channel 31 is just disable function.
    // There really is no channel 31.
	  // disable AIEN, Signle-ended, channel 31
    // ADC0->SC1[0] = AIEN_MASK|DIFF_SINGLE|ADC_SC1_ADCH(31);  
	 ADC0->SC1[0] = (~AIEN_MASK)& DIFF_SINGLE |ADC_SC1_ADCH(31);  
		
		// TODO: use hardware triger, TPM0
	  ADC0->SC2 &= ~0x40;   // ADTRG=0, software trigger
	
	  // clock div by 4, long sample time, single ended 12 bit, bus clock 
    ADC0->CFG1 =(0x1<<6 | 0x1<<4 |0x1<<2); //0b01010100; 
	  //  ADC0->CFG1 = 0x40 | 0x10 | 0x04 | 0x00;
	
	  // select the B set of ADC input channels for PTD5 (SE6b)
	  ADC0->CFG2 |=(0x1<<4); //CFG2.MUXSEL=1, ADxxb channels are selected; 
	
	  CAM_SI_LOW;
	  CAM_CK_LOW;

}   
		

void TFC_adcFSMInit()
{
	
	CurrentADC_State =	ADC_STATE_INIT;	
	
	   // disable_irq(INT_ADC0-16);   
    // Set the ICER register accordingly */
   //  NVIC_ICER = 1 << INT_ADC0-16;
   __disable_irq();    /* disable all IRQs */
	
	  NVIC->ICER[0] = 1 << 15; // disable IRQ15 (ADC0) interrupt
	
		camInit();
		ADC0_init();
	
	 //The pump will be primed with the PIT interrupt.  upon timeout/interrupt it will set the SI signal high
	//for the camera and then start the conversions for the pots.
	
	/*
	//Enable clock to the PIT
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	//We will use PIT0
	TFC_SetLineScanExposureTime(TFC_DEFAULT_LINESCAN_EXPOSURE_TIME_uS);
	//enable PIT0 and its interrupt
	PIT_TCTRL0 = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;

	PIT_MCR |= PIT_MCR_FRZ_MASK; // stop the pit when in debug mode
	//Enable the PIT module
	PIT_MCR &= ~PIT_MCR_MDIS_MASK;
	
	enable_irq(INT_PIT-16);
	*/
	
	// enable_irq(INT_ADC0-16);
	   // Set the ICPR and ISER registers accordingly */
    //  NVIC_ICPR |= 1 << (INT_ADC0-16);
    //  NVIC_ISER |= 1 << (INT_ADC0-16);
		
		// NVIC->ISER[0] |= 0x40000000;    /* enable INT30 (bit 30 of ISER[0]) for PORTA interrupt */
		NVIC->ISER[0] |= 1u<<15;    /* enable IRQ15 (bit 15 of ISER[0]) for ADC0 interrupt */
		
		
    __enable_irq();             /* global enable IRQs */
	
	CurrentADC_State =	ADC_STATE_IDLE;	
	
}

	//All Adc processing of the Pots and linescan will be done in the ADC0 IRQ!
	//A state machine will scan through the channels.
	//This is done to automate the linescan capture on Channel 0 to ensure that timing is very even
void TFC_adcFSM_Start()
{

	// start camera's integration immediately to guarantee the same integration time
	CAM_SI_HIGH;
	
	//Prime the ADC pump and start capturing POT 0
	CurrentADC_State = ADC_STATE_CAPTURE_POT_0;
	
	NVIC->ISER[0] |= 1u<<15;    /* enable IRQ15 (bit 15 of ISER[0]) for ADC0 interrupt */
	
	// ADC0_CFG2  &= ~ADC_CFG2_MUXSEL_MASK; //Select the A side of the mux for POT0
	ADC0->CFG2 &=~(0x1<<4); //CFG2.MUXSEL=0, ADxxa channels are selected; 
	 //Start the State machine at POT0
	ADC0->SC1[0] =  AIEN_MASK| ADC_SC1_ADCH(TFC_POT_0_ADC_CHANNEL) ; 
}


void ADC0_IRQHandler()
{
	short unsigned int Junk;
	switch(CurrentADC_State)
	{
		default:
			Junk =  ADC0->R[0];
		  break;
		
		case ADC_STATE_CAPTURE_POT_0:
				
				PotADC_Value[0] = ADC0->R[0];
		

		    // transfer to reading POT1    
		    ADC0->CFG2 &=~(0x1<<4); //CFG2.MUXSEL=0, Select the A side of the mux
			  ADC0->SC1[0] =  AIEN_MASK| ADC_SC1_ADCH(TFC_POT_1_ADC_CHANNEL) ; 
		
			//DXW: To debug, jump to camera capture directly  
		   //	CurrentADC_State = ADC_STATE_CAPTURE_POT_1;
				CurrentADC_State=ADC_STATE_CAPTURE_BATTERY_LEVEL;
			 
			break;
		
		case ADC_STATE_CAPTURE_POT_1:
		
				PotADC_Value[1] = ADC0->R[0];
		    ADC0->CFG2 |=(0x1<<4); //CFG2.MUXSEL=1, Select the B side of the mux
				ADC0->SC1[0]  =  AIEN_MASK| ADC_SC1_ADCH(TFC_BAT_SENSE_CHANNEL);
			  CurrentADC_State = ADC_STATE_CAPTURE_BATTERY_LEVEL;
				
			break;
		
		case ADC_STATE_CAPTURE_BATTERY_LEVEL:
			
				BatSenseADC_Value = ADC0_RA;
				
				//Now we will start the sequence for the Linescan camera
			 CAM_CK_HIGH;
			// TPM0_DelayOnce();
			 for(Junk = 0;Junk<50;Junk++){}
	     CAM_SI_LOW;
	     // TPM0_DelayOnce();
								
				 ADC0->CFG2 |=(0x1<<4); //CFG2.MUXSEL=1, Select the B side of the mux
				// ADC0->SC1[0]  =  AIEN_MASK| ADC_SC1_ADCH(TFC_LINESCAN0_ADC_CHANNEL);
		    ADC0_SC1A=  AIEN_MASK| ADC_SC1_ADCH(TFC_LINESCAN0_ADC_CHANNEL);
			 
				CurrentLineScanPixel = 0;
    	  CurrentADC_State = ADC_STATE_CAPTURE_LINE_SCAN;
				 
					
				break;
		
		case ADC_STATE_CAPTURE_LINE_SCAN:
					
					if(CurrentLineScanPixel<128)
					{
							imageData[0][CurrentLineScanPixel] = ADC0_RA;
	        
							// for next pixel
							CurrentLineScanPixel++;
							
						  CAM_CK_LOW;
							for(Junk = 0;Junk<50;Junk++) {}
							CAM_CK_HIGH;
							ADC0_SC1A=  AIEN_MASK| ADC_SC1_ADCH(TFC_LINESCAN0_ADC_CHANNEL);
							
						}
		
					else
					{
						// done with the capture sequence.  we can wait for the PIT0 IRQ to restart
						Junk =  ADC0_RA;
					 // one more clock to ensure the camera return a known state
						// DXW: this was dow when CurrentLineScanPixel=127
						CAM_CK_HIGH; //??
											
						for(Junk = 0;Junk<50;Junk++){	}
						
						CAM_CK_LOW;
						CurrentADC_State = ADC_STATE_IDLE;	 
						
						/*
						//swap the buffer, a ping-pong buffer
						
						if(LineScanWorkingBuffer == 0)
						{
							LineScanWorkingBuffer = 1;
							
							LineScanImage0WorkingBuffer = &LineScanImage0Buffer[1][0];
							LineScanImage1WorkingBuffer = &LineScanImage1Buffer[1][0];
							
							LineScanImage0 = &LineScanImage0Buffer[0][0];
							LineScanImage1 = &LineScanImage1Buffer[0][0];
						}
						else
						{
							LineScanWorkingBuffer = 0;
							LineScanImage0WorkingBuffer = &LineScanImage0Buffer[0][0];
							LineScanImage1WorkingBuffer = &LineScanImage1Buffer[0][0];
							
							LineScanImage0 = &LineScanImage0Buffer[1][0];
							LineScanImage1 = &LineScanImage1Buffer[1][0];
						}
						*/
						cameraImageReady = TRUE;
					}
					
					break;
	
	}

}







void cam_ReadImage(short int *imgData)
{  
	unsigned int i;
	// SI (PTD7) Digital output, CLK (PTE1) Digitaloutput
	// AO (PTD5) Analgoue input (channel 6)
	float delayQuarterCycle; // in us
		
	// initialize TPM0_MOD for the expected delays
	delayQuarterCycle=0.5;
	TPM0->MOD = (unsigned int)(delayQuarterCycle*(float)(FRQ_MCGFLLCLK)/1000000.0);
	
	
	 CAM_SI_HIGH;
	 TPM0_DelayOnce();
	 CAM_CK_HIGH;
	 TPM0_DelayOnce();
	 CAM_SI_LOW;
	 TPM0_DelayOnce();
	 
	// the following reading sequence is different from the diagram
	// Fig. 11, page 10, AMU's TLS1401CL.pdf. 
	// image sensor is red at the falling edge of CLK
	// howver, Fig. 11 suggests reading image at rising edge
	// read the 128 pixel image data
	 // imgData[0]=(short int)readADC(6); 
	 // CAM_CK_LOW;
		CAM_CK_LOW;
		TPM0_DelayOnce();
		TPM0_DelayOnce();
		
		// start ADC conversion
		ADC0->SC1[0] =6;  // channel 6 for camera SE6b
		
		
	for (i=0;i<128;i++)
	{
		imgData[i]=(short int)readADC(6); 
		CAM_CK_HIGH;
		TPM0_DelayOnce();
		TPM0_DelayOnce();
		CAM_CK_LOW;
		TPM0_DelayOnce();
		TPM0_DelayOnce();
	}
	
	// additional one CLK cycle to ensure the internal logic of the camera
  // 	is at a known state.  CLK is set to low at the end. 
	  CAM_CK_HIGH;
		TPM0_DelayOnce();
		TPM0_DelayOnce();
		CAM_CK_LOW;
	  TPM0_DelayOnce();
		TPM0_DelayOnce();
}   
		

		
void ADC0_init(void)
{
	
    // initialization for PORTB PTB3 (ADC0_SE13) 
    SIM->SCGC5 |= 0x0400;  // enable clock to PORTB 
	  PORTB->PCR[3] = 0;         // PTB3.MUX[10 9 8]=000, analog input
    		
	  SIM->SCGC6 |= 0x08000000;   // enable clock to ADC0 
	
		// Configure ADC as it will be used, but because ADC_SC1_ADCH is 31,
    // the ADC will be inactive.  Channel 31 is just disable function.
    // There really is no channel 31.
	  // disable AIEN, Signle-ended, channel 31
    ADC0->SC1[0] = DIFF_SINGLE|ADC_SC1_ADCH(31);  
    ADC0->SC2 &= ~0x40;   // ADTRG=0, software trigger
  	
	/* clock div by 4, long sample time, single ended 12 bit, bus clock */
    ADC0->CFG1 =(0x1<<6 | 0x1<<4 |0x1<<2); //0b01010100; 
	  //  ADC0->CFG1 = 0x40 | 0x10 | 0x04 | 0x00;
	
    // select the A set of ADC input channels for PTD5 (SE6b)
	  ADC0->CFG2 &=~(0x1<<4); //CFG2.MUXSEL=0, ADxxa channels are selected; 
}

short int readADC(short ChID) 
{
	short int result;     	
	
	ADC0->SC1[0] = ChID; //software triger conversion on channel 13, SE13
	while(!(ADC0->SC1[0] & 0x80)) { } /* wait for conversion complete */
	result = ADC0->R[0];        /* read conversion result and clear COCO flag */
	return result;
}

// Initialize the TPM0 to generate a specified delay in number of MCGFLLCLK clocks
// By default, the MCGFLLCLK set by system setup is 20.97152MHz
void TPM0_init()
{
	
	SIM->SCGC6 |= (0x01<<24); // 0x01000000;, enale clk to TPM0
	SIM->SOPT2 |=(0x01<<24); // 0x01000000, use MCGFLLCLK as timer counter clk
	TPM0->SC = 0; // diable timer when configuring
	TPM0->MOD = 0xFFFF; // by default, set the 16-bit modulo value to maximum
	                    // thus results in maximum delay
	// TPM0->MOD = initMODvalue;
	
	TPM0->SC|=0x80; // clear TOF
	// TPM0->SC|=0x08; // enable timer free-rnning mode
}

// Initialize the TPM0 to generate a specified delay in number of MCGFLLCLK clocks
// By default, the MCGFLLCLK set by system setup is 20.97152MHz

// A signle delay for a pre-specified period. 
// The amount of delay should be specified bedore invoking TPM0_DelayOnce()
// by setting TPM0->MOD to an expected value
void TPM0_DelayOnce(void)
{
	TPM0->SC|=0x80; // clear TOF
	TPM0->SC|=0x08; // enable timer free-rnning mode
	while((TPM0->SC & 0x80) == 0) { } // wait until the TOF is set
	TPM0->SC = 0; // diable timer when configuring
}

void TPM0_Delay(float time_in_us)
{
  float t;	
	TPM0->SC = 0; // diable timer when configuring
	
	//Figure out how many clocks we need for for the delay time
	t=time_in_us*(float)(FRQ_MCGFLLCLK)/1000000.0; //MOD value for delay
	
	TPM0->MOD = (unsigned int) t;
	TPM0->SC|=0x80; // clear TOF
	TPM0->SC|=0x08; // enable timer free-rnning mode
	while((TPM0->SC & 0x80) == 0) { } // wait until the TOF is set
	TPM0->SC = 0; // diable timer when configuring
}