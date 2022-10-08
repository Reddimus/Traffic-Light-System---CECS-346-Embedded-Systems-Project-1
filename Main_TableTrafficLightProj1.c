// KeviM
// Traffic Light System - CECS 346 Embedded Systems Project 1

// east/west is negative logic
// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3

// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0

// South Rt Turn LED connected to PF3 & PF1 (Port F is negative logic)

// north/south car detector connected to PA3 (1=car present)
// east/west car detector connected to PA2 (1=car present)
// South Rt Turn car detector connected to PF4 (0=car present)


// ***** 1. Pre-processor Directives Section *****

#include "SysTick.h"
#include "tm4c123gh6pm.h"
// ***** 2. Global Declarations Section *****

// Port A																								APB	0x4000.4000	
#define GPIO_PORTA_LOCK_R       (*((volatile unsigned long *)0x40004520))
#define GPIO_PORTA_CR_R         (*((volatile unsigned long *)0x40004524))
// enable Port A IO direction		Input = 0
#define GPIO_PORTA_DIR_R 				(*((volatile unsigned long *)0x40004400))
// enable port A digital IO pins	0 = disabled, 1 = enabled
#define GPIO_PORTA_DEN_R				(*((volatile unsigned long *)0x4000451C))
// enable port A Read/ write 		GPIO_PORTA_DEN_R must = 1
#define GPIO_PORTA_DATA_R				(*((volatile unsigned long *)0x400043FC))
// enable port A alternate function
#define GPIO_PORTA_AFSEL_R			(*((volatile unsigned long *)0x40004420))
// enable port A port control
#define GPIO_PORTA_PCTL_R				(*((volatile unsigned long *)0x4000452C))
// enable port A internal pull down resistor		offset 0x52C		1 = Enable Internal pull down resitor
#define GPIO_PORTA_PDR_R				(*((volatile unsigned long *)0x40004514))
// enable port A analog function 
#define GPIO_PORTA_AMSEL_R			(*((volatile unsigned long *)0x40004528))	
#define GPIO_PORTA_PUR_R			(*((volatile unsigned long *)0x40004510))
	

// Port F																							APB	0x4002.5000
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
// enable Port F IO direction		Output = 0
#define GPIO_PORTF_DIR_R				(*((volatile unsigned long *)0x40025400))
// enable port F digital IO pins	0 = disabled, 1 = enabled
#define GPIO_PORTF_DEN_R				(*((volatile unsigned long *)0x4002551C))
// enable port F Read/ write 		GPIO_PORTA_DEN_R must = 1
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
// enable port F alternate function
#define GPIO_PORTF_AFSEL_R			(*((volatile unsigned long *)0x40025420))
// enable port F port control
#define GPIO_PORTF_PCTL_R				(*((volatile unsigned long *)0x4002552C))
// enable port F analog function
#define GPIO_PORTF_AMSEL_R			(*((volatile unsigned long *)0x40025528))
//enable pull up resistors
#define GPIO_PORTF_PUR_R			(*((volatile unsigned long *)0x40025510))
	//#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400F5108))

//Port B 																						APB 0x4000.5000
// enable Port A IO direction		Input = 0
#define GPIO_PORTB_DIR_R 				(*((volatile unsigned long *)0x40005400))
// enable port B digital IO pins	0 = disabled, 1 = enabled
#define GPIO_PORTB_DEN_R				(*((volatile unsigned long *)0x4000551C))
// enable port B Read/ write 		GPIO_PORTA_DEN_R must = 1
#define GPIO_PORTB_DATA_R				(*((volatile unsigned long *)0x400053FC))
// enable port B alternate function
#define GPIO_PORTB_AFSEL_R			(*((volatile unsigned long *)0x40005420))
// enable port B port control
#define GPIO_PORTB_PCTL_R				(*((volatile unsigned long *)0x4000552C))
// enable port B internal pull down resistor		offset 0x52C		1 = Enable Internal pull down resitor
#define GPIO_PORTB_PDR_R				(*((volatile unsigned long *)0x40005514))
// enable port B analog function 
#define GPIO_PORTB_AMSEL_R			(*((volatile unsigned long *)0x40005528))	
#define GPIO_PORTB_PUR_R			(*((volatile unsigned long *)0x40005510))


void PortA_Init(void)
{   
  GPIO_PORTA_AMSEL_R &= ~0x0C;       // disable analog function
  GPIO_PORTA_PCTL_R = 0x00000000;    // GPIO clear bit PCTL  
  GPIO_PORTA_DIR_R |= 0xF3;          //  PA3,PA2 as input
  GPIO_PORTA_AFSEL_R &= ~0x0C;       // no alternate function  
  GPIO_PORTA_DEN_R |= 0x0C;          // enable digital pins PA4-PA0 
	GPIO_PORTA_PUR_R &= ~0x0C;         // disable pullup resistors
	GPIO_PORTA_PDR_R |= 0x0C;          // enable pull down resistors
}	


void PortF_Init(void)
{ 
  GPIO_PORTF_LOCK_R |= 0x4C4F434B;   // unlock Port
  GPIO_PORTF_CR_R |= 0x1F;           // allow changes to Port F PF4-0       
  GPIO_PORTF_AMSEL_R &= 0x00;        // disable analog function
  GPIO_PORTF_PCTL_R &= 0x00000000;   // GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R |= 0x0E;          // PF4, PF0 input, PF3, PF2, PF1 output 
  GPIO_PORTF_AFSEL_R &= 0x00;        // no alternate function
  GPIO_PORTF_PUR_R |= 0x11;          // disable pullup resistors	
  GPIO_PORTF_DEN_R |= 0x1F;          // enable digital pins PF4-PF0        
}


void PortB_Init(void)	//Negative logic LEDs
{
	GPIO_PORTB_AMSEL_R &= ~0x3F; 	// disable analog function
	GPIO_PORTB_PCTL_R &= ~0xFF;   // GPIO clear bit PCTL
	GPIO_PORTB_DIR_R |= 0x3F; 	  // PB0-PB5 as output
	GPIO_PORTB_AFSEL_R &= ~0x3F; //  no alternate function
  GPIO_PORTB_PUR_R &= ~0x3F; 	 //  disable pull up resistors
	GPIO_PORTB_PDR_R &= ~0x3F; 	 //  disable pull down resistors
  GPIO_PORTB_DEN_R |= 0x3F; 	 //  enable digital pins
}

// FSM data structure
struct State
{
	unsigned long Out[2]; // 2 element array holding Port B and Port F output values
	unsigned long Time;
	unsigned long Next[8]; //list of next states
};

typedef const struct State STyp;
// States
#define goWest 0
#define waitWest 1
#define goSouth 2
#define waitSouth 3
#define goRT 4
#define waitRT 5


STyp FSM[6] = 
{//{PB, PF},  time, {next states...}
	{{0x34, 0x12}, 6, {waitWest, 	goWest, 		waitWest, waitWest, 	goWest, 		goWest, 		waitWest, 	waitWest}},			//goWest
	{{0x2C, 0x12}, 2, {goRT, 			goRT, 			goRT, 		goRT, 			goSouth, 		goSouth, 		goSouth, 		goSouth}},			//waitWest
	{{0x19, 0x12}, 6, {goRT, 			waitSouth, 	goRT, 		waitSouth, 	goSouth, 		waitSouth, 	goSouth, 		waitSouth}},		//goSouth
	{{0x1A, 0x12}, 2, {goWest, 		goWest, 		goWest, 	goWest, 		goWest, 		goWest, 		goWest, 		goWest}},				//waitSouth
	{{0x19, 0x08}, 6, {goRT, 			waitRT, 		goRT, 		waitRT, 		goRT, 			waitRT, 		goRT, 			waitRT}},				//goRT
	{{0x1A, 0x1A}, 2, {goWest, 		goWest, 		goWest, 	goWest, 		goWest, 		goWest, 		goWest, 		goWest}}				//waitRT
};


int main(void)
{
	volatile unsigned long delay;
	unsigned long CS;		// index of current state
	unsigned long Input;
	SysTick_Init();
	SYSCTL_RCGC2_R |= 0x23; // set Port A, B, F
	PortA_Init(); //Call port A
	PortB_Init(); //call port B
	PortF_Init(); //Call port F
	CS = goWest; 	// initial state
	while(1)
	{
		// Loop through FSM data structure infinitely
		// LED output Port B && Port F
		GPIO_PORTB_DATA_R = FSM[CS].Out[0];
		GPIO_PORTF_DATA_R = FSM[CS].Out[1];
		
		// unsigned long delay = FSM[CS].Time;
		SysTick_Wait1s(FSM[CS].Time);
		
		//read button input PA2&3				&& PF4
		Input = (GPIO_PORTA_DATA_R >> 2) + (GPIO_PORTF_DATA_R >> 2);
		
		//Transition to new state
		CS = FSM[CS].Next[Input];
	}
}
