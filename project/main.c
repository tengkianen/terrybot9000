/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
#include "MKL25Z4.h"                    // Device header 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
 
/*----------------------------------------------------------------------------
 * Definitions
 *---------------------------------------------------------------------------*/
#define FREQUENCY_TO_MOD(x) (375000 / x)
#define BAUD_RATE 9600

#define MASK(x) (1 << (x)) // Mask shortkey

#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23

#define PTB0_Pin 0
#define PTB1_Pin 1
#define PTB2_Pin 2
#define PTB3_Pin 3
#define PTE29_Pin 29
#define PTE30_Pin 30

#define GREEN_LED_A 7
#define GREEN_LED_B 0
#define GREEN_LED_C 3
#define GREEN_LED_D 4
#define GREEN_LED_E 5
#define GREEN_LED_F 6
#define GREEN_LED_G 10
#define GREEN_LED_H 11
#define RED_LED_ALL 12

#define CONNECT 0x07
#define END 0x08
#define STOP 0x00
#define FORWARD 0x01
#define BACKWARD 0x02
#define FLEFT 0x03
#define FRIGHT 0x04
#define BLEFT 0x05
#define BRIGHT 0x06


#define Q_SIZE (32)

#define a4 440.00
#define b4 493.88
#define c5 523.25
#define d5 587.33
#define e5 659.25
#define fs4 369.99
#define fs5 739.9
#define g4 392.00
#define g5 783.99

#define f4 349.23
#define d4 293.66
#define e4 329.63
#define c4 261.63

#define B 617
#define Cs 693
#define Ds 777
#define E 824
#define F 873
#define Fs 925
#define G 980
#define Gs 1003
#define A2 1100
#define A2s 1165
#define B2 1234
#define Cthree 1308
#define C3s 1385
#define D3 1555


osMessageQueueId_t wheelsMsg, musicMsg, ledRedMsg, ledGreenMsg;

typedef enum states
{
    led_on,
    led_off,
} my_state_t;



typedef enum movStates
{
    moving,
    stationary,
		connecting,
		end
} my_movState_t;

my_movState_t state = stationary;

char led_mapping[8] = {GREEN_LED_A, GREEN_LED_B, GREEN_LED_C, GREEN_LED_D, GREEN_LED_E, GREEN_LED_F, GREEN_LED_G, GREEN_LED_H};

unsigned char command;

typedef struct {
	unsigned char Data[Q_SIZE];
	unsigned int Head; // points to oldest data element 
	unsigned int Tail; // points to next free space
	unsigned int Size; // quantity of elements in queue
} Q_T;

Q_T rx_q;

float half = 0.5;
float one = 1;
float two = 2;
float three = 3;
float one_half = 1.5;
float two_half = 2.5;

 /*----------------------------------------------------------------------------
 * Initialization
 *---------------------------------------------------------------------------*/
void Q_Init(Q_T * q) {
  unsigned int i;
  for (i=0; i<Q_SIZE; i++)
		q->Data[i] = 0; // to simplify our lives when debugging 
	q->Head = 0;
	q->Tail = 0;
	q->Size = 0;
}

int Q_Empty(Q_T * q) { 
	return q->Size == 0;
}
int Q_Full(Q_T * q) { 
	return q->Size == Q_SIZE;
}

int Q_Enqueue(Q_T * q, unsigned char d) {
  if (!Q_Full(q)) {
		q->Data[q->Tail++] = d; 
		q->Tail %= Q_SIZE; 
		q->Size++;
		return 1; // success
  } else
    return 0; // failure
}
unsigned char Q_Dequeue(Q_T * q) {
  // Must check to see if queue is empty before dequeueing
  unsigned char t=0;
  if (!Q_Empty(q)) {
		t = q->Data[q->Head];
		q->Data[q->Head++] = 0; 
		q->Head %= Q_SIZE;
		q->Size--;
	}
	return t; 
}

void initUART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;
	
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] = PORT_PCR_MUX(4);
	
	UART2->C2 &= ~(UART_C2_RE_MASK);
	
	bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	
	NVIC_SetPriority(UART2_IRQn, 128); 
	NVIC_ClearPendingIRQ(UART2_IRQn); 
	NVIC_EnableIRQ(UART2_IRQn);
	UART2->C2 |= UART_C2_RIE_MASK | UART_C2_RE_MASK; 
	Q_Init(&rx_q);
}
void initWheels() {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(3);
	
	//Enable clock gating to TPM
	SIM_SCGC6 |= (SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK);

	//Set TPM clk src to MCGFLLCLK clock or MCGPLLCLK/2
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	//Configure TPM1 to use LPTM counter increment on every LPTPM counter clock
	//Prescalar of 128. Operates in up counting mode
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~TPM_SC_CPWMS_MASK;
	
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~TPM_SC_CPWMS_MASK;

	//Clear and set Channel 0 Status and control of TPM1 to 
	//Edge-aligned PWM, clear output on match, set output on reload
	TPM1_C0SC &= ((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM1_C1SC &= ((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM2_C0SC &= ((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM2_C1SC &= ((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));	
	
	TPM1->MOD = FREQUENCY_TO_MOD(50);
	TPM2->MOD = FREQUENCY_TO_MOD(50);
	TPM1_C0V = 0;
	TPM1_C1V = 0;
	TPM2_C0V = 0;
	TPM2_C1V = 0;
	
}


void moveForward() {
	TPM1_C0V = 7499; //2 PTB0
	TPM1_C1V = 0; //4 PTB1
	TPM2_C0V = 7499; //1 PTB2
	TPM2_C1V = 0; //3 PTB3
}

void moveBackward() {
	TPM1_C0V = 0;
	TPM1_C1V = 7499; 
	TPM2_C0V = 0;
	TPM2_C1V = 7499;
}

void moveLeft() {
	TPM1_C0V = 850;
	TPM1_C1V = 0;
	TPM2_C0V = 7499;
	TPM2_C1V = 0;
}

void reverseLeft() {
	TPM1_C0V = 0;
	TPM1_C1V = 600;
	TPM2_C0V = 0;
	TPM2_C1V = 7499;
}

void reverseRight() {
	TPM1_C0V = 0;
	TPM1_C1V = 7499;
	TPM2_C0V = 0;
	TPM2_C1V = 600;
}

void moveRight() {
	TPM1_C0V = 7499;
	TPM1_C1V = 0;
	TPM2_C0V = 650;
	TPM2_C1V = 0;
}

void stop() {
	TPM1_C0V = 0;
	TPM1_C1V = 0;
	TPM2_C0V = 0;
	TPM2_C1V = 0;
}
 
void initGPIO() {
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	PORTC->PCR[GREEN_LED_A] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_A] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_B] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_B] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_C] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_C] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_D] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_D] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_E] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_E] |= PORT_PCR_MUX(1);

	PORTC->PCR[GREEN_LED_F] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_F] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_G] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_G] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_H] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_H] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[RED_LED_ALL] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RED_LED_ALL] |= PORT_PCR_MUX(1);
	
	PTC->PDDR |= (MASK(GREEN_LED_A) | MASK(GREEN_LED_B) | MASK(GREEN_LED_C) |
						    MASK(GREEN_LED_D) | MASK(GREEN_LED_E) | MASK(GREEN_LED_F) |
								MASK(GREEN_LED_G) | MASK(GREEN_LED_H) | MASK(RED_LED_ALL));
	
}

void offRGB() {
			PTC->PCOR |= (MASK(GREEN_LED_A) | MASK(GREEN_LED_B) | MASK(GREEN_LED_C) |
							    MASK(GREEN_LED_D) | MASK(GREEN_LED_E) | MASK(GREEN_LED_F) |
									MASK(GREEN_LED_G) | MASK(GREEN_LED_H))| MASK(RED_LED_ALL);
}

void offRGBr() {
			PTC->PCOR |= MASK(RED_LED_ALL);
}

void offRGBg() {
			PTC->PCOR |= (MASK(GREEN_LED_A) | MASK(GREEN_LED_B) | MASK(GREEN_LED_C) |
							    MASK(GREEN_LED_D) | MASK(GREEN_LED_E) | MASK(GREEN_LED_F) |
									MASK(GREEN_LED_G) | MASK(GREEN_LED_H));
}

void ledControl(int LEDtype, my_state_t state) {
	if(state == led_on) {
		PTC->PDOR |= MASK(LEDtype);
	} else {
		PTC->PCOR |= MASK(LEDtype);
	}
}

void initPWM(void) {
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[PTE29_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE29_Pin] |= PORT_PCR_MUX(3);
	
	PORTE->PCR[PTE30_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE30_Pin] |= PORT_PCR_MUX(3);
	
	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	// MOD = 48000000 /128 / 50 = 7500
	TPM0->MOD = 7499;
	TPM0_C2V = 0;	
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~TPM_SC_CPWMS_MASK;
	
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
}


void tone(float frequency, float delay_time) {
	TPM0->MOD = FREQUENCY_TO_MOD(frequency);
	TPM0_C2V = FREQUENCY_TO_MOD(frequency) /2;
	osDelay(delay_time + 250);
}


/*----------------------------------------------------------------------------
 * Interrupt handlers
 *---------------------------------------------------------------------------*/
void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		if (!Q_Full(&rx_q)) {
			Q_Enqueue(&rx_q, UART2->D);
		} 
	}
	if (UART2->S1 & (UART_S1_OR_MASK | UART_S1_NF_MASK | UART_S1_FE_MASK | UART_S1_PF_MASK)) {
		// handle error
		// clear flag
	}
}

/*----------------------------------------------------------------------------
 * Threads
 *---------------------------------------------------------------------------*/
void brain_thread(void *argument) {
	for (;;) {
		unsigned char data = 0x0;
		if (!Q_Empty(&rx_q)) {
			command = Q_Dequeue(&rx_q);
			data = command;
			switch(command) {
				case CONNECT:
					if (state != connecting) offRGB();
					state = connecting;
					osMessageQueuePut(ledRedMsg, &data, NULL, 0);
					osMessageQueuePut(ledGreenMsg, &data, NULL, 0);
					osMessageQueuePut(musicMsg, &data, NULL, 0);
					state = stationary;
					break;
				case END:
					if (state != end) offRGB();
					state = end;
					osMessageQueuePut(wheelsMsg, &data, NULL, 0);
					osMessageQueuePut(ledRedMsg, &data, NULL, 0);
					osMessageQueuePut(ledGreenMsg, &data, NULL, 0);
					osMessageQueuePut(musicMsg, &data, NULL, 0);
					break;
				case STOP:
					if (state != stationary) offRGB();
					state = stationary;
					osMessageQueuePut(wheelsMsg, &data, NULL, 0);
					osMessageQueuePut(ledRedMsg, &data, NULL, 0);
					osMessageQueuePut(ledGreenMsg, &data, NULL, 0);
					break;
				default:
					if(state != moving) offRGB();
					state = moving;
					osMessageQueuePut(wheelsMsg, &data, NULL, 0);
					osMessageQueuePut(ledRedMsg, &data, NULL, 0);
					osMessageQueuePut(ledGreenMsg, &data, NULL, 0);
					osMessageQueuePut(musicMsg, &data, NULL, 0);
					break;
			}
		}
		osDelay(100);
	}
}

void wheels_thread(void *argument) {
	unsigned char data;	
	for (;;) {
		osMessageQueueGet(wheelsMsg, &data, NULL, osWaitForever);
		switch(data) {
			case FORWARD:
				moveForward();
				break;
			case BACKWARD:
				moveBackward();
				break;
			case FLEFT:
				moveLeft();
				break;
			case FRIGHT:
				moveRight();
				break;
			case BLEFT:
				reverseLeft();
				break;
			case BRIGHT:
				reverseRight();
				break;
			default:
				stop();
				break;
		}
		osDelay(100);
	}
}

void led_red_thread (void *argument) {
	unsigned char data;	
	for (;;) {
		osMessageQueueGet(ledRedMsg, &data, NULL, 0);
		if (data == STOP || data == CONNECT || data == END) {
			ledControl(RED_LED_ALL,led_on);
			osDelay(250);
			ledControl(RED_LED_ALL,led_off);
			osDelay(250);
		} else {
			ledControl(RED_LED_ALL,led_on);
			osDelay(500);
			ledControl(RED_LED_ALL,led_off);
			osDelay(500);
		}
	}
}
void running_led_green_thread (void *argument) {
	unsigned char data;	
	for (;;) {
		osMessageQueueGet(ledGreenMsg, &data, NULL, 0);
		
		if (data == CONNECT) {
				for (int i = 0; i < 8; ++i) {
					ledControl(led_mapping[i], led_on);
				}
				osDelay(500);
				offRGBg();
				osDelay(500);
				for (int i = 0; i < 8; ++i) {
					ledControl(led_mapping[i], led_on);
				}
				osDelay(400);
				offRGBg();
				osDelay(200);
				data = 0x00;
				
		} else if (data == STOP || data == END) {
				ledControl(GREEN_LED_A, led_on);
				osDelay(100);
				ledControl(GREEN_LED_B, led_on);
				osDelay(100);
				ledControl(GREEN_LED_C, led_on);
				osDelay(100);
				ledControl(GREEN_LED_D, led_on);
				osDelay(100);
				ledControl(GREEN_LED_E, led_on);
				osDelay(100);
				ledControl(GREEN_LED_F, led_on);
				osDelay(100);
				ledControl(GREEN_LED_G, led_on);
				osDelay(100);
				ledControl(GREEN_LED_H, led_on);
				osDelay(200);
		} else {
				for (int i = 0; i < 8; ++i) {
					ledControl(led_mapping[i], led_on);
					osDelay(200);
					offRGBg();
					if (data == STOP || data == END) break;
				}
				for (int i = 6; i >0; --i) {
					ledControl(led_mapping[i], led_on);
					osDelay(200);
					offRGBg();
					if (data == STOP || data == END) break;
				}	
		}
	}
}

/*----------------------------------------------------------------------------
 * Music 
 *---------------------------------------------------------------------------*/

int tempo = 400;

void music_thread(void *argument) {
	unsigned char data;
	
	for (;;) {
		osMessageQueueGet(musicMsg, &data, NULL, osWaitForever);
		switch(data) {
			case CONNECT:
				tone(e5, tempo * half);
				if (data != CONNECT) break;
				tone(d5, tempo * half);
				if (data != CONNECT) break;
				tone(f4, tempo * one);
				if (data != CONNECT) break;
				tone(g4, tempo * one);
				if (data != CONNECT) break;

				tone(c5, tempo * half);
				if (data != CONNECT) break;
				tone(b4, tempo * half);
				if (data != CONNECT) break;
				tone(d4, tempo * one);
				if (data != CONNECT) break;
				tone(e4, tempo * one);
				if (data != CONNECT) break;

				tone(b4, tempo * half);
				if (data != CONNECT) break;
				tone(a4, tempo * half);
				if (data != CONNECT) break;
				tone(c4, tempo * one);
				if (data != CONNECT) break;
				tone(e4, tempo * one);
				if (data != CONNECT) break;
				tone(a4, tempo * two);
				break;
			default:
				while(state != end) {
					 //#1
					tone(d5,tempo * one);
					if (state == end) break;
					tone(g4,tempo * half);
					if (state == end) break;
					tone(a4,tempo * half);
					if (state == end) break;
					tone(b4,tempo * half);
					if (state == end) break;
					tone(c5,tempo * half);
					if (state == end) break;
					tone(d5,tempo * one);
					if (state == end) break;
					tone(g4,tempo * one);
					if (state == end) break;
					tone(g4,tempo * one);
					if (state == end) break;
					tone(e5,tempo * one);
					if (state == end) break;
					tone(c5,tempo * half);
					if (state == end) break;
					tone(d5,tempo * half);
					if (state == end) break;
					tone(e5,tempo * half);
					if (state == end) break;
					tone(fs5,tempo * half);
					if (state == end) break;
					tone(g5,tempo * one);
					if (state == end) break;
					tone(g4,tempo * one);
					if (state == end) break;
					tone(g4,tempo * one);
					if (state == end) break;

					//#2
					tone(c5,tempo * one);
					if (state == end) break;
					tone(d5,tempo * half);
					if (state == end) break;
					tone(c5,tempo * half);
					if (state == end) break;
					tone(b4,tempo * half);
					if (state == end) break;
					tone(a4,tempo * half);
					if (state == end) break;
					
					if (state == end) break;
					tone(b4,tempo * one);
					if (state == end) break;
					tone(c5,tempo * half);
					if (state == end) break;
					tone(b4,tempo * half);
					if (state == end) break;
					tone(a4,tempo * half);
					if (state == end) break;
					tone(g4,tempo * half);
					if (state == end) break;
					
					tone(fs4,tempo * one);
					if (state == end) break;
					tone(g4,tempo * half);
					if (state == end) break;
					tone(a4,tempo * half);
					if (state == end) break;
					tone(b4,tempo * half);
					if (state == end) break;
					tone(g4,tempo * half);
					if (state == end) break;
					tone(a4,tempo * three);
					if (state == end) break;
					
					//#3
					tone(d5,tempo * one);
					if (state == end) break;
					tone(g4,tempo * half);
					if (state == end) break;
					tone(a4,tempo * half);
					if (state == end) break;
					tone(b4,tempo * half);
					if (state == end) break;
					tone(c5,tempo * half);
					if (state == end) break;
					
					tone(d5,tempo * one);
					if (state == end) break;
					tone(g4,tempo * one);
					if (state == end) break;
					tone(g4,tempo * one);
					if (state == end) break;
					
					tone(e5,tempo * one);
					if (state == end) break;
					tone(c5,tempo * half);
					if (state == end) break;
					tone(d5,tempo * half);
					if (state == end) break;
					tone(e5,tempo * half);
					if (state == end) break;
					tone(fs5,tempo * half);
					if (state == end) break;
					
					tone(g5,tempo * one);
					if (state == end) break;
					tone(g4,tempo * one);
					if (state == end) break;
					tone(g4,tempo * one);
					if (state == end) break;
					
					//#4
					tone(c5,tempo * one);
					if (state == end) break;
					tone(d5,tempo * half);
					if (state == end) break;
					tone(c5,tempo * half);
					if (state == end) break;
					tone(b4,tempo * half);
					if (state == end) break;
					tone(a4,tempo * half);	
					if (state == end) break;
					
					tone(b4,tempo * one);
					if (state == end) break;
					tone(c5,tempo * half);
					if (state == end) break;
					tone(b4,tempo * half);
					if (state == end) break;
					tone(a4,tempo * half);
					if (state == end) break;
					tone(g4,tempo * half);
					if (state == end) break;
					
					tone(a4,tempo * one);
					if (state == end) break;
					tone(b4,tempo * half);
					if (state == end) break;
					tone(a4,tempo * half);
					if (state == end) break;
					tone(g4,tempo * half);
					if (state == end) break;
					tone(fs4,tempo * half);
					if (state == end) break;
					tone(g4,tempo * three);
				}
				tone(B, tempo * one);
				tone(E, tempo * one_half);
				tone(G, tempo * half);
				tone(F, tempo * one);
				tone(E, tempo * two);
				tone(B2, tempo * one);
				tone(A2, tempo * two_half);
				tone(Fs, tempo * two_half);

				tone(E, tempo * one_half);
				tone(G, tempo * half);
				tone(F, tempo * one);
				tone(Ds, tempo * two);
				tone(F, tempo * one);
				tone(B, tempo * two_half);
				break;
		}
		TPM0_C2V = 0;
		osDelay(50);
	}
}

int main (void) {
  SystemCoreClockUpdate();
	initPWM();
	initWheels();
	initGPIO();
	offRGB();
	initUART2(BAUD_RATE);
	
	
	wheelsMsg = osMessageQueueNew(2, sizeof(unsigned char), NULL);
	ledRedMsg = osMessageQueueNew(2, sizeof(unsigned char), NULL);
	ledGreenMsg = osMessageQueueNew(2, sizeof(unsigned char), NULL);
	musicMsg = osMessageQueueNew(2, sizeof(unsigned char), NULL);
	
	osKernelInitialize();
	osThreadNew(brain_thread, NULL, NULL);
	osThreadNew(wheels_thread, NULL, NULL);
	osThreadNew(music_thread, NULL, NULL);
	osThreadNew(led_red_thread, NULL, NULL);
  osThreadNew(running_led_green_thread, NULL, NULL); 
	osKernelStart();
	for (;;) {};
}

