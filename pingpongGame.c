// Lab 3
//      PA5 - SSI0Tx to SI
//      PA4 - SSI0Rx to 
//      PA3 - SSI0Fss to OC
//      PA2 - SSI0CLK to CL
//			PA6 - DC
//			PA7 - reset
//			PB2 - IR output


/*
Wireless Pong
USAGE: 	Push buttons on IR remote. Either the left paddle or right paddle will move depending on which player you are.
				The ball will be controlled by this Tiva, and the coordinates will be transmitted to the slave Tiva. The paddle1
				will also be transmitted to the slave Tiva. The slave Tiva will transmit paddle2 coordinates to this Tiva and
				will be updated here. The game is refreshed using the GPIO Timer at a rate of 5 times per second, updating ball
				and paddle coordinates.

*/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/rom.h"

// IR includes
#include "driverlib/timer.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"

#include "Adafruit_SSD1351.h"

// Codes used for decoding the button pushed
#define CTRL_ONE 1
#define CTRL_TWO 2
#define CTRL_THREE 3
#define CTRL_FOUR 4
#define CTRL_FIVE 5
#define CTRL_SIX 6
#define CTRL_SEVEN 7
#define CTRL_EIGHT 8
#define CTRL_NINE 9
#define CTRL_ZERO 10
#define CTRL_VUP 11
#define CTRL_VDOWN 12
#define CTRL_CHUP 13
#define CTRL_CHDOWN 14
#define CTRL_MUTE 15
#define CTRL_ENTER 16
#define CTRL_UP 17
#define CTRL_DOWN 18
#define CTRL_LEFT 19
#define CTRL_RIGHT 20
#define CTRL_ERROR -1

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

#define NUM_SSI_DATA            3
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

const int SPEED = 5;	// refresh rate = 1 sec / speed
const int BG_COLOR = BLACK;
const int ORIG_X_SPEED = 2;	// initial dx
const int ORIG_Y_SPEED = 1;	// initial dy

// IR detection
static int interrupted = 0;
static int edge1 = 1;
static int edge2 = 2;
static int total_time = 0;
static int count = 0;
static int times[52];
static int done_flag = 0;
static int last_time = 0;
static int start_flag = 0;

static int refresh = 0;

// Char detection and selection

static int display_now = 0;
static int error_sequence = 0;
static int error_address = 0;
static int print_code = 0;
static int valid = 0;

static int send_key = 0;
volatile int * Tx_ptr;
volatile bool Tx_done;

static int receive_index = 0;
static int receive_code[3] = {0,0,0};
static bool received = false;

static int move_paddle = 0;
static int pad_dir = 0;
static int update_paddles = 0;


// coordinates

static int ball_xc = 64;	// ball x coordinate
static int ball_yc = 64;
const int paddle1_xc = 0;
static int paddle1_yc = 54;
const int paddle2_xc = 121;	//127 - 6
static int paddle2_yc = 54;

static int ball_pxc = 64;	// previous ball x coordinate
static int ball_pyc = 64;
static int paddle1_pyc = 54;
static int paddle2_pyc = 54;

static int x_speed = ORIG_X_SPEED;
static int y_speed = ORIG_Y_SPEED;

static int gameover = 0;



void ConfigureUART(void);
void ConfigureUART1(void);
void UART1IntHandler(void);
void IR_Handler (void);
void SendStr( int * Tx_buf);
void decode(int times[], int size);
void oled_setup();
void restart(int winner);

void Timer1A_Int(void);

char char_selector(int code, int reps);

// ===========================================================================================
// Main

int main(void)
{
		ConfigureUART();
		ConfigureUART1();
    ROM_FPULazyStackingEnable();
		int ir_input;
		int send_code[3] = {0,0,0};
		uint32_t pui32DataTx[NUM_SSI_DATA];
		uint32_t pui32DataRx[NUM_SSI_DATA];
    uint32_t ui32Index;
		int char_num = 0;
	
		*Tx_ptr = NULL;
	
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);


		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
		// Button detection timer
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
		ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
		ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
		TimerEnable(TIMER0_BASE, TIMER_A);
		// character display timer
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
		ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
		ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()/SPEED);
		

		// SSI0 Stuff
		GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
		ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);
		GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
		GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
		GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
		ROM_SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);
		
		

		GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
		

		GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
		
		
		GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);

		GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_2);
		
		while(ROM_SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx[0]))
    {
    }
		
		
		
		IntEnable(INT_GPIOB);
		IntEnable(INT_TIMER1A);
		ROM_IntEnable(INT_UART1);
		ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
		
		TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
		
		IntMasterEnable();
		ROM_SSIEnable(SSI0_BASE);
		

    
		
		oled_setup();
		ROM_TimerEnable(TIMER1_BASE,TIMER_A);
		
		
// =================================================================================================
// While
		
		
    while(1)
    {               
      
			ROM_SysCtlSleep();
			
			
			// check which number pushed
			if (done_flag == 1)
			{
				decode(times, count);
				done_flag = 0;
				count = 0;
				start_flag = 0;
			}
			if (gameover != 0)
			{
				send_code[0] = 2;
				send_code[1] = gameover;
				SendStr(send_code);
				restart(gameover);
				gameover = 0;
			}
			
			if (refresh)
			{
				fillCircle(ball_pxc,ball_pyc,2,BG_COLOR);
				fillCircle(ball_xc,ball_yc,2,WHITE);
				send_code[0] = 3;
				send_code[1] = ball_xc;
				send_code[2] = ball_yc;
				SendStr(send_code);
				ROM_SysCtlDelay(SysCtlClockGet()/3/1000);
				if (print_code == CTRL_VUP || print_code == CTRL_VDOWN)
				{
					move_paddle = 1;
					if (print_code == CTRL_VUP)
						pad_dir = -10;
					else
						pad_dir = 10;
					print_code = 0;
				}
				if (update_paddles)
				{
					fillRect(paddle1_xc,paddle1_pyc,6,20,BG_COLOR);
					fillRect(paddle1_xc,paddle1_yc,6,20,WHITE);
					fillRect(paddle2_xc,paddle2_pyc,6,20,BG_COLOR);
					fillRect(paddle2_xc,paddle2_yc,6,20,WHITE);
					send_code[0] = 4;
					send_code[1] = paddle1_yc;
					send_code[2] = 0;
					SendStr(send_code);
					update_paddles = 0;
				}
				if (receive_code[0] == 5)
				{
					paddle2_pyc = paddle2_yc;
					paddle2_yc = receive_code[1];
					update_paddles = 1;
					receive_code[0] = 0;
				}
				refresh = 0;
			}

			

			GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_2);
			TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
			GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_2);
    }
}


// ==================================================================================================
// Functions
// ==================================================================================================


void oled_setup()
{
	char msg[12] = "Starting...\n";
	begin();
	fillScreen(BG_COLOR);
	setTextColor(WHITE,BG_COLOR);
	setTextSize(1);
	setCursor(24, 64);
	for (int i = 0;i < 12; i++)
	{
		write(msg[i]);
	}
	ROM_SysCtlDelay(ROM_SysCtlClockGet()/3);
	fillScreen(BG_COLOR);
	fillCircle(ball_xc,ball_yc,2,WHITE);
	fillRect(paddle1_xc,paddle1_yc,6,20,WHITE);
	fillRect(paddle2_xc,paddle2_yc,6,20,WHITE);


}

void restart(int winner)
{
	char msg[11]= "Winner: P1\n";
	ball_xc = 64;
	ball_yc = 64;
	ball_pxc = 64;
	ball_pyc = 64;
	paddle1_yc = 54;
	paddle1_pyc = 54;
	fillScreen(BG_COLOR);
	setCursor(24, 64);
	if (winner == 1)
		msg[9] = '2';
	else
		msg[9] = '1';
	for (int i = 0;i < 11; i++)
	{
		write(msg[i]);
	}
	ROM_SysCtlDelay(ROM_SysCtlClockGet()/3);
	fillScreen(BG_COLOR);
	fillCircle(ball_xc,ball_yc,2,WHITE);
	fillRect(paddle1_xc,paddle1_yc,6,20,WHITE);
	fillRect(paddle2_xc,paddle2_yc,6,20,WHITE);
}


void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 9600, 16000000);
}

void ConfigureUART1(void) {
	
		// Enable the GPIO Peripheral used by the UART1 and enable UART1
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);  
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

	
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
    ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    // Initialize UART1
    //
		ROM_UARTConfigSetExpClk(UART1_BASE, 16000000, 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
}




void UART1IntHandler(void) {
    uint32_t ui32Status;
		
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_PIN_2);

    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);
	
    ROM_UARTIntClear(UART1_BASE, ui32Status);

	// send
		while(ROM_UARTSpaceAvail(UART1_BASE))
		{
			if (*Tx_ptr) {

				// We can use NonBlocking Put since we know space is available.
			UARTprintf("send1\n");
				ROM_UARTCharPutNonBlocking(UART1_BASE, *Tx_ptr++);
			} else {
				UARTprintf("send2\n");
				Tx_done = true;
		    ROM_UARTIntDisable(UART1_BASE, UART_INT_TX);
				break;
			}
		}
		UARTprintf("between\n");
	// receive
    while(ROM_UARTCharsAvail(UART1_BASE))
    {
        receive_code[receive_index] = ROM_UARTCharGetNonBlocking(UART1_BASE);
				UARTprintf("receive %c\n",receive_code[receive_index]);
				receive_index++;
				
    }
		received = true;
		receive_index = 0;
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,0);

}



void SendStr( int * Tx_buf) {

		Tx_done = false;		// global flag used by ISR
		Tx_ptr = Tx_buf;		// global pointer used by ISR
		ROM_UARTIntDisable(UART1_BASE, UART_INT_TX);	// avoid critical section
		while(ROM_UARTSpaceAvail(UART1_BASE))
		{
			if (*Tx_ptr) {
				ROM_UARTCharPutNonBlocking(UART1_BASE, *Tx_ptr++);
			} else {
				break;
			}
			
		}
		
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}

void Timer1A_Int(void)
{
	//TimerIntDisable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	ball_pxc = ball_xc;
	ball_pyc = ball_yc;
	
	// Control Ball
	ball_xc = ball_xc + x_speed;
	ball_yc = ball_yc + y_speed;
	
	if (ball_xc < 2) // left screen edge
		gameover = 1;
	
	if (ball_xc > 125)	// right screen edge
		gameover = 2;
	
	if (ball_yc < 3 || ball_yc > 124)
		y_speed = -1 * y_speed;
	
	if (ball_xc < 9)
	{
		if (ball_yc > paddle1_yc && ball_yc < (paddle1_yc+20))
			x_speed = ORIG_X_SPEED;
	}
	if (ball_xc > 117)
	{
		if (ball_yc > paddle2_yc && ball_yc < (paddle2_yc+20))
			x_speed = -1 * ORIG_X_SPEED;
	}
	
	// Control paddle 1
	if (move_paddle == 1)
	{
		paddle1_pyc = paddle1_yc;
		paddle1_yc += pad_dir;
		if (paddle1_yc > 107 || paddle1_yc < 0)
			paddle1_yc = paddle1_pyc;
		move_paddle = 0;
		update_paddles = 1;
	}
	
	refresh = 1;
	//TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}


void IR_Handler (void) {
	// Disable interrupt for awhile to avoid switch bounce
	GPIOIntDisable(GPIO_PORTB_BASE, GPIO_INT_PIN_2);

	// Clear interrupt request
	GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_2);
	
// Get pulse width
	edge2 = edge1;
	edge1 = TimerValueGet(TIMER0_BASE,TIMER_A);
	total_time = edge2 - edge1;
	if (total_time < 0)
		total_time = total_time + SysCtlClockGet();
	
// assign time	
	if (count < 60)
		times[count] = total_time;
	else
		done_flag = 1;
	
// check if continuously held
	if (last_time != 425000)
	{
		if (total_time > 400000 && total_time < 410000)
		{
			start_flag = 1;
		}
		else
		{
			if ((last_time + total_time) > 420000 && (last_time + total_time) < 430000)
				start_flag = 1;
		}
	}
	if (total_time > 420000 && total_time < 430000)
	{
		total_time = 425000;
		//done_flag = 1;
	}
	else
	{
		if ((last_time + total_time) > 420000 && (last_time + total_time) < 430000)
		{
			total_time = 425000;
			//done_flag = 1;
		}
	}
if (start_flag)
		count++;

	last_time = total_time;
}	


void decode(int times[], int size)
{
	int value[60];
	int index = 0;
	int start = 0;
	int sequence = 0;
	int address = 0;
	
	
			for (int i=0; i<size;i++)
			{
				if (times[i] > 123000 && times[i] < 127000)
				{
					value[index] = 1;
					index = index + 1;
				}
		else
		{
			if (times[i] > 73000 && times[i] < 77000)
			{
				value[index] = 0;
				index = index + 1;
			}
			else
			{
				if ((times[i]+times[i+1]) > 123000 && (times[i]+times[i+1]) < 127000)
				{
					value[index] = 1;
					index = index+1;
					i++;
				}
				else
				{
					if ((times[i]+times[i+1]) > 73000 && (times[i]+times[i+1]) < 77000)
					{
						value[index] = 0;
						index = index+1;
						i++;
					}
					else
					{
						value[index] = -1;
						index = index+1;
					}
				}
			}
		}
			
	

	} // for(int i=0; i<size;i++)
		
	
	// check for 1111 ---- ---- 0000
	for (int i=0; i<index;i++)
	{
		if (value[i] == 1)
		{
			start = 1;
			for (int j=0; j<4;j++)
			{
				if (value[i+j] != 1)
					start = 0;
			}
			if (start == 1)
			{
				for (int j=0; j<4;j++)
				{
					if (value[i+j+12] != 0)
					start = 0;
				}
			}
		}
		if (start == 1)
		{
			sequence = 0;

			for (int j = 8; j<12;j++)
			{
				sequence = sequence * 10 + value[i+j];
			}
			for (int j = 4; j < 8; j++)
			{
				address = address * 10 + value[i+j];
			}
			if (address == 11)
			{
				switch(sequence)
				{
					case 0:
						print_code = CTRL_ZERO;
						break;
					case 1:
						print_code = CTRL_ONE;
						break;
					case 10:
						print_code = CTRL_TWO;
						break;
					case 11:
						print_code = CTRL_THREE;
						break;
					case 100:
						print_code = CTRL_FOUR;
						break;
					case 101:
						print_code = CTRL_FIVE;
						break;
					case 110:
						print_code = CTRL_SIX;
						break;
					case 111:
						print_code = CTRL_SEVEN;
						break;
					case 1000:
						print_code = CTRL_EIGHT;
						break;
					case 1001:
						print_code = CTRL_NINE;
						break;
					case 1111:
						print_code = CTRL_MUTE;
						break;
					default:
						print_code = CTRL_ERROR;
						error_sequence = sequence;
						error_address = address;
						address = 0;
						sequence = 0;
				}
			} // if address == 11
			if (address == 101)
			{
					
				switch(sequence)
				{
					case 110:
						print_code = CTRL_LEFT;
						break;
					case 111:
						print_code = CTRL_RIGHT;
						break;
					case 1001:
						print_code = CTRL_UP;
						break;
					case 1000:
						print_code = CTRL_DOWN;
						break;
					default:
						print_code = CTRL_ERROR;
						error_sequence = sequence;
						error_address = address;
						address = 0;
						sequence = 0;
				}
			} // if address == 101
			if (address == 10)
			{
				switch(sequence)
				{
					case 1111:
						print_code = CTRL_VUP;
						break;
					case 1110:
						print_code = CTRL_VDOWN;
						break;
					case 1101:
						print_code = CTRL_CHUP;
						break;
					case 1100:
						print_code = CTRL_CHDOWN;
						break;
					default:
						print_code = CTRL_ERROR;
						error_sequence = sequence;
						error_address = address;
						address = 0;
						sequence = 0;
				}
			} // if address == 10
			if (address == 1111)
			{
				switch(sequence)
				{
					case 100:
						print_code = CTRL_ENTER;
						break;
					default:
						print_code = CTRL_ERROR;
						error_sequence = sequence;
						error_address = address;
						address = 0;
						sequence = 0;
				}
			}
			ROM_SysCtlDelay(SysCtlClockGet()/3/10000);
			if (print_code != CTRL_ERROR && print_code != 0)
				valid = 1;
		}
	} // (int i=0; i<size;i++)
}


char char_selector(int code, int reps)
{
	char temp;
	switch(code)
	{
		case CTRL_ONE:
					temp = '1';
			break;
		case CTRL_TWO:
			switch(reps)
			{
				case 1:
					temp = 'a';
					break;
				case 2:
					temp = 'b';
					break;
				case 3:
					temp = 'c';
					break;
				default:
					temp = '2';
			}
			break;
		case CTRL_THREE:
			switch(reps)
			{
				case 1:
					temp = 'd';
					break;
				case 2:
					temp = 'e';
					break;
				case 3:
					temp = 'f';
					break;
				default:
					temp = '3';
			}
			break;
		case CTRL_FOUR:
			switch(reps)
			{
				case 1:
					temp = 'g';
					break;
				case 2:
					temp = 'h';
					break;
				case 3:
					temp = 'i';
					break;
				default:
					temp = '4';
			}
			break;
		case CTRL_FIVE:
			switch(reps)
			{
				case 1:
					temp = 'j';
					break;
				case 2:
					temp = 'k';
					break;
				case 3:
					temp = 'l';
					break;
				default:
					temp = '5';
			}
			break;
		case CTRL_SIX:
			switch(reps)
			{
				case 1:
					temp = 'm';
					break;
				case 2:
					temp = 'n';
					break;
				case 3:
					temp = 'o';
					break;
				default:
					temp = '6';
			}
			break;
		case CTRL_SEVEN:
			switch(reps)
			{
				case 1:
					temp = 'p';
					break;
				case 2:
					temp = 'q';
					break;
				case 3:
					temp = 'r';
					break;
				case 4:
					temp = 's';
					break;
				default:
					temp = '7';
			}
			break;
		case CTRL_EIGHT:
			switch(reps)
			{
				case 1:
					temp = 't';
					break;
				case 2:
					temp = 'u';
					break;
				case 3:
					temp = 'v';
					break;
				default:
					temp = '8';
			}
			break;
		case CTRL_NINE:
			switch(reps)
			{
				case 1:
					temp = 'w';
					break;
				case 2:
					temp = 'x';
					break;
				case 3:
					temp = 'y';
					break;
				case 4:
					temp = 'z';
					break;
				default:
					temp = '9';
			}
			break;
		case CTRL_ZERO:
			switch(reps)
			{
				case 1:
					temp = ' ';
					break;
				default:
					temp = '0';
			}
			break;
			
			
		case CTRL_VUP:
			break;
		case CTRL_VDOWN:
			break;
		case CTRL_CHUP:
			break;
		case CTRL_CHDOWN:
			break;
		case CTRL_UP:
			break;
		case CTRL_DOWN:
			break;
		case CTRL_LEFT:
			break;
		case CTRL_RIGHT:
			break;
		case CTRL_MUTE:
			temp = '\r';
			send_key = 1;
			break;
		case CTRL_ENTER:
			temp = '\r';
			send_key = 1;
			break;
		default:
			temp = '?';
	}
	return temp;
}
