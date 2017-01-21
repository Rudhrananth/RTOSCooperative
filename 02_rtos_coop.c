
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 4 Pushbuttons and 4 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for green and yellow LEDs (temporary to guarantee compilation)
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))
#define ORANGE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))
#define YELLOW_LED    (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4)))

#define PUSH_BUTTON_1  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))
#define PUSH_BUTTON_2  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define PUSH_BUTTON_3  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define PUSH_BUTTON_4  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_QUEUE_SIZE 10
struct semaphore
{ int windex;
  int rindex;
  unsigned int count;
  unsigned int queueSize;
  int processQueue[MAX_QUEUE_SIZE]; // store task index here
} *s, keyPressed, keyReleased, flashReq;

// task
#define STATE_INVALID    0 // no task
#define STATE_READY      1 // ready to run
#define STATE_BLOCKED    2 // has run, but now blocked by semaphore
#define STATE_DELAYED    3 // has run, but now awaiting timer

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

struct _tcb
{
  uint8_t state;                 // see STATE_ values above
  void *pid;                     // used to uniquely identify thread
  void *sp;                      // location of stack pointer for thread
  uint8_t priority;              // 0=highest, 7=lowest
  uint8_t currentPriority;       // used for priority inheritance
  uint32_t ticks;                // ticks until sleep complete
  char name[16];                 // name of task used in ps command
  void* semaphore;               // pointer to the semaphore that is blocking the thread
  uint8_t skipcount;
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];
bool magnetneed;
void magnet();
////////////////////////////////////////////////////////////////////////////////////////////////////////////
const uint8_t max=80;
char str[max+1]={NULL};
uint8_t position[10]={NULL};
uint8_t parsecount;
uint8_t ascstr[max+1];
////////////////////////////////////////////////////////////////////////////////////////////////////////////

int getSP()
{
__asm(" MOV R0,R13 ");
return;
}


int putSP(int g)
{
__asm(" MOV R13,R0 ");
__asm(" SUB R13,#0x8 ");
return;
}

isCommand(char fieldName[10],int minArgs)
{
	char *str1 = &str[position[0]];
	char *str2=fieldName;
	if (!strcmp(str1,str2))
	{
		if((parsecount-1)== minArgs)
			return true;
		else
			return false;
	}
	else
		return false;
}

isAlpha(int fieldNo)
{
	 uint8_t i;
	 uint8_t j=0;
	 char *argType = &str[position[fieldNo]];
	 for(i=0; i< strlen(argType); i++)
	 	{
	 		if ((ascstr[position[fieldNo]+i])  >= 65 && (ascstr[position[fieldNo]+ i]) <= 90)
	 		j++;
	 	}
	 	if (j == strlen(argType))
	 		 return true;
	  	 else
	  		return false;
}


isNumber(int fieldNo)
 {
	 uint8_t i;
	 uint8_t j=0;
	 char *argType;
	 argType=&str[position[fieldNo]];
	for(i=0; i< strlen(argType); i++)
	{
		if ((ascstr[position[fieldNo]+i])  >= 48 && (ascstr[position[fieldNo]+ i]) <= 57)
		j++;
	}
	if (j == strlen(argType))
		 return true;
	 else
		return false;
 }

uint8_t getNumber(int fieldNo)
{
	char *arg = &str[position[fieldNo]];
	uint8_t i;
	i=atoi(arg);
	return i;
}

char* getAlpha(int fieldNo)
{
	 char* arg;
	 arg= &str[position[fieldNo]];
	 return arg;
}
//-----------------------------------------------------------------------------
// RTOS Kernel
//-----------------------------------------------------------------------------

void rtosInit()
{
  uint8_t i;
  // no tasks running
  taskCount = 0;
  // clear out tcb records
  for (i = 0; i < MAX_TASKS; i++)
  {
    tcb[i].state = STATE_INVALID;
    tcb[i].pid = 0;
  }
  // REQUIRED: systick for 1ms system timer
  NVIC_ST_RELOAD_R = 39999;
  NVIC_ST_CURRENT_R = 0X1;
  NVIC_ST_CTRL_R |= 0x7; //0111
}

int rtosScheduler()
{
  // REQUIRED: Implement prioritization to 16 levels
  bool ok;
  bool stateready;
  static uint8_t task = 0xFF;
  ok = false;
  while (!ok)
  {
    task++;
    if (task >= MAX_TASKS)
      task = 0;
  stateready = (tcb[task].state == STATE_READY);
  if(stateready)
  	{
  	  if(tcb[task].skipcount == 0)
  	  {
  		  tcb[task].skipcount = tcb[task].priority;
  		ok = true;
  	  }
  	  else tcb[task].skipcount--;
    }
  }

  return task;
}

//int rtosScheduler()
//{
//  // REQUIRED: Implement prioritization to 16 levels
//  bool ok;
//  static uint8_t task = 0xFF;
//  ok = false;
//  while (!ok)
//  {
//    task++;
//    if (task >= MAX_TASKS)
//      task = 0;
//    ok = (tcb[task].state == STATE_READY);
//  }
//  return task;
//}

bool createThread(_fn fn, char name[], int priority)
{
  bool ok = false;
  uint8_t i = 0;
  bool found = false;
  // REQUIRED: store the thread name
  // REQUIRED: take steps to ensure a task switch cannot occur
  // save starting address if room in task list
  if (taskCount < MAX_TASKS)
  {
    // make sure fn not already in list (prevent reentrancy)
    while (!found && (i < MAX_TASKS))
    {
      found = (tcb[i++].pid ==  fn);
    }
    if (!found)
    {
      // find first available tcb record
      i = 0;
      while (tcb[i].state != STATE_INVALID) {i++;}
      tcb[i].state = STATE_READY;
      tcb[i].pid = fn;
      strcpy(tcb[i].name, name);

      // REQUIRED: preload stack to look like the task had run before
//      stack[i][255]= 0; //faker3
//      stack[i][254]= 0; //fakelr
      stack[i][255]= fn; //r14
      stack[i][254]= 11; //r11
      stack[i][253]= 10; //r10
      stack[i][252]= 9; //r9
      stack[i][251]= 8; //r8
      stack[i][250]= 7; //r7
      stack[i][249]= 6; //r6
      stack[i][248]= 5; //r5
      stack[i][247]= 4; //r4
      tcb[i].sp = &stack[i][247]; // REQUIRED: + offset as needed for the pre-loaded stack
      tcb[i].priority = priority;
      tcb[i].skipcount = priority;
      tcb[i].currentPriority = priority;
      // increment task count
      taskCount++;
      ok = true;
    }
  }
  // REQUIRED: allow tasks switches again
  return ok;
}

// REQUIRED: modify this function to destroy a process
void destroyProcess(_fn fn)
{
	 uint8_t task = 0xFF;
	  bool ok = false;
	  while (!ok)
	  {
	    task++;
	    if (task >= MAX_TASKS)
	      task = 0;
	    ok = (tcb[task].pid == fn);
	  }
//	  int ik= strcmp(tcb[task].name, "Idle");
	  if(tcb[task].state == STATE_BLOCKED)
	  {
		  uint8_t i=0;
	       s =tcb[task].semaphore;
		  while(s->processQueue[i] != task)
		  {
		  i++;
		  }
		  s->processQueue[i]= -1;
		  tcb[task].state = STATE_INVALID;
		  tcb[task].pid = 0;
		  taskCount--;
	  }

		  else if(((tcb[task].state == 1) || (tcb[task].state == 3) ) && strcmp(tcb[task].name, "Idle"))
		  {
		  tcb[task].state = STATE_INVALID;
	      tcb[task].pid = 0;
	      taskCount--;
		  }
	      else
	    	  putsUart0("\n\r Can not delete");
}

void rtosStart()
{
  // REQUIRED: add code to call the first task to be run, restoring the preloaded context
  _fn fn;
  taskCurrent = rtosScheduler();
  // Add code to initialize the SP with tcb[task_current].sp;
  // Restore the stack to run the first process
  	putSP(tcb[taskCurrent].sp); //change SP to taskCurrent's sp
	__asm(" POP {R4-R11} ");
	__asm(" POP {R14} ");   //LR
	__asm(" BX LR ");
}

void init(void* p, int count)
{
  s = p;
  s->count = count;
  s->queueSize = 0;
  s->rindex = 0;
  s->windex = 0;
}

// REQUIRED: modify this function to yield execution back to scheduler
void yield()
{
	// push registers, call scheduler, pop registers, return to new function
	__asm(" POP {R3} ");
	__asm(" PUSH {R4-R11} ");
	tcb[taskCurrent].sp= getSP(); //R13

	taskCurrent = rtosScheduler();

	putSP(tcb[taskCurrent].sp); //change SP to next task's sp
	__asm(" POP {R4-R11} ");
	__asm(" POP {R14} ");   //LR
	__asm(" BX LR ");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses
void sleep(uint32_t tick)
{
	tcb[taskCurrent].ticks= tick;
	tcb[taskCurrent].state= STATE_DELAYED;
	__asm(" POP {R3} ");
	// push registers, set state to delayed, store timeout, call scheduler, pop registers,
	// return to new function (separate unrun or ready processing)
	__asm(" PUSH {R4-R11} ");
	tcb[taskCurrent].sp= getSP(); //R13

	taskCurrent = rtosScheduler();

	putSP(tcb[taskCurrent].sp); //change SP to next task's sp
	__asm(" POP {R4-R11} ");
	__asm(" POP {R14} ");   //LR
	__asm(" BX LR ");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler
void wait(void* pSemaphore)
{
	s = pSemaphore;
	if((s->count) == 0)
	tcb[taskCurrent].semaphore = pSemaphore;
	__asm(" POP {R3} ");
	__asm(" PUSH {R4-R11} ");
	tcb[taskCurrent].sp= getSP();
	
	if((s->count) > 0)
	(s->count)--;
	else
	{
	tcb[taskCurrent].state = STATE_BLOCKED;
//	tcb[taskCurrent].semaphore = pSemaphore;
	s->processQueue[s->windex] = taskCurrent;
	s->windex = (s->windex +1) % MAX_QUEUE_SIZE;
	s->queueSize = (s->queueSize +1) % MAX_QUEUE_SIZE;
	taskCurrent = rtosScheduler();
	}

	putSP(tcb[taskCurrent].sp);
	__asm(" POP {R4-R11} ");
	__asm(" POP {R14} ");
	__asm(" BX LR ");

}

// REQUIRED: modify this function to signal a semaphore is available
void post(void* pSemaphore)
{

	s = pSemaphore;
	__asm(" POP {R3} ");
	__asm(" PUSH {R4-R11} ");
	tcb[taskCurrent].sp= getSP();

	s->count++;

	if( (s->count > 0) && (s->queueSize > 0) )
		{
			if(s->processQueue[s->rindex] == -1)
			s->rindex = (s->rindex +1) % MAX_QUEUE_SIZE;
			if(tcb[s->processQueue[s->rindex]].state == STATE_BLOCKED)
			{
			tcb[s->processQueue[s->rindex]].state = STATE_READY;
			s->rindex = (s->rindex +1) % MAX_QUEUE_SIZE;
			s->queueSize = (s->queueSize -1);
			s->count--;
		    }
		}

	putSP(tcb[taskCurrent].sp);
	__asm(" POP {R4-R11} ");
	__asm(" POP {R14} ");
	__asm(" BX LR ");


}

// REQUIRED: modify this function to add support for the system timer
void systickIsr()
{  uint8_t i;
	for(i=0; i < MAX_TASKS; i++)
	{
		if( tcb[i].state == STATE_DELAYED)
		{
		if((tcb[i].ticks)!= 0)
		(tcb[i].ticks)--;
		else
		tcb[i].state = STATE_READY;
		}
	}
}

// REQUIRED: modify this function to add support for the service call
void svcCallIsr()
{
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // REQUIRED: Add initialization for orange, red, green, and yellow LEDs
    //           4 pushbuttons, and uart
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use AP (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE;

    // Configure LED and pushbutton pins
    GPIO_PORTC_DIR_R &= ~0xF0;  // bits 4,5,6,7 are inputs, other pins unchanged
    GPIO_PORTD_DIR_R |= 0x0F;   // bits 1,2,3,4 are outputs, other pins unchanged
    GPIO_PORTC_DR2R_R = 0xF0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTD_DR2R_R = 0x0F;
    GPIO_PORTC_DEN_R = 0xF0;  // enable LEDs and pushbuttons
    GPIO_PORTD_DEN_R = 0x0F;
    GPIO_PORTC_PUR_R = 0xF0;  // enable internal pull-up for push button

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

        // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
      UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
      UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
      UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
      UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
      UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
      UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

      // Configure AN10,9,8,11 as an analog input
       SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
       GPIO_PORTE_AFSEL_R |= 0x30;                      //110000 for pe4(an9),pe5(an8)    // select alternative functions for AN0 (PE3)
       GPIO_PORTE_DEN_R &= ~0x30;                       // turn off digital operation on pin PE3
       GPIO_PORTE_AMSEL_R |= 0x30;                      // turn on analog operation on pin PE3
       GPIO_PORTB_AFSEL_R |= 0x30;                      //110000 for pb4(an10),pb5(an11)    // select alternative functions for AN0 (PE3)
       GPIO_PORTB_DEN_R &= ~0x30;                       // turn off digital operation on pin PE3
       GPIO_PORTB_AMSEL_R |= 0x30;
       ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
       ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN2;                // disable sample sequencer 2 (SS2) for programming
       ADC0_EMUX_R = ADC_EMUX_EM2_PROCESSOR;            //ADC_EMUX_EM2_PROCESSOR;   // select SS3 bit in ADCPSSI as trigger
       ADC0_SSMUX2_R = 0XBA98;                          // set first sample to AN0
       ADC0_SSCTL2_R = ADC_SSCTL2_END3;                 // mark first sample as the end
       ADC0_ACTSS_R |= ADC_ACTSS_ASEN2;                 // enable SS2 for operation
}


// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
	                                            // Approx clocks per us
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-15 indicating which of 4 PBs are pressed
uint8_t readPbs()
{
	int i = ((!PUSH_BUTTON_1)<< 0) + ((!PUSH_BUTTON_2)<<1)  + ((!PUSH_BUTTON_3)<<2) + ((!PUSH_BUTTON_4)<<3);
	return i;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char tempc)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = tempc;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* tempstr)
{
	uint8_t l;
    for (l = 0; l < strlen(tempstr); l++)
	  putcUart0(tempstr[l]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while(UART0_FR_R & UART_FR_RXFE)
	{yield();}
	return UART0_DR_R & 0xFF;
}

int16_t readAdc0Ss2()
{
    ADC0_PSSI_R |= ADC_PSSI_SS2;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY) ;          // wait until SS3 is not busy
    return ADC0_SSFIFO2_R;                           // get single result from the FIFO
}


// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
  while(true)
  {
    ORANGE_LED = 1;
    waitMicrosecond(1000);
    ORANGE_LED = 0;
    yield();
  }
}


void flash4Hz()
{
  while(true)
  {
    GREEN_LED ^= 1;
    sleep(125);
  }
}


void oneshot()
{
  while(true)
  {
    wait(&flashReq);
    YELLOW_LED = 1;
    sleep(1000);
    YELLOW_LED = 0;
  }
}

void partOfLengthyFn()
{
  // represent some lengthy operation
  waitMicrosecond(1000);
  // give another process a chance
  yield();
}

void lengthyFn()
{
  uint16_t i;
  while(true)
  {
    for (i = 0; i < 4000; i++)
    {
      partOfLengthyFn();
    }
    RED_LED ^= 1;
  }
}

void readKeys()
{
  uint8_t buttons;
  while(true)
  {
    wait(&keyReleased);
    buttons = 0;
    while (buttons == 0)
    {
      buttons = readPbs();
      yield();
    }
    post(&keyPressed);
    if ((buttons & 1) != 0)
    {
      YELLOW_LED ^= 1;
      RED_LED = 1;
    }
    if ((buttons & 2) != 0)
    {
      post(&flashReq);
      RED_LED = 0;
    }
    if ((buttons & 4) != 0)
    {
      createThread(flash4Hz, "Flash_4hz", 0);
    }
    if ((buttons & 8) != 0)
    {
      destroyProcess(flash4Hz);
	}

    yield();
  }
}

void debounce()
{
  uint8_t count;
  while(true)
  {
    wait(&keyPressed);
    count = 10;
    while (count != 0)
    {
      sleep(10);
      if (readPbs() == 0)
        count--;
      else
        count = 10;
    }
    post(&keyReleased);
  }
}

void uncooperative()
{
  while(true)
  {
    while (readPbs() == 8)
    {
    }
    yield();
  }
}

void shell()
{
  while (true)
  {
	  // REQUIRED: add processing for the shell commands here

	  uint8_t i;
	  uint8_t j;
	  uint8_t k;
	  parsecount=0;
	  uint8_t posDelim[80];
	  uint8_t delimCount=0;
	  char type[10];


	  memset(&str[0], 0, sizeof(str));
	  uint8_t count=0;
	  putsUart0(" \n \r Enter your Command:");

		while(count < max)
		{
				uint8_t c = getcUart0();
				putcUart0(c);
			if (c == 8)
		    {
		    	if (count>0)
		    		count--;
		    }
			else if (c == 13)
			{
				str[count]= NULL;
				break;
		    }
			else if (c >= 32)
			{
				str[count++]=c;
				if (count == max)
				{
					str[count]=NULL;
					break;
				}
			}
		}

		memset(&position[0], 0, sizeof(position));
			for(j=0;j<=strlen(str); j++)
			{
//				str[j]=toupper(str[j]);
				ascstr[j]=str[j];
			}
			if ((ascstr[0] >= 65 && ascstr[0]<= 90)  || (ascstr[0] == 95) || (ascstr[0] >= 97 && ascstr[0]<= 122) || (ascstr[0] == 38))
			{
			        type[parsecount] = 'a';
			        position[parsecount] = 0;
			        parsecount++;
			}
				else if (ascstr[0] >= 48 && ascstr[0]<= 57 )
				{
					type[parsecount] = 'n';
					position[parsecount] = 0;
					parsecount++;
				}
			for (i=0; i< strlen(str); i++)
			{
				if(( ascstr[i] >= 32 && ascstr[i]<= 37 )|| ( ascstr[i] >= 39 && ascstr[i]<= 47 ) || ( ascstr[i] >= 58 && ascstr[i]<= 64 )|| ( ascstr[i] >= 91 && ascstr[i]<= 94) || (ascstr[i] == 96) || ( ascstr[i] >= 123 && ascstr[i]<= 255 ) )
				{
						posDelim[delimCount]=i;
						delimCount++;
						if((ascstr[i+1] >= 65 && ascstr[i+1]<= 90 )|| (ascstr[i+1] >=97 && ascstr[i+1] <= 122))
						{
							type[parsecount] = 'a';
						    position[parsecount] = i+1;
						    parsecount++;
						}
						else if( ascstr[i+1] >= 48 && ascstr[i+1]<= 57)
						{
							type[parsecount] = 'n';
							position[parsecount] = i+1;
							parsecount++;
						}
				}
			}

			for(k=0; k<delimCount; k++)
			{
				str[posDelim[k]]=NULL;
			}


				 char displayRxd[80]={NULL};
				 uint8_t validCommand= false; //bool
				 int temi;
				 char *istr1 = &str[position[0]];

	               uint8_t teni;
	                uint8_t si;

				 if(isCommand("ps",0))
						{
						putsUart0("\n\r PID \t \t NAME \t \t \t STATE \n\r");
					for(temi=0; temi< MAX_TASKS; temi++)
					 {
//						&& tcb[temi].pid != 0
						if(tcb[temi].state != 0 )
							{
							sprintf(displayRxd,"\n\r 0X%x \t \t %s \t \t \t \t %d \n\r",tcb[temi].pid,tcb[temi].name,tcb[temi].state);
							putsUart0(displayRxd);
							}
					}
					validCommand= true;
					}

//				 if(isCommand("ps",0))
//					{
//					 putsUart0("\n\r PID \t \t NAME \t \t CPU cycle \t STATE \n\r");
//					 for(temi=0; temi<taskCount; temi++)
//					 {
//
//						 sprintf(displayRxd,"\n\r 0X%x \t \t %s \t \t *** \t \t %d \n\r",tcb[temi].pid,tcb[temi].name,tcb[temi].state);
//						 putsUart0(displayRxd);
//					 }
//					validCommand= true;
//					}

				 else if(isCommand("pidof",1))
				{
					 char *tempj;
					 tempj=getAlpha(1);
						for(temi=0; temi< taskCount; temi++)
						{
						if(!strcmp(tempj,tcb[temi].name))
								{
							 sprintf(displayRxd,"\n \r PID is 0X%x \n",tcb[temi].pid);
							 putsUart0(displayRxd);
								validCommand= true;
								}
						}
				}

				 else if(isCommand("reboot",0))
					{
					 NVIC_APINT_R = NVIC_APINT_VECTKEY | 0x4;
						validCommand= true;
					}

				  else if(isCommand("magnet",0))
				   {
				             validCommand= true;
				             magnetneed = true;
				   }

				 else if(isCommand("kill",1))
				{
					 char *tempi;
					 tempi=getAlpha(1);
					 int hex = strtol(tempi, NULL, 16);
//					 _fn fni = hex;
					 destroyProcess(hex);
				validCommand= true;
				}

//				alltask[][]
//				 uint8_t tik;
//				 for(tik=0; tik<MAX_TASKS; tik++)
//				 {
//					 char *tikc;
//					 if(tcb[tik].state != 0)
//					 {
//						 strcat(tikc,"&");
//						 if(!strcmp(istr1,tikc))
//						 {
//						 validCommand= true;
//						 createThread(idle,tcb[tik].name, 15);
//						 }
//					 }
//				 }

				 //Cannot create new task without hardcoding if the task is removed from the TCB or being added for the first time//
				 else if(!strcmp(istr1,"Idle&"))
				 {
				 validCommand= true;
				 createThread(idle, "Idle", 15);
				 }
				 else  if(!strcmp(istr1,"Flash_4hz&"))
				 {
					 validCommand= true;
					 createThread(flash4Hz, "Flash_4hz", 0);
				 }
				 else  if(!strcmp(istr1,"Lengthy_fn&"))
				 {
					 validCommand= true;
					 createThread(lengthyFn, "Lengthy_fn", 10);
				 }
				 else  if(!strcmp(istr1,"One_shot&"))
				 {
					 validCommand= true;
					 createThread(oneshot, "One_shot", 4);
				 }
				 else  if(!strcmp(istr1,"Read_keys&"))
				 {
					 validCommand= true;
					 createThread(readKeys, "Read_keys", 2);
				 }
				 else  if(!strcmp(istr1,"Debounce&"))
				 {
					 validCommand= true;
					 createThread(debounce, "Debounce", 6);
				 }
				 else  if(!strcmp(istr1,"Uncoop&"))
				 {
					 validCommand= true;
					 createThread(uncooperative, "Uncoop", 7);
				 }
				 else  if(!strcmp(istr1,"Shell&"))
				 {
					 validCommand= true;
					 createThread(shell, "Shell", 12);
				 }
				 else  if(!strcmp(istr1,"Magnet&"))
				 {
					 validCommand= true;
					 createThread(magnet, "Magnet", 8);
				 }


		            else if(isCommand("ipcs",0))
		            {

		                   sprintf(displayRxd," \n\r KeyPressed Count: %d  ", keyPressed.count);
		                   putsUart0(displayRxd);
		                   putsUart0(" \n\r KeyPressed queue contains: ");
		                   if(keyPressed.queueSize > 0 )
		                   {
		                   for(teni=0; teni< keyPressed.queueSize ; teni++)
		                   {
		                   if(keyPressed.processQueue[teni] != -1)
		                   {
		                   sprintf(displayRxd," Task %d ", keyPressed.processQueue[teni]);
		                   putsUart0(displayRxd);
		                   }
		                   validCommand= true;
		                   }
		                   }
		                   putsUart0(" \n\r ");



		                   sprintf(displayRxd," \n\r KeyReleased Count: %d  ", keyReleased.count);
		                   putsUart0(displayRxd);
		                   putsUart0(" \n\r keyReleased queue contains: ");
		                   if(keyReleased.queueSize > 0)
		                       {
		                       for(teni=0; teni< keyReleased.queueSize ; teni++)
		                       {
		                           if(keyPressed.processQueue[teni] != -1)
		                         {
		                           sprintf(displayRxd," Task %d ", keyReleased.processQueue[teni]);
		                           putsUart0(displayRxd);
		                         }
		                       validCommand= true;
		                       }
		                       }
		                       putsUart0(" \n\r ");



		                   sprintf(displayRxd," \n\r flashReq Count: %d  ", flashReq.count);
		                   putsUart0(displayRxd);
		                   putsUart0(" \n\r flashReq queue contains:  ");
		                   if(flashReq.queueSize > 0)
		                   {
		                          for(teni=0; teni< flashReq.queueSize ; teni++)
		                       {
		                       if(flashReq.processQueue[teni] != -1)
		                        {
		                       sprintf(displayRxd," Task %d ", flashReq.processQueue[teni]);
		                       putsUart0(displayRxd);
		                        }
		                       validCommand= true;
		                       }
		                       }
		                      putsUart0(" \n\r ");

		               }

		                   if(!validCommand)
		                                   putsUart0("\n \r Syntax Error \n");


  }
}

void magnet()
{
    // REQUIRED: add code to read the Hall effect sensors that maintain lane centering and report the lane position relative to center
    while(1)
    {
        uint16_t raw1;
        uint16_t raw2;
        uint16_t raw3;
        uint16_t raw4;
        int d;
        char z[10];
      yield();

      if(magnetneed)
  {
            raw1 = readAdc0Ss2();
            yield();
            raw2 = readAdc0Ss2();
            yield();
            raw3 = readAdc0Ss2();
            yield();
            raw4 = readAdc0Ss2();
            yield();
           d=2.46061 + ((raw1)*(-0.00626))+((raw2)*(-0.02585))+((raw3)*(0.025976)) + ((raw4)*(0.001255));
           sprintf(z, "\n\r The bot is %d away from the center \n\r",d);
           putsUart0(z);
           magnetneed=false;
  }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

	// Initialize hardware
	initHw();
	rtosInit();
	if((!PUSH_BUTTON_1)<< 0)
	{
	    GREEN_LED = 1;
	    waitMicrosecond(250000);
	    GREEN_LED = 0;
	    waitMicrosecond(250000);
	}
	else
	{
	// Power-up flash
	RED_LED = 1;
	waitMicrosecond(250000);
	RED_LED = 0;
	waitMicrosecond(250000);

	// Initialize semaphores
	init(&keyPressed, 1);
	init(&keyReleased, 0);
	init(&flashReq, 5);

	// Add required idle process
	ok =  createThread(idle, "Idle", 15);
//	 Add other processes
	ok &= createThread(flash4Hz, "Flash_4hz", 0);
	ok &= createThread(lengthyFn, "Lengthy_fn", 10);
	ok &= createThread(oneshot, "One_shot", 4);
	ok &= createThread(readKeys, "Read_keys", 2);
	ok &= createThread(debounce, "Debounce", 6);
	ok &= createThread(uncooperative, "Uncoop", 7);
	ok &= createThread(shell, "Shell", 12);
	ok &= createThread(magnet, "Magnet", 8);

	}
	// Start up RTOS
	if (ok)
	  rtosStart(); // never returns
	else
	  RED_LED = 1;

    return 0;
    // don't delete this unreachable code
    // if a function is only called once in your code, it will be
    // accessed with two goto instructions instead of call-return,
    // so any stack-based code will not function correctly
    yield(); sleep(0); wait(0); post(0);
}


