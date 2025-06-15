#ifndef TIMER_H
#define TIMER_H

#define TIMER_DEVICE_ID XPAR_XSCUTIMER_0_DEVICE_ID
#define INTC_DEVICE_ID XPAR_SCUGIC_SINGLE_DEVICE_ID
#define TIMER_IRPT_INTR XPAR_SCUTIMER_INTR

#define TIMER_1ms_LOAD_VALUE	332999
#define TIMER_1ms_PRESCALER		0x0

extern XScuTimer TimerInstance;

extern int Timer_ms1;

int Init_Timer_ms1(void);
void TimerIntrHandler(void *CallBackRef);

void Delay_ms(int ms);

#endif // TIMER_H