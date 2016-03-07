#include "watchdog.h"
#include <conf.h>

watchdog::watchdog(void) :
	tickCnt(0), currState(IDLE), userCB(0){

		timerCountAddr = ARC_V2_TMR1_COUNT;
		timerControlAddr = ARC_V2_TMR1_CONTROL;
		timerLimitAddr = ARC_V2_TMR1_LIMIT;
		timerIrqNum = ARC_V2_IRQ_TIMER1;
		isrFuncPtr = &timerONEIsrWrapper;
		pwmCB = &timerOnePwmCbWrapper;
}

void watchdog::attachInterrupt((void (*userCallBack)()){

	unsigned int reg;
	reg = aux_reg_read(timerControlAddr) & ~ARC_TIMER_EN_INTR_BIT_FLAG;
	aux_reg_write(timerControlAddr, reg);

	userCB = userCallBack;

}

unsigned int watchdog::rdRstTickcount(void){

	unsigned int tmp;
	tmp = tickCnt;
	tickCnt = 0;
	return tmp;
}

int watchdog::init(const unsigned int periodHz, 
	void (*userCallBack)()){

	if (userCallBack != 0)
		userCB = userCallBack;
	aux_reg_write(timerLimitAddr, periodHz); //load timer period

	aux_reg_write(timerCountAddr, 0); //reset variables
	tickCnt = 0;

	if(isrFuncPtr != 0){		// Enable watchdog timer running with interrupt
    	interrupt_connect(timerIrqNum, isrFuncPtr);
    	aux_reg_write(timerControlAddr, ARC_TIMER_EN_INTR_BIT_FLAG |
		ARC_TIMER_NON_HALT_ONLY_BIT_FLAG | ARC_TIMER_WATCHDOG_BIT_FLAG);
    	interrupt_enable(timerIrqNum);
   	}	
   	else {  // Timer runs without interrupt
    aux_reg_write(timerControlAddr, ARC_TIMER_NON_HALT_ONLY_BIT_FLAG);

  }

}
