#ifndef watchdog_h
#define watchdog_h

#include <Arduino.h>
#include <aux_regs.h>
#include <interrupt.h>
#include <conf.h>

//the ARC core has 2 timers, timer-1 is used since the clock frequency is defined in conf.h
const unsigned int HZ_SEC = (ARC_TIMER1_CLOCK_FREQ / 1000000);
const unsigned int MAX_PERIOD = (0x0FFFFFFFF / HZ_USEC); //the period of the watchdog should be set to this

const unsigned int ARC_TIMER_1 = 1; //the second ARC timer

//TIMERx_CONTROL register
const unsigned int ARC_TIMER_EN_INTR_BIT_FLAG = 0x01;  // Bit 0. Enable timer interrupt.
const unsigned int ARC_TIMER_NON_HALT_ONLY_BIT_FLAG = 0x02;  // Bit 1. Allow timer to run in Halted mode.
const unsigned int ARC_TIMER_WATCHDOG_BIT_FLAG = 0x04;  // Bit 2. Make timer a watchdog timer.
const unsigned int ARC_TIMER_INTR_PENDING_BIT_FLAG = 0x08;  // Bit 3. Interrupt pending and clearing bit.

typedef enum{
	IDLE = 0,
	RUNNING,
	PAUSED
} timerStateType;

class watchdog{
	public:
		watchdog(const unsigned int timerNum = ARC_TIMER_1);

		inline int start(const unsigned	int timerPeriodUsec = 0,
			void (*userCallBack)() = 0) {
    		return( init((timerPeriodUsec * HZ_USEC), userCallBack) ); 
   		}

    	//restart the timer from 0
    	int restart(const unsigned int timerPeriodUsec) { return start(timerPeriodUsec); }

		// Attach or detach the user call back routine.
    	void attachInterrupt(void (*userCallBack)());
   		 void detachInterrupt(void) { return attachInterrupt(0); };

    	// Timer interrupt count
    	inline unsigned int readTickCount(void) { return tickCnt; }
    	// Read and reset timer interrupt count.
    	unsigned int rdRstTickCount(void);

	private:
		unsigned int timerCountAddr;
  	 	unsigned int timerControlAddr;
    	unsigned int timerLimitAddr;
    	unsigned int timerIrqNum;
	    unsigned int tickCnt;	

		void (*isrFuncPtr)();
    	void (*userCB)();
    	void (*pwmCB)();

    	// Init:  Kick off a timer by initializing it with a period.
    	int init(const unsigned int periodHz, void (*userCallBack)());
};

#endif
