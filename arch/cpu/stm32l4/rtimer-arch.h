#ifndef __RTIMER_ARCH_H
#define __RTIMER_ARCH_H

#include "contiki.h"

rtimer_clock_t rtimer_arch_now(void);

void rtimer_interrupt_handler(void);

#endif //__RTIMER_ARCH_H
