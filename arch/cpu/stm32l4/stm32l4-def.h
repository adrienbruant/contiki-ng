#ifndef __STM32L4_DEF_H
#define __STM32L4_DEF_H

#include "cm4-def.h"

//This is defined by arm-def.h but ST's HAL requires a 1ms tick
#undef CLOCK_CONF_SECOND
#define CLOCK_CONF_SECOND   1000

#define RTIMER_CONF_CLOCK_SIZE 4 //32 bit rtimer
#define RTIMER_ARCH_SECOND  32768

#endif //__STM32L4_DEF_H