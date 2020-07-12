#ifndef CONTIKI_CONF_H
#define CONTIKI_CONF_H

/* Include Project Specific conf */
#ifdef PROJECT_CONF_PATH
#include PROJECT_CONF_PATH
#endif /* PROJECT_CONF_PATH */
/*---------------------------------------------------------------------------*/
#include "nucleo-l432-def.h"
#include "stm32l4-def.h"
/*---------------------------------------------------------------------------*/
/* 
 * All user configuration to go here
 */
/*---------------------------------------------------------------------------*/
/* Include CPU-related configuration */
#include "stm32l4-conf.h"
/*---------------------------------------------------------------------------*/

#endif /* CONTIKI_CONF_H */