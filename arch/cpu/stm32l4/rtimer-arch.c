#include "rtimer.h"
#include "dbg-handlers.h"
#include "stm32l4xx_hal.h"

LPTIM_HandleTypeDef hlptim2;

static volatile uint16_t rseconds;
static volatile uint16_t rseconds_alarm;

static volatile uint8_t pending;

/**
* @brief LPTIM MSP Initialization
* This function configures the hardware resources used in this example
* @param hlptim: LPTIM handle pointer
* @retval None
*/
void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef* hlptim)
{
  if(hlptim->Instance==LPTIM2)
  {
    /* Peripheral clock enable */
    __HAL_RCC_LPTIM2_CLK_ENABLE();
  }

}

/**
* @brief LPTIM MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hlptim: LPTIM handle pointer
* @retval None
*/
void HAL_LPTIM_MspDeInit(LPTIM_HandleTypeDef* hlptim)
{
  if(hlptim->Instance==LPTIM2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_LPTIM2_CLK_DISABLE();
  }

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM2_Init(void)
{
  hlptim2.Instance = LPTIM2;
  hlptim2.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim2.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim2.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim2.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim2.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim2.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim2.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim2.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim2) != HAL_OK)
  {
    Error_Handler();
  }
}

void rtimer_arch_init(void)
{
  MX_LPTIM2_Init();

  rseconds = 0;

#ifdef FREEZE_TIMER_WHEN_DEBUGGING
  //Freeze the timer on debug
  __HAL_DBGMCU_FREEZE_LPTIM2();
#endif

  HAL_LPTIM_PWM_Start_IT(&hlptim2, RTIMER_ARCH_SECOND-1, 0);
  __HAL_LPTIM_DISABLE_IT(&hlptim2, LPTIM_IT_CMPM);
  HAL_NVIC_EnableIRQ(LPTIM2_IRQn);
}

static uint16_t rtimer_arch_read_subseconds(void)
{
  uint16_t prev_cnt, cnt;

  cnt = hlptim2.Instance->CNT;
  prev_cnt = ~cnt;

  //Read counter twice because we are using LSE
  while(prev_cnt != cnt) {
    prev_cnt = cnt;
    cnt = hlptim2.Instance->CNT;
  }

  return cnt;
}

static void rtimer_arch_update_it(void)
{
  //If we are in the same second and before the alarm subseconds
  if( (pending) &&
      (rseconds_alarm == rseconds) &&
      (rtimer_arch_read_subseconds() < hlptim2.Instance->CMP) ) {
    //Enable compare interrupt
    __HAL_LPTIM_ENABLE_IT(&hlptim2, LPTIM_IT_CMPM);
  } else {
    __HAL_LPTIM_DISABLE_IT(&hlptim2, LPTIM_IT_CMPM);
  }
}

rtimer_clock_t rtimer_arch_now(void)
{
  uint16_t rsubs = rtimer_arch_read_subseconds();

  return ((rseconds << 15) | (rsubs));
}

void rtimer_arch_schedule(rtimer_clock_t t)
{
  uint16_t subseconds;

  rseconds_alarm = t >> 15;
  subseconds = t & 0x00007FFF;

  __HAL_LPTIM_COMPARE_SET(&hlptim2, subseconds);
  pending = 1;

  rtimer_arch_update_it();
}

void rtimer_interrupt_handler(void)
{
  HAL_LPTIM_IRQHandler(&hlptim2);
}

void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  if( hlptim == &hlptim2 ) {
    pending = 0;
    rtimer_arch_update_it();
    rtimer_run_next();
  }
}

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  if( hlptim == &hlptim2 ) {
    rseconds++;
    rtimer_arch_update_it();
  }
}
