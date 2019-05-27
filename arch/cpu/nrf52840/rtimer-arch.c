/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "nrf.h"
#include "nrf_timer.h"

#include <stdint.h>
#include <stddef.h>
/*---------------------------------------------------------------------------*/
/*
 * Use Timer RTIMER_TIMER at 62500Hz. Generates 1 tick per exactly 16 usecs,
 * which is exactly 1 .15.4 symbol period.
 */
#define TIMER_INSTANCE NRF_TIMER0
/*---------------------------------------------------------------------------*/
void
rtimer_arch_init(void)
{
  nrf_timer_event_clear(TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE0);

  nrf_timer_frequency_set(TIMER_INSTANCE, NRF_TIMER_FREQ_62500Hz);
  nrf_timer_bit_width_set(TIMER_INSTANCE, NRF_TIMER_BIT_WIDTH_32);
  nrf_timer_mode_set(TIMER_INSTANCE, NRF_TIMER_MODE_TIMER);
  nrf_timer_int_enable(TIMER_INSTANCE, NRF_TIMER_INT_COMPARE0_MASK);
  NVIC_ClearPendingIRQ(TIMER0_IRQn);
  NVIC_EnableIRQ(TIMER0_IRQn);
  nrf_timer_task_trigger(TIMER_INSTANCE, NRF_TIMER_TASK_START);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Schedules an rtimer task to be triggered at time t
 * \param t The time when the task will need executed.
 *
 * \e t is an absolute time, in other words the task will be executed AT
 * time \e t, not IN \e t rtimer ticks.
 *
 * This function schedules a one-shot event with the nRF RTC.
 */
void
rtimer_arch_schedule(rtimer_clock_t t)
{
  nrf_timer_cc_write(TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, t);
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
rtimer_arch_now()
{
  nrf_timer_task_trigger(TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE1);
  return nrf_timer_cc_read(TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL1);
}
/*---------------------------------------------------------------------------*/
void
TIMER0_IRQHandler(void)
{
  if(nrf_timer_event_check(TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE0)) {
    nrf_timer_event_clear(TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE0);
    rtimer_run_next();
  }
}
