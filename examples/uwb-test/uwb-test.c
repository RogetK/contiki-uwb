#include "contiki.h"
#include <stdio.h> /* For printf() */

#include "mdek1001-def.h"

/*---------------------------------------------------------------------------*/
#include "sys/log.h"
#define LOG_MODULE "Heartbeat Process"
#define LOG_LEVEL LOG_LEVEL_INFO
/*---------------------------------------------------------------------------*/
PROCESS(heartbeat_process, "Test process");
AUTOSTART_PROCESSES(&heartbeat_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(heartbeat_process, ev, data)
{
  static struct etimer timer;

  PROCESS_BEGIN();
  LOG_INFO("Process started\n\n\n");
  /* Setup a periodic timer that expires after 10 seconds. */
  etimer_set(&timer, CLOCK_SECOND * 0.5);

  while(1) {
    nrf_gpio_pin_toggle(LED_1); 
    /* Wait for the periodic timer to expire and then restart the timer. */
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    etimer_reset(&timer);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
