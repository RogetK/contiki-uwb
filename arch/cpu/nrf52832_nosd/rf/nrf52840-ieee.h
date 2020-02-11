/*---------------------------------------------------------------------------*/
#ifndef NRF52840_IEEE_H_
#define NRF52840_IEEE_H_
/*---------------------------------------------------------------------------*/
#include "contiki.h"
/*---------------------------------------------------------------------------*/
typedef void (*nrf52840_ieee_callback_t)(void);

/*
 * Register an external radio event callback.
 *
 * The radio driver will call this function right at the start of the radio
 * interrupt handler.
 *
 * Passing a NULL pointer as the argument of this function will de-register
 * the callback.
 *
 * The radio driver will expect this callback to clear the events that caused
 * the interrupt. The radio driver will make no other assumptions about the
 * logic and behaviour of the external module. This means that the external
 * module needs to enable the respective interrupt for any events it is
 * interested in getting notified about.
 *
 * Whenever the radio gets turned on and configured to be used by the netstack
 * API, the driver will reset all interrupts to known state suitable to be
 * used by the netstack API. This means that the external module will need to
 * re-enable its respective interrupts between two consecutive radio.on()
 * calls.
 *
 * More specifically:
 *
 * - CSMA MAC will call NETSTACK_RADIO.on() only once during device boot. This
 *   means that if the external module wishes to take control of the radio it
 *   should start with a call to NETSTACK_RADIO.off(). It should proceed with
 *   powering on the radio and enabling the correct radio interrupts before
 *   starting to manipulate the radio. Once finished, it should power off the
 *   radio (to clear all radio registers) and then pass control back to CSMA
 *   by calling NETSTACK_RADIO.on(). This call will restore the radio to a
 *   state suitable for CSMA operation. The callback will still get called
 *   whenever the radio interrupt fires even after control has been passed
 *   back to CSMA. If this is undesirable, the callback needs to be
 *   de-registered by calling nrf52840_ieee_register_handler(NULL);
 *
 * - TSCH MAC will power-cycle the radio by calling NETSCACK_RADIO.on() and
 *   .off() at the beginning / end of each time slot. Part of the timeslot
 *   operation is performed within an rtimer interrupt (Timer 0, Channel 0)
 *   context.
 *
 *   The external module should only use the radio during .off() periods. The
 *   external module must take steps to make sure that the TSCH slot operation
 *   rtimer interrupt does not attempt to manipulate the radio while the
 *   external module is using it. When finished, the external module must make
 *   sure that the radio is left in a powered-off state. This can be achieved
 *   by calling NETSTACK_RADIO.off(). Unlike under CSMA, when using TSCH the
 *   external module does not need to call NETSTACK_RADIO.on() since this will
 *   happen automatically at the start of the next TSCH slot.
 */
void nrf52840_ieee_register_handler(nrf52840_ieee_callback_t handler);
/*---------------------------------------------------------------------------*/
#endif /* NRF52840_IEEE_H_ */
/*---------------------------------------------------------------------------*/
