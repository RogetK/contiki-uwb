#include "contiki.h"
#include "nrf.h"
#include "nrf_uart.h"
#include "nrf_gpio.h"
#include "dev/uart0.h"

#include <stdlib.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
static int (*input_handler)(unsigned char c);

#define UART_INSTANCE NRF_UART0
/*---------------------------------------------------------------------------*/
#define TX_PIN  6
#define RX_PIN  8
/*---------------------------------------------------------------------------*/
void
uart0_set_input(int (*input)(unsigned char c))
{
  input_handler = input;

  if(input != NULL) {
    nrf_uart_int_enable(UART_INSTANCE, NRF_UART_INT_MASK_RXDRDY);
    NVIC_ClearPendingIRQ(UARTE0_UART0_IRQn);
    NVIC_EnableIRQ(UARTE0_UART0_IRQn);
    nrf_uart_task_trigger(UART_INSTANCE, NRF_UART_TASK_STARTRX);
  } else {
    nrf_uart_int_disable(UART_INSTANCE, NRF_UART_INT_MASK_RXDRDY);
    NVIC_ClearPendingIRQ(UARTE0_UART0_IRQn);
    NVIC_DisableIRQ(UARTE0_UART0_IRQn);
    nrf_uart_task_trigger(UART_INSTANCE, NRF_UART_TASK_STOPRX);
  }
}
/*---------------------------------------------------------------------------*/
void
uart0_writeb(unsigned char c)
{
  nrf_uart_txd_set(UART_INSTANCE, c);

  /* Block if previous TX is ongoing */
  while(nrf_uart_event_check(UART_INSTANCE, NRF_UART_EVENT_TXDRDY) == false);
  nrf_uart_event_clear(UART_INSTANCE, NRF_UART_EVENT_TXDRDY);
}
/*---------------------------------------------------------------------------*/
void
uart0_init(unsigned long ubr)
{
  nrf_uart_disable(UART_INSTANCE);
  nrf_gpio_cfg_output(TX_PIN);
  nrf_gpio_pin_set(TX_PIN);
  nrf_gpio_cfg_input(RX_PIN, NRF_GPIO_PIN_NOPULL);

  nrf_uart_baudrate_set(UART_INSTANCE, NRF_UART_BAUDRATE_115200);
  nrf_uart_configure(UART_INSTANCE, NRF_UART_PARITY_EXCLUDED,
                     NRF_UART_HWFC_DISABLED);
  nrf_uart_txrx_pins_set(UART_INSTANCE, TX_PIN, RX_PIN);
  nrf_uart_event_clear(UART_INSTANCE, NRF_UART_EVENT_TXDRDY);
  nrf_uart_enable(UART_INSTANCE);
  nrf_uart_task_trigger(UART_INSTANCE, NRF_UART_TASK_STARTTX);
}
/*---------------------------------------------------------------------------*/
void
UARTE0_UART0_IRQHandler(void)
{
  nrf_uart_event_clear(UART_INSTANCE, NRF_UART_EVENT_RXDRDY);
  input_handler(nrf_uart_rxd_get(UART_INSTANCE));
}
/*---------------------------------------------------------------------------*/
