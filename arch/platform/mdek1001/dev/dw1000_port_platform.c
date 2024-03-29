/*! ----------------------------------------------------------------------------
 * @file    port_platform.c
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "mdek1001-def.h"

#include "port_platform.h"
#include "deca_device_api.h"

// nRF platform specific headers
#include "boards.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "app_error.h"

/****************************************************************************//**
 *
 *                              APP global variables
 *
 *******************************************************************************/

#define SPI_INSTANCE  1 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context);

/****************************************************************************//**
 *
 *                  Port private variables and function prototypes
 *
 *******************************************************************************/
static volatile uint32_t signalResetDone;

/****************************************************************************//**
 *
 *                          DW1000 port section
 *
 *******************************************************************************/

/* @fn      setup_DW1000RSTnIRQ
 * @brief   setup the DW_RESET pin mode
 *          0 - output Open collector mode
 *          !0 - input mode with connected EXTI0 IRQ
 * */
void setup_DW1000RSTnIRQ(int enable)
{
}



/* @fn      port_wakeup_dw1000
 * @brief   "slow" waking up of DW1000 using DW_CS only
 * */
void port_wakeup_dw1000(void)
{
}

/* @fn      port_wakeup_dw1000_fast
 * @brief   waking up of DW1000 using DW_CS and DW_RESET pins.
 *          The DW_RESET signalling that the DW1000 is in the INIT state.
 *          the total fast wakeup takes ~2.2ms and depends on crystal startup time
 * */
void port_wakeup_dw1000_fast(void)
{
}


/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
    spi_xfer_done = true;
    nrf_gpio_pin_set(LED_1);
}

//================================================================================================
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{ uint8 * p1;
  uint32 idatalength=0;

  idatalength= headerLength + readlength;

  uint8 idatabuf[idatalength];
  uint8 itempbuf[idatalength];

  memset(idatabuf, 0, idatalength);
  memset(itempbuf, 0, idatalength);	

  p1=idatabuf;	
  memcpy(p1,headerBuffer, headerLength);

  p1 += headerLength;
  memset(p1,0x00,readlength);

  nrf_gpio_pin_clear(LED_1);
  spi_xfer_done = false;
  nrf_drv_spi_transfer(&spi, idatabuf, idatalength, itempbuf, idatalength);
  while(!spi_xfer_done);

  p1=itempbuf + headerLength;

  memcpy(readBuffer, p1, readlength);

  return 0;
} 


int writetospi( uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
  uint8 * p1;
  uint32 idatalength=0;

  idatalength= headerLength + bodylength;

  uint8 idatabuf[idatalength];
  uint8 itempbuf[idatalength];

  memset(idatabuf, 0, idatalength);
  memset(itempbuf, 0, idatalength);		 
  
  p1=idatabuf;	
  memcpy(p1,headerBuffer, headerLength);
  p1 += headerLength;
  memcpy(p1,bodyBuffer,bodylength);

  nrf_gpio_pin_clear(LED_1); 
  spi_xfer_done = false;
  nrf_drv_spi_transfer(&spi, idatabuf, idatalength, itempbuf, idatalength);
  while(!spi_xfer_done);

  return 0;
} 

//------------------------------other---------------------------

#define NRF_DRV_SPI_DEFAULT_CONFIG_2M(id)                       \
{                                                            \
    .sck_pin      = CONCAT_3(SPIM, id, _SCK_PIN),      \
    .mosi_pin     = CONCAT_3(SPIM, id, _MOSI_PIN),     \
    .miso_pin     = CONCAT_3(SPIM, id, _MISO_PIN),     \
    .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,                \
    .irq_priority = CONCAT_3(SPIM, id, _IRQ_PRIORITY), \
    .orc          = 0xFF,                                    \
    .frequency    = NRF_DRV_SPI_FREQ_2M,                     \
    .mode         = NRF_DRV_SPI_MODE_0,                      \
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         \
}


#define NRF_DRV_SPI_DEFAULT_CONFIG_8M(id)                       \
{                                                            \
    .sck_pin      = CONCAT_3(SPIM, id, _SCK_PIN),      \
    .mosi_pin     = CONCAT_3(SPIM, id, _MOSI_PIN),     \
    .miso_pin     = CONCAT_3(SPIM, id, _MISO_PIN),     \
    .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,                \
    .irq_priority = CONCAT_3(SPIM, id, _IRQ_PRIORITY), \
    .orc          = 0xFF,                                    \
    .frequency    = NRF_DRV_SPI_FREQ_8M,                     \
    .mode         = NRF_DRV_SPI_MODE_0,                      \
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         \
}

/* @fn      reset_DW1000
 * @brief   DW_RESET pin on DW1000 has 2 functions
 *          In general it is output, but it also can be used to reset the digital
 *          part of DW1000 by driving this pin low.
 *          Note, the DW_RESET pin should not be driven high externally.
 * */
void reset_DW1000(void)
{
  nrf_gpio_cfg_output(DW1000_RST);   
  nrf_gpio_pin_clear(DW1000_RST);  
  nrf_delay_ms(2); 
  nrf_gpio_cfg_input(DW1000_RST, NRF_GPIO_PIN_NOPULL); 
  nrf_delay_ms(2); 
}

/* @fn      port_set_dw1000_slowrate
 * @brief   set 2MHz
 *          
 * */
void port_set_dw1000_slowrate(void)
{
	nrf_drv_spi_config_t  spi_config = NRF_DRV_SPI_DEFAULT_CONFIG_2M(SPI_INSTANCE);
	spi_config.ss_pin = SPIM1_SS_PIN;
	APP_ERROR_CHECK( nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL) );
	nrf_delay_ms(2);	
}

/* @fn      port_set_dw1000_fastrate
 * @brief   set 8MHz
 *         
 * */
void port_set_dw1000_fastrate(void)
{ nrf_drv_spi_uninit(&spi);
	nrf_drv_spi_config_t  spi_config = NRF_DRV_SPI_DEFAULT_CONFIG_8M(SPI_INSTANCE);
	spi_config.ss_pin = SPIM1_SS_PIN;
	APP_ERROR_CHECK( nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL) );
	nrf_delay_ms(2);	
}

/* @fn      deca_sleep 
 * @brief   set sleep function in milliseconds can replace this with 
 *          contiki sleep functions as well.
 *         
 * */
void deca_sleep(unsigned int time_ms)
{
    nrf_delay_ms(time_ms);
}

decaIrqStatus_t decamutexon(void) {
  // need to add things if I intend to use interrrupts
    return 0;
}

void decamutexoff(decaIrqStatus_t s) {
  // interrupt service routine mutex
}
