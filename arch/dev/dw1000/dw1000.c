#include "contiki.h"
#include "sys/energest.h"

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_types.h"
#include "port_platform.h"
#include "mdek1001-def.h"

#include "nrf_ppi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"
#include "nrf_timer.h"
#include "nrf_clock.h"

#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/mac/tsch/tsch.h"

#include "sys/log.h"
#define LOG_MODULE "DW1000"

#if defined(LOG_LEVEL_DW1000)
#define LOG_LEVEL LOG_LEVEL_DW1000
#else
#define LOG_LEVEL LOG_LEVEL_NONE
#endif

/*---------------------------------------------------------------------------*/
// PROCESS(dw1000_process, "DW1000 driver");
/*---------------------------------------------------------------------------*/

#define CRC_LEN 2
#define MAX_PAYLOAD_LEN (127 - CRC_LEN)
#define DW1000_RX_AFTER_TX_DELAY 0

#define DW1000_CHANNEL_MIN 0
#define DW1000_CHANNEL_MAX (MAX_CHANNELS - 1)

static bool poll_mode = false;

static bool rx = false;

static volatile rtimer_clock_t timestamp_sfd;


/*------------------------------------------------------------------------
 * Function Declarations
 *----------------------------------------------------------------------*/

static int init(void);
static int prepare(const void *payload, unsigned short payload_len);
static int transmit(unsigned short transmit_len);
static int send(const void *payload, unsigned short payload_len);
static int read(void *buf, unsigned short buf_len);
static int channel_clear(void);
static int receiving_packet(void);
static int pending_packet(void);

int dw1000_on(void);
int dw1000_off(void);

static radio_result_t get_value(radio_param_t param, radio_value_t *value);
static radio_result_t set_value(radio_param_t param, radio_value_t value);

static radio_result_t get_object(radio_param_t param, void *dest, size_t size);
static radio_result_t set_object(radio_param_t param, const void *src, size_t size);


const struct radio_driver dw1000_driver =
{
  init,
  prepare,
  transmit,
  send,
  read,
  channel_clear,
  receiving_packet,
  pending_packet,
  dw1000_on,
  dw1000_off,
  get_value,
  set_value,
  get_object,
  set_object
};


static void dw_interrupt_init(void){

    ret_code_t err_code;
    nrf_drv_gpiote_init();
    nrf_drv_gpiote_in_config_t in_config = 
        GPIOTE_CONFIG_IN_SENSE_LOTOHI(true); 

    in_config.pull = NRF_GPIO_PIN_NOPULL;
    err_code = 
        nrf_drv_gpiote_in_init(DW1000_IRQ, &in_config, NULL);

    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(DW1000_IRQ, true);
   
}



static void ppi_init(void) {

    nrf_ppi_channel_endpoint_setup(
        NRF_PPI_CHANNEL0,
        (uint32_t) nrf_drv_gpiote_in_event_addr_get(DW1000_IRQ), 
        (uint32_t) nrf_timer_task_address_get(NRF_TIMER0, NRF_TIMER_TASK_CAPTURE3)
    );

    nrf_ppi_channel_enable(NRF_PPI_CHANNEL0);
}

static const dwt_config_t default_config = {
    5,                /* Channel number. */
    DWT_PRF_64M,      /* Pulse repetition frequency. */
    DWT_PLEN_64,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    12,               /* TX preamble code. Used in TX only. */
    12,               /* RX preamble code. Used in RX only. */
    0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    (65 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static const dwt_config_t defaults[12] ={
    {1, DWT_PRF_16M, DWT_PLEN_64, DWT_PAC8, 1, 1, 0, DWT_BR_6M8, DWT_PHRMODE_STD, 65},
    {1, DWT_PRF_64M, DWT_PLEN_64, DWT_PAC8, 9, 9, 0, DWT_BR_6M8, DWT_PHRMODE_STD, 65},
    {2, DWT_PRF_16M, DWT_PLEN_64, DWT_PAC8, 3, 3, 0, DWT_BR_6M8, DWT_PHRMODE_STD, 65},
    {2, DWT_PRF_64M, DWT_PLEN_64, DWT_PAC8, 10, 10, 0, DWT_BR_6M8, DWT_PHRMODE_STD, 65},
    {3, DWT_PRF_16M, DWT_PLEN_64, DWT_PAC8, 5, 5, 0, DWT_BR_6M8, DWT_PHRMODE_STD, 65},
    {3, DWT_PRF_64M, DWT_PLEN_64, DWT_PAC8, 11, 11, 0, DWT_BR_6M8, DWT_PHRMODE_STD, 65},
    {4, DWT_PRF_16M, DWT_PLEN_64, DWT_PAC8, 7, 7, 0, DWT_BR_6M8, DWT_PHRMODE_STD, 65},
    {4, DWT_PRF_64M, DWT_PLEN_64, DWT_PAC8, 17, 17, 0, DWT_BR_6M8, DWT_PHRMODE_STD, 65},
    {5, DWT_PRF_16M, DWT_PLEN_64, DWT_PAC8, 4, 4, 0, DWT_BR_6M8, DWT_PHRMODE_STD, 65},
    {5, DWT_PRF_64M, DWT_PLEN_64, DWT_PAC8, 12, 12, 0, DWT_BR_6M8, DWT_PHRMODE_STD, 65},
    {7, DWT_PRF_16M, DWT_PLEN_64, DWT_PAC8, 8, 8, 0, DWT_BR_6M8, DWT_PHRMODE_STD, 65},
    {7, DWT_PRF_64M, DWT_PLEN_64, DWT_PAC8, 19, 19, 0, DWT_BR_6M8, DWT_PHRMODE_STD, 65},
};

typedef enum {
    CH1_16 = 0, CH1_64, CH2_16, CH2_64,
    CH3_16, CH3_64, CH4_16, CH4_64,
    CH5_16, CH5_64, CH7_16, CH7_64, MAX_CHANNELS
} channels_e;

static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PRF_64M,      /* Pulse repetition frequency. */
    DWT_PLEN_64,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    10,               /* TX preamble code. Used in TX only. */
    10,               /* RX preamble code. Used in RX only. */
    0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    (65 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static int get_channel(dwt_config_t dw_config) {
    int val = 0;
    
    val = (2 * config.chan) + config.prf; 
    val = (config.chan == 7) ? val - 4 : val - 2;
    return val-1; 
}

static int set_channel(channels_e value) {

    dwt_forcetrxoff();
    rx = false;

    memcpy(&config, &(defaults[value]), sizeof(dwt_config_t));
    dwt_configure(&config);
    return 0;
}

/* 
 * DW1000 radio init function
 * 
 */

/*DW1000 config function*/

static int 
init(void) {

    timestamp_sfd = 0;
    // int result;
    dw_interrupt_init();
    /* Reset radio */
    reset_DW1000();
    /* Set 2MHz SPI for radio initialisation */
    port_set_dw1000_slowrate();
    /* Initialise the radio */
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        LOG_DBG("DW1000 Init failed\n");    
        return -1;
    }

    
    /* Set 8Mhz SPI for configuration and operation */
    port_set_dw1000_fastrate();
    /* Set default radio paramaters */
    dwt_configure(&config);
    /* Set radio indicator LEDs */
    dwt_setleds(1);

    dwt_setinterrupt(DWT_INT_RXPHD, 1);

    ppi_init();

    return RADIO_RESULT_OK;
}

/*
 * ------------------------------------------------------------------------
 */

static int 
prepare(const void *payload, unsigned short payload_len) {
    uint8_t frame_len = payload_len + CRC_LEN;

    if (frame_len > MAX_PAYLOAD_LEN) return RADIO_TX_ERR;
    
    // write data to radio buffer
    dwt_writetxdata(frame_len, (uint8_t *)payload, 0);
    dwt_writetxfctrl(frame_len, 0, 0);

    return RADIO_RESULT_OK;
}

/*
 * ------------------------------------------------------------------------
 */

static int 
transmit(unsigned short transmit_len) {
    if (transmit_len > MAX_PAYLOAD_LEN) return RADIO_TX_ERR;
    
    // Ensure radio is in a IDLE state before sending.
    dwt_forcetrxoff(); 
    rx = false;

    dwt_setrxaftertxdelay(DW1000_RX_AFTER_TX_DELAY);

    // start transmission
    if (dwt_starttx(0) != DWT_SUCCESS) return RADIO_TX_ERR;
    
    //wait for transmission confirmation
    uint32_t result;
    while(!((result = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS)) {}

    return RADIO_TX_OK;
}

/*
 * ------------------------------------------------------------------------
 */


static int 
send(const void *payload, unsigned short payload_len) {
    if (prepare(payload, payload_len) == 0) {
        return transmit(payload_len);
    } else {
        return RADIO_TX_ERR;
    }
}

/*
 * ------------------------------------------------------------------------
 */

/* 
 * Clear the event bits in the radio interrrupt registers
 * read packet data
 * enter rx 
 */


static int 
read(void *buf, unsigned short bufsize) {
    LOG_DBG("read\n");
    uint8_t frame_len; 
    
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXPHD);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
    dwt_readrxdata((uint8_t *) buf, frame_len, 0);

    // Timestamp in rtimer ticks for complete reception of SFD
    timestamp_sfd = nrf_timer_cc_read(NRF_TIMER0, NRF_TIMER_CC_CHANNEL3);
    return (frame_len - CRC_LEN);
}

/*
 * ------------------------------------------------------------------------
 */


static int 
channel_clear(void) {
    return 0;
}

/*
 * ------------------------------------------------------------------------
 */


static int 
receiving_packet(void) {
    if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXPRD) return 0;
    else return 1;
}

/*
 * ------------------------------------------------------------------------
 */

static int 
pending_packet(void) {
    uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
    if (status & SYS_STATUS_RXFCG){
        // Frame is now pending
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        return 1;
    } 
    else return 0;
}

/*
 * ------------------------------------------------------------------------
 */

int 
dw1000_on(void){
    // check if radio is in RX state prior to running on as dw1000 never turns off in TSCH
    if (!rx){
        if (dwt_rxenable(0) != DWT_SUCCESS){
            return RADIO_RESULT_ERROR;
        }
        rx = true;
        return RADIO_RESULT_OK;
    } else {
        return RADIO_RESULT_OK;
    }
}

/*
 * ------------------------------------------------------------------------
 */

int 
dw1000_off(void) {
    port_set_dw1000_fastrate();
    dwt_forcetrxoff();
    rx = false;
    return 0;
}

/*
 * ------------------------------------------------------------------------
 */

static radio_result_t 
get_value(radio_param_t param, radio_value_t *value) {
    // LOG_DBG("GET V\n");

    if (!value)  return RADIO_RESULT_INVALID_VALUE;

    switch(param) {

    case RADIO_PARAM_CHANNEL:
        *value = (radio_value_t) get_channel(config);
        return RADIO_RESULT_OK;
    
    case RADIO_PARAM_RX_MODE:
        *value = 0;
        if(poll_mode) {
            *value |= RADIO_RX_MODE_POLL_MODE;
        }
        return RADIO_RESULT_OK;


    case RADIO_PARAM_TX_MODE:
        // *value = 0;
        return RADIO_RESULT_OK;

    case RADIO_CONST_MAX_PAYLOAD_LEN:
        *value = (radio_value_t) MAX_PAYLOAD_LEN;
        return RADIO_RESULT_OK;
    default:
        return RADIO_RESULT_NOT_SUPPORTED;
    }


}

/*
 * ------------------------------------------------------------------------
 */

static radio_result_t 
set_value(radio_param_t param, radio_value_t value) {
    // LOG_DBG("SET V %d value %d\n", param, value);    
    switch(param){

    case RADIO_PARAM_RX_MODE:
        if ( value & ~(RADIO_RX_MODE_ADDRESS_FILTER |
        RADIO_RX_MODE_AUTOACK |
        RADIO_RX_MODE_POLL_MODE)) {
            return RADIO_RESULT_INVALID_VALUE;
        }
        poll_mode = true;
        return RADIO_RESULT_OK;

    case RADIO_PARAM_CHANNEL:
        if (value < DW1000_CHANNEL_MIN || value > DW1000_CHANNEL_MAX) {
            LOG_DBG("INVALID \n");
            return RADIO_RESULT_INVALID_VALUE;
        }
        set_channel(value);
        return RADIO_RESULT_OK;

    case RADIO_PARAM_TX_MODE:
       return RADIO_RESULT_OK; 
    }

    return RADIO_RESULT_NOT_SUPPORTED;
}

/*
 * ------------------------------------------------------------------------
 */

static radio_result_t 
get_object(radio_param_t param, void *dest, size_t size) {
        // LOG_DBG("GET O\n");
        if (param == RADIO_PARAM_LAST_PACKET_TIMESTAMP) {
        if (size != sizeof(rtimer_clock_t) || !dest) {
            return RADIO_RESULT_INVALID_VALUE;
        }
        *(rtimer_clock_t *) dest = timestamp_sfd;
        return RADIO_RESULT_OK;
    }

    if (param == RADIO_CONST_TSCH_TIMING) {
        if (size != sizeof(uint16_t *) || !dest) {
            return RADIO_RESULT_INVALID_VALUE;
        }

        *(const uint16_t **) dest = tsch_timeslot_timing_us_10000;
        return RADIO_RESULT_OK;
    }
    return RADIO_RESULT_NOT_SUPPORTED;
}


/*
 * ------------------------------------------------------------------------
 */

static radio_result_t 
set_object(radio_param_t param, const void *src, size_t size) {
    // LOG_DBG("SET V\n");
    return RADIO_RESULT_NOT_SUPPORTED;
}

/*
 * ------------------------------------------------------------------------
 */

// PROCESS_THREAD(dw1000_process, ev, data)
// {
//     int len;
//     PROCESS_BEGIN();

//     while(1) {
//         PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_NONE);
//     }

//   PROCESS_END();
// }
