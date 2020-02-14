#include "contiki.h"

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_types.h"
#include "port_platform.h"
#include "mdek1001-def.h"

#include "net/netstack.h"
#include "net/packetbuf.h"

/*---------------------------------------------------------------------------*/
PROCESS(dw1000_process, "DW1000 driver");
/*---------------------------------------------------------------------------*/

#define MAX_PAYLOAD_LEN (127)

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


const struct radio_driver dw1000_driver = {
    init,
    prepare,
    transmit,
    send,
    read,
    channel_clear,
    receiving_packet,
    dw1000_on,
    dw1000_off, 
    get_value,
    set_value,
    get_object,
    set_object,
};


/* 
 * DW1000 radio init function
 * 
 */

/*DW1000 config function*/
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


static int init(void) {
    int result;

    /* Reset radio */
    reset_DW1000();
    /* Set 2MHz SPI for radio initialisation */
    port_set_dw1000_slowrate();
    /* Initialise the radio */
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) 
        return -1;
    
    /* Set 8Mhz SPI for configuration and operation */
    port_set_dw1000_fastrate();
    /* Set default radio paramaters */
    dwt_configure(&config);
    /* Set radio indicator LEDs */
    dwt_setleds(1);

    return RADIO_RESULT_OK;
}

static int prepare(const void *payload, unsigned short payload_len) {
    int result;
    if (payload_len > MAX_PAYLOAD_LEN) return RADIO_TX_ERR;

    dwt_writetxfctrl(payload_len, 0, 0);
    result = dwt_writetxdata(payload_len, &payload, 0);
    if (result == DWT_ERROR) return RADIO_TX_ERR;
    return RADIO_RESULT_OK;
}

static int transmit(unsigned short transmit_len) {
    if (transmit_len > MAX_PAYLOAD_LEN) {
        return RADIO_TX_ERR;
    }

    // Ensure radio is in a IDLE state before sending.
    dwt_forcetrxoff(); 
    
    // start transmission
    if (dwt_starttx(0) == DWT_ERROR){
        return RADIO_TX_ERR;
    } else {
        return RADIO_TX_OK;
    }
}

static int send(const void *payload, unsigned short payload_len) {
    prepare(payload, payload_len);
}

static int read(void *buf, unsigned short bufsize) {
    uint8_t len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
    if (len > bufsize) return RADIO_RESULT_ERROR;
    dwt_readrxdata(&buf, len, 0);
}

static int channel_clear(void) {
    return 0;
}

static int receiving_packet(void) {
    return 0;
}

int dw1000_on(void){
    if (dwt_rxenable(0) == DWT_ERROR){
        return RADIO_RESULT_ERROR;
    }
    
    return RADIO_RESULT_OK;
}

int dw1000_off(void){
    dwt_forcetrxoff();
    return 0;
}