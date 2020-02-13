#include "dw1000_init.h"
#include "port_platform.h"
#include "mdek1001-def.h"

#include "sys/log.h"
#define LOG_MODULE "DW1000"
#define LOG_LEVEL LOG_LEVEL_INFO

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

dw_init_state_t dw_init(dw_init_state_t state) {
    reset_DW1000();
    port_set_dw1000_slowrate();			
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        state.init = -1;
    } else {
        state.init = 0;
    }
    port_set_dw1000_fastrate();

    dwt_configure(&config); state.config = 1;
    dwt_setleds(1); state.leds = 1;
    return state;
}

dw_init_state_t dw_get_state(dw_init_state_t state){
    
    switch(state.init){
        case -1:
        LOG_WARN("DW1000 Init Failure\n");
        break;

        case 0:
        LOG_INFO("DW1000 Init Success\n");
        break;

        default:
        break;
    }
    return state;
}