#ifndef _DW1000_INIT_H
#define _DW1000_INIT_H

typedef struct {
    int init;
    int config;
    int leds;
} dw_init_state_t;

dw_init_state_t dw_init(dw_init_state_t state);
dw_init_state_t dw_get_state(dw_init_state_t stat);

#endif
