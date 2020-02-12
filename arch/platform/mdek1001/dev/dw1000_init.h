#ifndef DW1000_INIT_H
#define DW1000_INIT_H

#include <stdbool.h>

typedef struct {
    bool init;
} dw_init_state_t;

void dw1000_init(void);

#endif
