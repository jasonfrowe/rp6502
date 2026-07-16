#pragma once

#include <stdint.h>

typedef enum {
    MISTER_TRANSPORT_FILE = 0,
    MISTER_TRANSPORT_DEVMEM = 1,
} mister_transport_mode_t;

typedef struct {
    mister_transport_mode_t mode;
    const char *reg_file;
    const char *devmem_path;
    uint64_t hps_base;
} mister_transport_config_t;

typedef struct {
    volatile uint32_t *regs;
    int fd;
    void *map_base;
    uint32_t map_size;
} mister_transport_t;

int mister_transport_open(const mister_transport_config_t *cfg, mister_transport_t *tp);
void mister_transport_close(mister_transport_t *tp);
