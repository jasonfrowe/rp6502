#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "arm_if_v1.h"
#include "arm_payload_v1.h"
#include "transport.h"

static int open_file_transport(const mister_transport_config_t *cfg, mister_transport_t *tp) {
    void *map;

    tp->fd = open(cfg->reg_file, O_RDWR | O_CREAT, 0644);
    if(tp->fd < 0) {
        perror("open reg-file");
        return -1;
    }

    if(ftruncate(tp->fd, RP6502_ARM_IF_WINDOW_SIZE) != 0) {
        perror("ftruncate reg-file");
        close(tp->fd);
        tp->fd = -1;
        return -1;
    }

    map = mmap(NULL, RP6502_ARM_IF_WINDOW_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, tp->fd, 0);
    if(map == MAP_FAILED) {
        perror("mmap reg-file");
        close(tp->fd);
        tp->fd = -1;
        return -1;
    }

    tp->map_base = map;
    tp->map_size = RP6502_ARM_IF_WINDOW_SIZE;
    tp->regs = (volatile uint32_t *)map;
    return 0;
}

static int open_devmem_transport(const mister_transport_config_t *cfg, mister_transport_t *tp) {
    const uint32_t page = (uint32_t)sysconf(_SC_PAGESIZE);
    const uint64_t base_page = cfg->hps_base & ~((uint64_t)page - 1u);
    const uint32_t page_off = (uint32_t)(cfg->hps_base - base_page);
    const uint32_t map_size = page_off + RP6502_ARM_IF_WINDOW_SIZE;
    void *map;

    tp->fd = open(cfg->devmem_path, O_RDWR | O_SYNC);
    if(tp->fd < 0) {
        perror("open /dev/mem");
        return -1;
    }

    map = mmap(NULL, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, tp->fd, (off_t)base_page);
    if(map == MAP_FAILED) {
        perror("mmap /dev/mem");
        close(tp->fd);
        tp->fd = -1;
        return -1;
    }

    tp->map_base = map;
    tp->map_size = map_size;
    tp->regs = (volatile uint32_t *)((uint8_t *)map + page_off);
    return 0;
}

int mister_transport_open(const mister_transport_config_t *cfg, mister_transport_t *tp) {
    memset(tp, 0, sizeof(*tp));
    tp->fd = -1;

    if(cfg->mode == MISTER_TRANSPORT_FILE) {
        return open_file_transport(cfg, tp);
    }

    if(cfg->mode == MISTER_TRANSPORT_DEVMEM) {
        return open_devmem_transport(cfg, tp);
    }

    fprintf(stderr, "unknown transport mode\n");
    return -1;
}

void mister_transport_close(mister_transport_t *tp) {
    if(tp->map_base && tp->map_size) {
        munmap(tp->map_base, tp->map_size);
    }
    if(tp->fd >= 0) {
        close(tp->fd);
    }
    memset(tp, 0, sizeof(*tp));
    tp->fd = -1;
}
