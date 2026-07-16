#define _GNU_SOURCE

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "arm_if_v1.h"
#include "arm_payload_v1.h"
#include "transport.h"

typedef struct {
    mister_transport_config_t transport;
    const char *runtime_path;
    const char *runtime_arg;
    unsigned timeout_ms;
    const char *cmd;
} ctl_config_t;

static void usage(const char *argv0) {
    fprintf(stderr,
            "Usage: %s [--transport file|devmem] [--reg-file <path>] [--devmem <path>] [--hps-base <hex>] [--runtime <path>] [--runtime-arg <arg>] [--timeout-ms <n>] <status|launch|stop|reset|clear-error>\n",
            argv0);
}

static int parse_args(int argc, char **argv, ctl_config_t *cfg) {
    int i;

    cfg->transport.mode = MISTER_TRANSPORT_FILE;
    cfg->transport.reg_file = "/tmp/rp6502_arm_if.bin";
    cfg->transport.devmem_path = "/dev/mem";
    cfg->transport.hps_base = 0;
    cfg->runtime_path = NULL;
    cfg->runtime_arg = NULL;
    cfg->timeout_ms = 2000;
    cfg->cmd = NULL;

    for(i = 1; i < argc; i++) {
        if(strcmp(argv[i], "--transport") == 0) {
            if(++i >= argc) return -1;
            if(strcmp(argv[i], "file") == 0) {
                cfg->transport.mode = MISTER_TRANSPORT_FILE;
            } else if(strcmp(argv[i], "devmem") == 0) {
                cfg->transport.mode = MISTER_TRANSPORT_DEVMEM;
            } else {
                return -1;
            }
        } else if(strcmp(argv[i], "--devmem") == 0) {
            if(++i >= argc) return -1;
            cfg->transport.devmem_path = argv[i];
        } else if(strcmp(argv[i], "--hps-base") == 0) {
            char *end = NULL;
            unsigned long long v;
            if(++i >= argc) return -1;
            v = strtoull(argv[i], &end, 0);
            if(!end || *end != '\0') return -1;
            cfg->transport.hps_base = (uint64_t)v;
        } else if(strcmp(argv[i], "--reg-file") == 0) {
            if(++i >= argc) return -1;
            cfg->transport.reg_file = argv[i];
        } else if(strcmp(argv[i], "--runtime") == 0) {
            if(++i >= argc) return -1;
            cfg->runtime_path = argv[i];
        } else if(strcmp(argv[i], "--runtime-arg") == 0) {
            if(++i >= argc) return -1;
            cfg->runtime_arg = argv[i];
        } else if(strcmp(argv[i], "--timeout-ms") == 0) {
            char *end = NULL;
            unsigned long v;
            if(++i >= argc) return -1;
            v = strtoul(argv[i], &end, 10);
            if(!end || *end != '\0' || v == 0 || v > 30000) return -1;
            cfg->timeout_ms = (unsigned)v;
        } else if(argv[i][0] == '-') {
            return -1;
        } else {
            if(cfg->cmd != NULL) return -1;
            cfg->cmd = argv[i];
        }
    }

    if(cfg->transport.mode == MISTER_TRANSPORT_DEVMEM && cfg->transport.hps_base == 0) {
        fprintf(stderr, "error: --hps-base is required for --transport devmem\n");
        return -1;
    }
    if(cfg->cmd == NULL) return -1;

    return 0;
}

static int write_payload_string(mister_transport_t *tp,
                                uint32_t off,
                                uint32_t max_len,
                                const char *s,
                                int allow_empty) {
    char *base;
    size_t len;
    size_t i;

    if(s == NULL) return 0;
    if(!tp->map_base || tp->map_size < off + max_len) return -1;

    len = strlen(s);
    if((!allow_empty && len == 0) || len + 1 > max_len) {
        return -1;
    }

    for(i = 0; i < len; i++) {
        unsigned char c = (unsigned char)s[i];
        if(c < 0x20 || c > 0x7e) return -1;
    }

    base = (char *)tp->map_base;
    memset(base + off, 0, max_len);
    memcpy(base + off, s, len);
    return 0;
}

static int wait_for_ack(volatile uint32_t *regs, uint32_t seq, unsigned timeout_ms) {
    unsigned i;
    for(i = 0; i < timeout_ms; i++) {
        if(rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_ACK_SEQ) == seq) {
            return 0;
        }
        usleep(1000);
    }
    return -1;
}

static int run_command(mister_transport_t *tp, const ctl_config_t *cfg) {
    volatile uint32_t *regs = tp->regs;
    uint32_t cmd_seq;
    uint32_t ctrl;

    if(strcmp(cfg->cmd, "status") == 0) {
        printf("magic=0x%08x\n", rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_MAGIC));
        printf("version=0x%08x\n", rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_VERSION));
        printf("ctrl=0x%08x\n", rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_CTRL));
        printf("status=0x%08x\n", rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_STATUS));
        printf("heartbeat=%u\n", rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_HEARTBEAT));
        printf("last_error=%u\n", rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_LAST_ERROR));
        printf("cmd_seq=%u\n", rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_CMD_SEQ));
        printf("ack_seq=%u\n", rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_ACK_SEQ));
        return 0;
    }

    if(strcmp(cfg->cmd, "launch") == 0) {
        ctrl = RP6502_ARM_CTRL_REQ_LAUNCH;
        if(write_payload_string(tp,
                                RP6502_ARM_PAYLOAD_V1_OFF_RUNTIME_PATH,
                                RP6502_ARM_PAYLOAD_V1_MAX_RUNTIME_PATH,
                                cfg->runtime_path,
                                0) != 0) {
            fprintf(stderr, "error: invalid --runtime payload string\n");
            return -1;
        }
        if(write_payload_string(tp,
                                RP6502_ARM_PAYLOAD_V1_OFF_RUNTIME_ARG,
                                RP6502_ARM_PAYLOAD_V1_MAX_RUNTIME_ARG,
                                cfg->runtime_arg,
                                1) != 0) {
            fprintf(stderr, "error: invalid --runtime-arg payload string\n");
            return -1;
        }
    } else if(strcmp(cfg->cmd, "stop") == 0) {
        ctrl = RP6502_ARM_CTRL_REQ_STOP;
    } else if(strcmp(cfg->cmd, "reset") == 0) {
        ctrl = RP6502_ARM_CTRL_REQ_RESET_RUNTIME;
    } else if(strcmp(cfg->cmd, "clear-error") == 0) {
        ctrl = RP6502_ARM_CTRL_REQ_CLEAR_ERROR;
    } else {
        fprintf(stderr, "error: unknown command '%s'\n", cfg->cmd);
        return -1;
    }

    cmd_seq = rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_CMD_SEQ) + 1u;
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CTRL, ctrl);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CMD_SEQ, cmd_seq);

    if(wait_for_ack(regs, cmd_seq, cfg->timeout_ms) != 0) {
        fprintf(stderr, "error: timeout waiting for ack_seq=%u\n", cmd_seq);
        return -1;
    }

    printf("ack_seq=%u\n", rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_ACK_SEQ));
    printf("status=0x%08x\n", rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_STATUS));
    printf("last_error=%u\n", rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_LAST_ERROR));
    return 0;
}

int main(int argc, char **argv) {
    ctl_config_t cfg;
    mister_transport_t tp;
    int ret = 1;

    if(parse_args(argc, argv, &cfg) != 0) {
        usage(argv[0]);
        return 2;
    }

    if(mister_transport_open(&cfg.transport, &tp) != 0) {
        return 1;
    }

    ret = run_command(&tp, &cfg) == 0 ? 0 : 1;
    mister_transport_close(&tp);
    return ret;
}
