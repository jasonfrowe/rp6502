#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "arm_if_v1.h"

typedef struct {
    const char *reg_file;
    const char *runtime_path;
    const char *runtime_arg;
    unsigned poll_ms;
} wrapper_config_t;

typedef struct {
    volatile uint32_t *regs;
    pid_t child_pid;
    uint32_t last_cmd_seq;
} wrapper_state_t;

static int g_terminate = 0;

static void usage(const char *argv0) {
    fprintf(stderr,
            "Usage: %s [--reg-file <path>] [--runtime <path>] [--runtime-arg <arg>] [--poll-ms <n>]\n",
            argv0);
}

static void on_signal(int sig) {
    (void)sig;
    g_terminate = 1;
}

static int parse_args(int argc, char **argv, wrapper_config_t *cfg) {
    int i;

    cfg->reg_file = "/tmp/rp6502_arm_if.bin";
    cfg->runtime_path = NULL;
    cfg->runtime_arg = NULL;
    cfg->poll_ms = 2;

    for(i = 1; i < argc; i++) {
        if(strcmp(argv[i], "--reg-file") == 0) {
            if(++i >= argc) return -1;
            cfg->reg_file = argv[i];
        } else if(strcmp(argv[i], "--runtime") == 0) {
            if(++i >= argc) return -1;
            cfg->runtime_path = argv[i];
        } else if(strcmp(argv[i], "--runtime-arg") == 0) {
            if(++i >= argc) return -1;
            cfg->runtime_arg = argv[i];
        } else if(strcmp(argv[i], "--poll-ms") == 0) {
            char *end = NULL;
            unsigned long v;
            if(++i >= argc) return -1;
            v = strtoul(argv[i], &end, 10);
            if(!end || *end != '\0' || v == 0 || v > 1000) return -1;
            cfg->poll_ms = (unsigned)v;
        } else {
            return -1;
        }
    }

    return 0;
}

static int map_register_file(const char *path, volatile uint32_t **regs_out) {
    int fd;
    void *map;

    fd = open(path, O_RDWR | O_CREAT, 0644);
    if(fd < 0) {
        perror("open reg-file");
        return -1;
    }

    if(ftruncate(fd, RP6502_ARM_IF_REG_SIZE) != 0) {
        perror("ftruncate reg-file");
        close(fd);
        return -1;
    }

    map = mmap(NULL, RP6502_ARM_IF_REG_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if(map == MAP_FAILED) {
        perror("mmap reg-file");
        return -1;
    }

    *regs_out = (volatile uint32_t *)map;
    return 0;
}

static void runtime_reap_if_exited(wrapper_state_t *st) {
    int status;
    pid_t r;

    if(st->child_pid <= 0) return;

    r = waitpid(st->child_pid, &status, WNOHANG);
    if(r == st->child_pid) {
        st->child_pid = -1;
        rp6502_arm_if_write(st->regs, RP6502_ARM_IF_OFF_STATUS,
                            (rp6502_arm_if_read(st->regs, RP6502_ARM_IF_OFF_STATUS) |
                             RP6502_ARM_STATUS_ERROR) & ~RP6502_ARM_STATUS_RUNNING);
        rp6502_arm_if_write(st->regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_RUNTIME_CRASH);
    }
}

static int runtime_start(wrapper_state_t *st, const wrapper_config_t *cfg) {
    pid_t pid;

    if(!cfg->runtime_path || cfg->runtime_path[0] == '\0') {
        rp6502_arm_if_write(st->regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_BAD_PAYLOAD_PATH);
        return -1;
    }

    pid = fork();
    if(pid < 0) {
        perror("fork runtime");
        rp6502_arm_if_write(st->regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_LAUNCH_FAILED);
        return -1;
    }

    if(pid == 0) {
        if(cfg->runtime_arg) {
            execl(cfg->runtime_path, cfg->runtime_path, cfg->runtime_arg, (char *)NULL);
        } else {
            execl(cfg->runtime_path, cfg->runtime_path, (char *)NULL);
        }
        _exit(127);
    }

    st->child_pid = pid;
    return 0;
}

static void runtime_stop(wrapper_state_t *st) {
    int i;
    if(st->child_pid <= 0) return;

    kill(st->child_pid, SIGTERM);
    for(i = 0; i < 20; i++) {
        int status;
        pid_t r = waitpid(st->child_pid, &status, WNOHANG);
        if(r == st->child_pid) {
            st->child_pid = -1;
            return;
        }
        usleep(10000);
    }

    kill(st->child_pid, SIGKILL);
    waitpid(st->child_pid, NULL, 0);
    st->child_pid = -1;
}

static void handle_control(wrapper_state_t *st, const wrapper_config_t *cfg) {
    uint32_t ctrl = rp6502_arm_if_read(st->regs, RP6502_ARM_IF_OFF_CTRL);
    uint32_t status = rp6502_arm_if_read(st->regs, RP6502_ARM_IF_OFF_STATUS);

    if(ctrl & RP6502_ARM_CTRL_REQ_CLEAR_ERROR) {
        if((status & RP6502_ARM_STATUS_RUNNING) == 0) {
            status &= ~RP6502_ARM_STATUS_ERROR;
            rp6502_arm_if_write(st->regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_NONE);
        }
        ctrl &= ~RP6502_ARM_CTRL_REQ_CLEAR_ERROR;
    }

    if(ctrl & RP6502_ARM_CTRL_REQ_RESET_RUNTIME) {
        runtime_stop(st);
        status &= ~(RP6502_ARM_STATUS_RUNNING | RP6502_ARM_STATUS_ERROR);
        rp6502_arm_if_write(st->regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_NONE);
        ctrl &= ~RP6502_ARM_CTRL_REQ_RESET_RUNTIME;
    }

    if(ctrl & RP6502_ARM_CTRL_REQ_STOP) {
        runtime_stop(st);
        status &= ~RP6502_ARM_STATUS_RUNNING;
        ctrl &= ~RP6502_ARM_CTRL_REQ_STOP;
    }

    if(ctrl & RP6502_ARM_CTRL_REQ_LAUNCH) {
        if(st->child_pid <= 0) {
            if(runtime_start(st, cfg) == 0) {
                status &= ~RP6502_ARM_STATUS_ERROR;
                status |= RP6502_ARM_STATUS_RUNNING;
                rp6502_arm_if_write(st->regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_NONE);
            } else {
                status |= RP6502_ARM_STATUS_ERROR;
                status &= ~RP6502_ARM_STATUS_RUNNING;
            }
        }
        ctrl &= ~RP6502_ARM_CTRL_REQ_LAUNCH;
    }

    rp6502_arm_if_write(st->regs, RP6502_ARM_IF_OFF_CTRL, ctrl);
    rp6502_arm_if_write(st->regs, RP6502_ARM_IF_OFF_STATUS, status);
}

int main(int argc, char **argv) {
    wrapper_config_t cfg;
    wrapper_state_t st;
    struct sigaction sa;

    memset(&st, 0, sizeof(st));
    st.child_pid = -1;

    if(parse_args(argc, argv, &cfg) != 0) {
        usage(argv[0]);
        return 2;
    }

    if(map_register_file(cfg.reg_file, &st.regs) != 0) {
        return 1;
    }

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    rp6502_arm_if_write(st.regs, RP6502_ARM_IF_OFF_MAGIC, RP6502_ARM_IF_MAGIC);
    rp6502_arm_if_write(st.regs, RP6502_ARM_IF_OFF_VERSION, RP6502_ARM_IF_VERSION);
    rp6502_arm_if_write(st.regs, RP6502_ARM_IF_OFF_STATUS,
                        RP6502_ARM_STATUS_PRESENT | RP6502_ARM_STATUS_READY);
    rp6502_arm_if_write(st.regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_NONE);

    while(!g_terminate) {
        uint32_t hb = rp6502_arm_if_read(st.regs, RP6502_ARM_IF_OFF_HEARTBEAT);
        uint32_t cmd_seq = rp6502_arm_if_read(st.regs, RP6502_ARM_IF_OFF_CMD_SEQ);

        runtime_reap_if_exited(&st);

        rp6502_arm_if_write(st.regs, RP6502_ARM_IF_OFF_HEARTBEAT, hb + 1);

        if(cmd_seq != st.last_cmd_seq) {
            st.last_cmd_seq = cmd_seq;
            handle_control(&st, &cfg);
            rp6502_arm_if_write(st.regs, RP6502_ARM_IF_OFF_ACK_SEQ, cmd_seq);
        }

        usleep(cfg.poll_ms * 1000);
    }

    runtime_stop(&st);
    rp6502_arm_if_write(st.regs, RP6502_ARM_IF_OFF_STATUS, 0);
    munmap((void *)st.regs, RP6502_ARM_IF_REG_SIZE);
    return 0;
}
