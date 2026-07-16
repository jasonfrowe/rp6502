/*
 * Copyright (c) 2026 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "utest.h"

#include "arm_if_v1.h"
#include "arm_payload_v1.h"

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

static bool wait_until_eq(volatile uint32_t *regs, uint32_t off, uint32_t expected, int timeout_ms)
{
    int i;
    for (i = 0; i < timeout_ms; i++)
    {
        if (rp6502_arm_if_read(regs, off) == expected)
        {
            return true;
        }
        usleep(1000);
    }
    return false;
}

static bool wait_until_mask(volatile uint32_t *regs, uint32_t off, uint32_t mask, uint32_t expected, int timeout_ms)
{
    int i;
    for (i = 0; i < timeout_ms; i++)
    {
        if ((rp6502_arm_if_read(regs, off) & mask) == expected)
        {
            return true;
        }
        usleep(1000);
    }
    return false;
}

UTEST(mister_wrapper, launch_stop_reset_flow)
{
    char path[] = "/tmp/rp6502-wrapper-flow-XXXXXX";
    int fd;
    void *map;
    volatile uint32_t *regs;
    pid_t child;
    uint32_t hb0;
    uint32_t hb1;

    fd = mkstemp(path);
    ASSERT_GT(fd, 0);
    ASSERT_EQ(ftruncate(fd, RP6502_ARM_IF_WINDOW_SIZE), 0);

    map = mmap(NULL, RP6502_ARM_IF_WINDOW_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ASSERT_NE(map, MAP_FAILED);
    regs = (volatile uint32_t *)map;
    memset((void *)regs, 0, RP6502_ARM_IF_WINDOW_SIZE);

    child = fork();
    ASSERT_GE(child, 0);

    if (child == 0)
    {
        execl(MISTER_WRAPPER_BIN,
              MISTER_WRAPPER_BIN,
              "--transport",
              "file",
              "--reg-file",
              path,
              "--poll-ms",
              "1",
              (char *)NULL);
        _exit(127);
    }

    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_MAGIC, RP6502_ARM_IF_MAGIC, 500));
    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_VERSION, RP6502_ARM_IF_VERSION, 500));
    ASSERT_TRUE(wait_until_mask(regs,
                                RP6502_ARM_IF_OFF_STATUS,
                                RP6502_ARM_STATUS_PRESENT | RP6502_ARM_STATUS_READY,
                                RP6502_ARM_STATUS_PRESENT | RP6502_ARM_STATUS_READY,
                                500));

    strcpy((char *)regs + RP6502_ARM_PAYLOAD_V1_OFF_RUNTIME_PATH, "/bin/sleep");
    strcpy((char *)regs + RP6502_ARM_PAYLOAD_V1_OFF_RUNTIME_ARG, "5");

    hb0 = rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_HEARTBEAT);
    usleep(8000);
    hb1 = rp6502_arm_if_read(regs, RP6502_ARM_IF_OFF_HEARTBEAT);
    ASSERT_GT(hb1, hb0);

    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CTRL, RP6502_ARM_CTRL_REQ_LAUNCH);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CMD_SEQ, 1);

    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_ACK_SEQ, 1, 500));
    ASSERT_TRUE(wait_until_mask(regs,
                                RP6502_ARM_IF_OFF_STATUS,
                                RP6502_ARM_STATUS_RUNNING | RP6502_ARM_STATUS_ERROR,
                                RP6502_ARM_STATUS_RUNNING,
                                500));

    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CTRL, RP6502_ARM_CTRL_REQ_STOP);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CMD_SEQ, 2);

    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_ACK_SEQ, 2, 500));
    ASSERT_TRUE(wait_until_mask(regs,
                                RP6502_ARM_IF_OFF_STATUS,
                                RP6502_ARM_STATUS_RUNNING,
                                0,
                                500));

    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CTRL, RP6502_ARM_CTRL_REQ_RESET_RUNTIME);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CMD_SEQ, 3);

    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_ACK_SEQ, 3, 500));
    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_NONE, 500));

    ASSERT_EQ(kill(child, SIGTERM), 0);
    ASSERT_EQ(waitpid(child, NULL, 0), child);

    ASSERT_EQ(munmap((void *)regs, RP6502_ARM_IF_WINDOW_SIZE), 0);
    ASSERT_EQ(close(fd), 0);
    ASSERT_EQ(unlink(path), 0);
}

UTEST(mister_wrapper, invalid_payload_path_sets_error)
{
    char path[] = "/tmp/rp6502-wrapper-flow-XXXXXX";
    int fd;
    void *map;
    volatile uint32_t *regs;
    pid_t child;

    fd = mkstemp(path);
    ASSERT_GT(fd, 0);
    ASSERT_EQ(ftruncate(fd, RP6502_ARM_IF_WINDOW_SIZE), 0);

    map = mmap(NULL, RP6502_ARM_IF_WINDOW_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ASSERT_NE(map, MAP_FAILED);
    regs = (volatile uint32_t *)map;
    memset((void *)regs, 0, RP6502_ARM_IF_WINDOW_SIZE);

    child = fork();
    ASSERT_GE(child, 0);

    if (child == 0)
    {
        execl(MISTER_WRAPPER_BIN,
              MISTER_WRAPPER_BIN,
              "--transport",
              "file",
              "--reg-file",
              path,
              "--poll-ms",
              "1",
              (char *)NULL);
        _exit(127);
    }

    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_MAGIC, RP6502_ARM_IF_MAGIC, 500));

    strcpy((char *)regs + RP6502_ARM_PAYLOAD_V1_OFF_RUNTIME_PATH, "bin/sleep");
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CTRL, RP6502_ARM_CTRL_REQ_LAUNCH);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CMD_SEQ, 1);

    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_ACK_SEQ, 1, 500));
    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_BAD_PAYLOAD_PATH, 500));
    ASSERT_TRUE(wait_until_mask(regs,
                                RP6502_ARM_IF_OFF_STATUS,
                                RP6502_ARM_STATUS_RUNNING | RP6502_ARM_STATUS_ERROR,
                                RP6502_ARM_STATUS_ERROR,
                                500));

    ASSERT_EQ(kill(child, SIGTERM), 0);
    ASSERT_EQ(waitpid(child, NULL, 0), child);

    ASSERT_EQ(munmap((void *)regs, RP6502_ARM_IF_WINDOW_SIZE), 0);
    ASSERT_EQ(close(fd), 0);
    ASSERT_EQ(unlink(path), 0);
}

UTEST(mister_wrapper, non_nul_terminated_payload_path_sets_error)
{
    char path[] = "/tmp/rp6502-wrapper-flow-XXXXXX";
    int fd;
    void *map;
    volatile uint32_t *regs;
    pid_t child;

    fd = mkstemp(path);
    ASSERT_GT(fd, 0);
    ASSERT_EQ(ftruncate(fd, RP6502_ARM_IF_WINDOW_SIZE), 0);

    map = mmap(NULL, RP6502_ARM_IF_WINDOW_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ASSERT_NE(map, MAP_FAILED);
    regs = (volatile uint32_t *)map;
    memset((void *)regs, 0, RP6502_ARM_IF_WINDOW_SIZE);

    child = fork();
    ASSERT_GE(child, 0);

    if (child == 0)
    {
        execl(MISTER_WRAPPER_BIN,
              MISTER_WRAPPER_BIN,
              "--transport",
              "file",
              "--reg-file",
              path,
              "--poll-ms",
              "1",
              (char *)NULL);
        _exit(127);
    }

    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_MAGIC, RP6502_ARM_IF_MAGIC, 500));

    memset((char *)regs + RP6502_ARM_PAYLOAD_V1_OFF_RUNTIME_PATH, 'a', RP6502_ARM_PAYLOAD_V1_MAX_RUNTIME_PATH);
    ((char *)regs)[RP6502_ARM_PAYLOAD_V1_OFF_RUNTIME_PATH] = '/';
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CTRL, RP6502_ARM_CTRL_REQ_LAUNCH);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CMD_SEQ, 1);

    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_ACK_SEQ, 1, 500));
    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_BAD_PAYLOAD_PATH, 500));
    ASSERT_TRUE(wait_until_mask(regs,
                                RP6502_ARM_IF_OFF_STATUS,
                                RP6502_ARM_STATUS_RUNNING | RP6502_ARM_STATUS_ERROR,
                                RP6502_ARM_STATUS_ERROR,
                                500));

    ASSERT_EQ(kill(child, SIGTERM), 0);
    ASSERT_EQ(waitpid(child, NULL, 0), child);

    ASSERT_EQ(munmap((void *)regs, RP6502_ARM_IF_WINDOW_SIZE), 0);
    ASSERT_EQ(close(fd), 0);
    ASSERT_EQ(unlink(path), 0);
}

UTEST(mister_wrapper, non_printable_payload_path_sets_error)
{
    char path[] = "/tmp/rp6502-wrapper-flow-XXXXXX";
    int fd;
    void *map;
    volatile uint32_t *regs;
    pid_t child;
    char *runtime_path;

    fd = mkstemp(path);
    ASSERT_GT(fd, 0);
    ASSERT_EQ(ftruncate(fd, RP6502_ARM_IF_WINDOW_SIZE), 0);

    map = mmap(NULL, RP6502_ARM_IF_WINDOW_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ASSERT_NE(map, MAP_FAILED);
    regs = (volatile uint32_t *)map;
    memset((void *)regs, 0, RP6502_ARM_IF_WINDOW_SIZE);

    child = fork();
    ASSERT_GE(child, 0);

    if (child == 0)
    {
        execl(MISTER_WRAPPER_BIN,
              MISTER_WRAPPER_BIN,
              "--transport",
              "file",
              "--reg-file",
              path,
              "--poll-ms",
              "1",
              (char *)NULL);
        _exit(127);
    }

    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_MAGIC, RP6502_ARM_IF_MAGIC, 500));

    runtime_path = (char *)regs + RP6502_ARM_PAYLOAD_V1_OFF_RUNTIME_PATH;
    strcpy(runtime_path, "/bin/sleep");
    runtime_path[5] = '\x01';
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CTRL, RP6502_ARM_CTRL_REQ_LAUNCH);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CMD_SEQ, 1);

    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_ACK_SEQ, 1, 500));
    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_BAD_PAYLOAD_PATH, 500));
    ASSERT_TRUE(wait_until_mask(regs,
                                RP6502_ARM_IF_OFF_STATUS,
                                RP6502_ARM_STATUS_RUNNING | RP6502_ARM_STATUS_ERROR,
                                RP6502_ARM_STATUS_ERROR,
                                500));

    ASSERT_EQ(kill(child, SIGTERM), 0);
    ASSERT_EQ(waitpid(child, NULL, 0), child);

    ASSERT_EQ(munmap((void *)regs, RP6502_ARM_IF_WINDOW_SIZE), 0);
    ASSERT_EQ(close(fd), 0);
    ASSERT_EQ(unlink(path), 0);
}

UTEST(mister_wrapper, clear_error_only_when_not_running)
{
    char path[] = "/tmp/rp6502-wrapper-flow-XXXXXX";
    int fd;
    void *map;
    volatile uint32_t *regs;
    pid_t child;

    fd = mkstemp(path);
    ASSERT_GT(fd, 0);
    ASSERT_EQ(ftruncate(fd, RP6502_ARM_IF_WINDOW_SIZE), 0);

    map = mmap(NULL, RP6502_ARM_IF_WINDOW_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ASSERT_NE(map, MAP_FAILED);
    regs = (volatile uint32_t *)map;
    memset((void *)regs, 0, RP6502_ARM_IF_WINDOW_SIZE);

    child = fork();
    ASSERT_GE(child, 0);

    if (child == 0)
    {
        execl(MISTER_WRAPPER_BIN,
              MISTER_WRAPPER_BIN,
              "--transport",
              "file",
              "--reg-file",
              path,
              "--poll-ms",
              "1",
              (char *)NULL);
        _exit(127);
    }

    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_MAGIC, RP6502_ARM_IF_MAGIC, 500));

    strcpy((char *)regs + RP6502_ARM_PAYLOAD_V1_OFF_RUNTIME_PATH, "/bin/sleep");
    strcpy((char *)regs + RP6502_ARM_PAYLOAD_V1_OFF_RUNTIME_ARG, "5");

    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CTRL, RP6502_ARM_CTRL_REQ_LAUNCH);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CMD_SEQ, 1);
    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_ACK_SEQ, 1, 500));
    ASSERT_TRUE(wait_until_mask(regs,
                                RP6502_ARM_IF_OFF_STATUS,
                                RP6502_ARM_STATUS_RUNNING,
                                RP6502_ARM_STATUS_RUNNING,
                                500));

    rp6502_arm_if_write(regs,
                        RP6502_ARM_IF_OFF_STATUS,
                        RP6502_ARM_STATUS_PRESENT | RP6502_ARM_STATUS_READY |
                            RP6502_ARM_STATUS_RUNNING | RP6502_ARM_STATUS_ERROR);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_LAUNCH_FAILED);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CTRL, RP6502_ARM_CTRL_REQ_CLEAR_ERROR);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CMD_SEQ, 2);

    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_ACK_SEQ, 2, 500));
    ASSERT_TRUE(wait_until_mask(regs,
                                RP6502_ARM_IF_OFF_STATUS,
                                RP6502_ARM_STATUS_ERROR,
                                RP6502_ARM_STATUS_ERROR,
                                500));
    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_LAUNCH_FAILED, 500));

    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CTRL, RP6502_ARM_CTRL_REQ_STOP);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CMD_SEQ, 3);
    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_ACK_SEQ, 3, 500));
    ASSERT_TRUE(wait_until_mask(regs,
                                RP6502_ARM_IF_OFF_STATUS,
                                RP6502_ARM_STATUS_RUNNING,
                                0,
                                500));

    rp6502_arm_if_write(regs,
                        RP6502_ARM_IF_OFF_STATUS,
                        RP6502_ARM_STATUS_PRESENT | RP6502_ARM_STATUS_READY | RP6502_ARM_STATUS_ERROR);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_LAUNCH_FAILED);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CTRL, RP6502_ARM_CTRL_REQ_CLEAR_ERROR);
    rp6502_arm_if_write(regs, RP6502_ARM_IF_OFF_CMD_SEQ, 4);

    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_ACK_SEQ, 4, 500));
    ASSERT_TRUE(wait_until_mask(regs,
                                RP6502_ARM_IF_OFF_STATUS,
                                RP6502_ARM_STATUS_ERROR,
                                0,
                                500));
    ASSERT_TRUE(wait_until_eq(regs, RP6502_ARM_IF_OFF_LAST_ERROR, RP6502_ARM_ERR_NONE, 500));

    ASSERT_EQ(kill(child, SIGTERM), 0);
    ASSERT_EQ(waitpid(child, NULL, 0), child);

    ASSERT_EQ(munmap((void *)regs, RP6502_ARM_IF_WINDOW_SIZE), 0);
    ASSERT_EQ(close(fd), 0);
    ASSERT_EQ(unlink(path), 0);
}

UTEST_MAIN()
