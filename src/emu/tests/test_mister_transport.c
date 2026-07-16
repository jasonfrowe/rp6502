/*
 * Copyright (c) 2026 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "utest.h"

#include "arm_if_v1.h"
#include "transport.h"

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

UTEST(mister_transport, file_backed_rw)
{
    char path[] = "/tmp/rp6502-arm-if-test-XXXXXX";
    int fd;
    struct stat st;
    mister_transport_config_t cfg;
    mister_transport_t tp;

    fd = mkstemp(path);
    ASSERT_GT(fd, 0);
    close(fd);

    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = MISTER_TRANSPORT_FILE;
    cfg.reg_file = path;

    ASSERT_EQ(mister_transport_open(&cfg, &tp), 0);
    ASSERT_TRUE(tp.regs != NULL);

    rp6502_arm_if_write(tp.regs, RP6502_ARM_IF_OFF_MAGIC, RP6502_ARM_IF_MAGIC);
    rp6502_arm_if_write(tp.regs, RP6502_ARM_IF_OFF_VERSION, RP6502_ARM_IF_VERSION);
    rp6502_arm_if_write(tp.regs, RP6502_ARM_IF_OFF_CMD_SEQ, 7u);

    ASSERT_EQ(rp6502_arm_if_read(tp.regs, RP6502_ARM_IF_OFF_MAGIC), RP6502_ARM_IF_MAGIC);
    ASSERT_EQ(rp6502_arm_if_read(tp.regs, RP6502_ARM_IF_OFF_VERSION), RP6502_ARM_IF_VERSION);
    ASSERT_EQ(rp6502_arm_if_read(tp.regs, RP6502_ARM_IF_OFF_CMD_SEQ), 7u);

    ASSERT_EQ(fstat(tp.fd, &st), 0);
    ASSERT_EQ((uint32_t)st.st_size, (uint32_t)RP6502_ARM_IF_REG_SIZE);

    mister_transport_close(&tp);
    ASSERT_EQ(unlink(path), 0);
}

UTEST_MAIN()
