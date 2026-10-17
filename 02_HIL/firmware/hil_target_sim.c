/**
 * @file    hil_target_sim.c
 * @brief   Serial-attached VIRTUAL TARGET — plays the STM32 on the PC.
 *
 * ---------------------------------------------------------------------------
 *  WHY THIS EXISTS
 *
 *  The real HIL loop needs an STM32F407 on the other end of a serial cable.
 *  This program removes that dependency: it opens a COM port, waits for
 *  104-byte SETPOINT frames, runs the SAME generated controller, and sends the
 *  34-byte COMMAND frame back — exactly what HIL_Step10ms() does on the board.
 *
 *  Pair it with a virtual null-modem (com0com) and Simulink talks to "COM10"
 *  while this program sits on "COM11".  Everything above the UART — frame
 *  sync, CRC, the sequence echo, the timeout policy, the blocking round trip
 *  inside hil_bridge_host.m — gets exercised with no hardware at all.
 *
 *  THE KEY POINT: this file does NOT re-implement the target logic.  It links
 *  the real hil_target.c and only substitutes the four hil_port_* hooks with
 *  Win32 serial-port code.  So what you validate here is the same run-time
 *  source that will be flashed, not a look-alike.
 *
 *      +----------------------------+        +----------------------------+
 *      |  rov_hil_host.slx          |        |  hil_target_sim.exe        |
 *      |  Controller_D (10 ms)      |        |                            |
 *      |  hil_bridge_host.m         | COM10  |  COM11                     |
 *      |    write 104 B ------------+------->+---> hil_parser_feed()      |
 *      |    block for 34 B  <-------+--------+---- hil_pack_command()     |
 *      |                            |        |   hil_target.c             |
 *      +----------------------------+        |   + rov_ctrl_deploy.c      |
 *                                            +----------------------------+
 *
 *  Links against: hil_protocol.c, hil_target.c,
 *                 rov_ctrl_deploy.c, rov_ctrl_deploy_data.c
 *  Does NOT link hil_port_stm32f4.c -- this file provides its own hil_port_*.
 *
 *  Build (MSYS2 / Git-Bash, needs -lwinmm for timeBeginPeriod):
 *      gcc -std=c99 -O2 -I. -I../rov_ctrl_deploy_ert_rtw \
 *          hil_target_sim.c hil_protocol.c hil_target.c \
 *          ../rov_ctrl_deploy_ert_rtw/rov_ctrl_deploy.c \
 *          ../rov_ctrl_deploy_ert_rtw/rov_ctrl_deploy_data.c \
 *          -lm -lwinmm -o hil_target_sim.exe
 *
 *  Run:
 *      ./hil_target_sim.exe COM11 460800
 *
 *  See docs/虚拟串口对测试.md for the com0com set-up and the expected output.
 * ---------------------------------------------------------------------------
 */

#include <windows.h>
#include <mmsystem.h>          /* timeBeginPeriod / timeEndPeriod */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>

#include "hil_protocol.h"
#include "hil_target.h"        /* the real target run-time: HIL_Init,
                                  HIL_OnByte, HIL_Step10ms              */
#include "hil_port.h"          /* the hooks this file implements         */

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */

static HANDLE         g_com   = INVALID_HANDLE_VALUE;
static uint32_t       g_t0    = 0u;      /* GetTickCount() at start-up    */
static volatile int   g_stop  = 0;

#define HIL_TICK_MS    (10u)             /* control period, matches TIM6   */

/* --selftest mode: outgoing frames are captured in RAM instead of being
 * written to a port, so the whole chain (parser -> generated controller ->
 * COMMAND frame) can be verified without any serial hardware at all.       */
#define HIL_CAP_FRAMES (16)
static int     g_capture = 0;
static int     g_cap_n   = 0;
static uint8_t g_cap[HIL_CAP_FRAMES][HIL_COMMAND_FRAME];

/* ------------------------------------------------------------------ */
/* hil_port_* implementation over the Win32 serial API                 */
/* ------------------------------------------------------------------ */

/**
 * @brief  Open a COM port in raw 8N1 mode with non-blocking reads.
 *
 * The "\\\\.\\" prefix is required for COM10 and above; without it
 * CreateFile() fails on any two-digit port name.
 *
 * ReadIntervalTimeout = MAXDWORD with the two total-timeout fields at 0 is the
 * documented way to make ReadFile() return immediately with whatever bytes are
 * already buffered instead of waiting for the requested count.
 */
static HANDLE open_com(const char *port, uint32_t baud)
{
    char   path[64];
    DCB    dcb;
    COMMTIMEOUTS to;

    snprintf(path, sizeof(path), "\\\\.\\%s", port);

    HANDLE h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE,
                           0, NULL, OPEN_EXISTING, 0, NULL);
    if (h == INVALID_HANDLE_VALUE) {
        return INVALID_HANDLE_VALUE;
    }

    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) { CloseHandle(h); return INVALID_HANDLE_VALUE; }

    dcb.BaudRate     = baud;
    dcb.ByteSize     = 8;
    dcb.Parity       = NOPARITY;
    dcb.StopBits     = ONESTOPBIT;
    dcb.fBinary      = TRUE;
    dcb.fParity      = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl  = DTR_CONTROL_DISABLE;
    dcb.fRtsControl  = RTS_CONTROL_DISABLE;
    dcb.fOutX        = FALSE;
    dcb.fInX         = FALSE;
    if (!SetCommState(h, &dcb)) { CloseHandle(h); return INVALID_HANDLE_VALUE; }

    memset(&to, 0, sizeof(to));
    to.ReadIntervalTimeout        = MAXDWORD;   /* -> return what is there    */
    to.ReadTotalTimeoutMultiplier = 0u;
    to.ReadTotalTimeoutConstant   = 0u;
    SetCommTimeouts(h, &to);

    SetupComm(h, 8192u, 8192u);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return h;
}

/**
 * @brief  One-off hardware setup.  The port itself is opened by main() so that
 *         a bad port name produces a clean error message before HIL_Init().
 */
void hil_port_init(void)
{
    /* nothing to do: g_com is already open */
}

/**
 * @brief  Transmit @p len bytes.  Called from HIL_Step10ms() once per tick.
 * @return number of bytes written, or -1.
 *
 * 34 bytes at 460800 baud take ~0.74 ms, so a blocking WriteFile() is fine
 * inside the 10 ms budget.  The real firmware uses DMA + a ring buffer
 * instead, but the wire result is identical.
 */
int hil_port_uart_write(const uint8_t *buf, uint16_t len)
{
    DWORD written = 0u;

    /* --selftest: divert to the capture buffer ------------------------- */
    if (g_capture != 0) {
        if ((g_cap_n < HIL_CAP_FRAMES) && (len == (uint16_t)HIL_COMMAND_FRAME)) {
            memcpy(g_cap[g_cap_n], buf, (size_t)len);
            g_cap_n++;
        }
        return (int)len;
    }

    if (g_com == INVALID_HANDLE_VALUE) { return -1; }
    if (!WriteFile(g_com, buf, (DWORD)len, &written, NULL)) { return -1; }
    return (int)written;
}

/**
 * @brief  Hand the six clamped thruster voltages to the plant.
 *
 * In pure HIL the plant lives in Simulink, so there is nothing to drive —
 * the voltages only ever leave this program inside the COMMAND frame.
 */
void hil_port_apply_u(const volatile float *u, int n)
{
    (void)u;
    (void)n;
}

/**
 * @brief  Free-running millisecond counter.
 *
 * GetTickCount() has 10-16 ms granularity by default; timeBeginPeriod(1) in
 * main() raises it to ~1 ms, which is what makes the 10 ms cadence stable.
 */
uint32_t hil_port_millis(void)
{
    return (uint32_t)(GetTickCount() - g_t0);
}

/* The three hooks below exist only in the DMA/IDLE world of the real board.
 * On the PC the receive path is a simple non-blocking ReadFile() poll, so
 * these are provided as no-ops to satisfy the link.                     */

void hil_port_rx_start(void)     { }
void hil_port_rx_poll_idle(void) { }
void hil_port_tim6_start(void)   { }
void hil_port_tim6_isr(void)     { }

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/**
 * @brief  Drain whatever the port has buffered and feed it to the parser.
 * @return number of bytes consumed.
 */
static int pump_rx(void)
{
    uint8_t buf[512];
    DWORD   n = 0u;
    DWORD   i;

    if (g_com == INVALID_HANDLE_VALUE) { return 0; }
    if (!ReadFile(g_com, buf, (DWORD)sizeof(buf), &n, NULL)) { return 0; }

    for (i = 0u; i < n; ++i) {
        HIL_OnByte(buf[i]);
    }
    return (int)n;
}

static void on_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

static void print_usage(const char *exe)
{
    printf("usage: %s [COMx] [baud]\n", exe);
    printf("       %s --selftest\n", exe);
    printf("       defaults: COM11 460800\n");
    printf("\n");
    printf("  The port must be the SECOND half of a com0com pair; Simulink\n");
    printf("  (hil_bridge_host.m -> HIL_PORT) talks to the other half.\n");
    printf("\n");
    printf("  --selftest runs the whole chain in RAM (no COM port needed):\n");
    printf("  pack a SETPOINT -> parser -> generated controller -> COMMAND.\n");
}

/* ------------------------------------------------------------------ */
/* --selftest : verify the whole chain without a serial port           */
/* ------------------------------------------------------------------ */

static int g_fail = 0;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        if (cond) { printf("  [OK]   "); }                                   \
        else      { printf("  [FAIL] "); g_fail++; }                         \
        printf(__VA_ARGS__);                                                \
        printf("\n");                                                       \
    } while (0)

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int selftest(void)
{
    hil_setpoint_t sp;
    uint8_t        frame[HIL_SETPOINT_FRAME];
    int            i, k;

    printf("==============================================================\n");
    printf(" hil_target_sim --selftest   (no serial port involved)\n");
    printf("==============================================================\n");

    /* ---- 1. build one setpoint -------------------------------------- */
    printf("\n[1] pack a SETPOINT frame\n");
    memset(&sp, 0, sizeof(sp));
    for (i = 0; i < HIL_N_ETA; ++i) {
        sp.eta[i]   = 0.50f * (float)(i + 1);
        sp.nu[i]    = 0.10f * (float)(i + 1);
        sp.eta_d[i] = 0.25f * (float)(i + 1);
        sp.nu_d[i]  = 0.05f * (float)(i + 1);
    }
    sp.seq = 1234u;
    k = hil_pack_setpoint(&sp, frame);
    CHECK(k == (int)HIL_SETPOINT_FRAME, "frame length = %d (expect %d)",
          k, HIL_SETPOINT_FRAME);

    /* ---- 2. start the target ---------------------------------------- */
    printf("\n[2] HIL_Init()\n");
    g_capture = 1;
    g_cap_n   = 0;
    HIL_Init();
    CHECK(g_hil_tick_cnt == 0u, "tick counter starts at 0");

    /* ---- 3. feed the frame byte by byte ----------------------------- */
    printf("\n[3] feed %d bytes through hil_parser_feed()\n", k);
    for (i = 0; i < k; ++i) { HIL_OnByte(frame[i]); }
    CHECK(g_hil_rx_ok    == 1u, "one setpoint accepted");
    CHECK(g_hil_rx_crc   == 0u, "no CRC error");
    CHECK(g_hil_rx_frame == 0u, "no framing error");
    CHECK(g_cap_n        == 0,  "nothing transmitted before the first tick");

    /* ---- 4. one control period -------------------------------------- */
    printf("\n[4] HIL_Step10ms()\n");
    HIL_Step10ms();
    CHECK(g_hil_tick_cnt == 1u, "tick counter advanced");
    CHECK(g_hil_tx_cnt   == 1u, "one COMMAND frame sent");
    CHECK(g_cap_n        == 1,  "captured 1 frame of %d bytes", HIL_COMMAND_FRAME);

    /* ---- 5. inspect the reply --------------------------------------- */
    printf("\n[5] validate the COMMAND frame\n");
    if (g_cap_n == 1) {
        const uint8_t *f   = g_cap[0];
        uint16_t       crc = hil_crc16(f, (size_t)(HIL_COMMAND_FRAME - 2));
        uint16_t       rxc = rd_u16(&f[HIL_COMMAND_FRAME - 2]);
        uint16_t       seq = rd_u16(&f[HIL_CMD_OFF_SEQ]);
        uint16_t       st  = rd_u16(&f[HIL_CMD_OFF_STATUS]);

        CHECK(f[0] == HIL_SYNC0 && f[1] == HIL_SYNC1, "sync bytes 55 AA");
        CHECK(f[2] == HIL_TYPE_COMMAND, "type = COMMAND (0x%02X)", (unsigned)f[2]);
        CHECK(f[3] == (uint8_t)HIL_COMMAND_PAYLOAD, "payload len = %u", (unsigned)f[3]);
        CHECK(crc == rxc, "CRC16 matches (0x%04X)", (unsigned)crc);
        CHECK(seq == 1234u, "seq echoed back = %u", (unsigned)seq);
        CHECK((st & HIL_ST_INIT_DONE)  != 0u, "status INIT_DONE set");
        CHECK((st & HIL_ST_DATA_FRESH) != 0u, "status DATA_FRESH set");
        CHECK((st & HIL_ST_TIMEOUT)    == 0u, "status TIMEOUT clear");
        CHECK((st & HIL_ST_RX_CRC_ERR) == 0u, "status RX_CRC_ERR clear");
    }

    /* ---- 6. sustained run ------------------------------------------- */
    printf("\n[6] run 200 further periods, a fresh setpoint every time\n");
    {
        int      bad_crc = 0, bad_seq = 0, worst = 0;
        uint16_t last_status = 0u;

        for (i = 0; i < 200; ++i) {
            int j;

            sp.seq = (uint16_t)(2000 + i);
            (void)hil_pack_setpoint(&sp, frame);
            for (k = 0; k < (int)HIL_SETPOINT_FRAME; ++k) { HIL_OnByte(frame[k]); }

            g_cap_n = 0;
            HIL_Step10ms();

            if (g_cap_n != 1) { bad_seq++; continue; }

            {
                const uint8_t *f = g_cap[0];
                uint16_t c = hil_crc16(f, (size_t)(HIL_COMMAND_FRAME - 2));
                uint16_t r = rd_u16(&f[HIL_COMMAND_FRAME - 2]);
                uint16_t q = rd_u16(&f[HIL_CMD_OFF_SEQ]);
                if (c != r)                  { bad_crc++; }
                if (q != (uint16_t)(2000 + i)) { bad_seq++; }
                last_status = rd_u16(&f[HIL_CMD_OFF_STATUS]);
            }
            for (j = 0; j < HIL_N_U; ++j) {
                float v = g_hil_last_u[j];
                int   a = (v < 0.0f) ? (int)(-v) : (int)v;
                if (a > worst) { worst = a; }
            }
        }

        CHECK(g_hil_tick_cnt == 201u, "201 ticks executed (%lu)",
              (unsigned long)g_hil_tick_cnt);
        CHECK(bad_crc == 0, "all 200 reply CRCs valid");
        CHECK(bad_seq == 0, "all 200 sequence echoes correct");
        CHECK((last_status & HIL_ST_TIMEOUT)    == 0u, "TIMEOUT still clear");
        CHECK((last_status & HIL_ST_DATA_FRESH) != 0u, "DATA_FRESH still set");
        printf("  [info] worst |u| seen = %d V (saturation limit 100 V)\n", worst);
    }

    /* ---- 7. timeout policy ------------------------------------------ */
    printf("\n[7] timeout policy: stop feeding, watch the status bit\n");
    for (i = 0; i < 10; ++i) {
        g_cap_n = 0;
        HIL_Step10ms();
    }
    if (g_cap_n == 1) {
        uint16_t st = rd_u16(&g_cap[0][HIL_CMD_OFF_STATUS]);
        CHECK((st & HIL_ST_DATA_FRESH) == 0u, "DATA_FRESH cleared");
        CHECK((st & HIL_ST_TIMEOUT)    != 0u,
              "TIMEOUT raised after %u ms of silence", HIL_SETPOINT_TIMEOUT_MS);
    } else {
        printf("  [FAIL] no frame captured in the timeout test\n");
        g_fail++;
    }

    printf("\n==============================================================\n");
    if (g_fail == 0) {
        printf(" RESULT: ALL PASS\n");
    } else {
        printf(" RESULT: %d CHECK(S) FAILED\n", g_fail);
    }
    printf("==============================================================\n");
    return (g_fail == 0) ? 0 : 1;
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    const char *port = (argc > 1) ? argv[1] : "COM11";
    uint32_t    baud = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 10)
                                  : 460800u;

    uint32_t next_tick;
    uint32_t next_report;

    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        print_usage(argv[0]);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "--selftest") == 0) {
        return selftest();
    }

    g_com = open_com(port, baud);
    if (g_com == INVALID_HANDLE_VALUE) {
        fprintf(stderr,
                "[ERR] cannot open %s @ %u bps (GetLastError = %lu)\n"
                "      - is the com0com pair created?\n"
                "      - is another program (Simulink, a terminal) holding it?\n"
                "      - does the name need the \\\\.\\ prefix? (handled here)\n",
                port, baud, (unsigned long)GetLastError());
        return 2;
    }

    g_t0 = GetTickCount();
    timeBeginPeriod(1u);          /* 1 ms timer resolution for Sleep() */
    signal(SIGINT, on_sigint);

    printf("==============================================================\n");
    printf(" HIL virtual target  (plays the STM32F407)\n");
    printf("   port      : %s @ %u bps, 8N1, no flow control\n", port, baud);
    printf("   tick      : every %u ms\n", HIL_TICK_MS);
    printf("   protocol  : 104 B in / 34 B out, CRC16-Modbus\n");
    printf("   Ctrl-C to stop.\n");
    printf("==============================================================\n");

    /* HIL_Init() -> rov_ctrl_deploy_initialize() + hil_port_init() */
    HIL_Init();

    next_tick   = g_t0 + HIL_TICK_MS;
    next_report = g_t0 + 2000u;

    while (g_stop == 0) {
        uint32_t now;
        int32_t  behind;

        /* ---- receive ------------------------------------------------- */
        pump_rx();

        /* ---- one control period -------------------------------------- */
        now = GetTickCount();
        behind = (int32_t)(now - next_tick);
        if (behind >= 0) {
            /* If the host machine stalled longer than one period, resync
             * instead of trying to catch up in a burst of ticks — a burst
             * would look to the PC like a run of duplicate frames.        */
            next_tick = (behind > (int32_t)HIL_TICK_MS)
                      ? (now + HIL_TICK_MS)
                      : (next_tick + HIL_TICK_MS);

            /* HIL_Step10ms() also calls hil_port_uart_write() to send the
             * 34-byte COMMAND frame back, using the same code the board
             * runs.                                                       */
            HIL_Step10ms();
        }

        /* ---- periodic diagnostics ------------------------------------ */
        if ((int32_t)(now - next_report) >= 0) {
            next_report = now + 2000u;
            printf("[SIM] tick=%-7lu rx_ok=%-6lu crc_err=%-4lu frame_err=%-4lu "
                   "drop=%-4lu tx=%-7lu u=[%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f]\n",
                   (unsigned long)g_hil_tick_cnt,
                   (unsigned long)g_hil_rx_ok,
                   (unsigned long)g_hil_rx_crc,
                   (unsigned long)g_hil_rx_frame,
                   (unsigned long)g_hil_rx_drop,
                   (unsigned long)g_hil_tx_cnt,
                   (double)g_hil_last_u[0], (double)g_hil_last_u[1],
                   (double)g_hil_last_u[2], (double)g_hil_last_u[3],
                   (double)g_hil_last_u[4], (double)g_hil_last_u[5]);
            fflush(stdout);
        }

        Sleep(1u);
    }

    /* ---- shutdown ---------------------------------------------------- */
    printf("\n[SIM] stopped.  ticks=%lu  setpoints ok=%lu  crc_err=%lu  "
           "frame_err=%lu  dropped=%lu  frames sent=%lu\n",
           (unsigned long)g_hil_tick_cnt, (unsigned long)g_hil_rx_ok,
           (unsigned long)g_hil_rx_crc, (unsigned long)g_hil_rx_frame,
           (unsigned long)g_hil_rx_drop, (unsigned long)g_hil_tx_cnt);

    timeEndPeriod(1u);
    if (g_com != INVALID_HANDLE_VALUE) { CloseHandle(g_com); }
    return 0;
}
