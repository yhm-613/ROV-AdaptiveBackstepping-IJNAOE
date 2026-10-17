/**
 * @file    hil_selftest.c
 * @brief   Host-side (PC) self test for the HIL serial protocol.
 *
 *  Build & run from a plain shell:
 *      gcc -std=c99 -Wall -Wextra -O2 -I. hil_selftest.c hil_protocol.c -o hil_selftest
 *      ./hil_selftest
 *
 *  Exits with status 0 on success, 1 on any failure.
 *
 *  This test also acts as the reference for the Simulink host model: the
 *  CRC and framing values printed below must match what the S-Function /
 *  MATLAB-side reference produces (see scripts/hil_protocol_ref.py).
 */

#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <math.h>

#include "hil_protocol.h"

static int g_fail = 0;

#define CHECK(cond, ...)                                            \
    do {                                                            \
        if (cond) {                                                 \
            printf("  [PASS] " __VA_ARGS__);                         \
            printf("\n");                                           \
        } else {                                                    \
            printf("  [FAIL] " __VA_ARGS__);                         \
            printf("\n");                                           \
            g_fail++;                                               \
        }                                                           \
    } while (0)

/* ------------------------------------------------------------------ */

static void test_sizes(void)
{
    printf("\n== 1. Frame geometry ==\n");
    printf("  sizeof(hil_setpoint_t) = %u\n", (unsigned)sizeof(hil_setpoint_t));
    printf("  sizeof(hil_command_t)  = %u\n", (unsigned)sizeof(hil_command_t));
    CHECK(HIL_N_U == 6,                    "HIL_N_U == 6 (six thrusters)");
    CHECK(HIL_SETPOINT_PAYLOAD == 98,      "SETPOINT payload = 98 B");
    CHECK(HIL_COMMAND_PAYLOAD  == 28,      "COMMAND  payload = 28 B");
    CHECK(HIL_SETPOINT_FRAME   == 104,     "SETPOINT frame   = 104 B");
    CHECK(HIL_COMMAND_FRAME    == 34,      "COMMAND  frame   = 34 B");
}

/* ------------------------------------------------------------------ */
/* CRC16-Modbus known-answer vectors                                   */
/* ------------------------------------------------------------------ */
static void test_crc(void)
{
    printf("\n== 2. CRC16-Modbus known answers ==\n");

    /* standard Modbus check value: CRC of "123456789" = 0x4B37 */
    const uint8_t kat[] = "123456789";
    uint16_t c = hil_crc16(kat, 9);
    printf("  crc16(\"123456789\") = 0x%04X\n", c);
    CHECK(c == 0x4B37u, "CRC of \"123456789\" == 0x4B37");

    const uint8_t z[] = { 0x00 };
    printf("  crc16({0x00})       = 0x%04X\n", hil_crc16(z, 1));

    const uint8_t s2[] = { 0x55, 0xAA, 0x02, 0x1C };
    printf("  crc16(55 AA 02 1C)  = 0x%04X\n", hil_crc16(s2, 4));
}

/* ------------------------------------------------------------------ */
static void test_roundtrip(void)
{
    printf("\n== 3. Round trip (SETPOINT) ==\n");

    hil_setpoint_t sp, back;
    uint8_t frame[HIL_SETPOINT_FRAME];
    int n;

    for (int i = 0; i < HIL_N_ETA; ++i) {
        sp.eta[i]   = (float)(i + 1) * 0.125f;
        sp.eta_d[i] = (float)(i + 1) * 0.250f;
    }
    for (int i = 0; i < HIL_N_NU; ++i) {
        sp.nu[i]    = -(float)(i + 1) * 0.03125f;
        sp.nu_d[i]  =  (float)(i + 1) * 0.0625f;
    }
    sp.seq = 0x1234u;

    n = hil_pack_setpoint(&sp, frame);
    CHECK(n == (int)HIL_SETPOINT_FRAME, "pack returns %d", n);
    CHECK(frame[0] == HIL_SYNC0 && frame[1] == HIL_SYNC1, "sync bytes 55 AA");
    CHECK(frame[2] == HIL_TYPE_SETPOINT, "type = 0x01");

    {
        uint16_t rx  = (uint16_t)(frame[n - 2] | ((uint16_t)frame[n - 1] << 8));
        uint16_t cal = hil_crc16(frame, (size_t)(n - 2));
        CHECK(rx == cal, "CRC in frame = 0x%04X == computed", rx);
    }

    memset(&back, 0, sizeof(back));
    hil_unpack_setpoint(frame, &back);
    /* 只比较「有意义的字节」= offsetof(seq) + sizeof(seq) == 98。
     * 不能用 sizeof(sp)（= 100）：hil_setpoint_t 末尾有 2 字节对齐填充，
     * hil_unpack_setpoint() 不会写它们，sp 的这 2 字节也从未被赋值，
     * 于是 memcmp 会去比未初始化的栈字节 —— 结果随编译器/栈布局变化
     * （GCC 下恰好为 0 所以通过，TCC 下就 FAIL）。这是个脆弱断言，不是协议问题。 */
    CHECK(memcmp(&sp, &back, offsetof(hil_setpoint_t, seq) + sizeof(sp.seq)) == 0,
          "unpack(pack(x)) == x (bit exact, 98 meaningful bytes)");
    CHECK(back.seq == 0x1234u, "sequence counter preserved");
}

/* ------------------------------------------------------------------ */
static void test_bytewise_parser(void)
{
    printf("\n== 4. Byte-wise incremental parser ==\n");

    hil_setpoint_t sp, out;
    hil_parser_t   ps;
    uint8_t        frame[HIL_SETPOINT_FRAME];
    uint8_t        stream[3 * HIL_SETPOINT_FRAME + 7];

    for (int i = 0; i < HIL_N_ETA; ++i) { sp.eta[i] = 1.5f; sp.eta_d[i] = 2.5f; }
    for (int i = 0; i < HIL_N_NU;  ++i) { sp.nu[i]  = 0.5f; sp.nu_d[i]  = 1.0f; }
    sp.seq = 7u;
    hil_pack_setpoint(&sp, frame);

    /* --- 4a: clean stream of 3 frames, one byte at a time --- */
    hil_parser_init(&ps);
    int hits = 0;
    for (int f = 0; f < 3; ++f) {
        for (int b = 0; b < HIL_SETPOINT_FRAME; ++b) {
            if (hil_parser_feed(&ps, frame[b], &out)) { hits++; }
        }
    }
    CHECK(hits == 3, "3 clean frames -> %d decoded", hits);
    CHECK(ps.crc_err == 0 && ps.frame_err == 0, "no CRC / framing errors");
    CHECK(out.seq == 7u, "last decoded seq = %u", out.seq);

    /* --- 4b: leading garbage must be skipped --- */
    hil_parser_init(&ps);
    hits = 0;
    for (int b = 0; b < 9; ++b) { hil_parser_feed(&ps, (uint8_t)(b + 1), &out); }
    for (int b = 0; b < HIL_SETPOINT_FRAME; ++b) { if (hil_parser_feed(&ps, frame[b], &out)) hits++; }
    CHECK(hits == 1, "garbage + 1 frame -> %d decoded", hits);

    /* --- 4c: a corrupted byte must cost exactly one frame --- */
    hil_parser_init(&ps);
    hits = 0;
    {
        uint8_t bad[HIL_SETPOINT_FRAME];
        memcpy(bad, frame, sizeof(bad));
        bad[10] ^= 0xFFu;                     /* flip a payload byte */
        for (int b = 0; b < HIL_SETPOINT_FRAME; ++b) hil_parser_feed(&ps, bad[b],  &out);
        for (int b = 0; b < HIL_SETPOINT_FRAME; ++b) { if (hil_parser_feed(&ps, frame[b], &out)) hits++; }
    }
    CHECK(hits == 1, "corrupt frame then good frame -> %d decoded", hits);
    CHECK(ps.crc_err == 1, "exactly 1 CRC error recorded (%u)", ps.crc_err);

    /* --- 4d: 55 55 AA AA .. resync tolerance --- */
    hil_parser_init(&ps);
    hits = 0;
    hil_parser_feed(&ps, 0x55u, &out);
    hil_parser_feed(&ps, 0x55u, &out);
    hil_parser_feed(&ps, 0xAAu, &out);
    for (int b = 2; b < HIL_SETPOINT_FRAME; ++b) { if (hil_parser_feed(&ps, frame[b], &out)) hits++; }
    CHECK(hits == 1, "duplicated SYNC0 still resyncs -> %d decoded", hits);

    /* --- 4e: wrong LEN rejected --- */
    hil_parser_init(&ps);
    hits = 0;
    {
        uint8_t bad[HIL_SETPOINT_FRAME];
        memcpy(bad, frame, sizeof(bad));
        bad[3] = 42u;
        for (int b = 0; b < HIL_SETPOINT_FRAME; ++b) { if (hil_parser_feed(&ps, bad[b], &out)) hits++; }
    }
    CHECK(hits == 0 && ps.frame_err == 1, "bad LEN rejected (%u framing err)", ps.frame_err);

    (void)stream;
}

/* ------------------------------------------------------------------ */
static void test_command_roundtrip(void)
{
    printf("\n== 5. Round trip (COMMAND) ==\n");

    hil_command_t cmd;
    uint8_t       frame[HIL_COMMAND_FRAME];
    int           n;

    for (int i = 0; i < HIL_N_U; ++i) { cmd.u[i] = (float)(10 * (i + 1)); }
    cmd.u[5] = -100.0f;
    cmd.seq    = 0xBEEFu;
    cmd.status = HIL_ST_INIT_DONE | HIL_ST_DATA_FRESH | HIL_ST_SATURATED;

    n = hil_pack_command(&cmd, frame);
    CHECK(n == (int)HIL_COMMAND_FRAME, "pack returns %d (expect %d)", n, (int)HIL_COMMAND_FRAME);
    CHECK(frame[2] == HIL_TYPE_COMMAND, "type = 0x02");
    CHECK(frame[3] == (uint8_t)HIL_COMMAND_PAYLOAD, "LEN = %u", frame[3]);

    {
        const uint8_t *p = frame + 4;
        int ok = 1;
        for (int i = 0; i < HIL_N_U; ++i) {
            float v; memcpy(&v, p + 4 * i, 4u);
            if (v != cmd.u[i]) { ok = 0; }
        }
        CHECK(ok, "6 thruster voltages survive byte packing");
    }
    {
        uint16_t rx  = (uint16_t)(frame[n - 2] | ((uint16_t)frame[n - 1] << 8));
        uint16_t cal = hil_crc16(frame, (size_t)(n - 2));
        CHECK(rx == cal, "COMMAND CRC = 0x%04X", rx);
    }
}

/* ------------------------------------------------------------------ */
static void test_bandwidth(void)
{
    printf("\n== 6. Bandwidth budget @ 100 Hz ==\n");
    const int per_frame = HIL_SETPOINT_FRAME + HIL_COMMAND_FRAME;
    const int per_sec   = per_frame * 100;
    printf("  bytes/cycle  = %d\n", per_frame);
    printf("  bytes/s      = %d  (%.1f kB/s)\n", per_sec, per_sec / 1000.0);
    {
        const double bits = per_sec * 10.0;   /* 8N1 -> 10 bit per byte */
        printf("  needed baud  = %.0f  (8N1)\n", bits);
        printf("  115200 -> %.2f x demand  (INSUFFICIENT)\n", 115200.0 / bits);
        printf("  230400 -> %.2f x demand  (minimum)\n",      230400.0 / bits);
        printf("  460800 -> %.2f x demand  (recommended)\n",  460800.0 / bits);
        printf("  921600 -> %.2f x demand  (safe margin)\n",  921600.0 / bits);
        CHECK(bits > 115200.0, "115200 baud is NOT enough");
        CHECK(bits <= 230400.0,
              "230400 baud sufficient (utilisation %.1f%%)", 100.0 * bits / 230400.0);
        CHECK(bits <= 921600.0,
              "921600 baud ample (utilisation %.1f%%)", 100.0 * bits / 921600.0);
    }
}

/* ------------------------------------------------------------------ */

int main(void)
{
    printf("==================================================\n");
    printf(" HIL protocol self test  (host side)\n");
    printf("==================================================\n");

    test_sizes();
    test_crc();
    test_roundtrip();
    test_bytewise_parser();
    test_command_roundtrip();
    test_bandwidth();

    printf("\n==================================================\n");
    if (g_fail == 0) {
        printf(" RESULT: ALL PASS\n");
    } else {
        printf(" RESULT: %d FAILURE(S)\n", g_fail);
    }
    printf("==================================================\n");
    return g_fail == 0 ? 0 : 1;
}
