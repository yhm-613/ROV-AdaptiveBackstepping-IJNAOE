/**
 * @file    hil_protocol.c
 * @brief   Serialisation and incremental frame parser for the HIL protocol.
 *
 * Deliberately free of HAL / RTOS dependencies so the same code can be
 * unit-tested on the host (see hil_selftest.c).
 */

#include "hil_protocol.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* Little-endian primitives (memcpy avoids unaligned access faults)     */
/* ------------------------------------------------------------------ */

static void put_f32(uint8_t *p, float v)   { memcpy(p, &v, 4u); }
static float get_f32(const uint8_t *p)     { float v; memcpy(&v, p, 4u); return v; }

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* ------------------------------------------------------------------ */
/* Packing / unpacking                                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief  Build a COMMAND frame (MCU -> PC) into @p out.
 * @return total frame length in bytes (HIL_COMMAND_FRAME).
 */
int hil_pack_command(const hil_command_t *c, uint8_t *out)
{
    uint8_t *p = out;

    p[0] = HIL_SYNC0;
    p[1] = HIL_SYNC1;
    p[2] = HIL_TYPE_COMMAND;
    p[3] = (uint8_t)HIL_COMMAND_PAYLOAD;
    p += 4;

    for (int i = 0; i < HIL_N_U; ++i) { put_f32(p, c->u[i]); p += 4; }
    put_u16(p, c->seq);    p += 2;
    put_u16(p, c->status); p += 2;

    {
        uint16_t crc = hil_crc16(out, (size_t)(HIL_COMMAND_FRAME - 2u));
        put_u16(p, crc);
    }
    return (int)HIL_COMMAND_FRAME;
}

/**
 * @brief  Unpack a validated SETPOINT frame (header + payload + CRC present).
 * @return 1 on success.
 */
int hil_unpack_setpoint(const uint8_t *frame, hil_setpoint_t *sp)
{
    const uint8_t *p = frame + 4;

    for (int i = 0; i < HIL_N_ETA; ++i) { sp->eta[i]   = get_f32(p); p += 4; }
    for (int i = 0; i < HIL_N_NU;  ++i) { sp->nu[i]    = get_f32(p); p += 4; }
    for (int i = 0; i < HIL_N_ETA; ++i) { sp->eta_d[i] = get_f32(p); p += 4; }
    for (int i = 0; i < HIL_N_NU;  ++i) { sp->nu_d[i]  = get_f32(p); p += 4; }
    sp->seq = get_u16(p);

    return 1;
}

/**
 * @brief  Build a SETPOINT frame.  Only needed by the host-side self test.
 * @return total frame length in bytes (HIL_SETPOINT_FRAME).
 */
int hil_pack_setpoint(const hil_setpoint_t *sp, uint8_t *out)
{
    uint8_t *p = out;

    p[0] = HIL_SYNC0;
    p[1] = HIL_SYNC1;
    p[2] = HIL_TYPE_SETPOINT;
    p[3] = (uint8_t)HIL_SETPOINT_PAYLOAD;
    p += 4;

    for (int i = 0; i < HIL_N_ETA; ++i) { put_f32(p, sp->eta[i]);   p += 4; }
    for (int i = 0; i < HIL_N_NU;  ++i) { put_f32(p, sp->nu[i]);    p += 4; }
    for (int i = 0; i < HIL_N_ETA; ++i) { put_f32(p, sp->eta_d[i]); p += 4; }
    for (int i = 0; i < HIL_N_NU;  ++i) { put_f32(p, sp->nu_d[i]);  p += 4; }
    put_u16(p, sp->seq); p += 2;

    {
        uint16_t crc = hil_crc16(out, (size_t)(HIL_SETPOINT_FRAME - 2u));
        put_u16(p, crc);
    }
    return (int)HIL_SETPOINT_FRAME;
}

/* ------------------------------------------------------------------ */
/* Incremental byte-wise parser                                        */
/* ------------------------------------------------------------------ */

void hil_parser_init(hil_parser_t *ps)
{
    memset(ps, 0, sizeof(*ps));
}

/**
 * @brief  Feed one received byte.
 * @return 1 when a complete, CRC-valid SETPOINT frame has been decoded
 *         into @p out (only meaningful for a SETPOINT-expecting parser).
 *
 * Resynchronisation: any malformed byte sequence restarts the search for
 * 0x55 0xAA, so a single corrupted frame costs at most one frame period.
 */
int hil_parser_feed(hil_parser_t *ps, uint8_t b, hil_setpoint_t *out)
{
    ps->byte_cnt++;

    switch (ps->idx) {

    case 0:                                   /* waiting for SYNC0 */
        if (b == HIL_SYNC0) {
            ps->buf[0] = b;
            ps->idx = 1u;
        }
        return 0;

    case 1:                                   /* waiting for SYNC1 */
        if (b == HIL_SYNC1) {
            ps->buf[1] = b;
            ps->idx = 2u;
        } else if (b == HIL_SYNC0) {
            ps->buf[0] = b;                   /* stay at 1: 55 55 AA ok */
        } else {
            ps->idx = 0u;
        }
        return 0;

    case 2:                                   /* TYPE */
        ps->buf[2] = b;
        ps->idx = 3u;
        return 0;

    case 3:                                   /* LEN */
    {
        const uint8_t expect = (uint8_t)HIL_SETPOINT_PAYLOAD;
        ps->buf[3] = b;
        if (b != expect) {                    /* unknown/short frame */
            ps->idx = 0u;
            ps->frame_err++;                  /* framing fault, not CRC */
            return 0;
        }
        ps->len = (uint16_t)(4u + (uint16_t)b + 2u);
        ps->idx = 4u;
        return 0;
    }

    default:                                  /* payload + CRC */
    {
        ps->buf[ps->idx++] = b;
        if (ps->idx < ps->len) {
            return 0;
        }
        ps->idx = 0u;

        {
            const uint16_t rx  = get_u16(&ps->buf[ps->len - 2u]);
            const uint16_t cal = hil_crc16(ps->buf, (size_t)(ps->len - 2u));
            if (rx != cal) {
                ps->crc_err++;
                return 0;
            }
            ps->ok_cnt++;
            return hil_unpack_setpoint(ps->buf, out);
        }
    }
    }
}
