/**
 * @file    hil_protocol.h
 * @brief   HIL (Hardware-in-the-Loop) serial protocol between host PC (Simulink)
 *          and STM32F407 target running the adaptive backstepping controller.
 *
 * Frame layout (little-endian, fixed size per type):
 *
 *   offset  size  field
 *   ------  ----  ----------------------------------------------------
 *      0      1   SYNC0 = 0x55
 *      1      1   SYNC1 = 0xAA
 *      2      1   TYPE  (see HIL_TYPE_xxx)
 *      3      1   LEN   payload length in bytes (excludes header and CRC)
 *      4    LEN    PAYLOAD
 *    4+LEN   2   CRC16-Modbus over [SYNC0 .. last payload byte]
 *
 * Two frame types are exchanged at the control rate (default 100 Hz):
 *
 *   SETPOINT  PC  -> MCU : eta, nu, eta_d, nu_d          LEN =  98  (total 104 B)
 *   COMMAND   MCU -> PC  : u[6], seq, status             LEN =  28  (total  34 B)
 *
 * Bandwidth: 138 B/frame @ 100 Hz = 13.8 kB/s -> use >= 460800 baud
 *            (921600 baud recommended, ~15% utilisation).
 *
 * NOTE: the ROV uses SIX thrusters.  Matrix T (thruster configuration) and
 *       pinv(T) in the Simulink model are both 6x6, so the command vector is
 *       u[6].  HIL_N_U MUST stay 6 in every file that includes this header.
 */

#ifndef HIL_PROTOCOL_H
#define HIL_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Protocol constants                                                  */
/* ------------------------------------------------------------------ */

#define HIL_SYNC0               (0x55u)
#define HIL_SYNC1               (0xAAu)

#define HIL_TYPE_SETPOINT       (0x01u)   /* PC  -> MCU */
#define HIL_TYPE_COMMAND        (0x02u)   /* MCU -> PC  */
#define HIL_TYPE_HEARTBEAT      (0x03u)   /* either way */

#define HIL_N_DOF               (6)       /* x y z phi theta psi        */
#define HIL_N_ETA               (6)
#define HIL_N_NU                (6)
#define HIL_N_U                 (6)       /* thruster count  <-- SIX    */

/* Thrust voltage saturation, mirrors 'Saturation (+/-100V)' in Simulink */
#define HIL_U_MIN               (-100.0f)
#define HIL_U_MAX               (100.0f)

/* Payload sizes ---------------------------------------------------- */
/* 4 arrays of 6 float32 = 96 B, plus 2 B sequence counter           */
#define HIL_SETPOINT_PAYLOAD    (4u * HIL_N_ETA * 4u + 2u)   /* =  98 */
/* 6 float32 voltages + 2 B seq + 2 B status                         */
#define HIL_COMMAND_PAYLOAD     (HIL_N_U * 4u + 2u + 2u)     /* =  28 */

#define HIL_SETPOINT_FRAME      (4u + HIL_SETPOINT_PAYLOAD + 2u)  /* = 104 */
#define HIL_COMMAND_FRAME       (4u + HIL_COMMAND_PAYLOAD  + 2u)  /* =  34 */

/* Absolute byte offsets of the COMMAND payload fields ---------------- */
/* Use these instead of magic indices such as buf[28] / buf[30]; the    */
/* layout must stay in step with hil_command_t above.                   */
#define HIL_HDR_LEN             (4u)                              /* SYNC0 SYNC1 TYPE LEN */
#define HIL_CMD_OFF_U           (HIL_HDR_LEN)                     /* =  4  float  u[6]    */
#define HIL_CMD_OFF_SEQ         (HIL_CMD_OFF_U + HIL_N_U * 4u)    /* = 28  uint16 seq     */
#define HIL_CMD_OFF_STATUS      (HIL_CMD_OFF_SEQ + 2u)            /* = 30  uint16 status  */

/* Status bitfield reported back to the host ------------------------ */
#define HIL_ST_INIT_DONE        (1u << 0)  /* controller initialised    */
#define HIL_ST_DATA_FRESH       (1u << 1)  /* setpoint arrived this tick*/
#define HIL_ST_TIMEOUT          (1u << 2)  /* setpoint older than N ms  */
#define HIL_ST_RX_CRC_ERR       (1u << 3)  /* at least one CRC failure  */
#define HIL_ST_RX_OVERFLOW      (1u << 4)  /* ring buffer overrun        */
#define HIL_ST_SATURATED        (1u << 5)  /* any |u| hit the limit     */
#define HIL_ST_TX_FAIL          (1u << 6)  /* UART TX not accepted      */

/* ------------------------------------------------------------------ */
/* Payload structures (packed, little-endian, float32)                 */
/* ------------------------------------------------------------------ */

/* PC -> MCU : 98 bytes */
typedef struct {
    float    eta[HIL_N_ETA];    /* [x y z phi theta psi]      m, rad  */
    float    nu[HIL_N_NU];      /* [u v w p q r]              m/s,rad/s */
    float    eta_d[HIL_N_ETA];  /* desired pose                       */
    float    nu_d[HIL_N_NU];    /* desired velocity                   */
    uint16_t seq;               /* rolling frame counter              */
} hil_setpoint_t;

/* MCU -> PC : 28 bytes */
typedef struct {
    float    u[HIL_N_U];        /* thruster command voltages  V       */
    uint16_t seq;               /* echoes the seq of the consumed SP  */
    uint16_t status;            /* HIL_ST_xxx bitfield                */
} hil_command_t;

/* ------------------------------------------------------------------ */
/* Incremental receive parser state                                    */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t  buf[HIL_SETPOINT_FRAME];  /* reassembly buffer            */
    uint16_t idx;                      /* 0 = hunting for SYNC0        */
    uint16_t len;                      /* total frame length once known*/
    uint32_t ok_cnt;                   /* valid SETPOINT frames        */
    uint32_t crc_err;                  /* CRC mismatches               */
    uint32_t frame_err;                /* bad LEN / desync events      */
    uint32_t byte_cnt;                 /* raw bytes consumed           */
} hil_parser_t;

/* ------------------------------------------------------------------ */
/* CRC16-Modbus                                                        */
/*   poly = 0x8005 (reflected 0xA001), init = 0xFFFF                   */
/* ------------------------------------------------------------------ */
static inline uint16_t hil_crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte;
    for (uint8_t i = 0u; i < 8u; ++i) {
        if (crc & 0x0001u) {
            crc = (uint16_t)((crc >> 1) ^ 0xA001u);
        } else {
            crc >>= 1;
        }
    }
    return crc;
}

static inline uint16_t hil_crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0u; i < len; ++i) {
        crc = hil_crc16_update(crc, buf[i]);
    }
    return crc;
}

/* ------------------------------------------------------------------ */
/* Public API (implemented in hil_protocol.c)                          */
/* ------------------------------------------------------------------ */

int  hil_pack_command (const hil_command_t  *c,  uint8_t *out);
int  hil_pack_setpoint(const hil_setpoint_t *sp, uint8_t *out);
int  hil_unpack_setpoint(const uint8_t *frame, hil_setpoint_t *sp);

void hil_parser_init(hil_parser_t *ps);
int  hil_parser_feed(hil_parser_t *ps, uint8_t b, hil_setpoint_t *out);

#ifdef __cplusplus
}
#endif

#endif /* HIL_PROTOCOL_H */
