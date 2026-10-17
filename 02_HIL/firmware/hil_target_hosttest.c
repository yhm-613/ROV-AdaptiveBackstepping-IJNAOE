/**
 * @file    hil_target_hosttest.c
 * @brief   Host-side (PC) replay test of the FULL target chain:
 *
 *              CSV setpoints
 *                   |
 *          hil_pack_setpoint()          (PC side, same protocol)
 *                   |  byte by byte
 *          HIL_OnByte() -> hil_parser_feed()
 *                   |
 *          HIL_Step10ms()  ->  rov_ctrl_deploy_step()   (GENERATED CODE)
 *                   |
 *          hil_pack_command() -> hil_port_uart_write()
 *                   |
 *          CSV of u[6] + status
 *
 *   This is a processor-in-the-loop equivalent: if the C u[] sequence matches
 *   the u[] the Simulink model produces for the same setpoint sequence, the
 *   code generation is verified end to end (including the 10 ms discretisation
 *   and the float precision used on the target).
 *
 *   Build:
 *     gcc -std=c99 -O2 -I. -I../rov_ctrl_deploy_ert_rtw \
 *         hil_target_hosttest.c hil_protocol.c hil_target.c \
 *         ../rov_ctrl_deploy_ert_rtw/rov_ctrl_deploy.c \
 *         ../rov_ctrl_deploy_ert_rtw/rov_ctrl_deploy_data.c \
 *         -lm -o hil_target_hosttest
 *
 *   Run:
 *     ./hil_target_hosttest ../out/hosttest_sp.csv ../out/hosttest_u_c.csv \
 *                           ../out/hosttest_sp_c_dump.csv
 *
 *   The optional 3rd argument dumps the float32-quantised setpoints that were
 *   actually handed to the generated controller, so the host can diff them
 *   against its own sequence and prove both sides see identical inputs.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "hil_protocol.h"
#include "hil_target.h"
#include "hil_port.h"

/* ================================================================== */
/* Host implementation of the hardware abstraction layer               */
/* ================================================================== */

static uint32_t g_ms          = 0u;
static uint32_t g_tx_frames   = 0u;
static uint32_t g_tx_crc_bad  = 0u;
static float    g_u_applied[HIL_N_U];
static uint16_t g_last_status = 0u;
static uint16_t g_last_seq    = 0u;

void hil_port_init(void) { }

int hil_port_uart_write(const uint8_t *buf, uint16_t len)
{
    uint16_t rx, cal;

    if (len != (uint16_t)HIL_COMMAND_FRAME) {
        fprintf(stderr, "[hosttest] unexpected frame length %u\n", (unsigned)len);
        return -1;
    }
    rx  = (uint16_t)(buf[len - 2u] | ((uint16_t)buf[len - 1u] << 8));
    cal = hil_crc16(buf, (size_t)(len - 2u));
    if (rx != cal) { g_tx_crc_bad++; }

    g_last_seq    = (uint16_t)(buf[HIL_CMD_OFF_SEQ]    |
                               ((uint16_t)buf[HIL_CMD_OFF_SEQ + 1u] << 8));
    g_last_status = (uint16_t)(buf[HIL_CMD_OFF_STATUS] |
                               ((uint16_t)buf[HIL_CMD_OFF_STATUS + 1u] << 8));
    g_tx_frames++;
    return (int)len;
}

void hil_port_apply_u(const volatile float *u, int n)
{
    int i;
    for (i = 0; i < n && i < HIL_N_U; ++i) { g_u_applied[i] = (float)u[i]; }
}

uint32_t hil_port_millis(void) { return g_ms; }

/* ================================================================== */

int main(int argc, char **argv)
{
    const char *in_path  = (argc > 1) ? argv[1] : "../out/hosttest_sp.csv";
    const char *out_path = (argc > 2) ? argv[2] : "../out/hosttest_u_c.csv";
    /* optional 3rd arg: dump the float32-quantised setpoints actually fed to the
     * generated controller.  Used to diff against the host side and prove that
     * both sides see bit-identical inputs. */
    const char *dmp_path = (argc > 3) ? argv[3] : NULL;

    FILE *fin  = fopen(in_path, "r");
    FILE *fout = fopen(out_path, "w");
    FILE *fdmp = (dmp_path != NULL) ? fopen(dmp_path, "w") : NULL;
    if (fin == NULL)  { fprintf(stderr, "cannot open %s\n", in_path);  return 2; }
    if (fout == NULL) { fprintf(stderr, "cannot open %s\n", out_path); return 2; }
    if (dmp_path != NULL && fdmp == NULL) {
        fprintf(stderr, "cannot open %s\n", dmp_path); return 2;
    }

    HIL_Init();

    fprintf(fout, "tick,t,u0,u1,u2,u3,u4,u5,status,seq\n");

    char  line[4096];
    long  tick = 0;
    int   bad  = 0;

    /* NOTE: do NOT pre-skip a header line.  The CSV written by MATLAB's
     * writematrix() has NO header, while a hand-written CSV may have one.
     * The parser below rejects any line that does not yield >= 19 numeric
     * fields, so a header is skipped automatically -- and no valid data
     * row is ever thrown away. */
    while (fgets(line, sizeof(line), fin) != NULL) {
        double v[32];
        int    n = 0;
        char  *p = line;
        char  *end;

        /* parse comma separated doubles */
        while (n < 32) {
            v[n] = strtod(p, &end);
            if (end == p) { break; }
            n++;
            if (*end != ',') { break; }
            p = end + 1;
        }
        if (n < 25) { continue; }              /* t + eta(6) + nu(6) + eta_d(6) + nu_d(6) */

        {
            hil_setpoint_t sp;
            uint8_t        frame[HIL_SETPOINT_FRAME];
            int            i, k;

            for (i = 0; i < HIL_N_ETA; ++i) {
                sp.eta[i]   = (float)v[1 + i];
                sp.eta_d[i] = (float)v[13 + i];
            }
            for (i = 0; i < HIL_N_NU; ++i) {
                sp.nu[i]   = (float)v[7 + i];
                sp.nu_d[i] = (float)v[19 + i];   /* reference velocity, analytic */
            }
            sp.seq = (uint16_t)(tick & 0xFFFF);

            if (fdmp != NULL) {
                int m;
                fprintf(fdmp, "%.17g", 0.01 * (double)tick);
                for (m = 0; m < 6; ++m) fprintf(fdmp, ",%.17g", (double)sp.eta[m]);
                for (m = 0; m < 6; ++m) fprintf(fdmp, ",%.17g", (double)sp.nu[m]);
                for (m = 0; m < 6; ++m) fprintf(fdmp, ",%.17g", (double)sp.eta_d[m]);
                for (m = 0; m < 6; ++m) fprintf(fdmp, ",%.17g", (double)sp.nu_d[m]);
                fprintf(fdmp, "\n");
            }

            hil_pack_setpoint(&sp, frame);

            /* feed every byte through the real target-side parser */
            for (k = 0; k < (int)HIL_SETPOINT_FRAME; ++k) {
                HIL_OnByte(frame[k]);
            }

            g_ms = (uint32_t)(10 * tick);
            HIL_Step10ms();

            if (g_last_seq != sp.seq) { bad++; }

            fprintf(fout, "%ld,%.6f", tick, 0.01 * (double)tick);
            for (i = 0; i < HIL_N_U; ++i) {
                fprintf(fout, ",%.9g", (double)g_u_applied[i]);
            }
            fprintf(fout, ",%u,%u\n",
                    (unsigned)g_last_status, (unsigned)g_last_seq);
            tick++;
        }
    }

    fclose(fin);
    fclose(fout);
    if (fdmp != NULL) { fclose(fdmp); }

    printf("==================================================\n");
    printf(" target chain replay\n");
    printf("--------------------------------------------------\n");
    printf("  ticks processed      : %ld\n", tick);
    printf("  command frames TX    : %u\n", g_tx_frames);
    printf("  TX CRC failures      : %u\n", g_tx_crc_bad);
    printf("  sequence mismatches  : %d\n", bad);
    printf("  RX ok / crc / frame  : %u / %u / %u\n",
           (unsigned)g_hil_rx_ok, (unsigned)g_hil_rx_crc, (unsigned)g_hil_rx_frame);
    printf("  setpoint drops       : %u\n", (unsigned)g_hil_rx_drop);
    printf("  last u               : ");
    { int i; for (i = 0; i < HIL_N_U; ++i) printf("%8.3f ", (double)g_u_applied[i]); }
    printf("\n  output written to    : %s\n", out_path);
    printf("==================================================\n");

    return (bad == 0 && g_tx_crc_bad == 0) ? 0 : 1;
}
