/**
 ******************************************************************************
 * @file    test_sdo_explicit_length.c
 * @author  HyundoKim
 * @brief   P2 host-side unit tests for the SDO explicit-length single format.
 * @date    2026-06-09
 *
 * @details
 * AGR_MW DOP Core 의 SDO 단일 명시-길이 포맷(0x21 write / 0x41 read-resp,
 * byte4-5 = 2B uint16 LE 길이) 회귀 테스트. 의존성 없는 순수 호스트 빌드:
 *   gcc -std=c11 -I DOP -I DOP/Core \
 *       test/test_sdo_explicit_length.c \
 *       DOP/Core/agr_sdo_protocol.c DOP/Core/agr_od.c -o test/test_sdo
 *   ./test/test_sdo      # exit 0 = 전체 통과
 *
 * 핵심 회귀: CAN-FD DLC 패딩으로 in_len 이 부풀려도 data_len 은 byte4-5 의
 * 명시 값에서 확정된다(프레임 길이 역산 금지). plan_sdo_explicit_length.md §8.
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "agr_sdo_protocol.h"
#include "agr_od.h"
#include "agr_dop_types.h"

/*------------------------------------------------------------
 * Minimal assert harness (no external framework)
 *------------------------------------------------------------*/
static int g_total = 0;
static int g_fail  = 0;

#define CHECK(cond, msg) do {                                   \
    g_total++;                                                  \
    if (!(cond)) {                                              \
        g_fail++;                                               \
        printf("    [FAIL] %-58s (line %d)\n", (msg), __LINE__);\
    }                                                           \
} while (0)

static void banner(const char* name) { printf("--- %s\n", name); }

/*============================================================
 * T1: Encode 0x21 write (6B) → 12 bytes, byte4-5 = 06 00
 *============================================================*/
static void t1_encode_write_layout(void)
{
    banner("T1 encode 0x21 write 6B layout");

    AGR_SDO_Msg_t m;
    memset(&m, 0, sizeof(m));
    m.cs       = AGR_SDO_CS_DOWNLOAD_INIT_REQ_SIZED;  /* 0x21 */
    m.index    = 0x6300;
    m.subindex = 0x00;
    m.data_len = 6;
    for (int i = 0; i < 6; i++) m.data[i] = (uint8_t)(0x10 + i);

    uint8_t buf[64];
    int n = AGR_SDO_Encode(&m, buf, sizeof(buf));

    CHECK(n == 12,                       "encode returns 6+6 = 12");
    CHECK(buf[0] == 0x21,                "cs byte = 0x21");
    CHECK(buf[1] == 0x00 && buf[2] == 0x63, "index 0x6300 LE");
    CHECK(buf[3] == 0x00,                "subindex 0x00");
    CHECK(buf[4] == 0x06 && buf[5] == 0x00, "size field = 06 00 (u16 LE)");
    CHECK(buf[6] == 0x10 && buf[11] == 0x15, "data payload @ offset 6");
}

/*============================================================
 * T2: Decode DLC-padded frame — data_len from size field, NOT in_len
 *     (the core regression: 6B payload in a 16B padded CAN-FD frame)
 *============================================================*/
static void t2_decode_dlc_padding_immune(void)
{
    banner("T2 decode immune to DLC padding");

    uint8_t frame[16];
    memset(frame, 0xAA, sizeof(frame));   /* trailing DLC padding = garbage */
    frame[0] = 0x21;
    frame[1] = 0x00; frame[2] = 0x63;     /* index 0x6300 */
    frame[3] = 0x00;
    frame[4] = 0x06; frame[5] = 0x00;     /* size = 6 */
    for (int i = 0; i < 6; i++) frame[6 + i] = (uint8_t)(0x10 + i);

    AGR_SDO_Msg_t d;
    int rc = AGR_SDO_Decode(frame, 16, &d);   /* in_len = 16 (padded), real = 6 */

    CHECK(rc == 0,            "decode ok");
    CHECK(d.cs == 0x21,       "cs preserved");
    CHECK(d.data_len == 6,    "data_len = 6 despite in_len = 16 (DLC-independent)");
    CHECK(d.data[0] == 0x10 && d.data[5] == 0x15, "payload intact");
}

/*============================================================
 * T3: Roundtrip CreateWriteReq → Encode → Decode for 1/4/5/6/37/58 B
 *============================================================*/
static void t3_roundtrip_sizes(void)
{
    banner("T3 roundtrip sizes 1/4/5/6/37/58");

    const uint16_t sizes[] = {1, 4, 5, 6, 37, 58};
    for (size_t k = 0; k < sizeof(sizes) / sizeof(sizes[0]); k++) {
        uint16_t s = sizes[k];
        uint8_t src[64];
        for (int i = 0; i < 64; i++) src[i] = (uint8_t)(i * 3 + 1);

        AGR_SDO_Msg_t w;
        AGR_SDO_CreateWriteReq(&w, 0x6300, 0x00, src, s);
        CHECK(w.cs == 0x21,        "CreateWriteReq always 0x21");
        CHECK(w.data_len == s,     "data_len matches requested size");

        uint8_t b[80];
        int n = AGR_SDO_Encode(&w, b, sizeof(b));
        CHECK(n == (int)(6 + s),   "encoded length = 6 + size");

        AGR_SDO_Msg_t r;
        int rc = AGR_SDO_Decode(b, (uint16_t)n, &r);
        CHECK(rc == 0,             "decode ok");
        CHECK(r.data_len == s,     "decoded data_len matches");
        CHECK(memcmp(r.data, src, s) == 0, "payload roundtrip identical");
    }
}

/*============================================================
 * T4: 0x41 read-response roundtrip (37B PDO-mapping-sized blob)
 *============================================================*/
static void t4_read_resp_roundtrip(void)
{
    banner("T4 read-resp 0x41 37B roundtrip");

    AGR_SDO_Msg_t up;
    memset(&up, 0, sizeof(up));
    up.cs       = AGR_SDO_CS_UPLOAD_INIT_RSP_SIZED;  /* 0x41 */
    up.index    = 0x1870;
    up.subindex = 0x00;
    up.data_len = 37;
    for (int i = 0; i < 37; i++) up.data[i] = (uint8_t)(0x80 + i);

    uint8_t b[64];
    int n = AGR_SDO_Encode(&up, b, sizeof(b));
    CHECK(n == 43,                  "encoded = 6 + 37 = 43");
    CHECK(b[0] == 0x41,             "cs = 0x41");
    CHECK(b[4] == 37 && b[5] == 0,  "size field = 37");

    AGR_SDO_Msg_t r;
    int rc = AGR_SDO_Decode(b, (uint16_t)n, &r);
    CHECK(rc == 0,                  "decode ok");
    CHECK(r.cs == 0x41,             "cs preserved");
    CHECK(r.data_len == 37,         "data_len = 37");
    CHECK(r.data[0] == 0x80 && r.data[36] == (uint8_t)(0x80 + 36), "payload intact");
}

/*============================================================
 * T5: oversize declared length → -3, no data committed; boundary 60 ok
 *============================================================*/
static void t5_oversize_rejected(void)
{
    banner("T5 oversize -> -3, boundary 60 ok");

    /* declared 61 > MAX(60) */
    uint8_t f[70];
    memset(f, 0, sizeof(f));
    f[0] = 0x21; f[2] = 0x63;
    f[4] = 61; f[5] = 0;
    AGR_SDO_Msg_t d;
    int rc = AGR_SDO_Decode(f, 70, &d);
    CHECK(rc == -3,         "declared 61 (> MAX 60) rejected with -3");
    CHECK(d.data_len == 0,  "no data committed on oversize");

    /* boundary: exactly MAX(60) accepted */
    uint8_t f2[70];
    memset(f2, 0x5A, sizeof(f2));
    f2[0] = 0x21; f2[1] = 0x00; f2[2] = 0x63; f2[3] = 0x00;
    f2[4] = 60; f2[5] = 0;
    AGR_SDO_Msg_t d2;
    int rc2 = AGR_SDO_Decode(f2, 66, &d2);
    CHECK(rc2 == 0,          "declared 60 (== MAX) accepted");
    CHECK(d2.data_len == 60, "data_len = 60 at boundary");
}

/*============================================================
 * T6: legacy formats (expedited / frame-len) → -4 INVALID_CS
 *============================================================*/
static void t6_legacy_rejected(void)
{
    banner("T6 legacy expedited/frame-len -> -4");

    /* 0x20 frame-len download; 0x23/0x27/0x2B/0x2F expedited download;
     * 0x43/0x47/0x4B/0x4F expedited upload response. All unsupported. */
    const uint8_t legacy[] = {0x20, 0x23, 0x27, 0x2B, 0x2F,
                              0x43, 0x47, 0x4B, 0x4F};
    for (size_t k = 0; k < sizeof(legacy); k++) {
        uint8_t f[8];
        memset(f, 0, sizeof(f));
        f[0] = legacy[k];
        f[2] = 0x63;        /* index high byte 0x6300 */
        AGR_SDO_Msg_t d;
        int rc = AGR_SDO_Decode(f, 8, &d);
        char msg[64];
        snprintf(msg, sizeof(msg), "cs 0x%02X rejected with -4 (INVALID_CS)", legacy[k]);
        CHECK(rc == -4, msg);
    }
}

/*============================================================
 * T7: CAN-FD single-frame data ceiling (58B) — Encode buf overflow guard
 *============================================================*/
static void t7_canfd_overflow_guard(void)
{
    banner("T7 CAN-FD 58B ceiling / overflow guard");

    AGR_SDO_Msg_t m;
    memset(&m, 0, sizeof(m));
    m.cs    = 0x21;
    m.index = 0x6300;

    uint8_t cbuf[64];

    m.data_len = 58;                                   /* 6 + 58 = 64 == buf */
    int n58 = AGR_SDO_Encode(&m, cbuf, 64);
    CHECK(n58 == 64, "58B data fits CAN-FD frame exactly (64)");

    m.data_len = 59;                                   /* 6 + 59 = 65 > 64 */
    int n59 = AGR_SDO_Encode(&m, cbuf, 64);
    CHECK(n59 == -2, "59B data overflows 64B CAN-FD buffer -> -2");
}

/*============================================================
 * T8: OD integration — FES ES-vector (0x6300, 6B incl burst_ms)
 *     the original defect: 6B write must land all 6 bytes, no DLC corruption
 *============================================================*/
static uint8_t s_es_vec[6];          /* OD-bound storage (mode,ch,amp_lo,amp_hi,burst_lo,burst_hi) */
static int     s_on_write_called;

static void es_on_write(void) { s_on_write_called++; }

static const AGR_OD_Entry_t s_od_entries[] = {
    /* idx,    sub,  type,          size, access,        data_ptr,  on_write,    name,        unit */
    { 0x6300, 0x00, AGR_TYPE_BLOB,  6,    AGR_ACCESS_RW, s_es_vec,  es_on_write, "es_vector", NULL },
};
static const AGR_OD_Table_t s_od = { s_od_entries, 1 };

static void t8_od_esvector_write(void)
{
    banner("T8 OD ES-vector write (0x6300, burst_ms)");

    s_on_write_called = 0;
    memset(s_es_vec, 0, sizeof(s_es_vec));

    /* 6B ES-vector in a 20B padded frame (simulate CAN-FD DLC20 quantization). */
    uint8_t frame[20];
    memset(frame, 0xEE, sizeof(frame));      /* [12..19] = DLC padding garbage */
    frame[0] = 0x21;
    frame[1] = 0x00; frame[2] = 0x63;        /* 0x6300 */
    frame[3] = 0x00;
    frame[4] = 0x06; frame[5] = 0x00;        /* size = 6 */
    frame[6] = 0x02;                         /* mode  = 2 */
    frame[7] = 0x01;                         /* ch    = 1 */
    frame[8] = 0x2C; frame[9] = 0x01;        /* amplitude = 0x012C = 300 */
    frame[10] = 0xF4; frame[11] = 0x01;      /* burst_ms  = 0x01F4 = 500 */

    AGR_SDO_Msg_t req, rsp;
    int rc = AGR_SDO_Decode(frame, 20, &req);
    CHECK(rc == 0 && req.data_len == 6, "ES-vector decode: data_len = 6 despite 20B frame");

    memset(&rsp, 0, sizeof(rsp));
    int pr = AGR_SDO_ProcessRequest(&s_od, &req, &rsp);
    CHECK(pr == 0,                                  "ES-vector write processed");
    CHECK(rsp.cs == AGR_SDO_CS_DOWNLOAD_INIT_RSP,   "write response cs = 0x60");

    CHECK(s_es_vec[0] == 0x02 && s_es_vec[1] == 0x01, "mode/channel landed");
    CHECK(s_es_vec[4] == 0xF4 && s_es_vec[5] == 0x01, "burst_ms bytes landed @ offset 4-5");
    uint16_t burst = (uint16_t)s_es_vec[4] | ((uint16_t)s_es_vec[5] << 8);
    CHECK(burst == 500,                             "burst_ms == 500 (not lost)");
    CHECK(s_on_write_called == 1,                   "on_write callback fired once");

    /* Regression guard: the OLD frame-length decode would have computed
     * data_len = in_len - 4 = 16, which OD WriteValue rejects (16 > size 6). */
    int old_fail = AGR_OD_WriteValue(&s_od_entries[0], &frame[4], 16);
    CHECK(old_fail == -4, "OLD frame-len decode (len 16) would be rejected -4");
}

/*============================================================
 * T9: Read path — 0x40 request decode + ProcessRequest → 0x41 response
 *============================================================*/
static void t9_read_path(void)
{
    banner("T9 read 0x40 -> 0x41 response");

    /* prerequisite: s_es_vec populated by T8 (burst_ms = 500) */

    uint8_t rdf[4] = {0x40, 0x00, 0x63, 0x00};
    AGR_SDO_Msg_t rd;
    int rdc = AGR_SDO_Decode(rdf, 4, &rd);
    CHECK(rdc == 0,            "read-req 0x40 decode ok");
    CHECK(rd.cs == 0x40,       "cs = 0x40");
    CHECK(rd.data_len == 0,    "read-req carries no data");

    AGR_SDO_Msg_t rrsp;
    memset(&rrsp, 0, sizeof(rrsp));
    int rp = AGR_SDO_ProcessRequest(&s_od, &rd, &rrsp);
    CHECK(rp == 0,                                     "read processed");
    CHECK(rrsp.cs == AGR_SDO_CS_UPLOAD_INIT_RSP_SIZED, "read response cs = 0x41");
    CHECK(rrsp.data_len == 6,                          "read response data_len = 6");
    CHECK(rrsp.data[4] == 0xF4 && rrsp.data[5] == 0x01, "read returns burst_ms");

    uint8_t rb[32];
    int rn = AGR_SDO_Encode(&rrsp, rb, sizeof(rb));
    CHECK(rn == 12,                       "read response encodes to 12B");
    CHECK(rb[0] == 0x41,                  "encoded cs = 0x41");
    CHECK(rb[4] == 6 && rb[5] == 0,       "encoded size field = 6");
}

/*============================================================
 * T10: short-frame / NULL guards
 *============================================================*/
static void t10_guards(void)
{
    banner("T10 short-frame & NULL guards");

    AGR_SDO_Msg_t d;
    uint8_t sf[3] = {0x21, 0x00, 0x63};
    CHECK(AGR_SDO_Decode(sf, 3, &d) == -2, "in_len < 4 (header) -> -2");

    uint8_t sf2[5] = {0x21, 0x00, 0x63, 0x00, 0x06};
    CHECK(AGR_SDO_Decode(sf2, 5, &d) == -2, "0x21 with in_len < 6 (size field) -> -2");

    CHECK(AGR_SDO_Decode(NULL, 8, &d) == -1, "Decode NULL in_buf -> -1");
    CHECK(AGR_SDO_Decode(sf, 8, NULL) == -1, "Decode NULL out_msg -> -1");

    AGR_SDO_Msg_t m;
    memset(&m, 0, sizeof(m));
    m.cs = 0x21; m.data_len = 4;
    uint8_t ob[16];
    CHECK(AGR_SDO_Encode(NULL, ob, sizeof(ob)) == -1, "Encode NULL msg -> -1");
    CHECK(AGR_SDO_Encode(&m, NULL, 16) == -1,         "Encode NULL out_buf -> -1");
}

/*============================================================
 * MAIN
 *============================================================*/
int main(void)
{
    printf("==== SDO explicit-length unit tests (P2) ====\n");

    t1_encode_write_layout();
    t2_decode_dlc_padding_immune();
    t3_roundtrip_sizes();
    t4_read_resp_roundtrip();
    t5_oversize_rejected();
    t6_legacy_rejected();
    t7_canfd_overflow_guard();
    t8_od_esvector_write();
    t9_read_path();
    t10_guards();

    printf("=============================================\n");
    printf("  %d checks, %d failed\n", g_total, g_fail);
    printf("  %s\n", g_fail == 0 ? "ALL PASS" : "FAILURES PRESENT");

    return g_fail == 0 ? 0 : 1;
}
