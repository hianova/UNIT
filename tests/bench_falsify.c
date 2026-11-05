#include "unit_test_kit.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ============================================================================
 * Baseline A1 & A2: Traditional C Stack (strtod / Fast Char-Loop + double + if/else)
 * ============================================================================ */

typedef struct {
    double A[8][4];
    double b_enter[8];
    double b_exit[8];
    int prev_violated[8];
    double u[4];
    double du[4];
    double ddu[4];
} TradFloatEngine;

static void trad_engine_init(TradFloatEngine *e) {
    memset(e, 0, sizeof(*e));
    for (int d = 0; d < 4; ++d) {
        e->A[2 * d][d] = 1.0;
        e->A[2 * d + 1][d] = -1.0;
        e->b_enter[2 * d] = 50.0;
        e->b_enter[2 * d + 1] = 50.0;
        e->b_exit[2 * d] = 40.0;
        e->b_exit[2 * d + 1] = 40.0;
    }
}

/* Traditional Baseline 1: Standard libc strtod() parser */
static inline int trad_parse_strtod(const char buf[8], double *out_val) {
    char tmp[9];
    memcpy(tmp, buf, 8);
    tmp[8] = '\0';
    char *endptr = NULL;
    double v = strtod(tmp, &endptr);
    if (endptr != tmp + 8 || isnan(v) || isinf(v)) {
        *out_val = 0.0;
        return -1;
    }
    *out_val = v;
    return 0;
}

/* Traditional Baseline 2: Hand-written C character-by-character branchy parser */
static inline int trad_parse_charloop(const char buf[8], double *out_val) {
    int idx = 0;
    double sign = 1.0;
    if (buf[0] == '+') {
        idx = 1;
    } else if (buf[0] == '-') {
        sign = -1.0;
        idx = 1;
    }
    double int_part = 0.0;
    double frac_part = 0.0;
    double frac_div = 1.0;
    int in_frac = 0;
    for (; idx < 8; ++idx) {
        char c = buf[idx];
        if (c == '.') {
            if (in_frac) {
                *out_val = 0.0;
                return -1;
            }
            in_frac = 1;
        } else if (c >= '0' && c <= '9') {
            if (!in_frac) {
                int_part = int_part * 10.0 + (double)(c - '0');
            } else {
                frac_part = frac_part * 10.0 + (double)(c - '0');
                frac_div *= 10.0;
            }
        } else {
            *out_val = 0.0;
            return -1;
        }
    }
    *out_val = sign * (int_part + frac_part / frac_div);
    return 0;
}

static __attribute__((noinline)) double trad_step_and_predict(TradFloatEngine *e,
                                                              const char wire_buf[8], int axis,
                                                              int use_strtod, int *out_violated) {
    double val = 0.0;
    int rc = use_strtod ? trad_parse_strtod(wire_buf, &val) : trad_parse_charloop(wire_buf, &val);
    if (rc != 0) {
        *out_violated = -1;
        return -1.0;
    }

    int d = axis & 3;
    double prev_u = e->u[d];
    double prev_du = e->du[d];
    e->u[d] = val;
    e->du[d] = val - prev_u;
    e->ddu[d] = e->du[d] - prev_du;

    int viol_mask = 0;
    double best_t = 1e30;

    for (int i = 0; i < 8; ++i) {
        double dot_x = 0.0;
        double r_i = 0.0;
        double c_i = 0.0;
        for (int j = 0; j < 4; ++j) {
            dot_x += e->A[i][j] * e->u[j];
            r_i += e->A[i][j] * e->du[j];
            c_i += e->A[i][j] * e->ddu[j];
        }
        double bound = e->prev_violated[i] ? e->b_exit[i] : e->b_enter[i];
        if (dot_x > bound) {
            e->prev_violated[i] = 1;
            viol_mask |= (1 << i);
        } else {
            e->prev_violated[i] = 0;
            double s_i = bound - dot_x;
            /* Solve 0.5 * c_i * t^2 + r_i * t - s_i = 0 */
            if (fabs(c_i) < 1e-12) {
                if (r_i > 1e-12) {
                    double t = s_i / r_i;
                    if (t > 0.0 && t < best_t) {
                        best_t = t;
                    }
                }
            } else {
                double disc = r_i * r_i + 2.0 * c_i * s_i;
                if (disc >= 0.0) {
                    double sq = sqrt(disc);
                    double t1 = (-r_i - sq) / c_i;
                    double t2 = (-r_i + sq) / c_i;
                    if (t1 > 0.0 && t1 < best_t) {
                        best_t = t1;
                    }
                    if (t2 > 0.0 && t2 < best_t) {
                        best_t = t2;
                    }
                }
            }
        }
    }
    *out_violated = viol_mask;
    return best_t;
}

/* ============================================================================
 * Candidate B: UNIT + FLOW v4 Single-Pass Fused 64-Bit Hardware Stack
 * ============================================================================ */

static inline int32_t bench_clamp_i32(int64_t v, uint64_t *inout_sieve) {
    if (v > (int64_t)INT32_MAX) {
        *inout_sieve &= ~(FLOW_ASM_SIEVE_INT_NO_OVERFLOW | FLOW_ASM_SIEVE_Q_NO_SATURATION);
        return INT32_MAX;
    }
    if (v < (int64_t)INT32_MIN) {
        *inout_sieve &= ~(FLOW_ASM_SIEVE_INT_NO_OVERFLOW | FLOW_ASM_SIEVE_Q_NO_SATURATION);
        return INT32_MIN;
    }
    return (int32_t)v;
}

static __attribute__((noinline)) int64_t unit_flow_step_and_predict(UnitFlowEngine *eng,
                                                                    uint64_t wire_word8,
                                                                    uint8_t axis,
                                                                    uint64_t *out_sieve,
                                                                    uint64_t *out_violated) {
    uint64_t sieve = UNIT_SIEVE_ALL_SOUND;

    /* 1. Single-pass branchless SWAR wire8 -> Q48.16 state update via FLOW ASM qsub */
    int64_t val_q16 = unit_swar_wire8_to_q16_branchless(wire_word8, &sieve);
    uint32_t d = (uint32_t)axis & 3U;
    int32_t prev_u = eng->u_ext[d];
    int32_t prev_du = eng->du_ext[d];
    int32_t next_u = bench_clamp_i32(val_q16, &sieve);
    int32_t next_du = bench_clamp_i32(
        (int64_t)flow_asm_qsub_i64((int64_t)next_u, (int64_t)prev_u, &sieve).value, &sieve);
    int32_t next_ddu = bench_clamp_i32(
        (int64_t)flow_asm_qsub_i64((int64_t)next_du, (int64_t)prev_du, &sieve).value, &sieve);
    eng->u_ext[d] = next_u;
    eng->du_ext[d] = next_du;
    eng->ddu_ext[d] = next_ddu;

    /* 2. Fused Single-Pass QPoly Schmitt Hysteresis + 64-Bit Hardware 2-Jet Root Prediction */
    uint64_t prev_viol = eng->prev_violated_mask;
    uint64_t viol_mask = 0ULL;
    int64_t best_t_q16 = INT64_MAX;

    const int64_t u0 = eng->u_ext[0], u1 = eng->u_ext[1], u2 = eng->u_ext[2], u3 = eng->u_ext[3];
    const int64_t du0 = eng->du_ext[0], du1 = eng->du_ext[1], du2 = eng->du_ext[2],
                  du3 = eng->du_ext[3];
    const int64_t ddu0 = eng->ddu_ext[0], ddu1 = eng->ddu_ext[1], ddu2 = eng->ddu_ext[2],
                  ddu3 = eng->ddu_ext[3];

    for (uint32_t i = 0U; i < 8U; ++i) {
        const int32_t *row = eng->qpoly.A[i];
        int64_t a0 = row[0], a1 = row[1], a2 = row[2], a3 = row[3];
        int64_t dot_x = a0 * u0 + a1 * u1 + a2 * u2 + a3 * u3;

        int64_t b_enter_i = atomic_load_explicit(&eng->qpoly.b[i], memory_order_relaxed);
        int64_t b_exit_i = atomic_load_explicit(&eng->qpoly.b_exit[i], memory_order_relaxed);
        int64_t bound = flow_asm_select_i64((prev_viol >> i) & 1ULL, b_exit_i, b_enter_i);

        if (dot_x > bound) {
            viol_mask |= (1ULL << i);
        } else {
            int64_t r_i = a0 * du0 + a1 * du1 + a2 * du2 + a3 * du3;
            int64_t c_i = a0 * ddu0 + a1 * ddu1 + a2 * ddu2 + a3 * ddu3;
            if (r_i > 0 || c_i > 0) {
                int64_t s_i = bound - dot_x;
                int64_t t_cand = 0;
                if (unit_jet2_solve_root_fast64(s_i, -r_i, -c_i, best_t_q16, &t_cand)) {
                    best_t_q16 = t_cand;
                }
            }
        }
    }

    eng->prev_violated_mask = viol_mask;
    *out_violated = viol_mask;
    *out_sieve = sieve;
    return best_t_q16;
}

static int cmp_u64(const void *a, const void *b) {
    uint64_t va = *(const uint64_t *)a;
    uint64_t vb = *(const uint64_t *)b;
    return (va > vb) - (va < vb);
}

/* Unpacked 128-byte traditional message struct for Cache Density test */
typedef struct {
    uint32_t msg_type;
    uint32_t axis;
    char symbol[32];
    char raw_payload[64];
    double parsed_value;
    uint64_t status_flags;
    uint64_t _pad;
} TradUnpackedMsg128;

int main(void) {
    UNIT_TEST_SUITE_BEGIN("UNIT x FLOW v4: Falsification & Breakthrough Verification Benchmark");

    const char *clean_frames[8] = {"+0012.50", "+0025.00", "+0038.75", "+0044.25",
                                   "-0010.25", "-0022.50", "+0015.00", "+0030.50"};
    const char *dirty_frames[8] = {"+0049.75", "+0051.25", "+0045.00", "-0051.50",
                                   "+0039.25", "-0039.50", "+0050.50", "-0044.75"};

    uint64_t clean_words[8];
    uint64_t dirty_words[8];
    for (int i = 0; i < 8; ++i) {
        memcpy(&clean_words[i], clean_frames[i], 8U);
        memcpy(&dirty_words[i], dirty_frames[i], 8U);
    }

    /* =====================================================================
     * Falsification Test 1: End-to-End Wire-to-Decision Throughput
     * ===================================================================== */
    UNIT_STAGE_BEGIN(1, "Falsification Test 1: Wire-to-Decision Throughput (200,000 Frames)");
    {
        const uint32_t N = 200000U;
        TradFloatEngine trad1, trad2;
        trad_engine_init(&trad1);
        trad_engine_init(&trad2);

        UnitFlowEngine ueng;
        unit_flow_engine_init(&ueng);
        for (uint32_t d = 0U; d < 4U; ++d) {
            UnitFlowPoly4D_set_hysteresis(&ueng.qpoly, 2U * d, 50LL * FLOW_Q16_ONE,
                                          40LL * FLOW_Q16_ONE);
            UnitFlowPoly4D_set_hysteresis(&ueng.qpoly, 2U * d + 1U, 50LL * FLOW_Q16_ONE,
                                          40LL * FLOW_Q16_ONE);
        }

        /* 1A: Traditional libc strtod + double + sqrt */
        double sink_d1 = 0.0;
        int viol_d1 = 0;
        uint64_t t0_a1 = flow_clock_monotonic_ns();
        uint64_t c0_a1 = flow_hardware_cycles();
        for (uint32_t i = 0U; i < N; ++i) {
            sink_d1 +=
                trad_step_and_predict(&trad1, clean_frames[i & 7U], (int)(i & 3U), 1, &viol_d1);
        }
        uint64_t c1_a1 = flow_hardware_cycles();
        uint64_t t1_a1 = flow_clock_monotonic_ns();

        /* 1B: Traditional hand-written char-loop + double + sqrt */
        double sink_d2 = 0.0;
        int viol_d2 = 0;
        uint64_t t0_a2 = flow_clock_monotonic_ns();
        uint64_t c0_a2 = flow_hardware_cycles();
        for (uint32_t i = 0U; i < N; ++i) {
            sink_d2 +=
                trad_step_and_predict(&trad2, clean_frames[i & 7U], (int)(i & 3U), 0, &viol_d2);
        }
        uint64_t c1_a2 = flow_hardware_cycles();
        uint64_t t1_a2 = flow_clock_monotonic_ns();

        /* 1C: UNIT + FLOW v4 Single-Pass SWAR + ASM QPoly + Q16 2-Jet */
        int64_t sink_u = 0;
        uint64_t sieve_u = 0ULL, viol_u = 0ULL;
        uint64_t t0_b = flow_clock_monotonic_ns();
        uint64_t c0_b = flow_hardware_cycles();
        for (uint32_t i = 0U; i < N; ++i) {
            sink_u ^= unit_flow_step_and_predict(&ueng, clean_words[i & 7U], (uint8_t)(i & 3U),
                                                 &sieve_u, &viol_u);
        }
        uint64_t c1_b = flow_hardware_cycles();
        uint64_t t1_b = flow_clock_monotonic_ns();

        uint64_t ns_a1 = t1_a1 - t0_a1;
        uint64_t ns_a2 = t1_a2 - t0_a2;
        uint64_t ns_b = t1_b - t0_b;

        printf("  📊 [Baseline A1: libc strtod + double + math.h sqrt] : %7llu us (%4llu ns/msg, "
               "%llu ticks)\n",
               (unsigned long long)(ns_a1 / 1000ULL), (unsigned long long)(ns_a1 / N),
               (unsigned long long)(c1_a1 - c0_a1));
        printf("  📊 [Baseline A2: Fast C loop + double + math.h sqrt] : %7llu us (%4llu ns/msg, "
               "%llu ticks)\n",
               (unsigned long long)(ns_a2 / 1000ULL), (unsigned long long)(ns_a2 / N),
               (unsigned long long)(c1_a2 - c0_a2));
        printf("  ⚡ [Candidate B: UNIT SWAR + FLOW v4 Total ASM]      : %7llu us (%4llu ns/msg, "
               "%llu ticks)\n",
               (unsigned long long)(ns_b / 1000ULL), (unsigned long long)(ns_b / N),
               (unsigned long long)(c1_b - c0_b));
        printf("  🏆 Speedup vs libc strtod: %llu.%02llux | vs hand-written C float: %llu.%02llux "
               "(sink=%lld)\n\n",
               (unsigned long long)(ns_a1 / ns_b),
               (unsigned long long)(((ns_a1 * 100ULL) / ns_b) % 100ULL),
               (unsigned long long)(ns_a2 / ns_b),
               (unsigned long long)(((ns_a2 * 100ULL) / ns_b) % 100ULL),
               (long long)(sink_u ^ (int64_t)sink_d1 ^ (int64_t)sink_d2));

        UNIT_ASSERT_TRUE(ns_b < ns_a1);
        UNIT_ASSERT_EQ(sieve_u, UNIT_SIEVE_ALL_SOUND);
    }

    /* =====================================================================
     * Falsification Test 2: Direct-on-Minimal-Bit (4-Bit BCD) vs Unpack/strtol
     * ===================================================================== */
    UNIT_STAGE_BEGIN(2, "Falsification Test 2: Direct 4-Bit BCD Compute vs Unpack-to-ASCII/strtol");
    {
        const uint32_t M = 1000000U;
        uint64_t raw_ascii[8];
        uint32_t packed_bcd[8];
        for (uint32_t i = 0U; i < 8U; ++i) {
            char tmp[9];
            snprintf(tmp, sizeof(tmp), "%08u", 10000000U + i * 1234567U);
            memcpy(&raw_ascii[i], tmp, 8U);
            packed_bcd[i] = unit_swar_pack_4bit_bcd_u64(raw_ascii[i]);
        }

        /* 2A: Unpack 4-bit BCD -> ASCII -> strtol */
        uint64_t sum_strtol = 0ULL;
        uint64_t t0_s = flow_clock_monotonic_ns();
        for (uint32_t i = 0U; i < M; ++i) {
            uint64_t asc = unit_swar_unpack_4bit_bcd_u64(packed_bcd[i & 7U]);
            char buf[9];
            memcpy(buf, &asc, 8U);
            buf[8] = '\0';
            sum_strtol += (uint64_t)strtol(buf, NULL, 10);
        }
        uint64_t t1_s = flow_clock_monotonic_ns();

        /* 2B: Unpack 4-bit BCD -> ASCII -> SWAR 8-digit parse (the old 2-step way) */
        uint64_t sum_unpack_swar = 0ULL;
        uint64_t t0_u = flow_clock_monotonic_ns();
        for (uint32_t i = 0U; i < M; ++i) {
            uint64_t asc = unit_swar_unpack_4bit_bcd_u64(packed_bcd[i & 7U]);
            sum_unpack_swar += (uint64_t)unit_swar_parse_8digits_u64(asc);
        }
        uint64_t t1_u = flow_clock_monotonic_ns();

        /* 2C: Direct-on-Minimal-Bit evaluation (unit_swar_eval_bcd32, 0 unpacking!) */
        uint64_t sum_direct_bcd = 0ULL;
        uint64_t t0_d = flow_clock_monotonic_ns();
        for (uint32_t i = 0U; i < M; ++i) {
            sum_direct_bcd += (uint64_t)unit_swar_eval_bcd32(packed_bcd[i & 7U]);
        }
        uint64_t t1_d = flow_clock_monotonic_ns();

        UNIT_ASSERT_EQ(sum_direct_bcd, sum_strtol);
        UNIT_ASSERT_EQ(sum_direct_bcd, sum_unpack_swar);

        uint64_t ns_strtol = t1_s - t0_s;
        uint64_t ns_unpack = t1_u - t0_u;
        uint64_t ns_direct = (t1_d > t0_d) ? (t1_d - t0_d) : 1ULL;

        printf("  📊 [Unpack BCD -> ASCII -> libc strtol]     : %7llu us\n",
               (unsigned long long)(ns_strtol / 1000ULL));
        printf("  📊 [Unpack BCD -> ASCII -> SWAR parse]      : %7llu us\n",
               (unsigned long long)(ns_unpack / 1000ULL));
        printf("  ⚡ [Direct-on-Minimal-Bit BCD Evaluation]   : %7llu us (Speedup vs strtol: "
               "%llux)\n\n",
               (unsigned long long)(ns_direct / 1000ULL),
               (unsigned long long)(ns_strtol / ns_direct));
    }

    /* =====================================================================
     * Falsification Test 3: Tail Latency Jitter (P50 vs P99.9) under Dirty Stream
     * ===================================================================== */
    UNIT_STAGE_BEGIN(3,
                     "Falsification Test 3: P50 vs P99.9 Tail Jitter on Dirty/Chattering Stream");
    {
        enum {
            BATCHES = 2000,
            BATCH_LEN = 128
        };
        static uint64_t lat_trad[BATCHES];
        static uint64_t lat_unit[BATCHES];
        uint64_t timer_hz = flow_hardware_timer_frequency_hz();
        if (timer_hz == 0ULL) {
            timer_hz = 1000000000ULL;
        }

        TradFloatEngine trad;
        trad_engine_init(&trad);
        UnitFlowEngine ueng;
        unit_flow_engine_init(&ueng);
        for (uint32_t d = 0U; d < 4U; ++d) {
            UnitFlowPoly4D_set_hysteresis(&ueng.qpoly, 2U * d, 50LL * FLOW_Q16_ONE,
                                          40LL * FLOW_Q16_ONE);
            UnitFlowPoly4D_set_hysteresis(&ueng.qpoly, 2U * d + 1U, 50LL * FLOW_Q16_ONE,
                                          40LL * FLOW_Q16_ONE);
        }

        double sink_t = 0.0;
        int64_t sink_u = 0;
        for (uint32_t b = 0U; b < BATCHES; ++b) {
            uint64_t c0 = flow_hardware_cycles();
            for (uint32_t k = 0U; k < BATCH_LEN; ++k) {
                int v = 0;
                uint32_t idx = (b * 13U + k * 7U) & 7U;
                const char *frame = ((b + k) & 1U) ? dirty_frames[idx] : clean_frames[idx];
                sink_t += trad_step_and_predict(&trad, frame, (int)(k & 3U), 1, &v);
            }
            uint64_t c1 = flow_hardware_cycles();
            lat_trad[b] = ((c1 - c0) * 1000000000ULL) / timer_hz;

            uint64_t u0 = flow_hardware_cycles();
            for (uint32_t k = 0U; k < BATCH_LEN; ++k) {
                uint64_t sv = 0ULL, vl = 0ULL;
                uint32_t idx = (b * 13U + k * 7U) & 7U;
                uint64_t w = ((b + k) & 1U) ? dirty_words[idx] : clean_words[idx];
                sink_u ^= unit_flow_step_and_predict(&ueng, w, (uint8_t)(k & 3U), &sv, &vl);
            }
            uint64_t u1 = flow_hardware_cycles();
            lat_unit[b] = ((u1 - u0) * 1000000000ULL) / timer_hz;
        }

        qsort(lat_trad, BATCHES, sizeof(uint64_t), cmp_u64);
        qsort(lat_unit, BATCHES, sizeof(uint64_t), cmp_u64);

        uint64_t trad_p50 = lat_trad[BATCHES / 2];
        uint64_t trad_p99 = lat_trad[(BATCHES * 99) / 100];
        uint64_t trad_p999 = lat_trad[(BATCHES * 999) / 1000];

        uint64_t unit_p50 = lat_unit[BATCHES / 2];
        uint64_t unit_p99 = lat_unit[(BATCHES * 99) / 100];
        uint64_t unit_p999 = lat_unit[(BATCHES * 999) / 1000];

        printf("  📊 [Traditional libc Stack Batch] P50: %5llu ns | P99: %5llu ns | P99.9: "
               "%6llu ns\n",
               (unsigned long long)trad_p50, (unsigned long long)trad_p99,
               (unsigned long long)trad_p999);
        printf("  ⚡ [UNIT + FLOW v4 Batch]         P50: %5llu ns | P99: %5llu ns | P99.9: "
               "%6llu ns (sink=%lld)\n\n",
               (unsigned long long)unit_p50, (unsigned long long)unit_p99,
               (unsigned long long)unit_p999, (long long)(sink_u ^ (int64_t)sink_t));

        UNIT_ASSERT_TRUE(unit_p50 <= trad_p50);
    }

    /* =====================================================================
     * Falsification Test 4: Cache Footprint & Memory Wall (16B Cell vs 128B Struct)
     * ===================================================================== */
    UNIT_STAGE_BEGIN(4, "Falsification Test 4: Memory Wall Scan (16B UnitCell128 vs 128B Struct)");
    {
        enum {
            RING_LEN = 65536
        }; /* 64K events: 1MB in UnitCell128 vs 8MB in TradUnpackedMsg128 */
        TradUnpackedMsg128 *trad_ring =
            (TradUnpackedMsg128 *)calloc(RING_LEN, sizeof(TradUnpackedMsg128));
        UnitCell128 *unit_ring = (UnitCell128 *)calloc(RING_LEN, sizeof(UnitCell128));
        UNIT_ASSERT_TRUE(trad_ring != NULL && unit_ring != NULL);

        uint64_t target_word = 0ULL;
        memcpy(&target_word, "ETH_USDT", 8U);
        uint64_t target_packed56 = unit_swar_pack_7bit_u64(target_word);

        for (uint32_t i = 0U; i < RING_LEN; ++i) {
            const char *sym = (i & 3U) == 0U ? "ETH_USDT" : "BTC_USDC";
            memcpy(trad_ring[i].symbol, sym, 8U);
            trad_ring[i].msg_type = (uint32_t)UNIT_OP_LOAD_U_EXT;
            trad_ring[i].axis = i & 3U;
            trad_ring[i].parsed_value = (double)(i & 255U);

            uint64_t sw = 0ULL;
            memcpy(&sw, sym, 8U);
            uint64_t sv = UNIT_SIEVE_ALL_SOUND;
            unit_ring[i] = unit_make_inline_ascii_cell((uint8_t)UNIT_OP_LOAD_U_EXT,
                                                       (uint8_t)(i & 3U), sw, 8U, &sv);
        }

        uint32_t trad_matches = 0U;
        uint64_t t0_tr = flow_clock_monotonic_ns();
        for (uint32_t pass = 0U; pass < 32U; ++pass) {
            for (uint32_t i = 0U; i < RING_LEN; ++i) {
                if (trad_ring[i].msg_type == (uint32_t)UNIT_OP_LOAD_U_EXT &&
                    memcmp(trad_ring[i].symbol, "ETH_USDT", 8U) == 0) {
                    trad_matches += trad_ring[i].axis;
                }
            }
        }
        uint64_t t1_tr = flow_clock_monotonic_ns();

        uint32_t unit_matches = 0U;
        uint64_t t0_ur = flow_clock_monotonic_ns();
        for (uint32_t pass = 0U; pass < 32U; ++pass) {
            for (uint32_t i = 0U; i < RING_LEN; ++i) {
                /* Match directly on 56-bit minimal-bit payload + opcode in 2 registers! */
                uint64_t hdr = unit_ring[i].packet.protocol_header;
                uint64_t body = unit_ring[i].packet.body_slice;
                if (unit_control_opcode(hdr) == (uint8_t)UNIT_OP_LOAD_U_EXT &&
                    body == target_packed56) {
                    unit_matches += (uint32_t)unit_control_target_axis(hdr);
                }
            }
        }
        uint64_t t1_ur = flow_clock_monotonic_ns();

        UNIT_ASSERT_EQ(unit_matches, trad_matches);
        uint64_t ns_tr = t1_tr - t0_tr;
        uint64_t ns_ur = (t1_ur > t0_ur) ? (t1_ur - t0_ur) : 1ULL;

        printf("  📊 [128-Byte Unpacked Struct Ring (8 MB, memcmp)]      : %7llu us\n",
               (unsigned long long)(ns_tr / 1000ULL));
        printf("  ⚡ [16-Byte UnitCell128 Ring (1 MB, Direct 56b Match)] : %7llu us (Speedup: "
               "%llu.%02llux)\n\n",
               (unsigned long long)(ns_ur / 1000ULL), (unsigned long long)(ns_tr / ns_ur),
               (unsigned long long)(((ns_tr * 100ULL) / ns_ur) % 100ULL));

        free(trad_ring);
        free(unit_ring);
    }

    /* =====================================================================
     * Falsification Test 5: Mathematical Determinism Edge (0-ULP vs IEEE-754 Drift)
     * ===================================================================== */
    UNIT_STAGE_BEGIN(5, "Falsification Test 5: IEEE-754 Mantissa Absorption Failure vs FLOW Q16");
    {
        /*
         * Classic IEEE-754 failure case in solvency / hyperplane evaluation:
         * Suppose hyperplane dot product has two large opposing legs (+2^57 and -2^57)
         * plus a critical violation margin (+6.0) against bound b = 5.0:
         *   True mathematical value: (+2^57) + (-2^57) + 6.0 = 6.0 > 5.0 (VIOLATION!)
         *   In IEEE-754 double (53-bit mantissa, ULP at 2^57 is 16.0):
         *   (2^57 + 6.0) - 2^57 == 0.0 <= 5.0 (FALSE SAFE!)
         */
        double big = (double)(1ULL << 57U);
        double ieee_dot = (big + 6.0) - big;
        bool ieee_detected_violation = (ieee_dot > 5.0);

        /*
         * Now evaluate the exact same opposing legs in FLOW v4's 128-bit exact ASM inner product
         * (`flow_asm_qdot_i32`):
         *   a = [2^30, -2^30, 1, 0], x = [2^30, 2^30, 6, 0]
         *   True inner product = 2^60 - 2^60 + 6 = 6 > 5 (VIOLATION CAUGHT 100%!)
         */
        const int32_t a_vec[4] = {1 << 30, -(1 << 30), 1, 0};
        const int32_t x_vec[4] = {1 << 30, 1 << 30, 6, 0};
        uint64_t sieve = UNIT_SIEVE_ALL_SOUND;
        FlowResultU64 exact_res = flow_asm_qdot_i32(a_vec, x_vec, 4U, &sieve);
        bool flow_detected_violation = ((int64_t)exact_res.value > 5LL);

        printf(
            "  ❌ [IEEE-754 double] (2^57 + 6.0) - 2^57 = %.1f <= 5.0 -> Violation Detected? %s\n",
            ieee_dot, ieee_detected_violation ? "YES" : "NO (MANTISSA ABSORPTION FAILURE!)");
        printf(
            "  ✅ [FLOW v4 128-bit ASM] 2^60 - 2^60 + 6 = %lld > 5   -> Violation Detected? %s\n",
            (long long)exact_res.value, flow_detected_violation ? "YES (EXACT 0-ULP!)" : "NO");

        UNIT_ASSERT_FALSE(ieee_detected_violation);
        UNIT_ASSERT_TRUE(flow_detected_violation);
        UNIT_ASSERT_EQ((int64_t)exact_res.value, 6LL);
    }

    UNIT_TEST_SUITE_END();
}
