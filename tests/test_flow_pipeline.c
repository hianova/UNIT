#include "unit_test_kit.h"

#include <sys/resource.h>

int main(int argc, char **argv) {
    bool report_mode = false;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--report") == 0) {
            report_mode = true;
        }
    }

    UNIT_TEST_SUITE_BEGIN(
        "UNIT Suite 3: End-to-End UTF-8 Wire -> Minimal-Bit -> FLOW v4 ASM Execution");

    /* =====================================================================
     * Stage 1: Full Wire-to-FLOW Pipeline (u_ext, b_ext, Schmitt, 2-Jet)
     * ===================================================================== */
    UNIT_STAGE_BEGIN(1, "Observable UTF-8 Ingest -> Minimal-Bit Arena -> FLOW QPoly & 2-Jet");
    {
        UnitFlowEngine eng;
        unit_flow_engine_init(&eng);

        /*
         * Step 1: Configure Plane 0 upper bound b_enter = 50.0, hysteresis recovery b_exit = 40.0
         * via UTF-8 packets ingested into the Z / 2^k Z arena.
         */
        const char *b_enter_utf8 = "50.0";
        const char *b_exit_utf8 = "40.0";
        UnitPacketView pkt_b_enter =
            unit_ingest_utf8_packet(&eng, (uint8_t)UNIT_OP_LOAD_B_EXT, 0U, 0ULL,
                                    (const uint8_t *)b_enter_utf8, strlen(b_enter_utf8));
        UnitPacketView pkt_b_exit =
            unit_ingest_utf8_packet(&eng, (uint8_t)UNIT_OP_SET_HYSTERESIS, 0U, 1ULL,
                                    (const uint8_t *)b_exit_utf8, strlen(b_exit_utf8));

        UnitCell128 c_enter = {.packet = pkt_b_enter};
        UnitCell128 c_exit = {.packet = pkt_b_exit};
        UNIT_ASSERT_EQ(UnitCellRing64_push(&eng.ingress_ring, c_enter), 1);
        UNIT_ASSERT_EQ(UnitCellRing64_push(&eng.ingress_ring, c_exit), 1);

        UnitExecOutcome last;
        UNIT_ASSERT_EQ(unit_flow_drain_ring(&eng, &last), 2U);
        UNIT_ASSERT_EQ(last.sieve, UNIT_SIEVE_ALL_SOUND);

        /*
         * Step 2: Stream three UTF-8 observations on axis 0:
         *   t=0: "10.0" -> u=10
         *   t=1: "25.0" -> u=25, du=+15
         *   t=2: "45.0" -> u=45, du=+20, ddu=+5
         */
        const char *obs[3] = {"10.0", "25.0", "45.0"};
        for (uint32_t k = 0U; k < 3U; ++k) {
            UnitPacketView pkt_u =
                unit_ingest_utf8_packet(&eng, (uint8_t)UNIT_OP_LOAD_U_EXT, 0U, 2ULL + k,
                                        (const uint8_t *)obs[k], strlen(obs[k]));
            UnitCell128 cell_u = {.packet = pkt_u};
            (void)unit_flow_exec_cell(&eng, cell_u);
        }

        UNIT_ASSERT_EQ(eng.u_ext[0], 45LL * FLOW_Q16_ONE);
        UNIT_ASSERT_EQ(eng.du_ext[0], 20LL * FLOW_Q16_ONE);
        UNIT_ASSERT_EQ(eng.ddu_ext[0], 5LL * FLOW_Q16_ONE);

        /* Step 3: Evaluate FlowQPoly at u_0 = 45.0 <= b_enter (50.0) -> Interior Sound */
        UnitCell128 eval_cell;
        eval_cell.packet.protocol_header =
            unit_control_pack((uint8_t)UNIT_OP_EVAL_QPOLY, 0U, (uint8_t)UNIT_CURRENT_L1_ARENA, 0U,
                              (uint16_t)(UNIT_SIEVE_ALL_SOUND & 0xFFFFULL), 0U);
        eval_cell.packet.body_slice = 0ULL;
        UnitExecOutcome ev_ok = unit_flow_exec_cell(&eng, eval_cell);
        UNIT_ASSERT_EQ(ev_ok.violated_mask, 0ULL);
        UNIT_ASSERT_EQ(ev_ok.primary_value_q16, 5LL * FLOW_Q16_ONE); /* min_slack = 50 - 45 = 5.0 */
        UNIT_ASSERT_NE(ev_ok.sieve & FLOW_ASM_SIEVE_QPOLY_INTERIOR, 0ULL);

        /* Step 4: Execute 2-Jet Worldline Forecast -> predicts imminent exit across Plane 0! */
        UnitCell128 jet_cell;
        jet_cell.packet.protocol_header =
            unit_control_pack((uint8_t)UNIT_OP_PREDICT_JET2, 0U, (uint8_t)UNIT_CURRENT_L1_ARENA, 0U,
                              (uint16_t)(UNIT_SIEVE_ALL_SOUND & 0xFFFFULL), 0U);
        jet_cell.packet.body_slice = 0ULL;
        UnitExecOutcome fc_out = unit_flow_exec_cell(&eng, jet_cell);
        UNIT_ASSERT_EQ(fc_out.will_exit_poly, 1U);
        UNIT_ASSERT_EQ(fc_out.focus_plane_or_dim, 0U);
        UNIT_ASSERT_TRUE(fc_out.primary_value_q16 > 0 && fc_out.primary_value_q16 < FLOW_Q16_ONE);

        /* Step 5: Push u_0 = "52.0" (> 50.0) -> violates Plane 0; then "45.0" stays violated
         * due to Schmitt hysteresis until dropping <= b_exit (40.0) */
        UnitPacketView pkt_52 = unit_ingest_utf8_packet(&eng, (uint8_t)UNIT_OP_LOAD_U_EXT, 0U,
                                                        10ULL, (const uint8_t *)"52.0", 4U);
        (void)unit_flow_exec_cell(&eng, (UnitCell128){.packet = pkt_52});
        UnitExecOutcome ev_viol = unit_flow_exec_cell(&eng, eval_cell);
        UNIT_ASSERT_EQ(ev_viol.violated_mask, 1ULL);
        UNIT_ASSERT_EQ(ev_viol.sieve & FLOW_ASM_SIEVE_QPOLY_INTERIOR, 0ULL);

        /* Drop to "45.0" (between b_exit=40 and b_enter=50) -> Schmitt keeps violation latched! */
        UnitPacketView pkt_45 = unit_ingest_utf8_packet(&eng, (uint8_t)UNIT_OP_LOAD_U_EXT, 0U,
                                                        11ULL, (const uint8_t *)"45.0", 4U);
        (void)unit_flow_exec_cell(&eng, (UnitCell128){.packet = pkt_45});
        UnitExecOutcome ev_latched = unit_flow_exec_cell(&eng, eval_cell);
        UNIT_ASSERT_EQ(ev_latched.violated_mask, 1ULL);

        /* Drop to "39.0" (<= b_exit=40.0) -> Schmitt recovers interior state! */
        UnitPacketView pkt_39 = unit_ingest_utf8_packet(&eng, (uint8_t)UNIT_OP_LOAD_U_EXT, 0U,
                                                        12ULL, (const uint8_t *)"39.0", 4U);
        (void)unit_flow_exec_cell(&eng, (UnitCell128){.packet = pkt_39});
        UnitExecOutcome ev_rec = unit_flow_exec_cell(&eng, eval_cell);
        UNIT_ASSERT_EQ(ev_rec.violated_mask, 0ULL);
        UNIT_ASSERT_NE(ev_rec.sieve & FLOW_ASM_SIEVE_QPOLY_INTERIOR, 0ULL);
    }

    /* =====================================================================
     * Stage 2: 1,000,000 Hot-Path Iterations (Zero Kernel Faults / Zero Copy)
     * ===================================================================== */
    UNIT_STAGE_BEGIN(2, "1,000,000 Unified Cell Executions (ru_minflt = 0, ru_majflt = 0)");
    {
        FlowUmaBuffer uma;
        UNIT_ASSERT_EQ(flow_uma_buffer_alloc(&uma, sizeof(UnitFlowEngine)), 0);
        UnitFlowEngine *eng = (UnitFlowEngine *)uma.cpu_ptr;
        unit_flow_engine_init(eng);

        uint64_t word8 = 0ULL;
        memcpy(&word8, "00000042", 8U);
        uint64_t sieve = UNIT_SIEVE_ALL_SOUND;
        UnitCell128 inline_u_cell =
            unit_make_inline_ascii_cell((uint8_t)UNIT_OP_LOAD_U_EXT, 0U, word8, 8U, &sieve);
        UnitCell128 eval_cell;
        eval_cell.packet.protocol_header =
            unit_control_pack((uint8_t)UNIT_OP_EVAL_QPOLY, 0U, (uint8_t)UNIT_CURRENT_INLINE_REG, 0U,
                              (uint16_t)(UNIT_SIEVE_ALL_SOUND & 0xFFFFULL), 0U);
        eval_cell.packet.body_slice = 0ULL;

        /* Warm up 1 pass to pre-fault code/rodata pages before rusage measurement */
        (void)flow_clock_monotonic_ns();
        (void)flow_hardware_cycles();
        (void)unit_flow_exec_cell(eng, inline_u_cell);
        (void)unit_flow_exec_cell(eng, eval_cell);
        eng->executed_cells = 0ULL;

        struct rusage ru_before, ru_after;
        getrusage(RUSAGE_SELF, &ru_before);
        uint64_t t0_ns = flow_clock_monotonic_ns();
        uint64_t c0 = flow_hardware_cycles();

        const uint32_t iters = 500000U;
        uint64_t acc_sieve = UNIT_SIEVE_ALL_SOUND;
        for (uint32_t i = 0U; i < iters; ++i) {
            UnitExecOutcome o1 = unit_flow_exec_cell(eng, inline_u_cell);
            UnitExecOutcome o2 = unit_flow_exec_cell(eng, eval_cell);
            acc_sieve &= (o1.sieve & o2.sieve);
        }

        uint64_t c1 = flow_hardware_cycles();
        uint64_t t1_ns = flow_clock_monotonic_ns();
        getrusage(RUSAGE_SELF, &ru_after);

        long minflt_delta = ru_after.ru_minflt - ru_before.ru_minflt;
        long majflt_delta = ru_after.ru_majflt - ru_before.ru_majflt;

        UNIT_ASSERT_EQ(eng->executed_cells, 1000000ULL);
        UNIT_ASSERT_EQ(acc_sieve, UNIT_SIEVE_ALL_SOUND);
        UNIT_ASSERT_EQ(minflt_delta, 0L);
        UNIT_ASSERT_EQ(majflt_delta, 0L);

        if (report_mode) {
            uint64_t elapsed_ns = (t1_ns >= t0_ns) ? (t1_ns - t0_ns) : 0ULL;
            uint64_t elapsed_us = elapsed_ns / 1000ULL;
            uint64_t ps_per_cell = (elapsed_ns * 1000ULL) / 1000000ULL;
            printf("  ⚡ [Benchmark] 1,000,000 Isomorphic Cell Dispatches: %llu us (%llu.%03llu "
                   "ns/cell, %llu hw ticks, minflt=%ld, majflt=%ld)\n",
                   (unsigned long long)elapsed_us, (unsigned long long)(ps_per_cell / 1000ULL),
                   (unsigned long long)(ps_per_cell % 1000ULL), (unsigned long long)(c1 - c0),
                   minflt_delta, majflt_delta);
        }

        flow_uma_buffer_free(&uma);
    }

    /* =====================================================================
     * Stage 3: IEEE Audit Regressions (D4, D5, D6, D9)
     * ===================================================================== */
    UNIT_STAGE_BEGIN(
        3, "IEEE Audit Regressions: D4 Clamp, D5 Sieve Guard, D6 QDot Ovf, D9 Jet2 Hysteresis");
    {
        UnitFlowEngine eng;
        unit_flow_engine_init(&eng);
        UNIT_ASSERT_EQ(UnitFlowPoly4D_SHIFT, 0);

        /* D9: PREDICT_JET2 respects latched Schmitt hysteresis b_exit (40.0 instead of 50.0) */
        UnitFlowPoly4D_set_hysteresis(&eng.qpoly, 0U, 50LL * FLOW_Q16_ONE, 40LL * FLOW_Q16_ONE);
        eng.u_ext[0] = (int32_t)(45LL * FLOW_Q16_ONE);
        eng.du_ext[0] = (int32_t)(5LL * FLOW_Q16_ONE);
        eng.ddu_ext[0] = 0;
        eng.prev_violated_mask = 1ULL; /* Plane 0 latched in violation (45.0 > b_exit=40.0) */

        UnitCell128 jet_cell;
        jet_cell.packet.protocol_header =
            unit_control_pack((uint8_t)UNIT_OP_PREDICT_JET2, 0U, (uint8_t)UNIT_CURRENT_INLINE_REG,
                              0U, (uint16_t)(UNIT_SIEVE_ALL_SOUND & 0xFFFFULL), 0U);
        jet_cell.packet.body_slice = 0ULL;
        UnitExecOutcome fc_hyst = unit_flow_exec_cell(&eng, jet_cell);
        UNIT_ASSERT_EQ(fc_hyst.will_exit_poly, 1U);
        UNIT_ASSERT_EQ(fc_hyst.primary_value_q16, 0LL); /* Already past active bound b_exit=40.0 */

        /* D5: Malformed packet & len > 64 packet must zero arena and NOT mutate u_ext / b_ext */
        eng.u_ext[0] = (int32_t)(25LL * FLOW_Q16_ONE);
        uint8_t over64[70];
        memset(over64, '1', sizeof(over64));
        UnitPacketView pkt_over64 = unit_ingest_utf8_packet(&eng, (uint8_t)UNIT_OP_LOAD_U_EXT, 0U,
                                                            0ULL, over64, sizeof(over64));
        UNIT_ASSERT_EQ(eng.wave_arena[0].control.byte_len, 0U);
        UnitExecOutcome out_over64 = unit_flow_exec_cell(&eng, (UnitCell128){.packet = pkt_over64});
        UNIT_ASSERT_EQ(out_over64.sieve & UNIT_SIEVE_BITPACK_INBOUNDS, 0ULL);
        UNIT_ASSERT_EQ(eng.u_ext[0], 25LL * FLOW_Q16_ONE); /* Unchanged! */

        const char *bad_dec = "bad_num!";
        UnitPacketView pkt_bad = unit_ingest_utf8_packet(
            &eng, (uint8_t)UNIT_OP_LOAD_U_EXT, 0U, 1ULL, (const uint8_t *)bad_dec, strlen(bad_dec));
        UnitExecOutcome out_bad = unit_flow_exec_cell(&eng, (UnitCell128){.packet = pkt_bad});
        UNIT_ASSERT_EQ(out_bad.sieve & UNIT_SIEVE_DECIMAL_VALID, 0ULL);
        UNIT_ASSERT_EQ(eng.u_ext[0], 25LL * FLOW_Q16_ONE); /* Unchanged! */

        /* D4: 32-bit velocity/acceleration clamping in UNIT_OP_LOAD_U_EXT without wraparound */
        eng.u_ext[1] = -1500000000;
        eng.du_ext[1] = 0;
        UnitCell128 big_pos_cell;
        big_pos_cell.packet.protocol_header =
            unit_control_pack((uint8_t)UNIT_OP_LOAD_U_EXT, (uint8_t)UNIT_BIT_CLASS_RAW_BYTES,
                              (uint8_t)UNIT_CURRENT_INLINE_REG, 1U,
                              (uint16_t)(UNIT_SIEVE_ALL_SOUND & 0xFFFFULL), 64U);
        big_pos_cell.packet.body_slice = (uint64_t)1500000000LL;
        UnitExecOutcome out_clamp = unit_flow_exec_cell(&eng, big_pos_cell);
        UNIT_ASSERT_EQ(eng.u_ext[1], 1500000000);
        UNIT_ASSERT_EQ(eng.du_ext[1], INT32_MAX); /* Clamped, not wrapped negative! */
        UNIT_ASSERT_EQ(out_clamp.sieve & FLOW_ASM_SIEVE_INT_NO_OVERFLOW, 0ULL);
        UNIT_ASSERT_EQ(out_clamp.sieve & FLOW_ASM_SIEVE_Q_NO_SATURATION, 0ULL);

        /* D6: Inner-product overflow in UNIT_OP_EVAL_QPOLY pulls down
         * FLOW_ASM_SIEVE_INT_NO_OVERFLOW */
        for (uint32_t d = 0U; d < UNIT_STATE_DIM; ++d) {
            eng.qpoly.A[0][d] = INT32_MAX;
            eng.u_ext[d] = INT32_MAX;
        }
        UnitCell128 eval_cell;
        eval_cell.packet.protocol_header =
            unit_control_pack((uint8_t)UNIT_OP_EVAL_QPOLY, 0U, (uint8_t)UNIT_CURRENT_INLINE_REG, 0U,
                              (uint16_t)(UNIT_SIEVE_ALL_SOUND & 0xFFFFULL), 0U);
        eval_cell.packet.body_slice = 0ULL;
        UnitExecOutcome out_qdot_ovf = unit_flow_exec_cell(&eng, eval_cell);
        UNIT_ASSERT_EQ(out_qdot_ovf.sieve & FLOW_ASM_SIEVE_INT_NO_OVERFLOW, 0ULL);
    }

    /* =====================================================================
     * Stage 4: Universal `unit_scan` (Data Layer) & `unit_guard` (Control Layer)
     * ===================================================================== */
    UNIT_STAGE_BEGIN(
        4, "Universal High-Level Substrate: unit_scan (MinBit) & unit_guard (1.58b+Jet2)");
    {
        /* 4A: unit_scan_bcd_range_wave64 direct-on-minimal-bit scan (8 x 8-digit BCD lanes) */
        const char *bcd_stream64 = "00000100"
                                   "00000250"
                                   "00000500"
                                   "00000750"
                                   "00001000"
                                   "00001250"
                                   "00001500"
                                   "00002000";
        uint64_t s_scan = UNIT_SIEVE_ALL_SOUND;
        UnitUtf8MinBitWave64 bcd_wave;
        UNIT_ASSERT_EQ(
            unit_utf8_encode_minbit_wave64((const uint8_t *)bcd_stream64, 64U, &bcd_wave, &s_scan),
            0);
        UNIT_ASSERT_EQ(bcd_wave.control.bit_class, UNIT_BIT_CLASS_4B_BCD);

        uint64_t bcd_sum = 0ULL;
        uint8_t lane_mask = unit_scan_bcd_range_wave64(&bcd_wave, 250U, 1250U, &bcd_sum, &s_scan);
        /* Lanes 1..5 (250, 500, 750, 1000, 1250) match -> mask = 0x3E, sum = 3750 */
        UNIT_ASSERT_EQ(s_scan, UNIT_SIEVE_ALL_SOUND);
        UNIT_ASSERT_EQ(lane_mask, 0x3EU);
        UNIT_ASSERT_EQ(bcd_sum, 3750ULL);

        /* 4B: unit_scan_cells_56bit_match & unit_scan_cells_q16_range */
        UnitCell128 cells[4];
        uint64_t w_sym1 = 0ULL, w_sym2 = 0ULL;
        memcpy(&w_sym1, "LOG_WARN", 8U);
        memcpy(&w_sym2, "LOG_INFO", 8U);
        cells[0] =
            unit_make_inline_ascii_cell((uint8_t)UNIT_OP_LOAD_U_EXT, 1U, w_sym1, 8U, &s_scan);
        cells[1] =
            unit_make_inline_ascii_cell((uint8_t)UNIT_OP_LOAD_U_EXT, 2U, w_sym2, 8U, &s_scan);
        cells[2] =
            unit_make_inline_ascii_cell((uint8_t)UNIT_OP_LOAD_U_EXT, 3U, w_sym1, 8U, &s_scan);
        uint64_t w_num = 0ULL;
        memcpy(&w_num, "+0042.50", 8U);
        cells[3] = unit_wire8_single_pass_cell((uint8_t)UNIT_OP_LOAD_U_EXT, 0U, w_num, &s_scan);

        uint64_t axis_sum = 0ULL;
        size_t m56 = unit_scan_cells_56bit_match(cells, 4U, (uint8_t)UNIT_OP_LOAD_U_EXT,
                                                 unit_swar_pack_7bit_u64(w_sym1), &axis_sum);
        UNIT_ASSERT_EQ(m56, 2U);
        UNIT_ASSERT_EQ(axis_sum, 4ULL); /* axes 1 + 3 */

        int64_t q16_sum = 0LL;
        size_t mq16 = unit_scan_cells_q16_range(cells, 4U, 0U, 10LL * FLOW_Q16_ONE,
                                                50LL * FLOW_Q16_ONE, &q16_sum, &s_scan);
        UNIT_ASSERT_EQ(mq16, 1U);
        UNIT_ASSERT_EQ(q16_sum, 42LL * FLOW_Q16_ONE + 32768LL);

        /* 4C: unit_guard_step_ternary_jet2 (PASS -> TUNNEL_INTERCEPT -> HYSTERESIS_HOLD) */
        UnitFlowEngine geng;
        unit_flow_engine_init(&geng);
        UnitFlowPoly4D_set_hysteresis(&geng.qpoly, 0U, 50LL * FLOW_Q16_ONE, 40LL * FLOW_Q16_ONE);
        geng.cubical_state_mask = 0x00ULL;
        geng.boundary_filter_mask = 0x00ULL;

        /* 10 active trits (+1) -> dot product = +10, scaled by 2.0 Q16 = +20.0 per step */
        FlowBmfTernary64 w_t = {.nonzero_mask = 0x3FFULL, .sign_mask = 0x0ULL};
        FlowBmfTernary64 a_t = {.nonzero_mask = 0x3FFULL, .sign_mask = 0x0ULL};

        /* Step 1: u[0] = 20.0, du[0] = 20.0, slack = 30.0 -> t_hit ~= 1.07s > horizon 0.5s (PASS)
         */
        UnitGuardReport r1 = unit_guard_step_ternary_jet2(&geng, 0U, w_t, a_t, 2LL * FLOW_Q16_ONE,
                                                          FLOW_Q16_ONE / 2LL);
        UNIT_ASSERT_EQ(r1.verdict, UNIT_GUARD_PASS);
        UNIT_ASSERT_EQ(r1.ternary_score_q16, 20LL * FLOW_Q16_ONE);
        UNIT_ASSERT_EQ(r1.sieve, UNIT_SIEVE_ALL_SOUND);

        /* Step 2: u[0] = 40.0, du[0] = 20.0, slack = 10.0 -> t_hit = 0.5s <= horizon 1.0s
         * (INTERCEPT!) */
        UnitGuardReport r2 =
            unit_guard_step_ternary_jet2(&geng, 0U, w_t, a_t, 2LL * FLOW_Q16_ONE, FLOW_Q16_ONE);
        UNIT_ASSERT_EQ(r2.verdict, UNIT_GUARD_TUNNEL_INTERCEPT);
        UNIT_ASSERT_EQ(r2.focus_plane, 0U);
        UNIT_ASSERT_EQ(r2.kan_routed, 1U);
        UNIT_ASSERT_EQ(r2.next_cubical_mask, 1ULL);

        /* Step 3: u[0] = 60.0 > 50.0 -> HYSTERESIS_HOLD */
        UnitGuardReport r3 =
            unit_guard_step_ternary_jet2(&geng, 0U, w_t, a_t, 2LL * FLOW_Q16_ONE, FLOW_Q16_ONE);
        UNIT_ASSERT_EQ(r3.verdict, UNIT_GUARD_HYSTERESIS_HOLD);
        UNIT_ASSERT_EQ(r3.violated_mask, 1ULL);
    }

    UNIT_TEST_SUITE_END();
}
