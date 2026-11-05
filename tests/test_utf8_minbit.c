#include "unit_test_kit.h"

int main(void) {
    UNIT_TEST_SUITE_BEGIN(
        "UNIT Suite 1: Branchless SWAR UTF-8 Control Sieve & Minimal-Bit Projection");

    /* =====================================================================
     * Stage 1: 64-Bit SWAR Classification & 8-Digit Parallel Parser
     * ===================================================================== */
    UNIT_STAGE_BEGIN(1, "64-Bit SWAR Byte Classification & 8-Digit Parallel Integer Parser");
    {
        const char *sample8 = "20261105";
        uint64_t word8 = 0ULL;
        memcpy(&word8, sample8, 8U);

        UNIT_ASSERT_EQ(unit_swar_ascii_mask8(word8), 0xFFU);
        UNIT_ASSERT_EQ(unit_swar_digit_mask8(word8), 0xFFU);
        UNIT_ASSERT_EQ(unit_swar_cont_mask8(word8), 0x00U);
        UNIT_ASSERT_EQ(unit_swar_parse_8digits_u64(word8), 20261105U);

        /* 4-bit BCD SWAR round-trip (64 bits -> 32 bits -> 64 bits) */
        uint32_t bcd32 = unit_swar_pack_4bit_bcd_u64(word8);
        uint64_t unpacked_bcd = unit_swar_unpack_4bit_bcd_u64(bcd32);
        UNIT_ASSERT_EQ(unpacked_bcd, word8);

        /* 7-bit ASCII SWAR round-trip (64 bits -> 56 bits -> 64 bits) */
        const char *asc8 = "FLOW_ASM";
        uint64_t asc_word = 0ULL;
        memcpy(&asc_word, asc8, 8U);
        uint64_t packed56 = unit_swar_pack_7bit_u64(asc_word);
        UNIT_ASSERT_EQ(packed56 >> 56U, 0ULL);
        uint64_t unpacked_asc = unit_swar_unpack_7bit_u64(packed56);
        UNIT_ASSERT_EQ(unpacked_asc, asc_word);
    }

    /* =====================================================================
     * Stage 2: Wave64 Minimal-Bit Lossless Codec (BCD, 7-bit, Multibyte UTF-8)
     * ===================================================================== */
    UNIT_STAGE_BEGIN(2, "Wave64 Minimal-Bit Lossless Codec (4b BCD, 7b ASCII, Prefix-Stripped)");
    {
        /* 2a. Pure decimal stream -> 4-bit BCD class (50% bit compression) */
        const char *digits = "9876543210123456";
        size_t dlen = strlen(digits);
        uint64_t sieve_d = UNIT_SIEVE_ALL_SOUND;
        UnitUtf8MinBitWave64 wave_d;
        UNIT_ASSERT_EQ(
            unit_utf8_encode_minbit_wave64((const uint8_t *)digits, dlen, &wave_d, &sieve_d), 0);
        UNIT_ASSERT_EQ(sieve_d, UNIT_SIEVE_ALL_SOUND);
        UNIT_ASSERT_EQ(wave_d.control.bit_class, UNIT_BIT_CLASS_4B_BCD);
        UNIT_ASSERT_EQ(wave_d.control.min_bit_len, dlen * 4U); /* 16 chars * 4b = 64 bits! */

        uint8_t dec_d[UNIT_WAVE_BYTES];
        size_t out_dlen = unit_utf8_decode_minbit_wave64(&wave_d, dec_d, &sieve_d);
        UNIT_ASSERT_EQ(out_dlen, dlen);
        UNIT_ASSERT_EQ(memcmp(dec_d, digits, dlen), 0);

        /* 2b. Pure ASCII protocol string -> 7-bit stripped class */
        const char *proto = "PROTO:TCP/LAN/ASM->FLOW_V4";
        size_t plen = strlen(proto);
        uint64_t sieve_p = UNIT_SIEVE_ALL_SOUND;
        UnitUtf8MinBitWave64 wave_p;
        UNIT_ASSERT_EQ(
            unit_utf8_encode_minbit_wave64((const uint8_t *)proto, plen, &wave_p, &sieve_p), 0);
        UNIT_ASSERT_EQ(wave_p.control.bit_class, UNIT_BIT_CLASS_7B_ASCII);
        UNIT_ASSERT_EQ(wave_p.control.min_bit_len, plen * 7U);

        uint8_t dec_p[UNIT_WAVE_BYTES];
        size_t out_plen = unit_utf8_decode_minbit_wave64(&wave_p, dec_p, &sieve_p);
        UNIT_ASSERT_EQ(out_plen, plen);
        UNIT_ASSERT_EQ(memcmp(dec_p, proto, plen), 0);

        /* 2c. Mixed 1/2/3/4-byte UTF-8: strips all prefix bits (0, 10, 110, 1110, 11110) */
        const char *mixed = "電:λ→⚡𐍈!";
        /*
         * Breakdown of "電:λ→⚡𐍈!":
         *  '電' (U+96FB, 3B: E9 9B BB)
         *  ':'  (U+003A, 1B: 3A)
         *  'λ'  (U+03BB, 2B: CE BB)
         *  '→'  (U+2192, 3B: E2 86 92)
         *  '⚡' (U+26A1, 3B: E2 9A A1)
         *  '𐍈'  (U+10348, 4B: F0 90 8D 88)
         *  '!'  (U+0021, 1B: 21)
         * Total = 17 UTF-8 bytes, 7 Unicode codepoints.
         * Raw bits = 17 * 8 = 136 bits.
         * Stripped minimal bits = 2*7(ASCII) + 1*5(lead2) + 3*4(lead3) + 1*3(lead4) + 10*6(cont)
         *                       = 14 + 5 + 12 + 3 + 60 = 94 bits!
         */
        size_t mlen = strlen(mixed);
        UNIT_ASSERT_EQ(mlen, 17U);
        uint64_t sieve_m = UNIT_SIEVE_ALL_SOUND;
        UnitUtf8MinBitWave64 wave_m;
        UNIT_ASSERT_EQ(
            unit_utf8_encode_minbit_wave64((const uint8_t *)mixed, mlen, &wave_m, &sieve_m), 0);
        UNIT_ASSERT_EQ(sieve_m, UNIT_SIEVE_ALL_SOUND);
        UNIT_ASSERT_EQ(wave_m.control.bit_class, UNIT_BIT_CLASS_UTF8_PREFIX_STRIP);
        UNIT_ASSERT_EQ(wave_m.control.codepoint_count, 7U);
        UNIT_ASSERT_EQ(wave_m.control.min_bit_len, 94U);

        uint8_t dec_m[UNIT_WAVE_BYTES];
        size_t out_mlen = unit_utf8_decode_minbit_wave64(&wave_m, dec_m, &sieve_m);
        UNIT_ASSERT_EQ(out_mlen, mlen);
        UNIT_ASSERT_EQ(memcmp(dec_m, mixed, mlen), 0);

        /* Direct 21-bit Unicode scalar extraction from minimal bits (no UTF-8 intermediate) */
        uint32_t cps[UNIT_WAVE_BYTES];
        size_t cp_cnt = unit_utf8_extract_codepoints_wave64(&wave_m, cps, &sieve_m);
        UNIT_ASSERT_EQ(cp_cnt, 7U);
        UNIT_ASSERT_EQ(cps[0], 0x96FBU);  /* 電 */
        UNIT_ASSERT_EQ(cps[1], 0x003AU);  /* :  */
        UNIT_ASSERT_EQ(cps[2], 0x03BBU);  /* λ  */
        UNIT_ASSERT_EQ(cps[3], 0x2192U);  /* →  */
        UNIT_ASSERT_EQ(cps[4], 0x26A1U);  /* ⚡ */
        UNIT_ASSERT_EQ(cps[5], 0x10348U); /* 𐍈  */
        UNIT_ASSERT_EQ(cps[6], 0x0021U);  /* !  */
    }

    /* =====================================================================
     * Stage 3: RFC 3629 Ill-Formed UTF-8 Sieve Rejection
     * ===================================================================== */
    UNIT_STAGE_BEGIN(3, "RFC 3629 Ill-Formed & Overlong UTF-8 Hardware/Protocol Sieve");
    {
        /* 3a. 2-byte overlong '/' (0xC0 0xAF) */
        const uint8_t overlong2[2] = {0xC0U, 0xAFU};
        uint64_t s1 = UNIT_SIEVE_ALL_SOUND;
        UnitUtf8ControlWave64 c1 = unit_utf8_classify_wave64(overlong2, 2U, &s1);
        UNIT_ASSERT_EQ(c1.is_valid_utf8, 0U);
        UNIT_ASSERT_EQ(s1 & UNIT_SIEVE_UTF8_WELL_FORMED, 0ULL);
        UNIT_ASSERT_EQ(s1 & UNIT_SIEVE_UTF8_MINIMAL_LEN, 0ULL);

        /* 3b. UTF-16 Surrogate U+D800 (0xED 0xA0 0x80) */
        const uint8_t surr[3] = {0xEDU, 0xA0U, 0x80U};
        uint64_t s2 = UNIT_SIEVE_ALL_SOUND;
        UnitUtf8ControlWave64 c2 = unit_utf8_classify_wave64(surr, 3U, &s2);
        UNIT_ASSERT_EQ(c2.is_valid_utf8, 0U);
        UNIT_ASSERT_EQ(s2 & UNIT_SIEVE_UTF8_WELL_FORMED, 0ULL);

        /* 3c. Lone continuation byte (0x80) */
        const uint8_t lone_cont[1] = {0x80U};
        uint64_t s3 = UNIT_SIEVE_ALL_SOUND;
        UnitUtf8ControlWave64 c3 = unit_utf8_classify_wave64(lone_cont, 1U, &s3);
        UNIT_ASSERT_EQ(c3.is_valid_utf8, 0U);
        UNIT_ASSERT_EQ(s3 & UNIT_SIEVE_UTF8_WELL_FORMED, 0ULL);
    }

    /* =====================================================================
     * Stage 4: Direct UTF-8 to FLOW Q48.16 Fixed-Point Projection
     * ===================================================================== */
    UNIT_STAGE_BEGIN(4, "Direct UTF-8 to FLOW Q48.16 Fixed-Point Projection");
    {
        uint64_t sieve_ok = UNIT_SIEVE_ALL_SOUND;
        const char *num_str = "-42.25";
        FlowResultU64 r1 =
            unit_utf8_parse_q16((const uint8_t *)num_str, strlen(num_str), &sieve_ok);
        UNIT_ASSERT_FALSE(flow_result_u64_is_boundary(r1));
        UNIT_ASSERT_EQ((int64_t)r1.value, -(42LL * 65536LL + 16384LL));
        UNIT_ASSERT_EQ(sieve_ok, UNIT_SIEVE_ALL_SOUND);

        /* 8-digit SWAR fast-path */
        const char *fast8 = "00001024";
        FlowResultU64 r2 = unit_utf8_parse_q16((const uint8_t *)fast8, 8U, &sieve_ok);
        UNIT_ASSERT_FALSE(flow_result_u64_is_boundary(r2));
        UNIT_ASSERT_EQ((int64_t)r2.value, 1024LL << 16U);

        /* Invalid non-numeric token -> pulls down UNIT_SIEVE_DECIMAL_VALID + Boundary Tag */
        uint64_t sieve_bad = UNIT_SIEVE_ALL_SOUND;
        const char *bad_str = "12.34xyz";
        FlowResultU64 r_bad =
            unit_utf8_parse_q16((const uint8_t *)bad_str, strlen(bad_str), &sieve_bad);
        UNIT_ASSERT_TRUE(flow_result_u64_is_boundary(r_bad));
        UNIT_ASSERT_EQ(sieve_bad & UNIT_SIEVE_DECIMAL_VALID, 0ULL);

        /* D10: Near-INT64_MAX fractional overflow guard (int_part == INT64_MAX >> 16, .999999) */
        uint64_t sieve_ovf = UNIT_SIEVE_ALL_SOUND;
        const char *ovf_str = "140737488355327.999999";
        FlowResultU64 r_ovf =
            unit_utf8_parse_q16((const uint8_t *)ovf_str, strlen(ovf_str), &sieve_ovf);
        UNIT_ASSERT_TRUE(flow_result_u64_is_boundary(r_ovf));
        UNIT_ASSERT_EQ(sieve_ovf & FLOW_ASM_SIEVE_INT_NO_OVERFLOW, 0ULL);
        UNIT_ASSERT_EQ(sieve_ovf & FLOW_ASM_SIEVE_DOMAIN_VALID, 0ULL);
    }

    /* =====================================================================
     * Stage 5: IEEE Audit Regressions (D1, D2, D3, D7, D8, D10)
     * ===================================================================== */
    UNIT_STAGE_BEGIN(
        5, "IEEE Audit Regressions: D1 ISQRT, D2 SWAR, D3 Jet2, D7 512b, D8 0-ULP, D10 Ternary");
    {
        /* D1: Exact floor(sqrt(x)) across [0, UINT64_MAX] */
        for (uint64_t x = 0ULL; x <= 4096ULL; ++x) {
            UNIT_ASSERT_EQ(unit_asm_isqrt_u64_fast(x), flow_asm_isqrt_u64(x));
        }
        for (uint32_t k = 0U; k < 64U; ++k) {
            uint64_t p = 1ULL << k;
            UNIT_ASSERT_EQ(unit_asm_isqrt_u64_fast(p - 1ULL), flow_asm_isqrt_u64(p - 1ULL));
            UNIT_ASSERT_EQ(unit_asm_isqrt_u64_fast(p), flow_asm_isqrt_u64(p));
            if (p < UINT64_MAX) {
                UNIT_ASSERT_EQ(unit_asm_isqrt_u64_fast(p + 1ULL), flow_asm_isqrt_u64(p + 1ULL));
            }
        }
        for (uint64_t s = 1ULL; s <= 65536ULL; s += 19ULL) {
            uint64_t sq = (s * 65537ULL);
            uint64_t x = sq * sq;
            UNIT_ASSERT_EQ(unit_asm_isqrt_u64_fast(x - 1ULL), sq - 1ULL);
            UNIT_ASSERT_EQ(unit_asm_isqrt_u64_fast(x), sq);
            UNIT_ASSERT_EQ(unit_asm_isqrt_u64_fast(x + 1ULL), sq);
        }
        for (uint64_t d = 0ULL; d < 256ULL; ++d) {
            uint64_t x = UINT64_MAX - d;
            UNIT_ASSERT_EQ(unit_asm_isqrt_u64_fast(x), UINT32_MAX);
        }
        uint64_t max_sq = (uint64_t)UINT32_MAX * (uint64_t)UINT32_MAX;
        UNIT_ASSERT_EQ(unit_asm_isqrt_u64_fast(max_sq - 1ULL), (uint64_t)UINT32_MAX - 1ULL);
        UNIT_ASSERT_EQ(unit_asm_isqrt_u64_fast(max_sq), (uint64_t)UINT32_MAX);

        /* D2: Inter-lane borrow/carry isolation in unit_swar_digit_mask8 */
        uint64_t w_dot_zeros = 0ULL;
        memcpy(&w_dot_zeros, "12.00000", 8U);
        UNIT_ASSERT_EQ(unit_swar_digit_mask8(w_dot_zeros), 0xFBU);

        uint64_t w_slash_zeros = 0ULL;
        memcpy(&w_slash_zeros, "/0000000", 8U);
        UNIT_ASSERT_EQ(unit_swar_digit_mask8(w_slash_zeros), 0xFEU);

        uint64_t w_utf8_nine = 0ULL;
        memcpy(&w_utf8_nine,
               "\xE9\x9B\xBB"
               "90123",
               8U); /* '電' (E9 9B BB) + "90123" */
        UNIT_ASSERT_EQ(unit_swar_digit_mask8(w_utf8_nine), 0xF8U);

        uint64_t w_ff_nines = 0ULL;
        memcpy(&w_ff_nines,
               "\xFF"
               "9999999",
               8U);
        UNIT_ASSERT_EQ(unit_swar_digit_mask8(w_ff_nines), 0xFEU);

        /* D3: unit_jet2_solve_root_fast64 fractional precision, cancellation & INT64_MIN safety */
        int64_t t_fast = 0, t_ref = 0;
        /* Small integers: v=1, d1=0, d2=-1 -> t = sqrt(2) ~= 1.41421356 -> 92681 in Q16 */
        UNIT_ASSERT_TRUE(unit_jet2_solve_root_fast64(1LL, 0LL, -1LL, INT64_MAX, &t_fast));
        UNIT_ASSERT_TRUE(flow_jet2_solve_root(flow_jet2_make(1LL, 0LL, -1LL), INT64_MAX, &t_ref));
        UNIT_ASSERT_EQ(t_fast, t_ref);
        UNIT_ASSERT_EQ(t_fast, 92681LL);

        /* Cancellation case: v=10, d1=100 (r=-100), d2=-1 (c=+1) -> t = 100 + sqrt(10020) */
        UNIT_ASSERT_TRUE(unit_jet2_solve_root_fast64(10LL, 100LL, -1LL, INT64_MAX, &t_fast));
        UNIT_ASSERT_TRUE(
            flow_jet2_solve_root(flow_jet2_make(10LL, 100LL, -1LL), INT64_MAX, &t_ref));
        UNIT_ASSERT_EQ(t_fast, t_ref);
        UNIT_ASSERT_EQ(t_fast, 13113750LL);

        /* Concave deceleration case: v=10, d1=-10 (r=10), d2=1 (c=-1) -> t = 10 - sqrt(80) */
        UNIT_ASSERT_TRUE(unit_jet2_solve_root_fast64(10LL, -10LL, 1LL, INT64_MAX, &t_fast));
        UNIT_ASSERT_TRUE(flow_jet2_solve_root(flow_jet2_make(10LL, -10LL, 1LL), INT64_MAX, &t_ref));
        UNIT_ASSERT_TRUE((t_fast - t_ref <= 1LL) && (t_ref - t_fast <= 1LL));

        /* INT64_MIN and boundary tags must return false without UB */
        UNIT_ASSERT_FALSE(unit_jet2_solve_root_fast64(10LL, INT64_MIN, -1LL, INT64_MAX, &t_fast));
        UNIT_ASSERT_FALSE(unit_jet2_solve_root_fast64(10LL, -1LL, INT64_MIN, INT64_MAX, &t_fast));
        UNIT_ASSERT_FALSE(unit_jet2_solve_root_fast64(INT64_MIN, -1LL, -1LL, INT64_MAX, &t_fast));

        /* D7: Cache-line compactness & 64-byte (512-bit) lossless raw byte passthrough */
        UNIT_ASSERT_EQ(sizeof(UnitUtf8ControlWave64), 64U);
        UNIT_ASSERT_EQ(sizeof(UnitUtf8MinBitWave64), 128U);
        uint8_t raw64[UNIT_WAVE_BYTES];
        for (size_t i = 0U; i < UNIT_WAVE_BYTES; ++i) {
            raw64[i] = (uint8_t)(0x80U | (uint8_t)i); /* Invalid UTF-8 across all 64 bytes */
        }
        uint64_t sieve_raw = UNIT_SIEVE_ALL_SOUND;
        UnitUtf8MinBitWave64 wave_raw;
        UNIT_ASSERT_EQ(
            unit_utf8_encode_minbit_wave64(raw64, UNIT_WAVE_BYTES, &wave_raw, &sieve_raw), -1);
        UNIT_ASSERT_EQ(wave_raw.control.bit_class, UNIT_BIT_CLASS_RAW_BYTES);
        UNIT_ASSERT_EQ(wave_raw.control.min_bit_len, 512U);
        uint8_t dec_raw[UNIT_WAVE_BYTES];
        uint64_t sieve_dec_raw = UNIT_SIEVE_ALL_SOUND;
        UNIT_ASSERT_EQ(unit_utf8_decode_minbit_wave64(&wave_raw, dec_raw, &sieve_dec_raw),
                       UNIT_WAVE_BYTES);
        UNIT_ASSERT_EQ(memcmp(dec_raw, raw64, UNIT_WAVE_BYTES), 0);
        UNIT_ASSERT_EQ(sieve_dec_raw & UNIT_SIEVE_UTF8_WELL_FORMED, 0ULL);

        /* D8: 0-ULP exact match between unit_swar_wire8_to_q16_branchless and unit_utf8_parse_q16
         */
        const char *ulp_cases[] = {"00000042", "+1234567", "-1234567", "123456.7", "12345.67",
                                   "1234.567", "123.4567", "12.34567", "1.234567", ".1234567",
                                   "+.125000", "-.375000", "0.000625", "12.00000", "99.99999",
                                   "-0.00001", "+0.33333", "-7.77777"};
        for (size_t i = 0U; i < sizeof(ulp_cases) / sizeof(ulp_cases[0]); ++i) {
            uint64_t w8 = 0ULL;
            memcpy(&w8, ulp_cases[i], 8U);
            uint64_t s_swar = UNIT_SIEVE_ALL_SOUND;
            uint64_t s_parse = UNIT_SIEVE_ALL_SOUND;
            int64_t q_swar = unit_swar_wire8_to_q16_branchless(w8, &s_swar);
            FlowResultU64 q_parse =
                unit_utf8_parse_q16((const uint8_t *)ulp_cases[i], 8U, &s_parse);
            UNIT_ASSERT_EQ(s_swar, UNIT_SIEVE_ALL_SOUND);
            UNIT_ASSERT_EQ(s_parse, UNIT_SIEVE_ALL_SOUND);
            UNIT_ASSERT_EQ(q_swar, (int64_t)q_parse.value);
        }

        /* D10: 1.58-Bit Balanced Ternary Wave64 Codec */
        int8_t trits[64];
        for (uint32_t i = 0U; i < 64U; ++i) {
            trits[i] = (int8_t)((int)(i % 3U) - 1); /* -1, 0, +1 repeating */
        }
        FlowBmfTernary64 t_in = flow_bmf_ternary64_encode(trits);
        uint64_t s_tern = UNIT_SIEVE_ALL_SOUND;
        UnitUtf8MinBitWave64 wave_t;
        UNIT_ASSERT_EQ(unit_ternary64_encode_minbit_wave64(t_in, 64U, &wave_t, &s_tern), 0);
        UNIT_ASSERT_EQ(s_tern, UNIT_SIEVE_ALL_SOUND);
        UNIT_ASSERT_EQ(wave_t.control.bit_class, UNIT_BIT_CLASS_1B58_TERNARY);
        UNIT_ASSERT_EQ(wave_t.control.min_bit_len, 102U); /* ceil(64 * 1.585) = 102 bits */

        FlowBmfTernary64 t_out = {0ULL, 0ULL};
        UNIT_ASSERT_EQ(unit_ternary64_decode_minbit_wave64(&wave_t, &t_out, &s_tern), 64U);
        UNIT_ASSERT_EQ(t_out.nonzero_mask, t_in.nonzero_mask);
        UNIT_ASSERT_EQ(t_out.sign_mask, t_in.sign_mask);

        /* Non-canonical ternary (sign_mask bit set where nonzero_mask is 0) clears sieve */
        FlowBmfTernary64 t_noncanon = {.nonzero_mask = 0x01ULL, .sign_mask = 0x03ULL};
        uint64_t s_noncanon = UNIT_SIEVE_ALL_SOUND;
        UNIT_ASSERT_EQ(unit_ternary64_encode_minbit_wave64(t_noncanon, 8U, &wave_t, &s_noncanon),
                       0);
        UNIT_ASSERT_EQ(s_noncanon & FLOW_ASM_SIEVE_DOMAIN_VALID, 0ULL);
        UNIT_ASSERT_EQ(s_noncanon & UNIT_SIEVE_UTF8_MINIMAL_LEN, 0ULL);
    }

    UNIT_TEST_SUITE_END();
}
