#ifndef UNIT_UTF8_BIT_H
#define UNIT_UTF8_BIT_H

#include "flow_hott_jet.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * UNIT v1: Observable UTF-8 <-> Minimal-Bit + Control Signal Substrate
 * ============================================================================
 * Realizes the fundamental decomposition of observable UTF-8 wire streams:
 *
 *   UTF-8 Byte Stream  ===  [ Control Signal Masks ]  x  [ Minimal-Bit Payload ]
 *
 * Every UTF-8 byte consists of prefix control bits (`0`, `10`, `110`, `1110`,
 * `11110`) and raw payload bits (7, 6, 5, 4, or 3 bits; or 4 bits for BCD
 * numeric streams). By lifting the prefix topology into 64-bit Wave64 control
 * bitmasks (`ascii_mask`, `cont_mask`, `lead2_mask`, `lead3_mask`, `lead4_mask`),
 * all framing bits are stripped from the data plane, leaving a dense minimal-bit
 * stream that:
 *   1. Round-trips losslessly back to canonical UTF-8 in O(1) bitwise ops.
 *   2. Decodes directly to 21-bit Unicode scalar values without UTF-8 branching.
 *   3. Projects numeric observations directly into Q48.16 integers (u_ext, b_ext)
 *      via SWAR register algebra and FLOW's Total ASM Sieve.
 * ============================================================================
 */

#define UNIT_VERSION_MAJOR 1
#define UNIT_VERSION_MINOR 0
#define UNIT_VERSION_PATCH 0
#ifndef UNIT_VERSION_STRING
#define UNIT_VERSION_STRING "1.0.0"
#endif

#define UNIT_WAVE_BYTES 64U

/* Additional Sieve Axis bits for Observable UTF-8 & Protocol Framing (bits 10..15) */
typedef enum {
    UNIT_SIEVE_UTF8_WELL_FORMED = (1ULL << 10), /* Valid UTF-8 prefix/continuation/scalar */
    UNIT_SIEVE_UTF8_MINIMAL_LEN = (1ULL << 11), /* No overlong encodings */
    UNIT_SIEVE_PROTOCOL_SYNC = (1ULL << 12),    /* Frame header magic & class valid */
    UNIT_SIEVE_BITPACK_INBOUNDS = (1ULL << 13), /* Bitstream cursor within capacity */
    UNIT_SIEVE_DECIMAL_VALID = (1ULL << 14),    /* Valid numeric ASCII token */
    UNIT_SIEVE_ISOMORPHIC_FRAME = (1ULL << 15), /* Packet <-> CPU Opcode layout sound */
    UNIT_SIEVE_ALL_SOUND = (FLOW_ASM_SIEVE_ALL_SOUND | 0xFC00ULL)
} UnitSieveAxis;

/* Minimal-Bit Encoding Class (Current Control Spec for Payload Density) */
typedef enum {
    UNIT_BIT_CLASS_RAW_BYTES = 0,        /* 8.00 bits/byte: passthrough binary */
    UNIT_BIT_CLASS_1B58_TERNARY = 1,     /* 1.58 bits/lane: FlowBmfTernary64 {-1, 0, +1} */
    UNIT_BIT_CLASS_4B_BCD = 2,           /* 4.00 bits/char: numeric/telemetry BCD nibbles */
    UNIT_BIT_CLASS_7B_ASCII = 3,         /* 7.00 bits/char: pure ASCII prefix-stripped */
    UNIT_BIT_CLASS_UTF8_PREFIX_STRIP = 4 /* 3..7 bits/byte: general UTF-8 prefix-stripped */
} UnitBitClass;

/*
 * 64-Byte Wave64 UTF-8 Control Signal Descriptor:
 * Captures the complete structural prefix topology of up to 64 UTF-8 bytes in
 * 64-bit register masks so the data plane stores ONLY raw payload bits.
 */
typedef struct {
    uint64_t ascii_mask;     /* bit i = 1 iff byte i is 0xxxxxxx (7 payload bits) */
    uint64_t cont_mask;      /* bit i = 1 iff byte i is 10xxxxxx (6 payload bits) */
    uint64_t lead2_mask;     /* bit i = 1 iff byte i is 110xxxxx (5 payload bits) */
    uint64_t lead3_mask;     /* bit i = 1 iff byte i is 1110xxxx (4 payload bits) */
    uint64_t lead4_mask;     /* bit i = 1 iff byte i is 11110xxx (3 payload bits) */
    uint64_t sync_mask;      /* bit i = 1 iff byte i starts a Unicode codepoint (~cont_mask) */
    uint64_t digit_mask;     /* bit i = 1 iff byte i is ASCII '0'..'9' */
    uint16_t min_bit_len;    /* Exact stripped payload bit count in [0, 512] */
    uint8_t byte_len;        /* Active UTF-8 bytes in [0, 64] */
    uint8_t codepoint_count; /* Exact Unicode scalar count = POPCNT(sync_mask) in [0, 64] */
    uint8_t bit_class;       /* Selected UnitBitClass for minimal-bit storage */
    uint8_t is_valid_utf8;   /* 1 iff well-formed UTF-8 with 0 overlong/surrogate */
    uint16_t _pad;
} UnitUtf8ControlWave64;

_Static_assert(sizeof(UnitUtf8ControlWave64) == 64U,
               "UnitUtf8ControlWave64 must fit in a single 64-byte cache line");

/*
 * Compact Minimal-Bit Packet Block (Control Signal + Stripped Minimal-Bit Payload):
 * A 64-byte UTF-8 wave is compressed into `control` (64B) + `bits[8]` (64B = 512 bits:
 * up to 448 bits for 64 ASCII chars, 256 bits for 64 BCD digits, 128 bits for 64 ternary
 * lanes, or 512 bits for 64 lossless raw passthrough bytes).
 */
typedef struct {
    UnitUtf8ControlWave64 control;
    uint64_t bits[8]; /* 512 bits (64 bytes) holds up to 64 lossless raw or stripped bytes */
} UnitUtf8MinBitWave64;

_Static_assert(sizeof(UnitUtf8MinBitWave64) == 128U,
               "UnitUtf8MinBitWave64 must occupy exactly 2 cache lines (128 bytes)");

/* ============================================================================
 * 1. Branchless 64-Bit SWAR Byte Classification Primitives
 * ============================================================================ */

#define UNIT_SWAR_ONES_U64  UINT64_C(0x0101010101010101)
#define UNIT_SWAR_HIGHS_U64 UINT64_C(0x8080808080808080)

/*
 * Pack the MSB (bit 7) of each of the 8 bytes in `w` into an 8-bit mask in [0, 255].
 * Uses single multiplication + shift (branchless SWAR movemask).
 */
static inline uint8_t unit_swar_movemask8(uint64_t high_bits_word) {
    uint64_t msbs = (high_bits_word >> 7U) & UNIT_SWAR_ONES_U64;
    /* Multiply maps byte k (bit 8k) -> bit (56 + k), then >> 56 places byte k at bit k */
    return (uint8_t)((msbs * UINT64_C(0x0102040810204080)) >> 56U);
}

/*
 * Branchless 8-byte ASCII check: returns 8-bit mask where bit k = 1 iff byte k < 0x80.
 */
static inline uint8_t unit_swar_ascii_mask8(uint64_t word) {
    uint64_t inv_high = (~word) & UNIT_SWAR_HIGHS_U64;
    return unit_swar_movemask8(inv_high);
}

/*
 * Branchless 8-byte decimal digit check ('0'..'9', 0x30..0x39):
 * Returns 8-bit mask where bit k = 1 iff byte k is in ['0', '9'].
 * Masks low 7 bits per lane first to guarantee 0 inter-lane carry/borrow propagation.
 */
static inline uint8_t unit_swar_digit_mask8(uint64_t word) {
    uint64_t not_high = ~word & UNIT_SWAR_HIGHS_U64;
    uint64_t low7 = word & UINT64_C(0x7F7F7F7F7F7F7F7F);
    /* Add (0x7F - '9') = 0x46: max per lane is 0x7F + 0x46 = 0xC5 < 0x100 (0 carry) */
    uint64_t le_nine = ~(low7 + UINT64_C(0x4646464646464646)) & UNIT_SWAR_HIGHS_U64;
    /* Pre-set bit 7 before subtracting '0' (0x30): min is 0x80 - 0x30 = 0x50 >= 0 (0 borrow) */
    uint64_t ge_zero =
        ((low7 | UNIT_SWAR_HIGHS_U64) - UINT64_C(0x3030303030303030)) & UNIT_SWAR_HIGHS_U64;
    return unit_swar_movemask8(not_high & le_nine & ge_zero);
}

/*
 * Branchless 8-byte continuation check (`(b & 0xC0) == 0x80`):
 * Bit 7 == 1 and Bit 6 == 0.
 */
static inline uint8_t unit_swar_cont_mask8(uint64_t word) {
    uint64_t bit7_set = word & UNIT_SWAR_HIGHS_U64;
    uint64_t bit6_clr = (~(word << 1U)) & UNIT_SWAR_HIGHS_U64;
    return unit_swar_movemask8(bit7_set & bit6_clr);
}

/*
 * Pack 8 ASCII bytes (0xxxxxxx) in a 64-bit little-endian word into 56 minimal bits
 * in the low 56 bits of a uint64_t (0 branches, O(log2(8)) parallel bit gather).
 */
static inline uint64_t unit_swar_pack_7bit_u64(uint64_t ascii_word) {
    uint64_t w = ascii_word & UINT64_C(0x7F7F7F7F7F7F7F7F);
    /* Merge pairs of 7-bit bytes into 14-bit chunks per 16-bit lane */
    w = (w & UINT64_C(0x007F007F007F007F)) | ((w & UINT64_C(0x7F007F007F007F00)) >> 1U);
    /* Merge pairs of 14-bit chunks into 28-bit chunks per 32-bit lane */
    w = (w & UINT64_C(0x00003FFF00003FFF)) | ((w & UINT64_C(0x3FFF00003FFF0000)) >> 2U);
    /* Merge two 28-bit chunks into a contiguous 56-bit integer */
    w = (w & UINT64_C(0x000000000FFFFFFF)) | ((w & UINT64_C(0x0FFFFFFF00000000)) >> 4U);
    return w;
}

/*
 * Unpack 56 minimal bits back into 8 canonical 8-bit ASCII bytes (0 branches).
 */
static inline uint64_t unit_swar_unpack_7bit_u64(uint64_t packed56) {
    uint64_t w = packed56 & UINT64_C(0x00FFFFFFFFFFFFFF);
    w = (w & UINT64_C(0x000000000FFFFFFF)) | ((w & UINT64_C(0x00FFFFFFF0000000)) << 4U);
    w = (w & UINT64_C(0x00003FFF00003FFF)) | ((w & UINT64_C(0x0FFFC0000FFFC000)) << 2U);
    w = (w & UINT64_C(0x007F007F007F007F)) | ((w & UINT64_C(0x3F803F803F803F80)) << 1U);
    return w;
}

/*
 * Pack 8 ASCII decimal digits ('0'..'9') in a 64-bit word into a 32-bit BCD nibble word
 * (4 bits per digit, 50% bit reduction, 0 branches).
 */
static inline uint32_t unit_swar_pack_4bit_bcd_u64(uint64_t digit_word) {
    uint64_t w = digit_word & UINT64_C(0x0F0F0F0F0F0F0F0F);
    w = (w | (w >> 4U)) & UINT64_C(0x00FF00FF00FF00FF);
    w = (w | (w >> 8U)) & UINT64_C(0x0000FFFF0000FFFF);
    w = (w | (w >> 16U)) & UINT64_C(0x00000000FFFFFFFF);
    return (uint32_t)w;
}

/*
 * Unpack a 32-bit BCD nibble word back into 8 ASCII decimal digits ('0'..'9').
 */
static inline uint64_t unit_swar_unpack_4bit_bcd_u64(uint32_t bcd32) {
    uint64_t w = (uint64_t)bcd32;
    w = (w | (w << 16U)) & UINT64_C(0x0000FFFF0000FFFF);
    w = (w | (w << 8U)) & UINT64_C(0x00FF00FF00FF00FF);
    w = (w | (w << 4U)) & UINT64_C(0x0F0F0F0F0F0F0F0F);
    return w | UINT64_C(0x3030303030303030);
}

/*
 * Evaluate a 32-bit 4-bit BCD word (8 packed decimal nibbles, nibble 0 = most significant
 * digit d0, ..., nibble 7 = least significant digit d7) directly into a binary uint32_t
 * WITHOUT unpacking back to 8-bit ASCII (0 branches, 3 parallel SWAR reductions).
 */
static inline uint32_t unit_swar_eval_bcd32(uint32_t bcd32) {
    uint32_t even_nibbles = bcd32 & UINT32_C(0x0F0F0F0F);
    uint32_t odd_nibbles = (bcd32 >> 4U) & UINT32_C(0x0F0F0F0F);
    uint32_t pairs = even_nibbles * 10U + odd_nibbles;
    uint32_t quads =
        ((pairs & UINT32_C(0x00FF00FF)) * 100U) + ((pairs >> 8U) & UINT32_C(0x00FF00FF));
    return (quads & UINT32_C(0x0000FFFF)) * 10000U + (quads >> 16U);
}

/*
 * Branchless SWAR 8-digit ASCII decimal parser:
 * Converts 8 ASCII digits (little-endian `uint64_t`, e.g. memcpy of "12345678")
 * into `uint32_t` integer 12345678 in 3 multiplications (0 branches, 0 loops).
 */
static inline uint32_t unit_swar_parse_8digits_u64(uint64_t ascii_digits) {
    uint64_t val = ascii_digits - UINT64_C(0x3030303030303030);
    /* Combine adjacent digit pairs: d0*10 + d1 */
    val = (val * 10U) + (val >> 8U);
    val &= UINT64_C(0x00FF00FF00FF00FF);
    /* Combine 2-digit pairs into 4-digit groups: p0*100 + p1 */
    val = (val * UINT64_C(100)) + (val >> 16U);
    val &= UINT64_C(0x0000FFFF0000FFFF);
    /* Combine 4-digit groups into 8-digit integer: g0*10000 + g1 */
    val = (val * UINT64_C(10000)) + (val >> 32U);
    return (uint32_t)(val & UINT64_C(0xFFFFFFFF));
}

/*
 * Branchless Single-Pass 8-Byte Wire Token -> Signed Q48.16 Fixed-Point Extractor:
 * Accepts ANY 8-byte ASCII numeric token with optional leading '+'/'-' and optional
 * decimal point '.' at any byte position (e.g. "00000042", "64.50000", "+0042.25",
 * "-1.23456"). Executes in O(1) SWAR register algebra without loops, updating
 * `*inout_sieve` if any byte is malformed, with 0-ULP exact agreement with
 * `unit_utf8_parse_q16` via Granlund-Montgomery Q63 ceil reciprocals.
 */
static inline int64_t unit_swar_wire8_to_q16_branchless(uint64_t word8, uint64_t *inout_sieve) {
    /*
     * Exact Q64 ceil reciprocal multipliers M_k = ceil(2^64 / 10^k) for k=1..7 and
     * half-divisors floor(10^k / 2): ((u128)X * M_k) >> 64 compiles to a single hardware
     * `umulh` instruction and equals X / 10^k with 0-ULP error across all 8-byte tokens.
     */
    static const uint64_t q64_rcp_pow10[8] = {
        0ULL,                          /* k=0: unused (frac_digits == 0 uses x_num) */
        UINT64_C(1844674407370955162), /* k=1: ceil(2^64 / 10)       */
        UINT64_C(184467440737095517),  /* k=2: ceil(2^64 / 100)      */
        UINT64_C(18446744073709552),   /* k=3: ceil(2^64 / 1000)     */
        UINT64_C(1844674407370956),    /* k=4: ceil(2^64 / 10000)    */
        UINT64_C(184467440737096),     /* k=5: ceil(2^64 / 100000)   */
        UINT64_C(18446744073710),      /* k=6: ceil(2^64 / 1000000)  */
        UINT64_C(1844674407371)        /* k=7: ceil(2^64 / 10000000) */
    };
    static const uint64_t half_pow10[8] = {0ULL,    5ULL,     50ULL,     500ULL,
                                           5000ULL, 50000ULL, 500000ULL, 5000000ULL};

    uint8_t b0 = (uint8_t)(word8 & 0xFFU);
    uint64_t has_sign = (uint64_t)((b0 == '+') | (b0 == '-'));
    uint64_t is_neg = (uint64_t)(b0 == '-');

    /* Replace leading sign at byte 0 with '0' (0x30) if present */
    uint64_t clean_w = has_sign ? ((word8 & ~UINT64_C(0xFF)) | UINT64_C(0x30)) : word8;

    /* SWAR parallel detection of '.' (0x2E) across all 8 bytes */
    uint64_t xor_dot = clean_w ^ UINT64_C(0x2E2E2E2E2E2E2E2E);
    uint64_t dot_high = (xor_dot - UNIT_SWAR_ONES_U64) & (~xor_dot) & UNIT_SWAR_HIGHS_U64;

    uint64_t has_one_dot = (uint64_t)(dot_high != 0ULL && ((dot_high & (dot_high - 1ULL)) == 0ULL));
    uint64_t multi_dot = (uint64_t)((dot_high & (dot_high - 1ULL)) != 0ULL);

    /*
     * For single bit dot_high = 2^(8*dot_idx + 7):
     *   up_to_dot = (dot_high - 1) | dot_high has 0xFF in bytes [0 .. dot_idx] and 0x00 above.
     *   ((clean_w << 8) & up_to_dot) shifts bytes [0 .. dot_idx-1] into [1 .. dot_idx] and
     *   zeros byte 0, where '0' (0x30) is inserted; ~up_to_dot preserves bytes [dot_idx+1 .. 7].
     *   clzll(dot_high) = 63 - (8*dot_idx + 7) = 8 * (7 - dot_idx) = 8 * frac_digits.
     */
    uint64_t up_to_dot = (dot_high - 1ULL) | dot_high;
    uint64_t spliced_w = ((clean_w << 8U) & up_to_dot) | UINT64_C(0x30) | (clean_w & ~up_to_dot);
    uint64_t dig_w = has_one_dot ? spliced_w : clean_w;

    uint8_t dmask = unit_swar_digit_mask8(dig_w);
    uint64_t valid = (uint64_t)((dmask == 0xFFU) && (multi_dot == 0ULL));
    if (inout_sieve) {
        uint64_t clear_bits = valid ? 0ULL
                                    : (UNIT_SIEVE_DECIMAL_VALID | UNIT_SIEVE_UTF8_WELL_FORMED |
                                       FLOW_ASM_SIEVE_DOMAIN_VALID);
        *inout_sieve &= ~clear_bits;
    }

    uint64_t raw_int = (uint64_t)unit_swar_parse_8digits_u64(dig_w);
    uint32_t frac_digits = has_one_dot ? ((uint32_t)__builtin_clzll(dot_high | 1ULL) >> 3U) : 0U;
    __extension__ typedef unsigned __int128 u128;
    uint64_t x_num = (raw_int << FLOW_Q16_SHIFT) + half_pow10[frac_digits];
    uint64_t q_div = (uint64_t)(((u128)x_num * (u128)q64_rcp_pow10[frac_digits]) >> 64U);
    int64_t mag_q16 = (frac_digits == 0U) ? (int64_t)x_num : (int64_t)q_div;
    int64_t signed_q16 = is_neg ? -mag_q16 : mag_q16;
    return valid ? signed_q16 : 0LL;
}

/*
 * Fast 64-Bit Hardware-Instruction Integer Square Root (`uint64_t -> uint64_t`):
 * Eliminates `__udivti3` software 128-bit division trap by normalizing `x` into
 * `[2^62, 2^64)`, computing a 17-bit Chebyshev piecewise-linear seed across `[1.0, 4.0)`,
 * and executing a SINGLE hardware `udiv` Newton step + overflow-safe floor fixup.
 * Zero floating-point instructions, exact floor(sqrt(x)) for all x in [0, 2^64 - 1].
 */
static inline uint64_t unit_asm_isqrt_u64_fast(uint64_t x) {
    if (x < 2ULL) {
        return x;
    }
    /*
     * 192-entry packed piecewise-linear Chebyshev seed table covering mantissas in [1.0, 4.0)
     * (low 16 bits: base, high 16 bits: slope). Delivers <= 2^-17 relative seed error so a
     * single Heron step `r = (r + x / r) >> 1` achieves < 0.5 ULP across all 64-bit integers.
     */
    static const uint32_t isqrt_lin_table[192] = {
        0x0FF08000U, 0x0FD180FFU, 0x0FB281FCU, 0x0F9482F7U, 0x0F7783F1U, 0x0F5B84E8U, 0x0F3F85DEU,
        0x0F2386D2U, 0x0F0887C4U, 0x0EEE88B4U, 0x0ED489A3U, 0x0EBB8A90U, 0x0EA28B7CU, 0x0E8A8C66U,
        0x0E728D4FU, 0x0E5B8E36U, 0x0E448F1CU, 0x0E2E9000U, 0x0E1890E3U, 0x0E0291C4U, 0x0DED92A5U,
        0x0DD89383U, 0x0DC39461U, 0x0DAF953DU, 0x0D9B9618U, 0x0D8896F2U, 0x0D7597CAU, 0x0D6298A1U,
        0x0D4F9978U, 0x0D3D9A4CU, 0x0D2B9B20U, 0x0D199BF3U, 0x0D089CC5U, 0x0CF79D95U, 0x0CE69E64U,
        0x0CD59F33U, 0x0CC5A000U, 0x0CB5A0CCU, 0x0CA5A198U, 0x0C95A262U, 0x0C85A32BU, 0x0C76A3F4U,
        0x0C67A4BBU, 0x0C58A581U, 0x0C4AA647U, 0x0C3BA70CU, 0x0C2DA7CFU, 0x0C1FA892U, 0x0C11A954U,
        0x0C04AA15U, 0x0BF6AAD5U, 0x0BE9AB95U, 0x0BDCAC53U, 0x0BCFAD11U, 0x0BC2ADCEU, 0x0BB6AE8AU,
        0x0BA9AF45U, 0x0B9DB000U, 0x0B91B0BAU, 0x0B85B173U, 0x0B79B22BU, 0x0B6DB2E3U, 0x0B61B39AU,
        0x0B56B450U, 0x0B4BB505U, 0x0B3FB5BAU, 0x0B34B66EU, 0x0B2AB721U, 0x0B1FB7D3U, 0x0B14B885U,
        0x0B09B937U, 0x0AFFB9E7U, 0x0AF5BA97U, 0x0AEABB46U, 0x0AE0BBF5U, 0x0AD6BCA3U, 0x0ACCBD51U,
        0x0AC3BDFDU, 0x0AB9BEAAU, 0x0AAFBF55U, 0x0AA6C000U, 0x0A9DC0AAU, 0x0A93C154U, 0x0A8AC1FDU,
        0x0A81C2A6U, 0x0A78C34EU, 0x0A6FC3F6U, 0x0A66C49DU, 0x0A5DC543U, 0x0A55C5E9U, 0x0A4CC68EU,
        0x0A44C733U, 0x0A3BC7D7U, 0x0A33C87BU, 0x0A2BC91EU, 0x0A23C9C1U, 0x0A1BCA63U, 0x0A12CB04U,
        0x0A0BCBA6U, 0x0A03CC46U, 0x09FBCCE6U, 0x09F3CD86U, 0x09EBCE25U, 0x09E4CEC4U, 0x09DCCF62U,
        0x09D5D000U, 0x09CED09DU, 0x09C6D13AU, 0x09BFD1D7U, 0x09B8D272U, 0x09B1D30EU, 0x09A9D3A9U,
        0x09A2D444U, 0x099CD4DEU, 0x0995D577U, 0x098ED611U, 0x0987D6AAU, 0x0980D742U, 0x097AD7DAU,
        0x0973D872U, 0x096CD909U, 0x0966D9A0U, 0x095FDA36U, 0x0959DACCU, 0x0953DB62U, 0x094CDBF7U,
        0x0946DC8BU, 0x0940DD20U, 0x093ADDB4U, 0x0934DE47U, 0x092EDEDBU, 0x0928DF6EU, 0x0922E000U,
        0x091CE092U, 0x0916E124U, 0x0910E1B5U, 0x090AE246U, 0x0904E2D7U, 0x08FFE367U, 0x08F9E3F7U,
        0x08F3E487U, 0x08EEE516U, 0x08E8E5A5U, 0x08E3E633U, 0x08DDE6C1U, 0x08D8E74FU, 0x08D3E7DDU,
        0x08CDE86AU, 0x08C8E8F7U, 0x08C3E983U, 0x08BDEA0FU, 0x08B8EA9BU, 0x08B3EB27U, 0x08AEEBB2U,
        0x08A9EC3DU, 0x08A4ECC7U, 0x089FED52U, 0x089AEDDBU, 0x0895EE65U, 0x0890EEEEU, 0x088BEF77U,
        0x0886F000U, 0x0881F088U, 0x087CF110U, 0x0878F198U, 0x0873F220U, 0x086EF2A7U, 0x086AF32EU,
        0x0865F3B4U, 0x0860F43BU, 0x085CF4C1U, 0x0857F547U, 0x0853F5CCU, 0x084EF651U, 0x084AF6D6U,
        0x0845F75BU, 0x0841F7DFU, 0x083DF863U, 0x0838F8E7U, 0x0834F96AU, 0x0830F9EEU, 0x082BFA71U,
        0x0827FAF3U, 0x0823FB76U, 0x081FFBF8U, 0x081BFC7AU, 0x0816FCFBU, 0x0812FD7DU, 0x080EFDFEU,
        0x080AFE7FU, 0x0806FF00U, 0x0802FF80U};

    uint32_t even_lz = (uint32_t)__builtin_clzll(x) & ~1U;
    uint64_t m = x << even_lz;
    uint32_t entry = isqrt_lin_table[(uint32_t)(m >> 56U) - 64U];
    uint32_t frac16 = (uint32_t)(m >> 40U) & 0xFFFFU;
    uint64_t seed32 = (uint64_t)(((entry & 0xFFFFU) << 4U) + (((entry >> 16U) * frac16) >> 16U))
                      << 12U;
    uint64_t r = (seed32 >> (even_lz >> 1U)) | 1ULL;
    r = (r + x / r) >> 1U;
    if (__builtin_expect(r > UINT32_MAX, 0)) {
        r = UINT32_MAX;
    }
    return r - (uint64_t)((r * r) > x);
}

/*
 * Unified Non-Cancelling 64-Bit Hardware-Register 2-Jet Root Solver:
 * Solves `s_i - r_i * t - 0.5 * c_i * t^2 = 0` (where v = s_i > 0, d1 = -r_i, d2 = -c_i)
 * for the first exit time `t_q16` in (0, t_max_q16]:
 *   - Keeps standard Q16 inputs in [2^12, 2^30) unshifted so `disc` executes the 1-div
 *     `isqrt` fast path directly, while normalizing tiny or huge inputs into `[2^21, 2^22)`
 *     for full precision and zero overflow.
 *   - Uses branchless selection between Citardauq `t = 2*v / (|r_i| + sq)` when `r_i >= 0`
 *     and `t = (|r_i| + sq) / c_i` when `r_i < 0`, guaranteeing zero catastrophic cancellation.
 */
static inline bool unit_jet2_solve_root_fast64(int64_t v, int64_t d1, int64_t d2, int64_t t_max_q16,
                                               int64_t *out_t_q16) {
    if (__builtin_expect(!out_t_q16 || v <= 0 || t_max_q16 <= 0, 0)) {
        return false;
    }
    uint64_t mv = (uint64_t)v;
    uint64_t mr = (d1 < 0) ? (0ULL - (uint64_t)d1) : (uint64_t)d1;
    uint64_t mc = (d2 < 0) ? (0ULL - (uint64_t)d2) : (uint64_t)d2;
    uint64_t or_m = mv | mr | mc;

    int64_t r_i;
    int64_t c_i;
    if (__builtin_expect(or_m < (1ULL << 12U) || or_m >= (1ULL << 30U), 0)) {
        if (d1 == INT64_MIN || d2 == INT64_MIN || flow_hott_is_boundary_tag((uint64_t)v) ||
            flow_hott_is_boundary_tag((uint64_t)d1) || flow_hott_is_boundary_tag((uint64_t)d2)) {
            return false;
        }
        r_i = -d1;
        c_i = -d2;
        if (r_i <= 0 && c_i <= 0) {
            return false;
        }
        uint64_t max_m = (mv > mr) ? mv : mr;
        if (mc > max_m) {
            max_m = mc;
        }
        uint32_t lz = (uint32_t)__builtin_clzll(max_m);
        if (lz < 42U) {
            uint32_t shr = 42U - lz;
            v >>= shr;
            r_i >>= shr;
            c_i >>= shr;
        } else if (lz > 42U) {
            uint32_t shl = lz - 42U;
            v = (int64_t)((uint64_t)v << shl);
            r_i = (int64_t)((uint64_t)r_i << shl);
            c_i = (int64_t)((uint64_t)c_i << shl);
        }
        if (v <= 0 || (r_i <= 0 && c_i <= 0)) {
            return false;
        }
        mr = (uint64_t)(r_i < 0 ? -r_i : r_i);
        mc = (uint64_t)(c_i < 0 ? -c_i : c_i);
    } else {
        r_i = -d1;
        c_i = -d2;
        if (r_i <= 0 && c_i <= 0) {
            return false;
        }
    }

    if (c_i == 0) {
        uint64_t t_u = (((uint64_t)v) << FLOW_Q16_SHIFT) / mr;
        if (t_u > 0ULL && t_u <= (uint64_t)t_max_q16) {
            *out_t_q16 = (int64_t)t_u;
            return true;
        }
        return false;
    }

    /*
     * Exact O(1) Horizon Reachability Sieve (Zero Division):
     * When a tighter horizon `t_max_q16 < 2^30` is already known from an earlier plane:
     *   - If c_i < 0 (concave), f(t) <= r_i * t - v, so r_i * t_max < v << 16 => unreachable.
     *   - If c_i > 0 (convex with f(0) = -v < 0), unique positive root <= t_max iff f(t_max) >= 0.
     */
    if ((uint64_t)t_max_q16 < (1ULL << 30U)) {
        __extension__ typedef __int128 i128;
        i128 target = (i128)v << FLOW_Q16_SHIFT;
        if (c_i < 0) {
            if (r_i <= 0 || (i128)r_i * (i128)t_max_q16 < target) {
                return false;
            }
        } else {
            uint64_t t_ub = (uint64_t)t_max_q16 + 1ULL;
            i128 f_ub = (i128)r_i * (i128)t_ub + (((i128)c_i * (i128)(t_ub * t_ub)) >> 17U);
            if (f_ub < target) {
                return false;
            }
        }
    }

    int64_t disc = r_i * r_i + 2LL * c_i * v;
    if (disc < 0) {
        return false;
    }
    uint64_t sq = unit_asm_isqrt_u64_fast((uint64_t)disc);
    uint64_t q_sum = mr + sq;
    uint64_t num =
        (r_i >= 0) ? (((uint64_t)v) << (FLOW_Q16_SHIFT + 1U)) : (q_sum << FLOW_Q16_SHIFT);
    uint64_t den = (r_i >= 0) ? q_sum : (uint64_t)c_i;
    if (den == 0ULL) {
        return false;
    }
    uint64_t t_u = num / den;
    if (t_u > 0ULL && t_u <= (uint64_t)t_max_q16) {
        *out_t_q16 = (int64_t)t_u;
        return true;
    }
    return false;
}

/* ============================================================================
 * 2. Wave64 UTF-8 Structural Sieve & Minimal-Bit Codec API
 * ============================================================================ */

/*
 * Classify up to 64 bytes of UTF-8 into a `UnitUtf8ControlWave64` control signal
 * descriptor, validating RFC 3629 UTF-8 well-formedness (no overlongs, surrogates,
 * or > U+10FFFF) and updating `inout_sieve`.
 */
UnitUtf8ControlWave64 unit_utf8_classify_wave64(const uint8_t *utf8, size_t len,
                                                uint64_t *inout_sieve);

/*
 * Encode up to 64 bytes of UTF-8 into a `UnitUtf8MinBitWave64` (Control Signal +
 * Stripped Minimal-Bit Payload). Automatically selects the tightest lossless
 * `UnitBitClass` (4-bit BCD if all digits, 7-bit if pure ASCII, or prefix-stripped
 * multibyte bit-packing).
 */
int unit_utf8_encode_minbit_wave64(const uint8_t *utf8, size_t len, UnitUtf8MinBitWave64 *out_wave,
                                   uint64_t *inout_sieve);

/*
 * Encode a 64-lane 1.58-bit balanced ternary vector (`FlowBmfTernary64`, lanes in {-1, 0, +1})
 * into a `UnitUtf8MinBitWave64` under `UNIT_BIT_CLASS_1B58_TERNARY`, verifying canonical
 * encoding (`(sign_mask & ~nonzero_mask) == 0`) via `FLOW_ASM_SIEVE_DOMAIN_VALID`.
 */
int unit_ternary64_encode_minbit_wave64(FlowBmfTernary64 t, uint8_t lane_count,
                                        UnitUtf8MinBitWave64 *out_wave, uint64_t *inout_sieve);

/*
 * Extract a `FlowBmfTernary64` balanced ternary vector from a `UnitUtf8MinBitWave64`
 * encoded with `UNIT_BIT_CLASS_1B58_TERNARY`. Returns active lane count.
 */
size_t unit_ternary64_decode_minbit_wave64(const UnitUtf8MinBitWave64 *wave,
                                           FlowBmfTernary64 *out_t, uint64_t *inout_sieve);

/*
 * Reconstruct exact canonical UTF-8 bytes from `UnitUtf8MinBitWave64` using ONLY
 * the control signal masks + stripped minimal-bit payload. Returns reconstructed byte length.
 */
size_t unit_utf8_decode_minbit_wave64(const UnitUtf8MinBitWave64 *wave,
                                      uint8_t out_utf8[UNIT_WAVE_BYTES], uint64_t *inout_sieve);

/*
 * Decode `UnitUtf8MinBitWave64` directly into 21-bit Unicode scalar values (`uint32_t`)
 * without re-generating UTF-8 intermediate bytes. Returns number of codepoints decoded.
 */
size_t unit_utf8_extract_codepoints_wave64(const UnitUtf8MinBitWave64 *wave,
                                           uint32_t out_codepoints[UNIT_WAVE_BYTES],
                                           uint64_t *inout_sieve);

/*
 * Project an observable UTF-8 numeric token (integer or decimal fixed-point like
 * "-123.456") directly into a `FLOW v4` Q48.16 integer (`FlowResultU64`) with
 * hardware overflow and domain sieve tracking.
 */
FlowResultU64 unit_utf8_parse_q16(const uint8_t *utf8, size_t len, uint64_t *inout_sieve);

#ifdef __cplusplus
}
#endif

#endif /* UNIT_UTF8_BIT_H */
