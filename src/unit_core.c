#include "unit_flow_exec.h"

#include <string.h>

/* ============================================================================
 * 1. Bitstream Pack / Unpack Helpers for Prefix-Stripped Variable Bit-Widths
 * ============================================================================ */

#define UNIT_PARSE_SOUND_MASK                                                               \
    (UNIT_SIEVE_UTF8_WELL_FORMED | UNIT_SIEVE_DECIMAL_VALID | UNIT_SIEVE_BITPACK_INBOUNDS | \
     FLOW_ASM_SIEVE_DOMAIN_VALID)

static inline void unit_bitbuf_put(uint64_t bits[8], uint32_t *bit_cursor, uint64_t val,
                                   uint32_t width) {
    uint32_t pos = *bit_cursor;
    uint32_t word_idx = pos >> 6U;
    uint32_t bit_off = pos & 63U;
    uint64_t masked = val & ((width >= 64U) ? ~0ULL : ((1ULL << width) - 1ULL));
    if (word_idx < 8U) {
        bits[word_idx] |= (masked << bit_off);
        if (bit_off + width > 64U && (word_idx + 1U) < 8U) {
            bits[word_idx + 1U] |= (masked >> (64U - bit_off));
        }
    }
    *bit_cursor = pos + width;
}

static inline uint64_t unit_bitbuf_get(const uint64_t bits[8], uint32_t *bit_cursor,
                                       uint32_t width) {
    uint32_t pos = *bit_cursor;
    uint32_t word_idx = pos >> 6U;
    uint32_t bit_off = pos & 63U;
    uint64_t val = 0ULL;
    if (word_idx < 8U) {
        val = bits[word_idx] >> bit_off;
        if (bit_off + width > 64U && (word_idx + 1U) < 8U) {
            val |= (bits[word_idx + 1U] << (64U - bit_off));
        }
    }
    *bit_cursor = pos + width;
    return val & ((width >= 64U) ? ~0ULL : ((1ULL << width) - 1ULL));
}

static inline int32_t unit_clamp_i32_sieve(int64_t v, uint64_t *inout_sieve) {
    if (v > (int64_t)INT32_MAX) {
        if (inout_sieve) {
            *inout_sieve &= ~(FLOW_ASM_SIEVE_INT_NO_OVERFLOW | FLOW_ASM_SIEVE_Q_NO_SATURATION);
        }
        return INT32_MAX;
    }
    if (v < (int64_t)INT32_MIN) {
        if (inout_sieve) {
            *inout_sieve &= ~(FLOW_ASM_SIEVE_INT_NO_OVERFLOW | FLOW_ASM_SIEVE_Q_NO_SATURATION);
        }
        return INT32_MIN;
    }
    return (int32_t)v;
}

/* ============================================================================
 * 2. Wave64 UTF-8 Structural Classifier & RFC 3629 Sieve Validator
 * ============================================================================ */

UnitUtf8ControlWave64 unit_utf8_classify_wave64(const uint8_t *utf8, size_t len,
                                                uint64_t *inout_sieve) {
    UnitUtf8ControlWave64 ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    if (!utf8 && len > 0U) {
        if (inout_sieve) {
            *inout_sieve &= ~UNIT_SIEVE_UTF8_WELL_FORMED;
        }
        return ctrl;
    }

    size_t n = (len > UNIT_WAVE_BYTES) ? UNIT_WAVE_BYTES : len;
    ctrl.byte_len = (uint8_t)n;
    uint64_t active_mask = (n >= 64U) ? ~0ULL : ((n == 0U) ? 0ULL : ((1ULL << n) - 1ULL));

    uint8_t padded[UNIT_WAVE_BYTES];
    memset(padded, 0, sizeof(padded));
    if (n > 0U) {
        memcpy(padded, utf8, n);
    }

    /* Pass 1: 64-bit SWAR fast classification across 8 x 8-byte words */
    for (uint32_t w = 0U; w < 8U; ++w) {
        uint64_t word = 0ULL;
        memcpy(&word, &padded[w * 8U], sizeof(word));
        uint64_t asc8 = (uint64_t)unit_swar_ascii_mask8(word);
        uint64_t dig8 = (uint64_t)unit_swar_digit_mask8(word);
        uint64_t cnt8 = (uint64_t)unit_swar_cont_mask8(word);
        ctrl.ascii_mask |= (asc8 << (w * 8U));
        ctrl.digit_mask |= (dig8 << (w * 8U));
        ctrl.cont_mask |= (cnt8 << (w * 8U));
    }
    ctrl.ascii_mask &= active_mask;
    ctrl.digit_mask &= active_mask;
    ctrl.cont_mask &= active_mask;

    /* Fast-path: pure ASCII wave (all active bytes have MSB == 0) */
    if (ctrl.ascii_mask == active_mask) {
        ctrl.sync_mask = active_mask;
        ctrl.codepoint_count = (uint8_t)__builtin_popcountll(active_mask);
        ctrl.is_valid_utf8 = 1U;
        if (n > 0U && ctrl.digit_mask == active_mask) {
            ctrl.bit_class = (uint8_t)UNIT_BIT_CLASS_4B_BCD;
            ctrl.min_bit_len = (uint16_t)(n * 4U);
        } else {
            ctrl.bit_class = (uint8_t)UNIT_BIT_CLASS_7B_ASCII;
            ctrl.min_bit_len = (uint16_t)(n * 7U);
        }
        return ctrl;
    }

    /* Pass 2: Multibyte leader classification & exact RFC 3629 state verification */
    bool well_formed = true;
    bool minimal_len = true;
    size_t i = 0U;
    while (i < n) {
        uint8_t b0 = padded[i];
        if (b0 < 0x80U) {
            i += 1U;
        } else if ((b0 & 0xE0U) == 0xC0U) {
            /* 2-byte sequence: 110xxxxx 10xxxxxx */
            ctrl.lead2_mask |= (1ULL << i);
            if (b0 < 0xC2U) {
                minimal_len = false;
                well_formed = false;
            }
            if (i + 1U >= n || (padded[i + 1U] & 0xC0U) != 0x80U) {
                well_formed = false;
                i += 1U;
            } else {
                i += 2U;
            }
        } else if ((b0 & 0xF0U) == 0xE0U) {
            /* 3-byte sequence: 1110xxxx 10xxxxxx 10xxxxxx */
            ctrl.lead3_mask |= (1ULL << i);
            if (i + 2U >= n || (padded[i + 1U] & 0xC0U) != 0x80U ||
                (padded[i + 2U] & 0xC0U) != 0x80U) {
                well_formed = false;
                i += 1U;
            } else {
                uint8_t b1 = padded[i + 1U];
                if (b0 == 0xE0U && b1 < 0xA0U) {
                    minimal_len = false;
                    well_formed = false;
                }
                if (b0 == 0xEDU && b1 >= 0xA0U) {
                    /* UTF-16 surrogate halves U+D800..U+DFFF are ill-formed in UTF-8 */
                    well_formed = false;
                }
                i += 3U;
            }
        } else if ((b0 & 0xF8U) == 0xF0U) {
            /* 4-byte sequence: 11110xxx 10xxxxxx 10xxxxxx 10xxxxxx */
            ctrl.lead4_mask |= (1ULL << i);
            if (b0 > 0xF4U || i + 3U >= n || (padded[i + 1U] & 0xC0U) != 0x80U ||
                (padded[i + 2U] & 0xC0U) != 0x80U || (padded[i + 3U] & 0xC0U) != 0x80U) {
                well_formed = false;
                i += 1U;
            } else {
                uint8_t b1 = padded[i + 1U];
                if (b0 == 0xF0U && b1 < 0x90U) {
                    minimal_len = false;
                    well_formed = false;
                }
                if (b0 == 0xF4U && b1 >= 0x90U) {
                    /* > U+10FFFF */
                    well_formed = false;
                }
                i += 4U;
            }
        } else {
            /* Unexpected continuation byte (10xxxxxx) or invalid byte 0xF8..0xFF */
            well_formed = false;
            i += 1U;
        }
    }

    if (well_formed) {
        ctrl.sync_mask =
            (ctrl.ascii_mask | ctrl.lead2_mask | ctrl.lead3_mask | ctrl.lead4_mask) & active_mask;
        ctrl.codepoint_count = (uint8_t)__builtin_popcountll(ctrl.sync_mask);
        ctrl.is_valid_utf8 = 1U;
        ctrl.bit_class = (uint8_t)UNIT_BIT_CLASS_UTF8_PREFIX_STRIP;
        uint32_t n_asc = (uint32_t)__builtin_popcountll(ctrl.ascii_mask);
        uint32_t n_cnt = (uint32_t)__builtin_popcountll(ctrl.cont_mask);
        uint32_t n_l2 = (uint32_t)__builtin_popcountll(ctrl.lead2_mask);
        uint32_t n_l3 = (uint32_t)__builtin_popcountll(ctrl.lead3_mask);
        uint32_t n_l4 = (uint32_t)__builtin_popcountll(ctrl.lead4_mask);
        ctrl.min_bit_len = (uint16_t)(n_asc * 7U + n_cnt * 6U + n_l2 * 5U + n_l3 * 4U + n_l4 * 3U);
    } else {
        ctrl.sync_mask = active_mask & ~ctrl.cont_mask;
        ctrl.codepoint_count = 0U;
        ctrl.is_valid_utf8 = 0U;
        ctrl.bit_class = (uint8_t)UNIT_BIT_CLASS_RAW_BYTES;
        ctrl.min_bit_len = (uint16_t)(n * 8U);
        if (inout_sieve) {
            uint64_t clr =
                UNIT_SIEVE_UTF8_WELL_FORMED | (minimal_len ? 0ULL : UNIT_SIEVE_UTF8_MINIMAL_LEN);
            *inout_sieve &= ~clr;
        }
    }

    return ctrl;
}

/* ============================================================================
 * 3. Lossless Minimal-Bit Wave64 Encoder / Decoder / Codepoint Projector
 * ============================================================================ */

int unit_utf8_encode_minbit_wave64(const uint8_t *utf8, size_t len, UnitUtf8MinBitWave64 *out_wave,
                                   uint64_t *inout_sieve) {
    if (!out_wave) {
        if (inout_sieve) {
            *inout_sieve &= ~UNIT_SIEVE_BITPACK_INBOUNDS;
        }
        return -1;
    }
    memset(out_wave, 0, sizeof(*out_wave));
    if (len > UNIT_WAVE_BYTES) {
        if (inout_sieve) {
            *inout_sieve &= ~UNIT_SIEVE_BITPACK_INBOUNDS;
        }
        return -1;
    }
    out_wave->control = unit_utf8_classify_wave64(utf8, len, inout_sieve);
    if (!utf8 && len > 0U) {
        return -1;
    }
    if (len == 0U) {
        return 0;
    }

    uint8_t padded[UNIT_WAVE_BYTES];
    memset(padded, 0, sizeof(padded));
    memcpy(padded, utf8, len);

    const UnitUtf8ControlWave64 *c = &out_wave->control;

    if (c->bit_class == (uint8_t)UNIT_BIT_CLASS_4B_BCD) {
        /* 64-bit SWAR 4-bit BCD packing: 8 ASCII digits -> 32 bits */
        for (uint32_t w = 0U; w < 8U; ++w) {
            uint64_t word = 0ULL;
            memcpy(&word, &padded[w * 8U], sizeof(word));
            uint32_t bcd32 = unit_swar_pack_4bit_bcd_u64(word);
            out_wave->bits[w >> 1U] |= ((uint64_t)bcd32 << ((w & 1U) * 32U));
        }
        return 0;
    }

    if (c->bit_class == (uint8_t)UNIT_BIT_CLASS_7B_ASCII) {
        /* 64-bit SWAR 7-bit ASCII packing: each 8 ASCII bytes -> 56 bits */
        uint32_t bit_cursor = 0U;
        uint32_t full_words = (uint32_t)(len >> 3U);
        for (uint32_t w = 0U; w < full_words; ++w) {
            uint64_t word = 0ULL;
            memcpy(&word, &padded[w * 8U], sizeof(word));
            uint64_t packed56 = unit_swar_pack_7bit_u64(word);
            unit_bitbuf_put(out_wave->bits, &bit_cursor, packed56, 56U);
        }
        for (size_t i = (size_t)full_words * 8U; i < len; ++i) {
            unit_bitbuf_put(out_wave->bits, &bit_cursor, (uint64_t)(padded[i] & 0x7FU), 7U);
        }
        return 0;
    }

    if (c->bit_class == (uint8_t)UNIT_BIT_CLASS_UTF8_PREFIX_STRIP) {
        /* Strip all UTF-8 prefix control bits; store only raw payload bits */
        uint32_t bit_cursor = 0U;
        for (size_t i = 0U; i < len; ++i) {
            uint64_t bit = 1ULL << i;
            uint8_t b = padded[i];
            if (c->ascii_mask & bit) {
                unit_bitbuf_put(out_wave->bits, &bit_cursor, (uint64_t)(b & 0x7FU), 7U);
            } else if (c->cont_mask & bit) {
                unit_bitbuf_put(out_wave->bits, &bit_cursor, (uint64_t)(b & 0x3FU), 6U);
            } else if (c->lead2_mask & bit) {
                unit_bitbuf_put(out_wave->bits, &bit_cursor, (uint64_t)(b & 0x1FU), 5U);
            } else if (c->lead3_mask & bit) {
                unit_bitbuf_put(out_wave->bits, &bit_cursor, (uint64_t)(b & 0x0FU), 4U);
            } else if (c->lead4_mask & bit) {
                unit_bitbuf_put(out_wave->bits, &bit_cursor, (uint64_t)(b & 0x07U), 3U);
            }
        }
        return 0;
    }

    /* Fallback for raw binary / invalid UTF-8: store all up to 64 raw bytes in bits[0..7] */
    uint32_t bit_cursor = 0U;
    for (size_t i = 0U; i < len; ++i) {
        if (bit_cursor + 8U <= 512U) {
            unit_bitbuf_put(out_wave->bits, &bit_cursor, (uint64_t)padded[i], 8U);
        }
    }
    return -1;
}

int unit_ternary64_encode_minbit_wave64(FlowBmfTernary64 t, uint8_t lane_count,
                                        UnitUtf8MinBitWave64 *out_wave, uint64_t *inout_sieve) {
    if (!out_wave) {
        if (inout_sieve) {
            *inout_sieve &= ~UNIT_SIEVE_BITPACK_INBOUNDS;
        }
        return -1;
    }
    memset(out_wave, 0, sizeof(*out_wave));
    if (lane_count > UNIT_WAVE_BYTES) {
        if (inout_sieve) {
            *inout_sieve &= ~UNIT_SIEVE_BITPACK_INBOUNDS;
        }
        return -1;
    }
    if (!flow_bmf_ternary64_is_canonical(t)) {
        if (inout_sieve) {
            *inout_sieve &= ~(FLOW_ASM_SIEVE_DOMAIN_VALID | UNIT_SIEVE_UTF8_MINIMAL_LEN);
        }
    }
    uint64_t lmask =
        (lane_count >= 64U) ? ~0ULL : ((lane_count == 0U) ? 0ULL : ((1ULL << lane_count) - 1ULL));
    FlowBmfTernary64 canon = flow_bmf_ternary64_canonicalize(t);
    canon.nonzero_mask &= lmask;
    canon.sign_mask &= lmask;

    out_wave->control.ascii_mask = lmask;
    out_wave->control.sync_mask = lmask;
    out_wave->control.digit_mask = lmask & ~canon.nonzero_mask;
    out_wave->control.byte_len = lane_count;
    out_wave->control.codepoint_count = lane_count;
    out_wave->control.min_bit_len = (uint16_t)(((uint32_t)lane_count * 1585U + 999U) / 1000U);
    out_wave->control.bit_class = (uint8_t)UNIT_BIT_CLASS_1B58_TERNARY;
    out_wave->control.is_valid_utf8 = 1U;
    out_wave->bits[0] = canon.nonzero_mask;
    out_wave->bits[1] = canon.sign_mask;
    return 0;
}

size_t unit_ternary64_decode_minbit_wave64(const UnitUtf8MinBitWave64 *wave,
                                           FlowBmfTernary64 *out_t, uint64_t *inout_sieve) {
    if (!wave || !out_t) {
        if (inout_sieve) {
            *inout_sieve &= ~UNIT_SIEVE_BITPACK_INBOUNDS;
        }
        return 0U;
    }
    out_t->nonzero_mask = 0ULL;
    out_t->sign_mask = 0ULL;
    if (wave->control.bit_class != (uint8_t)UNIT_BIT_CLASS_1B58_TERNARY ||
        wave->control.byte_len > UNIT_WAVE_BYTES) {
        if (inout_sieve) {
            *inout_sieve &= ~(UNIT_SIEVE_BITPACK_INBOUNDS | FLOW_ASM_SIEVE_DOMAIN_VALID);
        }
        return 0U;
    }
    size_t n = (size_t)wave->control.byte_len;
    uint64_t lmask = (n >= 64U) ? ~0ULL : ((n == 0U) ? 0ULL : ((1ULL << n) - 1ULL));
    FlowBmfTernary64 raw_t = {wave->bits[0] & lmask, wave->bits[1] & lmask};
    if (!flow_bmf_ternary64_is_canonical(raw_t)) {
        if (inout_sieve) {
            *inout_sieve &= ~(FLOW_ASM_SIEVE_DOMAIN_VALID | UNIT_SIEVE_UTF8_MINIMAL_LEN);
        }
    }
    *out_t = flow_bmf_ternary64_canonicalize(raw_t);
    return n;
}

size_t unit_utf8_decode_minbit_wave64(const UnitUtf8MinBitWave64 *wave,
                                      uint8_t out_utf8[UNIT_WAVE_BYTES], uint64_t *inout_sieve) {
    if (!wave || !out_utf8) {
        if (inout_sieve) {
            *inout_sieve &= ~UNIT_SIEVE_BITPACK_INBOUNDS;
        }
        return 0U;
    }
    memset(out_utf8, 0, UNIT_WAVE_BYTES);
    const UnitUtf8ControlWave64 *c = &wave->control;
    size_t len = (c->byte_len > UNIT_WAVE_BYTES) ? UNIT_WAVE_BYTES : (size_t)c->byte_len;
    if (len == 0U) {
        return 0U;
    }

    if (c->bit_class == (uint8_t)UNIT_BIT_CLASS_1B58_TERNARY) {
        FlowBmfTernary64 t = {0ULL, 0ULL};
        size_t n = unit_ternary64_decode_minbit_wave64(wave, &t, inout_sieve);
        for (size_t i = 0U; i < n; ++i) {
            int8_t v = flow_bmf_ternary64_get(t, (uint32_t)i);
            out_utf8[i] = (v > 0) ? (uint8_t)'+' : ((v < 0) ? (uint8_t)'-' : (uint8_t)'0');
        }
        return n;
    }

    if (c->bit_class == (uint8_t)UNIT_BIT_CLASS_4B_BCD) {
        uint8_t tmp[UNIT_WAVE_BYTES];
        for (uint32_t w = 0U; w < 8U; ++w) {
            uint32_t bcd32 = (uint32_t)(wave->bits[w >> 1U] >> ((w & 1U) * 32U));
            uint64_t word = unit_swar_unpack_4bit_bcd_u64(bcd32);
            memcpy(&tmp[w * 8U], &word, sizeof(word));
        }
        memcpy(out_utf8, tmp, len);
        return len;
    }

    if (c->bit_class == (uint8_t)UNIT_BIT_CLASS_7B_ASCII) {
        uint32_t bit_cursor = 0U;
        uint32_t full_words = (uint32_t)(len >> 3U);
        for (uint32_t w = 0U; w < full_words; ++w) {
            uint64_t packed56 = unit_bitbuf_get(wave->bits, &bit_cursor, 56U);
            uint64_t word = unit_swar_unpack_7bit_u64(packed56);
            memcpy(&out_utf8[w * 8U], &word, sizeof(word));
        }
        for (size_t i = (size_t)full_words * 8U; i < len; ++i) {
            out_utf8[i] = (uint8_t)unit_bitbuf_get(wave->bits, &bit_cursor, 7U);
        }
        return len;
    }

    if (c->bit_class == (uint8_t)UNIT_BIT_CLASS_UTF8_PREFIX_STRIP) {
        /* Recombine Control Signal prefix masks + Minimal-Bit payload */
        uint32_t bit_cursor = 0U;
        for (size_t i = 0U; i < len; ++i) {
            uint64_t bit = 1ULL << i;
            if (c->ascii_mask & bit) {
                out_utf8[i] = (uint8_t)unit_bitbuf_get(wave->bits, &bit_cursor, 7U);
            } else if (c->cont_mask & bit) {
                out_utf8[i] =
                    (uint8_t)(0x80U | (uint8_t)unit_bitbuf_get(wave->bits, &bit_cursor, 6U));
            } else if (c->lead2_mask & bit) {
                out_utf8[i] =
                    (uint8_t)(0xC0U | (uint8_t)unit_bitbuf_get(wave->bits, &bit_cursor, 5U));
            } else if (c->lead3_mask & bit) {
                out_utf8[i] =
                    (uint8_t)(0xE0U | (uint8_t)unit_bitbuf_get(wave->bits, &bit_cursor, 4U));
            } else if (c->lead4_mask & bit) {
                out_utf8[i] =
                    (uint8_t)(0xF0U | (uint8_t)unit_bitbuf_get(wave->bits, &bit_cursor, 3U));
            }
        }
        return len;
    }

    if (!c->is_valid_utf8 && inout_sieve) {
        *inout_sieve &= ~UNIT_SIEVE_UTF8_WELL_FORMED;
    }
    uint32_t bit_cursor = 0U;
    for (size_t i = 0U; i < len && bit_cursor + 8U <= 512U; ++i) {
        out_utf8[i] = (uint8_t)unit_bitbuf_get(wave->bits, &bit_cursor, 8U);
    }
    return len;
}

size_t unit_utf8_extract_codepoints_wave64(const UnitUtf8MinBitWave64 *wave,
                                           uint32_t out_codepoints[UNIT_WAVE_BYTES],
                                           uint64_t *inout_sieve) {
    if (!wave || !out_codepoints || !wave->control.is_valid_utf8) {
        if (inout_sieve) {
            *inout_sieve &= ~UNIT_SIEVE_UTF8_WELL_FORMED;
        }
        return 0U;
    }
    const UnitUtf8ControlWave64 *c = &wave->control;
    size_t len = (c->byte_len > UNIT_WAVE_BYTES) ? UNIT_WAVE_BYTES : (size_t)c->byte_len;

    if (c->bit_class == (uint8_t)UNIT_BIT_CLASS_1B58_TERNARY ||
        c->bit_class == (uint8_t)UNIT_BIT_CLASS_4B_BCD ||
        c->bit_class == (uint8_t)UNIT_BIT_CLASS_7B_ASCII) {
        uint8_t decoded[UNIT_WAVE_BYTES];
        size_t n = unit_utf8_decode_minbit_wave64(wave, decoded, inout_sieve);
        for (size_t i = 0U; i < n; ++i) {
            out_codepoints[i] = (uint32_t)decoded[i];
        }
        return n;
    }

    /* Direct minimal-bit to 21-bit Unicode scalar assembly without UTF-8 byte reconstruction */
    uint32_t bit_cursor = 0U;
    size_t cp_idx = 0U;
    size_t i = 0U;
    while (i < len) {
        uint64_t bit = 1ULL << i;
        if (c->ascii_mask & bit) {
            out_codepoints[cp_idx++] = (uint32_t)unit_bitbuf_get(wave->bits, &bit_cursor, 7U);
            i += 1U;
        } else if (c->lead2_mask & bit) {
            uint32_t hi = (uint32_t)unit_bitbuf_get(wave->bits, &bit_cursor, 5U);
            uint32_t c1 = (uint32_t)unit_bitbuf_get(wave->bits, &bit_cursor, 6U);
            out_codepoints[cp_idx++] = (hi << 6U) | c1;
            i += 2U;
        } else if (c->lead3_mask & bit) {
            uint32_t hi = (uint32_t)unit_bitbuf_get(wave->bits, &bit_cursor, 4U);
            uint32_t c1 = (uint32_t)unit_bitbuf_get(wave->bits, &bit_cursor, 6U);
            uint32_t c2 = (uint32_t)unit_bitbuf_get(wave->bits, &bit_cursor, 6U);
            out_codepoints[cp_idx++] = (hi << 12U) | (c1 << 6U) | c2;
            i += 3U;
        } else if (c->lead4_mask & bit) {
            uint32_t hi = (uint32_t)unit_bitbuf_get(wave->bits, &bit_cursor, 3U);
            uint32_t c1 = (uint32_t)unit_bitbuf_get(wave->bits, &bit_cursor, 6U);
            uint32_t c2 = (uint32_t)unit_bitbuf_get(wave->bits, &bit_cursor, 6U);
            uint32_t c3 = (uint32_t)unit_bitbuf_get(wave->bits, &bit_cursor, 6U);
            out_codepoints[cp_idx++] = (hi << 18U) | (c1 << 12U) | (c2 << 6U) | c3;
            i += 4U;
        } else {
            break;
        }
    }
    return cp_idx;
}

/* ============================================================================
 * 4. Branchless SWAR + FLOW Total ASM UTF-8 to Q48.16 Projector
 * ============================================================================ */

FlowResultU64 unit_utf8_parse_q16(const uint8_t *utf8, size_t len, uint64_t *inout_sieve) {
    if (!utf8 || len == 0U) {
        if (inout_sieve) {
            *inout_sieve &= ~(UNIT_SIEVE_DECIMAL_VALID | FLOW_ASM_SIEVE_DOMAIN_VALID);
        }
        return flow_result_u64_boundary(FLOW_HOTT_BOUNDARY_OVERFLOW, 7U,
                                        (uint8_t)FLOW_BOUNDARY_DOMAIN_SINGULARITY, 0U, 1U);
    }

    size_t idx = 0U;
    while (idx < len && (utf8[idx] == ' ' || utf8[idx] == '\t')) {
        idx++;
    }
    bool neg = false;
    if (idx < len && (utf8[idx] == '-' || utf8[idx] == '+')) {
        neg = (utf8[idx] == '-');
        idx++;
    }

    /* Fast-path: exact 8-digit integer via 64-bit SWAR multiplication */
    if (len - idx == 8U) {
        uint64_t word = 0ULL;
        memcpy(&word, &utf8[idx], 8U);
        if (unit_swar_digit_mask8(word) == 0xFFU) {
            int64_t val = (int64_t)unit_swar_parse_8digits_u64(word);
            int64_t q16 = val << FLOW_Q16_SHIFT;
            return flow_result_u64_interior((uint64_t)(neg ? -q16 : q16));
        }
    }

    int64_t int_part = 0;
    bool has_digits = false;
    while (idx < len && utf8[idx] >= '0' && utf8[idx] <= '9') {
        has_digits = true;
        int64_t d = (int64_t)(utf8[idx] - '0');
        FlowResultU64 mul10 = flow_asm_qadd_i64(int_part << 3U, int_part << 1U, inout_sieve);
        FlowResultU64 next = flow_asm_qadd_i64((int64_t)mul10.value, d, inout_sieve);
        if (flow_result_u64_is_boundary(mul10) || flow_result_u64_is_boundary(next) ||
            (int64_t)next.value > (INT64_MAX >> FLOW_Q16_SHIFT)) {
            if (inout_sieve) {
                *inout_sieve &= ~(FLOW_ASM_SIEVE_INT_NO_OVERFLOW | FLOW_ASM_SIEVE_Q_NO_SATURATION |
                                  FLOW_ASM_SIEVE_DOMAIN_VALID);
            }
            return flow_result_u64_boundary(FLOW_HOTT_BOUNDARY_OVERFLOW, 7U,
                                            (uint8_t)FLOW_BOUNDARY_DOMAIN_SINGULARITY, 0U, 2U);
        }
        int_part = (int64_t)next.value;
        idx++;
    }

    int64_t frac_num = 0;
    int64_t frac_den = 1;
    if (idx < len && utf8[idx] == '.') {
        idx++;
        while (idx < len && utf8[idx] >= '0' && utf8[idx] <= '9') {
            has_digits = true;
            if (frac_den < 1000000000LL) {
                frac_num = frac_num * 10LL + (int64_t)(utf8[idx] - '0');
                frac_den *= 10LL;
            }
            idx++;
        }
    }

    while (idx < len && (utf8[idx] == ' ' || utf8[idx] == '\t' || utf8[idx] == '\n' ||
                         utf8[idx] == '\r' || utf8[idx] == '\0')) {
        idx++;
    }

    if (!has_digits || idx != len) {
        if (inout_sieve) {
            *inout_sieve &= ~(UNIT_SIEVE_DECIMAL_VALID | FLOW_ASM_SIEVE_DOMAIN_VALID);
        }
        return flow_result_u64_boundary(FLOW_HOTT_BOUNDARY_OVERFLOW, 7U,
                                        (uint8_t)FLOW_BOUNDARY_DOMAIN_SINGULARITY, 0U, 3U);
    }

    __extension__ typedef __int128 i128;
    i128 frac_q16 = ((i128)frac_num * (i128)FLOW_Q16_ONE + (i128)(frac_den >> 1U)) / (i128)frac_den;
    i128 q16_mag = ((i128)int_part << FLOW_Q16_SHIFT) + frac_q16;
    i128 signed_i128 = neg ? -q16_mag : q16_mag;
    if (signed_i128 > (i128)INT64_MAX || signed_i128 <= (i128)INT64_MIN) {
        if (inout_sieve) {
            *inout_sieve &= ~(FLOW_ASM_SIEVE_INT_NO_OVERFLOW | FLOW_ASM_SIEVE_Q_NO_SATURATION |
                              FLOW_ASM_SIEVE_DOMAIN_VALID);
        }
        return flow_result_u64_boundary(FLOW_HOTT_BOUNDARY_OVERFLOW, 7U,
                                        (uint8_t)FLOW_BOUNDARY_DOMAIN_SINGULARITY, 0U, 2U);
    }
    return flow_result_u64_interior((uint64_t)(int64_t)signed_i128);
}

/* ============================================================================
 * 5. Unified Data-to-FLOW Execution Engine Implementation
 * ============================================================================ */

void unit_flow_engine_init(UnitFlowEngine *eng) {
    if (!eng) {
        return;
    }
    memset(eng, 0, sizeof(*eng));
    UnitCellRing64_init(&eng->ingress_ring);
    UnitFlowPoly4D_init(&eng->qpoly);

    /* Default 4D hypercube box constraints: +/- 100.0 in Q16 on axes 0..3 */
    for (uint32_t d = 0U; d < UNIT_STATE_DIM; ++d) {
        eng->qpoly.A[2U * d][d] = 1;
        eng->qpoly.A[2U * d + 1U][d] = -1;
        atomic_store_explicit(&eng->qpoly.b[2U * d], 100LL * FLOW_Q16_ONE, memory_order_relaxed);
        atomic_store_explicit(&eng->qpoly.b[2U * d + 1U], 100LL * FLOW_Q16_ONE,
                              memory_order_relaxed);
        atomic_store_explicit(&eng->qpoly.b_exit[2U * d], 100LL * FLOW_Q16_ONE,
                              memory_order_relaxed);
        atomic_store_explicit(&eng->qpoly.b_exit[2U * d + 1U], 100LL * FLOW_Q16_ONE,
                              memory_order_relaxed);
    }
    atomic_store_explicit(&eng->qpoly.active_mask, 0xFFULL, memory_order_release);
    eng->cumulative_sieve = UNIT_SIEVE_ALL_SOUND;
}

UnitPacketView unit_ingest_utf8_packet(UnitFlowEngine *eng, uint8_t opcode, uint8_t target_axis,
                                       uint64_t raw_slot_idx, const uint8_t *utf8_bytes,
                                       size_t byte_len) {
    UnitPacketView pkt = {0ULL, 0ULL};
    if (!eng) {
        return pkt;
    }
    uint64_t local_sieve = UNIT_SIEVE_ALL_SOUND;
    uint64_t masked_slot =
        unit_resolve_arena_index(raw_slot_idx, UNIT_ARENA_CAP_MASK, &local_sieve);

    UnitUtf8MinBitWave64 *wave = &eng->wave_arena[masked_slot];
    (void)unit_utf8_encode_minbit_wave64(utf8_bytes, byte_len, wave, &local_sieve);

    eng->cumulative_sieve &= local_sieve;
    uint16_t sieve16 = (uint16_t)(local_sieve & 0xFFFFULL);
    pkt.protocol_header =
        unit_control_pack(opcode, wave->control.bit_class, (uint8_t)UNIT_CURRENT_L1_ARENA,
                          target_axis, sieve16, wave->control.min_bit_len);
    pkt.body_slice = raw_slot_idx;
    return pkt;
}

static int64_t unit_extract_cell_q16(UnitFlowEngine *eng, UnitCpuInstView inst,
                                     uint64_t *inout_sieve) {
    uint8_t drive = unit_control_current_drive(inst.operator_code);
    uint8_t bclass = unit_control_bit_class(inst.operator_code);
    uint16_t bit_len = unit_control_bit_length(inst.operator_code);

    if (drive == (uint8_t)UNIT_CURRENT_INLINE_REG) {
        if (bclass == (uint8_t)UNIT_BIT_CLASS_1B58_TERNARY) {
            FlowBmfTernary64 t = {(uint64_t)(uint32_t)inst.operand_pointer,
                                  (uint64_t)(uint32_t)(inst.operand_pointer >> 32U)};
            if (!flow_bmf_ternary64_is_canonical(t) && inout_sieve) {
                *inout_sieve &= ~(FLOW_ASM_SIEVE_DOMAIN_VALID | UNIT_SIEVE_UTF8_MINIMAL_LEN);
            }
            FlowBmfTernary64 ones = {UINT64_C(0xFFFFFFFF), 0ULL};
            int32_t net = flow_bmf_ternary64_dot(t, ones);
            return ((int64_t)net) << FLOW_Q16_SHIFT;
        }
        if (bclass == (uint8_t)UNIT_BIT_CLASS_4B_BCD) {
            uint32_t bcd32 = (uint32_t)inst.operand_pointer;
            uint32_t bad_bcd = bcd32 & ((bcd32 & UINT32_C(0x77777777)) + UINT32_C(0x66666666)) &
                               UINT32_C(0x88888888);
            if ((bad_bcd != 0U || (inst.operand_pointer >> 32U) != 0ULL) && inout_sieve) {
                *inout_sieve &= ~(UNIT_SIEVE_DECIMAL_VALID | FLOW_ASM_SIEVE_DOMAIN_VALID);
            }
            /* Direct 4-bit BCD evaluation without unpacking back to ASCII */
            uint32_t val = unit_swar_eval_bcd32(bcd32);
            return ((int64_t)val) << FLOW_Q16_SHIFT;
        }
        if (bclass == (uint8_t)UNIT_BIT_CLASS_7B_ASCII) {
            if (bit_len == 0U || bit_len > 56U || (bit_len % 7U) != 0U) {
                if (inout_sieve) {
                    *inout_sieve &= ~(UNIT_SIEVE_BITPACK_INBOUNDS | FLOW_ASM_SIEVE_DOMAIN_VALID);
                }
                return 0LL;
            }
            uint64_t word8 = unit_swar_unpack_7bit_u64(inst.operand_pointer);
            if (bit_len == 56U) {
                return unit_swar_wire8_to_q16_branchless(word8, inout_sieve);
            }
            uint8_t chars[8];
            memcpy(chars, &word8, 8U);
            size_t n = (size_t)(bit_len / 7U);
            FlowResultU64 r = unit_utf8_parse_q16(chars, n, inout_sieve);
            return (int64_t)r.value;
        }
        return (int64_t)inst.operand_pointer;
    }

    uint64_t slot =
        unit_resolve_arena_index(inst.operand_pointer, UNIT_ARENA_CAP_MASK, inout_sieve);
    if (bclass == (uint8_t)UNIT_BIT_CLASS_RAW_BYTES) {
        const UnitUtf8MinBitWave64 *raw_wave = &eng->wave_arena[slot];
        if (raw_wave->control.byte_len > 0U && !raw_wave->control.is_valid_utf8) {
            if (inout_sieve) {
                *inout_sieve &= ~(UNIT_SIEVE_UTF8_WELL_FORMED | FLOW_ASM_SIEVE_DOMAIN_VALID);
            }
            return 0LL;
        }
        return (int64_t)flow_asm_arena_load_u64(eng->raw_u64_arena, inst.operand_pointer,
                                                UNIT_ARENA_CAP_MASK, inout_sieve);
    }

    const UnitUtf8MinBitWave64 *wave = &eng->wave_arena[slot];
    if (wave->control.bit_class == (uint8_t)UNIT_BIT_CLASS_1B58_TERNARY ||
        bclass == (uint8_t)UNIT_BIT_CLASS_1B58_TERNARY) {
        FlowBmfTernary64 t = {0ULL, 0ULL};
        size_t n = unit_ternary64_decode_minbit_wave64(wave, &t, inout_sieve);
        uint64_t lmask = (n >= 64U) ? ~0ULL : ((n == 0U) ? 0ULL : ((1ULL << n) - 1ULL));
        FlowBmfTernary64 ones = {lmask, 0ULL};
        int32_t net = flow_bmf_ternary64_dot(t, ones);
        return ((int64_t)net) << FLOW_Q16_SHIFT;
    }

    if (wave->control.bit_class == (uint8_t)UNIT_BIT_CLASS_4B_BCD && wave->control.byte_len == 8U) {
        /* Direct 4-bit BCD arena evaluation without UTF-8 byte reconstruction */
        uint32_t val = unit_swar_eval_bcd32((uint32_t)wave->bits[0]);
        return ((int64_t)val) << FLOW_Q16_SHIFT;
    }

    uint8_t decoded[UNIT_WAVE_BYTES];
    size_t n = unit_utf8_decode_minbit_wave64(wave, decoded, inout_sieve);
    FlowResultU64 r = unit_utf8_parse_q16(decoded, n, inout_sieve);
    return (int64_t)r.value;
}

typedef void (*UnitOpHandlerFn)(UnitFlowEngine *eng, UnitCpuInstView inst, uint8_t axis,
                                UnitExecOutcome *out, uint64_t *local_sieve);

static void unit_op_handle_nop(UnitFlowEngine *eng, UnitCpuInstView inst, uint8_t axis,
                               UnitExecOutcome *out, uint64_t *local_sieve) {
    (void)eng;
    (void)inst;
    (void)axis;
    (void)local_sieve;
    out->current_drive = (uint8_t)UNIT_CURRENT_ZERO_IDLE;
}

static void unit_op_handle_utf8_minbit(UnitFlowEngine *eng, UnitCpuInstView inst, uint8_t axis,
                                       UnitExecOutcome *out, uint64_t *local_sieve) {
    (void)axis;
    uint64_t slot =
        unit_resolve_arena_index(inst.operand_pointer, UNIT_ARENA_CAP_MASK, local_sieve);
    const UnitUtf8MinBitWave64 *w = &eng->wave_arena[slot];
    if (!w->control.is_valid_utf8) {
        *local_sieve &= ~UNIT_SIEVE_UTF8_WELL_FORMED;
    }
    out->primary_value_q16 = (int64_t)w->control.min_bit_len;
}

static void unit_op_handle_load_u_ext(UnitFlowEngine *eng, UnitCpuInstView inst, uint8_t axis,
                                      UnitExecOutcome *out, uint64_t *local_sieve) {
    int64_t val_q16 = unit_extract_cell_q16(eng, inst, local_sieve);
    uint32_t dim = (uint32_t)axis & (UNIT_STATE_DIM - 1U);
    out->focus_plane_or_dim = dim;
    if ((*local_sieve & UNIT_PARSE_SOUND_MASK) == UNIT_PARSE_SOUND_MASK) {
        int32_t prev_u = eng->u_ext[dim];
        int32_t prev_du = eng->du_ext[dim];
        int32_t next_u = unit_clamp_i32_sieve(val_q16, local_sieve);
        FlowResultU64 du_res = flow_asm_qsub_i64((int64_t)next_u, (int64_t)prev_u, local_sieve);
        int32_t next_du = unit_clamp_i32_sieve((int64_t)du_res.value, local_sieve);
        FlowResultU64 ddu_res = flow_asm_qsub_i64((int64_t)next_du, (int64_t)prev_du, local_sieve);
        int32_t next_ddu = unit_clamp_i32_sieve((int64_t)ddu_res.value, local_sieve);
        eng->u_ext[dim] = next_u;
        eng->du_ext[dim] = next_du;
        eng->ddu_ext[dim] = next_ddu;
        out->primary_value_q16 = (int64_t)next_u;
    } else {
        out->primary_value_q16 = (int64_t)eng->u_ext[dim];
    }
}

static void unit_op_handle_load_b_ext(UnitFlowEngine *eng, UnitCpuInstView inst, uint8_t axis,
                                      UnitExecOutcome *out, uint64_t *local_sieve) {
    int64_t bound_q16 = unit_extract_cell_q16(eng, inst, local_sieve);
    uint32_t plane = (uint32_t)axis & 15U;
    out->focus_plane_or_dim = plane;
    if ((*local_sieve & UNIT_PARSE_SOUND_MASK) == UNIT_PARSE_SOUND_MASK) {
        atomic_store_explicit(&eng->qpoly.b[plane], bound_q16, memory_order_relaxed);
        atomic_store_explicit(&eng->qpoly.b_exit[plane], bound_q16, memory_order_relaxed);
        atomic_fetch_or_explicit(&eng->qpoly.active_mask, (1ULL << plane), memory_order_release);
        out->primary_value_q16 = bound_q16;
    } else {
        out->primary_value_q16 = atomic_load_explicit(&eng->qpoly.b[plane], memory_order_relaxed);
    }
}

static void unit_op_handle_set_hysteresis(UnitFlowEngine *eng, UnitCpuInstView inst, uint8_t axis,
                                          UnitExecOutcome *out, uint64_t *local_sieve) {
    uint32_t plane = (uint32_t)axis & 15U;
    int64_t b_enter = atomic_load_explicit(&eng->qpoly.b[plane], memory_order_relaxed);
    int64_t b_exit = unit_extract_cell_q16(eng, inst, local_sieve);
    out->focus_plane_or_dim = plane;
    if ((*local_sieve & UNIT_PARSE_SOUND_MASK) == UNIT_PARSE_SOUND_MASK) {
        UnitFlowPoly4D_set_hysteresis(&eng->qpoly, plane, b_enter, b_exit);
        out->primary_value_q16 = b_exit;
    } else {
        out->primary_value_q16 =
            atomic_load_explicit(&eng->qpoly.b_exit[plane], memory_order_relaxed);
    }
}

static void unit_op_handle_eval_qpoly(UnitFlowEngine *eng, UnitCpuInstView inst, uint8_t axis,
                                      UnitExecOutcome *out, uint64_t *local_sieve) {
    (void)inst;
    (void)axis;
    uint64_t active = atomic_load_explicit(&eng->qpoly.active_mask, memory_order_acquire) &
                      UnitFlowPoly4D_valid_plane_mask();
    uint64_t rem = active;
    while (rem != 0ULL) {
        uint32_t i = (uint32_t)__builtin_ctzll(rem);
        rem &= (rem - 1ULL);
        (void)flow_asm_qdot_i32(eng->qpoly.A[i], eng->u_ext, UNIT_STATE_DIM, local_sieve);
    }
    FlowQPolyEval ev =
        UnitFlowPoly4D_eval_hysteresis(&eng->qpoly, eng->u_ext, eng->prev_violated_mask);
    eng->prev_violated_mask = ev.violated_mask;
    out->violated_mask = ev.violated_mask;
    if (ev.violated_mask != 0ULL) {
        *local_sieve &= ~FLOW_ASM_SIEVE_QPOLY_INTERIOR;
        out->primary_value_q16 = -ev.worst_violation;
        out->focus_plane_or_dim = ev.worst_plane;
    } else {
        out->primary_value_q16 = ev.min_slack;
        out->focus_plane_or_dim = ev.tightest_plane;
    }
}

static void unit_op_handle_predict_jet2(UnitFlowEngine *eng, UnitCpuInstView inst, uint8_t axis,
                                        UnitExecOutcome *out, uint64_t *local_sieve) {
    (void)inst;
    (void)axis;
    uint64_t active = atomic_load_explicit(&eng->qpoly.active_mask, memory_order_acquire) &
                      UnitFlowPoly4D_valid_plane_mask();
    uint64_t use_exit_mask =
        eng->prev_violated_mask
            ? (eng->prev_violated_mask &
               atomic_load_explicit(&eng->qpoly.hysteresis_mask, memory_order_relaxed))
            : 0ULL;
    int64_t b_snap[16];
    for (uint32_t i = 0U; i < 16U; ++i) {
        int64_t b_enter_i = atomic_load_explicit(&eng->qpoly.b[i], memory_order_relaxed);
        int64_t b_exit_i = atomic_load_explicit(&eng->qpoly.b_exit[i], memory_order_relaxed);
        b_snap[i] = flow_asm_select_i64((use_exit_mask >> i) & 1ULL, b_exit_i, b_enter_i);
    }
    uint64_t rem = active;
    while (rem != 0ULL) {
        uint32_t i = (uint32_t)__builtin_ctzll(rem);
        rem &= (rem - 1ULL);
        (void)flow_asm_qdot_i32(eng->qpoly.A[i], eng->u_ext, UNIT_STATE_DIM, local_sieve);
        (void)flow_asm_qdot_i32(eng->qpoly.A[i], eng->du_ext, UNIT_STATE_DIM, local_sieve);
        (void)flow_asm_qdot_i32(eng->qpoly.A[i], eng->ddu_ext, UNIT_STATE_DIM, local_sieve);
    }
    FlowQPolyForecast fc = flow_qpoly_predict_next_face(
        &eng->qpoly.A[0][0], b_snap, active, UNIT_STATE_DIM, eng->u_ext, eng->du_ext, eng->ddu_ext);
    out->will_exit_poly = fc.will_exit ? 1U : 0U;
    out->focus_plane_or_dim = fc.plane;
    out->primary_value_q16 = fc.t_hit_q;
}

static void unit_op_handle_cubical_step(UnitFlowEngine *eng, UnitCpuInstView inst, uint8_t axis,
                                        UnitExecOutcome *out, uint64_t *local_sieve) {
    (void)inst;
    uint64_t next_mask = 0ULL;
    FlowKanStatus st = flow_cubical_step_neighbor(eng->cubical_state_mask, (uint32_t)axis,
                                                  eng->boundary_filter_mask, &next_mask);
    eng->cubical_state_mask = next_mask;
    out->kan_homotopic = (st == FLOW_KAN_FILLED_HOMOTOPIC) ? 1U : 0U;
    out->primary_value_q16 = (int64_t)next_mask;
    if (st != FLOW_KAN_FILLED_HOMOTOPIC) {
        *local_sieve &= ~FLOW_ASM_SIEVE_DOMAIN_VALID;
    }
}

static void unit_op_handle_solvency_q16(UnitFlowEngine *eng, UnitCpuInstView inst, uint8_t axis,
                                        UnitExecOutcome *out, uint64_t *local_sieve) {
    (void)inst;
    (void)axis;
    FlowResultU64 margin =
        flow_asm_solvency_check_q16(eng->solvency_assets_q16, eng->solvency_liabilities_q16,
                                    eng->solvency_buffer_q16, local_sieve);
    out->primary_value_q16 = (int64_t)margin.value;
}

static void unit_op_handle_unmapped(UnitFlowEngine *eng, UnitCpuInstView inst, uint8_t axis,
                                    UnitExecOutcome *out, uint64_t *local_sieve) {
    (void)eng;
    (void)inst;
    (void)axis;
    (void)out;
    *local_sieve &= ~UNIT_SIEVE_PROTOCOL_SYNC;
}

static const UnitOpHandlerFn k_unit_op_lut[16] = {
    unit_op_handle_nop,            /* 0x0: UNIT_OP_NOP            */
    unit_op_handle_utf8_minbit,    /* 0x1: UNIT_OP_UTF8_MINBIT    */
    unit_op_handle_load_u_ext,     /* 0x2: UNIT_OP_LOAD_U_EXT     */
    unit_op_handle_load_b_ext,     /* 0x3: UNIT_OP_LOAD_B_EXT     */
    unit_op_handle_set_hysteresis, /* 0x4: UNIT_OP_SET_HYSTERESIS */
    unit_op_handle_eval_qpoly,     /* 0x5: UNIT_OP_EVAL_QPOLY     */
    unit_op_handle_predict_jet2,   /* 0x6: UNIT_OP_PREDICT_JET2   */
    unit_op_handle_cubical_step,   /* 0x7: UNIT_OP_CUBICAL_STEP   */
    unit_op_handle_solvency_q16,   /* 0x8: UNIT_OP_SOLVENCY_Q16   */
    unit_op_handle_unmapped,       /* 0x9 */
    unit_op_handle_unmapped,       /* 0xA */
    unit_op_handle_unmapped,       /* 0xB */
    unit_op_handle_unmapped,       /* 0xC */
    unit_op_handle_unmapped,       /* 0xD */
    unit_op_handle_unmapped,       /* 0xE */
    unit_op_handle_unmapped        /* 0xF */
};

UnitExecOutcome unit_flow_exec_cell(UnitFlowEngine *eng, UnitCell128 cell) {
    UnitExecOutcome out;
    memset(&out, 0, sizeof(out));
    if (!eng) {
        return out;
    }

    uint64_t local_sieve = UNIT_SIEVE_ALL_SOUND;

    /* 1. Zero-Cycle Univalent Transport: Network Packet View -> CPU Instruction View */
    UnitCpuInstView inst = unit_transport_packet_to_inst(cell.packet, &local_sieve);
    local_sieve &= ((uint64_t)unit_control_sieve(inst.operator_code) | ~0xFFFFULL);

    uint8_t op = unit_control_opcode(inst.operator_code);
    uint8_t drive = unit_control_current_drive(inst.operator_code);
    uint8_t axis = unit_control_target_axis(inst.operator_code);

    out.executed_opcode = op;
    out.current_drive = drive;
    out.focus_plane_or_dim = axis;

    /* 2. O(1) Zero-Switch LUT Dispatch */
    k_unit_op_lut[op & 0x0FU](eng, inst, axis, &out, &local_sieve);

    eng->cumulative_sieve &= local_sieve;
    eng->executed_cells++;
    out.sieve = local_sieve;
    return out;
}

size_t unit_flow_drain_ring(UnitFlowEngine *eng, UnitExecOutcome *last_outcome) {
    if (!eng) {
        return 0U;
    }
    size_t count = 0U;
    UnitCell128 cell;
    while (UnitCellRing64_pop(&eng->ingress_ring, &cell)) {
        UnitExecOutcome o = unit_flow_exec_cell(eng, cell);
        if (last_outcome) {
            *last_outcome = o;
        }
        count++;
    }
    return count;
}

/* ============================================================================
 * 6. Universal Data Layer: `unit_scan` (Direct-on-Minimal-Bit Columnar Scanner)
 * ============================================================================ */

uint8_t unit_scan_bcd_range_wave64(const UnitUtf8MinBitWave64 *wave, uint32_t min_val,
                                   uint32_t max_val, uint64_t *out_sum, uint64_t *inout_sieve) {
    if (out_sum) {
        *out_sum = 0ULL;
    }
    if (!wave || min_val > max_val) {
        if (inout_sieve) {
            *inout_sieve &= ~(UNIT_SIEVE_BITPACK_INBOUNDS | FLOW_ASM_SIEVE_DOMAIN_VALID);
        }
        return 0U;
    }
    if (wave->control.bit_class != (uint8_t)UNIT_BIT_CLASS_4B_BCD ||
        wave->control.byte_len > UNIT_WAVE_BYTES || (wave->control.byte_len & 7U) != 0U) {
        if (inout_sieve) {
            *inout_sieve &= ~(UNIT_SIEVE_DECIMAL_VALID | UNIT_SIEVE_BITPACK_INBOUNDS);
        }
        return 0U;
    }

    uint32_t lanes = (uint32_t)wave->control.byte_len >> 3U;
    uint8_t match_mask = 0U;
    int64_t acc = 0LL;

    for (uint32_t w = 0U; w < lanes; ++w) {
        uint32_t bcd32 = (uint32_t)(wave->bits[w >> 1U] >> ((w & 1U) * 32U));
        uint32_t over9 = (bcd32 + 0x66666666U) ^ bcd32;
        if ((over9 & 0x11111110U) != 0U || (bcd32 >> 28U) > 9U) {
            if (inout_sieve) {
                *inout_sieve &= ~(UNIT_SIEVE_DECIMAL_VALID | FLOW_ASM_SIEVE_DOMAIN_VALID);
            }
            continue;
        }
        uint32_t v = unit_swar_eval_bcd32(bcd32);
        if (v >= min_val && v <= max_val) {
            match_mask |= (uint8_t)(1U << w);
            acc = (int64_t)flow_asm_qadd_i64(acc, (int64_t)v, inout_sieve).value;
        }
    }

    if (out_sum) {
        *out_sum = (uint64_t)acc;
    }
    return match_mask;
}

size_t unit_scan_cells_56bit_match(const UnitCell128 *cells, size_t count, uint8_t required_opcode,
                                   uint64_t target_packed56, uint64_t *out_axis_sum) {
    if (out_axis_sum) {
        *out_axis_sum = 0ULL;
    }
    if (!cells || count == 0U) {
        return 0U;
    }
    size_t matches = 0U;
    uint64_t axis_acc = 0ULL;
    uint64_t target56 = target_packed56 & UINT64_C(0x00FFFFFFFFFFFFFF);

    for (size_t i = 0U; i < count; ++i) {
        uint64_t hdr = cells[i].packet.protocol_header;
        uint64_t body = cells[i].packet.body_slice & UINT64_C(0x00FFFFFFFFFFFFFF);
        if (unit_control_magic(hdr) == UNIT_SYNC_MAGIC_BYTE &&
            unit_control_opcode(hdr) == required_opcode &&
            unit_control_bit_class(hdr) == (uint8_t)UNIT_BIT_CLASS_7B_ASCII && body == target56) {
            matches++;
            axis_acc += (uint64_t)unit_control_target_axis(hdr);
        }
    }

    if (out_axis_sum) {
        *out_axis_sum = axis_acc;
    }
    return matches;
}

size_t unit_scan_cells_q16_range(const UnitCell128 *cells, size_t count, uint8_t target_axis,
                                 int64_t min_q16, int64_t max_q16, int64_t *out_sum_q16,
                                 uint64_t *inout_sieve) {
    if (out_sum_q16) {
        *out_sum_q16 = 0LL;
    }
    if (!cells || min_q16 > max_q16) {
        if (inout_sieve) {
            *inout_sieve &= ~FLOW_ASM_SIEVE_DOMAIN_VALID;
        }
        return 0U;
    }

    size_t matches = 0U;
    int64_t acc_q16 = 0LL;
    uint8_t want_axis = target_axis & 3U;

    for (size_t i = 0U; i < count; ++i) {
        uint64_t hdr = cells[i].inst.operator_code;
        if (unit_control_magic(hdr) != UNIT_SYNC_MAGIC_BYTE) {
            if (inout_sieve) {
                *inout_sieve &= ~UNIT_SIEVE_PROTOCOL_SYNC;
            }
            continue;
        }
        if ((unit_control_target_axis(hdr) & 3U) != want_axis ||
            unit_control_current_drive(hdr) != (uint8_t)UNIT_CURRENT_INLINE_REG) {
            continue;
        }

        uint64_t cell_sieve = UNIT_SIEVE_ALL_SOUND & (uint64_t)unit_control_sieve(hdr);
        int64_t v_q16 = unit_extract_cell_q16(NULL, cells[i].inst, &cell_sieve);
        if ((cell_sieve & UNIT_PARSE_SOUND_MASK) != UNIT_PARSE_SOUND_MASK) {
            if (inout_sieve) {
                *inout_sieve &= cell_sieve;
            }
            continue;
        }
        if (v_q16 >= min_q16 && v_q16 <= max_q16) {
            matches++;
            acc_q16 = (int64_t)flow_asm_qadd_i64(acc_q16, v_q16, inout_sieve).value;
        }
    }

    if (out_sum_q16) {
        *out_sum_q16 = acc_q16;
    }
    return matches;
}

/* ============================================================================
 * 7. Universal Control Layer: `unit_guard` (1.58-Bit Ternary + 2-Jet Guardrail)
 * ============================================================================ */

UnitGuardReport unit_guard_step_ternary_jet2(UnitFlowEngine *eng, uint8_t target_axis,
                                             FlowBmfTernary64 weights, FlowBmfTernary64 activations,
                                             int64_t scale_q16, int64_t horizon_q16) {
    UnitGuardReport rep;
    memset(&rep, 0, sizeof(rep));
    rep.verdict = UNIT_GUARD_SIEVE_REJECT;
    rep.t_hit_q16 = INT64_MAX;
    if (!eng || horizon_q16 < 0) {
        return rep;
    }

    uint64_t local_sieve = UNIT_SIEVE_ALL_SOUND;
    if (!flow_bmf_ternary64_is_canonical(weights) ||
        !flow_bmf_ternary64_is_canonical(activations)) {
        local_sieve &= ~(FLOW_ASM_SIEVE_DOMAIN_VALID | UNIT_SIEVE_UTF8_MINIMAL_LEN);
        eng->cumulative_sieve &= local_sieve;
        rep.sieve = local_sieve;
        return rep;
    }

    /* 1. 64-lane 1.58-bit balanced ternary POPCNT inner product -> Q48.16 */
    int32_t dot_trits = flow_bmf_ternary64_dot(weights, activations);
    __extension__ typedef __int128 i128;
    i128 scaled_i128 = (i128)dot_trits * (i128)scale_q16;
    int64_t score_q16 = flow_q16_clamp_i128(scaled_i128);
    if (scaled_i128 > (i128)INT64_MAX || scaled_i128 < (i128)INT64_MIN) {
        local_sieve &= ~(FLOW_ASM_SIEVE_INT_NO_OVERFLOW | FLOW_ASM_SIEVE_Q_NO_SATURATION);
    }
    rep.ternary_score_q16 = score_q16;

    /* 2. Update 2-Jet fiber (u_ext, du_ext, ddu_ext) on target_axis */
    uint32_t d = (uint32_t)target_axis & (UNIT_STATE_DIM - 1U);
    int32_t prev_u = eng->u_ext[d];
    int32_t prev_du = eng->du_ext[d];
    int64_t sum_u = (int64_t)flow_asm_qadd_i64((int64_t)prev_u, score_q16, &local_sieve).value;
    int32_t next_u = unit_clamp_i32_sieve(sum_u, &local_sieve);
    int64_t raw_du =
        (int64_t)flow_asm_qsub_i64((int64_t)next_u, (int64_t)prev_u, &local_sieve).value;
    int32_t next_du = unit_clamp_i32_sieve(raw_du, &local_sieve);
    int64_t raw_ddu =
        (int64_t)flow_asm_qsub_i64((int64_t)next_du, (int64_t)prev_du, &local_sieve).value;
    int32_t next_ddu = unit_clamp_i32_sieve(raw_ddu, &local_sieve);

    eng->u_ext[d] = next_u;
    eng->du_ext[d] = next_du;
    eng->ddu_ext[d] = next_ddu;

    /* 3. Fused QPoly Schmitt Hysteresis + 2-Jet Quadratic Boundary Lookahead */
    uint64_t active = atomic_load_explicit(&eng->qpoly.active_mask, memory_order_acquire) &
                      UnitFlowPoly4D_valid_plane_mask();
    uint64_t h_mask = atomic_load_explicit(&eng->qpoly.hysteresis_mask, memory_order_relaxed);
    uint64_t use_exit = eng->prev_violated_mask & h_mask;

    uint64_t viol_mask = 0ULL;
    int64_t min_slack = INT64_MAX;
    int64_t best_t = INT64_MAX;
    uint8_t tightest_plane = 0U;
    uint8_t hit_plane = 0U;

    uint64_t rem = active;
    while (rem != 0ULL) {
        uint32_t i = (uint32_t)__builtin_ctzll(rem);
        rem &= (rem - 1ULL);

        int64_t dot_x =
            (int64_t)flow_asm_qdot_i32(eng->qpoly.A[i], eng->u_ext, UNIT_STATE_DIM, &local_sieve)
                .value;
        int64_t b_enter_i = atomic_load_explicit(&eng->qpoly.b[i], memory_order_relaxed);
        int64_t b_exit_i = atomic_load_explicit(&eng->qpoly.b_exit[i], memory_order_relaxed);
        int64_t bound = ((use_exit >> i) & 1ULL) ? b_exit_i : b_enter_i;
        int64_t slack = bound - dot_x;

        if (slack < min_slack) {
            min_slack = slack;
            tightest_plane = (uint8_t)i;
        }

        if (dot_x > bound) {
            viol_mask |= (1ULL << i);
        } else {
            int64_t r_i = (int64_t)flow_asm_qdot_i32(eng->qpoly.A[i], eng->du_ext, UNIT_STATE_DIM,
                                                     &local_sieve)
                              .value;
            int64_t c_i = (int64_t)flow_asm_qdot_i32(eng->qpoly.A[i], eng->ddu_ext, UNIT_STATE_DIM,
                                                     &local_sieve)
                              .value;
            if (r_i > 0 || c_i > 0) {
                int64_t t_cand = 0LL;
                if (unit_jet2_solve_root_fast64(slack, -r_i, -c_i, best_t, &t_cand)) {
                    best_t = t_cand;
                    hit_plane = (uint8_t)i;
                }
            }
        }
    }

    eng->prev_violated_mask = viol_mask;
    rep.violated_mask = viol_mask;
    rep.min_slack_q16 = (min_slack == INT64_MAX) ? 0LL : min_slack;
    rep.t_hit_q16 = best_t;
    rep.next_cubical_mask = eng->cubical_state_mask;

    if (viol_mask != 0ULL) {
        local_sieve &= ~FLOW_ASM_SIEVE_QPOLY_INTERIOR;
        rep.verdict = UNIT_GUARD_HYSTERESIS_HOLD;
        rep.focus_plane = (uint8_t)__builtin_ctzll(viol_mask);
        uint64_t next_mask = 0ULL;
        if (flow_cubical_step_neighbor(eng->cubical_state_mask, (uint32_t)(rep.focus_plane & 63U),
                                       eng->boundary_filter_mask,
                                       &next_mask) == FLOW_KAN_FILLED_HOMOTOPIC) {
            eng->cubical_state_mask = next_mask;
            rep.next_cubical_mask = next_mask;
            rep.kan_routed = 1U;
        }
    } else if (best_t <= horizon_q16) {
        rep.verdict = UNIT_GUARD_TUNNEL_INTERCEPT;
        rep.focus_plane = hit_plane;
        uint64_t next_mask = 0ULL;
        if (flow_cubical_step_neighbor(eng->cubical_state_mask, (uint32_t)(hit_plane & 63U),
                                       eng->boundary_filter_mask,
                                       &next_mask) == FLOW_KAN_FILLED_HOMOTOPIC) {
            eng->cubical_state_mask = next_mask;
            rep.next_cubical_mask = next_mask;
            rep.kan_routed = 1U;
        }
    } else {
        rep.verdict = UNIT_GUARD_PASS;
        rep.focus_plane = tightest_plane;
    }

    eng->cumulative_sieve &= local_sieve;
    eng->executed_cells++;
    rep.sieve = local_sieve;
    return rep;
}
