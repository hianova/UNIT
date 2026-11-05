#ifndef UNIT_ISOMORPHISM_H
#define UNIT_ISOMORPHISM_H

#include "unit_utf8_bit.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * UNIT v1: Protocol-Body <~> Opcode-Pointer Structural Isomorphism
 * ============================================================================
 * Formalizes the exact compile-time and runtime isomorphism between:
 *
 *   1. Network Packet Perspective : [ Protocol Header ]  x  [ Body Bit-Slice ]
 *   2. CPU Instruction Perspective: [ Operator Opcode ]  x  [ Memory Pointer ]
 *   3. Electrical Current Spec    : [ Gate / Routing  ]  x  [ Minimal-Bit Q  ]
 *
 * By enforcing layout univalence (`FLOW_STATIC_ASSERT_UNIVALENT`) and mapping
 * memory pointers into a Z / 2^k Z masked UMA arena (`and` + `bic`), a network
 * frame header IS already a decoded FLOW instruction, and a packet body offset
 * IS already a safe, fault-free memory operand.
 * ============================================================================
 */

#define UNIT_SYNC_MAGIC_BYTE UINT8_C(0xA5) /* Alternating current sync 10100101b */

/* Unified Protocol Message Type === FLOW Instruction Opcode */
typedef enum {
    UNIT_OP_NOP = 0x00,            /* Idle / zero-current keepalive */
    UNIT_OP_UTF8_MINBIT = 0x01,    /* Ingest UTF-8 -> Minimal-Bit Wave64 in arena */
    UNIT_OP_LOAD_U_EXT = 0x02,     /* Project UTF-8 / MinBit payload -> u_ext[DIM] */
    UNIT_OP_LOAD_B_EXT = 0x03,     /* Project UTF-8 / MinBit payload -> b_ext[PLANES] */
    UNIT_OP_SET_HYSTERESIS = 0x04, /* Configure dual-threshold Schmitt (b_enter/b_exit) */
    UNIT_OP_EVAL_QPOLY = 0x05,     /* Execute FLOW A * u_ext <= b_ext polyhedral sieve */
    UNIT_OP_PREDICT_JET2 = 0x06,   /* Execute FLOW 2-Jet quadratic worldline root solver */
    UNIT_OP_CUBICAL_STEP = 0x07,   /* Execute FLOW 1-bit Kan homotopy neighbor step */
    UNIT_OP_SOLVENCY_Q16 = 0x08    /* Execute FLOW ASM Q48.16 solvency check */
} UnitUnifiedOpcode;

/* Electrical Current / Routing Drive Specification (4-bit hardware power/path tier) */
typedef enum {
    UNIT_CURRENT_ZERO_IDLE = 0x0,  /* Gate closed: zero switching activity */
    UNIT_CURRENT_INLINE_REG = 0x1, /* Payload carried inline in 64-bit register B */
    UNIT_CURRENT_L1_ARENA = 0x2,   /* Payload in L1-resident Z / 2^k Z masked arena */
    UNIT_CURRENT_UMA_SHARED = 0x3, /* Payload in 16KB page-aligned zero-copy UMA pool */
    UNIT_CURRENT_WIRE_STREAM = 0x4 /* External wire / io_uring ring ingest/egress */
} UnitCurrentDriveSpec;

/*
 * 64-Bit Unified Control Word Layout:
 *   Bits [63:56] (8b) : magic_sync    (0xA5 alternating bit sync)
 *   Bits [55:48] (8b) : opcode_proto  (UnitUnifiedOpcode: Protocol ID == CPU Opcode)
 *   Bits [47:44] (4b) : bit_class     (UnitBitClass: 1.58b, 4b BCD, 7b ASCII, UTF-8 strip)
 *   Bits [43:40] (4b) : current_drive (UnitCurrentDriveSpec: register, L1 arena, UMA, wire)
 *   Bits [39:32] (8b) : target_axis   (Dimension / hyperplane index / channel ID)
 *   Bits [31:16] (16b): sieve_status  (10-bit FlowAsmSieveAxis | 6-bit UnitSieveAxis)
 *   Bits [15:0]  (16b): bit_length    (Exact minimal-bit payload length in [0, 65535])
 */
static inline uint64_t unit_control_pack(uint8_t opcode_proto, uint8_t bit_class,
                                         uint8_t current_drive, uint8_t target_axis,
                                         uint16_t sieve_status, uint16_t bit_length) {
    return ((uint64_t)UNIT_SYNC_MAGIC_BYTE << 56U) | ((uint64_t)opcode_proto << 48U) |
           ((uint64_t)(bit_class & 0x0FU) << 44U) | ((uint64_t)(current_drive & 0x0FU) << 40U) |
           ((uint64_t)target_axis << 32U) | ((uint64_t)sieve_status << 16U) | (uint64_t)bit_length;
}

static inline uint8_t unit_control_magic(uint64_t ctrl) {
    return (uint8_t)(ctrl >> 56U);
}
static inline uint8_t unit_control_opcode(uint64_t ctrl) {
    return (uint8_t)((ctrl >> 48U) & 0xFFU);
}
static inline uint8_t unit_control_bit_class(uint64_t ctrl) {
    return (uint8_t)((ctrl >> 44U) & 0x0FU);
}
static inline uint8_t unit_control_current_drive(uint64_t ctrl) {
    return (uint8_t)((ctrl >> 40U) & 0x0FU);
}
static inline uint8_t unit_control_target_axis(uint64_t ctrl) {
    return (uint8_t)((ctrl >> 32U) & 0xFFU);
}
static inline uint16_t unit_control_sieve(uint64_t ctrl) {
    return (uint16_t)((ctrl >> 16U) & 0xFFFFU);
}
static inline uint16_t unit_control_bit_length(uint64_t ctrl) {
    return (uint16_t)(ctrl & 0xFFFFU);
}

static inline uint64_t unit_control_with_sieve(uint64_t ctrl, uint64_t full_sieve) {
    uint64_t mask16 = full_sieve & 0xFFFFULL;
    return (ctrl & ~(UINT64_C(0xFFFF) << 16U)) | (mask16 << 16U);
}

/* ============================================================================
 * 1. Compile-Time Univalence: Network Packet View <~> CPU Instruction View
 * ============================================================================ */

/* Perspective A: Network Wire Packet (Protocol Header + Body Bit-Slice) */
typedef struct {
    uint64_t protocol_header; /* Protocol type, encoding class, sync & QoS/current flags */
    uint64_t body_slice;      /* Arena ring offset (or inline 56-bit/64-bit minimal-bit body) */
} UnitPacketView;

/* Perspective B: CPU / FLOW Instruction (Operator Opcode + Masked Memory Pointer) */
typedef struct {
    uint64_t operator_code;   /* ALU/FLOW opcode, condition sieve & current gate spec */
    uint64_t operand_pointer; /* Z / 2^k Z masked memory index or immediate register operand */
} UnitCpuInstView;

/* Zero-Runtime-Byte HoTT Univalence Proof: Packet View == Instruction View */
FLOW_STATIC_ASSERT_UNIVALENT(UnitPacketView, UnitCpuInstView);

/* Unified 16-Byte (128-Bit) Isomorphic Unit Cell */
typedef union {
    UnitPacketView packet;
    UnitCpuInstView inst;
    uint64_t raw_words[2];
} UnitCell128;

FLOW_STATIC_ASSERT_UNIVALENT(UnitCell128, UnitPacketView);
FLOW_STATIC_ASSERT_UNIVALENT(UnitCell128, UnitCpuInstView);

/* ============================================================================
 * 2. Zero-Cost Isomorphic Transport & Masked Pointer Resolution
 * ============================================================================ */

/*
 * Transport a Network Packet directly into a CPU Instruction (0 instructions,
 * verifying protocol sync magic in branchless bitwise logic).
 */
static inline UnitCpuInstView unit_transport_packet_to_inst(UnitPacketView pkt,
                                                            uint64_t *inout_sieve) {
    UnitCell128 cell;
    cell.packet = pkt;
    bool sync_ok = (unit_control_magic(pkt.protocol_header) == UNIT_SYNC_MAGIC_BYTE);
    if (!sync_ok && inout_sieve) {
        *inout_sieve &= ~(UNIT_SIEVE_PROTOCOL_SYNC | UNIT_SIEVE_ISOMORPHIC_FRAME);
    }
    return cell.inst;
}

/*
 * Transport a CPU Instruction directly into a Network Packet (0 instructions).
 */
static inline UnitPacketView unit_transport_inst_to_packet(UnitCpuInstView inst) {
    UnitCell128 cell;
    cell.inst = inst;
    return cell.packet;
}

/*
 * Resolve an instruction's `operand_pointer` (or packet's `body_slice`) against
 * a power-of-two capacity Arena `base[0 .. cap_mask]` using FLOW's branchless
 * `and` + `bic` masked indexing. Guarantees 0 segmentation faults even on
 * corrupted wire offsets, recording any out-of-bounds upper bits in `inout_sieve`.
 */
static inline uint64_t unit_resolve_arena_index(uint64_t raw_operand_ptr, uint64_t cap_mask,
                                                uint64_t *inout_sieve) {
    uint64_t masked_idx = 0ULL;
    uint64_t oob_bits = 0ULL;
#if !defined(FLOW_FORCE_C17_FALLBACK) && (defined(__aarch64__) || defined(__arm64__))
    __asm__("and %[midx], %[idx], %[mask]\n\t"
            "bic %[oob],  %[idx], %[mask]"
            : [midx] "=&r"(masked_idx), [oob] "=r"(oob_bits)
            : [idx] "r"(raw_operand_ptr), [mask] "r"(cap_mask));
#else
    masked_idx = raw_operand_ptr & cap_mask;
    oob_bits = raw_operand_ptr & ~cap_mask;
#endif
    if (oob_bits != 0ULL && inout_sieve) {
        *inout_sieve &= ~FLOW_ASM_SIEVE_MEM_MASKED_INBOUNDS;
    }
    return masked_idx;
}

/*
 * Construct an inline 7-bit ASCII Packet/Instruction Cell (up to 8 ASCII chars
 * packed into 56 minimal bits inside `body_slice` / `operand_pointer`).
 */
static inline UnitCell128 unit_make_inline_ascii_cell(uint8_t opcode, uint8_t target_axis,
                                                      uint64_t ascii_word8, uint8_t char_count,
                                                      uint64_t *inout_sieve) {
    if ((char_count == 0U || char_count > 8U) && inout_sieve) {
        *inout_sieve &= ~UNIT_SIEVE_BITPACK_INBOUNDS;
    }
    uint8_t asc_mask = unit_swar_ascii_mask8(ascii_word8);
    uint8_t active_mask = (char_count >= 8U) ? 0xFFU : (uint8_t)((1U << char_count) - 1U);
    if ((asc_mask & active_mask) != active_mask && inout_sieve) {
        *inout_sieve &= ~UNIT_SIEVE_UTF8_WELL_FORMED;
    }
    uint64_t packed56 = unit_swar_pack_7bit_u64(ascii_word8);
    uint16_t sieve16 = (uint16_t)((inout_sieve ? *inout_sieve : UNIT_SIEVE_ALL_SOUND) & 0xFFFFULL);
    UnitCell128 cell;
    cell.packet.protocol_header = unit_control_pack(opcode, (uint8_t)UNIT_BIT_CLASS_7B_ASCII,
                                                    (uint8_t)UNIT_CURRENT_INLINE_REG, target_axis,
                                                    sieve16, (uint16_t)((uint16_t)char_count * 7U));
    cell.packet.body_slice = packed56;
    return cell;
}

/*
 * Construct an inline 1.58-bit Balanced Ternary Packet/Instruction Cell (up to 32
 * ternary lanes {-1, 0, +1} packed into `body_slice` as low32 `nonzero_mask` and
 * high32 `sign_mask`, verified against `FLOW_ASM_SIEVE_DOMAIN_VALID`).
 */
static inline UnitCell128 unit_make_inline_ternary32_cell(uint8_t opcode, uint8_t target_axis,
                                                          uint32_t nonzero_mask, uint32_t sign_mask,
                                                          uint64_t *inout_sieve) {
    if ((sign_mask & ~nonzero_mask) != 0U && inout_sieve) {
        *inout_sieve &= ~(FLOW_ASM_SIEVE_DOMAIN_VALID | UNIT_SIEVE_UTF8_MINIMAL_LEN);
    }
    uint32_t canon_sign = sign_mask & nonzero_mask;
    uint16_t sieve16 = (uint16_t)((inout_sieve ? *inout_sieve : UNIT_SIEVE_ALL_SOUND) & 0xFFFFULL);
    UnitCell128 cell;
    cell.packet.protocol_header =
        unit_control_pack(opcode, (uint8_t)UNIT_BIT_CLASS_1B58_TERNARY,
                          (uint8_t)UNIT_CURRENT_INLINE_REG, target_axis, sieve16, 51U);
    cell.packet.body_slice = ((uint64_t)canon_sign << 32U) | (uint64_t)nonzero_mask;
    return cell;
}

/*
 * Construct an inline 4-bit BCD Packet/Instruction Cell (8 ASCII decimal digits
 * packed into 32 minimal bits inside `body_slice`, evaluated directly without
 * ever unpacking back to ASCII).
 */
static inline UnitCell128 unit_make_inline_bcd_cell(uint8_t opcode, uint8_t target_axis,
                                                    uint64_t digit_word8, uint64_t *inout_sieve) {
    uint8_t dmask = unit_swar_digit_mask8(digit_word8);
    if (dmask != 0xFFU && inout_sieve) {
        *inout_sieve &= ~(UNIT_SIEVE_DECIMAL_VALID | UNIT_SIEVE_UTF8_WELL_FORMED);
    }
    uint32_t bcd32 = unit_swar_pack_4bit_bcd_u64(digit_word8);
    uint16_t sieve16 = (uint16_t)((inout_sieve ? *inout_sieve : UNIT_SIEVE_ALL_SOUND) & 0xFFFFULL);
    UnitCell128 cell;
    cell.packet.protocol_header =
        unit_control_pack(opcode, (uint8_t)UNIT_BIT_CLASS_4B_BCD, (uint8_t)UNIT_CURRENT_INLINE_REG,
                          target_axis, sieve16, 32U);
    cell.packet.body_slice = (uint64_t)bcd32;
    return cell;
}

/*
 * Single-Pass Wire-to-Cell Projection:
 * Ingests an 8-byte UTF-8 wire token (`word8`), validates UTF-8/decimal syntax
 * into `inout_sieve`, and projects directly into a ready-to-execute Q48.16
 * register cell in a single branchless pass.
 */
static inline UnitCell128 unit_wire8_single_pass_cell(uint8_t opcode, uint8_t target_axis,
                                                      uint64_t word8, uint64_t *inout_sieve) {
    int64_t q16 = unit_swar_wire8_to_q16_branchless(word8, inout_sieve);
    uint16_t sieve16 = (uint16_t)((inout_sieve ? *inout_sieve : UNIT_SIEVE_ALL_SOUND) & 0xFFFFULL);
    UnitCell128 cell;
    cell.packet.protocol_header =
        unit_control_pack(opcode, (uint8_t)UNIT_BIT_CLASS_RAW_BYTES,
                          (uint8_t)UNIT_CURRENT_INLINE_REG, target_axis, sieve16, 64U);
    cell.packet.body_slice = (uint64_t)q16;
    return cell;
}

#ifdef __cplusplus
}
#endif

#endif /* UNIT_ISOMORPHISM_H */
