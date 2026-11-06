#ifndef UNIT_FLOW_EXEC_H
#define UNIT_FLOW_EXEC_H

#include "flow_io.h"
#include "unit_isomorphism.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * UNIT v1: Unified Data-to-FLOW Instruction Execution Engine
 * ============================================================================
 * Connects the observable Data Plane (UTF-8 <-> Minimal-Bit Wave64 + Isomorphic
 * Packet/Instruction Cells) directly down into the FLOW v4 Total ASM Substrate:
 *
 *   [ Observable UTF-8 Packet ]
 *        │  (SWAR Prefix Strip -> Control Signal + Minimal-Bit Payload)
 *        ▼
 *   [ UnitCell128: Protocol+Body === Opcode+Pointer ]
 *        │  (0-Cycle Univalent Transport + Z / 2^k Z Masked Arena Fetch)
 *        ▼
 *   [ FLOW v4 Total Execution Kernel ]
 *        ├─► flow_asm_qadd_i64 / qsub_i64 / qdot_i32 (Hardware NZCV.V Sieve)
 *        ├─► FlowQPoly (A * u_ext <= b_ext + Branchless Schmitt Hysteresis)
 *        ├─► flow_qpoly_predict_next_face (Pure Q16 2-Jet Worldline Root)
 *        └─► flow_cubical_step_neighbor (1-Cycle Kan Homotopy Transition)
 * ============================================================================
 */

#define UNIT_ARENA_POW2_SLOTS 64U
#define UNIT_ARENA_CAP_MASK   (UNIT_ARENA_POW2_SLOTS - 1U)
#define UNIT_STATE_DIM        4U

_Static_assert(UNIT_HW_SIEVE_MEM_MASKED_INBOUNDS == FLOW_ASM_SIEVE_MEM_MASKED_INBOUNDS,
               "UNIT_HW_SIEVE_MEM_MASKED_INBOUNDS must match FLOW_ASM_SIEVE_MEM_MASKED_INBOUNDS");
_Static_assert(UNIT_HW_SIEVE_DOMAIN_VALID == FLOW_ASM_SIEVE_DOMAIN_VALID,
               "UNIT_HW_SIEVE_DOMAIN_VALID must match FLOW_ASM_SIEVE_DOMAIN_VALID");
_Static_assert(UNIT_HW_SIEVE_ALL_SOUND == FLOW_ASM_SIEVE_ALL_SOUND,
               "UNIT_HW_SIEVE_ALL_SOUND must match FLOW_ASM_SIEVE_ALL_SOUND");

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
 * Project an observable UTF-8 numeric token (integer or decimal fixed-point like
 * "-123.456") directly into a `FLOW v4` Q48.16 integer (`FlowResultU64`) with
 * hardware overflow and domain sieve tracking.
 */
FlowResultU64 unit_utf8_parse_q16(const uint8_t *utf8, size_t len, uint64_t *inout_sieve);

/*
 * Define 4D External-Constraint Polyhedron and Lock-Free Cell SPSC Ring:
 * `u_ext` and `b` carry Q48.16 fixed-point values (`1.0 == FLOW_Q16_ONE`), while
 * `A` holds unshifted Q0 integer hyperplane normals (`SHIFT = 0`) so `A * u_ext`
 * and `b` share identical Q48.16 units without redundant scaling.
 */
FLOW_QPOLY_DEFINE_EX(UnitFlowPoly4D, UNIT_STATE_DIM, 0, 16)
FLOW_SPSC_DEFINE(UnitCellRing64, UnitCell128, 64)

typedef struct {
    uint64_t sieve;              /* Combined 16-bit FLOW + UNIT Hardware/Protocol Sieve */
    uint64_t violated_mask;      /* Active hyperplane violation bitmask from FlowQPoly */
    int64_t primary_value_q16;   /* Resulting Q48.16 scalar (slack, root t_hit_q, or margin) */
    uint32_t focus_plane_or_dim; /* Target hyperplane or state axis */
    uint8_t executed_opcode;     /* Opcode executed */
    uint8_t current_drive;       /* Active current drive tier */
    uint8_t will_exit_poly;      /* 1 iff 2-Jet worldline predicts boundary exit */
    uint8_t kan_homotopic;       /* 1 iff Cubical Kan step was filled without obstruction */
} UnitExecOutcome;

typedef struct __attribute__((aligned(FLOW_HW_CACHE_LINE_BYTES))) {
    /* 1. Unified Data Plane: Z / 2^k Z Masked Minimal-Bit Wave64 Arena */
    UnitUtf8MinBitWave64 wave_arena[UNIT_ARENA_POW2_SLOTS];
    uint64_t raw_u64_arena[UNIT_ARENA_POW2_SLOTS];
    UnitCellRing64 ingress_ring;

    /* 2. Observable State Fiber (0-Jet u_ext, 1-Jet du_ext, 2-Jet ddu_ext) */
    int32_t u_ext[UNIT_STATE_DIM];
    int32_t du_ext[UNIT_STATE_DIM];
    int32_t ddu_ext[UNIT_STATE_DIM];

    /* 3. FLOW v4 Bottom-Half Instruction & Constraint Substrate */
    UnitFlowPoly4D qpoly;
    uint64_t prev_violated_mask;
    uint64_t cubical_state_mask;
    uint64_t boundary_filter_mask;
    int64_t solvency_assets_q16;
    int64_t solvency_liabilities_q16;
    int64_t solvency_buffer_q16;
    uint64_t cumulative_sieve;
    uint64_t executed_cells;
} UnitFlowEngine;

/* Initialize the Unified Data + FLOW Execution Engine */
void unit_flow_engine_init(UnitFlowEngine *eng);

/*
 * Store a UTF-8 wire payload into the engine's Z / 2^k Z masked arena at
 * `raw_slot_idx` (automatically masked by `UNIT_ARENA_CAP_MASK`), stripping
 * UTF-8 prefixes into minimal bits and returning an isomorphic `UnitPacketView`
 * ready for wire transmission or immediate CPU execution.
 */
UnitPacketView unit_ingest_utf8_packet(UnitFlowEngine *eng, uint8_t opcode, uint8_t target_axis,
                                       uint64_t raw_slot_idx, const uint8_t *utf8_bytes,
                                       size_t byte_len);

/*
 * Execute a single 128-bit `UnitCell128` (either received as a `UnitPacketView`
 * from the network/ring or issued as a `UnitCpuInstView`), dispatching top-down
 * into the FLOW v4 ASM substrate.
 */
UnitExecOutcome unit_flow_exec_cell(UnitFlowEngine *eng, UnitCell128 cell);

/*
 * Drain and execute all pending cells in `eng->ingress_ring`, returning the
 * number of cells processed.
 */
size_t unit_flow_drain_ring(UnitFlowEngine *eng, UnitExecOutcome *last_outcome);

/* ============================================================================
 * 1. Universal Data Layer: `unit_scan` (Direct-on-Minimal-Bit Columnar Scanner)
 * ============================================================================
 * Evaluates range predicates, exact 7-bit packed symbol filters, and saturated
 * aggregations directly on compressed `UnitUtf8MinBitWave64` and 16-byte
 * `UnitCell128` streams without unpacking to ASCII or wide structs.
 * ============================================================================ */

/*
 * Scan up to eight 8-digit BCD integers (64 ASCII digits packed into 256 bits)
 * inside a `UNIT_BIT_CLASS_4B_BCD` wave directly in registers without unpacking.
 * Returns an 8-bit lane match bitmask (bit w == 1 iff lane w is in [min_val, max_val])
 * and accumulates the matching values into `*out_sum`.
 */
uint8_t unit_scan_bcd_range_wave64(const UnitUtf8MinBitWave64 *wave, uint32_t min_val,
                                   uint32_t max_val, uint64_t *out_sum, uint64_t *inout_sieve);

/*
 * Scan a contiguous array/ring of 16-byte `UnitCell128` cells directly on their
 * 56-bit minimal-bit ASCII payload (`UNIT_BIT_CLASS_7B_ASCII`) and opcode in
 * 2 registers without memory-wall struct unpacking or `memcmp`.
 */
size_t unit_scan_cells_56bit_match(const UnitCell128 *cells, size_t count, uint8_t required_opcode,
                                   uint64_t target_packed56, uint64_t *out_axis_sum);

/*
 * Scan a contiguous stream of inline `UnitCell128` cells on `target_axis`,
 * filtering Q48.16 payloads in `[min_q16, max_q16]` and accumulating the exact
 * overflow-sieved sum into `*out_sum_q16` via `flow_asm_qadd_i64`.
 */
size_t unit_scan_cells_q16_range(const UnitCell128 *cells, size_t count, uint8_t target_axis,
                                 int64_t min_q16, int64_t max_q16, int64_t *out_sum_q16,
                                 uint64_t *inout_sieve);

/* ============================================================================
 * 2. Universal Control Layer: `unit_guard` (1.58-Bit Ternary + 2-Jet Guardrail)
 * ============================================================================
 * Combines 1.58-bit balanced ternary POPCNT inner-product excitation with
 * `FlowQPoly` Schmitt hysteresis and `FlowJet2` quadratic boundary tunneling
 * interception, automatically routing onto a safe cubical neighbor face on
 * imminent boundary exit.
 * ============================================================================ */

typedef enum {
    UNIT_GUARD_PASS = 0,             /* Interior & safe beyond horizon_q16          */
    UNIT_GUARD_HYSTERESIS_HOLD = 1,  /* Latched in Schmitt hysteresis or violated   */
    UNIT_GUARD_TUNNEL_INTERCEPT = 2, /* 2-Jet predicts exit <= horizon -> Kan step  */
    UNIT_GUARD_SIEVE_REJECT = 3      /* Hardware/protocol/canonical sieve rejected  */
} UnitGuardVerdict;

typedef struct __attribute__((aligned(FLOW_HW_CACHE_LINE_BYTES))) {
    int64_t ternary_score_q16;  /* <weights, activations> * scale_q16 via POPCNT  */
    int64_t min_slack_q16;      /* Minimum Q48.16 slack across active hyperplanes */
    int64_t t_hit_q16;          /* Predicted 2-Jet boundary impact time in Q48.16 */
    uint64_t violated_mask;     /* Active hyperplane violation mask               */
    uint64_t next_cubical_mask; /* Resulting cubical state mask after Kan routing */
    uint64_t sieve;             /* Combined hardware + protocol sieve             */
    uint64_t reserved_abi64;    /* Explicit 8B pad for 64-byte cache-line univalence */
    UnitGuardVerdict verdict;   /* 4B guard verdict                               */
    uint8_t focus_plane;        /* Tightest, violated, or first-hit hyperplane    */
    uint8_t kan_routed;         /* 1 iff cubical neighbor transition was taken    */
    uint16_t reserved_flags;    /* Explicit 2B alignment completion (0 holes)     */
} UnitGuardReport;

/*
 * Execute a fused 1.58-Bit Ternary State Update + Polyhedral 2-Jet Guardrail Step:
 *   1. Evaluates 64-lane POPCNT ternary dot product `<weights, activations> * scale_q16`.
 *   2. Updates `(u_ext, du_ext, ddu_ext)` on `target_axis` with 32-bit overflow clamping.
 *   3. Evaluates `UnitFlowPoly4D` Schmitt hysteresis and 64-bit 2-Jet root lookahead.
 *   4. If violated or `0 < t_hit_q16 <= horizon_q16`, triggers a 1-cycle cubical Kan
 *      transition (`flow_cubical_step_neighbor`) along the offending hyperplane axis.
 */
UnitGuardReport unit_guard_step_ternary_jet2(UnitFlowEngine *eng, uint8_t target_axis,
                                             FlowBmfTernary64 weights, FlowBmfTernary64 activations,
                                             int64_t scale_q16, int64_t horizon_q16);

#ifdef __cplusplus
}
#endif

#endif /* UNIT_FLOW_EXEC_H */
