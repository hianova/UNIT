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

typedef struct {
    UnitGuardVerdict verdict;
    int64_t ternary_score_q16;  /* <weights, activations> * scale_q16 via POPCNT  */
    int64_t min_slack_q16;      /* Minimum Q48.16 slack across active hyperplanes */
    int64_t t_hit_q16;          /* Predicted 2-Jet boundary impact time in Q48.16 */
    uint64_t violated_mask;     /* Active hyperplane violation mask               */
    uint64_t next_cubical_mask; /* Resulting cubical state mask after Kan routing */
    uint64_t sieve;             /* Combined hardware + protocol sieve             */
    uint8_t focus_plane;        /* Tightest, violated, or first-hit hyperplane    */
    uint8_t kan_routed;         /* 1 iff cubical neighbor transition was taken    */
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
