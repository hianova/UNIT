#include "unit_test_kit.h"

int main(void) {
    UNIT_TEST_SUITE_BEGIN("UNIT Suite 2: Protocol-Body <~> Opcode-Pointer Structural Isomorphism");

    /* =====================================================================
     * Stage 1: Compile-Time HoTT Univalence & Zero-Copy Transport
     * ===================================================================== */
    UNIT_STAGE_BEGIN(1, "Compile-Time Univalence (Packet View == CPU Instruction View)");
    {
        UNIT_ASSERT_EQ(sizeof(UnitPacketView), 16U);
        UNIT_ASSERT_EQ(sizeof(UnitCpuInstView), 16U);
        UNIT_ASSERT_EQ(sizeof(UnitCell128), 16U);

        uint64_t ctrl = unit_control_pack(
            (uint8_t)UNIT_OP_LOAD_U_EXT, (uint8_t)UNIT_BIT_CLASS_7B_ASCII,
            (uint8_t)UNIT_CURRENT_L1_ARENA, 2U, (uint16_t)(UNIT_SIEVE_ALL_SOUND & 0xFFFFULL), 56U);
        UnitPacketView pkt = {.protocol_header = ctrl, .body_slice = 7ULL};

        uint64_t sieve = UNIT_SIEVE_ALL_SOUND;
        UnitCpuInstView inst = unit_transport_packet_to_inst(pkt, &sieve);
        UNIT_ASSERT_EQ(sieve, UNIT_SIEVE_ALL_SOUND);
        UNIT_ASSERT_EQ(inst.operator_code, pkt.protocol_header);
        UNIT_ASSERT_EQ(inst.operand_pointer, pkt.body_slice);
        UNIT_ASSERT_EQ(unit_control_opcode(inst.operator_code), UNIT_OP_LOAD_U_EXT);
        UNIT_ASSERT_EQ(unit_control_bit_class(inst.operator_code), UNIT_BIT_CLASS_7B_ASCII);
        UNIT_ASSERT_EQ(unit_control_current_drive(inst.operator_code), UNIT_CURRENT_L1_ARENA);
        UNIT_ASSERT_EQ(unit_control_target_axis(inst.operator_code), 2U);
        UNIT_ASSERT_EQ(unit_control_bit_length(inst.operator_code), 56U);
    }

    /* =====================================================================
     * Stage 2: Corrupted Protocol Sync Magic Detection
     * ===================================================================== */
    UNIT_STAGE_BEGIN(2, "Alternating Current Sync Magic (0xA5) & Sieve Rejection");
    {
        /* Corrupt top byte from 0xA5 to 0x00 */
        UnitPacketView bad_pkt = {.protocol_header = 0x0001000000000000ULL, .body_slice = 0ULL};
        uint64_t sieve = UNIT_SIEVE_ALL_SOUND;
        (void)unit_transport_packet_to_inst(bad_pkt, &sieve);
        UNIT_ASSERT_EQ(sieve & UNIT_SIEVE_PROTOCOL_SYNC, 0ULL);
        UNIT_ASSERT_EQ(sieve & UNIT_SIEVE_ISOMORPHIC_FRAME, 0ULL);
    }

    /* =====================================================================
     * Stage 3: Z / 2^k Z Masked Arena Pointer Resolution (Zero Segfault)
     * ===================================================================== */
    UNIT_STAGE_BEGIN(3, "Z / 2^k Z Masked Arena Pointer Resolution (and + bic -> Zero OOB)");
    {
        uint64_t sieve_in = UNIT_SIEVE_ALL_SOUND;
        uint64_t idx_ok = unit_resolve_arena_index(19ULL, UNIT_ARENA_CAP_MASK, &sieve_in);
        UNIT_ASSERT_EQ(idx_ok, 19ULL);
        UNIT_ASSERT_EQ(sieve_in, UNIT_SIEVE_ALL_SOUND);

        /* Massive out-of-bounds pointer from untrusted wire packet wraps safely */
        uint64_t sieve_oob = UNIT_SIEVE_ALL_SOUND;
        uint64_t raw_wild = UINT64_C(0xCAFEBABE00000005);
        uint64_t idx_wrapped = unit_resolve_arena_index(raw_wild, UNIT_ARENA_CAP_MASK, &sieve_oob);
        UNIT_ASSERT_EQ(idx_wrapped, 5ULL);
        UNIT_ASSERT_EQ(sieve_oob & FLOW_ASM_SIEVE_MEM_MASKED_INBOUNDS, 0ULL);
    }

    /* =====================================================================
     * Stage 4: Inline 7-Bit ASCII Register Cell (56-Bit Immediate Body)
     * ===================================================================== */
    UNIT_STAGE_BEGIN(4, "Inline 7-Bit ASCII Cell (Protocol+Body in 2 Registers -> FLOW Exec)");
    {
        const char *inline_val = "64.50000";
        uint64_t word8 = 0ULL;
        memcpy(&word8, inline_val, 8U);

        uint64_t sieve = UNIT_SIEVE_ALL_SOUND;
        UnitCell128 cell =
            unit_make_inline_ascii_cell((uint8_t)UNIT_OP_LOAD_U_EXT, 1U, word8, 8U, &sieve);
        UNIT_ASSERT_EQ(sieve, UNIT_SIEVE_ALL_SOUND);
        UNIT_ASSERT_EQ(unit_control_current_drive(cell.inst.operator_code),
                       UNIT_CURRENT_INLINE_REG);
        UNIT_ASSERT_EQ(unit_control_bit_length(cell.inst.operator_code), 56U);

        UnitFlowEngine eng;
        unit_flow_engine_init(&eng);
        UnitExecOutcome out = unit_flow_exec_cell(&eng, cell);
        UNIT_ASSERT_EQ(out.sieve, UNIT_SIEVE_ALL_SOUND);
        UNIT_ASSERT_EQ(eng.u_ext[1], 64LL * 65536LL + 32768LL);

        /* D10: Inline 1.58-bit balanced ternary cell (7 pos - 2 neg = +5.0 in Q16) */
        uint64_t s_tern = UNIT_SIEVE_ALL_SOUND;
        UnitCell128 tern_cell = unit_make_inline_ternary32_cell((uint8_t)UNIT_OP_LOAD_U_EXT, 2U,
                                                                0x01FFU /* 9 active lanes */,
                                                                0x0003U /* 2 negative */, &s_tern);
        UNIT_ASSERT_EQ(s_tern, UNIT_SIEVE_ALL_SOUND);
        UnitExecOutcome out_tern = unit_flow_exec_cell(&eng, tern_cell);
        UNIT_ASSERT_EQ(out_tern.sieve, UNIT_SIEVE_ALL_SOUND);
        UNIT_ASSERT_EQ(eng.u_ext[2], 5LL * FLOW_Q16_ONE);
    }

    UNIT_TEST_SUITE_END();
}
