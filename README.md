# UNIT v1 — Unified Observable UTF-8 Minimal-Bit × Protocol-Opcode Isomorphic Substrate

> **可觀測資料層統一規格（UTF-8 $\leftrightarrow$ 最小 Bit ＋ 控制訊號電流規格）$\times$ 封包與 CPU 指令完全同構（Protocol & Body $\cong$ Opcode & Pointer）$\times$ 底層直通 [`FLOW v4`](../FLOW) 全函數 ASM 核心。**

`UNIT` 實踐了計算機底層架構「資訊存儲（Data Layer）」與「控制訊號（Control Layer）」的同構收斂：
1. **可觀測資料層（Observable Data Plane）**：外部世界與網路邊界以標準 **UTF-8** 進行觀測與通訊，無需外部社群更改協定。進入記憶體的第一刻，`UNIT` 透過無分支 64-bit SWAR 位元平行代數，將 UTF-8 內建的前綴控制位元（`0`, `10`, `110`, `1110`, `11110`）剝離為 **64-Byte L1 Cache 對齊之控制訊號波塊（[`UnitUtf8ControlWave64`](src/unit_utf8_bit.h)）**，資料平面僅存儲**最小位元載荷（[`UnitUtf8MinBitWave64`](src/unit_utf8_bit.h)：4-bit BCD、7-bit ASCII、1.58-bit 平衡三態 `FlowBmfTernary64`、或剝離前綴之 3..6-bit 緊湊位元流）**。
2. **封包與指令結構同構（Structural Isomorphism）**：以封包視角是 **`[Protocol Header + Body Slice]`**（[`UnitPacketView`](src/unit_isomorphism.h)），以 CPU 視角則是 **`[Operator Opcode + Memory Pointer]`**（[`UnitCpuInstView`](src/unit_isomorphism.h)）。兩者在 16-Byte [`UnitCell128`](src/unit_isomorphism.h) 中通過編譯期 HoTT Univalence 同構證明（`FLOW_STATIC_ASSERT_UNIVALENT`），實現 **0 指令轉型**與 $\mathbb{Z}/2^k\mathbb{Z}$ 遮罩（ARM64 `and` + `bic`）零越界定址。
3. **指令層由 `FLOW v4` 往下貫穿到底層**：統一單元 [`UnitCell128`](src/unit_isomorphism.h) 直接驅動 [`FLOW v4`](../FLOW/src) 的無分支 Q48.16 ASM 算術、多面體外部約束（`FlowQPoly` $A\mathbf{u}_{\text{ext}} \le \mathbf{b}_{\text{ext}}$ + Schmitt 雙閾值防抖遲滯）、2-Jet 二次世界線求根（[`unit_jet2_solve_root_fast64`](src/unit_utf8_bit.h)）與 Cubical Kan 1-Cycle 鄰居步進。
4. **雙通用高階引擎（`unit_scan` ＋ `unit_guard`）**：
   - **`unit_scan`（資料層：最小位元列式與串流無解壓掃描）**：直接在 4-bit BCD 波塊與 16B `UnitCell128` 環形緩衝區上執行範圍過濾、56-bit 壓縮符號匹配與飽和累加，徹底免除解壓縮與 `memcmp` 記憶體牆成本。
   - **`unit_guard`（控制層：1.58-Bit 三態狀態機 ＋ 二階多面體安全護欄）**：以單指令 `POPCNT` 計算 64 通道 `{-1, 0, +1}` 平衡三態內積，融合 `FlowQPoly` 施密特遲滯與 2-Jet 穿隧時間預測，並在預計越界前自動觸發 1-Cycle 立方體 Kan 降維路由。

---

## 1. 架構全景

```
【可觀測邊界 / 網路封包】 (Standard UTF-8 Wire Stream)
       │
       │  1. SWAR 64-Bit 前綴剝離與 RFC 3629 硬體篩網 (src/unit_utf8_bit.h)
       ▼
┌──────────────────────────────────────────────────────────────────────────┐
│ 統一最小位元波塊 UnitUtf8MinBitWave64 (128B = 64B Control + 64B Payload) │
│  • Control Signal : ascii_mask | cont_mask | lead2/3/4_mask | sync_mask  │
│  • Minimal-Bit    : 4-bit BCD / 7-bit ASCII / 1.58-bit Ternary / 512b Raw│
└────────────────────────────────────┬─────────────────────────────────────┘
                                     │
       │  2. 封包 <~> CPU 指令同構映射 (src/unit_isomorphism.h)
       ▼
┌──────────────────────────────────────────────────────────────────────────┐
│ 128-Bit 同構單元 UnitCell128 (UnitPacketView === UnitCpuInstView)        │
│  • Word 0 : [ 0xA5 Sync | Opcode==Proto | BitClass | CurrentDrive | Sieve]│
│  • Word 1 : [ Z / 2^k Z Masked Arena Pointer  OR  56-Bit Inline Payload ]│
└────────────────────────────────────┬─────────────────────────────────────┘
                                     │
       │  3. 通用高階雙引擎與 FLOW v4 總體 ASM 執行層 (src/unit_flow_exec.h)
       ▼
┌──────────────────────────────────────────────────────────────────────────┐
│ • Data Layer    : unit_scan_*  (無解壓 4b BCD 區間過濾 / 16B 56b 符號掃描)│
│ • Control Layer : unit_guard_* (1.58b POPCNT 內積 + Schmitt + 2-Jet 護欄)│
│ • FLOW v4 ASM   : adds/subs/smulh/extr + cset vs -> 16-bit Total Sieve   │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 模組架構（3 Headers + 1 C 檔，零 `double` / 零 `-lm`）

| 檔案 | 職責說明 |
| :--- | :--- |
| [`src/unit_utf8_bit.h`](src/unit_utf8_bit.h) | **SWAR UTF-8 控制訊號篩網與最小 Bit 編解碼規格**：防跨位元組進借位之 `unit_swar_digit_mask8`、`unit_swar_pack_7bit_u64`、`unit_swar_pack_4bit_bcd_u64`、`unit_swar_eval_bcd32`、Granlund-Montgomery Q64 倒數 0-ULP 單次提取器 `unit_swar_wire8_to_q16_branchless`、單次 `udiv` 切比雪夫分段線性精確平方根 `unit_asm_isqrt_u64_fast`、Citardauq 無消去二次求根器 `unit_jet2_solve_root_fast64`、1.58-bit 平衡三態波塊編解碼 `unit_ternary64_encode_minbit_wave64` / `unit_ternary64_decode_minbit_wave64` 與 `unit_utf8_parse_q16` |
| [`src/unit_isomorphism.h`](src/unit_isomorphism.h) | **Protocol-Body $\cong$ Opcode-Pointer 同構定義**：64-bit 控制訊號與電流驅動規格 `unit_control_pack`、`UnitPacketView` $\simeq$ `UnitCpuInstView` Univalence 靜態證明、`unit_resolve_arena_index`（ARM64 `and` + `bic` 零 Segfault 遮罩指標）、內嵌 7-bit ASCII / 4-bit BCD / 1.58-bit Ternary 暫存器單元建構器與單次直通 `unit_wire8_single_pass_cell` |
| [`src/unit_flow_exec.h`](src/unit_flow_exec.h), [`src/unit_core.c`](src/unit_core.c) | **統一資料層直通 FLOW v4 執行引擎與通用高階模組**：`UnitFlowEngine`、`unit_ingest_utf8_packet`、`unit_flow_exec_cell`、`unit_flow_drain_ring`，以及通用掃描器 `unit_scan_bcd_range_wave64` / `unit_scan_cells_56bit_match` / `unit_scan_cells_q16_range` 與通用護欄 `unit_guard_step_ternary_jet2` |
| [`tests/test_utf8_minbit.c`](tests/test_utf8_minbit.c), [`tests/test_isomorphism.c`](tests/test_isomorphism.c), [`tests/test_flow_pipeline.c`](tests/test_flow_pipeline.c), [`tests/bench_falsify.c`](tests/bench_falsify.c) | **15,100+ 斷言之 IEEE 審計回歸與殘酷證偽基準套件（Falsification Suite）**：涵蓋 `D1`–`D10` 全數邊界回歸測試、SWAR 壓縮無損往返、RFC 3629 非法 UTF-8 攔截、直接最小 bit 計算（12x–13x 加速）、16B vs 128B 快取牆掃描（3x–4x 加速）、Wire-to-Decision 端到端吞吐量與 P50/P99.9 尾延遲壓制，以及 IEEE-754 尾數吞噬失效對照 |

---

## 3. IEEE 等級審計與確定性保證（D1–D10 Verified）

1. **D1（全 64-Bit 定義域精確整數平方根 `unit_asm_isqrt_u64_fast`）**：採用涵蓋 $[1.0, 4.0)$ 之 192 項切比雪夫分段線性查表（初始相對誤差 $\le 2^{-17}$），僅需**單次硬體 `udiv` 牛頓步**與防溢位修正即可在 $[0, 2^{64}-1]$ 全域達成 0-ULP 精確 $\lfloor\sqrt{x}\rfloor$，完全消除編譯器 `__udivti3` 軟體模擬陷阱。
2. **D2（SWAR 跨通道進位/借位隔離 `unit_swar_digit_mask8`）**：先遮罩低 7 位元並隔離高元位，徹底杜絕 `< 0x30`（如 `"12.00000"`）對相鄰 `'0'` 的連鎖借位與 $\ge \text{0xBA}$（如 UTF-8 多位元組序列 `"電90123"`）對相鄰 `'9'` 的進位污染。
3. **D3（Citardauq 無相減消去二階求根與凸包可達性剪枝 `unit_jet2_solve_root_fast64`）**：針對微小或超大輸入自動正規化至 $[2^{21}, 2^{22})$，透過 `r_i >= 0` 與 `r_i < 0` 雙代數形式無分支切換消除相減有效位數喪失，並以 `flow_q16_neg_sat` 與 HoTT Boundary Tag 防護杜絕 `INT64_MIN` 未定義行為。
4. **D4–D6 & D9（狀態纖維防溢位夾限、Header Sieve 阻斷與施密特遲滯預測）**：`UNIT_OP_LOAD_U_EXT` 具 32-bit 溢位篩網夾限、格式錯誤封包自動阻斷狀態污染、`UnitFlowPoly4D` 統一 Q48.16 尺度（`SHIFT = 0`）並捕捉 128-bit 內積溢位、`UNIT_OP_PREDICT_JET2` 嚴格遵守施密特遲滯鎖定邊界 `b_exit`。
5. **D7–D8 & D10（64B 快取線對齊、512-Bit 無損載荷、Q64 0-ULP 倒數與 1.58-Bit 三態編解碼）**：`sizeof(UnitUtf8ControlWave64) == 64`、`UnitUtf8MinBitWave64` 具備完整 `bits[8]`（512 bits）無損容量、`unit_swar_wire8_to_q16_branchless` 以單指令 `umulh` Q64 倒數與 `unit_utf8_parse_q16` 達成 0-ULP 完全一致，並原生支援 `UNIT_BIT_CLASS_1B58_TERNARY`。

---

## 4. 快速建置與測試

```bash
make fmt-check
make clean && make all && make test
```
