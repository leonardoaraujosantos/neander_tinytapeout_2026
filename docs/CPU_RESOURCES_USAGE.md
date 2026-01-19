# CPU Resources Usage Analysis

This document analyzes the NEANDER-X CPU features, their estimated tile usage, and risk assessment for removal based on LCC compiler usage patterns.

## LCC Compiler Instruction Usage

Analysis based on all assembly files in `lcc_samples/*.s`:

| Instruction | Usage Count | Category | Required by LCC |
|-------------|-------------|----------|-----------------|
| LDI | 404 | Load/Store | ✅ Critical |
| PUSH | 366 | Stack | ✅ Critical |
| STA | 280 | Load/Store | ✅ Critical |
| LDA | 241 | Load/Store | ✅ Critical |
| CALL | 80 | Control Flow | ✅ Critical |
| PUSH_ADDR | 76 | Stack | ✅ Critical |
| POP_ADDR | 76 | Stack | ✅ Critical |
| ADD | 64 | ALU | ✅ Critical |
| TSF | 60 | Frame Pointer | ✅ Critical |
| TFS | 60 | Frame Pointer | ✅ Critical |
| RET | 60 | Control Flow | ✅ Critical |
| PUSH_FP | 60 | Frame Pointer | ✅ Critical |
| POP_FP | 60 | Frame Pointer | ✅ Critical |
| HLT | 46 | Control Flow | ✅ Critical |
| JMP | 40 | Control Flow | ✅ Critical |
| CMP | 20 | ALU | ✅ Needed |
| SUB | 5 | ALU | ✅ Needed |
| JGE | 5 | Control Flow | ✅ Needed |
| AND | 5 | ALU | ✅ Needed |
| XOR | 4 | ALU | ✅ Needed |
| TAX | 3 | Register Transfer | ✅ Needed (MUL/DIV) |
| JLE | 3 | Control Flow | ✅ Needed |
| JGT | 3 | Control Flow | ✅ Needed |
| NEG | 2 | ALU | ✅ Needed |
| JNZ | 2 | Control Flow | ✅ Needed |
| NOT | 1 | ALU | ✅ Needed |
| MUL | 1 | ALU | ⚠️ Rarely used |
| MOD | 1 | ALU | ⚠️ Rarely used |
| DIV | 1 | ALU | ⚠️ Rarely used |

---

## Feature Categories by Estimated Tile Usage

### Critical Features (Must Keep)

| Feature | Module(s) | Est. Area | Description |
|---------|-----------|-----------|-------------|
| AC Register | `neander_x_datapath.sv` | ~2% | Main accumulator |
| PC Register | `neander_x_datapath.sv` | ~2% | Program counter (16-bit) |
| SP Register | `neander_x_datapath.sv` | ~2% | Stack pointer (16-bit) |
| FP Register | `neander_x_datapath.sv` | ~2% | Frame pointer (16-bit) |
| RDM Register | `neander_x_datapath.sv` | ~2% | Memory data register |
| RI Register | `neander_x_datapath.sv` | ~1% | Instruction register |
| NZC Flags | `neander_x_datapath.sv` | ~1% | Status flags |
| Basic ALU | `neander_x_alu.sv` | ~8% | ADD, SUB, AND, XOR, NOT, NEG |
| SPI Controller | `spi_memory_controller.sv` | ~15% | Memory interface |
| Control Unit Core | `neander_x_control_unit.sv` | ~20% | State machine |

**Subtotal: ~55% (Cannot be reduced)**

---

### High Impact - Removable Features

#### 1. B Register Subsystem (NOT USED BY LCC)

| Feature | Opcodes | Est. Area | Risk |
|---------|---------|-----------|------|
| B Register | - | ~2% | None |
| TAB (B = AC) | 0x1C | ~0.5% | None |
| TBA (AC = B) | 0x1D | ~0.5% | None |
| LDB addr | 0x1E | ~1% | None |
| STB addr | 0x1F | ~1% | None |
| LDBI imm | 0xE4 | ~0.5% | None |
| INB (B++) | 0x36 | ~0.3% | None |
| DEB (B--) | 0x37 | ~0.3% | None |
| SWPB (AC↔B) | 0x38 | ~0.5% | None |
| ADDB (AC+B) | 0x39 | ~0.3% | None |
| SUBB (AC-B) | 0x3A | ~0.3% | None |

**Subtotal: ~7-8% savings, Zero risk**

#### 2. Sequential Multiplier/Divider (RARELY USED BY LCC)

| Feature | Module | Est. Area | Risk |
|---------|--------|-----------|------|
| Sequential Multiplier | `sequential_multiplier.sv` | ~8-10% | 3 tests affected |
| Sequential Divider | `sequential_divider.sv` | ~8-10% | 3 tests affected |
| MUL instruction | 0x09 | ~1% | 07_factorial |
| DIV instruction | 0x0E | ~1% | 12_division |
| MOD instruction | 0x0F | ~0.5% | 12_division |

**Subtotal: ~18-22% savings, Medium risk (3 LCC tests fail: 07_factorial, 12_division)**

---

### Medium Impact - Removable Features

#### 3. X/Y Register Extended Operations (NOT USED BY LCC)

| Feature | Opcodes | Est. Area | Risk |
|---------|---------|-----------|------|
| LDX addr | 0x7A | ~1% | None |
| STX addr | 0x7B | ~1% | None |
| LDXI imm | 0x7C | ~0.5% | None |
| TXA (AC = X) | 0x7E | ~0.3% | None |
| INX (X++) | 0x7F | ~0.3% | None |
| DEX (X--) | - | ~0.3% | None |
| LDY addr | 0x07 | ~1% | None |
| STY addr | 0x08 | ~1% | None |
| LDYI imm | 0x06 | ~0.5% | None |
| TAY (Y = AC) | 0x03 | ~0.3% | None |
| TYA (AC = Y) | 0x04 | ~0.3% | None |
| INY (Y++) | 0x05 | ~0.3% | None |
| DEY (Y--) | - | ~0.3% | None |
| SWPX (AC↔X) | - | ~0.5% | None |
| SWPY (AC↔Y) | - | ~0.5% | None |

**Subtotal: ~8-9% savings, Zero risk**

#### 4. Indexed Addressing Modes (NOT USED BY LCC)

| Feature | Opcodes | Est. Area | Risk |
|---------|---------|-----------|------|
| LDA addr,X | 0x21 | ~1% | None |
| STA addr,X | 0x11 | ~1% | None |
| LDA addr,Y | 0x22 | ~1% | None |
| STA addr,Y | 0x12 | ~1% | None |

**Subtotal: ~4% savings, Zero risk**

Note: LDA addr,FP and STA addr,FP (0x24, 0x14) ARE used by LCC for local variable access.

---

### Low Impact - Removable Features

#### 5. Shift Operations (NOT USED BY LCC)

| Feature | Opcodes | Est. Area | Risk |
|---------|---------|-----------|------|
| SHL (AC << 1) | 0x78 | ~0.5% | None |
| SHR (AC >> 1) | 0x79 | ~0.5% | None |
| ASR (arith shift) | 0x61 | ~0.5% | None |

**Subtotal: ~1.5% savings, Zero risk**

#### 6. Carry-Based Operations (NOT USED BY LCC)

| Feature | Opcodes | Est. Area | Risk |
|---------|---------|-----------|------|
| ADC (add with carry) | 0x31 | ~0.5% | None |
| SBC (sub with borrow) | 0x51 | ~0.5% | None |
| JC (jump if carry) | 0x81 | ~0.3% | None |
| JNC (jump if no carry) | 0x82 | ~0.3% | None |

**Subtotal: ~1.5% savings, Zero risk**

#### 7. Unsigned Comparison Jumps (NOT USED BY LCC)

| Feature | Opcodes | Est. Area | Risk |
|---------|---------|-----------|------|
| JBE (below or equal) | 0x86 | ~0.3% | None |
| JA (above) | 0x87 | ~0.3% | None |

**Subtotal: ~0.6% savings, Zero risk**

#### 8. Misc Unused Instructions

| Feature | Opcodes | Est. Area | Risk |
|---------|---------|-----------|------|
| INC (AC++) | 0x75 | ~0.3% | None |
| DEC (AC--) | 0x76 | ~0.3% | None |
| OR (bitwise) | 0x40 | ~0.3% | None |
| JN (jump if negative) | 0x90 | ~0.3% | None |
| JZ (jump if zero) | 0xA0 | ~0.3% | None |
| IN (I/O input) | 0xC0 | ~0.5% | Test compatibility |
| DECJNZ | 0x88 | ~0.5% | None |
| CMPI imm | 0xE1 | ~0.5% | None |
| MULI imm | 0xE2 | ~0.5% | None |
| DIVI imm | 0xE3 | ~0.5% | None |

**Subtotal: ~4% savings, Low risk**

---

## Summary: Removal Recommendations

| Priority | Feature Set | Est. Savings | Risk Level | LCC Tests Affected |
|----------|-------------|--------------|------------|-------------------|
| 🔴 1 | B Register subsystem | 7-8% | None | 0 |
| 🔴 2 | Sequential MUL/DIV | 18-22% | Medium | 3 (07, 12) |
| 🟡 3 | X/Y extended ops | 8-9% | None | 0 |
| 🟡 4 | X/Y indexed addressing | 4% | None | 0 |
| 🟢 5 | Shift operations | 1.5% | None | 0 |
| 🟢 6 | Carry operations | 1.5% | None | 0 |
| 🟢 7 | Unsigned jumps | 0.6% | None | 0 |
| 🟢 8 | Misc unused | 4% | Low | 0 |

### Total Potential Savings

| Scenario | Features Removed | Est. Savings | Tests Passing |
|----------|------------------|--------------|---------------|
| Conservative | 1, 3-8 | ~25-28% | All 23 LCC tests |
| Aggressive | 1-8 (all) | ~45-50% | 20/23 LCC tests |

---

## Interconnect Hub Features

The interconnect hub adds area but provides essential functionality:

| Module | Est. Area | Removable | Notes |
|--------|-----------|-----------|-------|
| `interconnect_hub.sv` | ~5% | No | Address decoding required |
| `debug_rom.sv` | ~2% | Yes | Only for boot mode testing |
| `pwm.sv` | ~2% | Yes | Peripheral - optional |
| `timer.sv` | ~2% | Yes | Peripheral - optional |
| `irq_ctrl.sv` | ~1% | Yes | Peripheral - optional |
| `spi_periph.sv` | ~3% | Yes | Future expansion |

**Potential interconnect savings: ~10% if peripherals removed**

---

## Recommended Removal Order for 2x2 Tile Target

If the design doesn't fit in 2x2 tiles (~35% reduction needed):

1. **Phase 1**: Remove B Register subsystem (~8%)
2. **Phase 2**: Remove X/Y extended operations (~9%)
3. **Phase 3**: Remove X/Y indexed addressing (~4%)
4. **Phase 4**: Remove shift/carry/misc (~8%)
5. **Phase 5**: Remove MUL/DIV hardware (~20%) - last resort

**Phase 1-4 total: ~29% reduction with zero LCC test impact**

If still not enough, Phase 5 removes MUL/DIV but breaks 3 tests.

---

## Files to Modify for Removal

| Feature | Files Affected |
|---------|----------------|
| B Register | `neander_x_datapath.sv`, `neander_x_control_unit.sv`, `top_cpu_neander_x.sv` |
| MUL/DIV | `sequential_multiplier.sv`, `sequential_divider.sv`, `neander_x_alu.sv`, `neander_x_control_unit.sv`, `top_cpu_neander_x.sv` |
| X/Y ops | `neander_x_datapath.sv`, `neander_x_control_unit.sv` |
| Shift ops | `neander_x_alu.sv`, `neander_x_control_unit.sv` |
| Carry ops | `neander_x_alu.sv`, `neander_x_control_unit.sv` |

---

## Current Tile Usage

- **Current**: 3x2 tiles (as per info.yaml)
- **Target**: 2x2 tiles
- **Required reduction**: ~33%

With conservative removal (Phase 1-4), we get ~29% reduction. Combined with potential interconnect peripheral removal (~10%), we should achieve the 2x2 target.
