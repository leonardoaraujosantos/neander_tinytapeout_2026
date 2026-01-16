# Porting NEANDER-X CPU from 8-bit to 16-bit

This document describes all the changes made to convert the NEANDER-X CPU from 8-bit data width to 16-bit data width, enabling more powerful arithmetic and better C compiler support.

## Overview

The 16-bit conversion was done in two phases:
1. **16-bit Addressing** (commit `fd1ecb1`): Extended address space from 256 bytes to 64KB
2. **16-bit Data Width** (commit `d546601`): Extended data registers and ALU from 8-bit to 16-bit

## Benefits of 16-bit Data Width

- **Standard C `int` = 16-bit**: More portable code with LCC compiler
- **Native 16-bit Arithmetic**: Single instructions instead of multi-byte sequences
- **Better Pointer Support**: Matches the 16-bit address space
- **Larger Value Range**: 0-65535 instead of 0-255

---

## Hardware Changes

### 1. ALU (`neander_x_alu.sv`)

The ALU was extended to operate on 16-bit values:

```systemverilog
// Before (8-bit)
input  logic [7:0] a,
input  logic [7:0] b,
output logic [7:0] result,

// After (16-bit)
input  logic [15:0] a,
input  logic [15:0] b,
output logic [15:0] result,
```

Key changes:
- All operands and results are now 16-bit
- Carry detection uses 17-bit temporary: `logic [16:0] temp;`
- Sign bit moved from bit 7 to bit 15
- Carry detection on shift operations moved to bit 15

### 2. Datapath Registers (`neander_x_datapath.sv`)

All data registers were extended to 16-bit:

| Register | Before | After | Notes |
|----------|--------|-------|-------|
| AC (Accumulator) | 8-bit | 16-bit | Main working register |
| X (Index) | 8-bit | 16-bit | For indexed addressing |
| Y (Index) | 8-bit | 16-bit | For indexed addressing and MUL high word |
| PC (Program Counter) | 8-bit | 16-bit | For 64KB address space |
| SP (Stack Pointer) | 8-bit | 16-bit | For 64KB stack |
| FP (Frame Pointer) | 8-bit | 16-bit | For stack frame access |
| REM (Memory Address) | 8-bit | 16-bit | Memory address register |
| RDM (Memory Data) | 8-bit | 16-bit | Memory data register |
| RI (Instruction Register) | 8-bit | 8-bit | Opcodes remain 8-bit |

The RDM register was extended with separate low/high byte loading for 16-bit address fetch:
```systemverilog
module rdm_reg (
    input  logic        load_lo,      // Load low byte from data_in
    input  logic        load_hi,      // Load high byte from data_in
    input  logic        load_full,    // Load full 16-bit from memory
    input  logic [7:0]  data_in,      // 8-bit data for address bytes
    input  logic [15:0] data_full,    // 16-bit data from memory
    output logic [15:0] value
);
```

### 3. Sequential Multiplier (`sequential_multiplier.sv`)

Extended from 8x8=16-bit to 16x16=32-bit multiplication:

```systemverilog
// 16-bit x 16-bit = 32-bit result
input  logic [15:0] multiplicand,
input  logic [15:0] multiplier,
output logic [15:0] product_low,   // Low 16 bits -> AC
output logic [15:0] product_high,  // High 16 bits -> Y
```

Uses shift-and-add algorithm over 16 cycles for area efficiency.

### 4. Sequential Divider (`sequential_divider.sv`)

Extended from 8/8-bit to 16/16-bit division:

```systemverilog
// 16-bit / 16-bit division
input  logic [15:0] dividend,
input  logic [15:0] divisor,
output logic [15:0] quotient,      // Result -> AC
output logic [15:0] remainder,     // Remainder -> Y
```

Uses restoring division algorithm over 16 cycles.

### 5. SPI Memory Controller (`spi_memory_controller.sv`)

Extended to transfer 16-bit data (2 bytes per access):

```
READ:  CS=0, send 0x03, send addr_hi, send addr_lo, receive data_lo, data_hi, CS=1
WRITE: CS=0, send 0x02, send addr_hi, send addr_lo, send data_lo, data_hi, CS=1
```

Key states added:
- `SEND_DATA_LO`, `SEND_DATA_HI` - For 16-bit writes
- `RECV_DATA_LO`, `RECV_DATA_HI` - For 16-bit reads

Data format is **little-endian**: low byte first, then high byte.

### 6. Control Unit (`neander_x_control_unit.sv`)

Major FSM changes for 16-bit operation:

#### Address Fetch (2-phase)
All memory-addressing instructions now fetch 16-bit addresses in two 8-bit reads:
```
S_xxx_1:    PC -> REM, start fetch
S_xxx_2:    Load addr_lo into RDM[7:0], increment PC
S_xxx_2B:   PC -> REM for high byte
S_xxx_2C:   Load addr_hi into RDM[15:8], increment PC
S_xxx_3:    RDM -> REM (16-bit address ready)
S_xxx_4:    Execute operation
```

#### Immediate Instructions (16-bit values)
`LDI`, `LDXI`, `LDYI` now fetch 16-bit immediate values:
```systemverilog
// Increment PC by 2 for 16-bit immediate
output logic pc_inc_2,    // New signal
```

#### Stack Operations (PUSH/POP)
PUSH and POP now decrement/increment SP by 2 for 16-bit values:

```systemverilog
// PUSH: SP decrements by 2 (two sp_dec cycles)
S_PUSH_1:  sp_dec = 1; next_state = S_PUSH_1B;  // First decrement
S_PUSH_1B: sp_dec = 1; next_state = S_PUSH_2;   // Second decrement
S_PUSH_2:  Write 16-bit AC to stack
S_PUSH_3:  Wait for memory ready

// POP: SP increments by 2 (two sp_inc cycles)
S_POP_1:  SP -> REM, start memory read
S_POP_2:  Wait for memory ready
S_POP_3:  Load 16-bit value into AC, sp_inc = 1  // First increment
S_POP_4:  sp_inc = 1; next_state = S_FETCH_1    // Second increment
```

This ensures consecutive PUSHes don't overlap (each 16-bit value occupies 2 bytes).

#### CALL/RET
CALL pushes 16-bit return address, RET pops 16-bit return address.

#### PUSH_FP/POP_FP
Frame pointer operations also decrement/increment SP by 2:
```systemverilog
// PUSH_FP: SP -= 2
S_PUSH_FP_1:  sp_dec = 1; next_state = S_PUSH_FP_4;  // First decrement
S_PUSH_FP_4:  sp_dec = 1; next_state = S_PUSH_FP_2;  // Second decrement
S_PUSH_FP_2:  Write 16-bit FP to stack

// POP_FP: SP += 2
S_POP_FP_1:  SP -> REM, start read
S_POP_FP_2:  Wait for memory ready
S_POP_FP_3:  Load 16-bit FP, sp_inc = 1  // First increment
S_POP_FP_4:  sp_inc = 1; return to FETCH // Second increment
```

---

## Instruction Format Changes

### Memory-Addressing Instructions
Before: `[opcode] [addr8]` (2 bytes)
After: `[opcode] [addr_lo] [addr_hi]` (3 bytes)

Affected instructions: `LDA`, `STA`, `ADD`, `SUB`, `AND`, `OR`, `XOR`, `CMP`, `LDX`, `STX`, `LDY`, `STY`, `JMP`, `JN`, `JZ`, `JNZ`, `JC`, `JNC`, `JLE`, `JGT`, `JGE`, `JBE`, `JA`, `CALL`

### 16-bit Immediate Instructions
Before: `[opcode] [imm8]` (2 bytes)
After: `[opcode] [imm_lo] [imm_hi]` (3 bytes)

Affected instructions: `LDI`, `LDXI`, `LDYI`

### 8-bit Port Instructions (unchanged)
`IN` and `OUT` still use 8-bit port numbers: `[opcode] [port8]` (2 bytes)

---

## Test Infrastructure Changes

### Program Converter (`convert_program_to_16bit`)

A utility function was added to both test files to convert old 8-bit format programs to 16-bit format:

```python
def convert_program_to_16bit(program, data_area_start=0x40):
    """
    Convert old 8-bit address format program to 16-bit address format.

    - Memory address opcodes: expand from 2 bytes to 3 bytes
    - 16-bit immediate opcodes (LDI, LDXI, LDYI): expand to 3 bytes
    - 8-bit immediate opcodes (IN, OUT): keep as 2 bytes
    - Single byte opcodes: keep as 1 byte
    - Jump targets are remapped to new addresses
    """
```

### Opcode Categories

```python
# Opcodes that take a memory address (need expansion)
MEMORY_ADDRESS_OPCODES = {
    0x10, 0x11, 0x12, 0x14,  # STA family
    0x20, 0x21, 0x22, 0x24,  # LDA family
    0x30, 0x31,              # ADD, ADC
    0x40,                    # OR
    0x50, 0x51,              # AND, SBC
    0x74, 0x77,              # SUB, XOR
    0x02,                    # CMP
    0x7A, 0x7B,              # LDX, STX
    0x07, 0x08,              # LDY, STY
    0x80-0x87, 0x90, 0xA0, 0xB0,  # JMP and conditional jumps
    0x72,                    # CALL
}

# 16-bit immediate opcodes (expand to 3 bytes with sign extension)
IMMEDIATE_16BIT_OPCODES = {
    0xE0,  # LDI
    0x7C,  # LDXI
    0x06,  # LDYI
}

# 8-bit immediate opcodes (keep as 2 bytes)
IMMEDIATE_8BIT_OPCODES = {
    0xD0,  # OUT (port number)
    0xC0,  # IN  (port number)
}
```

### Address Remapping

The converter handles two types of addresses:
1. **Code addresses** (jump targets): Remapped based on instruction expansion
2. **Data addresses**: Remapped to account for 16-bit data slots

```python
# Data addresses are remapped for 16-bit values
# Each old 1-byte slot maps to 2 bytes
new_addr = data_area_start + (addr - data_area_start) * 2
```

---

## Stack Layout (16-bit)

With 16-bit data, the stack layout for function calls is:

```
Higher addresses
+------------------+
| Parameter N      | <- FP + 4 + 2*(N-1)
| ...              |
| Parameter 1      | <- FP + 4
| Return Address   | <- FP + 2 (16-bit)
| Old FP           | <- FP (16-bit)
| Local Variable 1 | <- FP - 2
| Local Variable 2 | <- FP - 4
| ...              |
+------------------+
Lower addresses    <- SP
```

Function prologue:
```assembly
PUSH_FP       ; Save old FP (SP -= 2)
TSF           ; FP = SP
; Allocate locals with PUSH
```

Function epilogue:
```assembly
TFS           ; SP = FP (deallocate locals)
POP_FP        ; Restore old FP (SP += 2)
RET           ; Return (pops 16-bit return address)
```

---

## Files Modified

| File | Changes |
|------|---------|
| `src/neander_x_alu.sv` | 16-bit operands, 17-bit carry detection |
| `src/neander_x_datapath.sv` | 16-bit registers (AC, X, Y, PC, SP, FP, REM, RDM) |
| `src/neander_x_control_unit.sv` | Extended FSM for 16-bit address fetch, 16-bit stack ops |
| `src/sequential_multiplier.sv` | 16x16=32-bit multiplication |
| `src/sequential_divider.sv` | 16/16-bit division |
| `src/spi_memory_controller.sv` | 16-bit data transfers (2 bytes per access) |
| `src/top_cpu_neander_x.sv` | 16-bit interface signals |
| `src/project.sv` | 16-bit top-level data bus |
| `cocotb_tests/test_neander.py` | `convert_program_to_16bit()`, updated tests |
| `test/test.py` | `convert_program_to_16bit()`, updated tests |

---

## Test Coverage

After the 16-bit conversion:
- **cocotb_tests/**: 189 tests passing
- **test/**: 127 tests passing

Tests verify:
- All arithmetic operations work with 16-bit values
- Stack operations (PUSH/POP) correctly change SP by 2
- Function calls (CALL/RET) work with 16-bit return addresses
- Frame pointer operations (PUSH_FP/POP_FP) work with 16-bit FP
- Consecutive stack operations don't overlap
- Indexed addressing works with 16-bit offsets
- MUL produces 32-bit result (low in AC, high in Y)
- DIV/MOD produce 16-bit quotient and remainder
