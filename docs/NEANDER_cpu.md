# Neander CPU Implementation Guide

This document provides a complete guide for implementing the Neander processor in SystemVerilog. The Neander is an educational 8-bit processor developed at UFRGS (Universidade Federal do Rio Grande do Sul) for teaching fundamental computer architecture concepts.

**NEANDER-X Extension:** This implementation extends the original NEANDER with 16-bit addressing, enabling a full 64KB address space accessed via SPI SRAM.

## Table of Contents

1. [Overview](#overview)
2. [Architecture Specifications](#architecture-specifications)
3. [Instruction Set Architecture](#instruction-set-architecture)
4. [Register Set](#register-set)
5. [Memory Organization](#memory-organization)
6. [Datapath Design](#datapath-design)
7. [ALU Design](#alu-design)
8. [Control Unit (FSM)](#control-unit-fsm)
9. [Module Implementation](#module-implementation)
10. [Testing and Verification](#testing-and-verification)
11. [Example Programs](#example-programs)
12. [References](#references)

---

## Overview

The Neander processor is a minimal, accumulator-based architecture designed for educational purposes. Its simplicity makes it ideal for learning:

- Fetch-Decode-Execute cycle
- Control unit design using FSMs
- Datapath components and interconnections
- Assembly language programming fundamentals

### Key Features (NEANDER-X with 16-bit Addressing)

- 8-bit data width
- **16-bit address space (64KB of memory via SPI SRAM)**
- Single accumulator architecture with X, Y, and FP registers
- Direct and indexed addressing modes
- Three condition flags (N, Z, and C)
- **60+ instructions** including stack operations, hardware multiply/divide
- **16-bit registers**: PC, SP, FP, REM, RDM for full address range
- FSM-based control unit with ~90 states

---

## Architecture Specifications

| Parameter | Value (Original NEANDER) | Value (NEANDER-X) |
|-----------|--------------------------|-------------------|
| Data Width | 8 bits | 8 bits |
| Address Width | 8 bits | **16 bits** |
| Memory Size | 256 bytes | **64KB (via SPI SRAM)** |
| Number Format | Two's complement | Two's complement |
| Addressing Mode | Direct only | Direct, Indexed (X, Y, FP) |
| Accumulator Count | 1 | 1 + X, Y registers |
| Condition Flags | 2 (N, Z) | **3 (N, Z, C)** |
| Instruction Count | 11 | **60+** |

### NEANDER-X Register Widths

| Register | Width | Description |
|----------|-------|-------------|
| PC | 16-bit | Program Counter (0x0000-0xFFFF) |
| SP | 16-bit | Stack Pointer (reset: 0x00FF) |
| FP | 16-bit | Frame Pointer |
| REM | 16-bit | Memory Address Register |
| RDM | 16-bit | Memory Data Register (for 16-bit addresses) |
| AC | 8-bit | Accumulator |
| X | 8-bit | Index Register X |
| Y | 8-bit | Index Register Y |
| RI | 8-bit | Instruction Register |

### Block Diagram (NEANDER-X with 16-bit Addressing)

```
                    +------------------+
                    |   SPI SRAM       |
                    |    (64KB)        |
                    +--------+---------+
                             |
              +--------------+---------------+
              | 16-bit addr  |  8-bit data   |
              v              |               v
         +----+----+         |         +-----+-----+
         |   REM   |<---+    |         |    RDM    |
         | (16-bit)|    |    |         |  (16-bit) |
         +---------+    |    |         +-----------+
              |         |    |               |
              v         |    |               v
         +----+----+    |    |         +-----+-----+
         |   MUX   |----+    |         |    RI     |
         | (16-bit)|         |         |  (8-bit)  |
         +---------+         |         +-----------+
              ^              |               |
              |              |               v
         +----+----+    +----+----+    +-----+-----+
         |   PC    |    |   SP    |    |  Decoder  |
         | (16-bit)|    | (16-bit)|    +-----------+
         +---------+    +---------+          |
                                             v
              +------------------------------+
              |
              v
         +----+----+     +----------+     +-----+-----+
         |   AC    |<----|   ALU    |---->|   X, Y    |
         | (8-bit) |     +----------+     |  (8-bit)  |
         +---------+                      +-----------+
              |
              v
         +----+----+
         | N |Z| C |
         +---------+
```

---

## Instruction Set Architecture

### Instruction Format (16-bit Addressing)

With 16-bit addressing, instructions use variable-length encoding:

**3-byte format (memory operations):**
```
Byte 0: [Opcode (8 bits)]
Byte 1: [Address Low (8 bits)]
Byte 2: [Address High (8 bits)]
```

**2-byte format (immediate/port operations):**
```
Byte 0: [Opcode (8 bits)]
Byte 1: [Immediate/Port (8 bits)]
```

**1-byte format (register operations):**
```
Byte 0: [Opcode (8 bits)]
```

The address is stored in **little-endian** order (low byte first, high byte second).

### Complete Instruction Set

| Opcode (Hex) | Opcode (Binary) | Mnemonic | Operation | Flags Affected |
|--------------|-----------------|----------|-----------|----------------|
| 0x0 | 0000 | NOP | No operation | None |
| 0x1 | 0001 | STA addr | MEM[addr] <- AC | None |
| 0x2 | 0010 | LDA addr | AC <- MEM[addr] | N, Z |
| 0x3 | 0011 | ADD addr | AC <- AC + MEM[addr] | N, Z |
| 0x4 | 0100 | OR addr | AC <- AC OR MEM[addr] | N, Z |
| 0x5 | 0101 | AND addr | AC <- AC AND MEM[addr] | N, Z |
| 0x6 | 0110 | NOT | AC <- NOT AC | N, Z |
| 0x8 | 1000 | JMP addr | PC <- addr | None |
| 0x9 | 1001 | JN addr | IF N=1 THEN PC <- addr | None |
| 0xA | 1010 | JZ addr | IF Z=1 THEN PC <- addr | None |
| 0xF | 1111 | HLT | Halt execution | None |

### Extended Instructions (Implemented in NeanderX)

The following instructions enhance the processor:

| Opcode (Hex) | Mnemonic | Operation | Description |
|--------------|----------|-----------|-------------|
| 0x70 | PUSH | SP <- SP-1; MEM[SP] <- AC | Push AC onto stack |
| 0x71 | POP | AC <- MEM[SP]; SP <- SP+1 | Pop from stack to AC |
| 0xB | JNZ addr | IF Z=0 THEN PC <- addr | Jump if not zero |
| 0xC | IN port | AC <- IO[port] | Input from I/O port |
| 0xD | OUT port | IO[port] <- AC | Output to I/O port |
| 0xE | LDI imm | AC <- imm | Load immediate value |

### Stack Operations

The stack pointer (SP) is initialized to 0xFF and grows downward. The stack
occupies the upper region of memory (addresses 0xFF downward).

**PUSH (0x70):** Decrements SP first, then writes AC to MEM[SP]
**POP (0x71):** Reads MEM[SP] to AC, then increments SP. Updates N and Z flags.

### Reserved for Future Implementation

| Opcode (Hex) | Mnemonic | Operation | Description |
|--------------|----------|-----------|-------------|
| 0x72 | CALL addr | PUSH PC+2; PC <- addr | Call subroutine |
| 0x73 | RET | POP to PC | Return from subroutine |

### Instruction Encoding Examples (16-bit Addressing)

```
; Memory operations (3 bytes: opcode + addr_lo + addr_hi)
LDA 0x0080  -> 0x20 0x80 0x00    ; Load from address 0x0080
ADD 0x1234  -> 0x30 0x34 0x12    ; Add value at address 0x1234
STA 0x8000  -> 0x10 0x00 0x80    ; Store at address 0x8000
JMP 0x0100  -> 0x80 0x00 0x01    ; Jump to address 0x0100
CALL 0x2000 -> 0x72 0x00 0x20    ; Call subroutine at 0x2000

; Immediate/port operations (2 bytes)
LDI 0x42    -> 0xE0 0x42         ; Load immediate 0x42
OUT 0       -> 0xD0 0x00         ; Output to port 0

; Single-byte operations
NOT         -> 0x60              ; Complement accumulator
HLT         -> 0xF0              ; Halt
RET         -> 0x73              ; Return from subroutine
PUSH        -> 0x70              ; Push AC onto stack
POP         -> 0x71              ; Pop from stack to AC
```

---

## Register Set

### Program Counter (PC)

- **Width**: **16 bits** (extended from 8 bits)
- **Function**: Holds the address of the next instruction to fetch
- **Operations**: Increment, Load (for jumps)
- **Reset Value**: 0x0000
- **Address Range**: 0x0000 - 0xFFFF (64KB)

```systemverilog
module pc_reg (
    input  logic        clk,
    input  logic        reset,
    input  logic        pc_inc,
    input  logic        pc_load,
    input  logic [15:0] data_in,    // 16-bit input
    output logic [15:0] pc_value    // 16-bit output
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            pc_value <= 16'h0000;
        else if (pc_load)
            pc_value <= data_in;
        else if (pc_inc)
            pc_value <= pc_value + 16'h0001;
    end
endmodule
```

### Accumulator (AC)

- **Width**: 8 bits
- **Function**: Main data register for arithmetic and logic operations
- **Operations**: Load from ALU result or memory
- **Reset Value**: 0x00

### Memory Address Register (REM)

- **Width**: **16 bits** (extended from 8 bits)
- **Function**: Holds the address for memory access
- **Source**: PC (instruction fetch), RDM (operand address), or SP (stack access)
- **Indexed Addressing**: Supports REM + X, REM + Y, REM + FP for indexed modes

### Memory Data Register (RDM)

- **Width**: **16 bits** (extended from 8 bits)
- **Function**: Holds data read from memory (including 16-bit addresses)
- **Usage**: Temporarily stores instruction operands and 16-bit target addresses
- **Special Feature**: Separate load signals for low byte (`rdm_load`) and high byte (`rdm_load_hi`) to support fetching 16-bit addresses in two memory reads

```systemverilog
module rdm_reg (
    input  logic        clk,
    input  logic        reset,
    input  logic        load_lo,    // Load low byte (addr_lo)
    input  logic        load_hi,    // Load high byte (addr_hi)
    input  logic [7:0]  data_in,    // 8-bit data from memory
    output logic [15:0] rdm_value   // Full 16-bit value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            rdm_value <= 16'h0000;
        else if (load_lo)
            rdm_value[7:0] <= data_in;   // Load low byte
        else if (load_hi)
            rdm_value[15:8] <= data_in;  // Load high byte
    end
endmodule
```

### Instruction Register (RI)

- **Width**: 8 bits
- **Function**: Holds the current instruction opcode
- **Opcode Extraction**: Upper 4 bits (RI[7:4])

### Condition Flags (N, Z, C)

- **N (Negative)**: Set when AC[7] = 1 (MSB indicates negative in two's complement)
- **Z (Zero)**: Set when AC = 0x00
- **C (Carry)**: Set on arithmetic overflow/borrow, shift out, or MUL overflow

The Carry flag enables:
- Unsigned comparisons (JC, JNC after CMP)
- Multi-byte arithmetic (ADC, SBC propagate carry between bytes)
- Hardware multiply overflow detection (C=1 when result > 255)

```systemverilog
module nzc_reg (
    input  logic clk,
    input  logic reset,
    input  logic nzc_load,
    input  logic N_in,
    input  logic Z_in,
    input  logic C_in,
    output logic N_flag,
    output logic Z_flag,
    output logic C_flag
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            N_flag <= 1'b0;
            Z_flag <= 1'b0;
            C_flag <= 1'b0;
        end
        else if (nzc_load) begin
            N_flag <= N_in;
            Z_flag <= Z_in;
            C_flag <= C_in;
        end
    end
endmodule
```

---

## Memory Organization

### Address Space (64KB)

With 16-bit addressing, the full 64KB address space is available:

```
+----------------+------------------+
| Address Range  | Usage            |
+----------------+------------------+
| 0x0000-0x00FF  | Page 0 (stack grows down from 0x00FF) |
| 0x0100-0x7FFF  | Program Code / Data |
| 0x8000-0xFFFF  | Extended Data / Large Programs |
+----------------+------------------+
```

The Stack Pointer (SP) is initialized to 0x00FF and grows downward toward 0x0000.

### Memory Interface (SPI SRAM)

Memory is accessed via an external SPI SRAM chip (e.g., 23LC512). The SPI Memory Controller handles the parallel-to-serial conversion:

```systemverilog
// CPU Memory Interface (parallel, 16-bit address)
output logic [15:0] mem_addr,      // 16-bit address
output logic [7:0]  mem_wdata,     // 8-bit write data
input  logic [7:0]  mem_rdata,     // 8-bit read data
output logic        mem_req,       // Memory request
input  logic        mem_ready,     // Memory ready
output logic        mem_we         // Write enable
```

Each memory access takes approximately **70 CPU cycles** due to the SPI protocol overhead (command + 16-bit address + data).

### SPI Memory Controller

The SPI controller converts parallel CPU requests to serial SPI transactions:

```
CPU Request → SPI Controller → SPI SRAM
  [addr 16-bit]     [CS, SCLK, MOSI]     [64KB storage]
  [data 8-bit]      [MISO]
  [req/ready]
```

Protocol: SPI Mode 0 (CPOL=0, CPHA=0)
- READ command: 0x03 + addr_hi + addr_lo + (receive data)
- WRITE command: 0x02 + addr_hi + addr_lo + data

---

## Datapath Design

### Control Signals (16-bit Addressing)

| Signal | Description |
|--------|-------------|
| `mem_read` | Enable memory read operation |
| `mem_write` | Enable memory write operation |
| `pc_inc` | Increment 16-bit program counter |
| `pc_load` | Load 16-bit program counter (jump) |
| `ac_load` | Load accumulator |
| `ri_load` | Load instruction register |
| `rem_load` | Load 16-bit memory address register |
| `rdm_load` | Load RDM low byte (addr_lo) |
| `rdm_load_hi` | Load RDM high byte (addr_hi) - **NEW** |
| `nzc_load` | Update condition flags (N, Z, C) |
| `addr_sel[1:0]` | 3-way MUX: 00=RDM, 01=PC, 10=SP |
| `alu_op[3:0]` | Extended ALU operation select |
| `sp_inc` | Increment 16-bit stack pointer |
| `sp_dec` | Decrement 16-bit stack pointer |
| `fp_load` | Load 16-bit frame pointer from SP |
| `fp_load_lo` | Load FP low byte (POP_FP) - **NEW** |
| `fp_load_hi` | Load FP high byte (POP_FP) - **NEW** |
| `mem_data_sel_ext[2:0]` | Memory write data source (extended for 16-bit) |

### Address MUX (16-bit, 3-way)

Selects between PC (for instruction fetch), RDM (for operand address), and SP (for stack access):

```systemverilog
module mux_addr (
    input  logic [1:0]  sel,     // 00=RDM, 01=PC, 10=SP
    input  logic [15:0] pc,      // 16-bit PC
    input  logic [15:0] rdm,     // 16-bit RDM
    input  logic [15:0] sp,      // 16-bit SP
    output logic [15:0] out      // 16-bit output
);
    always_comb begin
        case (sel)
            2'b00:   out = rdm;
            2'b01:   out = pc;
            2'b10:   out = sp;
            default: out = rdm;
        endcase
    end
endmodule
```

### Indexed Address Calculation

For indexed addressing modes, the effective address is computed as:

```systemverilog
// Indexed addressing: base + index
// X and Y are 8-bit, zero-extended to 16-bit
// FP is already 16-bit
always_comb begin
    case (index_sel)
        2'b00:   indexed_addr = rem;                      // Direct
        2'b01:   indexed_addr = rem + {8'h00, x_reg};    // Indexed with X
        2'b10:   indexed_addr = rem + {8'h00, y_reg};    // Indexed with Y
        2'b11:   indexed_addr = rem + fp_reg;            // Indexed with FP (16-bit)
        default: indexed_addr = rem;
    endcase
end
```

### Datapath Module (16-bit Addressing)

```systemverilog
module neander_datapath (
    input  logic        clk,
    input  logic        reset,

    // Control Signals
    input  logic        mem_read,
    input  logic        mem_write,
    input  logic        pc_inc,
    input  logic        pc_load,
    input  logic        ac_load,
    input  logic        ri_load,
    input  logic        rem_load,
    input  logic        rdm_load,
    input  logic        rdm_load_hi,    // NEW: Load RDM high byte
    input  logic        nzc_load,
    input  logic [1:0]  addr_sel,       // 3-way mux select
    input  logic [3:0]  alu_op,
    input  logic        sp_inc,
    input  logic        sp_dec,
    input  logic        fp_load,
    input  logic        fp_load_lo,     // NEW: Load FP low byte
    input  logic        fp_load_hi,     // NEW: Load FP high byte

    // Memory Interface (16-bit address)
    input  logic [7:0]  mem_data_in,
    output logic [15:0] mem_addr,       // 16-bit address
    output logic [7:0]  mem_data_out,

    // Control Unit Interface
    output logic [7:0]  opcode,
    output logic        flagN,
    output logic        flagZ,
    output logic        flagC,

    // Debug (16-bit)
    output logic [15:0] dbg_pc,
    output logic [15:0] dbg_sp,
    output logic [15:0] dbg_fp,
    output logic [7:0]  dbg_ac
);
    // Internal signals (16-bit for address registers)
    logic [15:0] pc, rem, rdm, sp, fp;
    logic [7:0]  ri, ac, x, y;
    logic [7:0]  alu_res;
    logic [15:0] addr_mux;
    logic        N_in, Z_in, C_in;

    // Module instantiations and connections...
endmodule
```

---

## ALU Design

### ALU Operations

| alu_op | Operation | Description |
|--------|-----------|-------------|
| 2'b00 | ADD | AC + operand |
| 2'b01 | AND | AC AND operand |
| 2'b10 | OR | AC OR operand |
| 2'b11 | NOT | NOT AC (inverts all bits) |

### ALU Implementation

```systemverilog
module neander_alu (
    input  logic [7:0] a,       // Accumulator
    input  logic [7:0] b,       // Operand from memory
    input  logic [1:0] alu_op,  // Operation select
    output logic [7:0] result
);
    always_comb begin
        case (alu_op)
            2'b00:   result = a + b;   // ADD
            2'b01:   result = a & b;   // AND
            2'b10:   result = a | b;   // OR
            2'b11:   result = ~a;      // NOT (operand b ignored)
            default: result = 8'h00;
        endcase
    end
endmodule
```

### Flag Generation

```systemverilog
// N flag: set when result is negative (MSB = 1)
assign N_in = ac_in[7];

// Z flag: set when result is zero
assign Z_in = (ac_in == 8'h00);
```

---

## Control Unit (FSM)

### State Diagram Overview

The control unit implements a finite state machine with the following major phases:

1. **FETCH**: Load instruction opcode from memory
2. **DECODE**: Determine instruction type
3. **ADDRESS FETCH**: For memory instructions, fetch 16-bit address (2 bytes)
4. **EXECUTE**: Perform the operation (instruction-specific states)

### 16-bit Address Fetch Pattern

With 16-bit addressing, memory-addressing instructions require **6 states** instead of 4:

```
S_xxx_1:  PC → REM (setup for addr_lo fetch)
S_xxx_2:  Fetch addr_lo, RDM[7:0] ← mem, PC++
S_xxx_2B: PC → REM (setup for addr_hi fetch)  ← NEW
S_xxx_2C: Fetch addr_hi, RDM[15:8] ← mem, PC++ ← NEW
S_xxx_3:  RDM → REM (set effective address)
S_xxx_4:  Execute (read/write data)
```

### State Definitions (9-bit encoding for ~90 states)

```systemverilog
typedef enum logic [8:0] {
    // Fetch states
    S_FETCH_1,      // PC -> REM, initiate read
    S_FETCH_2,      // Read complete, load RDM
    S_FETCH_3,      // RDM -> RI, increment PC
    S_DECODE,       // Decode opcode

    // LDA states (6 states for 16-bit address)
    S_LDA_1,        // PC -> REM
    S_LDA_2,        // Fetch addr_lo to RDM[7:0], PC++
    S_LDA_2B,       // PC -> REM (for addr_hi)
    S_LDA_2C,       // Fetch addr_hi to RDM[15:8], PC++
    S_LDA_3,        // RDM -> REM
    S_LDA_4,        // Read data to AC

    // STA states (6 states for 16-bit address)
    S_STA_1, S_STA_2, S_STA_2B, S_STA_2C, S_STA_3, S_STA_4,

    // ADD/SUB/AND/OR/XOR states (6 states each)
    S_ADD_1, S_ADD_2, S_ADD_2B, S_ADD_2C, S_ADD_3, S_ADD_4,
    S_SUB_1, S_SUB_2, S_SUB_2B, S_SUB_2C, S_SUB_3, S_SUB_4,
    S_AND_1, S_AND_2, S_AND_2B, S_AND_2C, S_AND_3, S_AND_4,
    S_OR_1,  S_OR_2,  S_OR_2B,  S_OR_2C,  S_OR_3,  S_OR_4,
    S_XOR_1, S_XOR_2, S_XOR_2B, S_XOR_2C, S_XOR_3, S_XOR_4,

    // Jump states (6 states each for 16-bit target address)
    S_JMP_1, S_JMP_2, S_JMP_2B, S_JMP_2C, S_JMP_3, S_JMP_4,
    S_JN_1,  S_JN_2,  S_JN_2B,  S_JN_2C,  S_JN_3,  S_JN_4,
    S_JZ_1,  S_JZ_2,  S_JZ_2B,  S_JZ_2C,  S_JZ_3,  S_JZ_4,
    S_JNZ_1, S_JNZ_2, S_JNZ_2B, S_JNZ_2C, S_JNZ_3, S_JNZ_4,

    // CALL states (9 states: fetch addr + push 16-bit return addr + jump)
    S_CALL_1, S_CALL_2, S_CALL_2B, S_CALL_2C,  // Fetch 16-bit target
    S_CALL_3, S_CALL_4,                         // Push PC high byte
    S_CALL_5, S_CALL_6,                         // Push PC low byte
    S_CALL_7,                                   // Load PC with target

    // RET states (6 states: pop 16-bit return address)
    S_RET_1, S_RET_2,  // Pop PC low byte
    S_RET_3, S_RET_4,  // Pop PC high byte
    S_RET_5, S_RET_6,  // Load PC

    // Stack operations
    S_PUSH_1, S_PUSH_2, S_PUSH_3,
    S_POP_1,  S_POP_2,  S_POP_3,

    // PUSH_FP (6 states: push 16-bit FP)
    S_PUSH_FP_1, S_PUSH_FP_2, S_PUSH_FP_3,  // Push FP low byte
    S_PUSH_FP_4, S_PUSH_FP_5, S_PUSH_FP_6,  // Push FP high byte

    // POP_FP (6 states: pop 16-bit FP)
    S_POP_FP_1, S_POP_FP_2, S_POP_FP_3,     // Pop FP low byte
    S_POP_FP_4, S_POP_FP_5, S_POP_FP_6,     // Pop FP high byte

    // Single-cycle operations
    S_NOT, S_INC, S_DEC, S_SHL, S_SHR, S_ASR, S_NEG,
    S_TAX, S_TXA, S_INX, S_TAY, S_TYA, S_INY,
    S_TSF, S_TFS, S_MUL, S_DIV, S_MOD,

    // Immediate/IO operations (2-byte, no change)
    S_LDI_1, S_LDI_2,
    S_IN_1,  S_IN_2,  S_IN_3,
    S_OUT_1, S_OUT_2, S_OUT_3,

    // Halt
    S_HLT
} state_t;
```

### Instruction Execution Cycles (16-bit Addressing)

#### FETCH Cycle (All Instructions)

```
S_FETCH_1: addr_sel=01(PC), rem_load=1, mem_read=1  ; PC -> REM
S_FETCH_2: mem_read=1, rdm_load=1                   ; Read opcode to RDM
S_FETCH_3: ri_load=1, pc_inc=1                      ; RDM -> RI, PC++
S_DECODE:  (decode opcode)
```

#### LDA (Load Accumulator) - 6 states for 16-bit address

```
S_LDA_1:  addr_sel=01(PC), rem_load=1, mem_read=1   ; PC -> REM
S_LDA_2:  mem_read=1, rdm_load=1, pc_inc=1          ; Fetch addr_lo to RDM[7:0], PC++
S_LDA_2B: addr_sel=01(PC), rem_load=1, mem_read=1   ; PC -> REM (for addr_hi)
S_LDA_2C: mem_read=1, rdm_load_hi=1, pc_inc=1       ; Fetch addr_hi to RDM[15:8], PC++
S_LDA_3:  addr_sel=00(RDM), rem_load=1, mem_read=1  ; RDM(16-bit) -> REM
S_LDA_4:  mem_read=1, ac_load=1, nzc_load=1         ; Read data to AC
```

#### STA (Store Accumulator) - 6 states for 16-bit address

```
S_STA_1:  addr_sel=01(PC), rem_load=1, mem_read=1   ; PC -> REM
S_STA_2:  mem_read=1, rdm_load=1, pc_inc=1          ; Fetch addr_lo to RDM[7:0], PC++
S_STA_2B: addr_sel=01(PC), rem_load=1, mem_read=1   ; PC -> REM (for addr_hi)
S_STA_2C: mem_read=1, rdm_load_hi=1, pc_inc=1       ; Fetch addr_hi to RDM[15:8], PC++
S_STA_3:  addr_sel=00(RDM), rem_load=1              ; RDM(16-bit) -> REM
S_STA_4:  mem_write=1                               ; Write AC to memory
```

#### ADD/AND/OR/SUB/XOR (Arithmetic/Logic) - 6 states each

```
S_xxx_1:  addr_sel=01(PC), rem_load=1, mem_read=1   ; PC -> REM
S_xxx_2:  mem_read=1, rdm_load=1, pc_inc=1          ; Fetch addr_lo, PC++
S_xxx_2B: addr_sel=01(PC), rem_load=1, mem_read=1   ; PC -> REM (for addr_hi)
S_xxx_2C: mem_read=1, rdm_load_hi=1, pc_inc=1       ; Fetch addr_hi, PC++
S_xxx_3:  addr_sel=00(RDM), rem_load=1, mem_read=1  ; RDM -> REM
S_xxx_4:  mem_read=1, ac_load=1, alu_op=xxx, nzc_load=1 ; Perform operation
```

#### NOT (Single-cycle, no address fetch)

```
S_NOT: ac_load=1, alu_op=NOT, nzc_load=1            ; Single cycle
```

#### JMP (Unconditional Jump) - 6 states for 16-bit target

```
S_JMP_1:  addr_sel=01(PC), rem_load=1, mem_read=1   ; PC -> REM
S_JMP_2:  mem_read=1, rdm_load=1, pc_inc=1          ; Fetch addr_lo to RDM[7:0]
S_JMP_2B: addr_sel=01(PC), rem_load=1, mem_read=1   ; PC -> REM (for addr_hi)
S_JMP_2C: mem_read=1, rdm_load_hi=1                 ; Fetch addr_hi to RDM[15:8]
S_JMP_3:  pc_load=1                                 ; Load 16-bit RDM -> PC
S_JMP_4:  (return to FETCH)
```

#### JN/JZ/JNZ (Conditional Jumps) - 6 states for 16-bit target

```
S_Jx_1:  addr_sel=01(PC), rem_load=1, mem_read=1    ; PC -> REM
S_Jx_2:  mem_read=1, rdm_load=1, pc_inc=1           ; Fetch addr_lo
S_Jx_2B: addr_sel=01(PC), rem_load=1, mem_read=1    ; PC -> REM (for addr_hi)
S_Jx_2C: mem_read=1, rdm_load_hi=1, pc_inc=1        ; Fetch addr_hi, PC++
S_Jx_3:  if(condition) pc_load=1                    ; Conditional load PC
S_Jx_4:  (return to FETCH)
```

#### CALL (9 states: fetch 16-bit addr + push 16-bit PC + jump)

```
S_CALL_1:  addr_sel=01(PC), rem_load=1, mem_read=1  ; PC -> REM
S_CALL_2:  mem_read=1, rdm_load=1, pc_inc=1         ; Fetch target addr_lo
S_CALL_2B: addr_sel=01(PC), rem_load=1, mem_read=1  ; PC -> REM
S_CALL_2C: mem_read=1, rdm_load_hi=1, pc_inc=1      ; Fetch target addr_hi
S_CALL_3:  sp_dec=1                                 ; SP--
S_CALL_4:  addr_sel=10(SP), rem_load=1, mem_write=1 ; Push PC high byte
           mem_data_sel=PC_HI
S_CALL_5:  sp_dec=1                                 ; SP--
S_CALL_6:  addr_sel=10(SP), rem_load=1, mem_write=1 ; Push PC low byte
           mem_data_sel=PC_LO
S_CALL_7:  pc_load=1                                ; PC <- RDM (16-bit target)
```

#### RET (6 states: pop 16-bit return address)

```
S_RET_1:  addr_sel=10(SP), rem_load=1, mem_read=1   ; SP -> REM
S_RET_2:  mem_read=1, rdm_load=1, sp_inc=1          ; Pop PC low byte, SP++
S_RET_3:  addr_sel=10(SP), rem_load=1, mem_read=1   ; SP -> REM
S_RET_4:  mem_read=1, rdm_load_hi=1, sp_inc=1       ; Pop PC high byte, SP++
S_RET_5:  pc_load=1                                 ; PC <- RDM (16-bit)
S_RET_6:  (return to FETCH)
```

### Control Unit Implementation

```systemverilog
module neander_control (
    input  logic       clk,
    input  logic       reset,
    input  logic [3:0] opcode,
    input  logic       flagN,
    input  logic       flagZ,

    output logic       mem_read,
    output logic       mem_write,
    output logic       pc_inc,
    output logic       pc_load,
    output logic       ac_load,
    output logic       ri_load,
    output logic       rem_load,
    output logic       rdm_load,
    output logic       nz_load,
    output logic       addr_sel_pc,
    output logic [1:0] alu_op
);

    state_t state, next_state;

    // State register
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            state <= S_FETCH_1;
        else
            state <= next_state;
    end

    // Next state and output logic
    always_comb begin
        // Default values
        mem_read    = 1'b0;
        mem_write   = 1'b0;
        pc_inc      = 1'b0;
        pc_load     = 1'b0;
        ac_load     = 1'b0;
        ri_load     = 1'b0;
        rem_load    = 1'b0;
        rdm_load    = 1'b0;
        nz_load     = 1'b0;
        addr_sel_pc = 1'b1;
        alu_op      = 2'b00;
        next_state  = state;

        case (state)
            S_FETCH_1: begin
                addr_sel_pc = 1'b1;
                rem_load    = 1'b1;
                mem_read    = 1'b1;
                next_state  = S_FETCH_2;
            end
            // ... additional states
        endcase
    end
endmodule
```

---

## Module Implementation

### Module Hierarchy (16-bit Addressing)

```
tt_um_cpu_leonardoaraujosantos (TinyTapeout wrapper - project.sv)
├── cpu_top (top_cpu_neander_x.sv)
│   ├── neander_datapath (neander_x_datapath.sv)
│   │   ├── pc_reg (16-bit Program Counter)
│   │   ├── sp_reg (16-bit Stack Pointer, reset=0x00FF)
│   │   ├── fp_reg (16-bit Frame Pointer)
│   │   ├── rdm_reg (16-bit, separate lo/hi loading)
│   │   ├── mux_addr (3-way, 16-bit: PC/RDM/SP)
│   │   ├── generic_reg_16 (REM - 16-bit)
│   │   ├── generic_reg (RI, AC - 8-bit)
│   │   ├── x_reg (8-bit X index)
│   │   ├── y_reg (8-bit Y index)
│   │   ├── neander_alu (ADD, ADC, SUB, SBC, MUL, DIV, MOD, AND, OR, XOR, NOT, SHL, SHR, ASR, NEG)
│   │   └── nzc_reg (N, Z, C Flags)
│   └── neander_control (neander_x_control_unit.sv - FSM ~90 states, 9-bit encoding)
│
└── spi_memory_controller (spi_memory_controller.sv)
    ├── SPI Master interface (to external SRAM)
    ├── 16-bit address support (addr[15:0])
    ├── 8-state FSM (IDLE → CMD → ADDR_HI → ADDR_LO → DATA → DONE)
    └── ~70 CPU cycles per memory access
```

### Generic Register Module

```systemverilog
module generic_reg (
    input  logic       clk,
    input  logic       reset,
    input  logic       load,
    input  logic [7:0] data_in,
    output logic [7:0] value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            value <= 8'h00;
        else if (load)
            value <= data_in;
    end
endmodule
```

### Top-Level CPU Module (16-bit Addressing)

```systemverilog
module cpu_top (
    input  logic        clk,
    input  logic        reset,

    // Memory Interface (16-bit address)
    output logic [15:0] mem_addr,       // 16-bit address
    output logic [7:0]  mem_data_out,   // 8-bit data
    input  logic [7:0]  mem_data_in,
    output logic        mem_write,
    output logic        mem_read,
    output logic        mem_req,
    input  logic        mem_ready,

    // I/O Interface
    input  logic [7:0]  io_in,
    output logic        io_write,

    // Debug outputs (16-bit for address registers)
    output logic [15:0] dbg_pc,         // 16-bit PC
    output logic [15:0] dbg_sp,         // 16-bit SP
    output logic [15:0] dbg_fp,         // 16-bit FP
    output logic [7:0]  dbg_ac,
    output logic [7:0]  dbg_ri,
    output logic        halt
);
    // Internal signals
    logic        pc_inc, pc_load;
    logic        ac_load, ri_load, rem_load;
    logic        rdm_load, rdm_load_hi;    // Separate lo/hi loading
    logic        nzc_load;
    logic [1:0]  addr_sel;                 // 3-way mux
    logic [3:0]  alu_op;
    logic [7:0]  opcode;
    logic        flagN, flagZ, flagC;
    logic        sp_inc, sp_dec;
    logic        fp_load, fp_load_lo, fp_load_hi;  // FP loading signals

    // Datapath instantiation
    neander_datapath dp (
        .clk(clk),
        .reset(reset),
        .mem_addr(mem_addr),            // 16-bit
        .rdm_load_hi(rdm_load_hi),      // NEW
        .fp_load_lo(fp_load_lo),        // NEW
        .fp_load_hi(fp_load_hi),        // NEW
        .dbg_pc(dbg_pc),                // 16-bit
        .dbg_sp(dbg_sp),                // 16-bit
        .dbg_fp(dbg_fp),                // 16-bit
        // ... other port connections
    );

    // Control unit instantiation (9-bit state encoding)
    neander_control uc (
        .clk(clk),
        .reset(reset),
        .rdm_load_hi(rdm_load_hi),      // NEW
        .fp_load_lo(fp_load_lo),        // NEW
        .fp_load_hi(fp_load_hi),        // NEW
        .flagC(flagC),                  // Carry flag
        // ... other port connections
    );
endmodule
```

---

## Testing and Verification

### Testbench Structure

```systemverilog
`timescale 1ns/1ps

module tb_cpu;
    logic clk = 0;
    logic reset = 1;
    logic [7:0] pc, ac, ri;

    // Clock generation (100 MHz)
    always #5 clk = ~clk;

    // DUT instantiation
    cpu_top dut (
        .clk(clk),
        .reset(reset),
        .dbg_pc(pc),
        .dbg_ac(ac),
        .dbg_ri(ri)
    );

    initial begin
        // Waveform dump
        $dumpfile("neander_sim.vcd");
        $dumpvars(0, tb_cpu);

        $display("=== Neander CPU Simulation ===");

        // Release reset after 20ns
        #20 reset = 0;

        // Monitor execution
        $monitor("t=%0t | PC=%02h | AC=%02h | RI=%02h",
                 $time, pc, ac, ri);

        // Run simulation
        #2000 $finish;
    end
endmodule
```

### ALU Testbench

```systemverilog
module neander_alu_tb;
    logic [7:0] a, b;
    logic [1:0] alu_op;
    logic [7:0] result;

    neander_alu dut (
        .a(a), .b(b), .alu_op(alu_op), .result(result)
    );

    task automatic check(
        input logic [1:0] op,
        input logic [7:0] av, bv, expected
    );
        alu_op = op; a = av; b = bv;
        #5;
        if (result !== expected)
            $error("FAIL: op=%b a=%02h b=%02h result=%02h expected=%02h",
                   op, av, bv, result, expected);
        else
            $display("PASS: op=%b a=%02h b=%02h -> %02h", op, av, bv, result);
    endtask

    initial begin
        $display("=== ALU Testbench ===");
        check(2'b00, 8'h0A, 8'h05, 8'h0F);  // ADD: 10 + 5 = 15
        check(2'b01, 8'hF0, 8'h0F, 8'h00);  // AND: 0xF0 & 0x0F = 0x00
        check(2'b10, 8'hF0, 8'h0F, 8'hFF);  // OR:  0xF0 | 0x0F = 0xFF
        check(2'b11, 8'h0A, 8'h00, 8'hF5);  // NOT: ~0x0A = 0xF5
        $display("=== ALU Test Complete ===");
        $finish;
    end
endmodule
```

### Verification Checklist

- [ ] ALU operations produce correct results
- [ ] Flags N and Z update correctly
- [ ] PC increments after each instruction byte fetch
- [ ] PC loads correctly on jumps
- [ ] Conditional jumps respect flag conditions
- [ ] Memory read/write operations work correctly
- [ ] Reset initializes all registers to zero
- [ ] HLT instruction halts execution

---

## Example Programs

### Program 1: Addition (X + Y = Z) - 16-bit Addressing

```assembly
; Add two numbers and store result
; X = 10, Y = 20, Z = X + Y = 30

        LDI  10         ; AC = 10
        STA  X          ; MEM[X] = 10
        LDI  20         ; AC = 20
        STA  Y          ; MEM[Y] = 20
        LDA  X          ; AC = MEM[X]
        ADD  Y          ; AC = AC + MEM[Y]
        STA  Z          ; MEM[Z] = AC
        HLT             ; Stop

X:      .equ 0x0080     ; 16-bit address
Y:      .equ 0x0081
Z:      .equ 0x0082
```

**Machine Code (16-bit addressing - 3 bytes for memory ops):**
```
Address  Data    Instruction
0x0000   0xE0    LDI 10
0x0001   0x0A    ; immediate value
0x0002   0x10    STA 0x0080
0x0003   0x80    ; addr_lo
0x0004   0x00    ; addr_hi
0x0005   0xE0    LDI 20
0x0006   0x14    ; immediate value
0x0007   0x10    STA 0x0081
0x0008   0x81    ; addr_lo
0x0009   0x00    ; addr_hi
0x000A   0x20    LDA 0x0080
0x000B   0x80    ; addr_lo
0x000C   0x00    ; addr_hi
0x000D   0x30    ADD 0x0081
0x000E   0x81    ; addr_lo
0x000F   0x00    ; addr_hi
0x0010   0x10    STA 0x0082
0x0011   0x82    ; addr_lo
0x0012   0x00    ; addr_hi
0x0013   0xF0    HLT
```

**Note:** Memory operations now use 3 bytes: `[opcode] [addr_lo] [addr_hi]`. This little-endian format allows full 64KB addressing.

### Program 2: Count Down Loop

```assembly
; Count down from 5 to 0

        LDI  5          ; AC = 5
        STA  COUNT      ; Initialize counter
LOOP:   LDA  COUNT      ; Load counter
        JZ   DONE       ; Exit if zero
        ADD  NEG1       ; Decrement (add -1)
        STA  COUNT      ; Store counter
        JMP  LOOP       ; Repeat
DONE:   HLT

COUNT:  .data 0x80
NEG1:   .data 0x81      ; Contains 0xFF (-1 in two's complement)
```

### Program 3: Logical Operations

```assembly
; Demonstrate AND, OR, NOT

        LDI  0xF0       ; AC = 11110000
        STA  A
        LDI  0x0F       ; AC = 00001111
        STA  B
        LDA  A
        AND  B          ; AC = 00000000
        STA  R_AND
        LDA  A
        OR   B          ; AC = 11111111
        STA  R_OR
        LDA  A
        NOT             ; AC = 00001111
        STA  R_NOT
        HLT

A:      .data 0x80
B:      .data 0x81
R_AND:  .data 0x82
R_OR:   .data 0x83
R_NOT:  .data 0x84
```

---

## References

### Primary Sources

1. **UFRGS Neander Instruction Set**
   - https://www.inf.ufrgs.br/arq/wiki/doku.php?id=insneander

2. **UFRGS Neander Architecture**
   - https://www.inf.ufrgs.br/arq/wiki/doku.php?id=neander

### Additional Resources

3. **WEBER, R. F.** *Fundamentos de Arquitetura de Computadores*
   - Foundation textbook for Brazilian computer architecture courses

4. **Neander Simulator**
   - Version 2.1.2 (2002) - Available from UFRGS

### Project Files

The following SystemVerilog files are available in the `examples/systemverilog/` directory:

| File | Description |
|------|-------------|
| `neander_alu.sv` | Standalone ALU module |
| `neander_alu_tb.sv` | ALU testbench |
| `top_cpu.sv` | Complete CPU (version 1) with embedded RAM |
| `top_cpu_2.sv` | Complete CPU (version 2) with external interfaces |
| `tb_cpu.sv` | CPU testbench |

---

## Implementation Roadmap

### Phase 1: Basic Components
1. Implement and verify the ALU
2. Implement all register modules
3. Implement the address MUX

### Phase 2: Datapath Integration
1. Connect all datapath components
2. Add memory interface
3. Verify datapath with manual control signals

### Phase 3: Control Unit
1. Implement the FSM state register
2. Implement FETCH cycle states
3. Add DECODE logic
4. Implement each instruction's execute states

### Phase 4: Integration and Testing
1. Connect control unit to datapath
2. Run simple test programs
3. Verify all instructions
4. Debug using waveform analysis

### Phase 5: Extensions (Optional)
1. Add I/O instructions (IN, OUT)
2. Add immediate addressing (LDI)
3. Add JNZ instruction
4. Implement interrupt support

---

## Debugging Tips

1. **Use waveform viewers** - VCD files can be viewed with GTKWave
2. **Add debug outputs** - Monitor PC, AC, and state transitions
3. **Test incrementally** - Verify each instruction type before moving on
4. **Check timing** - Ensure control signals are active at the right clock edges
5. **Verify reset behavior** - All registers should initialize to known values

---

*Document Version: 1.0*
*Last Updated: January 2026*
