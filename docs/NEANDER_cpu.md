# Neander CPU Implementation Guide

This document provides a complete guide for implementing the Neander processor in SystemVerilog. The Neander is an educational 8-bit processor developed at UFRGS (Universidade Federal do Rio Grande do Sul) for teaching fundamental computer architecture concepts.

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

### Key Features

- 8-bit data width
- 8-bit address space (256 bytes of memory)
- Single accumulator architecture
- Direct addressing mode only
- Two condition flags (N and Z)
- 11 instructions (including NOP and HLT)

---

## Architecture Specifications

| Parameter | Value |
|-----------|-------|
| Data Width | 8 bits |
| Address Width | 8 bits |
| Memory Size | 256 bytes |
| Number Format | Two's complement |
| Addressing Mode | Direct only |
| Accumulator Count | 1 |
| Condition Flags | 2 (N, Z) |

### Block Diagram

```
                    +------------------+
                    |      Memory      |
                    |    (256 x 8)     |
                    +--------+---------+
                             |
              +--------------+---------------+
              |                              |
              v                              v
         +----+----+                   +-----+-----+
         |   REM   |<---+              |    RDM    |
         +---------+    |              +-----------+
              |         |                    |
              v         |                    v
         +----+----+    |              +-----+-----+
         |   MUX   |----+              |    RI     |
         +---------+                   +-----------+
              ^                              |
              |                              v
         +----+----+                   +-----+-----+
         |   PC    |                   |  Decoder  |
         +---------+                   +-----------+
                                             |
              +------------------------------+
              |
              v
         +----+----+     +----------+
         |   AC    |<----|   ALU    |
         +---------+     +----------+
              |
              v
         +----+----+
         |  N | Z  |
         +---------+
```

---

## Instruction Set Architecture

### Instruction Format

All instructions follow a simple two-byte format:

```
Byte 0: [Opcode (4 bits)] [Don't Care (4 bits)]
Byte 1: [Address/Operand (8 bits)]
```

For instructions without operands (NOP, NOT, HLT), only the first byte is used.

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

### Extended Instructions (Optional)

The following instructions can be added to enhance the processor:

| Opcode (Hex) | Mnemonic | Operation | Description |
|--------------|----------|-----------|-------------|
| 0xB | JNZ addr | IF Z=0 THEN PC <- addr | Jump if not zero |
| 0xC | IN port | AC <- IO[port] | Input from I/O port |
| 0xD | OUT port | IO[port] <- AC | Output to I/O port |
| 0xE | LDI imm | AC <- imm | Load immediate value |

### Instruction Encoding Examples

```
LDA 0x80    -> 0x20 0x80    ; Load from address 128
ADD 0x81    -> 0x30 0x81    ; Add value at address 129
STA 0x82    -> 0x10 0x82    ; Store at address 130
JMP 0x00    -> 0x80 0x00    ; Jump to address 0
NOT         -> 0x60 0x00    ; Complement accumulator
HLT         -> 0xF0 0x00    ; Halt
```

---

## Register Set

### Program Counter (PC)

- **Width**: 8 bits
- **Function**: Holds the address of the next instruction to fetch
- **Operations**: Increment, Load (for jumps)
- **Reset Value**: 0x00

```systemverilog
module pc_reg (
    input  logic       clk,
    input  logic       reset,
    input  logic       pc_inc,
    input  logic       pc_load,
    input  logic [7:0] data_in,
    output logic [7:0] pc_value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            pc_value <= 8'h00;
        else if (pc_load)
            pc_value <= data_in;
        else if (pc_inc)
            pc_value <= pc_value + 8'h01;
    end
endmodule
```

### Accumulator (AC)

- **Width**: 8 bits
- **Function**: Main data register for arithmetic and logic operations
- **Operations**: Load from ALU result or memory
- **Reset Value**: 0x00

### Memory Address Register (REM)

- **Width**: 8 bits
- **Function**: Holds the address for memory access
- **Source**: PC (instruction fetch) or RDM (operand address)

### Memory Data Register (RDM)

- **Width**: 8 bits
- **Function**: Holds data read from memory
- **Usage**: Temporarily stores instruction operands

### Instruction Register (RI)

- **Width**: 8 bits
- **Function**: Holds the current instruction opcode
- **Opcode Extraction**: Upper 4 bits (RI[7:4])

### Condition Flags (N, Z)

- **N (Negative)**: Set when AC[7] = 1 (MSB indicates negative in two's complement)
- **Z (Zero)**: Set when AC = 0x00

```systemverilog
module nz_reg (
    input  logic clk,
    input  logic reset,
    input  logic nz_load,
    input  logic N_in,
    input  logic Z_in,
    output logic N_flag,
    output logic Z_flag
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            N_flag <= 1'b0;
            Z_flag <= 1'b0;
        end
        else if (nz_load) begin
            N_flag <= N_in;
            Z_flag <= Z_in;
        end
    end
endmodule
```

---

## Memory Organization

### Address Space

```
+----------+------------------+
| Address  | Usage            |
+----------+------------------+
| 0x00-0x7F| Program Code     |
| 0x80-0xFF| Data/Variables   |
+----------+------------------+
```

### Memory Module

The memory is implemented as a 256x8 asynchronous RAM:

```systemverilog
module ram_256x8 (
    input  logic        clk,
    input  logic        mem_write,
    input  logic [7:0]  addr,
    input  logic [7:0]  data_in,
    output logic [7:0]  data_out
);
    logic [7:0] mem [0:255];

    // Asynchronous read
    assign data_out = mem[addr];

    // Synchronous write
    always_ff @(posedge clk) begin
        if (mem_write)
            mem[addr] <= data_in;
    end
endmodule
```

---

## Datapath Design

### Control Signals

| Signal | Description |
|--------|-------------|
| `mem_read` | Enable memory read operation |
| `mem_write` | Enable memory write operation |
| `pc_inc` | Increment program counter |
| `pc_load` | Load program counter (jump) |
| `ac_load` | Load accumulator |
| `ri_load` | Load instruction register |
| `rem_load` | Load memory address register |
| `rdm_load` | Load memory data register |
| `nz_load` | Update condition flags |
| `addr_sel_pc` | MUX select: 1=PC, 0=RDM |
| `alu_op[1:0]` | ALU operation select |

### Address MUX

Selects between PC (for instruction fetch) and RDM (for operand access):

```systemverilog
module mux_pc_rdm (
    input  logic       sel,    // 1 = PC, 0 = RDM
    input  logic [7:0] pc,
    input  logic [7:0] rdm,
    output logic [7:0] out
);
    assign out = sel ? pc : rdm;
endmodule
```

### Datapath Module

```systemverilog
module neander_datapath (
    input  logic       clk,
    input  logic       reset,

    // Control Signals
    input  logic       mem_read,
    input  logic       mem_write,
    input  logic       pc_inc,
    input  logic       pc_load,
    input  logic       ac_load,
    input  logic       ri_load,
    input  logic       rem_load,
    input  logic       rdm_load,
    input  logic       nz_load,
    input  logic       addr_sel_pc,
    input  logic [1:0] alu_op,

    // Memory Interface
    input  logic [7:0] mem_data_in,
    output logic [7:0] mem_addr,
    output logic [7:0] mem_data_out,

    // Control Unit Interface
    output logic [3:0] opcode,
    output logic       flagN,
    output logic       flagZ,

    // Debug
    output logic [7:0] dbg_pc,
    output logic [7:0] dbg_ac
);
    // Internal signals
    logic [7:0] pc, rem, rdm, ri, ac;
    logic [7:0] alu_res, addr_mux, ac_in;
    logic       N_in, Z_in;

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

1. **FETCH**: Load instruction from memory
2. **DECODE**: Determine instruction type
3. **EXECUTE**: Perform the operation (instruction-specific states)

### State Definitions

```systemverilog
typedef enum logic [5:0] {
    // Fetch states
    S_FETCH_1,      // PC -> REM, initiate read
    S_FETCH_2,      // Read complete, load RDM
    S_FETCH_3,      // RDM -> RI, increment PC
    S_DECODE,       // Decode opcode

    // LDA states
    S_LDA_1,        // Fetch operand address
    S_LDA_2,        // Load address to RDM
    S_LDA_3,        // RDM -> REM
    S_LDA_4,        // Read data to AC

    // STA states
    S_STA_1,        // Fetch operand address
    S_STA_2,        // Load address to RDM
    S_STA_3,        // RDM -> REM
    S_STA_4,        // Write AC to memory

    // ADD/AND/OR states (similar pattern)
    S_ADD_1, S_ADD_2, S_ADD_3, S_ADD_4,
    S_AND_1, S_AND_2, S_AND_3, S_AND_4,
    S_OR_1,  S_OR_2,  S_OR_3,  S_OR_4,

    // NOT state
    S_NOT,          // Single cycle operation

    // Jump states
    S_JMP_1, S_JMP_2, S_JMP_3,
    S_JN_1,  S_JN_2,  S_JN_3,
    S_JZ_1,  S_JZ_2,  S_JZ_3,
    S_JNZ_1, S_JNZ_2, S_JNZ_3,

    // Extended instructions
    S_LDI_1, S_LDI_2,
    S_IN_1,  S_IN_2,  S_IN_3,
    S_OUT_1, S_OUT_2, S_OUT_3,

    // Halt
    S_HLT
} state_t;
```

### Instruction Execution Cycles

#### FETCH Cycle (All Instructions)

```
S_FETCH_1: addr_sel_pc=1, rem_load=1, mem_read=1
S_FETCH_2: mem_read=1, rdm_load=1
S_FETCH_3: ri_load=1, pc_inc=1
S_DECODE:  (decode opcode)
```

#### LDA (Load Accumulator)

```
S_LDA_1: addr_sel_pc=1, rem_load=1, mem_read=1  ; Fetch address byte
S_LDA_2: mem_read=1, rdm_load=1, pc_inc=1       ; Load address to RDM
S_LDA_3: addr_sel_pc=0, rem_load=1, mem_read=1  ; RDM -> REM
S_LDA_4: mem_read=1, ac_load=1, nz_load=1       ; Load data to AC
```

#### STA (Store Accumulator)

```
S_STA_1: addr_sel_pc=1, rem_load=1, mem_read=1  ; Fetch address byte
S_STA_2: mem_read=1, rdm_load=1, pc_inc=1       ; Load address to RDM
S_STA_3: addr_sel_pc=0, rem_load=1              ; RDM -> REM
S_STA_4: mem_write=1                            ; Write AC to memory
```

#### ADD/AND/OR (Arithmetic/Logic)

```
S_xxx_1: addr_sel_pc=1, rem_load=1, mem_read=1  ; Fetch address byte
S_xxx_2: mem_read=1, rdm_load=1, pc_inc=1       ; Load address to RDM
S_xxx_3: addr_sel_pc=0, rem_load=1, mem_read=1  ; RDM -> REM
S_xxx_4: mem_read=1, ac_load=1, alu_op=xx, nz_load=1  ; Perform operation
```

#### NOT

```
S_NOT: ac_load=1, alu_op=2'b11, nz_load=1       ; Single cycle
```

#### JMP (Unconditional Jump)

```
S_JMP_1: addr_sel_pc=1, rem_load=1, mem_read=1  ; Fetch address byte
S_JMP_2: mem_read=1, rdm_load=1                 ; Load address to RDM
S_JMP_3: pc_load=1                              ; RDM -> PC
```

#### JN/JZ (Conditional Jumps)

```
S_Jx_1: addr_sel_pc=1, rem_load=1, mem_read=1   ; Fetch address byte
S_Jx_2: mem_read=1, rdm_load=1                  ; Load address to RDM
S_Jx_3: if(condition) pc_load=1 else pc_inc=1   ; Conditional branch
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

### Module Hierarchy

```
cpu_top
├── neander_datapath
│   ├── pc_reg
│   ├── mux_pc_rdm
│   ├── generic_reg (REM)
│   ├── generic_reg (RDM)
│   ├── generic_reg (RI)
│   ├── generic_reg (AC)
│   ├── neander_alu
│   └── nz_reg
├── neander_control
└── ram_256x8 (external)
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

### Top-Level CPU Module

```systemverilog
module cpu_top (
    input  logic       clk,
    input  logic       reset,

    // Memory Interface
    output logic [7:0] mem_addr,
    output logic [7:0] mem_data_out,
    input  logic [7:0] mem_data_in,
    output logic       mem_write,

    // Debug outputs
    output logic [7:0] dbg_pc,
    output logic [7:0] dbg_ac,
    output logic [7:0] dbg_ri
);
    // Internal signals
    logic       mem_read;
    logic       pc_inc, pc_load;
    logic       ac_load, ri_load, rem_load, rdm_load, nz_load;
    logic       addr_sel_pc;
    logic [1:0] alu_op;
    logic [3:0] opcode;
    logic       flagN, flagZ;

    // Datapath instantiation
    neander_datapath dp (
        .clk(clk),
        .reset(reset),
        // ... port connections
    );

    // Control unit instantiation
    neander_control uc (
        .clk(clk),
        .reset(reset),
        // ... port connections
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

### Program 1: Addition (X + Y = Z)

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

X:      .data 0x80
Y:      .data 0x81
Z:      .data 0x82
```

**Machine Code:**
```
Address  Data    Instruction
0x00     0xE0    LDI
0x01     0x0A    10
0x02     0x10    STA
0x03     0x80    X
0x04     0xE0    LDI
0x05     0x14    20
0x06     0x10    STA
0x07     0x81    Y
0x08     0x20    LDA
0x09     0x80    X
0x0A     0x30    ADD
0x0B     0x81    Y
0x0C     0x10    STA
0x0D     0x82    Z
0x0E     0xF0    HLT
```

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
