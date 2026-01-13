# NEANDER-X Stack Extension

This document describes the stack extension for the NEANDER-X processor, adding PUSH, POP, CALL, and RET instructions to enable subroutine support and stack-based data management.

**16-bit Addressing Update:** With 16-bit addressing, the Stack Pointer (SP) and Frame Pointer (FP) are now **16-bit registers**, and CALL/RET operations push/pop **16-bit return addresses**.

## Table of Contents

1. [Overview](#overview)
2. [Differences from Vanilla NEANDER-X](#differences-from-vanilla-neander-x)
3. [Stack Architecture](#stack-architecture)
4. [New Instructions](#new-instructions)
5. [Hardware Changes](#hardware-changes)
6. [Instruction Encoding](#instruction-encoding)
7. [Example Programs](#example-programs)
8. [Testing](#testing)

---

## Overview

The NEANDER-X Stack Extension adds four new instructions using opcode 0x7 with sub-opcodes in the lower nibble:

| Instruction | Encoding | Operation |
|-------------|----------|-----------|
| PUSH | 0x70 | Push AC (8-bit) onto stack |
| POP | 0x71 | Pop from stack to AC (8-bit) |
| CALL addr | 0x72 addr_lo addr_hi | Call subroutine (pushes 16-bit return address) |
| RET | 0x73 | Return from subroutine (pops 16-bit return address) |

These instructions enable:
- **Data stack operations** - Save/restore 8-bit values during computation
- **Subroutine calls** - Modular code with 16-bit target addresses (full 64KB range)
- **Nested function calls** - Functions calling other functions with 16-bit return addresses
- **Local variable storage** - Temporary storage on stack

---

## Differences from Vanilla NEANDER-X

### Vanilla NEANDER-X

The original NEANDER-X (as documented in NEANDER_cpu.md) includes these extended instructions beyond the basic NEANDER:

| Opcode | Instruction | Description |
|--------|-------------|-------------|
| 0xB | JNZ addr | Jump if not zero |
| 0xC | IN port | Input from I/O port |
| 0xD | OUT port | Output to I/O port |
| 0xE | LDI imm | Load immediate value |

Opcode 0x7 was **reserved/unused** in vanilla NEANDER-X.

### Stack-Extended NEANDER-X

This extension uses the previously unused opcode 0x7 with sub-opcodes:

| Encoding | Instruction | Description |
|----------|-------------|-------------|
| 0x70 | PUSH | Push AC onto stack |
| 0x71 | POP | Pop from stack to AC |
| 0x72 addr | CALL addr | Call subroutine at address |
| 0x73 | RET | Return from subroutine |

### Backward Compatibility

The stack extension is **fully backward compatible** with vanilla NEANDER-X:
- All existing instructions retain their opcodes and behavior
- Programs written for vanilla NEANDER-X run unchanged
- Opcode 0x7 was previously undefined, so no conflicts exist
- NOP (0x00) remains unchanged

---

## Stack Architecture

### Stack Pointer (SP) - 16-bit

The Stack Pointer is now a **16-bit register** to support the full 64KB address space:

| Property | Value |
|----------|-------|
| Width | **16 bits** |
| Reset Value | **0x00FF** (top of page 0, backward compatible) |
| Growth Direction | Downward (decrement on push) |
| Operations | Increment, Decrement |
| Address Range | 0x0000 - 0xFFFF |

### Memory Layout (64KB)

```
+------------------+------------------+
| Address Range    | Usage            |
+------------------+------------------+
| 0x0000-0x00FF    | Page 0 (stack grows down from 0x00FF) |
| 0x0100-0x7FFF    | Program Code / Data |
| 0x8000-0xFFFF    | Extended Data / Large Programs |
+------------------+------------------+
         ^
         |
    SP starts at 0x00FF,
    grows downward toward 0x0000
```

### Stack Operation Model

**Push (pre-decrement):**
```
SP = SP - 1      ; Decrement 16-bit SP first
MEM[SP] = data   ; Then write 8-bit data
```

**Pop (post-increment):**
```
data = MEM[SP]   ; Read 8-bit data first
SP = SP + 1      ; Then increment 16-bit SP
```

**CALL (16-bit return address):**
```
SP = SP - 1
MEM[SP] = PC[15:8]   ; Push PC high byte first
SP = SP - 1
MEM[SP] = PC[7:0]    ; Push PC low byte second
PC = target_addr     ; Jump to 16-bit target
```

**RET (16-bit return address):**
```
PC[7:0] = MEM[SP]    ; Pop PC low byte first
SP = SP + 1
PC[15:8] = MEM[SP]   ; Pop PC high byte second
SP = SP + 1
```

This model allows the stack to grow from 0x00FF toward 0x0000, with SP always pointing to the top (most recently pushed) item. Each CALL/RET uses 2 bytes of stack space for the 16-bit return address.

---

## New Instructions

### PUSH (0x70)

Push the accumulator onto the stack.

**Encoding:** `0x70 0xXX` (second byte ignored)

**Operation:**
```
SP = SP - 1
MEM[SP] = AC
```

**Flags Affected:** None

**Cycles:** 3 states (S_PUSH_1, S_PUSH_2, S_PUSH_3)

**FSM States:**
```
S_PUSH_1: sp_dec = 1                    ; Decrement SP
S_PUSH_2: addr_sel = 2'b10, rem_load = 1 ; SP -> REM
S_PUSH_3: mem_write = 1                  ; Write AC to [REM]
```

### POP (0x71)

Pop from the stack into the accumulator.

**Encoding:** `0x71 0xXX` (second byte ignored)

**Operation:**
```
AC = MEM[SP]
SP = SP + 1
Update N, Z flags
```

**Flags Affected:** N, Z (based on popped value)

**Cycles:** 3 states (S_POP_1, S_POP_2, S_POP_3)

**FSM States:**
```
S_POP_1: addr_sel = 2'b10, rem_load = 1  ; SP -> REM
S_POP_2: mem_read = 1                     ; Read [REM]
S_POP_3: ac_load = 1, nz_load = 1, sp_inc = 1 ; Load AC, update flags, increment SP
```

### CALL (0x72 addr_lo addr_hi) - 16-bit Target Address

Call a subroutine at the specified 16-bit address.

**Encoding:** `0x72 addr_lo addr_hi` (3 bytes, little-endian)

**Operation:**
```
; Fetch 16-bit target address to RDM
RDM[7:0]  = MEM[PC]; PC++   ; Fetch addr_lo
RDM[15:8] = MEM[PC]; PC++   ; Fetch addr_hi
; Push 16-bit return address onto stack (2 bytes)
SP = SP - 1
MEM[SP] = PC[15:8]          ; Push PC high byte
SP = SP - 1
MEM[SP] = PC[7:0]           ; Push PC low byte
; Jump to subroutine
PC = RDM                    ; Load 16-bit target address
```

**Flags Affected:** None

**Cycles:** 9 states (S_CALL_1 through S_CALL_7)

**FSM States:**
```
S_CALL_1:  addr_sel=01(PC), rem_load=1, mem_read=1   ; PC -> REM
S_CALL_2:  rdm_load=1, pc_inc=1                      ; Fetch addr_lo to RDM[7:0]
S_CALL_2B: addr_sel=01(PC), rem_load=1, mem_read=1   ; PC -> REM
S_CALL_2C: rdm_load_hi=1, pc_inc=1                   ; Fetch addr_hi to RDM[15:8]
S_CALL_3:  sp_dec=1                                  ; SP--
S_CALL_4:  addr_sel=10(SP), rem_load=1, mem_write=1  ; Push PC[15:8]
           mem_data_sel=PC_HI
S_CALL_5:  sp_dec=1                                  ; SP--
S_CALL_6:  addr_sel=10(SP), rem_load=1, mem_write=1  ; Push PC[7:0]
           mem_data_sel=PC_LO
S_CALL_7:  pc_load=1                                 ; PC <- RDM (16-bit)
```

**Stack Usage:** Each CALL consumes **2 bytes** of stack space for the 16-bit return address.

### RET (0x73) - 16-bit Return Address

Return from a subroutine by popping a 16-bit return address.

**Encoding:** `0x73` (1 byte)

**Operation:**
```
; Pop 16-bit return address from stack (2 bytes)
PC[7:0]  = MEM[SP]; SP++    ; Pop PC low byte
PC[15:8] = MEM[SP]; SP++    ; Pop PC high byte
; Resume execution at return address
```

**Flags Affected:** None

**Cycles:** 6 states (S_RET_1 through S_RET_6)

**FSM States:**
```
S_RET_1: addr_sel=10(SP), rem_load=1, mem_read=1   ; SP -> REM
S_RET_2: rdm_load=1, sp_inc=1                      ; Pop PC[7:0] to RDM[7:0], SP++
S_RET_3: addr_sel=10(SP), rem_load=1, mem_read=1   ; SP -> REM
S_RET_4: rdm_load_hi=1, sp_inc=1                   ; Pop PC[15:8] to RDM[15:8], SP++
S_RET_5: pc_load=1                                 ; PC <- RDM (16-bit)
S_RET_6: (return to FETCH)
```

**Stack Usage:** Each RET restores **2 bytes** from the stack.

---

## Hardware Changes

### New Registers (16-bit)

**Stack Pointer (SP) - 16-bit:**
```systemverilog
module sp_reg (
    input  logic        clk,
    input  logic        reset,
    input  logic        sp_inc,
    input  logic        sp_dec,
    input  logic        sp_load,
    input  logic [15:0] data_in,
    output logic [15:0] sp_value    // 16-bit
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            sp_value <= 16'h00FF;   // Stack starts at 0x00FF (backward compatible)
        else if (sp_load)
            sp_value <= data_in;
        else if (sp_dec)
            sp_value <= sp_value - 16'h0001;
        else if (sp_inc)
            sp_value <= sp_value + 16'h0001;
    end
endmodule
```

**Frame Pointer (FP) - 16-bit:**
```systemverilog
module fp_reg (
    input  logic        clk,
    input  logic        reset,
    input  logic        fp_load,      // Full 16-bit load from SP (TSF)
    input  logic        fp_load_lo,   // Load low byte (POP_FP step 1)
    input  logic        fp_load_hi,   // Load high byte (POP_FP step 2)
    input  logic [15:0] data_in,      // 16-bit input (from SP or memory)
    input  logic [7:0]  data_in_byte, // 8-bit input (from memory)
    output logic [15:0] fp_value      // 16-bit
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            fp_value <= 16'h0000;
        else if (fp_load)
            fp_value <= data_in;      // Full 16-bit load
        else if (fp_load_lo)
            fp_value[7:0] <= data_in_byte;   // Load low byte
        else if (fp_load_hi)
            fp_value[15:8] <= data_in_byte;  // Load high byte
    end
endmodule
```

### Extended Address MUX (16-bit, 3-way)

The address multiplexer handles 16-bit addresses:

```systemverilog
// addr_sel[1:0]: 00 = RDM, 01 = PC, 10 = SP
module mux_addr (
    input  logic [1:0]  sel,
    input  logic [15:0] pc,      // 16-bit
    input  logic [15:0] rdm,     // 16-bit
    input  logic [15:0] sp,      // 16-bit
    output logic [15:0] out      // 16-bit
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

### Memory Data MUX (Extended for 16-bit PC)

The memory data multiplexer now supports writing individual bytes of 16-bit registers:

```systemverilog
// mem_data_sel_ext[2:0]: Extended for 16-bit operations
//   000 = AC     (for STA, PUSH)
//   001 = PC_LO  (for CALL - push return address low byte)
//   010 = X      (for STX)
//   011 = Y      (for STY)
//   100 = FP_LO  (for PUSH_FP - push FP low byte)
//   101 = PC_HI  (for CALL - push return address high byte)
//   110 = FP_HI  (for PUSH_FP - push FP high byte)
always_comb begin
    case (mem_data_sel_ext)
        3'b000:  mem_data_out = ac;
        3'b001:  mem_data_out = pc[7:0];    // PC low byte
        3'b010:  mem_data_out = x;
        3'b011:  mem_data_out = y;
        3'b100:  mem_data_out = fp[7:0];    // FP low byte
        3'b101:  mem_data_out = pc[15:8];   // PC high byte
        3'b110:  mem_data_out = fp[15:8];   // FP high byte
        default: mem_data_out = ac;
    endcase
end
```

### New Control Signals (16-bit Support)

| Signal | Width | Description |
|--------|-------|-------------|
| `addr_sel` | 2 bits | Address source: 00=RDM, 01=PC, 10=SP (all 16-bit) |
| `sp_inc` | 1 bit | Increment 16-bit stack pointer (POP, RET) |
| `sp_dec` | 1 bit | Decrement 16-bit stack pointer (PUSH, CALL) |
| `rdm_load_hi` | 1 bit | Load RDM high byte (16-bit address fetch) - **NEW** |
| `fp_load_lo` | 1 bit | Load FP low byte (POP_FP) - **NEW** |
| `fp_load_hi` | 1 bit | Load FP high byte (POP_FP) - **NEW** |
| `mem_data_sel_ext` | 3 bits | Extended: AC, PC_LO, PC_HI, X, Y, FP_LO, FP_HI |
| `sub_opcode` | 4 bits | Lower nibble of RI for stack ops |

### Updated Module Hierarchy (16-bit)

```
tt_um_cpu_leonardoaraujosantos (TinyTapeout wrapper)
├── cpu_top
│   ├── neander_datapath
│   │   ├── pc_reg (16-bit)
│   │   ├── sp_reg (16-bit, reset=0x00FF)
│   │   ├── fp_reg (16-bit, separate lo/hi loading)
│   │   ├── rdm_reg (16-bit, separate lo/hi loading)
│   │   ├── mux_addr (3-way, 16-bit)
│   │   ├── generic_reg_16 (REM - 16-bit)
│   │   ├── generic_reg (RI, AC - 8-bit)
│   │   ├── x_reg (8-bit)
│   │   ├── y_reg (8-bit)
│   │   ├── neander_alu
│   │   └── nzc_reg (N, Z, C flags)
│   └── neander_control (FSM ~90 states, 9-bit encoding)
│
└── spi_memory_controller (16-bit address support)
```

---

## Instruction Encoding

### Opcode 0x7 Sub-opcodes

The stack instructions share opcode 0x7 (upper nibble) with the specific operation encoded in the lower nibble:

```
Byte 0: [0111] [sub-opcode]
        ^^^^   ^^^^
        0x7    operation

Sub-opcodes:
  0x0 = PUSH
  0x1 = POP
  0x2 = CALL
  0x3 = RET
  0x4-0xF = Reserved
```

### Machine Code Examples

```
PUSH        -> 0x70 0x00    ; Push AC (second byte ignored)
POP         -> 0x71 0x00    ; Pop to AC (second byte ignored)
CALL 0x20   -> 0x72 0x20    ; Call subroutine at 0x20
RET         -> 0x73 0x00    ; Return (second byte ignored)
```

---

## Example Programs

### Example 1: Simple PUSH/POP

Save and restore a value:

```assembly
        LDI  0x42       ; AC = 0x42
        PUSH            ; Save AC on stack
        LDI  0x00       ; AC = 0 (clear)
        POP             ; Restore AC = 0x42
        OUT  0          ; Output 0x42
        HLT
```

**Machine Code:**
```
0x00: 0xE0 0x42    ; LDI 0x42
0x02: 0x70 0x00    ; PUSH
0x04: 0xE0 0x00    ; LDI 0x00
0x06: 0x71 0x00    ; POP
0x08: 0xD0 0x00    ; OUT 0
0x0A: 0xF0 0x00    ; HLT
```

### Example 2: Multiple PUSH/POP (LIFO Order)

Demonstrate Last-In-First-Out behavior:

```assembly
        LDI  0x11       ; Push three values
        PUSH
        LDI  0x22
        PUSH
        LDI  0x33
        PUSH

        POP             ; Pop in reverse order
        OUT  0          ; Output 0x33
        POP
        OUT  0          ; Output 0x22
        POP
        OUT  0          ; Output 0x11
        HLT
```

### Example 3: Simple Subroutine

Call a subroutine that adds 5 to AC:

```assembly
; Main program
        LDI  10         ; AC = 10
        CALL add5       ; Call subroutine
        OUT  0          ; Output 15
        HLT

; Subroutine: add5
; Input: AC
; Output: AC = AC + 5
add5:   ADD  five       ; AC = AC + 5
        RET             ; Return

; Data
five:   .byte 5
```

**Machine Code:**
```
; Main at 0x00
0x00: 0xE0 0x0A    ; LDI 10
0x02: 0x72 0x0A    ; CALL 0x0A (add5)
0x04: 0xD0 0x00    ; OUT 0
0x06: 0xF0 0x00    ; HLT

; Padding
0x08: 0x00 0x00

; Subroutine at 0x0A
0x0A: 0x30 0x0E    ; ADD 0x0E (five)
0x0C: 0x73 0x00    ; RET

; Data at 0x0E
0x0E: 0x05        ; five = 5
```

### Example 4: Nested Subroutine Calls

Demonstrate nested calls (main -> sub1 -> sub2):

```assembly
; Main program
main:   CALL sub1       ; Call sub1
        OUT  0          ; Output result
        HLT

; Subroutine 1: calls sub2
sub1:   LDI  0
        CALL sub2       ; Nested call
        RET

; Subroutine 2: loads value
sub2:   LDI  0xAA
        RET
```

### Example 5: Loop with Function Call

Calculate f(x) = x + 5 four times using a loop:

```assembly
; Main: result = 0, counter = 4
; loop: result = add5(result), counter--, if counter != 0 goto loop
; OUT result

        LDI  0
        STA  result     ; result = 0
        LDI  4
        STA  counter    ; counter = 4

loop:   LDA  result
        CALL add5       ; AC = result + 5
        STA  result
        LDA  counter
        ADD  neg1       ; counter--
        STA  counter
        JNZ  loop       ; if counter != 0, loop

        LDA  result
        OUT  0          ; Output 20 (0+5+5+5+5)
        HLT

; Subroutine
add5:   ADD  five
        RET

; Data
result:  .byte 0
counter: .byte 0
neg1:    .byte 0xFF     ; -1 in two's complement
five:    .byte 5
```

### Example 6: Saving Registers in Subroutine

Use PUSH/POP to preserve AC across subroutine work:

```assembly
; Main
        LDI  0xAA       ; Important value in AC
        CALL work       ; Call subroutine (preserves AC)
        OUT  0          ; Output 0xAA (preserved)
        HLT

; Subroutine that does work but preserves AC
work:   PUSH            ; Save caller's AC
        LDI  0x55       ; Do some work with different value
        ; ... more work ...
        POP             ; Restore caller's AC
        RET
```

---

## Testing

### Test Coverage

The stack extension is verified by the following tests:

**cocotb_tests/test_neander.py:**
- `test_push_pop_single` - Basic PUSH/POP
- `test_push_pop_multiple` - LIFO order verification
- `test_push_pop_flags` - N flag after POP
- `test_push_pop_zero_flag` - Z flag after POP
- `test_stack_with_subroutine_simulation` - Save/restore pattern
- `test_call_ret_simple` - Basic CALL/RET
- `test_call_ret_with_parameter` - Parameter passing via AC
- `test_nested_calls` - 2-level nested calls
- `test_call_ret_preserves_stack` - CALL/RET with PUSH/POP
- `test_multiple_calls_same_subroutine` - Repeated calls
- `test_loop_with_function_call` - Loop calling f(x) = x + 5

**test/test.py (TinyTapeout):**
- `test_neander_push_pop` - Basic PUSH/POP
- `test_neander_push_pop_multiple` - LIFO verification
- `test_neander_call_ret` - Basic CALL/RET
- `test_neander_nested_calls` - Nested subroutines
- `test_loop_with_function_call` - Multiple function calls

### Running Tests

```bash
# Run cocotb tests
cd cocotb_tests
make

# Run TinyTapeout tests
cd test
make
```

---

## Summary

The NEANDER-X Stack Extension provides essential subroutine and stack support with **full 16-bit addressing**:

| Feature | Original 8-bit | 16-bit Extended |
|---------|----------------|-----------------|
| Stack Pointer (SP) | 8-bit | **16-bit** |
| Frame Pointer (FP) | 8-bit | **16-bit** |
| Return Address | 8-bit (1 byte on stack) | **16-bit (2 bytes on stack)** |
| CALL Target | 8-bit address | **16-bit address (3-byte instruction)** |
| Address MUX | 2-way (PC/RDM) | **3-way (PC/RDM/SP), 16-bit** |
| Address Range | 256 bytes | **64KB** |
| FSM States for CALL | 5 states | **9 states** |
| FSM States for RET | 3 states | **6 states** |

### Stack Usage Comparison

| Operation | 8-bit Version | 16-bit Version |
|-----------|---------------|----------------|
| PUSH AC | 1 byte | 1 byte (unchanged) |
| POP AC | 1 byte | 1 byte (unchanged) |
| CALL | 1 byte (8-bit return addr) | **2 bytes (16-bit return addr)** |
| RET | 1 byte | **2 bytes** |
| PUSH_FP | 1 byte | **2 bytes (16-bit FP)** |
| POP_FP | 1 byte | **2 bytes (16-bit FP)** |

This extension enables modular programming with reusable subroutines across the full 64KB address space, making the NEANDER-X suitable for larger programs and C compiler support.

---

*Document Version: 2.0 (16-bit Addressing)*
*Last Updated: January 2026*
