# NEANDER-X Stack Extension

This document describes the stack extension for the NEANDER-X processor, adding PUSH, POP, CALL, and RET instructions to enable subroutine support and stack-based data management.

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
| PUSH | 0x70 | Push AC onto stack |
| POP | 0x71 | Pop from stack to AC |
| CALL addr | 0x72 addr | Call subroutine |
| RET | 0x73 | Return from subroutine |

These instructions enable:
- **Data stack operations** - Save/restore values during computation
- **Subroutine calls** - Modular code with reusable functions
- **Nested function calls** - Functions calling other functions
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

### Stack Pointer (SP)

A new 8-bit register manages the stack:

| Property | Value |
|----------|-------|
| Width | 8 bits |
| Reset Value | 0xFF |
| Growth Direction | Downward (decrement on push) |
| Operations | Increment, Decrement |

### Memory Layout

```
+----------+------------------+
| Address  | Usage            |
+----------+------------------+
| 0x00-0x7F| Program Code     |
| 0x80-0xEF| Data/Variables   |
| 0xF0-0xFF| Stack (16 bytes) |
+----------+------------------+
         ^
         |
    SP starts at 0xFF,
    grows downward
```

### Stack Operation Model

**Push (pre-decrement):**
```
SP = SP - 1      ; Decrement first
MEM[SP] = data   ; Then write
```

**Pop (post-increment):**
```
data = MEM[SP]   ; Read first
SP = SP + 1      ; Then increment
```

This model allows the stack to grow from high memory (0xFF) toward low memory, with SP always pointing to the top (most recently pushed) item.

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

### CALL (0x72 addr)

Call a subroutine at the specified address.

**Encoding:** `0x72 addr`

**Operation:**
```
; Fetch target address to RDM
; Push return address (PC after CALL) onto stack
SP = SP - 1
MEM[SP] = PC
; Jump to subroutine
PC = target_addr
```

**Flags Affected:** None

**Cycles:** 5 states (S_CALL_1 through S_CALL_5)

**FSM States:**
```
S_CALL_1: addr_sel = 2'b01, rem_load = 1, mem_read = 1  ; Fetch target addr
S_CALL_2: rdm_load = 1, pc_inc = 1                       ; Load target to RDM, PC = return addr
S_CALL_3: sp_dec = 1                                     ; Decrement SP
S_CALL_4: addr_sel = 2'b10, rem_load = 1                 ; SP -> REM
S_CALL_5: mem_write = 1, mem_data_sel = 1, pc_load = 1   ; Write PC to stack, jump to target
```

### RET (0x73)

Return from a subroutine.

**Encoding:** `0x73 0xXX` (second byte ignored)

**Operation:**
```
PC = MEM[SP]
SP = SP + 1
```

**Flags Affected:** None

**Cycles:** 3 states (S_RET_1, S_RET_2, S_RET_3)

**FSM States:**
```
S_RET_1: addr_sel = 2'b10, rem_load = 1  ; SP -> REM
S_RET_2: mem_read = 1, rdm_load = 1       ; Read return address to RDM
S_RET_3: pc_load = 1, sp_inc = 1          ; Load PC from RDM, increment SP
```

---

## Hardware Changes

### New Registers

**Stack Pointer (SP):**
```systemverilog
module sp_reg (
    input  logic       clk,
    input  logic       reset,
    input  logic       sp_inc,
    input  logic       sp_dec,
    input  logic       sp_load,
    input  logic [7:0] data_in,
    output logic [7:0] sp_value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            sp_value <= 8'hFF;  // Stack starts at top of memory
        else if (sp_load)
            sp_value <= data_in;
        else if (sp_dec)
            sp_value <= sp_value - 8'h01;
        else if (sp_inc)
            sp_value <= sp_value + 8'h01;
    end
endmodule
```

### Extended Address MUX

The address multiplexer is extended from 2-way to 3-way:

**Before (vanilla NEANDER-X):**
```systemverilog
// addr_sel_pc: 1 = PC, 0 = RDM
assign addr_mux = addr_sel_pc ? pc : rdm;
```

**After (stack extension):**
```systemverilog
// addr_sel[1:0]: 00 = RDM, 01 = PC, 10 = SP
module mux_addr (
    input  logic [1:0] sel,
    input  logic [7:0] pc,
    input  logic [7:0] rdm,
    input  logic [7:0] sp,
    output logic [7:0] out
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

### Memory Data MUX

A new multiplexer selects the data source for memory writes (needed for CALL to write PC):

```systemverilog
// mem_data_sel: 0 = AC (for STA, PUSH), 1 = PC (for CALL)
assign mem_data_out = mem_data_sel ? pc : ac;
```

### New Control Signals

| Signal | Width | Description |
|--------|-------|-------------|
| `addr_sel` | 2 bits | Address source: 00=RDM, 01=PC, 10=SP |
| `sp_inc` | 1 bit | Increment stack pointer (POP, RET) |
| `sp_dec` | 1 bit | Decrement stack pointer (PUSH, CALL) |
| `mem_data_sel` | 1 bit | Memory data source: 0=AC, 1=PC |
| `sub_opcode` | 4 bits | Lower nibble of RI for stack ops |

### Updated Module Hierarchy

```
cpu_top
├── neander_datapath
│   ├── pc_reg
│   ├── sp_reg              <-- NEW
│   ├── mux_addr (3-way)    <-- EXTENDED
│   ├── generic_reg (REM)
│   ├── generic_reg (RDM)
│   ├── generic_reg (RI)
│   ├── generic_reg (AC)
│   ├── neander_alu
│   └── nz_reg
├── neander_control         <-- EXTENDED with new states
└── ram_256x8 (external)
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

The NEANDER-X Stack Extension provides essential subroutine and stack support while maintaining full backward compatibility:

| Feature | Vanilla NEANDER-X | Stack-Extended |
|---------|-------------------|----------------|
| Opcode 0x7 | Unused | PUSH/POP/CALL/RET |
| Stack Pointer | None | SP register (8-bit) |
| Address MUX | 2-way (PC/RDM) | 3-way (PC/RDM/SP) |
| Subroutines | Not supported | Full support |
| Nested Calls | Not supported | Supported |
| Data Stack | Not available | Available |

This extension enables modular programming with reusable subroutines, making the NEANDER-X a more capable educational processor while preserving its simplicity and educational value.

---

*Document Version: 1.0*
*Last Updated: January 2026*
