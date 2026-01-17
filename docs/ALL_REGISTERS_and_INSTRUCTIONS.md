# NEANDER-X Complete Register and Instruction Reference

This document provides a comprehensive reference for all registers and instructions in the NEANDER-X processor.

## Table of Contents

1. [Registers](#registers)
2. [Instruction Set Overview](#instruction-set-overview)
3. [Core Instructions](#core-instructions)
4. [Stack Instructions](#stack-instructions)
5. [LCC Extension Instructions](#lcc-extension-instructions)
6. [X Register Instructions](#x-register-instructions)
7. [Y Register Instructions](#y-register-instructions)
8. [B Register Instructions](#b-register-instructions)
9. [Frame Pointer Instructions](#frame-pointer-instructions)
10. [Memory Stack Operations](#memory-stack-operations)
11. [Jump Instructions](#jump-instructions)
12. [Hardware Multiply/Divide](#hardware-multiplydivide)
13. [I/O Instructions](#io-instructions)
14. [Instruction Encoding](#instruction-encoding)
15. [Code Examples](#code-examples)

---

## Registers

### Register Summary Table

| Register | Width | Reset Value | Description |
|----------|-------|-------------|-------------|
| **PC** | 16-bit | 0x0000 | Program Counter - holds address of next instruction |
| **SP** | 16-bit | 0x00FF | Stack Pointer - grows downward toward 0x0000 |
| **FP** | 16-bit | 0x0000 | Frame Pointer - for local variable access in functions |
| **AC** | 16-bit | 0x0000 | Accumulator - main register for arithmetic/logic operations |
| **X** | 16-bit | 0x0000 | Index Register X - for indexed addressing and loops |
| **Y** | 16-bit | 0x0000 | Index Register Y - second index for dual-pointer operations |
| **B** | 16-bit | 0x0000 | Auxiliary Register B - extra storage for complex operations |
| **REM** | 16-bit | 0x0000 | Memory Address Register - internal, holds current address |
| **RDM** | 16-bit | 0x0000 | Memory Data Register - internal, holds fetched data |
| **RI** | 8-bit | 0x00 | Instruction Register - internal, holds current opcode |

### Condition Flags

| Flag | Description | Set When |
|------|-------------|----------|
| **N** (Negative) | Sign flag | AC[15] = 1 (MSB is 1, indicating negative in two's complement) |
| **Z** (Zero) | Zero flag | AC = 0x0000 |
| **C** (Carry) | Carry/Borrow flag | Arithmetic overflow, shift-out, or MUL overflow |

### Register Details

#### Program Counter (PC)
- **Purpose**: Points to the next instruction to execute
- **Operations**: Increment (after fetch), Load (for jumps/calls)
- **Address Range**: 0x0000 - 0xFFFF (64KB)

#### Stack Pointer (SP)
- **Purpose**: Points to top of stack
- **Stack Direction**: Grows downward (PUSH decrements, POP increments)
- **Initial Value**: 0x00FF (page 0, grows toward 0x0000)

#### Accumulator (AC)
- **Purpose**: Primary register for all arithmetic and logic operations
- **ALU Operations**: ADD, SUB, AND, OR, XOR, NOT, SHL, SHR, ASR, NEG, MUL, DIV, MOD
- **Transfers**: Can transfer to/from X, Y, B registers

#### Index Register X
- **Purpose**: Array indexing, loop counters, pointer arithmetic
- **Indexed Modes**: LDA addr,X and STA addr,X
- **Arithmetic**: Can be added/subtracted from AC (ADDX, SUBX)

#### Index Register Y
- **Purpose**: Second index for dual-pointer operations, MUL high result
- **Indexed Modes**: LDA addr,Y and STA addr,Y
- **Special**: Receives high word from MUL, remainder from DIV

#### Auxiliary Register B
- **Purpose**: Extra storage for complex calculations, temporary values
- **Arithmetic**: Can be added/subtracted from AC (ADDB, SUBB)
- **Use Case**: Reduces memory traffic when working with multiple operands

#### Frame Pointer (FP)
- **Purpose**: Stack frame management for function calls
- **Indexed Mode**: LDA addr,FP and STA addr,FP for local variables
- **Typical Use**: Set to SP at function entry, access parameters and locals

---

## Instruction Set Overview

| Category | Count | Description |
|----------|-------|-------------|
| Core | 11 | Original NEANDER instructions |
| Stack | 4 | PUSH, POP, CALL, RET |
| LCC Extension | 15+ | SUB, INC, DEC, XOR, SHL, SHR, ASR, NEG, CMP, ADC, SBC, etc. |
| X Register | 8 | LDX, STX, LDXI, TAX, TXA, INX, DEX, indexed modes |
| Y Register | 9 | LDY, STY, LDYI, TAY, TYA, INY, DEY, indexed modes |
| B Register | 10 | LDB, STB, LDBI, TAB, TBA, INB, DEB, SWPB, ADDB, SUBB |
| Frame Pointer | 6 | TSF, TFS, PUSH_FP, POP_FP, indexed modes |
| Memory Stack | 2 | PUSH_ADDR, POP_ADDR |
| Jump | 12 | JMP, JN, JZ, JNZ, JC, JNC, JLE, JGT, JGE, JBE, JA, DECJNZ |
| Multiply/Divide | 3 | MUL, DIV, MOD |
| I/O | 2 | IN, OUT |
| **Total** | **70+** | |

---

## Core Instructions

These are the original NEANDER instructions.

| Opcode | Mnemonic | Operation | Flags | Cycles |
|--------|----------|-----------|-------|--------|
| 0x00 | NOP | No operation | - | 4 |
| 0x10 | STA addr | MEM[addr] <- AC | - | 6 |
| 0x20 | LDA addr | AC <- MEM[addr] | N, Z | 6 |
| 0x30 | ADD addr | AC <- AC + MEM[addr] | N, Z, C | 6 |
| 0x40 | OR addr | AC <- AC \| MEM[addr] | N, Z | 6 |
| 0x50 | AND addr | AC <- AC & MEM[addr] | N, Z | 6 |
| 0x60 | NOT | AC <- ~AC | N, Z | 1 |
| 0x80 | JMP addr | PC <- addr | - | 6 |
| 0x90 | JN addr | if N=1: PC <- addr | - | 6 |
| 0xA0 | JZ addr | if Z=1: PC <- addr | - | 6 |
| 0xF0 | HLT | Halt execution | - | - |

### Examples

```assembly
; Load, add, and store
    LDA operand1    ; AC = MEM[operand1]
    ADD operand2    ; AC = AC + MEM[operand2]
    STA result      ; MEM[result] = AC

; Logical operations
    LDA mask        ; AC = mask value
    AND data        ; AC = AC & data (mask bits)
    NOT             ; AC = ~AC (invert all bits)

; Conditional jump
    LDA counter     ; Load counter
    JZ done         ; If counter is zero, jump to done
    JN negative     ; If counter is negative, jump to negative
```

---

## Stack Instructions

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x70 | PUSH | SP -= 2; MEM[SP] <- AC | - | Push AC onto stack (16-bit) |
| 0x71 | POP | AC <- MEM[SP]; SP += 2 | N, Z | Pop from stack to AC (16-bit) |
| 0x72 | CALL addr | SP -= 2; MEM[SP] <- PC; PC <- addr | - | Call subroutine |
| 0x73 | RET | PC <- MEM[SP]; SP += 2 | - | Return from subroutine |

### Examples

```assembly
; Save and restore AC
    PUSH            ; Save AC on stack
    ; ... do work that modifies AC ...
    POP             ; Restore original AC

; Subroutine call
    LDI 10          ; AC = 10 (parameter)
    CALL multiply   ; Call subroutine
    STA result      ; Store return value
    HLT

multiply:
    ; Subroutine code here
    ; AC contains parameter, return value in AC
    RET             ; Return to caller
```

---

## LCC Extension Instructions

These instructions support C compiler code generation.

### Arithmetic Extensions

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x74 | SUB addr | AC <- AC - MEM[addr] | N, Z, C | Subtract memory from AC |
| 0x75 | INC | AC <- AC + 1 | N, Z | Increment AC |
| 0x76 | DEC | AC <- AC - 1 | N, Z | Decrement AC |
| 0x01 | NEG | AC <- -AC | N, Z, C | Two's complement negate |
| 0x31 | ADC addr | AC <- AC + MEM[addr] + C | N, Z, C | Add with carry |
| 0x51 | SBC addr | AC <- AC - MEM[addr] - C | N, Z, C | Subtract with borrow |

### Logic Extensions

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x77 | XOR addr | AC <- AC ^ MEM[addr] | N, Z | Exclusive OR |

### Shift Operations

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x78 | SHL | AC <- AC << 1 | N, Z, C | Shift left (C = MSB out) |
| 0x79 | SHR | AC <- AC >> 1 (logical) | N, Z, C | Shift right (0 into MSB) |
| 0x61 | ASR | AC <- AC >> 1 (arithmetic) | N, Z, C | Arithmetic shift (sign preserved) |

### Comparison

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x02 | CMP addr | AC - MEM[addr] (flags only) | N, Z, C | Compare (no result stored) |

### Examples

```assembly
; Subtraction
    LDA value1      ; AC = value1
    SUB value2      ; AC = value1 - value2

; Increment/Decrement loop
    LDI 10          ; AC = 10
loop:
    DEC             ; AC = AC - 1
    JNZ loop        ; Loop until AC = 0

; Two's complement negation
    LDA number      ; AC = number
    NEG             ; AC = -number

; Shift operations (multiply/divide by 2)
    LDA value       ; AC = value
    SHL             ; AC = value * 2
    SHR             ; AC = value (unsigned divide by 2)
    ASR             ; AC = value (signed divide by 2, preserves sign)

; 16-bit addition using ADC
    LDA a_lo        ; Load low byte of A
    ADD b_lo        ; Add low byte of B (sets carry)
    STA r_lo        ; Store low byte result
    LDA a_hi        ; Load high byte of A
    ADC b_hi        ; Add high byte of B + carry
    STA r_hi        ; Store high byte result

; Comparison and conditional branch
    LDA var_a       ; Load first value
    CMP var_b       ; Compare with second (sets flags)
    JZ equal        ; Jump if var_a == var_b
    JN less         ; Jump if var_a < var_b (signed)
```

---

## X Register Instructions

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x7A | LDX addr | X <- MEM[addr] | - | Load X from memory |
| 0x7B | STX addr | MEM[addr] <- X | - | Store X to memory |
| 0x7C | LDXI imm | X <- imm | - | Load X with immediate (16-bit) |
| 0x7D | TAX | X <- AC | - | Transfer AC to X |
| 0x7E | TXA | AC <- X | N, Z | Transfer X to AC |
| 0x7F | INX | X <- X + 1 | - | Increment X |
| 0x18 | DEX | X <- X - 1 | - | Decrement X |
| 0x1A | SWPX | AC <-> X | N, Z | Swap AC and X |

### X-Indexed Addressing

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x21 | LDA addr,X | AC <- MEM[addr + X] | N, Z | Load with X offset |
| 0x11 | STA addr,X | MEM[addr + X] <- AC | - | Store with X offset |

### X Register ALU Operations

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x32 | ADDX | AC <- AC + X | N, Z, C | Add X to AC |
| 0x33 | SUBX | AC <- AC - X | N, Z, C | Subtract X from AC |
| 0x42 | ORX | AC <- AC \| X | N, Z | OR X with AC |
| 0x52 | ANDX | AC <- AC & X | N, Z | AND X with AC |
| 0x43 | XORX | AC <- AC ^ X | N, Z | XOR X with AC |

### Examples

```assembly
; Array sum using X as index
    LDXI 0          ; X = 0 (start index)
    LDI 0           ; AC = 0 (sum)
loop:
    ADD array,X     ; AC += array[X]
    INX             ; X++
    INX             ; X++ (16-bit values = 2 bytes each)
    TXA             ; AC = X (to check index)
    CMP array_end   ; Compare with end
    TXA             ; Restore sum (oops, need to save it!)

; Better array sum
    LDXI 0          ; X = 0 (index)
    LDI 0           ; AC = 0 (sum)
    TAB             ; B = 0 (save sum in B)
loop:
    LDA array,X     ; AC = array[X]
    ADDB            ; AC = AC + B (add to running sum)
    TAB             ; B = AC (save new sum)
    INX             ; X++
    INX             ; X++ (16-bit elements)
    TXA             ; AC = X
    LDXI 20         ; Compare limit (10 elements * 2 bytes)
    SUBX            ; AC = X - 20
    JN loop         ; Continue if X < 20
    TBA             ; AC = B (final sum)

; String copy using X
    LDXI 0          ; X = 0
copy:
    LDA src,X       ; AC = src[X]
    JZ done         ; If zero (null terminator), done
    STA dst,X       ; dst[X] = AC
    INX             ; X++
    JMP copy
done:
    STA dst,X       ; Store null terminator
```

---

## Y Register Instructions

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x07 | LDY addr | Y <- MEM[addr] | - | Load Y from memory |
| 0x08 | STY addr | MEM[addr] <- Y | - | Store Y to memory |
| 0x06 | LDYI imm | Y <- imm | - | Load Y with immediate (16-bit) |
| 0x03 | TAY | Y <- AC | - | Transfer AC to Y |
| 0x04 | TYA | AC <- Y | N, Z | Transfer Y to AC |
| 0x05 | INY | Y <- Y + 1 | - | Increment Y |
| 0x19 | DEY | Y <- Y - 1 | - | Decrement Y |
| 0x1B | SWPY | AC <-> Y | N, Z | Swap AC and Y |

### Y-Indexed Addressing

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x22 | LDA addr,Y | AC <- MEM[addr + Y] | N, Z | Load with Y offset |
| 0x12 | STA addr,Y | MEM[addr + Y] <- AC | - | Store with Y offset |

### Y Register ALU Operations

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x34 | ADDY | AC <- AC + Y | N, Z, C | Add Y to AC |
| 0x35 | SUBY | AC <- AC - Y | N, Z, C | Subtract Y from AC |

### Examples

```assembly
; Memory copy using X (source) and Y (destination)
    LDXI 0          ; X = 0 (source index)
    LDYI 0          ; Y = 0 (dest index)
    LDI 10          ; AC = 10 (count)
    TAB             ; B = count
copy:
    LDA src,X       ; AC = src[X]
    STA dst,Y       ; dst[Y] = AC
    INX             ; X++
    INY             ; Y++
    DEB             ; B--
    TBA             ; AC = B
    JNZ copy        ; Continue if count != 0

; Two-pointer merge
    LDXI 0          ; X points to array1
    LDYI 0          ; Y points to array2
merge:
    LDA array1,X    ; AC = array1[X]
    CMP array2,Y    ; Compare with array2[Y]
    JN take_x       ; If array1[X] < array2[Y], take from X
    ; Take from Y
    LDA array2,Y
    INY
    INY
    JMP store
take_x:
    LDA array1,X
    INX
    INX
store:
    STA output,X    ; Store to output (simplified)
```

---

## B Register Instructions

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x1E | LDB addr | B <- MEM[addr] | - | Load B from memory |
| 0x1F | STB addr | MEM[addr] <- B | - | Store B to memory |
| 0xE4 | LDBI imm | B <- imm | - | Load B with immediate (16-bit) |
| 0x1C | TAB | B <- AC | - | Transfer AC to B |
| 0x1D | TBA | AC <- B | N, Z | Transfer B to AC |
| 0x36 | INB | B <- B + 1 | - | Increment B |
| 0x37 | DEB | B <- B - 1 | - | Decrement B |
| 0x38 | SWPB | AC <-> B | N, Z | Swap AC and B |

### B Register ALU Operations

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x39 | ADDB | AC <- AC + B | N, Z, C | Add B to AC |
| 0x3A | SUBB | AC <- AC - B | N, Z, C | Subtract B from AC |

### Examples

```assembly
; Use B as temporary storage during calculation
; Calculate: (a + b) * c  where we need to preserve intermediate results
    LDA a           ; AC = a
    ADD b           ; AC = a + b
    TAB             ; B = a + b (save intermediate)
    LDA c           ; AC = c
    TAX             ; X = c (for MUL)
    TBA             ; AC = a + b
    MUL             ; Y:AC = (a + b) * c
    STA result_lo   ; Store low word
    TYA             ; AC = Y (high word)
    STA result_hi   ; Store high word

; Use B as a loop counter while X is used for indexing
    LDBI 10         ; B = 10 (counter)
    LDXI 0          ; X = 0 (index)
loop:
    LDA array,X     ; Process array[X]
    ; ... do something with AC ...
    INX             ; X++
    INX             ; X++ (16-bit elements)
    DEB             ; B-- (decrement counter)
    TBA             ; AC = B
    JNZ loop        ; Continue if B != 0

; Three-value calculation: result = a - b + c
    LDA a           ; AC = a
    TAB             ; B = a
    LDA b           ; AC = b
    SUBB            ; AC = b - a (wait, that's wrong!)

; Correct version:
    LDA a           ; AC = a
    SUB b           ; AC = a - b
    TAB             ; B = a - b
    LDA c           ; AC = c
    ADDB            ; AC = c + (a - b) = a - b + c
    STA result

; Swap two memory locations using B
    LDA var1        ; AC = var1
    TAB             ; B = var1
    LDA var2        ; AC = var2
    STA var1        ; var1 = var2
    TBA             ; AC = B (original var1)
    STA var2        ; var2 = original var1
```

---

## Frame Pointer Instructions

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x0A | TSF | FP <- SP | - | Transfer SP to FP |
| 0x0B | TFS | SP <- FP | - | Transfer FP to SP |
| 0x0C | PUSH_FP | SP -= 2; MEM[SP] <- FP | - | Push FP onto stack |
| 0x0D | POP_FP | FP <- MEM[SP]; SP += 2 | - | Pop FP from stack |

### FP-Indexed Addressing

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x24 | LDA addr,FP | AC <- MEM[addr + FP] | N, Z | Load with FP offset |
| 0x14 | STA addr,FP | MEM[addr + FP] <- AC | - | Store with FP offset |

### Examples

```assembly
; Standard function prologue/epilogue
; Function: int add(int a, int b)
; Parameters pushed by caller before CALL

add:
    ; Prologue
    PUSH_FP         ; Save caller's frame pointer
    TSF             ; FP = SP (establish new frame)

    ; Stack layout after prologue:
    ; FP+0: saved FP
    ; FP+2: return address
    ; FP+4: parameter a
    ; FP+6: parameter b

    ; Function body
    LDA 0x04,FP     ; AC = parameter a (at FP+4)
    TAB             ; B = a
    LDA 0x06,FP     ; AC = parameter b (at FP+6)
    ADDB            ; AC = a + b

    ; Epilogue
    TFS             ; SP = FP (deallocate locals if any)
    POP_FP          ; Restore caller's FP
    RET             ; Return (result in AC)

; Calling the function
main:
    LDI 5           ; First parameter
    PUSH            ; Push a=5
    LDI 3           ; Second parameter
    PUSH            ; Push b=3
    CALL add        ; Call add(5, 3)
    ; AC now contains 8
    ; Clean up stack (remove parameters)
    POP             ; Remove one parameter (discarded)
    POP             ; Remove other parameter (discarded)
    STA result      ; Store result
    HLT

; Function with local variables
factorial:
    PUSH_FP         ; Save caller's FP
    TSF             ; FP = SP

    ; Allocate local variable (result)
    LDI 1           ; Initial result = 1
    PUSH            ; Local at FP-2

    ; FP-2: local result
    ; FP+0: saved FP
    ; FP+2: return address
    ; FP+4: parameter n

loop:
    LDA 0x04,FP     ; AC = n
    JZ done         ; If n == 0, done

    ; result = result * n
    LDA 0xFE,FP     ; AC = local result (FP-2 = FP+0xFFFE in 16-bit)
    TAX             ; X = result (for MUL)
    LDA 0x04,FP     ; AC = n
    MUL             ; Y:AC = n * result
    STA 0xFE,FP     ; Store new result

    ; n = n - 1
    LDA 0x04,FP     ; AC = n
    DEC             ; AC = n - 1
    STA 0x04,FP     ; n = n - 1

    JMP loop

done:
    LDA 0xFE,FP     ; AC = result
    TFS             ; SP = FP (deallocate local)
    POP_FP          ; Restore FP
    RET
```

---

## Memory Stack Operations

These instructions push/pop values directly from memory addresses without using AC.

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x89 | PUSH_ADDR addr | SP -= 2; MEM[SP] <- MEM[addr] | - | Push memory value to stack |
| 0x8A | POP_ADDR addr | MEM[addr] <- MEM[SP]; SP += 2 | - | Pop stack to memory |

### Examples

```assembly
; Save multiple variables without using AC
    PUSH_ADDR var1  ; Push var1 to stack
    PUSH_ADDR var2  ; Push var2 to stack
    PUSH_ADDR var3  ; Push var3 to stack

    ; Do computation that uses AC freely
    LDA var1
    ADD var2
    MUL             ; etc...
    STA result

    ; Restore variables in reverse order (LIFO)
    POP_ADDR var3   ; Restore var3
    POP_ADDR var2   ; Restore var2
    POP_ADDR var1   ; Restore var1

; Swap two memory locations using stack
    PUSH_ADDR var1  ; Stack: [var1]
    PUSH_ADDR var2  ; Stack: [var2, var1]
    POP_ADDR var1   ; var1 = var2, Stack: [var1_original]
    POP_ADDR var2   ; var2 = var1_original, Stack: []

; Function call with multiple parameters (without using AC)
    PUSH_ADDR arg3  ; Push third argument
    PUSH_ADDR arg2  ; Push second argument
    PUSH_ADDR arg1  ; Push first argument
    CALL function   ; Call function
    ; Clean up stack after return
    POP_ADDR temp   ; Remove arg1 (or add to SP)
    POP_ADDR temp   ; Remove arg2
    POP_ADDR temp   ; Remove arg3
```

---

## Jump Instructions

### Unconditional Jump

| Opcode | Mnemonic | Operation | Description |
|--------|----------|-----------|-------------|
| 0x80 | JMP addr | PC <- addr | Always jump |

### Flag-Based Jumps

| Opcode | Mnemonic | Condition | Description |
|--------|----------|-----------|-------------|
| 0x90 | JN addr | N = 1 | Jump if Negative |
| 0xA0 | JZ addr | Z = 1 | Jump if Zero |
| 0xB0 | JNZ addr | Z = 0 | Jump if Not Zero |
| 0x81 | JC addr | C = 1 | Jump if Carry |
| 0x82 | JNC addr | C = 0 | Jump if No Carry |

### Comparison Jumps (After CMP)

#### Signed Comparisons

| Opcode | Mnemonic | Condition | Description |
|--------|----------|-----------|-------------|
| 0x83 | JLE addr | N=1 OR Z=1 | Jump if Less or Equal |
| 0x84 | JGT addr | N=0 AND Z=0 | Jump if Greater Than |
| 0x85 | JGE addr | N=0 | Jump if Greater or Equal |

#### Unsigned Comparisons

| Opcode | Mnemonic | Condition | Description |
|--------|----------|-----------|-------------|
| 0x86 | JBE addr | C=1 OR Z=1 | Jump if Below or Equal |
| 0x87 | JA addr | C=0 AND Z=0 | Jump if Above |

### Decrement and Branch

| Opcode | Mnemonic | Operation | Description |
|--------|----------|-----------|-------------|
| 0x88 | DECJNZ addr | AC--; if AC != 0: PC <- addr | Decrement and jump if not zero |

### Examples

```assembly
; Simple loop with counter
    LDI 10          ; AC = 10
loop:
    ; ... loop body ...
    DEC             ; AC--
    JNZ loop        ; Continue while AC != 0

; Same loop using DECJNZ (more efficient)
    LDI 10          ; AC = 10
loop:
    ; ... loop body ...
    DECJNZ loop     ; AC--; jump if AC != 0

; Signed comparison: if (a > b) goto greater
    LDA a
    CMP b           ; Compare a with b
    JGT greater     ; Jump if a > b (signed)
    ; ... else branch ...
    JMP endif
greater:
    ; ... greater branch ...
endif:

; Unsigned comparison: if (a > b) goto above
    LDA a
    CMP b           ; Compare a with b
    JA above        ; Jump if a > b (unsigned)
    ; ... else branch ...
    JMP endif
above:
    ; ... above branch ...
endif:

; Switch-like construct
    LDA selector
    CMP case1_val
    JZ case1
    CMP case2_val
    JZ case2
    CMP case3_val
    JZ case3
    JMP default

case1:
    ; ... case 1 code ...
    JMP end_switch
case2:
    ; ... case 2 code ...
    JMP end_switch
case3:
    ; ... case 3 code ...
    JMP end_switch
default:
    ; ... default code ...
end_switch:
```

---

## Hardware Multiply/Divide

### Instructions

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0x09 | MUL | Y:AC <- AC * X | N, Z, C | 16x16 -> 32-bit multiply |
| 0x0E | DIV | AC <- AC / X; Y <- AC % X | N, Z, C | 16/16 divide with remainder |
| 0x0F | MOD | AC <- AC % X; Y <- AC / X | N, Z, C | 16/16 modulo with quotient |

### Notes

- **MUL**: Takes 16 cycles (sequential multiplier)
  - Result: 32-bit product split into Y (high 16 bits) and AC (low 16 bits)
  - Carry flag set if result > 0xFFFF (overflow to Y)

- **DIV**: Takes 16 cycles (sequential divider)
  - AC receives quotient, Y receives remainder
  - Division by zero: C=1 (error), AC=0xFFFF, Y=original dividend

- **MOD**: Takes 16 cycles (sequential divider)
  - AC receives remainder, Y receives quotient
  - Division by zero: C=1 (error), AC=original dividend, Y=0xFFFF

### Examples

```assembly
; Simple multiplication: result = 7 * 5
    LDI 7           ; AC = 7
    LDXI 5          ; X = 5
    MUL             ; Y:AC = 7 * 5 = 35
    ; AC = 35 (0x0023), Y = 0
    STA result

; Large multiplication with overflow
    LDI 1000        ; AC = 1000
    LDXI 1000       ; X = 1000
    MUL             ; Y:AC = 1000 * 1000 = 1,000,000
    ; AC = 0x4240 (low 16 bits)
    ; Y = 0x000F (high 16 bits)
    ; Full result: 0x000F4240 = 1,000,000
    STA result_lo   ; Store low word
    TYA             ; AC = Y
    STA result_hi   ; Store high word

; Division: quotient = 17 / 5, remainder = 17 % 5
    LDI 17          ; AC = 17
    LDXI 5          ; X = 5
    DIV             ; AC = 3 (quotient), Y = 2 (remainder)
    STA quotient    ; quotient = 3
    TYA             ; AC = Y
    STA remainder   ; remainder = 2

; Modulo: remainder = 17 % 5
    LDI 17          ; AC = 17
    LDXI 5          ; X = 5
    MOD             ; AC = 2 (remainder), Y = 3 (quotient)
    STA remainder   ; remainder = 2

; Check for division by zero
    LDA divisor
    JZ div_by_zero  ; Handle division by zero
    TAX             ; X = divisor
    LDA dividend    ; AC = dividend
    DIV             ; Perform division
    JC div_error    ; Check carry (error flag)
    STA result
    JMP done
div_by_zero:
div_error:
    LDI 0xFF        ; Error indicator
    STA result
done:

; Integer average: avg = (a + b) / 2
    LDA a
    ADD b           ; AC = a + b
    LDXI 2          ; X = 2
    DIV             ; AC = (a + b) / 2
    STA average
```

---

## I/O Instructions

| Opcode | Mnemonic | Operation | Flags | Description |
|--------|----------|-----------|-------|-------------|
| 0xC0 | IN port | AC <- IO_IN[port] | N, Z | Input from port |
| 0xD0 | OUT port | IO_OUT <- AC | - | Output to port |

### Notes

- Port number is 8-bit (0-255)
- IN reads 8-bit value, zero-extended to AC
- OUT writes low 8 bits of AC

### Examples

```assembly
; Read from input port and echo to output
    IN 0            ; AC = input from port 0
    OUT 0           ; Output AC to port 0

; Wait for input ready, then read
wait_input:
    IN 1            ; Read status port
    AND ready_mask  ; Check ready bit
    JZ wait_input   ; Wait until ready
    IN 0            ; Read data port
    STA input_data

; Output a string
    LDXI 0          ; X = 0 (index)
output_loop:
    LDA message,X   ; AC = message[X]
    JZ output_done  ; If null terminator, done
    OUT 0           ; Output character
    INX             ; X++
    JMP output_loop
output_done:
    HLT
```

---

## Instruction Encoding

### Format Summary

| Type | Format | Bytes | Example |
|------|--------|-------|---------|
| Memory ops | `[opcode] [addr_lo] [addr_hi]` | 3 | `LDA 0x1234` -> `0x20 0x34 0x12` |
| 16-bit Immediate | `[opcode] [imm_lo] [imm_hi]` | 3 | `LDI 0x1234` -> `0xE0 0x34 0x12` |
| I/O ops | `[opcode] [port]` | 2 | `OUT 0` -> `0xD0 0x00` |
| Single-byte | `[opcode]` | 1 | `NOT` -> `0x60` |

### Complete Opcode Map

```
0x00: NOP           0x40: OR addr       0x80: JMP addr
0x01: NEG           0x42: ORX           0x81: JC addr
0x02: CMP addr      0x43: XORX          0x82: JNC addr
0x03: TAY           0x50: AND addr      0x83: JLE addr
0x04: TYA           0x51: SBC addr      0x84: JGT addr
0x05: INY           0x52: ANDX          0x85: JGE addr
0x06: LDYI imm                          0x86: JBE addr
0x07: LDY addr      0x60: NOT           0x87: JA addr
0x08: STY addr      0x61: ASR           0x88: DECJNZ addr
0x09: MUL                               0x89: PUSH_ADDR addr
0x0A: TSF           0x70: PUSH          0x8A: POP_ADDR addr
0x0B: TFS           0x71: POP           0x90: JN addr
0x0C: PUSH_FP       0x72: CALL addr
0x0D: POP_FP        0x73: RET           0xA0: JZ addr
0x0E: DIV           0x74: SUB addr      0xB0: JNZ addr
0x0F: MOD           0x75: INC
                    0x76: DEC           0xC0: IN port
0x10: STA addr      0x77: XOR addr      0xD0: OUT port
0x11: STA addr,X    0x78: SHL           0xE0: LDI imm
0x12: STA addr,Y    0x79: SHR           0xE4: LDBI imm
0x14: STA addr,FP   0x7A: LDX addr
0x17: STA off,SP    0x7B: STX addr      0xF0: HLT
0x18: DEX           0x7C: LDXI imm
0x19: DEY           0x7D: TAX
0x1A: SWPX          0x7E: TXA
0x1B: SWPY          0x7F: INX
0x1C: TAB
0x1D: TBA           0x20: LDA addr
0x1E: LDB addr      0x21: LDA addr,X
0x1F: STB addr      0x22: LDA addr,Y
                    0x24: LDA addr,FP
0x30: ADD addr      0x26: LDA (addr)
0x31: ADC addr      0x27: LDA (addr),Y
0x32: ADDX          0x28: LDA off,SP
0x33: SUBX
0x34: ADDY
0x35: SUBY
0x36: INB
0x37: DEB
0x38: SWPB
0x39: ADDB
0x3A: SUBB
```

---

## Code Examples

### Example 1: Bubble Sort

```assembly
; Bubble sort an array of 16-bit values
; array: starting address of array
; count: number of elements

bubble_sort:
    LDBI 1          ; B = 1 (swapped flag, start true)

outer_loop:
    TBA             ; AC = B (swapped flag)
    JZ sort_done    ; If no swaps occurred, done

    LDBI 0          ; B = 0 (reset swapped flag)
    LDXI 0          ; X = 0 (index)

inner_loop:
    ; Compare array[X] with array[X+2]
    LDA array,X     ; AC = array[X]
    TAY             ; Y = array[X] (save for swap)
    INX
    INX             ; X += 2 (point to next element)

    ; Check if we've reached the end
    TXA             ; AC = X
    CMP count_bytes ; Compare with count*2
    JGE outer_loop  ; If X >= count*2, go to outer loop

    ; Compare current with next
    TYA             ; AC = Y (array[X-2])
    CMP array,X     ; Compare with array[X]
    JLE no_swap     ; If array[X-2] <= array[X], no swap

    ; Swap array[X-2] and array[X]
    LDA array,X     ; AC = array[X]
    PUSH            ; Save array[X]
    TYA             ; AC = Y (array[X-2])
    STA array,X     ; array[X] = array[X-2]
    POP             ; AC = saved array[X]
    DEX
    DEX             ; X -= 2
    STA array,X     ; array[X-2] = old array[X]
    INX
    INX             ; X += 2 (restore)

    LDBI 1          ; B = 1 (set swapped flag)

no_swap:
    JMP inner_loop

sort_done:
    RET
```

### Example 2: String Length

```assembly
; Calculate string length
; Input: string address in 'str_ptr'
; Output: length in AC

strlen:
    LDX str_ptr     ; X = string pointer
    LDYI 0          ; Y = 0 (length counter)

strlen_loop:
    LDA 0,X         ; AC = *X (current character)
    JZ strlen_done  ; If null terminator, done
    INY             ; length++
    INX             ; pointer++
    JMP strlen_loop

strlen_done:
    TYA             ; AC = length
    RET
```

### Example 3: 32-bit Addition

```assembly
; 32-bit addition: result = a + b
; a: a_lo (low 16 bits), a_hi (high 16 bits)
; b: b_lo (low 16 bits), b_hi (high 16 bits)
; result: r_lo, r_hi

add32:
    ; Add low words
    LDA a_lo
    ADD b_lo        ; AC = a_lo + b_lo, C = carry
    STA r_lo        ; Store low result

    ; Add high words with carry
    LDA a_hi
    ADC b_hi        ; AC = a_hi + b_hi + C
    STA r_hi        ; Store high result

    RET
```

### Example 4: Factorial

```assembly
; Calculate factorial(n)
; Input: n in AC
; Output: result in AC (may overflow for n > 8)

factorial:
    TAB             ; B = n (save input)
    LDI 1           ; AC = 1 (result)

fact_loop:
    TBA             ; AC = B (current n)
    JZ fact_done    ; If n == 0, done

    ; result = result * n
    TAX             ; X = n
    SWPB            ; AC = result, B = n
    MUL             ; Y:AC = result * n
    ; (ignore Y for small results)
    TAB             ; B = new result

    ; n = n - 1
    SWPB            ; AC = n, B = result
    DEC             ; AC = n - 1
    SWPB            ; AC = result, B = n-1
    TAB             ; Save result back to B... wait this is confusing

; Simpler version:
factorial2:
    STA n           ; Store input
    LDI 1
    STA result      ; result = 1

fact2_loop:
    LDA n
    JZ fact2_done   ; If n == 0, done

    TAX             ; X = n
    LDA result      ; AC = result
    MUL             ; Y:AC = result * n
    STA result      ; result = AC (low word)

    LDA n
    DEC
    STA n           ; n--

    JMP fact2_loop

fact2_done:
    LDA result      ; Return result in AC
    RET
```

### Example 5: Memory Fill

```assembly
; Fill memory with a value
; start_addr: starting address
; count: number of 16-bit words
; value: value to fill

memfill:
    LDA value       ; AC = fill value
    LDX start_addr  ; X = destination pointer
    LDY count       ; Y = count

fill_loop:
    TYA             ; AC = Y (count)
    JZ fill_done    ; If count == 0, done

    LDA value       ; AC = value
    STA 0,X         ; *X = value
    INX
    INX             ; X += 2 (16-bit values)
    DEY             ; count--

    JMP fill_loop

fill_done:
    RET
```

### Example 6: Binary to BCD Conversion

```assembly
; Convert 8-bit binary to BCD (0-99)
; Input: binary value in AC (0-99)
; Output: BCD in AC (high nibble = tens, low nibble = ones)

bin_to_bcd:
    LDXI 10         ; X = 10 (divisor)
    DIV             ; AC = quotient (tens), Y = remainder (ones)
    SHL             ; AC = tens << 1
    SHL             ; AC = tens << 2
    SHL             ; AC = tens << 3
    SHL             ; AC = tens << 4 (tens in high nibble)
    TAB             ; B = tens in high nibble
    TYA             ; AC = ones
    ADDB            ; AC = (tens << 4) | ones = BCD
    RET
```

---

## Quick Reference Card

### Registers
| Reg | Width | Purpose |
|-----|-------|---------|
| AC | 16-bit | Main accumulator |
| X | 16-bit | Index register |
| Y | 16-bit | Index register |
| B | 16-bit | Auxiliary register |
| PC | 16-bit | Program counter |
| SP | 16-bit | Stack pointer |
| FP | 16-bit | Frame pointer |

### Flags
| Flag | Meaning |
|------|---------|
| N | Negative (AC[15] = 1) |
| Z | Zero (AC = 0) |
| C | Carry/Borrow |

### Common Operations
| Task | Instruction(s) |
|------|----------------|
| Load constant | `LDI imm` |
| Load from memory | `LDA addr` |
| Store to memory | `STA addr` |
| Add | `ADD addr` or `ADDB` or `ADDX` or `ADDY` |
| Subtract | `SUB addr` or `SUBB` or `SUBX` or `SUBY` |
| Multiply | `MUL` (AC * X -> Y:AC) |
| Divide | `DIV` (AC / X -> AC, Y=remainder) |
| Compare | `CMP addr` (then JZ/JN/JLE/JGT/etc.) |
| Loop | `DECJNZ addr` or `DEC; JNZ addr` |
| Call function | `CALL addr` |
| Return | `RET` |
| Push value | `PUSH` or `PUSH_ADDR addr` |
| Pop value | `POP` or `POP_ADDR addr` |

---

*Document Version: 1.0*
*Last Updated: January 2026*
