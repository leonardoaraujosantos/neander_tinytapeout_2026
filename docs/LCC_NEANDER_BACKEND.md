# LCC Compiler Backend for NEANDER-X 16-bit

This document describes the LCC (Little C Compiler) backend targeting the NEANDER-X 16-bit processor.

## Overview

The NEANDER-X is a **16-bit** accumulator-based processor with the following characteristics:

- **16-bit native word size** - int = 2 bytes, pointers = 2 bytes
- **16-bit address space** - 64KB addressable via SPI SRAM (0x0000-0xFFFF)
- **Accumulator architecture** - Single AC register for most operations
- **Index registers** - X, Y for array indexing and temporary storage
- **Frame pointer** - FP for local variable and parameter access
- **Hardware arithmetic** - MUL, DIV, MOD instructions
- **Multi-byte support** - ADC, SBC for 32-bit arithmetic
- **Stack operations** - PUSH, POP, CALL, RET (16-bit values)
- **Little-endian** byte order

## Registers

| Register | Size | LCC Usage | Notes |
|----------|------|-----------|-------|
| AC | 16-bit | Result register | Primary accumulator for computation |
| X | 16-bit | Index/temp | Array indexing, MUL operand |
| Y | 16-bit | Index/temp | MUL high byte, DIV remainder |
| FP | 16-bit | Frame pointer | Local variable and parameter access |
| SP | 16-bit | Stack pointer | Grows downward, managed by PUSH/POP |
| PC | 16-bit | Program counter | Current instruction address |

## Type Metrics

The NEANDER-X LCC backend defines these type sizes:

```c
Interface neanderxIR = {
    1, 1, 0,  /* char:        1 byte, 1-byte align */
    2, 2, 0,  /* short:       2 bytes, 2-byte align (16-bit native) */
    2, 2, 0,  /* int:         2 bytes, 2-byte align (16-bit native) */
    4, 2, 0,  /* long:        4 bytes, 2-byte align (32-bit using ADC/SBC) */
    4, 2, 0,  /* long long:   4 bytes, 2-byte align (32-bit) */
    0, 1, 1,  /* float:       not supported (outofline=1) */
    0, 1, 1,  /* double:      not supported */
    0, 1, 1,  /* long double: not supported */
    2, 2, 0,  /* pointer:     2 bytes, 2-byte align (16-bit address) */
    0, 2, 0,  /* struct:      2-byte alignment */
    ...
};
```

## Calling Convention

- **Arguments**: Pushed right-to-left on stack (2-byte aligned)
- **Return value**: In AC (16-bit) or Y:AC for results larger than 16-bit
- **Caller cleanup**: Caller responsible for removing arguments from stack
- **Frame-relative addressing**: Parameters and locals accessed via FP

## Stack Frame Layout

```
    Higher addresses
    +------------------+
    | Parameter N      | <- FP + 4 + 2*(N-1)
    | ...              |
    | Parameter 1      | <- FP + 4
    | Return Address   | <- FP + 2 (16-bit)
    | Saved FP         | <- FP (16-bit)
    | Local Variable 1 | <- FP - 2
    | Local Variable 2 | <- FP - 4
    | ...              |
    +------------------+
    Lower addresses    <- SP
```

## Instruction Set Summary

### Data Movement
| Instruction | Operation | LCC Usage |
|-------------|-----------|-----------|
| LDA addr | AC = MEM[addr] | Load from memory |
| STA addr | MEM[addr] = AC | Store to memory |
| LDA offset,FP | AC = MEM[FP+offset] | Load local/parameter |
| STA offset,FP | MEM[FP+offset] = AC | Store local/parameter |
| LDA addr,X | AC = MEM[addr+X] | Indexed load |
| STA addr,X | MEM[addr+X] = AC | Indexed store |
| LDI imm | AC = imm | Load immediate |
| TAX/TXA | Transfer AC <-> X | Register move |
| TAY/TYA | Transfer AC <-> Y | Register move |

### Arithmetic
| Instruction | Operation | LCC Usage |
|-------------|-----------|-----------|
| ADD addr | AC = AC + MEM[addr] | Addition |
| SUB addr | AC = AC - MEM[addr] | Subtraction |
| ADC addr | AC = AC + MEM[addr] + C | Multi-byte addition |
| SBC addr | AC = AC - MEM[addr] - C | Multi-byte subtraction |
| MUL | Y:AC = AC * X | 16x16 -> 32-bit multiply |
| DIV | AC = AC / X, Y = rem | Division |
| MOD | AC = AC % X | Modulo |
| INC/DEC | AC = AC +/- 1 | Increment/Decrement |
| NEG | AC = -AC | Negate |

### Logic and Shifts
| Instruction | Operation | LCC Usage |
|-------------|-----------|-----------|
| AND addr | AC = AC & MEM[addr] | Bitwise AND |
| OR addr | AC = AC \| MEM[addr] | Bitwise OR |
| XOR addr | AC = AC ^ MEM[addr] | Bitwise XOR |
| NOT | AC = ~AC | Complement |
| SHL | AC = AC << 1 | Left shift |
| SHR | AC = AC >> 1 (logical) | Unsigned right shift |
| ASR | AC = AC >> 1 (arithmetic) | Signed right shift |

### Comparison and Branch
| Instruction | Operation | LCC Usage |
|-------------|-----------|-----------|
| CMP addr | Sets flags: AC - MEM[addr] | Compare for branching |
| JMP addr | PC = addr | Unconditional jump |
| JZ addr | if Z: PC = addr | Equal |
| JNZ addr | if !Z: PC = addr | Not equal |
| JN addr | if N: PC = addr | Less than (signed) |
| JC addr | if C: PC = addr | Less than (unsigned) |
| JLE addr | if N\|Z: PC = addr | Less or equal (signed) |
| JGT addr | if !N&!Z: PC = addr | Greater than (signed) |
| JGE addr | if !N: PC = addr | Greater or equal (signed) |
| JBE addr | if C\|Z: PC = addr | Below or equal (unsigned) |
| JA addr | if !C&!Z: PC = addr | Above (unsigned) |

### Stack and Function
| Instruction | Operation | LCC Usage |
|-------------|-----------|-----------|
| PUSH | MEM[--SP] = AC | Push 16-bit value |
| POP | AC = MEM[SP++] | Pop 16-bit value |
| PUSH_FP | MEM[--SP] = FP | Save frame pointer |
| POP_FP | FP = MEM[SP++] | Restore frame pointer |
| TSF | FP = SP | Transfer SP to FP |
| TFS | SP = FP | Transfer FP to SP |
| CALL addr | PUSH PC; PC = addr | Function call |
| RET | PC = MEM[SP++] | Return from function |

## VREG Implementation

LCC uses Virtual Registers (VREGs) as an abstraction for computation. On the NEANDER-X accumulator architecture, VREGs are allocated on the stack as part of the function's local variables via the `local()` function.

### Stack-Based VREG Allocation (Correct Approach)

VREGs are allocated on the stack through LCC's standard `local()` function, which assigns them negative offsets from FP:

```c
static void local(Symbol p) {
    /* Ensure 2-byte alignment for 16-bit architecture */
    offset = roundup(offset + p->type->size, p->type->align < 2 ? 2 : p->type->align);
    p->x.offset = -offset;
    p->x.name = stringf("%d", -offset);  /* e.g., "-2", "-4", "-6" */
}
```

This means VREGs live on the stack alongside other local variables, accessed via FP-relative addressing (e.g., `LDA -2,FP`).

### emit2 Callback for VREG Operations

The `emit2` function handles VREG read/write using the symbol's `x.name` field (the offset set by `local()`):

```c
static void emit2(Node p) {
    int op = specific(p->op);
    Symbol reg, reg1, reg2;
    Node left, right;

    switch (op) {
    case ASGN+I:
    case ASGN+U:
    case ASGN+P:
        /* Write to VREG - store using FP-relative offset from local() */
        if (LEFT_CHILD(p) && LEFT_CHILD(p)->op == VREG+P) {
            reg = LEFT_CHILD(p)->syms[0];
            if (reg && reg->x.name) {
                /* Use the symbol's offset set by local() */
                print("    STA %s,FP\n", reg->x.name);  /* e.g., "STA -2,FP" */
            }
        }
        break;
    case INDIR+I:
    case INDIR+U:
    case INDIR+P:
        /* Read from VREG - load using FP-relative offset from local() */
        if (LEFT_CHILD(p) && LEFT_CHILD(p)->op == VREG+P) {
            reg = LEFT_CHILD(p)->syms[0];
            if (reg && reg->x.name) {
                print("    LDA %s,FP\n", reg->x.name);  /* e.g., "LDA -2,FP" */
            }
        }
        break;
    case ADD+I:
    case ADD+U:
    case ADD+P:
        /* Handle VREG + VREG */
        left = LEFT_CHILD(p);
        right = RIGHT_CHILD(p);
        if (left && right &&
            generic(left->op) == INDIR && LEFT_CHILD(left) &&
            LEFT_CHILD(left)->op == VREG+P &&
            generic(right->op) == INDIR && LEFT_CHILD(right) &&
            LEFT_CHILD(right)->op == VREG+P) {
            reg1 = LEFT_CHILD(left)->syms[0];
            reg2 = LEFT_CHILD(right)->syms[0];
            if (reg1 && reg1->x.name && reg2 && reg2->x.name) {
                print("    LDA %s,FP\n", reg1->x.name);
                print("    STA _tmp\n");
                print("    LDA %s,FP\n", reg2->x.name);
                print("    ADD _tmp\n");
            }
        }
        break;
    case MUL+I:
    case MUL+U:
        /* Handle VREG * VREG */
        left = LEFT_CHILD(p);
        right = RIGHT_CHILD(p);
        if (left && right &&
            generic(left->op) == INDIR && LEFT_CHILD(left) &&
            LEFT_CHILD(left)->op == VREG+P &&
            generic(right->op) == INDIR && LEFT_CHILD(right) &&
            LEFT_CHILD(right)->op == VREG+P) {
            reg1 = LEFT_CHILD(left)->syms[0];
            reg2 = LEFT_CHILD(right)->syms[0];
            if (reg1 && reg1->x.name && reg2 && reg2->x.name) {
                print("    LDA %s,FP\n", reg2->x.name);
                print("    TAX\n");
                print("    LDA %s,FP\n", reg1->x.name);
                print("    MUL\n");
            }
        }
        break;
    }
}
```

### Why Stack-Based VREGs Matter

**Critical**: Using stack-based VREGs (via `local()`) instead of global memory slots is essential for:

1. **Recursion**: Each function invocation gets its own stack frame with unique VREG storage
2. **Reentrancy**: Functions can be called from interrupts without corrupting state
3. **Correctness**: Prevents VREG values from being overwritten by nested calls

**Common Bug**: Using hardcoded global offsets for VREGs (like `_vreg0`, `_vreg1`) causes recursive functions like Fibonacci to fail because all invocations share the same memory locations.

## lburg Grammar Rules

### Non-terminals

The backend uses these non-terminals:

| Non-terminal | Description |
|--------------|-------------|
| `stmt` | Statement (no value produced) |
| `reg` | Value in accumulator |
| `addr` | Global memory address |
| `faddr` | Frame-relative address |
| `con1` | 8-bit constant |
| `con2` | 16-bit constant |
| `con4` | 32-bit constant |
| `conN` | Constant value 1 (for INC/DEC) |

### Key Grammar Rules

#### Constants

```c
con2: CNSTI2  "%a"
con2: CNSTU2  "%a"
con2: CNSTP2  "%a"

reg: con2  "    LDI %0\n"  1
```

#### Addresses

```c
/* Global address */
addr: ADDRGP2  "%a"

/* Frame-relative address (parameters and locals) */
faddr: ADDRFP2  "%a,FP"
faddr: ADDRLP2  "%a,FP"

/* faddr is also an addr */
addr: faddr  "%0"
```

#### Load and Store

```c
/* Frame-relative load */
reg: INDIRI2(faddr)  "    LDA %0\n"  1
reg: INDIRU2(faddr)  "    LDA %0\n"  1
reg: INDIRP2(faddr)  "    LDA %0\n"  1

/* Frame-relative store */
stmt: ASGNI2(faddr,reg)  "    STA %0\n"  1
stmt: ASGNU2(faddr,reg)  "    STA %0\n"  1
stmt: ASGNP2(faddr,reg)  "    STA %0\n"  1

/* Global memory load */
reg: INDIRI2(addr)  "    LDA %0\n"  2
reg: INDIRU2(addr)  "    LDA %0\n"  2
reg: INDIRP2(addr)  "    LDA %0\n"  2
```

#### Arithmetic

```c
/* Add FP-relative variable + constant */
reg: ADDI2(INDIRI2(faddr),con2)  "    LDA %0\n    STA _tmp\n    LDI %1\n    ADD _tmp\n"  3

/* Add two FP-relative variables */
reg: ADDI2(INDIRI2(faddr),INDIRI2(faddr))  "    LDA %0\n    STA _tmp\n    LDA %1\n    ADD _tmp\n"  4

/* Add global variables */
reg: ADDI2(INDIRI2(addr),INDIRI2(addr))  "    LDA %0\n    STA _tmp\n    LDA %1\n    ADD _tmp\n"  4

/* VREG binary operations (handled via emit2) */
reg: ADDI2(INDIRI2(VREGP),INDIRI2(VREGP))  "# add vreg+vreg\n"  3
reg: MULI2(INDIRI2(VREGP),INDIRI2(VREGP))  "# mul vreg*vreg\n"  3
```

#### Multiplication and Division

```c
/* 16x16 -> 16-bit result (low word) */
reg: MULI2(reg,reg)  "    TAX\n    POP\n    MUL\n"  3
reg: MULU2(reg,reg)  "    TAX\n    POP\n    MUL\n"  3

/* Division */
reg: DIVI2(reg,reg)  "    TAX\n    POP\n    DIV\n"  3
reg: DIVU2(reg,reg)  "    TAX\n    POP\n    DIV\n"  3

/* Modulo */
reg: MODI2(reg,reg)  "    TAX\n    POP\n    MOD\n"  3
reg: MODU2(reg,reg)  "    TAX\n    POP\n    MOD\n"  3
```

#### Comparisons and Branches

```c
/* Compare two FP-relative variables */
stmt: LEI2(INDIRI2(faddr),INDIRI2(faddr))  "    LDA %1\n    STA _tmp\n    LDA %0\n    CMP _tmp\n    JLE %a\n"  3

/* Compare reg with FP-relative variable */
stmt: LEI2(reg,INDIRI2(faddr))  "    STA _tmp2\n    LDA %1\n    STA _tmp\n    LDA _tmp2\n    CMP _tmp\n    JLE %a\n"  4

/* Labels and jumps */
stmt: LABELV  "%a:\n"
stmt: JUMPV(addr)  "    JMP %0\n"  1
```

#### Function Calls

```c
/* Push arguments */
stmt: ARGI2(reg)  "    PUSH\n"  1
stmt: ARGU2(reg)  "    PUSH\n"  1
stmt: ARGP2(reg)  "    PUSH\n"  1

/* Call function */
reg: CALLI2(addr)  "    CALL %0\n"  5
reg: CALLU2(addr)  "    CALL %0\n"  5
reg: CALLP2(addr)  "    CALL %0\n"  5
stmt: CALLV(addr)  "    CALL %0\n"  5

/* Return value (in AC) */
stmt: RETI2(reg)  "; ret - value in AC\n"  0
stmt: RETU2(reg)  "; ret - value in AC\n"  0
stmt: RETP2(reg)  "; ret - value in AC\n"  0
stmt: RETV  "; ret void\n"  0
```

## Code Generation Examples

### Simple Function

C code:
```c
int add(int a, int b) {
    return a + b;
}
```

Generated assembly:
```asm
; Function: add
_add:
    ; Prologue
    PUSH_FP
    TSF
    ; Load a (FP+4), add b (FP+6)
    LDA 4,FP
    STA _tmp
    LDA 6,FP
    ADD _tmp
; ret - value in AC
    ; Epilogue
    TFS
    POP_FP
    RET
```

### Local Variables

C code:
```c
int main() {
    int x = 100;
    int y = 200;
    return x + y;
}
```

Generated assembly:
```asm
; Function: main
_main:
    ; Prologue
    PUSH_FP
    TSF
    ; Allocate 4 bytes for locals (x at -2, y at -4)
    LDI 0
    PUSH
    LDI 0
    PUSH
    ; x = 100 (stored at FP-2)
    LDI 100
    STA -2,FP
    ; y = 200 (stored at FP-4)
    LDI 200
    STA -4,FP
    ; return x + y
    LDA -2,FP
    STA _tmp
    LDA -4,FP
    ADD _tmp
; ret - value in AC
    ; Epilogue
    TFS
    POP_FP
    RET
```

### Factorial (Iterative)

C code:
```c
int factorial(int n) {
    int result = 1;
    int i = 1;
    while (i <= n) {
        result = result * i;
        i = i + 1;
    }
    return result;
}
```

Generated assembly:
```asm
; Function: factorial
_factorial:
    ; Prologue
    PUSH_FP
    TSF
    ; Allocate 6 bytes for locals
    ; result at -2, i at -4, temp at -6
    LDI 0
    PUSH
    LDI 0
    PUSH
    LDI 0
    PUSH
    ; result = 1
    LDI 1
    STA -4,FP
    ; i = 1
    LDI 1
    STA -2,FP
    JMP _L3
_L2:
    ; result = result * i
    LDA -2,FP
    TAX
    LDA -4,FP
    MUL
    STA -4,FP
    ; i = i + 1
    LDA -2,FP
    STA _tmp
    LDI 1
    ADD _tmp
    STA -2,FP
_L3:
    ; if (i <= n) goto L2
    LDA -2,FP
    STA _tmp2
    LDA 4,FP      ; n is parameter at FP+4
    STA _tmp
    LDA _tmp2
    CMP _tmp
    JLE _L2
    ; return result
    LDA -4,FP
; ret - value in AC
_L1:
    ; Epilogue
    TFS
    POP_FP
    RET
```

**Note:** The stack-based approach is essential for recursion. Each call to `factorial` gets its own stack frame with its own `result` and `i` variables.

## Runtime Variables

The backend emits these runtime support variables at program end:

```asm
; Runtime variables
_tmp:     .word 0     ; General purpose 16-bit temp
_tmp_hi:  .word 0     ; For 32-bit ops (high word)
_tmp2:    .word 0     ; Second 16-bit temp
_tmp2_hi: .word 0     ; For 32-bit ops (high word)
_mask_ff: .word 0x00FF ; Mask for 8-bit values

; End of program
    HLT
```

**Note:** VREGs are NOT stored in global memory slots. They are allocated on the stack via `local()` and accessed using FP-relative addressing (e.g., `LDA -2,FP`). This enables recursion to work correctly.

## Building and Using

### Compiling the Backend

```bash
# Generate C code from grammar
./lburg/lburg src/neanderx.md > src/neanderx.c

# Build LCC with Neander-X backend
make TARGET=neanderx
```

### Compiling C Code

```bash
# Generate assembly
./lcc -Wf-target=neanderx -S test.c

# Output goes to test.s
```

### Testing

The backend includes comprehensive tests in `cocotb_tests/test_lcc_samples.py`. See the Test Results section below for the full list.

## Critical Pitfalls and Solutions

### VREG Memory Slot Overlap

**Problem:** VREGs and local variables overlap in memory, causing wrong results.

**Symptoms:**
- Recursive functions fail (Fibonacci, factorial)
- Values mysteriously get overwritten
- Works for simple functions but fails for complex ones

**Root Cause:** The `emit2()` function uses hardcoded VREG offsets (e.g., `VREG_OFFSET(slot)`) while `local()` allocates variables at different offsets. These can overlap.

**Solution:** Use `reg->x.name` in `emit2()` which contains the offset already set by `local()`:

```c
/* CORRECT */
print("    STA %s,FP\n", reg->x.name);  /* Uses local()'s offset */

/* WRONG */
print("    STA %d,FP\n", VREG_OFFSET(slot));  /* Hardcoded offset */
```

### Stack Allocation in 16-bit Mode

**Problem:** Stack allocation loop allocates double the needed space.

**Symptoms:**
- Recursive functions timeout
- Stack corruption
- Fibonacci test fails

**Root Cause:** The allocation loop runs once per byte, but PUSH is a 16-bit operation:

```c
/* WRONG - allocates 2x the needed space */
for (i = 0; i < maxoffset; i++) {
    print("    PUSH\n");  /* Each PUSH is 2 bytes */
}

/* CORRECT - step by word size */
for (i = 0; i < maxoffset; i += 2) {
    print("    PUSH\n");
}
```

## Test Results

All 10 LCC test programs pass:

| Test | Description | Expected | Result |
|------|-------------|----------|--------|
| 01_hello | Return constant | 42 | PASS |
| 02_locals | Local variables | 300 | PASS |
| 03_arithmetic | Function calls | 100 | PASS |
| 04_globals | Global variables | 15 | PASS |
| 05_loop | While loop | 55 | PASS |
| 06_array | Array access | 150 | PASS |
| 07_factorial | factorial(5) | 120 | PASS |
| 08_fibonacci | fib(10) recursive | 55 | PASS |
| 09_bitwise | AND/OR/XOR ops | 8190 | PASS |
| 10_char | Char operations | 145 | PASS |

## Limitations

1. **No floating point** - Float/double operations not supported
2. **Limited 32-bit support** - Long operations use ADC/SBC pairs but are slow
3. **No indirect addressing** - Must use indexed addressing modes
4. **Single accumulator** - Complex expressions require temporary storage

## References

- [LCC Compiler Documentation](https://drh.github.io/lcc/)
- [NEANDER-X Instruction Set](../README.md)
- [LCC Backend Tutorial](LCC_COMPILER_COMPLETE.md)
- [LCC Porting Guide](PORTING_NEANDER.md) - Comprehensive tutorial with detailed pitfalls section
