# LCC Compiler Backend for NEANDER-X

This document describes how to create an LCC (Little C Compiler) backend targeting the NEANDER-X processor.

## Overview

The NEANDER-X is an 8-bit accumulator-based processor with the following characteristics that make it suitable as an LCC target:

- **8-bit data width** - All operations are 8-bit
- **Accumulator architecture** - Single AC register for most operations
- **Index registers** - X, Y, FP for array and stack frame access
- **Hardware arithmetic** - MUL, DIV, MOD instructions
- **Multi-byte support** - ADC, SBC for 16/32-bit arithmetic
- **Stack operations** - PUSH, POP, CALL, RET
- **Frame pointer** - Full support for C-style stack frames

## NEANDER-X Instruction Set Summary

### Instruction Categories for LCC

| Category | Instructions | LCC Usage |
|----------|--------------|-----------|
| **Data Movement** | LDA, STA, LDI, TAX, TXA, TAY, TYA | Load/store operations |
| **Arithmetic** | ADD, ADC, SUB, SBC, INC, DEC, NEG | Arithmetic operators |
| **Multiplication** | MUL (8x8->16) | MULI1, MULU1 |
| **Division** | DIV, MOD | DIVI1, DIVU1, MODI1, MODU1 |
| **Logic** | AND, OR, XOR, NOT | BANDI1, BORI1, BXORI1, BCOMI1 |
| **Shifts** | SHL, SHR, ASR | LSHI1, RSHU1, RSHI1 |
| **Comparison** | CMP | Sets flags for branches |
| **Branches** | JMP, JZ, JNZ, JN, JC, JNC, JLE, JGT, JGE, JBE, JA | Control flow |
| **Stack** | PUSH, POP, CALL, RET | Function calls |
| **Frame** | TSF, TFS, PUSH_FP, POP_FP, LDA_FP, STA_FP | Local variables |
| **Indexed** | LDA_X, STA_X, LDA_Y, STA_Y | Array access |

## Complete Opcode Reference

```
Opcode  Mnemonic    Operation                           Flags
------  --------    ---------                           -----
0x00    NOP         No operation                        -
0x01    NEG         AC = -AC                            N, Z, C
0x02    CMP addr    Compare AC with MEM[addr]           N, Z, C
0x03    TAY         Y = AC                              -
0x04    TYA         AC = Y                              N, Z
0x05    INY         Y = Y + 1                           -
0x06    LDYI imm    Y = imm                             -
0x07    LDY addr    Y = MEM[addr]                       -
0x08    STY addr    MEM[addr] = Y                       -
0x09    MUL         Y:AC = AC * X                       N, Z, C
0x0A    TSF         FP = SP                             -
0x0B    TFS         SP = FP                             -
0x0C    PUSH_FP     MEM[--SP] = FP                      -
0x0D    POP_FP      FP = MEM[SP++]                      -
0x0E    DIV         AC = AC / X, Y = remainder          N, Z, C
0x0F    MOD         AC = AC % X, Y = quotient           N, Z, C
0x10    STA addr    MEM[addr] = AC                      -
0x11    STA_X addr  MEM[addr + X] = AC                  -
0x12    STA_Y addr  MEM[addr + Y] = AC                  -
0x14    STA_FP off  MEM[FP + off] = AC                  -
0x20    LDA addr    AC = MEM[addr]                      N, Z
0x21    LDA_X addr  AC = MEM[addr + X]                  N, Z
0x22    LDA_Y addr  AC = MEM[addr + Y]                  N, Z
0x24    LDA_FP off  AC = MEM[FP + off]                  N, Z
0x30    ADD addr    AC = AC + MEM[addr]                 N, Z, C
0x31    ADC addr    AC = AC + MEM[addr] + C             N, Z, C
0x40    OR addr     AC = AC | MEM[addr]                 N, Z
0x50    AND addr    AC = AC & MEM[addr]                 N, Z
0x51    SBC addr    AC = AC - MEM[addr] - C             N, Z, C
0x60    NOT         AC = ~AC                            N, Z
0x61    ASR         AC = AC >> 1 (arithmetic)           N, Z, C
0x70    PUSH        MEM[--SP] = AC                      -
0x71    POP         AC = MEM[SP++]                      N, Z
0x72    CALL addr   MEM[--SP] = PC; PC = addr           -
0x73    RET         PC = MEM[SP++]                      -
0x74    SUB addr    AC = AC - MEM[addr]                 N, Z, C
0x75    INC         AC = AC + 1                         N, Z
0x76    DEC         AC = AC - 1                         N, Z
0x77    XOR addr    AC = AC ^ MEM[addr]                 N, Z
0x78    SHL         AC = AC << 1                        N, Z, C
0x79    SHR         AC = AC >> 1 (logical)              N, Z, C
0x7A    LDX addr    X = MEM[addr]                       -
0x7B    STX addr    MEM[addr] = X                       -
0x7C    LDXI imm    X = imm                             -
0x7D    TAX         X = AC                              -
0x7E    TXA         AC = X                              N, Z
0x7F    INX         X = X + 1                           -
0x80    JMP addr    PC = addr                           -
0x81    JC addr     if C: PC = addr                     -
0x82    JNC addr    if !C: PC = addr                    -
0x83    JLE addr    if N|Z: PC = addr                   -
0x84    JGT addr    if !N&!Z: PC = addr                 -
0x85    JGE addr    if !N: PC = addr                    -
0x86    JBE addr    if C|Z: PC = addr                   -
0x87    JA addr     if !C&!Z: PC = addr                 -
0x90    JN addr     if N: PC = addr                     -
0xA0    JZ addr     if Z: PC = addr                     -
0xB0    JNZ addr    if !Z: PC = addr                    -
0xC0    IN port     AC = IO_IN                          N, Z
0xD0    OUT port    IO_OUT = AC                         -
0xE0    LDI imm     AC = imm                            N, Z
0xF0    HLT         Halt execution                      -
```

## LCC Backend Implementation

### Type Metrics

For an 8-bit processor, define type sizes in the Interface structure:

```c
Interface neanderIR = {
    /* charmetric:   size, align, outofline */
    1, 1, 0,    /* char: 1 byte */
    1, 1, 0,    /* short: 1 byte (8-bit processor) */
    1, 1, 0,    /* int: 1 byte */
    2, 1, 0,    /* long: 2 bytes (use ADC/SBC) */
    2, 1, 0,    /* long long: 2 bytes (limited support) */
    0, 1, 1,    /* float: not supported (outofline) */
    0, 1, 1,    /* double: not supported */
    0, 1, 1,    /* long double: not supported */
    1, 1, 0,    /* pointer: 1 byte (8-bit address space) */
    0, 1, 0,    /* struct: calculated */
    ...
};
```

### Register Mapping

The NEANDER-X has limited registers:

| Register | LCC Usage | Notes |
|----------|-----------|-------|
| AC | Result register | Primary accumulator |
| X | Index/temp | Array indexing |
| Y | Index/temp | Second index, MUL high byte |
| FP | Frame pointer | Local variable access |
| SP | Stack pointer | Implicit, managed by PUSH/POP |

```c
/* Register symbols */
static Symbol ac_reg;   /* Accumulator */
static Symbol x_reg;    /* X index */
static Symbol y_reg;    /* Y index */
static Symbol fp_reg;   /* Frame pointer */

static void progbeg(int argc, char *argv[]) {
    ac_reg = mkreg("AC", 0, 1, IREG);
    x_reg  = mkreg("X", 1, 1, IREG);
    y_reg  = mkreg("Y", 2, 1, IREG);
    /* FP and SP are implicit */
}
```

### lburg Rules (.md file)

#### Constants and Addresses

```c
%term CNSTI1=1045 CNSTU1=1046

/* Load immediate constant */
reg: CNSTI1  "LDI %a\n"  1
reg: CNSTU1  "LDI %a\n"  1

/* Address of global */
addr: ADDRGP1  "%a"
addr: ADDRFP1  "%a"    /* FP-relative offset */
addr: ADDRLP1  "%a"    /* Local (FP-relative) */
```

#### Load and Store

```c
%term INDIRI1=1093 INDIRU1=1094
%term ASGNI1=1077 ASGNU1=1078

/* Load from memory */
reg: INDIRI1(addr)  "LDA %0\n"  1
reg: INDIRU1(addr)  "LDA %0\n"  1

/* Store to memory */
stmt: ASGNI1(addr,reg)  "STA %0\n"  1
stmt: ASGNU1(addr,reg)  "STA %0\n"  1

/* FP-indexed load/store for locals */
reg: INDIRI1(ADDRFP1)   "LDA_FP %a\n"  1
stmt: ASGNI1(ADDRFP1,reg)  "STA_FP %a\n"  1
```

#### Arithmetic Operations

```c
%term ADDI1=1173 ADDU1=1174
%term SUBI1=1189 SUBU1=1190
%term NEGI1=1061

/* Addition: AC = AC + MEM[addr] */
reg: ADDI1(reg,mem)  "ADD %1\n"  1
reg: ADDU1(reg,mem)  "ADD %1\n"  1

/* Subtraction: AC = AC - MEM[addr] */
reg: SUBI1(reg,mem)  "SUB %1\n"  1
reg: SUBU1(reg,mem)  "SUB %1\n"  1

/* Negation: AC = -AC */
reg: NEGI1(reg)  "NEG\n"  1

/* Increment/Decrement */
reg: ADDI1(reg,one)  "INC\n"  1
reg: SUBI1(reg,one)  "DEC\n"  1
```

#### Multi-Byte Arithmetic (16-bit)

For 16-bit operations, use ADC/SBC with carry propagation:

```c
/* 16-bit addition pattern:
   ADD low bytes (sets carry)
   ADC high bytes (uses carry)
*/
reg: ADDI2(reg,mem)  "# 16-bit add\n"  emit_add16

/* Custom emission function */
static void emit_add16(Node p) {
    /* Low byte: AC = low(AC) + low(MEM) */
    print("LDA %s_lo\n", p->kids[0]->syms[0]->x.name);
    print("ADD %s_lo\n", p->kids[1]->syms[0]->x.name);
    print("STA result_lo\n");
    /* High byte: AC = high(AC) + high(MEM) + Carry */
    print("LDA %s_hi\n", p->kids[0]->syms[0]->x.name);
    print("ADC %s_hi\n", p->kids[1]->syms[0]->x.name);
    print("STA result_hi\n");
}
```

#### Multiplication

```c
%term MULI1=1269 MULU1=1270

/* 8x8 multiplication: Y:AC = AC * X */
reg: MULI1(reg,xreg)  "MUL\n"  10
reg: MULU1(reg,xreg)  "MUL\n"  10

/* Setup: move operand to X first */
stmt: ARGI1(reg)  "TAX\n"  1  /* Move to X for MUL */
```

#### Division and Modulo

```c
%term DIVI1=1253 DIVU1=1254
%term MODI1=1221 MODU1=1222

/* Division: AC = AC / X, remainder in Y */
reg: DIVI1(reg,xreg)  "DIV\n"  20
reg: DIVU1(reg,xreg)  "DIV\n"  20

/* Modulo: AC = AC % X, quotient in Y */
reg: MODI1(reg,xreg)  "MOD\n"  20
reg: MODU1(reg,xreg)  "MOD\n"  20
```

#### Logical Operations

```c
%term BANDI1=1285 BANDU1=1286
%term BORI1=1317 BORU1=1318
%term BXORI1=1333 BXORU1=1334
%term BCOMI1=1301 BCOMU1=1302

/* AND: AC = AC & MEM[addr] */
reg: BANDI1(reg,mem)  "AND %1\n"  1
reg: BANDU1(reg,mem)  "AND %1\n"  1

/* OR: AC = AC | MEM[addr] */
reg: BORI1(reg,mem)   "OR %1\n"   1
reg: BORU1(reg,mem)   "OR %1\n"   1

/* XOR: AC = AC ^ MEM[addr] */
reg: BXORI1(reg,mem)  "XOR %1\n"  1
reg: BXORU1(reg,mem)  "XOR %1\n"  1

/* NOT: AC = ~AC */
reg: BCOMI1(reg)  "NOT\n"  1
reg: BCOMU1(reg)  "NOT\n"  1
```

#### Shift Operations

```c
%term LSHI1=1205 LSHU1=1206
%term RSHI1=1237 RSHU1=1238

/* Left shift: AC = AC << 1 */
reg: LSHI1(reg,one)  "SHL\n"  1
reg: LSHU1(reg,one)  "SHL\n"  1

/* Right shift (logical): AC = AC >> 1 */
reg: RSHU1(reg,one)  "SHR\n"  1

/* Arithmetic right shift (preserves sign) */
reg: RSHI1(reg,one)  "ASR\n"  1

/* Multi-bit shifts require loops */
reg: LSHI1(reg,con)  "# shift loop\n"  shift_left_n
reg: RSHI1(reg,con)  "# shift loop\n"  shift_right_n
```

#### Comparisons and Branches

```c
%term EQI1=1349 NEI1=1365 LTI1=1381 LEI1=1397 GTI1=1413 GEI1=1429
%term LTU1=1382 LEU1=1398 GTU1=1414 GEU1=1430

/* Signed comparisons (after CMP) */
stmt: EQI1(reg,mem)  "CMP %1\nJZ %a\n"   2
stmt: NEI1(reg,mem)  "CMP %1\nJNZ %a\n"  2
stmt: LTI1(reg,mem)  "CMP %1\nJN %a\n"   2  /* Less: N flag set */
stmt: LEI1(reg,mem)  "CMP %1\nJLE %a\n"  2  /* Less or equal */
stmt: GTI1(reg,mem)  "CMP %1\nJGT %a\n"  2  /* Greater */
stmt: GEI1(reg,mem)  "CMP %1\nJGE %a\n"  2  /* Greater or equal */

/* Unsigned comparisons */
stmt: LTU1(reg,mem)  "CMP %1\nJC %a\n"   2  /* Below: Carry set */
stmt: LEU1(reg,mem)  "CMP %1\nJBE %a\n"  2  /* Below or equal */
stmt: GTU1(reg,mem)  "CMP %1\nJA %a\n"   2  /* Above */
stmt: GEU1(reg,mem)  "CMP %1\nJNC %a\n"  2  /* Above or equal */

/* Unconditional jump */
stmt: JUMPV(addr)  "JMP %0\n"  1
stmt: LABELV       "%a:\n"
```

#### Function Calls

```c
%term CALLI1=1077 CALLU1=1078 CALLV=216
%term RETI1=1109 RETU1=1110 RETV=248
%term ARGI1=901 ARGU1=902

/* Function call */
reg: CALLI1(addr)   "CALL %0\n"  1
reg: CALLU1(addr)   "CALL %0\n"  1
stmt: CALLV(addr)   "CALL %0\n"  1

/* Push argument (right-to-left) */
stmt: ARGI1(reg)  "PUSH\n"  1
stmt: ARGU1(reg)  "PUSH\n"  1

/* Return value (in AC) */
stmt: RETI1(reg)  "# return in AC\n"  1
stmt: RETU1(reg)  "# return in AC\n"  1
stmt: RETV        "# return void\n"
```

### Function Prologue/Epilogue

```c
static void function(Symbol f, Symbol caller[], Symbol callee[], int ncalls) {
    int i;

    /* Emit function label */
    print("\n; Function: %s\n", f->name);
    print("%s:\n", f->x.name);

    /* Prologue: save FP, set up new frame */
    print("    PUSH_FP         ; Save caller's FP\n");
    print("    TSF             ; FP = SP\n");

    /* Calculate parameter offsets (passed on stack) */
    offset = 2;  /* After saved FP and return address */
    for (i = 0; callee[i]; i++) {
        Symbol p = callee[i];
        Symbol q = caller[i];
        p->x.offset = q->x.offset = offset;
        p->x.name = q->x.name = stringf("%d", offset);
        offset += 1;  /* 1 byte per parameter */
    }

    /* Generate code for function body */
    offset = maxoffset = 0;
    gencode(caller, callee);

    /* Allocate space for locals (if any) */
    if (maxoffset > 0) {
        for (i = 0; i < maxoffset; i++) {
            print("    PUSH            ; Allocate local\n");
        }
    }

    /* Emit generated code */
    emitcode();

    /* Epilogue: restore SP and FP, return */
    print("    TFS             ; SP = FP\n");
    print("    POP_FP          ; Restore caller's FP\n");
    print("    RET\n");
}
```

### Local Variable Access

```c
static void local(Symbol p) {
    /* All locals are on the stack, accessed via FP */
    offset = roundup(offset + p->type->size, 1);
    p->x.offset = -offset;  /* Negative offset from FP */
    p->x.name = stringf("%d", -offset);
}

/* In .md file: */
reg: INDIRI1(ADDRLP1)  "LDA_FP %a\n"  1
stmt: ASGNI1(ADDRLP1,reg)  "STA_FP %a\n"  1
```

### Array Access with Index Registers

```c
/* Array load: arr[i] where i is in X */
reg: INDIRI1(ADDI1(addr,xreg))  "LDA_X %0\n"  1

/* Array store */
stmt: ASGNI1(ADDI1(addr,xreg),reg)  "STA_X %0\n"  1

/* Setup index: move index value to X */
xreg: reg  "TAX\n"  1
```

## Code Generation Examples

### Simple Addition

C code:
```c
char add(char a, char b) {
    return a + b;
}
```

Generated assembly:
```asm
_add:
    PUSH_FP         ; Save FP
    TSF             ; FP = SP
    LDA_FP 2        ; Load 'a' (first param)
    PUSH            ; Save to temp
    LDA_FP 3        ; Load 'b' (second param)
    TAX             ; Move to X
    POP             ; Restore 'a' to AC
    ADD temp        ; AC = a + b (using temp workaround)
    TFS             ; SP = FP
    POP_FP          ; Restore FP
    RET
```

### Multiplication

C code:
```c
char multiply(char a, char b) {
    return a * b;
}
```

Generated assembly:
```asm
_multiply:
    PUSH_FP
    TSF
    LDA_FP 3        ; Load 'b'
    TAX             ; X = b
    LDA_FP 2        ; Load 'a'
    MUL             ; Y:AC = a * b (low byte in AC)
    TFS
    POP_FP
    RET
```

### Division with Remainder

C code:
```c
char divide(char a, char b, char *remainder) {
    *remainder = a % b;
    return a / b;
}
```

Generated assembly:
```asm
_divide:
    PUSH_FP
    TSF
    LDA_FP 3        ; Load 'b'
    TAX             ; X = b (divisor)
    LDA_FP 2        ; Load 'a'
    DIV             ; AC = quotient, Y = remainder
    PUSH            ; Save quotient
    TYA             ; AC = remainder
    STA_FP 4        ; Store to *remainder (param 3)
    POP             ; Restore quotient
    TFS
    POP_FP
    RET
```

### 16-bit Addition

C code:
```c
int add16(int a, int b) {  /* int = 16 bits */
    return a + b;
}
```

Generated assembly:
```asm
_add16:
    PUSH_FP
    TSF
    ; Add low bytes
    LDA_FP 2        ; a_lo
    ADD a_temp_lo   ; a_lo + b_lo (must load b_lo first)
    PUSH            ; Save to get b_lo
    LDA_FP 4        ; b_lo
    STA temp
    POP
    ADD temp        ; AC = a_lo + b_lo, carry set if overflow
    STA result_lo
    ; Add high bytes with carry
    LDA_FP 3        ; a_hi
    PUSH
    LDA_FP 5        ; b_hi
    STA temp
    POP
    ADC temp        ; AC = a_hi + b_hi + carry
    STA result_hi
    ; Return 16-bit result (convention needed)
    TFS
    POP_FP
    RET
```

### Array Sum

C code:
```c
char sum_array(char *arr, char len) {
    char sum = 0;
    char i;
    for (i = 0; i < len; i++) {
        sum += arr[i];
    }
    return sum;
}
```

Generated assembly:
```asm
_sum_array:
    PUSH_FP
    TSF
    ; Allocate locals: sum, i
    PUSH            ; sum at FP-1
    PUSH            ; i at FP-2

    ; sum = 0
    LDI 0
    STA_FP -1

    ; i = 0
    LDI 0
    STA_FP -2

.loop:
    ; if (i >= len) goto end
    LDA_FP -2       ; Load i
    CMP_FP 3        ; Compare with len
    JGE .end

    ; sum += arr[i]
    LDA_FP -2       ; Load i
    TAX             ; X = i
    LDA_FP 2        ; Load arr base address
    ; Need to do indexed load
    ; ... (complex - may need helper)

    ; i++
    LDA_FP -2
    INC
    STA_FP -2
    JMP .loop

.end:
    LDA_FP -1       ; Return sum
    TFS
    POP_FP
    RET
```

## Limitations and Workarounds

### Single Accumulator

The NEANDER-X has only one accumulator, which limits complex expressions. The LCC backend must:

1. Use temporary memory locations for intermediate results
2. Spill to stack frequently
3. Use X/Y registers for second operands when possible

### No Direct Register-to-Register Operations

Most ALU operations require a memory operand:
- `ADD addr` not `ADD X`

Workaround: Store register to temp, then use temp:
```asm
; To add X to AC:
STX temp
ADD temp
```

### Limited Addressing Modes

Only indexed addressing with X, Y, or FP offsets. No indirect addressing like `LDA (ptr)`.

Workaround: Self-modifying code or helper functions (not recommended for ROM).

### 8-bit Address Space

Only 256 bytes addressable. Programs and data must fit in this space.

Considerations:
- Keep functions small
- Use overlays or bank switching for larger programs
- Prioritize frequently-used data in low memory

## Testing the Backend

### Test Program

```c
/* test.c - Basic functionality test */
char global_var = 10;

char add(char a, char b) {
    return a + b;
}

char factorial(char n) {
    if (n <= 1) return 1;
    return n * factorial(n - 1);
}

char main() {
    char x = 5;
    char y = 3;
    char sum = add(x, y);
    char fact = factorial(5);
    return sum + fact;  /* Should return 8 + 120 = 128 */
}
```

### Compilation

```bash
# Generate NEANDER-X assembly
./lcc -Wf-target=neander -S test.c

# Assemble (with custom assembler)
./nasm test.s -o test.bin

# Run in simulator
./nsim test.bin
```

## Future Enhancements

1. **Peephole Optimization**: Eliminate redundant load/store pairs
2. **Register Allocation**: Better use of X and Y registers
3. **16-bit Support**: Full 16-bit int support using ADC/SBC pairs
4. **Inline Assembly**: Support for `__asm__` blocks
5. **Interrupt Handling**: Support for ISR functions

## References

- [LCC Compiler Documentation](https://drh.github.io/lcc/)
- [NEANDER-X Instruction Set](../README.md)
- [LCC Backend Tutorial](LCC_COMPILER_COMPLETE.md)
