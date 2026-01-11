# Neander CPU - Cocotb Tests

Testes automatizados para a CPU Neander-X usando cocotb.

## Requisitos

```bash
pip install cocotb
```

Para simuladores:
- **Icarus Verilog** (recomendado): `brew install icarus-verilog`
- **Verilator**: `brew install verilator`

## Executar Testes

```bash
cd cocotb_tests
make
```

Para usar Verilator:
```bash
make SIM=verilator
```

## Testes Incluidos

O arquivo de testes contém **123 testes automatizados** cobrindo todas as instruções do NEANDER-X.

### Testes Básicos

| Teste | Descrição |
|-------|-----------|
| `test_multiply_by_5_basic` | f(3,7) = 3*5 + 7 = 22 |
| `test_multiply_by_5_larger` | f(10,5) = 10*5 + 5 = 55 |
| `test_multiply_by_5_overflow` | f(50,10) = 260 & 0xFF = 4 |
| `test_multiply_by_5_zero` | f(0,42) = 0*5 + 42 = 42 |
| `test_simple_add` | 10 + 20 = 30 |
| `test_loop_countdown` | Conta de 10 até 0 |
| `test_logical_operations` | AND, OR, NOT |
| `test_conditional_jump_jn` | Teste de JN (jump if negative) |

### Testes LCC Extension (NEG, CMP, JC, JNC)

| Teste | Descrição |
|-------|-----------|
| `test_neg_positive` | NEG de valor positivo |
| `test_neg_negative` | NEG de valor negativo |
| `test_neg_zero` | NEG de zero |
| `test_neg_sets_n_flag` | NEG atualiza flag N |
| `test_neg_sets_z_flag` | NEG atualiza flag Z |
| `test_cmp_equal` | CMP valores iguais (Z=1) |
| `test_cmp_not_equal` | CMP valores diferentes |
| `test_cmp_preserves_ac` | CMP não modifica AC |
| `test_cmp_less_than_sets_n` | CMP menor que (N=1) |
| `test_jc_taken` | JC salta quando C=1 |
| `test_jc_not_taken` | JC não salta quando C=0 |
| `test_jnc_taken` | JNC salta quando C=0 |
| `test_jnc_not_taken` | JNC não salta quando C=1 |
| `test_sub_sets_carry` | SUB atualiza flag C |
| `test_cmp_sets_carry_on_borrow` | CMP atualiza flag C |

### Testes Y Register Extension

| Teste | Descrição |
|-------|-----------|
| `test_tay_basic` | TAY: Y = AC |
| `test_tya_sets_flags` | TYA: AC = Y (atualiza flags) |
| `test_iny_basic` | INY: Y = Y + 1 |
| `test_ldyi_basic` | LDYI imm: Y = imediato |
| `test_ldy_basic` | LDY addr: Y = MEM[addr] |
| `test_sty_basic` | STY addr: MEM[addr] = Y |
| `test_lda_indexed_y` | LDA addr,Y: AC = MEM[addr + Y] |
| `test_sta_indexed_y` | STA addr,Y: MEM[addr + Y] = AC |
| `test_array_copy_xy` | Cópia de array usando X e Y |

### Testes MUL Instruction

| Teste | Descrição |
|-------|-----------|
| `test_mul_basic` | 5 * 3 = 15 (resultado 8-bit) |
| `test_mul_zero` | 42 * 0 = 0 |
| `test_mul_one` | 42 * 1 = 42 |
| `test_mul_16bit_result` | 16 * 16 = 256 (Y=1, AC=0) |
| `test_mul_large_result` | 200 * 200 = 40000 (Y=0x9C, AC=0x40) |
| `test_mul_max_values` | 255 * 255 = 65025 (Y=0xFE, AC=0x01) |
| `test_mul_sets_carry_on_overflow` | MUL seta carry quando overflow |
| `test_mul_no_carry_small_result` | MUL não seta carry para resultado pequeno |
| `test_mul_factorial_5` | Calcula 5! = 120 |
| `test_mul_power_of_2` | Calcula 2^8 = 256 |

### Testes Comparison Jumps (JLE, JGT, JGE, JBE, JA)

| Teste | Descrição |
|-------|-----------|
| `test_jle_taken_less` | JLE salta quando AC < valor (N=1) |
| `test_jle_taken_equal` | JLE salta quando AC == valor (Z=1) |
| `test_jle_not_taken` | JLE não salta quando AC > valor |
| `test_jgt_taken` | JGT salta quando AC > valor (N=0, Z=0) |
| `test_jgt_not_taken_less` | JGT não salta quando AC < valor |
| `test_jgt_not_taken_equal` | JGT não salta quando AC == valor |
| `test_jge_taken_greater` | JGE salta quando AC > valor (N=0) |
| `test_jge_taken_equal` | JGE salta quando AC == valor |
| `test_jge_not_taken` | JGE não salta quando AC < valor |
| `test_jbe_taken_below` | JBE salta quando AC < valor unsigned (C=1) |
| `test_jbe_taken_equal` | JBE salta quando AC == valor (Z=1) |
| `test_jbe_not_taken` | JBE não salta quando AC > valor unsigned |
| `test_ja_taken` | JA salta quando AC > valor unsigned (C=0, Z=0) |
| `test_ja_not_taken_below` | JA não salta quando AC < valor unsigned |
| `test_ja_not_taken_equal` | JA não salta quando AC == valor |

### Testes Frame Pointer Extension

| Teste | Descrição |
|-------|-----------|
| `test_tsf_basic` | TSF: FP = SP |
| `test_tfs_basic` | TFS: SP = FP |
| `test_push_fp_basic` | PUSH_FP: MEM[--SP] = FP |
| `test_pop_fp_basic` | POP_FP: FP = MEM[SP++] |
| `test_lda_fp_indexed` | LDA addr,FP: AC = MEM[addr + FP] |
| `test_sta_fp_indexed` | STA addr,FP: MEM[addr + FP] = AC |
| `test_function_prologue_epilogue` | Testa padrão PUSH_FP/TSF e TFS/POP_FP |
| `test_fp_local_variable_access` | Acesso a variáveis locais via FP |
| `test_fp_parameter_access` | Acesso a parâmetros via FP |

## Instruction Set Reference

### Instruções Básicas (Neander Original)

| Opcode | Mnemonic | Operação |
|--------|----------|----------|
| 0x00 | NOP | No operation |
| 0x10 | STA addr | MEM[addr] = AC |
| 0x20 | LDA addr | AC = MEM[addr] |
| 0x30 | ADD addr | AC = AC + MEM[addr] |
| 0x40 | OR addr | AC = AC \| MEM[addr] |
| 0x50 | AND addr | AC = AC & MEM[addr] |
| 0x60 | NOT | AC = ~AC |
| 0x80 | JMP addr | PC = addr |
| 0x90 | JN addr | if (N) PC = addr |
| 0xA0 | JZ addr | if (Z) PC = addr |
| 0xB0 | JNZ addr | if (!Z) PC = addr |
| 0xC0 | IN port | AC = IO[port] |
| 0xD0 | OUT port | IO[port] = AC |
| 0xE0 | LDI imm | AC = imm |
| 0xF0 | HLT | Halt |

### Stack Extension

| Opcode | Mnemonic | Operação |
|--------|----------|----------|
| 0x70 | PUSH | MEM[--SP] = AC |
| 0x71 | POP | AC = MEM[SP++] |
| 0x72 | CALL addr | MEM[--SP] = PC; PC = addr |
| 0x73 | RET | PC = MEM[SP++] |

### LCC Compiler Extension

| Opcode | Mnemonic | Operação |
|--------|----------|----------|
| 0x01 | NEG | AC = -AC (two's complement) |
| 0x02 | CMP addr | Flags = AC - MEM[addr] (AC unchanged) |
| 0x74 | SUB addr | AC = AC - MEM[addr] |
| 0x75 | INC | AC = AC + 1 |
| 0x76 | DEC | AC = AC - 1 |
| 0x77 | XOR addr | AC = AC ^ MEM[addr] |
| 0x78 | SHL | AC = AC << 1 (C = MSB) |
| 0x79 | SHR | AC = AC >> 1 (C = LSB) |
| 0x81 | JC addr | if (C) PC = addr |
| 0x82 | JNC addr | if (!C) PC = addr |
| 0x83 | JLE addr | if (N=1 OR Z=1) PC = addr |
| 0x84 | JGT addr | if (N=0 AND Z=0) PC = addr |
| 0x85 | JGE addr | if (N=0) PC = addr |
| 0x86 | JBE addr | if (C=1 OR Z=1) PC = addr |
| 0x87 | JA addr | if (C=0 AND Z=0) PC = addr |

### X Register Extension

| Opcode | Mnemonic | Operação |
|--------|----------|----------|
| 0x7A | LDX addr | X = MEM[addr] |
| 0x7B | STX addr | MEM[addr] = X |
| 0x7C | LDXI imm | X = imm |
| 0x7D | TAX | X = AC |
| 0x7E | TXA | AC = X |
| 0x7F | INX | X = X + 1 |
| 0x21 | LDA addr,X | AC = MEM[addr + X] |
| 0x11 | STA addr,X | MEM[addr + X] = AC |

### Y Register Extension

| Opcode | Mnemonic | Operação |
|--------|----------|----------|
| 0x03 | TAY | Y = AC |
| 0x04 | TYA | AC = Y |
| 0x05 | INY | Y = Y + 1 |
| 0x06 | LDYI imm | Y = imm |
| 0x07 | LDY addr | Y = MEM[addr] |
| 0x08 | STY addr | MEM[addr] = Y |
| 0x22 | LDA addr,Y | AC = MEM[addr + Y] |
| 0x12 | STA addr,Y | MEM[addr + Y] = AC |

### Frame Pointer Extension

| Opcode | Mnemonic | Operação |
|--------|----------|----------|
| 0x0A | TSF | FP = SP |
| 0x0B | TFS | SP = FP |
| 0x0C | PUSH_FP | MEM[--SP] = FP |
| 0x0D | POP_FP | FP = MEM[SP++] |
| 0x24 | LDA addr,FP | AC = MEM[addr + FP] |
| 0x14 | STA addr,FP | MEM[addr + FP] = AC |

### Multiplication Instruction

| Opcode | Mnemonic | Operação |
|--------|----------|----------|
| 0x09 | MUL | Y:AC = AC * X (16-bit result) |

The MUL instruction multiplies AC by X using a combinational 8x8 multiplier. The 16-bit result is stored with the high byte in Y and the low byte in AC. The carry flag is set if the result overflows 8 bits (high byte != 0).

## Programa Principal: f(a,b) = a*5 + b

O teste principal calcula `f(a,b) = a*5 + b` usando um loop:

```assembly
; Dados em 0x80-0x84
        LDI  a          ; Carregar valor de a
        STA  0x80       ; A = a
        LDI  b          ; Carregar valor de b
        STA  0x81       ; B = b
        LDI  5          ; Contador = 5
        STA  0x83       ; COUNT = 5
        LDI  0          ; Resultado = 0
        STA  0x82       ; RESULT = 0
        LDI  0xFF       ; Constante -1
        STA  0x84       ; NEG1 = -1

LOOP:   LDA  0x83       ; AC = COUNT
        JZ   DONE       ; Se COUNT==0, sai do loop
        LDA  0x82       ; AC = RESULT
        ADD  0x80       ; AC += A
        STA  0x82       ; RESULT = AC
        LDA  0x83       ; AC = COUNT
        ADD  0x84       ; AC += NEG1 (decrementa)
        STA  0x83       ; COUNT = AC
        JMP  LOOP

DONE:   LDA  0x82       ; AC = RESULT
        ADD  0x81       ; AC += B
        STA  0x82       ; RESULT = AC
        OUT  0x00       ; Envia resultado para porta 0
        HLT             ; Para
```

Com a instrução MUL, o mesmo programa pode ser simplificado:

```assembly
        LDI  a          ; AC = a
        LDXI 5          ; X = 5
        MUL             ; AC = a * 5 (low byte)
        ADD  0x81       ; AC += b
        OUT  0x00       ; Output result
        HLT
```

## Estrutura de Arquivos

```
cocotb_tests/
├── Makefile              # Makefile para cocotb
├── neander_tb_wrapper.sv # Wrapper com RAM e interface de carga
├── test_neander.py       # Testes Python (123 testes)
└── README.md             # Este arquivo
```

## Implementação

O arquivo `src/top_cpu_neander_x.sv` é a implementação principal da CPU Neander-X com:
- Suporte completo a I/O (IN/OUT)
- Stack operations (PUSH, POP, CALL, RET)
- LCC compiler extension (NEG, CMP, SUB, INC, DEC, XOR, SHL, SHR, JC, JNC)
- X register extension (LDX, STX, LDXI, TAX, TXA, INX, indexed addressing)
- Y register extension (LDY, STY, LDYI, TAY, TYA, INY, indexed addressing)
- Frame pointer extension (TSF, TFS, PUSH_FP, POP_FP, FP-indexed addressing)
- Hardware multiplication (MUL: AC * X -> Y:AC)
- Carry flag (C) for arithmetic overflow detection
- Interface externa para RAM
