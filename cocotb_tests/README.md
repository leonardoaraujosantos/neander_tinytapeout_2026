# Neander CPU - Cocotb Tests

Testes automatizados para a CPU Neander usando cocotb.

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

## Testes Incluídos

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

## Programa Principal: f(a,b) = a*5 + b

O teste principal calcula `f(a,b) = a*5 + b` usando um loop (sem instrução de multiplicação):

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

## Estrutura de Arquivos

```
cocotb_tests/
├── Makefile              # Makefile para cocotb
├── neander_tb_wrapper.sv # Wrapper com RAM e interface de carga
├── test_neander.py       # Testes Python
└── README.md             # Este arquivo
```

## Implementação Correta

O arquivo `src/top_cpu_2.sv` é a implementação correta da CPU Neander com:
- Suporte completo a I/O (IN/OUT)
- Instrução LDI (0xE)
- Instrução HLT (0xF)
- Interface externa para RAM

O arquivo `src/top_cpu.sv` tem problemas:
- Não suporta instruções I/O
- Opcode LDI/HLT incorreto no programa embarcado
