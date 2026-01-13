Perfeito. Abaixo está o documento completo, já no formato Markdown, pronto para você salvar como:

SPI_PROTOCOL.md

Ele é escrito pensando explicitamente no seu caso: CPU 8-bits, endereçamento 16-bits, memória SPI, implementação em SystemVerilog com FSM clara.

⸻

SPI_PROTOCOL.md

1. Introdução

O SPI (Serial Peripheral Interface) é um protocolo de comunicação serial síncrono, amplamente utilizado para conectar microprocessadores, microcontroladores e FPGAs a dispositivos periféricos como:
 • Memórias SPI (Flash, SRAM, FRAM)
 • Sensores
 • Conversores ADC/DAC
 • Displays

Neste projeto, o SPI será usado como barramento de memória externa para um processador 8-bits com endereçamento de 16 bits, devido à sua alta velocidade, baixo overhead e simplicidade de implementação em hardware.

⸻

1. Conceitos Fundamentais do SPI

2.1 Arquitetura Mestre–Escravo

O SPI opera em modelo master–slave:
 • Master: CPU / FPGA / SoC
 • Slave: memória SPI

O master:
 • gera o clock
 • controla o chip select
 • inicia toda comunicação

⸻

2.2 Sinais do SPI

Sinal Direção Descrição
SCLK Master → Slave Clock serial
MOSI Master → Slave Master Out, Slave In
MISO Slave → Master Master In, Slave Out
CS / SS Master → Slave Chip Select (ativo em nível baixo)

📌 Importante:
O SPI não possui endereçamento interno como o I²C. Cada dispositivo é selecionado por um CS dedicado.

⸻

1. Funcionamento Básico do SPI

3.1 Transferência de Dados
 • Cada pulso de clock transfere 1 bit
 • Transferência full-duplex
 • Dados são deslocados em registradores de shift

Exemplo (8 bits):

Clock:  ↑ ↓ ↑ ↓ ↑ ↓ ↑ ↓ ↑ ↓ ↑ ↓ ↑ ↓ ↑ ↓
MOSI :  b7 b6 b5 b4 b3 b2 b1 b0
MISO :  d7 d6 d5 d4 d3 d2 d1 d0

⸻

3.2 Modos SPI (CPOL / CPHA)

Modo CPOL CPHA Descrição
0 0 0 Dados válidos na subida
1 0 1 Dados válidos na descida
2 1 0 Clock idle alto
3 1 1 Clock idle alto + shift invertido

📌 Memórias SPI geralmente usam Modo 0 ou 3.
Este documento assume SPI Modo 0 (CPOL=0, CPHA=0).

⸻

1. Protocolo SPI para Memórias

4.1 Estrutura Típica de Acesso

Memórias SPI usam o formato:

[COMANDO][ENDEREÇO_MSB][ENDEREÇO_LSB][DADOS...]

Exemplo (leitura):

0x03  0x12  0x34  -> retorna DATA

Campo Tamanho
Comando 8 bits
Endereço 16 bits
Dados 8 bits (ou burst)

⸻

4.2 Comandos Comuns

Operação Código
READ 0x03
WRITE 0x02
WREN (Flash) 0x06

📌 SPI RAM/FRAM geralmente não precisa de WREN.

⸻

1. Integração com CPU 8-bits

5.1 Interface Lógica

A CPU não acessa SPI diretamente.
Ela conversa com um SPI Memory Controller, que:
 • recebe endereço de 16 bits
 • recebe dado de escrita (8 bits)
 • retorna dado lido (8 bits)
 • gera sinais SPI físicos

CPU → SPI_CTRL → SPI Memory

⸻

5.2 Sinais Internos Recomendados

input  logic        mem_req;
input  logic        mem_we;
input  logic [15:0] mem_addr;
input  logic [7:0]  mem_wdata;
output logic [7:0]  mem_rdata;
output logic        mem_ready;

⸻

1. Máquina de Estados (FSM) do SPI

6.1 Estados Típicos

Estado Função
IDLE Espera requisição
CMD Envia comando
ADDR_H Endereço MSB
ADDR_L Endereço LSB
READ Lê dado
WRITE Escreve dado
DONE Finaliza

⸻

6.2 Diagrama Conceitual

IDLE
 ↓
CMD
 ↓
ADDR_H
 ↓
ADDR_L
 ↓
READ / WRITE
 ↓
DONE → IDLE

⸻

1. Implementação em SystemVerilog

7.1 Interface SPI (pinos)

output logic spi_sck;
output logic spi_mosi;
input  logic spi_miso;
output logic spi_cs_n;

⸻

7.2 Registradores Internos

logic [7:0]  shift_reg;
logic [4:0]  bit_cnt;
logic [2:0]  state;

⸻

7.3 FSM Principal (simplificada)

typedef enum logic [2:0] {
    IDLE,
    CMD,
    ADDR_H,
    ADDR_L,
    READ,
    WRITE,
    DONE
} spi_state_t;

spi_state_t state;

⸻

7.4 Controle de Clock SPI

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        spi_sck <= 1'b0;
    else if (state != IDLE)
        spi_sck <= ~spi_sck;
    else
        spi_sck <= 1'b0;
end

⸻

7.5 Shift Register SPI

always_ff @(posedge clk) begin
    if (spi_sck) begin
        shift_reg <= {shift_reg[6:0], spi_miso};
        bit_cnt   <= bit_cnt - 1;
    end
end

⸻

7.6 Sequência de Leitura (exemplo)

case (state)
CMD: begin
    shift_reg <= 8'h03; // READ
    bit_cnt   <= 7;
end

ADDR_H: shift_reg <= mem_addr[15:8];
ADDR_L: shift_reg <= mem_addr[7:0];

READ: if (bit_cnt == 0) begin
    mem_rdata <= shift_reg;
    state     <= DONE;
end
endcase

⸻

1. Leitura Sequencial (Burst)

Após enviar endereço:
 • manter CS ativo
 • continuar clockando
 • memória retorna bytes consecutivos

Perfeito para:
 • fetch de instruções
 • leitura de vetores
 • loops rápidos

⸻

1. Considerações de Desempenho

Item SPI
Ciclos por byte ~8 clocks SPI
Overhead Baixo
Determinismo Alto
Ideal para CPU ✅

📌 Normalmente:
 • CPU roda mais lento que SPI
 • 1 instrução = vários clocks internos
 • SPI não é gargalo

⸻

1. Boas Práticas de Projeto

✔ Usar FSM clara
✔ Separar CPU e controlador SPI
✔ Usar burst sempre que possível
✔ CS ativo durante múltiplos bytes
✔ Evitar I²C para memória principal

⸻

1. Conclusão

O SPI é a melhor escolha para implementar memória externa em um processador 8-bits com poucos pinos.

⸻

1. Implementation in Neander-X

The Neander-X CPU implements an SPI memory controller based on these concepts:

### Actual Implementation

| File | Description |
|------|-------------|
| `src/spi_memory_controller.sv` | SPI Memory Controller RTL |
| `src/spi_sram_model.sv` | Behavioral SPI SRAM model for simulation |
| `src/project.sv` | TinyTapeout top-level with SPI pin mapping |

### Key Features

- **8-bit CPU addressing** (256 byte address space)
- **16-bit SPI addressing** (high byte always 0x00)
- **Request/Ready handshaking** for CPU stall during memory access
- **~70 cycle latency** per memory access
- **SPI Mode 0** (CPOL=0, CPHA=0)

### Pin Mapping (TinyTapeout)

| Signal | Pin | Direction |
|--------|-----|-----------|
| SPI_CS_N | uo_out[0] | Output |
| SPI_SCLK | uo_out[1] | Output |
| SPI_MOSI | uo_out[2] | Output |
| SPI_MISO | ui_in[0] | Input |

📌 For detailed implementation information, see [SPI Memory Controller Documentation](SPI_MEM_CONTROLLER.md).
