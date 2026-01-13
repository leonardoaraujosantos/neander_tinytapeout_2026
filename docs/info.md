<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

This project implements the Neander-X CPU, an 8-bit educational processor compatible with UFRGS's Neander-X architecture. It features an accumulator-based design with **16-bit addressing for a full 64KB address space**.

**Core Architecture:**
- 8-bit Accumulator (AC) for arithmetic/logic operations
- **16-bit Program Counter (PC)** - addresses full 64KB range
- **16-bit Stack Pointer (SP)** - initialized to 0x00FF, grows downward
- **16-bit Frame Pointer (FP)** - for local variable access in functions
- 8-bit X and Y index registers for array/pointer operations
- Condition flags: N (negative), Z (zero), C (carry)
- **SPI memory interface (64KB address space via external SPI SRAM)**

**Key Features:**
- 60+ instructions including stack ops, indexed addressing, hardware multiply/divide
- **Full 64KB memory addressing** via 16-bit registers (PC, SP, FP, REM, RDM)
- SPI memory interface uses only 4 pins (vs 15 for parallel RAM)
- ~70 CPU cycles per memory access via SPI
- Compatible with standard SPI SRAMs (23LC512 for 64KB, 23LC1024 for 128KB)

**Basic Instruction Set:**
| Opcode | Mnemonic | Description |
|--------|----------|-------------|
| 0x1 | STA addr | Store AC to memory |
| 0x2 | LDA addr | Load AC from memory |
| 0x3 | ADD addr | AC = AC + mem[addr] |
| 0x4 | OR addr | AC = AC \| mem[addr] |
| 0x5 | AND addr | AC = AC & mem[addr] |
| 0x6 | NOT | AC = ~AC |
| 0x8 | JMP addr | Unconditional jump |
| 0x9 | JN addr | Jump if N flag set |
| 0xA | JZ addr | Jump if Z flag set |
| 0xB | JNZ addr | Jump if Z flag clear |
| 0xC | IN port | Load AC from input port |
| 0xD | OUT port | Output AC to port |
| 0xE | LDI imm | Load immediate value |
| 0xF | HLT | Halt execution |

See the main README for the complete instruction set including stack operations, indexed addressing, and hardware multiply/divide.

**Pin Interface (SPI Memory):**
- `uo_out[0]`: SPI_CS_N (chip select, active low)
- `uo_out[1]`: SPI_SCLK (serial clock)
- `uo_out[2]`: SPI_MOSI (data to SRAM)
- `uo_out[3:6]`: Debug AC bits [3:0]
- `uo_out[7]`: I/O write strobe
- `uio[7:0]`: Debug output (Program Counter low byte - PC[7:0])
- `ui_in[0]`: SPI_MISO (data from SRAM)
- `ui_in[7:1]`: Input port (7-bit, directly from switches/keyboard)

**16-bit Instruction Encoding:**
Memory-addressing instructions use 3 bytes: `[opcode] [addr_lo] [addr_hi]` (little-endian).

## How to test

1. **Connect external SPI SRAM (e.g., 23LC512):**
   - `uo_out[0]` → CS (pin 1)
   - `uo_out[1]` → SCK (pin 6)
   - `uo_out[2]` → SI (pin 5)
   - `ui_in[0]` ← SO (pin 2)
   - VCC (pin 8) → 3.3V
   - VSS (pin 4) → GND
   - HOLD (pin 7) → 3.3V

2. **Pre-load a program** into the SPI SRAM starting at address 0x0000. Example program to add two numbers (16-bit addressing):
   ```
   0x0000: 0xE0  ; LDI 5 (load immediate)
   0x0001: 0x05  ; value = 5
   0x0002: 0x30  ; ADD addr (3-byte instruction)
   0x0003: 0x10  ; addr_lo = 0x10
   0x0004: 0x00  ; addr_hi = 0x00 (address = 0x0010)
   0x0005: 0xD0  ; OUT port
   0x0006: 0x00  ; port 0
   0x0007: 0xF0  ; HLT
   ...
   0x0010: 0x03  ; second operand = 3
   ```
   Result: AC = 5 + 3 = 8, output to I/O port

   **Note:** Memory operations now use 3 bytes: `[opcode] [addr_lo] [addr_hi]`

3. **Apply reset** (rst_n low, then high) to start execution from address 0x00.

4. **Monitor outputs:**
   - Watch `uo_out[7]` for I/O write strobe
   - AC bits [3:0] visible on `uo_out[6:3]`
   - Program Counter visible on `uio[7:0]`

5. **Provide input** via `ui_in[7:1]` when using the IN instruction (note: bit 0 is SPI_MISO).

## External hardware

**Required:**
- **SPI SRAM** (23LC512 or 23LC1024) - **Full 64KB address space**
  - 8-pin SOIC or DIP package
  - 2.5-5.5V operation
  - SPI Mode 0 compatible
  - Recommended: **23LC512** (64KB, ~$1)

**Optional:**
- **8-bit output latch** (74HC574) to capture I/O output when `uo_out[7]` strobes
- **7-bit DIP switches** for input via `ui_in[7:1]`
- **LED display** connected to latch output for visualization
- **Logic analyzer** to monitor SPI bus traffic

**Pin Savings vs Parallel RAM:**
| Interface | Memory Pins | Address Space |
|-----------|-------------|---------------|
| Parallel | 15 pins | 32 bytes |
| **SPI** | **4 pins** | **64KB** |

The SPI interface provides **2048x more memory** (64KB vs 32 bytes) while saving 11 pins!

For detailed SPI protocol information, see [SPI_SRAM_MEMORY.md](SPI_SRAM_MEMORY.md).
