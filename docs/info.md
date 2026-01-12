<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

This project implements the Neander-X CPU, an 8-bit educational processor compatible with UFRJ's Neander-X architecture. It features an accumulator-based design with the following components:

**Core Architecture:**
- 8-bit Accumulator (AC) for arithmetic/logic operations
- 8-bit Program Counter (PC)
- 8-bit Stack Pointer (SP) and Frame Pointer (FP)
- X and Y index registers for array/pointer operations
- Condition flags: N (negative), Z (zero), C (carry)
- SPI memory interface (256-byte address space via external SPI SRAM)

**Key Features:**
- 60+ instructions including stack ops, indexed addressing, hardware multiply/divide
- SPI memory interface uses only 4 pins (vs 15 for parallel RAM)
- ~70 CPU cycles per memory access via SPI
- Compatible with standard SPI SRAMs (23LC512, 23K256, etc.)

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
- `uio[7:0]`: Debug output (Program Counter)
- `ui_in[0]`: SPI_MISO (data from SRAM)
- `ui_in[7:1]`: Input port (7-bit, directly from switches/keyboard)

## How to test

1. **Connect external SPI SRAM (e.g., 23LC512):**
   - `uo_out[0]` → CS (pin 1)
   - `uo_out[1]` → SCK (pin 6)
   - `uo_out[2]` → SI (pin 5)
   - `ui_in[0]` ← SO (pin 2)
   - VCC (pin 8) → 3.3V
   - VSS (pin 4) → GND
   - HOLD (pin 7) → 3.3V

2. **Pre-load a program** into the SPI SRAM starting at address 0x00. Example program to add two numbers:
   ```
   0x00: 0xE0  ; LDI 5 (load immediate)
   0x01: 0x05  ; value = 5
   0x02: 0x30  ; ADD addr
   0x03: 0x10  ; addr = 0x10 (memory location with second operand)
   0x04: 0xD0  ; OUT port
   0x05: 0x00  ; port 0
   0x06: 0xF0  ; HLT
   ...
   0x10: 0x03  ; second operand = 3
   ```
   Result: AC = 5 + 3 = 8, output to I/O port

3. **Apply reset** (rst_n low, then high) to start execution from address 0x00.

4. **Monitor outputs:**
   - Watch `uo_out[7]` for I/O write strobe
   - AC bits [3:0] visible on `uo_out[6:3]`
   - Program Counter visible on `uio[7:0]`

5. **Provide input** via `ui_in[7:1]` when using the IN instruction (note: bit 0 is SPI_MISO).

## External hardware

**Required:**
- **SPI SRAM** (23LC512, 23K256, or similar) - 256 bytes used, 64KB available
  - 8-pin SOIC or DIP package
  - 2.5-5.5V operation
  - SPI Mode 0 compatible

**Optional:**
- **8-bit output latch** (74HC574) to capture I/O output when `uo_out[7]` strobes
- **7-bit DIP switches** for input via `ui_in[7:1]`
- **LED display** connected to latch output for visualization
- **Logic analyzer** to monitor SPI bus traffic

**Pin Savings vs Parallel RAM:**
| Interface | Memory Pins | Address Space |
|-----------|-------------|---------------|
| Parallel | 15 pins | 32 bytes |
| **SPI** | **4 pins** | **256 bytes** |

For detailed SPI protocol information, see [SPI_SRAM_MEMORY.md](SPI_SRAM_MEMORY.md).
