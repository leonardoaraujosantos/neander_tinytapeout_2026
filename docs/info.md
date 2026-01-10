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
- Condition flags: N (negative) and Z (zero)
- 5-bit external memory address bus (32 bytes addressable)

**Instruction Set:**
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

**Pin Interface:**
- `uo_out[4:0]`: 5-bit RAM address
- `uo_out[5]`: RAM write enable (WE)
- `uo_out[6]`: RAM output enable (OE)
- `uo_out[7]`: I/O write strobe
- `uio[7:0]`: Bidirectional 8-bit RAM data bus
- `ui_in[7:0]`: Input port (directly from switches/keyboard)

## How to test

1. **Connect external 32-byte RAM:**
   - Wire `uo_out[4:0]` to the RAM address inputs
   - Wire `uo_out[5]` to RAM write enable
   - Wire `uo_out[6]` to RAM output enable
   - Connect `uio[7:0]` bidirectionally to RAM data bus

2. **Pre-load a program** into the external RAM starting at address 0x00. Example program to add two numbers:
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

3. **Apply reset** (rst_n low, then high) to start execution from address 0x00.

4. **Monitor outputs:**
   - Watch `uo_out[7]` for I/O write strobe to capture output data
   - The result appears on `uio[7:0]` when `uo_out[7]` pulses high

5. **Provide input** via `ui_in[7:0]` when using the IN instruction.

## External hardware

- **32-byte SRAM or equivalent** (e.g., 74HC574 latches + SRAM, or FPGA block RAM on a PMOD)
- **8-bit output latch** (optional, to capture I/O output when `uo_out[7]` strobes)
- **8-bit DIP switches or buttons** for input via `ui_in[7:0]`
- **LED display or 7-segment** to visualize output data
