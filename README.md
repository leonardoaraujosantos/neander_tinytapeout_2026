![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# NEANDER-X Processor

A SystemVerilog implementation of the NEANDER-X educational 8-bit processor for the [TinyTapeout](https://tinytapeout.com) VLSI course.

## Overview

NEANDER is a minimal accumulator-based processor developed at [UFRGS](https://www.inf.ufrgs.br/arq/wiki/doku.php?id=neander) (Universidade Federal do Rio Grande do Sul) for teaching fundamental computer architecture concepts in Brazil. This implementation extends the original NEANDER with I/O instructions, immediate addressing, and stack operations.

### Key Features

- 8-bit data width and address space
- Single accumulator architecture
- Two condition flags (N: Negative, Z: Zero)
- 16 instructions including stack operations (PUSH/POP/CALL/RET)
- FSM-based control unit
- External memory interface (32 bytes addressable via TinyTapeout pins)

## Instruction Set

### Core Instructions (Original NEANDER)

| Opcode | Mnemonic | Operation | Flags |
|--------|----------|-----------|-------|
| 0x0 | NOP | No operation | - |
| 0x1 | STA addr | MEM[addr] <- AC | - |
| 0x2 | LDA addr | AC <- MEM[addr] | N, Z |
| 0x3 | ADD addr | AC <- AC + MEM[addr] | N, Z |
| 0x4 | OR addr | AC <- AC \| MEM[addr] | N, Z |
| 0x5 | AND addr | AC <- AC & MEM[addr] | N, Z |
| 0x6 | NOT | AC <- ~AC | N, Z |
| 0x8 | JMP addr | PC <- addr | - |
| 0x9 | JN addr | if N: PC <- addr | - |
| 0xA | JZ addr | if Z: PC <- addr | - |
| 0xF | HLT | Halt execution | - |

### Extended Instructions (NEANDER-X)

| Opcode | Mnemonic | Operation | Flags |
|--------|----------|-----------|-------|
| 0xB | JNZ addr | if !Z: PC <- addr | - |
| 0xC | IN port | AC <- IO_IN[port] | N, Z |
| 0xD | OUT port | IO_OUT <- AC | - |
| 0xE | LDI imm | AC <- imm | N, Z |

### Stack Extension

| Opcode | Mnemonic | Operation | Flags |
|--------|----------|-----------|-------|
| 0x70 | PUSH | SP--; MEM[SP] <- AC | - |
| 0x71 | POP | AC <- MEM[SP]; SP++ | N, Z |
| 0x72 | CALL addr | SP--; MEM[SP] <- PC; PC <- addr | - |
| 0x73 | RET | PC <- MEM[SP]; SP++ | - |

## Architecture

```mermaid
graph TB
    subgraph Control Unit
        FSM[FSM<br/>Control]
    end

    subgraph Datapath
        PC[PC<br/>Program Counter]
        SP[SP<br/>Stack Pointer]
        RI[RI<br/>Instruction Reg]
        REM[REM<br/>Address Reg]
        RDM[RDM<br/>Data Reg]
        AC[AC<br/>Accumulator]
        ALU[ALU<br/>+, &, |, ~]
        NZ[N Z<br/>Flags]
        MUX[Address MUX<br/>PC/RDM/SP]
    end

    subgraph External
        RAM[(External<br/>RAM)]
        IO_IN[IO Input]
        IO_OUT[IO Output]
    end

    FSM -->|control signals| Datapath
    RI -->|opcode| FSM
    NZ -->|flags| FSM

    PC --> MUX
    RDM --> MUX
    SP --> MUX
    MUX --> REM
    REM -->|address| RAM
    RAM -->|data| RDM
    RDM --> RI
    AC --> ALU
    RAM -->|data| ALU
    ALU --> AC
    AC --> NZ
    AC -->|data| RAM
    IO_IN --> AC
    AC --> IO_OUT
```

### Module Hierarchy

```
tt_um_neander (TinyTapeout wrapper)
└── cpu_top
    ├── neander_datapath
    │   ├── pc_reg (Program Counter)
    │   ├── sp_reg (Stack Pointer)
    │   ├── mux_addr (3-way Address MUX)
    │   ├── generic_reg (REM, RDM, RI, AC)
    │   ├── neander_alu
    │   └── nz_reg (Flags)
    └── neander_control (FSM)
```

## Pin Connections

The NEANDER-X processor requires an external 32-byte RAM and optionally an output latch for I/O.

### Pin Map

#### Dedicated Outputs (`uo_out`)

| Pin | Signal | Description |
|-----|--------|-------------|
| uo_out[4:0] | RAM_ADDR | 5-bit RAM address (0x00-0x1F) |
| uo_out[5] | RAM_WE | RAM Write Enable (active high) |
| uo_out[6] | RAM_OE | RAM Output Enable / Read strobe (active high) |
| uo_out[7] | IO_WRITE | I/O Write strobe (directly from pulse for output latch) |

#### Bidirectional I/O (`uio`)

| Pin | Signal | Description |
|-----|--------|-------------|
| uio[7:0] | RAM_DATA | 8-bit bidirectional RAM data bus |

- **Direction**: Output when `RAM_WE=1` (writing), Input when `RAM_WE=0` (reading)
- `uio_oe` = 0xFF during writes, 0x00 during reads

#### Dedicated Inputs (`ui_in`)

| Pin | Signal | Description |
|-----|--------|-------------|
| ui_in[7:0] | IO_IN | 8-bit input port (directly from for IN instruction) |

### Connection Diagram

```
                    TinyTapeout Chip
                   ┌─────────────────┐
                   │   NEANDER-X     │
    Switches/      │                 │
    Keyboard ────► │ ui_in[7:0]      │
    (8-bit)        │                 │
                   │                 │         ┌──────────────┐
                   │ uo_out[4:0] ───────────► │ ADDR[4:0]    │
                   │                 │         │              │
                   │ uo_out[5] ─────────────► │ WE           │
                   │         (RAM_WE)│         │   External   │
                   │ uo_out[6] ─────────────► │ OE           │
                   │         (RAM_OE)│         │   32-byte    │
                   │                 │         │   SRAM       │
                   │ uio[7:0] ◄────────────► │ DATA[7:0]    │
                   │    (bidirectional)       │              │
                   │                 │         └──────────────┘
                   │                 │
                   │ uo_out[7] ─────────────► Output Latch
                   │      (IO_WRITE) │         (optional)
                   │                 │
                   └─────────────────┘
```

### External RAM Timing

**Read Cycle:**
1. CPU asserts address on `uo_out[4:0]`
2. CPU asserts `RAM_OE` (uo_out[6] = 1)
3. RAM places data on `uio[7:0]` (CPU reads via uio_in)
4. CPU captures data on next clock edge

**Write Cycle:**
1. CPU asserts address on `uo_out[4:0]`
2. CPU drives data on `uio[7:0]` (uio_oe = 0xFF)
3. CPU asserts `RAM_WE` (uo_out[5] = 1)
4. RAM captures data on write enable

### I/O Interface

**Input (IN instruction):**
- Connect switches, keyboard, or other input device to `ui_in[7:0]`
- The IN instruction reads this port directly into AC

**Output (OUT instruction):**
- When executing OUT, the CPU pulses `uo_out[7]` (IO_WRITE) high
- Data appears on `uio[7:0]` simultaneously
- Connect an external latch (e.g., 74HC574) to capture output:
  - Latch clock ← `uo_out[7]`
  - Latch data ← `uio[7:0]`
  - Latch output → LEDs or display

### Recommended External Components

| Component | Purpose | Example Parts |
|-----------|---------|---------------|
| 32-byte SRAM | Program and data storage | 6116 (2Kx8), AS6C1008 |
| 8-bit Latch | Capture I/O output | 74HC574, 74HC374 |
| DIP Switches | Input port | 8-position DIP |
| LED Array | Display output | 8x LEDs with resistors |

## Documentation

- [Project Info (TinyTapeout)](docs/info.md) - Pin interface and testing guide
- [NEANDER CPU Guide](docs/NEANDER_cpu.md) - Complete architecture and implementation details
- [Stack Extension](docs/NEANDER_X_STACK.md) - PUSH/POP/CALL/RET documentation
- [GitHub Actions](docs/GITHUB_ACTIONS.md) - CI/CD workflow explanation

## Quick Start

### Running Tests

```bash
# Run TinyTapeout cocotb tests
cd test
make

# Run comprehensive cocotb tests
cd cocotb_tests
make
```

### Example Program

A simple program that calculates 5 + 3 and outputs the result:

```
Address  Code   Instruction
0x00     0xE0   LDI 5         ; AC = 5
0x01     0x05
0x02     0x30   ADD [0x10]    ; AC = AC + MEM[0x10]
0x03     0x10
0x04     0xD0   OUT 0         ; Output AC
0x05     0x00
0x06     0xF0   HLT           ; Halt
...
0x10     0x03   ; Data: 3
```

Result: Output = 8

## What is Tiny Tapeout?

Tiny Tapeout is an educational project that aims to make it easier and cheaper than ever to get your digital and analog designs manufactured on a real chip.

To learn more and get started, visit <https://tinytapeout.com>.

## Set up your Verilog project

1. Add your Verilog files to the `src` folder.
2. Edit the [info.yaml](info.yaml) and update information about your project, paying special attention to the `source_files` and `top_module` properties. If you are upgrading an existing Tiny Tapeout project, check out our [online info.yaml migration tool](https://tinytapeout.github.io/tt-yaml-upgrade-tool/).
3. Edit [docs/info.md](docs/info.md) and add a description of your project.
4. Adapt the testbench to your design. See [test/README.md](test/README.md) for more information.

The GitHub action will automatically build the ASIC files using [LibreLane](https://www.zerotoasiccourse.com/terminology/librelane/).

## Enable GitHub actions to build the results page

- [Enabling GitHub Pages](https://tinytapeout.com/faq/#my-github-action-is-failing-on-the-pages-part)

## Resources

- [FAQ](https://tinytapeout.com/faq/)
- [Digital design lessons](https://tinytapeout.com/digital_design/)
- [Learn how semiconductors work](https://tinytapeout.com/siliwiz/)
- [Join the community](https://tinytapeout.com/discord)
- [Build your design locally](https://www.tinytapeout.com/guides/local-hardening/)

## What next?

- [Submit your design to the next shuttle](https://app.tinytapeout.com/).
- Edit [this README](README.md) and explain your design, how it works, and how to test it.
- Share your project on your social network of choice:
  - LinkedIn [#tinytapeout](https://www.linkedin.com/search/results/content/?keywords=%23tinytapeout) [@TinyTapeout](https://www.linkedin.com/company/100708654/)
  - Mastodon [#tinytapeout](https://chaos.social/tags/tinytapeout) [@matthewvenn](https://chaos.social/@matthewvenn)
  - X (formerly Twitter) [#tinytapeout](https://twitter.com/hashtag/tinytapeout) [@tinytapeout](https://twitter.com/tinytapeout)
  - Bluesky [@tinytapeout.com](https://bsky.app/profile/tinytapeout.com)
