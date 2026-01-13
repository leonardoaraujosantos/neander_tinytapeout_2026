# SPI SRAM Memory Protocol

This document describes the SPI SRAM protocol used by the Neander-X CPU for external memory access. The protocol is compatible with standard serial SRAM chips from Microchip and other manufacturers.

## Overview

SPI SRAM (Serial Peripheral Interface Static RAM) provides a simple 4-wire interface to access memory, making it ideal for pin-constrained designs like TinyTapeout.

**NEANDER-X uses 16-bit addressing**, enabling the full **64KB address space** of SPI SRAM chips like the 23LC512.

```
┌─────────────┐                    ┌─────────────┐
│             │      CS_N ────────►│             │
│  Neander-X  │      SCLK ────────►│  SPI SRAM   │
│     CPU     │      MOSI ────────►│   (64KB)    │
│  (16-bit    │      MISO ◄────────│             │
│  addressing)│                    │             │
└─────────────┘                    └─────────────┘
     Master                            Slave
```

## Signal Description

| Signal | Direction | Description |
|--------|-----------|-------------|
| CS_N | Master → Slave | Chip Select (active LOW). Must be LOW during entire transaction |
| SCLK | Master → Slave | Serial Clock. Data is shifted on clock edges |
| MOSI | Master → Slave | Master Out Slave In. Command, address, and write data |
| MISO | Slave → Master | Master In Slave Out. Read data from memory |

## SPI Mode

The Neander-X SPI controller uses **SPI Mode 0**:

| Parameter | Value | Description |
|-----------|-------|-------------|
| CPOL | 0 | Clock idle state is LOW |
| CPHA | 0 | Data sampled on rising edge, shifted on falling edge |

```
        CPOL=0, CPHA=0 (Mode 0)

SCLK:   ____╱╲__╱╲__╱╲__╱╲__╱╲__╱╲__╱╲__╱╲____
             ↑   ↑   ↑   ↑   ↑   ↑   ↑   ↑
             │   │   │   │   │   │   │   │
           Sample points (rising edge)

MOSI:   ────<b7><b6><b5><b4><b3><b2><b1><b0>────
             MSB                      LSB
```

## Command Set

### Supported Commands

| Command | Opcode | Description |
|---------|--------|-------------|
| READ | 0x03 | Read data from memory |
| WRITE | 0x02 | Write data to memory |

### Commands Not Used (but available on most chips)

| Command | Opcode | Description |
|---------|--------|-------------|
| RDSR | 0x05 | Read Status Register |
| WRSR | 0x01 | Write Status Register |
| RDMR | 0x05 | Read Mode Register (some chips) |
| WRMR | 0x01 | Write Mode Register (some chips) |

## Protocol Sequences

### READ Sequence (Opcode 0x03)

Reads a single byte from memory at the specified 16-bit address.

```
Byte:      1        2          3         4
        ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐
MOSI:   │ 0x03 │ │ADDR_H│ │ADDR_L│ │ xxxx │
        └──────┘ └──────┘ └──────┘ └──────┘
        ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐
MISO:   │ xxxx │ │ xxxx │ │ xxxx │ │ DATA │
        └──────┘ └──────┘ └──────┘ └──────┘
         Command  Address   Address   Data
                   High      Low      Out
```

**Timing Diagram:**
```
CS_N:  ▔▔▔▔▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▔▔▔▔
            │                                                          │
SCLK:  ▔▔▔▔▔╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱▔▔▔▔▔
            │←── 8 bits ──→│←── 8 bits ──→│←── 8 bits ──→│←── 8 bits ──→│
            │              │              │              │              │
MOSI:  ─────<   0x03      ><   0x00      ><   addr      ><   xxxx      >─────
            │  (READ cmd)  │ (addr high)  │  (addr low)  │  (don't care)│
            │              │              │              │              │
MISO:  ─────<   xxxx      ><   xxxx      ><   xxxx      ><   DATA      >─────
            │  (ignored)   │  (ignored)   │  (ignored)   │  (read data) │
```

**Example:** Read from address 0x0080
```
MOSI: 0x03, 0x00, 0x80, 0xFF (0xFF = dummy byte)
MISO: 0xFF, 0xFF, 0xFF, 0x42 (0x42 = data at address 0x0080)
```

### WRITE Sequence (Opcode 0x02)

Writes a single byte to memory at the specified 16-bit address.

```
Byte:      1        2          3         4
        ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐
MOSI:   │ 0x02 │ │ADDR_H│ │ADDR_L│ │ DATA │
        └──────┘ └──────┘ └──────┘ └──────┘
        ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐
MISO:   │ xxxx │ │ xxxx │ │ xxxx │ │ xxxx │
        └──────┘ └──────┘ └──────┘ └──────┘
         Command  Address   Address   Data
                   High      Low       In
```

**Timing Diagram:**
```
CS_N:  ▔▔▔▔▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▔▔▔▔
            │                                                          │
SCLK:  ▔▔▔▔▔╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱▔▔▔▔▔
            │←── 8 bits ──→│←── 8 bits ──→│←── 8 bits ──→│←── 8 bits ──→│
            │              │              │              │              │
MOSI:  ─────<   0x02      ><   0x00      ><   addr      ><   DATA      >─────
            │ (WRITE cmd)  │ (addr high)  │  (addr low)  │ (write data) │
            │              │              │              │              │
MISO:  ─────<   xxxx      ><   xxxx      ><   xxxx      ><   xxxx      >─────
            │  (ignored)   │  (ignored)   │  (ignored)   │  (ignored)   │
```

**Example:** Write 0x42 to address 0x0080
```
MOSI: 0x02, 0x00, 0x80, 0x42
MISO: (ignored)
```

## Burst Mode (Sequential Read/Write)

Most SPI SRAMs support burst mode where multiple bytes can be read/written sequentially without re-sending the command and address. The address auto-increments after each byte.

```
Sequential READ:
CS_N:  ▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁
MOSI:  [0x03][ADDR_H][ADDR_L][xxxx][xxxx][xxxx][xxxx]...
MISO:  [xxxx][ xxxx ][ xxxx ][D0  ][D1  ][D2  ][D3  ]...
                              addr  addr  addr  addr
                              +0    +1    +2    +3
```

**Note:** The Neander-X controller currently uses single-byte transfers only (no burst mode).

## Timing Parameters

Typical timing for SPI SRAM at 3.3V:

| Parameter | Symbol | Min | Max | Unit |
|-----------|--------|-----|-----|------|
| Clock Frequency | fSCK | - | 20 | MHz |
| CS_N Setup Time | tCSS | 25 | - | ns |
| CS_N Hold Time | tCSH | 50 | - | ns |
| Clock High Time | tCH | 18 | - | ns |
| Clock Low Time | tCL | 18 | - | ns |
| MOSI Setup Time | tSU | 5 | - | ns |
| MOSI Hold Time | tHD | 5 | - | ns |
| MISO Output Delay | tV | - | 25 | ns |

## Compatible SPI SRAM Chips

### Microchip (formerly Microchip/SST)

| Part Number | Density | Voltage | Max Clock | Package |
|-------------|---------|---------|-----------|---------|
| **23LC512** | 64KB | 2.5-5.5V | 20MHz | 8-SOIC, 8-DIP |
| **23LC1024** | 128KB | 2.5-5.5V | 20MHz | 8-SOIC, 8-DIP |
| **23LCV512** | 64KB | 2.5-5.5V | 20MHz | 8-SOIC (battery backup) |
| **23LCV1024** | 128KB | 2.5-5.5V | 20MHz | 8-SOIC (battery backup) |
| **23K256** | 32KB | 2.5-5.5V | 20MHz | 8-SOIC, 8-DIP |
| **23A512** | 64KB | 2.5-5.5V | 20MHz | 8-SOIC |
| **23A1024** | 128KB | 2.5-5.5V | 20MHz | 8-SOIC |

### ISSI

| Part Number | Density | Voltage | Max Clock | Package |
|-------------|---------|---------|-----------|---------|
| **IS62WVS2568** | 32KB | 2.5-3.6V | 45MHz | 8-SOIC |
| **IS62WVS5128** | 64KB | 2.5-3.6V | 45MHz | 8-SOIC |

### Lyontek

| Part Number | Density | Voltage | Max Clock | Package |
|-------------|---------|---------|-----------|---------|
| **LY62W2048** | 32KB | 2.5-5.5V | 33MHz | 8-SOIC |
| **LY62W4096** | 64KB | 2.5-5.5V | 33MHz | 8-SOIC |

### Recommended for Neander-X / TinyTapeout

| Chip | Reason |
|------|--------|
| **23LC512** | Most common, 64KB, wide voltage range, cheap (~$1) |
| **23K256** | Smaller (32KB), even cheaper, good for testing |
| **23LC1024** | If you need 128KB address space |

## Pin Configuration (8-pin SOIC/DIP)

```
        ┌───────────┐
   CS ──┤1        8├── VCC (3.3V)
   SO ──┤2   SPI  7├── HOLD (tie to VCC)
   NC ──┤3  SRAM  6├── SCK
  VSS ──┤4        5├── SI
        └───────────┘

Pin mapping:
  CS   = Chip Select (directly from SPI_CS_N)
  SO   = Serial Out  (directly to SPI_MISO)
  SCK  = Serial Clock (directly from SPI_SCLK)
  SI   = Serial In   (directly from SPI_MOSI)
  HOLD = Hold pin (tie to VCC if not used)
  NC   = No Connect (or second chip select on some parts)
```

## Connecting to TinyTapeout

### Pin Mapping

| TinyTapeout Pin | Signal | SPI SRAM Pin |
|-----------------|--------|--------------|
| uo_out[0] | SPI_CS_N | Pin 1 (CS) |
| uo_out[1] | SPI_SCLK | Pin 6 (SCK) |
| uo_out[2] | SPI_MOSI | Pin 5 (SI) |
| ui_in[0] | SPI_MISO | Pin 2 (SO) |

### Example Schematic

```
TinyTapeout ASIC                          23LC512 SPI SRAM
┌─────────────────┐                       ┌─────────────────┐
│                 │                       │                 │
│     uo_out[0] ──┼───────────────────────┼─► CS  (pin 1)   │
│                 │                       │                 │
│     uo_out[1] ──┼───────────────────────┼─► SCK (pin 6)   │
│                 │                       │                 │
│     uo_out[2] ──┼───────────────────────┼─► SI  (pin 5)   │
│                 │                       │                 │
│      ui_in[0] ◄─┼───────────────────────┼── SO  (pin 2)   │
│                 │                       │                 │
└─────────────────┘                       │     VCC (pin 8) ├──── 3.3V
                                          │     VSS (pin 4) ├──── GND
                                          │    HOLD (pin 7) ├──── 3.3V
                                          └─────────────────┘
```

## Performance in Neander-X

| Metric | Value |
|--------|-------|
| SPI Clock | CPU_CLK / 2 |
| Bits per byte | 8 |
| Bytes per access | 4 (cmd + 2 addr bytes + data) |
| Cycles per access | ~70 CPU cycles |
| Effective bandwidth | ~1 byte per 70 cycles |
| Address Space | **64KB (16-bit addressing)** |

At 10MHz CPU clock:
- SPI clock = 5MHz
- Memory access time = 7us per byte
- Instruction fetch = ~7us (opcode only)
- Memory operation (16-bit addr) = ~21us (3 fetches: opcode + addr_lo + addr_hi)

**16-bit Address Format:**
The CPU sends addresses in big-endian order over SPI (high byte first), matching the SPI SRAM protocol:
- ADDR_H (bits 15:8) sent first
- ADDR_L (bits 7:0) sent second

## Related Documentation

- [SPI Memory Controller](SPI_MEM_CONTROLLER.md) - Controller implementation details
- [SPI Protocol](SPI_protocol.md) - General SPI protocol concepts
- [Neander CPU](NEANDER_cpu.md) - CPU architecture and instruction set
