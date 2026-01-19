# Neander-X TinyTapeout System Ports

## TinyTapeout Port Constraints

TinyTapeout provides a fixed set of I/O pins that cannot be changed. The design must work within these constraints:

| Port | Width | Direction | Description |
|------|-------|-----------|-------------|
| `ui_in` | 8 bits | Input | Dedicated inputs |
| `uo_out` | 8 bits | Output | Dedicated outputs |
| `uio_in` | 8 bits | Input | Bidirectional I/O input path |
| `uio_out` | 8 bits | Output | Bidirectional I/O output path |
| `uio_oe` | 8 bits | Output | Bidirectional I/O enable (1=output, 0=input) |
| `ena` | 1 bit | Input | Always 1 when design is powered |
| `clk` | 1 bit | Input | System clock |
| `rst_n` | 1 bit | Input | Active-low reset |

**Total available I/O: 24 bits** (8 dedicated in + 8 dedicated out + 8 bidirectional)

---

## Current Pin Allocation

### Dedicated Inputs (`ui_in[7:0]`)

| Bit | Signal | Usage | Required |
|-----|--------|-------|----------|
| 0 | `SPI_MISO` | SPI Memory data input | Yes |
| 1-7 | `io_in[7:1]` | General purpose CPU input | Optional |

### Dedicated Outputs (`uo_out[7:0]`)

| Bit | Signal | Usage | Required |
|-----|--------|-------|----------|
| 0 | `SPI_CS_N` | SPI Memory chip select | Yes |
| 1 | `SPI_SCLK` | SPI Memory clock | Yes |
| 2 | `SPI_MOSI` | SPI Memory data output | Yes |
| 3 | `dbg_ac[0]` | Debug: AC bit 0 | Debug only |
| 4 | `dbg_ac[1]` | Debug: AC bit 1 | Debug only |
| 5 | `dbg_ac[2]` | Debug: AC bit 2 | Debug only |
| 6 | `dbg_ac[3]` | Debug: AC bit 3 | Debug only |
| 7 | `io_write` | I/O write strobe | Functional |

### Bidirectional I/O (`uio[7:0]`)

Currently configured as **all outputs** (`uio_oe = 8'hFF`):

| Bit | Signal | Usage | Required |
|-----|--------|-------|----------|
| 0-7 | `dbg_pc[7:0]` | Debug: Program Counter low byte | Debug only |

---

## Pin Classification Summary

| Category | Count | Pins |
|----------|-------|------|
| **Essential (SPI Memory)** | 4 | ui_in[0], uo_out[0:2] |
| **Functional I/O** | 8 | ui_in[7:1], uo_out[7] |
| **Debug (repurposable)** | 12 | uo_out[3:6], uio[7:0] |
| **Unused** | 1 | ena |
| **Total** | 25 | - |

---

## Future Architecture: Dual SPI Controller

### Overview

The plan is to support two SPI interfaces:

1. **SPI Memory Controller** - Dedicated to external SRAM for program/data memory
2. **General Purpose SPI Controller** - For peripherals (directly under CPU control)

### Potential Peripheral Support

The general purpose SPI controller could interface with:

| Peripheral Type | Example Chips | Use Case |
|-----------------|---------------|----------|
| GPIO Expander | MCP23S17 | Additional I/O pins |
| LED Driver | TLC5940, MAX7219 | Display output |
| NOR Flash | W25Q32 | Non-volatile storage |
| DAC | MCP4921 | Analog output |
| ADC | MCP3008 | Analog input |
| RTC | DS3234 | Real-time clock |
| SD Card | - | Mass storage |

### Proposed Pin Allocation

To support dual SPI, we need to repurpose debug pins:

#### Option A: Shared SCLK/MOSI (5 pins total)

```
SPI Memory:
  ui_in[0]  = MEM_MISO    (existing)
  uo_out[0] = MEM_CS_N    (existing)
  uo_out[1] = SPI_SCLK    (shared)
  uo_out[2] = SPI_MOSI    (shared)

General Purpose SPI:
  uio[0]    = PERIPH_CS_N (directly controlled)
  uio[1]    = PERIPH_MISO (directly read)
```

#### Option B: Fully Independent (8 pins total)

```
SPI Memory:
  ui_in[0]  = MEM_MISO
  uo_out[0] = MEM_CS_N
  uo_out[1] = MEM_SCLK
  uo_out[2] = MEM_MOSI

General Purpose SPI:
  uo_out[3] = PERIPH_CS_N
  uo_out[4] = PERIPH_SCLK
  uo_out[5] = PERIPH_MOSI
  uio[0]    = PERIPH_MISO (input)
```

#### Option C: Multiple Chip Selects (scalable)

```
SPI Memory:
  ui_in[0]  = MEM_MISO
  uo_out[0] = MEM_CS_N
  uo_out[1] = SPI_SCLK    (shared)
  uo_out[2] = SPI_MOSI    (shared)

Peripheral SPI (directly addressable by CPU):
  uo_out[3] = PERIPH_CS0_N
  uo_out[4] = PERIPH_CS1_N
  uo_out[5] = PERIPH_CS2_N
  uo_out[6] = PERIPH_CS3_N
  uio[0]    = PERIPH_MISO (directly readable)
```

---

## Implementation Considerations

### Memory-Mapped I/O Approach

The general purpose SPI could be controlled via memory-mapped registers:

| Address | Register | Description |
|---------|----------|-------------|
| `0xFFF0` | SPI_DATA | Read/Write data register |
| `0xFFF1` | SPI_CTRL | Control (CS select, speed) |
| `0xFFF2` | SPI_STATUS | Status (busy, rx ready) |

### Bit-Banged Approach

Alternatively, expose SPI pins directly to CPU I/O:

```c
// Example: Bit-banged SPI write
void spi_write(uint8_t data) {
    for (int i = 7; i >= 0; i--) {
        IO_OUT = (data >> i) & 1;  // Set MOSI
        IO_OUT |= SCLK;            // Clock high
        IO_OUT &= ~SCLK;           // Clock low
    }
}
```

### Trade-offs

| Approach | Pros | Cons |
|----------|------|------|
| Hardware SPI | Fast, CPU-free transfers | Uses more gates |
| Bit-banged | Flexible, minimal hardware | Slow, CPU-intensive |
| Memory-mapped | Clean software interface | Address decoding overhead |

---

## Recommended Configuration

For maximum flexibility with minimal pin usage:

```
Dedicated Inputs (ui_in):
  [0]   SPI_MISO (memory)
  [7:1] General input / directly readable

Dedicated Outputs (uo_out):
  [0]   MEM_CS_N
  [1]   SPI_SCLK (directly controllable for peripherals)
  [2]   SPI_MOSI (directly controllable for peripherals)
  [3]   PERIPH_CS0_N (directly controllable)
  [4]   PERIPH_CS1_N (directly controllable)
  [5]   PERIPH_CS2_N (directly controllable)
  [6]   PERIPH_CS3_N (directly controllable)
  [7]   IO_WRITE strobe

Bidirectional (uio):
  [0]   PERIPH_MISO (input, directly readable)
  [7:1] Debug or additional CS lines
```

This provides:
- 1 dedicated SPI for memory (hardware controller)
- 4 directly addressable chip select lines for peripherals
- Shared SCLK/MOSI under direct CPU control
- Peripheral MISO directly readable by CPU
