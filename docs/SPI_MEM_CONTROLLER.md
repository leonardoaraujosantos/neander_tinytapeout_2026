# SPI Memory Controller for Neander-X CPU

This document describes the SPI Memory Controller implementation used to interface the Neander-X CPU with external SPI SRAM memory.

## Overview

The Neander-X CPU uses a parallel memory interface internally, but due to pin constraints on TinyTapeout, external memory is accessed through a serial SPI interface. The SPI Memory Controller bridges these two interfaces.

**With 16-bit addressing**, the controller supports the full **64KB address space** of SPI SRAM chips.

```
┌─────────────────┐     Parallel Interface      ┌─────────────────┐     SPI Interface
│                 │  mem_addr[15:0] (16-bit)    │                 │     MOSI, MISO
│   Neander CPU   │  mem_wdata[7:0]             │  SPI Memory     │     SCLK, CS_N
│                 │◄─────────────────────────────│  Controller     │◄────────────────► SPI SRAM
│                 │  mem_rdata[7:0]             │   (16-bit)      │       (64KB)
│                 │  mem_req, mem_ready         │                 │
└─────────────────┘                             └─────────────────┘
```

## Files

| File | Description |
|------|-------------|
| `src/spi_memory_controller.sv` | SPI Memory Controller RTL |
| `src/spi_sram_model.sv` | Behavioral SPI SRAM model for simulation |

## CPU Interface

The controller presents a simple request/ready handshaking interface to the CPU:

```systemverilog
// CPU → SPI Controller
input  logic        mem_req,      // Memory access request
input  logic        mem_we,       // 0 = read, 1 = write
input  logic [15:0] mem_addr,     // 16-bit address (64KB space)
input  logic [7:0]  mem_wdata,    // Write data

// SPI Controller → CPU
output logic [7:0]  mem_rdata,    // Read data
output logic        mem_ready     // Access complete (1 cycle pulse)
```

**16-bit Address Support:** The controller accepts full 16-bit addresses from the CPU and transmits them to the SPI SRAM as two bytes (high byte first, then low byte).

### Handshaking Protocol

1. CPU asserts `mem_req` along with address and control signals
2. Controller latches inputs and starts SPI transaction
3. CPU waits (keeps `mem_req` asserted) until `mem_ready` pulses
4. On `mem_ready`:
   - For reads: `mem_rdata` contains valid data
   - For writes: write is complete
5. CPU deasserts `mem_req` and controller returns to IDLE

**Important**: The CPU control unit only asserts `mem_req` when `mem_ready` is NOT high. This prevents double-fetch bugs where a new transaction starts immediately after the previous one completes.

## SPI Interface

The controller generates standard SPI signals:

```systemverilog
output logic        spi_cs_n,     // Chip select (active low)
output logic        spi_sclk,     // Serial clock (clk/2)
output logic        spi_mosi,     // Master Out Slave In
input  logic        spi_miso      // Master In Slave Out
```

### SPI Timing

- **SPI Clock**: Generated at half the CPU clock frequency (clk/2)
- **SPI Mode**: Mode 0 (CPOL=0, CPHA=0)
  - Data sampled on rising edge of SCLK
  - Data shifted on falling edge of SCLK
- **Bit Order**: MSB first

## State Machine

The controller uses an 8-state FSM:

```
┌──────┐
│ IDLE │◄────────────────────────────────┐
└──┬───┘                                 │
   │ mem_req                             │
   ▼                                     │
┌──────────┐                             │
│ADDR_LATCH│ (1 cycle address stabilize) │
└──┬───────┘                             │
   │                                     │
   ▼                                     │
┌────────┐                               │
│SEND_CMD│ (0x03=READ, 0x02=WRITE)       │
└──┬─────┘                               │
   │ 8 bits                              │
   ▼                                     │
┌───────────┐                            │
│SEND_ADDR_H│ (always 0x00 for 8-bit)    │
└──┬────────┘                            │
   │ 8 bits                              │
   ▼                                     │
┌───────────┐                            │
│SEND_ADDR_L│ (actual 8-bit address)     │
└──┬────────┘                            │
   │ 8 bits                              │
   ▼                                     │
┌─────────┐        ┌──────────┐          │
│SEND_DATA│ write  │ RECV_DATA│ read     │
└──┬──────┘        └──┬───────┘          │
   │ 8 bits           │ 8 bits           │
   ▼                  ▼                  │
┌──────┐                                 │
│ DONE │─────────────────────────────────┘
└──────┘
   │ mem_ready pulse
```

### State Descriptions

| State | Description |
|-------|-------------|
| `IDLE` | Wait for `mem_req`. CS_N=1, SCLK=0 |
| `ADDR_LATCH` | Latch 16-bit address/data, assert CS_N=0, load command |
| `SEND_CMD` | Shift out 0x03 (READ) or 0x02 (WRITE) |
| `SEND_ADDR_HI` | Shift out mem_addr[15:8] (high byte) |
| `SEND_ADDR_LO` | Shift out mem_addr[7:0] (low byte) |
| `SEND_DATA` | Shift out write data (write only) |
| `RECV_DATA` | Shift in read data (read only) |
| `DONE` | Assert `mem_ready`, deassert CS_N, return to IDLE |

**16-bit Address Handling:**
- `SEND_ADDR_HI` now sends the actual high byte (`mem_addr[15:8]`) instead of always 0x00
- This enables full 64KB addressing (0x0000 - 0xFFFF)

## SPI Protocol

The controller implements standard SPI SRAM commands compatible with 23LC512, 23K256, and similar devices.

### Read Sequence (0x03) - 16-bit Address

```
CS_N:  ▔▔▔▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▔▔▔
SCLK:     ╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲
MOSI:     [  0x03  ][addr_hi ][ addr_lo][xxxxxxxx]
MISO:     [xxxxxxxx][xxxxxxxx][xxxxxxxx][  data  ]
          |  CMD   | ADDR_HI | ADDR_LO |  DATA   |
                   | [15:8]  |  [7:0]  |
```

**Example:** Read from address 0x8000:
- MOSI: 0x03, 0x80, 0x00, 0xFF
- MISO: xx, xx, xx, [data]

### Write Sequence (0x02) - 16-bit Address

```
CS_N:  ▔▔▔▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▔▔▔
SCLK:     ╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲
MOSI:     [  0x02  ][addr_hi ][ addr_lo][  data  ]
MISO:     [xxxxxxxx][xxxxxxxx][xxxxxxxx][xxxxxxxx]
          |  CMD   | ADDR_HI | ADDR_LO |  DATA   |
                   | [15:8]  |  [7:0]  |
```

**Example:** Write 0x42 to address 0x1234:
- MOSI: 0x02, 0x12, 0x34, 0x42

## Timing Analysis

Each SPI byte transfer takes 16 CPU clock cycles (8 bits × 2 phases):

| Operation | Bytes | CPU Cycles |
|-----------|-------|------------|
| Read | 4 (cmd + addr_hi + addr_lo + data) | 64 + overhead ≈ 70 |
| Write | 4 (cmd + addr_hi + addr_lo + data) | 64 + overhead ≈ 70 |

The CPU stalls approximately 70 cycles per memory access while waiting for `mem_ready`.

## TinyTapeout Pin Mapping

The SPI signals are mapped to TinyTapeout pins in `project.sv`:

| Signal | TinyTapeout Pin | Direction |
|--------|-----------------|-----------|
| `spi_cs_n` | `uo_out[0]` | Output |
| `spi_sclk` | `uo_out[1]` | Output |
| `spi_mosi` | `uo_out[2]` | Output |
| `spi_miso` | `ui_in[0]` | Input |
| `io_write` | `uo_out[7]` | Output |
| `dbg_pc[7:0]` | `uio_out[7:0]` | Output |

## Simulation Model

For simulation, `spi_sram_model.sv` provides a behavioral SPI SRAM:

```systemverilog
module spi_sram_model (
    input  logic spi_cs_n,
    input  logic spi_sclk,
    input  logic spi_mosi,
    output logic spi_miso
);
```

### Features

- 64KB memory array (uses 8-bit addresses in current config)
- Supports READ (0x03) and WRITE (0x02) commands
- Proper SPI Mode 0 timing
- `load_byte()` task for testbench memory initialization
- `read_byte()` function for testbench verification

### Testbench Usage

```python
# In cocotb test, load memory directly:
dut.spi_ram.memory[0x00].value = 0xE0  # LDI opcode
dut.spi_ram.memory[0x01].value = 0x42  # immediate value
```

## Integration Example

### In project.sv (Top Level)

```systemverilog
// Internal signals (16-bit address)
logic [15:0] cpu_mem_addr;    // 16-bit address from CPU
logic [7:0]  cpu_mem_data_out;
logic [7:0]  cpu_mem_data_in;
logic        cpu_mem_write, cpu_mem_read;
logic        cpu_mem_req, cpu_mem_ready;

// CPU instance
cpu_top cpu (
    .clk(clk),
    .reset(reset),
    .mem_addr(cpu_mem_addr),       // 16-bit address
    .mem_data_out(cpu_mem_data_out),
    .mem_data_in(cpu_mem_data_in),
    .mem_write(cpu_mem_write),
    .mem_read(cpu_mem_read),
    .mem_req(cpu_mem_req),
    .mem_ready(cpu_mem_ready),
    // ... other ports
);

// SPI Memory Controller (16-bit address support)
spi_memory_controller spi_ctrl (
    .clk(clk),
    .reset(reset),
    .mem_req(cpu_mem_req),
    .mem_we(cpu_mem_write),
    .mem_addr(cpu_mem_addr),       // 16-bit address
    .mem_wdata(cpu_mem_data_out),
    .mem_rdata(cpu_mem_data_in),
    .mem_ready(cpu_mem_ready),
    .spi_cs_n(spi_cs_n),
    .spi_sclk(spi_sclk),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso)
);
```

### In Testbench (tb.v)

```verilog
// SPI SRAM Model for simulation
spi_sram_model spi_ram (
    .spi_cs_n (spi_cs_n),
    .spi_sclk (spi_sclk),
    .spi_mosi (spi_mosi),
    .spi_miso (spi_miso)
);
```

## Key Design Decisions

1. **Address Latch State**: Added 1-cycle delay (`ADDR_LATCH`) to ensure the 16-bit address is stable before starting SPI transaction

2. **Request Deassertion**: CPU only asserts `mem_req` when `mem_ready=0` to prevent double-fetch bugs

3. **Full 16-bit Addressing**: The controller now sends actual 16-bit addresses to enable the full 64KB address space
   - Previous: High byte was always 0x00 (limited to 256 bytes)
   - Current: High byte is `mem_addr[15:8]` (full 64KB range)

4. **Single-byte Transfers**: No burst mode implemented; each access is independent

5. **Little-endian CPU, Big-endian SPI**: The CPU stores addresses in little-endian format (addr_lo first), but the SPI protocol sends them in big-endian order (addr_hi first). The controller handles this conversion internally.

## Related Documents

- [SPI Protocol Overview](SPI_protocol.md) - General SPI protocol concepts
- [Neander CPU Architecture](NEANDER_cpu.md) - CPU instruction set and architecture
- [TinyTapeout Integration](info.md) - Pin mapping and integration details
