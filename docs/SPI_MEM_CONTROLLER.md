# SPI Memory Controller for Neander-X CPU

This document describes the SPI Memory Controller implementation used to interface the Neander-X CPU with external SPI SRAM memory.

## Overview

The Neander-X CPU uses a parallel memory interface internally, but due to pin constraints on TinyTapeout, external memory is accessed through a serial SPI interface. The SPI Memory Controller bridges these two interfaces.

```
┌─────────────────┐     Parallel Interface      ┌─────────────────┐     SPI Interface
│                 │  mem_addr[7:0]              │                 │     MOSI, MISO
│   Neander CPU   │  mem_wdata[7:0]             │  SPI Memory     │     SCLK, CS_N
│                 │◄─────────────────────────────│  Controller     │◄────────────────► SPI SRAM
│                 │  mem_rdata[7:0]             │                 │
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
input  logic [7:0]  mem_addr,     // 8-bit address (256 byte space)
input  logic [7:0]  mem_wdata,    // Write data

// SPI Controller → CPU
output logic [7:0]  mem_rdata,    // Read data
output logic        mem_ready     // Access complete (1 cycle pulse)
```

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
| `ADDR_LATCH` | Latch address/data, assert CS_N=0, load command |
| `SEND_CMD` | Shift out 0x03 (READ) or 0x02 (WRITE) |
| `SEND_ADDR_HI` | Shift out 0x00 (high byte, always zero for 8-bit) |
| `SEND_ADDR_LO` | Shift out address[7:0] |
| `SEND_DATA` | Shift out write data (write only) |
| `RECV_DATA` | Shift in read data (read only) |
| `DONE` | Assert `mem_ready`, deassert CS_N, return to IDLE |

## SPI Protocol

The controller implements standard SPI SRAM commands compatible with 23LC512, 23K256, and similar devices.

### Read Sequence (0x03)

```
CS_N:  ▔▔▔▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▔▔▔
SCLK:     ╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲
MOSI:     [  0x03  ][  0x00  ][ addr_lo][xxxxxxxx]
MISO:     [xxxxxxxx][xxxxxxxx][xxxxxxxx][  data  ]
          |  CMD   | ADDR_HI | ADDR_LO |  DATA   |
```

### Write Sequence (0x02)

```
CS_N:  ▔▔▔▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▔▔▔
SCLK:     ╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲
MOSI:     [  0x02  ][  0x00  ][ addr_lo][  data  ]
MISO:     [xxxxxxxx][xxxxxxxx][xxxxxxxx][xxxxxxxx]
          |  CMD   | ADDR_HI | ADDR_LO |  DATA   |
```

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
// CPU instance
cpu_top cpu (
    .clk(clk),
    .reset(reset),
    .mem_addr(cpu_mem_addr),
    .mem_data_out(cpu_mem_data_out),
    .mem_data_in(cpu_mem_data_in),
    .mem_write(cpu_mem_write),
    .mem_read(cpu_mem_read),
    .mem_req(cpu_mem_req),
    .mem_ready(cpu_mem_ready),
    // ... other ports
);

// SPI Memory Controller
spi_memory_controller spi_ctrl (
    .clk(clk),
    .reset(reset),
    .mem_req(cpu_mem_req),
    .mem_we(cpu_mem_write),
    .mem_addr(cpu_mem_addr),
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

1. **Address Latch State**: Added 1-cycle delay (`ADDR_LATCH`) to ensure address is stable before starting SPI transaction

2. **Request Deassertion**: CPU only asserts `mem_req` when `mem_ready=0` to prevent double-fetch bugs

3. **16-bit SPI Address**: Even though CPU uses 8-bit addresses, SPI protocol sends 16-bit addresses (high byte always 0x00) for compatibility with standard SPI SRAM chips

4. **Single-byte Transfers**: No burst mode implemented; each access is independent

## Related Documents

- [SPI Protocol Overview](SPI_protocol.md) - General SPI protocol concepts
- [Neander CPU Architecture](NEANDER_cpu.md) - CPU instruction set and architecture
- [TinyTapeout Integration](info.md) - Pin mapping and integration details
