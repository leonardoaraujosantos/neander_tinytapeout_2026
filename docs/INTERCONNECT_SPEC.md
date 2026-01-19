# INTERCONNECT_SPEC.md

## 1. Scope and goals

This spec defines the **NEANDER-X Interconnect / MMIO Hub** ("the hub") for TinyTapeout.

CPU clock: **10 MHz** (as defined in info.yaml; PWM/timer calculations below use this value).

Required capabilities:

1. The hub **looks like memory** to the CPU using the existing `mem_*` request/ready handshake.
2. The hub **automatically serves external RAM** (SPI SRAM) for normal instruction fetch and data loads/stores.
3. A strap pin **BOOT_SEL** selects instruction fetch from an **internal debug ROM** or from external RAM/Flash.
4. The hub exposes a shared SPI bus plus chip-selects for external peripherals.
5. The hub includes an internal **PWM** (divider + period + duty) and in debug mode it is **forced to 10%** duty.
6. The hub includes an internal **timer** (divider + counter + compare) and may optionally expose a timer output pin.
7. The hub exposes an **IRQ input** and provides IRQ status/enable/ack via MMIO (polling-friendly).
8. The hub exposes **2 external input pins and 2 external output pins** that are driven by the CPU IN/OUT path.

---

## 2. CPU-facing interfaces

### 2.1 Memory interface (unchanged)

From `cpu_top`:

- `mem_addr[15:0]`
- `mem_data_out[15:0]`
- `mem_data_in[15:0]`
- `mem_write`, `mem_read`
- `mem_req` (request valid)
- `mem_ready` (transaction done)

Rules:

- The CPU asserts `mem_req` together with `mem_read` or `mem_write`.
- The hub must assert `mem_ready` **for exactly one cycle** when the transaction completes.
- For MMIO reads, `mem_ready` should be asserted quickly (recommended: next cycle).
- For SPI-backed accesses, `mem_ready` is asserted when the SPI transaction finishes.
- For reads, `mem_data_in` is valid when `mem_ready=1`.

### 2.2 IN/OUT interface (few real pins)

From `cpu_top`:

- `io_in[7:0]` (read data)
- `io_status[7:0]` (status flags)
- `io_out[7:0]` (write data)
- `io_write` (strobe)

Fixed external pin mapping:

- `io_in[0]` = `EXT_IN0`
- `io_in[1]` = `EXT_IN1`
- `io_in[7:2]` = 0

- `EXT_OUT0` latches `io_out[0]` on `io_write`
- `EXT_OUT1` latches `io_out[1]` on `io_write`

Recommended status bits (optional but useful):

- `io_status[0]` = `IRQ_PENDING` (masked)
- `io_status[1]` = `SPI_MEM_BUSY`
- `io_status[2]` = `SPI_PERIPH_BUSY`
- others = 0

---

## 3. Boot select and internal debug ROM

### 3.1 BOOT_SEL sampling

- `BOOT_SEL` is sampled and latched on reset release (first clock where reset is deasserted).
- Latched value is `boot_mode` and must not change during runtime.

### 3.2 Debug ROM behavior

When `boot_mode=1`:

- **All reads** from addresses **0x0000..0x00FF** are served from the internal ROM.
- Writes to 0x0000..0x00FF still go to SPI RAM (ROM is read-only).
- The internal ROM program must **toggle EXT_OUT0 forever** using the CPU `OUT` instruction.
- The debug ROM must not depend on external RAM/Flash.

> **Implementation Note:** The CPU does not provide a separate "instruction fetch" signal - both opcode fetch and operand fetch use `addr_sel=01` (PC-based addressing). Therefore, the simplest approach is to shadow ALL reads from the ROM address range when in boot mode. This is safe because the ROM program is self-contained and does not access data from 0x0000-0x00FF.

```systemverilog
// ROM access detection (simplified)
wire is_rom_access = boot_mode && mem_read && (mem_addr[15:8] == 8'h00);
```

> Note: The exact instruction bytes depend on your assembler encoding for `OUT port`. The hub only needs to guarantee the external pins exist and the ROM is reachable.

### 3.3 Debug-mode fixed PWM (10%)

When `boot_mode=1`, PWM is forced to a fixed 10% duty (ignoring PWM MMIO registers), to provide a second always-visible signal.

Recommended fixed parameters @ 10 MHz:

- `DIV = 1`
- `PERIOD = 9999`   (1 kHz)
- `DUTY = 1000`     (10%)

When `boot_mode=0`, PWM is controlled by MMIO registers.

---

## 4. Address map (hardware-fixed)

The hub decodes the CPU address space as follows:

- **0x0000–0xDFFF** : External RAM window (SPI SRAM, read/write)
- **0xE000–0xEFFF** : External Flash window (SPI Flash, typically read-only / XIP-like)
- **0xF000–0xF0FF** : MMIO registers (internal)

In `boot_mode=1`, 0x0000..0x00FF instruction fetches come from the internal ROM; the remaining ranges behave normally.

---

## 5. External SPI bus

### 5.1 Shared SPI pins

- `SPI_SCLK` (out)
- `SPI_MOSI` (out)
- `SPI_MISO` (in)

### 5.2 Chip selects

You requested **6 chip selects** for these peripherals:

1. ADC
2. DAC
3. GPIO expander
4. Flash
5. SPI-to-UART bridge (TX)
6. Ethernet

**Important:** automatic external RAM (SPI SRAM) also needs its own chip select (`CS_RAM`).

Therefore this design uses **7 CS lines total**:

- `CS_RAM` (SPI SRAM)  **(required for requirement #2)**
- `CS_FLASH`
- `CS_ADC`
- `CS_DAC`
- `CS_GPIO`
- `CS_UART`
- `CS_ETH`

(If you must keep exactly 6 CS lines, RAM would need an external decoder or a serialized CS expander; the simplest robust approach is keeping `CS_RAM` as an additional line.)

### 5.3 Internal SPI engines

To keep RAM automatic while still supporting other devices:

- **SPI_MEM engine (automatic):** used by the address map for the RAM/Flash windows.
- **SPI_PERIPH engine (MMIO-controlled):** used for ADC/DAC/GPIO/UART/Ethernet transactions.

Arbitration:

- SPI_MEM has priority.
- If SPI_MEM is busy, SPI_PERIPH must stall and report busy.

### 5.4 SPI arbitration details

```
                ┌─────────────────────────────────────────┐
                │            SPI ARBITER                  │
                │                                         │
  SPI_MEM ──────┤  spi_mem_req ────┐                      │
  Engine        │  spi_mem_busy ───┼──> io_status[1]      │
                │                  │                      │
  SPI_PERIPH ───┤  spi_periph_req ─┤                      │
  Engine        │  spi_periph_busy ┼──> io_status[2]      │
                │                  │                      │
                │         ┌────────┴────────┐             │
                │         │   Priority MUX  │             │
                │         │ (MEM wins ties) │             │
                │         └────────┬────────┘             │
                │                  │                      │
                └──────────────────┼──────────────────────┘
                                   │
                    ┌──────────────┴──────────────┐
                    │     Shared SPI Outputs      │
                    │  SPI_SCLK, SPI_MOSI         │
                    │     (directly active CS)    │
                    └─────────────────────────────┘
```

Grant logic:
- `spi_mem_grant = spi_mem_req`
- `spi_periph_grant = spi_periph_req & ~spi_mem_busy`

Software should poll `io_status[1]` (SPI_MEM_BUSY) before starting a peripheral SPI transaction to avoid stalls.

---

## 6. Internal PWM (normal mode)

### 6.1 Output

- `PWM_OUT` is a dedicated top-level output pin.

### 6.2 Registers (MMIO)

- `PWM_CTRL`  (enable, invert)
- `PWM_DIV`   (>=1)
- `PWM_PERIOD`
- `PWM_DUTY`  (0..PERIOD)

Frequency:

`f_pwm = 10_000_000 / (PWM_DIV * (PWM_PERIOD + 1))`

Duty:

Output is high when `counter < PWM_DUTY`.

Debug mode forces PWM to 10% (Section 3.3).

---

## 7. Internal timer and optional TIMER_OUT pin

### 7.1 Should the timer expose a pin?

**Recommendation: Yes (optional).**

Reason: It is extremely useful for bring-up (scope/LA-visible heartbeat) even when software is not fully working.

If pin budget becomes tight, `TIMER_OUT` is the first thing you can drop; timer functionality remains available via MMIO.

### 7.2 Timer features

- Divider/prescaler
- Free-running counter
- Compare register
- Sticky compare-hit flag (W1C)
- Optional timer interrupt source

Optional `TIMER_OUT` behaviors (selectable):

- Mode 0: toggle output on each compare hit
- Mode 1: generate a 1-tick pulse on each compare hit

---

## 8. IRQ input

- External pin: `IRQ_IN`

MMIO interrupt model (polling-friendly):

- `IRQ_STATUS` exposes pending sources
- `IRQ_ENABLE` masks sources
- `IRQ_ACK` clears sticky sources (W1C)

Define:

`IRQ_PENDING = (IRQ_STATUS & IRQ_ENABLE) != 0`

Expose `IRQ_PENDING` as `io_status[0]` (recommended).

---

## 9. MMIO register map

MMIO base: **0xF000** (16-bit addressed; registers are 16-bit words).

### 9.1 Register summary

| Address | Name | Access | Reset | Description |
|---------|------|--------|-------|-------------|
| 0xF000 | IRQ_STATUS | R | 0x0000 | Interrupt status (sticky) |
| 0xF002 | IRQ_ENABLE | R/W | 0x0000 | Interrupt enable mask |
| 0xF004 | IRQ_ACK | W1C | - | Interrupt acknowledge |
| 0xF010 | PWM_CTRL | R/W | 0x0000 | PWM control |
| 0xF012 | PWM_DIV | R/W | 0x0001 | PWM clock divider |
| 0xF014 | PWM_PERIOD | R/W | 0xFFFF | PWM period |
| 0xF016 | PWM_DUTY | R/W | 0x0000 | PWM duty cycle |
| 0xF020 | TMR_CTRL | R/W | 0x0000 | Timer control |
| 0xF022 | TMR_DIV | R/W | 0x0001 | Timer clock divider |
| 0xF024 | TMR_COUNT | R | 0x0000 | Timer counter (read-only) |
| 0xF026 | TMR_CMP | R/W | 0xFFFF | Timer compare value |
| 0xF028 | TMR_STATUS | R/W1C | 0x0000 | Timer status |
| 0xF030 | SPI_CTRL | R/W | 0x0000 | SPI peripheral control |
| 0xF032 | SPI_DIV | R/W | 0x0001 | SPI clock divider |
| 0xF034 | SPI_SS | R/W | 0x0000 | SPI slave select |
| 0xF036 | SPI_TXRX | R/W | 0x0000 | SPI data register |
| 0xF038 | SPI_STATUS | R | 0x0000 | SPI status |

### 9.2 IRQ registers (0xF000-0xF004)

#### IRQ_STATUS (0xF000) - Read-only

| Bit | Name | Description |
|-----|------|-------------|
| 0 | EXT_IRQ | External IRQ_IN pin is asserted (active high, sticky) |
| 1 | TMR_IRQ | Timer compare match occurred (sticky) |
| 2 | SPI_IRQ | SPI peripheral transfer complete (sticky) |
| 15:3 | - | Reserved (read as 0) |

#### IRQ_ENABLE (0xF002) - Read/Write

| Bit | Name | Description |
|-----|------|-------------|
| 0 | EXT_EN | Enable external IRQ |
| 1 | TMR_EN | Enable timer IRQ |
| 2 | SPI_EN | Enable SPI complete IRQ |
| 15:3 | - | Reserved (write 0) |

#### IRQ_ACK (0xF004) - Write-1-to-Clear

| Bit | Name | Description |
|-----|------|-------------|
| 0 | EXT_ACK | Write 1 to clear EXT_IRQ status |
| 1 | TMR_ACK | Write 1 to clear TMR_IRQ status |
| 2 | SPI_ACK | Write 1 to clear SPI_IRQ status |
| 15:3 | - | Reserved |

**IRQ_PENDING** (exposed as `io_status[0]`):
```
IRQ_PENDING = (IRQ_STATUS & IRQ_ENABLE) != 0
```

### 9.3 PWM registers (0xF010-0xF016)

#### PWM_CTRL (0xF010) - Read/Write

| Bit | Name | Reset | Description |
|-----|------|-------|-------------|
| 0 | ENABLE | 0 | 1 = PWM enabled, 0 = PWM output low |
| 1 | INVERT | 0 | 1 = Invert output polarity |
| 15:2 | - | 0 | Reserved (write 0) |

> **Note:** In `boot_mode=1`, PWM_CTRL is ignored and PWM runs with fixed 10% duty.

#### PWM_DIV (0xF012) - Read/Write

| Bit | Name | Reset | Description |
|-----|------|-------|-------------|
| 15:0 | DIVIDER | 0x0001 | Clock divider (1-65535). 0 treated as 1. |

#### PWM_PERIOD (0xF014) - Read/Write

| Bit | Name | Reset | Description |
|-----|------|-------|-------------|
| 15:0 | PERIOD | 0xFFFF | PWM period in divided clock ticks (0-65535) |

#### PWM_DUTY (0xF016) - Read/Write

| Bit | Name | Reset | Description |
|-----|------|-------|-------------|
| 15:0 | DUTY | 0x0000 | Duty cycle count (0 to PERIOD) |

**PWM Frequency:**
```
f_pwm = 10_000_000 / (PWM_DIV * (PWM_PERIOD + 1))
```

**PWM Output:**
```
PWM_OUT = (counter < PWM_DUTY) ^ INVERT   (when ENABLE=1)
PWM_OUT = 0 ^ INVERT                       (when ENABLE=0)
```

### 9.4 Timer registers (0xF020-0xF028)

#### TMR_CTRL (0xF020) - Read/Write

| Bit | Name | Reset | Description |
|-----|------|-------|-------------|
| 0 | ENABLE | 0 | 1 = Timer counting enabled |
| 1 | CLEAR | 0 | Write 1 to reset counter to 0 (self-clearing) |
| 2 | IRQ_EN | 0 | 1 = Generate IRQ on compare match |
| 3 | OUT_EN | 0 | 1 = Enable TIMER_OUT pin |
| 4 | OUT_MODE | 0 | 0 = Toggle on match, 1 = Pulse on match |
| 5 | AUTO_RELOAD | 0 | 1 = Reset counter on match, 0 = Free-running |
| 15:6 | - | 0 | Reserved (write 0) |

#### TMR_DIV (0xF022) - Read/Write

| Bit | Name | Reset | Description |
|-----|------|-------|-------------|
| 15:0 | DIVIDER | 0x0001 | Prescaler divider (1-65535). 0 treated as 1. |

**Timer tick rate:**
```
f_tick = 10_000_000 / TMR_DIV
```

#### TMR_COUNT (0xF024) - Read-only

| Bit | Name | Description |
|-----|------|-------------|
| 15:0 | COUNT | Current counter value (read-only) |

> Writing to TMR_COUNT has no effect. Use TMR_CTRL.CLEAR to reset.

#### TMR_CMP (0xF026) - Read/Write

| Bit | Name | Reset | Description |
|-----|------|-------|-------------|
| 15:0 | COMPARE | 0xFFFF | Compare match value |

**Compare match** occurs when `TMR_COUNT == TMR_CMP`.

#### TMR_STATUS (0xF028) - Read/Write-1-to-Clear

| Bit | Name | Description |
|-----|------|-------------|
| 0 | HIT | 1 = Compare match occurred (write 1 to clear) |
| 1 | RUNNING | 1 = Timer is currently counting (read-only) |
| 15:2 | - | Reserved |

### 9.5 SPI peripheral registers (0xF030-0xF038)

#### SPI_CTRL (0xF030) - Read/Write

| Bit | Name | Reset | Description |
|-----|------|-------|-------------|
| 0 | ENABLE | 0 | 1 = SPI peripheral enabled |
| 1 | CPOL | 0 | Clock polarity: 0 = idle low, 1 = idle high |
| 2 | CPHA | 0 | Clock phase: 0 = sample on 1st edge, 1 = sample on 2nd edge |
| 3 | LSB_FIRST | 0 | 0 = MSB first (default), 1 = LSB first |
| 4 | CS_ACTIVE_HIGH | 0 | 0 = CS active low (default), 1 = CS active high |
| 5 | AUTO_CS | 1 | 1 = Auto-assert CS during transfer, 0 = Manual CS control |
| 15:6 | - | 0 | Reserved (write 0) |

> **Note:** CPOL=0, CPHA=0 (Mode 0) is most common for SPI peripherals.

#### SPI_DIV (0xF032) - Read/Write

| Bit | Name | Reset | Description |
|-----|------|-------|-------------|
| 15:0 | DIVIDER | 0x0001 | SPI clock divider (1-65535). 0 treated as 1. |

**SPI clock frequency:**
```
f_spi = 10_000_000 / (2 * SPI_DIV)
```

| SPI_DIV | f_spi |
|---------|-------|
| 1 | 5 MHz |
| 2 | 2.5 MHz |
| 5 | 1 MHz |
| 10 | 500 kHz |
| 50 | 100 kHz |

#### SPI_SS (0xF034) - Read/Write

| Bit | Name | Reset | Description |
|-----|------|-------|-------------|
| 2:0 | SELECT | 0 | CS line selection (directly active when AUTO_CS=0) |
| 3 | CS_MANUAL | 0 | 1 = Assert selected CS line manually |
| 15:4 | - | 0 | Reserved |

**CS line encoding (SELECT field):**

| SELECT | CS Line | Peripheral |
|--------|---------|------------|
| 0 | CS_ADC | ADC (uo_out[7]) |
| 1 | CS_DAC | DAC (uo_out[6]) |
| 2 | CS_UART | SPI-UART bridge (uo_out[4]) |
| 3 | CS_ETH | Ethernet (uo_out[5]) |
| 4 | CS_GPIO | GPIO expander (uio[4]) |
| 5 | CS_FLASH | Flash (uio[3]) - for peripheral access |
| 6-7 | - | Reserved |

> **Note:** CS_RAM (uio[5]) is controlled exclusively by the SPI_MEM engine and cannot be selected by SPI_SS.

#### SPI_TXRX (0xF036) - Read/Write

| Bit | Name | Description |
|-----|------|-------------|
| 7:0 | DATA | TX: Write data to transmit. RX: Read last received data. |
| 15:8 | - | Reserved (read as 0) |

**Operation:**
- Writing to SPI_TXRX starts an 8-bit SPI transfer (if ENABLE=1)
- Reading returns the last received byte
- Writing while busy is ignored

#### SPI_STATUS (0xF038) - Read-only

| Bit | Name | Description |
|-----|------|-------------|
| 0 | BUSY | 1 = Transfer in progress |
| 1 | DONE | 1 = Transfer complete (cleared on SPI_TXRX read) |
| 2 | RX_VALID | 1 = Valid data in RX buffer |
| 15:3 | - | Reserved (read as 0) |

**Typical SPI transfer sequence:**
```c
// 1. Configure SPI (once)
MMIO[SPI_CTRL] = 0x0021;  // Enable, Auto-CS, Mode 0
MMIO[SPI_DIV]  = 5;       // 1 MHz SPI clock
MMIO[SPI_SS]   = 0;       // Select ADC

// 2. Wait for any previous transfer
while (MMIO[SPI_STATUS] & 0x01);  // Wait for not busy

// 3. Start transfer
MMIO[SPI_TXRX] = 0x42;    // Send byte, starts transfer

// 4. Wait for completion
while (!(MMIO[SPI_STATUS] & 0x02));  // Wait for done

// 5. Read received data
uint8_t rx = MMIO[SPI_TXRX];
```

---

## 10. TinyTapeout pin mapping (fits)

TinyTapeout top-level IOs:

- `ui_in[7:0]`  : dedicated inputs
- `uo_out[7:0]` : dedicated outputs
- `uio[7:0]`    : bidirectional (with `uio_oe`)

### 10.1 Recommended assignment

Inputs (`ui_in`):

- `ui_in[0]` = BOOT_SEL
- `ui_in[1]` = IRQ_IN
- `ui_in[2]` = EXT_IN0
- `ui_in[3]` = EXT_IN1
- `ui_in[7:4]` = spare straps/debug

Outputs (`uo_out`):

- `uo_out[0]` = PWM_OUT
- `uo_out[1]` = EXT_OUT0
- `uo_out[2]` = EXT_OUT1
- `uo_out[3]` = TIMER_OUT (optional)
- `uo_out[4]` = CS_UART
- `uo_out[5]` = CS_ETH
- `uo_out[6]` = CS_DAC
- `uo_out[7]` = CS_ADC

Bidirectional (`uio`):

- `uio[0]` = SPI_SCLK  (out, `uio_oe=1`)
- `uio[1]` = SPI_MOSI  (out, `uio_oe=1`)
- `uio[2]` = SPI_MISO  (in,  `uio_oe=0`)
- `uio[3]` = CS_FLASH  (out, `uio_oe=1`)
- `uio[4]` = CS_GPIO   (out, `uio_oe=1`)
- `uio[5]` = CS_RAM    (out, `uio_oe=1`)
- `uio[6]` = spare CS / debug
- `uio[7]` = spare CS / debug

### 10.2 Pin budget summary

**Verification: YES, we have enough pins.**

| Category | Signal(s) | Count | Location |
|----------|-----------|-------|----------|
| SPI bus | SCLK, MOSI, MISO | 3 | uio[0:2] |
| CS lines | RAM, FLASH, GPIO | 3 | uio[3:5] |
| CS lines | ADC, DAC, UART, ETH | 4 | uo_out[4:7] |
| PWM output | PWM_OUT | 1 | uo_out[0] |
| Timer output | TIMER_OUT | 1 | uo_out[3] |
| External I/O | EXT_OUT0, EXT_OUT1 | 2 | uo_out[1:2] |
| External I/O | EXT_IN0, EXT_IN1 | 2 | ui_in[2:3] |
| Control | BOOT_SEL | 1 | ui_in[0] |
| Control | IRQ_IN | 1 | ui_in[1] |
| **Total used** | | **18** | |
| Spare inputs | ui_in[7:4] | 4 | Available for debug/straps |
| Spare bidir | uio[7:6] | 2 | Available for extra CS or debug |
| **Total spare** | | **6** | |

### 10.3 uio_oe configuration

```systemverilog
// Bidirectional pin direction configuration
// 1 = output, 0 = input
assign uio_oe = 8'b11111011;
//                    ││││││││
//                    │││││││└─ uio[0] = SPI_SCLK  (out)
//                    ││││││└── uio[1] = SPI_MOSI  (out)
//                    │││││└─── uio[2] = SPI_MISO  (in)  <-- only input
//                    ││││└──── uio[3] = CS_FLASH  (out)
//                    │││└───── uio[4] = CS_GPIO   (out)
//                    ││└────── uio[5] = CS_RAM    (out)
//                    │└─────── uio[6] = spare     (out)
//                    └──────── uio[7] = spare     (out)
```

---

## 11. Debug ROM implementation

### 11.1 ROM size

The debug ROM is **only 25 bytes**, not 256. It is implemented as combinational logic (case statement), which synthesizes to approximately 100-150 LUTs. For addresses 0x0019-0x00FF, the ROM returns `0x00` (NOP).

### 11.2 Program listing

```
Address  Bytes           Instruction       Comment
-------  --------------  ----------------  --------------------------
0x0000:  0xE0 0x01 0x00  LDI 0x0001        ; AC = 1
0x0003:  0xD0 0x01       OUT 0x01          ; EXT_OUT0 = 1
0x0005:  0xE0 0xFF 0xFF  LDI 0xFFFF        ; Delay counter = 65535
0x0008:  0x88 0x08 0x00  DECJNZ 0x0008     ; Loop until AC=0
0x000B:  0xE0 0x00 0x00  LDI 0x0000        ; AC = 0
0x000E:  0xD0 0x01       OUT 0x01          ; EXT_OUT0 = 0
0x0010:  0xE0 0xFF 0xFF  LDI 0xFFFF        ; Delay counter = 65535
0x0013:  0x88 0x13 0x00  DECJNZ 0x0013     ; Loop until AC=0
0x0016:  0x80 0x00 0x00  JMP 0x0000        ; Restart
                                          ; Total: 25 bytes
```

### 11.3 Toggle frequency calculation

At 10 MHz CPU clock:
- Each DECJNZ loop iteration: ~6-8 cycles (fetch + decode + execute)
- 65535 iterations × 7 cycles ≈ 458,745 cycles per delay
- Delay time: 458,745 / 10,000,000 ≈ 45.9 ms
- Full toggle period: ~92 ms (two delays)
- **Toggle frequency: ~11 Hz** (easily visible on LED or scope)

### 11.4 RTL implementation

```systemverilog
module debug_rom (
    input  logic [7:0] addr,
    output logic [7:0] data
);
    always_comb begin
        case (addr[4:0])  // Only 5 bits needed for 25 addresses
            5'h00: data = 8'hE0;  // LDI opcode
            5'h01: data = 8'h01;  // imm_lo = 1
            5'h02: data = 8'h00;  // imm_hi = 0
            5'h03: data = 8'hD0;  // OUT opcode
            5'h04: data = 8'h01;  // port = 1
            5'h05: data = 8'hE0;  // LDI opcode
            5'h06: data = 8'hFF;  // imm_lo = 0xFF
            5'h07: data = 8'h00;  // imm_hi = 0x00 (changed from 0xFF for faster toggle)
            5'h08: data = 8'h88;  // DECJNZ opcode
            5'h09: data = 8'h08;  // addr_lo = 0x08
            5'h0A: data = 8'h00;  // addr_hi = 0x00
            5'h0B: data = 8'hE0;  // LDI opcode
            5'h0C: data = 8'h00;  // imm_lo = 0
            5'h0D: data = 8'h00;  // imm_hi = 0
            5'h0E: data = 8'hD0;  // OUT opcode
            5'h0F: data = 8'h01;  // port = 1
            5'h10: data = 8'hE0;  // LDI opcode
            5'h11: data = 8'hFF;  // imm_lo = 0xFF
            5'h12: data = 8'h00;  // imm_hi = 0x00
            5'h13: data = 8'h88;  // DECJNZ opcode
            5'h14: data = 8'h13;  // addr_lo = 0x13
            5'h15: data = 8'h00;  // addr_hi = 0x00
            5'h16: data = 8'h80;  // JMP opcode
            5'h17: data = 8'h00;  // addr_lo = 0x00
            5'h18: data = 8'h00;  // addr_hi = 0x00
            default: data = 8'h00;  // NOP for unused addresses
        endcase
    end
endmodule
```

> **Note:** The delay counter uses 0x00FF (255) instead of 0xFFFF (65535) in the RTL example above for faster toggling during testing. Adjust as needed.

The hub requirement is only that `EXT_OUT0` is connected to the CPU OUT path and that the ROM is selected by BOOT_SEL.

---

## 12. Implementation timing requirements

### 12.1 Memory access latency

| Access Type | Latency | Notes |
|-------------|---------|-------|
| MMIO read/write | 1 cycle | Combinational decode + registered ready |
| Debug ROM read | 1 cycle | Combinational ROM + registered ready |
| SPI RAM read | ~80-100 cycles | 5 bytes @ 16 cycles/byte |
| SPI RAM write | ~80-100 cycles | 5 bytes @ 16 cycles/byte |
| SPI Flash read | ~80-100 cycles | Same as RAM |

### 12.2 Ready signal generation

```systemverilog
// Simplified ready logic
always_ff @(posedge clk or posedge reset) begin
    if (reset)
        mem_ready <= 1'b0;
    else if (is_mmio_access || is_rom_access)
        mem_ready <= mem_req & ~mem_ready;  // 1-cycle pulse
    else
        mem_ready <= spi_mem_ready;  // From SPI engine
end
```

### 12.3 Data path muxing

```systemverilog
// Read data mux (active when mem_ready asserted)
always_comb begin
    if (is_rom_access)
        mem_data_in = {8'h00, rom_data};  // ROM returns 8-bit, zero-extend
    else if (is_mmio_access)
        mem_data_in = mmio_rdata;
    else
        mem_data_in = spi_mem_rdata;
end
```

---

## 13. Module hierarchy

```
tt_um_cpu_leonardoaraujosantos (project.sv)
│
├── cpu_top (top_cpu_neander_x.sv)
│   ├── neander_datapath
│   │   ├── neander_alu
│   │   ├── sequential_multiplier
│   │   └── sequential_divider
│   └── neander_control
│
└── interconnect_hub (interconnect_hub.sv)  [NEW]
    │
    ├── debug_rom (debug_rom.sv)  [NEW]
    │   └── 25-byte combinational ROM
    │
    ├── spi_mem_engine (spi_memory_controller.sv)  [MODIFY]
    │   └── Add: cs_select input, busy output
    │
    ├── pwm (pwm.sv)  [NEW]
    │   └── PWM generator with MMIO interface
    │
    ├── timer (timer.sv)  [NEW]
    │   └── Timer/counter with compare match
    │
    ├── irq_ctrl (irq_ctrl.sv)  [NEW]
    │   └── IRQ status/enable/ack registers
    │
    └── spi_periph (spi_periph.sv)  [NEW]
        └── Software-controlled SPI engine
```

### 13.1 Files to create

| File | Lines (est.) | Purpose |
|------|--------------|---------|
| `interconnect_hub.sv` | ~300 | Main hub: address decode, muxes, arbitration |
| `debug_rom.sv` | ~50 | 25-byte toggle program |
| `pwm.sv` | ~80 | PWM generator with debug mode override |
| `timer.sv` | ~100 | Timer with prescaler and compare |
| `irq_ctrl.sv` | ~60 | IRQ status/enable/ack |
| `spi_periph.sv` | ~150 | MMIO-controlled SPI for peripherals |

### 13.2 Files to modify

| File | Changes |
|------|---------|
| `spi_memory_controller.sv` | Add CS select input, busy output signal |
| `project.sv` | Instantiate hub, update pin mapping |
| `info.yaml` | Update pinout descriptions |

---

## 14. Block diagram

```
                              TinyTapeout Pins
        ┌────────────────────────────────────────────────────────┐
        │   ui_in[7:0]        uo_out[7:0]        uio[7:0]        │
        │   ┌───────┐         ┌───────┐         ┌───────┐       │
        │   │BOOT_SEL│         │PWM_OUT│         │SPI_SCLK│       │
        │   │IRQ_IN │         │EXT_OUT│         │SPI_MOSI│       │
        │   │EXT_IN │         │TIMER  │         │SPI_MISO│       │
        │   │spare  │         │CS_x4  │         │CS_x3  │       │
        │   └───┬───┘         └───┬───┘         └───┬───┘       │
        └───────┼─────────────────┼─────────────────┼───────────┘
                │                 │                 │
    ┌───────────┴─────────────────┴─────────────────┴───────────┐
    │                    INTERCONNECT HUB                        │
    │  ┌─────────────────────────────────────────────────────┐  │
    │  │                 Address Decoder                      │  │
    │  │  0x0000-0xDFFF: RAM    0xE000-0xEFFF: Flash         │  │
    │  │  0xF000-0xF0FF: MMIO   (boot: 0x0000-0x00FF: ROM)   │  │
    │  └─────────────────────────────────────────────────────┘  │
    │           │                    │                │         │
    │           ▼                    ▼                ▼         │
    │  ┌─────────────┐      ┌─────────────┐   ┌────────────┐   │
    │  │ Debug ROM   │      │ SPI_MEM     │   │   MMIO     │   │
    │  │ (25 bytes)  │      │ Engine      │   │ Registers  │   │
    │  └─────────────┘      └──────┬──────┘   │            │   │
    │                              │          │ ┌────────┐ │   │
    │  ┌───────────────────────────┤          │ │  IRQ   │ │   │
    │  │       SPI Arbiter         │          │ │  PWM   │ │   │
    │  │  (MEM priority > PERIPH)  │          │ │ Timer  │ │   │
    │  └───────────────┬───────────┘          │ │SPI_PER │ │   │
    │                  │                      │ └────────┘ │   │
    │                  ▼                      └────────────┘   │
    │         Shared SPI Bus                                   │
    └──────────────────────────────────────────────────────────┘
                │
                ▼
    ┌──────────────────────────────────────────────────────────┐
    │                      CPU (cpu_top)                        │
    │  mem_addr[15:0]  mem_data_out[15:0]  mem_data_in[15:0]   │
    │  mem_write       mem_read            mem_req  mem_ready   │
    │  io_in[7:0]      io_out[7:0]         io_write io_status   │
    └──────────────────────────────────────────────────────────┘
```

---

---

## 15. Testing strategy

### 15.1 Current test infrastructure

The project uses **cocotb** (Python-based hardware verification) with **Icarus Verilog** as the simulator.

**Existing test assets:**
- `cocotb_tests/` — 189 CPU tests with SPI memory model
- `cocotb_tests/neander_tb_wrapper.sv` — Testbench wrapper
- `src/spi_sram_model.sv` — Behavioral SPI SRAM model
- `cocotb_tests/neander_common.py` — Shared test infrastructure

### 15.2 New testbench architecture

The interconnect requires a new testbench wrapper that includes all hub components:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    interconnect_tb_wrapper.sv                       │
│                                                                     │
│  ┌─────────┐    ┌─────────────────┐    ┌─────────────────────────┐ │
│  │ cpu_top │───>│ interconnect_hub │───>│ SPI SRAM Model (RAM)   │ │
│  └─────────┘    │                 │    │ SPI Flash Model (FLASH) │ │
│                 │  - debug_rom    │    │ SPI Periph Model (ADC)  │ │
│                 │  - pwm          │    └─────────────────────────┘ │
│                 │  - timer        │                                 │
│                 │  - irq_ctrl     │    Test control signals:       │
│                 │  - spi_periph   │    - boot_sel_tb               │
│                 └─────────────────┘    - irq_in_tb                 │
│                                        - mmio_probe_*              │
│  Exposed debug signals:                                             │
│  - pwm_out, timer_out, ext_out[1:0], ext_in[1:0]                   │
│  - All CS lines, SPI bus                                            │
│  - Internal MMIO register values (for verification)                 │
└─────────────────────────────────────────────────────────────────────┘
```

### 15.3 Test categories

#### A. Address decoding tests (`test_interconnect_decode.py`)

| Test | Description | Expected |
|------|-------------|----------|
| `test_ram_access_low` | Read/write at 0x0000 | Goes to SPI RAM |
| `test_ram_access_high` | Read/write at 0xDFFF | Goes to SPI RAM |
| `test_flash_access` | Read at 0xE000 | Goes to SPI Flash (CS_FLASH) |
| `test_mmio_access` | Read/write at 0xF000 | Returns in 1 cycle |
| `test_mmio_boundary` | Access at 0xF0FF | MMIO region boundary |
| `test_invalid_high` | Access at 0xF100+ | Undefined (return 0?) |

#### B. Debug ROM tests (`test_debug_rom.py`)

| Test | Description | Expected |
|------|-------------|----------|
| `test_boot_mode_0` | BOOT_SEL=0, fetch 0x0000 | Goes to SPI RAM |
| `test_boot_mode_1_fetch` | BOOT_SEL=1, fetch 0x0000 | Returns ROM byte (0xE0) |
| `test_boot_mode_1_data` | BOOT_SEL=1, data read 0x0000 | Goes to SPI RAM (not ROM) |
| `test_rom_toggle_program` | Run full ROM program | EXT_OUT0 toggles |
| `test_rom_returns_nop` | Fetch 0x0080 (boot_mode=1) | Returns 0x00 (NOP) |

#### C. MMIO register tests (`test_mmio_regs.py`)

| Test | Description | Expected |
|------|-------------|----------|
| `test_mmio_read_default` | Read all regs after reset | Default values |
| `test_irq_enable_rw` | Write/read IRQ_ENABLE | Value persists |
| `test_irq_status_readonly` | Write to IRQ_STATUS | No effect (read-only) |
| `test_irq_ack_w1c` | Set status, write ACK | Status clears |
| `test_pwm_regs_rw` | Write/read PWM regs | Values persist |
| `test_timer_regs_rw` | Write/read timer regs | Values persist |
| `test_timer_count_readonly` | Write to TMR_COUNT | No effect (read-only) |
| `test_spi_regs_rw` | Write/read SPI_PERIPH regs | Values persist |
| `test_mmio_1_cycle_latency` | Measure MMIO access time | 1 CPU cycle |

#### D. PWM tests (`test_pwm.py`)

| Test | Description | Expected |
|------|-------------|----------|
| `test_pwm_disabled` | PWM_CTRL.enable=0 | PWM_OUT=0 |
| `test_pwm_50_duty` | PERIOD=99, DUTY=50 | 50% duty cycle |
| `test_pwm_10_duty` | PERIOD=99, DUTY=10 | 10% duty cycle |
| `test_pwm_invert` | PWM_CTRL.invert=1 | Output inverted |
| `test_pwm_debug_override` | boot_mode=1 | Forced 10% duty |
| `test_pwm_frequency` | DIV=1, PERIOD=9 | 1 MHz output |

#### E. Timer tests (`test_timer.py`)

| Test | Description | Expected |
|------|-------------|----------|
| `test_timer_disabled` | TMR_CTRL.enable=0 | Count stays 0 |
| `test_timer_count_up` | Enable timer | Count increments |
| `test_timer_prescaler` | DIV=10 | Count at 1/10 rate |
| `test_timer_compare_hit` | COUNT reaches CMP | Status.hit=1 |
| `test_timer_irq` | irq_enable=1, hit | IRQ_STATUS.timer=1 |
| `test_timer_out_toggle` | timer_out_mode=0 | Output toggles |
| `test_timer_out_pulse` | timer_out_mode=1 | 1-cycle pulse |
| `test_timer_clear` | TMR_CTRL.clear=1 | Count resets |

#### F. IRQ tests (`test_irq.py`)

| Test | Description | Expected |
|------|-------------|----------|
| `test_irq_in_sets_status` | Assert IRQ_IN pin | STATUS.ext=1 |
| `test_irq_masked` | ENABLE=0, IRQ_IN=1 | io_status[0]=0 |
| `test_irq_unmasked` | ENABLE=1, IRQ_IN=1 | io_status[0]=1 |
| `test_irq_ack_clears` | Write ACK=1 | STATUS.ext=0 |
| `test_irq_timer_source` | Timer hit | STATUS.timer=1 |
| `test_irq_combined` | Multiple sources | OR of enabled |

#### G. SPI peripheral engine tests (`test_spi_periph.py`)

| Test | Description | Expected |
|------|-------------|----------|
| `test_spi_periph_idle` | No transfer started | STATUS.busy=0 |
| `test_spi_periph_tx` | Write TXRX | Transfer starts |
| `test_spi_periph_rx` | Complete transfer | TXRX has RX data |
| `test_spi_periph_cs_select` | Set SPI_SS=2 | CS_DAC asserts |
| `test_spi_periph_divider` | SPI_DIV=4 | Clock at clk/8 |
| `test_spi_periph_loopback` | MOSI→MISO loopback | TX=RX |

#### H. SPI arbitration tests (`test_spi_arb.py`)

| Test | Description | Expected |
|------|-------------|----------|
| `test_mem_has_priority` | RAM access during periph | RAM completes first |
| `test_periph_stalls` | Start periph, then RAM | Periph waits |
| `test_status_bits` | Check io_status[1:2] | Busy bits correct |
| `test_no_conflict` | Sequential accesses | Both complete |

#### I. System integration tests (`test_system.py`)

| Test | Description | Expected |
|------|-------------|----------|
| `test_boot_mode_1_full` | Full boot from ROM | LED toggles, PWM 10% |
| `test_boot_mode_0_spi` | Boot from SPI RAM | Program runs |
| `test_mmio_from_cpu` | CPU writes PWM regs | PWM changes |
| `test_irq_polling` | CPU polls io_status | Sees IRQ |
| `test_spi_periph_from_cpu` | CPU does SPI transfer | Data exchanged |
| `test_ext_io` | CPU IN/OUT instructions | EXT pins toggle |

### 15.4 Simulation models needed

#### Existing models (reuse):
- `spi_sram_model.sv` — SPI SRAM for RAM region

#### New models to create:

**A. `spi_flash_model.sv`** — SPI Flash for Flash region
```systemverilog
module spi_flash_model (
    input  logic spi_cs_n,
    input  logic spi_sclk,
    input  logic spi_mosi,
    output logic spi_miso
);
    // Similar to SRAM model but read-only
    // Supports READ (0x03) command only
    logic [7:0] memory [0:4095];  // 4KB flash window
endmodule
```

**B. `spi_periph_loopback.sv`** — Simple loopback for peripheral SPI testing
```systemverilog
module spi_periph_loopback (
    input  logic spi_cs_n,
    input  logic spi_sclk,
    input  logic spi_mosi,
    output logic spi_miso
);
    // Loopback: MISO = delayed MOSI (for testing)
    logic [7:0] shift_reg;
    assign spi_miso = shift_reg[7];
    // Shift on clock edges...
endmodule
```

### 15.5 Test execution

```bash
# Run all interconnect tests
cd cocotb_tests
make MODULE=test_interconnect

# Run specific test category
make MODULE=test_debug_rom
make MODULE=test_mmio_regs
make MODULE=test_pwm
make MODULE=test_timer
make MODULE=test_irq
make MODULE=test_spi_periph
make MODULE=test_spi_arb
make MODULE=test_system

# Run with waveform capture
make MODULE=test_debug_rom WAVES=1

# Run with Verilator (faster)
make MODULE=test_interconnect SIM=verilator
```

### 15.6 Hardware testing (post-tapeout)

Once the chip is fabricated, test with this procedure:

**1. Boot mode test (BOOT_SEL=1):**
- Connect oscilloscope to PWM_OUT and EXT_OUT0
- Power on with BOOT_SEL=1
- Expected: PWM_OUT shows 10% duty @ 1kHz, EXT_OUT0 toggles @ ~11 Hz

**2. SPI RAM test (BOOT_SEL=0):**
- Connect SPI SRAM (23LC512) to CS_RAM, SCLK, MOSI, MISO
- Load test program via external programmer
- Expected: CPU executes from external RAM

**3. Peripheral test:**
- Connect SPI DAC (MCP4921) to CS_DAC
- Run program that writes to SPI_PERIPH registers
- Expected: DAC outputs voltage

**4. Timer/IRQ test:**
- Configure timer via MMIO
- Expected: TIMER_OUT toggles at configured rate

### 15.7 Test coverage goals

| Category | Min Coverage | Notes |
|----------|--------------|-------|
| Address decoding | 100% | All regions tested |
| MMIO registers | 100% | All regs R/W verified |
| Debug ROM | 100% | All 25 bytes |
| PWM | 90% | All modes except edge cases |
| Timer | 90% | All modes |
| IRQ | 100% | All sources, masks |
| SPI arbitration | 100% | Priority + stall |
| Boot modes | 100% | Both modes |

### 15.8 CI integration

Add to `.github/workflows/test.yaml`:

```yaml
- name: Run interconnect tests
  run: |
    cd cocotb_tests
    make MODULE=test_interconnect
    ! grep failure results.xml
```

---

## 16. Revision history

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | Initial | Original specification |
| 1.1 | Updated | Clock corrected to 10 MHz, ROM size clarified (25 bytes), pin budget verified, implementation details added |
| 1.2 | Updated | Added comprehensive testing strategy (Section 15) |
| 1.3 | Updated | Clarified ROM access detection (all reads shadowed in boot_mode), added detailed MMIO register bit field definitions with reset values |
