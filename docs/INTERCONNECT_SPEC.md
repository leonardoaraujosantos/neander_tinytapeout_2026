# NTERCONNECT_SPEC.md

## 1. Scope and goals

This spec defines the **NEANDER-X Interconnect / MMIO Hub** ("the hub") for TinyTapeout.

CPU clock: **50 MHz**.

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

- Instruction fetches in the range **0x0000..0x00FF** are served from an **internal ROM**.
- The internal ROM program must **toggle EXT_OUT0 forever** using the CPU `OUT` instruction.
- The debug ROM must not depend on external RAM/Flash.

> Note: The exact instruction bytes depend on your assembler encoding for `OUT port`. The hub only needs to guarantee the external pins exist and the ROM is reachable.

### 3.3 Debug-mode fixed PWM (10%)

When `boot_mode=1`, PWM is forced to a fixed 10% duty (ignoring PWM MMIO registers), to provide a second always-visible signal.

Recommended fixed parameters @ 50 MHz:

- `DIV = 1`
- `PERIOD = 49999`  (1 kHz)
- `DUTY = 5000`     (10%)

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

`f_pwm = 50_000_000 / (PWM_DIV * (PWM_PERIOD + 1))`

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

### 9.1 IRQ

- **0xF000** `IRQ_STATUS` (R)
  - bit0: ext IRQ_IN
  - bit1: timer hit (if enabled)
  - bit2: spi_periph_done (optional)
- **0xF002** `IRQ_ENABLE` (R/W)
- **0xF004** `IRQ_ACK` (W1C)

### 9.2 PWM

- **0xF010** `PWM_CTRL` (R/W)
- **0xF012** `PWM_DIV` (R/W)
- **0xF014** `PWM_PERIOD` (R/W)
- **0xF016** `PWM_DUTY` (R/W)

### 9.3 Timer

- **0xF020** `TMR_CTRL` (R/W)
  - bit0: enable
  - bit1: clear
  - bit2: irq_enable
  - bit3: timer_out_enable
  - bit4: timer_out_mode (optional)
- **0xF022** `TMR_DIV` (R/W)
- **0xF024** `TMR_COUNT` (R)
- **0xF026** `TMR_CMP` (R/W)
- **0xF028** `TMR_STATUS` (R/W1C)
  - bit0: hit

### 9.4 SPI peripheral engine (MMIO-controlled)

- **0xF030** `SPI_CTRL` (R/W)
- **0xF032** `SPI_DIV` (R/W)
- **0xF034** `SPI_SS` (R/W)
  - selects which CS line (ADC/DAC/GPIO/UART/ETH/FLASH as needed)
- **0xF036** `SPI_TXRX` (R/W)
  - write low byte triggers transfer; read low byte returns last RX
- **0xF038** `SPI_STATUS` (R)
  - busy/done/rx_valid

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

Used signals (recommended):

- SPI shared: 3
- CS: 7 (includes CS_RAM)
- PWM_OUT: 1
- TIMER_OUT: 1 (optional)
- BOOT_SEL: 1
- IRQ_IN: 1
- EXT_IN: 2
- EXT_OUT: 2

Total = 18 (or 17 if TIMER_OUT omitted) out of 24 available IO pads.

---

## 11. Debug ROM minimum program (functional requirement)

The internal ROM must implement a minimal loop:

- Write alternating values to `EXT_OUT0` using `OUT`.
- Include a delay loop so toggling is observable.

A typical implementation is:

1) `LDI 0x0001` ; AC = 1
2) `OUT <port_for_ext_out>`
3) delay loop
4) `LDI 0x0000` ; AC = 0
5) `OUT <port_for_ext_out>`
6) delay loop
7) `JMP 0x0000`

The hub requirement is only that `EXT_OUT0` is connected to the CPU OUT path and that the ROM is selected by BOOT_SEL.
