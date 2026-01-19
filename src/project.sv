/*
 * Copyright (c) 2024 Leonardo Araujo Santos
 * SPDX-License-Identifier: Apache-2.0
 *
 * Neander-X CPU for TinyTapeout with Interconnect Hub
 * 16-bit educational processor with 64KB address space via SPI SRAM/Flash
 * Includes PWM, Timer, IRQ controller, and SPI peripheral engine
 */

`default_nettype none

module tt_um_cpu_leonardoaraujosantos (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // ============================================================================
  // Pin Mapping (per INTERCONNECT_SPEC.md Section 10)
  // ============================================================================
  // ui_in[0] = BOOT_SEL   - Boot mode selection (0=SPI RAM, 1=Debug ROM)
  // ui_in[1] = IRQ_IN     - External interrupt input
  // ui_in[2] = EXT_IN0    - External input 0
  // ui_in[3] = EXT_IN1    - External input 1
  // ui_in[7:4] = spare    - Spare inputs for debug/straps
  //
  // uo_out[0] = PWM_OUT   - PWM output
  // uo_out[1] = EXT_OUT0  - External output 0
  // uo_out[2] = EXT_OUT1  - External output 1
  // uo_out[3] = TIMER_OUT - Timer output
  // uo_out[4] = CS_UART   - SPI CS for UART bridge
  // uo_out[5] = CS_ETH    - SPI CS for Ethernet
  // uo_out[6] = CS_DAC    - SPI CS for DAC
  // uo_out[7] = CS_ADC    - SPI CS for ADC
  //
  // uio[0] = SPI_SCLK     - Shared SPI clock (out)
  // uio[1] = SPI_MOSI     - Shared SPI MOSI (out)
  // uio[2] = SPI_MISO     - Shared SPI MISO (in)
  // uio[3] = CS_FLASH     - SPI CS for Flash (out)
  // uio[4] = CS_GPIO      - SPI CS for GPIO expander (out)
  // uio[5] = CS_RAM       - SPI CS for RAM (out)
  // uio[6] = spare        - Spare (out)
  // uio[7] = spare        - Spare (out)
  // ============================================================================

  // Reset is active high internally, rst_n is active low
  wire reset = ~rst_n;

  // ============================================================================
  // Input Signal Mapping
  // ============================================================================
  wire        boot_sel = ui_in[0];
  wire        irq_in   = ui_in[1];
  wire [1:0]  ext_in   = ui_in[3:2];

  // SPI MISO from bidirectional IO
  wire        spi_miso = uio_in[2];

  // ============================================================================
  // CPU Interface Signals
  // ============================================================================
  wire [15:0] cpu_mem_addr;
  wire [15:0] cpu_mem_data_out;
  wire [15:0] cpu_mem_data_in;
  wire        cpu_mem_write;
  wire        cpu_mem_read;
  wire        cpu_mem_req;
  wire        cpu_mem_ready;

  wire [7:0]  cpu_io_out;
  wire        cpu_io_write;
  wire [7:0]  cpu_io_in;
  wire [7:0]  cpu_io_status;

  // ============================================================================
  // Debug signals (directly from CPU)
  // ============================================================================
  wire [15:0] dbg_pc;
  wire [15:0] dbg_ac;
  wire [7:0]  dbg_ri;
  wire [15:0] dbg_sp;
  wire [15:0] dbg_x;
  wire [15:0] dbg_y;
  wire [15:0] dbg_fp;

  // ============================================================================
  // Interconnect Hub <-> SPI Memory Controller Interface
  // ============================================================================
  wire        spi_mem_req;
  wire        spi_mem_we;
  wire [15:0] spi_mem_addr;
  wire [15:0] spi_mem_wdata;
  wire [15:0] spi_mem_rdata;
  wire        spi_mem_ready;
  wire        spi_mem_cs_select;
  wire        spi_mem_busy;

  // SPI Memory controller outputs
  wire        spi_mem_cs_ram_n;
  wire        spi_mem_cs_flash_n;
  wire        spi_mem_sclk;
  wire        spi_mem_mosi;

  // ============================================================================
  // SPI Peripheral Engine Interface
  // ============================================================================
  wire        spi_periph_sclk;
  wire        spi_periph_mosi;
  wire [5:0]  spi_periph_cs_n;
  wire        spi_periph_busy;

  // ============================================================================
  // Peripheral Outputs
  // ============================================================================
  wire        pwm_out;
  wire        timer_out;
  wire [1:0]  ext_out;

  // ============================================================================
  // CPU Instantiation
  // ============================================================================
  cpu_top cpu (
    .clk(clk),
    .reset(reset),

    // Memory interface (to interconnect hub)
    .mem_addr(cpu_mem_addr),
    .mem_data_out(cpu_mem_data_out),
    .mem_data_in(cpu_mem_data_in),
    .mem_write(cpu_mem_write),
    .mem_read(cpu_mem_read),
    .mem_req(cpu_mem_req),
    .mem_ready(cpu_mem_ready),

    // I/O interface (to interconnect hub)
    .io_in(cpu_io_in),
    .io_status(cpu_io_status),
    .io_out(cpu_io_out),
    .io_write(cpu_io_write),

    // Debug outputs
    .dbg_pc(dbg_pc),
    .dbg_ac(dbg_ac),
    .dbg_ri(dbg_ri),
    .dbg_sp(dbg_sp),
    .dbg_x(dbg_x),
    .dbg_y(dbg_y),
    .dbg_fp(dbg_fp)
  );

  // ============================================================================
  // Interconnect Hub Instantiation
  // ============================================================================
  interconnect_hub hub (
    .clk(clk),
    .reset(reset),

    // Boot mode selection
    .boot_sel(boot_sel),

    // CPU memory interface
    .cpu_mem_addr(cpu_mem_addr),
    .cpu_mem_data_out(cpu_mem_data_out),
    .cpu_mem_data_in(cpu_mem_data_in),
    .cpu_mem_write(cpu_mem_write),
    .cpu_mem_read(cpu_mem_read),
    .cpu_mem_req(cpu_mem_req),
    .cpu_mem_ready(cpu_mem_ready),

    // CPU I/O interface
    .cpu_io_out(cpu_io_out),
    .cpu_io_write(cpu_io_write),
    .cpu_io_in(cpu_io_in),
    .cpu_io_status(cpu_io_status),

    // SPI Memory engine interface
    .spi_mem_req(spi_mem_req),
    .spi_mem_we(spi_mem_we),
    .spi_mem_addr(spi_mem_addr),
    .spi_mem_wdata(spi_mem_wdata),
    .spi_mem_rdata(spi_mem_rdata),
    .spi_mem_ready(spi_mem_ready),
    .spi_mem_cs_select(spi_mem_cs_select),
    .spi_mem_busy(spi_mem_busy),

    // SPI peripheral outputs
    .spi_periph_sclk(spi_periph_sclk),
    .spi_periph_mosi(spi_periph_mosi),
    .spi_periph_miso(spi_miso),
    .spi_periph_cs_n(spi_periph_cs_n),
    .spi_periph_busy(spi_periph_busy),

    // PWM output
    .pwm_out(pwm_out),

    // Timer output
    .timer_out(timer_out),

    // External I/O
    .ext_in(ext_in),
    .ext_out(ext_out),

    // IRQ input
    .irq_in(irq_in)
  );

  // ============================================================================
  // SPI Memory Controller Instantiation
  // ============================================================================
  spi_memory_controller spi_ctrl (
    .clk(clk),
    .reset(reset),

    // Hub interface
    .mem_req(spi_mem_req),
    .mem_we(spi_mem_we),
    .mem_addr(spi_mem_addr),
    .mem_wdata(spi_mem_wdata),
    .mem_rdata(spi_mem_rdata),
    .mem_ready(spi_mem_ready),

    // CS select and busy
    .cs_select(spi_mem_cs_select),
    .spi_busy(spi_mem_busy),

    // SPI Interface
    .spi_cs_ram_n(spi_mem_cs_ram_n),
    .spi_cs_flash_n(spi_mem_cs_flash_n),
    .spi_sclk(spi_mem_sclk),
    .spi_mosi(spi_mem_mosi),
    .spi_miso(spi_miso)
  );

  // ============================================================================
  // SPI Bus Arbitration
  // ============================================================================
  // SPI_MEM has priority over SPI_PERIPH (arbitration handled in hub)
  // When SPI_MEM is active, its signals drive the bus; otherwise SPI_PERIPH
  wire spi_sclk_out = spi_mem_busy ? spi_mem_sclk : spi_periph_sclk;
  wire spi_mosi_out = spi_mem_busy ? spi_mem_mosi : spi_periph_mosi;

  // ============================================================================
  // Output Pin Assignments
  // ============================================================================
  // Dedicated outputs (uo_out)
  assign uo_out[0] = pwm_out;              // PWM output
  assign uo_out[1] = ext_out[0];           // EXT_OUT0
  assign uo_out[2] = ext_out[1];           // EXT_OUT1
  assign uo_out[3] = timer_out;            // Timer output
  assign uo_out[4] = spi_periph_cs_n[2];   // CS_UART (from spi_periph, index 2)
  assign uo_out[5] = spi_periph_cs_n[3];   // CS_ETH  (from spi_periph, index 3)
  assign uo_out[6] = spi_periph_cs_n[1];   // CS_DAC  (from spi_periph, index 1)
  assign uo_out[7] = spi_periph_cs_n[0];   // CS_ADC  (from spi_periph, index 0)

  // Bidirectional IOs
  // uio_oe: 1=output, 0=input
  // uio[2] is SPI_MISO (input), all others are outputs
  assign uio_oe = 8'b11111011;

  assign uio_out[0] = spi_sclk_out;        // SPI_SCLK (shared)
  assign uio_out[1] = spi_mosi_out;        // SPI_MOSI (shared)
  assign uio_out[2] = 1'b0;                // SPI_MISO (input, drive low when output)
  assign uio_out[3] = spi_mem_cs_flash_n & spi_periph_cs_n[5];  // CS_FLASH (either engine)
  assign uio_out[4] = spi_periph_cs_n[4];  // CS_GPIO
  assign uio_out[5] = spi_mem_cs_ram_n;    // CS_RAM (from spi_mem only)
  assign uio_out[6] = 1'b1;                // Spare (inactive high)
  assign uio_out[7] = 1'b1;                // Spare (inactive high)

  // ============================================================================
  // Unused signals
  // ============================================================================
  wire _unused = &{ena, ui_in[7:4], uio_in[7:3], uio_in[1:0],
                   dbg_ac, dbg_ri, dbg_sp, dbg_x, dbg_y, dbg_fp, dbg_pc, 1'b0};

endmodule
