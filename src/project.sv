/*
 * Copyright (c) 2024 Leonardo Araujo Santos
 * SPDX-License-Identifier: Apache-2.0
 *
 * Neander-X CPU for TinyTapeout with SPI Memory Interface
 * 16-bit educational processor with 64KB address space via SPI SRAM
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
  // Pin Mapping for SPI Memory Interface
  // ============================================================================
  // uo_out[0] = SPI_CS_N   - Chip select (directly to SPI SRAM)
  // uo_out[1] = SPI_SCLK   - Serial clock (directly to SPI SRAM)
  // uo_out[2] = SPI_MOSI   - Master Out Slave In (directly to SPI SRAM)
  // uo_out[3] = reserved (directly from debug)
  // uo_out[4] = reserved (directly from debug)
  // uo_out[5] = reserved (directly from debug)
  // uo_out[6] = reserved (directly from debug)
  // uo_out[7] = IO_WRITE   - I/O write strobe for external output latch
  //
  // ui_in[0]  = SPI_MISO   - Master In Slave Out (directly from SPI SRAM)
  // ui_in[7:1]= io_in      - General purpose input (directly from switches/keyboard)
  //
  // uio[7:0]  = Debug outputs (directly from PC low byte and other debug)
  // ============================================================================

  // Reset is active high internally, rst_n is active low
  wire reset = ~rst_n;

  // ============================================================================
  // CPU <-> SPI Controller Interface (16-bit data)
  // ============================================================================
  wire [15:0] cpu_mem_addr;       // 16-bit addressing (64KB space)
  wire [15:0] cpu_mem_data_out;   // 16-bit data
  wire [15:0] cpu_mem_data_in;    // 16-bit data
  wire        cpu_mem_write;
  wire        cpu_mem_read;
  wire        cpu_mem_req;
  wire        cpu_mem_ready;

  // ============================================================================
  // SPI Controller <-> External SPI SRAM Interface
  // ============================================================================
  wire        spi_cs_n;
  wire        spi_sclk;
  wire        spi_mosi;
  wire        spi_miso;

  // ============================================================================
  // CPU <-> I/O Interface
  // ============================================================================
  wire [7:0]  io_out_internal;
  wire        io_write;
  wire [7:0]  io_in_cpu;
  wire [7:0]  io_status;

  // I/O input: use ui_in[7:1] for general I/O (ui_in[0] is SPI_MISO)
  assign io_in_cpu = {ui_in[7:1], 1'b0};
  assign io_status = 8'b0;  // Status register (unused for now)

  // ============================================================================
  // Debug signals - 16-bit data width
  // ============================================================================
  wire [15:0] dbg_pc;
  wire [15:0] dbg_ac;   // 16-bit AC
  wire [7:0]  dbg_ri;
  wire [15:0] dbg_sp;
  wire [15:0] dbg_x;    // 16-bit X
  wire [15:0] dbg_y;    // 16-bit Y
  wire [15:0] dbg_fp;
  wire [15:0] dbg_b;    // 16-bit B register

  // ============================================================================
  // CPU Instantiation
  // ============================================================================
  cpu_top cpu (
    .clk(clk),
    .reset(reset),

    // Memory interface (to SPI controller)
    .mem_addr(cpu_mem_addr),
    .mem_data_out(cpu_mem_data_out),
    .mem_data_in(cpu_mem_data_in),
    .mem_write(cpu_mem_write),
    .mem_read(cpu_mem_read),
    .mem_req(cpu_mem_req),
    .mem_ready(cpu_mem_ready),

    // I/O interface
    .io_in(io_in_cpu),
    .io_status(io_status),
    .io_out(io_out_internal),
    .io_write(io_write),

    // Debug outputs
    .dbg_pc(dbg_pc),
    .dbg_ac(dbg_ac),
    .dbg_ri(dbg_ri),
    .dbg_sp(dbg_sp),
    .dbg_x(dbg_x),
    .dbg_y(dbg_y),
    .dbg_fp(dbg_fp),
    .dbg_b(dbg_b)
  );

  // ============================================================================
  // SPI Memory Controller Instantiation
  // ============================================================================
  spi_memory_controller spi_ctrl (
    .clk(clk),
    .reset(reset),

    // CPU Interface
    .mem_req(cpu_mem_req),
    .mem_we(cpu_mem_write),
    .mem_addr(cpu_mem_addr),
    .mem_wdata(cpu_mem_data_out),
    .mem_rdata(cpu_mem_data_in),
    .mem_ready(cpu_mem_ready),

    // SPI Interface (directly to external SPI SRAM)
    .spi_cs_n(spi_cs_n),
    .spi_sclk(spi_sclk),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso)
  );

  // ============================================================================
  // Output Pin Assignments
  // ============================================================================
  // SPI signals on dedicated outputs
  assign uo_out[0] = spi_cs_n;           // SPI Chip Select (directly to SRAM)
  assign uo_out[1] = spi_sclk;           // SPI Clock (directly to SRAM)
  assign uo_out[2] = spi_mosi;           // SPI MOSI (directly to SRAM)
  assign uo_out[3] = dbg_ac[0];          // Debug: AC bit 0
  assign uo_out[4] = dbg_ac[1];          // Debug: AC bit 1
  assign uo_out[5] = dbg_ac[2];          // Debug: AC bit 2
  assign uo_out[6] = dbg_ac[3];          // Debug: AC bit 3
  assign uo_out[7] = io_write;           // I/O write strobe

  // SPI MISO input
  assign spi_miso = ui_in[0];

  // Bidirectional IOs configured as outputs for debug
  assign uio_oe  = 8'hFF;                // All outputs
  assign uio_out = dbg_pc[7:0];          // Debug: PC low byte (for 16-bit PC)

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, uio_in, io_out_internal, dbg_ac[15:4], dbg_ri, dbg_sp, dbg_x, dbg_y, dbg_fp, dbg_b, dbg_pc[15:8], 1'b0};

endmodule
