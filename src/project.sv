/*
 * Copyright (c) 2024 Leonardo Araujo Santos
 * SPDX-License-Identifier: Apache-2.0
 *
 * Neander-X CPU for TinyTapeout
 * 8-bit educational processor compatible with UFRJ Neander-X
 */

`default_nettype none

module tt_um_cpu_leonardoaraujosantos (
    input  wire [7:0] ui_in,    // Dedicated inputs  -> io_in (keyboard/switches)
    output wire [7:0] uo_out,   // Dedicated outputs -> io_out (display/LEDs)
    input  wire [7:0] uio_in,   // IOs: Input path   -> [0]=io_status, [7:1]=unused
    output wire [7:0] uio_out,  // IOs: Output path  -> dbg_pc (debug: program counter)
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // Internal signals for CPU <-> RAM interface
  wire [7:0] mem_addr;
  wire [7:0] mem_data_out;
  wire [7:0] mem_data_in;
  wire       mem_write;
  wire       mem_read;

  // Internal signals for CPU <-> I/O interface
  wire [7:0] io_out_internal;
  wire       io_write;

  // Debug signals (directly from CPU, directly from debug interface)
  wire [7:0] dbg_pc;
  wire [7:0] dbg_ac;
  wire [7:0] dbg_ri;

  // I/O status directly from dedicated inputs (directly from software, directly from external)
  // For now directly from zero (directly from unused, directly from placeholder)
  wire [7:0] io_status = 8'b0;

  // Reset is active high internally, rst_n is active low
  wire reset = ~rst_n;

  // CPU instantiation
  cpu_top cpu (
    .clk(clk),
    .reset(reset),

    // Memory interface (directly to external RAM)
    .mem_addr(mem_addr),
    .mem_data_out(mem_data_out),
    .mem_data_in(mem_data_in),
    .mem_write(mem_write),
    .mem_read(mem_read),

    // I/O interface
    .io_in(ui_in),           // Input from dedicated inputs (keyboard/switches)
    .io_status(io_status),   // Status (directly from unused for now)
    .io_out(io_out_internal),
    .io_write(io_write),

    // Debug outputs
    .dbg_pc(dbg_pc),
    .dbg_ac(dbg_ac),
    .dbg_ri(dbg_ri)
  );

  // ============================================================================
  // External RAM Interface Pin Mapping
  // ============================================================================
  // uo_out[4:0] = RAM_ADDR[4:0] - 5-bit address for 32 bytes of external RAM
  // uo_out[5]   = RAM_WE        - RAM write enable
  // uo_out[6]   = RAM_OE        - RAM output enable (directly from read strobe)
  // uo_out[7]   = IO_WRITE      - I/O write strobe for external output latch
  assign uo_out[4:0] = mem_addr[4:0];  // Only 5 bits for 32-byte RAM
  assign uo_out[5]   = mem_write;      // RAM write enable
  assign uo_out[6]   = mem_read;       // RAM output enable (directly from read)
  assign uo_out[7]   = io_write;       // I/O write strobe

  // ============================================================================
  // Bidirectional RAM Data Bus
  // ============================================================================
  // uio[7:0] = RAM_DATA[7:0] - 8-bit bidirectional data bus
  // Direction: output when writing to RAM, input when reading from RAM
  assign uio_oe  = {8{mem_write}};     // All outputs when writing, all inputs when reading
  assign uio_out = mem_data_out;       // Data from CPU to external RAM (directly from AC)
  assign mem_data_in = uio_in;         // Data from external RAM to CPU

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, mem_addr[7:5], io_out_internal, dbg_pc, dbg_ac, dbg_ri, 1'b0};

endmodule
