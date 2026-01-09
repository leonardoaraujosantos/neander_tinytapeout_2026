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

  // Internal signals for CPU <-> I/O interface
  wire [7:0] io_out_internal;
  wire       io_write;

  // Debug signals
  wire [7:0] dbg_pc;
  wire [7:0] dbg_ac;
  wire [7:0] dbg_ri;

  // I/O status: bit 0 from uio_in indicates "data available"
  wire [7:0] io_status = {7'b0, uio_in[0]};

  // Reset is active high internally, rst_n is active low
  wire reset = ~rst_n;

  // RAM 256x8 (internal)
  reg [7:0] ram [0:255];

  // Asynchronous read
  assign mem_data_in = ram[mem_addr];

  // Synchronous write
  always @(posedge clk) begin
    if (mem_write)
      ram[mem_addr] <= mem_data_out;
  end

  // CPU instantiation
  cpu_top cpu (
    .clk(clk),
    .reset(reset),

    // Memory interface
    .mem_addr(mem_addr),
    .mem_data_out(mem_data_out),
    .mem_data_in(mem_data_in),
    .mem_write(mem_write),

    // I/O interface
    .io_in(ui_in),           // Input from dedicated inputs (keyboard/switches)
    .io_status(io_status),   // Status from bidirectional pins
    .io_out(io_out_internal),
    .io_write(io_write),

    // Debug outputs
    .dbg_pc(dbg_pc),
    .dbg_ac(dbg_ac),
    .dbg_ri(dbg_ri)
  );

  // Output assignments
  assign uo_out  = io_out_internal;  // CPU output -> dedicated outputs (display)
  assign uio_out = dbg_pc;           // Debug: show PC on bidirectional outputs
  assign uio_oe  = 8'b11111111;      // All bidirectional pins as outputs (for debug)

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, uio_in[7:1], 1'b0};

endmodule
