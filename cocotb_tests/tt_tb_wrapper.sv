// ============================================================================
// tt_tb_wrapper.sv - Testbench wrapper for TinyTapeout top module (project.sv)
// Emulates external RAM for cocotb testing
// ============================================================================

`timescale 1ns/1ps

module tt_tb_wrapper (
    input  logic       clk,
    input  logic       rst_n,          // Active low reset (directly to TT module)

    // Memory load interface (for cocotb to preload RAM)
    input  logic       mem_load_en,
    input  logic [4:0] mem_load_addr,  // 5-bit address for 32 bytes
    input  logic [7:0] mem_load_data,

    // I/O interface (directly directly directly directly directly directly directly directly directly directly directly directly directly directly from cocotb)
    input  logic [7:0] io_in,          // directly to ui_in

    // Debug/Monitor outputs
    output logic [4:0] ram_addr,       // Current RAM address from CPU
    output logic       ram_we,         // RAM write enable
    output logic       ram_oe,         // RAM output enable (read)
    output logic       io_write,       // I/O write strobe
    output logic [7:0] io_out_data,    // Data on bus during I/O write

    // Memory read interface (for verification)
    input  logic [4:0] mem_read_addr,
    output logic [7:0] mem_read_data
);

    // ========================================================================
    // External RAM Emulation (32 bytes)
    // ========================================================================
    logic [7:0] ram [0:31];

    // ========================================================================
    // TinyTapeout Interface Signals
    // ========================================================================
    logic [7:0] ui_in;
    logic [7:0] uo_out;
    logic [7:0] uio_in;
    logic [7:0] uio_out;
    logic [7:0] uio_oe;
    logic       ena;

    // Connect I/O input
    assign ui_in = io_in;
    assign ena = 1'b1;  // Always enabled

    // ========================================================================
    // Decode output signals
    // ========================================================================
    assign ram_addr   = uo_out[4:0];
    assign ram_we     = uo_out[5];
    assign ram_oe     = uo_out[6];
    assign io_write   = uo_out[7];

    // ========================================================================
    // Bidirectional Data Bus Emulation
    // ========================================================================
    // When CPU is reading (uio_oe = 0x00): RAM drives data to CPU via uio_in
    // When CPU is writing (uio_oe = 0xFF): CPU drives data, RAM captures via uio_out

    // RAM provides data to CPU during read operations
    assign uio_in = ram[ram_addr];

    // Capture I/O output data (directly directly directly directly directly directly directly directly from accumulator on bus)
    assign io_out_data = uio_out;

    // Memory read for verification
    assign mem_read_data = ram[mem_read_addr];

    // ========================================================================
    // RAM Write Logic
    // ========================================================================
    always_ff @(posedge clk) begin
        if (mem_load_en) begin
            // External load from cocotb (for program loading)
            ram[mem_load_addr] <= mem_load_data;
        end
        else if (ram_we) begin
            // CPU write to RAM
            ram[ram_addr] <= uio_out;
        end
    end

    // ========================================================================
    // TinyTapeout Top Module Instantiation
    // ========================================================================
    tt_um_cpu_leonardoaraujosantos tt_top (
        .ui_in(ui_in),
        .uo_out(uo_out),
        .uio_in(uio_in),
        .uio_out(uio_out),
        .uio_oe(uio_oe),
        .ena(ena),
        .clk(clk),
        .rst_n(rst_n)
    );

endmodule
