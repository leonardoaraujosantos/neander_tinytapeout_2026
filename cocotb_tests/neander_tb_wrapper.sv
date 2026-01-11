// ============================================================================
// neander_tb_wrapper.sv - Testbench wrapper for cocotb
// Instantiates CPU + RAM with loadable memory interface
// ============================================================================

`timescale 1ns/1ps

module neander_tb_wrapper (
    input  logic       clk,
    input  logic       reset,

    // Memory load interface (for cocotb to load programs)
    input  logic       mem_load_en,
    input  logic [7:0] mem_load_addr,
    input  logic [7:0] mem_load_data,

    // I/O interface
    input  logic [7:0] io_in,
    input  logic [7:0] io_status,
    output logic [7:0] io_out,
    output logic       io_write,

    // Debug outputs
    output logic [7:0] dbg_pc,
    output logic [7:0] dbg_ac,
    output logic [7:0] dbg_ri,
    output logic [7:0] dbg_sp,

    // Memory read interface (for verification)
    input  logic [7:0] mem_read_addr,
    output logic [7:0] mem_read_data
);

    // Internal signals
    logic [7:0] mem_addr;
    logic [7:0] mem_data_out;
    logic [7:0] mem_data_in;
    logic       mem_write;
    logic       mem_read;

    // RAM 256x8
    logic [7:0] ram [0:255];

    // Asynchronous read
    assign mem_data_in = ram[mem_addr];
    assign mem_read_data = ram[mem_read_addr];

    // Synchronous write with load capability
    always_ff @(posedge clk) begin
        if (mem_load_en) begin
            // External load (from cocotb)
            ram[mem_load_addr] <= mem_load_data;
        end
        else if (mem_write) begin
            // CPU write
            ram[mem_addr] <= mem_data_out;
        end
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
        .mem_read(mem_read),

        // I/O interface
        .io_in(io_in),
        .io_status(io_status),
        .io_out(io_out),
        .io_write(io_write),

        // Debug
        .dbg_pc(dbg_pc),
        .dbg_ac(dbg_ac),
        .dbg_ri(dbg_ri),
        .dbg_sp(dbg_sp)
    );

endmodule
