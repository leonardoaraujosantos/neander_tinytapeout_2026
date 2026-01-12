// ============================================================================
// neander_tb_wrapper.sv - Testbench wrapper for cocotb (8-bit + SPI)
// Instantiates CPU + SPI Controller + SPI SRAM Model
// ============================================================================

`timescale 1ns/1ps

module neander_tb_wrapper (
    input  logic        clk,
    input  logic        reset,

    // Memory load interface (for cocotb to load programs)
    // Uses 8-bit addresses for 256 byte address space
    input  logic        mem_load_en,
    input  logic [7:0]  mem_load_addr,
    input  logic [7:0]  mem_load_data,

    // I/O interface
    input  logic [7:0]  io_in,
    input  logic [7:0]  io_status,
    output logic [7:0]  io_out,
    output logic        io_write,

    // Debug outputs (8-bit for PC, SP, FP)
    output logic [7:0]  dbg_pc,
    output logic [7:0]  dbg_ac,
    output logic [7:0]  dbg_ri,
    output logic [7:0]  dbg_sp,
    output logic [7:0]  dbg_x,
    output logic [7:0]  dbg_y,
    output logic [7:0]  dbg_fp,

    // Memory read interface (for verification) - 8-bit address
    input  logic [7:0]  mem_read_addr,
    output logic [7:0]  mem_read_data,

    // Debug signals for SPI handshaking (visible from cocotb)
    output logic [7:0]  cpu_mem_addr,
    output logic        cpu_mem_req,
    output logic        cpu_mem_ready,
    output logic [7:0]  cpu_mem_data_in,

    // Debug signals for SPI controller (to trace address latching)
    output logic [7:0]  spi_addr_latch,
    output logic [3:0]  spi_state
);

    // ============================================================================
    // CPU <-> SPI Controller Interface (8-bit address)
    // ============================================================================
    logic [7:0]  cpu_mem_addr_int;
    logic [7:0]  cpu_mem_data_out;
    logic [7:0]  cpu_mem_data_in_int;
    logic        cpu_mem_write;
    logic        cpu_mem_read;
    logic        cpu_mem_req_int;
    logic        cpu_mem_ready_int;

    // Expose signals for debug
    assign cpu_mem_addr = cpu_mem_addr_int;
    assign cpu_mem_req = cpu_mem_req_int;
    assign cpu_mem_ready = cpu_mem_ready_int;
    assign cpu_mem_data_in = cpu_mem_data_in_int;

    // Expose SPI controller internals for debug
    assign spi_addr_latch = spi_ctrl.addr_latch;
    assign spi_state = spi_ctrl.state;

    // ============================================================================
    // SPI Controller <-> SPI SRAM Interface
    // ============================================================================
    logic        spi_cs_n;
    logic        spi_sclk;
    logic        spi_mosi;
    logic        spi_miso;

    // ============================================================================
    // CPU Instantiation (8-bit address)
    // ============================================================================
    cpu_top cpu (
        .clk(clk),
        .reset(reset),

        // Memory interface (to SPI controller)
        .mem_addr(cpu_mem_addr_int),
        .mem_data_out(cpu_mem_data_out),
        .mem_data_in(cpu_mem_data_in_int),
        .mem_write(cpu_mem_write),
        .mem_read(cpu_mem_read),
        .mem_req(cpu_mem_req_int),
        .mem_ready(cpu_mem_ready_int),

        // I/O interface
        .io_in(io_in),
        .io_status(io_status),
        .io_out(io_out),
        .io_write(io_write),

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
    // SPI Memory Controller Instantiation (8-bit address)
    // ============================================================================
    spi_memory_controller spi_ctrl (
        .clk(clk),
        .reset(reset),

        // CPU Interface (8-bit address)
        .mem_req(cpu_mem_req_int),
        .mem_we(cpu_mem_write),
        .mem_addr(cpu_mem_addr_int),
        .mem_wdata(cpu_mem_data_out),
        .mem_rdata(cpu_mem_data_in_int),
        .mem_ready(cpu_mem_ready_int),

        // SPI Interface (to SPI SRAM model)
        .spi_cs_n(spi_cs_n),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso)
    );

    // ============================================================================
    // SPI SRAM Model (64KB for simulation)
    // ============================================================================
    spi_sram_model spi_ram (
        .spi_cs_n(spi_cs_n),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso)
    );

    // ============================================================================
    // Memory Load Interface (direct access to SPI SRAM for loading programs)
    // Uses the spi_sram_model's internal memory array directly
    // Zero-extend 8-bit address to 16-bit for SPI SRAM model
    // ============================================================================
    always_ff @(posedge clk) begin
        if (mem_load_en) begin
            spi_ram.memory[{8'h00, mem_load_addr}] <= mem_load_data;
        end
    end

    // Memory read interface for verification (zero-extend 8-bit address)
    assign mem_read_data = spi_ram.memory[{8'h00, mem_read_addr}];

endmodule
