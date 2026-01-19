// ============================================================================
// interconnect_tb_wrapper.sv - Testbench wrapper for interconnect hub testing
// Instantiates CPU + Interconnect Hub + SPI Controller + SPI SRAM Model
// ============================================================================

`timescale 1ns/1ps

module interconnect_tb_wrapper (
    input  logic        clk,
    input  logic        reset,

    // Boot mode and control
    input  logic        boot_sel,           // Boot mode selection (0=SPI RAM, 1=Debug ROM)
    input  logic        irq_in,             // External IRQ input
    input  logic [1:0]  ext_in,             // External inputs (EXT_IN0, EXT_IN1)

    // Memory load interface (for cocotb to load programs)
    input  logic        mem_load_en,
    input  logic [15:0] mem_load_addr,
    input  logic [7:0]  mem_load_data,

    // Debug outputs (CPU registers)
    output logic [15:0] dbg_pc,
    output logic [15:0] dbg_ac,
    output logic [7:0]  dbg_ri,
    output logic [15:0] dbg_sp,
    output logic [15:0] dbg_x,
    output logic [15:0] dbg_y,
    output logic [15:0] dbg_fp,
    output logic [15:0] dbg_b,

    // Peripheral outputs
    output logic        pwm_out,
    output logic        timer_out,
    output logic [1:0]  ext_out,

    // I/O interface outputs
    output logic [7:0]  io_out,
    output logic        io_write,
    output logic [7:0]  io_status,

    // Memory read interface (for verification)
    input  logic [15:0] mem_read_addr,
    output logic [7:0]  mem_read_data,

    // SPI peripheral chip selects (directly active from spi_periph)
    output logic [5:0]  spi_periph_cs_n,

    // SPI Memory CS (from spi_memory_controller)
    output logic        spi_cs_ram_n,
    output logic        spi_cs_flash_n,

    // Debug signals for CPU memory interface
    output logic [15:0] cpu_mem_addr,
    output logic        cpu_mem_req,
    output logic        cpu_mem_ready,
    output logic [15:0] cpu_mem_data_in,
    output logic [15:0] cpu_mem_data_out,
    output logic        cpu_mem_write,
    output logic        cpu_mem_read,

    // SPI bus signals (for waveform debugging)
    output logic        spi_sclk,
    output logic        spi_mosi,
    output logic        spi_miso,

    // Hub internal signals (for testing)
    output logic        spi_mem_busy,
    output logic        spi_periph_busy
);

    // ============================================================================
    // CPU <-> Interconnect Hub Interface
    // ============================================================================
    logic [15:0] cpu_mem_addr_int;
    logic [15:0] cpu_mem_data_out_int;
    logic [15:0] cpu_mem_data_in_int;
    logic        cpu_mem_write_int;
    logic        cpu_mem_read_int;
    logic        cpu_mem_req_int;
    logic        cpu_mem_ready_int;

    logic [7:0]  cpu_io_out_int;
    logic        cpu_io_write_int;
    logic [7:0]  cpu_io_in_int;
    logic [7:0]  cpu_io_status_int;

    // Expose signals for debug
    assign cpu_mem_addr = cpu_mem_addr_int;
    assign cpu_mem_req = cpu_mem_req_int;
    assign cpu_mem_ready = cpu_mem_ready_int;
    assign cpu_mem_data_in = cpu_mem_data_in_int;
    assign cpu_mem_data_out = cpu_mem_data_out_int;
    assign cpu_mem_write = cpu_mem_write_int;
    assign cpu_mem_read = cpu_mem_read_int;
    assign io_out = cpu_io_out_int;
    assign io_write = cpu_io_write_int;
    assign io_status = cpu_io_status_int;

    // ============================================================================
    // Interconnect Hub <-> SPI Memory Controller Interface
    // ============================================================================
    logic        spi_mem_req;
    logic        spi_mem_we;
    logic [15:0] spi_mem_addr;
    logic [15:0] spi_mem_wdata;
    logic [15:0] spi_mem_rdata;
    logic        spi_mem_ready;
    logic        spi_mem_cs_select;
    logic        spi_mem_busy_int;

    assign spi_mem_busy = spi_mem_busy_int;

    // ============================================================================
    // SPI Peripheral Interface
    // ============================================================================
    logic        spi_periph_sclk;
    logic        spi_periph_mosi;
    logic        spi_periph_miso;
    logic [5:0]  spi_periph_cs_n_int;
    logic        spi_periph_busy_int;

    assign spi_periph_cs_n = spi_periph_cs_n_int;
    assign spi_periph_busy = spi_periph_busy_int;

    // ============================================================================
    // SPI Memory Controller Outputs
    // ============================================================================
    logic        spi_mem_cs_ram_n;
    logic        spi_mem_cs_flash_n;
    logic        spi_mem_sclk;
    logic        spi_mem_mosi;

    assign spi_cs_ram_n = spi_mem_cs_ram_n;
    assign spi_cs_flash_n = spi_mem_cs_flash_n;

    // ============================================================================
    // SPI Bus Arbitration (same as project.sv)
    // ============================================================================
    assign spi_sclk = spi_mem_busy_int ? spi_mem_sclk : spi_periph_sclk;
    assign spi_mosi = spi_mem_busy_int ? spi_mem_mosi : spi_periph_mosi;
    assign spi_periph_miso = spi_miso;

    // ============================================================================
    // Peripheral Outputs
    // ============================================================================
    logic        pwm_out_int;
    logic        timer_out_int;
    logic [1:0]  ext_out_int;

    assign pwm_out = pwm_out_int;
    assign timer_out = timer_out_int;
    assign ext_out = ext_out_int;

    // ============================================================================
    // CPU Instantiation
    // ============================================================================
    cpu_top cpu (
        .clk(clk),
        .reset(reset),

        // Memory interface (to interconnect hub)
        .mem_addr(cpu_mem_addr_int),
        .mem_data_out(cpu_mem_data_out_int),
        .mem_data_in(cpu_mem_data_in_int),
        .mem_write(cpu_mem_write_int),
        .mem_read(cpu_mem_read_int),
        .mem_req(cpu_mem_req_int),
        .mem_ready(cpu_mem_ready_int),

        // I/O interface (to interconnect hub)
        .io_in(cpu_io_in_int),
        .io_status(cpu_io_status_int),
        .io_out(cpu_io_out_int),
        .io_write(cpu_io_write_int),

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
    // Interconnect Hub Instantiation
    // ============================================================================
    interconnect_hub hub (
        .clk(clk),
        .reset(reset),

        // Boot mode selection
        .boot_sel(boot_sel),

        // CPU memory interface
        .cpu_mem_addr(cpu_mem_addr_int),
        .cpu_mem_data_out(cpu_mem_data_out_int),
        .cpu_mem_data_in(cpu_mem_data_in_int),
        .cpu_mem_write(cpu_mem_write_int),
        .cpu_mem_read(cpu_mem_read_int),
        .cpu_mem_req(cpu_mem_req_int),
        .cpu_mem_ready(cpu_mem_ready_int),

        // CPU I/O interface
        .cpu_io_out(cpu_io_out_int),
        .cpu_io_write(cpu_io_write_int),
        .cpu_io_in(cpu_io_in_int),
        .cpu_io_status(cpu_io_status_int),

        // SPI Memory engine interface
        .spi_mem_req(spi_mem_req),
        .spi_mem_we(spi_mem_we),
        .spi_mem_addr(spi_mem_addr),
        .spi_mem_wdata(spi_mem_wdata),
        .spi_mem_rdata(spi_mem_rdata),
        .spi_mem_ready(spi_mem_ready),
        .spi_mem_cs_select(spi_mem_cs_select),
        .spi_mem_busy(spi_mem_busy_int),

        // SPI peripheral outputs
        .spi_periph_sclk(spi_periph_sclk),
        .spi_periph_mosi(spi_periph_mosi),
        .spi_periph_miso(spi_periph_miso),
        .spi_periph_cs_n(spi_periph_cs_n_int),
        .spi_periph_busy(spi_periph_busy_int),

        // PWM output
        .pwm_out(pwm_out_int),

        // Timer output
        .timer_out(timer_out_int),

        // External I/O
        .ext_in(ext_in),
        .ext_out(ext_out_int),

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
        .spi_busy(spi_mem_busy_int),

        // SPI Interface
        .spi_cs_ram_n(spi_mem_cs_ram_n),
        .spi_cs_flash_n(spi_mem_cs_flash_n),
        .spi_sclk(spi_mem_sclk),
        .spi_mosi(spi_mem_mosi),
        .spi_miso(spi_miso)
    );

    // ============================================================================
    // SPI SRAM Model (64KB for simulation)
    // ============================================================================
    spi_sram_model spi_ram (
        .spi_cs_n(spi_mem_cs_ram_n),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso)
    );

    // ============================================================================
    // Memory Load Interface (direct access to SPI SRAM for loading programs)
    // ============================================================================
    always_ff @(posedge clk) begin
        if (mem_load_en) begin
            spi_ram.memory[mem_load_addr] <= mem_load_data;
        end
    end

    // Memory read interface for verification
    assign mem_read_data = spi_ram.memory[mem_read_addr];

endmodule
