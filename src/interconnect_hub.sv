// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2024 Leonardo Araujo Santos
//
// interconnect_hub.sv - NEANDER-X Interconnect / MMIO Hub for TinyTapeout
//
// This module provides:
//   1. Address decoding for RAM (0x0000-0xDFFF), Flash (0xE000-0xEFFF), MMIO (0xF000-0xF0FF)
//   2. Debug ROM shadowing in boot mode (BOOT_SEL=1)
//   3. MMIO registers for PWM, Timer, IRQ, SPI peripheral
//   4. SPI arbitration between memory and peripheral engines
//   5. External I/O management
//
// Address Map:
//   0x0000-0xDFFF : External RAM (SPI SRAM)
//   0xE000-0xEFFF : External Flash (SPI Flash, typically read-only)
//   0xF000-0xF0FF : MMIO registers
//
// In boot_mode=1, reads from 0x0000-0x00FF come from internal debug ROM.

`default_nettype none

module interconnect_hub (
    input  wire        clk,
    input  wire        reset,

    // Boot mode selection
    input  wire        boot_sel,      // Sampled on reset release

    // CPU memory interface
    input  wire [15:0] cpu_mem_addr,
    input  wire [15:0] cpu_mem_data_out,
    output wire [15:0] cpu_mem_data_in,
    input  wire        cpu_mem_write,
    input  wire        cpu_mem_read,
    input  wire        cpu_mem_req,
    output wire        cpu_mem_ready,

    // CPU I/O interface
    input  wire [7:0]  cpu_io_out,
    input  wire        cpu_io_write,
    output wire [7:0]  cpu_io_in,
    output wire [7:0]  cpu_io_status,

    // SPI Memory engine interface
    output wire        spi_mem_req,
    output wire        spi_mem_we,
    output wire [15:0] spi_mem_addr,
    output wire [15:0] spi_mem_wdata,
    input  wire [15:0] spi_mem_rdata,
    input  wire        spi_mem_ready,
    output wire        spi_mem_cs_select,  // 0=RAM, 1=Flash
    input  wire        spi_mem_busy,

    // SPI peripheral outputs (directly active CS and bus signals from spi_periph)
    output wire        spi_periph_sclk,
    output wire        spi_periph_mosi,
    input  wire        spi_periph_miso,
    output wire [5:0]  spi_periph_cs_n,
    output wire        spi_periph_busy,

    // PWM output
    output wire        pwm_out,

    // External I/O pins
    input  wire [1:0]  ext_in,        // EXT_IN0, EXT_IN1
    output wire [1:0]  ext_out,       // EXT_OUT0, EXT_OUT1

    // IRQ input
    input  wire        irq_in
);

    // ========================================================================
    // Boot mode latch
    // ========================================================================
    reg boot_mode;
    reg boot_mode_latched;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            boot_mode <= 1'b0;
            boot_mode_latched <= 1'b0;
        end else if (!boot_mode_latched) begin
            boot_mode <= boot_sel;
            boot_mode_latched <= 1'b1;
        end
    end

    // ========================================================================
    // Address decoding
    // ========================================================================
    // RAM:   0x0000-0xDFFF (57344 bytes)
    // Flash: 0xE000-0xEFFF (4096 bytes)
    // MMIO:  0xF000-0xF0FF (256 bytes, only some addresses used)
    // Reserved: 0xF100-0xFFFF

    wire is_ram_access   = (cpu_mem_addr[15:13] != 3'b111);                    // 0x0000-0xDFFF
    wire is_flash_access = (cpu_mem_addr[15:12] == 4'hE);                      // 0xE000-0xEFFF
    wire is_mmio_access  = (cpu_mem_addr[15:8] == 8'hF0);                      // 0xF000-0xF0FF

    // Debug ROM shadowing: In boot mode, reads from 0x0000-0x00FF come from ROM
    wire is_rom_access   = boot_mode && cpu_mem_read && (cpu_mem_addr[15:8] == 8'h00);

    // ========================================================================
    // Debug ROM
    // ========================================================================
    wire [7:0] rom_data;

    debug_rom u_debug_rom (
        .addr(cpu_mem_addr[7:0]),
        .data(rom_data)
    );

    // ========================================================================
    // MMIO Registers
    // ========================================================================
    // Address offsets within MMIO space (0xF0xx)
    localparam ADDR_IRQ_STATUS  = 8'h00;  // 0xF000
    localparam ADDR_IRQ_ENABLE  = 8'h02;  // 0xF002
    localparam ADDR_IRQ_ACK     = 8'h04;  // 0xF004
    localparam ADDR_PWM_CTRL    = 8'h10;  // 0xF010
    localparam ADDR_PWM_DIV     = 8'h12;  // 0xF012
    localparam ADDR_PWM_PERIOD  = 8'h14;  // 0xF014
    localparam ADDR_PWM_DUTY    = 8'h16;  // 0xF016
    localparam ADDR_SPI_CTRL    = 8'h30;  // 0xF030
    localparam ADDR_SPI_DIV     = 8'h32;  // 0xF032
    localparam ADDR_SPI_SS      = 8'h34;  // 0xF034
    localparam ADDR_SPI_TXRX    = 8'h36;  // 0xF036
    localparam ADDR_SPI_STATUS  = 8'h38;  // 0xF038

    wire [7:0] mmio_addr = cpu_mem_addr[7:0];

    // MMIO register storage
    reg [15:0] reg_irq_enable;
    reg [15:0] reg_pwm_ctrl;
    reg [15:0] reg_pwm_div;
    reg [15:0] reg_pwm_period;
    reg [15:0] reg_pwm_duty;
    reg [15:0] reg_spi_ctrl;
    reg [15:0] reg_spi_div;
    reg [15:0] reg_spi_ss;

    // MMIO write logic
    wire mmio_write = is_mmio_access && cpu_mem_write && cpu_mem_req;
    wire mmio_read  = is_mmio_access && cpu_mem_read && cpu_mem_req;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            reg_irq_enable <= 16'h0000;
            reg_pwm_ctrl   <= 16'h0000;
            reg_pwm_div    <= 16'h0001;
            reg_pwm_period <= 16'hFFFF;
            reg_pwm_duty   <= 16'h0000;
            reg_spi_ctrl   <= 16'h0020;  // AUTO_CS=1 by default
            reg_spi_div    <= 16'h0001;
            reg_spi_ss     <= 16'h0000;
        end else if (mmio_write) begin
            case (mmio_addr)
                ADDR_IRQ_ENABLE:  reg_irq_enable <= cpu_mem_data_out;
                ADDR_PWM_CTRL:    reg_pwm_ctrl   <= cpu_mem_data_out;
                ADDR_PWM_DIV:     reg_pwm_div    <= cpu_mem_data_out;
                ADDR_PWM_PERIOD:  reg_pwm_period <= cpu_mem_data_out;
                ADDR_PWM_DUTY:    reg_pwm_duty   <= cpu_mem_data_out;
                ADDR_SPI_CTRL:    reg_spi_ctrl   <= cpu_mem_data_out;
                ADDR_SPI_DIV:     reg_spi_div    <= cpu_mem_data_out;
                ADDR_SPI_SS:      reg_spi_ss     <= cpu_mem_data_out;
                default: ;  // Ignore writes to other addresses
            endcase
        end
    end

    // ========================================================================
    // IRQ Controller
    // ========================================================================
    wire [15:0] irq_status;
    wire [15:0] irq_enable_out;
    wire        irq_pending;
    wire        spi_done_irq;
    wire        irq_ack_write = mmio_write && (mmio_addr == ADDR_IRQ_ACK);

    irq_ctrl u_irq_ctrl (
        .clk(clk),
        .reset(reset),
        .irq_in(irq_in),
        .tmr_irq(1'b0),  // Timer removed
        .spi_irq(spi_done_irq),
        .irq_enable_in(reg_irq_enable),
        .irq_ack_in(cpu_mem_data_out),
        .irq_ack_write(irq_ack_write),
        .irq_status(irq_status),
        .irq_enable(irq_enable_out),
        .irq_pending(irq_pending)
    );

    // ========================================================================
    // PWM
    // ========================================================================
    pwm u_pwm (
        .clk(clk),
        .reset(reset),
        .boot_mode(boot_mode),
        .pwm_ctrl(reg_pwm_ctrl),
        .pwm_div(reg_pwm_div),
        .pwm_period(reg_pwm_period),
        .pwm_duty(reg_pwm_duty),
        .pwm_out(pwm_out)
    );

    // ========================================================================
    // SPI Peripheral Engine
    // ========================================================================
    wire [15:0] spi_status;
    wire [7:0]  spi_rx_data;
    wire        spi_tx_start = mmio_write && (mmio_addr == ADDR_SPI_TXRX);
    wire        spi_rx_read  = mmio_read && (mmio_addr == ADDR_SPI_TXRX);

    spi_periph u_spi_periph (
        .clk(clk),
        .reset(reset),
        .spi_mem_busy(spi_mem_busy),
        .spi_periph_busy(spi_periph_busy),
        .spi_ctrl(reg_spi_ctrl),
        .spi_div(reg_spi_div),
        .spi_ss(reg_spi_ss),
        .spi_tx_data(cpu_mem_data_out[7:0]),
        .spi_tx_start(spi_tx_start),
        .spi_rx_read(spi_rx_read),
        .spi_status(spi_status),
        .spi_rx_data(spi_rx_data),
        .spi_done_irq(spi_done_irq),
        .spi_sclk(spi_periph_sclk),
        .spi_mosi(spi_periph_mosi),
        .spi_miso(spi_periph_miso),
        .spi_cs_n(spi_periph_cs_n)
    );

    // ========================================================================
    // MMIO Read Mux
    // ========================================================================
    reg [15:0] mmio_rdata;

    always @(*) begin
        case (mmio_addr)
            ADDR_IRQ_STATUS:  mmio_rdata = irq_status;
            ADDR_IRQ_ENABLE:  mmio_rdata = irq_enable_out;
            ADDR_IRQ_ACK:     mmio_rdata = 16'h0000;  // Write-only
            ADDR_PWM_CTRL:    mmio_rdata = reg_pwm_ctrl;
            ADDR_PWM_DIV:     mmio_rdata = reg_pwm_div;
            ADDR_PWM_PERIOD:  mmio_rdata = reg_pwm_period;
            ADDR_PWM_DUTY:    mmio_rdata = reg_pwm_duty;
            ADDR_SPI_CTRL:    mmio_rdata = reg_spi_ctrl;
            ADDR_SPI_DIV:     mmio_rdata = reg_spi_div;
            ADDR_SPI_SS:      mmio_rdata = reg_spi_ss;
            ADDR_SPI_TXRX:    mmio_rdata = {8'h00, spi_rx_data};
            ADDR_SPI_STATUS:  mmio_rdata = spi_status;
            default:          mmio_rdata = 16'h0000;
        endcase
    end

    // ========================================================================
    // Memory request routing
    // ========================================================================
    // SPI memory access for RAM or Flash regions (but not ROM-shadowed or MMIO)
    wire spi_access = (is_ram_access || is_flash_access) && !is_rom_access;

    assign spi_mem_req = cpu_mem_req && spi_access;
    assign spi_mem_we = cpu_mem_write;
    assign spi_mem_addr = cpu_mem_addr;
    assign spi_mem_wdata = cpu_mem_data_out;
    assign spi_mem_cs_select = is_flash_access;  // 0=RAM, 1=Flash

    // ========================================================================
    // Read data mux
    // ========================================================================
    reg [15:0] mem_data_mux;

    always @(*) begin
        if (is_rom_access) begin
            mem_data_mux = {8'h00, rom_data};  // ROM is 8-bit, zero-extend
        end else if (is_mmio_access) begin
            mem_data_mux = mmio_rdata;
        end else begin
            mem_data_mux = spi_mem_rdata;  // SPI RAM or Flash
        end
    end

    assign cpu_mem_data_in = mem_data_mux;

    // ========================================================================
    // Ready signal generation
    // ========================================================================
    // MMIO and ROM: 1-cycle access
    // SPI: wait for spi_mem_ready

    reg mmio_rom_ready;
    reg mmio_rom_req_prev;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mmio_rom_ready <= 1'b0;
            mmio_rom_req_prev <= 1'b0;
        end else begin
            mmio_rom_req_prev <= cpu_mem_req && (is_mmio_access || is_rom_access);
            // Generate 1-cycle pulse for MMIO/ROM access
            mmio_rom_ready <= cpu_mem_req && (is_mmio_access || is_rom_access) && !mmio_rom_req_prev;
        end
    end

    assign cpu_mem_ready = (is_mmio_access || is_rom_access) ? mmio_rom_ready : spi_mem_ready;

    // ========================================================================
    // External I/O
    // ========================================================================
    // EXT_OUT latches on io_write
    reg [1:0] ext_out_reg;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ext_out_reg <= 2'b00;
        end else if (cpu_io_write) begin
            ext_out_reg <= cpu_io_out[1:0];
        end
    end

    assign ext_out = ext_out_reg;

    // EXT_IN directly mapped
    assign cpu_io_in = {6'b000000, ext_in};

    // I/O status bits
    // [0] = IRQ_PENDING
    // [1] = SPI_MEM_BUSY
    // [2] = SPI_PERIPH_BUSY
    assign cpu_io_status = {5'b00000, spi_periph_busy, spi_mem_busy, irq_pending};

endmodule
