// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2024 Leonardo Araujo Santos
//
// irq_ctrl.sv - IRQ Controller for NEANDER-X Interconnect
//
// Features:
//   - 3 interrupt sources: External, Timer, SPI
//   - Status register (sticky, set on event)
//   - Enable mask register
//   - Acknowledge register (W1C to clear status)
//   - IRQ_PENDING output for CPU polling
//
// Registers (MMIO):
//   IRQ_STATUS (0xF000): R   - Interrupt status (sticky)
//     [0] EXT_IRQ  - External IRQ_IN pin asserted
//     [1] TMR_IRQ  - Timer compare match occurred
//     [2] SPI_IRQ  - SPI peripheral transfer complete
//   IRQ_ENABLE (0xF002): R/W - Interrupt enable mask
//     [0] EXT_EN   - Enable external IRQ
//     [1] TMR_EN   - Enable timer IRQ
//     [2] SPI_EN   - Enable SPI complete IRQ
//   IRQ_ACK    (0xF004): W1C - Interrupt acknowledge
//     Write 1 to clear corresponding status bit

`default_nettype none

module irq_ctrl (
    input  wire        clk,
    input  wire        reset,

    // External IRQ sources
    input  wire        irq_in,         // External IRQ pin
    input  wire        tmr_irq,        // Timer IRQ
    input  wire        spi_irq,        // SPI IRQ

    // MMIO register interface
    input  wire [15:0] irq_enable_in,  // Enable mask (written by CPU)
    input  wire [15:0] irq_ack_in,     // Acknowledge bits (W1C)
    input  wire        irq_ack_write,  // Write strobe for ACK register

    // Status outputs
    output wire [15:0] irq_status,     // Status register (read by CPU)
    output wire [15:0] irq_enable,     // Enable register (read by CPU)
    output wire        irq_pending     // Any enabled IRQ is pending
);

    // Enable register
    reg [2:0] enable_reg;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            enable_reg <= 3'b000;
        end else begin
            enable_reg <= irq_enable_in[2:0];
        end
    end

    // Status bits (sticky, set on rising edge of IRQ source)
    reg [2:0] status_reg;

    // Edge detection for external IRQ
    reg irq_in_prev;
    wire irq_in_edge = irq_in && !irq_in_prev;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            irq_in_prev <= 1'b0;
        end else begin
            irq_in_prev <= irq_in;
        end
    end

    // External IRQ status
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            status_reg[0] <= 1'b0;
        end else if (irq_ack_write && irq_ack_in[0]) begin
            status_reg[0] <= 1'b0;  // W1C
        end else if (irq_in_edge) begin
            status_reg[0] <= 1'b1;  // Set on rising edge
        end
    end

    // Timer IRQ status (level-triggered from timer module)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            status_reg[1] <= 1'b0;
        end else if (irq_ack_write && irq_ack_in[1]) begin
            status_reg[1] <= 1'b0;  // W1C
        end else if (tmr_irq) begin
            status_reg[1] <= 1'b1;  // Set when timer IRQ is high
        end
    end

    // SPI IRQ status (level-triggered from SPI module)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            status_reg[2] <= 1'b0;
        end else if (irq_ack_write && irq_ack_in[2]) begin
            status_reg[2] <= 1'b0;  // W1C
        end else if (spi_irq) begin
            status_reg[2] <= 1'b1;  // Set when SPI IRQ is high
        end
    end

    // Outputs
    assign irq_status = {13'b0, status_reg};
    assign irq_enable = {13'b0, enable_reg};
    assign irq_pending = |(status_reg & enable_reg);  // Any enabled IRQ pending

endmodule
