// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2024 Leonardo Araujo Santos
//
// spi_periph.sv - Software-Controlled SPI Engine for NEANDER-X Interconnect
//
// Features:
//   - 8-bit SPI transfers (byte-by-byte)
//   - Configurable clock divider
//   - Configurable polarity (CPOL) and phase (CPHA)
//   - MSB-first or LSB-first data order
//   - 6 chip-select lines (ADC, DAC, UART, ETH, GPIO, FLASH)
//   - Auto-CS or manual CS control
//
// Registers (MMIO):
//   SPI_CTRL   (0xF030): Control register
//     [0] ENABLE         - SPI enabled
//     [1] CPOL           - Clock polarity (0=idle low, 1=idle high)
//     [2] CPHA           - Clock phase (0=sample 1st edge, 1=sample 2nd edge)
//     [3] LSB_FIRST      - Bit order (0=MSB first, 1=LSB first)
//     [4] CS_ACTIVE_HIGH - CS polarity (0=active low, 1=active high)
//     [5] AUTO_CS        - Auto-assert CS during transfer
//   SPI_DIV    (0xF032): Clock divider (f_spi = clk / (2 * DIV))
//   SPI_SS     (0xF034): Slave select
//     [2:0] SELECT       - CS line selection (0-5)
//     [3]   CS_MANUAL    - Manually assert selected CS
//   SPI_TXRX   (0xF036): Data register
//     [7:0] DATA         - TX: data to send, RX: last received
//   SPI_STATUS (0xF038): Status register
//     [0] BUSY           - Transfer in progress
//     [1] DONE           - Transfer complete (cleared on TXRX read)
//     [2] RX_VALID       - Valid data in RX buffer
//
// CS Line Encoding:
//   0 = CS_ADC   (uo_out[7])
//   1 = CS_DAC   (uo_out[6])
//   2 = CS_UART  (uo_out[4])
//   3 = CS_ETH   (uo_out[5])
//   4 = CS_GPIO  (uio[4])
//   5 = CS_FLASH (uio[3])

`default_nettype none

module spi_periph (
    input  wire        clk,
    input  wire        reset,

    // Arbitration interface
    input  wire        spi_mem_busy,   // SPI_MEM engine is busy
    output wire        spi_periph_busy, // This engine is busy

    // MMIO register interface
    input  wire [15:0] spi_ctrl,       // Control register
    input  wire [15:0] spi_div,        // Clock divider
    input  wire [15:0] spi_ss,         // Slave select
    input  wire [7:0]  spi_tx_data,    // TX data
    input  wire        spi_tx_start,   // Start transfer (write to TXRX)
    input  wire        spi_rx_read,    // Read RX data (read from TXRX)

    // Status outputs
    output wire [15:0] spi_status,     // Status register
    output wire [7:0]  spi_rx_data,    // Received data

    // IRQ output
    output wire        spi_done_irq,   // Transfer complete IRQ

    // SPI pins (directly active chip select is handled externally)
    output wire        spi_sclk,       // SPI clock
    output wire        spi_mosi,       // Master out
    input  wire        spi_miso,       // Master in
    output wire [5:0]  spi_cs_n        // Chip selects (directly active)
);

    // Control bits
    wire enable         = spi_ctrl[0];
    wire cpol           = spi_ctrl[1];
    wire cpha           = spi_ctrl[2];
    wire lsb_first      = spi_ctrl[3];
    wire cs_active_high = spi_ctrl[4];
    wire auto_cs        = spi_ctrl[5];

    // Slave select bits
    wire [2:0] cs_select = spi_ss[2:0];
    wire       cs_manual = spi_ss[3];

    // Effective divider (minimum 1)
    wire [15:0] eff_div = (spi_div == 16'h0000) ? 16'h0001 : spi_div;

    // FSM states
    localparam IDLE      = 3'd0;
    localparam START     = 3'd1;
    localparam SHIFT     = 3'd2;
    localparam DONE      = 3'd3;
    localparam WAIT_ARB  = 3'd4;  // Waiting for arbitration

    reg [2:0] state;
    reg [7:0] shift_reg;
    reg [7:0] rx_buffer;
    reg [3:0] bit_count;      // 0-7 for 8 bits
    reg [15:0] div_counter;
    reg       sclk_reg;
    reg       busy_flag;
    reg       done_flag;
    reg       rx_valid_flag;
    reg       cs_active;      // CS is asserted during transfer

    // Clock phase tracking
    reg       phase;          // 0=first half of bit, 1=second half

    // Blocked by arbitration
    wire blocked = spi_mem_busy;

    // State machine
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            shift_reg <= 8'h00;
            rx_buffer <= 8'h00;
            bit_count <= 4'd0;
            div_counter <= 16'h0000;
            sclk_reg <= 1'b0;
            busy_flag <= 1'b0;
            done_flag <= 1'b0;
            rx_valid_flag <= 1'b0;
            cs_active <= 1'b0;
            phase <= 1'b0;
        end else begin
            // Clear done flag on RX read
            if (spi_rx_read) begin
                done_flag <= 1'b0;
            end

            case (state)
                IDLE: begin
                    sclk_reg <= cpol;  // Idle clock level
                    phase <= 1'b0;
                    if (spi_tx_start && enable) begin
                        if (blocked) begin
                            state <= WAIT_ARB;
                            shift_reg <= lsb_first ?
                                {spi_tx_data[0], spi_tx_data[1], spi_tx_data[2], spi_tx_data[3],
                                 spi_tx_data[4], spi_tx_data[5], spi_tx_data[6], spi_tx_data[7]} :
                                spi_tx_data;
                        end else begin
                            shift_reg <= lsb_first ?
                                {spi_tx_data[0], spi_tx_data[1], spi_tx_data[2], spi_tx_data[3],
                                 spi_tx_data[4], spi_tx_data[5], spi_tx_data[6], spi_tx_data[7]} :
                                spi_tx_data;
                            busy_flag <= 1'b1;
                            done_flag <= 1'b0;
                            rx_valid_flag <= 1'b0;
                            bit_count <= 4'd0;
                            div_counter <= 16'h0000;
                            cs_active <= auto_cs;
                            state <= START;
                        end
                    end
                end

                WAIT_ARB: begin
                    // Wait for SPI_MEM to finish
                    if (!blocked) begin
                        busy_flag <= 1'b1;
                        done_flag <= 1'b0;
                        rx_valid_flag <= 1'b0;
                        bit_count <= 4'd0;
                        div_counter <= 16'h0000;
                        cs_active <= auto_cs;
                        state <= START;
                    end
                end

                START: begin
                    // Small setup time before first clock edge
                    if (div_counter == eff_div - 1) begin
                        div_counter <= 16'h0000;
                        state <= SHIFT;
                        phase <= 1'b0;
                    end else begin
                        div_counter <= div_counter + 1;
                    end
                end

                SHIFT: begin
                    if (div_counter == eff_div - 1) begin
                        div_counter <= 16'h0000;

                        if (phase == 1'b0) begin
                            // First half of bit period - first clock edge
                            sclk_reg <= ~sclk_reg;
                            if (cpha == 1'b0) begin
                                // CPHA=0: Sample on first edge
                                rx_buffer <= {rx_buffer[6:0], spi_miso};
                            end
                            phase <= 1'b1;
                        end else begin
                            // Second half of bit period - second clock edge
                            sclk_reg <= ~sclk_reg;
                            if (cpha == 1'b1) begin
                                // CPHA=1: Sample on second edge
                                rx_buffer <= {rx_buffer[6:0], spi_miso};
                            end
                            // Shift out next bit
                            shift_reg <= {shift_reg[6:0], 1'b0};
                            phase <= 1'b0;

                            if (bit_count == 4'd7) begin
                                state <= DONE;
                            end else begin
                                bit_count <= bit_count + 1;
                            end
                        end
                    end else begin
                        div_counter <= div_counter + 1;
                    end
                end

                DONE: begin
                    sclk_reg <= cpol;  // Return to idle level
                    cs_active <= 1'b0;
                    busy_flag <= 1'b0;
                    done_flag <= 1'b1;
                    rx_valid_flag <= 1'b1;
                    state <= IDLE;
                end

                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

    // Chip select decode and polarity
    reg [5:0] cs_decode;
    always @(*) begin
        cs_decode = 6'b111111;  // All deasserted (active low)
        if ((cs_active || cs_manual) && enable) begin
            case (cs_select)
                3'd0: cs_decode[0] = 1'b0;  // CS_ADC
                3'd1: cs_decode[1] = 1'b0;  // CS_DAC
                3'd2: cs_decode[2] = 1'b0;  // CS_UART
                3'd3: cs_decode[3] = 1'b0;  // CS_ETH
                3'd4: cs_decode[4] = 1'b0;  // CS_GPIO
                3'd5: cs_decode[5] = 1'b0;  // CS_FLASH
                default: cs_decode = 6'b111111;
            endcase
        end
    end

    // Apply CS polarity
    assign spi_cs_n = cs_active_high ? ~cs_decode : cs_decode;

    // Outputs
    assign spi_sclk = sclk_reg;
    assign spi_mosi = shift_reg[7];  // MSB out (already reversed if LSB_FIRST)
    assign spi_rx_data = lsb_first ?
        {rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3],
         rx_buffer[4], rx_buffer[5], rx_buffer[6], rx_buffer[7]} :
        rx_buffer;
    assign spi_status = {13'b0, rx_valid_flag, done_flag, busy_flag};
    assign spi_periph_busy = busy_flag || (state == WAIT_ARB);
    assign spi_done_irq = done_flag;

endmodule
