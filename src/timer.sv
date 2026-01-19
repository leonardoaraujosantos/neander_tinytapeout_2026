// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2024 Leonardo Araujo Santos
//
// timer.sv - Timer/Counter for NEANDER-X Interconnect
//
// Features:
//   - 16-bit prescaler divider
//   - 16-bit free-running or auto-reload counter
//   - 16-bit compare value
//   - Compare match flag (sticky, W1C)
//   - Optional timer output pin (toggle or pulse mode)
//   - IRQ on compare match
//
// Registers (MMIO):
//   TMR_CTRL   (0xF020): Control register
//     [0] ENABLE      - Timer counting enabled
//     [1] CLEAR       - Write 1 to reset counter (self-clearing)
//     [2] IRQ_EN      - Generate IRQ on compare match
//     [3] OUT_EN      - Enable TIMER_OUT pin
//     [4] OUT_MODE    - 0=toggle on match, 1=pulse on match
//     [5] AUTO_RELOAD - 1=reset counter on match, 0=free-running
//   TMR_DIV    (0xF022): 16-bit divider (1-65535, 0 treated as 1)
//   TMR_COUNT  (0xF024): 16-bit counter (read-only)
//   TMR_CMP    (0xF026): 16-bit compare value
//   TMR_STATUS (0xF028): Status register
//     [0] HIT     - Compare match occurred (write 1 to clear)
//     [1] RUNNING - Timer is currently counting (read-only)

`default_nettype none

module timer (
    input  wire        clk,
    input  wire        reset,

    // MMIO register interface
    input  wire [15:0] tmr_ctrl,       // Control register
    input  wire [15:0] tmr_div,        // Clock divider
    input  wire [15:0] tmr_cmp,        // Compare value

    // Status register interface (W1C)
    input  wire        status_ack,     // Write 1 to clear HIT flag
    output wire [15:0] tmr_status,     // Status register
    output wire [15:0] tmr_count,      // Counter value (read-only)

    // IRQ output
    output wire        tmr_irq,        // IRQ on compare match (if enabled)

    // Timer output pin
    output wire        timer_out
);

    // Control bits
    wire enable      = tmr_ctrl[0];
    wire clear       = tmr_ctrl[1];
    wire irq_en      = tmr_ctrl[2];
    wire out_en      = tmr_ctrl[3];
    wire out_mode    = tmr_ctrl[4];    // 0=toggle, 1=pulse
    wire auto_reload = tmr_ctrl[5];

    // Effective divider (minimum 1)
    wire [15:0] eff_div = (tmr_div == 16'h0000) ? 16'h0001 : tmr_div;

    // Prescaler counter
    reg [15:0] div_counter;
    wire       div_tick = (div_counter == eff_div - 1);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            div_counter <= 16'h0000;
        end else if (clear || !enable) begin
            div_counter <= 16'h0000;
        end else if (div_tick) begin
            div_counter <= 16'h0000;
        end else begin
            div_counter <= div_counter + 1;
        end
    end

    // Timer counter
    reg [15:0] counter;
    wire       compare_hit = (counter == tmr_cmp);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            counter <= 16'h0000;
        end else if (clear) begin
            counter <= 16'h0000;
        end else if (enable && div_tick) begin
            if (compare_hit && auto_reload) begin
                counter <= 16'h0000;
            end else begin
                counter <= counter + 1;
            end
        end
    end

    // Compare hit flag (sticky, W1C)
    reg hit_flag;
    wire compare_event = enable && div_tick && compare_hit;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            hit_flag <= 1'b0;
        end else if (status_ack) begin
            hit_flag <= 1'b0;
        end else if (compare_event) begin
            hit_flag <= 1'b1;
        end
    end

    // Timer output logic
    reg timer_out_reg;
    reg pulse_state;  // For pulse mode

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            timer_out_reg <= 1'b0;
            pulse_state <= 1'b0;
        end else if (!out_en) begin
            timer_out_reg <= 1'b0;
            pulse_state <= 1'b0;
        end else if (compare_event) begin
            if (out_mode) begin
                // Pulse mode: generate 1-cycle pulse
                pulse_state <= 1'b1;
                timer_out_reg <= 1'b1;
            end else begin
                // Toggle mode
                timer_out_reg <= ~timer_out_reg;
            end
        end else if (pulse_state) begin
            // End pulse after 1 cycle
            pulse_state <= 1'b0;
            timer_out_reg <= 1'b0;
        end
    end

    // Outputs
    assign tmr_count = counter;
    assign tmr_status = {14'b0, enable, hit_flag};  // [1]=RUNNING, [0]=HIT
    assign tmr_irq = hit_flag && irq_en;
    assign timer_out = out_en ? timer_out_reg : 1'b0;

endmodule
