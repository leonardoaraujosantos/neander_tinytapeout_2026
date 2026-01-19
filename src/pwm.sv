// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2024 Leonardo Araujo Santos
//
// pwm.sv - PWM Generator for NEANDER-X Interconnect
//
// Features:
//   - 16-bit prescaler divider
//   - 16-bit period counter
//   - 16-bit duty cycle
//   - Enable and invert control
//   - Debug mode override (forces 10% duty @ 1 kHz)
//
// Registers (MMIO):
//   PWM_CTRL   (0xF010): [0] ENABLE, [1] INVERT
//   PWM_DIV    (0xF012): 16-bit divider (1-65535, 0 treated as 1)
//   PWM_PERIOD (0xF014): 16-bit period
//   PWM_DUTY   (0xF016): 16-bit duty cycle
//
// Frequency: f_pwm = clk_freq / (PWM_DIV * (PWM_PERIOD + 1))
// Output high when counter < PWM_DUTY (active high, unless inverted)

`default_nettype none

module pwm (
    input  wire        clk,
    input  wire        reset,

    // Debug mode override
    input  wire        boot_mode,      // When 1, force 10% duty

    // MMIO register interface
    input  wire [15:0] pwm_ctrl,       // Control register
    input  wire [15:0] pwm_div,        // Clock divider
    input  wire [15:0] pwm_period,     // Period value
    input  wire [15:0] pwm_duty,       // Duty cycle value

    // Output
    output wire        pwm_out
);

    // Control bits
    wire enable = pwm_ctrl[0];
    wire invert = pwm_ctrl[1];

    // Effective divider (minimum 1)
    wire [15:0] eff_div = (pwm_div == 16'h0000) ? 16'h0001 : pwm_div;

    // Debug mode fixed values (10% duty @ ~1 kHz with 10 MHz clock)
    // DIV=1, PERIOD=9999, DUTY=1000 -> 10 MHz / (1 * 10000) = 1 kHz, 10% duty
    localparam [15:0] DEBUG_DIV    = 16'd1;
    localparam [15:0] DEBUG_PERIOD = 16'd9999;
    localparam [15:0] DEBUG_DUTY   = 16'd1000;

    // Select between normal and debug mode values
    wire [15:0] active_div    = boot_mode ? DEBUG_DIV    : eff_div;
    wire [15:0] active_period = boot_mode ? DEBUG_PERIOD : pwm_period;
    wire [15:0] active_duty   = boot_mode ? DEBUG_DUTY   : pwm_duty;
    wire        active_enable = boot_mode ? 1'b1         : enable;

    // Prescaler counter
    reg [15:0] div_counter;
    wire       div_tick = (div_counter == active_div - 1);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            div_counter <= 16'h0000;
        end else if (active_enable) begin
            if (div_tick) begin
                div_counter <= 16'h0000;
            end else begin
                div_counter <= div_counter + 1;
            end
        end else begin
            div_counter <= 16'h0000;
        end
    end

    // PWM counter
    reg [15:0] pwm_counter;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pwm_counter <= 16'h0000;
        end else if (active_enable && div_tick) begin
            if (pwm_counter >= active_period) begin
                pwm_counter <= 16'h0000;
            end else begin
                pwm_counter <= pwm_counter + 1;
            end
        end else if (!active_enable) begin
            pwm_counter <= 16'h0000;
        end
    end

    // PWM output logic
    reg pwm_raw;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pwm_raw <= 1'b0;
        end else if (active_enable) begin
            pwm_raw <= (pwm_counter < active_duty);
        end else begin
            pwm_raw <= 1'b0;
        end
    end

    // Apply invert (only in normal mode, not debug mode)
    assign pwm_out = boot_mode ? pwm_raw : (pwm_raw ^ invert);

endmodule
