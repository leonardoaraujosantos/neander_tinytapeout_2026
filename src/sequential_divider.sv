// ============================================================================
// sequential_divider.sv — Sequential 8-bit Divider for NEANDER-X CPU
// ============================================================================
// Implements restoring division algorithm for 8-bit unsigned division.
// Takes 8 clock cycles to complete a division operation.
//
// Operation:
//   dividend / divisor = quotient, remainder
//
// Interface:
//   - start: Pulse high for 1 cycle to begin division
//   - busy: High while division is in progress
//   - done: Pulses high for 1 cycle when result is ready
//   - div_by_zero: High if divisor is 0 (sets error flag)
//
// Area savings vs combinational: ~300-400 gates saved
// Trade-off: 8 cycles vs 1 cycle execution time
// ============================================================================

module sequential_divider (
    input  logic       clk,
    input  logic       reset,
    input  logic       start,        // Start division (pulse)
    input  logic [7:0] dividend,     // Dividend (numerator)
    input  logic [7:0] divisor,      // Divisor (denominator)
    output logic [7:0] quotient,     // Result: dividend / divisor
    output logic [7:0] remainder,    // Result: dividend % divisor
    output logic       busy,         // Division in progress
    output logic       done,         // Result ready (pulse)
    output logic       div_by_zero   // Division by zero error
);

    // Internal registers for restoring division
    logic [7:0] A;           // Accumulator (becomes remainder)
    logic [7:0] Q;           // Quotient register
    logic [7:0] M;           // Divisor register
    logic [3:0] count;       // Iteration counter (0-7 for 8 bits)
    logic [8:0] diff;        // 9-bit for subtraction with borrow detection

    // State machine
    typedef enum logic [1:0] {
        IDLE    = 2'b00,
        DIVIDE  = 2'b01,
        FINISH  = 2'b10
    } state_t;

    state_t state, next_state;

    // Division by zero detection
    assign div_by_zero = (state == FINISH) && (M == 8'h00);

    // Output assignments
    assign quotient = Q;
    assign remainder = A;
    assign busy = (state == DIVIDE);
    assign done = (state == FINISH);

    // State register
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            state <= IDLE;
        else
            state <= next_state;
    end

    // Next state logic
    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (start)
                    next_state = DIVIDE;
            end
            DIVIDE: begin
                if (count == 4'd7)
                    next_state = FINISH;
            end
            FINISH: begin
                next_state = IDLE;
            end
            default: next_state = IDLE;
        endcase
    end

    // Restoring division datapath
    // Algorithm: For each bit position (MSB to LSB):
    //   1. Shift A:Q left by 1 (MSB of Q goes into LSB of A)
    //   2. Subtract M from A
    //   3. If result >= 0: keep result, set Q[0] = 1
    //      If result < 0: restore A, set Q[0] = 0

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            A <= 8'h00;
            Q <= 8'h00;
            M <= 8'h00;
            count <= 4'h0;
        end else begin
            case (state)
                IDLE: begin
                    if (start) begin
                        // Initialize registers
                        A <= 8'h00;
                        Q <= dividend;
                        M <= divisor;
                        count <= 4'h0;
                    end
                end

                DIVIDE: begin
                    // Restoring division step
                    // Step 1: Shift left A:Q (A gets MSB of Q, Q shifts left)
                    // Step 2: Try A = A - M
                    // Step 3: If A >= 0 (no borrow), keep and set Q[0]=1
                    //         If A < 0 (borrow), restore A and set Q[0]=0

                    // Calculate: shift left then subtract
                    diff = {A[6:0], Q[7]} - {1'b0, M};

                    if (diff[8] == 1'b0) begin
                        // No borrow: A >= M, keep subtraction result
                        A <= diff[7:0];
                        Q <= {Q[6:0], 1'b1};  // Shift Q left, set LSB = 1
                    end else begin
                        // Borrow: A < M, restore (don't use subtraction result)
                        A <= {A[6:0], Q[7]};  // Just shift, keep original value
                        Q <= {Q[6:0], 1'b0};  // Shift Q left, set LSB = 0
                    end

                    count <= count + 4'h1;
                end

                FINISH: begin
                    // Handle division by zero
                    if (M == 8'h00) begin
                        Q <= 8'hFF;       // Return max value for quotient
                        A <= dividend;    // Preserve dividend as remainder
                    end
                    // Otherwise Q and A already contain correct results
                end

                default: begin
                    A <= 8'h00;
                    Q <= 8'h00;
                    M <= 8'h00;
                    count <= 4'h0;
                end
            endcase
        end
    end

endmodule
