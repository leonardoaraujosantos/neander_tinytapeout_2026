// ============================================================================
// sequential_multiplier.sv — Sequential 16-bit Multiplier for NEANDER-X CPU
// ============================================================================
// Implements shift-and-add multiplication algorithm for 16-bit unsigned values.
// Takes 16 clock cycles to complete a multiplication operation.
//
// Operation:
//   multiplicand * multiplier = product (32-bit result)
//
// Interface:
//   - start: Pulse high for 1 cycle to begin multiplication
//   - busy: High while multiplication is in progress
//   - done: Pulses high for 1 cycle when result is ready
//
// Algorithm: Shift-and-add
//   For each bit of the multiplier (LSB to MSB):
//     1. If current multiplier bit is 1, add multiplicand to accumulator
//     2. Shift multiplicand left by 1
//     3. Shift multiplier right by 1
//
// Area savings vs combinational: ~400-600 gates saved
// Trade-off: 16 cycles vs 1 cycle execution time
// ============================================================================

module sequential_multiplier (
    input  logic        clk,
    input  logic        reset,
    input  logic        start,           // Start multiplication (pulse)
    input  logic [15:0] multiplicand,    // First operand (A)
    input  logic [15:0] multiplier,      // Second operand (B)
    output logic [15:0] product_low,     // Result low word (bits 15:0)
    output logic [15:0] product_high,    // Result high word (bits 31:16)
    output logic        busy,            // Multiplication in progress
    output logic        done             // Result ready (pulse)
);

    // Internal registers for shift-and-add multiplication
    logic [31:0] P;          // Product accumulator (32-bit)
    logic [31:0] A;          // Shifted multiplicand (extended to 32-bit)
    logic [15:0] B;          // Multiplier (shifted right each cycle)
    logic [4:0]  count;      // Iteration counter (0-15 for 16 bits)

    // State machine
    typedef enum logic [1:0] {
        IDLE     = 2'b00,
        MULTIPLY = 2'b01,
        FINISH   = 2'b10
    } state_t;

    state_t state, next_state;

    // Output assignments
    assign product_low = P[15:0];
    assign product_high = P[31:16];
    assign busy = (state == MULTIPLY);
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
                    next_state = MULTIPLY;
            end
            MULTIPLY: begin
                if (count == 5'd15)
                    next_state = FINISH;
            end
            FINISH: begin
                next_state = IDLE;
            end
            default: next_state = IDLE;
        endcase
    end

    // Shift-and-add multiplication datapath
    // Algorithm:
    //   P = 0
    //   A = multiplicand (zero-extended to 32 bits)
    //   B = multiplier
    //   For i = 0 to 15:
    //     if B[0] == 1: P = P + A
    //     A = A << 1
    //     B = B >> 1

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            P <= 32'h00000000;
            A <= 32'h00000000;
            B <= 16'h0000;
            count <= 5'h0;
        end else begin
            case (state)
                IDLE: begin
                    if (start) begin
                        // Initialize registers
                        P <= 32'h00000000;
                        A <= {16'h0000, multiplicand};  // Zero-extend to 32 bits
                        B <= multiplier;
                        count <= 5'h0;
                    end
                end

                MULTIPLY: begin
                    // Shift-and-add step
                    // If LSB of B is 1, add A to P
                    if (B[0])
                        P <= P + A;

                    // Shift A left (grows into upper bits)
                    A <= A << 1;

                    // Shift B right (consume next bit)
                    B <= B >> 1;

                    count <= count + 5'h1;
                end

                FINISH: begin
                    // Result is ready in P
                    // P[15:0] = low word, P[31:16] = high word
                end

                default: begin
                    P <= 32'h00000000;
                    A <= 32'h00000000;
                    B <= 16'h0000;
                    count <= 5'h0;
                end
            endcase
        end
    end

endmodule
