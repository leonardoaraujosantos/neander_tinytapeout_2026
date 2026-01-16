// ============================================================================
// neander_x_alu.sv — ALU for NEANDER-X CPU (16-bit Data Width)
// ============================================================================
// Extended ALU operations (4-bit opcode):
//   0000: ADD  - a + b           (sets carry on overflow)
//   0001: SUB  - a - b           (sets carry on borrow)
//   0010: AND  - a & b
//   0011: OR   - a | b
//   0100: XOR  - a ^ b
//   0101: NOT  - ~a
//   0110: SHL  - a << 1          (carry = MSB shifted out)
//   0111: SHR  - a >> 1          (carry = LSB shifted out, logical)
//   1000: NEG  - 0 - a = -a      (two's complement negation)
//   1001: MUL  - a * b           (32-bit result: mul_high:result)
//                                 ** Uses sequential multiplier (16 cycles) **
//   1010: DIV  - a / b           (quotient in result, remainder in mul_high)
//                                 ** Uses sequential divider (16 cycles) **
//   1011: MOD  - a % b           (remainder in result, quotient in mul_high)
//                                 ** Uses sequential divider (16 cycles) **
//   1100: ADC  - a + b + carry   (add with carry for multi-byte arithmetic)
//   1101: SBC  - a - b - carry   (subtract with borrow for multi-byte arithmetic)
//   1110: ASR  - a >> 1 (arith)  (arithmetic shift right, preserves sign bit)
// ============================================================================

module neander_alu (
    input  logic [15:0] a,
    input  logic [15:0] b,
    input  logic [3:0]  alu_op,  // Extended to 4 bits for NEG and future ops
    input  logic        carry_in, // Carry input for ADC/SBC operations
    // Sequential multiplier interface (results from external sequential_multiplier module)
    input  logic [15:0] mul_product_low,  // Low word from sequential multiplier
    input  logic [15:0] mul_product_high, // High word from sequential multiplier
    // Sequential divider interface (results from external sequential_divider module)
    input  logic [15:0] div_quotient,   // Quotient from sequential divider
    input  logic [15:0] div_remainder,  // Remainder from sequential divider
    input  logic        div_by_zero,    // Division by zero flag from sequential divider
    output logic [15:0] result,
    output logic [15:0] mul_high,  // High word of multiplication result (or div remainder/quotient)
    output logic        carry_out  // Carry/borrow flag output
);
    logic [16:0] temp;  // 17-bit for carry detection

    always_comb begin
        temp = 17'b0;
        carry_out = 1'b0;
        result = 16'h0000;
        mul_high = 16'h0000;

        case (alu_op)
            4'b0000: begin  // ADD
                temp = {1'b0, a} + {1'b0, b};
                result = temp[15:0];
                carry_out = temp[16];
            end
            4'b0001: begin  // SUB (a - b)
                temp = {1'b0, a} - {1'b0, b};
                result = temp[15:0];
                carry_out = temp[16];  // Borrow flag (1 if a < b)
            end
            4'b0010: begin  // AND
                result = a & b;
            end
            4'b0011: begin  // OR
                result = a | b;
            end
            4'b0100: begin  // XOR
                result = a ^ b;
            end
            4'b0101: begin  // NOT
                result = ~a;
            end
            4'b0110: begin  // SHL (shift left)
                carry_out = a[15];  // MSB shifted out to carry
                result = a << 1;
            end
            4'b0111: begin  // SHR (shift right logical)
                carry_out = a[0];  // LSB shifted out to carry
                result = a >> 1;
            end
            4'b1000: begin  // NEG (two's complement: -a = ~a + 1 = 0 - a)
                temp = 17'b0 - {1'b0, a};
                result = temp[15:0];
                carry_out = (a != 16'h0000);  // Carry set if result is non-zero (a was not 0)
            end
            4'b1001: begin  // MUL (a * b = 32-bit result)
                // Uses sequential multiplier results (16 cycles, area-efficient)
                result = mul_product_low;      // Low word to AC
                mul_high = mul_product_high;   // High word to Y register
                carry_out = (mul_product_high != 16'h0000);  // Carry set if overflow (high word non-zero)
            end
            4'b1010: begin  // DIV (a / b = quotient, a % b = remainder)
                // Uses sequential divider results (16 cycles, area-efficient)
                result = div_quotient;     // Quotient to AC
                mul_high = div_remainder;  // Remainder to Y (reusing mul_high output)
                carry_out = div_by_zero;   // Set carry if division by zero
            end
            4'b1011: begin  // MOD (a % b = remainder, a / b = quotient)
                // Uses sequential divider results (16 cycles, area-efficient)
                result = div_remainder;    // Remainder to AC
                mul_high = div_quotient;   // Quotient to Y (reusing mul_high output)
                carry_out = div_by_zero;   // Set carry if division by zero
            end
            4'b1100: begin  // ADC (a + b + carry_in) - Add with Carry
                temp = {1'b0, a} + {1'b0, b} + {16'b0, carry_in};
                result = temp[15:0];
                carry_out = temp[16];
            end
            4'b1101: begin  // SBC (a - b - carry_in) - Subtract with Borrow
                temp = {1'b0, a} - {1'b0, b} - {16'b0, carry_in};
                result = temp[15:0];
                carry_out = temp[16];  // Borrow flag (1 if underflow)
            end
            4'b1110: begin  // ASR (arithmetic shift right) - preserves sign bit
                carry_out = a[0];          // LSB shifted out to carry
                result = {a[15], a[15:1]}; // Shift right, keeping sign bit
            end
            default: begin
                result = 16'h0000;
            end
        endcase
    end
endmodule
