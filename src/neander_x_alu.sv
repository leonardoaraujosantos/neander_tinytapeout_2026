// ============================================================================
// neander_x_alu.sv — ALU for NEANDER-X CPU (LCC Extension + Carry Flag)
// ============================================================================
// Extended ALU operations (4-bit opcode):
//   0000: ADD  - a + b           (sets carry on overflow)
//   0001: SUB  - a - b           (sets carry on borrow)
//   0010: AND  - a & b
//   0011: OR   - a | b
//   0100: XOR  - a ^ b
//   0101: NOT  - ~a
//   0110: SHL  - a << 1          (carry = MSB shifted out)
//   0111: SHR  - a >> 1          (carry = LSB shifted out)
//   1000: NEG  - 0 - a = -a      (two's complement negation)
// ============================================================================

module neander_alu (
    input  logic [7:0] a,
    input  logic [7:0] b,
    input  logic [3:0] alu_op,  // Extended to 4 bits for NEG and future ops
    output logic [7:0] result,
    output logic       carry_out  // Carry/borrow flag output
);
    logic [8:0] temp;  // 9-bit for carry detection

    always_comb begin
        temp = 9'b0;
        carry_out = 1'b0;
        result = 8'h00;

        case (alu_op)
            4'b0000: begin  // ADD
                temp = {1'b0, a} + {1'b0, b};
                result = temp[7:0];
                carry_out = temp[8];
            end
            4'b0001: begin  // SUB (a - b)
                temp = {1'b0, a} - {1'b0, b};
                result = temp[7:0];
                carry_out = temp[8];  // Borrow flag (1 if a < b)
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
                carry_out = a[7];  // MSB shifted out to carry
                result = a << 1;
            end
            4'b0111: begin  // SHR (shift right logical)
                carry_out = a[0];  // LSB shifted out to carry
                result = a >> 1;
            end
            4'b1000: begin  // NEG (two's complement: -a = ~a + 1 = 0 - a)
                temp = 9'b0 - {1'b0, a};
                result = temp[7:0];
                carry_out = (a != 8'h00);  // Carry set if result is non-zero (a was not 0)
            end
            default: begin
                result = 8'h00;
            end
        endcase
    end
endmodule
