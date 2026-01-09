// ============================================================================
// neander_x_alu.sv — ALU for NEANDER-X CPU
// ============================================================================

module neander_alu (
    input  logic [7:0] a,
    input  logic [7:0] b,
    input  logic [1:0] alu_op,
    output logic [7:0] result
);
    always_comb begin
        case (alu_op)
            2'b00: result = a + b;   // ADD
            2'b01: result = a & b;   // AND
            2'b10: result = a | b;   // OR
            2'b11: result = ~a;      // NOT (on A)
            default: result = 8'h00;
        endcase
    end
endmodule
