// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2024 Leonardo Araujo Santos
//
// debug_rom.sv - Debug ROM for NEANDER-X Boot Mode
//
// This ROM contains a simple program that toggles EXT_OUT0 forever.
// Used when BOOT_SEL=1 to verify the CPU is functioning without
// requiring external SPI memory.
//
// Program (uses DEC + JNZ instead of DECJNZ):
//   0x0000: LDI 0x0001       ; AC = 1
//   0x0003: OUT 0x01         ; EXT_OUT0 = 1
//   0x0005: LDI 0x00FF       ; Delay counter = 255 (fast for testing)
//   0x0008: DEC              ; AC = AC - 1
//   0x0009: JNZ 0x0008       ; Loop until AC=0
//   0x000C: LDI 0x0000       ; AC = 0
//   0x000F: OUT 0x01         ; EXT_OUT0 = 0
//   0x0011: LDI 0x00FF       ; Delay counter = 255
//   0x0014: DEC              ; AC = AC - 1
//   0x0015: JNZ 0x0014       ; Loop until AC=0
//   0x0018: JMP 0x0000       ; Restart

`default_nettype none

module debug_rom (
    input  wire [7:0] addr,
    output reg  [7:0] data
);

    // Combinational ROM - synthesizes to LUT logic
    always @(*) begin
        case (addr[4:0])  // Only 5 bits needed for 32 addresses
            // LDI 0x0001 - Load immediate 1 into AC
            5'h00: data = 8'hE0;  // LDI opcode
            5'h01: data = 8'h01;  // imm_lo = 1
            5'h02: data = 8'h00;  // imm_hi = 0

            // OUT 0x01 - Output AC to port 1 (EXT_OUT)
            5'h03: data = 8'hD0;  // OUT opcode
            5'h04: data = 8'h01;  // port = 1

            // LDI 0x00FF - Load delay counter (255 for fast toggle)
            5'h05: data = 8'hE0;  // LDI opcode
            5'h06: data = 8'hFF;  // imm_lo = 0xFF
            5'h07: data = 8'h00;  // imm_hi = 0x00

            // DEC - Decrement AC
            5'h08: data = 8'h76;  // DEC opcode

            // JNZ 0x0008 - Jump if not zero (loop back to DEC)
            5'h09: data = 8'hB0;  // JNZ opcode
            5'h0A: data = 8'h08;  // addr_lo = 0x08
            5'h0B: data = 8'h00;  // addr_hi = 0x00

            // LDI 0x0000 - Load 0 into AC
            5'h0C: data = 8'hE0;  // LDI opcode
            5'h0D: data = 8'h00;  // imm_lo = 0
            5'h0E: data = 8'h00;  // imm_hi = 0

            // OUT 0x01 - Output AC to port 1 (EXT_OUT)
            5'h0F: data = 8'hD0;  // OUT opcode
            5'h10: data = 8'h01;  // port = 1

            // LDI 0x00FF - Load delay counter
            5'h11: data = 8'hE0;  // LDI opcode
            5'h12: data = 8'hFF;  // imm_lo = 0xFF
            5'h13: data = 8'h00;  // imm_hi = 0x00

            // DEC - Decrement AC
            5'h14: data = 8'h76;  // DEC opcode

            // JNZ 0x0014 - Jump if not zero (loop back to DEC)
            5'h15: data = 8'hB0;  // JNZ opcode
            5'h16: data = 8'h14;  // addr_lo = 0x14
            5'h17: data = 8'h00;  // addr_hi = 0x00

            // JMP 0x0000 - Jump back to start
            5'h18: data = 8'h80;  // JMP opcode
            5'h19: data = 8'h00;  // addr_lo = 0x00
            5'h1A: data = 8'h00;  // addr_hi = 0x00

            // Unused addresses return NOP (0x00)
            default: data = 8'h00;
        endcase
    end

endmodule
