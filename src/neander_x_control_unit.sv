// ============================================================================
// neander_x_control_unit.sv — Control Unit for NEANDER-X CPU (LCC + X/Y Register + Carry Flag)
// ============================================================================
// LCC Compiler Extension Instructions:
//
// Opcode 0x0 sub-opcodes (single-byte instructions):
//   0x00: NOP       - No operation
//   0x01: NEG       - AC = -AC (two's complement negation)
//   0x03: TAY       - Y = AC (transfer AC to Y)
//   0x04: TYA       - AC = Y (transfer Y to AC)
//   0x05: INY       - Y = Y + 1 (increment Y)
//
// Opcode 0x0 with address/immediate operand:
//   0x02: CMP addr  - Compare AC with MEM[addr], set flags only (N, Z, C)
//   0x06: LDYI imm  - Y = imm (load immediate)
//   0x07: LDY addr  - Y = MEM[addr]
//   0x08: STY addr  - MEM[addr] = Y
//   0x09: MUL       - AC * X -> Y:AC (16-bit result, high byte in Y, low byte in AC)
//
// LCC Extension using opcode 0x7 sub-opcodes:
//   0x74: SUB addr  - AC = AC - MEM[addr]
//   0x75: INC       - AC = AC + 1
//   0x76: DEC       - AC = AC - 1
//   0x77: XOR addr  - AC = AC ^ MEM[addr]
//   0x78: SHL       - AC = AC << 1
//   0x79: SHR       - AC = AC >> 1
//
// X Register Extension:
//   0x7A: LDX addr  - X = MEM[addr]
//   0x7B: STX addr  - MEM[addr] = X
//   0x7C: LDXI imm  - X = imm (load immediate)
//   0x7D: TAX       - X = AC (transfer AC to X)
//   0x7E: TXA       - AC = X (transfer X to AC)
//   0x7F: INX       - X = X + 1 (increment X)
//
// Indexed Addressing with X (sub_opcode bit 0):
//   0x21: LDA addr,X - AC = MEM[addr + X]
//   0x11: STA addr,X - MEM[addr + X] = AC
//
// Indexed Addressing with Y (sub_opcode bit 1):
//   0x22: LDA addr,Y - AC = MEM[addr + Y]
//   0x12: STA addr,Y - MEM[addr + Y] = AC
//
// Carry-based Jump Instructions (opcode 0x8 with sub-opcodes):
//   0x80: JMP addr  - Unconditional jump
//   0x81: JC addr   - Jump if Carry flag set
//   0x82: JNC addr  - Jump if Carry flag clear
//
// Signed Comparison Jumps (after CMP instruction):
//   0x83: JLE addr  - Jump if Less or Equal (N=1 OR Z=1)
//   0x84: JGT addr  - Jump if Greater Than (N=0 AND Z=0)
//   0x85: JGE addr  - Jump if Greater or Equal (N=0)
//
// Unsigned Comparison Jumps (after CMP instruction):
//   0x86: JBE addr  - Jump if Below or Equal (C=1 OR Z=1)
//   0x87: JA addr   - Jump if Above (C=0 AND Z=0)
// ============================================================================

module neander_control (
    input  logic       clk,
    input  logic       reset,
    input  logic [3:0] opcode,
    input  logic [3:0] sub_opcode,  // Lower nibble for stack/LCC ops and indexed mode
    input  logic       flagN,
    input  logic       flagZ,
    input  logic       flagC,       // Carry flag input (for JC/JNC)

    output logic       mem_read,
    output logic       mem_write,
    output logic       pc_inc,
    output logic       pc_load,
    output logic       ac_load,
    output logic       ri_load,
    output logic       rem_load,
    output logic       rdm_load,
    output logic       nz_load,
    output logic       c_load,      // Carry flag load
    output logic [1:0] addr_sel,    // 00=RDM, 01=PC, 10=SP
    output logic [3:0] alu_op,      // Extended to 4 bits for NEG
    output logic       io_write,
    output logic       sp_inc,      // Stack pointer increment (POP/RET)
    output logic       sp_dec,      // Stack pointer decrement (PUSH/CALL)
    output logic [1:0] mem_data_sel,// Memory data select: 00=AC, 01=PC, 10=X, 11=Y
    output logic [1:0] alu_b_sel,   // ALU B input select: 00=mem_data, 01=constant 1 (INC/DEC), 10=X (MUL)
    // X Register Extension signals
    output logic       x_load,      // Load X register
    output logic       x_inc,       // Increment X register
    output logic       x_to_ac,     // Transfer X to AC (TXA)
    output logic       indexed_mode, // Use indexed addressing (addr + X)
    // Y Register Extension signals
    output logic       y_load,      // Load Y register
    output logic       y_inc,       // Increment Y register
    output logic       y_to_ac,     // Transfer Y to AC (TYA)
    output logic       indexed_mode_y, // Use indexed addressing (addr + Y)
    output logic       mul_to_y     // Load Y with MUL high byte
);

    // Using ENUM for states (Easier to debug in Waveforms)
    typedef enum logic [7:0] {  // Extended to 8 bits for all extension states
        S_FETCH_1, S_FETCH_2, S_FETCH_3, S_DECODE,
        S_LDA_1, S_LDA_2, S_LDA_3, S_LDA_4,
        S_STA_1, S_STA_2, S_STA_3, S_STA_4,
        S_ADD_1, S_ADD_2, S_ADD_3, S_ADD_4,
        S_AND_1, S_AND_2, S_AND_3, S_AND_4,
        S_OR_1,  S_OR_2,  S_OR_3,  S_OR_4,
        S_NOT,
        S_JMP_1, S_JMP_2, S_JMP_3,
        S_JN_1,  S_JN_2,  S_JN_3,
        S_JZ_1,  S_JZ_2,  S_JZ_3,
        S_JNZ_1, S_JNZ_2, S_JNZ_3,
        S_LDI_1, S_LDI_2,
        S_IN_1,  S_IN_2,  S_IN_3,
        S_OUT_1, S_OUT_2, S_OUT_3,
        S_PUSH_1, S_PUSH_2, S_PUSH_3,  // Stack PUSH
        S_POP_1,  S_POP_2,  S_POP_3,   // Stack POP
        S_CALL_1, S_CALL_2, S_CALL_3, S_CALL_4, S_CALL_5, // CALL subroutine
        S_RET_1,  S_RET_2,  S_RET_3,   // RET from subroutine
        // LCC Extension states
        S_SUB_1, S_SUB_2, S_SUB_3, S_SUB_4,  // SUB addr (0x74)
        S_INC,                                 // INC (0x75) - single cycle
        S_DEC,                                 // DEC (0x76) - single cycle
        S_XOR_1, S_XOR_2, S_XOR_3, S_XOR_4,  // XOR addr (0x77)
        S_SHL,                                 // SHL (0x78) - single cycle
        S_SHR,                                 // SHR (0x79) - single cycle
        // X Register Extension states
        S_LDX_1, S_LDX_2, S_LDX_3, S_LDX_4,  // LDX addr (0x7A)
        S_STX_1, S_STX_2, S_STX_3, S_STX_4,  // STX addr (0x7B)
        S_LDXI_1, S_LDXI_2,                   // LDXI imm (0x7C)
        S_TAX,                                 // TAX (0x7D) - single cycle
        S_TXA,                                 // TXA (0x7E) - single cycle
        S_INX,                                 // INX (0x7F) - single cycle
        // Indexed addressing states (X)
        S_LDA_X_1, S_LDA_X_2, S_LDA_X_3, S_LDA_X_4,  // LDA addr,X (0x21)
        S_STA_X_1, S_STA_X_2, S_STA_X_3, S_STA_X_4,  // STA addr,X (0x11)
        // LCC Compiler Extension states (NEG, CMP, JC, JNC)
        S_NEG,                                 // NEG (0x01) - single cycle
        S_CMP_1, S_CMP_2, S_CMP_3, S_CMP_4,  // CMP addr (0x02) - compare, set flags only
        S_JC_1,  S_JC_2,  S_JC_3,             // JC addr (0x81) - jump if carry
        S_JNC_1, S_JNC_2, S_JNC_3,            // JNC addr (0x82) - jump if no carry
        // Signed comparison jumps
        S_JLE_1, S_JLE_2, S_JLE_3,            // JLE addr (0x83) - jump if less or equal (N=1 OR Z=1)
        S_JGT_1, S_JGT_2, S_JGT_3,            // JGT addr (0x84) - jump if greater than (N=0 AND Z=0)
        S_JGE_1, S_JGE_2, S_JGE_3,            // JGE addr (0x85) - jump if greater or equal (N=0)
        // Unsigned comparison jumps
        S_JBE_1, S_JBE_2, S_JBE_3,            // JBE addr (0x86) - jump if below or equal (C=1 OR Z=1)
        S_JA_1,  S_JA_2,  S_JA_3,             // JA addr (0x87) - jump if above (C=0 AND Z=0)
        // Y Register Extension states
        S_LDY_1, S_LDY_2, S_LDY_3, S_LDY_4,  // LDY addr (0x07)
        S_STY_1, S_STY_2, S_STY_3, S_STY_4,  // STY addr (0x08)
        S_LDYI_1, S_LDYI_2,                   // LDYI imm (0x06)
        S_TAY,                                 // TAY (0x03) - single cycle
        S_TYA,                                 // TYA (0x04) - single cycle
        S_INY,                                 // INY (0x05) - single cycle
        // Indexed addressing states (Y)
        S_LDA_Y_1, S_LDA_Y_2, S_LDA_Y_3, S_LDA_Y_4,  // LDA addr,Y (0x22)
        S_STA_Y_1, S_STA_Y_2, S_STA_Y_3, S_STA_Y_4,  // STA addr,Y (0x12)
        // Multiplication
        S_MUL,                                        // MUL (0x09) - AC * X -> Y:AC
        S_HLT
    } state_t;

    state_t state, next_state;

    // State Register
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            state <= S_FETCH_1;
        else
            state <= next_state;
    end

    // Next State & Output Logic
    always_comb begin
        // Defaults
        mem_read     = '0;
        mem_write    = '0;
        pc_inc       = '0;
        pc_load      = '0;
        ac_load      = '0;
        ri_load      = '0;
        rem_load     = '0;
        rdm_load     = '0;
        nz_load      = '0;
        c_load       = '0;     // Carry flag load default
        addr_sel     = 2'b01;  // Default to PC (01=PC)
        alu_op       = 4'b0000; // Default to ADD (4 bits now)
        io_write     = '0;
        sp_inc       = '0;
        sp_dec       = '0;
        mem_data_sel = 2'b00;  // Default to AC (00=AC)
        alu_b_sel    = 2'b00;  // Default to mem_data (00=mem_data)
        // X Register Extension defaults
        x_load       = '0;
        x_inc        = '0;
        x_to_ac      = '0;
        indexed_mode = '0;
        // Y Register Extension defaults
        y_load       = '0;
        y_inc        = '0;
        y_to_ac      = '0;
        indexed_mode_y = '0;
        mul_to_y     = '0;

        next_state  = state;

        case (state)
            // --- FETCH ---
            S_FETCH_1: begin
                addr_sel    = 2'b01; // PC -> REM
                rem_load    = 1;
                mem_read    = 1;
                next_state  = S_FETCH_2;
            end
            S_FETCH_2: begin
                mem_read    = 1;
                rdm_load    = 1;
                next_state  = S_FETCH_3;
            end
            S_FETCH_3: begin
                ri_load     = 1;
                pc_inc      = 1;
                next_state  = S_DECODE;
            end

            // --- DECODE ---
            S_DECODE: begin
                case (opcode)
                    4'h0: begin  // NOP family + LCC extensions (NEG, CMP) + Y register ops + MUL
                        case (sub_opcode)
                            4'h0: next_state = S_FETCH_1;  // NOP (0x00)
                            4'h1: next_state = S_NEG;      // NEG (0x01)
                            4'h2: next_state = S_CMP_1;    // CMP addr (0x02)
                            4'h3: next_state = S_TAY;      // TAY (0x03)
                            4'h4: next_state = S_TYA;      // TYA (0x04)
                            4'h5: next_state = S_INY;      // INY (0x05)
                            4'h6: next_state = S_LDYI_1;   // LDYI imm (0x06)
                            4'h7: next_state = S_LDY_1;    // LDY addr (0x07)
                            4'h8: next_state = S_STY_1;    // STY addr (0x08)
                            4'h9: next_state = S_MUL;      // MUL (0x09) - AC * X -> Y:AC
                            default: next_state = S_FETCH_1;
                        endcase
                    end
                    4'h2: begin
                        // LDA: check sub_opcode for indexed mode
                        if (sub_opcode[1])
                            next_state = S_LDA_Y_1;  // LDA addr,Y (0x22)
                        else if (sub_opcode[0])
                            next_state = S_LDA_X_1;  // LDA addr,X (0x21)
                        else
                            next_state = S_LDA_1;    // LDA addr (0x20)
                    end
                    4'h1: begin
                        // STA: check sub_opcode for indexed mode
                        if (sub_opcode[1])
                            next_state = S_STA_Y_1;  // STA addr,Y (0x12)
                        else if (sub_opcode[0])
                            next_state = S_STA_X_1;  // STA addr,X (0x11)
                        else
                            next_state = S_STA_1;    // STA addr (0x10)
                    end
                    4'h3: next_state = S_ADD_1;
                    4'h5: next_state = S_AND_1;
                    4'h4: next_state = S_OR_1;
                    4'h6: next_state = S_NOT;
                    4'h7: begin  // Stack operations + LCC extension (sub-opcode in lower nibble)
                        case (sub_opcode)
                            4'h0: next_state = S_PUSH_1;  // PUSH (0x70)
                            4'h1: next_state = S_POP_1;   // POP  (0x71)
                            4'h2: next_state = S_CALL_1;  // CALL (0x72)
                            4'h3: next_state = S_RET_1;   // RET  (0x73)
                            // LCC Extension instructions
                            4'h4: next_state = S_SUB_1;   // SUB  (0x74)
                            4'h5: next_state = S_INC;     // INC  (0x75)
                            4'h6: next_state = S_DEC;     // DEC  (0x76)
                            4'h7: next_state = S_XOR_1;   // XOR  (0x77)
                            4'h8: next_state = S_SHL;     // SHL  (0x78)
                            4'h9: next_state = S_SHR;     // SHR  (0x79)
                            // X Register Extension instructions
                            4'hA: next_state = S_LDX_1;   // LDX  (0x7A)
                            4'hB: next_state = S_STX_1;   // STX  (0x7B)
                            4'hC: next_state = S_LDXI_1;  // LDXI (0x7C)
                            4'hD: next_state = S_TAX;     // TAX  (0x7D)
                            4'hE: next_state = S_TXA;     // TXA  (0x7E)
                            4'hF: next_state = S_INX;     // INX  (0x7F)
                            default: next_state = S_FETCH_1;
                        endcase
                    end
                    4'h8: begin  // JMP family + carry-based jumps + comparison jumps
                        case (sub_opcode)
                            4'h0: next_state = S_JMP_1;   // JMP  (0x80)
                            4'h1: next_state = S_JC_1;    // JC   (0x81)
                            4'h2: next_state = S_JNC_1;   // JNC  (0x82)
                            // Signed comparison jumps (after CMP)
                            4'h3: next_state = S_JLE_1;   // JLE  (0x83)
                            4'h4: next_state = S_JGT_1;   // JGT  (0x84)
                            4'h5: next_state = S_JGE_1;   // JGE  (0x85)
                            // Unsigned comparison jumps (after CMP)
                            4'h6: next_state = S_JBE_1;   // JBE  (0x86)
                            4'h7: next_state = S_JA_1;    // JA   (0x87)
                            default: next_state = S_JMP_1; // Default to JMP for backward compat
                        endcase
                    end
                    4'h9: next_state = S_JN_1;
                    4'hA: next_state = S_JZ_1;
                    4'hB: next_state = S_JNZ_1;
                    4'hE: next_state = S_LDI_1;
                    4'hC: next_state = S_IN_1;
                    4'hD: next_state = S_OUT_1;
                    4'hF: next_state = S_HLT;
                    default: next_state = S_FETCH_1;
                endcase
            end

            // --- LDI ---
            S_LDI_1: begin
                addr_sel = 2'b01;
                rem_load    = 1;
                mem_read    = 1;
                next_state  = S_LDI_2;
            end
            S_LDI_2: begin
                mem_read    = 1;
                ac_load     = 1;
                nz_load     = 1;
                pc_inc      = 1;
                next_state  = S_FETCH_1;
            end

            // --- LDA ---
            S_LDA_1: begin
                addr_sel = 2'b01;
                rem_load    = 1;
                mem_read    = 1;
                next_state  = S_LDA_2;
            end
            S_LDA_2: begin
                mem_read    = 1;
                rdm_load    = 1;
                pc_inc      = 1;
                next_state  = S_LDA_3;
            end
            S_LDA_3: begin
                addr_sel = 2'b00; // RDM -> REM
                rem_load    = 1;
                mem_read    = 1;
                next_state  = S_LDA_4;
            end
            S_LDA_4: begin
                mem_read    = 1;
                ac_load     = 1;
                nz_load     = 1;
                next_state  = S_FETCH_1;
            end

            // --- STA ---
            S_STA_1: begin
                addr_sel = 2'b01;
                rem_load    = 1;
                mem_read    = 1;
                next_state  = S_STA_2;
            end
            S_STA_2: begin
                mem_read    = 1;
                rdm_load    = 1;
                pc_inc      = 1;
                next_state  = S_STA_3;
            end
            S_STA_3: begin
                addr_sel = 2'b00;
                rem_load    = 1;
                next_state  = S_STA_4;
            end
            S_STA_4: begin
                mem_write   = 1;
                next_state  = S_FETCH_1;
            end

            // --- ADD ---
            S_ADD_1: begin
                addr_sel = 2'b01;
                rem_load    = 1;
                mem_read    = 1;
                next_state  = S_ADD_2;
            end
            S_ADD_2: begin
                mem_read    = 1;
                rdm_load    = 1;
                pc_inc      = 1;
                next_state  = S_ADD_3;
            end
            S_ADD_3: begin
                addr_sel = 2'b00;
                rem_load    = 1;
                mem_read    = 1;
                next_state  = S_ADD_4;
            end
            S_ADD_4: begin
                mem_read    = 1;
                ac_load     = 1;
                alu_op      = 4'b0000;  // ADD
                nz_load     = 1;
                c_load      = 1;        // Update carry flag
                next_state  = S_FETCH_1;
            end

            // --- AND ---
            S_AND_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_AND_2;
            end
            S_AND_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_AND_3;
            end
            S_AND_3: begin
                addr_sel = 2'b00; rem_load = 1; mem_read = 1; next_state = S_AND_4;
            end
            S_AND_4: begin
                mem_read = 1; ac_load = 1; alu_op = 4'b0010; nz_load = 1; next_state = S_FETCH_1;  // AND
            end

            // --- OR ---
            S_OR_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_OR_2;
            end
            S_OR_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_OR_3;
            end
            S_OR_3: begin
                addr_sel = 2'b00; rem_load = 1; mem_read = 1; next_state = S_OR_4;
            end
            S_OR_4: begin
                mem_read = 1; ac_load = 1; alu_op = 4'b0011; nz_load = 1; next_state = S_FETCH_1;  // OR
            end

            // --- NOT ---
            S_NOT: begin
                ac_load = 1; alu_op = 4'b0101; nz_load = 1; next_state = S_FETCH_1;  // NOT
            end

            // --- JMP ---
            S_JMP_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_JMP_2;
            end
            S_JMP_2: begin
                mem_read = 1; rdm_load = 1; next_state = S_JMP_3;
            end
            S_JMP_3: begin
                pc_load = 1; next_state = S_FETCH_1;
            end

            // --- JN ---
            S_JN_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_JN_2;
            end
            S_JN_2: begin
                mem_read = 1; rdm_load = 1; next_state = S_JN_3;
            end
            S_JN_3: begin
                if (flagN) pc_load = 1;
                else       pc_inc  = 1;
                next_state = S_FETCH_1;
            end

            // --- JZ ---
            S_JZ_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_JZ_2;
            end
            S_JZ_2: begin
                mem_read = 1; rdm_load = 1; next_state = S_JZ_3;
            end
            S_JZ_3: begin
                if (flagZ) pc_load = 1;
                else       pc_inc  = 1;
                next_state = S_FETCH_1;
            end

            // --- JNZ ---
            S_JNZ_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_JNZ_2;
            end
            S_JNZ_2: begin
                mem_read = 1; rdm_load = 1; next_state = S_JNZ_3;
            end
            S_JNZ_3: begin
                if (!flagZ) pc_load = 1;
                else        pc_inc  = 1;
                next_state  = S_FETCH_1;
            end

            // --- IN ---
            S_IN_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_IN_2;
            end
            S_IN_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_IN_3;
            end
            S_IN_3: begin
                ac_load = 1; nz_load = 1; next_state = S_FETCH_1;
            end

            // --- OUT ---
            S_OUT_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_OUT_2;
            end
            S_OUT_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_OUT_3;
            end
            S_OUT_3: begin
                io_write = 1; next_state = S_FETCH_1;
            end

            // --- PUSH (0x70) ---
            // Push AC onto stack: decrement SP, load SP to REM, write AC to [SP]
            S_PUSH_1: begin
                sp_dec = 1;  // Decrement SP first (SP now points to new stack top)
                next_state = S_PUSH_2;
            end
            S_PUSH_2: begin
                addr_sel = 2'b10;  // SP -> REM (load new SP value into REM)
                rem_load = 1;
                next_state = S_PUSH_3;
            end
            S_PUSH_3: begin
                mem_write = 1;     // Write AC to memory at [REM]
                next_state = S_FETCH_1;
            end

            // --- POP (0x71) ---
            // Pop from stack to AC: read [SP] to AC, then increment SP
            S_POP_1: begin
                addr_sel = 2'b10;  // SP -> REM
                rem_load = 1;
                next_state = S_POP_2;
            end
            S_POP_2: begin
                mem_read = 1;      // Read memory at [REM]
                next_state = S_POP_3;
            end
            S_POP_3: begin
                mem_read = 1;
                ac_load = 1;       // Load data into AC
                nz_load = 1;       // Update flags
                sp_inc = 1;        // Increment SP (point to next stack item)
                next_state = S_FETCH_1;
            end

            // --- CALL (0x72 addr) ---
            // Call subroutine: fetch target addr, push return addr (PC), jump to target
            S_CALL_1: begin
                addr_sel = 2'b01;  // PC -> REM (fetch target address)
                rem_load = 1;
                mem_read = 1;
                next_state = S_CALL_2;
            end
            S_CALL_2: begin
                mem_read = 1;
                rdm_load = 1;      // Load target address to RDM
                pc_inc = 1;        // PC now points to instruction after CALL (return address)
                next_state = S_CALL_3;
            end
            S_CALL_3: begin
                sp_dec = 1;        // Decrement SP (make room for return address)
                next_state = S_CALL_4;
            end
            S_CALL_4: begin
                addr_sel = 2'b10;  // SP -> REM
                rem_load = 1;
                next_state = S_CALL_5;
            end
            S_CALL_5: begin
                mem_write = 1;     // Write return address (PC) to [SP]
                mem_data_sel = 1;  // Select PC as data source
                pc_load = 1;       // Load target address from RDM to PC
                next_state = S_FETCH_1;
            end

            // --- RET (0x73) ---
            // Return from subroutine: pop return address from stack to PC
            S_RET_1: begin
                addr_sel = 2'b10;  // SP -> REM
                rem_load = 1;
                next_state = S_RET_2;
            end
            S_RET_2: begin
                mem_read = 1;
                rdm_load = 1;      // Load return address to RDM
                next_state = S_RET_3;
            end
            S_RET_3: begin
                pc_load = 1;       // Load return address from RDM to PC
                sp_inc = 1;        // Increment SP (pop)
                next_state = S_FETCH_1;
            end

            // ================================================================
            // LCC EXTENSION INSTRUCTIONS
            // ================================================================

            // --- SUB addr (0x74) ---
            // Subtract memory from AC: AC = AC - MEM[addr]
            S_SUB_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_SUB_2;
            end
            S_SUB_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_SUB_3;
            end
            S_SUB_3: begin
                addr_sel = 2'b00; rem_load = 1; mem_read = 1; next_state = S_SUB_4;
            end
            S_SUB_4: begin
                mem_read = 1; ac_load = 1; alu_op = 4'b0001; nz_load = 1; c_load = 1; next_state = S_FETCH_1;  // SUB with carry
            end

            // --- INC (0x75) ---
            // Increment AC: AC = AC + 1 (single cycle)
            S_INC: begin
                ac_load = 1;
                alu_op = 4'b0000;   // ADD
                alu_b_sel = 2'b01;  // Select constant 1 as ALU B input
                nz_load = 1;
                next_state = S_FETCH_1;
            end

            // --- DEC (0x76) ---
            // Decrement AC: AC = AC - 1 (single cycle)
            S_DEC: begin
                ac_load = 1;
                alu_op = 4'b0001;   // SUB
                alu_b_sel = 2'b01;  // Select constant 1 as ALU B input
                nz_load = 1;
                next_state = S_FETCH_1;
            end

            // --- XOR addr (0x77) ---
            // XOR memory with AC: AC = AC ^ MEM[addr]
            S_XOR_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_XOR_2;
            end
            S_XOR_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_XOR_3;
            end
            S_XOR_3: begin
                addr_sel = 2'b00; rem_load = 1; mem_read = 1; next_state = S_XOR_4;
            end
            S_XOR_4: begin
                mem_read = 1; ac_load = 1; alu_op = 4'b0100; nz_load = 1; next_state = S_FETCH_1;  // XOR
            end

            // --- SHL (0x78) ---
            // Shift AC left: AC = AC << 1 (single cycle)
            S_SHL: begin
                ac_load = 1;
                alu_op = 4'b0110;   // SHL
                nz_load = 1;
                next_state = S_FETCH_1;
            end

            // --- SHR (0x79) ---
            // Shift AC right: AC = AC >> 1 (single cycle)
            S_SHR: begin
                ac_load = 1;
                alu_op = 4'b0111;   // SHR
                nz_load = 1;
                next_state = S_FETCH_1;
            end

            // ================================================================
            // X REGISTER EXTENSION INSTRUCTIONS
            // ================================================================

            // --- LDX addr (0x7A) ---
            // Load X from memory: X = MEM[addr]
            S_LDX_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_LDX_2;
            end
            S_LDX_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_LDX_3;
            end
            S_LDX_3: begin
                addr_sel = 2'b00; rem_load = 1; mem_read = 1; next_state = S_LDX_4;
            end
            S_LDX_4: begin
                mem_read = 1;
                x_load = 1;        // Load X from memory
                next_state = S_FETCH_1;
            end

            // --- STX addr (0x7B) ---
            // Store X to memory: MEM[addr] = X
            S_STX_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_STX_2;
            end
            S_STX_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_STX_3;
            end
            S_STX_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_STX_4;
            end
            S_STX_4: begin
                mem_write = 1;
                mem_data_sel = 2'b10;  // Select X as data source
                next_state = S_FETCH_1;
            end

            // --- LDXI imm (0x7C) ---
            // Load X with immediate: X = imm
            S_LDXI_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_LDXI_2;
            end
            S_LDXI_2: begin
                mem_read = 1;
                x_load = 1;        // Load X from immediate (mem_data_in)
                pc_inc = 1;
                next_state = S_FETCH_1;
            end

            // --- TAX (0x7D) ---
            // Transfer AC to X: X = AC (single cycle)
            S_TAX: begin
                x_load = 1;        // Load X from AC (datapath handles mux)
                next_state = S_FETCH_1;
            end

            // --- TXA (0x7E) ---
            // Transfer X to AC: AC = X (single cycle)
            S_TXA: begin
                x_to_ac = 1;       // Signal datapath to select X for AC input
                ac_load = 1;
                nz_load = 1;       // Update flags based on X value
                next_state = S_FETCH_1;
            end

            // --- INX (0x7F) ---
            // Increment X: X = X + 1 (single cycle)
            S_INX: begin
                x_inc = 1;         // Increment X register
                next_state = S_FETCH_1;
            end

            // ================================================================
            // INDEXED ADDRESSING MODES
            // ================================================================

            // --- LDA addr,X (0x21) ---
            // Load AC from memory with indexed addressing: AC = MEM[addr + X]
            S_LDA_X_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_LDA_X_2;
            end
            S_LDA_X_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_LDA_X_3;
            end
            S_LDA_X_3: begin
                addr_sel = 2'b00; rem_load = 1; mem_read = 1; next_state = S_LDA_X_4;
            end
            S_LDA_X_4: begin
                mem_read = 1;
                indexed_mode = 1;  // Use addr + X for memory access
                ac_load = 1;
                nz_load = 1;
                next_state = S_FETCH_1;
            end

            // --- STA addr,X (0x11) ---
            // Store AC to memory with indexed addressing: MEM[addr + X] = AC
            S_STA_X_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_STA_X_2;
            end
            S_STA_X_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_STA_X_3;
            end
            S_STA_X_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_STA_X_4;
            end
            S_STA_X_4: begin
                indexed_mode = 1;  // Use addr + X for memory access
                mem_write = 1;
                next_state = S_FETCH_1;
            end

            // ================================================================
            // LCC COMPILER EXTENSION: NEG, CMP, JC, JNC
            // ================================================================

            // --- NEG (0x01) ---
            // Negate AC: AC = -AC (two's complement)
            S_NEG: begin
                ac_load = 1;
                alu_op = 4'b1000;  // NEG operation
                nz_load = 1;
                c_load = 1;        // Also update carry flag
                next_state = S_FETCH_1;
            end

            // --- CMP addr (0x02) ---
            // Compare AC with memory: set flags N, Z, C based on (AC - MEM[addr])
            // Does NOT modify AC, only sets flags
            S_CMP_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_CMP_2;
            end
            S_CMP_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_CMP_3;
            end
            S_CMP_3: begin
                addr_sel = 2'b00; rem_load = 1; mem_read = 1; next_state = S_CMP_4;
            end
            S_CMP_4: begin
                mem_read = 1;
                // Do NOT load AC - only set flags
                alu_op = 4'b0001;  // SUB (for comparison)
                nz_load = 1;       // Update N and Z flags
                c_load = 1;        // Update C flag (borrow)
                next_state = S_FETCH_1;
            end

            // --- JC addr (0x81) ---
            // Jump if Carry flag is set
            S_JC_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_JC_2;
            end
            S_JC_2: begin
                mem_read = 1; rdm_load = 1; next_state = S_JC_3;
            end
            S_JC_3: begin
                if (flagC) pc_load = 1;
                else       pc_inc  = 1;
                next_state = S_FETCH_1;
            end

            // --- JNC addr (0x82) ---
            // Jump if Carry flag is clear (No Carry)
            S_JNC_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_JNC_2;
            end
            S_JNC_2: begin
                mem_read = 1; rdm_load = 1; next_state = S_JNC_3;
            end
            S_JNC_3: begin
                if (!flagC) pc_load = 1;
                else        pc_inc  = 1;
                next_state = S_FETCH_1;
            end

            // ================================================================
            // SIGNED COMPARISON JUMPS (after CMP instruction)
            // ================================================================

            // --- JLE addr (0x83) ---
            // Jump if Less or Equal (signed): N=1 OR Z=1
            S_JLE_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_JLE_2;
            end
            S_JLE_2: begin
                mem_read = 1; rdm_load = 1; next_state = S_JLE_3;
            end
            S_JLE_3: begin
                if (flagN || flagZ) pc_load = 1;
                else                pc_inc  = 1;
                next_state = S_FETCH_1;
            end

            // --- JGT addr (0x84) ---
            // Jump if Greater Than (signed): N=0 AND Z=0
            S_JGT_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_JGT_2;
            end
            S_JGT_2: begin
                mem_read = 1; rdm_load = 1; next_state = S_JGT_3;
            end
            S_JGT_3: begin
                if (!flagN && !flagZ) pc_load = 1;
                else                  pc_inc  = 1;
                next_state = S_FETCH_1;
            end

            // --- JGE addr (0x85) ---
            // Jump if Greater or Equal (signed): N=0
            S_JGE_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_JGE_2;
            end
            S_JGE_2: begin
                mem_read = 1; rdm_load = 1; next_state = S_JGE_3;
            end
            S_JGE_3: begin
                if (!flagN) pc_load = 1;
                else        pc_inc  = 1;
                next_state = S_FETCH_1;
            end

            // ================================================================
            // UNSIGNED COMPARISON JUMPS (after CMP instruction)
            // ================================================================

            // --- JBE addr (0x86) ---
            // Jump if Below or Equal (unsigned): C=1 OR Z=1
            S_JBE_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_JBE_2;
            end
            S_JBE_2: begin
                mem_read = 1; rdm_load = 1; next_state = S_JBE_3;
            end
            S_JBE_3: begin
                if (flagC || flagZ) pc_load = 1;
                else                pc_inc  = 1;
                next_state = S_FETCH_1;
            end

            // --- JA addr (0x87) ---
            // Jump if Above (unsigned): C=0 AND Z=0
            S_JA_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_JA_2;
            end
            S_JA_2: begin
                mem_read = 1; rdm_load = 1; next_state = S_JA_3;
            end
            S_JA_3: begin
                if (!flagC && !flagZ) pc_load = 1;
                else                  pc_inc  = 1;
                next_state = S_FETCH_1;
            end

            // ================================================================
            // Y REGISTER EXTENSION INSTRUCTIONS
            // ================================================================

            // --- LDY addr (0x07) ---
            // Load Y from memory: Y = MEM[addr]
            S_LDY_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_LDY_2;
            end
            S_LDY_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_LDY_3;
            end
            S_LDY_3: begin
                addr_sel = 2'b00; rem_load = 1; mem_read = 1; next_state = S_LDY_4;
            end
            S_LDY_4: begin
                mem_read = 1;
                y_load = 1;        // Load Y from memory
                next_state = S_FETCH_1;
            end

            // --- STY addr (0x08) ---
            // Store Y to memory: MEM[addr] = Y
            S_STY_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_STY_2;
            end
            S_STY_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_STY_3;
            end
            S_STY_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_STY_4;
            end
            S_STY_4: begin
                mem_write = 1;
                mem_data_sel = 2'b11;  // Select Y as data source
                next_state = S_FETCH_1;
            end

            // --- LDYI imm (0x06) ---
            // Load Y with immediate: Y = imm
            S_LDYI_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_LDYI_2;
            end
            S_LDYI_2: begin
                mem_read = 1;
                y_load = 1;        // Load Y from immediate (mem_data_in)
                pc_inc = 1;
                next_state = S_FETCH_1;
            end

            // --- TAY (0x03) ---
            // Transfer AC to Y: Y = AC (single cycle)
            S_TAY: begin
                y_load = 1;        // Load Y from AC (datapath handles mux)
                next_state = S_FETCH_1;
            end

            // --- TYA (0x04) ---
            // Transfer Y to AC: AC = Y (single cycle)
            S_TYA: begin
                y_to_ac = 1;       // Signal datapath to select Y for AC input
                ac_load = 1;
                nz_load = 1;       // Update flags based on Y value
                next_state = S_FETCH_1;
            end

            // --- INY (0x05) ---
            // Increment Y: Y = Y + 1 (single cycle)
            S_INY: begin
                y_inc = 1;         // Increment Y register
                next_state = S_FETCH_1;
            end

            // ================================================================
            // INDEXED ADDRESSING MODES (Y)
            // ================================================================

            // --- LDA addr,Y (0x22) ---
            // Load AC from memory with indexed addressing: AC = MEM[addr + Y]
            S_LDA_Y_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_LDA_Y_2;
            end
            S_LDA_Y_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_LDA_Y_3;
            end
            S_LDA_Y_3: begin
                addr_sel = 2'b00; rem_load = 1; mem_read = 1; next_state = S_LDA_Y_4;
            end
            S_LDA_Y_4: begin
                mem_read = 1;
                indexed_mode_y = 1;  // Use addr + Y for memory access
                ac_load = 1;
                nz_load = 1;
                next_state = S_FETCH_1;
            end

            // --- STA addr,Y (0x12) ---
            // Store AC to memory with indexed addressing: MEM[addr + Y] = AC
            S_STA_Y_1: begin
                addr_sel = 2'b01; rem_load = 1; mem_read = 1; next_state = S_STA_Y_2;
            end
            S_STA_Y_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_STA_Y_3;
            end
            S_STA_Y_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_STA_Y_4;
            end
            S_STA_Y_4: begin
                indexed_mode_y = 1;  // Use addr + Y for memory access
                mem_write = 1;
                next_state = S_FETCH_1;
            end

            // ================================================================
            // MULTIPLICATION INSTRUCTION
            // ================================================================

            // --- MUL (0x09) ---
            // Multiply AC by X: AC * X -> Y:AC (16-bit result)
            // Low byte goes to AC, high byte goes to Y
            // Single cycle operation (combinational multiplier in ALU)
            S_MUL: begin
                alu_op = 4'b1001;   // MUL operation
                alu_b_sel = 2'b10;  // Select X register as ALU B input
                ac_load = 1;        // Load low byte to AC
                y_load = 1;         // Load high byte to Y
                mul_to_y = 1;       // Signal datapath to use mul_high for Y input
                nz_load = 1;        // Update N and Z based on low byte (AC)
                c_load = 1;         // Set carry if overflow (high byte != 0)
                next_state = S_FETCH_1;
            end

            // --- HLT ---
            S_HLT: begin
                next_state = S_HLT;
            end

            default: next_state = S_FETCH_1;
        endcase
    end

endmodule
