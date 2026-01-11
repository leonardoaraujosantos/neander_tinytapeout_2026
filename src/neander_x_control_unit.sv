// ============================================================================
// neander_x_control_unit.sv — Control Unit for NEANDER-X CPU
// ============================================================================

module neander_control (
    input  logic       clk,
    input  logic       reset,
    input  logic [3:0] opcode,
    input  logic [3:0] sub_opcode,  // Lower nibble for stack ops
    input  logic       flagN,
    input  logic       flagZ,

    output logic       mem_read,
    output logic       mem_write,
    output logic       pc_inc,
    output logic       pc_load,
    output logic       ac_load,
    output logic       ri_load,
    output logic       rem_load,
    output logic       rdm_load,
    output logic       nz_load,
    output logic [1:0] addr_sel,    // 00=RDM, 01=PC, 10=SP
    output logic [1:0] alu_op,
    output logic       io_write,
    output logic       sp_inc,      // Stack pointer increment (POP/RET)
    output logic       sp_dec,      // Stack pointer decrement (PUSH/CALL)
    output logic       mem_data_sel // Memory data select: 0=AC, 1=PC (for CALL)
);

    // Using ENUM for states (Easier to debug in Waveforms)
    typedef enum logic [5:0] {
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
        S_POP_1,  S_POP_2,  S_POP_3,  // Stack POP
        S_CALL_1, S_CALL_2, S_CALL_3, S_CALL_4, S_CALL_5, // CALL subroutine
        S_RET_1,  S_RET_2,  S_RET_3,  // RET from subroutine
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
        mem_read    = '0;
        mem_write   = '0;
        pc_inc      = '0;
        pc_load     = '0;
        ac_load     = '0;
        ri_load     = '0;
        rem_load    = '0;
        rdm_load    = '0;
        nz_load     = '0;
        addr_sel    = 2'b01;  // Default to PC (01=PC)
        alu_op      = 2'b00;
        io_write    = '0;
        sp_inc      = '0;
        sp_dec      = '0;
        mem_data_sel = '0;    // Default to AC (0=AC)

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
                    4'h2: next_state = S_LDA_1;
                    4'h1: next_state = S_STA_1;
                    4'h3: next_state = S_ADD_1;
                    4'h5: next_state = S_AND_1;
                    4'h4: next_state = S_OR_1;
                    4'h6: next_state = S_NOT;
                    4'h7: begin  // Stack operations (sub-opcode in lower nibble)
                        case (sub_opcode)
                            4'h0: next_state = S_PUSH_1;  // PUSH (0x70)
                            4'h1: next_state = S_POP_1;   // POP  (0x71)
                            4'h2: next_state = S_CALL_1;  // CALL (0x72)
                            4'h3: next_state = S_RET_1;   // RET  (0x73)
                            default: next_state = S_FETCH_1;
                        endcase
                    end
                    4'h8: next_state = S_JMP_1;
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
                alu_op      = 2'b00;
                nz_load     = 1;
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
                mem_read = 1; ac_load = 1; alu_op = 2'b01; nz_load = 1; next_state = S_FETCH_1;
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
                mem_read = 1; ac_load = 1; alu_op = 2'b10; nz_load = 1; next_state = S_FETCH_1;
            end

            // --- NOT ---
            S_NOT: begin
                ac_load = 1; alu_op = 2'b11; nz_load = 1; next_state = S_FETCH_1;
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

            // --- HLT ---
            S_HLT: begin
                next_state = S_HLT;
            end

            default: next_state = S_FETCH_1;
        endcase
    end

endmodule
