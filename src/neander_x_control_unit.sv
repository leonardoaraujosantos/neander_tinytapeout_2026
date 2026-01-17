// ============================================================================
// neander_x_control_unit.sv — Control Unit for NEANDER-X CPU (16-bit Data Width)
// ============================================================================
// 16-bit data width: AC, X, Y registers are 16-bit, memory data bus is 16-bit
// Immediate instructions (LDI, LDXI, LDYI) fetch 16-bit values and increment PC by 2
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
//   0x0E: DIV       - AC / X -> AC (quotient), Y (remainder)
//   0x0F: MOD       - AC % X -> AC (remainder), Y (quotient)
//   0x0A: TSF       - FP = SP (transfer SP to FP)
//   0x0B: TFS       - SP = FP (transfer FP to SP)
//   0x0C: PUSH_FP   - MEM[--SP] = FP (push frame pointer)
//   0x0D: POP_FP    - FP = MEM[SP++] (pop frame pointer)
//
// Indexed Addressing with FP (sub_opcode bit 2):
//   0x24: LDA addr,FP - AC = MEM[addr + FP]
//   0x14: STA addr,FP - MEM[addr + FP] = AC
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
//
// Multi-byte Arithmetic Extension (for 16-bit and 32-bit operations):
//   0x31: ADC addr  - AC = AC + MEM[addr] + Carry (Add with Carry)
//   0x51: SBC addr  - AC = AC - MEM[addr] - Carry (Subtract with Borrow)
//   0x61: ASR       - AC = AC >> 1 (Arithmetic Shift Right, preserves sign)
// ============================================================================

module neander_control (
    input  logic       clk,
    input  logic       reset,
    input  logic [3:0] opcode,
    input  logic [3:0] sub_opcode,  // Lower nibble for stack/LCC ops and indexed mode
    input  logic       flagN,
    input  logic       flagZ,
    input  logic       flagC,       // Carry flag input (for JC/JNC)
    input  logic       mem_ready,   // Memory access complete (from SPI controller)

    output logic       mem_read,
    output logic       mem_write,
    output logic       mem_req,     // Memory access request (to SPI controller)
    output logic       pc_inc,
    output logic       pc_inc_2,    // Increment PC by 2 (for 16-bit immediate fetch)
    output logic       pc_load,
    output logic       ac_load,
    output logic       ri_load,
    output logic       rem_load,
    output logic       rdm_load,     // Load RDM low byte
    output logic       rdm_load_hi,  // Load RDM high byte (for 16-bit addresses)
    output logic       rdm_load_full, // Load full 16-bit RDM (for 16-bit memory ops)
    output logic       nz_load,
    output logic       c_load,      // Carry flag load
    output logic [1:0] addr_sel,    // 00=RDM, 01=PC, 10=SP
    output logic [3:0] alu_op,      // Extended to 4 bits for NEG
    output logic       io_write,
    output logic       sp_inc,      // Stack pointer increment (POP/RET)
    output logic       sp_dec,      // Stack pointer decrement (PUSH/CALL)
    output logic [1:0] mem_data_sel,// Memory data select: 00=AC, 01=PC, 10=X, 11=Y
    output logic [2:0] alu_b_sel,   // ALU B input select: 000=mem_data, 001=constant 1, 010=X, 011=Y
    // X Register Extension signals
    output logic       x_load,      // Load X register
    output logic       x_inc,       // Increment X register
    output logic       x_dec,       // Decrement X register (DEX)
    output logic       x_to_ac,     // Transfer X to AC (TXA)
    output logic       indexed_mode, // Use indexed addressing (addr + X)
    // Y Register Extension signals
    output logic       y_load,      // Load Y register
    output logic       y_inc,       // Increment Y register
    output logic       y_dec,       // Decrement Y register (DEY)
    output logic       y_to_ac,     // Transfer Y to AC (TYA)
    output logic       indexed_mode_y, // Use indexed addressing (addr + Y)
    output logic       mul_to_y,    // Load Y with MUL high byte
    // B Register Extension signals
    output logic       b_load,      // Load B register
    output logic       b_inc,       // Increment B register
    output logic       b_dec,       // Decrement B register (DEB)
    output logic       b_to_ac,     // Transfer B to AC (TBA)
    output logic       b_from_temp, // Load B from swap temp (for SWPB)
    // Frame Pointer Extension signals
    output logic       fp_load,      // Load FP full 16-bit (for TSF)
    output logic       fp_load_lo,   // Load FP low byte (for POP_FP)
    output logic       fp_load_hi,   // Load FP high byte (for POP_FP)
    output logic       sp_load,     // Load SP from FP (for TFS)
    output logic       indexed_mode_fp, // Use indexed addressing (addr + FP)
    output logic       indexed_mode_sp, // Use indexed addressing (addr + SP)
    output logic [2:0] mem_data_sel_ext, // Extended: 000=AC, 001=PC_LO, 010=X, 011=Y, 100=FP_LO, 101=PC_HI, 110=FP_HI
    // Swap instruction support
    output logic       swap_temp_load,   // Load swap temp register from AC
    output logic       x_from_temp,      // Load X from swap temp (for SWPX)
    output logic       y_from_temp,      // Load Y from swap temp (for SWPY)
    // Indirect addressing support
    output logic       ind_temp_load,    // Load swap_temp from mem_data_in (for indirect)
    output logic       rdm_lo_from_temp, // Load RDM low byte from swap_temp
    output logic       rdm_inc,          // Increment RDM by 1
    // Sequential divider interface (area-efficient DIV/MOD)
    output logic       div_start,        // Start sequential division
    input  logic       div_busy,         // Division in progress
    input  logic       div_done,         // Division complete (pulse)
    // Sequential multiplier interface (area-efficient MUL)
    output logic       mul_start,        // Start sequential multiplication
    input  logic       mul_busy,         // Multiplication in progress
    input  logic       mul_done          // Multiplication complete (pulse)
);

    // Using ENUM for states (Easier to debug in Waveforms)
    // Extended to 9 bits for all 16-bit addressing extension states
    typedef enum logic [8:0] {
        S_FETCH_1, S_FETCH_1B, S_FETCH_2, S_FETCH_3, S_DECODE,
        // LDA: now 6 states for 16-bit address fetch
        S_LDA_1, S_LDA_2, S_LDA_2B, S_LDA_2C, S_LDA_3, S_LDA_4,
        // STA: now 6 states for 16-bit address fetch
        S_STA_1, S_STA_2, S_STA_2B, S_STA_2C, S_STA_3, S_STA_4,
        // ADD: now 6 states for 16-bit address fetch
        S_ADD_1, S_ADD_2, S_ADD_2B, S_ADD_2C, S_ADD_3, S_ADD_4,
        // AND: now 6 states for 16-bit address fetch
        S_AND_1, S_AND_2, S_AND_2B, S_AND_2C, S_AND_3, S_AND_4,
        // OR: now 6 states for 16-bit address fetch
        S_OR_1,  S_OR_2,  S_OR_2B,  S_OR_2C,  S_OR_3,  S_OR_4,
        S_NOT,
        // JMP: now 5 states for 16-bit address fetch
        S_JMP_1, S_JMP_2, S_JMP_2B, S_JMP_2C, S_JMP_3,
        // JN: now 5 states for 16-bit address fetch
        S_JN_1,  S_JN_2,  S_JN_2B,  S_JN_2C,  S_JN_3,
        // JZ: now 5 states for 16-bit address fetch
        S_JZ_1,  S_JZ_2,  S_JZ_2B,  S_JZ_2C,  S_JZ_3,
        // JNZ: now 5 states for 16-bit address fetch
        S_JNZ_1, S_JNZ_2, S_JNZ_2B, S_JNZ_2C, S_JNZ_3,
        S_LDI_1, S_LDI_2,
        // IN: now 5 states for 16-bit port address
        S_IN_1,  S_IN_2,  S_IN_2B,  S_IN_2C,  S_IN_3,
        // OUT: now 5 states for 16-bit port address
        S_OUT_1, S_OUT_2, S_OUT_2B, S_OUT_2C, S_OUT_3,
        S_PUSH_1, S_PUSH_1B, S_PUSH_2, S_PUSH_3,  // Stack PUSH (16-bit data, SP -= 2)
        S_POP_1,  S_POP_2,  S_POP_3, S_POP_4,   // Stack POP (16-bit data, SP += 2)
        // CALL: extended for 16-bit target address and 16-bit return address push
        S_CALL_1, S_CALL_2, S_CALL_2B, S_CALL_2C, S_CALL_3, S_CALL_4, S_CALL_5, S_CALL_6, S_CALL_7,
        // RET: extended for 16-bit return address pop
        S_RET_1,  S_RET_2,  S_RET_3, S_RET_4, S_RET_5, S_RET_6,
        // LCC Extension states - SUB with 16-bit address
        S_SUB_1, S_SUB_2, S_SUB_2B, S_SUB_2C, S_SUB_3, S_SUB_4,
        S_INC,                                 // INC (0x75) - single cycle
        S_DEC,                                 // DEC (0x76) - single cycle
        // XOR with 16-bit address
        S_XOR_1, S_XOR_2, S_XOR_2B, S_XOR_2C, S_XOR_3, S_XOR_4,
        S_SHL,                                 // SHL (0x78) - single cycle
        S_SHR,                                 // SHR (0x79) - single cycle
        // X Register Extension states - LDX with 16-bit address
        S_LDX_1, S_LDX_2, S_LDX_2B, S_LDX_2C, S_LDX_3, S_LDX_4,
        // STX with 16-bit address
        S_STX_1, S_STX_2, S_STX_2B, S_STX_2C, S_STX_3, S_STX_4,
        S_LDXI_1, S_LDXI_2,                   // LDXI imm (0x7C)
        S_TAX,                                 // TAX (0x7D) - single cycle
        S_TXA,                                 // TXA (0x7E) - single cycle
        S_INX,                                 // INX (0x7F) - single cycle
        S_DEX,                                 // DEX (0x18) - single cycle
        S_DEY,                                 // DEY (0x19) - single cycle
        // Register-to-register ALU operations (single cycle)
        S_ADDX,                                // ADDX (0x32) - AC = AC + X
        S_SUBX,                                // SUBX (0x33) - AC = AC - X
        S_ADDY,                                // ADDY (0x34) - AC = AC + Y
        S_SUBY,                                // SUBY (0x35) - AC = AC - Y
        S_ORX,                                 // ORX  (0x42) - AC = AC | X
        S_ANDX,                                // ANDX (0x52) - AC = AC & X
        S_XORX,                                // XORX (0x43) - AC = AC ^ X
        // Compare Immediate
        S_CMPI_1, S_CMPI_2,                    // CMPI imm (0xE1) - compare AC with immediate
        // Multiply/Divide Immediate
        S_MULI_1, S_MULI_2, S_MULI_WAIT,       // MULI imm (0xE2) - AC = AC * imm - sequential
        S_DIVI_1, S_DIVI_2,                    // DIVI imm (0xE3) - AC = AC / imm
        // Decrement and jump if not zero - with 16-bit target address
        S_DECJNZ_1, S_DECJNZ_2, S_DECJNZ_2B, S_DECJNZ_2C, S_DECJNZ_3,
        // SP-relative addressing states - with 16-bit offset
        S_LDA_SP_1, S_LDA_SP_2, S_LDA_SP_2B, S_LDA_SP_2C, S_LDA_SP_3, S_LDA_SP_4, // LDA off,SP (0x28)
        S_STA_SP_1, S_STA_SP_2, S_STA_SP_2B, S_STA_SP_2C, S_STA_SP_3, S_STA_SP_4, // STA off,SP (0x17)
        // Swap instructions (2-cycle each)
        S_SWPX_1, S_SWPX_2,                    // SWPX (0x1A) - swap AC <-> X
        S_SWPY_1, S_SWPY_2,                    // SWPY (0x1B) - swap AC <-> Y
        // Indirect addressing states - LDA (addr) - AC = MEM[MEM[addr]]
        // 10 states: fetch ptr addr (5), fetch target addr (4), fetch data (1)
        S_LDA_IND_1, S_LDA_IND_2, S_LDA_IND_2B, S_LDA_IND_2C, S_LDA_IND_3,
        S_LDA_IND_4, S_LDA_IND_4B, S_LDA_IND_5, S_LDA_IND_5B, S_LDA_IND_6,
        // Indirect addressing states - STA (addr) - MEM[MEM[addr]] = AC
        S_STA_IND_1, S_STA_IND_2, S_STA_IND_2B, S_STA_IND_2C, S_STA_IND_3,
        S_STA_IND_4, S_STA_IND_4B, S_STA_IND_5, S_STA_IND_5B, S_STA_IND_6,
        // Post-indexed indirect addressing - LDA (addr),Y - AC = MEM[MEM[addr] + Y]
        S_LDA_IND_Y_1, S_LDA_IND_Y_2, S_LDA_IND_Y_2B, S_LDA_IND_Y_2C, S_LDA_IND_Y_3,
        S_LDA_IND_Y_4, S_LDA_IND_Y_4B, S_LDA_IND_Y_5, S_LDA_IND_Y_5B, S_LDA_IND_Y_6,
        // Indexed addressing states (X) - with 16-bit base address
        S_LDA_X_1, S_LDA_X_2, S_LDA_X_2B, S_LDA_X_2C, S_LDA_X_3, S_LDA_X_4,
        S_STA_X_1, S_STA_X_2, S_STA_X_2B, S_STA_X_2C, S_STA_X_3, S_STA_X_4,
        // LCC Compiler Extension states (NEG, CMP, JC, JNC)
        S_NEG,                                 // NEG (0x01) - single cycle
        // CMP with 16-bit address
        S_CMP_1, S_CMP_2, S_CMP_2B, S_CMP_2C, S_CMP_3, S_CMP_4,
        // JC with 16-bit address
        S_JC_1,  S_JC_2,  S_JC_2B,  S_JC_2C,  S_JC_3,
        // JNC with 16-bit address
        S_JNC_1, S_JNC_2, S_JNC_2B, S_JNC_2C, S_JNC_3,
        // Signed comparison jumps - with 16-bit address
        S_JLE_1, S_JLE_2, S_JLE_2B, S_JLE_2C, S_JLE_3,
        S_JGT_1, S_JGT_2, S_JGT_2B, S_JGT_2C, S_JGT_3,
        S_JGE_1, S_JGE_2, S_JGE_2B, S_JGE_2C, S_JGE_3,
        // Unsigned comparison jumps - with 16-bit address
        S_JBE_1, S_JBE_2, S_JBE_2B, S_JBE_2C, S_JBE_3,
        S_JA_1,  S_JA_2,  S_JA_2B,  S_JA_2C,  S_JA_3,
        // Y Register Extension states - LDY with 16-bit address
        S_LDY_1, S_LDY_2, S_LDY_2B, S_LDY_2C, S_LDY_3, S_LDY_4,
        // STY with 16-bit address
        S_STY_1, S_STY_2, S_STY_2B, S_STY_2C, S_STY_3, S_STY_4,
        S_LDYI_1, S_LDYI_2,                   // LDYI imm (0x06)
        S_TAY,                                 // TAY (0x03) - single cycle
        S_TYA,                                 // TYA (0x04) - single cycle
        S_INY,                                 // INY (0x05) - single cycle
        // Indexed addressing states (Y) - with 16-bit base address
        S_LDA_Y_1, S_LDA_Y_2, S_LDA_Y_2B, S_LDA_Y_2C, S_LDA_Y_3, S_LDA_Y_4,
        S_STA_Y_1, S_STA_Y_2, S_STA_Y_2B, S_STA_Y_2C, S_STA_Y_3, S_STA_Y_4,
        // Multiplication and Division
        S_MUL, S_MUL_WAIT,                            // MUL (0x09) - AC * X -> Y:AC - sequential
        S_DIV, S_DIV_WAIT,                            // DIV (0x0E) - AC / X -> AC (quotient), Y (remainder) - sequential
        S_MOD, S_MOD_WAIT,                            // MOD (0x0F) - AC % X -> AC (remainder), Y (quotient) - sequential
        S_DIVI_WAIT,                                  // DIVI wait state for sequential divider
        // Multi-byte arithmetic (ADC, SBC) with 16-bit address
        S_ADC_1, S_ADC_2, S_ADC_2B, S_ADC_2C, S_ADC_3, S_ADC_4,
        S_SBC_1, S_SBC_2, S_SBC_2B, S_SBC_2C, S_SBC_3, S_SBC_4,
        S_ASR,                                        // ASR (0x61) - Arithmetic Shift Right
        // Frame Pointer Extension states
        S_TSF,                                        // TSF (0x0A) - FP = SP
        S_TFS,                                        // TFS (0x0B) - SP = FP
        // PUSH_FP: extended for 16-bit FP (push low byte, then high byte)
        S_PUSH_FP_1, S_PUSH_FP_2, S_PUSH_FP_3, S_PUSH_FP_4, S_PUSH_FP_5, S_PUSH_FP_6,
        // POP_FP: extended for 16-bit FP (pop low byte, then high byte)
        S_POP_FP_1,  S_POP_FP_2,  S_POP_FP_3, S_POP_FP_4, S_POP_FP_5, S_POP_FP_6,
        // Indexed addressing states (FP) - with 16-bit base address
        S_LDA_FP_1, S_LDA_FP_2, S_LDA_FP_2B, S_LDA_FP_2C, S_LDA_FP_3, S_LDA_FP_4,
        S_STA_FP_1, S_STA_FP_2, S_STA_FP_2B, S_STA_FP_2C, S_STA_FP_3, S_STA_FP_4,
        // B Register Extension states
        S_TAB,                                 // TAB (0x1C) - single cycle
        S_TBA,                                 // TBA (0x1D) - single cycle
        S_LDB_1, S_LDB_2, S_LDB_2B, S_LDB_2C, S_LDB_3, S_LDB_4, // LDB addr (0x1E)
        S_STB_1, S_STB_2, S_STB_2B, S_STB_2C, S_STB_3, S_STB_4, // STB addr (0x1F)
        S_INB,                                 // INB (0x36) - single cycle
        S_DEB,                                 // DEB (0x37) - single cycle
        S_SWPB_1, S_SWPB_2,                    // SWPB (0x38) - swap AC <-> B
        S_ADDB,                                // ADDB (0x39) - AC = AC + B
        S_SUBB,                                // SUBB (0x3A) - AC = AC - B
        S_LDBI_1, S_LDBI_2,                    // LDBI imm (0xE4)
        // PUSH_ADDR and POP_ADDR states
        S_PUSH_ADDR_1, S_PUSH_ADDR_2, S_PUSH_ADDR_2B, S_PUSH_ADDR_2C, S_PUSH_ADDR_3, S_PUSH_ADDR_4, S_PUSH_ADDR_5, S_PUSH_ADDR_6, S_PUSH_ADDR_7, // PUSH_ADDR (0x89)
        S_POP_ADDR_1, S_POP_ADDR_2, S_POP_ADDR_2B, S_POP_ADDR_2C, S_POP_ADDR_3, S_POP_ADDR_4, S_POP_ADDR_5, S_POP_ADDR_6, S_POP_ADDR_7, // POP_ADDR (0x8A)
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
        mem_req      = '0;  // Memory request default (SPI handshaking)
        pc_inc       = '0;
        pc_inc_2     = '0;  // PC increment by 2 for 16-bit immediates
        pc_load      = '0;
        ac_load      = '0;
        ri_load      = '0;
        rem_load     = '0;
        rdm_load     = '0;
        rdm_load_hi  = '0;  // Load RDM high byte (for 16-bit addresses)
        rdm_load_full = '0; // Load full 16-bit RDM (for 16-bit memory ops)
        nz_load      = '0;
        c_load       = '0;     // Carry flag load default
        addr_sel     = 2'b01;  // Default to PC (01=PC)
        alu_op       = 4'b0000; // Default to ADD (4 bits now)
        io_write     = '0;
        sp_inc       = '0;
        sp_dec       = '0;
        mem_data_sel = 2'b00;  // Default to AC (00=AC)
        alu_b_sel    = 3'b000; // Default to mem_data (000=mem_data)
        // X Register Extension defaults
        x_load       = '0;
        x_inc        = '0;
        x_dec        = '0;
        x_to_ac      = '0;
        indexed_mode = '0;
        // Y Register Extension defaults
        y_load       = '0;
        y_inc        = '0;
        y_dec        = '0;
        y_to_ac      = '0;
        indexed_mode_y = '0;
        mul_to_y     = '0;
        // B Register Extension defaults
        b_load       = '0;
        b_inc        = '0;
        b_dec        = '0;
        b_to_ac      = '0;
        b_from_temp  = '0;
        // Frame Pointer Extension defaults
        fp_load      = '0;
        fp_load_lo   = '0;  // Load FP low byte (for POP_FP)
        fp_load_hi   = '0;  // Load FP high byte (for POP_FP)
        sp_load      = '0;
        indexed_mode_fp = '0;
        indexed_mode_sp = '0;
        mem_data_sel_ext = 3'b000;  // Default to AC
        // Swap instruction defaults
        swap_temp_load = '0;
        x_from_temp    = '0;
        y_from_temp    = '0;
        // Indirect addressing defaults
        ind_temp_load    = '0;
        rdm_lo_from_temp = '0;
        rdm_inc          = '0;
        // Sequential divider defaults
        div_start        = '0;
        // Sequential multiplier defaults
        mul_start        = '0;

        next_state  = state;

        case (state)
            // --- FETCH ---
            // S_FETCH_1: Setup address (PC -> REM)
            S_FETCH_1: begin
                addr_sel    = 2'b01; // PC -> REM
                rem_load    = 1;
                next_state  = S_FETCH_1B;  // Wait for REM to be stable
            end
            // S_FETCH_1B: REM is now stable with PC value
            S_FETCH_1B: begin
                next_state  = S_FETCH_2;
            end
            // S_FETCH_2: Request memory read, wait for ready
            S_FETCH_2: begin
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load    = 1;
                    next_state  = S_FETCH_3;
                end else begin
                    mem_req     = 1;  // Only request when NOT ready (prevents double-fetch)
                    next_state  = S_FETCH_2;  // Wait for SPI
                end
            end
            S_FETCH_3: begin
                ri_load     = 1;
                pc_inc      = 1;
                next_state  = S_DECODE;
            end

            // --- DECODE ---
            S_DECODE: begin
                case (opcode)
                    4'h0: begin  // NOP family + LCC extensions (NEG, CMP) + Y register ops + MUL + FP ops
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
                            4'hA: next_state = S_TSF;      // TSF (0x0A) - FP = SP
                            4'hB: next_state = S_TFS;      // TFS (0x0B) - SP = FP
                            4'hC: next_state = S_PUSH_FP_1; // PUSH_FP (0x0C)
                            4'hD: next_state = S_POP_FP_1;  // POP_FP (0x0D)
                            4'hE: next_state = S_DIV;      // DIV (0x0E) - AC / X -> AC, Y
                            4'hF: next_state = S_MOD;      // MOD (0x0F) - AC % X -> AC, Y
                            default: next_state = S_FETCH_1;
                        endcase
                    end
                    4'h2: begin
                        // LDA: check sub_opcode for indexed mode and indirect
                        case (sub_opcode)
                            4'h0: next_state = S_LDA_1;       // LDA addr (0x20)
                            4'h1: next_state = S_LDA_X_1;     // LDA addr,X (0x21)
                            4'h2: next_state = S_LDA_Y_1;     // LDA addr,Y (0x22)
                            4'h4: next_state = S_LDA_FP_1;    // LDA addr,FP (0x24)
                            4'h6: next_state = S_LDA_IND_1;   // LDA (addr) (0x26) - indirect
                            4'h7: next_state = S_LDA_IND_Y_1; // LDA (addr),Y (0x27) - post-indexed indirect
                            4'h8: next_state = S_LDA_SP_1;    // LDA off,SP (0x28)
                            default: next_state = S_LDA_1;
                        endcase
                    end
                    4'h1: begin
                        // STA: check sub_opcode for indexed mode and indirect
                        // Also DEX/DEY/SWPX/SWPY/TAB/TBA/LDB/STB in this opcode family
                        case (sub_opcode)
                            4'h0: next_state = S_STA_1;     // STA addr (0x10)
                            4'h1: next_state = S_STA_X_1;   // STA addr,X (0x11)
                            4'h2: next_state = S_STA_Y_1;   // STA addr,Y (0x12)
                            4'h4: next_state = S_STA_FP_1;  // STA addr,FP (0x14)
                            4'h6: next_state = S_STA_IND_1; // STA (addr) (0x16) - indirect
                            4'h7: next_state = S_STA_SP_1;  // STA off,SP (0x17)
                            4'h8: next_state = S_DEX;       // DEX (0x18)
                            4'h9: next_state = S_DEY;       // DEY (0x19)
                            4'hA: next_state = S_SWPX_1;    // SWPX (0x1A) - Swap AC <-> X
                            4'hB: next_state = S_SWPY_1;    // SWPY (0x1B) - Swap AC <-> Y
                            4'hC: next_state = S_TAB;       // TAB (0x1C) - B = AC
                            4'hD: next_state = S_TBA;       // TBA (0x1D) - AC = B
                            4'hE: next_state = S_LDB_1;     // LDB addr (0x1E)
                            4'hF: next_state = S_STB_1;     // STB addr (0x1F)
                            default: next_state = S_STA_1;
                        endcase
                    end
                    4'h3: begin  // ADD family: 0x30=ADD, 0x31=ADC, 0x32=ADDX, 0x33=SUBX, 0x34=ADDY, 0x35=SUBY, 0x36=INB, 0x37=DEB, 0x38=SWPB, 0x39=ADDB, 0x3A=SUBB
                        case (sub_opcode)
                            4'h0: next_state = S_ADD_1;   // ADD addr (0x30)
                            4'h1: next_state = S_ADC_1;   // ADC addr (0x31)
                            4'h2: next_state = S_ADDX;    // ADDX (0x32) - AC = AC + X
                            4'h3: next_state = S_SUBX;    // SUBX (0x33) - AC = AC - X
                            4'h4: next_state = S_ADDY;    // ADDY (0x34) - AC = AC + Y
                            4'h5: next_state = S_SUBY;    // SUBY (0x35) - AC = AC - Y
                            4'h6: next_state = S_INB;     // INB (0x36) - B = B + 1
                            4'h7: next_state = S_DEB;     // DEB (0x37) - B = B - 1
                            4'h8: next_state = S_SWPB_1;  // SWPB (0x38) - Swap AC <-> B
                            4'h9: next_state = S_ADDB;    // ADDB (0x39) - AC = AC + B
                            4'hA: next_state = S_SUBB;    // SUBB (0x3A) - AC = AC - B
                            default: next_state = S_ADD_1;
                        endcase
                    end
                    4'h5: begin  // AND family: 0x50=AND, 0x51=SBC, 0x52=ANDX
                        case (sub_opcode)
                            4'h0: next_state = S_AND_1;   // AND addr (0x50)
                            4'h1: next_state = S_SBC_1;   // SBC addr (0x51)
                            4'h2: next_state = S_ANDX;    // ANDX (0x52) - AC = AC & X
                            default: next_state = S_AND_1;
                        endcase
                    end
                    4'h4: begin  // OR family: 0x40=OR, 0x42=ORX, 0x43=XORX
                        case (sub_opcode)
                            4'h0: next_state = S_OR_1;    // OR addr (0x40)
                            4'h2: next_state = S_ORX;     // ORX (0x42) - AC = AC | X
                            4'h3: next_state = S_XORX;    // XORX (0x43) - AC = AC ^ X
                            default: next_state = S_OR_1;
                        endcase
                    end
                    4'h6: begin  // NOT family: 0x60=NOT, 0x61=ASR
                        if (sub_opcode[0])
                            next_state = S_ASR;    // ASR (0x61)
                        else
                            next_state = S_NOT;    // NOT (0x60)
                    end
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
                    4'h8: begin  // JMP family + carry-based jumps + comparison jumps + DECJNZ + PUSH_ADDR/POP_ADDR
                        case (sub_opcode)
                            4'h0: next_state = S_JMP_1;     // JMP  (0x80)
                            4'h1: next_state = S_JC_1;      // JC   (0x81)
                            4'h2: next_state = S_JNC_1;     // JNC  (0x82)
                            // Signed comparison jumps (after CMP)
                            4'h3: next_state = S_JLE_1;     // JLE  (0x83)
                            4'h4: next_state = S_JGT_1;     // JGT  (0x84)
                            4'h5: next_state = S_JGE_1;     // JGE  (0x85)
                            // Unsigned comparison jumps (after CMP)
                            4'h6: next_state = S_JBE_1;     // JBE  (0x86)
                            4'h7: next_state = S_JA_1;      // JA   (0x87)
                            4'h8: next_state = S_DECJNZ_1;  // DECJNZ (0x88)
                            4'h9: next_state = S_PUSH_ADDR_1; // PUSH_ADDR (0x89) - MEM[--SP] = MEM[addr]
                            4'hA: next_state = S_POP_ADDR_1;  // POP_ADDR (0x8A) - MEM[addr] = MEM[SP++]
                            default: next_state = S_JMP_1;  // Default to JMP for backward compat
                        endcase
                    end
                    4'h9: next_state = S_JN_1;
                    4'hA: next_state = S_JZ_1;
                    4'hB: next_state = S_JNZ_1;
                    4'hE: begin  // LDI family: 0xE0=LDI, 0xE1=CMPI, 0xE2=MULI, 0xE3=DIVI, 0xE4=LDBI
                        case (sub_opcode)
                            4'h0: next_state = S_LDI_1;    // LDI imm (0xE0)
                            4'h1: next_state = S_CMPI_1;   // CMPI imm (0xE1)
                            4'h2: next_state = S_MULI_1;   // MULI imm (0xE2)
                            4'h3: next_state = S_DIVI_1;   // DIVI imm (0xE3)
                            4'h4: next_state = S_LDBI_1;   // LDBI imm (0xE4) - B = immediate
                            default: next_state = S_LDI_1;
                        endcase
                    end
                    4'hC: next_state = S_IN_1;
                    4'hD: next_state = S_OUT_1;
                    4'hF: next_state = S_HLT;
                    default: next_state = S_FETCH_1;
                endcase
            end

            // --- LDI --- (16-bit immediate)
            S_LDI_1: begin
                addr_sel    = 2'b01;
                rem_load    = 1;
                next_state  = S_LDI_2;
            end
            S_LDI_2: begin
                mem_read    = 1;
                if (mem_ready) begin
                    ac_load     = 1;
                    nz_load     = 1;
                    pc_inc_2    = 1;  // Increment PC by 2 (consumed 2 bytes for 16-bit immediate)
                    next_state  = S_FETCH_1;
                end else begin
                    mem_req     = 1;
                    next_state  = S_LDI_2;  // Wait for SPI
                end
            end

            // --- LDA --- (16-bit address fetch)
            S_LDA_1: begin
                addr_sel    = 2'b01;  // PC -> REM
                rem_load    = 1;
                next_state  = S_LDA_2;
            end
            S_LDA_2: begin
                // Fetch addr_lo byte
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load    = 1;  // Load low byte of address
                    pc_inc      = 1;
                    next_state  = S_LDA_2B;
                end else begin
                    mem_req     = 1;
                    next_state  = S_LDA_2;
                end
            end
            S_LDA_2B: begin
                addr_sel    = 2'b01;  // PC -> REM (for addr_hi)
                rem_load    = 1;
                next_state  = S_LDA_2C;
            end
            S_LDA_2C: begin
                // Fetch addr_hi byte
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load_hi = 1;  // Load high byte of address
                    pc_inc      = 1;
                    next_state  = S_LDA_3;
                end else begin
                    mem_req     = 1;
                    next_state  = S_LDA_2C;
                end
            end
            S_LDA_3: begin
                addr_sel    = 2'b00; // RDM (16-bit) -> REM
                rem_load    = 1;
                next_state  = S_LDA_4;
            end
            S_LDA_4: begin
                mem_read    = 1;
                if (mem_ready) begin
                    ac_load     = 1;
                    nz_load     = 1;
                    next_state  = S_FETCH_1;
                end else begin
                    mem_req     = 1;
                    next_state  = S_LDA_4;
                end
            end

            // --- STA --- (16-bit address fetch)
            S_STA_1: begin
                addr_sel    = 2'b01;  // PC -> REM
                rem_load    = 1;
                next_state  = S_STA_2;
            end
            S_STA_2: begin
                // Fetch addr_lo byte
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load    = 1;  // Load low byte of address
                    pc_inc      = 1;
                    next_state  = S_STA_2B;
                end else begin
                    mem_req     = 1;
                    next_state  = S_STA_2;
                end
            end
            S_STA_2B: begin
                addr_sel    = 2'b01;  // PC -> REM (for addr_hi)
                rem_load    = 1;
                next_state  = S_STA_2C;
            end
            S_STA_2C: begin
                // Fetch addr_hi byte
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load_hi = 1;  // Load high byte of address
                    pc_inc      = 1;
                    next_state  = S_STA_3;
                end else begin
                    mem_req     = 1;
                    next_state  = S_STA_2C;
                end
            end
            S_STA_3: begin
                addr_sel    = 2'b00;  // RDM (16-bit) -> REM
                rem_load    = 1;
                next_state  = S_STA_4;
            end
            S_STA_4: begin
                mem_write   = 1;
                if (mem_ready) begin
                    next_state  = S_FETCH_1;
                end else begin
                    mem_req     = 1;
                    next_state  = S_STA_4;
                end
            end

            // --- ADD --- (16-bit address fetch)
            S_ADD_1: begin
                addr_sel    = 2'b01;  // PC -> REM
                rem_load    = 1;
                next_state  = S_ADD_2;
            end
            S_ADD_2: begin
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load    = 1;
                    pc_inc      = 1;
                    next_state  = S_ADD_2B;
                end else begin
                    mem_req     = 1;
                    next_state  = S_ADD_2;
                end
            end
            S_ADD_2B: begin
                addr_sel    = 2'b01;  // PC -> REM (for addr_hi)
                rem_load    = 1;
                next_state  = S_ADD_2C;
            end
            S_ADD_2C: begin
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load_hi = 1;
                    pc_inc      = 1;
                    next_state  = S_ADD_3;
                end else begin
                    mem_req     = 1;
                    next_state  = S_ADD_2C;
                end
            end
            S_ADD_3: begin
                addr_sel    = 2'b00;  // RDM -> REM
                rem_load    = 1;
                next_state  = S_ADD_4;
            end
            S_ADD_4: begin
                mem_read    = 1;
                if (mem_ready) begin
                    ac_load     = 1;
                    alu_op      = 4'b0000;  // ADD
                    nz_load     = 1;
                    c_load      = 1;
                    next_state  = S_FETCH_1;
                end else begin
                    mem_req     = 1;
                    next_state  = S_ADD_4;
                end
            end

            // --- AND (16-bit address) ---
            S_AND_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_AND_2;
            end
            S_AND_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_AND_2B; end
                else begin mem_req = 1; next_state = S_AND_2; end
            end
            S_AND_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_AND_2C;
            end
            S_AND_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_AND_3; end
                else begin mem_req = 1; next_state = S_AND_2C; end
            end
            S_AND_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_AND_4;
            end
            S_AND_4: begin
                mem_read = 1;
                if (mem_ready) begin ac_load = 1; alu_op = 4'b0010; nz_load = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_AND_4; end
            end

            // --- OR (16-bit address) ---
            S_OR_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_OR_2;
            end
            S_OR_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_OR_2B; end
                else begin mem_req = 1; next_state = S_OR_2; end
            end
            S_OR_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_OR_2C;
            end
            S_OR_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_OR_3; end
                else begin mem_req = 1; next_state = S_OR_2C; end
            end
            S_OR_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_OR_4;
            end
            S_OR_4: begin
                mem_read = 1;
                if (mem_ready) begin ac_load = 1; alu_op = 4'b0011; nz_load = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_OR_4; end
            end

            // --- NOT ---
            S_NOT: begin
                ac_load = 1; alu_op = 4'b0101; nz_load = 1; next_state = S_FETCH_1;  // NOT
            end

            // --- JMP --- (16-bit address)
            S_JMP_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_JMP_2;
            end
            S_JMP_2: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_JMP_2B; end
                else begin mem_req = 1; next_state = S_JMP_2; end
            end
            S_JMP_2B: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_JMP_2C;
            end
            S_JMP_2C: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; next_state = S_JMP_3; end
                else begin mem_req = 1; next_state = S_JMP_2C; end
            end
            S_JMP_3: begin
                pc_load = 1; next_state = S_FETCH_1;
            end

            // --- JN --- (16-bit address)
            S_JN_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_JN_2;
            end
            S_JN_2: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_JN_2B; end
                else begin mem_req = 1; next_state = S_JN_2; end
            end
            S_JN_2B: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_JN_2C;
            end
            S_JN_2C: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_JN_3; end
                else begin mem_req = 1; next_state = S_JN_2C; end
            end
            S_JN_3: begin
                if (flagN) pc_load = 1;
                // PC already points to next instruction after fetching both address bytes
                next_state = S_FETCH_1;
            end

            // --- JZ (16-bit address) ---
            S_JZ_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_JZ_2;
            end
            S_JZ_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_JZ_2B; end
                else begin mem_req = 1; next_state = S_JZ_2; end
            end
            S_JZ_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_JZ_2C;
            end
            S_JZ_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_JZ_3; end
                else begin mem_req = 1; next_state = S_JZ_2C; end
            end
            S_JZ_3: begin
                // If Z flag set, load PC from RDM (16-bit); else PC already incremented
                if (flagZ) pc_load = 1;
                next_state = S_FETCH_1;
            end

            // --- JNZ (16-bit address) ---
            S_JNZ_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_JNZ_2;
            end
            S_JNZ_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_JNZ_2B; end
                else begin mem_req = 1; next_state = S_JNZ_2; end
            end
            S_JNZ_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_JNZ_2C;
            end
            S_JNZ_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_JNZ_3; end
                else begin mem_req = 1; next_state = S_JNZ_2C; end
            end
            S_JNZ_3: begin
                // If Z flag NOT set, load PC from RDM (16-bit); else PC already incremented
                if (!flagZ) pc_load = 1;
                next_state  = S_FETCH_1;
            end

            // --- IN ---
            S_IN_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_IN_2;
            end
            S_IN_2: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_IN_3; end
                else begin mem_req = 1; next_state = S_IN_2; end
            end
            S_IN_3: begin
                ac_load = 1; nz_load = 1; next_state = S_FETCH_1;
            end

            // --- OUT ---
            S_OUT_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_OUT_2;
            end
            S_OUT_2: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_OUT_3; end
                else begin mem_req = 1; next_state = S_OUT_2; end
            end
            S_OUT_3: begin
                io_write = 1; next_state = S_FETCH_1;
            end

            // --- PUSH (0x70) ---
            // Push AC onto stack: decrement SP by 2, load SP to REM, write AC to [SP]
            // 16-bit data requires SP -= 2 to avoid overlap with previous stack entries
            S_PUSH_1: begin
                sp_dec = 1;  // Decrement SP first (first of 2)
                next_state = S_PUSH_1B;
            end
            S_PUSH_1B: begin
                sp_dec = 1;  // Decrement SP (second of 2, now SP -= 2 total)
                next_state = S_PUSH_2;
            end
            S_PUSH_2: begin
                addr_sel = 2'b10;  // SP -> REM (load new SP value into REM)
                rem_load = 1;
                next_state = S_PUSH_3;
            end
            S_PUSH_3: begin
                mem_write = 1;
                if (mem_ready) begin next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_PUSH_3; end
            end

            // --- POP (0x71) ---
            // Pop from stack to AC: read [SP] to AC, then increment SP by 2
            // 16-bit data requires SP += 2 to properly advance past the 16-bit value
            S_POP_1: begin
                addr_sel = 2'b10;  // SP -> REM
                rem_load = 1;
                next_state = S_POP_2;
            end
            S_POP_2: begin
                mem_read = 1; mem_req = 1;
                next_state = S_POP_3;
            end
            S_POP_3: begin
                mem_read = 1;
                if (mem_ready) begin
                    ac_load = 1;       // Load data into AC
                    nz_load = 1;       // Update flags
                    sp_inc = 1;        // Increment SP (first of 2)
                    next_state = S_POP_4;
                end else begin
                    mem_req = 1;
                    next_state = S_POP_3;
                end
            end
            S_POP_4: begin
                sp_inc = 1;            // Increment SP (second of 2, now SP += 2 total)
                next_state = S_FETCH_1;
            end

            // --- CALL (0x72 addr) --- 16-bit target address, 16-bit return address push
            // With 16-bit memory, push full PC in one 16-bit write operation
            S_CALL_1: begin
                addr_sel = 2'b01;  // PC -> REM (fetch target address)
                rem_load = 1;
                next_state = S_CALL_2;
            end
            S_CALL_2: begin
                // Read 16-bit target address in one operation
                mem_read = 1;
                if (mem_ready) begin
                    rdm_load_full = 1; // Load full 16-bit target address from SPI
                    pc_inc_2 = 1;      // Skip past 2-byte address
                    next_state = S_CALL_3;
                end else begin
                    mem_req = 1;
                    next_state = S_CALL_2;
                end
            end
            S_CALL_2B: begin
                // Unused - kept for state enum compatibility
                next_state = S_FETCH_1;
            end
            S_CALL_2C: begin
                // Unused - kept for state enum compatibility
                next_state = S_FETCH_1;
            end
            S_CALL_3: begin
                sp_dec = 1;        // Decrement SP by 1 (first of 2)
                next_state = S_CALL_4;
            end
            S_CALL_4: begin
                sp_dec = 1;        // Decrement SP by 1 (second of 2, now SP -= 2 total)
                next_state = S_CALL_5;
            end
            S_CALL_5: begin
                addr_sel = 2'b10;  // SP -> REM
                rem_load = 1;
                next_state = S_CALL_6;
            end
            S_CALL_6: begin
                // Write 16-bit PC to stack in one operation
                mem_write = 1;
                mem_data_sel_ext = 3'b001;  // Select PC as data source (full 16-bit)
                if (mem_ready) begin
                    pc_load = 1;       // Load target address from RDM to PC
                    next_state = S_FETCH_1;
                end else begin
                    mem_req = 1;
                    next_state = S_CALL_6;
                end
            end
            S_CALL_7: begin
                // Unused - kept for state enum compatibility
                next_state = S_FETCH_1;
            end

            // --- RET (0x73) --- 16-bit return address pop
            // With 16-bit memory, pop full PC in one 16-bit read operation
            S_RET_1: begin
                addr_sel = 2'b10;  // SP -> REM
                rem_load = 1;
                next_state = S_RET_2;
            end
            S_RET_2: begin
                // Read 16-bit return address in one operation
                mem_read = 1;
                if (mem_ready) begin
                    rdm_load_full = 1; // Load full 16-bit return address from SPI
                    sp_inc = 1;        // Increment SP by 1 (first of 2)
                    next_state = S_RET_3;
                end else begin
                    mem_req = 1;
                    next_state = S_RET_2;
                end
            end
            S_RET_3: begin
                sp_inc = 1;        // Increment SP by 1 (second of 2, now SP += 2 total)
                next_state = S_RET_4;
            end
            S_RET_4: begin
                pc_load = 1;       // Load return address from RDM to PC
                next_state = S_FETCH_1;
            end
            S_RET_5: begin
                // Unused - kept for state enum compatibility
                next_state = S_FETCH_1;
            end
            S_RET_6: begin
                // Unused - kept for state enum compatibility
                next_state = S_FETCH_1;
            end

            // ================================================================
            // LCC EXTENSION INSTRUCTIONS
            // ================================================================

            // --- SUB addr (0x74) --- 16-bit address
            // Subtract memory from AC: AC = AC - MEM[addr]
            S_SUB_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_SUB_2;
            end
            S_SUB_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_SUB_2B; end
                else begin mem_req = 1; next_state = S_SUB_2; end
            end
            S_SUB_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_SUB_2C;
            end
            S_SUB_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_SUB_3; end
                else begin mem_req = 1; next_state = S_SUB_2C; end
            end
            S_SUB_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_SUB_4;
            end
            S_SUB_4: begin
                mem_read = 1;
                if (mem_ready) begin ac_load = 1; alu_op = 4'b0001; nz_load = 1; c_load = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_SUB_4; end
            end

            // --- INC (0x75) ---
            // Increment AC: AC = AC + 1 (single cycle)
            S_INC: begin
                ac_load = 1;
                alu_op = 4'b0000;   // ADD
                alu_b_sel = 3'b001;  // Select constant 1 as ALU B input
                nz_load = 1;
                next_state = S_FETCH_1;
            end

            // --- DEC (0x76) ---
            // Decrement AC: AC = AC - 1 (single cycle)
            S_DEC: begin
                ac_load = 1;
                alu_op = 4'b0001;   // SUB
                alu_b_sel = 3'b001;  // Select constant 1 as ALU B input
                nz_load = 1;
                next_state = S_FETCH_1;
            end

            // --- XOR addr (0x77) --- 16-bit address
            // XOR memory with AC: AC = AC ^ MEM[addr]
            S_XOR_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_XOR_2;
            end
            S_XOR_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_XOR_2B; end
                else begin mem_req = 1; next_state = S_XOR_2; end
            end
            S_XOR_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_XOR_2C;
            end
            S_XOR_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_XOR_3; end
                else begin mem_req = 1; next_state = S_XOR_2C; end
            end
            S_XOR_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_XOR_4;
            end
            S_XOR_4: begin
                mem_read = 1;
                if (mem_ready) begin ac_load = 1; alu_op = 4'b0100; nz_load = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_XOR_4; end
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
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDX_2;
            end
            S_LDX_2: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_LDX_3; end
                else begin mem_req = 1; next_state = S_LDX_2; end
            end
            S_LDX_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_LDX_4;
            end
            S_LDX_4: begin
                mem_read = 1;
                if (mem_ready) begin x_load = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_LDX_4; end
            end

            // --- STX addr (0x7B) ---
            // Store X to memory: MEM[addr] = X
            S_STX_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_STX_2;
            end
            S_STX_2: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_STX_3; end
                else begin mem_req = 1; next_state = S_STX_2; end
            end
            S_STX_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_STX_4;
            end
            S_STX_4: begin
                mem_write = 1;
                mem_data_sel_ext = 3'b010;  // Select X as data source
                if (mem_ready) begin next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_STX_4; end
            end

            // --- LDXI imm (0x7C) --- (16-bit immediate)
            // Load X with immediate: X = imm (16-bit)
            S_LDXI_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDXI_2;
            end
            S_LDXI_2: begin
                mem_read = 1;
                if (mem_ready) begin x_load = 1; pc_inc_2 = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_LDXI_2; end
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

            // --- DEX (0x18) ---
            // Decrement X: X = X - 1 (single cycle)
            S_DEX: begin
                x_dec = 1;         // Decrement X register
                next_state = S_FETCH_1;
            end

            // --- DEY (0x19) ---
            // Decrement Y: Y = Y - 1 (single cycle)
            S_DEY: begin
                y_dec = 1;         // Decrement Y register
                next_state = S_FETCH_1;
            end

            // ================================================================
            // SWAP INSTRUCTIONS
            // ================================================================

            // --- SWPX (0x1A) ---
            // Swap AC and X: temp = AC; AC = X; X = temp (2 cycles)
            S_SWPX_1: begin
                swap_temp_load = 1;  // Save AC to temp register
                x_to_ac = 1;         // Load X value to AC
                ac_load = 1;         // Enable AC load
                nz_load = 1;         // Update N and Z flags based on new AC value
                next_state = S_SWPX_2;
            end
            S_SWPX_2: begin
                x_from_temp = 1;     // Load X from temp (original AC value)
                x_load = 1;          // Enable X load
                next_state = S_FETCH_1;
            end

            // --- SWPY (0x1B) ---
            // Swap AC and Y: temp = AC; AC = Y; Y = temp (2 cycles)
            S_SWPY_1: begin
                swap_temp_load = 1;  // Save AC to temp register
                y_to_ac = 1;         // Load Y value to AC
                ac_load = 1;         // Enable AC load
                nz_load = 1;         // Update N and Z flags based on new AC value
                next_state = S_SWPY_2;
            end
            S_SWPY_2: begin
                y_from_temp = 1;     // Load Y from temp (original AC value)
                y_load = 1;          // Enable Y load
                next_state = S_FETCH_1;
            end

            // ================================================================
            // INDIRECT ADDRESSING OPERATIONS
            // ================================================================

            // --- LDA (addr) --- Indirect: AC = MEM[MEM[addr]]
            // Steps:
            // 1-4: Fetch 16-bit pointer address from operand
            // 5-8: Fetch 16-bit target address from pointer location
            // 9: Fetch data from target address into AC
            S_LDA_IND_1: begin
                addr_sel    = 2'b01;  // PC -> REM
                rem_load    = 1;
                next_state  = S_LDA_IND_2;
            end
            S_LDA_IND_2: begin
                // Fetch pointer address low byte
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load    = 1;  // Store ptr_addr_lo in RDM[7:0]
                    pc_inc      = 1;
                    next_state  = S_LDA_IND_2B;
                end else begin
                    mem_req     = 1;
                    next_state  = S_LDA_IND_2;
                end
            end
            S_LDA_IND_2B: begin
                addr_sel    = 2'b01;  // PC -> REM
                rem_load    = 1;
                next_state  = S_LDA_IND_2C;
            end
            S_LDA_IND_2C: begin
                // Fetch pointer address high byte
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load_hi = 1;  // Store ptr_addr_hi in RDM[15:8]
                    pc_inc      = 1;
                    next_state  = S_LDA_IND_3;
                end else begin
                    mem_req     = 1;
                    next_state  = S_LDA_IND_2C;
                end
            end
            S_LDA_IND_3: begin
                // RDM now holds pointer address, setup to fetch target addr low byte
                addr_sel    = 2'b00;  // RDM -> REM
                rem_load    = 1;
                next_state  = S_LDA_IND_4;
            end
            S_LDA_IND_4: begin
                // Fetch target address low byte from MEM[pointer]
                mem_read    = 1;
                if (mem_ready) begin
                    ind_temp_load = 1;  // Store target_addr_lo in swap_temp
                    next_state    = S_LDA_IND_4B;
                end else begin
                    mem_req     = 1;
                    next_state  = S_LDA_IND_4;
                end
            end
            S_LDA_IND_4B: begin
                // Increment RDM to point to high byte of target address
                rdm_inc     = 1;
                next_state  = S_LDA_IND_5;
            end
            S_LDA_IND_5: begin
                // Setup RDM+1 -> REM for fetching target addr high byte
                addr_sel    = 2'b00;  // RDM -> REM (now incremented)
                rem_load    = 1;
                next_state  = S_LDA_IND_5B;
            end
            S_LDA_IND_5B: begin
                // Fetch target address high byte, restore low byte from temp
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load_hi      = 1;  // Store target_addr_hi in RDM[15:8]
                    rdm_lo_from_temp = 1;  // Restore target_addr_lo from swap_temp to RDM[7:0]
                    next_state       = S_LDA_IND_6;
                end else begin
                    mem_req     = 1;
                    next_state  = S_LDA_IND_5B;
                end
            end
            S_LDA_IND_6: begin
                // RDM now holds target address, setup and fetch data
                addr_sel    = 2'b00;  // RDM -> REM
                rem_load    = 1;
                mem_read    = 1;
                if (mem_ready) begin
                    ac_load     = 1;
                    nz_load     = 1;
                    next_state  = S_FETCH_1;
                end else begin
                    mem_req     = 1;
                    next_state  = S_LDA_IND_6;
                end
            end

            // --- STA (addr) --- Indirect: MEM[MEM[addr]] = AC
            // Same as LDA (addr) but writes AC to target address
            S_STA_IND_1: begin
                addr_sel    = 2'b01;  // PC -> REM
                rem_load    = 1;
                next_state  = S_STA_IND_2;
            end
            S_STA_IND_2: begin
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load    = 1;
                    pc_inc      = 1;
                    next_state  = S_STA_IND_2B;
                end else begin
                    mem_req     = 1;
                    next_state  = S_STA_IND_2;
                end
            end
            S_STA_IND_2B: begin
                addr_sel    = 2'b01;
                rem_load    = 1;
                next_state  = S_STA_IND_2C;
            end
            S_STA_IND_2C: begin
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load_hi = 1;
                    pc_inc      = 1;
                    next_state  = S_STA_IND_3;
                end else begin
                    mem_req     = 1;
                    next_state  = S_STA_IND_2C;
                end
            end
            S_STA_IND_3: begin
                addr_sel    = 2'b00;
                rem_load    = 1;
                next_state  = S_STA_IND_4;
            end
            S_STA_IND_4: begin
                mem_read    = 1;
                if (mem_ready) begin
                    ind_temp_load = 1;
                    next_state    = S_STA_IND_4B;
                end else begin
                    mem_req     = 1;
                    next_state  = S_STA_IND_4;
                end
            end
            S_STA_IND_4B: begin
                rdm_inc     = 1;
                next_state  = S_STA_IND_5;
            end
            S_STA_IND_5: begin
                addr_sel    = 2'b00;
                rem_load    = 1;
                next_state  = S_STA_IND_5B;
            end
            S_STA_IND_5B: begin
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load_hi      = 1;
                    rdm_lo_from_temp = 1;
                    next_state       = S_STA_IND_6;
                end else begin
                    mem_req     = 1;
                    next_state  = S_STA_IND_5B;
                end
            end
            S_STA_IND_6: begin
                // RDM holds target address, write AC to it
                addr_sel    = 2'b00;
                rem_load    = 1;
                mem_write   = 1;
                mem_data_sel = 2'b00;  // AC
                if (mem_ready) begin
                    next_state  = S_FETCH_1;
                end else begin
                    mem_req     = 1;
                    next_state  = S_STA_IND_6;
                end
            end

            // --- LDA (addr),Y --- Post-indexed indirect: AC = MEM[MEM[addr] + Y]
            // Same as LDA (addr) but adds Y to target address before fetch
            S_LDA_IND_Y_1: begin
                addr_sel    = 2'b01;
                rem_load    = 1;
                next_state  = S_LDA_IND_Y_2;
            end
            S_LDA_IND_Y_2: begin
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load    = 1;
                    pc_inc      = 1;
                    next_state  = S_LDA_IND_Y_2B;
                end else begin
                    mem_req     = 1;
                    next_state  = S_LDA_IND_Y_2;
                end
            end
            S_LDA_IND_Y_2B: begin
                addr_sel    = 2'b01;
                rem_load    = 1;
                next_state  = S_LDA_IND_Y_2C;
            end
            S_LDA_IND_Y_2C: begin
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load_hi = 1;
                    pc_inc      = 1;
                    next_state  = S_LDA_IND_Y_3;
                end else begin
                    mem_req     = 1;
                    next_state  = S_LDA_IND_Y_2C;
                end
            end
            S_LDA_IND_Y_3: begin
                addr_sel    = 2'b00;
                rem_load    = 1;
                next_state  = S_LDA_IND_Y_4;
            end
            S_LDA_IND_Y_4: begin
                mem_read    = 1;
                if (mem_ready) begin
                    ind_temp_load = 1;
                    next_state    = S_LDA_IND_Y_4B;
                end else begin
                    mem_req     = 1;
                    next_state  = S_LDA_IND_Y_4;
                end
            end
            S_LDA_IND_Y_4B: begin
                rdm_inc     = 1;
                next_state  = S_LDA_IND_Y_5;
            end
            S_LDA_IND_Y_5: begin
                addr_sel    = 2'b00;
                rem_load    = 1;
                next_state  = S_LDA_IND_Y_5B;
            end
            S_LDA_IND_Y_5B: begin
                mem_read    = 1;
                if (mem_ready) begin
                    rdm_load_hi      = 1;
                    rdm_lo_from_temp = 1;
                    next_state       = S_LDA_IND_Y_6;
                end else begin
                    mem_req     = 1;
                    next_state  = S_LDA_IND_Y_5B;
                end
            end
            S_LDA_IND_Y_6: begin
                // RDM holds target base address, add Y via indexed_mode_y and fetch
                addr_sel     = 2'b00;
                rem_load     = 1;
                indexed_mode_y = 1;  // mem_addr = REM + Y
                mem_read     = 1;
                if (mem_ready) begin
                    ac_load     = 1;
                    nz_load     = 1;
                    next_state  = S_FETCH_1;
                end else begin
                    mem_req     = 1;
                    next_state  = S_LDA_IND_Y_6;
                end
            end

            // ================================================================
            // REGISTER-TO-REGISTER ALU OPERATIONS
            // ================================================================

            // --- ADDX (0x32) ---
            // Add X to AC: AC = AC + X (single cycle)
            S_ADDX: begin
                alu_op = 4'b0000;     // ADD operation
                alu_b_sel = 3'b010;   // Select X register as ALU B input
                ac_load = 1;          // Load result to AC
                nz_load = 1;          // Update N and Z flags
                c_load = 1;           // Update carry flag
                next_state = S_FETCH_1;
            end

            // --- SUBX (0x33) ---
            // Subtract X from AC: AC = AC - X (single cycle)
            S_SUBX: begin
                alu_op = 4'b0001;     // SUB operation
                alu_b_sel = 3'b010;   // Select X register as ALU B input
                ac_load = 1;          // Load result to AC
                nz_load = 1;          // Update N and Z flags
                c_load = 1;           // Update carry flag
                next_state = S_FETCH_1;
            end

            // --- ADDY (0x34) ---
            // Add Y to AC: AC = AC + Y (single cycle)
            S_ADDY: begin
                alu_op = 4'b0000;     // ADD operation
                alu_b_sel = 3'b011;   // Select Y register as ALU B input
                ac_load = 1;          // Load result to AC
                nz_load = 1;          // Update N and Z flags
                c_load = 1;           // Update carry flag
                next_state = S_FETCH_1;
            end

            // --- SUBY (0x35) ---
            // Subtract Y from AC: AC = AC - Y (single cycle)
            S_SUBY: begin
                alu_op = 4'b0001;     // SUB operation
                alu_b_sel = 3'b011;   // Select Y register as ALU B input
                ac_load = 1;          // Load result to AC
                nz_load = 1;          // Update N and Z flags
                c_load = 1;           // Update carry flag
                next_state = S_FETCH_1;
            end

            // --- ORX (0x42) ---
            // OR AC with X: AC = AC | X (single cycle)
            S_ORX: begin
                alu_op = 4'b0011;     // OR operation
                alu_b_sel = 3'b010;   // Select X register as ALU B input
                ac_load = 1;          // Load result to AC
                nz_load = 1;          // Update N and Z flags
                next_state = S_FETCH_1;
            end

            // --- ANDX (0x52) ---
            // AND AC with X: AC = AC & X (single cycle)
            S_ANDX: begin
                alu_op = 4'b0010;     // AND operation
                alu_b_sel = 3'b010;   // Select X register as ALU B input
                ac_load = 1;          // Load result to AC
                nz_load = 1;          // Update N and Z flags
                next_state = S_FETCH_1;
            end

            // --- XORX (0x43) ---
            // XOR AC with X: AC = AC ^ X (single cycle)
            S_XORX: begin
                alu_op = 4'b0100;     // XOR operation
                alu_b_sel = 3'b010;   // Select X register as ALU B input
                ac_load = 1;          // Load result to AC
                nz_load = 1;          // Update N and Z flags
                next_state = S_FETCH_1;
            end

            // ================================================================
            // COMPARE IMMEDIATE
            // ================================================================

            // --- CMPI imm (0xE1) ---
            // Compare AC with immediate value: AC - imm, set flags only (no store)
            S_CMPI_1: begin
                addr_sel = 2'b01;     // PC -> REM
                rem_load = 1;
                next_state = S_CMPI_2;
            end
            S_CMPI_2: begin
                mem_read = 1;
                alu_op = 4'b0001;     // SUB operation (for comparison)
                if (mem_ready) begin
                    // Set flags based on AC - imm, but DON'T load AC
                    nz_load = 1;      // Update N and Z flags
                    c_load = 1;       // Update carry flag
                    pc_inc = 1;       // Increment PC past immediate
                    next_state = S_FETCH_1;
                end else begin
                    mem_req = 1;
                    next_state = S_CMPI_2;
                end
            end

            // --- MULI imm (0xE2) --- Multiply Immediate: AC = AC * imm
            // Sequential multiplier: 8 cycles for area efficiency
            S_MULI_1: begin
                addr_sel = 2'b01;     // PC -> REM
                rem_load = 1;
                next_state = S_MULI_2;
            end
            S_MULI_2: begin
                mem_read = 1;
                alu_b_sel = 3'b000;   // Select mem_data_in as multiplier
                if (mem_ready) begin
                    // Memory ready - start sequential multiplier
                    mul_start = 1;    // Start the sequential multiplier
                    pc_inc = 1;       // Increment PC past immediate operand
                    next_state = S_MULI_WAIT;
                end else begin
                    mem_req = 1;
                    next_state = S_MULI_2;
                end
            end
            S_MULI_WAIT: begin
                mem_read = 1;         // Keep memory read active to hold multiplier value
                alu_b_sel = 3'b000;   // Keep mem_data_in as multiplier
                if (mul_done) begin
                    // Multiplication complete - load results
                    alu_op = 4'b1001;     // MUL operation (routes multiplier results through ALU)
                    ac_load = 1;          // Load result (low byte) into AC
                    y_load = 1;           // Enable Y register load
                    mul_to_y = 1;         // Select mul_high as Y input source
                    nz_load = 1;          // Update N and Z flags
                    c_load = 1;           // Update carry (overflow indicator)
                    next_state = S_FETCH_1;
                end else begin
                    // Still multiplying - wait
                    next_state = S_MULI_WAIT;
                end
            end

            // --- DIVI imm (0xE3) --- Divide Immediate: AC = AC / imm, Y = AC % imm
            // Sequential divider: 8 cycles for area efficiency
            S_DIVI_1: begin
                addr_sel = 2'b01;     // PC -> REM
                rem_load = 1;
                next_state = S_DIVI_2;
            end
            S_DIVI_2: begin
                mem_read = 1;
                alu_b_sel = 3'b000;   // Select mem_data_in as divisor
                if (mem_ready) begin
                    // Memory ready - start sequential divider
                    div_start = 1;    // Start the sequential divider
                    pc_inc = 1;       // Increment PC past immediate operand
                    next_state = S_DIVI_WAIT;
                end else begin
                    mem_req = 1;
                    next_state = S_DIVI_2;
                end
            end
            S_DIVI_WAIT: begin
                mem_read = 1;         // Keep memory read active to hold divisor value
                alu_b_sel = 3'b000;   // Keep mem_data_in as divisor
                if (div_done) begin
                    // Division complete - load results
                    alu_op = 4'b1010;     // DIV operation (routes divider results through ALU)
                    ac_load = 1;          // Load quotient into AC
                    y_load = 1;           // Enable Y register load
                    mul_to_y = 1;         // Select remainder as Y input source
                    nz_load = 1;          // Update N and Z flags
                    c_load = 1;           // Update carry (division by zero indicator)
                    next_state = S_FETCH_1;
                end else begin
                    // Still dividing - wait
                    next_state = S_DIVI_WAIT;
                end
            end

            // --- DECJNZ addr (0x88) --- Decrement AC and Jump if Not Zero
            // First decrement AC, then fetch 16-bit address and jump if AC != 0
            S_DECJNZ_1: begin
                // Decrement AC: AC = AC - 1
                alu_op = 4'b0001;     // SUB operation
                alu_b_sel = 3'b001;   // Select constant 1
                ac_load = 1;
                nz_load = 1;
                c_load = 1;
                // Setup to fetch address low byte
                addr_sel = 2'b01;     // PC -> REM
                rem_load = 1;
                next_state = S_DECJNZ_2;
            end
            S_DECJNZ_2: begin
                // Fetch jump address low byte
                mem_read = 1;
                if (mem_ready) begin
                    rdm_load = 1;
                    pc_inc = 1;
                    next_state = S_DECJNZ_2B;
                end else begin
                    mem_req = 1;
                    next_state = S_DECJNZ_2;
                end
            end
            S_DECJNZ_2B: begin
                addr_sel = 2'b01;     // PC -> REM
                rem_load = 1;
                next_state = S_DECJNZ_2C;
            end
            S_DECJNZ_2C: begin
                // Fetch jump address high byte
                mem_read = 1;
                if (mem_ready) begin
                    rdm_load_hi = 1;
                    pc_inc = 1;
                    next_state = S_DECJNZ_3;
                end else begin
                    mem_req = 1;
                    next_state = S_DECJNZ_2C;
                end
            end
            S_DECJNZ_3: begin
                // Check if AC is not zero (flagZ was set in S_DECJNZ_1)
                // If not zero, load PC from RDM
                if (!flagZ) begin
                    pc_load = 1;  // PC = RDM (jump to address)
                end
                // If zero, PC already incremented past address, continue to fetch
                next_state = S_FETCH_1;
            end

            // ================================================================
            // SP-RELATIVE ADDRESSING
            // ================================================================

            // --- LDA off,SP (0x28) --- 16-bit offset
            // Load AC from memory: AC = MEM[SP + offset]
            S_LDA_SP_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDA_SP_2;
            end
            S_LDA_SP_2: begin
                // Fetch offset_lo
                mem_read = 1;
                if (mem_ready) begin
                    rdm_load = 1; pc_inc = 1; next_state = S_LDA_SP_2B;
                end else begin
                    mem_req = 1; next_state = S_LDA_SP_2;
                end
            end
            S_LDA_SP_2B: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDA_SP_2C;
            end
            S_LDA_SP_2C: begin
                // Fetch offset_hi
                mem_read = 1;
                if (mem_ready) begin
                    rdm_load_hi = 1; pc_inc = 1; next_state = S_LDA_SP_3;
                end else begin
                    mem_req = 1; next_state = S_LDA_SP_2C;
                end
            end
            S_LDA_SP_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_LDA_SP_4;
            end
            S_LDA_SP_4: begin
                mem_read = 1;
                indexed_mode_sp = 1;  // Use SP + RDM as address
                if (mem_ready) begin
                    ac_load = 1; nz_load = 1; next_state = S_FETCH_1;
                end else begin
                    mem_req = 1; next_state = S_LDA_SP_4;
                end
            end

            // --- STA off,SP (0x17) --- 16-bit offset
            // Store AC to memory: MEM[SP + offset] = AC
            S_STA_SP_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_STA_SP_2;
            end
            S_STA_SP_2: begin
                // Fetch offset_lo
                mem_read = 1;
                if (mem_ready) begin
                    rdm_load = 1; pc_inc = 1; next_state = S_STA_SP_2B;
                end else begin
                    mem_req = 1; next_state = S_STA_SP_2;
                end
            end
            S_STA_SP_2B: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_STA_SP_2C;
            end
            S_STA_SP_2C: begin
                // Fetch offset_hi
                mem_read = 1;
                if (mem_ready) begin
                    rdm_load_hi = 1; pc_inc = 1; next_state = S_STA_SP_3;
                end else begin
                    mem_req = 1; next_state = S_STA_SP_2C;
                end
            end
            S_STA_SP_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_STA_SP_4;
            end
            S_STA_SP_4: begin
                mem_write = 1;
                indexed_mode_sp = 1;  // Use SP + RDM as address
                mem_data_sel_ext = 3'b000;  // AC
                if (mem_ready) begin
                    next_state = S_FETCH_1;
                end else begin
                    mem_req = 1; next_state = S_STA_SP_4;
                end
            end

            // ================================================================
            // INDEXED ADDRESSING MODES
            // ================================================================

            // --- LDA addr,X (0x21) --- 16-bit address
            // Load AC from memory with indexed addressing: AC = MEM[addr + X]
            S_LDA_X_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDA_X_2;
            end
            S_LDA_X_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_LDA_X_2B; end
                else begin mem_req = 1; next_state = S_LDA_X_2; end
            end
            S_LDA_X_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDA_X_2C;
            end
            S_LDA_X_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_LDA_X_3; end
                else begin mem_req = 1; next_state = S_LDA_X_2C; end
            end
            S_LDA_X_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_LDA_X_4;
            end
            S_LDA_X_4: begin
                mem_read = 1;
                indexed_mode = 1;  // Use addr + X for memory access
                if (mem_ready) begin ac_load = 1; nz_load = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_LDA_X_4; end
            end

            // --- STA addr,X (0x11) --- 16-bit address
            // Store AC to memory with indexed addressing: MEM[addr + X] = AC
            S_STA_X_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_STA_X_2;
            end
            S_STA_X_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_STA_X_2B; end
                else begin mem_req = 1; next_state = S_STA_X_2; end
            end
            S_STA_X_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_STA_X_2C;
            end
            S_STA_X_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_STA_X_3; end
                else begin mem_req = 1; next_state = S_STA_X_2C; end
            end
            S_STA_X_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_STA_X_4;
            end
            S_STA_X_4: begin
                indexed_mode = 1;  // Use addr + X for memory access
                mem_write = 1;
                if (mem_ready) begin next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_STA_X_4; end
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

            // --- CMP addr (0x02) --- 16-bit address
            // Compare AC with memory: set flags N, Z, C based on (AC - MEM[addr])
            // Does NOT modify AC, only sets flags
            S_CMP_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_CMP_2;
            end
            S_CMP_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_CMP_2B; end
                else begin mem_req = 1; next_state = S_CMP_2; end
            end
            S_CMP_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_CMP_2C;
            end
            S_CMP_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_CMP_3; end
                else begin mem_req = 1; next_state = S_CMP_2C; end
            end
            S_CMP_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_CMP_4;
            end
            S_CMP_4: begin
                mem_read = 1;
                // Do NOT load AC - only set flags
                alu_op = 4'b0001;  // SUB (for comparison)
                if (mem_ready) begin nz_load = 1; c_load = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_CMP_4; end
            end

            // --- JC addr (0x81) --- 16-bit address
            // Jump if Carry flag is set
            S_JC_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_JC_2;
            end
            S_JC_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_JC_2B; end
                else begin mem_req = 1; next_state = S_JC_2; end
            end
            S_JC_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_JC_2C;
            end
            S_JC_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_JC_3; end
                else begin mem_req = 1; next_state = S_JC_2C; end
            end
            S_JC_3: begin
                if (flagC) pc_load = 1;
                next_state = S_FETCH_1;
            end

            // --- JNC addr (0x82) --- 16-bit address
            // Jump if Carry flag is clear (No Carry)
            S_JNC_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_JNC_2;
            end
            S_JNC_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_JNC_2B; end
                else begin mem_req = 1; next_state = S_JNC_2; end
            end
            S_JNC_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_JNC_2C;
            end
            S_JNC_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_JNC_3; end
                else begin mem_req = 1; next_state = S_JNC_2C; end
            end
            S_JNC_3: begin
                if (!flagC) pc_load = 1;
                next_state = S_FETCH_1;
            end

            // ================================================================
            // SIGNED COMPARISON JUMPS (after CMP instruction)
            // ================================================================

            // --- JLE addr (0x83) --- 16-bit address
            // Jump if Less or Equal (signed): N=1 OR Z=1
            S_JLE_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_JLE_2;
            end
            S_JLE_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_JLE_2B; end
                else begin mem_req = 1; next_state = S_JLE_2; end
            end
            S_JLE_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_JLE_2C;
            end
            S_JLE_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_JLE_3; end
                else begin mem_req = 1; next_state = S_JLE_2C; end
            end
            S_JLE_3: begin
                if (flagN || flagZ) pc_load = 1;
                next_state = S_FETCH_1;
            end

            // --- JGT addr (0x84) --- 16-bit address
            // Jump if Greater Than (signed): N=0 AND Z=0
            S_JGT_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_JGT_2;
            end
            S_JGT_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_JGT_2B; end
                else begin mem_req = 1; next_state = S_JGT_2; end
            end
            S_JGT_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_JGT_2C;
            end
            S_JGT_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_JGT_3; end
                else begin mem_req = 1; next_state = S_JGT_2C; end
            end
            S_JGT_3: begin
                if (!flagN && !flagZ) pc_load = 1;
                next_state = S_FETCH_1;
            end

            // --- JGE addr (0x85) --- 16-bit address
            // Jump if Greater or Equal (signed): N=0
            S_JGE_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_JGE_2;
            end
            S_JGE_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_JGE_2B; end
                else begin mem_req = 1; next_state = S_JGE_2; end
            end
            S_JGE_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_JGE_2C;
            end
            S_JGE_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_JGE_3; end
                else begin mem_req = 1; next_state = S_JGE_2C; end
            end
            S_JGE_3: begin
                if (!flagN) pc_load = 1;
                next_state = S_FETCH_1;
            end

            // ================================================================
            // UNSIGNED COMPARISON JUMPS (after CMP instruction)
            // ================================================================

            // --- JBE addr (0x86) ---
            // Jump if Below or Equal (unsigned): C=1 OR Z=1
            S_JBE_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_JBE_2;
            end
            S_JBE_2: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; next_state = S_JBE_3; end
                else begin mem_req = 1; next_state = S_JBE_2; end
            end
            S_JBE_3: begin
                if (flagC || flagZ) pc_load = 1;
                else                pc_inc  = 1;
                next_state = S_FETCH_1;
            end

            // --- JA addr (0x87) ---
            // Jump if Above (unsigned): C=0 AND Z=0
            S_JA_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_JA_2;
            end
            S_JA_2: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; next_state = S_JA_3; end
                else begin mem_req = 1; next_state = S_JA_2; end
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
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDY_2;
            end
            S_LDY_2: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_LDY_3; end
                else begin mem_req = 1; next_state = S_LDY_2; end
            end
            S_LDY_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_LDY_4;
            end
            S_LDY_4: begin
                mem_read = 1;
                if (mem_ready) begin y_load = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_LDY_4; end
            end

            // --- STY addr (0x08) ---
            // Store Y to memory: MEM[addr] = Y
            S_STY_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_STY_2;
            end
            S_STY_2: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_STY_3; end
                else begin mem_req = 1; next_state = S_STY_2; end
            end
            S_STY_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_STY_4;
            end
            S_STY_4: begin
                mem_write = 1;
                mem_data_sel_ext = 3'b011;  // Select Y as data source
                if (mem_ready) begin next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_STY_4; end
            end

            // --- LDYI imm (0x06) --- (16-bit immediate)
            // Load Y with immediate: Y = imm (16-bit)
            S_LDYI_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDYI_2;
            end
            S_LDYI_2: begin
                mem_read = 1;
                if (mem_ready) begin y_load = 1; pc_inc_2 = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_LDYI_2; end
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

            // --- LDA addr,Y (0x22) --- 16-bit address
            // Load AC from memory with indexed addressing: AC = MEM[addr + Y]
            S_LDA_Y_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDA_Y_2;
            end
            S_LDA_Y_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_LDA_Y_2B; end
                else begin mem_req = 1; next_state = S_LDA_Y_2; end
            end
            S_LDA_Y_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDA_Y_2C;
            end
            S_LDA_Y_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_LDA_Y_3; end
                else begin mem_req = 1; next_state = S_LDA_Y_2C; end
            end
            S_LDA_Y_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_LDA_Y_4;
            end
            S_LDA_Y_4: begin
                mem_read = 1;
                indexed_mode_y = 1;  // Use addr + Y for memory access
                if (mem_ready) begin ac_load = 1; nz_load = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_LDA_Y_4; end
            end

            // --- STA addr,Y (0x12) --- 16-bit address
            // Store AC to memory with indexed addressing: MEM[addr + Y] = AC
            S_STA_Y_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_STA_Y_2;
            end
            S_STA_Y_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_STA_Y_2B; end
                else begin mem_req = 1; next_state = S_STA_Y_2; end
            end
            S_STA_Y_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_STA_Y_2C;
            end
            S_STA_Y_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_STA_Y_3; end
                else begin mem_req = 1; next_state = S_STA_Y_2C; end
            end
            S_STA_Y_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_STA_Y_4;
            end
            S_STA_Y_4: begin
                indexed_mode_y = 1;  // Use addr + Y for memory access
                mem_write = 1;
                if (mem_ready) begin next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_STA_Y_4; end
            end

            // ================================================================
            // MULTIPLICATION AND DIVISION INSTRUCTIONS
            // ================================================================

            // --- MUL (0x09) ---
            // Multiply AC by X: AC * X -> Y:AC (16-bit result)
            // Low byte goes to AC, high byte goes to Y
            // Sequential multiplier: 8 cycles for area efficiency
            S_MUL: begin
                // Start sequential multiplier (multiplicand=AC, multiplier=X)
                alu_b_sel = 3'b010;  // Select X register as multiplier input
                mul_start = 1;       // Start the sequential multiplier
                next_state = S_MUL_WAIT;
            end
            S_MUL_WAIT: begin
                alu_b_sel = 3'b010;  // Keep X selected as multiplier
                if (mul_done) begin
                    // Multiplication complete - load results
                    alu_op = 4'b1001;   // MUL operation (routes multiplier results through ALU)
                    ac_load = 1;        // Load low byte to AC
                    y_load = 1;         // Load high byte to Y
                    mul_to_y = 1;       // Signal datapath to use mul_high for Y input
                    nz_load = 1;        // Update N and Z based on low byte (AC)
                    c_load = 1;         // Set carry if overflow (high byte != 0)
                    next_state = S_FETCH_1;
                end else begin
                    // Still multiplying - wait
                    next_state = S_MUL_WAIT;
                end
            end

            // --- DIV (0x0E) ---
            // Divide AC by X: AC / X -> AC (quotient), Y (remainder)
            // Sequential divider: 8 cycles for area efficiency
            // Carry flag set on division by zero
            S_DIV: begin
                // Start sequential divider (dividend=AC, divisor=X)
                alu_b_sel = 3'b010;  // Select X register as divisor input
                div_start = 1;       // Start the sequential divider
                next_state = S_DIV_WAIT;
            end
            S_DIV_WAIT: begin
                alu_b_sel = 3'b010;  // Keep X selected as divisor
                if (div_done) begin
                    // Division complete - load results
                    alu_op = 4'b1010;   // DIV operation (routes divider results through ALU)
                    ac_load = 1;        // Load quotient to AC
                    y_load = 1;         // Load remainder to Y
                    mul_to_y = 1;       // Signal datapath to use mul_high (remainder) for Y input
                    nz_load = 1;        // Update N and Z based on quotient (AC)
                    c_load = 1;         // Set carry if division by zero
                    next_state = S_FETCH_1;
                end else begin
                    // Still dividing - wait
                    next_state = S_DIV_WAIT;
                end
            end

            // --- MOD (0x0F) ---
            // Modulo AC by X: AC % X -> AC (remainder), Y (quotient)
            // Sequential divider: 8 cycles for area efficiency
            // Carry flag set on division by zero
            S_MOD: begin
                // Start sequential divider (dividend=AC, divisor=X)
                alu_b_sel = 3'b010;  // Select X register as divisor input
                div_start = 1;       // Start the sequential divider
                next_state = S_MOD_WAIT;
            end
            S_MOD_WAIT: begin
                alu_b_sel = 3'b010;  // Keep X selected as divisor
                if (div_done) begin
                    // Division complete - load results (swapped for MOD)
                    alu_op = 4'b1011;   // MOD operation (routes divider results through ALU, swapped)
                    ac_load = 1;        // Load remainder to AC
                    y_load = 1;         // Load quotient to Y
                    mul_to_y = 1;       // Signal datapath to use mul_high (quotient) for Y input
                    nz_load = 1;        // Update N and Z based on remainder (AC)
                    c_load = 1;         // Set carry if division by zero
                    next_state = S_FETCH_1;
                end else begin
                    // Still dividing - wait
                    next_state = S_MOD_WAIT;
                end
            end

            // ================================================================
            // MULTI-BYTE ARITHMETIC INSTRUCTIONS (ADC, SBC, ASR)
            // ================================================================

            // --- ADC addr (0x31) ---
            // Add with Carry: AC = AC + MEM[addr] + Carry
            // Essential for multi-byte (16-bit, 32-bit) addition
            S_ADC_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_ADC_2;
            end
            S_ADC_2: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_ADC_3; end
                else begin mem_req = 1; next_state = S_ADC_2; end
            end
            S_ADC_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_ADC_4;
            end
            S_ADC_4: begin
                mem_read = 1;
                alu_op = 4'b1100;  // ADC operation
                if (mem_ready) begin ac_load = 1; nz_load = 1; c_load = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_ADC_4; end
            end

            // --- SBC addr (0x51) ---
            // Subtract with Borrow: AC = AC - MEM[addr] - Carry
            // Essential for multi-byte (16-bit, 32-bit) subtraction
            S_SBC_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_SBC_2;
            end
            S_SBC_2: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_SBC_3; end
                else begin mem_req = 1; next_state = S_SBC_2; end
            end
            S_SBC_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_SBC_4;
            end
            S_SBC_4: begin
                mem_read = 1;
                alu_op = 4'b1101;  // SBC operation
                if (mem_ready) begin ac_load = 1; nz_load = 1; c_load = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_SBC_4; end
            end

            // --- ASR (0x61) ---
            // Arithmetic Shift Right: AC = AC >> 1 (preserves sign bit)
            // Essential for signed division by 2 and sign extension
            S_ASR: begin
                ac_load = 1;
                alu_op = 4'b1110;  // ASR operation
                nz_load = 1;
                c_load = 1;        // Carry gets the shifted-out bit
                next_state = S_FETCH_1;
            end

            // ================================================================
            // FRAME POINTER EXTENSION INSTRUCTIONS
            // ================================================================

            // --- TSF (0x0A) ---
            // Transfer SP to FP: FP = SP (single cycle)
            S_TSF: begin
                fp_load = 1;       // Load FP from SP (datapath handles mux)
                next_state = S_FETCH_1;
            end

            // --- TFS (0x0B) ---
            // Transfer FP to SP: SP = FP (single cycle)
            S_TFS: begin
                sp_load = 1;       // Load SP from FP
                next_state = S_FETCH_1;
            end

            // --- PUSH_FP (0x0C) --- 16-bit push for frame pointer
            // Push full 16-bit FP onto stack: SP -= 2, MEM[SP] = FP
            S_PUSH_FP_1: begin
                sp_dec = 1;        // Decrement SP first (first of 2)
                next_state = S_PUSH_FP_4;
            end
            S_PUSH_FP_4: begin
                sp_dec = 1;        // Decrement SP (second of 2, now SP -= 2 total)
                next_state = S_PUSH_FP_2;
            end
            S_PUSH_FP_2: begin
                addr_sel = 2'b10;  // SP -> REM
                rem_load = 1;
                next_state = S_PUSH_FP_3;
            end
            S_PUSH_FP_3: begin
                mem_write = 1;
                mem_data_sel_ext = 3'b100;  // FP (full 16-bit)
                if (mem_ready) begin next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_PUSH_FP_3; end
            end
            // Unused states for compatibility
            S_PUSH_FP_5: begin next_state = S_FETCH_1; end
            S_PUSH_FP_6: begin next_state = S_FETCH_1; end

            // --- POP_FP (0x0D) --- 16-bit pop for frame pointer
            // Pop full 16-bit FP from stack: FP = MEM[SP], SP += 2
            S_POP_FP_1: begin
                addr_sel = 2'b10;  // SP -> REM
                rem_load = 1;
                next_state = S_POP_FP_2;
            end
            S_POP_FP_2: begin
                mem_read = 1; mem_req = 1;
                next_state = S_POP_FP_3;
            end
            S_POP_FP_3: begin
                mem_read = 1;
                if (mem_ready) begin
                    fp_load_lo = 1;    // Load full 16-bit FP from memory
                    sp_inc = 1;        // Increment SP (first of 2)
                    next_state = S_POP_FP_4;
                end else begin
                    mem_req = 1;
                    next_state = S_POP_FP_3;
                end
            end
            S_POP_FP_4: begin
                sp_inc = 1;            // Increment SP (second of 2, now SP += 2 total)
                next_state = S_FETCH_1;
            end
            // Unused states for compatibility
            S_POP_FP_5: begin next_state = S_FETCH_1; end
            S_POP_FP_6: begin next_state = S_FETCH_1; end

            // ================================================================
            // INDEXED ADDRESSING MODES (FP)
            // ================================================================

            // --- LDA addr,FP (0x24) --- 16-bit address
            // Load AC from memory with FP-indexed addressing: AC = MEM[addr + FP]
            S_LDA_FP_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDA_FP_2;
            end
            S_LDA_FP_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_LDA_FP_2B; end
                else begin mem_req = 1; next_state = S_LDA_FP_2; end
            end
            S_LDA_FP_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDA_FP_2C;
            end
            S_LDA_FP_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_LDA_FP_3; end
                else begin mem_req = 1; next_state = S_LDA_FP_2C; end
            end
            S_LDA_FP_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_LDA_FP_4;
            end
            S_LDA_FP_4: begin
                mem_read = 1;
                indexed_mode_fp = 1;  // Use addr + FP for memory access
                if (mem_ready) begin ac_load = 1; nz_load = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_LDA_FP_4; end
            end

            // --- STA addr,FP (0x14) --- 16-bit address
            // Store AC to memory with FP-indexed addressing: MEM[addr + FP] = AC
            S_STA_FP_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_STA_FP_2;
            end
            S_STA_FP_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_STA_FP_2B; end
                else begin mem_req = 1; next_state = S_STA_FP_2; end
            end
            S_STA_FP_2B: begin
                // Set REM = PC to fetch addr_hi
                addr_sel = 2'b01; rem_load = 1; next_state = S_STA_FP_2C;
            end
            S_STA_FP_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_STA_FP_3; end
                else begin mem_req = 1; next_state = S_STA_FP_2C; end
            end
            S_STA_FP_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_STA_FP_4;
            end
            S_STA_FP_4: begin
                indexed_mode_fp = 1;  // Use addr + FP for memory access
                mem_write = 1;
                if (mem_ready) begin next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_STA_FP_4; end
            end

            // ================================================================
            // B REGISTER EXTENSION INSTRUCTIONS
            // ================================================================

            // --- TAB (0x1C) ---
            // Transfer AC to B: B = AC (single cycle)
            S_TAB: begin
                b_load = 1;        // Load B from AC (datapath handles mux)
                next_state = S_FETCH_1;
            end

            // --- TBA (0x1D) ---
            // Transfer B to AC: AC = B (single cycle)
            S_TBA: begin
                b_to_ac = 1;       // Signal datapath to select B for AC input
                ac_load = 1;
                nz_load = 1;       // Update flags based on B value
                next_state = S_FETCH_1;
            end

            // --- LDB addr (0x1E) ---
            // Load B from memory: B = MEM[addr]
            S_LDB_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDB_2;
            end
            S_LDB_2: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_LDB_2B; end
                else begin mem_req = 1; next_state = S_LDB_2; end
            end
            S_LDB_2B: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDB_2C;
            end
            S_LDB_2C: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_LDB_3; end
                else begin mem_req = 1; next_state = S_LDB_2C; end
            end
            S_LDB_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_LDB_4;
            end
            S_LDB_4: begin
                mem_read = 1;
                if (mem_ready) begin b_load = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_LDB_4; end
            end

            // --- STB addr (0x1F) ---
            // Store B to memory: MEM[addr] = B
            S_STB_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_STB_2;
            end
            S_STB_2: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_STB_2B; end
                else begin mem_req = 1; next_state = S_STB_2; end
            end
            S_STB_2B: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_STB_2C;
            end
            S_STB_2C: begin
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_STB_3; end
                else begin mem_req = 1; next_state = S_STB_2C; end
            end
            S_STB_3: begin
                addr_sel = 2'b00; rem_load = 1; next_state = S_STB_4;
            end
            S_STB_4: begin
                mem_write = 1;
                mem_data_sel_ext = 3'b101;  // Select B as data source
                if (mem_ready) begin next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_STB_4; end
            end

            // --- INB (0x36) ---
            // Increment B: B = B + 1 (single cycle)
            S_INB: begin
                b_inc = 1;         // Increment B register
                next_state = S_FETCH_1;
            end

            // --- DEB (0x37) ---
            // Decrement B: B = B - 1 (single cycle)
            S_DEB: begin
                b_dec = 1;         // Decrement B register
                next_state = S_FETCH_1;
            end

            // --- SWPB (0x38) ---
            // Swap AC and B: temp = AC; AC = B; B = temp (2 cycles)
            S_SWPB_1: begin
                swap_temp_load = 1;  // Save AC to temp register
                b_to_ac = 1;         // Load B value to AC
                ac_load = 1;         // Enable AC load
                nz_load = 1;         // Update N and Z flags based on new AC value
                next_state = S_SWPB_2;
            end
            S_SWPB_2: begin
                b_from_temp = 1;     // Load B from temp (original AC value)
                b_load = 1;          // Enable B load
                next_state = S_FETCH_1;
            end

            // --- ADDB (0x39) ---
            // Add B to AC: AC = AC + B (single cycle)
            S_ADDB: begin
                alu_op = 4'b0000;     // ADD operation
                alu_b_sel = 3'b100;   // Select B register as ALU B input
                ac_load = 1;          // Load result to AC
                nz_load = 1;          // Update N and Z flags
                c_load = 1;           // Update carry flag
                next_state = S_FETCH_1;
            end

            // --- SUBB (0x3A) ---
            // Subtract B from AC: AC = AC - B (single cycle)
            S_SUBB: begin
                alu_op = 4'b0001;     // SUB operation
                alu_b_sel = 3'b100;   // Select B register as ALU B input
                ac_load = 1;          // Load result to AC
                nz_load = 1;          // Update N and Z flags
                c_load = 1;           // Update carry flag
                next_state = S_FETCH_1;
            end

            // --- LDBI imm (0xE4) --- (16-bit immediate)
            // Load B with immediate: B = imm (16-bit)
            S_LDBI_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_LDBI_2;
            end
            S_LDBI_2: begin
                mem_read = 1;
                if (mem_ready) begin b_load = 1; pc_inc_2 = 1; next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_LDBI_2; end
            end

            // ================================================================
            // PUSH_ADDR AND POP_ADDR INSTRUCTIONS
            // ================================================================

            // --- PUSH_ADDR addr (0x89) ---
            // Push value from any address onto stack: MEM[--SP] = MEM[addr]
            // Steps:
            // 1-4: Fetch 16-bit address from operand
            // 5: Read value from memory at that address into swap_temp
            // 6-7: Decrement SP and write swap_temp to stack
            S_PUSH_ADDR_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_PUSH_ADDR_2;
            end
            S_PUSH_ADDR_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_PUSH_ADDR_2B; end
                else begin mem_req = 1; next_state = S_PUSH_ADDR_2; end
            end
            S_PUSH_ADDR_2B: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_PUSH_ADDR_2C;
            end
            S_PUSH_ADDR_2C: begin
                // Fetch addr_hi
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_PUSH_ADDR_3; end
                else begin mem_req = 1; next_state = S_PUSH_ADDR_2C; end
            end
            S_PUSH_ADDR_3: begin
                // Setup to read from the source address
                addr_sel = 2'b00; rem_load = 1; next_state = S_PUSH_ADDR_4;
            end
            S_PUSH_ADDR_4: begin
                // Read value from source address into swap_temp
                mem_read = 1;
                if (mem_ready) begin
                    ind_temp_load = 1;  // Store value in swap_temp
                    sp_dec = 1;         // Decrement SP (first of 2)
                    next_state = S_PUSH_ADDR_5;
                end else begin
                    mem_req = 1;
                    next_state = S_PUSH_ADDR_4;
                end
            end
            S_PUSH_ADDR_5: begin
                sp_dec = 1;  // Decrement SP (second of 2, SP -= 2 total)
                next_state = S_PUSH_ADDR_6;
            end
            S_PUSH_ADDR_6: begin
                // Setup SP as memory address
                addr_sel = 2'b10;  // SP -> REM
                rem_load = 1;
                // Load RDM from swap_temp so we can write it
                rdm_lo_from_temp = 1;
                next_state = S_PUSH_ADDR_7;
            end
            S_PUSH_ADDR_7: begin
                // Write swap_temp value to stack
                mem_write = 1;
                mem_data_sel_ext = 3'b110;  // Select swap_temp as data source
                if (mem_ready) begin next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_PUSH_ADDR_7; end
            end

            // --- POP_ADDR addr (0x8A) ---
            // Pop value from stack to any address: MEM[addr] = MEM[SP++]
            // Steps:
            // 1-4: Fetch 16-bit target address from operand
            // 5: Save target address to swap_temp
            // 6: Read value from stack into AC-style temp
            // 7: Write that value to target address
            S_POP_ADDR_1: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_POP_ADDR_2;
            end
            S_POP_ADDR_2: begin
                // Fetch addr_lo
                mem_read = 1;
                if (mem_ready) begin rdm_load = 1; pc_inc = 1; next_state = S_POP_ADDR_2B; end
                else begin mem_req = 1; next_state = S_POP_ADDR_2; end
            end
            S_POP_ADDR_2B: begin
                addr_sel = 2'b01; rem_load = 1; next_state = S_POP_ADDR_2C;
            end
            S_POP_ADDR_2C: begin
                // Fetch addr_hi - now RDM has target address
                mem_read = 1;
                if (mem_ready) begin rdm_load_hi = 1; pc_inc = 1; next_state = S_POP_ADDR_3; end
                else begin mem_req = 1; next_state = S_POP_ADDR_2C; end
            end
            S_POP_ADDR_3: begin
                // Save target address from RDM to swap_temp
                // We need to save RDM, read from stack, then restore for write
                // Use ind_temp_load with mem_data? No, that's for mem_data_in.
                // For now, let's use a different approach: read stack first into AC,
                // then write AC to target.
                // Setup SP as source address
                addr_sel = 2'b10;  // SP -> REM
                rem_load = 1;
                next_state = S_POP_ADDR_4;
            end
            S_POP_ADDR_4: begin
                // Read value from stack
                mem_read = 1;
                if (mem_ready) begin
                    // Store stack value in swap_temp (can use it later)
                    ind_temp_load = 1;
                    sp_inc = 1;  // Increment SP (first of 2)
                    next_state = S_POP_ADDR_5;
                end else begin
                    mem_req = 1;
                    next_state = S_POP_ADDR_4;
                end
            end
            S_POP_ADDR_5: begin
                sp_inc = 1;  // Increment SP (second of 2, SP += 2 total)
                next_state = S_POP_ADDR_6;
            end
            S_POP_ADDR_6: begin
                // Setup target address (still in RDM from earlier)
                addr_sel = 2'b00;  // RDM -> REM
                rem_load = 1;
                next_state = S_POP_ADDR_7;
            end
            S_POP_ADDR_7: begin
                // Write swap_temp value to target address
                mem_write = 1;
                mem_data_sel_ext = 3'b110;  // Select swap_temp as data source
                if (mem_ready) begin next_state = S_FETCH_1; end
                else begin mem_req = 1; next_state = S_POP_ADDR_7; end
            end

            // --- HLT ---
            S_HLT: begin
                next_state = S_HLT;
            end

            default: next_state = S_FETCH_1;
        endcase
    end

endmodule
