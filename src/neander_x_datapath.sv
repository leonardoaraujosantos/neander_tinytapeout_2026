// ============================================================================
// neander_x_datapath.sv — Datapath for NEANDER-X CPU (LCC + X/Y/FP Register Extension)
// ============================================================================
// X Register Extension adds:
//   - X index register for indexed addressing
//   - LDX, STX, LDXI, TAX, TXA, INX instructions
//   - Indexed addressing modes: LDA addr,X and STA addr,X
//
// Y Register Extension adds:
//   - Y index register for indexed addressing
//   - LDY, STY, LDYI, TAY, TYA, INY instructions
//   - Indexed addressing modes: LDA addr,Y and STA addr,Y
//
// Frame Pointer Extension adds:
//   - FP register for stack frame management
//   - TSF, TFS, PUSH_FP, POP_FP instructions
//   - Indexed addressing modes: LDA addr,FP and STA addr,FP
// ============================================================================

// ---------------------------------------------------------------------------
// Generic Register Module (Can be reused)
// ---------------------------------------------------------------------------
module generic_reg (
    input  logic       clk,
    input  logic       reset,
    input  logic       load,
    input  logic [7:0] data_in,
    output logic [7:0] value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            value <= 8'h00;
        else if (load)
            value <= data_in;
    end
endmodule

// ---------------------------------------------------------------------------
// PC Register (Specific behavior: Increment & Load)
// ---------------------------------------------------------------------------
module pc_reg (
    input  logic       clk,
    input  logic       reset,
    input  logic       pc_inc,
    input  logic       pc_load,
    input  logic [7:0] data_in,
    output logic [7:0] pc_value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            pc_value <= 8'h00;
        else if (pc_load)
            pc_value <= data_in;
        else if (pc_inc)
            pc_value <= pc_value + 8'h01;
    end
endmodule

// ---------------------------------------------------------------------------
// SP Register (Stack Pointer: Increment, Decrement & Load)
// Initialized to 0xFF (stack grows downward from top of memory)
// ---------------------------------------------------------------------------
module sp_reg (
    input  logic       clk,
    input  logic       reset,
    input  logic       sp_inc,
    input  logic       sp_dec,
    input  logic       sp_load,
    input  logic [7:0] data_in,
    output logic [7:0] sp_value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            sp_value <= 8'hFF;  // Stack starts at top of memory
        else if (sp_load)
            sp_value <= data_in;
        else if (sp_dec)
            sp_value <= sp_value - 8'h01;  // PUSH: decrement first, then write
        else if (sp_inc)
            sp_value <= sp_value + 8'h01;  // POP: read first, then increment
    end
endmodule

// ---------------------------------------------------------------------------
// NZC Register (Flags: Negative, Zero, Carry)
// ---------------------------------------------------------------------------
module nzc_reg (
    input  logic clk,
    input  logic reset,
    input  logic nz_load,    // Load N and Z flags
    input  logic c_load,     // Load C flag (separate control for CMP/arithmetic)
    input  logic N_in,
    input  logic Z_in,
    input  logic C_in,
    output logic N_flag,
    output logic Z_flag,
    output logic C_flag
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            N_flag <= 1'b0;
            Z_flag <= 1'b0;
            C_flag <= 1'b0;
        end
        else begin
            if (nz_load) begin
                N_flag <= N_in;
                Z_flag <= Z_in;
            end
            if (c_load) begin
                C_flag <= C_in;
            end
        end
    end
endmodule

// ---------------------------------------------------------------------------
// X Index Register (Load & Increment)
// ---------------------------------------------------------------------------
module x_reg (
    input  logic       clk,
    input  logic       reset,
    input  logic       x_load,
    input  logic       x_inc,
    input  logic [7:0] data_in,
    output logic [7:0] x_value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            x_value <= 8'h00;
        else if (x_load)
            x_value <= data_in;
        else if (x_inc)
            x_value <= x_value + 8'h01;
    end
endmodule

// ---------------------------------------------------------------------------
// Y Index Register (Load & Increment)
// ---------------------------------------------------------------------------
module y_reg (
    input  logic       clk,
    input  logic       reset,
    input  logic       y_load,
    input  logic       y_inc,
    input  logic [7:0] data_in,
    output logic [7:0] y_value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            y_value <= 8'h00;
        else if (y_load)
            y_value <= data_in;
        else if (y_inc)
            y_value <= y_value + 8'h01;
    end
endmodule

// ---------------------------------------------------------------------------
// FP Frame Pointer Register (Load only)
// Used for stack frame management in function calls
// ---------------------------------------------------------------------------
module fp_reg (
    input  logic       clk,
    input  logic       reset,
    input  logic       fp_load,
    input  logic [7:0] data_in,
    output logic [7:0] fp_value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            fp_value <= 8'h00;
        else if (fp_load)
            fp_value <= data_in;
    end
endmodule

// ---------------------------------------------------------------------------
// MUX PC/RDM/SP -> REM (3-way address mux)
// ---------------------------------------------------------------------------
module mux_addr (
    input  logic [1:0] sel, // 00 = RDM, 01 = PC, 10 = SP
    input  logic [7:0] pc,
    input  logic [7:0] rdm,
    input  logic [7:0] sp,
    output logic [7:0] out
);
    always_comb begin
        case (sel)
            2'b00:   out = rdm;
            2'b01:   out = pc;
            2'b10:   out = sp;
            default: out = rdm;
        endcase
    end
endmodule

// ============================================================================
// DATAPATH
// ============================================================================
module neander_datapath (
    input  logic       clk,
    input  logic       reset,

    // Control Signals
    input  logic       mem_read, // Used for RDM load logic
    input  logic       mem_write,
    input  logic       pc_inc,
    input  logic       pc_load,
    input  logic       ac_load,
    input  logic       ri_load,
    input  logic       rem_load,
    input  logic       rdm_load,
    input  logic       nz_load,
    input  logic       c_load,   // Carry flag load (for CMP and arithmetic ops)
    input  logic [1:0] addr_sel,  // 00=RDM, 01=PC, 10=SP
    input  logic [3:0] alu_op,    // Extended to 4 bits for NEG and future ops
    input  logic       sp_inc,    // Stack pointer increment (POP/RET)
    input  logic       sp_dec,    // Stack pointer decrement (PUSH/CALL)
    input  logic [1:0] mem_data_sel, // Memory data select: 00=AC, 01=PC, 10=X, 11=Y
    input  logic [1:0] alu_b_sel, // ALU B input select: 00=mem_data, 01=constant 1 (INC/DEC), 10=X (MUL)
    // X Register Extension signals
    input  logic       x_load,    // Load X register
    input  logic       x_inc,     // Increment X register
    input  logic       x_to_ac,   // Transfer X to AC (TXA)
    input  logic       indexed_mode, // Use indexed addressing (addr + X)
    // Y Register Extension signals
    input  logic       y_load,    // Load Y register
    input  logic       y_inc,     // Increment Y register
    input  logic       y_to_ac,   // Transfer Y to AC (TYA)
    input  logic       indexed_mode_y, // Use indexed addressing (addr + Y)
    input  logic       mul_to_y,  // Load Y with high byte of multiplication result
    // Frame Pointer Extension signals
    input  logic       fp_load,   // Load FP register
    input  logic       sp_load,   // Load SP from FP (for TFS)
    input  logic       indexed_mode_fp, // Use indexed addressing (addr + FP)
    input  logic [2:0] mem_data_sel_ext, // Extended: 000=AC, 001=PC, 010=X, 011=Y, 100=FP

    // External RAM Interface
    input  logic [7:0] mem_data_in,
    output logic [7:0] mem_addr,
    output logic [7:0] mem_data_out,

    // I/O Interface
    input  logic [7:0] io_in,
    input  logic [7:0] io_status,
    output logic [7:0] io_out,
    input  logic       io_write_ctrl,
    output logic       io_write,

    // Output to Control Unit / Debug
    output logic [3:0] opcode,
    output logic [3:0] sub_opcode, // Lower nibble for stack ops
    output logic       flagN,
    output logic       flagZ,
    output logic       flagC,      // Carry flag output
    output logic [7:0] dbg_pc,
    output logic [7:0] dbg_ac,
    output logic [7:0] dbg_ri,
    output logic [7:0] dbg_sp,
    output logic [7:0] dbg_x,     // X register debug output
    output logic [7:0] dbg_y,     // Y register debug output
    output logic [7:0] dbg_fp     // FP register debug output
);

    // Internal Signals (using 'logic' for everything)
    logic [7:0] pc, rem, rdm, ri, ac, sp, x, y, fp;  // Added X, Y, and FP registers
    logic [7:0] alu_res;
    logic [7:0] alu_mul_high;  // High byte of multiplication result from ALU
    logic       alu_carry;  // Carry output from ALU
    logic [7:0] alu_b_in;  // ALU B input (mem_data or constant 1)
    logic [7:0] addr_mux;
    logic [7:0] addr_indexed;  // Address + X for indexed modes
    logic [7:0] addr_indexed_y; // Address + Y for indexed modes
    logic [7:0] addr_indexed_fp; // Address + FP for indexed modes
    logic [7:0] fp_in;    // FP register input (from SP for TSF, or from memory for POP_FP)
    logic [7:0] ac_in;
    logic [7:0] x_in;     // X register input (from AC for TAX, or from memory for LDX)
    logic [7:0] y_in;     // Y register input (from AC for TAY, or from memory for LDY)
    logic       N_in, Z_in, C_in;

    // Indexed address calculation: addr + X or addr + Y or addr + FP
    assign addr_indexed = rdm + x;
    assign addr_indexed_y = rdm + y;
    assign addr_indexed_fp = rdm + fp;

    // Direct assignments
    // Memory address: use indexed address when indexed_mode, indexed_mode_y, or indexed_mode_fp is set
    assign mem_addr = indexed_mode_fp ? (rem + fp) : (indexed_mode_y ? (rem + y) : (indexed_mode ? (rem + x) : rem));
    // Memory data output MUX (extended): 000=AC, 001=PC, 010=X, 011=Y, 100=FP
    always_comb begin
        case (mem_data_sel_ext)
            3'b000:  mem_data_out = ac;   // STA, PUSH
            3'b001:  mem_data_out = pc;   // CALL (return address)
            3'b010:  mem_data_out = x;    // STX
            3'b011:  mem_data_out = y;    // STY
            3'b100:  mem_data_out = fp;   // PUSH_FP
            default: mem_data_out = ac;
        endcase
    end
    assign io_out       = ac;
    assign io_write     = io_write_ctrl;
    assign opcode       = ri[7:4];
    assign sub_opcode   = ri[3:0];  // Lower nibble for PUSH/POP/CALL/RET and indexed mode flag

    // Debug outputs
    assign dbg_pc = pc;
    assign dbg_ac = ac;
    assign dbg_ri = ri;
    assign dbg_sp = sp;
    assign dbg_x  = x;  // X register debug output
    assign dbg_y  = y;  // Y register debug output
    assign dbg_fp = fp; // FP register debug output

    // --- Instantiations ---

    pc_reg u_pc (
        .clk(clk), .reset(reset),
        .pc_inc(pc_inc), .pc_load(pc_load),
        .data_in(rdm), .pc_value(pc)
    );

    sp_reg u_sp (
        .clk(clk), .reset(reset),
        .sp_inc(sp_inc), .sp_dec(sp_dec), .sp_load(sp_load),
        .data_in(fp), .sp_value(sp)  // sp_load used for TFS: SP ← FP
    );

    // X Index Register
    // Input can be from memory (LDX), immediate (LDXI), or AC (TAX)
    x_reg u_x (
        .clk(clk), .reset(reset),
        .x_load(x_load), .x_inc(x_inc),
        .data_in(x_in), .x_value(x)
    );

    // X register input mux: select between mem_data_in (LDX/LDXI) and AC (TAX)
    // When opcode is 0x7 and sub_opcode is 0xD (TAX), input comes from AC
    // Otherwise (LDX/LDXI), input comes from memory/immediate
    assign x_in = (opcode == 4'h7 && sub_opcode == 4'hD) ? ac : mem_data_in;

    // Y Index Register
    // Input can be from memory (LDY), immediate (LDYI), or AC (TAY)
    y_reg u_y (
        .clk(clk), .reset(reset),
        .y_load(y_load), .y_inc(y_inc),
        .data_in(y_in), .y_value(y)
    );

    // Y register input mux: select between mem_data_in (LDY/LDYI), AC (TAY), or mul_high (MUL)
    // Priority: mul_to_y > TAY > LDY/LDYI
    // When mul_to_y is set, input comes from ALU mul_high (MUL instruction)
    // When opcode is 0x0 and sub_opcode is 0x3 (TAY), input comes from AC
    // Otherwise (LDY/LDYI), input comes from memory/immediate
    assign y_in = mul_to_y ? alu_mul_high :
                  (opcode == 4'h0 && sub_opcode == 4'h3) ? ac : mem_data_in;

    // FP Frame Pointer Register
    // Input can be from SP (TSF) or from memory (POP_FP)
    fp_reg u_fp (
        .clk(clk), .reset(reset),
        .fp_load(fp_load),
        .data_in(fp_in), .fp_value(fp)
    );

    // FP register input mux: select between SP (TSF) and mem_data_in (POP_FP)
    // When opcode is 0x0 and sub_opcode is 0xA (TSF), input comes from SP
    // When opcode is 0x0 and sub_opcode is 0xD (POP_FP), input comes from memory
    assign fp_in = (opcode == 4'h0 && sub_opcode == 4'hA) ? sp : mem_data_in;

    mux_addr u_mux (
        .sel(addr_sel), .pc(pc), .rdm(rdm), .sp(sp), .out(addr_mux)
    );

    // Reusing generic_reg for standard registers to reduce code size
    generic_reg u_rem (
        .clk(clk), .reset(reset), .load(rem_load),
        .data_in(addr_mux), .value(rem)
    );

    // Logic for RDM Load is specific (load & mem_read)
    generic_reg u_rdm (
        .clk(clk), .reset(reset), .load(rdm_load & mem_read),
        .data_in(mem_data_in), .value(rdm)
    );

    generic_reg u_ri (
        .clk(clk), .reset(reset), .load(ri_load),
        .data_in(rdm), .value(ri)
    );

    // ALU B input MUX: select between mem_data_in (00), constant 1 (01) for INC/DEC, X (10) for MUL
    always_comb begin
        case (alu_b_sel)
            2'b00:   alu_b_in = mem_data_in;  // Default: memory data
            2'b01:   alu_b_in = 8'h01;        // Constant 1 for INC/DEC
            2'b10:   alu_b_in = x;            // X register for MUL
            default: alu_b_in = mem_data_in;
        endcase
    end

    neander_alu u_alu (
        .a(ac), .b(alu_b_in), .alu_op(alu_op), .carry_in(flagC),
        .result(alu_res), .mul_high(alu_mul_high), .carry_out(alu_carry)
    );

    // Combinational Logic for AC Input Mux
    always_comb begin
        if (x_to_ac) begin
            // TXA: Transfer X to AC
            ac_in = x;
        end
        else if (y_to_ac) begin
            // TYA: Transfer Y to AC
            ac_in = y;
        end
        else if (opcode == 4'h2 || opcode == 4'hE) begin
            // LDA or LDI (including indexed LDA when sub_opcode[0]=1 or sub_opcode[1]=1)
            ac_in = mem_data_in;
        end
        else if (opcode == 4'h7 && sub_opcode == 4'h1) begin
            // POP: load from memory (stack)
            ac_in = mem_data_in;
        end
        else if (opcode == 4'hC) begin
            // IN: 0=data, 1=status
            case (rdm)
                8'h00: ac_in = io_in;
                8'h01: ac_in = io_status;
                default: ac_in = 8'h00;
            endcase
        end
        else begin
            // Arithmetic/Logic
            ac_in = alu_res;
        end
    end

    generic_reg u_ac (
        .clk(clk), .reset(reset), .load(ac_load),
        .data_in(ac_in), .value(ac)
    );

    // Flag Logic
    // N and Z flags are based on AC input (for loads) or ALU result
    assign N_in = ac_in[7];
    assign Z_in = (ac_in == 8'h00);
    // Carry flag comes directly from ALU
    assign C_in = alu_carry;

    nzc_reg u_nzc (
        .clk(clk), .reset(reset),
        .nz_load(nz_load), .c_load(c_load),
        .N_in(N_in), .Z_in(Z_in), .C_in(C_in),
        .N_flag(flagN), .Z_flag(flagZ), .C_flag(flagC)
    );

endmodule
