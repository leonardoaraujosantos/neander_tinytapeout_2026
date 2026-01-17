// ============================================================================
// neander_x_datapath.sv — Datapath for NEANDER-X CPU (16-bit Data Width)
// ============================================================================
// 16-bit data width extension:
//   - AC, X, Y registers are now 16-bit
//   - Memory data bus is 16-bit
//   - ALU operates on 16-bit values
//
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
// Generic Register Module (Can be reused) - 8-bit version (for RI opcode register)
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
// Generic Register Module - 16-bit version (for PC, SP, REM)
// ---------------------------------------------------------------------------
module generic_reg_16 (
    input  logic        clk,
    input  logic        reset,
    input  logic        load,
    input  logic [15:0] data_in,
    output logic [15:0] value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            value <= 16'h0000;
        else if (load)
            value <= data_in;
    end
endmodule

// ---------------------------------------------------------------------------
// RDM Register - 16-bit with separate low/high byte loading for address fetch
// Also supports full 16-bit load from memory for indirect addressing
// ---------------------------------------------------------------------------
module rdm_reg (
    input  logic        clk,
    input  logic        reset,
    input  logic        load_lo,      // Load low byte from data_in (for address fetch)
    input  logic        load_hi,      // Load high byte from data_in (for address fetch)
    input  logic        load_full,    // Load full 16-bit from data_full (for 16-bit memory)
    input  logic        load_lo_alt,  // Load full 16-bit from alternate source (for indirect addressing)
    input  logic        rdm_inc,      // Increment RDM by 1 (for indirect addressing)
    input  logic [7:0]  data_in,      // 8-bit data from memory (for address bytes)
    input  logic [15:0] data_full,    // 16-bit data from memory (for 16-bit operations)
    input  logic [15:0] alt_data,     // Alternate data source (swap_temp for indirect, 16-bit)
    output logic [15:0] value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            value <= 16'h0000;
        else if (rdm_inc)
            value <= value + 16'h0001;  // Increment RDM by 1
        else if (load_full)
            value <= data_full;         // Load full 16-bit from memory
        else if (load_lo_alt)
            value <= alt_data;          // Load full 16-bit from alternate source (swap_temp)
        else begin
            if (load_lo)
                value[7:0] <= data_in;
            if (load_hi)
                value[15:8] <= data_in;
        end
    end
endmodule

// ---------------------------------------------------------------------------
// PC Register (Specific behavior: Increment & Load) - 16-bit for 64KB addressing
// ---------------------------------------------------------------------------
module pc_reg (
    input  logic        clk,
    input  logic        reset,
    input  logic        pc_inc,
    input  logic        pc_inc_2,    // Increment by 2 (for 16-bit immediate fetch)
    input  logic        pc_load,
    input  logic [15:0] data_in,
    output logic [15:0] pc_value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            pc_value <= 16'h0000;
        else if (pc_load)
            pc_value <= data_in;
        else if (pc_inc_2)
            pc_value <= pc_value + 16'h0002;  // Increment by 2 for 16-bit values
        else if (pc_inc)
            pc_value <= pc_value + 16'h0001;
    end
endmodule

// ---------------------------------------------------------------------------
// SP Register (Stack Pointer: Increment, Decrement & Load) - 16-bit for 64KB addressing
// Initialized to 0xFFFF (stack grows downward from top of memory)
// ---------------------------------------------------------------------------
module sp_reg (
    input  logic        clk,
    input  logic        reset,
    input  logic        sp_inc,
    input  logic        sp_dec,
    input  logic        sp_load,
    input  logic [15:0] data_in,
    output logic [15:0] sp_value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            sp_value <= 16'h00FF;  // Stack starts at top of page 0 for compatibility
        else if (sp_load)
            sp_value <= data_in;
        else if (sp_dec)
            sp_value <= sp_value - 16'h0001;  // PUSH: decrement first, then write
        else if (sp_inc)
            sp_value <= sp_value + 16'h0001;  // POP: read first, then increment
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
// X Index Register (Load, Increment & Decrement) - 16-bit
// ---------------------------------------------------------------------------
module x_reg (
    input  logic        clk,
    input  logic        reset,
    input  logic        x_load,
    input  logic        x_inc,
    input  logic        x_dec,
    input  logic [15:0] data_in,
    output logic [15:0] x_value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            x_value <= 16'h0000;
        else if (x_load)
            x_value <= data_in;
        else if (x_inc)
            x_value <= x_value + 16'h0001;
        else if (x_dec)
            x_value <= x_value - 16'h0001;
    end
endmodule

// ---------------------------------------------------------------------------
// Y Index Register (Load, Increment & Decrement) - 16-bit
// ---------------------------------------------------------------------------
module y_reg (
    input  logic        clk,
    input  logic        reset,
    input  logic        y_load,
    input  logic        y_inc,
    input  logic        y_dec,
    input  logic [15:0] data_in,
    output logic [15:0] y_value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            y_value <= 16'h0000;
        else if (y_load)
            y_value <= data_in;
        else if (y_inc)
            y_value <= y_value + 16'h0001;
        else if (y_dec)
            y_value <= y_value - 16'h0001;
    end
endmodule

// ---------------------------------------------------------------------------
// B Index Register (Load, Increment & Decrement) - 16-bit
// ---------------------------------------------------------------------------
module b_reg (
    input  logic        clk,
    input  logic        reset,
    input  logic        b_load,
    input  logic        b_inc,
    input  logic        b_dec,
    input  logic [15:0] data_in,
    output logic [15:0] b_value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            b_value <= 16'h0000;
        else if (b_load)
            b_value <= data_in;
        else if (b_inc)
            b_value <= b_value + 16'h0001;
        else if (b_dec)
            b_value <= b_value - 16'h0001;
    end
endmodule

// ---------------------------------------------------------------------------
// FP Frame Pointer Register - 16-bit for 64KB addressing
// Supports full 16-bit load (TSF) or from memory (POP_FP)
// ---------------------------------------------------------------------------
module fp_reg (
    input  logic        clk,
    input  logic        reset,
    input  logic        fp_load,      // Load full 16-bit value (for TSF)
    input  logic        fp_load_lo,   // Load from memory data (for POP_FP) - now loads full 16-bit
    input  logic        fp_load_hi,   // Reserved for compatibility (unused in 16-bit mode)
    input  logic [15:0] data_in,      // 16-bit input (from SP for TSF)
    input  logic [15:0] data_in_byte, // 16-bit input (from memory for POP_FP)
    output logic [15:0] fp_value
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            fp_value <= 16'h0000;
        else if (fp_load)
            fp_value <= data_in;
        else if (fp_load_lo)
            fp_value <= data_in_byte;  // Load full 16-bit from memory
        // fp_load_hi is unused in 16-bit mode but kept for interface compatibility
    end
endmodule

// ---------------------------------------------------------------------------
// MUX PC/RDM/SP -> REM (3-way address mux) - 16-bit for 64KB addressing
// ---------------------------------------------------------------------------
module mux_addr (
    input  logic [1:0]  sel, // 00 = RDM, 01 = PC, 10 = SP
    input  logic [15:0] pc,
    input  logic [15:0] rdm,
    input  logic [15:0] sp,
    output logic [15:0] out
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
// DATAPATH (16-bit Data Width)
// ============================================================================
module neander_datapath (
    input  logic       clk,
    input  logic       reset,

    // Control Signals
    input  logic       mem_read, // Used for RDM load logic
    input  logic       mem_write,
    input  logic       pc_inc,
    input  logic       pc_inc_2,  // Increment PC by 2 (for 16-bit immediate fetch)
    input  logic       pc_load,
    input  logic       ac_load,
    input  logic       ri_load,
    input  logic       rem_load,
    input  logic       rdm_load,     // Load RDM low byte (for 16-bit address fetch)
    input  logic       rdm_load_hi,  // Load RDM high byte (for 16-bit address fetch)
    input  logic       rdm_load_full, // Load full 16-bit RDM (for 16-bit memory operations)
    input  logic       nz_load,
    input  logic       c_load,   // Carry flag load (for CMP and arithmetic ops)
    input  logic [1:0] addr_sel,  // 00=RDM, 01=PC, 10=SP
    input  logic [3:0] alu_op,    // Extended to 4 bits for NEG and future ops
    input  logic       sp_inc,    // Stack pointer increment (POP/RET)
    input  logic       sp_dec,    // Stack pointer decrement (PUSH/CALL)
    input  logic [1:0] mem_data_sel, // Memory data select: 00=AC, 01=PC, 10=X, 11=Y
    input  logic [2:0] alu_b_sel, // ALU B input select: 000=mem_data, 001=constant 1 (INC/DEC), 010=X (MUL), 011=Y
    // X Register Extension signals
    input  logic       x_load,    // Load X register
    input  logic       x_inc,     // Increment X register
    input  logic       x_dec,     // Decrement X register (DEX)
    input  logic       x_to_ac,   // Transfer X to AC (TXA)
    input  logic       indexed_mode, // Use indexed addressing (addr + X)
    // Y Register Extension signals
    input  logic       y_load,    // Load Y register
    input  logic       y_inc,     // Increment Y register
    input  logic       y_dec,     // Decrement Y register (DEY)
    input  logic       y_to_ac,   // Transfer Y to AC (TYA)
    input  logic       indexed_mode_y, // Use indexed addressing (addr + Y)
    input  logic       mul_to_y,  // Load Y with high word of multiplication result
    // B Register Extension signals
    input  logic       b_load,    // Load B register
    input  logic       b_inc,     // Increment B register
    input  logic       b_dec,     // Decrement B register (DEB)
    input  logic       b_to_ac,   // Transfer B to AC (TBA)
    input  logic       b_from_temp, // Load B from swap temp (for SWPB)
    // Frame Pointer Extension signals
    input  logic       fp_load,      // Load FP full 16-bit (for TSF)
    input  logic       fp_load_lo,   // Load FP low byte (for POP_FP)
    input  logic       fp_load_hi,   // Load FP high byte (for POP_FP)
    input  logic       sp_load,      // Load SP from FP (for TFS)
    input  logic       indexed_mode_fp, // Use indexed addressing (addr + FP)
    input  logic       indexed_mode_sp, // Use indexed addressing (addr + SP)
    input  logic [2:0] mem_data_sel_ext, // Extended: 000=AC, 001=PC_LO, 010=X, 011=Y, 100=FP_LO, 101=PC_HI, 110=FP_HI
    // Swap instruction support
    input  logic       swap_temp_load,   // Load swap temp register from AC
    input  logic       x_from_temp,      // Load X from swap temp (for SWPX)
    input  logic       y_from_temp,      // Load Y from swap temp (for SWPY)
    // Indirect addressing support
    input  logic       ind_temp_load,    // Load swap_temp from mem_data_in (for indirect)
    input  logic       rdm_lo_from_temp, // Load RDM low byte from swap_temp
    input  logic       rdm_inc,          // Increment RDM by 1
    // Sequential divider interface (area-efficient DIV/MOD)
    input  logic       div_start,        // Start sequential division
    output logic       div_busy,         // Division in progress
    output logic       div_done,         // Division complete (pulse)
    // Sequential multiplier interface (area-efficient MUL)
    input  logic       mul_start,        // Start sequential multiplication
    output logic       mul_busy,         // Multiplication in progress
    output logic       mul_done,         // Multiplication complete (pulse)

    // External RAM Interface (16-bit addressing, 16-bit data)
    input  logic [15:0] mem_data_in,
    output logic [15:0] mem_addr,
    output logic [15:0] mem_data_out,

    // I/O Interface (remains 8-bit for peripheral compatibility)
    input  logic [7:0] io_in,
    input  logic [7:0] io_status,
    output logic [7:0] io_out,
    input  logic       io_write_ctrl,
    output logic       io_write,

    // Output to Control Unit / Debug
    output logic [3:0]  opcode,
    output logic [3:0]  sub_opcode, // Lower nibble for stack ops
    output logic        flagN,
    output logic        flagZ,
    output logic        flagC,      // Carry flag output
    output logic [15:0] dbg_pc,     // 16-bit PC for 64KB addressing
    output logic [15:0] dbg_ac,     // 16-bit AC
    output logic [7:0]  dbg_ri,
    output logic [15:0] dbg_sp,     // 16-bit SP for 64KB addressing
    output logic [15:0] dbg_x,      // 16-bit X register debug output
    output logic [15:0] dbg_y,      // 16-bit Y register debug output
    output logic [15:0] dbg_b,      // 16-bit B register debug output
    output logic [15:0] dbg_fp      // 16-bit FP for 64KB addressing
);

    // Internal Signals (using 'logic' for everything)
    // 16-bit registers for 64KB addressing
    logic [15:0] pc, rem, rdm, sp, fp;
    // 16-bit data registers
    logic [15:0] ac, x, y, b;
    logic [7:0] ri;  // Opcode register stays 8-bit
    logic [15:0] swap_temp;  // Temporary register for SWPX/SWPY instructions (16-bit)
    logic [15:0] alu_res;
    logic [15:0] alu_mul_high;  // High word of multiplication result from ALU
    logic        alu_carry;  // Carry output from ALU
    // Sequential divider signals
    logic [15:0] div_quotient;   // Quotient from sequential divider
    logic [15:0] div_remainder;  // Remainder from sequential divider
    logic        div_by_zero;    // Division by zero error flag
    // Sequential multiplier signals
    logic [15:0] mul_product_low;  // Low word from sequential multiplier
    logic [15:0] mul_product_high; // High word from sequential multiplier
    logic [15:0] alu_b_in;  // ALU B input (mem_data or constant 1)
    logic [15:0] addr_mux;
    logic [15:0] addr_indexed;  // Address + X for indexed modes
    logic [15:0] addr_indexed_y; // Address + Y for indexed modes
    logic [15:0] addr_indexed_fp; // Address + FP for indexed modes
    logic [15:0] ac_in;
    logic [15:0] x_in;     // X register input (from AC for TAX, from swap_temp for SWPX, or from memory for LDX)
    logic [15:0] y_in;     // Y register input (from AC for TAY, from swap_temp for SWPY, or from memory for LDY)
    logic [15:0] b_in;     // B register input (from AC for TAB, from swap_temp for SWPB, or from memory for LDB)
    logic        N_in, Z_in, C_in;

    // Indexed address calculation: addr + X or addr + Y or addr + FP
    // X and Y are now 16-bit
    assign addr_indexed = rdm + x;
    assign addr_indexed_y = rdm + y;
    assign addr_indexed_fp = rdm + fp;

    // Direct assignments
    // Memory address: use indexed address when indexed_mode, indexed_mode_y, indexed_mode_fp, or indexed_mode_sp is set
    // X, Y, FP, and SP are all 16-bit
    assign mem_addr = indexed_mode_sp ? (rem + sp) :
                      (indexed_mode_fp ? (rem + fp) :
                       (indexed_mode_y ? (rem + y) :
                        (indexed_mode ? (rem + x) : rem)));
    // Memory data output MUX (16-bit data):
    // 000=AC, 001=PC, 010=X, 011=Y, 100=FP, 101=B, 110=swap_temp (for PUSH_ADDR/POP_ADDR)
    always_comb begin
        case (mem_data_sel_ext)
            3'b000:  mem_data_out = ac;        // STA, PUSH
            3'b001:  mem_data_out = pc;        // CALL (return address)
            3'b010:  mem_data_out = x;         // STX
            3'b011:  mem_data_out = y;         // STY
            3'b100:  mem_data_out = fp;        // PUSH_FP
            3'b101:  mem_data_out = b;         // STB
            3'b110:  mem_data_out = swap_temp; // PUSH_ADDR/POP_ADDR (value from memory)
            default: mem_data_out = ac;
        endcase
    end
    assign io_out       = ac[7:0];  // I/O uses low byte of AC
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
    assign dbg_b  = b;  // B register debug output
    assign dbg_fp = fp; // FP register debug output

    // --- Instantiations ---

    pc_reg u_pc (
        .clk(clk), .reset(reset),
        .pc_inc(pc_inc), .pc_inc_2(pc_inc_2), .pc_load(pc_load),
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
        .x_load(x_load), .x_inc(x_inc), .x_dec(x_dec),
        .data_in(x_in), .x_value(x)
    );

    // X register input mux: select between mem_data_in (LDX/LDXI), AC (TAX), or swap_temp (SWPX)
    // Priority: x_from_temp > TAX > LDX/LDXI
    assign x_in = x_from_temp ? swap_temp :
                  (opcode == 4'h7 && sub_opcode == 4'hD) ? ac : mem_data_in;

    // Y Index Register
    // Input can be from memory (LDY), immediate (LDYI), or AC (TAY)
    y_reg u_y (
        .clk(clk), .reset(reset),
        .y_load(y_load), .y_inc(y_inc), .y_dec(y_dec),
        .data_in(y_in), .y_value(y)
    );

    // Y register input mux: select between mul_high (MUL), AC (TAY), swap_temp (SWPY), or mem_data_in
    // Priority: y_from_temp > mul_to_y > TAY > LDY/LDYI
    assign y_in = y_from_temp ? swap_temp :
                  mul_to_y ? alu_mul_high :
                  (opcode == 4'h0 && sub_opcode == 4'h3) ? ac : mem_data_in;

    // B Index Register
    // Input can be from memory (LDB), immediate (LDBI), or AC (TAB)
    b_reg u_b (
        .clk(clk), .reset(reset),
        .b_load(b_load), .b_inc(b_inc), .b_dec(b_dec),
        .data_in(b_in), .b_value(b)
    );

    // B register input mux: select between mem_data_in (LDB/LDBI), AC (TAB), or swap_temp (SWPB)
    // Priority: b_from_temp > TAB (0x1C) > LDB/LDBI
    assign b_in = b_from_temp ? swap_temp :
                  (opcode == 4'h1 && sub_opcode == 4'hC) ? ac : mem_data_in;

    // FP Frame Pointer Register - 16-bit
    // Input can be from SP (TSF via fp_load) or from memory (POP_FP via fp_load_lo)
    fp_reg u_fp (
        .clk(clk), .reset(reset),
        .fp_load(fp_load),           // Full 16-bit load from SP (TSF)
        .fp_load_lo(fp_load_lo),     // Full 16-bit from memory (POP_FP)
        .fp_load_hi(fp_load_hi),     // Reserved for compatibility
        .data_in(sp),                // 16-bit input from SP (for TSF)
        .data_in_byte(mem_data_in),  // 16-bit input from memory (for POP_FP)
        .fp_value(fp)
    );

    // Swap temp register - used for SWPX/SWPY atomic swap operations and indirect addressing
    // For SWPX/SWPY: Stores AC value temporarily while loading X/Y into AC
    // For indirect addressing: Stores intermediate pointer from memory
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            swap_temp <= 16'h0000;
        else if (swap_temp_load)
            swap_temp <= ac;
        else if (ind_temp_load)
            swap_temp <= mem_data_in;  // Load from memory for indirect addressing
    end

    mux_addr u_mux (
        .sel(addr_sel), .pc(pc), .rdm(rdm), .sp(sp), .out(addr_mux)
    );

    // REM register - 16-bit for 64KB addressing
    generic_reg_16 u_rem (
        .clk(clk), .reset(reset), .load(rem_load),
        .data_in(addr_mux), .value(rem)
    );

    // RDM register - 16-bit with separate low/high byte loading
    // Used for fetching 16-bit addresses in two 8-bit memory reads
    // Extended with indirect addressing support (load from temp, increment)
    rdm_reg u_rdm (
        .clk(clk), .reset(reset),
        .load_lo(rdm_load & mem_read),      // Load low byte when rdm_load & mem_read
        .load_hi(rdm_load_hi & mem_read),   // Load high byte when rdm_load_hi & mem_read
        .load_full(rdm_load_full & mem_read), // Load full 16-bit from memory (for 16-bit operations)
        .load_lo_alt(rdm_lo_from_temp),     // Load full 16-bit from swap_temp (indirect addressing)
        .rdm_inc(rdm_inc),                  // Increment RDM by 1 (indirect addressing)
        .data_in(mem_data_in[7:0]),         // Use low byte for address byte loading
        .data_full(mem_data_in),            // Full 16-bit from memory
        .alt_data(swap_temp),               // Alternate source: swap_temp (16-bit)
        .value(rdm)
    );

    // RI register - 8-bit, loads opcode from RDM low byte
    generic_reg u_ri (
        .clk(clk), .reset(reset), .load(ri_load),
        .data_in(rdm[7:0]), .value(ri)
    );

    // ALU B input MUX: select between mem_data_in (000), constant 1 (001) for INC/DEC, X (010) for MUL/ADDX, Y (011) for ADDY, B (100) for ADDB/SUBB
    always_comb begin
        case (alu_b_sel)
            3'b000:  alu_b_in = mem_data_in;  // Default: memory data (16-bit)
            3'b001:  alu_b_in = 16'h0001;     // Constant 1 for INC/DEC
            3'b010:  alu_b_in = x;            // X register for MUL, ADDX, SUBX, etc.
            3'b011:  alu_b_in = y;            // Y register for ADDY, SUBY
            3'b100:  alu_b_in = b;            // B register for ADDB, SUBB
            default: alu_b_in = mem_data_in;
        endcase
    end

    // Sequential Divider for area-efficient DIV/MOD operations (16 cycles)
    // Dividend is AC, Divisor is from alu_b_in (usually X register)
    sequential_divider u_div (
        .clk(clk),
        .reset(reset),
        .start(div_start),
        .dividend(ac),
        .divisor(alu_b_in),
        .quotient(div_quotient),
        .remainder(div_remainder),
        .busy(div_busy),
        .done(div_done),
        .div_by_zero(div_by_zero)
    );

    // Sequential Multiplier for area-efficient MUL operations (16 cycles)
    // Multiplicand is AC, Multiplier is from alu_b_in (usually X register)
    sequential_multiplier u_mul (
        .clk(clk),
        .reset(reset),
        .start(mul_start),
        .multiplicand(ac),
        .multiplier(alu_b_in),
        .product_low(mul_product_low),
        .product_high(mul_product_high),
        .busy(mul_busy),
        .done(mul_done)
    );

    neander_alu u_alu (
        .a(ac), .b(alu_b_in), .alu_op(alu_op), .carry_in(flagC),
        .mul_product_low(mul_product_low),   // Sequential multiplier results
        .mul_product_high(mul_product_high),
        .div_quotient(div_quotient),         // Sequential divider results
        .div_remainder(div_remainder),
        .div_by_zero(div_by_zero),
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
        else if (b_to_ac) begin
            // TBA: Transfer B to AC
            ac_in = b;
        end
        else if (opcode == 4'h2 || (opcode == 4'hE && sub_opcode == 4'h0)) begin
            // LDA or LDI (0xE0 only - MULI/DIVI/CMPI use ALU result)
            ac_in = mem_data_in;
        end
        else if (opcode == 4'h7 && sub_opcode == 4'h1) begin
            // POP: load from memory (stack)
            ac_in = mem_data_in;
        end
        else if (opcode == 4'hC) begin
            // IN: 0=data, 1=status (use low byte of rdm for port selection)
            // I/O is 8-bit, zero-extend to 16-bit
            case (rdm[7:0])
                8'h00: ac_in = {8'h00, io_in};
                8'h01: ac_in = {8'h00, io_status};
                default: ac_in = 16'h0000;
            endcase
        end
        else begin
            // Arithmetic/Logic
            ac_in = alu_res;
        end
    end

    // AC register - 16-bit
    generic_reg_16 u_ac (
        .clk(clk), .reset(reset), .load(ac_load),
        .data_in(ac_in), .value(ac)
    );

    // Flag Logic
    // N and Z flags are based on AC input (for loads) or ALU result
    assign N_in = ac_in[15];  // Sign bit is now bit 15
    assign Z_in = (ac_in == 16'h0000);
    // Carry flag comes directly from ALU
    assign C_in = alu_carry;

    nzc_reg u_nzc (
        .clk(clk), .reset(reset),
        .nz_load(nz_load), .c_load(c_load),
        .N_in(N_in), .Z_in(Z_in), .C_in(C_in),
        .N_flag(flagN), .Z_flag(flagZ), .C_flag(flagC)
    );

endmodule
