// ============================================================================
// neander_x_datapath.sv — Datapath for NEANDER-X CPU
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
// NZ Register (Flags)
// ---------------------------------------------------------------------------
module nz_reg (
    input  logic clk,
    input  logic reset,
    input  logic nz_load,
    input  logic N_in,
    input  logic Z_in,
    output logic N_flag,
    output logic Z_flag
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            N_flag <= 1'b0;
            Z_flag <= 1'b0;
        end
        else if (nz_load) begin
            N_flag <= N_in;
            Z_flag <= Z_in;
        end
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
    input  logic [1:0] addr_sel,  // 00=RDM, 01=PC, 10=SP
    input  logic [1:0] alu_op,
    input  logic       sp_inc,    // Stack pointer increment (POP/RET)
    input  logic       sp_dec,    // Stack pointer decrement (PUSH/CALL)
    input  logic       mem_data_sel, // Memory data select: 0=AC, 1=PC (for CALL)

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
    output logic [7:0] dbg_pc,
    output logic [7:0] dbg_ac,
    output logic [7:0] dbg_ri,
    output logic [7:0] dbg_sp
);

    // Internal Signals (using 'logic' for everything)
    logic [7:0] pc, rem, rdm, ri, ac, sp;
    logic [7:0] alu_res;
    logic [7:0] addr_mux;
    logic [7:0] ac_in;
    logic       N_in, Z_in;

    // Direct assignments
    assign mem_addr     = rem;
    assign mem_data_out = mem_data_sel ? pc : ac;  // MUX: 0=AC (STA/PUSH), 1=PC (CALL)
    assign io_out       = ac;
    assign io_write     = io_write_ctrl;
    assign opcode       = ri[7:4];
    assign sub_opcode   = ri[3:0];  // Lower nibble for PUSH/POP/CALL/RET

    // Debug outputs
    assign dbg_pc = pc;
    assign dbg_ac = ac;
    assign dbg_ri = ri;
    assign dbg_sp = sp;

    // --- Instantiations ---

    pc_reg u_pc (
        .clk(clk), .reset(reset),
        .pc_inc(pc_inc), .pc_load(pc_load),
        .data_in(rdm), .pc_value(pc)
    );

    sp_reg u_sp (
        .clk(clk), .reset(reset),
        .sp_inc(sp_inc), .sp_dec(sp_dec), .sp_load(1'b0),
        .data_in(8'h00), .sp_value(sp)
    );

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

    neander_alu u_alu (
        .a(ac), .b(mem_data_in), .alu_op(alu_op), .result(alu_res)
    );

    // Combinational Logic for AC Input Mux
    always_comb begin
        if (opcode == 4'h2 || opcode == 4'hE) begin
            // LDA or LDI
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
    assign N_in = ac_in[7];
    assign Z_in = (ac_in == 8'h00);

    nz_reg u_nz (
        .clk(clk), .reset(reset), .nz_load(nz_load),
        .N_in(N_in), .Z_in(Z_in), .N_flag(flagN), .Z_flag(flagZ)
    );

endmodule
