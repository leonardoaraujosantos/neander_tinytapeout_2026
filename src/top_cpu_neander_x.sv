// ============================================================================
// cpu_top.sv — CPU NEANDER converted to SystemVerilog
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
// ALU
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// MUX PC/RDM -> REM
// ---------------------------------------------------------------------------
module mux_pc_rdm (
    input  logic       sel, // 1 = PC, 0 = RDM
    input  logic [7:0] pc,
    input  logic [7:0] rdm,
    output logic [7:0] out
);
    assign out = sel ? pc : rdm;
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
    input  logic       addr_sel_pc,
    input  logic [1:0] alu_op,

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
    output logic       flagN,
    output logic       flagZ,
    output logic [7:0] dbg_pc,
    output logic [7:0] dbg_ac,
    output logic [7:0] dbg_ri
);

    // Internal Signals (using 'logic' for everything)
    logic [7:0] pc, rem, rdm, ri, ac;
    logic [7:0] alu_res;
    logic [7:0] addr_mux;
    logic [7:0] ac_in;
    logic       N_in, Z_in;

    // Direct assignments
    assign mem_addr     = rem;
    assign mem_data_out = ac;
    assign io_out       = ac;
    assign io_write     = io_write_ctrl;
    assign opcode       = ri[7:4];
    
    // Debug outputs
    assign dbg_pc = pc;
    assign dbg_ac = ac;
    assign dbg_ri = ri;

    // --- Instantiations ---

    pc_reg u_pc (
        .clk(clk), .reset(reset),
        .pc_inc(pc_inc), .pc_load(pc_load),
        .data_in(rdm), .pc_value(pc)
    );

    mux_pc_rdm u_mux (
        .sel(addr_sel_pc), .pc(pc), .rdm(rdm), .out(addr_mux)
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

// ============================================================================
// CONTROL UNIT
// ============================================================================
module neander_control (
    input  logic       clk,
    input  logic       reset,
    input  logic [3:0] opcode,
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
    output logic       addr_sel_pc,
    output logic [1:0] alu_op,
    output logic       io_write
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
        addr_sel_pc = 1'b1;
        alu_op      = 2'b00;
        io_write    = '0;
        
        next_state  = state;

        case (state)
            // --- FETCH ---
            S_FETCH_1: begin
                addr_sel_pc = 1; // PC -> REM
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
                addr_sel_pc = 1; 
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
                addr_sel_pc = 1;
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
                addr_sel_pc = 0; // RDM -> REM
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
                addr_sel_pc = 1;
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
                addr_sel_pc = 0;
                rem_load    = 1;
                next_state  = S_STA_4;
            end
            S_STA_4: begin
                mem_write   = 1;
                next_state  = S_FETCH_1;
            end

            // --- ADD ---
            S_ADD_1: begin
                addr_sel_pc = 1;
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
                addr_sel_pc = 0;
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
                addr_sel_pc = 1; rem_load = 1; mem_read = 1; next_state = S_AND_2;
            end
            S_AND_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_AND_3;
            end
            S_AND_3: begin
                addr_sel_pc = 0; rem_load = 1; mem_read = 1; next_state = S_AND_4;
            end
            S_AND_4: begin
                mem_read = 1; ac_load = 1; alu_op = 2'b01; nz_load = 1; next_state = S_FETCH_1;
            end

            // --- OR ---
            S_OR_1: begin
                addr_sel_pc = 1; rem_load = 1; mem_read = 1; next_state = S_OR_2;
            end
            S_OR_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_OR_3;
            end
            S_OR_3: begin
                addr_sel_pc = 0; rem_load = 1; mem_read = 1; next_state = S_OR_4;
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
                addr_sel_pc = 1; rem_load = 1; mem_read = 1; next_state = S_JMP_2;
            end
            S_JMP_2: begin
                mem_read = 1; rdm_load = 1; next_state = S_JMP_3;
            end
            S_JMP_3: begin
                pc_load = 1; next_state = S_FETCH_1;
            end

            // --- JN ---
            S_JN_1: begin
                addr_sel_pc = 1; rem_load = 1; mem_read = 1; next_state = S_JN_2;
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
                addr_sel_pc = 1; rem_load = 1; mem_read = 1; next_state = S_JZ_2;
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
                addr_sel_pc = 1; rem_load = 1; mem_read = 1; next_state = S_JNZ_2;
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
                addr_sel_pc = 1; rem_load = 1; mem_read = 1; next_state = S_IN_2;
            end
            S_IN_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_IN_3;
            end
            S_IN_3: begin
                ac_load = 1; nz_load = 1; next_state = S_FETCH_1;
            end

            // --- OUT ---
            S_OUT_1: begin
                addr_sel_pc = 1; rem_load = 1; mem_read = 1; next_state = S_OUT_2;
            end
            S_OUT_2: begin
                mem_read = 1; rdm_load = 1; pc_inc = 1; next_state = S_OUT_3;
            end
            S_OUT_3: begin
                io_write = 1; next_state = S_FETCH_1;
            end

            // --- HLT ---
            S_HLT: begin
                next_state = S_HLT;
            end

            default: next_state = S_FETCH_1;
        endcase
    end

endmodule

// ============================================================================
// CPU TOP
// ============================================================================
module cpu_top (
    input  logic       clk,
    input  logic       reset,

    // Interface com RAM
    output logic [7:0] mem_addr,
    output logic [7:0] mem_data_out,
    input  logic [7:0] mem_data_in,
    output logic       mem_write,

    // Interface com I/O
    input  logic [7:0] io_in,
    input  logic [7:0] io_status,
    output logic [7:0] io_out,
    output logic       io_write,

    // Debug
    output logic [7:0] dbg_pc,
    output logic [7:0] dbg_ac,
    output logic [7:0] dbg_ri
);

    logic       mem_read;
    logic       pc_inc, pc_load;
    logic       ac_load, ri_load, rem_load, rdm_load, nz_load;
    logic       addr_sel_pc;
    logic [1:0] alu_op;
    logic [3:0] opcode;
    logic       flagN, flagZ;
    logic       io_write_ctrl;

    neander_datapath dp (
        .clk(clk),
        .reset(reset),
        // Control Inputs
        .mem_read(mem_read),
        .mem_write(mem_write),
        .pc_inc(pc_inc),
        .pc_load(pc_load),
        .ac_load(ac_load),
        .ri_load(ri_load),
        .rem_load(rem_load),
        .rdm_load(rdm_load),
        .nz_load(nz_load),
        .addr_sel_pc(addr_sel_pc),
        .alu_op(alu_op),
        .io_write_ctrl(io_write_ctrl),
        // Data/Status I/O
        .mem_data_in(mem_data_in),
        .mem_addr(mem_addr),
        .mem_data_out(mem_data_out),
        .io_in(io_in),
        .io_status(io_status),
        .io_out(io_out),
        .io_write(io_write),
        .opcode(opcode),
        .flagN(flagN),
        .flagZ(flagZ),
        .dbg_pc(dbg_pc),
        .dbg_ac(dbg_ac),
        .dbg_ri(dbg_ri)
    );

    neander_control uc (
        .clk(clk),
        .reset(reset),
        .opcode(opcode),
        .flagN(flagN),
        .flagZ(flagZ),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .pc_inc(pc_inc),
        .pc_load(pc_load),
        .ac_load(ac_load),
        .ri_load(ri_load),
        .rem_load(rem_load),
        .rdm_load(rdm_load),
        .nz_load(nz_load),
        .addr_sel_pc(addr_sel_pc),
        .alu_op(alu_op),
        .io_write(io_write_ctrl)
    );

endmodule
