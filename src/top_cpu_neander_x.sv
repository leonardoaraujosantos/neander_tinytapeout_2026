// ============================================================================
// top_cpu_neander_x.sv — CPU TOP for NEANDER-X (LCC + X/Y/FP Register + Carry Flag)
// ============================================================================

module cpu_top (
    input  logic       clk,
    input  logic       reset,

    // Interface com RAM (SPI memory controller handshaking)
    output logic [7:0] mem_addr,
    output logic [7:0] mem_data_out,
    input  logic [7:0] mem_data_in,
    output logic       mem_write,
    output logic       mem_read,
    output logic       mem_req,       // Memory access request (to SPI controller)
    input  logic       mem_ready,     // Memory access complete (from SPI controller)

    // Interface com I/O
    input  logic [7:0] io_in,
    input  logic [7:0] io_status,
    output logic [7:0] io_out,
    output logic       io_write,

    // Debug
    output logic [7:0] dbg_pc,
    output logic [7:0] dbg_ac,
    output logic [7:0] dbg_ri,
    output logic [7:0] dbg_sp,
    output logic [7:0] dbg_x,     // X register debug output
    output logic [7:0] dbg_y,     // Y register debug output
    output logic [7:0] dbg_fp     // FP register debug output
);

    logic       pc_inc, pc_load;
    logic       ac_load, ri_load, rem_load, rdm_load, nz_load;
    logic       c_load;          // Carry flag load (LCC extension)
    logic [1:0] addr_sel;
    logic [3:0] alu_op;          // Extended to 4 bits for NEG
    logic [3:0] opcode;
    logic [3:0] sub_opcode;
    logic       flagN, flagZ;
    logic       flagC;           // Carry flag (LCC extension)
    logic       io_write_ctrl;
    logic       sp_inc, sp_dec;
    logic [1:0] mem_data_sel;    // Extended to 2 bits for X register
    logic [1:0] alu_b_sel;       // ALU B input select: 00=mem, 01=const1, 10=X
    // X Register Extension signals
    logic       x_load;          // Load X register
    logic       x_inc;           // Increment X register
    logic       x_to_ac;         // Transfer X to AC (TXA)
    logic       indexed_mode;    // Use indexed addressing (addr + X)
    // Y Register Extension signals
    logic       y_load;          // Load Y register
    logic       y_inc;           // Increment Y register
    logic       y_to_ac;         // Transfer Y to AC (TYA)
    logic       indexed_mode_y;  // Use indexed addressing (addr + Y)
    logic       mul_to_y;        // Load Y with MUL high byte
    // Frame Pointer Extension signals
    logic       fp_load;         // Load FP register
    logic       sp_load;         // Load SP from FP (for TFS)
    logic       indexed_mode_fp; // Use indexed addressing (addr + FP)
    logic [2:0] mem_data_sel_ext; // Extended: 000=AC, 001=PC, 010=X, 011=Y, 100=FP

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
        .c_load(c_load),           // Carry flag load (LCC extension)
        .addr_sel(addr_sel),
        .alu_op(alu_op),
        .sp_inc(sp_inc),
        .sp_dec(sp_dec),
        .mem_data_sel(mem_data_sel),
        .alu_b_sel(alu_b_sel),
        // X Register Extension signals
        .x_load(x_load),
        .x_inc(x_inc),
        .x_to_ac(x_to_ac),
        .indexed_mode(indexed_mode),
        // Y Register Extension signals
        .y_load(y_load),
        .y_inc(y_inc),
        .y_to_ac(y_to_ac),
        .indexed_mode_y(indexed_mode_y),
        .mul_to_y(mul_to_y),
        // Frame Pointer Extension signals
        .fp_load(fp_load),
        .sp_load(sp_load),
        .indexed_mode_fp(indexed_mode_fp),
        .mem_data_sel_ext(mem_data_sel_ext),
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
        .sub_opcode(sub_opcode),
        .flagN(flagN),
        .flagZ(flagZ),
        .flagC(flagC),             // Carry flag output (LCC extension)
        .dbg_pc(dbg_pc),
        .dbg_ac(dbg_ac),
        .dbg_ri(dbg_ri),
        .dbg_sp(dbg_sp),
        .dbg_x(dbg_x),
        .dbg_y(dbg_y),
        .dbg_fp(dbg_fp)
    );

    neander_control uc (
        .clk(clk),
        .reset(reset),
        .opcode(opcode),
        .sub_opcode(sub_opcode),
        .flagN(flagN),
        .flagZ(flagZ),
        .flagC(flagC),             // Carry flag input (LCC extension)
        .mem_ready(mem_ready),     // Memory access complete (from SPI controller)
        .mem_read(mem_read),
        .mem_write(mem_write),
        .mem_req(mem_req),         // Memory access request (to SPI controller)
        .pc_inc(pc_inc),
        .pc_load(pc_load),
        .ac_load(ac_load),
        .ri_load(ri_load),
        .rem_load(rem_load),
        .rdm_load(rdm_load),
        .nz_load(nz_load),
        .c_load(c_load),           // Carry flag load (LCC extension)
        .addr_sel(addr_sel),
        .alu_op(alu_op),
        .io_write(io_write_ctrl),
        .sp_inc(sp_inc),
        .sp_dec(sp_dec),
        .mem_data_sel(mem_data_sel),
        .alu_b_sel(alu_b_sel),
        // X Register Extension signals
        .x_load(x_load),
        .x_inc(x_inc),
        .x_to_ac(x_to_ac),
        .indexed_mode(indexed_mode),
        // Y Register Extension signals
        .y_load(y_load),
        .y_inc(y_inc),
        .y_to_ac(y_to_ac),
        .indexed_mode_y(indexed_mode_y),
        .mul_to_y(mul_to_y),
        // Frame Pointer Extension signals
        .fp_load(fp_load),
        .sp_load(sp_load),
        .indexed_mode_fp(indexed_mode_fp),
        .mem_data_sel_ext(mem_data_sel_ext)
    );

endmodule
