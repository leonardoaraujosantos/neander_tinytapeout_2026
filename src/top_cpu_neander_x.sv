// ============================================================================
// top_cpu_neander_x.sv — CPU TOP for NEANDER-X (LCC + X Register + Carry Flag)
// ============================================================================

module cpu_top (
    input  logic       clk,
    input  logic       reset,

    // Interface com RAM
    output logic [7:0] mem_addr,
    output logic [7:0] mem_data_out,
    input  logic [7:0] mem_data_in,
    output logic       mem_write,
    output logic       mem_read,

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
    output logic [7:0] dbg_x      // X register debug output
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
    logic       alu_b_sel;       // ALU B input select for INC/DEC
    // X Register Extension signals
    logic       x_load;          // Load X register
    logic       x_inc;           // Increment X register
    logic       x_to_ac;         // Transfer X to AC (TXA)
    logic       indexed_mode;    // Use indexed addressing (addr + X)

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
        .dbg_x(dbg_x)
    );

    neander_control uc (
        .clk(clk),
        .reset(reset),
        .opcode(opcode),
        .sub_opcode(sub_opcode),
        .flagN(flagN),
        .flagZ(flagZ),
        .flagC(flagC),             // Carry flag input (LCC extension)
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
        .io_write(io_write_ctrl),
        .sp_inc(sp_inc),
        .sp_dec(sp_dec),
        .mem_data_sel(mem_data_sel),
        .alu_b_sel(alu_b_sel),
        // X Register Extension signals
        .x_load(x_load),
        .x_inc(x_inc),
        .x_to_ac(x_to_ac),
        .indexed_mode(indexed_mode)
    );

endmodule
