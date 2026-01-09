// ============================================================================
// top_cpu_neander_x.sv — CPU TOP for NEANDER-X
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
