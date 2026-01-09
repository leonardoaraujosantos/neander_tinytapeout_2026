`timescale 1ns/1ps

module tb_cpu;

    logic clk = 0;
    logic reset = 1;

    logic [7:0] pc, ac;

    cpu_top dut(
        .clk(clk),
        .reset(reset),
        .dbg_pc(pc),
        .dbg_ac(ac)
    );

    // clock
    always #5 clk = ~clk;

    initial begin
        // Generate VCD waveform file
        $dumpfile("cpu_waveform_sv.vcd");
        $dumpvars(0, tb_cpu);

        $display("=== Iniciando simulação NEANDER ===");

        // reset
        #20 reset = 0;

        // monitor
        $monitor("t=%0t | PC=%0d | AC=%0d",
                  $time, pc, ac);

        // 200 ciclos
        #2000 $finish;
    end

endmodule
