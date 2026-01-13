`default_nettype none
`timescale 1ns / 1ps

/* This testbench instantiates the CPU project and SPI SRAM model.
   The SPI signals are connected between the project and SRAM.
   Tests can load memory directly via the spi_ram.memory array.
*/
module tb ();

  // Dump the signals to a FST file. You can view it with gtkwave or surfer.
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // SPI signals extracted from project outputs/inputs
  wire spi_cs_n  = uo_out[0];   // SPI Chip Select
  wire spi_sclk  = uo_out[1];   // SPI Clock
  wire spi_mosi  = uo_out[2];   // SPI MOSI
  wire spi_miso;                 // SPI MISO (from SRAM to CPU)
  wire io_write  = uo_out[7];   // I/O write strobe

  // Debug signals from uio_out
  wire [7:0] dbg_pc = uio_out;

  // Connect SPI MISO to ui_in[0]
  always @(*) begin
    ui_in[0] = spi_miso;
  end

  // CPU Project (TinyTapeout module)
  tt_um_cpu_leonardoaraujosantos user_project (
      .ui_in  (ui_in),    // Dedicated inputs
      .uo_out (uo_out),   // Dedicated outputs
      .uio_in (uio_in),   // IOs: Input path
      .uio_out(uio_out),  // IOs: Output path
      .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
      .ena    (ena),      // enable - goes high when design is selected
      .clk    (clk),      // clock
      .rst_n  (rst_n)     // not reset
  );

  // SPI SRAM Model
  spi_sram_model spi_ram (
      .spi_cs_n (spi_cs_n),
      .spi_sclk (spi_sclk),
      .spi_mosi (spi_mosi),
      .spi_miso (spi_miso)
  );

endmodule
