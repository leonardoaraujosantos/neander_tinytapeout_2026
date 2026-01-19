`default_nettype none
`timescale 1ns / 1ps

/* This testbench instantiates the CPU project (with interconnect hub) and SPI SRAM model.
   The SPI signals are connected between the project and SRAM.
   Tests can load memory directly via the spi_ram.memory array.

   Pin Mapping (per INTERCONNECT_SPEC.md Section 10):
   ui_in[0] = BOOT_SEL   - Boot mode selection (0=SPI RAM, 1=Debug ROM)
   ui_in[1] = IRQ_IN     - External interrupt input
   ui_in[2] = EXT_IN0    - External input 0
   ui_in[3] = EXT_IN1    - External input 1
   ui_in[7:4] = spare    - Spare inputs

   uo_out[0] = PWM_OUT   - PWM output
   uo_out[1] = EXT_OUT0  - External output 0
   uo_out[2] = EXT_OUT1  - External output 1
   uo_out[3] = TIMER_OUT - Timer output
   uo_out[4] = CS_UART   - SPI CS for UART bridge
   uo_out[5] = CS_ETH    - SPI CS for Ethernet
   uo_out[6] = CS_DAC    - SPI CS for DAC
   uo_out[7] = CS_ADC    - SPI CS for ADC

   uio[0] = SPI_SCLK     - Shared SPI clock (out)
   uio[1] = SPI_MOSI     - Shared SPI MOSI (out)
   uio[2] = SPI_MISO     - Shared SPI MISO (in)
   uio[3] = CS_FLASH     - SPI CS for Flash (out)
   uio[4] = CS_GPIO      - SPI CS for GPIO expander (out)
   uio[5] = CS_RAM       - SPI CS for RAM (out)
   uio[6:7] = spare      - Spare (out)
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

  // SPI signals extracted from bidirectional IO (new interconnect hub mapping)
  wire spi_cs_ram_n = uio_out[5];  // SPI CS for RAM
  wire spi_sclk     = uio_out[0];  // SPI Clock
  wire spi_mosi     = uio_out[1];  // SPI MOSI
  wire spi_miso;                    // SPI MISO (from SRAM to CPU)

  // Control signals from dedicated outputs
  wire pwm_out   = uo_out[0];   // PWM output
  wire ext_out0  = uo_out[1];   // External output 0
  wire ext_out1  = uo_out[2];   // External output 1
  wire timer_out = uo_out[3];   // Timer output

  // Control signals from dedicated inputs
  // ui_in[0] = boot_sel (directly settable from test)
  // ui_in[1] = irq_in
  // ui_in[3:2] = ext_in[1:0]

  // Connect SPI MISO to uio_in[2]
  always @(*) begin
    uio_in[2] = spi_miso;
    uio_in[7:3] = 5'b0;
    uio_in[1:0] = 2'b0;
  end

  // CPU Project (TinyTapeout module with interconnect hub)
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
      .spi_cs_n (spi_cs_ram_n),
      .spi_sclk (spi_sclk),
      .spi_mosi (spi_mosi),
      .spi_miso (spi_miso)
  );

endmodule
