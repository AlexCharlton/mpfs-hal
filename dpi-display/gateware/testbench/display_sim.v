`include "display_tb.v"

`timescale 1ns / 100ps

module display_sim;

  logic clk;
  logic reset;
  logic buffer0_ready;
  logic buffer1_ready;

  logic buffer0_locked;
  logic buffer1_locked;

  logic [63:0] spi_data;

  logic pclk;
  logic vsync;
  logic hsync;
  logic data_enable;
  logic [7:0] r;
  logic [7:0] g;
  logic [7:0] b;

  display_tb #(
      .RUN_TIME(300000),
      .MEMORY_SIZE(6)
  ) display_tb (
      .clk(clk),
      .reset(reset),
      .buffer0_ready(buffer0_ready),
      .buffer1_ready(buffer1_ready),
      .buffer0_locked(buffer0_locked),
      .buffer1_locked(buffer1_locked),
      .spi_data(spi_data),
      .pclk(pclk),
      .vsync(vsync),
      .hsync(hsync),
      .data_enable(data_enable),
      .r(r),
      .g(g),
      .b(b)
  );

  // Clock generation
  initial begin
    clk = 0;
    forever #1 clk = ~clk;
  end

  initial begin
    reset = 0;
    buffer0_ready = 0;
    buffer1_ready = 0;

    // Reset sequence
    #10 reset = 1;
    #10 buffer0_ready = 1;
  end
endmodule
