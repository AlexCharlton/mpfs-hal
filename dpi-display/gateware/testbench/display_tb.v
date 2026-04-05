`include "axi4_read_from_file.v"
`include "helpers.v"

`define TEST 

`timescale 1ns / 100ps

module display_tb #(
    parameter integer MEMORY_SIZE = 144000,
    parameter integer RUN_TIME = 12000000,
    parameter integer H_PIXELS = 800,
    parameter integer V_PIXELS = 480
) (
    input  logic clk,
    input  logic reset,
    input  logic buffer0_ready,
    input  logic buffer1_ready,
    output logic buffer0_locked,
    output logic buffer1_locked,

    output logic [63:0] spi_data,

    output logic       pclk,
    output logic       vsync,
    output logic       hsync,
    output logic       data_enable,
    output logic [7:0] r,
    output logic [7:0] g,
    output logic [7:0] b
);

  // FIC_0 AXI4 Read Address Channel
  logic [37:0] fic0_araddr;
  logic [ 3:0] fic0_arid;
  logic [ 7:0] fic0_arlen;
  logic [ 1:0] fic0_arburst;
  logic [ 2:0] fic0_arsize;
  logic        fic0_arlock;
  logic [ 3:0] fic0_arcache;
  logic [ 2:0] fic0_arprot;
  logic [ 3:0] fic0_arqos;
  logic        fic0_arvalid;
  logic        fic0_arready;

  // FIC_0 AXI4 Read Data Channel
  logic        fic0_rready;
  logic [63:0] fic0_rdata;
  logic [ 3:0] fic0_rid;
  logic [ 1:0] fic0_rresp;
  logic        fic0_rlast;
  logic        fic0_rvalid;

  // FIC_1 AXI4 Read Address Channel
  logic [37:0] fic1_araddr;
  logic [ 3:0] fic1_arid;
  logic [ 7:0] fic1_arlen;
  logic [ 1:0] fic1_arburst;
  logic [ 2:0] fic1_arsize;
  logic        fic1_arlock;
  logic [ 3:0] fic1_arcache;
  logic [ 2:0] fic1_arprot;
  logic [ 3:0] fic1_arqos;
  logic        fic1_arvalid;
  logic        fic1_arready;

  // FIC_1 AXI4 Read Data Channel
  logic        fic1_rready;
  logic [63:0] fic1_rdata;
  logic [ 3:0] fic1_rid;
  logic [ 1:0] fic1_rresp;
  logic        fic1_rlast;
  logic        fic1_rvalid;

  display #(
      .MEMORY_SIZE(MEMORY_SIZE)
  ) display_0 (
      .clk(clk),
      .reset(reset),
      .buffer0_ready(buffer0_ready),
      .buffer1_ready(buffer1_ready),
      .buffer0_locked(buffer0_locked),
      .buffer1_locked(buffer1_locked),

      // Debug
      .spi_data(spi_data),

      // Display interface
      .pclk(pclk),
      .vsync(vsync),
      .hsync(hsync),
      .data_enable(data_enable),
      .r(r),
      .g(g),
      .b(b),

      // Connect FIC_0
      .fic0_araddr(fic0_araddr),
      .fic0_arid(fic0_arid),
      .fic0_arlen(fic0_arlen),
      .fic0_arburst(fic0_arburst),
      .fic0_arsize(fic0_arsize),
      .fic0_arlock(fic0_arlock),
      .fic0_arcache(fic0_arcache),
      .fic0_arprot(fic0_arprot),
      .fic0_arqos(fic0_arqos),
      .fic0_arvalid(fic0_arvalid),
      .fic0_arready(fic0_arready),
      .fic0_rready(fic0_rready),
      .fic0_rdata(fic0_rdata),
      .fic0_rid(fic0_rid),
      .fic0_rresp(fic0_rresp),
      .fic0_rlast(fic0_rlast),
      .fic0_rvalid(fic0_rvalid),

      // Connect FIC_1
      .fic1_araddr(fic1_araddr),
      .fic1_arid(fic1_arid),
      .fic1_arlen(fic1_arlen),
      .fic1_arburst(fic1_arburst),
      .fic1_arsize(fic1_arsize),
      .fic1_arlock(fic1_arlock),
      .fic1_arcache(fic1_arcache),
      .fic1_arprot(fic1_arprot),
      .fic1_arqos(fic1_arqos),
      .fic1_arvalid(fic1_arvalid),
      .fic1_arready(fic1_arready),
      .fic1_rready(fic1_rready),
      .fic1_rdata(fic1_rdata),
      .fic1_rid(fic1_rid),
      .fic1_rresp(fic1_rresp),
      .fic1_rlast(fic1_rlast),
      .fic1_rvalid(fic1_rvalid)
  );

  // Instantiate memory simulators for FIC_0 and FIC_1
  axi4_read_from_file #(
      .MEMORY_SIZE(MEMORY_SIZE)
  ) fic0_mem (
      .clk  (clk),
      .reset(reset),

      // Address channel
      .araddr(fic0_araddr),
      .arid(fic0_arid),
      .arlen(fic0_arlen),
      .arburst(fic0_arburst),
      .arsize(fic0_arsize),
      .arlock(fic0_arlock),
      .arcache(fic0_arcache),
      .arprot(fic0_arprot),
      .arqos(fic0_arqos),
      .arvalid(fic0_arvalid),
      .arready(fic0_arready),

      // Data channel
      .rready(fic0_rready),
      .rdata(fic0_rdata),
      .rid(fic0_rid),
      .rresp(fic0_rresp),
      .rlast(fic0_rlast),
      .rvalid(fic0_rvalid)
  );

  axi4_read_from_file #(
      .MEMORY_SIZE(MEMORY_SIZE)
  ) fic1_mem (
      .clk  (clk),
      .reset(reset),

      // Address channel
      .araddr(fic1_araddr),
      .arid(fic1_arid),
      .arlen(fic1_arlen),
      .arburst(fic1_arburst),
      .arsize(fic1_arsize),
      .arlock(fic1_arlock),
      .arcache(fic1_arcache),
      .arprot(fic1_arprot),
      .arqos(fic1_arqos),
      .arvalid(fic1_arvalid),
      .arready(fic1_arready),

      // Data channel
      .rready(fic1_rready),
      .rdata(fic1_rdata),
      .rid(fic1_rid),
      .rresp(fic1_rresp),
      .rlast(fic1_rlast),
      .rvalid(fic1_rvalid)
  );


  logic spi_mosi;
  logic spi_enable;
  logic spi_clk;
  spi #(
      .DATA_WIDTH(64)
  ) spi_0 (
      .clk(clk),
      .resetn(reset),
      .data_in(spi_data),
      .spi_mosi(spi_mosi),
      .spi_clk(spi_clk),
      .spi_enable(spi_enable)
  );

  initial begin
    $dumpfile("display_tb.vcd");
    $dumpvars(0, display_tb);
    $display("Simulation started");
  end

  // Monitor the RGB values to check that they are correct
  logic [63:0] memory64[  0:(MEMORY_SIZE*2)-1];
  logic [ 7:0] memory8 [0:(MEMORY_SIZE*8*3)-1];
  logic [$clog2(MEMORY_SIZE*8*2)-1:0] read_pos, read_offset, pixel_offset, test_pattern_offset;
  logic [4:0] screen_num;

  initial begin
    for (int i = 0; i < MEMORY_SIZE * 2; i = i + 6) begin
      if (i < 72000) begin  // Buffer 0: half of the screen
        // RRRR GGGG BBBB WWWW
        memory64[i+0] = 64'hFF0000FF0000FF00;
        memory64[i+1] = 64'h00FF000000FF0000;
        memory64[i+2] = 64'hFF0000FF0000FF00;
        memory64[i+3] = 64'h0000FF0000FF0000;
        memory64[i+4] = 64'hFF0000FFFFFFFFFF;
        memory64[i+5] = 64'hFFFFFFFFFFFFFFFF;
      end else if (i < 144000) begin  // Buffer 0: the other half of the screen
        // Just Red
        memory64[i+0] = 64'hFF0000FF0000FF00;
        memory64[i+1] = 64'h00FF0000FF0000FF;
        memory64[i+2] = 64'h0000FF0000FF0000;
        memory64[i+3] = 64'hFF0000FF0000FF00;
        memory64[i+4] = 64'h00FF0000FF0000FF;
        memory64[i+5] = 64'h0000FF0000FF0000;
      end else if (i < 216000) begin  // Buffer 1: half of the screen
        // Just Green
        memory64[i+0] = 64'h00FF0000FF0000FF;
        memory64[i+1] = 64'h0000FF0000FF0000;
        memory64[i+2] = 64'hFF0000FF0000FF00;
        memory64[i+3] = 64'h00FF0000FF0000FF;
        memory64[i+4] = 64'h0000FF0000FF0000;
        memory64[i+5] = 64'hFF0000FF0000FF00;
      end else if (i < 288000) begin  // Buffer 1: the other half of the screen
        // Just Blue
        memory64[i+0] = 64'h0000FF0000FF0000;
        memory64[i+1] = 64'hFF0000FF0000FF00;
        memory64[i+2] = 64'h00FF0000FF0000FF;
        memory64[i+3] = 64'h0000FF0000FF0000;
        memory64[i+4] = 64'hFF0000FF0000FF00;
        memory64[i+5] = 64'h00FF0000FF0000FF;
      end
    end

    // Copy the memory64 to memory8
    for (int i = 0; i < MEMORY_SIZE * 2; i++) begin
      memory8[(i*8)+0] = memory64[i][63:56];
      memory8[(i*8)+1] = memory64[i][55:48];
      memory8[(i*8)+2] = memory64[i][47:40];
      memory8[(i*8)+3] = memory64[i][39:32];
      memory8[(i*8)+4] = memory64[i][31:24];
      memory8[(i*8)+5] = memory64[i][23:16];
      memory8[(i*8)+6] = memory64[i][15:8];
      memory8[(i*8)+7] = memory64[i][7:0];
      //   $display("memory8[%d] = %h", i * 8 + 0, memory8[i*8+0]);
      //   $display("memory8[%d] = %h", i * 8 + 1, memory8[i*8+1]);
      //   $display("memory8[%d] = %h", i * 8 + 2, memory8[i*8+2]);
      //   $display("memory8[%d] = %h", i * 8 + 3, memory8[i*8+3]);
      //   $display("memory8[%d] = %h", i * 8 + 4, memory8[i*8+4]);
      //   $display("memory8[%d] = %h", i * 8 + 5, memory8[i*8+5]);
      //   $display("memory8[%d] = %h", i * 8 + 6, memory8[i*8+6]);
      //   $display("memory8[%d] = %h", i * 8 + 7, memory8[i*8+7]);
    end

    // Test pattern
    test_pattern_offset <= H_PIXELS * V_PIXELS * 3 * 2;  // 3 pixels, 3rd buffer
    for (int i = 0; i < H_PIXELS * V_PIXELS; i++) begin
      if (i / H_PIXELS < V_PIXELS / 2) memory8[(i*3)+test_pattern_offset] <= 8'h00;  // R
      else memory8[(i*3)+test_pattern_offset] <= 8'hff;  // R

      if (i % H_PIXELS < H_PIXELS / 4) begin
        memory8[(i*3)+1+test_pattern_offset] <= 8'h00;  // G
        memory8[(i*3)+2+test_pattern_offset] <= 8'h00;  // B
      end else if (i % H_PIXELS < H_PIXELS / 2) begin
        memory8[(i*3)+1+test_pattern_offset] <= 8'hff;  // G
        memory8[(i*3)+2+test_pattern_offset] <= 8'h00;  // B
      end else if (i % H_PIXELS < H_PIXELS * 3 / 4) begin
        memory8[(i*3)+1+test_pattern_offset] <= 8'h00;  // G
        memory8[(i*3)+2+test_pattern_offset] <= 8'hff;  // B
      end else begin
        memory8[(i*3)+1+test_pattern_offset] <= 8'hff;  // G
        memory8[(i*3)+2+test_pattern_offset] <= 8'hff;  // B
      end
    end

    read_pos <= 0;
    read_offset <= 0;
    screen_num <= 0;
  end

  always @(posedge vsync) begin
    case (screen_num)
      0: read_offset <= MEMORY_SIZE * 8 * 2;  // Test pattern
      1: read_offset <= 0;  // Buffer 0
      2: read_offset <= 0;  // Buffer 1
      3: read_offset <= MEMORY_SIZE * 8 * 1;  // Buffer 1
      4: read_offset <= MEMORY_SIZE * 8 * 1;  // Buffer 1
      default: read_offset <= 0;  // Not used
    endcase
    screen_num <= screen_num + 1;
  end

  always @(posedge pclk) begin
    if (data_enable) begin
      assert ({r, g, b} === {memory8[read_pos + read_offset], memory8[read_pos+1 + read_offset], memory8[read_pos+2 + read_offset]})
      else begin
        $display("Error: {r, g, b} = %h, expected %h (read_pos = %d, offset = %d)", {r, g, b}, {
                 memory8[read_pos+read_offset], memory8[read_pos+1+read_offset],
                 memory8[read_pos+2+read_offset]}, read_pos, read_offset);
        $stop;
      end
      read_pos <= (read_pos + 3) % (H_PIXELS * V_PIXELS * 3);
    end
  end

  logic [$clog2(RUN_TIME)-1:0] cycle;
  always @(posedge clk) begin
    if (!reset) begin
      cycle <= 0;
    end else begin
      cycle <= cycle + 1;
      if (cycle > RUN_TIME) begin
        $display("Simulation finished");
        $finish;
      end
    end
  end
endmodule
