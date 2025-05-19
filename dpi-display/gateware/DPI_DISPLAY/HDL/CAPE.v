`timescale 1ns / 100ps

// CAPE
module CAPE (
    // Inputs
    input logic ACLK,
    input logic ARESETN,
    input logic [27:0] GPIO_2_M2F,
    output logic [27:0] GPIO_2_F2M,

    // Outputs
    output logic PCLK,
    output logic VSYNC,
    output logic HSYNC,
    output logic DATA_ENABLE,
    output logic R0,
    output logic R1,
    output logic R2,
    output logic R3,
    output logic R4,
    output logic R5,
    output logic R6,
    output logic R7,
    output logic G0,
    output logic G1,
    output logic G2,
    output logic G3,
    output logic G4,
    output logic G5,
    output logic G6,
    output logic G7,
    output logic B0,
    output logic B1,
    output logic B2,
    output logic B3,
    output logic B4,
    output logic B5,
    output logic B6,
    output logic B7,

    output logic BLINK,
    output logic SPI_MOSI,
    output logic SPI_CLK,
    output logic SPI_ENABLE,

    // FIC_0 AXI4 Read Address Channel
    output logic [37:0] FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARADDR,
    output logic [ 3:0] FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARID,
    output logic [ 7:0] FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARLEN,
    output logic [ 1:0] FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARBURST,
    output logic [ 2:0] FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARSIZE,
    output logic        FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARLOCK,
    output logic [ 3:0] FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARCACHE,
    output logic [ 2:0] FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARPROT,
    output logic [ 3:0] FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARQOS,
    output logic        FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARVALID,
    input  logic        FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARREADY,

    // FIC_0 AXI4 Read Data Channel
    output logic        FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RREADY,
    input  logic [63:0] FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RDATA,
    input  logic [ 3:0] FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RID,
    input  logic [ 1:0] FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RRESP,
    input  logic        FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RLAST,
    input  logic        FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RVALID,

    // FIC_1 AXI4 Read Address Channel
    output logic [37:0] FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARADDR,
    output logic [ 3:0] FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARID,
    output logic [ 7:0] FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARLEN,
    output logic [ 1:0] FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARBURST,
    output logic [ 2:0] FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARSIZE,
    output logic        FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARLOCK,
    output logic [ 3:0] FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARCACHE,
    output logic [ 2:0] FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARPROT,
    output logic [ 3:0] FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARQOS,
    output logic        FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARVALID,
    input  logic        FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARREADY,

    // FIC_1 AXI4 Read Data Channel
    output logic        FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RREADY,
    input  logic [63:0] FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RDATA,
    input  logic [ 3:0] FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RID,
    input  logic [ 1:0] FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RRESP,
    input  logic        FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RLAST,
    input  logic        FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RVALID
);

  logic [63:0] spi_data;

  logic [7:0] r, g, b;
  assign {R7, R6, R5, R4, R3, R2, R1, R0} = r;
  assign {G7, G6, G5, G4, G3, G2, G1, G0} = g;
  assign {B7, B6, B5, B4, B3, B2, B1, B0} = b;

  assign GPIO_2_F2M[25:0] = 26'b0;

  display display_0 (
      .clk(ACLK),
      .reset(ARESETN),
      .buffer0_ready(GPIO_2_M2F[26]),
      .buffer1_ready(GPIO_2_M2F[27]),
      .buffer0_locked(GPIO_2_F2M[26]),
      .buffer1_locked(GPIO_2_F2M[27]),
      .spi_data(spi_data),

      .pclk(PCLK),
      .vsync(VSYNC),
      .hsync(HSYNC),
      .data_enable(DATA_ENABLE),
      .r(r),
      .g(g),
      .b(b),

      // Connect FIC_0
      .fic0_araddr(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARADDR),
      .fic0_arid(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARID),
      .fic0_arlen(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARLEN),
      .fic0_arburst(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARBURST),
      .fic0_arsize(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARSIZE),
      .fic0_arlock(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARLOCK),
      .fic0_arcache(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARCACHE),
      .fic0_arprot(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARPROT),
      .fic0_arqos(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARQOS),
      .fic0_arvalid(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARVALID),
      .fic0_arready(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARREADY),
      .fic0_rready(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RREADY),
      .fic0_rdata(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RDATA),
      .fic0_rid(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RID),
      .fic0_rresp(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RRESP),
      .fic0_rlast(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RLAST),
      .fic0_rvalid(FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RVALID),

      // Connect FIC_1
      .fic1_araddr(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARADDR),
      .fic1_arid(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARID),
      .fic1_arlen(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARLEN),
      .fic1_arburst(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARBURST),
      .fic1_arsize(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARSIZE),
      .fic1_arlock(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARLOCK),
      .fic1_arcache(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARCACHE),
      .fic1_arprot(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARPROT),
      .fic1_arqos(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARQOS),
      .fic1_arvalid(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARVALID),
      .fic1_arready(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARREADY),
      .fic1_rready(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RREADY),
      .fic1_rdata(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RDATA),
      .fic1_rid(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RID),
      .fic1_rresp(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RRESP),
      .fic1_rlast(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RLAST),
      .fic1_rvalid(FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RVALID)
  );

  //   spi #(
  //       .DATA_WIDTH(64)
  //   ) spi_0 (
  //       .clk(ACLK),
  //       .resetn(ARESETN),
  //       .data_in(spi_data),
  //       .spi_mosi(SPI_MOSI),
  //       .spi_clk(SPI_CLK),
  //       .spi_enable(SPI_ENABLE)
  //   );

endmodule
