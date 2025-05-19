`timescale 1ns / 100ps
module spi #(
    parameter DATA_WIDTH = 64  // Configurable data width
) (
    input    clk,
    input    resetn,
    input    [DATA_WIDTH-1:0] data_in,
    output   spi_mosi,
    output   spi_clk,
    output   spi_enable
);

  // State registers
  reg [$clog2(DATA_WIDTH)-1:0] bit_counter;  // Sized based on DATA_WIDTH
  reg [                   3:0] delay_counter;
  reg [        DATA_WIDTH-1:0] shift_reg;
  reg                          sending;

  // Generate SCLK - toggle every clock cycle during sending
  reg                          spi_clk_reg;
  assign spi_clk = sending ? spi_clk_reg : 1'b0;

  // Connect MOSI to MSB of shift register during sending
  assign spi_mosi = sending ? shift_reg[DATA_WIDTH-1] : 1'b0;

  // Chip select is active low during sending
  assign spi_enable = sending;

  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      bit_counter <= 0;
      delay_counter <= 0;
      shift_reg <= data_in;
      sending <= 1'b1;
      spi_clk_reg <= 1'b0;
    end else begin
      spi_clk_reg <= ~spi_clk_reg;  // Toggle SCLK

      if (sending) begin
        if (spi_clk_reg) begin  // Shift on falling edge of SCLK
          shift_reg   <= {shift_reg[DATA_WIDTH-2:0], 1'b0};
          bit_counter <= bit_counter + 1;

          if (bit_counter == DATA_WIDTH - 1) begin  // Finished all bits
            sending <= 1'b0;  // Enter delay state
            delay_counter <= 0;
          end
        end
      end else begin  // In delay state
        if (delay_counter == 4'b0111) begin  // After 8 cycles
          sending <= 1'b1;  // Start next transmission
          shift_reg <= data_in;  // Load new data
          bit_counter <= 0;
        end else begin
          delay_counter <= delay_counter + 1;
        end
      end
    end
  end

endmodule
