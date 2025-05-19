`timescale 1ns / 100ps

module axi4_read_from_file #(
    parameter MEMORY_SIZE = 144000,  // 480x800 pixels / 2 2/3 pixels per word
    parameter ADDRESS_BASE = 38'h147fc00000
) (
    input logic clk,
    input logic reset,

    input  logic [37:0] araddr,
    input  logic [ 3:0] arid,
    input  logic [ 7:0] arlen,
    input  logic [ 1:0] arburst,
    input  logic [ 2:0] arsize,
    input  logic        arlock,
    input  logic [ 3:0] arcache,
    input  logic [ 2:0] arprot,
    input  logic [ 3:0] arqos,
    input  logic        arvalid,
    output logic        arready,

    // AXI4 Read Data Channel
    input  logic        rready,
    output logic [63:0] rdata,
    output logic [ 3:0] rid,
    output logic [ 1:0] rresp,
    output logic        rlast,
    output logic        rvalid
);
  logic [63:0] memory[0:MEMORY_SIZE*2-1];

  logic [4:0] delay_counter;  // 5 bits for counting up to 30
  logic [4:0] random_delay;  // Store the random delay value

  // State machine
  typedef enum {
    IDLE,
    WAIT_DELAY,
    READ_DATA
  } state_t;
  state_t state;

  // Address calculation
  logic [37:0] relative_addr;
  logic [31:0] word_addr;

  assign relative_addr = araddr - ADDRESS_BASE;
  assign word_addr = relative_addr[31:3];  // Divide by 8 for 64-bit words

  initial begin
    for (int i = 0; i < MEMORY_SIZE * 2; i = i + 6) begin
      if (i < 72000) begin
        // RRRR GGGG BBBB WWWW
        memory[i+0] = 64'hFF0000FF0000FF00;
        memory[i+1] = 64'h00FF000000FF0000;
        memory[i+2] = 64'hFF0000FF0000FF00;
        memory[i+3] = 64'h0000FF0000FF0000;
        memory[i+4] = 64'hFF0000FFFFFFFFFF;
        memory[i+5] = 64'hFFFFFFFFFFFFFFFF;
      end else if (i < 144000) begin
        // Just Red
        memory[i+0] = 64'hFF0000FF0000FF00;
        memory[i+1] = 64'h00FF0000FF0000FF;
        memory[i+2] = 64'h0000FF0000FF0000;
        memory[i+3] = 64'hFF0000FF0000FF00;
        memory[i+4] = 64'h00FF0000FF0000FF;
        memory[i+5] = 64'h0000FF0000FF0000;
      end else if (i < 216000) begin
        // Just Green
        memory[i+0] = 64'h00FF0000FF0000FF;
        memory[i+1] = 64'h0000FF0000FF0000;
        memory[i+2] = 64'hFF0000FF0000FF00;
        memory[i+3] = 64'h00FF0000FF0000FF;
        memory[i+4] = 64'h0000FF0000FF0000;
        memory[i+5] = 64'hFF0000FF0000FF00;
      end else begin
        // Just Blue
        memory[i+0] = 64'h0000FF0000FF0000;
        memory[i+1] = 64'hFF0000FF0000FF00;
        memory[i+2] = 64'h00FF0000FF0000FF;
        memory[i+3] = 64'h0000FF0000FF0000;
        memory[i+4] = 64'hFF0000FF0000FF00;
        memory[i+5] = 64'h00FF0000FF0000FF;
      end
    end

    state   = IDLE;
    arready = 0;
    rvalid  = 0;
    rresp   = 0;
    rlast   = 0;

    // $monitor("araddr: %h, word_addr: %h", araddr, word_addr);
  end

  always_ff @(posedge clk or negedge reset) begin
    if (!reset) begin
      state <= IDLE;
      arready <= 0;
      rvalid <= 0;
      rresp <= 0;
      rlast <= 0;
      delay_counter <= 0;
    end else begin
      case (state)
        IDLE: begin
          if (arvalid) begin
            // Accept address
            arready <= 1;
            rid <= arid;

            // Check if address is in range
            if (word_addr < MEMORY_SIZE * 2) begin
              rdata <= memory[word_addr];
              rresp <= 2'b00;  // OKAY
            end else begin
              rdata <= 64'h0;
              rresp <= 2'b10;  // SLVERR
            end

            // Generate random delay between 10 and 30 cycles
            random_delay <= ($urandom % 21) + 10;
            delay_counter <= 0;
            state <= WAIT_DELAY;
          end
        end

        WAIT_DELAY: begin
          arready <= 0;
          if (delay_counter >= random_delay) begin
            state <= READ_DATA;
          end else begin
            delay_counter <= delay_counter + 1;
          end
        end

        READ_DATA: begin
          if (!rvalid || (rvalid && rready)) begin
            rvalid <= 1;
            rlast  <= 1;  // Single transfer
            if (rvalid && rready) begin
              // Transfer complete
              rvalid <= 0;
              rlast  <= 0;
              state  <= IDLE;
            end
          end
        end
      endcase
    end
  end
endmodule

