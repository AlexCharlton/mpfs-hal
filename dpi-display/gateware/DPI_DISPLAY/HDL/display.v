`timescale 1ns / 100ps

// ----------------------------------------------------------------------------
// MARK: AXI4 read
// ----------------------------------------------------------------------------

module axi4_read (
    input logic clk,
    input logic reset,
    input logic [37:0] address,
    input logic ready,

    output logic [63:0] data,
    output logic valid,

    // AXI4 address read channel
    output logic [37:0] araddr,
    output logic [ 3:0] arid,
    output logic [ 7:0] arlen,
    output logic [ 1:0] arburst,
    output logic [ 2:0] arsize,
    output logic        arlock,
    output logic [ 3:0] arcache,
    output logic [ 2:0] arprot,
    output logic [ 3:0] arqos,
    output logic        arvalid,
    input  logic        arready,

    // AXI4 read data channel
    output logic        rready,
    input  logic [63:0] rdata,
    input  logic [ 3:0] rid,
    input  logic [ 1:0] rresp,
    input  logic        rlast,
    input  logic        rvalid
);
  // State machine
  typedef enum {
    IDLE,
    ADDR,
    READ
  } state_t;
  state_t state;

  always_ff @(posedge clk or negedge reset) begin
    if (!reset) begin
      state <= IDLE;
      data <= 64'h0;
      valid <= 0;
      arvalid <= 0;
      rready <= 0;
    end else begin
      case (state)
        IDLE: begin
          arvalid <= 0;
          rready  <= 0;
          valid   <= 0;

          if (ready) begin
            state <= ADDR;
            // Setup read
            araddr <= address;
            arvalid <= 1;
            arid <= 4'b0;
            arlen <= 8'd0;  // Single transfer
            arburst <= 2'b01;
            arsize <= 3'b011;
            arlock <= 0;
            arcache <= 4'b0011;
            arprot <= 3'b0;
            arqos <= 4'b1111;
          end
        end

        ADDR: begin
          if (arready) begin
            arvalid <= 0;
            rready  <= 1;
            state   <= READ;
          end
        end

        READ: begin
          if (rvalid) begin
            data   <= rdata;
            valid  <= 1;
            rready <= 0;
            state  <= IDLE;
          end
        end
      endcase
    end
  end
endmodule

// ----------------------------------------------------------------------------
// MARK: Display
// ----------------------------------------------------------------------------

module display #(
    // Display resolution
    parameter H_PIXELS = 800,
    parameter V_PIXELS = 480,

    // Horizontal timing (in pixel clocks)
    parameter H_FRONT_PORCH_WIDTH = 40,
    parameter H_SYNC_WIDTH = 48,
    parameter H_BACK_PORCH_WIDTH = 88,

    // Vertical timing (in lines)
    parameter V_FRONT_PORCH_WIDTH = 13,
    parameter V_SYNC_WIDTH = 3,
    parameter V_BACK_PORCH_WIDTH = 32,

    // Clock divider for pixel clock
    parameter CLOCK_DIVIDER   = 5,
    parameter CLOCK_LOW_TIME  = 3,
    parameter CLOCK_HIGH_TIME = CLOCK_DIVIDER - CLOCK_LOW_TIME,

    // Pixel data
    parameter AXI_READ_ADDR = 38'h147fc00000,
    parameter MEMORY_SIZE = 144000,  // In 64-bit words, for a single buffer; MUST BE A MULTIPLE OF 6
    parameter BUFFER_SIZE = 6,  // 64-bit words; circular buffer (must be a multiple of 3; The buffer is double this size, since we create a buffer per AXI channel. Large values cause synthesis to take forever)

    // Derived parameters (don't override these)
    // parameter UPPER_ADDR_LIMIT = AXI_READ_ADDR + (MEMORY_SIZE * 8 * 2),
    parameter UPPER_ADDR_LIMIT = AXI_READ_ADDR + (MEMORY_SIZE * 8),
    parameter NUM_PIXELS = H_PIXELS * V_PIXELS,
    parameter LINE_CLK_COUNT = H_PIXELS + H_FRONT_PORCH_WIDTH + H_SYNC_WIDTH + H_BACK_PORCH_WIDTH,
    parameter NUM_LINES = V_PIXELS + V_FRONT_PORCH_WIDTH + V_SYNC_WIDTH + V_BACK_PORCH_WIDTH,
    parameter FRAME_CLK_COUNT = NUM_LINES * LINE_CLK_COUNT
) (
    input logic clk,
    input logic reset,

    input logic buffer0_ready,
    input logic buffer1_ready,

    output logic buffer0_locked,
    output logic buffer1_locked,

    // Debug
    output logic [63:0] spi_data,

    // Display interface
    output logic pclk,
    output logic vsync,
    output logic hsync,
    output logic data_enable,
    output logic [7:0] r,
    output logic [7:0] g,
    output logic [7:0] b,

    // FIC_0 AXI4 Read Address Channel
    output logic [37:0] fic0_araddr,
    output logic [ 3:0] fic0_arid,
    output logic [ 7:0] fic0_arlen,
    output logic [ 1:0] fic0_arburst,
    output logic [ 2:0] fic0_arsize,
    output logic        fic0_arlock,
    output logic [ 3:0] fic0_arcache,
    output logic [ 2:0] fic0_arprot,
    output logic [ 3:0] fic0_arqos,
    output logic        fic0_arvalid,
    input  logic        fic0_arready,

    // FIC_0 AXI4 Read Data Channel
    output logic        fic0_rready,
    input  logic [63:0] fic0_rdata,
    input  logic [ 3:0] fic0_rid,
    input  logic [ 1:0] fic0_rresp,
    input  logic        fic0_rlast,
    input  logic        fic0_rvalid,

    // FIC_1 AXI4 Read Address Channel
    output logic [37:0] fic1_araddr,
    output logic [ 3:0] fic1_arid,
    output logic [ 7:0] fic1_arlen,
    output logic [ 1:0] fic1_arburst,
    output logic [ 2:0] fic1_arsize,
    output logic        fic1_arlock,
    output logic [ 3:0] fic1_arcache,
    output logic [ 2:0] fic1_arprot,
    output logic [ 3:0] fic1_arqos,
    output logic        fic1_arvalid,
    input  logic        fic1_arready,

    // FIC_1 AXI4 Read Data Channel
    output logic        fic1_rready,
    input  logic [63:0] fic1_rdata,
    input  logic [ 3:0] fic1_rid,
    input  logic [ 1:0] fic1_rresp,
    input  logic        fic1_rlast,
    input  logic        fic1_rvalid
);

`ifdef TEST
  initial begin
    // Both the memory and buffer must be multiples of 48 bytes (memory is 64-bit words, buffer is 8-bit)
    `ASSERT_EQ(MEMORY_SIZE % 6, 0)
    `ASSERT_EQ(BUFFER_SIZE % 3, 0)
  end
`endif

  // AXI read results
  logic fic0_valid, fic1_valid;
  logic [63:0] fic0_data, fic1_data;

  // Circular buffer storage
  logic [63:0] buffer0[BUFFER_SIZE-1:0];
  logic [63:0] buffer1[BUFFER_SIZE-1:0];
  // Position for writes
  logic [$clog2(BUFFER_SIZE)-1:0]
      fic0_write_pos,
      fic1_write_pos,
      fic0_next_write_pos,
      fic1_next_write_pos,
      last_seen_fic0_write_pos,
      last_seen_fic1_write_pos;
  // Track reads in terms of bytes
  logic [$clog2(BUFFER_SIZE*8*2)-1:0] read_pos, next_read_pos;
  // Number of words used in buffer by FIC0 and FIC1
  logic [$clog2(BUFFER_SIZE)-1:0] fic0_buffer_used, fic1_buffer_used;

  // Where are we reading from?
  logic [37:0] fic0_addr;
  logic [37:0] fic1_addr;
  // Address boundaries
  logic [37:0] address_limit, address_buffer_offset;
  // State of buffers, and are we ready to read?
  logic fic0_ready, fic1_ready;
  logic fic0_buffer_full, fic1_buffer_full;
  logic buffer_empty;


  // Are the buffers full?
  assign fic0_buffer_full = fic0_buffer_used + 1 > BUFFER_SIZE;
  assign fic1_buffer_full = fic1_buffer_used + 1 > BUFFER_SIZE;
  // Approximation of an empty buffer, which is good enough for our purposes
  assign buffer_empty = fic0_buffer_used == 0 || fic1_buffer_used == 0;

  axi4_read axi4_read_0 (
      .clk(clk),
      .reset(reset),
      .ready(fic0_ready),
      .address(fic0_addr),
      .data(fic0_data),
      .valid(fic0_valid),

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
      .rready(fic0_rready),
      .rdata(fic0_rdata),
      .rid(fic0_rid),
      .rresp(fic0_rresp),
      .rlast(fic0_rlast),
      .rvalid(fic0_rvalid)
  );

  axi4_read axi4_read_1 (
      .clk(clk),
      .reset(reset),
      .ready(fic1_ready),
      .address(fic1_addr),
      .data(fic1_data),
      .valid(fic1_valid),

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
      .rready(fic1_rready),
      .rdata(fic1_rdata),
      .rid(fic1_rid),
      .rresp(fic1_rresp),
      .rlast(fic1_rlast),
      .rvalid(fic1_rvalid)
  );

  // ----------------------------------------------------------------------------
  // MARK: Acquire lock
  // Decide which buffer to lock, and set FIC ready
  // ----------------------------------------------------------------------------
  logic buffer_counter, last_buffer_counter, last_buffer_used, have_lock;

  always_ff @(posedge clk or negedge reset) begin
    if (!reset) begin
      fic0_ready <= 0;
      fic1_ready <= 0;
      buffer0_locked <= 0;
      buffer1_locked <= 0;

      // Will wrap around on the first sync
      last_buffer_counter <= 1;
      last_buffer_used <= 1;
      have_lock <= 0;

      address_buffer_offset <= 0;
    end else if (current_v_state == VSYNC) begin
      fic0_ready <= 0;
      fic1_ready <= 0;

      if (buffer_counter != last_buffer_counter) begin
        last_buffer_counter <= buffer_counter;

        if (last_buffer_used == 1) begin
          // If the next buffer is ready, lock it
          // If not, we don't do anything and the previous buffer will stay locked
          if (buffer0_ready) begin
            buffer0_locked <= 1;
            buffer1_locked <= 0;
            last_buffer_used <= 0;
            have_lock <= 1;
            address_buffer_offset <= 0;
          end
        end else begin
          // If the next buffer is ready, lock it
          // If not, we don't do anything and the previous buffer will stay locked
          if (buffer1_ready) begin
            buffer1_locked <= 1;
            buffer0_locked <= 0;
            last_buffer_used <= 1;
            have_lock <= 1;
            address_buffer_offset <= MEMORY_SIZE * 8;
          end
        end
      end
    end else begin
      if (!fic0_buffer_full && have_lock) fic0_ready <= 1;
      else fic0_ready <= 0;

      if (!fic1_buffer_full && have_lock) fic1_ready <= 1;
      else fic1_ready <= 0;
    end
  end

  // ----------------------------------------------------------------------------
  // MARK: Fill buffer
  // Read from AXI memory and write to buffer
  // ----------------------------------------------------------------------------
  typedef enum {
    IDLE,
    FIC_READ
  } buffer_write_state_t;

  buffer_write_state_t fic0_buffer_write_state, fic1_buffer_write_state;

  always_ff @(posedge clk or negedge reset) begin
    if (!reset) begin
      fic0_write_pos <= 0;
      fic0_addr <= AXI_READ_ADDR;
      fic0_buffer_write_state <= IDLE;
      fic0_next_write_pos <= 0;

      fic1_write_pos <= 0;
      fic1_addr <= AXI_READ_ADDR + 8;
      fic1_buffer_write_state <= IDLE;
      fic1_next_write_pos <= 0;

    end else if (current_v_state == VSYNC) begin // Same as !reset, but libero doesn't like combining the two
      fic0_write_pos <= 0;
      fic0_addr <= AXI_READ_ADDR + address_buffer_offset;
      fic0_buffer_write_state <= IDLE;
      fic0_next_write_pos <= 0;

      fic1_write_pos <= 0;
      fic1_addr <= AXI_READ_ADDR + address_buffer_offset + 8;
      fic1_buffer_write_state <= IDLE;
      fic1_next_write_pos <= 0;
    end else begin
      // When data is arriving, advance the address
      if (!fic0_buffer_full && fic0_rvalid) begin
        fic0_addr <= fic0_addr >= (address_buffer_offset + UPPER_ADDR_LIMIT - 16) ? AXI_READ_ADDR + address_buffer_offset : fic0_addr + 16;
      end
      if (!fic1_buffer_full && fic1_rvalid) begin
        fic1_addr <= fic1_addr >= (address_buffer_offset + UPPER_ADDR_LIMIT - 16) ? AXI_READ_ADDR + address_buffer_offset + 8 : fic1_addr + 16;
      end

      // FIC0 write to buffer
      case (fic0_buffer_write_state)
        IDLE: begin
          if (fic0_ready && fic0_valid) begin
            fic0_buffer_write_state <= FIC_READ;
          end

          if (fic0_write_pos != fic0_next_write_pos) begin
            fic0_write_pos <= fic0_next_write_pos;
          end
        end
        FIC_READ: begin
          // Write FIC0 data bytes
          buffer0[fic0_write_pos] <= fic0_data;

          fic0_next_write_pos <= (fic0_write_pos + 1) % BUFFER_SIZE;
          fic0_buffer_write_state <= IDLE;
        end
      endcase

      case (fic1_buffer_write_state)
        IDLE: begin
          if (fic1_ready && fic1_valid) begin
            fic1_buffer_write_state <= FIC_READ;
          end

          if (fic1_write_pos != fic1_next_write_pos) begin
            fic1_write_pos <= fic1_next_write_pos;
          end
        end
        FIC_READ: begin
          // Write FIC1 data bytes
          buffer1[fic1_write_pos] <= fic1_data;

          fic1_next_write_pos <= (fic1_write_pos + 1) % BUFFER_SIZE;
          fic1_buffer_write_state <= IDLE;
        end
      endcase
    end
  end

  // ----------------------------------------------------------------------------
  // MARK: Display state machine
  // ----------------------------------------------------------------------------

  typedef enum {
    VSYNC,
    V_BACK_PORCH,
    V_ADR,
    V_FRONT_PORCH
  } v_state_t;

  typedef enum {
    HSYNC,
    H_BACK_PORCH,
    H_ADR,
    H_FRONT_PORCH
  } h_state_t;

  v_state_t current_v_state, next_v_state;
  h_state_t current_h_state, next_h_state;

  // Clock divider counter
  logic [$clog2(CLOCK_DIVIDER)-1:0] clock_counter;
  logic [$clog2(LINE_CLK_COUNT)-1:0] x_counter;
  logic [$clog2(NUM_LINES)-1:0] y_counter;
  logic [$clog2(NUM_PIXELS*3)-1:0] pixel_offset;

  always_comb begin
    if (!reset) begin
      current_v_state = VSYNC;
      current_h_state = HSYNC;
      next_v_state = VSYNC;
      next_h_state = HSYNC;
    end else begin

      if (current_v_state != next_v_state) begin
        current_v_state = next_v_state;
      end else begin
        case (current_v_state)
          VSYNC: begin
            if (y_counter == V_SYNC_WIDTH) begin
              next_v_state = V_BACK_PORCH;
            end
          end
          V_BACK_PORCH: begin
            if (y_counter == V_SYNC_WIDTH + V_BACK_PORCH_WIDTH) begin
              next_v_state = V_ADR;
            end
          end
          V_ADR: begin
            if (y_counter == V_SYNC_WIDTH + V_BACK_PORCH_WIDTH + V_PIXELS) begin
              next_v_state = V_FRONT_PORCH;
            end
          end
          V_FRONT_PORCH: begin
            if (y_counter == NUM_LINES) begin
              next_v_state = VSYNC;
            end
          end
        endcase
      end

      if (current_h_state != next_h_state) begin
        current_h_state = next_h_state;
      end else begin
        case (current_h_state)
          HSYNC: begin
            if (x_counter == H_SYNC_WIDTH - 1) begin
              next_h_state = H_BACK_PORCH;
            end
          end
          H_BACK_PORCH: begin
            if (x_counter == H_SYNC_WIDTH + H_BACK_PORCH_WIDTH - 1) begin
              next_h_state = H_ADR;
            end
          end
          H_ADR: begin
            if (x_counter == H_SYNC_WIDTH + H_BACK_PORCH_WIDTH + H_PIXELS - 1) begin
              next_h_state = H_FRONT_PORCH;
            end
          end
          H_FRONT_PORCH: begin
            if (x_counter == LINE_CLK_COUNT - 1) begin
              next_h_state = HSYNC;
            end
          end
        endcase
      end
    end
  end

  // ----------------------------------------------------------------------------
  // MARK: Pixel clock and data
  // ----------------------------------------------------------------------------

  logic [$clog2(BUFFER_SIZE*8*2)-1:0] read_pos_div48_mult3;

  `define UPDATE_BUFFER_USED_COUNT(buffer_used, write_pos, last_seen_write_pos) \
    if (write_pos != last_seen_write_pos) begin \
      buffer_used <= buffer_used + 1; \
      last_seen_write_pos <= write_pos; \
    end

  always_ff @(posedge clk or negedge reset) begin
    if (!reset) begin
      clock_counter <= 0;
      pclk <= 0;
      hsync <= 1;

      x_counter <= 0;
      y_counter <= 0;
      pixel_offset <= 0;
      buffer_counter <= 0;

      read_pos <= 0;
      read_pos_div48_mult3 <= 0;
      next_read_pos <= 0;
      fic0_buffer_used <= 0;
      fic1_buffer_used <= 0;
      last_seen_fic0_write_pos <= 0;
      last_seen_fic1_write_pos <= 0;
    end else begin
      if (clock_counter == CLOCK_DIVIDER - 1) begin
        // These three variables need to be updated every clock/branch
        clock_counter <= 0;

        // Reset clock
        pclk <= 0;

        if (current_v_state == VSYNC) begin
          read_pos <= 0;
          read_pos_div48_mult3 <= 0;
          next_read_pos <= 0;
          fic0_buffer_used <= 0;
          fic1_buffer_used <= 0;
          last_seen_fic0_write_pos <= 0;
          last_seen_fic1_write_pos <= 0;
        end else if (next_read_pos != read_pos) begin
          `UPDATE_BUFFER_USED_COUNT(fic0_buffer_used, fic0_write_pos, last_seen_fic0_write_pos);
          `UPDATE_BUFFER_USED_COUNT(fic1_buffer_used, fic1_write_pos, last_seen_fic1_write_pos);
          read_pos <= next_read_pos;
          read_pos_div48_mult3 <= (next_read_pos / 48) * 3;
        end

        // Update position counters
        if (y_counter == NUM_LINES) begin
          y_counter <= 0;
          x_counter <= 0;
          pixel_offset <= 0;
          buffer_counter <= buffer_counter + 1;
        end else begin
          if (x_counter == LINE_CLK_COUNT - 1) begin
            x_counter <= 0;
            y_counter <= y_counter + 1;
          end else begin
            x_counter <= x_counter + 1;
          end

          if (current_h_state == H_ADR && current_v_state == V_ADR) begin
            pixel_offset <= pixel_offset + 1;
          end
        end
      end else if (clock_counter == 0) begin
        // Update the display control signals
        data_enable <= (current_h_state == H_ADR) && (current_v_state == V_ADR);  // Active high
        vsync <= (current_v_state != VSYNC);  // Active low
        hsync <= (current_h_state != HSYNC);  // Active low

        // Update pixel data before the clock goes high
        if (current_h_state == H_ADR && current_v_state == V_ADR) begin

          // We're reading from memory
          if (have_lock) begin
            // If the buffer is empty, we don't advance the clock
            if (buffer_empty) begin
              // But we do need to update the buffer used count
              `UPDATE_BUFFER_USED_COUNT(fic0_buffer_used, fic0_write_pos, last_seen_fic0_write_pos);
              `UPDATE_BUFFER_USED_COUNT(fic1_buffer_used, fic1_write_pos, last_seen_fic1_write_pos);
            end else begin  // We have data
              clock_counter <= clock_counter + 1;

              case (read_pos % 48)
                0: begin
                  r <= buffer0[0+read_pos_div48_mult3][63:56];
                  g <= buffer0[0+read_pos_div48_mult3][55:48];
                  b <= buffer0[0+read_pos_div48_mult3][47:40];
                end
                3: begin
                  r <= buffer0[0+read_pos_div48_mult3][39:32];
                  g <= buffer0[0+read_pos_div48_mult3][31:24];
                  b <= buffer0[0+read_pos_div48_mult3][23:16];
                end
                6: begin
                  r <= buffer0[0+read_pos_div48_mult3][15:8];
                  g <= buffer0[0+read_pos_div48_mult3][7:0];
                  b <= buffer1[0+read_pos_div48_mult3][63:56];
                end
                9: begin
                  r <= buffer1[0+read_pos_div48_mult3][55:48];
                  g <= buffer1[0+read_pos_div48_mult3][47:40];
                  b <= buffer1[0+read_pos_div48_mult3][39:32];
                end
                12: begin
                  r <= buffer1[0+read_pos_div48_mult3][31:24];
                  g <= buffer1[0+read_pos_div48_mult3][23:16];
                  b <= buffer1[0+read_pos_div48_mult3][15:8];
                end
                15: begin
                  r <= buffer1[0+read_pos_div48_mult3][7:0];
                  g <= buffer0[1+read_pos_div48_mult3][63:56];
                  b <= buffer0[1+read_pos_div48_mult3][55:48];
                end
                18: begin
                  r <= buffer0[1+read_pos_div48_mult3][47:40];
                  g <= buffer0[1+read_pos_div48_mult3][39:32];
                  b <= buffer0[1+read_pos_div48_mult3][31:24];
                end
                21: begin
                  r <= buffer0[1+read_pos_div48_mult3][23:16];
                  g <= buffer0[1+read_pos_div48_mult3][15:8];
                  b <= buffer0[1+read_pos_div48_mult3][7:0];
                end
                24: begin
                  r <= buffer1[1+read_pos_div48_mult3][63:56];
                  g <= buffer1[1+read_pos_div48_mult3][55:48];
                  b <= buffer1[1+read_pos_div48_mult3][47:40];
                end
                27: begin
                  r <= buffer1[1+read_pos_div48_mult3][39:32];
                  g <= buffer1[1+read_pos_div48_mult3][31:24];
                  b <= buffer1[1+read_pos_div48_mult3][23:16];
                end
                30: begin
                  r <= buffer1[1+read_pos_div48_mult3][15:8];
                  g <= buffer1[1+read_pos_div48_mult3][7:0];
                  b <= buffer0[2+read_pos_div48_mult3][63:56];
                end
                33: begin
                  r <= buffer0[2+read_pos_div48_mult3][55:48];
                  g <= buffer0[2+read_pos_div48_mult3][47:40];
                  b <= buffer0[2+read_pos_div48_mult3][39:32];
                end
                36: begin
                  r <= buffer0[2+read_pos_div48_mult3][31:24];
                  g <= buffer0[2+read_pos_div48_mult3][23:16];
                  b <= buffer0[2+read_pos_div48_mult3][15:8];
                end
                39: begin
                  r <= buffer0[2+read_pos_div48_mult3][7:0];
                  g <= buffer1[2+read_pos_div48_mult3][63:56];
                  b <= buffer1[2+read_pos_div48_mult3][55:48];
                end
                42: begin
                  r <= buffer1[2+read_pos_div48_mult3][47:40];
                  g <= buffer1[2+read_pos_div48_mult3][39:32];
                  b <= buffer1[2+read_pos_div48_mult3][31:24];
                end
                45: begin
                  r <= buffer1[2+read_pos_div48_mult3][23:16];
                  g <= buffer1[2+read_pos_div48_mult3][15:8];
                  b <= buffer1[2+read_pos_div48_mult3][7:0];
                end
              endcase
              next_read_pos <= (read_pos + 3) % (BUFFER_SIZE * 8 * 2);
              // If we've crossed a 2-byte boundary, we need to mark that we're consuming a new pair of bytes
              if ((read_pos + 3) / 16 != read_pos / 16) begin
                // If the write position is the same as the next write position, we're consuming a byte
                // Otherwise, we have added and consumed a byte this cycle, so we don't need alter the buffer used count
                if (fic0_write_pos != last_seen_fic0_write_pos) begin
                  last_seen_fic0_write_pos <= fic0_write_pos;
                end else begin
                  fic0_buffer_used <= fic0_buffer_used - 1;
                end
                if (fic1_write_pos != last_seen_fic1_write_pos) begin
                  last_seen_fic1_write_pos <= fic1_write_pos;
                end else begin
                  fic1_buffer_used <= fic1_buffer_used - 1;
                end
              end else begin
                `UPDATE_BUFFER_USED_COUNT(fic0_buffer_used, fic0_write_pos,
                                          last_seen_fic0_write_pos);
                `UPDATE_BUFFER_USED_COUNT(fic1_buffer_used, fic1_write_pos,
                                          last_seen_fic1_write_pos);
              end
            end
          end else begin
            // Memory isn't ready, so we're just displaying a test pattern
            clock_counter <= clock_counter + 1;
            `UPDATE_BUFFER_USED_COUNT(fic0_buffer_used, fic0_write_pos, last_seen_fic0_write_pos);
            `UPDATE_BUFFER_USED_COUNT(fic1_buffer_used, fic1_write_pos, last_seen_fic1_write_pos);

            // Set colors
            if (pixel_offset / H_PIXELS < V_PIXELS / 2) r <= 8'h00;
            else r <= 8'hff;

            if (pixel_offset % H_PIXELS < H_PIXELS / 4) begin
              g <= 8'h00;
              b <= 8'h00;
            end else if (pixel_offset % H_PIXELS < H_PIXELS / 2) begin
              g <= 8'hff;
              b <= 8'h00;
            end else if (pixel_offset % H_PIXELS < H_PIXELS * 3 / 4) begin
              g <= 8'h00;
              b <= 8'hff;
            end else begin
              g <= 8'hff;
              b <= 8'hff;
            end
          end
        end else begin
          // Always update the clock counter when not displaying pixel data
          clock_counter <= clock_counter + 1;
          `UPDATE_BUFFER_USED_COUNT(fic0_buffer_used, fic0_write_pos, last_seen_fic0_write_pos);
          `UPDATE_BUFFER_USED_COUNT(fic1_buffer_used, fic1_write_pos, last_seen_fic1_write_pos);

          r <= 8'h00;
          g <= 8'h00;
          b <= 8'h00;
        end

      end else if (clock_counter == CLOCK_LOW_TIME - 1) begin
        pclk <= 1;

        clock_counter <= clock_counter + 1;
        `UPDATE_BUFFER_USED_COUNT(fic0_buffer_used, fic0_write_pos, last_seen_fic0_write_pos);
        `UPDATE_BUFFER_USED_COUNT(fic1_buffer_used, fic1_write_pos, last_seen_fic1_write_pos);
      end else begin
        clock_counter <= clock_counter + 1;

        `UPDATE_BUFFER_USED_COUNT(fic0_buffer_used, fic0_write_pos, last_seen_fic0_write_pos);
        `UPDATE_BUFFER_USED_COUNT(fic1_buffer_used, fic1_write_pos, last_seen_fic1_write_pos);
      end
    end
  end

endmodule
