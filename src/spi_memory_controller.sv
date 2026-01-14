// ============================================================================
// spi_memory_controller.sv — SPI Memory Controller for Neander CPU (16-bit data)
// ============================================================================
// Bridges the CPU's parallel memory interface to serial SPI SRAM
// Supports standard SPI SRAM commands (23LC512, 23K256, etc.)
//
// CPU uses 16-bit addresses (64KB address space) and 16-bit data
//
// Protocol for 16-bit data (little-endian, two byte transfers):
//   READ:  CS=0, send 0x03, send addr_hi, send addr_lo, receive data_lo, data_hi, CS=1
//   WRITE: CS=0, send 0x02, send addr_hi, send addr_lo, send data_lo, data_hi, CS=1
//
// Timing: SPI clock = clk/2 (half-speed), each byte takes 16 CPU cycles
// Total: ~5 bytes = ~80 CPU cycles per 16-bit access
// ============================================================================

module spi_memory_controller (
    input  logic        clk,
    input  logic        reset,

    // CPU Interface (16-bit addressing, 16-bit data)
    input  logic        mem_req,      // Memory access request
    input  logic        mem_we,       // 0 = read, 1 = write
    input  logic [15:0] mem_addr,     // 16-bit address (64KB space)
    input  logic [15:0] mem_wdata,    // Write data (16-bit)
    output logic [15:0] mem_rdata,    // Read data (16-bit)
    output logic        mem_ready,    // Access complete (1 cycle pulse)

    // SPI Interface
    output logic        spi_cs_n,     // Chip select (active low)
    output logic        spi_sclk,     // Serial clock
    output logic        spi_mosi,     // Master Out Slave In
    input  logic        spi_miso      // Master In Slave Out
);

    // State machine states
    typedef enum logic [3:0] {
        IDLE,
        ADDR_LATCH,    // Wait one cycle for address to stabilize
        SEND_CMD,
        SEND_ADDR_HI,
        SEND_ADDR_LO,
        SEND_DATA_LO,  // Send low byte of data
        SEND_DATA_HI,  // Send high byte of data
        RECV_DATA_LO,  // Receive low byte of data
        RECV_DATA_HI,  // Receive high byte of data
        DONE
    } state_t;

    state_t state, next_state;

    // Internal registers
    logic [2:0]  bit_cnt;          // Bit counter (0-7)
    logic [7:0]  shift_out;        // Shift register for output
    logic [7:0]  shift_in;         // Shift register for input
    logic        sclk_phase;       // SCLK phase toggle
    logic        is_write;         // Latched write flag
    logic [15:0] addr_latch;       // Latched address (16-bit)
    logic [15:0] wdata_latch;      // Latched write data (16-bit)
    logic [7:0]  rdata_lo;         // Received low byte

    // SPI clock generation (clk/2)
    // SCLK toggles every CPU cycle when active
    // Data is shifted on rising edge of sclk_phase

    // MOSI output - MSB first
    assign spi_mosi = shift_out[7];

    // State register
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    // Sequential logic for SPI operation
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            spi_cs_n    <= 1'b1;
            spi_sclk    <= 1'b0;
            mem_ready   <= 1'b0;
            mem_rdata   <= 16'h0000;
            bit_cnt     <= 3'd0;
            shift_out   <= 8'h00;
            shift_in    <= 8'h00;
            sclk_phase  <= 1'b0;
            is_write    <= 1'b0;
            addr_latch  <= 16'h0000;
            wdata_latch <= 16'h0000;
            rdata_lo    <= 8'h00;
        end else begin
            // Default: clear ready pulse
            mem_ready <= 1'b0;

            case (state)
                IDLE: begin
                    spi_cs_n   <= 1'b1;
                    spi_sclk   <= 1'b0;
                    sclk_phase <= 1'b0;
                    bit_cnt    <= 3'd7;
                    // Wait for mem_req, then go to ADDR_LATCH to let address stabilize
                end

                ADDR_LATCH: begin
                    // Latch inputs (address is now stable after 1 cycle delay)
                    is_write    <= mem_we;
                    addr_latch  <= mem_addr;
                    wdata_latch <= mem_wdata;
                    // Start transaction
                    spi_cs_n    <= 1'b0;
                    // Load command: 0x03 for read, 0x02 for write
                    shift_out   <= mem_we ? 8'h02 : 8'h03;
                end

                SEND_CMD, SEND_ADDR_HI, SEND_ADDR_LO, SEND_DATA_LO, SEND_DATA_HI: begin
                    sclk_phase <= ~sclk_phase;
                    spi_sclk   <= ~sclk_phase;  // Generate SCLK

                    if (sclk_phase) begin
                        // Falling edge: shift out next bit
                        shift_out <= {shift_out[6:0], 1'b0};

                        if (bit_cnt == 3'd0) begin
                            // Byte complete, prepare next
                            bit_cnt <= 3'd7;
                            case (state)
                                SEND_CMD:     shift_out <= addr_latch[15:8];    // Load addr_hi
                                SEND_ADDR_HI: shift_out <= addr_latch[7:0];     // Load addr_lo
                                SEND_ADDR_LO: shift_out <= wdata_latch[7:0];    // Load data_lo (little-endian)
                                SEND_DATA_LO: shift_out <= wdata_latch[15:8];   // Load data_hi
                                default:      shift_out <= 8'h00;
                            endcase
                        end else begin
                            bit_cnt <= bit_cnt - 3'd1;
                        end
                    end
                end

                RECV_DATA_LO: begin
                    sclk_phase <= ~sclk_phase;
                    spi_sclk   <= ~sclk_phase;

                    if (~sclk_phase) begin
                        // Rising edge: sample MISO
                        shift_in <= {shift_in[6:0], spi_miso};
                    end

                    if (sclk_phase) begin
                        // Falling edge: check if done
                        if (bit_cnt == 3'd0) begin
                            // Low byte received - store it
                            rdata_lo <= shift_in;
                            bit_cnt <= 3'd7;
                        end else begin
                            bit_cnt <= bit_cnt - 3'd1;
                        end
                    end
                end

                RECV_DATA_HI: begin
                    sclk_phase <= ~sclk_phase;
                    spi_sclk   <= ~sclk_phase;

                    if (~sclk_phase) begin
                        // Rising edge: sample MISO
                        shift_in <= {shift_in[6:0], spi_miso};
                    end

                    if (sclk_phase) begin
                        // Falling edge: check if done
                        if (bit_cnt == 3'd0) begin
                            // High byte received - combine with low byte (little-endian)
                            mem_rdata <= {shift_in, rdata_lo};
                        end else begin
                            bit_cnt <= bit_cnt - 3'd1;
                        end
                    end
                end

                DONE: begin
                    spi_cs_n  <= 1'b1;
                    spi_sclk  <= 1'b0;
                    mem_ready <= 1'b1;  // Pulse ready for one cycle
                end

                default: begin
                    spi_cs_n <= 1'b1;
                end
            endcase
        end
    end

    // Next state logic
    always_comb begin
        next_state = state;

        case (state)
            IDLE: begin
                if (mem_req)
                    next_state = ADDR_LATCH;  // Wait for address to stabilize
            end

            ADDR_LATCH: begin
                next_state = SEND_CMD;  // Address is now stable, start SPI transaction
            end

            SEND_CMD: begin
                if (sclk_phase && bit_cnt == 3'd0)
                    next_state = SEND_ADDR_HI;
            end

            SEND_ADDR_HI: begin
                if (sclk_phase && bit_cnt == 3'd0)
                    next_state = SEND_ADDR_LO;
            end

            SEND_ADDR_LO: begin
                if (sclk_phase && bit_cnt == 3'd0) begin
                    if (is_write)
                        next_state = SEND_DATA_LO;  // Write: send low byte first
                    else
                        next_state = RECV_DATA_LO;  // Read: receive low byte first
                end
            end

            SEND_DATA_LO: begin
                if (sclk_phase && bit_cnt == 3'd0)
                    next_state = SEND_DATA_HI;  // Then send high byte
            end

            SEND_DATA_HI: begin
                if (sclk_phase && bit_cnt == 3'd0)
                    next_state = DONE;
            end

            RECV_DATA_LO: begin
                if (sclk_phase && bit_cnt == 3'd0)
                    next_state = RECV_DATA_HI;  // Then receive high byte
            end

            RECV_DATA_HI: begin
                if (sclk_phase && bit_cnt == 3'd0)
                    next_state = DONE;
            end

            DONE: begin
                next_state = IDLE;
            end

            default: begin
                next_state = IDLE;
            end
        endcase
    end

endmodule
