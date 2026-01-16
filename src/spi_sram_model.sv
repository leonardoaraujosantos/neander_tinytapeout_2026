// ============================================================================
// spi_sram_model.sv — Behavioral SPI SRAM Model for Simulation
// ============================================================================
// Simulates a 64KB SPI SRAM (like 23LC512 or similar)
// Supports READ (0x03) and WRITE (0x02) commands
//
// This is a behavioral model for simulation only - not synthesizable
// ============================================================================

module spi_sram_model (
    input  logic spi_cs_n,    // Chip select (active low)
    input  logic spi_sclk,    // Serial clock
    input  logic spi_mosi,    // Master Out Slave In
    output logic spi_miso     // Master In Slave Out
);

    // 64KB memory array
    logic [7:0] memory [0:65535];

    // Internal state
    typedef enum logic [3:0] {
        IDLE,
        GET_CMD,
        GET_ADDR_HI,
        GET_ADDR_LO,
        READ_DATA,
        READ_DATA_HI,
        WRITE_DATA,
        WRITE_DATA_HI
    } state_t;

    state_t state;
    logic [7:0] cmd;
    logic [7:0] addr_hi;
    logic [7:0] addr_lo;
    logic [15:0] address;
    logic [7:0] shift_in;
    logic [7:0] shift_out;
    logic [2:0] bit_cnt;
    logic [2:0] read_bit_cnt;  // Counter for read data bits (to skip first negedge shift)
    logic       is_read;

    // Initialize memory to zero
    initial begin
        for (int i = 0; i < 65536; i++) begin
            memory[i] = 8'h00;
        end
        state = IDLE;
        bit_cnt = 3'd0;
        read_bit_cnt = 3'd0;
        shift_in = 8'h00;
        shift_out = 8'h00;
    end

    // MISO output - driven by shift register MSB
    assign spi_miso = shift_out[7];

    // Handle CS assertion/deassertion
    always @(posedge spi_cs_n or negedge spi_cs_n) begin
        if (spi_cs_n) begin
            // CS deasserted - reset state
            state <= IDLE;
            bit_cnt <= 3'd0;
        end else begin
            // CS asserted - start transaction
            state <= GET_CMD;
            bit_cnt <= 3'd0;
            shift_in <= 8'h00;
        end
    end

    // Handle SCLK edges (sample on rising, shift on falling)
    always @(posedge spi_sclk) begin
        if (!spi_cs_n) begin
            // Sample MOSI on rising edge
            shift_in <= {shift_in[6:0], spi_mosi};
            bit_cnt <= bit_cnt + 3'd1;

            // Check for byte complete (after 8 bits)
            if (bit_cnt == 3'd7) begin
                case (state)
                    GET_CMD: begin
                        cmd <= {shift_in[6:0], spi_mosi};
                        is_read <= ({shift_in[6:0], spi_mosi} == 8'h03);
                        state <= GET_ADDR_HI;
                    end
                    GET_ADDR_HI: begin
                        addr_hi <= {shift_in[6:0], spi_mosi};
                        state <= GET_ADDR_LO;
                    end
                    GET_ADDR_LO: begin
                        addr_lo <= {shift_in[6:0], spi_mosi};
                        address <= {addr_hi, shift_in[6:0], spi_mosi};
                        if (is_read) begin
                            state <= READ_DATA;
                            // Pre-load data for read
                            shift_out <= memory[{addr_hi, shift_in[6:0], spi_mosi}];
                            read_bit_cnt <= 3'd0;  // Reset read bit counter
                        end else begin
                            state <= WRITE_DATA;
                        end
                    end
                    WRITE_DATA: begin
                        // Write byte 0 to memory
                        memory[address] <= {shift_in[6:0], spi_mosi};
                        // Move to WRITE_DATA_HI for second byte
                        state <= WRITE_DATA_HI;
                    end
                    WRITE_DATA_HI: begin
                        // Write byte 1 to memory (address + 1)
                        memory[address + 16'd1] <= {shift_in[6:0], spi_mosi};
                        state <= IDLE;
                    end
                    READ_DATA: begin
                        // First byte done, prepare second byte
                        address <= address + 16'd1;
                        shift_out <= memory[address + 16'd1];
                        read_bit_cnt <= 3'd0;
                        state <= READ_DATA_HI;
                    end
                    READ_DATA_HI: begin
                        // Second byte done, go to idle
                        state <= IDLE;
                    end
                    default: ;
                endcase
            end
        end
    end

    // Shift out on falling edge for read
    // Skip the first negedge after entering READ_DATA/READ_DATA_HI - data must be stable for first sample
    always @(negedge spi_sclk) begin
        if (!spi_cs_n && (state == READ_DATA || state == READ_DATA_HI)) begin
            if (read_bit_cnt > 0) begin
                // Only shift after the first bit has been sampled
                shift_out <= {shift_out[6:0], 1'b0};
            end
            // Increment counter on each negedge (tracks how many bits have been sent)
            read_bit_cnt <= read_bit_cnt + 3'd1;
        end
    end

    // Task to load memory (for testbench use)
    task load_byte(input logic [15:0] addr, input logic [7:0] data);
        memory[addr] = data;
    endtask

    // Task to read memory (for testbench verification)
    function logic [7:0] read_byte(input logic [15:0] addr);
        return memory[addr];
    endfunction

endmodule
