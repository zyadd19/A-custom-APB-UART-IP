`timescale 1ns / 1ps

module uart_receiver_tb();
    
    parameter BAUD_RATE  = 9600;          // baud rate in bit/s
    parameter CLK_FREQ   = 100_000_000;   // 100 MHz clock
    parameter DATA_BITS  = 8;             // number of bits to be transmitted

    integer i;
    real BAUD_RATE_REAL = BAUD_RATE;
    real BIT_PERIOD_NS  = 104166.6667; // 1_000_000_000.0 / BAUD_RATE_REAL; // bit period in ns (~104.167us)

    reg  PCLK, PRESETn;
    reg  rx_en, rx_rst;
    reg  rx_serial;
    wire rx_done, rx_busy, rx_error;
    wire [DATA_BITS-1:0] rx_data;

    // DUT instantiation
    uart_receiver #(
        .BAUD_RATE(BAUD_RATE),
        .CLK_FREQ(CLK_FREQ),
        .DATA_BITS(DATA_BITS)
    ) dut (
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .rx_en(rx_en),
        .rx_rst(rx_rst),
        .rx_serial(rx_serial),
        .rx_done(rx_done),
        .rx_busy(rx_busy),
        .rx_error(rx_error),
        .rx_data(rx_data)
    );

    // Test sequence
    initial begin
        PCLK      = 1;
        PRESETn   = 0;
        rx_en     = 0;
        rx_rst    = 0;
        rx_serial = 1'b1;  // idle line is high
        #200 PRESETn = 1;

        // Enable receiver
        rx_en = 1;

        // Reset receiver
        rx_rst = 1;
        #100 rx_rst = 0;

        // Wait a little then send data
        #500;
        send_byte(8'b0001_0110);    // 0x16
        send_byte(8'b0011_0010);  // 8'h32
        send_byte(8'b1010_1111);  // 8'haf

        #200;
        $finish;
    end

    // Task to send one UART frame
    task send_byte;
        input [7:0] data;
        begin
            // start bit
            rx_serial = 1'b0;
            #(BIT_PERIOD_NS);

            // data bits (LSB first)
            for (i = 0; i < DATA_BITS; i = i + 1) begin
                rx_serial = data[i];
                #(BIT_PERIOD_NS);
            end

            // stop bit
            rx_serial = 1'b1;
            #(BIT_PERIOD_NS);

            // idle between frames
            #(BIT_PERIOD_NS);
        end
    endtask

    // Clock generation: 100 MHz (10 ns period)
    always #5 PCLK = ~PCLK;

    // Optional waveform dump
    `ifdef VERDI_INSPECT
        initial begin
            $fsdbDumpfile("rx.fsdb");
            $fsdbDumpvars;
        end
    `endif  

endmodule
