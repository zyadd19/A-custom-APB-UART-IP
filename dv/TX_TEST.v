
`timescale 1ns / 1ps

module uart_transmitter_tb();
    
    parameter BAUD_RATE   = 9600;          // baud rate in bit/s
    parameter CLK_FREQ    = 100_000_000;   // 100 MHz clock
    parameter DATA_BITS   = 8;             // number of bits to be transmitted

    real BAUD_RATE_REAL = BAUD_RATE;
    real BAUD_PERIOD_NS = 104166.6667 ; //1_000_000_000.0 / BAUD_RATE_REAL;   // baud period in ns (~104.167us)
    real WAIT_TIME      = 1041666.667 ; //BAUD_PERIOD_NS * (2 + DATA_BITS);   // frame duration (start + data + stop)

    reg  PCLK, PRESETn;
    reg  tx_en;
    reg  [DATA_BITS-1:0] tx_data;
    wire tx_busy, tx_done;
    wire tx_serial;

    // DUT instantiation
    uart_transmitter #(
        .BAUD_RATE(BAUD_RATE),
        .CLK_FREQ(CLK_FREQ),
        .DATA_BITS(DATA_BITS)
    ) dut (
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .tx_en(tx_en),
        .tx_data(tx_data),
        .tx_busy(tx_busy),
        .tx_done(tx_done),
        .tx_serial(tx_serial)
    );

    // Test sequence
    initial begin
        PCLK     = 1;
        PRESETn  = 0;
        tx_en    = 1'b0;
        #100 PRESETn = 1;

        send_data(8'b0101_0101);   // 0x55
        send_data(8'b1001_1001);   // 0x99

        #200;
        $finish;
    end

    // Task to send one byte
    task send_data;
        input [7:0] data;
        begin
            tx_data = data;
            #20 tx_en = 1'b1;      // trigger pulse
            #20 tx_en = 1'b0;
            #WAIT_TIME;            // wait until frame finishes
            #200;                  // gap between frames
        end
    endtask

    // 100 MHz clock generation (10 ns period)
    always #5 PCLK = ~PCLK; 

endmodule
