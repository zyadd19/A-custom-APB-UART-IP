
////////////////////////////////////////////////////////////////////////
//  UART Receiver 
//
//  rx_done set 1 for 1 cycle when all bits are received
//  async active low reset_n
////////////////////////////////////////////////////////////////////////

module uart_receiver #(
    parameter BAUD_RATE =       9600,           // baud rate in bit/s
    parameter CLK_FREQ  =       100_000_000,    // clock frequency in Hz
    parameter DATA_BITS =       8               // number of bits to be transmitted
) ( 
    input                           PCLK,       // system clock (100 MHz)
    input                           PRESETn,    // async active-low reset
    input                           rx_en,      // enable reception
    input                           rx_rst,     // soft reset for RX
    input                           rx_serial,  // RX serial input

    output  reg                     rx_done,    // reception complete
    output  reg                     rx_busy,    // busy during reception
    output  reg                     rx_error,   // framing error
    output  reg [DATA_BITS-1:0]     rx_data     // received data
);

    localparam CLKS_PER_BIT =   CLK_FREQ / BAUD_RATE;          
    localparam CLK_CNTER_BW =   $clog2(CLKS_PER_BIT) + 1;     
    localparam BIT_CNTER_BW =   $clog2(DATA_BITS) + 1;        

    // FSM states
    localparam IDLE            = 2'b00;
    localparam START_BIT       = 2'b01;
    localparam DATA_BITS_STATE = 2'b11;
    localparam STOP_BIT        = 2'b10;

    reg [CLK_CNTER_BW-1:0]  clk_cnter;  
    reg [BIT_CNTER_BW-1:0]  bit_cnter;  
    
    // synchronizer
    reg serial_sync0, serial_sync1;

    reg [1:0]  state, next_state;

    reg start_bit_mid;
    reg data_bit_mid;
    reg stop_bit_mid;

    // -------------------------------
    // FSM next state logic
    // -------------------------------
    always @(*) begin
        next_state = IDLE;
        case (state)
            IDLE: if (serial_sync1 == 1'b0 && rx_en) 
                      next_state = START_BIT;   
                  else 
                      next_state = IDLE;

            START_BIT: if (start_bit_mid) begin
                          if (serial_sync1 == 1'b0) 
                              next_state = DATA_BITS_STATE;
                          else 
                              next_state = IDLE;  // noise
                       end else 
                          next_state = START_BIT;

            DATA_BITS_STATE: if (bit_cnter == DATA_BITS) 
                                next_state = STOP_BIT;
                             else 
                                next_state = DATA_BITS_STATE;

            STOP_BIT: if (stop_bit_mid) 
                          next_state = IDLE; 
                      else 
                          next_state = STOP_BIT;
        endcase
    end

    // -------------------------------
    // FSM output logic
    // -------------------------------
    always @(*) begin
        start_bit_mid = 1'b0;
        data_bit_mid  = 1'b0;
        stop_bit_mid  = 1'b0;
        case (state)
            START_BIT:       start_bit_mid = (clk_cnter == CLKS_PER_BIT/2);
            DATA_BITS_STATE: data_bit_mid  = (clk_cnter == CLKS_PER_BIT);
            STOP_BIT:        stop_bit_mid  = (clk_cnter == CLKS_PER_BIT);
        endcase
    end

    // -------------------------------
    // FSM state register
    // -------------------------------
    always @(posedge PCLK or negedge PRESETn) begin
        if (~PRESETn || rx_rst) 
            state <= IDLE;
        else 
            state <= next_state;
    end

    // -------------------------------
    // Input synchronizer
    // -------------------------------
    always @(posedge PCLK or negedge PRESETn) begin
        if (~PRESETn) begin
            serial_sync0 <= 1'b1;
            serial_sync1 <= 1'b1;
        end else begin
            serial_sync0 <= rx_serial;
            serial_sync1 <= serial_sync0;
        end
    end

    // -------------------------------
    // Data sampling
    // -------------------------------
    always @(posedge PCLK or negedge PRESETn) begin
        if (~PRESETn || rx_rst) 
            rx_data <= {DATA_BITS{1'b0}};
        else if (data_bit_mid) 
            rx_data[bit_cnter] <= serial_sync1;
    end

    // -------------------------------
    // Counters
    // -------------------------------
    always @(posedge PCLK or negedge PRESETn) begin
        if (~PRESETn || rx_rst) begin
            clk_cnter <= {CLK_CNTER_BW{1'b0}};
            bit_cnter <= {BIT_CNTER_BW{1'b0}};
        end else begin
            case (state)
                IDLE: begin
                    clk_cnter <= 0;
                    bit_cnter <= 0;
                end
                START_BIT: if (clk_cnter < CLKS_PER_BIT/2) 
                               clk_cnter <= clk_cnter + 1'b1;
                           else 
                               clk_cnter <= 0;
                DATA_BITS_STATE: begin
                    if (clk_cnter < CLKS_PER_BIT) 
                        clk_cnter <= clk_cnter + 1'b1;
                    else 
                        clk_cnter <= 0;

                    if (data_bit_mid) begin
                        if (bit_cnter < DATA_BITS) 
                            bit_cnter <= bit_cnter + 1'b1;
                        else 
                            bit_cnter <= 0;
                    end
                end
                STOP_BIT: if (clk_cnter < CLKS_PER_BIT) 
                              clk_cnter <= clk_cnter + 1'b1;
                          else 
                              clk_cnter <= 0;
            endcase
        end
    end

    // -------------------------------
    // Status signals
    // -------------------------------
    always @(posedge PCLK or negedge PRESETn) begin
        if (~PRESETn || rx_rst) begin
            rx_done  <= 1'b0;
            rx_busy  <= 1'b0;
            rx_error <= 1'b0;
        end else begin
            rx_busy  <= (state != IDLE);
            rx_done  <= stop_bit_mid;
            rx_error <= (stop_bit_mid && serial_sync1 != 1'b1); // framing error
        end
    end

endmodule


