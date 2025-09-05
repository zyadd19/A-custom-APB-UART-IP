
module apb_uart_wrapper #(
    parameter PADDR_WIDTH = 32,
    parameter PWDATA_WIDTH = 32,
    parameter PRDATA_WIDTH = 32,
    parameter DATA_BITS = 8
)(
    // APB Slave Interface
    input                       PCLK,
    input                       PRESETn,
    input  [PADDR_WIDTH-1:0]    PADDR,
    input                       PSEL,
    input                       PENABLE,
    input                       PWRITE,
    input  [PWDATA_WIDTH-1:0]   PWDATA,
    output reg [PRDATA_WIDTH-1:0] PRDATA,
    output reg                  PREADY,
    output reg                  PSLVERR,

    // UART I/O
    output                      tx_serial,
    input                       rx_serial
);

    // Register Definitions (32-bit registers)
    localparam CTRL_REG_ADDR = 32'h0000;
    localparam STATS_REG_ADDR = 32'h0001;
    localparam TX_DATA_ADDR = 32'h0002;
    localparam RX_DATA_ADDR = 32'h0003;
    localparam BAUDIV_ADDR = 32'h0004;

    // Internal Registers
    reg [PWDATA_WIDTH-1:0] ctrl_reg;
    reg [PWDATA_WIDTH-1:0] stats_reg;
    reg [PWDATA_WIDTH-1:0] tx_data_reg;
    reg [PWDATA_WIDTH-1:0] rx_data_reg;
    reg [PWDATA_WIDTH-1:0] baudiv_reg;

    // Control and Status Signals for UART Core
    wire tx_en, rx_en, tx_rst, rx_rst;
    wire tx_busy, tx_done, rx_busy, rx_done, rx_error;
    wire [DATA_BITS-1:0] tx_data_uart, rx_data_uart;
    wire [PWDATA_WIDTH-1:0] baudiv_uart;

    assign tx_en  = ctrl_reg[0];
    assign rx_en  = ctrl_reg[1];
    assign tx_rst = ctrl_reg[2];
    assign rx_rst = ctrl_reg[3];
    assign tx_data_uart = tx_data_reg[DATA_BITS-1:0];
    assign baudiv_uart = baudiv_reg;
    
    // Status bits for STATS_REG
    localparam TX_BUSY_BIT = 0;
    localparam TX_DONE_BIT = 1;
    localparam RX_BUSY_BIT = 2;
    localparam RX_DONE_BIT = 3;
    localparam RX_ERROR_BIT = 4;

    // APB State Machine (Simplified)
    reg [1:0] apb_state;
    localparam APB_IDLE = 2'b00;
    localparam APB_SETUP = 2'b01;
    localparam APB_ACCESS = 2'b10;

    // FSM for APB Protocol
    always @(posedge PCLK or negedge PRESETn) begin
        if (~PRESETn) begin
            apb_state <= APB_IDLE;
            PREADY <= 1'b0;
        end else begin
            case (apb_state)
                APB_IDLE: begin
                    PREADY <= 1'b0;
                    if (PSEL) begin
                        apb_state <= APB_SETUP;
                    end else begin
                        apb_state <= APB_IDLE;
                    end
                end
                APB_SETUP: begin
                    PREADY <= 1'b0;
                    if (PENABLE) begin
                        apb_state <= APB_ACCESS;
                    end else begin
                        apb_state <= APB_SETUP;
                    end
                end
                APB_ACCESS: begin
                    PREADY <= 1'b1;
                    apb_state <= APB_IDLE;
                end
                default: begin
                    apb_state <= APB_IDLE;
                    PREADY <= 1'b0;
                end
            endcase
        end
    end
    
    // APB Read/Write Logic
    always @(posedge PCLK or negedge PRESETn) begin
        if (~PRESETn) begin
            ctrl_reg <= 0;
            stats_reg <= 0;
            tx_data_reg <= 0;
            rx_data_reg <= 0;
            baudiv_reg <= 0;
            PRDATA <= 0;
            PSLVERR <= 0;
        end else if (PSEL && PENABLE) begin // APB Access Phase
            if (PWRITE) begin // Write Cycle
                PSLVERR <= 1'b0; // No error on writes (for this design)
                case (PADDR)
                    CTRL_REG_ADDR:  ctrl_reg <= PWDATA;
                    TX_DATA_ADDR:   tx_data_reg <= PWDATA;
                    BAUDIV_ADDR:    baudiv_reg <= PWDATA;
                    default:        PSLVERR <= 1'b1; // Slave error for invalid address
                endcase
            end else begin // Read Cycle
                PSLVERR <= 1'b0; // No error on reads (for this design)
                case (PADDR)
                    CTRL_REG_ADDR:  PRDATA <= ctrl_reg;
                    STATS_REG_ADDR: PRDATA <= stats_reg;
                    TX_DATA_ADDR:   PRDATA <= tx_data_reg;
                    RX_DATA_ADDR:   PRDATA <= rx_data_reg;
                    BAUDIV_ADDR:    PRDATA <= baudiv_reg;
                    default: begin
                        PRDATA <= 0;
                        PSLVERR <= 1'b1; // Slave error for invalid address
                    end
                endcase
            end
        end
        // Update stats_reg from UART core outputs
        stats_reg <= {27'h0, rx_error, rx_done, rx_busy, tx_done, tx_busy};
        
        // Update RX_DATA_REG from UART receiver
        if (rx_done) begin
            rx_data_reg <= {24'h0, rx_data_uart};
        end
    end

    // Instance of UART Transmitter
    uart_transmitter #(
        .BAUD_RATE(9600),
        .CLK_FREQ(100_000_000),
        .DATA_BITS(DATA_BITS)
    ) tx_inst (
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .tx_en(tx_en),
        .tx_data(tx_data_uart),
        .tx_busy(tx_busy),
        .tx_done(tx_done),
        .tx_serial(tx_serial)
    );
    
    // Instance of UART Receiver
    uart_receiver #(
        .BAUD_RATE(9600),
        .CLK_FREQ(100_000_000),
        .DATA_BITS(DATA_BITS)
    ) rx_inst (
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .rx_en(rx_en),
        .rx_rst(rx_rst),
        .rx_serial(rx_serial),
        .rx_done(rx_done),
        .rx_busy(rx_busy),
        .rx_error(rx_error),
        .rx_data(rx_data_uart)
    );

endmodule

















`timescale 1ns / 1ps

module apb_uart_wrapper_tb();

    // APB Parameters
    localparam PADDR_WIDTH = 32;
    localparam PWDATA_WIDTH = 32;
    localparam PRDATA_WIDTH = 32;

    // Register Addresses
    localparam CTRL_REG_ADDR  = 32'h0000;
    localparam STATS_REG_ADDR = 32'h0001;
    localparam TX_DATA_ADDR   = 32'h0002;
    localparam RX_DATA_ADDR   = 32'h0003;
    localparam BAUDIV_ADDR    = 32'h0004;

    // Testbench signals
    reg                 PCLK;
    reg                 PRESETn;
    reg [PADDR_WIDTH-1:0] PADDR;
    reg                 PSEL;
    reg                 PENABLE;
    reg                 PWRITE;
    reg [PWDATA_WIDTH-1:0]  PWDATA;
    wire [PRDATA_WIDTH-1:0] PRDATA;
    wire                PREADY;
    wire                PSLVERR;
    
    // UART signals
    wire tx_serial;
    reg  rx_serial;

    // A register to hold the data read from the APB wrapper
    reg [PRDATA_WIDTH-1:0] read_data;

    // Instantiate the APB UART Wrapper
    apb_uart_wrapper #(
        .PADDR_WIDTH(PADDR_WIDTH),
        .PWDATA_WIDTH(PWDATA_WIDTH),
        .PRDATA_WIDTH(PRDATA_WIDTH)
    ) dut (
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .PADDR(PADDR),
        .PSEL(PSEL),
        .PENABLE(PENABLE),
        .PWRITE(PWRITE),
        .PWDATA(PWDATA),
        .PRDATA(PRDATA),
        .PREADY(PREADY),
        .PSLVERR(PSLVERR),
        .tx_serial(tx_serial),
        .rx_serial(rx_serial)
    );

    // Clock Generation
    always #5 PCLK = ~PCLK;

    // Test Tasks
    task apb_write;
        input [PADDR_WIDTH-1:0] addr;
        input [PWDATA_WIDTH-1:0] data;
    begin
        // Setup phase
        @(posedge PCLK);
        PSEL = 1'b1;
        PENABLE = 1'b0;
        PWRITE = 1'b1;
        PADDR = addr;
        PWDATA = data;
        
        // Access phase
        @(posedge PCLK);
        PENABLE = 1'b1;
        
        // Wait for transaction to complete
        @(posedge PCLK);
        while (PREADY == 1'b0) @(posedge PCLK);
        
        // Return to IDLE
        @(posedge PCLK);
        PSEL = 1'b0;
        PENABLE = 1'b0;
    end
    endtask

    task apb_read;
        input [PADDR_WIDTH-1:0] addr;
        output reg [PRDATA_WIDTH-1:0] data;
    begin
        // Setup phase
        @(posedge PCLK);
        PSEL = 1'b1;
        PENABLE = 1'b0;
        PWRITE = 1'b0;
        PADDR = addr;
        
        // Access phase
        @(posedge PCLK);
        PENABLE = 1'b1;
        
        // Wait for transaction to complete
        @(posedge PCLK);
        while (PREADY == 1'b0) @(posedge PCLK);
        
        data = PRDATA;
        
        // Return to IDLE
        @(posedge PCLK);
        PSEL = 1'b0;
        PENABLE = 1'b0;
    end
    endtask

    // Main Test Sequence
    initial begin
        $dumpfile("apb_uart_wrapper.vcd");
        $dumpvars(0, apb_uart_wrapper_tb);

        // Initial values
        PCLK = 0;
        PRESETn = 0;
        PSEL = 0;
        PENABLE = 0;
        PWRITE = 0;
        PADDR = 0;
        PWDATA = 0;
        rx_serial = 1;
        read_data = 0;

        // Apply reset
        #100 PRESETn = 1;
        
        $display("--- Starting APB Wrapper Test ---");

        // 1. Write to TX_DATA and CTRL_REG to start transmission
        $display("Test 1: Writing data 8'h55 to TX_DATA register...");
        apb_write(TX_DATA_ADDR, 32'h0000_0055);

        $display("Test 2: Writing 8'h1 to CTRL_REG to enable TX...");
        apb_write(CTRL_REG_ADDR, 32'h0000_0001); // tx_en = 1
        
        // Wait for a full transmission
        #1_000_000;
        
        // 2. Read STATS_REG to check if tx_done is set
        $display("Test 3: Reading STATS_REG to check status...");
        apb_read(STATS_REG_ADDR, read_data);
        $display("STATS_REG after TX: 0x%h. Expected tx_done.", read_data);

        // 3. Reset TX and RX
        $display("Test 4: Writing 8'hC to CTRL_REG to reset TX and RX...");
        apb_write(CTRL_REG_ADDR, 32'h0000_000C); // tx_rst = 1, rx_rst = 1
        #100;
        apb_write(CTRL_REG_ADDR, 32'h0000_0000); // Clear reset bits

        // 4. Simulate an incoming byte and read RX_DATA
        $display("Test 5: Simulating incoming byte 8'hAF...");
        apb_write(CTRL_REG_ADDR, 32'h0000_0002); // rx_en = 1
        @(posedge PCLK);
        
        // Simulate start bit
        rx_serial = 1'b0;
        #104167;

        // Simulate data bits (8'b1010_1111, LSB first)
        rx_serial = 1'b1; #104167;
        rx_serial = 1'b1; #104167;
        rx_serial = 1'b1; #104167;
        rx_serial = 1'b1; #104167;
        rx_serial = 1'b0; #104167;
        rx_serial = 1'b1; #104167;
        rx_serial = 1'b0; #104167;
        rx_serial = 1'b1; #104167;

        // Simulate stop bit
        rx_serial = 1'b1;
        #104167;

        // Read received data
        $display("Test 6: Reading RX_DATA register...");
        apb_read(RX_DATA_ADDR, read_data);
        $display("RX_DATA: 0x%h. Expected 0x0000_00AF", read_data);

        $display("--- Test finished ---");
        $finish;
    end

endmodule
