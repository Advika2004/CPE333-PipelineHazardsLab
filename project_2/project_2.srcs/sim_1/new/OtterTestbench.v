`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/28/2024 02:57:12 PM
// Design Name: 
// Module Name: OtterTestbench
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module test_pipeline();
    logic CLK;
    logic rst;
    logic [15:0] LEDS;
    // logic [4:0] BTNC;
    logic [3:0] ANODES;

    default clocking tb_clk @(posedge CLK); endclocking

 OTTER_MCU dut(
    .INTR (),
    .CLK (CLK),
    .RESET (rst),
    .IOBUS_IN (),
    .IOBUS_OUT (),
    .IOBUS_ADDR (),
    .IOBUS_WR ()
   );

    // assign BTNC[4] = rst;

    always begin
    #5 CLK = ~CLK; 
    end  

    initial begin
        CLK = 0; rst = 1;
       // @(posedge CLK iff DUT.my_otter.DE_EX.PC == 32'h01ec);
        #10 rst = 0;

        // $display("Passed Testall");
        // $finish;
    end

endmodule