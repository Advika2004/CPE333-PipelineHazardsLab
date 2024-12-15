`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/13/2024 03:51:41 PM
// Design Name: 
// Module Name: Branch_Hazard_Module
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


module Hazard_Module ( 
    input logic branch_taken,       // Signal indicating branch is taken
    output logic fetch2dec_flush,   // Flush signal for fetch to decode stage
    output logic dec2exec_flush,    // Flush signal for decode to execute stage
    output logic exec2mem_flush,    // Flush signal for execute to memory stage
    output logic mem2wb_flush       // Flush signal for memory to write-back stage
);

    // Generate flush signals based on branch_taken
    always_comb begin
        fetch2dec_flush = branch_taken;
        dec2exec_flush = branch_taken;
        exec2mem_flush = 1'b0; 
        mem2wb_flush = 1'b0;
    end

endmodule
