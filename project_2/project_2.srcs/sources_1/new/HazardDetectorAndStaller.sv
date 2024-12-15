`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/14/2024 08:57:44 PM
// Design Name: 
// Module Name: HazardDetectorAndStaller
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


module HazardDetectorAndStaller(
    input logic MEMREAD,              //memrden2
    input logic [4:0] dec2exec_rd,    //destination register from execute stage
    input logic [4:0] fetch2dec_rs1,  //rs1 from decode stage
    input logic [4:0] fetch2dec_rs2,  //rs2 from decode stage
    output logic HazardDetected       //Hazard detection signal
);

    always_comb begin
        // Default value
        HazardDetected = 1'b0;

        // Load-use hazard detection
        if (MEMREAD && ((dec2exec_rd == fetch2dec_rs1) || (dec2exec_rd == fetch2dec_rs2))) 
        begin
            HazardDetected = 1'b1;
        end
    end

endmodule
