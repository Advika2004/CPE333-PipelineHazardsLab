`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/13/2024 11:06:59 PM
// Design Name: 
// Module Name: ForwardingUnit
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


module ForwardingUnit(
    input logic [4:0] dec2exec_rs1,        // rs1 from decode2exec stage
    input logic [4:0] dec2exec_rs2,        // rs2 from decode2exec stage
    input logic [4:0] mem_rd,         // rd in execute stage
    input logic mem_reg_write,        // regwrite from execute stage
    input logic [4:0] wb_rd,        // rd in memory stage
    input logic wb_reg_write,       // regwrite in memory stage
    output logic [1:0] FORWARDA,     // mux select for forwarding rs1
    output logic [1:0] FORWARDB      // mux select for forwarding rs2
);

    always_comb begin
         //Default to no forwarding
        FORWARDA = 2'b00;
        FORWARDB = 2'b00;

        // Forward from memory stage for A
        if (mem_reg_write && (mem_rd != 0) && (mem_rd == dec2exec_rs1)) 
            FORWARDA = 2'b10;
        
        else if (wb_reg_write && (wb_rd != 0) && (wb_rd == dec2exec_rs1))
            FORWARDA = 2'b01;
    
        else 
            FORWARDA = 2'b00;
             
        //forwarding from writeback stage for B
        
        if (mem_reg_write && (mem_rd != 0) && (mem_rd == dec2exec_rs2)) 
            FORWARDB = 2'b10;
        
        else if (wb_reg_write && (wb_rd != 0) && (wb_rd == dec2exec_rs2))
            FORWARDB = 2'b01;
            
        else 
            FORWARDB = 2'b00;
        
end        
endmodule       
        
        
//        if (mem_reg_write && (mem_rd != 0)) begin // if my memory register write is on and my destination register is not zero
//            if (mem_rd == dec2exec_rs1) FORWARDA = 2'b10; //if my memory destination register is the same as my rs1 from my decode stage then forward stuff
//            if (mem_rd == dec2exec_rs2) FORWARDB = 2'b10; //check same thing for rs2 
//        end

//        // Forward from memory stage
//        if (wb_reg_write && (wb_rd != 0)) begin // if my writeback register write is on and my writeback destination is not zero 
//            if (wb_rd == dec2exec_rs1) FORWARDA = 2'b01; //if my writeback destination register doesnt e 
//            if (wb_rd == dec2exec_rs2) FORWARDB = 2'b01;
//        end
//    end

