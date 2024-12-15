`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: California Polytechnic University, San Luis Obispo
// Engineer: Diego Renato Curiel
// Create Date: 02/25/2023 10:55:14 PM
// Module Name: TwoMux
//////////////////////////////////////////////////////////////////////////////////

module TwoMux(
    input logic ALU_SRC_A, //Select signal
    input logic [31:0] RS1, //register value from id/ex
    input logic [31:0] U_TYPE, //immediate value
    output logic [31:0] SRC_A //alu input A
    );
    
    //Create a generic two-to-one MUX to be used for the ALU.
    always_comb begin
        case(ALU_SRC_A)
            2'b00: begin SRC_A = RS1; end
            2'b01: begin SRC_A = U_TYPE; end
            default:  SRC_A = RS1; //do I need a default case? 
        endcase
    end
    
endmodule
