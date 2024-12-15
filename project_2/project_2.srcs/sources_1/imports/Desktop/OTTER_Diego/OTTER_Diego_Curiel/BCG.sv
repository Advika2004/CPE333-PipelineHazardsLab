`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly San Luis Obispo
// Engineer: Diego Curiel
// Create Date: 02/09/2023 11:30:51 AM
// Module Name: BCG
//////////////////////////////////////////////////////////////////////////////////
module BCG(
    input logic [31:0] RS1,
    input logic [31:0] RS2,
    input logic [2:0] func3,
    input logic [31:0] ir,
    input logic jal,
    input logic jalr,
    input logic branch,
    output logic branch_taken,
    output logic [2:0] pc_source
    );

    // Intermediate signals for branch conditions
    logic BR_EQ, BR_LT, BR_LTU;

    // Assign branch condition comparisons
    assign BR_LT = $signed(RS1) < $signed(RS2);
    assign BR_LTU = RS1 < RS2;
    assign BR_EQ = RS1 == RS2;

    // Functional logic to determine PC source
    always_comb begin
        // Default to PC+4 unless specified otherwise
        pc_source = 2'b00; // Maps to 3'b000 for PC+4
        branch_taken = 1'b0;

        if (jal) begin
            pc_source = 3'b011; // Maps to 3'b011 for JAL
            branch_taken = 1'b1; //still branching for jal
        end else if (jalr) begin
            pc_source = 3'b001; // Maps to 3'b001 for JALR
            branch_taken = 1'b1; //still branching for jalr
        end else if (branch) begin
            case (func3)
                3'b000: begin
                    if (BR_EQ) 
                    begin
                        pc_source = 3'b010; // Maps to branch taken
                        branch_taken = 1'b1;
                    end
                    else
                    begin
                        pc_source = 3'b000; // Maps to PC+4
                        branch_taken = 1'b0;
                    end
                end
                3'b001: begin //BNE
                    if (!BR_EQ) begin
                        pc_source = 3'b010; // Maps to branch taken
                        branch_taken = 1'b1; $display("here"); end
                    else begin
                        pc_source = 3'b000; // Maps to PC+
                        branch_taken = 1'b0; end
                end
                3'b100: begin //BLT
                    if (BR_LT) begin
                        pc_source = 3'b010; // Maps to branch taken
                        branch_taken = 1'b1; end
                    else begin
                        pc_source = 3'b000; // Maps to PC+4
                        branch_taken = 1'b0; end
                end
                3'b101: begin //BGE
                    if (!BR_LT) begin
                        pc_source = 3'b010; // Maps to branch taken
                        branch_taken = 1'b1; end
                    else begin
                        pc_source = 3'b000; // Maps to PC+4
                        branch_taken = 1'b0; end
                end
                3'b110: begin //BLTU
                    if (BR_LTU) begin
                        pc_source = 3'b010; // Maps to branch taken
                        branch_taken = 1'b1; end
                    else begin
                        pc_source = 3'b000; // Maps to PC+4
                        branch_taken = 1'b0; end
                end
                3'b111: begin //BGEU
                    if (!BR_LTU) begin
                        pc_source = 3'b010; // Maps to branch taken
                        branch_taken = 1'b1;  end
                    else begin
                        pc_source = 3'b000; // Maps to PC+4
                        branch_taken = 1'b0; end
                end
                default: begin
                    pc_source = 3'b000; // Maps to PC+4
                    branch_taken = 1'b0;
                end
            endcase
        end
//        else 
//            pc_source = 3'b000;
//            branch_taken = 1'b1;
    end
endmodule

