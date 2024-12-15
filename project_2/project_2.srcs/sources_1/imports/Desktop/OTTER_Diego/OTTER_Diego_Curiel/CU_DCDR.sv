`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: California Polytechnic University, San Luis Obispo
// Engineer: Diego Renato Curiel
// Create Date: 02/23/2023 09:39:49 AM
// Module Name: CU_DCDR
//////////////////////////////////////////////////////////////////////////////////

module CU_DCDR(
    input logic IR_30,
    input logic [6:0] IR_OPCODE,
    input logic [2:0] IR_FUNCT,
    output logic BRANCHsig, //used to be BR_EQ
    output logic JALsig, //used to be BR_LT
    output logic JALRsig, //used to be BR_LTU
    output logic [3:0] ALU_FUN,
    output logic ALU_SRCA,
    output logic [1:0] ALU_SRCB,
    output logic [1:0] RF_WR_SEL,
    
    //FSM outputs
    
    output logic REG_WRITE,
    output logic MEM_WE2,
    output logic MEM_RDEN2
    
    );
    
    
    //ENUMS for the instruction stuff
    typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BTYPE    = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           ITYPE    = 7'b0010011,
           RTYPE    = 7'b0110011
 } opcode_t;  
 opcode_t OPCODE;
 
 assign OPCODE = opcode_t'(IR_OPCODE);
    
    //Create always comb clock for decoder logic
    always_comb begin
        //Instantiate all outputs to 0 so as to avoid
        //unwanted leftovers from previous operations
        //and maintain direct control of outputs through
        //case statement below
        ALU_FUN = 4'b0000;
        ALU_SRCA = 1'b0;
        ALU_SRCB = 2'b00;
        RF_WR_SEL = 2'b00;
        
        //FSM things
        REG_WRITE = 1'b0;
        MEM_WE2 = 1'b0;
        MEM_RDEN2 = 1'b0; 
        
        BRANCHsig = 1'b0;
        JALsig = 1'b0;
        JALRsig = 1'b0;
    
        //Case statement depending on the opcode for the 
        //instruction, or the last seven bits of each instruction
  
        
        case (IR_OPCODE)
            AUIPC: begin // AUIPC
                ALU_SRCA = 1'b1;
                ALU_SRCB = 2'b11;
                RF_WR_SEL = 2'b01; //comes from ALU RESULT
//                PC_WRITE = 1'b1; 
                REG_WRITE = 1'b1;
            end
            JAL: begin // JAL
//                PC_WRITE = 1'b1;
                REG_WRITE = 1'b1;
                JALsig = 1'b1;
            end
            JALR: begin // JALR
//                PC_WRITE = 1'b1;
                REG_WRITE = 1'b1;
                JALRsig = 1'b1;
            end
            STORE: begin // Store Instructions
                ALU_SRCB = 2'b10;
 //               PC_WRITE = 1'b1;
                MEM_WE2 = 1'b1;
            end
            LOAD: begin // Load Instructions
                ALU_SRCB = 2'b01;
                RF_WR_SEL = 2'b10; //dout2 because load is reading from memory output and storing that in register
                MEM_RDEN2 = 1'b1;
//                PC_WRITE = 1'b0;
                REG_WRITE = 1'b1;
            end
            LUI: begin // LUI
                ALU_FUN = 4'b1001;
                ALU_SRCA = 1'b1;
                RF_WR_SEL = 2'b01; //immediate value comes from ALU RESULT 
 //               PC_WRITE = 1'b1;
                REG_WRITE = 1'b1;
            end
            ITYPE: begin // I-Type
                //set constants for all I-type instructions
                ALU_SRCB = 2'b01;
                RF_WR_SEL = 2'b01; //immediate values come from alu result again 
 //               PC_WRITE = 1'b1;
                REG_WRITE = 1'b1;
                
                //Nested case statement
                //dependent on the function 3 bits
                case (IR_FUNCT)
                    3'b000: begin 
                        ALU_FUN = 4'b0000; //ADDI
                    end
                    
                    3'b001: begin
                        ALU_FUN = 4'b0001; //SLLI
                    end
                    
                    3'b010: begin
                        ALU_FUN = 4'b0010;  //SLTI
                    end
                    
                    3'b011: begin
                        ALU_FUN = 4'b0011; //SLTIU
                    end
                    
                    3'b100: begin
                        ALU_FUN = 4'b0100; //XORI
                    end
                    
    
                    
                    3'b101: begin
                        //nested case statement
                        //dependent on the 30th bit for 
                        //instructions that have the same opcode and 
                        //fucntion 3 bits
                        case(IR_30)
                            1'b0: begin //SRLI
                                ALU_FUN = 4'b0101;
                            end
                            
                            1'b1: begin //SRAI
                                ALU_FUN = 4'b1101; 
                            end
                            
                            default: begin end
                        endcase
                    end
                    3'b110: begin
                        ALU_FUN = 4'b0110;  //ORI
                    end
                    3'b111: begin
                        ALU_FUN = 4'b0111; //ANDI
                    end
                endcase
            end
            RTYPE: begin // R-Type
                //set constants for all R-types;
                //ALU_FUN is just the concatenation of
                //the 30th bit and the function 3 bits
                RF_WR_SEL = 2'b01; //should come from ALU result
                ALU_FUN = {IR_30, IR_FUNCT};
                //PC_WRITE = 1'b1;
                REG_WRITE = 1'b1;
            end
            BTYPE: begin // B-Type
                //nested case statement dependent on the
                //function three bits.
                //Because there are six real branch instructions, there
                //are six pairs of if-else statements in each of six cases
                //for the branch instructions.
                //PC_WRITE = 1'b1;
                BRANCHsig = 1'b1;
            end
            default: begin end
        endcase
    end
    
endmodule
