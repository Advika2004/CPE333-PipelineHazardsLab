`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
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

  typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
 
//the struct that was given        
//typedef struct packed{
//    opcode_t opcode;
//    logic [4:0] rs1_addr;
//    logic [4:0] rs2_addr;
//    logic [4:0] rd_addr;
//    logic rs1_used;
//    logic rs2_used;
//    logic rd_used;
//    logic [3:0] alu_fun;
//    logic memWrite;
//    logic memRead2;
//    logic regWrite;
//    logic [1:0] rf_wr_sel;
//    logic [2:0] mem_type;  //sign, size
//    logic [31:0] pc;
//} instr_t;

//my new struct that I added signals to
typedef struct packed{
    logic [31:0] pc_out;
    logic [31:0] pc_out_inc;
    logic [31:0] rs1;
    logic [31:0] rs2;
    logic [31:0] Utype;
    logic [31:0] Itype;
    logic [31:0] Stype;
    logic [31:0] Btype;
    logic [31:0] Jtype;
    logic alu_src_a; //fist mux select
    logic [1:0] alu_src_b; //second mux select
    logic [1:0] wb_sel; //selecting the output to the reg file
    logic [3:0] alu_fun;
    logic mem_we2; //memory write enable 
    logic mem_rden2; //memory read enable 2 (data memory)
    logic  branchStr;
    logic  jalStr;
    logic  jalrStr; 
    logic [31:0] alu_result; //output of the ALU
    logic [31:0] dout2; //data memory output
    logic [31:0] ir; //instruction memory output
    logic [2:0] func3; //func3 code
    logic reg_wr;
} instr_t;


module OTTER_MCU(
                input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           

//these are all the things directly connected between modules so I made them wires 
    logic [31:0] jalrWire;
    logic [31:0] branchWire;
    logic [31:0] jalWire;
    logic [31:0] int_pc;
    logic [2:0] pc_source; //should be 2 bits so that it can make 4
    logic [31:0] srcA;
    logic [31:0] srcB;
    logic [31:0] wb_out;
    logic branch_taken;
    
  // Stalling signals
    logic HazardDetected;
    logic fetch2dec_write;  
    logic pc_write;

    
    
    //signals for the branch hazard module
    logic fetch2dec_flush;
    logic dec2exec_flush;
    logic exec2mem_flush;
    logic mem2wb_flush;
    
   //instantiating that module
   Hazard_Module hazard_mod (
    .branch_taken(branch_taken),
    .fetch2dec_flush(fetch2dec_flush),
    .dec2exec_flush(dec2exec_flush),
    .exec2mem_flush(exec2mem_flush),
    .mem2wb_flush(mem2wb_flush)
); 

// Instantiate Hazard Detector and Staller
  HazardDetectorAndStaller hazard_detector (
    .MEMREAD(dec2exec.mem_rden2),
    .dec2exec_rd(dec2exec.ir[11:7]),
    .fetch2dec_rs1(ir[19:15]),
    .fetch2dec_rs2(ir[24:20]),
    .HazardDetected(HazardDetected)
);
    
//    // Signals for forwarding
    logic [1:0] forwardA;
    logic [1:0] forwardB;

    // Instantiate forwarding unit
    ForwardingUnit forwarding_unit (
        .dec2exec_rs1(dec2exec.ir[19:15]),
        .dec2exec_rs2(dec2exec.ir[24:20]),
        .mem_rd(exec2mem.ir[11:7]),
        .mem_reg_write(exec2mem.reg_wr),
        .wb_rd(mem2wb.ir[11:7]),
        .wb_reg_write(mem2wb.reg_wr),
        .FORWARDA(forwardA),
        .FORWARDB(forwardB)
    );     
     
   
 //Stalling Logic:
 logic loadUseFlush;
 
 // Control signals for stalling
always_comb begin
    // Default values
    pc_write = 1'b1; //pcc should be written to
    fetch2dec_write = 1'b1; //register should be written to regularly 
    loadUseFlush = 1'b0; //struct should not be flushed

    // Use HazardDetected signal
    if (HazardDetected) begin
        pc_write = 1'b0;      // Stall the PC by not writing to it
        fetch2dec_write = 1'b0;   // Stall the reguster by not writing to it
        loadUseFlush = 1'b1; // add the bubble by flushing the register 
    end
end
   
   
   
    
//    wire [6:0] opcode;
//    wire [31:0] pc;
//    wire [31:0] pc_value;
//    wire [31:0] next_pc;
//    wire [31:0] B;
//    wire [31:0] I_immed;
//    wire [31:0] S_immed;
//    wire [31:0] U_immed;
//    wire [31:0] aluBin;
//    wire [31:0] aluAin;
//    wire [31:0] aluResult;
//    wire [31:0] rfIn;
//    wire [31:0] csr_reg; 
//    wire [31:0] mem_data;
//    wire [31:0] IR;
//    wire memRead1;
//    wire memRead2;
//    wire pcWrite;
//    wire regWrite;
//    wire memWrite; 
//    wire op1_sel;
//    wire mem_op;
//    wire IorD;
//    wire pcWriteCond;
//    wire memRead;
//    wire [1:0] opB_sel;
//    wire [1:0] rf_sel;
//    wire [1:0] wb_sel;
//    wire [1:0] mSize;
//    logic [1:0] pc_sel;
//    wire [3:0] alu_fun;
//    wire opA_sel;
    
    
    logic br_lt,br_eq,br_ltu;
    instr_t fetch2dec, dec2exec, exec2mem, mem2wb;
              
//==== Instruction Fetch ===========================================
//Create logic for PC
//    logic [3:0] pc_source;
    logic [31:0] pc_out, pc_out_inc;
    
 //Creating logic for instruction memory
    logic [31:0] ir;
//    logic [13:0] addr1;
    logic mem_rden1;

//assigning my pc values 
//    assign pc_write = 1'b1; 	//Hardwired high, assuming now hazards
     
//assigning my instruction memory values
    assign mem_rden1 = 1'b1; 	//Fetch new instruction every cycle
//    assign addr1 = pc_out[15:2]; //the address to get the instruction is the pc_out
    
//Instantiate the PC and connect relevant I/O
    PC OTTER_PC(
    .CLK(CLK), 
    .RST(RESET), 
    .PC_WRITE(pc_write), 
    .PC_SOURCE(pc_source),
    //.PC_SOURCE(3'b000),
    .JALR(jalrWire), 
    .JAL(jalWire), 
    .BRANCH(branchWire), 
    .MTVEC(32'b0), //just set to all zero
    .MEPC(32'b0), 
    .PC_OUT(pc_out), 
    .PC_OUT_INC(pc_out_inc)); 
    
 //Must instantiate my fetch2dec struct   


always_ff @(posedge CLK) begin
     if (RESET)
     begin
        fetch2dec <= 0;  
     end
     
     else if (fetch2dec_flush) 
     begin
     fetch2dec <= 0; //flush this stage
     end
     
     else if (fetch2dec_write)
     begin
        fetch2dec.pc_out <= pc_out;
        fetch2dec.pc_out_inc <= pc_out_inc;  
     end   
end
     
////==== Instruction Decode ===========================================

////Assigning for my Regfile inputs
//    assign reg_adr1 = fetch2dec.ir[19:15]; 
//    assign reg_adr2 = fetch2dec.ir[24:20]; 
//    assign reg_wa = fetch2dec.ir[11:7];
//    //assign wd = wb_out; //will come from wrapper

//Create logic for Decoder outputs (things going into the structs)
    logic alu_src_a;
    logic [1:0] alu_src_b;
    logic [1:0] wb_sel;
    logic [3:0] alu_fun;
    logic mem_we2;
    logic mem_rden2;
    logic branchStr;
    logic jalStr;
    logic jalrStr;
    logic reg_wr;
//    logic [1:0] forwardA;
//    logic [1:0] forwardB;
    
    
//Assigning things for my decoder inputs

      
//Create logic for Regfile inputs
    logic [4:0] reg_wa;
    logic [31:0] rs1, rs2;
    
//Assigning stuff for regfile inputs
    assign reg_adr1 = ir[19:15];
    assign reg_adr2 = ir[24:20];
    assign reg_wa = ir[11:7];
//the wd will be a wire coming from writeback     

//Create logic for immedGen outputs
    logic [31:0] Utype, Itype, Stype, Btype, Jtype;

//Assigning for the immedGen inputs
    assign imgen_ir = ir[31:7];
      
// Instantiating my Reg File, Immediate Generator, and Decoder
 REG_FILE OTTER_REG_FILE(
   .CLK(CLK), 
   .EN(mem2wb.reg_wr), //will come from writeback
   .ADR1(ir[19:15]), 
   .ADR2(ir[24:20]), 
   .WA(mem2wb.ir[11:7]), //will come from writeback
   .WD(wb_out), //will come from writeback
   .RS1(rs1), 
   .RS2(rs2));
    
ImmediateGenerator OTTER_IMGEN(
   .IR(ir[31:7]), //it gets its input from the memory file (from the instruction mem)
   .U_TYPE(Utype), 
   .I_TYPE(Itype), 
   .S_TYPE(Stype),
   .B_TYPE(Btype), 
   .J_TYPE(Jtype));   
   
CU_DCDR OTTER_DCDR(
   .IR_30(ir[30]), 
   .IR_OPCODE(ir[6:0]), 
   .IR_FUNCT(ir[14:12]), 
   .BRANCHsig(branchStr), 
   .JALsig(jalStr),
   .JALRsig(jalrStr), 
   .ALU_FUN(alu_fun), 
   .ALU_SRCA(alu_src_a), 
   .ALU_SRCB(alu_src_b), 
   .RF_WR_SEL(wb_sel),
   .REG_WRITE(reg_wr), 
   .MEM_WE2(mem_we2), 
   .MEM_RDEN2(mem_rden2));
      //do i put this reset here or what 
      
      
    always_ff@(posedge CLK) begin
    if (RESET)
    begin
        dec2exec <= 0;  
    end
    
    else if (dec2exec_flush || loadUseFlush) 
     begin
     dec2exec <= 0; //flush this stage
     end
     
    else
    begin
    dec2exec <= fetch2dec;
 //defining my dec2exec struct
    dec2exec.rs1 <= rs1;
    dec2exec.rs2 <= rs2;;
    dec2exec.Utype <= Utype;
    dec2exec.Itype <= Itype;
    dec2exec.Stype <= Stype;
    dec2exec.Btype <= Btype;
    dec2exec.Jtype <= Jtype;
    dec2exec.alu_src_a <= alu_src_a;
    dec2exec.alu_src_b <= alu_src_b;
    dec2exec.wb_sel <= wb_sel;
    dec2exec.alu_fun <= alu_fun;
    dec2exec.reg_wr <= reg_wr;
    dec2exec.mem_we2 <= mem_we2;
    dec2exec.mem_rden2 <= mem_rden2;
    dec2exec.branchStr <= branchStr;
    dec2exec.jalStr <= jalStr;
    dec2exec.jalrStr <= jalrStr;
    dec2exec.ir <= ir;
   end
   end
     
    

//   //logic to check if rs1 is used for certain instructions
//   //do the same for rs2 and rv?
////    assign decode.rs1_used = decode.rs1 != 0
////                             && decode.opcode != LUI
////                             && decode.opcode != AUIPC
////                             && de_inst.opcode != JAL; 	
	
////==== Execute ======================================================
//Create logic for ALU
    logic [31:0] alu_result;


//creating logic for forwarding muxes
    logic [31:0] ForwardMuxA_out;
    logic [31:0] ForwardMuxB_out;
      
    
//mux for before the ALU MUX A
    always_comb 
    begin
        case (forwardA) 
            2'b00: ForwardMuxA_out = dec2exec.rs1;
            2'b10: ForwardMuxA_out = exec2mem.alu_result;
            2'b01: ForwardMuxA_out = wb_out;
            2'b11: ForwardMuxA_out = 32'b0;
        endcase
    end  
    
    
//mux for before the ALU MUX B
    always_comb 
    begin
        case (forwardB) 
            2'b00: ForwardMuxB_out = dec2exec.rs2;
            2'b10: ForwardMuxB_out = exec2mem.alu_result;
            2'b01: ForwardMuxB_out = wb_out;
            2'b11: ForwardMuxB_out = 32'b0;
        endcase
    end   
    
    
//Instantiate ALUsrcA MUX    
    TwoMux OTTER_ALU_MUXA(
    .ALU_SRC_A(dec2exec.alu_src_a), 
    .RS1(ForwardMuxA_out), 
    .U_TYPE(dec2exec.Utype), 
    .SRC_A(srcA));
    
//Instatiate ALUsrcB MUX
    FourMux OTTER_ALU_MUXB(
    .SEL(dec2exec.alu_src_b), 
    .ZERO(ForwardMuxB_out), 
    .ONE(dec2exec.Itype), 
    .TWO(dec2exec.Stype), 
    .THREE(dec2exec.pc_out), 
    .OUT(srcB));
    
//Instantiate the ALU itself
    ALU OTTER_ALU(
    .SRC_A(srcA), 
    .SRC_B(srcB), 
    .ALU_FUN(dec2exec.alu_fun), 
    .RESULT(alu_result));
        
//Instantiate Branch Address Generator, connect all relevant I/O    
    BAG OTTER_BAG(
    .RS1(dec2exec.rs1), 
    .I_TYPE(dec2exec.Itype), 
    .J_TYPE(dec2exec.Jtype), 
    .B_TYPE(dec2exec.Btype), 
    .FROM_PC(dec2exec.pc_out),
    .JAL(jalWire), //connected to PC
    .JALR(jalrWire), //connected to PC
    .BRANCH(branchWire)); //connected to PC
         
// Instantiate Branch Condition Generator, connect all relevant I/O
BCG OTTER_BCG(
    .RS1(dec2exec.rs1),          // Connect RS1 from decoder to execution unit
    .RS2(dec2exec.rs2),          // Connect RS2 from decoder to execution unit
    .func3(dec2exec.ir[14:12]),      // func3 extracted from instruction, assumes part of decoder to execution unit data
    .ir(dec2exec.ir),            // Instruction register value, assumes it's part of decoder to execution unit data
    .jal(dec2exec.jalStr),          // JAL control signal from decoder
    .jalr(dec2exec.jalrStr),        // JALR control signal from decoder
    .branch(dec2exec.branchStr),    // Branch control signal from decoder
    .branch_taken(branch_taken),
    .pc_source(pc_source)        // Output connected to select signal of the PC MUX
);
   
   
   always_ff@(posedge CLK) begin
    if (RESET)
    begin 
        exec2mem <= 0; 
    end
    
    else if (exec2mem_flush) 
    begin
        exec2mem <= 0; // Flush pipeline stage
    end
      
    else 
    begin
        exec2mem <= dec2exec; //everything in this struct gets passed over.
        exec2mem.alu_result <= alu_result;
    end
        
        //if you dont have things specifically filled out, it will just get rid of them and not pass them on
    end
    
//    //defining my exec2mem struct
//    always_ff@(posedge CLK) begin
//    if (RESET)
//        mem2wb <= 0;  
//    else
//        exec2mem.alu_result <= alu_result;
//    end
    
//    always_ff@(posedge CLK) begin
//    if (RESET)
//        mem2wb <= 0;  
//    else 
//        mem2wb <= exec2mem; 
//    end
    
////     logic [31:0] ex_mem_rs2;
////     logic ex_mem_aluRes = 0;
////     instr_t ex_mem_inst;
////     logic [31:0] opA_forwarded;
////     logic [31:0] opB_forwarded;

////==== Memory ======================================================
     
    //Create logic for Memory module    
    logic [13:0] addr1;
    assign addr1 = pc_out[15:2];
//    logic mem_rden2, mem_we2;
//    logic [31:0] dout2;
//    logic sign;
//    assign sign = ir[14];
//    logic [1:0] size;
//    assign size = ir[13:12];     

     logic [31:0] dout2;
     
     logic sign;
     logic [1:0] size;
     
     assign sign = dec2exec.ir[14];
     assign size = dec2exec.ir[13:12];
     
    //Instantiate the Memory module and connect relevant I/O    
    Memory OTTER_MEMORY(
    .MEM_CLK(CLK), 
    .MEM_RDEN1(1'b1), 
    .MEM_RDEN2(dec2exec.mem_rden2), 
    .MEM_WE2(dec2exec.mem_we2), 
    .MEM_ADDR1(addr1), 
    .MEM_ADDR2(alu_result), //don't pass it from the register 
    .MEM_DIN2(dec2exec.rs2), 
    .MEM_SIZE(exec2mem.ir[13:12]), //im doing all of this wrong i think
    .MEM_SIGN(exec2mem.ir[14]), //make this in my struct?
    .IO_IN(IOBUS_IN), 
    .IO_WR(IOBUS_WR), 
    .MEM_DOUT1(ir), 
    .MEM_DOUT2(dout2));
   
    always_ff@(posedge CLK) begin
    if (RESET)
    begin
        mem2wb <= 0;  
    end 
    
    else if (mem2wb_flush) begin
        mem2wb <= 0; 
    end
    
    else
    begin
        mem2wb <= exec2mem; //i have run out of structs?? how is this even possible?
        mem2wb.dout2 <= dout2;
    end
    end
    
////    assign IOBUS_ADDR = ex_mem_aluRes;
////    assign IOBUS_OUT = ex_mem_rs2;
     
////==== Write Back ==================================================
     
 //Instantiate RegFile Mux, connect all relevant I/O
    
    always_comb 
    begin
        case (mem2wb.wb_sel) 
            2'b00: wb_out = mem2wb.pc_out_inc;
            2'b01: wb_out = mem2wb.alu_result;
            2'b10: wb_out = mem2wb.dout2;
            2'b11: wb_out = 32'b0;
        endcase
    end 
    
    
            
endmodule
