`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  Filippo Cheein
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
        
typedef struct packed{
    opcode_t opcode;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead1;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc;
    logic [31:0] next_pc;
    logic pcWrite;
    logic [3:0]  pc_sel;
    logic [31:0] rs1;
    logic [31:0] rs2;
    logic [31:0] aluRes;
    logic [31:0] IR;
    logic [31:0] I_immed;
    logic stall;
    logic flush;
} instr_t;

module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           
    wire [6:0] opcode;
    wire [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc,A,B,
        I_immed,S_immed,U_immed,aluBin,aluAin,aluResult,rfIn,csr_reg, mem_data;
    
    wire [31:0] IR;
    wire memRead1,memRead2;
    
    wire pcWrite,regWrite,memWrite, op1_sel,mem_op,IorD,pcWriteCond,memRead;
    wire [1:0] opB_sel, rf_wr_sel, wb_sel, mSize;
    logic [3:0] pc_sel, ex_pc_sel, ex_haz_pc_sel; 
    wire [3:0]alu_fun;
    wire opA_sel;
    
    wire mepcWrite, csrWrite,intCLR, mie, intTaken;
    wire [31:0] mepc, mtvec;
    
    logic br_lt, br_eq, br_ltu;
    logic [31:0] ex_jalr_pc, ex_branch_pc, ex_jump_pc, ex_next_pc;
    logic  [31:0] wb_rfIn;
    logic wb_regWrite;
    logic [4:0] wb_rd_addr;
    logic stall;
    logic flush;
    
       // Control for farwarding 
     logic haz_regWrite;
     logic haz_memWrite;
     logic haz_memRead2;
     logic [3:0] haz_alu_fun;
     logic [1:0] haz_rf_wr_sel;
     logic haz_pcWrite;
     logic haz_memRead1;
    
//==== Instruction Fetch ===========================================
     instr_t if_inst, if_de_inst;
    // assign pcWrite = 1'b1; 	//Hardwired high, assuming now hazards
    // assign memRead1 = 1'b1; 	//Fetch new instruction every cycle
     
     assign next_pc = pc + 4;
     assign if_de_inst.stall = stall;
     
      assign if_inst.pc_sel = pc_sel;
      assign if_inst.pc = pc;
      assign if_inst.next_pc = next_pc;
     
          // Branch, JAL  JALR flush next 2 instruction
 /*    always_comb
     begin
     if (( if_inst.pc_sel != 0 || de_inst.pc_sel != 0) )
       flush = 1;
     else
       flush = 0;
     end
   */  
     always_ff @(posedge CLK) begin
                // write 
               if_de_inst.pc <= if_inst.pc;
               if_de_inst.next_pc <= if_inst.next_pc;
               if_de_inst.pc_sel <= if_inst.pc_sel;
     end
     
    // Creates a 2-to-1 multiplexor used to select the source of the next PC
     Mult6to1 PCdatasrc (next_pc, ex_jalr_pc, ex_branch_pc, ex_jump_pc, mtvec, mepc, ex_haz_pc_sel, pc_value);    

    ProgCount PC (.PC_CLK(CLK), .PC_RST(RESET),
                  .PC_LD(de_ex_inst.pcWrite),
                  //.PC_LD(haz_pcWrite),
                  .PC_DIN(pc_value), .PC_COUNT(pc)); 
    
                  
//==== Instruction Decode ===========================================
    instr_t de_ex_inst, de_inst;
    instr_t ex_mem_inst, ex_inst;
    instr_t mem_inst, mem_wb_inst;
   
    // output
    logic [31:0] de_ex_opA, de_ex_opB;
    logic [31:0] de_ex_I_immed;
    
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(IR[6:0]);
    
    // Read IF/ID pipeline register
    assign de_inst.pc = if_de_inst.pc;
    assign de_inst.next_pc = if_de_inst.next_pc;
    assign de_inst.pc_sel = if_de_inst.pc_sel;
    
    // write IR and opcode to Decode local register
    assign de_inst.IR = IR;
    assign de_inst.rs1_addr = IR[19:15];
    assign de_inst.rs2_addr = IR[24:20];
    assign de_inst.rd_addr  = IR[11:7];
    assign de_inst.opcode   = OPCODE;
   
     assign de_inst.rs1 = A;
     assign de_inst.rs2 = B;
     
    
    assign de_inst.rs1_used = ~(opA_sel);
    assign de_inst.rs2_used = opB_sel==0 ? 1'b1:1'b0; 
     
      // Branch, JAL  JALR flush next 2 instruction
  //    assign flush = ( if_inst.pc_sel != 0 || de_inst.pc_sel != 0);
    always_comb
       begin
      // if (( if_inst.pc_sel != 0 || de_inst.pc_sel != 0) )
       if(ex_haz_pc_sel != 0)
         flush = 1;
       else
         flush = 0;
       end
    
    always_comb 
    begin
        if (stall) 
        begin
            haz_regWrite = 0;
            haz_memWrite = 0;
            haz_memRead2 = 0;
            haz_alu_fun = 0;
            haz_rf_wr_sel = 0;
            
        //    ex_haz_pc_sel = 0; 
            
            haz_pcWrite = 0;
            haz_memRead1 = 0;
          
        end 
        
        else if (flush)
        begin
            haz_regWrite = 0;
            haz_memWrite = 0;
            haz_memRead2 = 0;
            haz_alu_fun = 0;
            haz_rf_wr_sel = 0;
          
       //     ex_haz_pc_sel = 0;
            
            haz_pcWrite = pcWrite;
            haz_memRead1 = memRead1;
        end
        
        else 
        begin
            haz_regWrite = regWrite;
            haz_memWrite = memWrite;
            haz_memRead2 = memRead2;
            haz_alu_fun = alu_fun;
            haz_rf_wr_sel = rf_wr_sel;
          
       //     ex_haz_pc_sel = ex_pc_sel; 
            
            haz_pcWrite = pcWrite;
            haz_memRead1 = memRead1;
            
        end
    end

    always_ff @(posedge CLK)
    begin
        // Write local register in ID/EX pipeline register 
        de_ex_inst    <= de_inst;
        de_ex_inst.pcWrite <= haz_pcWrite;
        de_ex_inst.memRead1 <= haz_memRead1;
        
        // write to logic to assign in Execute stage
        de_ex_opA <= aluAin;
        de_ex_opB <= aluBin;
        
        // Write Immediate to pipeline register
        de_ex_inst.I_immed <= I_immed;
        
        // Write CU Outputs
        de_ex_inst.regWrite  <= haz_regWrite;
        de_ex_inst.memWrite  <= haz_memWrite;
        de_ex_inst.memRead2  <= haz_memRead2;
        de_ex_inst.alu_fun   <= haz_alu_fun;
        de_ex_inst.rf_wr_sel <= haz_rf_wr_sel;
 
    end

	//  Generate immediates - I_immediate is then stored in Pipeline Register
     assign S_immed = {{20{de_inst.IR[31]}},de_inst.IR[31:25],de_inst.IR[11:7]};
     assign I_immed = {{20{de_inst.IR[31]}},de_inst.IR[31:20]};
     assign U_immed = {de_inst.IR[31:12],{12{1'b0}}};
     
     // Hazard Detection Unit -    
     HazardDetection HazardDetection_Unit( .DE_EX_MemRead(ex_inst.memRead2), .DE_EX_regRead(ex_inst.rd_addr), 
                                             .IF_DE_regRs1(de_inst.rs1_addr), .IF_DE_regRs2(de_inst.rs2_addr), 
                                             .RS1_USED(de_inst.rs1_used), .RS2_USED(de_inst.rs2_used),
                                             .STALL(stall));
     
     // Updated Decoder for Pipelining use
     OTTER_CU_DECODER_PIPE CU_DECODER_PIPE(.CU_OPCODE(de_inst.opcode), .CU_FUNC3(de_inst.IR[14:12]),.CU_FUNC7(de_inst.IR[31:25]), 
                 .CU_BR_EQ(),.CU_BR_LT(),.CU_BR_LTU(),.CU_PCSOURCE(),
                 .CU_ALU_SRCA(opA_sel),.CU_ALU_SRCB(opB_sel),.CU_ALU_FUN(alu_fun),.CU_RF_WR_SEL(rf_wr_sel),.intTaken(intTaken),
                 .CU_PCWRITE(pcWrite), .CU_REGWRITE(regWrite), .CU_MEMWRITE(memWrite), .CU_MEMREAD1(memRead1), .CU_MEMREAD2(memRead2));
     
     logic prev_INT=0;
     
     // Register 
     OTTER_registerFile RF (de_inst.rs1_addr, de_inst.rs2_addr, wb_rd_addr, wb_rfIn, wb_regWrite, A, B, CLK); // Register file

     // ALU MUXes
     Mult2to1 ALUAinput (A, U_immed, opA_sel, aluAin);
     Mult4to1 ALUBinput (B, I_immed, S_immed, de_inst.pc, opB_sel, aluBin);
    	
//==== Execute ======================================================
     // Execute stage variables
     logic [31:0] ex_opA_forwarded;
     logic [31:0] ex_opB_forwarded;
     logic [1:0] ex_forwardA;
     logic [1:0] ex_forwardB;
     logic [31:0] ex_opA, ex_opB;
     logic [31:0] ex_I_immed; 
     
     // Variable stored in EX/MM regeister
     logic [31:0] ex_mem_aluRes = 0;
     logic [31:0] ex_mem_opB_forwarded;
     
     //  memory stage variables
     logic [31:0] mem_aluRes;
     
     // read from DE/EX register 
     assign ex_inst = de_ex_inst;
     
   //  assign ex_pc_sel = pc_sel;
     assign ex_I_immed = de_ex_inst.I_immed;
     assign ex_opA = de_ex_opA;
     assign ex_opB = de_ex_opB;
     
     always_comb
     begin
         if (stall) 
             ex_haz_pc_sel = 0;
         else if (ex_mem_inst.flush || mem_wb_inst.flush)
            ex_haz_pc_sel = 0;
         else 
             ex_haz_pc_sel = pc_sel; 
     end
     
     always_ff @(posedge CLK)
     begin       
        ex_mem_inst <= ex_inst;
        ex_mem_inst.flush <= flush;
        ex_mem_inst.aluRes <= aluResult;
        ex_mem_opB_forwarded <= ex_opB_forwarded;
     end
     
    // Target Gen 
    //pc target calculations 
    assign ex_jalr_pc = ex_I_immed + ex_inst.rs1;
    assign ex_branch_pc = ex_inst.pc + {{20{ex_inst.IR[31]}},ex_inst.IR[7],ex_inst.IR[30:25],ex_inst.IR[11:8],1'b0};   //byte aligned addresses
    assign ex_jump_pc = ex_inst.pc + {{12{ex_inst.IR[31]}}, ex_inst.IR[19:12], ex_inst.IR[20], ex_inst.IR[30:21],1'b0};
    assign int_pc = 0;
    
    BranchCondGen Branch_Generator( .A(ex_opA_forwarded), .B(ex_opB_forwarded), .BR_OPCODE(ex_inst.opcode), 
                                               .FUNC3(ex_inst.IR[14:12]), .intTaken(),
                                               // .FLUSH(flush),
                                                .pcSource(pc_sel) );

    // Forwarding unit - 
    ForwardingUnit FU(  .EX_MEM_regRead(ex_mem_inst.rd_addr), .EX_MEM_regWrite(ex_mem_inst.regWrite),  
                        .MEM_WB_regRead(mem_wb_inst.rd_addr), .MEM_WB_regWrite(mem_wb_inst.regWrite), 
                        .DE_EX_regRs1(de_ex_inst.rs1_addr), .DE_EX_regRs2(de_ex_inst.rs2_addr), 
                        .RS1_USED(ex_inst.rs1_used), .RS2_USED(ex_inst.rs2_used),
                        .forwardA(ex_forwardA), .forwardB(ex_forwardB) );
    
     // ALU Farwarding MUXes
     Mult3to1 forwardA_ALU (.In1(ex_opA), .In2(wb_rfIn), .In3(mem_aluRes), .Sel(ex_forwardA), .Out(ex_opA_forwarded));
     Mult3to1 forwardB_ALU (.In1(ex_opB), .In2(wb_rfIn), .In3(mem_aluRes), .Sel(ex_forwardB), .Out(ex_opB_forwarded));
    // Mult2to1 EX_MUX (.In1(), .In2(), .Sel(), .Out());
     
    // Creates a RISC-V ALU
     OTTER_ALU ALU (ex_inst.alu_fun, ex_opA_forwarded, ex_opB_forwarded, aluResult);
     
     always_ff @ (posedge CLK)
     begin
          if(INTR && mie)
             prev_INT=1'b1;
          if(intCLR || RESET)
             prev_INT=1'b0;
     end

//==== Memory ======================================================
      // Memory Stage Varaibles
      logic [31:0] mem_rs2;
      logic [31:0] mem_wb_dout2;
      logic [31:0] mem_opB_forwarded;
      
      // Read Variables from Execute stage
      assign mem_inst = ex_mem_inst;
    //  substituted with mem_opB_forwarded
    //  assign mem_rs2 = mem_inst.rs2;
      assign mem_opB_forwarded = ex_mem_opB_forwarded;
      assign mem_aluRes = ex_mem_inst.aluRes;
      
     always_ff @(posedge CLK)
     begin
          
         mem_wb_dout2 <= mem_data;
         
         mem_wb_inst <= mem_inst;
     end
         
        assign IOBUS_ADDR = mem_inst.aluRes;
        assign IOBUS_OUT = mem_opB_forwarded;
        
     // Memory 1 & 2
     OTTER_mem_byte #(14) memory  (.MEM_CLK(CLK),.MEM_ADDR1(pc),.MEM_ADDR2(mem_aluRes),.MEM_DIN2(mem_opB_forwarded),
                                       //                                        de_ex_inst.memRead1
                                       .MEM_WRITE2(mem_inst.memWrite),.MEM_READ1(de_ex_inst.memRead1),.MEM_READ2(mem_inst.memRead2),
                                       .ERR(),.MEM_DOUT1(IR),.MEM_DOUT2(mem_data),.IO_IN(IOBUS_IN),
                                         .IO_WR(IOBUS_WR),.MEM_SIZE(mem_inst.IR[14:12]),.MEM_SIGN(mem_inst.IR[14]));
                                        
     
//==== Write Back ==================================================
     logic [31:0] wb_dout2;
     logic [31:0] wb_aluResult;
     logic [1:0] wb_rf_wr_sel;
     logic [31:0] wb_next_pc;
     
     assign wb_dout2 = mem_wb_dout2;
     assign wb_regWrite = mem_wb_inst.regWrite;
     assign wb_aluResult = mem_wb_inst.aluRes;
     assign wb_rf_wr_sel = mem_wb_inst.rf_wr_sel;
     assign wb_next_pc = mem_wb_inst.next_pc;
     assign wb_rd_addr = mem_wb_inst.rd_addr;
     assign wb_rfIn = rfIn;
         
    //Creates 4-to-1 multiplexor used to select reg write back data
    Mult4to1 regWriteback (wb_next_pc, csr_reg, wb_dout2 , wb_aluResult, wb_rf_wr_sel, rfIn);     
    
    
    //****do not touch****//
     //CSR registers and interrupt logic
         CSR CSRs(.clk(CLK),.rst(RESET),.intTaken(intTaken),.addr(IR[31:20]),.next_pc(pc),.wd(aluResult),.wr_en(csrWrite),
           .rd(csr_reg),.mepc(mepc),.mtvec(mtvec),.mie(mie));
    // ******************* //    
                
endmodule
