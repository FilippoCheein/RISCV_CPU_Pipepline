`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Filippo Cheein
// 
// Create Date: 02/19/2021 03:56:55 PM
// Design Name: 
// Module Name: ForwardingUnit
// Project Name: Pipeline - Control Hazard 
//
// input: ex_mem_inst.RegisterRd
//        ex_mem_inst.regWrite
//        mem_wb_inst.registerRd
//        mem_wb_inst.regWrite
//        de_ex_inst.rs1
//        de_ex_inst.rs2
//
// output: ALU_sourceA
//         ALU_sourceB
//
//  Instantiation:
//  ForwardingUnit( .EX_MEM_regRead(), .EX_MEM_regWrite(),  .MEM_WB_regRead(), .MEM_WB_regWrite(), .DE_EX_regRs1(), .DE_EX_regRs2(), .forwardA(), .forwardB() );
//
//
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
        input logic [4:0] EX_MEM_regRead, 
        input logic EX_MEM_regWrite, 
        input logic [4:0] MEM_WB_regRead,
        input logic MEM_WB_regWrite,
        input logic [4:0] DE_EX_regRs1,
        input logic [4:0] DE_EX_regRs2,
        
        input logic RS1_USED,
        input logic RS2_USED,
        
        output logic [1:0] forwardA,
        output logic [1:0] forwardB 
    );
    
    // add rs1 used
  always_comb
  begin  
    // EX Hazard
    if ( EX_MEM_regWrite
         && (EX_MEM_regRead != 0)
         && (EX_MEM_regRead == DE_EX_regRs1)
         && RS1_USED
       )
       forwardA = 2;
    // MEM Hazard
    else if (  MEM_WB_regWrite 
               && (MEM_WB_regRead != 0) 
               && !(EX_MEM_regWrite && (EX_MEM_regRead != 0) && (EX_MEM_regRead == DE_EX_regRs1) )
               && (MEM_WB_regRead == DE_EX_regRs1) 
               && RS1_USED
             )
       forwardA = 1;
    // no hazard
    else
       forwardA = 0;
      
    // EX Hazard
    if ( EX_MEM_regWrite
        && (EX_MEM_regRead != 0)
        && (EX_MEM_regRead == DE_EX_regRs2) 
        && RS2_USED
       )
       forwardB = 2;
    // MEM Hazard
    else if (  MEM_WB_regWrite 
               && (MEM_WB_regRead != 0) 
               && !(EX_MEM_regWrite && (EX_MEM_regRead != 0) && (EX_MEM_regRead == DE_EX_regRs2) ) 
               && (MEM_WB_regRead == DE_EX_regRs2) 
               && RS2_USED 
            )
       forwardB = 1;
    // no hazard
    else
       forwardB = 0;
    
  end
    
   
endmodule
