`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Filippo Cheein
// 
// Create Date: 02/17/2021 01:26:09 PM
// Design Name: 
// Module Name: hd
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


module HazardDetection ( DE_EX_MemRead, DE_EX_regRead, IF_DE_regRs1, IF_DE_regRs2,
                         RS1_USED, RS2_USED, 
                         STALL
                        );

input logic DE_EX_MemRead;
input logic RS1_USED, RS2_USED;
input logic [4:0] DE_EX_regRead;
input logic [4:0] IF_DE_regRs1, IF_DE_regRs2;
output logic STALL;

always_comb
begin
   // load stall - Add rs1 used
   if(DE_EX_MemRead 
     && ( ((DE_EX_regRead == IF_DE_regRs1) && RS1_USED)  
          || ( (DE_EX_regRead == IF_DE_regRs2) && RS2_USED) 
        ) 
      )
     STALL = 1;
   else
     STALL = 0;
 
end
endmodule
