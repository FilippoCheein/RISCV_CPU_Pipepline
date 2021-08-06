
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/17/2021 07:47:03 PM
// Design Name: 
// Module Name: CU_DECODER_PIPE
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


module OTTER_CU_DECODER_PIPE(
        input [6:0] CU_OPCODE,
        input [2:0] CU_FUNC3,
        input [6:0] CU_FUNC7,
        input CU_BR_EQ,
        input CU_BR_LT,
        input CU_BR_LTU,
        input intTaken,
        
        output logic CU_ALU_SRCA,
        output logic [1:0] CU_ALU_SRCB,
        output logic [3:0] CU_ALU_FUN,
        output logic [1:0] CU_RF_WR_SEL,   
        output logic [3:0] CU_PCSOURCE,
        
        // From CU_DECODER
        output logic CU_PCWRITE,
        output logic CU_REGWRITE,    
        output logic CU_MEMWRITE,
        output logic CU_MEMREAD1,
        output logic CU_MEMREAD2
       
    );
    
    
    // From CU_DECODER
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
     
   // opcode_t OPCODE;
   // assign OPCODE = opcode_t'(CU_OPCODE);     
     
    typedef enum logic [2:0] {
            Func3_CSRRW  = 3'b001,
            Func3_CSRRS  = 3'b010,
            Func3_CSRRC  = 3'b011,
            Func3_CSRRWI = 3'b101,
            Func3_CSRRSI = 3'b110,
            Func3_CSRRCI = 3'b111,
            Func3_PRIV   = 3'b000       //mret
    } funct3_system_t;

     
     logic brn_cond;
     //DECODING  (does not depend on state)  ////////////////////////////////////////////
    //SEPERATE DECODER
    // assign CU_ALU_FUN = (CU_OPCODE!=LUI)? (CU_OPCODE== )? {CU_FUNC7[5],CU_FUNC3}:4'b1001 ;
     always_comb
         case(CU_OPCODE)
         
             AUIPC: begin
                     CU_ALU_FUN = 4'b0;
                     CU_ALU_SRCB = 3;
                     CU_MEMREAD2 = 0;  
                     CU_MEMWRITE = 0;
                     CU_PCSOURCE=3'b000;
                     CU_REGWRITE = 1;  
                     CU_RF_WR_SEL = 3;
                    end
                    
             BRANCH: begin
                     CU_ALU_FUN = 4'b0;
                     CU_ALU_SRCB = 0;
                     CU_PCSOURCE=(brn_cond)?3'b010:2'b000;
                     CU_REGWRITE=0;
                     CU_MEMREAD2 = 0;  
                     CU_MEMWRITE = 0;  
                     CU_RF_WR_SEL = 3;
                     end
           
             OP_IMM: begin
                     CU_ALU_FUN= (CU_FUNC3==3'b101)?{CU_FUNC7[5],CU_FUNC3}:{1'b0,CU_FUNC3};
                     CU_ALU_SRCB = 1;
                     CU_MEMREAD2 = 0;  
                     CU_MEMWRITE = 0;  
                     CU_PCSOURCE=3'b000;
                     CU_REGWRITE = 1;  
                     CU_RF_WR_SEL = 3;
                     end
                    
             OP: begin
                  CU_ALU_FUN = {CU_FUNC7[5],CU_FUNC3};
                  CU_ALU_SRCB = 0;
                  CU_MEMREAD2 = 0;  
                  CU_MEMWRITE = 0; 
                  CU_PCSOURCE=3'b000; 
                  CU_REGWRITE = 1;  
                  CU_RF_WR_SEL = 3;
                 end
                 
             LUI: begin
                  CU_ALU_FUN = 4'b1001;
                  CU_ALU_SRCB = 1;
                  CU_MEMREAD2 = 0;  
                  CU_MEMWRITE = 0;  
                  CU_REGWRITE = 1;
                  CU_PCSOURCE=3'b000;
                  CU_RF_WR_SEL = 3;  
                  end
                  
             JAL: begin
                  CU_ALU_FUN = 4'b0;
                  CU_ALU_SRCB = 1;
                  CU_RF_WR_SEL=0;
                  CU_PCSOURCE =3'b011;
                  CU_MEMREAD2 = 0;  
                  CU_MEMWRITE = 0;  
                  CU_REGWRITE = 1;  
                  end
                  
             JALR: begin
                   CU_ALU_FUN = 4'b0;
                   CU_ALU_SRCB = 0;
                   CU_RF_WR_SEL = 0;
                   CU_PCSOURCE=3'b001;
                   CU_MEMREAD2 = 0;  
                   CU_MEMWRITE = 0; 
                   CU_REGWRITE = 1;  
                   end
                   
             LOAD: begin
                   CU_ALU_FUN = 4'b0;
                   CU_ALU_SRCB = 1;
                   CU_RF_WR_SEL = 2;
                   CU_MEMREAD2 = 1;
                   CU_MEMWRITE = 0;  
                   CU_PCSOURCE=3'b000;
                   CU_REGWRITE = 1;  
                   end
                   
             STORE: begin 
                    CU_ALU_SRCB = 2; 
                    CU_MEMWRITE = 1;
                    
                    CU_ALU_FUN = 4'b0;
                    CU_REGWRITE = 0;
                    CU_MEMREAD2 = 0;  
                    CU_PCSOURCE=3'b000;
                    CU_RF_WR_SEL = 3;
                    end
                    
             SYSTEM: begin
                     CU_ALU_FUN = 4'b1001;
                     CU_ALU_SRCB = 0;
                     CU_RF_WR_SEL=1;
                     CU_PCSOURCE = (CU_FUNC3==Func3_PRIV)? 3'b101:3'b000;
                     CU_MEMREAD2 = 0;  
                     CU_MEMWRITE = 0;  
                     CU_REGWRITE = 1;  
                     end

             default: begin
                      CU_ALU_FUN = 4'b0;
                      CU_RF_WR_SEL=3;
                      CU_ALU_SRCB=0; 
                      CU_PCSOURCE=3'b000; 
                      CU_MEMREAD2 = 0;  
                      CU_MEMWRITE = 0;  
                      CU_REGWRITE = 1;  
                      end
         endcase
       
       
         always_comb begin
         case(CU_FUNC3)
                     3'b000: brn_cond = CU_BR_EQ;     //BEQ 
                     3'b001: brn_cond = ~CU_BR_EQ;    //BNE
                     3'b100: brn_cond = CU_BR_LT;     //BLT
                     3'b101: brn_cond = ~CU_BR_LT;    //BGE
                     3'b110: brn_cond = CU_BR_LTU;    //BLTU
                     3'b111: brn_cond = ~CU_BR_LTU;   //BGEU
                     default: brn_cond =0;
         endcase
         end
    assign CU_ALU_SRCA = (CU_OPCODE==LUI || CU_OPCODE==AUIPC) ? 1 : 0;
    assign CU_MEMREAD1 = 1'b1;
    assign CU_PCWRITE = 1'b1;
    
endmodule

