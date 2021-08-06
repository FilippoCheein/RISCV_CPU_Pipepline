`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Filippo Cheein
// 
// Create Date: 02/22/2021 09:39:49 AM
// Design Name: 
// Module Name: BranchCondGen
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// inst
// BranchCondGen( .A(), .B(), .BR_OPCODE(), .FUNC3(), .intTaken(), .pcSource() );
// 
//////////////////////////////////////////////////////////////////////////////////


module BranchCondGen( 
     input logic [31:0] A,
     input logic [31:0] B,
     input [6:0] BR_OPCODE,
     input [2:0] FUNC3,
     input intTaken,
     
     output logic FLUSH,
     output logic [3:0] pcSource
    );
    
    logic br_lt, br_eq, br_ltu;
    logic brn_cond;
    logic previous_pc_sel;
     
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
     
    typedef enum logic [2:0] {
            Func3_CSRRW  = 3'b001,
            Func3_CSRRS  = 3'b010,
            Func3_CSRRC  = 3'b011,
            Func3_CSRRWI = 3'b101,
            Func3_CSRRSI = 3'b110,
            Func3_CSRRCI = 3'b111,
            Func3_PRIV   = 3'b000       //mret
    } funct3_system_t;
    
     // Branch Gen
     always_comb
     begin
         br_lt = 0; br_eq = 0; br_ltu = 0;
         if($signed(A) < $signed(B)) br_lt=1;
         if(A == B) br_eq=1;
         if(A < B) br_ltu=1;
     end
     
     always_comb begin
     case(FUNC3)
                 3'b000: brn_cond = br_eq;     //BEQ 
                 3'b001: brn_cond = ~br_eq;    //BNE
                 3'b100: brn_cond = br_lt;     //BLT
                 3'b101: brn_cond = ~br_lt;    //BGE
                 3'b110: brn_cond = br_ltu;    //BLTU
                 3'b111: brn_cond = ~br_ltu;   //BGEU
                 default: brn_cond = 0;
     endcase
     end
    
     always_comb begin
            case(BR_OPCODE)
                JAL: begin 
                     pcSource =3'b011;
                     //FLUSH = 1;
                     end
                JALR: begin
                      pcSource=3'b001;
                      //FLUSH = 1;
                      end
                BRANCH: begin
                        pcSource=(brn_cond)?3'b010:2'b000; 
                        end
                SYSTEM: pcSource = (FUNC3==Func3_PRIV)? 3'b101:3'b000;
                default: begin
                         pcSource=3'b000; 
                        // FLUSH = 0;
                         end
            endcase
            
            previous_pc_sel = pcSource;
            
           // if(previous_pc_sel != 0)
           // FLUSH = 1;
            
            if(intTaken)    
                pcSource=3'b100;   
    end
endmodule
