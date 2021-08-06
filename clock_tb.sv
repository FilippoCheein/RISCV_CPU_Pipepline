`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Jake Park
// 
// Create Date: 07/08/2020 03:20:31 PM
// Design Name: 
// Module Name: clock_tb
// Project Name: Clock Testbench for OTTER Pipeline
// Target Devices: 
// Tool Versions: 
// Description: A simple testbench to set up a clock so that you don't have to "force clock" every time you start a simulation
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created 7/8/2020
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

//The sole purpose of this file is to establish a clock for simulations

module clock_tb();

    // 100MHz clock
    logic clk = 0;
    initial begin
        #10 clk = 1;
        forever #5 clk = ~clk;
    end
    
    //if your clock signal in OTTER_Wrapper is named something other than "CLK", change it here
    OTTER_Wrapper OTTER_Wrapper(.CLK(clk));
endmodule
