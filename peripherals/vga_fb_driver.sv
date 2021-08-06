`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly
// Engineer: Paul Hummel
//
// Create Date: 06/07/2018 06:00:59 PM
// Design Name:
// Module Name: VGA_FB_DRIVER
// Project Name: VGA 40x30
// Target Devices: RAT MCU on Basys3
// Description: VGA framebuffer interface driver for the the RAT MCU. Creates
//              2k x 8 framebuffer, control input interfaces (WA, WD, WE, RD),
//              and VGA output signals (ROUT, GOUT, BOUT, HS, VS).
//
// Revision:
// Revision 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////


module vga_fb_driver(
    input CLK,            // 50 MHz
    input [10:0] WA,
    input [7:0] WD,
    input WE,
    output [7:0] RD,
    output [2:0] ROUT,
    output [2:0] GOUT,
    output [1:0] BOUT,
    output HS,
    output VS);
    
    logic clk_25 = 1'b0;
    logic [7:0] s_fb_rd;
    logic [10:0] s_fb_ra;
    logic [2:0] s_vga_red, s_vga_green;
    logic [1:0] s_vga_blue;
    logic [9:0] s_vga_row, s_vga_col;
    
    
    // divide by 2 clock divider to create 25 MHz clock
    always_ff @(posedge CLK)
      clk_25 <= ~clk_25;

    // VGA output
    vga_driver vga_out(.CLK(clk_25), .RED(s_vga_red), .GREEN(s_vga_green),
                      .BLUE(s_vga_blue), .ROW(s_vga_row), .COLUMN(s_vga_col),
                      .ROUT(ROUT), .GOUT(GOUT), .BOUT(BOUT),
                      .HSYNC(HS), .VSYNC(VS));
    
    // Framebuffer
    ram2k_8 framebuffer(.CLK(CLK), .WE(WE), .RA2(s_fb_ra), .WA1(WA), .WD(WD),
                        .RD2(s_fb_rd), .RD1(RD));
    
    // combine row and column from the vga_driver to create read address RA2
    // for the framebuffer which returns color data as RD2
    assign s_fb_ra = {s_vga_row[8:4],s_vga_col[9:4]};
    
    // divide the color data from the framebuffer into individual RGB values
    // for the vga_driver
    assign s_vga_red = s_fb_rd[7:5];
    assign s_vga_green = s_fb_rd[4:2];
    assign s_vga_blue = s_fb_rd[1:0];

endmodule