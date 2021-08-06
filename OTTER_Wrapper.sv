`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: J. Calllenes
//           P. Hummel
// 
// Create Date: 01/20/2019 10:36:50 AM
// Design Name: 
// Module Name: OTTER_Wrapper 
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

module OTTER_Wrapper(
   input CLK,
   input BTNL,
   input BTNC,
   input [15:0] SWITCHES,
   input PS2Clk,
   input PS2Data,
   output logic [15:0] LEDS,
   output [7:0] CATHODES,
   output [3:0] ANODES,
   output [7:0] VGA_RGB,
   output VGA_HS,
   output VGA_VS,
   output Tx
   );
       
    //   logic sclk =0;
    // INPUT PORT IDS ////////////////////////////////////////////////////////
    // Right now, the only possible inputs are the switches
    // In future labs you can add more MMIO, and you'll have
    // to add constants here for the mux below
    localparam SWITCHES_AD = 32'h11000000;
    localparam VGA_READ_AD = 32'h11040000;
           
    // OUTPUT PORT IDS ///////////////////////////////////////////////////////
    // In future labs you can add more MMIO
    localparam LEDS_AD      = 32'h11080000;
    localparam SSEG_AD     = 32'h110C0000;
    localparam VGA_ADDR_AD = 32'h11100000;
    localparam VGA_COLOR_AD = 32'h11140000;
    localparam UART_DATA_AD = 32'h11180000;
    localparam UART_RDY_AD = 32'h111C0000;  
    localparam KEYBOARD_AD = 32'h11200000;       
    
   // Signals for connecting OTTER_MCU to OTTER_wrapper /////////////////////////
   logic s_interrupt, keyboard_int,btn_int;
   logic s_reset,s_load;
   logic sclk= 1'b0;   
   
   // Signals for connecting VGA Framebuffer Driver
   logic r_vga_we;             // write enable
   logic [12:0] r_vga_wa;      // address of framebuffer to read and write
   logic [7:0] r_vga_wd;       // pixel color data to write to framebuffer
   logic [7:0] r_vga_rd;       // pixel color data read from framebuffer
 
   logic [15:0]  r_SSEG;// = 16'h0000;
   logic uart_start, uart_ready;
   logic [7:0] uart_data=0;
   logic [7:0] s_scancode;
     
   logic [31:0] IOBUS_out,IOBUS_in,IOBUS_addr;
   logic IOBUS_wr;
   
   assign s_interrupt = keyboard_int | btn_int;
   
    // Declare OTTER_CPU ///////////////////////////////////////////////////////
   OTTER_MCU MCU (.RESET(s_reset),.INTR(s_interrupt), .CLK(sclk), 
                   .IOBUS_OUT(IOBUS_out),.IOBUS_IN(IOBUS_in),.IOBUS_ADDR(IOBUS_addr),.IOBUS_WR(IOBUS_wr));

   // Declare Seven Segment Display /////////////////////////////////////////
   SevSegDisp SSG_DISP (.DATA_IN(r_SSEG), .CLK(CLK), .MODE(1'b0),
                       .CATHODES(CATHODES), .ANODES(ANODES));
   
   // Declare Debouncer One Shot  ///////////////////////////////////////////
   debounce_one_shot DB(.CLK(sclk), .BTN(BTNL), .DB_BTN(btn_int));
   
   
   //wire vgaCLK;
   //clk_wiz_1  pll(.clk_out1(vgaCLK), .reset(s_reset),.locked(),.clk_in1(CLK)); 

   // Declare VGA Frame Buffer //////////////////////////////////////////////
   vga_fb_driver VGA(.CLK(sclk), .WA(r_vga_wa), .WD(r_vga_wd),
                       .WE(r_vga_we), .RD(r_vga_rd), .ROUT(VGA_RGB[7:5]),
                       .GOUT(VGA_RGB[4:2]), .BOUT(VGA_RGB[1:0]),
                       .HS(VGA_HS), .VS(VGA_VS)); //.clk_vga(vgaCLK));
                                           
  // Declare UART Driver Module ////////////////////////////////////////////
   UART uart_driver(.CLK(CLK), .RST(s_reset),.start(uart_start),.data(uart_data),.Tx(Tx),.ready(uart_ready));
  
 // Declare Keyboard Driver //////////////////////////////////////////////
    KeyboardDriver KEYBD (.CLK(CLK), .PS2DATA(PS2Data), .PS2CLK(PS2Clk),
                          .INTRPT(keyboard_int), .SCANCODE(s_scancode)); 
                           
   // Clock Divider to create 50 MHz Clock /////////////////////////////////
   always_ff @(posedge CLK) begin
       sclk <= ~sclk;
   end
   
   //clk_div clkDIv(CLK, sclk);
  //assign sclk =CLK;
    // Connect Signals ////////////////////////////////////////////////////////////
   assign s_reset = BTNC;
   
   //assign LEDS[15]=keyboard_int;
   // Connect Board peripherals (Memory Mapped IO devices) to IOBUS /////////////////////////////////////////
    always_ff @ (posedge sclk)
    begin
        r_vga_we<=0;
        uart_start<=1'h0;
        if(IOBUS_wr)
            case(IOBUS_addr)
                LEDS_AD: LEDS <= IOBUS_out;    
                SSEG_AD: r_SSEG <= IOBUS_out[15:0];
                VGA_ADDR_AD: r_vga_wa <= IOBUS_out[16:0];  //[12:0];
                VGA_COLOR_AD: begin  r_vga_wd <= IOBUS_out[7:0];
                                     r_vga_we <= 1;  
                              end     
                UART_DATA_AD: begin
                                     uart_data  <= IOBUS_out[7:0];
                                     uart_start <= 1;
                              end
            endcase
            //if(keyboard_int)
            //r_SSEG <= {8'b0,s_scancode};
    end
   
    always_comb
    begin
        IOBUS_in=32'b0;
        case(IOBUS_addr)
            SWITCHES_AD: IOBUS_in[15:0] = SWITCHES;
            VGA_READ_AD: IOBUS_in[15:0] = r_vga_rd;
            UART_RDY_AD: IOBUS_in[0] = uart_ready;
            KEYBOARD_AD: IOBUS_in[7:0] = s_scancode;
            default: IOBUS_in=32'b0;
        endcase
    end
   endmodule

