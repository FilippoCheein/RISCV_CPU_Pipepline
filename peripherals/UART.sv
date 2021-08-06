`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/03/2019 02:15:41 PM
// Design Name: 
// Module Name: UART
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


module UART(input CLK,
            input RST,
            input start,     //signal to trigger transmit
            input [7:0] data,   //data to transmit
            output logic Tx=0, //serial output.  held high during reset or when not transmitting
            output ready
    );
    logic [3:0] bitcounter;  //4 bit counter to count up to 10
    logic [13:0] counter;  //14 bit counter to count the baud rate, counter = clock/baud rate
    logic PS=0,NS;
    //10 bits of data sent during transmission
    // the least significant bit is 0 (start bit) and msb is 1 (Stop bit)
    logic [9:0] rightshiftreg;
    logic shift;    //control signal for bit shifting in UART
    logic load; //control signal for loading the shift reg and add start/stop bits
    logic clearstart;  //clear signal to start/reset the bitcounter for UART transmission
    
    logic start_buffer=0,clear=0;
    always_ff@ (posedge CLK)
    begin
        if(start) start_buffer<=1;
        if(clearstart) start_buffer<=0;
    end
    assign ready = ~PS;
    //UART transmission logic
    always_ff @ (posedge CLK)
    begin
        clearstart<=0;
        if (RST) begin
            PS <= 0;
            counter <=0;
            bitcounter <=0;
        end else begin
            counter <= counter +1;
            if (counter >= 10415) begin  //9600 baudrate =100MHz/10416
                PS <= NS;
                clearstart <=1;
                counter <=0;
                if (load) rightshiftreg <= {1'b1, data, 1'b0};
                if (clear) bitcounter <=0;
                if (shift) begin rightshiftreg <= rightshiftreg >>1;
                                bitcounter <= bitcounter +1;
                end
            end
        end
    end
    
    //state machine
    always_ff @ (posedge CLK)
    begin
        load <=0; shift<=0; clear<=0; Tx<=1;
        case (PS)
            0: begin    //idle state
                    if(start_buffer) begin 
                        NS <=1;  //move to transmit state
                        load <=1;
                    end else
                        NS <= 0;
               end
            1: begin //transmit state
                    if(bitcounter >=10) begin   //transmit complete?
                        NS <=0;
                        clear <=1;
                    end else begin
                        NS <=1;
                        Tx <= rightshiftreg[0];
                        shift <=1;
                    end
               end
            default: NS <=0;
        endcase            
    end
    
endmodule

