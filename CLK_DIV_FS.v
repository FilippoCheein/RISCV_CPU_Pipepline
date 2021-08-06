
//-----------------------------------------------------------------------
//-- Module to divide the clock 
//-----------------------------------------------------------------------
module clk_div (  input clk,
                  output logic sclk=0);

  integer MAX_COUNT = 200000; //2
  integer div_cnt =0;
   always @ (posedge clk)              
   begin
         if (div_cnt == MAX_COUNT) 
         begin 
            sclk = ~sclk;
            div_cnt = 0;
         end else
            div_cnt = div_cnt + 1;  
   end 
endmodule
