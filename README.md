# RISCV_CPU_Pipepline

This repository contains a SystemVerilog version of a RISC-V CPU Pipelined with hazard control applied.
The memory is dual port so to avoid Structural Hazards.

The hierarchy of the files is:

* clock_tb: to give timing to the system.
 * OTTER_Wrapper: to connect the cpu to peripherals like 7-segments display, VGA, UART, Keyboard, Board LEDs & Buttons.
   * OTTER_MCU: This is the CPU of the systems. It is pipelined in 5 stages. 

Fetch stage:
 \n PCdatasrc      : MUX selecting the correct input to the pc
 PC             : Register that either load the value inputted from the PCdatasearch or it reset to zero.
 Instruction Memory: based on the PC value it outputs the instruction stored in that memory location.

Decode Stage:
          CU_DECODER_PIPE: Based on the instruction it sends out signals to register, data memory and ALU
          Register File: Stores values throughout the program run. 
                          At this stage ypu can read from the register file.
          ALU_srcA, ALU_srcB: Two MUX that based on the decoder select the correct inputs to the ALU.
          Hazard Detection Unit: Sends signals if a possible hazard is detected 

Execute Stage:
          Target Gemneraor: send values to the PCdatasrc for jump, branch jalr.
          Branch CondGenerator: sends signals to avoid control hazards.
          ALU            : Does selected math based on decoder signals
          Forwarding Unit: orchestrates the forwarding event when execute data hard is detected by the Hazard Detection Unit.
          ForwardA_ALU, ForwardB_ALU: signal from forwarding unit chooses whether to farward the value to the ALU

Memory Stage:
          Data Memory: at This stage you vsn read or write to the memory.

WriteBack Stage
          regWriteBack: based on the signal coming from the decoder it output a determined value to he register file.
          Register File: at this stage it is possible to write to the register file.
       
       
Pipeline CPU Design Diagram: 

![alt text](https://github.com/FilippoCheein/RISCV_CPU_Pipepline/blob/main/Pipeline_Diagram.png?raw=true)

