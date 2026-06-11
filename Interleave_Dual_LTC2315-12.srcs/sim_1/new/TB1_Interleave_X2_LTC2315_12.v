`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: IMR Engineering, LLC
// Engineer: Hab Collector
// 
// Create Date: 05/09/2026 03:04:50 PM
// Design Name: Dual Interleave LTC2315-12
// Module Name: Interleave_X2_LTC2315_12
// Project Name: Interleave_Dual_LTC2315-12
// Target Devices: XC7A100TCSG324-X
// Tool Versions: 2024.2
// Description: Dual 5Msps LTC2315-12 devices interleave to create a 10Msps acquistion
// 
// Dependencies: 
// 
// Revision: 1.0
// Revision 0.01 - File Created 
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////



module TB1_Interleave_X2_LTC2315_12();

// OUTPUTS FROM THE TEST BENCH ARE INPUTS TO RTL
// System
reg Reset_n;
reg SysClock;
reg AXI_Acquire;
reg AXI_Reset;

// ADC Channel Data
reg A_SDATA;
reg B_SDATA;

// FIFO
reg FIFO_Ready;


// INPUTS TO THE TEST BENCH ARE OUTPUTS FROM THE RTL
// System
wire ADC_SampleRate_TP;
wire SysDone_IRQ;

// ADC
wire A_CS_n;
wire A_SCLK;
wire B_CS_n;
wire B_SCLK;

// FIFO
wire FIFO_WriteEnable;
wire FIFO_Reset;
wire [15:0] FIFO_Data;


Interleave_X2_LTC2315_12 DUT
(
   .Reset_n(Reset_n),
   .SysClock(SysClock),
   .AXI_Acquire(AXI_Acquire),
   .AXI_Reset(AXI_Reset),
   .SysDone_IRQ(SysDone_IRQ),
   .ADC_SampleRate_TP(ADC_SampleRate_TP),

   .A_SDATA(A_SDATA),
   .A_CS_n(A_CS_n),
   .A_SCLK(A_SCLK),

   .B_SDATA(B_SDATA),
   .B_CS_n(B_CS_n),
   .B_SCLK(B_SCLK),

   .FIFO_Ready(FIFO_Ready),
   .FIFO_WriteEnable(FIFO_WriteEnable),
   .FIFO_Reset(FIFO_Reset),
   .FIFO_Data(FIFO_Data)
);


// SysClock generation
initial
begin
   SysClock = 1'b0;
   forever #5 SysClock = !SysClock;
end


// Initialize simulation and start one acquisition
initial
begin
   Reset_n = 1'b0;
   AXI_Acquire = 1'b0;
   AXI_Reset = 1'b0;
   A_SDATA = 1'b0;
   B_SDATA = 1'b0;
   FIFO_Ready = 1'b1;

   #20;
   Reset_n = 1'b1;
   
   #20;
   AXI_Reset = 1'b1;
   #20;
   AXI_Reset = 1'b0;

   #20;
   AXI_Acquire = 1'b1;
   #20;
   AXI_Acquire = 1'b0;

   #800;
   $finish;
end


// ADC simulated responses run in parallel
always begin
      Drive_ADC_A_Sample({1'b0, 12'h7A5, 1'b0});
end

always begin
      Drive_ADC_B_Sample({1'b0, 12'h8C3, 1'b0});
end


// ADC CH A Perfect Simulation Model
task Drive_ADC_A_Sample;
    input [13:0] ADC_Sample;
    integer BitIndex;
    begin
        @(negedge A_CS_n);
        // Drive the first bit (Bit 13) immediately on CS drop!
        #1 
        A_SDATA = ADC_Sample[13]; 
        
        // Loop handles the remaining 13 bits on subsequent clock transitions
        for (BitIndex = 12; BitIndex >= 0; BitIndex = BitIndex - 1) begin
            @(negedge A_SCLK);
            #1 
            A_SDATA = ADC_Sample[BitIndex];
        end
        @(posedge A_CS_n);
        #1 
        A_SDATA = 1'b0;
    end
endtask

// ADC CH B Perfect Simulation Model
task Drive_ADC_B_Sample;
    input [13:0] ADC_Sample;
    integer BitIndex;
    begin
        @(negedge B_CS_n);
        // Drive the first bit (Bit 13) immediately on CS drop!
        #1 
        B_SDATA = ADC_Sample[13]; 
        
        // Loop handles the remaining 13 bits on subsequent clock transitions
        for (BitIndex = 12; BitIndex >= 0; BitIndex = BitIndex - 1) begin
            @(negedge B_SCLK);
            #1 B_SDATA = ADC_Sample[BitIndex];
        end
        @(posedge B_CS_n);
        #1 
        B_SDATA = 1'b0;
    end
endtask


endmodule