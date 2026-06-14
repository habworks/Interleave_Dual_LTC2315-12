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

`ifndef PROJECT_DEFINES_VH
`define PROJECT_DEFINES_VH

// Define RTL Revision
`define RTL_VALID_REV       8'hA5
`define RTL_MAJOR_REV       8'd1
`define RTL_MINOR_REV       8'd3
`define RTL_TEST_REV        8'd14
// Define Boolean Status
`define TRUE                1'b1
`define FALSE               1'b0
`define HIGH                1'b1
`define LOW                 1'b0
`define CS_ENABLE           1'b0
`define CS_DISABLE          1'b1
`define CLK_ENABLE          1'b1
`define CLK_DISABLE         1'b0
`define FIFO_RESET_ENABLE   1'b1
`define FIFO_RESET_DISABLE  1'b0
`define FIFO_WR_ENABLE      1'b1
`define FIFO_WR_DISABLE     1'b0

// Define Channel States
`define STATE_WAIT                  4'd0
`define STATE_BEGIN                 4'd1
`define STATE_CLK_DATA_IN           4'd2
`define STATE_FIFO_WRITE            4'd3
`define STATE_SAMPLE_ACQUIRE_TIME   4'd4

// Define Constants
`define CYCLES_PER_ADC_TRANSFER     20
`define CYCLES_PER_DATA_TRANSFER    14
`define SAMPLE_ACQUIRE_CYCLES       4

// Define Status Register Bit Position
`define NO_ERROR                    4'b0000
`define ERROR_UNDEFINED_STATE       4'b0001 

`endif