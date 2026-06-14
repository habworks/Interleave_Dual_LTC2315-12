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
// DESCRIPTION:
// ***OVERVIEW***
// This RTL interleaves 2 LTC2315-12 12b ADC(s) at their maximum BW (5Msps) to achieve an aggregate BW of 10Msps.
// The two channels are denoted as A and B.  This implementation follows Figure 5 of the LTC2315-12 datasheet.
// The implementation is done in a single clock domain (SysClock).  To achieve the max desired sample rate (10Msps)
// SysClock must run at 100MHz.  It is possible for SysClock to run slower than 100MHz, but it cannot run faster
// as to run faster would violate the LTC2315-12 timing.  The sample rate is calculated as:
//   SampleRate = 2 x (1 / (20 x (1/SysClock)))
//   Where SysClock is in Hz
//
// ***VERBIAGE***
// Clock and cycle are interchangeable: Both refer to SysClock, clock cycles
// Sample: A single 12b ADC acquisition (note raw capture from ADC is 14b it is padded MSb and LSb with 0s)
// Frame: A collection of samples of size TOTAL_SAMPLES_PER_FRAME (samples are stored as 16b in the FIFO 4xMSb are 0's)
//
// ***HOW IT WORKS***
// Though this RTL is implemented in a single clock domain, there are multiple blocks.  From the datasheet timing
// diagram there are 20 clocks per acquisition. In this order the clocks occur:
//    1x clock chip select active
//    14x clocks to clock out the data
//    1x clock chip select inactive
//    4x clocks to acquire the next sample
// Once a frame request has been started Sys_A_MainCycleCounterReg keeps a repetitive count of these 20 cycles.
// Channel A is started on the Sys_A_MainCycleCounterReg of 0.  Note that at 100MHz and 20 cycles later the sample
// will be completed only to start again.  This gives a sample rate of 5Msps.  Channel A and channel B block
// captures work identically, however they start at different times.  Channel B starts when Sys_A_MainCycleCounterReg
// reaches a count of 9 (10 clock cycles - counting from 0) which is halfway from when channel A started or said 
// differently, is started 180 degrees out of phase with channel A.  As Channel B rate of acquisition is identical 
// to channel A, but 180 degrees out of phase, there is a sample available (from A or B) every 10 clock cycles.  At a SysClock
// of 100MHz this creates an aggregate sample rate of 10Msps.  You can run sysclk slower than 100MHz for a slower
// aggregate sample rate than 10Msps, but you cannot run it faster than as this would violate the ADC: setup time, SCLK high time,
// data hold times and sample acquistion times. 
//
// ***CAUTION ON REUSE***
// This RTL is written in a single clock domain.  Both the ADC clock and AXI clocks are the same (SysClock).  
// Should this code be re-used, but at a different AXI bus clock, additional logic would be necessary to handle
// Clock Domain Crossing (CDC) between the system AXI and ADC clock domains.  
//
// ***HOW TO INTEGRATE WITH FIRMWARE***
// This RTL IP works to capture a single frame upon trigger.  The frame capture is completed upon interrupt.
// This IP is memory mapped - you will add the AXI memory map interface in another file as the top file.  
// SysDone_IRQ must feed into the PL Interrupt Controller.  Configuration of the interrupt, the address of the
// AXI FIFO memory map, and FIFO reset should all be a part of the RTL IP driver init.  
// After the RTL IP has been init (init TBD with AXI memory map) a frame request should follow these steps:
// 1. FW Driver asserts AXI_Reset (note at the IP level this is seen as a pulse)
// 2. FW Driver clears AXI_Reset (necessary for next rearm or optional clear before you assert)
// 3. FW Driver asserts AXI_Acquire (note at the IP level this is seen as a pulse)
// 4. FW Driver clears AXI_Acquire (necessary for next reset or optional clear before you assert)
// 5. FW do nothing until SysDone_IRQ interrupt
// 6. On interrupt read status register: 16MSb acquired sample count - must match TOTAL_SAMPLES_PER_FRAME, next 8b = 0, next 4b = Ch A ERROR CODE, last 4b = Ch B Error Code
// 7. If no error code and the acquired sample count matches TOTAL_SAMPLES_PER_FRAME then the frame sample data can be used
//////////////////////////////////////////////////////////////////////////////////


`include "Project_Defines.vh"

module Interleave_X2_LTC2315_12
#
(
    parameter TOTAL_SAMPLES_PER_FRAME = 4
)

(
    // SYSTEM INTERFACE (100MHz CLOCK)
    input wire  Reset_n,            // System Reset
    input wire  SysClock,           // System Clock 100MHz
    input wire  AXI_Acquire,        // Simulates AXI (memory mapped register) interface to start an acquistion frame
    input wire  AXI_Reset,          // Simulates AXI (memory mapped register) interface to reset (clear) the FIFO
    output reg  SysDone_IRQ,        // Positive Edge signals that an acquistion frame is completed - PL must connect this Interrupt Controller
    output reg  ADC_SampleRate_TP,  // Test point for measuring the aggragate ADC sample rate: There is a new sample available at every positive / negative edge - must be 50% duty - said differently square wwave frequency = SampleRate / 2.
    
    // ADC INTERFACE
    // Channel A
    input wire  A_SDATA,            // Data out from ADC
    output reg  A_CS_n,             // ADC chip select active low
    output wire A_SCLK,             // ADC serial clock - data is available on SDATA on clock's positive edge
    // Channel B
    input wire  B_SDATA,
    output reg  B_CS_n,
    output wire B_SCLK,
    
    // FIFO INTERFACE
    input wire  FIFO_Ready,         // FIFO Full active high - cleard with FIFO not full condition or FIFO reset
    output reg  FIFO_WriteEnable,   // Enables a write of the FIFO_Data into the FIFO
    output reg  FIFO_Reset,         // Resets the FIFO, a reset will clear the contents of the FIFO
    output reg  [15:0] FIFO_Data,   // 16b data bus for writing samples into the FIFO
    
    // REGISTER interface
    output wire [31:0] AXI_MM_StatusReg,
    output wire [31:0] AXI_MM_RevisionReg
);

// SYSTEM REGISTERS
reg [4:0]   Sys_A_MainCycleCounterReg;      // There are 20 cycles per ADC acquision 
reg [4:0]   Sys_B_MainCycleCounterReg;      // There are 20 cycles per ADC acquision 
reg [15:0]  SysSampleCountCompletedReg;     // Keeps a running total of the total number of samples completed
reg [15:0]  SysSampleCountInProgressReg;    // The number of acquisions in progress to be completed - total number of samples completed is one less
reg         SysFrameActiveReg;              // Indicates the capture of a frame is in progress
reg         SysAcquirePulseReg;             // Start pulse from system to begin a frame acquistion
reg         Sys_B_PhaseDelayInit;           // Channel B is 180 degrees out of phase with channel A or said differently it starts at the 10th clock cycle of A - this is used as a flag for channel B to start
       
// CHANNEL REGISTERS
// Channel A
reg [3:0]   A_State;                // State Machine State
reg [13:0]  A_ADC_ShiftReg;         // Data is shifted in to this register - 14b register - when sample is fully loaded the MSb and LSb will be 0
reg [15:0]  A_ADC_SampleData;       // Once all 14 bits have been shifted into the shift register this register is updated - formated with the 4MSb as 0 and data bits 12 to 1 from the shift register
reg [3:0]   A_Status;               // The error stats of the Channel 0 = No errors
reg         A_ClockGate;            // Enables clocking to SCLK
reg         A_SampleTrigger;        // When channel sample acquisition starts - used in knowing the next pending number of total acquistions completed

// Channel B 
reg [3:0]   B_State;
reg [13:0]  B_ADC_ShiftReg;
reg [15:0]  B_ADC_SampleData;
reg [3:0]   B_Status;
reg         B_ClockGate;
reg         B_SampleTrigger;

// MEMORY MAP REGISTER INFO
reg [31:0]   Sys_FIFO_StatusReg;    // Contains the status information {SysSampleCountCompletedReg[15:0], 8'd0, A_Status, B_Status}
reg [31:0]   Sys_RTL_Revision;      // The RTL revision {`RTL_VALID_REV, `RTL_MAJOR_REV, `RTL_MINOR_REV, `RTL_TEST_REV}



// ASSIGN AXI MEMORY MAP REGISTERS
assign AXI_MM_StatusReg = Sys_FIFO_StatusReg;
assign AXI_MM_RevisionReg = Sys_RTL_Revision;


// CLOCK GATING ODDR (Output Double Data Rate Register)
// Channel A: Forwards SysClock to pin only when A_ClockGate is active
ODDR #(
    .DDR_CLK_EDGE("OPPOSITE_EDGE"), // D1 is captured on rising edge, D2 on falling edge
    .INIT(1'b0),                    // Initial state of Q
    .SRTYPE("SYNC")                 // Forces Reset and Set to be synchronus with clock as opposed to ASYNC
) ODDR_inst_a (
    .Q  (A_SCLK),                   // This is output - Drives physical pin directly
    .C  (SysClock),                 // This is input clock - 100MHz system clock
    .CE (1'b1),                     // This is Clock Enable input - Always enable
    .D1 (A_ClockGate),              // On rising edge output (Q) =  A_ClockGate value
    .D2 (1'b0),                     // On falling edge output (Q) = 0
    .R  (~Reset_n),                 // Reset input
    .S  (1'b0)                      // If set high the output will be high - make low do not ever want to force output high
);

// Channel B: Forwards SysClock to pin only when B_ClockGate is active
ODDR #(
    .DDR_CLK_EDGE("OPPOSITE_EDGE"),
    .INIT(1'b0),
    .SRTYPE("SYNC")
) ODDR_inst_b (
    .Q  (B_SCLK),
    .C  (SysClock),
    .CE (1'b1),
    .D1 (B_ClockGate),
    .D2 (1'b0),
    .R  (~Reset_n),     
    .S  (1'b0)
);



// AXI ACQUIRE FRAME START PULSE BLOCK: Turn AXI_Acquire Signal into a pulse - avoid unitended re-trigger
reg     AcquireFramePulseReg_d1;
wire    AcquireFrameStartPulse;
always @(posedge SysClock or negedge Reset_n)
begin
    if (Reset_n == `LOW)
        AcquireFramePulseReg_d1 <= `FALSE;
    else
        AcquireFramePulseReg_d1 <= AXI_Acquire;
end
assign AcquireFrameStartPulse = AXI_Acquire & !AcquireFramePulseReg_d1;   // This is only true if AXI_Acquire was previously low, now high and will self clear in the next clock cycle



// AXI RESET OF FIFO: Turn AXI_Reset Signal into a pulse - avoid unitended re-trigger
reg     ResetPulseReg_d1;
wire    ResetPulse;
always @(posedge SysClock or negedge Reset_n)
begin
    if (Reset_n == `LOW)
        ResetPulseReg_d1 <= `FALSE;
    else
        ResetPulseReg_d1 <= AXI_Reset;
end
assign ResetPulse = AXI_Reset & !ResetPulseReg_d1;   // This is only true if AXI_Reset was previously low, now high and will self clear in the next clock cycle



// SYSTEM BLOCK: Enable FIFO, Software start trigger, Count Acquistions, Check if done - Note because of the mixing of sync and async signals in the always - the structure must be if, else if, else if... else and not if, if, if... if
always @(posedge SysClock or negedge Reset_n)
begin
    if (Reset_n == `LOW)
    begin
        SysFrameActiveReg <= `FALSE;
        SysDone_IRQ <= `FALSE;
        FIFO_Reset <= `FIFO_RESET_ENABLE;
        Sys_A_MainCycleCounterReg <= 5'd0;
        Sys_B_MainCycleCounterReg <= 5'd0;
        SysSampleCountCompletedReg <= 16'd0;
        SysSampleCountInProgressReg <= 16'd0;
        Sys_FIFO_StatusReg <= 32'd0;
        Sys_B_PhaseDelayInit <= `FALSE;
        Sys_RTL_Revision <= {`RTL_VALID_REV, `RTL_MAJOR_REV, `RTL_MINOR_REV, `RTL_TEST_REV};
    end else
       
    begin
        // The reset pulse takes same action as Reset_n
        if (ResetPulse)
        begin
            SysFrameActiveReg <= `FALSE;
            SysDone_IRQ <= `FALSE;
            FIFO_Reset <= `FIFO_RESET_ENABLE;
            Sys_A_MainCycleCounterReg <= 5'd0;
            Sys_B_MainCycleCounterReg <= 5'd0;
            SysSampleCountCompletedReg <= 16'd0;
            SysSampleCountInProgressReg <= 16'd0;
            Sys_FIFO_StatusReg <= 32'd0;
            Sys_B_PhaseDelayInit <= `FALSE;
        end 
        
        // SOFTWARE TRIGGER: An acquire signal was given to start acquistion - set init conditions for acquistion, SysFrameActiveReg indicates frame acquistion active
        if (AcquireFrameStartPulse && !SysFrameActiveReg && !ResetPulse)
        begin
            SysFrameActiveReg <= `TRUE;
            SysDone_IRQ <= `FALSE;
            FIFO_Reset <= `FIFO_RESET_DISABLE;
            Sys_A_MainCycleCounterReg <= 5'd0;
            Sys_B_MainCycleCounterReg <= 5'd0;
            SysSampleCountCompletedReg <= 16'd0;
            Sys_FIFO_StatusReg <= 32'd0;
        end
        
        // TRACK NEXT SAMPLE IN PROGRESS: Need to know this so you know when to be completed
        if (A_SampleTrigger || B_SampleTrigger)
            SysSampleCountInProgressReg <= SysSampleCountInProgressReg + 16'd1;
        
        // ADC CYCLE COUNTER: If frame acquistion active then the main counter needs to be counting - note there are (1 + 14 + 1 + 4) 20 cycles per transfer
        if (SysFrameActiveReg)
        begin
            // A main clock clounter repeats every 20 clicks
            if (Sys_A_MainCycleCounterReg == `CYCLES_PER_ADC_TRANSFER - 1)
                Sys_A_MainCycleCounterReg <= 5'd0;
            else
                Sys_A_MainCycleCounterReg <= Sys_A_MainCycleCounterReg + 1'd1;
            // On the inital A counter 1/2 through (180 degrees out of phase) set flag to allow the B counter to start
            if ((Sys_A_MainCycleCounterReg == 5'd9) && (!Sys_B_PhaseDelayInit))
                Sys_B_PhaseDelayInit <= `TRUE;
            
            // B main clock clounter repeats every 20 clicks         
            if (Sys_B_PhaseDelayInit)
            begin
                if (Sys_B_MainCycleCounterReg == `CYCLES_PER_ADC_TRANSFER - 1)
                    Sys_B_MainCycleCounterReg <= 5'd0;
                else
                    Sys_B_MainCycleCounterReg <= Sys_B_MainCycleCounterReg + 1'd1;
            end
        end
        
        // FRAME COMPLETE CHECK: A frame is complete when the requested number of samples have been acquired.  FIFO_WriteEnable indicates a write occurred
        if (FIFO_WriteEnable && FIFO_Ready && SysFrameActiveReg)
        begin
            if ((SysSampleCountCompletedReg + 16'd1) == TOTAL_SAMPLES_PER_FRAME)
            begin
                Sys_FIFO_StatusReg <= {SysSampleCountCompletedReg[15:0] + 1, 8'd0, A_Status, B_Status};
                SysFrameActiveReg <= `FALSE;
                SysDone_IRQ <= `TRUE;
            end else
            begin
                SysSampleCountCompletedReg <= SysSampleCountCompletedReg + 16'd1;
            end
        end
        
        // ERROR CHECKING: Check for errors in either Channel A or B
        if (SysFrameActiveReg && (A_Status | B_Status))
        begin
            Sys_FIFO_StatusReg <= {16'd0, 8'd0, A_Status, B_Status};
            SysFrameActiveReg <= `FALSE;
            SysDone_IRQ <= `TRUE;
        end
        
    end   
end



// ADC CHANNEL A SPI INTERFACE: When active capture a sample from ADC A
always @(posedge SysClock or negedge Reset_n)
begin
    if (Reset_n == `LOW)
    begin
        A_CS_n <= `CS_DISABLE;
        A_ClockGate <= `CLK_DISABLE;
        A_State <= `STATE_WAIT;
        A_Status <= `NO_ERROR;
    end else
    
    begin
        case (A_State)
            // WAIT STATE: To advance count 1 cycle
            `STATE_WAIT:
            begin
                if ((SysFrameActiveReg) && (Sys_A_MainCycleCounterReg == 5'd0) && (SysSampleCountInProgressReg != TOTAL_SAMPLES_PER_FRAME)) // Starts at Sys_A_MainCycleCounterReg 0 and takes 20 cycles for state machine to advance back to STATE_WAIT
                begin
                    A_ADC_ShiftReg <= 14'd0;
                    A_CS_n <= `CS_ENABLE;
                    A_ClockGate <= `CLK_ENABLE;
                    A_SampleTrigger <= `TRUE;
                    A_State <= `STATE_BEGIN; 
                end else
                begin
                    A_CS_n <= `CS_DISABLE;
                    A_ClockGate <= `CLK_DISABLE;
                    A_Status <= `NO_ERROR;
                    A_SampleTrigger <= `FALSE;
                end
            end
            
            `STATE_BEGIN:
            begin
                A_SampleTrigger <= `FALSE;
                A_State <= `STATE_CLK_DATA_IN;
                A_ADC_ShiftReg <= {A_ADC_ShiftReg[12:0], A_SDATA};
            end
            
            // CLOCK IN DATA STATE: The ADC sample is being clocked into the shift register a total of 14b.  To advance to next state count 14 cycles
            `STATE_CLK_DATA_IN:
            begin
                if (Sys_A_MainCycleCounterReg == 5'd14)
                    A_ClockGate <= `CLK_DISABLE;
                if (Sys_A_MainCycleCounterReg == 5'd15)
                    A_State <= `STATE_FIFO_WRITE;
                else
                A_ADC_ShiftReg <= {A_ADC_ShiftReg[12:0], A_SDATA};
            end
            
            // WRITE TO FIFO STATE: To advance count 1 cycle
            `STATE_FIFO_WRITE:
            begin
                A_CS_n <= `CS_DISABLE;
                A_ADC_SampleData <= {4'd0, A_ADC_ShiftReg[12:1]};
                A_State <= `STATE_SAMPLE_ACQUIRE_TIME;
            end
            
            // ACQUIRE NEXT SAMPLE STATE: To advance count 4 cycles - Total cycles per acquistion 1 + 14 + 1 + (4) = 20
            `STATE_SAMPLE_ACQUIRE_TIME:
            begin
                if (Sys_A_MainCycleCounterReg == 5'd19)
                    A_State <= `STATE_WAIT;
            end
            
            // UNHANDLE USE CASE: Load status error, stop and set IRQ
            default:
            begin
                A_Status <= `ERROR_UNDEFINED_STATE;
            end  
        endcase
    end 
end



// ADC CHANNEL B SPI INTERFACE: When active capture a sample from ADC B - Channel B is identical to channel A except for when it starts (180 degrees out of phase) 
always @(posedge SysClock or negedge Reset_n)
begin
    if (Reset_n == `LOW)
    begin
        B_CS_n <= `CS_DISABLE;
        B_ClockGate <= `CLK_DISABLE;
        B_State <= `STATE_WAIT;
        B_Status <= `NO_ERROR;
    end else
    
    begin
        case (B_State)
            // WAIT STATE: To advance count 1 cycle
            `STATE_WAIT:
            begin
                if ((SysFrameActiveReg) && (Sys_B_MainCycleCounterReg  == 5'd0) && (SysSampleCountInProgressReg != TOTAL_SAMPLES_PER_FRAME) && Sys_B_PhaseDelayInit) // Starts at Sys_A_MainCycleCounterReg 9 (10 clycles later - count from 0) and takes 20 cycles (Acquistion Rate 5Msps) - Acquistion begins 1/2 way into acquistion A complete 180 degrees out of phase with A start
                begin
                    B_ADC_ShiftReg <= 14'd0;
                    B_CS_n <= `CS_ENABLE;
                    B_ClockGate <= `CLK_ENABLE;
                    B_SampleTrigger <= `TRUE;
                    B_State <= `STATE_BEGIN;
                end else
                begin
                    B_CS_n <= `CS_DISABLE;
                    B_ClockGate <= `CLK_DISABLE;
                    B_Status <= `NO_ERROR;
                    B_SampleTrigger <= `FALSE;
                end
            end
            
            `STATE_BEGIN:
            begin
                B_SampleTrigger <= `FALSE;
                B_State <= `STATE_CLK_DATA_IN;
                B_ADC_ShiftReg <= {B_ADC_ShiftReg[12:0], B_SDATA};
            end
            
            // CLOCK IN DATA STATE: To advance count 14 cycles
            `STATE_CLK_DATA_IN:
            begin
                if (Sys_B_MainCycleCounterReg == 5'd14)
                    B_ClockGate <= `CLK_DISABLE;
                if (Sys_B_MainCycleCounterReg == 5'd15)
                    B_State <= `STATE_FIFO_WRITE;
                else
                    B_ADC_ShiftReg <= {B_ADC_ShiftReg[12:0], B_SDATA};
            end
            
            // WRITE TO FIFO STATE: To advance count 1 cycle
            `STATE_FIFO_WRITE:
            begin
                B_CS_n <= `CS_DISABLE;
                B_ADC_SampleData <= {4'd0, B_ADC_ShiftReg[12:1]};
                B_State <= `STATE_SAMPLE_ACQUIRE_TIME;
            end
            
            // ACQUIRE NEXT SAMPLE STATE: To advance count 4 cycles - Total cycles per acquistion 1 + 14 + 1 + 4 = 20
            `STATE_SAMPLE_ACQUIRE_TIME:
            begin
                if (Sys_B_MainCycleCounterReg == 5'd19)
                    B_State <= `STATE_WAIT;
            end
            
            // UNHANDLE USE CASE: Load status error, stop and set IRQ
            default:
            begin
                B_Status <= `ERROR_UNDEFINED_STATE;
            end
        endcase 
    end
end



// FIFO WRITE BLOCK (FIXED): Cleanly decoupled data routing and strobe generation
always @(posedge SysClock or negedge Reset_n) begin
    if (Reset_n == `LOW) 
    begin
        FIFO_Data         <= 16'd0;
        FIFO_WriteEnable  <= `FIFO_WR_DISABLE;
        ADC_SampleRate_TP <= `LOW;
    end else 
    
    begin
        // =====================================================================
        // 1. DATA ROUTING PATH
        // =====================================================================
        // Safely capture A or B data on cycle 17. Because these counters are 
        // 180 degrees out of phase, these two specific conditions never happen 
        // on the exact same clock edge, keeping the data bus perfectly shared.
        if (Sys_A_MainCycleCounterReg == 5'd17) 
            FIFO_Data <= A_ADC_SampleData;
        else if ((Sys_B_PhaseDelayInit == `TRUE) && (Sys_B_MainCycleCounterReg == 5'd17))
            FIFO_Data <= B_ADC_SampleData;

        // =====================================================================
        // 2. WRITE ENABLE & TEST POINT STROBE PATH
        // =====================================================================
        // This conditiona handles the FIFO Wr strobe. If either channel A or B
        // hits cycle 18, it creates a clean, 1-cycle wide pulse. Otherwise,
        // FIFO Wr strobe defaults to disabled 
        if ((Sys_A_MainCycleCounterReg == 5'd18) || ((Sys_B_PhaseDelayInit == `TRUE) && (Sys_B_MainCycleCounterReg == 5'd18))) 
        begin
            FIFO_WriteEnable  <= `FIFO_WR_ENABLE;
            ADC_SampleRate_TP <= !ADC_SampleRate_TP;
        end else 
        begin
            // Falls back here on all other 18 cycles, keeping the strobe low
            FIFO_WriteEnable  <= `FIFO_WR_DISABLE;
        end
    end
end


endmodule