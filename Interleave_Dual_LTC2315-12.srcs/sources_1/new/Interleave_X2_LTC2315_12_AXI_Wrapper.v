`include "Project_Defines.vh"


// Revised
module Interleave_ADC_AXI_Wrapper # (
    parameter integer TOTAL_SAMPLES_PER_FRAME = 4, // Parametric default mapping
    parameter integer C_S_AXI_DATA_WIDTH     = 32,
    parameter integer C_S_AXI_ADDR_WIDTH     = 4
) (
    // =========================================================================
    // PHYSICAL ADC / FIFO INTERFACE (Your core external physical pins)
    // =========================================================================
    output wire SysDone_IRQ,
    output wire ADC_SampleRate_TP,
    
    // Channel A
    input wire  A_SDATA,
    output wire A_CS_n,
    output wire A_SCLK,
    
    // Channel B
    input wire  B_SDATA,
    output wire B_CS_n,
    output wire B_SCLK,
    
    // FIFO Interface
    input wire         FIFO_Ready,
    output wire        FIFO_WriteEnable,
    output wire        FIFO_Reset,
    output wire [15:0] FIFO_Data,

    // =========================================================================
    // AXI-LITE BUS INTERFACE
    // =========================================================================
    input wire                              S_AXI_ACLK,
    input wire                              S_AXI_ARESETN,
    input wire  [C_S_AXI_ADDR_WIDTH-1 : 0]   S_AXI_AWADDR,
    input wire  [2 : 0]                     S_AXI_AWPROT,
    input wire                              S_AXI_AWVALID,
    output wire                             S_AXI_AWREADY,
    input wire  [C_S_AXI_DATA_WIDTH-1 : 0]   S_AXI_WDATA,
    input wire  [(C_S_AXI_DATA_WIDTH/8)-1:0] S_AXI_WSTRB,
    input wire                              S_AXI_WVALID,
    output wire                             S_AXI_WREADY,
    output wire [1 : 0]                     S_AXI_BRESP,
    output wire                             S_AXI_BVALID,
    input wire                              S_AXI_BREADY,
    input wire  [C_S_AXI_ADDR_WIDTH-1 : 0]   S_AXI_ARADDR,
    input wire  [2 : 0]                     S_AXI_ARPROT,
    input wire                              S_AXI_ARVALID,
    output wire                             S_AXI_ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1 : 0]   S_AXI_RDATA,
    output wire [1 : 0]                     S_AXI_RRESP,
    output wire                             S_AXI_RVALID,
    input wire                              S_AXI_RREADY
);

    // =========================================================================
    // INTERNAL WIRES & REGISTERS FOR BUS HANDSHAKING
    // =========================================================================
    reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_awaddr;
    reg                            axi_awready;
    reg                            axi_wready;
    reg [1 : 0]                    axi_bresp;
    reg                            axi_bvalid;
    reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_araddr;
    reg                            axi_arready;
    reg [C_S_AXI_DATA_WIDTH-1 : 0] axi_rdata;
    reg [1 : 0]                    axi_rresp;
    reg                            axi_rvalid;

    // Pulse-generation and baseline registers
    reg                            reg_axi_acquire_pulse;
    reg                            reg_axi_reset_pulse;
    reg [15:0]                     reg_total_samples; 

    // Wires pulling status back out from your core instance
    wire [31:0]                    w_axi_mm_status_reg;
    wire [31:0]                    w_axi_mm_revision_reg;

    // Connect local ports directly to standard AXI status outputs
    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BRESP   = axi_bresp;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = axi_rresp;
    assign S_AXI_RVALID  = axi_rvalid;

    // =========================================================================
    // AXI-LITE HANDSHAKE STATE MACHINE (Write Operations)
    // =========================================================================
    always @( posedge S_AXI_ACLK ) begin
        if ( S_AXI_ARESETN == 1'b0 ) begin
            axi_awready <= 1'b0;
            axi_awaddr  <= 0;
            axi_wready  <= 1'b0;
        end else begin
            // Address Write Ready
            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID) begin
                axi_awready <= 1'b1;
                axi_awaddr  <= S_AXI_AWADDR;
            end else begin
                axi_awready <= 1'b0;
            end
            
            // Data Write Ready
            if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID) begin
                axi_wready <= 1'b1;
            end else begin
                axi_wready <= 1'b0;
            end
        end
    end

    wire loc_write_en = axi_wready && S_AXI_WVALID && axi_awready && S_AXI_AWVALID;

    // =========================================================================
    // AXI-LITE REGISTER WRITE TRANSACTION WITH AUTO-HARDWARE PULSING
    // =========================================================================
    always @( posedge S_AXI_ACLK ) begin
        if ( S_AXI_ARESETN == 1'b0 ) begin
            reg_axi_acquire_pulse <= 1'b0;
            reg_axi_reset_pulse   <= 1'b0;
            reg_total_samples     <= 16'd4; // Default mapping matching parameter
        end else begin
            // Crucial: These drop back down to zero on the immediate next cycle.
            // This transforms a software register 'set' into an exact 1-cycle strobe.
            reg_axi_acquire_pulse <= 1'b0;
            reg_axi_reset_pulse   <= 1'b0;

            if (loc_write_en) begin
                case ( axi_awaddr[3:2] )
                    2'b00: begin // Offset 0x00: Control Register
                        if (S_AXI_WSTRB[0]) begin
                            reg_axi_acquire_pulse <= S_AXI_WDATA[0]; // Bit 0 drives Acquire
                            reg_axi_reset_pulse   <= S_AXI_WDATA[1]; // Bit 1 drives Reset
                        end
                    end
                    2'b11: begin // Offset 0x0C: Configuration Register
                        if (S_AXI_WSTRB[0]) begin
                            reg_total_samples[7:0] <= S_AXI_WDATA[7:0];
                        end
                        if (S_AXI_WSTRB[1]) begin
                            reg_total_samples[15:8] <= S_AXI_WDATA[15:8];
                        end
                    end
                    default: ; // 0x04 and 0x08 are completely read-only
                endcase
            end
        end
    end

    // Write Response Handshaking
    always @( posedge S_AXI_ACLK ) begin
        if ( S_AXI_ARESETN == 1'b0 ) begin
            axi_bvalid <= 1'b0;
            axi_bresp  <= 2'b00;
        end else begin
            if (axi_awready && S_AXI_AWVALID && ~axi_bvalid && axi_wready && S_AXI_WVALID) begin
                axi_bvalid <= 1'b1;
                axi_bresp  <= 2'b00;
            end else if (S_AXI_BREADY && axi_bvalid) begin
                axi_bvalid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // AXI-LITE HANDSHAKE STATE MACHINE (Read Operations)
    // =========================================================================
    always @( posedge S_AXI_ACLK ) begin
        if ( S_AXI_ARESETN == 1'b0 ) begin
            axi_arready <= 1'b0;
            axi_araddr  <= 32'b0;
        end else begin
            if (~axi_arready && S_AXI_ARVALID) begin
                axi_arready <= 1'b1;
                axi_araddr  <= S_AXI_ARADDR;
            end else begin
                axi_arready <= 1'b0;
            end
        end
    end

    // =========================================================================
    // AXI-LITE REGISTER READ TRANSACTION
    // =========================================================================
    always @( posedge S_AXI_ACLK ) begin
        if ( S_AXI_ARESETN == 1'b0 ) begin
            axi_rvalid <= 1'b0;
            axi_rresp  <= 2'b00;
            axi_rdata  <= 32'b0;
        end else begin
            if (axi_arready && S_AXI_ARVALID && ~axi_rvalid) begin
                axi_rvalid <= 1'b1;
                axi_rresp  <= 2'b00;
                case ( axi_araddr[3:2] )
                    // 0x00: Reads back momentary pulse state
                    2'b00: axi_rdata <= {30'd0, reg_axi_reset_pulse, reg_axi_acquire_pulse};
                    // 0x04: Pulls out status from your newly updated core port
                    2'b01: axi_rdata <= w_axi_mm_status_reg;
                    // 0x08: Pulls out revision from your newly updated core port
                    2'b10: axi_rdata <= w_axi_mm_revision_reg;
                    // 0x0C: Reads config register
                    2'b11: axi_rdata <= {16'd0, reg_total_samples};
                    default: axi_rdata <= 32'hDEADBEEF;
                endcase
            end else if (axi_rvalid && S_AXI_RREADY) begin
                axi_rvalid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // CHILD INSTANTIATION OF YOUR ADC SYSTEM RTL
    // =========================================================================
    Interleave_X2_LTC2315_12 # (
        .TOTAL_SAMPLES_PER_FRAME (TOTAL_SAMPLES_PER_FRAME)
    ) u_core_adc_rtl (
        .SysClock           (S_AXI_ACLK),
        .Reset_n            (S_AXI_ARESETN),
        .AXI_Acquire        (reg_axi_acquire_pulse),
        .AXI_Reset          (reg_axi_reset_pulse),
        .SysDone_IRQ        (SysDone_IRQ),
        .ADC_SampleRate_TP  (ADC_SampleRate_TP),
        .A_SDATA            (A_SDATA),
        .A_CS_n             (A_CS_n),
        .A_SCLK             (A_SCLK),
        .B_SDATA            (B_SDATA),
        .B_CS_n             (B_CS_n),
        .B_SCLK             (B_SCLK),
        .FIFO_Ready         (FIFO_Ready),
        .FIFO_WriteEnable   (FIFO_WriteEnable),
        .FIFO_Reset         (FIFO_Reset),
        .FIFO_Data          (FIFO_Data),
        .AXI_MM_StatusReg   (w_axi_mm_status_reg),
        .AXI_MM_RevisionReg (w_axi_mm_revision_reg)
        );
        
endmodule
