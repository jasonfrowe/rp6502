module rp6502_arm_if_regs_v1 (
    input  logic        clk,
    input  logic        reset_n,

    input  logic [7:0]  bus_addr,
    input  logic        bus_wr,
    input  logic        bus_rd,
    input  logic [31:0] bus_wdata,
    output logic [31:0] bus_rdata,

    output logic [31:0] reg_ctrl,
    output logic [31:0] reg_status,
    output logic [31:0] reg_heartbeat,
    output logic [31:0] reg_last_error,
    output logic [31:0] reg_cmd_seq,
    output logic [31:0] reg_ack_seq
);

    localparam logic [31:0] IF_MAGIC   = 32'h52503631; // "RP61"
    localparam logic [31:0] IF_VERSION = 32'h00010000;

    localparam logic [7:0] OFF_MAGIC     = 8'h00;
    localparam logic [7:0] OFF_VERSION   = 8'h04;
    localparam logic [7:0] OFF_CTRL      = 8'h08;
    localparam logic [7:0] OFF_STATUS    = 8'h0C;
    localparam logic [7:0] OFF_HEARTBEAT = 8'h10;
    localparam logic [7:0] OFF_LAST_ERR  = 8'h14;
    localparam logic [7:0] OFF_CMD_SEQ   = 8'h18;
    localparam logic [7:0] OFF_ACK_SEQ   = 8'h1C;

    always_ff @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            reg_ctrl      <= 32'd0;
            reg_status    <= 32'd0;
            reg_heartbeat <= 32'd0;
            reg_last_error<= 32'd0;
            reg_cmd_seq   <= 32'd0;
            reg_ack_seq   <= 32'd0;
        end else if(bus_wr) begin
            unique case(bus_addr)
                OFF_CTRL:      reg_ctrl       <= bus_wdata;
                OFF_STATUS:    reg_status     <= bus_wdata;
                OFF_HEARTBEAT: reg_heartbeat  <= bus_wdata;
                OFF_LAST_ERR:  reg_last_error <= bus_wdata;
                OFF_CMD_SEQ:   reg_cmd_seq    <= bus_wdata;
                OFF_ACK_SEQ:   reg_ack_seq    <= bus_wdata;
                default: ;
            endcase
        end
    end

    always_comb begin
        bus_rdata = 32'd0;
        if(bus_rd) begin
            unique case(bus_addr)
                OFF_MAGIC:     bus_rdata = IF_MAGIC;
                OFF_VERSION:   bus_rdata = IF_VERSION;
                OFF_CTRL:      bus_rdata = reg_ctrl;
                OFF_STATUS:    bus_rdata = reg_status;
                OFF_HEARTBEAT: bus_rdata = reg_heartbeat;
                OFF_LAST_ERR:  bus_rdata = reg_last_error;
                OFF_CMD_SEQ:   bus_rdata = reg_cmd_seq;
                OFF_ACK_SEQ:   bus_rdata = reg_ack_seq;
                default:       bus_rdata = 32'd0;
            endcase
        end
    end

endmodule
