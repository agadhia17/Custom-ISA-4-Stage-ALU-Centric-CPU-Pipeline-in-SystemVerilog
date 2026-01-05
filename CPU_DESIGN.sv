
// Code your design here
`timescale 1ns/1ns

`define OPC_ADDI   4'h0        // I-type: add immediate
`define OPC_SUBI   4'h1	       // I-type: subtract immediate
`define OPC_ANDI   4'h2        // I-type: and immediate
`define OPC_ORI    4'h3        // I-type: or immediate
`define OPC_XORI   4'h4		   // I=type: xor immediate
`define OPC_SLLI   4'h5 	   // I-type: shift-left immediate
`define OPC_SRLI   4'h6		   // I-type: shift-right immediate
`define OPC_ADD	   4'h7		   // R-type: add
`define OPC_SUB	   4'h8		   // R-type: subtract
`define OPC_AND	   4'h9		   // R-type: and
`define OPC_OR	   4'hA		   // R-type: or
`define OPC_XOR	   4'hB		   // R-type: xor


`define ALU_ADD    3'd0        // ALU op code: add
`define ALU_SUB    3'd1        // ALU op code: sub
`define ALU_AND    3'd2        // ALU op code: and
`define ALU_OR     3'd3        // ALU op code: or
`define ALU_XOR    3'd4        // ALU op code: xor
`define ALU_SLL    3'd5        // ALU op code: shift left
`define ALU_SRL    3'd6        // ALU op code: shift right


//------------------------------------------------------------------------------
// Module: IF_STAGE
// Purpose: RTL block for the 16-bit 4-stage ALU-focused pipelined CPU.
// Notes  : See README for ISA + pipeline behavior (IF/ID/EX/WB).
//------------------------------------------------------------------------------
//INFORMATION FETCH STAGE: READS 16-BIT INSTRUCTION CODE FROM AN 8WORD BY 16BIT READ ONLY FILE AND OUTPUTS EACH ONE EVERY CLOCK CYCLE TO THE FETCH-TO-DECODE PIPELINE REGISTER
module IF_STAGE #(parameter iMemDepth = 8, //instr code hex file will have only 8 instructions
            parameter iMemAWL = $clog2(iMemDepth), //log2 of depth gives number of addr bits
            parameter InstrMem = "instr_mem.txt", //parameter for whatever ROM file you use
            parameter instrDWL = 16 //how many bits each word in instruction ROM is
           ) 
  
 (input CLK, RST, 
  output reg [iMemAWL-1:0] pc_out,
  output wire [instrDWL-1:0] instr_out);
  
  reg [instrDWL-1:0] iROM [0:iMemDepth-1]; //create memory array to read file into
  
  initial begin
    integer i;
    for (i = 0; i < iMemDepth; i=i+1) begin
      iROM[i] = 16'h0000; // setting each ROM element value to 0 before loading file to it
    end
    $readmemh(InstrMem, iROM); //load file to iROM
  end
    
  always @(posedge CLK) begin //sequential logic for this stage
    if(RST) pc_out <= '0; //synchronous RST input resets program counter 
    else pc_out <= pc_out + 1'b1; //otherwise incrememnts it
  end
    
  assign instr_out = iROM[pc_out]; //combinational logic to make sure opcode is updated based on pc_out's new value as it's updated
endmodule

    
//------------------------------------------------------------------------------
// Module: registerFile
// Purpose: RTL block for the 16-bit 4-stage ALU-focused pipelined CPU.
// Notes  : See README for ISA + pipeline behavior (IF/ID/EX/WB).
//------------------------------------------------------------------------------
//REGISTER FILE: IS THE 2READ 1WRITE MEMORY THAT HOLDS DESTINATION, SECONDARY, AND OPTIONAL THIRD REGISTERS FOR ARITMETIC. IT IS ALSO WHERE THE RESULTS OF THE WRITEBACK STAGE ARE STORED
module registerFile(input CLK, RST,
                    input [3:0] RA1, RA2, //2read 1write memory with synchronous reset
                    output wire [15:0] RD1, RD2,
                    input WE,
                    input [3:0] WA,
                    input [15:0] WD);
  
  reg [15:0] RAM [0:15]; //16 word x 16 bit registers for the register file
  integer i;
  
  assign RD1 = RAM[RA1]; //combinational logic for outputting the read requests
  assign RD2 = RAM[RA2];
  
  //check for RST or WE to update register values every clock  cycle
  always @(posedge CLK) begin 
    if (RST) begin
      for(i=0; i<16; i=i+1) begin //if reset, clear all registers to 0
        RAM[i] <= 16'h0000;
      end
    end else if (WE && (WA != 4'd0)) begin //making sure r0 stays 0
      RAM[WA] <= WD;
    end
  end
endmodule


//------------------------------------------------------------------------------
// Module: ID_STAGE
// Purpose: RTL block for the 16-bit 4-stage ALU-focused pipelined CPU.
// Notes  : See README for ISA + pipeline behavior (IF/ID/EX/WB).
//------------------------------------------------------------------------------
//INFORMATION DECODE: THIS MODULE SPLITS THE INPUT INSTRUCTION CODE INTO OPCODE, DESTINATION REGISTER ADDRESS, SECONDARY REGISTER ADDRESS, AND LO4(IF R-TYPE, IS THE SHIFT AMOUNT AND THE ADDRESS OF THE THIRD REGISTER, otherwise the immediate value)
module ID_STAGE (input CLK, RST,
           input wire [15:0] instr_in,
           input wire [15:0] wb_rf_wd,
           input wire [3:0] wb_rf_wa,
           input wire wb_rf_we,
           output wire [2:0] alu_sel_out,
           output wire [3:0] shamt_out,
           output wire [15:0] rs_data_out,
           output wire rd_we_out, 
           output wire [3:0] rd_addr_out,
           output wire [15:0] roi_data_out,
           output wire use_imm_out);
  
  wire [3:0] opcode = instr_in[15:12];
  wire [3:0] rd_addr = instr_in[11:8];
  wire [3:0] rs_addr = instr_in[7:4];
  wire [3:0] lo4 = instr_in[3:0];
  
  reg [2:0] alu_sel;
  reg rd_we;
  reg use_imm;
  reg [3:0] shamt;
  
  always @(*) begin
    alu_sel = `ALU_ADD;
    rd_we = 0;
    use_imm = 0;
    shamt = lo4;
    case(opcode)
      `OPC_ADDI: begin
        alu_sel = `ALU_ADD; //redundant due to default
        rd_we = 1; //set write-enable to 1
        use_imm = 1; //use immediate set to 1
      end
      `OPC_SUBI: begin
        alu_sel = `ALU_SUB;
        rd_we = 1;
        use_imm = 1;
      end
      `OPC_ANDI: begin
        alu_sel = `ALU_AND;
        rd_we = 1;
        use_imm = 1;
      end
      `OPC_ORI: begin
        alu_sel = `ALU_OR;
        rd_we = 1;
        use_imm = 1;
      end
      `OPC_XORI: begin
        alu_sel = `ALU_XOR;
        rd_we = 1;
        use_imm = 1;
      end
      `OPC_SLLI: begin
        alu_sel = `ALU_SLL;
        rd_we = 1;
        use_imm = 1;
      end
      `OPC_SRLI: begin
        alu_sel = `ALU_SRL;
        rd_we = 1;
        use_imm = 1;
      end
      `OPC_ADD: begin
        alu_sel = `ALU_ADD; //redundant due to default statements
        rd_we = 1; //no need to set use_imm to 0 since that's the default case
      end
      `OPC_SUB: begin
        alu_sel = `ALU_SUB;
        rd_we = 1;
      end
      `OPC_AND: begin
        alu_sel = `ALU_AND;
        rd_we = 1;
      end
      `OPC_OR: begin
        alu_sel = `ALU_OR;
        rd_we = 1;
      end
      `OPC_XOR: begin
        alu_sel = `ALU_XOR;
        rd_we = 1;
      end
      default: begin
        alu_sel = `ALU_ADD;
        use_imm = 0;
        rd_we = 0;
      end
    endcase
  end  
  
  //is_regtype is 1 if opcode is one of the  r-type ones, 0 if not
  wire is_regtype = ((opcode >= `OPC_ADD) && (opcode <= `OPC_XOR));
  
  //makes sure that if rtype op, lo4 is rt, but if not, rt_data comes from r0 for the MUX
  wire [3:0] ra2 = is_regtype ? lo4 : 4'd0; 
 
  
  wire [15:0] rt_data;//if register type arithmetic, this is the third register data
  wire [15:0] rs_data;//secondary register data from rs_addr
  
  registerFile MEM(.CLK(CLK), .RST(RST), .RA1(rs_addr), .RA2(ra2), .RD1(rs_data), 
                   .RD2(rt_data), .WD(wb_rf_wd), .WE(wb_rf_we), .WA(wb_rf_wa));
  
  
  wire [15:0] imm_ext = {12'h000, lo4};//zero extend lo4 so it can be compared with rt_data
  
  //assign internal wire/reg values (post-decoding) to output wires
  assign alu_sel_out = alu_sel;
  assign roi_data_out = use_imm ? imm_ext: rt_data;
  assign use_imm_out = use_imm;//not really needed past the ternary statement reprenting MUX
  assign rs_data_out = rs_data;
  assign rd_we_out = rd_we;
  assign rd_addr_out = rd_addr;
  assign shamt_out = shamt; // shift amount (lo4) forwarded for shift ops
  
endmodule


//------------------------------------------------------------------------------
// Module: EX_STAGE
// Purpose: RTL block for the 16-bit 4-stage ALU-focused pipelined CPU.
// Notes  : See README for ISA + pipeline behavior (IF/ID/EX/WB).
//------------------------------------------------------------------------------
//EXECUTE STAGE:GETS OPERATION SELECT, SHIFT AMOUNT, SECONDARY REGISTER DATA, AND EITHER RT OR IMMEDIATE DATA FROM IDEX PIPELINE REGISTER IN ORDER TO CONDUCT OPERATIONS BASED ON SELECT. RAISES A ZERO FLAG IF THE RESULT IS ZERO OR INVALID SELECT INPUT
module EX_STAGE(input wire [2:0] IDEX_sel,
                input wire [3:0] IDEX_shamt,
                input wire [15:0] IDEX_rsdata,
                input wire [15:0] IDEX_roi,
                output reg [15:0] ex_result,
                output wire ex_zero);
  
  always @(*) begin
    case(IDEX_sel)
      `ALU_ADD: ex_result = IDEX_rsdata + IDEX_roi;
      `ALU_SUB: ex_result = IDEX_rsdata - IDEX_roi;
      `ALU_AND: ex_result = IDEX_rsdata & IDEX_roi;
      `ALU_OR: ex_result = IDEX_rsdata | IDEX_roi;
      `ALU_XOR: ex_result = IDEX_rsdata ^ IDEX_roi;
      `ALU_SLL: ex_result = IDEX_rsdata << IDEX_shamt;
      `ALU_SRL: ex_result = IDEX_rsdata >> IDEX_shamt;
      default: ex_result = 16'h0000;
    endcase
  end
  
  assign ex_zero = (ex_result == 16'h0000);
  
endmodule


//------------------------------------------------------------------------------
// Module: WB_STAGE
// Purpose: RTL block for the 16-bit 4-stage ALU-focused pipelined CPU.
// Notes  : See README for ISA + pipeline behavior (IF/ID/EX/WB).
//------------------------------------------------------------------------------
//WRITEBACK STAGE: RECIEVES RESULT OF ALU FROM EXWB PIPELINE REGISTER AND ASSIGNS IT TO THE OUTPUT THAT WILL GO TO THE DECODE STAGE'S REGISTER FILE INSTANTIATION
module WB_STAGE(input wire [15:0] EXWB_result,
                input wire EXWB_rdwe,
                input wire [3:0] EXWB_rd_addr,
                output wire [15:0] wb_rf_wd,
                output wire wb_rf_we,
                output wire [3:0] wb_rf_wa);
  assign wb_rf_wd = EXWB_result;
  assign wb_rf_we = EXWB_rdwe;
  assign wb_rf_wa = EXWB_rd_addr;
  
endmodule


//------------------------------------------------------------------------------
// Module: CPU_Pipeline
// Purpose: RTL block for the 16-bit 4-stage ALU-focused pipelined CPU.
// Notes  : See README for ISA + pipeline behavior (IF/ID/EX/WB).
//------------------------------------------------------------------------------
//TOP MODULE: PARAMETRIZED MODULE THAT INITIALIZES INSTRUCTION MEMORY DEPTH, INSTRUCTION CODE BIT LENGTH, AND THE ACTUAL INSTRUCTION FILE I'LL BE TESTING. STRUCTURALLY CONNECTS EACH MODULE VIA INSTANTIATION AND PIPELINE REGS' THAT UPDATE EVERY CLOCK CYCLE IF NOT RESET
module CPU_Pipeline #(parameter ROMDepth = 8,
                      parameter ROMFile = "instr_mem.txt",
                      parameter DWL = 16)
  (input CLK, RST);
  
  //wires to hold IF output values
  wire [$clog2(ROMDepth)-1:0] pc_out; 
  wire [DWL-1:0] instr_out;
  
  //IF stage instantiation, assigning its output values to the wires
  IF_STAGE #(.iMemDepth(ROMDepth), .InstrMem(ROMFile), .instrDWL(DWL))
  stage1(.CLK(CLK), .RST(RST), .pc_out(pc_out), .instr_out(instr_out));
  
  //pipeline reg's to input next stage
  reg [$clog2(ROMDepth)-1:0] PC_OUT; 
  reg [DWL-1:0] IFID_instr;
  
  always@(posedge CLK) begin //update pipeline reg's on each clock cycle(unless reset)
    if(RST) begin
      PC_OUT <= '0;
      IFID_instr <= '0;
    end else begin
      PC_OUT <= pc_out;
      IFID_instr <= instr_out;
    end
  end
  
  
  wire [15:0] wb_rf_wd; //will be updated after WB stage instantiation
  wire [3:0] wb_rf_wa; //will be updated after WB stage instantiation
  wire wb_rf_we; //will be updated after WB stage instantiation
  
  //wires to hold values of ID stage output values
  wire [2:0] alu_sel_out;
  wire rd_we_out;
  wire [3:0] rd_addr_out;
  wire [3:0] shamt_out;
  wire use_imm_out;
  wire [15:0] rs_data_out;
  wire [15:0] roi_out;
  
  //stage 2 decode instantiation
  ID_STAGE stage2 (.CLK(CLK), .RST(RST), .instr_in(IFID_instr), .wb_rf_wd(wb_rf_wd),
                   .wb_rf_wa(wb_rf_wa), .wb_rf_we(wb_rf_we), .alu_sel_out(alu_sel_out),
                   .rd_we_out(rd_we_out), 
                   .rd_addr_out(rd_addr_out), .shamt_out(shamt_out), .use_imm_out(use_imm_out),
                   .rs_data_out(rs_data_out), .roi_data_out(roi_out));
  
  //stage 2 output pipeline reg's
  reg [2:0] IDEX_sel;
  reg IDEX_rdwe;
  reg [3:0] IDEX_rd;
  reg [3:0] IDEX_shamt;
  reg [15:0] IDEX_rsdata;
  reg [15:0] IDEX_roi;
  
  always @(posedge CLK) begin //update pipeline reg's on clock cycle(unless RST)
    if(RST) begin
      IDEX_sel <= 3'b000;
      IDEX_rdwe <= 1'b0;
      IDEX_rd <= 4'b0000;
      IDEX_shamt <= 4'h0;
      IDEX_rsdata <= 16'h0000;
      IDEX_roi <= 16'h0000;
    end else begin
      IDEX_sel <= alu_sel_out;
      IDEX_rdwe <= rd_we_out;
      IDEX_rd <= rd_addr_out;
      IDEX_shamt <= shamt_out;
      IDEX_rsdata <= rs_data_out;
      IDEX_roi <= roi_out;
    end
  end
  
  //wires to hold values of execute stage output
  wire [15:0] ex_result;
  wire ex_zero;
  
  //stage 3 execute instantiation assigning outputs to wires
  EX_STAGE stage3(.IDEX_sel(IDEX_sel), .IDEX_shamt(IDEX_shamt), .IDEX_rsdata(IDEX_rsdata),
                  .IDEX_roi(IDEX_roi), .ex_result(ex_result), .ex_zero(ex_zero));
  
  reg [15:0] EXWB_result;//reg to hold stage 3 result value before inputting stage 4(clock cycle 
  
  reg [3:0] EXWB_rd_addr;//reg to delay IDEX_rd and rdwe so it's not a clock cycle ahead
  reg EXWB_rdwe;//considering these two skip the EX stage and go straight to EXWB pipeline
  
  always @(posedge CLK) begin
    if(RST) begin
      EXWB_result <= 16'h0000;
      EXWB_rd_addr <= 4'h0;
      EXWB_rdwe <= 1'b0;
    end else begin
      EXWB_result <= ex_result;
      EXWB_rd_addr <= IDEX_rd; //rd and rdwe's IDEX pipeline regs' pipelined again for proper timing
      EXWB_rdwe <= IDEX_rdwe;
    end
  end
  
  //instantiate stage 4, which will give the inputs of stage2(decode) their values to store in the register file
  WB_STAGE stage4(.EXWB_result(EXWB_result), .EXWB_rd_addr(EXWB_rd_addr),
                  .EXWB_rdwe(EXWB_rdwe), .wb_rf_wd(wb_rf_wd), .wb_rf_wa(wb_rf_wa),
                  .wb_rf_we(wb_rf_we));
  
      
endmodule

