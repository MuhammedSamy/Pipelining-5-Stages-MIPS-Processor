`define MEM_CELL_SIZE 32
`define DATA_MEM_SIZE 1024
`define INSTR_MEM_SIZE 1024
// Wire widths
`define WORD_LEN 32
`define REG_FILE_ADDR_LEN 5
`define EXE_CMD_LEN 4
`define FORW_SEL_LEN 2
`define OP_CODE_LEN 6

// Memory constants
`define DATA_MEM_SIZE 1024
`define INSTR_MEM_SIZE 1024
`define REG_FILE_SIZE 32


// To be used inside controller.v
`define OP_NOP 6'b000000
`define OP_ADD 6'b000001
`define OP_SUB 6'b000011
`define OP_AND 6'b000101
`define OP_OR 6'b000110
`define OP_NOR 6'b000111
`define OP_XOR 6'b001000
`define OP_SLA 6'b001001
`define OP_SLL 6'b001010
`define OP_SRA 6'b001011
`define OP_SRL 6'b001100
`define OP_ADDI 6'b100000
`define OP_SUBI 6'b100001
`define OP_LD 6'b100100
`define OP_ST 6'b100101
`define OP_BEZ 6'b101000
`define OP_BNE 6'b101001
`define OP_JMP 6'b101010

// To be used in side ALU
`define EXE_ADD 4'b0000
`define EXE_SUB 4'b0010
`define EXE_AND 4'b0100
`define EXE_OR 4'b0101
`define EXE_NOR 4'b0110
`define EXE_XOR 4'b0111
`define EXE_SLA 4'b1000
`define EXE_SLL 4'b1000
`define EXE_SRA 4'b1001
`define EXE_SRL 4'b1010
`define EXE_NO_OPERATION 4'b1111 // for NOP, BEZ, BNQ, JMP

// To be used in conditionChecker
`define COND_JUMP 2'b10
`define COND_BEZ 2'b11
`define COND_BNE 2'b01
`define COND_NOTHING 2'b00

module mux #(parameter integer LENGTH = 32 ) (in1, in2, sel, out);
  input sel;
  input [LENGTH-1:0] in1, in2;
  output [LENGTH-1:0] out;

  assign out = (sel == 0) ? in1 : in2;
endmodule // mux

module mux_3input #(parameter integer LENGTH = 32 ) (in1, in2, in3, sel, out);
  input [LENGTH-1:0] in1, in2, in3;
  input [1:0] sel;
  output [LENGTH-1:0] out;

  assign out = (sel == 2'd0) ? in1 :
               (sel == 2'd1) ? in2 : in3;
endmodule // mux_3_inputs

module adder (in1, in2, out);
  input [31:0] in1, in2;
  output [31:0] out;

  assign out = in1 + in2;
endmodule // adder

module register (clk, reset, writeEn, regIn, regOut);  //PC
  input clk, reset, writeEn;
  input [31:0] regIn;
  output reg [31:0] regOut;

  always @ (posedge clk) begin
    if (reset == 1) regOut <= 0;
    else if (writeEn) regOut <= regIn;
  end
endmodule // register

module regFile (clk, rst, src1, src2, dest, writeVal, writeEn, reg1, reg2);
  input clk, rst, writeEn;
  input [`REG_FILE_ADDR_LEN-1:0] src1, src2, dest;
  input [`WORD_LEN-1:0] writeVal;
  output [`WORD_LEN-1:0] reg1, reg2;

  reg [`WORD_LEN-1:0] regMem [0:`REG_FILE_SIZE-1];
  integer i;

  always @ (negedge clk) begin
    if (rst) begin
      for (i = 0; i < `WORD_LEN; i = i + 1)
        regMem[i] <= 0;
	    end

    else if (writeEn) regMem[dest] <= writeVal;
    regMem[0] <= 0;
  end

  assign reg1 = (regMem[src1]);
  assign reg2 = (regMem[src2]);
endmodule // regFile


module DataMemory (MemWrite,Memread,address,writeData,clk,reset,readData);
 output reg [31:0] readData;

  input MemWrite,Memread,clk,reset;
  input [31:0] address,writeData;
integer i ;
 reg [`MEM_CELL_SIZE-1:0] memory [0:`DATA_MEM_SIZE-1];   //MEM_CELL = 32 , DATA_MEM_SIZE = 1024

always @ (posedge clk) begin
    if (reset)
      for (i = 0; i < `DATA_MEM_SIZE; i = i + 1)
       memory[i] <= 0;
    else if (MemWrite==1)
        memory[address]<=writeData;
    end
  
    always@(address or Memread)
    begin
      
      if(Memread==1)
        readData=(address > 1023 ) ? 0 : memory[address];
      
    end
endmodule // dataMem



module controller (opCode, branchEn, EXE_CMD, Branch_command, Is_Imm, ST_or_BNE, WB_EN, MEM_R_EN, MEM_W_EN, hazard_detected);
//EXE_CMD = Control signal of ALU , IS IMM = ITYPE Instruction , ST_OR_BNE = Storeword or branch non equ , WB = Writeback ,branch_comand = input to condition checker
  input hazard_detected;
  input [`OP_CODE_LEN-1:0] opCode; //op code is 5 bits
  output reg branchEn;
  output reg [`EXE_CMD_LEN-1:0] EXE_CMD; //SIGNALCONTROL TO ALU IS 4 BIT
  output reg [1:0] Branch_command; //input to condition checker 
  output reg Is_Imm, ST_or_BNE, WB_EN, MEM_R_EN, MEM_W_EN;

  always @ ( * ) begin
    if (hazard_detected == 0) begin
      {branchEn, EXE_CMD, Branch_command, Is_Imm, ST_or_BNE, WB_EN, MEM_R_EN, MEM_W_EN} <= 0;
      case (opCode)
        // operations writing to the register file
        `OP_ADD: begin EXE_CMD <= `EXE_ADD; WB_EN <= 1; end
        `OP_SUB: begin EXE_CMD <= `EXE_SUB; WB_EN <= 1; end
        `OP_AND: begin EXE_CMD <= `EXE_AND; WB_EN <= 1; end
        `OP_OR:  begin EXE_CMD <= `EXE_OR;  WB_EN <= 1; end
        `OP_NOR: begin EXE_CMD <= `EXE_NOR; WB_EN <= 1; end
        `OP_XOR: begin EXE_CMD <= `EXE_XOR; WB_EN <= 1; end
        `OP_SLA: begin EXE_CMD <= `EXE_SLA; WB_EN <= 1; end
        `OP_SLL: begin EXE_CMD <= `EXE_SLL; WB_EN <= 1; end
        `OP_SRA: begin EXE_CMD <= `EXE_SRA; WB_EN <= 1; end
        `OP_SRL: begin EXE_CMD <= `EXE_SRL; WB_EN <= 1; end
        // operations using an immediate value embedded in the instruction
        `OP_ADDI: begin EXE_CMD <= `EXE_ADD; WB_EN <= 1; Is_Imm <= 1; end
        `OP_SUBI: begin EXE_CMD <= `EXE_SUB; WB_EN <= 1; Is_Imm <= 1; end
        // memory operations
        `OP_LD: begin EXE_CMD <= `EXE_ADD; WB_EN <= 1; Is_Imm <= 1; ST_or_BNE <= 1; MEM_R_EN <= 1; end
        `OP_ST: begin EXE_CMD <= `EXE_ADD; Is_Imm <= 1; MEM_W_EN <= 1; ST_or_BNE <= 1; end
        // branch operations
        `OP_BEZ: begin EXE_CMD <= `EXE_NO_OPERATION; Is_Imm <= 1; Branch_command <= `COND_BEZ; branchEn <= 1; end
        `OP_BNE: begin EXE_CMD <= `EXE_NO_OPERATION; Is_Imm <= 1; Branch_command <= `COND_BNE; branchEn <= 1; ST_or_BNE <= 1; end
        `OP_JMP: begin EXE_CMD <= `EXE_NO_OPERATION; Is_Imm <= 1; Branch_command <= `COND_JUMP; branchEn <= 1; end
        default: {branchEn, EXE_CMD, Branch_command, Is_Imm, ST_or_BNE, WB_EN, MEM_R_EN, MEM_W_EN} <= 0;
      endcase
    end

    else if (hazard_detected ==  1) begin
      // preventing any writes to the register file or the memory
      {EXE_CMD, WB_EN, MEM_W_EN} <= 0;
    end
  end
endmodule // controller

module conditionChecker (reg1, reg2, cuBranchComm, brCond); //jump /bezero /bne 
  input [31: 0] reg1, reg2;
  input [1:0] cuBranchComm; //last two bit of branch op code
  output reg brCond;

  always @ ( * ) begin
    case (cuBranchComm)
      `COND_JUMP: brCond <= 1;
      `COND_BEZ: brCond <= (reg1 == 0) ? 1 : 0;
      `COND_BNE: brCond <= (reg1 != reg2) ? 1 : 0;
      default: brCond <= 0;
    endcase
  end
endmodule // conditionChecker

module signExtend (in, out);
  input [15:0] in;
  output [31:0] out;

  assign out = (in[15] == 1) ? {16'b1111111111111111, in} : {16'b0000000000000000, in};
endmodule // signExtend

module ALU (val1, val2, EXE_CMD, aluOut);
  input [`WORD_LEN-1:0] val1, val2;
  input [`EXE_CMD_LEN-1:0] EXE_CMD;
  output reg [`WORD_LEN-1:0] aluOut;

  always @ ( * ) begin
    case (EXE_CMD)
      `EXE_ADD: aluOut <= val1 + val2;
      `EXE_SUB: aluOut <= val1 - val2;
      `EXE_AND: aluOut <= val1 & val2;
      `EXE_OR: aluOut <= val1 | val2;
      `EXE_NOR: aluOut <= ~(val1 | val2);
      `EXE_XOR: aluOut <= val1 ^ val2;
      `EXE_SLA: aluOut <= val1 << val2;
      `EXE_SLL: aluOut <= val1 <<< val2;
      `EXE_SRA: aluOut <= val1 >> val2;
      `EXE_SRL: aluOut <= val1 >>> val2;
      default: aluOut <= 0;
    endcase
  end
endmodule // ALU
