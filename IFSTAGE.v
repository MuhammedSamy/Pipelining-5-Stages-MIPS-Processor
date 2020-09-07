`define WORD_LENGHT 32
module IFStage (clk, reset, brTaken, brOffset, Haz_Det, PC, instruction);
  input clk, reset, brTaken, Haz_Det;
  input [31:0] brOffset;
  output [31:0] PC, instruction;

  wire [31:0] adderIn1, adderOut, brOffserTimes4;

  mux #(.LENGTH(`WORD_LENGHT))  adderInput  (
    .in1(32'd4),
    .in2(brOffserTimes4),
    .sel(brTaken),
    .out(adderIn1)
  );

  adder add4 (
    .in1(adderIn1),
    .in2(PC),
    .out(adderOut)
  );

  register PCReg (
    .clk(clk),
    .reset(reset),
    .writeEn(~Haz_Det),
    .regIn(adderOut),
    .regOut(PC)
  );

  instructionMem instructions (
    .rst(reset),
    .addr(PC),
    .instruction(instruction)
  );

  assign brOffserTimes4 = brOffset << 2;
endmodule // IFStage

