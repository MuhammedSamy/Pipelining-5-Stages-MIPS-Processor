module MemoryStage (clk, reset, MEM_R_EN, MEM_W_EN, ALU_result, ST_value, dataMem_out);

  input clk, reset, MEM_R_EN, MEM_W_EN;
  input [31:0] ALU_result, ST_value;
  output [31:0]  dataMem_out;

  DataMemory DataMemory (
    .clk(clk),
    .reset(reset),
    .MemWrite(MEM_W_EN),
    .Memread(MEM_R_EN),
    .address(ALU_result),
    .writeData(ST_value), //reg of stage value
    .readData(dataMem_out)
  );
endmodule

module WBStage (MEM_R_EN, dataMem_out, ALUresult, WB_res);
  input MEM_R_EN;
  input [31:0] dataMem_out, ALUresult;
  output [31:0] WB_res;

  assign WB_res = (MEM_R_EN) ? dataMem_out : ALUresult;
endmodule // WBStage