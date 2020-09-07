`timescale 1ns/1ns

module testbench ();
  reg clk,rst, forwarding_EN;
  MIPS_Processor top_module (clk, rst, forwarding_EN);

  initial begin
    clk=1;

    repeat(500)begin #50 clk=~clk ;
  end
end

  initial begin
    rst = 1;
    forwarding_EN = 0;
    #100
    rst = 0;
  end
endmodule // test
