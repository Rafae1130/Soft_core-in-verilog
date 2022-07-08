//`timescale 1 ns/100 ps

module Soft_core_tb();

reg [3:0]KEY;
wire [31:0] out_reg;
wire [6:0]  HEX0,HEX1, HEX2, HEX3, HEX4, HEX5;
wire [9:0]LEDR;

assignment2 uut(.KEY(KEY), .LEDR(LEDR), .out_reg(out_reg), .HEX0(HEX0), .HEX1(HEX1), .HEX2(HEX2),.HEX3(HEX3) ,.HEX4(HEX4), .HEX5(HEX5));

integer j;

initial 
begin

KEY[0]=0;
KEY[1]=0;
#100

for(j=0;j<=70;j=j+1)
begin
	#500;
	KEY[0]=~KEY[0];
	KEY[1]=1;
end
end



endmodule

