module soft_core(KEY, LEDR, out_reg, HEX0,HEX1,HEX2, HEX3 ,HEX4, HEX5);

input [3:0]KEY;

wire [31:0] read_val_A;
wire [31:0] read_val_B;	
wire  [2:0]  control;
wire [7:0]i;
wire [38:0]inst;
 
wire wr_en_A;
wire wr_en_B;
wire [31:0]write_val;
wire [3:0]addr_A;
wire [3:0]addr_B;
wire zero_flag;
wire clk, reset;

output [31:0] out_reg;
output [6:0]  HEX0,HEX1, HEX2, HEX3, HEX4, HEX5;
output [9:0]LEDR;


assign clk=KEY[0];
assign reset= KEY[1];


//counter for counting the instruction queue address
counter c1(.clk(clk), .reset(reset), .i(i));

//instruction queue where memory will be loaded
inst_queue q1(.clk(clk), .i(i), .inst(inst));


//decoder will decode the loaded instruction
decoder d1(.inst(inst), .wr_en_A(wr_en_A), .wr_en_B(wr_en_B), .write_val(write_val), .addr_A(addr_A), .addr_B(addr_B), .control(control));

//first register file 
reg_file reg_A(.clk(clk), .addr(addr_A), .write_val(write_val), .read_val(read_val_A), .wr_en(wr_en_A));
//second register file 
reg_file reg_B(.clk(clk), .addr(addr_B), .write_val(write_val), .read_val(read_val_B), .wr_en(wr_en_B));

//ALU for performing required arithmetic funtions
ALU a1(.clk(clk),.A(read_val_A),.B(read_val_B), .control(control), .out_reg(out_reg) ,.zero_flag(zero_flag));

//for displaying the output on FPGA
decoder_led out(.A(out_reg[3:0]), .HEX0(HEX0));
decoder_led out1(.A(out_reg[7:4]), .HEX0(HEX1));
decoder_led out2(.A(out_reg[11:8]), .HEX0(HEX2));
decoder_led out3(.A(out_reg[15:12]), .HEX0(HEX3));
decoder_led out4(.A(i[3:0]), .HEX0(HEX4));
decoder_led out5(.A(i[7:4]), .HEX0(HEX5));
assign LEDR[0]=zero_flag;


endmodule


//module for instruction queue

module inst_queue(clk, i,inst);

	input clk;
	output reg [38:0] inst=0;
	reg [38:0] memory [0:63];
	input [7:0]i;
	
	initial begin
//loading instructions
	$readmemb("E:\\6th_semester\\DSD\\assginment\\instructions1.dat", memory);
	
	end
//reading from instruction queue
	always @(posedge clk)
		inst<= memory[i];
	
endmodule


// module for counter

module counter(clk, reset, i);

	input clk, reset;
	output reg [7:0]i=0;
	 
	
	always @(posedge clk)
	begin
	if(!reset)
		i<=0;
	else 
		i=i+1;
	end
endmodule


//module for decoder


module decoder(inst, wr_en_A, wr_en_B, write_val, addr_A, addr_B, control);

input [38:0]inst;


output reg wr_en_A;
output reg wr_en_B;
output reg [31:0]write_val;
output reg [2:0]control;
output reg [3:0]addr_A;
output reg [3:0]addr_B;

//producing singals according to last 3 bits of the instruction
always @(inst)
begin
	case(inst[38:36])
		3'b000: begin 
					control=3'b000;
					wr_en_A=0;
					wr_en_B=0;
					addr_A=inst[35:32];
					addr_B=inst[31:28];
				  end
		3'b001: begin
					control=3'b001;
					wr_en_A=1;
					wr_en_B=0;
					write_val=inst[31:0];
					addr_A=inst[35:32];
				  end
		3'b010: begin
					control=3'b010;
					wr_en_A=0;
					wr_en_B=1;
					write_val=inst[31:0];
					addr_B=inst[35:32];
				  end
		3'b011: begin
					control=3'b011;
					wr_en_A=0;
					wr_en_B=0;
					addr_A=inst[35:32];
					addr_B=inst[31:28];
				  end

		3'b100: begin
					control=3'b100;
					wr_en_A=0;
					wr_en_B=0;
					addr_A=inst[35:32];
					addr_B=inst[31:28];
				  end
		3'b101: begin
					control=3'b101;
					wr_en_A=0;
					wr_en_B=0;
					addr_A=inst[35:32];
					addr_B=inst[31:28];
				  end
		3'b110: begin
					control=3'b110;
					wr_en_A=0;
					wr_en_B=0;
					addr_A=inst[35:32];
					addr_B=inst[31:28];
				  end
		3'b111: begin
					control=3'b111;
					wr_en_A=0;
					wr_en_B=0;
					addr_A=inst[35:32];
					addr_B=inst[31:28];
				  end
	endcase
end


endmodule


//module for ALU

module ALU(clk,A,B, control, out_reg ,zero_flag);

input clk;
input [31:0]A;
input [31:0]B;

input [2:0]control;
output reg zero_flag;
reg [31:0]out;
output reg [31:0]out_reg;


//perfoming operation on operands according to control signal coming from decoder
always @(A,B,control)
begin
	case(control)
	3'b011: out=A+B;
	3'b100: out=A-B;
	3'b101: out=A|B;
	3'b110: out=A&B;
	3'b111: out=A<<B;
	default: out=0;
	endcase
end

//producing the zero flag
always @(posedge clk)
begin
	if(out==0 || control==3'b000)
		zero_flag<=1'b1;
	else
		zero_flag<=1'b0;
end

//registering the output of ALU
always @(posedge clk)
begin
	if(control==3'b000)
		out_reg<=0;
	else
		out_reg<=out;
end
endmodule






//module for register files

module reg_file(clk, addr, write_val, read_val, wr_en);

input 		clk;
input 		[3:0] addr;
input 		[31:0] write_val;
output      [31:0] read_val;
input 		wr_en;


reg [31:0] register[0:15];

always @(posedge clk)

//synchronous writing to register file
begin
	if(wr_en)
	begin
		register[addr]<=write_val;
	end
end
//asynchronous reading from register file
	assign read_val=register[addr];

endmodule




//module for LED HEX display on FPGA
module decoder_led(A, HEX0);
input [3:0]A;

output reg [6:0] HEX0;

always @(A[3:0])
begin 

case(A[3:0])
	4'b0000: HEX0= 7'b1000000;
	4'b0001: HEX0= 7'b1111001;
	4'b0010: HEX0= 7'b0100100;
	4'b0011: HEX0= 7'b0110000;
	4'b0100: HEX0= 7'b0011001;
	4'b0101: HEX0= 7'b0010010;
	4'b0110: HEX0= 7'b0000010;
	4'b0111: HEX0= 7'b1111000;
	4'b1000: HEX0= 7'b0000000;
	4'b1001: HEX0= 7'b0010000;
	4'b1010: HEX0= 7'b0001000;
	4'b1011: HEX0= 7'b0000011;
	4'b1100: HEX0= 7'b1000110;
	4'b1101: HEX0= 7'b0100001;
	4'b1110: HEX0= 7'b0000110;
	4'b1111: HEX0= 7'b0001110;
endcase
end

endmodule

