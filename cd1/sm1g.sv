
// A fun system verilog file
`timescale 1ns/10ps

`include "sm_gates.sv"


module top();

logic clk,rst;
reg [2:0] outx,outy;

initial begin
	clk=1;
	repeat(100) begin
		#5 clk=~clk;
	end
end

initial begin
	rst=1;
	repeat(3) @(posedge(clk));
	#1 rst=0;
end

initial begin
	$dumpfile("sm1.vcd");
	$dumpvars(9,top);
	
end

sm bob(clk,rst,outx,outy);

endmodule : top

