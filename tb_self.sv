// This is a simple test bench for the permutation block
//
`timescale 1ns/10ps

`include "perm.sv"
`include "m55.sv"

module tb_self();
reg clk,reset;
reg pushin,firstin,firstout,firstoutH;
wire stopin;
reg [63:0] din;
reg [5:0] dix;	// data index for 1600 bits

//changed to wires
wire [2:0] m1ax,m1ay,m1wx,m1wy,m2ax,m2ay,m2wx,m2wy,m3ax,m3ay,m3wx,m3wy,m4ax,m4ay,m4wx,m4wy;
wire m1wr,m2wr,m3wr,m4wr;
wire [63:0] m1rd,m1wd,m2rd,m2wd,m3rd,m3wd,m4rd,m4wd;
reg errpos=0;

wire pushout;
reg stopout;
reg pushoutH;
wire [63:0] dout;
reg [63:0] doutH;

perm_blk p(clk,reset,pushin,stopin,firstin,din,
    m1ax,m1ay,m1rd,m1wx,m1wy,m1wr,m1wd,
    m2ax,m2ay,m2rd,m2wx,m2wy,m2wr,m2wd,
    m3ax,m3ay,m3rd,m3wx,m3wy,m3wr,m3wd,
    m4ax,m4ay,m4rd,m4wx,m4wy,m4wr,m4wd,
    pushout,stopout,firstout,dout);


m55 m1(clk,reset,m1ax,m1ay,m1rd,m1wx,m1wy,m1wr,m1wd);
m55 m2(clk,reset,m2ax,m2ay,m2rd,m2wx,m2wy,m2wr,m2wd);
m55 m3(clk,reset,m3ax,m3ay,m3rd,m3wx,m3wy,m3wr,m3wd);
m55 m4(clk,reset,m4ax,m4ay,m4rd,m4wx,m4wy,m4wr,m4wd);

initial begin
	clk=0;
	reset=1;
	pushin=0;
	firstin=0;
	#10;
	reset=0;
	pushin=1;
	firstin=1;
	din=64'h60a636261;
	#10;
	firstin=0;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h8000000000000000;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	din=64'h0;
	#10;
	pushin=0;
	
	#100000;
	$finish;
end

initial begin
    clk=1;
    repeat(100000) begin
        #5 clk=~clk;
        #5 clk=~clk;
    end
    $display("ran out of clocks");
    $finish;
end

//To monitor m1
//initial $monitor ("%0t",$time,"clk:%b,p%b,f%b,din%h,wr:%b,wx:%d,wy:%d,wd:%h,cs:%b ",clk,pushin,firstin,din,m1wr,m1wx,m1wy,m1wd,p.cs);

initial begin
    //repeat(10_000_000) @(posedge(clk));
    $dumpfile("self.vcd");
    $dumpvars(9,tb_self);
    //repeat(100000) @(posedge(clk));
    #5;
    //$dumpoff;

end

endmodule : tb_self
