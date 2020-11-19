module fifo_smit (clk,reset,w_en,r_en,datain,dataout,empty,full);
input clk,reset;
input w_en,r_en;
input  [39:0] datain;
output  [39:0] dataout;
output reg empty,full;
reg [4:0] count,w_ptr,r_ptr;
reg [31:0] [39:0] mem ;

always @ (posedge clk or posedge reset)begin
	if (reset) begin
		r_ptr <= 0;
	end else if (r_en && empty== 1'b0)begin
		r_ptr <= #1 r_ptr +1;
	end
end

assign dataout =  mem[r_ptr];


always @ (posedge clk or posedge reset)begin
	if (reset) begin
		w_ptr <= 0;
	end else if(w_en & full== 1'b0 ) begin
		w_ptr <= #1 w_ptr +1;
	end
end

always @ (posedge clk or posedge reset)begin
	if(reset)begin
		count <= 0;
	end else begin
		case ({r_en,w_en})
		2'b00: count <= #1 count;
		2'b10 : begin if (!empty)
			  count <= #1 count-1;
		end
		2'b01 : begin if (!full)
			count <= #1 count+1;
		end
		2'b11 : count <= #1 count;
		endcase
	end
end

always @ (posedge clk or posedge reset) begin
	if(reset)begin
		mem <= 40'h00000;
	end else if (w_en && full == 1'b0)begin
		mem[w_ptr] <= #1 datain;
	end
end

always @ (count)begin
	if(count == 0)begin
		empty = 1'b1;
	end else begin
		empty = 1'b0;
	end
end

always @ (count) begin
	if (count < 31)begin	//waste one space
		full = 1'b0;
	end else begin
		full = 1'b1;
	end
end

endmodule


