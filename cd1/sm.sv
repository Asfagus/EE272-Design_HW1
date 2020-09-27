module sm(input reg clk, input logic rst,output reg [2:0] x, 
	output reg[2:0] y);

reg [2:0] vx,vx_d,vy,vy_d;

enum reg [2:0] {
	R,
	incx,
	incy
} cs,ns;

always @(*) begin
	ns=cs;
	vx_d=vx;
	vy_d=vy;
	x=vx;
	y=vy;
	case(cs)
		R: begin
			vx_d=0;
			vy_d=0;
			ns=incx;
		end
		
		incx: begin
			vx_d=vx+1;
			if(vx_d>=4) ns=incy;
		end
		
		incy: begin
			vx_d=0;
			vy_d=vy+1;
			if(vy>=4) begin
				vy_d=0;
				ns=incx;
			end else ns=incx;
		end
		
		default:
			$display("I'm lost with %b",cs);
	endcase

end


always @(posedge(clk) or posedge(rst)) begin
	if(rst) begin
		cs<=R;
		vx<=0;
		vy<=0;
	end else begin
		cs<= #1 ns;
		vx<= #1 vx_d;
		vy<= #1 vy_d;
	end
end


endmodule : sm
