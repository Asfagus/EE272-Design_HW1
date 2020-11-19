module perm_blk(
	input clk, input rst, 
	input pushin,
	output reg stopin, 
	input firstin, input [63:0] din,		       //firstin:first data 
	output reg [2:0] m1rx, output reg [2:0] m1ry, input [63:0] m1rd,		       //read mem1
	output reg [2:0] m1wx, output reg [2:0] m1wy, output reg m1wr, output logic [63:0] m1wd, //write mem1
	output reg [2:0] m2rx, output reg [2:0] m2ry, input [63:0] m2rd,		       //read mem2
	output reg [2:0] m2wx, output reg [2:0] m2wy, output reg m2wr, output reg [63:0] m2wd, //write mem2
	output reg [2:0] m3rx, output reg [2:0] m3ry, input [63:0] m3rd,		       //read mem3
	output reg [2:0] m3wx, output reg [2:0] m3wy, output reg m3wr, output reg [63:0] m3wd, //write mem3
	output reg [2:0] m4rx, output reg [2:0] m4ry, input [63:0] m4rd,		       //read mem4
	output reg [2:0] m4wx, output reg [2:0] m4wy, output reg m4wr, output reg [63:0] m4wd, //write mem4
	output reg pushout, input stopout, output reg firstout, output reg [63:0] dout 
);
const reg [0:23][63:0] cmx={
    64'h0000000000000001, 64'h0000000000008082,
    64'h800000000000808a, 64'h8000000080008000,
    64'h000000000000808b, 64'h0000000080000001,
    64'h8000000080008081, 64'h8000000000008009,
    64'h000000000000008a, 64'h0000000000000088,
    64'h0000000080008009, 64'h000000008000000a,
    64'h000000008000808b, 64'h800000000000008b,
    64'h8000000000008089, 64'h8000000000008003,
    64'h8000000000008002, 64'h8000000000000080,
    64'h000000000000800a, 64'h800000008000000a,
    64'h8000000080008081, 64'h8000000000008080,
    64'h0000000080000001, 64'h8000000080008008    
};
    
//rotation for rho  	//y,x
const reg [0:4] [0:4] [63:0] rot ={
	   	64'd0,64'd1,64'd62,64'd28,64'd27,
	   	64'd36,64'd44,64'd6,64'd55,64'd20,
	   	64'd3,64'd10,64'd43,64'd25,64'd39,
	   	64'd41,64'd45,64'd15,64'd21,64'd8,
	   	64'd18,64'd2,64'd61,64'd56,64'd14
	};
	
//hardcoding d values	
const reg [0:4][3:0]cxminus1={4'd4,4'd0,4'd1,4'd2,4'd3};
const reg [0:4][3:0]cxplus1={4'd1,4'd2,4'd3,4'd4,4'd0};
reg [3:0] i,i_d,j_d,j;//counter for cxminus1 

//round keeps track of 24 rounds
reg [4:0] round,round_d;

//c_round keeps track of C rounds
reg [2:0] c_round,c_round_d;

//combinational reg for m1w
logic [2:0] m1wx_d, m1wy_d;  //write mem1_d
reg [2:0] m1rx_d, m1ry_d; 

//combinational reg for m2
logic [2:0] m2wy_d,m2wx_d;
reg [2:0] m2rx_d, m2ry_d; 
//reg [2:0] m2wr_d;

//combinational reg for m4
logic [2:0] m3wy_d,m3wx_d;
reg [2:0] m3rx_d, m3ry_d; 
//reg [2:0] m3wr_d;

//combinational reg for m4
logic [2:0] m4wy_d,m4wx_d;
reg [2:0] m4rx_d, m4ry_d; 

//reg [1599:0] S;	//modified bit version of input
reg [63:0] r1,r1_d,r2,r2_d,r3,r3_d;

//To check if m1 is done
reg m1_done,m1_done_d;

//Stopin ff
reg stopin_d;

//theta start
reg theta_start;

//accept next data
reg next_data,next_data_d;

//chi start
reg chi_done,chi_done_d;
reg [1:0] ns_chi,cs_chi;
reg [2:0] m2rx_temp; 
integer chi_ctr,chi_ctr_d;

//rho rotation
reg [127:0] rho_rot;

//Enum for currentstate, nextstate
enum reg [1:0] {
	reset_state,
	load_state,done_state
} cs,ns;

enum reg [4:0]{
	reset_perm,copym1m2,findC,dummy,findD,findD1,dotheta,dotheta1,dorho,donothing,copym3tom2,dochi1,dochi2,dochi3,doiota,donothing2,copym3m1,doout,copym3m4,done_perm,doneoutput
}cs_perm,ns_perm;

enum reg [1:0]{reset_out,copym3m4_out,working_out,done_out}cs_out,ns_out;

//perm is done
reg perm_finish;
reg perm_copy;//copy perm
reg output_working,output_working_d;//tells if output is complete or going on

//for simulation
//assign dout=0;
//assign firstout=0;
//assign pushout=0;

//Always comb to put input data in mem1
always @ (*)begin
	ns=cs;
	m1wx_d=m1wx;
	m1wy_d=m1wy;
	stopin_d=stopin;
	m1wr=0;
	m1wd=0;
	m1_done_d=m1_done;
	ns_perm=cs_perm;
	//$display("ns:%d",ns);
	//perm regs
	r1_d=r1;
	m1rx_d=m1rx;
	m1ry_d=m1ry;
	c_round_d=c_round;
	r2_d=r2;
	m2wr=0;
	m2wd=0;
	m2wx_d=m2wx;
	m2wy_d=m2wy;
	m3wr=0;
	m3wd=0;
	m3wx_d=m3wx;
	m3wy_d=m3wy;
	m2rx_d=m2rx;
	m2ry_d=m2ry;
	m3rx_d=m3rx;
	m3ry_d=m3ry;
	r3_d=r3;
	theta_start=0;
	rho_rot=0;
	chi_done_d=0;
	ns_chi=cs_chi;
	m2rx_temp=0;
	chi_ctr_d<=chi_ctr;
	round_d=round;
	
	m4wd=0;
	m4wr=0;
	m4wx_d=m4wx;
	m4wy_d=m4wy;
	m4rx_d=m4rx;
	m4ry_d=m4ry;
	pushout=0;
	firstout=0;
	dout=0;
	next_data_d=next_data;
	i_d=i;
	j_d=j;
	perm_finish=0;
	ns_out=cs_out;
	perm_copy=0;
			//To put data in 25 locations mem1 
		case(cs)
			reset_state:begin
				//Start reset wx,wy in mem1
				//m1wr=1;
				m1wy_d=0;
				m1wx_d=0;
				//m1wd=din;
				m1rx_d=0;
				m1ry_d=0;
				
				
				//ns=load_state;
				
				if(firstin && pushin && !m1_done) begin
					ns=load_state;
					m1rx_d=m1rx+1;
					m1wx_d=m1wx+1;
					m1wr=1'b1;
					m1wd=din;
				end
				else ns=reset_state;
				
			end
			load_state:begin	
			if (pushin)begin		
				//Increment m1wx until 5 steps
				if (m1wx>=4)begin
					m1wx_d=0;
					if (m1wy>=4)
						m1wy_d=0;
					else begin
						m1wy_d=m1wy+1; 
					end
				end
				else begin
					m1wx_d=m1wx+1;			
				end
				
				m1wr=1'b1;
				m1wd=din;
				//m1wr=0;
				 
				if (m1wx==4&&m1wy==4)begin
					stopin_d=1;
			//		$display ($time,"Done 25 locations!m1wx:%b,m1wy:%b",m1wx,m1wy);	
					m1_done_d=1;  //1
					ns=done_state;							
				end
				else begin 
					//stopin_d=0;//to avoid latch
					m1_done_d=0;
					ns=load_state;
				end
			end
			end
			done_state:begin
				if(next_data) begin
					stopin_d=0;
					next_data_d=0;
					ns=reset_state;
				end
				//if (stopin==0)
				//$display("1:rx:%dry:%drd:%h",m1rx_d,m1ry_d,m1rd);
			end

			default: begin
				m1wd=0;
				m1wy_d=0;
				m1wx_d=0;	
			end
		endcase
	
	if(next_data) 
		stopin_d=0;
	
	//Do Perm
	case (cs_perm)
		reset_perm:begin
			round_d=0;
			c_round_d=0;		//Have not reset this at top, might create a latch
			if(m1_done&&cs==2)begin
				ns_perm=copym1m2;
			m1rx_d=0;
			m1ry_d=0;
			m2rx_d=0;
			m2ry_d=0;
			m3rx_d=0;
			m3ry_d=0;
			m2wx_d=0;
			m2wy_d=0;	
			r1_d=0;	
			r2_d=0;
			r3_d=0;	
			m2wr=0;	
			m3wr=0;
			rho_rot=0;
			chi_done=0;
			chi_ctr_d<=chi_ctr;
			//next_data_d<=next_data;
			end
		//	$display("perm reset state");
		end
		copym1m2:begin
		//copy data from m1 to m2
		if (cs==2)begin
			//code for copying m3 to m1
			m1wr=0;
			m2wr=1;
			//Increment m3r	//changed ffs x,y
			if (m1ry>=4)begin
				m1ry_d=0;
				if (m1rx>=4)
					m1rx_d=0;
				else begin
					m1rx_d=m1rx+1; 
				end
			end
			else begin
				m1ry_d=m1ry+1;			
			end			
			
			m2wx_d=m1rx_d;
			m2wy_d=m1ry_d;
			
			m2wd=m1rd;
			
			if(m1rx==4&&m1ry==4)begin
				ns_perm=findC;
				//make m1 free
				
				//stopin_d=0;	//added stopin here
				next_data_d=1;
				m1_done_d=0;
			end
			else ns_perm=copym1m2;
		//	$display("copym3m1:m1wd:%h,m3rd:%h m1wx:%h,m1wy:%h",m1wd,m3rd,m1wx,m1wy);
		end
		end	
		findC:begin
			//$display("perm findC state,m1ry:%d,m1rx:%d,c_round_d:%d m1rd:%h m1_done:%b r1=%h",m1ry,m1rx,c_round_d,m1rd,m1_done,r1);
			//input in r1
			m2wr=0;		//read
			//r2=1;
			if (m2ry==0)begin		
				r1_d=m2rd;
				//$display("m1ry==0");
			end
			else r1_d=r1_d^m2rd;						
			
			if (m2ry>=4)begin
				//done one x 
				m3wr=1;
				//$display($time,"r1:%h m1ry%d,m1rx:%d",r1,m1ry,m1rx); 
				m2ry_d=0;
				r2_d=r1_d;
				
				//inputting in m2 & m3
				m2wy_d=0;			
				m2wd=r2_d;	//Not passing through flip flop r2_d
				//m2wr=0;
				m2wx_d=m2wx+1;
				
				//m3
				m3wr=1;
				m3wd=r2_d;	//Not passing through flip flop r2_d
				//m3wr=0;
				
				m3wx_d=m3wx+1;
				
				
				if(m2rx>=4)begin
					m2rx_d=0;
				end
				else begin 
					m2rx_d=m2rx+1;
				end
			end
			else begin
				//$display($time,"m1ry_d:%d",m1ry_d); 
				m2ry_d=m2ry+1;	
			end
			
			if (m2ry==4 && m2rx==4)begin
				//$display("doneC");
				m3wr=1;	//might be for C
				ns_perm=dummy;
				
				// copied dummy state here
				ns_perm=findD;
				//m3wr=0;
				//start from these values for D
				m2ry_d=0;
				j_d=j+1;
				m2rx_d=cxminus1[i];	//4
				m3ry_d=0;
				m3rx_d=cxplus1[i];   	//1
				m3wx_d=0;
				m3wy_d=1;	//Store D in m3 y=1
				
			end
			else ns_perm=findC;
					
			c_round_d=c_round+1;			
		end
		//removed
		dummy:begin
			ns_perm=findD;
			m3wr=0;
			//start from these values for D
			m2ry_d=0;
			j_d=j+1;
			m2rx_d=cxminus1[i];	//4
			m3ry_d=0;
			m3rx_d=cxplus1[i];   	//1
			m3wx_d=0;
			m3wy_d=1;	//Store D in m3 y=1
			
		end
		findD:begin
			//$display("m2rd:%h, m3rd:%h. m2rx:%d,m3rx:%d,m2ry:%d",m2rd, m3rd, m2rx,m3rx,m2ry);
			m3wr=0;//reading from m3 future values
			m2wr=0;//reading from m2 past values
			r1_d=m2rd;
			
			//changed to use constant regs
			m2rx_d=cxminus1[i];
			
			if(i==4)begin
				r3_d=r1_d^{r2_d[62:0],r2_d[63]};
								
				//r1_d=0;
			//	r2_d=0;
				//r3_d=0;
			end
			 
			ns_perm=findD1;
			m3rx_d=cxminus1[i];	//set for the lower part
			r2_d=m3rd;
			//$display("D:%h,m3wy:%h,m3wx:%h",m3wd,m3wy,m3wx);
		end
		findD1:begin
		//D part 2: xor cxplus1 with prev value and store in m3 
			ns_perm=findD;
			if (i==4)
				i_d=0;	
			else i_d=i+1;

			if (j==4)
				j_d=0;	
			else j_d=j+1;
			m3rx_d=cxplus1[j];	//for the upper part	
			r1_d=m3rd;
			r3_d=r1_d^{r2_d[62:0],r2_d[63]};
			
			m3wr=1;
			m3wd=r3_d;	
			m3wx_d=m3wx+1;

			if (j==0) begin
				ns_perm=dotheta;
				j_d=0;
				i_d=0;

				//resetting ffs for theta take i/p from m1,m3 store in m2
				//m1wr=0;
				//m3wr=0;
				m1ry_d=0;
				m1rx_d=0;
				m3ry_d=0;
				m3rx_d=0;
				m2wr=0;
				m2wx_d=0;
				m2wy_d=0;

				//new theta setup copied frojm cpm1m2
				//new theta setup
				//resetting ffs for theta take i/p from m1,m3 store in m2
				//m1wr=0;
				m1ry_d=0;
				m1rx_d=0;
				
				//converting from m1 to m2
				m2ry_d=0;
				m2rx_d=0;

				m3ry_d=1;
				m3rx_d=0;
				m2wx_d=0;
				m2wy_d=0;

			end
		end
	
		dotheta:begin
			theta_start=1;
			//m1wr=0;
			m3wr=0;

			//added m2
			m2wr=0;
			//$display("m1rd:%h,m2rd:%h diff:%d",m1rd,m2rd,m2rd-m1rd);
			r1_d=m2rd;
			r2_d=m3rd;
			r3_d=r1_d^r2_d;
						
			//Increment m1 //input
			if (m2rx>=4)begin
				m2rx_d=0;
				if (m2ry>=4)
					m2ry_d=0;
				else begin
					m2ry_d=m2ry+1; 
				end
			end
			else begin
				m2rx_d=m2rx+1;			
			end
			
			//m3
			//Increment m3 //D
			m3ry_d=1;
			if (m3rx>=4)begin
				m3rx_d=0;
			end
			else begin
				m3rx_d=m3rx+1;			
			end
			
			if(m2rx==4&&m2ry==4)begin
				theta_start=0;
				m2wd=r3_d;
			//	$display("donetheta");
				//resetting for rhopi
				//m2wr=0;
				m2rx_d=0;
				m2ry_d=0;
				m3wx_d=0;
				m3wy_d=0;
			end
			 ns_perm=dotheta1;
			
			//$display(" Thetam1ry:%h,m1rx:%h,m1rd:%h m3ry:%h,m3rx:%h,m3rd:%h m2wy:%h,m2wx:%h,m2wd:%h",m1ry,m1rx,m1rd,m3ry,m3rx,m3rd,m2wy,m2wx,r3_d);
			//copied from theta1
			if (m2wx>=4)begin
				m2wx_d=0;
				if (m2wy>=4)begin
					m2wy_d=0;
				end
				else begin
					m2wy_d=m2wy+1;			
				end
			end
			else begin
				m2wx_d=m2wx+1; 
			end

			m2wr=1;
			m2wd=r3_d;

			ns_perm=dotheta;

			if(m2wx==4&&m2wy==4)begin
				ns_perm=dorho;
			end
		end
		//removed state
		dotheta1:begin
			//m2write here

			if (m2wx>=4)begin
				m2wx_d=0;
				if (m2wy>=4)begin
					m2wy_d=0;
				end
				else begin
					m2wy_d=m2wy+1;			
				end
			end
			else begin
				m2wx_d=m2wx+1; 
			end

			m2wr=1;
			m2wd=r3_d;

			ns_perm=dotheta;

			if(m2wx==4&&m2wy==4)begin
				ns_perm=dorho;
			end
		end
		//step4
		dorho:begin
			//read from m2 and store in m3
			m2wr=0;
			r1_d=m2rd;//m2rd;
			r2_d=rot[m2ry][m2rx];	//changed x,y
			rho_rot={64'h0,r1_d}<<r2_d;
			r3_d=rho_rot[127:64]|rho_rot[63:0];
			
			//new method
		//	rho_rot=r1_d<<r2_d;
			//r3_d=rho_rot[127:64]|r2_d;
			//$display("r2_d:%d",r2_d);
			//shift by r2_d times
			//r3_d={r1_d[62:0],r1_d[63]};
			
			//Increment m2	//changed ffs x,y
			if (m2ry>=4)begin
				m2ry_d=0;
				if (m2rx>=4)
					m2rx_d=0;
				else begin
					m2rx_d=m2rx+1; 
				end
			end
			else begin
				m2ry_d=m2ry+1;			
			end
			
			//Increment m3
			m3wx_d=m2ry_d;	//no flip flop on rhs
			m3wy_d=modulo(m2rx_d*2+m2ry_d*3,5);
			
			m3wr=1;
			m3wd=r3_d;
			
			if(m3wx==4&&m3wy==0)begin
				ns_perm=donothing;
				//m3wr=0;
				/*m3rx_d=0;
				m3ry_d=0;
				m2wx_d=0;
				m2wy_d=0;
				r1_d=0;
				r2_d=0;*/
				
				//copied from donothing
				//m3wr=0;
				m2wr=0;
				ns_perm=dochi1;

				m3rx_d=0;
				m3ry_d=0;

				//prep for dochi1
				m3rx_d=modulo(m3rx+1,5);	//modulo x+1
				m3ry_d=0;

			end
			else ns_perm=dorho;
			//$display("RHOPI:r1_d:%h,r2_d:%d,r3_d:%h,m2rx:%h,m2ry:%h,m3wx:%h,m3wy:%h(x*2+3*y)",r1_d,r2_d,r3_d,m2rx,m2ry,m3wx,m3wy);
			
		end
		//removed state
		//step5
		donothing:begin
			m3wr=0;
			m2wr=0;
			ns_perm=copym3tom2;
			m3rx_d=0;
			m3ry_d=0;
			
		end
		//removed state
		//to remove this state, change chi1, ch2, chi3 to use m3 instead of m2
		copym3tom2:begin
			//state 10
			m2wr=1;
			m3wr=0;
			//Increment m3r	//changed ffs x,y
			if (m3ry>=4)begin
				m3ry_d=0;
				if (m3rx>=4)
					m3rx_d=0;
				else begin
					m3rx_d=m3rx+1; 
				end
			end
			else begin
				m3ry_d=m3ry+1;			
			end			
			
			m2wx_d=m3rx_d;
			m2wy_d=m3ry_d;
			
			m2wd=m3rd;
			
			if(m3rx==4&&m3ry==4)begin
				ns_perm=dochi1;
			
				/*			
				m2rx_d=modulo(m2rx+1,5);	//modulo x+1
				m2ry_d=0;
				m2wx_d=0;
				m2wy_d=0;
				*/
				//changing to m3
				m2rx_d=modulo(m2rx+1,5);	//modulo x+1
				m2ry_d=0;
				m2wx_d=0;
				m2wy_d=0;
			end
			else ns_perm=copym3tom2;
		//	$display("copy:m2wd:%h,m3rd:%h m2wx:%h,m2wy:%h",m2wd,m3rd,m2wx,m2wy);
		end
		//step7
		dochi1:begin
		//changing m2 with m3 

		//read from m2, store +1 in m3
		//it shoiuld start a cycle ahead than prev stage
			m3wr=0;
			m2wr=1;
			
			//inc m2 for read
			if (m3ry>=4)begin
				m3ry_d=0;
				m3rx_d=modulo(m3rx+1,5);	//for ~B(x+1)
			end
			else begin
				m3ry_d=m3ry+1;			
			end	 
			
			//Increment m3w	
			if (m2wy>=4)begin
				m2wy_d=0;
				if (m2wx>=4)begin
					m2wx_d=0;
				end
				else begin
					m2wx_d=m2wx+1;			
				end
			end
			else begin
				m2wy_d=m2wy+1; 
			end
			
			r1_d=m3rd;
			r2_d=~r1_d;
			
			m2wd=r2_d;
									
			if(m2wy==4 &&m2wx==4) begin
				ns_perm=dochi2;
				//reset ffs for chi2
				m3rx_d=modulo(m3rx+2,5);	//modulo x+2
				m3ry_d=0;
				m3wx_d=0;
				m3wy_d=0;
			end
			else begin
				ns_perm=dochi1;
			end
		//	$display("dochi1:m2rd(r1_d):%h,m3wd(r2_d):%h m3wx:%h,m3wy:%h,m2rx:%h,m2ry:%h",m2rd,m3wd,m3wx,m3wy,m2rx,m2ry);
		
		//old logic with extra copym3m2 stage
/*
	//read from m2, store +1 in m3
		//it shoiuld start a cycle ahead than prev stage
			m2wr=0;
			m3wr=1;
			
			//inc m2 for read
			if (m2ry>=4)begin
				m2ry_d=0;
				m2rx_d=modulo(m2rx+1,5);	//for ~B(x+1)
			end
			else begin
				m2ry_d=m2ry+1;			
			end	 
			
			//Increment m3w	
			if (m3wy>=4)begin
				m3wy_d=0;
				if (m3wx>=4)begin
					m3wx_d=0;
				end
				else begin
					m3wx_d=m3wx+1;			
				end
			end
			else begin
				m3wy_d=m3wy+1; 
			end
			
			r1_d=m2rd;
			r2_d=~r1_d;
			
			m3wd=r2_d;
									
			if(m3wy==4 &&m3wx==4) begin
				ns_perm=dochi2;
				//reset ffs for chi2
				m2rx_d=modulo(m2rx+2,5);	//modulo x+2
				m2ry_d=0;
				m2wx_d=0;
				m2wy_d=0;
			end
			else begin
				ns_perm=dochi1;
			end
		//	$display("dochi1:m2rd(r1_d):%h,m3wd(r2_d):%h m3wx:%h,m3wy:%h,m2rx:%h,m2ry:%h",m2rd,m3wd,m3wx,m3wy,m2rx,m2ry);
		
*/
		end
		//state11
		dochi2:begin
			//read from m2, store +2 in m3 after & with it
			//it shoiuld start 1 cycles ahead than prev stage
			m3wr=0;
			m2wr=0;
			
			//inc m2 for read
			if (m3ry>=4)begin
				m3ry_d=0;
				if (m3rx>=4)begin
					m3rx_d=0;
				end
				else begin
					m3rx_d=m3rx+1;			
				end
				
			end
			else begin
				m3ry_d=m3ry+1;			
			end	 
			
			//Increment m3r	
			if (m2ry>=4)begin
				m2ry_d=0;
				if (m2rx>=4)begin
					m2rx_d=0;
				end
				else begin
					m2rx_d=m2rx+1;			
				end
			end
			else begin
				m2ry_d=m2ry+1; 
			end
			
			//Increment m3w	
			if (m2wy>=4)begin
				m2wy_d=0;
				if (m2wx>=4)begin
					m2wx_d=0;
				end
				else begin
					m2wx_d=m2wx+1;			
				end
			end
			else begin
				m2wy_d=m2wy+1; 
			end
			m2wr=0;
			r1_d=m3rd;	//x+2
			r2_d=m2rd;	//~(x+1)
			r3_d=r1_d&r2_d;	//~B(x+1) & B(x+2)
			m2wr=1;
			m2wd=r3_d;
									
			if(m2wy==4 &&m2wx==4) begin
				ns_perm=dochi3;
				//reset ffs for chi3
				m3rx_d=0;	
				m3ry_d=0;
				m3wx_d=0;
				m3wy_d=0;
			end
			else begin
				ns_perm=dochi2;
			end
		//	$display("dochi2:r3_d:%h,m3wx;%h,m3wy;%h,m2rx:%h,m2ry:%h",r3_d,m3wx,m3wy,m2rx,m2ry);
		
	
	
	
/*
			//old 
			
			//read from m2, store +2 in m3 after & with it
			//it shoiuld start 1 cycles ahead than prev stage
			m2wr=0;
			m3wr=0;
			
			//inc m2 for read
			if (m2ry>=4)begin
				m2ry_d=0;
				if (m2rx>=4)begin
					m2rx_d=0;
				end
				else begin
					m2rx_d=m2rx+1;			
				end
				
			end
			else begin
				m2ry_d=m2ry+1;			
			end	 
			
			//Increment m3r	
			if (m3ry>=4)begin
				m3ry_d=0;
				if (m3rx>=4)begin
					m3rx_d=0;
				end
				else begin
					m3rx_d=m3rx+1;			
				end
			end
			else begin
				m3ry_d=m3ry+1; 
			end
			
			//Increment m3w	
			if (m3wy>=4)begin
				m3wy_d=0;
				if (m3wx>=4)begin
					m3wx_d=0;
				end
				else begin
					m3wx_d=m3wx+1;			
				end
			end
			else begin
				m3wy_d=m3wy+1; 
			end
			m3wr=0;
			r1_d=m2rd;	//x+2
			r2_d=m3rd;	//~(x+1)
			r3_d=r1_d&r2_d;	//~B(x+1) & B(x+2)
			m3wr=1;
			m3wd=r3_d;
									
			if(m3wy==4 &&m3wx==4) begin
				ns_perm=dochi3;
				//reset ffs for chi3
				m2rx_d=0;	
				m2ry_d=0;
				m2wx_d=0;
				m2wy_d=0;
			end
			else begin
				ns_perm=dochi2;
			end
		//	$display("dochi2:r3_d:%h,m3wx;%h,m3wy;%h,m2rx:%h,m2ry:%h",r3_d,m3wx,m3wy,m2rx,m2ry);
		*/
		end
		dochi3:begin 
			//read from m2 and store in m2, m3
			//read from m2, store B(x,y) in m3 after ^ with m3
			//also storing in m2 to save clock cycle
			//it shoiuld start 1 cycles ahead than prev stage
			m3wr=0;
			m2wr=0;
			
			//Increment m2r	
			if (m3ry>=4)begin
				m3ry_d=0;
				if (m3rx>=4)begin
					m3rx_d=0;
				end
				else begin
					m3rx_d=m3rx+1;			
				end
			end
			else begin
				m3ry_d=m3ry+1; 
			end
			
			//Increment m3r	
			if (m2ry>=4)begin
				m2ry_d=0;
				if (m2rx>=4)begin
					m2rx_d=0;
				end
				else begin
					m2rx_d=m2rx+1;			
				end
			end
			else begin
				m2ry_d=m2ry+1; 
			end
			
			//Increment m3w	
			if (m2wy>=4)begin
				m2wy_d=0;
				if (m2wx>=4)begin
					m2wx_d=0;
				end
				else begin
					m2wx_d=m2wx+1;			
				end
			end
			else begin
				m2wy_d=m2wy+1; 
			end
			
			m2wr=0;
			r1_d=m3rd;	//x
			r2_d=m2rd;	//~B(x+1) & B(x+2)
			r3_d=r1_d^r2_d;	//
			m2wr=1;
			m2wd=r3_d;
			
			
			//iota
			if(m2wy==0 &&m2wx==0) begin
				
				m2wr=0;
				r1_d=m3rd;	//x
				r2_d=m2rd;	//~B(x+1) & B(x+2)
				r3_d=r1_d^r2_d^cmx[round];	//
				m2wr=1;
				m2wd=r3_d;
				
			end
									
			if(m2wy==4 &&m2wx==4) begin
				ns_perm=doout;
				//reset ffs 
				m2wx_d=0;
				m2wy_d=0;
				//m3wr=0;		//might break 0,0 or might miss 4,4

				//m2 write stuff
				m3wx_d=0;
				m3wy_d=0;


				//iota
				if (round==24)	
					round_d=0;
				else round_d=round+1;
			end
			else begin
				ns_perm=dochi3;
			end
		//	$display("dochi3:r3_d:%h,m3wx;%h,m3wy;%h",r3_d,m3wx,m3wy);

			//try to store in m2
			
			m3wr=1;
			//m2 stuff
			m3wd=m2wd;
			m3wx_d=m2wx_d;
			m3wy_d=m2wy_d;

/*
			//old
			//read from m2 and store in m2, m3
			//read from m2, store B(x,y) in m3 after ^ with m3
			//also storing in m2 to save clock cycle
			//it shoiuld start 1 cycles ahead than prev stage
			m2wr=0;
			m3wr=0;
			
			//Increment m2r	
			if (m2ry>=4)begin
				m2ry_d=0;
				if (m2rx>=4)begin
					m2rx_d=0;
				end
				else begin
					m2rx_d=m2rx+1;			
				end
			end
			else begin
				m2ry_d=m2ry+1; 
			end
			
			//Increment m3r	
			if (m3ry>=4)begin
				m3ry_d=0;
				if (m3rx>=4)begin
					m3rx_d=0;
				end
				else begin
					m3rx_d=m3rx+1;			
				end
			end
			else begin
				m3ry_d=m3ry+1; 
			end
			
			//Increment m3w	
			if (m3wy>=4)begin
				m3wy_d=0;
				if (m3wx>=4)begin
					m3wx_d=0;
				end
				else begin
					m3wx_d=m3wx+1;			
				end
			end
			else begin
				m3wy_d=m3wy+1; 
			end
			
			m3wr=0;
			r1_d=m2rd;	//x
			r2_d=m3rd;	//~B(x+1) & B(x+2)
			r3_d=r1_d^r2_d;	//
			m3wr=1;
			m3wd=r3_d;
			
			
			//iota
			if(m3wy==0 &&m3wx==0) begin
				
				m3wr=0;
				r1_d=m2rd;	//x
				r2_d=m3rd;	//~B(x+1) & B(x+2)
				r3_d=r1_d^r2_d^cmx[round];	//
				m3wr=1;
				m3wd=r3_d;
				
			end
									
			if(m3wy==4 &&m3wx==4) begin
				ns_perm=doout;
				//reset ffs 
				m3wx_d=0;
				m3wy_d=0;
				//m3wr=0;		//might break 0,0 or might miss 4,4

				//m2 write stuff
				m2wx_d=0;
				m2wy_d=0;


				//iota
				if (round==24)	
					round_d=0;
				else round_d=round+1;
			end
			else begin
				ns_perm=dochi3;
			end
		//	$display("dochi3:r3_d:%h,m3wx;%h,m3wy;%h",r3_d,m3wx,m3wy);

			//try to store in m2
			
			m2wr=1;
			//m2 stuff
			m2wd=m3wd;
			m2wx_d=m3wx_d;
			m2wy_d=m3wy_d;



*/

		end
		//removed state
		doiota:begin
			//  A[0,0] = A[0,0] xor RC
			//also storing in m2 to save clock cycle
			m3wr=0;
			r1_d=m3rd;
			m3wr=1;
			m2wr=1;	
			r2_d=r1_d^cmx[round];
			m3wd=r2_d;
		//	$display("round:%dcmx[round]:%h,r1_d:%h,m3wd:%h,m3wx;%h,m3wy;%h",round,cmx[round],r1_d,m3wd,m3wx,m3wy);			
			
			//m3wx_d=m3wx+1;			
			if(m3wy==0 &&m3wx==0) begin
				m3wr=0;
				r1_d=m3rd;
				m3wr=1;
				r2_d=r1_d^cmx[round];
				m3wd=r2_d;
				//reset ffs
				//m3wr=0;
				m3rx_d=0;
				m3ry_d=0;
				m2wr=1;
				//add new space saver stuff
				ns_perm=doout;
			//	m2wx_d=0;
			//	m2wy_d=0;

			end
			else begin
				ns_perm=doiota;
			end
			
			//m2 stuff
			m2wd=m3wd;
			m2wx_d=m3wx_d;
			m2wy_d=m3wy_d;
			

		if (round==24)	
			round_d=0;
		else round_d=round+1;	// check latch	
		
		end
		//removed state
		donothing2:begin
			m3wr=0;
			m2wr=0;
			r2_d=0;
			ns_perm=doout;
			m2wx_d=0;
			m2wy_d=0;

		end
		//removed state
		copym3m1:begin
		
			//its actually copying to m2
			//code for copying m3 to m2
			m2wr=0;
			m3wr=0;
			//Increment m3r	//changed ffs x,y
			if (m3ry>=4)begin
				m3ry_d=0;
				if (m3rx>=4)
					m3rx_d=0;
				else begin
					m3rx_d=m3rx+1; 
				end
			end
			else begin
				m3ry_d=m3ry+1;			
			end			
			
			m2wx_d=m3rx_d;
			m2wy_d=m3ry_d;
			
			m2wd=m3rd;
			
			if(m3rx==4&&m3ry==4)begin
				ns_perm=doout;
				m2wx_d=0;
				m2wy_d=0;
			end
			else ns_perm=copym3m1;
		//	$display("copym3m1:m1wd:%h,m3rd:%h m1wx:%h,m1wy:%h",m1wd,m3rd,m1wx,m1wy);
		
		
		end
		//state11
		doout:begin
		//check 24 rounds working
			if (round==24&&cs_out==0) begin	//added line for check output state machine in reset state
				ns_perm=doneoutput;
				//copy perm state high might fail if sending out takes too long
				perm_copy=1;	
			//	$display("DONE PERM");
				//adding below states to reduce cycles
				ns_perm=reset_perm;
				firstout=0;
			end
			else begin 
				ns_perm=findC;
			end
		end
		//removed state
		copym3m4:begin
			m4wr=1;
			m3wr=0;
			//Increment m3r	//changed ffs x,y
			if (m3ry>=4)begin
				m3ry_d=0;
				if (m3rx>=4)
					m3rx_d=0;
				else begin
					m3rx_d=m3rx+1; 
				end
			end
			else begin
				m3ry_d=m3ry+1;			
			end			
			
			m4wx_d=m3rx_d;
			m4wy_d=m3ry_d;
			
			m4wd=m3rd;
			
			if(m3rx==4&&m3ry==4)begin
				ns_perm=doneoutput;	//changed to LAST state
								//setting flag for 3rd sm
				//perm_finish=1;
				//added from last states
				ns_perm=reset_perm;
				//perm_finish=0; //set perm finish to low
				//add condition for round 0
				firstout=0;
			end
			else ns_perm = copym3m4;
	//		$display("copym3m4:m4wd:%h,m3rd:%h m4wx:%h,m4wy:%h",m4wd,m3rd,m4wx,m4wy);
		end
		//removed state
		done_perm:begin
			//check if 3rd sm finishes
			ns_perm=doneoutput;
			//perm_finish=0; //set perm finish to low
		end
		//removed state
		doneoutput:begin
			ns_perm=reset_perm;
			//perm_finish=0; //set perm finish to low
			//add condition for round 0
			firstout=0;
		end
		default:begin
			round_d=0;
			ns_perm=reset_perm;	
		end
	endcase
	
	//reset_out,working_out,done_out
	//output state machine
	case (cs_out)
	reset_out:begin
		if (perm_finish)
			ns_out=working_out;
		m4rx_d=0;
		m4ry_d=0;
		if (perm_copy)
			ns_out=copym3m4_out;
	end
	copym3m4_out:begin
		//copying m3 to m4 here to save cycles
		m4wr=1;
		m3wr=0;
		//Increment m3r	//changed ffs x,y
		if (m3ry>=4)begin
			m3ry_d=0;
			if (m3rx>=4)
				m3rx_d=0;
			else begin
				m3rx_d=m3rx+1; 
			end
		end
		else begin
			m3ry_d=m3ry+1;			
		end			
		
		m4wx_d=m3rx_d;
		m4wy_d=m3ry_d;
		
		m4wd=m3rd;
		
		if(m3rx==4&&m3ry==4)begin
			ns_out=working_out;	//changed to LAST state
							//setting flag for 3rd sm
			//perm_finish=1;
			perm_copy=0;
		end
	//		$display("copym3m4:m4wd:%h,m3rd:%h m4wx:%h,m4wy:%h",m4wd,m3rd,m4wx,m4wy);

	end
	working_out:begin
		ns_out=working_out;

		//paste output from sm2
		m4wr=0;
		//set flag high and give data to new state machine
		//Increment m3r	//changed ffs x,y
		if(stopout==0)
			if (m4rx>=4)begin
				m4rx_d=0;
				if (m4ry>=4) begin
					m4ry_d=0;
				end
				else begin
					m4ry_d=m4ry+1; 
				end
			end
			else begin
				m4rx_d=m4rx+1;			
			end
			
		dout=m4rd;
		
		pushout=1;
		
		if(m4rx==0 && m4ry==0)
			firstout=1;
		else firstout=0;
//		$display("dout:%h,m4rx:%h,m4ry:%h",dout,m4rx,m4ry);
		if(m4rx==4&&m4ry==4 && !stopout)begin		//added !stopout
			ns_out=done_out;
		end
	end
	done_out:begin
		ns_out=reset_out;
	end
	endcase
	
end

//Always seq to put input data in mem1
always @(posedge(clk) or posedge(rst)) begin
	if(rst) begin
		cs<=reset_state;
		m1wx<=0;
		m1wy<=0;
		cs_perm<=reset_perm;
		c_round<=0;
		round<=0;
		m1rx<=0;
		m1ry<=0;
		m1_done<=0;
		r1<=0;
		r2<=0;	
		stopin<=0;
		//m2wr<=0;
		m2wx<=0;
		m2wy<=0;
		m3wx<=0;
		m3wy<=0;
		m2rx<=0;
		m2ry<=0;
		m3rx<=0;
		m3ry<=0;
		r3<=0;
		cs_chi<=0;
		chi_done<=0;
		chi_ctr<=0;
		m4wx<=0;
		m4wy<=0;
		m4rx<=0;
		m4ry<=0;
		next_data<=0;
		i<=0;
		j<=0;
		cs_out<= reset_out;
	end else begin
		cs<= #1 ns;
		m1wx<= #1 m1wx_d;
		m1wy<= #1 m1wy_d;
		cs_perm<= #1 ns_perm;
		c_round<= #1 c_round_d;
		m1rx<= #1 m1rx_d;
		m1ry<= #1 m1ry_d;
		m1_done<= #1 m1_done_d;
		r1<= #1 r1_d;
		r2<= #1 r2_d;
		stopin<=stopin_d;
		//m2wr<= #1 m2wr_d;
		m2wx<= #1 m2wx_d;
		m2wy<= #1 m2wy_d;
		m3wx<= #1 m3wx_d;
		m3wy<= #1 m3wy_d;
		m2rx<= #1 m2rx_d;
		m2ry<= #1 m2ry_d;
		m3rx<= #1 m3rx_d;
		m3ry<= #1 m3ry_d;
		//m2wd_d<= #1 m2wd_d;
		r3<= #1 r3_d;
		cs_chi<= #1 ns_chi;
		chi_done<= #1 chi_done_d;
		chi_ctr<= #1 chi_ctr_d;
		round<= #1 round_d;
		m4wy<= #1 m4wy_d;
		m4wx<= #1 m4wx_d;
		m4rx<= #1 m4rx_d;
		m4ry<= #1 m4ry_d;
		next_data<= #1 next_data_d;
		i<=#1 i_d;
		j<=#1 j_d;
		cs_out<=#1 ns_out;
	end
end

function integer modulo; 
	input int a,b; 
	//output int z; 
	int i; 
	 
	if (a%b>=0) 
		modulo = a%b; 
	else modulo = a%b+b; 
	//$display ("%0dmod%0d=%0d",a,b,modulo); 
endfunction 

endmodule


