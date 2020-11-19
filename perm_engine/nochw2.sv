module noc_intf (
	input clk,reset,			//from tb
    input tod_ctl,				//to device from tb
    input [7:0] tod_data,		//to device	from tb
    output frm_ctl,				//from device to tb//reg
    output [7:0] frm_data,  	//from device to tb	//reg
    output reg pushin_q,firstin_q,  //to device	
    input stopin,				//from device	
    output reg [63:0]din_q,		//to device
    input pushout,firstout,		//from device
    output  stopout,			//to device until NOC_intf read
    input [63:0] dout);			//from device to NOC

//internal regs to read message
reg [1:0] Alen;
reg [2:0] Dlen;

enum reg [2:0] {NOP,Read,Write,ReadResp,WriteResp,Message,Reserved1,Reserved2}cmd;

//used to read stuff from tod_data cmd 3 write
enum {reset_state,read_dest,read_src,read_addr,read_data,write_resp_state}ns,ps;

//used to handle cmd 2 read
enum {rreset_state,rread_dest,rread_src,rread_addr,read_resp_state}ns_read,cs_read;

//write response
enum {wreset_state,write_cmd,write_dest,write_src,write_data}ns_wresp,cs_wresp;

//stopin response
enum {sreset_state,swrite_dest,swrite_src,swrite_addr,swrite_data,sdone_state}	cs_sresp,ns_sresp;

//pushout resp
enum {preset_state,pwrite_dest,pwrite_src,pwrite_addr,pwrite_data,pdone_state}	cs_presp,ns_presp;

//fifo resp
enum {freset_state,fwrite_cmd,fwrite_dest,fwrite_src,fwrite_addr,fwrite_data}ns_fifo,cs_fifo;

//read response
enum {rrreset_state,rrwrite_cmd,rrwrite_dest,rrwrite_src,rrwrite_data,rrwrite_data_act}ns_rresp,cs_rresp;


//write addr and data size actual
integer addr_size,addr_size_d, data_size,data_size_d;

//read addr and data size actual
integer raddr_size,raddr_size_d, rdata_size, rdata_size_d;

//destination and source ID for cmd 2
reg [7:0] dest_id,dest_id_d,src_id,src_id_d; 

//d and s for cmd1
reg [7:0] dest_id_r,src_id_r; 

//packet size counter
integer packet_size,packet_size_d;

//8-bytes max address for write
reg [7:0][7:0] addr,addr_d;

//cmd 1 address
reg [7:0][7:0] addr_r;

//reg to count addr cmd 2
reg [3:0] addr_ctr,addr_ctr_d;

//cmd 1
reg [3:0] raddr_ctr,raddr_ctr_d;
			
			
//8-byte max data read from tb;
reg [7:0][7:0] datar,datar_d; 	//dataread
//reg to count datar
reg [7:0] datar_ctr,datar_ctr_d;

//reg to count data left in one packet
reg [7:0] data_size_ctr,data_size_ctr_d;

//reg to count firstin (comes after 25 inputs)
integer first_ctr,first_ctr_d;
reg firstin_d;

//pushin
reg pushin_d;

//response
reg frm_ctl_d;
reg [7:0] frm_data_d;

//write response 
reg write_resp,write_resp_d;
reg [7:0] wr_dest,wr_src;

//stopin response
reg [7:0] swr_dest,swr_src;

//pushout response
reg [7:0] pwr_dest,pwr_src;

//error codes
reg [1:0] werr;
reg [7:0] werr_adl;//actual data length written

//NOPS counter
//enum reg [1:0] {reset_state,} ns_nops,cs_nops;

//write start
//reg write_start;

//stopin resp
reg wstopin;

//pushin became ff also firstin
reg pushin,firstin;
reg [63:0] din;

//pushout resp
reg wpushout,wpushout_d;

//stopout regs
reg stopout_d;

//stopin regs
reg wstopin_d;

//fifo regs
reg w_en,r_en;
wire empty,full;
reg [39:0] datain,datain_d;
wire [39:0] dataout;

reg w_en_d;

//old pushout
reg pushoutold_d,pushoutold;

//typdef
typedef struct {
	reg [7:0]cmd;
	reg [7:0] d_id;
	reg [7:0] s_id;
	reg [7:0] data_length;

}wresponse;

wresponse write_response;

//made packed
typedef struct packed {
	reg [7:0] data_length;
	reg [7:0] addr;
	reg [7:0] s_id;
	reg [7:0] d_id;
	reg [7:0]cmd;
	
}mresponse;

mresponse message_response;

mresponse dataout_d,dataout_d1;	//response to the fifo


//fifo instantiation
fifo_smit f1 (clk,reset,w_en,r_en,datain,dataout,empty,full);

//reg for read
reg read,read_d;

//to check if writing to fifo
reg write_to_fifo;

//read responses
reg read_response,read_response_d;

reg fifo_disable;

//write data reg in cmd 1
reg [7:0][7:0] dataw,dataw_d; 	//data write
reg [7:0] dataw_size_ctr,dataw_size_ctr_d;	//count size of data max 128 
reg [2:0] w_shift,w_shift_d; 	//count number of shifts 
reg [7:0] total_data,total_data_d;	//count to 200 bytes written cmd1


//to read future inputs
assign stopout=stopout_d;	// stopout_d to read inputs from perm temporarily
assign frm_data =frm_data_d;
assign frm_ctl= frm_ctl_d;

//getting command signals 
assign {Alen,Dlen,cmd}=(tod_ctl)?tod_data:{Alen,Dlen,cmd};

//Store it in 8 byte location fifo

//Take input from todata
always @ (*)begin
	packet_size_d=packet_size;
	dest_id_d=dest_id;
	src_id_d=src_id;
	addr_d=addr;	//Address
	ns=ps;
	addr_ctr_d=addr_ctr;
	datar_ctr_d=datar_ctr;
	first_ctr_d=first_ctr;
	firstin=firstin_q;
	datar_d=datar;
	//pushin_d=pushin;
	data_size_ctr_d=data_size_ctr;
	pushin=pushin_q;
	pushin=0;
	frm_ctl_d=1;
	frm_data_d=0;
	//write_resp=0;
	ns_wresp=cs_wresp;
	//din=0;
	ns_sresp=cs_sresp;
	ns_presp=cs_presp;
	stopout_d=1;	//default stopout_d is 1
	
	//fifo enables
	w_en_d=w_en;
	r_en=0;
	//write_to_fifo=0;
	
	//fifo sm
	ns_fifo=cs_fifo;
	
	//keeping wstopin low
	wstopin_d=wstopin;
	
	//read response
	ns_read=cs_read;
	read_response_d=read_response;
	
	//read add counter
	raddr_ctr_d=raddr_ctr;
	
	//making error codes 0
	//werr_adl=0;
	//werr=0;
	
	//making wr_dest 0
	wr_dest=0;
	wr_src=0;
	write_resp_d=write_resp;
	ns_rresp=cs_rresp;
	read_d=read;
	dataw_d=dataw;
	dataw_size_ctr_d=dataw_size_ctr;
	w_shift_d=w_shift;
	total_data_d=total_data;
	pushoutold_d=pushoutold;
	
	
	//remove latches
	data_size_d=data_size;
	addr_size_d=addr_size;
	rdata_size_d=rdata_size;
	raddr_size_d=raddr_size;
	dataout_d1=dataout_d;
	dest_id_r=0;
	src_id_r=0;
	din=din_q;
	werr_adl=0;
	swr_src=0;
	swr_dest=0;
	pwr_src=0;
	pwr_dest=0;
	datain_d=datain;
	wpushout_d=wpushout;
	
	
	//Packet size counter	
	if (tod_ctl)
		packet_size_d=0;
	else packet_size_d=packet_size+1;
	
	//If tod ctl, then new command is present, set add and data size 
	if(tod_ctl)begin
		case (cmd)
		NOP:begin 
			//do nothing
			pushin=0;
			w_en_d=0;
		end
		Read:begin
			w_en_d=0;
			read_d=1;
			raddr_size_d=2**Alen;
			rdata_size_d=2**Dlen;
		end
		Write:begin
			//Sample DI, SI, Data
			//Write to device and send Write Response
			;
			//set add and data size
			//Check if synthesizable
			addr_size_d=2**Alen;
			data_size_d=2**Dlen;
		end		
				
		default:begin
		;
		end
		endcase
	end
	
	if (read)begin //read
		case (cs_read)
		rreset_state:begin
			//read_response_d=0;
			if (tod_ctl)begin
				ns_read=rread_dest;
			end
			stopout_d=1;
		end
		rread_dest:begin
			stopout_d=1;
			dest_id_r=tod_data;
			ns_read=rread_src;
		end
		rread_src:begin
			src_id_r=tod_data;
			ns_read=rread_addr;
		end
		rread_addr:begin
			addr_r[0]=tod_data;
			raddr_ctr_d=raddr_ctr+1;
			
			if (raddr_ctr_d==raddr_size)begin
				raddr_ctr_d=0;
				ns_read=read_resp_state;
			end
			else begin
				addr_d=addr_d<<8;
			end
		end
		read_resp_state:begin
			//$display("give read response");
			
			ns_read=rreset_state;
			read_d=0;
			read_response_d=1;
		end
		endcase
	end
	
	if(cmd==2)	begin //Write
		//take in source and dest id
		case (ps)		//changed to ps from ns
		reset_state:begin
			pushin=0;
			if (tod_ctl)begin
				ns=read_dest;	//might fail if tod_ctl more than one clock
			end
			else ns=reset_state;
			dest_id_d=0;
			src_id_d=0;
			w_en_d=0;
		end
		read_dest:begin
			dest_id_d=tod_data;
			ns=read_src;
		end
		read_src:begin
			src_id_d=tod_data;
			ns=read_addr;
		end
		read_addr:begin
			//read addr from tb in a 8-byte mem
			addr_d[0]=tod_data;
			addr_ctr_d=addr_ctr+1;
			
			if (addr_ctr_d==addr_size)begin
				addr_ctr_d=0;
				ns=read_data;
				datar_d=0;
			end
			else begin
				ns=read_addr;	
				//left shift to accomodate new data	
				addr_d=addr_d<<8;	//Warning:Little endian becoms big endian
			end
		end
		read_data:begin
			//Read data in temp reg
			//write also here only
			//Data is sent 8,16,32,64,128
			//if message has less than 8 bytes, we must store until next data arrives
			datar_d=datar>>8;	//Warning:data starts from 127 Check endian
			pushin=0;
			//firstin=0;
			datar_d[7]=tod_data;
			datar_ctr_d=datar_ctr+1;			//byte counter
			data_size_ctr_d=data_size_ctr+1;	//message data counter
			
			//output if buffer filled
			if (datar_ctr==7) begin	//done one set
				datar_ctr_d=0;
				
				if (!stopin)begin		//remove sotpin in future
					pushin=1;
					//count how many outputs to perm (25 for one set)
					if (first_ctr==24)	//done 25 sets of 8byte each
						first_ctr_d=0;
					else first_ctr_d=first_ctr+1;
					
					
					if (first_ctr==0)
						firstin=1;	//firstin
					else firstin=0;
				
					din=datar_d;		//output data
					
				end
				else begin
					//when stopin is high give an error response 
					//$display("tried to write while stopin high %t",$time);
					//commented the foll lines as it interrupts
					//pushin=0;
					//firstin=0;
				end
			end
			else pushin=0;
			
			if (data_size_ctr_d==data_size) begin		//go out when done
				data_size_ctr_d=0;	//reset data_size_ctr_d
				//go to reset state eventually
				ns=write_resp_state;
				
				//keeping size here to prevent overwrite
				{werr,werr_adl}={0,data_size};
				
				//write resp state
			
				ns=reset_state;
				write_resp_d=1;
				wr_dest=dest_id_d;
				wr_src=src_id_d;
			end	
			else begin
				ns=read_data;	
			end
			
			end	
			default:begin
				src_id_d=src_id;
				dest_id_d=dest_id_d;
			end
		endcase
		end//end cmd==2

		/*
		//if cmd==1 Read
		if (cmd==1)begin
			stopout_d=1;			
		end
		*/

		//old pushout logic
		pushoutold_d=pushout;

		//Responses
		//stopin response
		if (stopin)begin
			wstopin_d=1;
			swr_dest=dest_id_d;
			swr_src=src_id_d;
		end		
		
		//pushout (may overlap)
		if (pushout&&firstout&&pushoutold==0)begin		//added else statement REMOVE: or else they get discarded
			wpushout_d=1;
			pwr_dest=dest_id_d;
			pwr_src=src_id_d;
		end
		
		
		//write response to fifo
		if (write_resp_d)begin
			//write resp to fifo
			if (!w_en && !full) 	begin //if write is low, (not being used)
				w_en_d=1;
				write_to_fifo=1;
				//write 
				write_response.cmd={werr,000,3'b100};
				write_response.d_id=wr_dest;
				write_response.s_id=wr_src;
				write_response.data_length=werr_adl;
				datain_d={8'h0,write_response.data_length,write_response.s_id,write_response.d_id,write_response.cmd};
			end
			else begin
				//fifo busy
		//		if (!write_to_fifo) $display("writeerr:fifo_busy or full full:%b, %0t",full,$time);
			end
			
			if (w_en==1)begin
				write_resp_d=0;	
				w_en_d=0;
				write_to_fifo=0;
			end
		end
		//stopin in fifo
		else if (wstopin)begin
			//stopin high to low
				if (stopin==0)begin
					//write resp to fifo
					if (!w_en && !full) 	begin//if write is low, (not being used)
						w_en_d=1;
						//write 
						message_response.cmd={2'b00,3'b000,3'b101};
						message_response.d_id=swr_dest;
						message_response.s_id=swr_src;
						message_response.addr=8'h42;
						message_response.data_length=8'h78;
						datain_d={message_response.data_length,message_response.addr,message_response.s_id,message_response.d_id,message_response.cmd};
						//wstopin=0;
						
					end
					else begin
						//fifo busy
				//		$display("stoperr:fifo_busy or full full:%b,w2fifo:%b %0t",full,write_to_fifo,$time);
					end
				end
				
				if (w_en==1)begin
					wstopin_d=0;	
					w_en_d=0;
				end
		end
		else if (wpushout)begin
			//write resp to fifo
			if (!w_en && !full) 	begin//if write is low, (not being used)
				w_en_d=1;
				//write 
				message_response.cmd={2'b00,3'b000,3'b101};
				message_response.d_id=pwr_dest;
				message_response.s_id=pwr_src;
				message_response.addr=8'h17;
				message_response.data_length=8'h12;
				datain_d={message_response.data_length,message_response.addr,message_response.s_id,message_response.d_id,message_response.cmd};
			end
			else begin
				//fifo busy
		//		$display("pusherr:fifo_busy or full full:%b,w2fifo:%b %0t",full,write_to_fifo,$time);
			end
			
			if (w_en==1)begin
				wpushout_d=0;	
				w_en_d=0;
			end
		end
		else begin
			w_en_d=0;
		end
		
					
		//Handle responses using fifo
		if (read_response && cs_fifo==0)begin	//do not interrupt an ongoing response
			case(cs_rresp)
			rrreset_state:begin
				frm_data_d=0;	//resetting
				ns_rresp=rrwrite_cmd;
				frm_ctl_d=1;
			end
			rrwrite_cmd:begin
				r_en=0;
				frm_ctl_d=1;
				frm_data_d={2'b00,3'b000,3'b011};
				ns_rresp=rrwrite_dest;
				
			end
			rrwrite_dest:begin
				frm_ctl_d=0;
				frm_data_d=dest_id_r;
				ns_rresp=rrwrite_src;
				stopout_d=1;
			end
			rrwrite_src:begin
				frm_ctl_d=0;
				frm_data_d=	src_id_r;
				ns_rresp=rrwrite_data;
			end
			rrwrite_data:begin//actual data length
				frm_ctl_d=0;
				rdata_size_d=rdata_size;
				frm_data_d=	rdata_size;	//size of dlen**
				ns_rresp=rrwrite_data_act;
			end
			rrwrite_data_act:begin	//actual data
				if (w_shift==0) begin //take new 8 bytes
					stopout_d=0;
					dataw_d=dout;
				end
				else begin
					stopout_d=1;
				end
				total_data_d=7;
				dataw_size_ctr_d=dataw_size_ctr+1;	//stop when == rdata_size_d
				w_shift_d=w_shift+1;	//shift one byte every cycle
				
				frm_data_d=dataw_d[0];
				
				if (dataw_size_ctr==rdata_size_d-1) begin
					//done read response
					dataw_size_ctr_d=0;
					ns_rresp=rrreset_state;
					read_response_d=0;
				end
				else begin
					dataw_d=dataw_d>>8;	//changed to ff
				end
				
				if (total_data==199) begin
					total_data_d=0;
					ns_rresp=rrreset_state;
					
				end
				else begin
					total_data_d=total_data+1;
				end

			end
			endcase
			
		end
		else if (empty==0 || cs_fifo!=0)begin
				//not checking for wen
				//r_en=1;
				case (cs_fifo)
				freset_state:begin
					if (!empty)begin
						ns_fifo=fwrite_cmd;
						r_en=1;
						dataout_d1=dataout;
						frm_ctl_d=1;
					end
					frm_data_d=0;	//resetting
				end
				fwrite_cmd:begin
					r_en=0;
					frm_ctl_d=1;
					frm_data_d=dataout_d.cmd;
					ns_fifo=fwrite_dest;
				end
				fwrite_dest:begin
					
					frm_ctl_d=0;
					frm_data_d=dataout_d.d_id;
					ns_fifo=fwrite_src;
				end
				fwrite_src:begin
					frm_ctl_d=0;
					frm_data_d=	dataout_d.s_id;
					ns_fifo=fwrite_addr;
				end
				fwrite_addr:begin
					frm_ctl_d=0;
					//cmd write go to reset from here
					if (dataout_d.cmd[2:0]==4)begin	//write response
						frm_data_d=dataout_d.addr;	//it actually contains adl
						ns_fifo=freset_state;		//go to reset	
						write_resp_d=0;			
					end
					else if (dataout_d.cmd[2:0]==5)begin	//message response
						frm_data_d=dataout_d.addr;	//it  contains addr
						ns_fifo=fwrite_data;
					end	
					else begin
						//sometghings wrong
						$display("somethings wrong");
					end
				end
				fwrite_data:begin
					frm_ctl_d=0;
					frm_data_d=dataout_d.data_length;	//it actually contains adl
					
					ns_fifo=freset_state;		//go to reset	
					//if (dataout_d.data_length==78)
					//	wstopin=0;
					//if (dataout_d.data_length==12)
					//	wpushout_d=0;			
				end
				endcase
			end
			else begin
				//send nops
				frm_data_d=0;
				frm_ctl_d=1;
			end
		
end

always @ (posedge clk or posedge reset)begin
	if(reset)begin
		packet_size<=0;
		dest_id<=0;
		src_id<=0;
		addr<=0;
		ps<=reset_state;
		addr_ctr<=0;
		datar_ctr<=0;
		first_ctr<=0;
	//	firstin<=0;
		datar<=0;
		//pushin<=0;
		data_size_ctr<=0;
		cs_wresp <=wreset_state;
		write_resp<=0;
		cs_sresp<=sreset_state;
		cs_presp<=preset_state;
		cs_fifo<=freset_state;
		cs_read<=rreset_state;
		raddr_ctr<=0;
		wstopin<=0;
		w_en<=0;
		read_response<=0;
		cs_rresp<= rrreset_state;
		read<= 0;
		dataw<=0;
		dataw_size_ctr<=0;
		w_shift<=0;
		total_data<=0;
		pushoutold<=0;
		addr_size<=0;
		rdata_size<=0;
		raddr_size<=0;
		data_size<=0;
		dataout_d<=0;
		datain<=0;
		wpushout<=0;
		pushin_q<=0;
		firstin_q<=0;
		din_q<=0;
	end
	else begin
		packet_size<= #1 packet_size_d;
		dest_id<= #1 dest_id_d;
		src_id<= #1 src_id_d;
		addr<= #1 addr_d;
		ps<= #1 ns;
		addr_ctr<= #1 addr_ctr_d;
		datar_ctr<= #1 datar_ctr_d;
		first_ctr<= #1 first_ctr_d;
		//firstin<= #1 firstin_d;
		datar<= #1 datar_d;
		//pushin<= #1 pushin_d;
		data_size_ctr<= #1 data_size_ctr_d;
		cs_wresp<= #1 ns_wresp;
		write_resp<= #1 write_resp_d;
		cs_sresp<=#1 ns_sresp;
		cs_presp<=#1 ns_presp;
		cs_fifo<= #1 ns_fifo;
		cs_read<= #1 ns_read;
		raddr_ctr<= #1 raddr_ctr_d;
		wstopin<= #1 wstopin_d;
		w_en<= #1 w_en_d;
		read_response<= #1 read_response_d;
		cs_rresp<= #1 ns_rresp;
		read<= #1 read_d;
		dataw<= #1 dataw_d;
		dataw_size_ctr<= #1 dataw_size_ctr_d;
		w_shift<= #1 w_shift_d;
		total_data<= #1 total_data_d;
		pushoutold<= #1 pushoutold_d;
		addr_size<= #1 addr_size_d;
		rdata_size<= #1 rdata_size_d;
		raddr_size<= #1 raddr_size_d;
		data_size<= #1 data_size_d;
		dataout_d<= #1 dataout_d1;
		datain<= #1 datain_d;
		wpushout<= #1 wpushout_d;
		pushin_q<= #1 pushin;
		firstin_q<= #1 firstin;
		din_q<= #1 din;
	end
	
	
	
end

endmodule
`include "fifo_s.v"
