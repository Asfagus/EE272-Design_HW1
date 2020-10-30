// NOC Module 

`include "fifo_block.sv"

module noc_intf ( 
	input clk, input rst,
	input noc_to_dev_ctl, input [7:0] noc_to_dev_data,
	output reg noc_from_dev_ctl, output reg[7:0] noc_from_dev_data,
	output reg pushin, output reg firstin, input stopin, output reg [63:0] din,
	input pushout, input firstout, output reg stopout, input [63:0] dout );
	
	
	
	typedef enum reg[3:0] {	w_Reset,							//	0
							w_get_Dest_ID,						//	1
							w_get_Src_ID,						//	2
							w_get_Addr,							//	3
							w_get_Data							//	4							
							} write_req_sm;
	
	typedef enum reg[3:0] {	wr_Reset,							//	0		
							wr_send_Dest_ID,					//	1
							wr_send_Src_ID,						//	2
							wr_send_Actual_Data_Len,			//	3
							wr_send_Read_Data,					//	4
							wr_send_message_data				//	5
							} write_resp_sm;
	
	typedef enum reg[3:0] {	m_Reset,							//	0
							m_Pushout,							//	1
							m_Stopin							//	2
							} message_sm;
							
							
	write_req_sm cs_write, ns_write;
	write_resp_sm cs_write_resp, ns_write_resp;
	
	message_sm cs_message, ns_message;
	
	
	reg [7:0] w_command_line, w_command_line_d, r_command_line, wr_command_line, rr_command_line;
	reg is_write, is_read;
	reg [3:0] w_Alen_d, w_Alen;
	reg [7:0] w_Dlen_d, w_Dlen, Dlen_fifo, Dlen_fifo_d;
	reg [7:0] w_Source, w_Source_d, w_Dest, w_Dest_d, w_Addr, w_Data;
	reg [63:0] w_buffer, w_buffer_d;
	reg [7:0] w_buffer_index, w_buffer_index_d;
	reg [4:0] counter, counter_d, r_counter, r_counter_d;
	
	reg pushout_cs, pushout_ns, stopin_cs, stopin_ns;
	
	reg stopout1_d, stopout1;
	
	reg noc_from_dev_ctl_d;
	
	reg [3:0] r_Alen_d, r_Alen;
	reg [7:0] r_Dlen_d, r_Dlen;
	reg [7:0] r_Source, r_Dest, r_Addr, r_Data;
	
	reg [63:0] temp_variable, temp_variable_d;
	
	reg [63:0] r_buffer, r_buffer_d;
	reg [7:0] r_buffer_index, r_buffer_index_d;
	
	reg write_enable, read_enable, full, empty;
	
	
	typedef struct packed{
		reg [7:0] message_data, Actual_data_length, Src_ID, Dest_ID, command_code;
	} fifo_data;
	
	fifo_data data_in, data_out, temp_data, temp_data_d;
	
	reg [1:0] w_RC, r_RC;
	
	
	fifo_block fifo_object (.clk(clk), .rst(rst), .w_en(write_enable), .r_en(read_enable), .full(full), .empty(empty), .data_in(data_in), .data_out(data_out));
	
	
	// initial 
	// begin
		// stopout = 1'b0;
	// end
	
	
	always @(*)
	begin
		ns_write = cs_write;
		ns_write_resp = cs_write_resp;
		
		ns_message = cs_message;
		
		//pushout_ns = pushout_cs;
		
		w_command_line_d = w_command_line;
		is_write = 0;
		is_read = 0;
		w_Alen_d = w_Alen;
		w_Dlen_d = w_Dlen;
		w_Addr = 0;
		w_Data = 0;
		w_Source_d = w_Source;
		w_Dest_d = w_Dest;
		
		write_enable = 0;
		read_enable = 0;
		data_in = 0;
		temp_data_d = temp_data;
		
		w_buffer_d = w_buffer;
		w_buffer_index_d = w_buffer_index;
		pushin = 1'b0;
		counter_d = counter;
		noc_from_dev_ctl = 1'b1;
		noc_from_dev_data = 0;
		
		//stopout = 0;
		stopout = stopout1;
		stopout1_d = stopout1;
		
		r_counter_d = r_counter;
		
		r_Alen_d = r_Alen;
		r_Dlen_d = r_Dlen;
		Dlen_fifo_d = Dlen_fifo;
		r_Addr = 0;
		r_Data = 0;
		r_Source = 0;
		r_Dest = 0;
		r_buffer_d = r_buffer;
		r_buffer_index_d = r_buffer_index;
		
		temp_variable_d = temp_variable;


		
		// Request Block
		
		case(cs_write)
			w_Reset:
			begin
				if (noc_to_dev_ctl == 1'b1)
				begin
					w_command_line_d = noc_to_dev_data;
					if( w_command_line_d[2:0] == 3'b010)
					begin
						is_write = 1'b1;
						w_Alen_d = 2 ** (w_command_line_d [7:6]);
						w_Dlen_d = 2 ** (w_command_line_d [5:3]); 
						Dlen_fifo_d = 2 ** (w_command_line_d [5:3]);
						ns_write = w_get_Dest_ID;
						w_buffer_index_d = 0;						
					end
					else if (w_command_line_d[2:0] == 3'b001)
					begin
						is_read = 1'b1;
						w_Alen_d = 2 ** (w_command_line_d [7:6]);
						w_Dlen_d = 2 ** (w_command_line_d [5:3]); 
						Dlen_fifo_d = 2 ** (w_command_line_d [5:3]);
						ns_write = w_get_Dest_ID;						
					end
					else
					begin
						ns_write = w_Reset;
					end	
				end
				else
				begin
					ns_write = w_Reset;
				end
			end			
			
			w_get_Dest_ID:
			begin
				w_Source_d = noc_to_dev_data;
				ns_write = w_get_Src_ID;				
			end
			
			w_get_Src_ID:
			begin
				w_Dest_d = noc_to_dev_data;
				ns_write = w_get_Addr;
			end
			
			w_get_Addr:
			begin
				w_Addr = noc_to_dev_data;
				w_Alen_d = w_Alen - 1;
				if(w_Alen_d == 0)
				begin
					if (w_command_line_d[2:0] == 3'b010)
						ns_write = w_get_Data;
					else 
					begin
						w_RC = 00;					
						data_in.command_code = {w_RC, 6'b000011};
						data_in.Dest_ID = w_Source;
						data_in.Src_ID = w_Dest;
						data_in.Actual_data_length = Dlen_fifo;
						data_in.message_data = 8'b0;					
						write_enable = 1'b1;			
						
						ns_write = w_Reset;
					end
						
				end					
				else
					ns_write = w_get_Addr;
			end
			
			w_get_Data:
			begin
				w_Data = noc_to_dev_data;
				w_Dlen_d = w_Dlen - 1;
								
				w_buffer_d = w_buffer >> 8;				
				w_buffer_d[63:56] = w_Data;				
				w_buffer_index_d = w_buffer_index + 1;
				
				if (w_buffer_index == 7)
				begin
					pushin = 1'b1;
					din = w_buffer_d;
					
					if (counter == 0 )
						firstin = 1'b1;
					else 
						firstin = 1'b0;

//					if (counter == 24)
	//					stopout1_d = 0;
						
					counter_d = counter + 1;										
					w_buffer_index_d = 0;
					
					
					
				end
				else
				begin
					pushin = 1'b0;
					din = 64'b0;
					firstin = 1'b0;
				end
				
				if(w_Dlen_d == 0)
				begin				
					w_RC = 00;					
					data_in.command_code = {w_RC, 6'b000100};
					data_in.Dest_ID = w_Source;
					data_in.Src_ID = w_Dest;
					data_in.Actual_data_length = Dlen_fifo;
					data_in.message_data = 8'b0;					
					write_enable = 1'b1;			
					
					ns_write = w_Reset;
				end	
				else
				begin
					ns_write = w_get_Data;
				end
			end
				
		endcase
		


		
		// Message Block
		case(cs_message)
			m_Reset:
			begin
				if (pushout_cs == 0 && pushout_ns == 1)
//				if ( pushout == 1'b1 && firstout == 1'b1)
					ns_message = m_Pushout;
//				else if ( stopin == 1'b0 && firstin == 1'b1)
				else if ( stopin_cs == 1 && stopin_ns == 0)
					ns_message = m_Stopin;
				else 
					ns_message = m_Reset;
			end
			
			m_Pushout:
			begin
				data_in.command_code = 8'h05;
				data_in.Dest_ID = w_Source;
				data_in.Src_ID = w_Dest;
				data_in.Actual_data_length = 8'h17;
				data_in.message_data = 8'h12;					
				write_enable = 1'b1;
				
				ns_message = m_Reset;
			end
			
			m_Stopin:
			begin
				data_in.command_code = 8'h05;
				data_in.Dest_ID = w_Source;
				data_in.Src_ID = w_Dest;
				data_in.Actual_data_length = 8'h42;
				data_in.message_data = 8'h78;					
				write_enable = 1'b1;
				
				ns_message = m_Reset;
			end
			
		endcase
		
		
		
		// Response Block
		
		case(cs_write_resp)
			wr_Reset:
			begin	
				stopout1_d = 1;
				if (!empty)
				begin
					read_enable = 1'b1;
					temp_data_d = data_out;
					$display("%b %b %b %b %b", temp_data.command_code, temp_data.Dest_ID, temp_data.Src_ID, temp_data.Actual_data_length, temp_data.message_data);	
					noc_from_dev_data = data_out.command_code;
					ns_write_resp = wr_send_Dest_ID;
				end
				else
					ns_write_resp = wr_Reset;
			end
			
			wr_send_Dest_ID:
			begin	
				noc_from_dev_ctl = 1'b0;
				noc_from_dev_data = temp_data.Dest_ID;
				ns_write_resp = wr_send_Src_ID;
			end
			
			wr_send_Src_ID:	
			begin	
				noc_from_dev_ctl = 1'b0;
				noc_from_dev_data = temp_data.Src_ID;
				ns_write_resp = wr_send_Actual_Data_Len;
			end
			
			wr_send_Actual_Data_Len:
			begin	
				noc_from_dev_ctl = 1'b0;
				noc_from_dev_data = temp_data.Actual_data_length;
				r_Dlen_d = temp_data.Actual_data_length - 1;
				if( temp_data.command_code == 3'b100)
					ns_write_resp = wr_Reset;			
				else if (temp_data.command_code == 3'b011)
				begin
					// stopout_d = 1'b0;								// DONT TOUCH THIS
					// r_buffer_d = dout;
					// temp_variable_d = dout;
					stopout1_d = 0;			
					ns_write_resp = wr_send_Read_Data;
				end
				else if (temp_data.command_code == 3'b101)
					ns_write_resp = wr_send_message_data;
			end
			
			wr_send_Read_Data:
			begin
			/*
				if (r_buffer_index == 0)								// STUPID CODE. DONT USE.
				begin
					stopout_d = 0;
					r_buffer = dout;
				end
				else
					stopout_d = 1;
				*/
				
				
				if (r_buffer_index == 7 && (r_Dlen != 0))
				//if (r_buffer_index == 7)	
					stopout1_d = 0;
				else 
					stopout1_d = 1;


				if (r_buffer_index == 0)
				begin
					temp_variable_d = dout;
					//stopout1_d = 0;
				end
				else
				begin
					//stopout1_d = 1;
				end
				
				$display (" wr_send_Read_Data after - %h %h", dout, temp_variable);
				
				if(r_buffer_index == 0)
					r_Data = temp_variable_d[7:0];
				else if (r_buffer_index == 1)
					r_Data = temp_variable_d[15:8];
				else if (r_buffer_index == 2)
					r_Data = temp_variable_d[23:16];
				else if (r_buffer_index == 3)
					r_Data = temp_variable_d[31:24];
				else if (r_buffer_index == 4)
					r_Data = temp_variable_d[39:32];
				else if (r_buffer_index == 5)
					r_Data = temp_variable_d[47:40];
				else if (r_buffer_index == 6)
					r_Data = temp_variable_d[55:48];
				else if (r_buffer_index == 7)
					r_Data = temp_variable_d[63:56];
				
				
				r_buffer_index_d = r_buffer_index + 1;
				
				noc_from_dev_ctl = 1'b0;
				noc_from_dev_data = r_Data;
				r_Dlen_d = r_Dlen - 1;
				
				if ( r_buffer_index == 7 )
					r_buffer_index_d = 0;
				
				if ( r_Dlen == 0)
				begin
					ns_write_resp = wr_Reset;
				end
				else
				begin
					ns_write_resp = wr_send_Read_Data;
				end
				
				
			end
						
			wr_send_message_data:
			begin	
				noc_from_dev_ctl = 1'b0;
				noc_from_dev_data = temp_data.message_data;
				ns_write_resp = wr_Reset;	
			end			
			
		endcase
		
		
	end
	
	
	always @ (posedge clk or posedge rst)
	begin
		if (rst)
		begin
			cs_write <= w_Reset;
			cs_write_resp <= wr_Reset;			
			cs_message <= m_Reset;
			
			pushout_cs <= 0;
			pushout_ns <= 0;
			
			stopin_cs <= 0;
			stopin_ns <= 0;
			w_command_line <= 0;
			
			r_counter <= 0;
			stopout1 <= 1;
			
			w_Alen <= 0;
			w_Dlen <= 0;
			Dlen_fifo <= 0;
			w_Source <= 0;
			w_Dest <= 0;
			w_buffer_index <= 0;
			w_buffer <= 0;
			counter <= 0;			
			temp_data <= 0;			
			r_Alen <= 0;
			r_Dlen <= 0;
			r_buffer_index <= 0;
			r_buffer <= 0;
			
			temp_variable <= 0;
		end
		else
		begin
			cs_write <= #1 ns_write;
			cs_write_resp <= #1 ns_write_resp;				
			cs_message <= #1 ns_message;
			
			w_command_line <= w_command_line_d;
			
			r_counter <= #1 r_counter_d;
			stopout1 <= #1 stopout1_d;
			
			pushout_ns <= #1 pushout;
			pushout_cs <= #1 pushout_ns;
			
			stopin_ns <= #1 stopin;
			stopin_cs <= #1 stopin_ns;
			
			temp_data <= #1 temp_data_d;
			w_Alen <= #1 w_Alen_d;
			w_Dlen <= #1 w_Dlen_d;
			Dlen_fifo <= #1 Dlen_fifo_d;
			w_Source <= #1 w_Source_d;
			w_Dest <= #1 w_Dest_d;
			w_buffer_index <=  #1 w_buffer_index_d;
			w_buffer <=  #1 w_buffer_d;
			counter <= #1 counter_d;			
			r_Alen <= #1 r_Alen_d;
			r_Dlen <=  #1 r_Dlen_d;
			r_buffer_index <=  #1 r_buffer_index_d;
			r_buffer <=  #1 r_buffer_d;
			
			temp_variable <= #1 temp_variable_d;
		end
	end
	
	
endmodule : noc_intf

