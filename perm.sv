// PERM module

module perm_blk (
	input clk, input rst, input pushin, output reg stopin, input firstin, input [63:0] din,
	
	output reg [2:0] m1rx, output reg [2:0] m1ry,
	input [63:0] m1rd,
	output reg [2:0] m1wx, output reg [2:0] m1wy,output reg m1wr,
	output reg [63:0] m1wd,
	
	output reg [2:0] m2rx, output reg [2:0] m2ry,
	input [63:0] m2rd,
	output reg [2:0] m2wx, output reg [2:0] m2wy,output reg m2wr,
	output reg [63:0] m2wd,
	
	output reg [2:0] m3rx, output reg [2:0] m3ry,
	input [63:0] m3rd,
	output reg [2:0] m3wx, output reg [2:0] m3wy,output reg m3wr,
	output reg [63:0] m3wd,
	
	output reg [2:0] m4rx, output reg [2:0] m4ry,
	input [63:0] m4rd,
	output reg [2:0] m4wx, output reg [2:0] m4wy,output reg m4wr,
	output reg [63:0] m4wd,
	
	output reg pushout, input stopout, output reg firstout, output reg [63:0] dout);
	
	
	// ENUM for Input State Machine:
	typedef enum reg[4:0] { 	R_input,											//	0
								input_inc_x, 										//	1
								input_inc_y, 										//	2
								input_memory_full									//	3
								} input_sm;
	
	// ENUM for Perm State Machine:
	typedef enum reg[4:0] { 	R_perm,												//	0
								write_C_inc_y_read_from_mem1,						//	1
								write_c_inc_x,										//	2
								write_C_into_mem2,                      			//	3
								write_rotated_C_into_mem2,							//	4
								cal_D_read_C_from_bottom_row,						//	5
								cal_D_read_rotated_C_from_top_row,					//	6
								Theta_A_run_through_y_index,						//	7
								inc_D,												//	8
								Done_with_Theta_A_Onto_rho_pi,						//	9
								Rho_Pi_inc_x,										//	10
								Rho_Pi_inc_y,										//	11
								Done_with_Rho_Pi,									//	12
								Chi_Iota_inc_x_read_x,								//	13
								Chi_Iota_inc_x_read_x_plus_1,						//	14
								Chi_Iota_inc_x_read_x_plus_2,						//	15
								Chi_Iota_inc_y,										//	16
								Done_with_Chi_Iota,									//	17
								After_Iota_write_C_inc_y_read_from_mem3,			//	18
								After_Iota_write_C_into_mem2,						//	19			
								After_Iota_write_c_inc_x							//	20
								} perm_sm;
	
	// ENUM for Output State Machine:
	typedef enum reg[4:0] {		R_output,											//	0
								Write_output,										//	1
								output_inc_x,										//	2
								output_inc_y,										//	3
								Done_reading										//	4
								} output_sm;
	
	input_sm cs_input, ns_input;
	perm_sm cs_perm, ns_perm;
	output_sm cs_output, ns_output;
	
	
	
	// Const registers:
	const int x_minus_1 [0:4] = { 4, 0, 1, 2, 3 };
	const int x_plus_1 [0:4]  = { 1, 2, 3, 4, 0 };
	const int x_plus_2 [0:4]  = { 2, 3, 4, 0, 1 };
	
	const int t_rho [0:4][0:4] =  '{	'{0,  1,  62, 28, 27 },
										'{36, 44, 6,  55, 20 },
										'{3,  10, 43, 25, 39 },
										'{41, 45, 15, 21, 8  },
										'{18, 2,  61, 56, 14 } };
	
	const int t_rho_wrong [4:0][4:0] =  '{	'{25, 39, 3,  10, 43}, 
										'{55, 20, 36, 44, 6 }, 
										'{28, 27, 0,  1,  62}, 
										'{56, 14, 18, 2,  61}, 
										'{21, 8,  41, 45, 15} };
										
		// x in pi step. new index : old index
	const int x_of_x_plus_3_y [0:4][0:4] = '{	'{0, 3, 1, 4, 2}, 
												'{1, 4, 2, 0, 3}, 
												'{2, 0, 3, 1, 4}, 
												'{3, 1, 4, 2, 0}, 
												'{4, 2, 0, 3, 1} };
										
	
	// round values for iota_value
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
	
	
	
					
	// Registers for calculation:
	reg [2:0] input_index_x, input_index_y, input_index_x_d, input_index_y_d;
	reg [2:0] C_index_x, C_index_y, C_index_x_d, C_index_y_d;
	reg [2:0] output_index_x, output_index_y, output_index_x_d, output_index_y_d;
	reg [2:0] rotate_C_inc_x, rotate_C_inc_x_d;
	reg [2:0] index_1_for_D_calc, index_2_for_D_calc, D_index_x, D_index_x_d;
	reg [2:0] Theta_index_x, Theta_index_y, Theta_index_x_d, Theta_index_y_d;
	reg [2:0] Rho_Pi_index_x_d, Rho_Pi_index_y_d, Rho_Pi_index_x, Rho_Pi_index_y, new_index_x_for_pi_stage, new_index_y_for_pi_stage;
	reg [2:0] index_x_of_theta_stage, index_y_of_theta_stage;
	reg [2:0] Chi_Iota_index_x_d, Chi_Iota_index_x, Chi_Iota_index_y_d, Chi_Iota_index_y, index_2_for_Chi_Iota, index_3_for_Chi_Iota;
	
	reg [63:0] temp_reg_after_C_rotation, C1, C1_d, D_1, D_1_d, D_2, D, D_d, old_A_from_input, new_A_for_Theta, value_from_theta_stage;
	reg [63:0] temp_variable_for_Chi_Iota, temp_variable_for_Chi_Iota_d, temp_variable_for_Chi_Iota_1, temp_variable_for_Chi_Iota_1_d,
				temp_variable_for_Chi_Iota_2, temp_variable_for_Chi_Iota_3, iota_value;
	reg [63:0] temp1, temp1_d, temp, temp_d;
	reg [127:0] rotation_reg_for_z;
	reg [5:0] offset_t_value_for_Rho_stage;
	
	reg [4:0] round_counter_i, round_counter_i_d;
	
	reg start_writing, start_writing_d;
	
	reg [63:0] newC, newC_d;
	
	reg stopin_d;
	reg stopin1, stopin1_d;
	
	reg enable_input, enable_input_d, enable_output, enable_output_d, enable_perm, enable_perm_d, done_with_prev_perm, done_with_prev_perm_d;
	

	// Combinational Logic
	always @(*)
	begin
		ns_input = cs_input;
		ns_perm = cs_perm;
		ns_output = cs_output;
		
		enable_input_d = enable_input;
		enable_perm_d = enable_perm;
		enable_output_d = enable_output;
		done_with_prev_perm_d = done_with_prev_perm;
		
		input_index_x_d = input_index_x;
		input_index_y_d = input_index_y;
		rotate_C_inc_x_d = rotate_C_inc_x;
	
		D_index_x_d = D_index_x;
		Theta_index_x_d = Theta_index_x;
		Theta_index_y_d = Theta_index_y;
		Rho_Pi_index_x_d = Rho_Pi_index_x;
		Rho_Pi_index_y_d = Rho_Pi_index_y;
		Chi_Iota_index_x_d = Chi_Iota_index_x;
		Chi_Iota_index_y_d = Chi_Iota_index_y;
		round_counter_i_d = round_counter_i;
		output_index_x_d = output_index_x;
		output_index_y_d = output_index_y;
		
		
		m1wx = input_index_x;
		m1wy = input_index_y;
		//m1rx = D_index_x; 
		//m1ry = Theta_index_y;
		
		m1rx = 3'b000; 
		m1ry = 3'b000;
		
		m2wx = 3'b000;
		m2wy = 3'b000;
		m3wx = 3'b000;
		m3wy = 3'b000;		
		m4wx = 3'b000;
		m4wy = 3'b000;
		
		m2wr = 1'b0;
		m3wr = 1'b0;
		m4wr = 1'b0;
		
		m2wd = 64'b0;
		m3wd = 64'b0;
		m4wd = 64'b0;
		
		m4ry = output_index_y;
		m4rx = output_index_x;
		m2rx = rotate_C_inc_x;
		m2ry = 3'b000;
		m3rx = D_index_x;
		m3ry = Theta_index_y;
		
		pushout = 1'b0;
		dout = 64'b0;
		firstout = 1'b0;
		iota_value = 64'b0;
		temp_d = temp;
		temp1_d = temp1;

		C_index_x_d = C_index_x;
		C_index_y_d = C_index_y;
		
		stopin1_d = stopin1;
		
		stopin = stopin1;
		
		//	REMOVE
		//stopin = 1'b1;												// LATCH
		//stopin_d = stopin;
		
		
		m1wr = 0;
		m1wd = 64'b0;
		
		
		start_writing_d = start_writing;
		
		
		case(cs_input)
			R_input:
			begin			
			
				input_index_x_d = 0;
				input_index_y_d = 0;
				
						//	REMOVE
				//stopin = 1'b1;
			//	stopin_d = 1'b1;
				
				m1wr = 1'b0;
				m1wd = 64'b0;
				start_writing_d = 1'b0;
				
				/*
				if (done_with_prev_perm == 1'b1)
					enable_perm_d = 1'b1;
				else
					enable_perm_d = 1'b0;
				*/
				
				if (enable_input)
				begin
					ns_input = input_inc_x;
					stopin1_d = 0;
				end
				else
					ns_input = R_input;
			end
			
			
			input_inc_x:
			begin
				stopin1_d = 0;
				if(pushin)
				begin
					if(firstin)
					begin
						start_writing_d = 1'b1;
					end
					if(start_writing_d == 1'b1)
					begin
						m1wr = 1'b1;
						
								//	REMOVE
						//stopin = 1'b0;
						//stopin_d = 1'b0;
						
						m1wd = din;		
						input_index_x_d = input_index_x + 1;
					end
				end

				
				if(input_index_x_d >= 4) 
				begin
					//input_index_x_d = 0;
					ns_input = input_inc_y;							
				end
			end
			
			
			input_inc_y:
			begin
				stopin1_d = 0;
				if(pushin)
				begin
					if(start_writing_d == 1'b1)
					begin
					
								//	REMOVE
						//stopin = 1'b0;
						//stopin_d = 1'b0;
						
						m1wr = 1'b1;
						m1wd = din;	
						input_index_x_d = 0;
						input_index_y_d = input_index_y + 1;					
					end
				end
				//if(input_index_x_d == 0)
					//input_index_y_d = input_index_y + 1;
				
				if(pushin)
				begin
					if (input_index_y >= 4) 
					begin
						
					//	- - - - - - - - - - - Error - - - - - - - - - -
					//	  at   9299.0ns
					//	  dout wrong, got 19c18c4aa008718c
					//	  expected        e21f3d72a7ac6cd0
					//					  ^^^^^^^^ ^^^^^^^
					//	- - - - - - - - - - - Error - - - - - - - - - -

						
						input_index_y_d = 0;
						input_index_x_d = 0;
					
						stopin1_d = 1;
						
						//m1wr = 1'b0;
//						m1wd = 64'b0;
						
						enable_input_d = 1'b0;
						enable_perm_d = 1'b1;
						ns_input = R_input;
						
						
						/*
						
						input_index_y_d = 0;
						stopin1_d = 1;
						ns_input = input_memory_full;
						
						*/
					end
					else
						ns_input = input_inc_x;				
				end
				else
				begin
					ns_input = input_inc_y;
				end
				
				
				
			end
			
			
			input_memory_full:
			begin
				input_index_y_d = 0;
				input_index_x_d = 0;
			
				stopin1_d = 1;
				
						//	REMOVE
				//	stopin = 1'b1;
				//stopin_d = 1'b1;
				
				
				m1wr = 1'b0;
				m1wd = 64'b0;
				
				enable_input_d = 1'b0;
				/*
				if (done_with_prev_perm == 1'b1)
					enable_perm_d = 1'b1;
				else
					enable_perm_d = 1'b0;
				*/
				
				enable_perm_d = 1'b1;
				ns_input = R_input;								
			end
		endcase
		
		
		
		case(cs_perm)
			R_perm:
			begin
			
				rotate_C_inc_x_d = 0;			
				m2wr = 1'b0;
				D_index_x_d = 0;
				Theta_index_x_d = 0;
				Theta_index_y_d = 0;
				Rho_Pi_index_x_d = 0;
				Rho_Pi_index_y_d = 0;
				Chi_Iota_index_x_d = 0;
				Chi_Iota_index_y_d = 0;
				round_counter_i_d = 0;
				temp_d = 0;
				temp1_d = 0;
				C_index_x_d = 0;
				C_index_y_d = 0;
				
			
				if (enable_perm)
					ns_perm = write_C_inc_y_read_from_mem1;
				else
					ns_perm = R_perm;				
			end
			
			write_C_inc_y_read_from_mem1:
			begin
				m1rx = C_index_x; m1ry = C_index_y;
				
				if (C_index_y == 0)
					temp_d = m1rd;
				else
					temp_d = temp ^ m1rd;
				
				m3wr = 1'b1;
				m3wx = C_index_x; m3wy = C_index_y;
				m3wd = m1rd;
				
				C_index_y_d = C_index_y + 1;
				
				if(C_index_y >= 4)
				begin
				/*
					m3wr = 1'b0;
					m2wr = 1'b1;
					m2wx = C_index_x; m2wy = 3'b000;
					m2wd = temp_d;
					ns_perm = write_c_inc_x;
					
					
					- - - - - - - - - - - Error - - - - - - - - - -
					  at   9290.0ns
					  dout wrong, got 4081d3447f9d6c9a
					  expected        e21f3d72a7ac6cd0
									  ^^^^^^^^^^^^  ^^
					- - - - - - - - - - - Error - - - - - - - - - -

					*/
				
					ns_perm = write_C_into_mem2;
				end
								
			end
			
			
			write_c_inc_x:
			begin
				C_index_x_d = C_index_x + 1;
				C_index_y_d = 0;
				
				if (C_index_x >= 4)
				begin
					C_index_x_d = 0;
					if (round_counter_i == 0)
						enable_input_d = 1'b1;					
//					ns_perm = write_rotated_C_into_mem2;
					D_index_x_d = 0;
					ns_perm = cal_D_read_C_from_bottom_row;
				end
				else
				begin
					ns_perm = write_C_inc_y_read_from_mem1;
				end			
			end
			
			
			write_C_into_mem2:
			begin
				m3wr = 1'b0;
				m2wr = 1'b1;
				m2wx = C_index_x; m2wy = 3'b000;
				m2wd = temp;
//				ns_perm = write_c_inc_x;				
				
				
				C_index_x_d = C_index_x + 1;
				C_index_y_d = 0;
				
				if (C_index_x >= 4)
				begin
					C_index_x_d = 0;
					if (round_counter_i == 0)
						enable_input_d = 1'b1;					
//					ns_perm = write_rotated_C_into_mem2;
					D_index_x_d = 0;
					ns_perm = cal_D_read_C_from_bottom_row;
				end
				else
				begin
					ns_perm = write_C_inc_y_read_from_mem1;
				end
				
				



			end
			
			
			write_rotated_C_into_mem2:
			begin				
				m2rx = rotate_C_inc_x;	m2ry = 3'b000;
				C1 = m2rd;
				
				rotation_reg_for_z = ({64'b0, C1}) << 1;
				temp_reg_after_C_rotation = rotation_reg_for_z[127:64] | rotation_reg_for_z[63:0];
				
				m2wr = 1'b1;
				m2wx = rotate_C_inc_x; m2wy = 3'b001;
				m2wd = temp_reg_after_C_rotation;				
				
				rotate_C_inc_x_d = rotate_C_inc_x + 1;
				
				if(rotate_C_inc_x >= 4) 
				begin
					 D_index_x_d = 0;
					 ns_perm = cal_D_read_C_from_bottom_row;
				end
				
			end
			
			cal_D_read_C_from_bottom_row:
			begin
				rotate_C_inc_x_d = 0;
				m2wr = 1'b0;
				
				index_1_for_D_calc = x_minus_1[D_index_x_d];
				
				m2rx = index_1_for_D_calc; m2ry = 3'b000;
				temp1_d = m2rd;				
				
				ns_perm = cal_D_read_rotated_C_from_top_row;				
			end
			
			
			cal_D_read_rotated_C_from_top_row:
			begin
				index_2_for_D_calc = x_plus_1[D_index_x_d];
				/*
				m2rx = index_2_for_D_calc; m2ry = 3'b001;
				D_2 = m2rd;
				temp_d = temp1 ^ D_2;
			*/
				m2rx = index_2_for_D_calc; m2ry = 3'b000;
				D_2 = m2rd;
				
				rotation_reg_for_z = ({64'b0, D_2}) << 1;
				temp_reg_after_C_rotation = rotation_reg_for_z[127:64] | rotation_reg_for_z[63:0];
				temp_d = temp1 ^ temp_reg_after_C_rotation;
				
			
				
				
				
				Theta_index_y_d = 0;
				ns_perm = Theta_A_run_through_y_index;			
			end
			
			
			Theta_A_run_through_y_index:
			begin
			/*
				// read A from m1 or m3:
				if(round_counter_i == 0)
				begin
					m1rx = D_index_x; m1ry = Theta_index_y;
					old_A_from_input = m1rd;
				end
				else
				begin
					m3rx = D_index_x; m3ry = Theta_index_y;
					old_A_from_input = m3rd;
				end
				*/
				
				// read from m3
				m3rx = D_index_x; m3ry = Theta_index_y;
				old_A_from_input = m3rd;
				
				
				new_A_for_Theta = old_A_from_input ^ temp;
				
				// write into m3:
				m3wr = 1'b1;
				m3wx = D_index_x; m3wy = Theta_index_y;
				m3wd = new_A_for_Theta;
				
				Theta_index_y_d = Theta_index_y + 1;
				
				if(Theta_index_y >= 4)
				begin
					//ns_perm = inc_D;
					
					D_index_x_d = D_index_x + 1;
					
					if(D_index_x >= 4)
					begin					
						//m3wr = 1'b0;
						
						Rho_Pi_index_x_d = 0;
						Rho_Pi_index_y_d = 0;
						ns_perm = Rho_Pi_inc_x;
										
						//ns_perm = Done_with_Theta_A_Onto_rho_pi;
					end
					else
						ns_perm = cal_D_read_C_from_bottom_row;
					
				end
					
			end
			 
			
			inc_D:
			begin				
				D_index_x_d = D_index_x + 1;
				
				if(D_index_x >= 4)
				begin					
					m3wr = 1'b0;
					
					Rho_Pi_index_x_d = 0;
					Rho_Pi_index_y_d = 0;
					ns_perm = Rho_Pi_inc_x;
									
					//ns_perm = Done_with_Theta_A_Onto_rho_pi;
				end
				else
					ns_perm = cal_D_read_C_from_bottom_row;
			end
			
			// no need for this anymore
			Done_with_Theta_A_Onto_rho_pi:
			begin			
				// indices of new location.
				Rho_Pi_index_x_d = 0;
				Rho_Pi_index_y_d = 0;
				/*
				if (round_counter_i == 0)
					enable_input_d = 1'b1;
					*/
				ns_perm = Rho_Pi_inc_x;
			end
			
			
			Rho_Pi_inc_x:
			begin
				
				// find old location corresponding to the new one:				
				index_x_of_theta_stage = x_of_x_plus_3_y[Rho_Pi_index_x][Rho_Pi_index_y];
				index_y_of_theta_stage = Rho_Pi_index_x;
				
							
				// get value of lane from theta stage:
				m3rx = index_x_of_theta_stage; m3ry = index_y_of_theta_stage;
				value_from_theta_stage = m3rd;
				
				// rho stage - calculate rotated value:				
				offset_t_value_for_Rho_stage = t_rho[index_y_of_theta_stage][index_x_of_theta_stage];
				
				// reuse rotation_reg_for_z:				
				rotation_reg_for_z = ({64'b0,value_from_theta_stage}) << offset_t_value_for_Rho_stage;
				temp_reg_after_C_rotation = rotation_reg_for_z[127:64] | rotation_reg_for_z[63:0];				
				
				//write into m2:
				m2wr = 1'b1;
				m2wy = Rho_Pi_index_y;
				m2wx = Rho_Pi_index_x;
				m2wd = temp_reg_after_C_rotation;
				
				Rho_Pi_index_x_d = Rho_Pi_index_x + 1;
					
				if (Rho_Pi_index_x >= 4)
					ns_perm = Rho_Pi_inc_y;			
			end
			
			
			Rho_Pi_inc_y:
			begin				
				Rho_Pi_index_y_d = Rho_Pi_index_y + 1;
				Rho_Pi_index_x_d = 0;
				
				if (Rho_Pi_index_y >= 4 ) 
				begin
					m2wr = 1'b0;
					
					Chi_Iota_index_x_d = 0;
					Chi_Iota_index_y_d = 0;
					ns_perm = Chi_Iota_inc_x_read_x;
					
//					ns_perm = Done_with_Rho_Pi;
				end
				else
					ns_perm = Rho_Pi_inc_x;
			end
			
			// no need
			Done_with_Rho_Pi:
			begin
				Chi_Iota_index_x_d = 0;
				Chi_Iota_index_y_d = 0;
				ns_perm = Chi_Iota_inc_x_read_x;
			end
			
			
			Chi_Iota_inc_x_read_x:
			begin
				m2rx = Chi_Iota_index_x; m2ry = Chi_Iota_index_y;
				temp_d = m2rd;
				ns_perm = Chi_Iota_inc_x_read_x_plus_1;
			end
			
			
			Chi_Iota_inc_x_read_x_plus_1:
			begin
				index_2_for_Chi_Iota = x_plus_1[Chi_Iota_index_x];
				
				m2rx = index_2_for_Chi_Iota; m2ry = Chi_Iota_index_y;
				temp1_d = ~ m2rd;								
				ns_perm = Chi_Iota_inc_x_read_x_plus_2;								
			end
			
			
			Chi_Iota_inc_x_read_x_plus_2:
			begin
				index_3_for_Chi_Iota = x_plus_2[Chi_Iota_index_x];
				
				m2rx = index_3_for_Chi_Iota; m2ry = Chi_Iota_index_y;
				temp_variable_for_Chi_Iota_2 = m2rd;
				
				temp_variable_for_Chi_Iota_3 = temp ^ (temp1 & temp_variable_for_Chi_Iota_2);
				
				if((Chi_Iota_index_x == 0) && ( Chi_Iota_index_y == 0))
					iota_value = temp_variable_for_Chi_Iota_3 ^ cmx[round_counter_i];
				else
					iota_value = temp_variable_for_Chi_Iota_3;
				
				// write into m3:
				m3wr = 1'b1;
				m3wx = Chi_Iota_index_x; m3wy = Chi_Iota_index_y;
				m3wd = iota_value;
				
				if(round_counter_i == 23)
				begin
					m4wr = 1'b1;
					m4wx = Chi_Iota_index_x; m4wy = Chi_Iota_index_y;
					m4wd = iota_value;					
				end
				
				Chi_Iota_index_x_d = Chi_Iota_index_x + 1;
				
				if(Chi_Iota_index_x >= 4)
					ns_perm = Chi_Iota_inc_y;
				else
					ns_perm = Chi_Iota_inc_x_read_x;
			end
			
			
			Chi_Iota_inc_y:
			begin															
				Chi_Iota_index_y_d = Chi_Iota_index_y + 1;
				Chi_Iota_index_x_d = 0;
				
				if(Chi_Iota_index_y >= 4)
				begin
					m3wr = 1'b0; m2wr = 1'b0; m4wr = 1'b0;
//					ns_perm = Done_with_Chi_Iota;
					
					
					m2wr = 1'b0;
					round_counter_i_d = round_counter_i + 1;
					
					if(round_counter_i_d > 23)
					begin
						done_with_prev_perm_d = 1'b1;
						enable_output_d = 1'b1;
						ns_perm = R_perm;
					end
					else
					begin					
						C_index_x_d = 0;
						C_index_y_d = 0;
						done_with_prev_perm_d = 1'b0;
						ns_perm = After_Iota_write_C_inc_y_read_from_mem3;
					end
					
				end
				else 
					ns_perm = Chi_Iota_inc_x_read_x;			
			end
			
			
			Done_with_Chi_Iota:
			begin
				m2wr = 1'b0;
				round_counter_i_d = round_counter_i + 1;
				
				if(round_counter_i_d > 23)
				begin
					done_with_prev_perm_d = 1'b1;
					enable_output_d = 1'b1;
					ns_perm = R_perm;
				end
				else
				begin					
					C_index_x_d = 0;
					C_index_y_d = 0;
					done_with_prev_perm_d = 1'b0;
					ns_perm = After_Iota_write_C_inc_y_read_from_mem3;
				end
			end
			
			
			
			After_Iota_write_C_inc_y_read_from_mem3:
			begin
				m3rx = C_index_x; m3ry = C_index_y;
				
				if (C_index_y == 0)
					temp_d = m3rd;
				else
					temp_d = temp ^ m3rd;

				C_index_y_d = C_index_y + 1;
				
				if(C_index_y >= 4)
					ns_perm = After_Iota_write_C_into_mem2;			
			end
			
			
			After_Iota_write_c_inc_x:
			begin
				C_index_x_d = C_index_x + 1;
				C_index_y_d = 0;
				
				if (C_index_x >= 4)
				begin
					C_index_x_d = 0;
//					ns_perm = write_rotated_C_into_mem2;
					D_index_x_d = 0;
					ns_perm = cal_D_read_C_from_bottom_row;
				end
				else
					ns_perm = After_Iota_write_C_inc_y_read_from_mem3;			
			end
			
			
			After_Iota_write_C_into_mem2:
			begin
				m2wr = 1'b1;
				m2wx = C_index_x; m2wy = 3'b000;
				m2wd = temp;
//				ns_perm = After_Iota_write_c_inc_x;	
					
				C_index_x_d = C_index_x + 1;
				C_index_y_d = 0;
				
				if (C_index_x >= 4)
				begin
					C_index_x_d = 0;
//					ns_perm = write_rotated_C_into_mem2;
					D_index_x_d = 0;
					ns_perm = cal_D_read_C_from_bottom_row;
				end
				else
					ns_perm = After_Iota_write_C_inc_y_read_from_mem3;
				
				
				
				
							
			end
			
			
		
		endcase
		
		case(cs_output)
			R_output:
			begin
				output_index_x_d = 0;
				output_index_y_d = 0;
				pushout = 1'b0;
				dout = 64'b0;
				firstout = 1'b0;
			
			
				if (enable_output)
				begin
					ns_output = output_inc_x;
//					ns_output = Write_output;
				end
				else
					ns_output = R_output;				
			end
			
			Write_output:
			begin
				output_index_x_d = 0; output_index_y_d = 0; dout = 64'b0;
				ns_output = output_inc_x;
			end
			
			
			output_inc_x:
			begin
				pushout = 1'b1;
				
				if ((output_index_x == 0) && (output_index_y == 0))
					firstout = 1'b1;
				else 
					firstout = 1'b0;
					
				if( stopout == 0)
				begin
					//pushout = 1'b1;
					/*
					if ((output_index_x == 0) && (output_index_y == 0))
						firstout = 1'b1;
					else 
						firstout = 1'b0;
						*/
						
					m4rx = output_index_x; m4ry = output_index_y;
					dout = m4rd;	
				
			
					output_index_x_d = output_index_x + 1;
					
					if(output_index_x_d >= 4)
						ns_output = output_inc_y;										
				end
				//else
					//pushout = 1'b0;
			end
			
			
			output_inc_y:
			begin
				pushout = 1'b1;
//				firstout = 1'b0;
				if(stopout == 0)
				begin
					pushout = 1'b1;
					
					m4rx = output_index_x; m4ry = output_index_y;
					dout = m4rd;
					
					output_index_y_d = output_index_y + 1;
					output_index_x_d = 0;
					
					if(output_index_y >= 4)
						ns_output = Done_reading;
					else 
						ns_output = output_inc_x;
				end
				//else
					//pushout = 1'b0;			
			end
			
			
			Done_reading:
			begin
				pushout = 1'b0;
				firstout = 1'b0;
				dout = 64'b0;
				
				enable_output_d = 1'b0;
				ns_output = R_output;				
			end
			
		endcase
		
	end
	
	
	
	
	
	// Sequential Logic
	always @(posedge clk or posedge rst)
	begin
		if (rst)
		begin
			cs_input <= R_input;
			cs_perm <= R_perm;
			cs_output <= R_output;
			
			enable_input <= 1'b1;
			enable_perm <= 0;
			enable_output <= 0;
			
			done_with_prev_perm <= 1'b1;
			
			stopin1 <= 0;
			
		//	stopin <= 1;
			
			input_index_x <= 0;
			input_index_y <= 0;	
			rotate_C_inc_x <= 0;
			D_index_x <= 0;
			Theta_index_x <= 0;
			Theta_index_y <= 0;
			Rho_Pi_index_x <= 0;
			Rho_Pi_index_y <= 0;
			Chi_Iota_index_x <= 0;
			Chi_Iota_index_y <= 0;
			round_counter_i <= 0;
			output_index_x <= 0;
			output_index_y <= 0;
			temp <= 0;
			temp1 <= 0;
			C_index_x <= 0;
			C_index_y <= 0;
			start_writing <= 0;
			
		end
		else 
		begin
			cs_input <= #1 ns_input;
			cs_perm <= #1 ns_perm;
			cs_output <= #1 ns_output;
			
			enable_input <= #1 enable_input_d;
			enable_perm <= #1 enable_perm_d;
			enable_output <= #1 enable_output_d;
			
			done_with_prev_perm <= #1 done_with_prev_perm_d;
			
			stopin1 <= #1 stopin1_d;
			
			//stopin <= #1 stopin_d;
			
			input_index_x <= #1 input_index_x_d;
			input_index_y <= #1 input_index_y_d;
			rotate_C_inc_x <= #1 rotate_C_inc_x_d;
			D_index_x <= #1 D_index_x_d;
			Theta_index_x <= #1 Theta_index_x_d;
			Theta_index_y <= #1 Theta_index_y_d;
			Rho_Pi_index_x <= #1 Rho_Pi_index_x_d;
			Rho_Pi_index_y <= #1 Rho_Pi_index_y_d;
			Chi_Iota_index_x <= #1 Chi_Iota_index_x_d;
			Chi_Iota_index_y <= #1 Chi_Iota_index_y_d;
			round_counter_i <= #1 round_counter_i_d;
			output_index_x <= #1 output_index_x_d;
			output_index_y <= #1 output_index_y_d;
			temp <= #1 temp_d;
			temp1 <= #1 temp1_d;
			C_index_x <= #1 C_index_x_d;
			C_index_y <= #1 C_index_y_d;
			start_writing <= #1 start_writing_d;
			

		end
	
	end 

endmodule : perm_blk








