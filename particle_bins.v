////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Module: particle_bins
//			Store the particle information in each bins, during force evaluation, it sends out the particle information one by one to force evaluation
//			During motion update process, it will take new particles from neighboring bins or from its original bin, and write to the second set of memory module
//			Thus, the user don't need to manage the replacement scheme. Which also serves as double buffer mechanism
//			The outgoing particle ports give priority to passthrough particles over local request
//
// Input/Output Singals:
//			rd_addr: this signal is received from the force evaluation module
//			evaluation_particle_read_out: particle data output for force evaluation
//			evaluation_particle_valid: Valid flag for read out particle data
//			motion_update_enable: this one should set as high the entire motion update process. The falling edge of which, signify the ending of motion update, then the memory sel signal is flip
//			incom_particle_data: the new incoming particle data
//			incom_particle_data_valid: only when this signal is high, the write address will increment by one
//			incom_particle_data_target_bin_x: the targeting bin of the incoming particle, use this to determine if the current the bin is the destination or just a intermediate hop
//			particle_data_out: the output particle information for force evaluation
//			particle_out_valid: output particle valid
//			particle_data_dest_bin: output particle's destination bin
//			local_motion_update_done: set this flag when the local particles are all processed. When all the bins have this flag set, then wait for a coupe more cycles to make sure all the intermediate particles reached its desitnation. Then the full motion update process is done.
//
// By: Chen Yang
// 08/28/2018
// CAAD Lab, Boston University
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module particle_bins
(
	clk,
	rst_n,														// reset on low
	// Force evaluation related signal
	rd_addr,														// Input, Particle read address during force evaluation
	evaluation_particle_read_out,							// Output, Particle data information for force evaluation
	evaluation_particle_valid,								// Output, Valid flag for read out particle data
	// Motion update enable signal
	motion_update_enable,									// Input, This signal should keep high the entire motion update process
																	// When detect the falling edge, signifies the end of the motion update process
	// Incoming particles information
	incom_particle_data_valid,								// Input
	incom_particle_data_target_bin_x,					// Input, Incoming particle destination bin
	incom_particle_data_target_bin_y,					// Input
	incom_particle_data_target_bin_z,					// Input
	incom_particle_data_in,									// Input
	particle_input_available,								// Output, let the router know if the memory module is ready to take a new particle information
	// Out going particles information
	particle_output_available,								// Input, Only send out data if this flag is high
	particle_data_out,										// Output, particle information
	particle_out_valid,										// Output, particle valid
	particle_data_dest_bin_x,								// Output, particle destination bin
	particle_data_dest_bin_y,								// Output
	particle_data_dest_bin_z,								// Output
	// Output to controllers
	local_motion_update_done								// Output, When the motion update of all the local particles are done, set this flag, when all the bins finished, then the motion update process is finished
);

	parameter BIN_ID_X = 0;
	parameter BIN_ID_Y = 0;
	parameter BIN_ID_Z = 0;
	parameter BIN_ADDR_WIDTH = 4;
	parameter DATA_WIDTH = 32*5;
	parameter ADDR_WIDTH = 7;
	parameter BIN_DEPTH = 128;
	parameter BIN_OFFSET_WIDTH = 3;						// log(CUTOFF) / log 2
	
	// Input & Output ports
	input clk;
	input rst_n;																// reset on low
	input [ADDR_WIDTH-1:0] rd_addr;
	output reg [DATA_WIDTH-1:0] evaluation_particle_read_out;
	output reg evaluation_particle_valid;
	input motion_update_enable;											// This signal should keep high the entire motion update process
																					// When detect the falling edge, signifies the end of the motion update process
	input incom_particle_data_valid;
	input [BIN_ADDR_WIDTH-1:0] incom_particle_data_target_bin_x;
	input [BIN_ADDR_WIDTH-1:0] incom_particle_data_target_bin_y;
	input [BIN_ADDR_WIDTH-1:0] incom_particle_data_target_bin_z;
	input [DATA_WIDTH-1:0] incom_particle_data_in;
	output reg particle_input_available;
	input particle_output_available;
	output reg [DATA_WIDTH-1:0] particle_data_out;
	output reg particle_out_valid;
	output reg [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_x;
	output reg [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_y;
	output reg [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_z;
	output reg local_motion_update_done;								// Flag set as high when all the local particles are done processed for motion update
	
	// Intermediate signals
	reg [ADDR_WIDTH-1:0] NUM_PARTICLE_IN_BIN;							// Keeping track of how many particles are in the current bin
	reg local_particle_readout_valid;									// Signify if the local readout particle is valid
	reg prev_motion_update_enable;										// use this signal to detect the falling edge of motion update process, which means the end of the process
	reg [ADDR_WIDTH-1:0] wr_addr;
	reg rd_mem_sel;															// selection flag, decide which one of the double buffer is used for calculation
	
	wire [DATA_WIDTH-1:0] particle_data_out_0, particle_data_out_1, read_particle_data;
	reg [ADDR_WIDTH-1:0] motion_update_rd_addr;						// Read address during motion update, traverse all the particles in the bin
	reg [DATA_WIDTH-1:0] mem_wr_data;									// Write data to memory module, may come from input, or come from the reading memory module
	
	
	// Assign output data port
	assign read_particle_data = rd_mem_sel ? particle_data_out_1 : particle_data_out_0;
	// Assign output particle's destination bin
	wire [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_x_wire;
	wire [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_y_wire;
	wire [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_z_wire;
	assign particle_data_dest_bin_x_wire = read_particle_data[BIN_OFFSET_WIDTH+BIN_ADDR_WIDTH-1:BIN_OFFSET_WIDTH];
	assign particle_data_dest_bin_y_wire = read_particle_data[BIN_OFFSET_WIDTH+BIN_ADDR_WIDTH-1+32:BIN_OFFSET_WIDTH+32];
	assign particle_data_dest_bin_z_wire = read_particle_data[BIN_OFFSET_WIDTH+BIN_ADDR_WIDTH-1+64:BIN_OFFSET_WIDTH+64];
	
	// Assign memory module write enable signal
	wire ram_0_wren, ram_1_wren;
	assign ram_0_wren = motion_update_enable && rd_mem_sel;
	assign ram_1_wren = motion_update_enable && ~rd_mem_sel;
	
	// Assign memory module read address
	// During force evaluation, the rd_addr is from input
	// During motion update, the rd_addr is generated by FSM
	wire [ADDR_WIDTH-1:0] ram_0_addr, ram_1_addr;
	assign ram_0_addr = rd_mem_sel ? wr_addr : (motion_update_enable ? motion_update_rd_addr : rd_addr);
	assign ram_1_addr = rd_mem_sel ? (motion_update_enable ? motion_update_rd_addr : rd_addr) : wr_addr;
	
	
	// Assign read out particle data for evaluation
	always@(posedge clk)
		begin
		if(~rst_n)
			begin
			evaluation_particle_read_out <= 0;
			evaluation_particle_valid <= 1'b0;
			end
		// only output data during non motion update process
		else if(~motion_update_enable)
			begin
			evaluation_particle_read_out <= read_particle_data;
			evaluation_particle_valid <= 1'b1;
			end
		// Output invalid during motion update process
		else
			begin
			evaluation_particle_read_out <= 0;
			evaluation_particle_valid <= 1'b0;
			end
		end
	
	// Tracking the number of particles in the current bin
	always@(posedge clk)
		begin
		if(~rst_n)
			NUM_PARTICLE_IN_BIN <= 0;
		// Update the particles counter by the end of motion update
		else if(prev_motion_update_enable && ~motion_update_enable)
			NUM_PARTICLE_IN_BIN <= wr_addr;		
		end
	
	// Collect the previous motion_update_enable signal
	always@(posedge clk)
		begin
		if(~rst_n)
			prev_motion_update_enable <= 1'b0;
		else
			prev_motion_update_enable <= motion_update_enable;
		end
	
	// Select one of the double buffer for read out data
	always@(posedge clk)
		begin
		if(~rst_n)
			rd_mem_sel <= 1'b0;
		// if detected falling edge, then flip the sel signal
		else if (prev_motion_update_enable && ~motion_update_enable)
			rd_mem_sel <= ~rd_mem_sel;
		else
			rd_mem_sel <= rd_mem_sel;
		end
		
	// Write address for new incoming particle data
	always@(posedge clk)
		begin
		if(~rst_n || ~motion_update_enable)
			wr_addr <= 0;
		else if(motion_update_enable)
			begin
			// increment the wr_addr when the incoming particle has a destination that meets the corrent bin's coordinate
			if(incom_particle_data_valid && incom_particle_data_target_bin_x == BIN_ID_X && incom_particle_data_target_bin_y == BIN_ID_Y && incom_particle_data_target_bin_z == BIN_ID_Z)
				wr_addr <= wr_addr + 1'b1;
			// increment the wr_addr when the local particles remains in the same bin
			else if(local_particle_readout_valid && particle_data_dest_bin_x_wire == BIN_ID_X && particle_data_dest_bin_y_wire == BIN_ID_Y && particle_data_dest_bin_z_wire == BIN_ID_Z)
				wr_addr <= wr_addr + 1'b1;
			else
				wr_addr <= wr_addr;
			end
		else
			wr_addr <= wr_addr;
		end
	
	
	// Generating the input available flag
	// This one is combinational logic, since if the output is not available, the input avaible should set as low immediately
	always@(*)
		begin
		// During reset, the input is always available
		if(~rst_n)
			begin
			particle_input_available <= 1'b1;
			end
		// Only need to change is flag during motion update process
		else if(motion_update_enable)
			begin
			// if the incoming particle reach its destination, then it will write into the memory module immediately, then the input is available next cycle
			if(incom_particle_data_valid && incom_particle_data_target_bin_x == BIN_ID_X && incom_particle_data_target_bin_y == BIN_ID_Y && incom_particle_data_target_bin_z == BIN_ID_Z)
				begin
				particle_input_available <= 1'b1;
				end
			// if this is just the intermediate bin, then the particle will write to the output port, the input aviablity is depending on whether this particle can be send out in time
			else if(incom_particle_data_valid)
				begin
				particle_input_available <= particle_output_available;
				end
			// If there's no particles incoming from other bins, then the output port is available
			else
				begin
				particle_input_available <= 1'b1;
				end
			end
		// If not in motion update process, then the input is always available
		else
			begin
			particle_input_available <= 1'b1;
			end
		end
	

	// Generating the read address to read out the particles in the current bin
	always@(posedge clk)
		begin
		if(~rst_n)
			begin
			motion_update_rd_addr <= 0;
			local_particle_readout_valid <= 1'b0;
			end
		// Only works during the motion update process, and the read address is less than the total number of particles in the current bin
		else if(motion_update_enable)
			begin
			// Only traverse the valid # of particles in the bin
			if (motion_update_rd_addr < NUM_PARTICLE_IN_BIN)
				begin
				// Set the readout particle as valid
				local_particle_readout_valid <= 1'b1;
				
				// if the incoming particle reach its destination, the output port is free, only when the readout particle want to move to other bins, then read address increment
				if(incom_particle_data_valid && incom_particle_data_target_bin_x == BIN_ID_X && incom_particle_data_target_bin_y == BIN_ID_Y && incom_particle_data_target_bin_z == BIN_ID_Z)
					begin
					// if the new readout data also want to write back, then it should pause and let the incoming particle write in first
					if(local_particle_readout_valid && particle_data_dest_bin_x_wire == BIN_ID_X && particle_data_dest_bin_y_wire == BIN_ID_Y && particle_data_dest_bin_z_wire == BIN_ID_Z)
						begin
						motion_update_rd_addr <= motion_update_rd_addr;
						end
					// when the new readout data want to write to other bins, and the outut port is available, then increment the read address
					else if(local_particle_readout_valid && particle_output_available)
						begin
						motion_update_rd_addr <= motion_update_rd_addr + 1'b1;
						end
					// If the local readout particle is not valid, or the output is not available, then read address remain
					else
						begin
						motion_update_rd_addr <= motion_update_rd_addr;
						end
					end
				// if this is just the intermediate bin, which means the output port is occupied by passthrough, only when the new readout particle is ready to write back, then increment read address
				else if(incom_particle_data_valid)
					begin
					// if the new readout data want to write back, then increment the read address
					if(local_particle_readout_valid && particle_data_dest_bin_x_wire == BIN_ID_X && particle_data_dest_bin_y_wire == BIN_ID_Y && particle_data_dest_bin_z_wire == BIN_ID_Z)
						begin
						motion_update_rd_addr <= motion_update_rd_addr + 1'b1;
						end
					// when the new readout data is also want to move to other bins, then it will hold
					else
						begin
						motion_update_rd_addr <= motion_update_rd_addr;
						end
					end
				// If there's no particles incoming from other bins, then increment the read address by one when output is available
				else
					begin
					if(local_particle_readout_valid && particle_output_available)
						begin
						motion_update_rd_addr <= motion_update_rd_addr + 1'b1;
						end
					else
						begin
						motion_update_rd_addr <= motion_update_rd_addr;
						end
					end
				end
			// if the read address exceeds the particle count, keep the current read_address
			else
				begin
				local_particle_readout_valid <= 1'b0;
				motion_update_rd_addr <= motion_update_rd_addr;
				end
			end
		// If not in motion update process, then the read address remain 0
		else
			begin
			local_particle_readout_valid <= 1'b0;
			motion_update_rd_addr <= 0;
			end
		end
	
	// Assign the finish flag of local motion update
	always@(posedge clk)
		begin
		if(~rst_n)
			local_motion_update_done <= 1'b0;
		// Only set this during the motion update process
		else if(motion_update_enable)
			begin
			// When the read address is larger than the # of particles in the bin, then set the flag
			if(motion_update_rd_addr >= NUM_PARTICLE_IN_BIN)
				local_motion_update_done <= 1'b1;
			else
				local_motion_update_done <= 1'b0;
			end
		else
			local_motion_update_done <= 1'b0;
		end

	// Assign the output ports for out going particles (priority is given to passthrough particles over local request)
	always@(posedge clk)
		begin
		if(~rst_n)
			begin
			particle_data_out <= 0;
			particle_out_valid <= 1'b0;
			particle_data_dest_bin_x <= 0;
			particle_data_dest_bin_y <= 0;
			particle_data_dest_bin_z <= 0;
			end
		// Only works during the motion update process 
		else if(motion_update_enable)
			begin
			// Only update the output port when the output is available
			if(particle_output_available)
				begin
				// if the incoming particle reach its destination, the output port is free
				if(incom_particle_data_valid && incom_particle_data_target_bin_x == BIN_ID_X && incom_particle_data_target_bin_y == BIN_ID_Y && incom_particle_data_target_bin_z == BIN_ID_Z)
					begin
					// if the new readout data also want to write back, then the output port is not occupied
					if(local_particle_readout_valid && particle_data_dest_bin_x_wire == BIN_ID_X && particle_data_dest_bin_y_wire == BIN_ID_Y && particle_data_dest_bin_z_wire == BIN_ID_Z)
						begin
						particle_data_out <= 0;
						particle_out_valid <= 1'b0;
						particle_data_dest_bin_x <= 0;
						particle_data_dest_bin_y <= 0;
						particle_data_dest_bin_z <= 0;
						end
					// when the new readout data want to write to other bins, then assign the output port
					else if(local_particle_readout_valid)
						begin
						particle_data_out <= read_particle_data;
						particle_out_valid <= 1'b1;
						particle_data_dest_bin_x <= particle_data_dest_bin_x_wire;
						particle_data_dest_bin_y <= particle_data_dest_bin_y_wire;
						particle_data_dest_bin_z <= particle_data_dest_bin_z_wire;
						end
					// When the local readout particle is not valid, then the output is idle
					else
						begin
						particle_data_out <= 0;
						particle_out_valid <= 1'b0;
						particle_data_dest_bin_x <= 0;
						particle_data_dest_bin_y <= 0;
						particle_data_dest_bin_z <= 0;
						end
					end
				// if this is just the intermediate bin, the output port is occupied by passthrough
				else if(incom_particle_data_valid)
					begin
					particle_data_out <= incom_particle_data_in;
					particle_out_valid <= 1'b1;
					particle_data_dest_bin_x <= incom_particle_data_target_bin_x;
					particle_data_dest_bin_y <= incom_particle_data_target_bin_y;
					particle_data_dest_bin_z <= incom_particle_data_target_bin_z;
					end
				// if there is no valid data from the neighbor bins	
				else
					begin
					// if the new readout data want to write back, then the output port is not occupied
					if(incom_particle_data_valid && particle_data_dest_bin_x_wire == BIN_ID_X && particle_data_dest_bin_y_wire == BIN_ID_Y && particle_data_dest_bin_z_wire == BIN_ID_Z)
						begin
						particle_data_out <= 0;
						particle_out_valid <= 1'b0;
						particle_data_dest_bin_x <= 0;
						particle_data_dest_bin_y <= 0;
						particle_data_dest_bin_z <= 0;
						end
					// when the new readout data has other desitnation bins, then send it to the output
					else if(incom_particle_data_valid)
						begin
						particle_data_out <= read_particle_data;
						particle_out_valid <= 1'b1;
						particle_data_dest_bin_x <= particle_data_dest_bin_x_wire;
						particle_data_dest_bin_y <= particle_data_dest_bin_y_wire;
						particle_data_dest_bin_z <= particle_data_dest_bin_z_wire;
						end
					// if the local readout particle is not valid, the leave the output idle
					else
						begin
						particle_data_out <= 0;
						particle_out_valid <= 1'b0;
						particle_data_dest_bin_x <= 0;
						particle_data_dest_bin_y <= 0;
						particle_data_dest_bin_z <= 0;
						end
					end
				end
			// When the output is not available, then the output port data remains
			else
				begin
				particle_data_out <= particle_data_out;
				particle_out_valid <= particle_out_valid;
				particle_data_dest_bin_x <= particle_data_dest_bin_x;
				particle_data_dest_bin_y <= particle_data_dest_bin_y;
				particle_data_dest_bin_z <= particle_data_dest_bin_z;
				end
			end
			
		// If not in motion update process, then the readout particle is directly connected to the memory module readout particle
		else
			begin
			particle_data_out <= read_particle_data;
			particle_out_valid <= 1'b1;
			particle_data_dest_bin_x <= 0;
			particle_data_dest_bin_y <= 0;
			particle_data_dest_bin_z <= 0;
			end
		end
	
	
	// Assign the memory module write data
	// Data may come from input particles, or from local memory
	// The 2 memory module can share the same input data singal, since they are controlled by different write enable signal
	always@(posedge clk)
		begin
		if(~rst_n)
			begin
			mem_wr_data <= 0;
			end
		// Only works during the motion update process 
		else if(motion_update_enable)
			begin
			// if the incoming particle reach its destination, then write the incoming particle into the memory
			if(incom_particle_data_valid && incom_particle_data_target_bin_x == BIN_ID_X && incom_particle_data_target_bin_y == BIN_ID_Y && incom_particle_data_target_bin_z == BIN_ID_Z)
				begin
				mem_wr_data <= incom_particle_data_in;
				end
			// if the local particle want to write back, then assign
			else if(incom_particle_data_valid && particle_data_dest_bin_x_wire == BIN_ID_X && particle_data_dest_bin_y_wire == BIN_ID_Y && particle_data_dest_bin_z_wire == BIN_ID_Z)
				begin
				mem_wr_data <= read_particle_data;
				end
			// otherwise, write nothing
			else
				begin
				mem_wr_data <= 0;
				end
			end
		// If not in motion update process, then not writing data into the memory
		// ??????????? May need to update the position information here, will deal with it later ????????????????
		else
			begin
			mem_wr_data <= 0;
			end
		end
	
	
	
	
	// Memory module 0
	RAM_1_Port 
	#(
		.WIDTH(DATA_WIDTH),
		.DEPTH(BIN_DEPTH),
		.ADDR_WIDTH(ADDR_WIDTH)
	)
	BIN_0
	(
		.address(ram_0_addr),
		.clock(clk),
		.data(mem_wr_data),
		.wren(ram_0_wren),
		.q(particle_data_out_0)
	);
	
	// Memory module 1
	RAM_1_Port 
	#(
		.WIDTH(DATA_WIDTH),
		.DEPTH(BIN_DEPTH),
		.ADDR_WIDTH(ADDR_WIDTH)
	)
	BIN_1
	(
		.address(ram_1_addr),
		.clock(clk),
		.data(mem_wr_data),
		.wren(ram_1_wren),
		.q(particle_data_out_1)
	);
	

endmodule