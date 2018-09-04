////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Module: particle_bin_top
//			Encapsulate the particle_bin
//			Handles the communication with the neighboring bins:
//					Performs arbitration on the incoming particles
//					Determine the outgoing bin based on the destination bin of the particles
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
//			particle_input_available_to_neighbors:
//
//
// By: Chen Yang
// 08/30/2018
// CAAD Lab, Boston University
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module particle_bin_top(
	clk,
	rst_n,
	// Force evaluation related signal
	rd_addr,														// Input, Particle read address during force evaluation
	evaluation_particle_read_out,							// Output, Particle data information for force evaluation
	evaluation_particle_valid,								// Output, Valid flag for read out particle data
	// Motion update related signals
	motion_update_enable,									// Input, router only work during motion update process
	local_motion_update_done,								// Output, When the motion update of all the local particles are done, set this flag, when all the bins finished, then the motion update process is finished
	// Ports connected to all the neighboring bins
	particle_input_available_to_neighbors,				// Output, to all the neighboring bins, let neighbor bins know if the current bin is ready to take particles
	particle_output_available_from_neighbors,			// Input, from all neighboring bins, only send out data to that bin if the flag is high
	global_incom_particle_data_valid,					// Input, incoming particle valid signal from all neighbor bins
	global_incom_particle_data_in,						// Input, incoming particle data from all neighbor bin
	global_outgoing_particle_data_valid,				// Output, particle valid flag to neighbor bins
	global_particle_data_out								// Output, particle data send to neighbor bins
);

	parameter BIN_ID_X = 1;
	parameter BIN_ID_Y = 1;
	parameter BIN_ID_Z = 1;
	parameter BIN_ADDR_WIDTH = 4;
	parameter DATA_WIDTH = 32*5;
	parameter ADDR_WIDTH = 7;
	parameter BIN_DEPTH = 128;
	parameter BIN_OFFSET_WIDTH = 3;	
	
	parameter NUM_NEIGHBOR_BIN = 6;
	parameter NEIGHBOR_BIN_ADDR_WIDTH = 3;				// log(NUM_NEIGHBOR_BIN)/log(2)

	// Input & Output ports
	input clk;
	input rst_n;
	input [ADDR_WIDTH-1:0] rd_addr;
	output [DATA_WIDTH-1:0] evaluation_particle_read_out;
	output evaluation_particle_valid;
	input motion_update_enable;																	// Flag set as high when all the local particles are done processed for motion update. When detect the falling edge, signifies the end of the motion update process
	output local_motion_update_done;											
	output reg particle_input_available_to_neighbors;
	input [NUM_NEIGHBOR_BIN-1:0] particle_output_available_from_neighbors;
	input [NUM_NEIGHBOR_BIN-1:0] global_incom_particle_data_valid;
	input [NUM_NEIGHBOR_BIN*DATA_WIDTH-1:0] global_incom_particle_data_in;
	output reg [NUM_NEIGHBOR_BIN-1:0] global_outgoing_particle_data_valid;
	output reg [NUM_NEIGHBOR_BIN*DATA_WIDTH-1:0] global_particle_data_out;
	
	
	// Ports connecting to particle_bins
	// Incoming particles information
	reg incom_particle_data_valid;
	wire [BIN_ADDR_WIDTH-1:0] incom_particle_data_target_bin_x;
	wire [BIN_ADDR_WIDTH-1:0] incom_particle_data_target_bin_y;
	wire [BIN_ADDR_WIDTH-1:0] incom_particle_data_target_bin_z;
	reg [DATA_WIDTH-1:0] incom_particle_data_in;
	wire particle_input_available;
	// Outgoing particles information
	reg particle_output_available;
	wire [DATA_WIDTH-1:0] particle_data_out;
	wire particle_out_valid;
	wire [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_x;
	wire [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_y;
	wire [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_z;

	
	// Register buffer for the incoming valid particle data
	reg [NUM_NEIGHBOR_BIN*DATA_WIDTH-1:0] reg_incoming_particle_data;
	reg [NUM_NEIGHBOR_BIN-1:0] reg_global_incom_particle_data_valid;
	
	// Signals for arbitration
	wire [NUM_NEIGHBOR_BIN-1:0] arbiter;
	// Find the least significant 1 bit of the data valid flag
	assign arbiter = ((~reg_global_incom_particle_data_valid) + 1'b1) & reg_global_incom_particle_data_valid;
	/*
	always@(*)
		begin
		if(~rst_n)
			arbiter <= 0;
		else
			arbiter <= ((~reg_global_incom_particle_data_valid) + 1'b1) & reg_global_incom_particle_data_valid;
		end
	*/
	
	// Create dummy all 0 wires for concatenation use
	wire [DATA_WIDTH-1:0] dummy_all_0_particle_data;
	assign dummy_all_0_particle_data = 0;
	
	// Signals for determine outgoing particle destination
	// X_POS: 0
	// Y_POS: 1
	// Z_POS: 2
	// X_NEG: 3
	// Y_NEG: 4
	// Z_NEG: 5
	reg [NEIGHBOR_BIN_ADDR_WIDTH-1:0] outgoing_port;
	always@(*)
		begin
		if(~rst_n)
			begin
			outgoing_port <= 0;
			end
		else
			// Destination calculation process (DOR routing)
			begin
			// X direction
			if(particle_data_dest_bin_x != BIN_ID_X)
				begin
				// X_POS
				if (particle_data_dest_bin_x > BIN_ID_X)
					outgoing_port <= 0;
				// X_NEG
				else
					outgoing_port <= 3;
				end
			// Y direction
			else if(particle_data_dest_bin_y != BIN_ID_Y)
				begin
				// Y_POS
				if (particle_data_dest_bin_y > BIN_ID_Y)
					outgoing_port <= 1;
				// Y_NEG
				else
					outgoing_port <= 4;
				end
			// Z direction
			else
				begin
				// Z_POS
				if (particle_data_dest_bin_z > BIN_ID_Z)
					outgoing_port <= 2;
				// Z_NEG
				else
					outgoing_port <= 5;
				end
			end
		end
	
	// assign the wires to particle_bin
	assign incom_particle_data_target_bin_x = incom_particle_data_in[BIN_OFFSET_WIDTH+BIN_ADDR_WIDTH-1:BIN_OFFSET_WIDTH];
	assign incom_particle_data_target_bin_y = incom_particle_data_in[BIN_OFFSET_WIDTH+BIN_ADDR_WIDTH-1+32:BIN_OFFSET_WIDTH+32];
	assign incom_particle_data_target_bin_z = incom_particle_data_in[BIN_OFFSET_WIDTH+BIN_ADDR_WIDTH-1+64:BIN_OFFSET_WIDTH+64]; 
		
	// Handles incoming data, and output available signal
	// Principle: Every cycle, check the incoming data, if one of the port has valid incoming particles, then clear the input available signal particle_input_available_to_neighbors
	// All the valid data will be registered, and a round-robin mechanism is used to take those valid particle data one by one
	// When all the registered data is processed (eigher load into the memory or send out), then set particle_input_available_to_neighbors when the memory is ready to take new particles
	always@(posedge clk)
		begin
		if(~rst_n)
			begin
			particle_input_available_to_neighbors <= 1'b1;
			reg_incoming_particle_data <= 0;
			reg_global_incom_particle_data_valid <= 0;
			// signals to particle bin
			incom_particle_data_in <= 0;
			incom_particle_data_valid <= 1'b0;
			end
		// Only handle the incoming particles during motion update process
		else if(motion_update_enable)
			begin
			// When there are valid data on the incoming port and the input is available, then register those data
			if(particle_input_available_to_neighbors && global_incom_particle_data_valid != 0)
				begin
				particle_input_available_to_neighbors <= 1'b0;
				reg_incoming_particle_data <= global_incom_particle_data_in;
				reg_global_incom_particle_data_valid <= global_incom_particle_data_valid;
				// signals to particle bin
				incom_particle_data_in <= 0;
				incom_particle_data_valid <= 1'b0;
				end
			// When the input is available, but no valid data incoming, then keep waiting for a valid particle to arrive
			else if(particle_input_available_to_neighbors)
				begin
				particle_input_available_to_neighbors <= 1'b1;
				reg_incoming_particle_data <= 0;
				reg_global_incom_particle_data_valid <= 0;
				// signals to particle bin
				incom_particle_data_in <= 0;
				incom_particle_data_valid <= 1'b0;
				end
			// The input is not availale and there are registered data being processed
			// During this process, keep checking the reg_global_incom_particle_data_valid flag, if this one turns to 0, means all the registered particles has been processed. At this point, check if the memory module is available for new input. If so, then reset the input available flag
			else if(~particle_input_available_to_neighbors && reg_global_incom_particle_data_valid != 0)
				begin
				particle_input_available_to_neighbors <= 1'b0;					// During the processing, the input available should be 0 to prevent new particles coming in
				reg_incoming_particle_data <= reg_incoming_particle_data;	// the registered particle data keeps
				// Only perform arbitration and select input data when the particle bin is ready to take incoming particles
				if(particle_input_available)
					begin
					// Clear the selected bit from the valid register
					reg_global_incom_particle_data_valid <= reg_global_incom_particle_data_valid & (~arbiter);
					// Set the particle as valid
					incom_particle_data_valid <= 1'b1;
					// Arbitration process to select from one of the valid inputs in a round-robin fashion
					// assign the data to particle bin
					case(arbiter)
						1: begin
							incom_particle_data_in <= reg_incoming_particle_data[DATA_WIDTH*1-1:DATA_WIDTH*0];
							incom_particle_data_valid <= 1'b1;
							reg_global_incom_particle_data_valid <= reg_global_incom_particle_data_valid & (~arbiter);
							end
						2: begin
							incom_particle_data_in <= reg_incoming_particle_data[DATA_WIDTH*2-1:DATA_WIDTH*1];
							incom_particle_data_valid <= 1'b1;
							reg_global_incom_particle_data_valid <= reg_global_incom_particle_data_valid & (~arbiter);
							end
						4: begin
							incom_particle_data_in <= reg_incoming_particle_data[DATA_WIDTH*3-1:DATA_WIDTH*2];
							incom_particle_data_valid <= 1'b1;
							reg_global_incom_particle_data_valid <= reg_global_incom_particle_data_valid & (~arbiter);
							end
						8: begin
							incom_particle_data_in <= reg_incoming_particle_data[DATA_WIDTH*4-1:DATA_WIDTH*3];
							incom_particle_data_valid <= 1'b1;
							reg_global_incom_particle_data_valid <= reg_global_incom_particle_data_valid & (~arbiter);
							end
						16: begin
							incom_particle_data_in <= reg_incoming_particle_data[DATA_WIDTH*5-1:DATA_WIDTH*4];
							incom_particle_data_valid <= 1'b1;
							reg_global_incom_particle_data_valid <= reg_global_incom_particle_data_valid & (~arbiter);
							end
						32: begin
							incom_particle_data_in <= reg_incoming_particle_data[DATA_WIDTH*6-1:DATA_WIDTH*5];
							incom_particle_data_valid <= 1'b1;
							reg_global_incom_particle_data_valid <= reg_global_incom_particle_data_valid & (~arbiter);
							end
						64: begin
							incom_particle_data_in <= reg_incoming_particle_data[DATA_WIDTH*7-1:DATA_WIDTH*6];
							incom_particle_data_valid <= 1'b1;
							reg_global_incom_particle_data_valid <= reg_global_incom_particle_data_valid & (~arbiter);
							end
						128: begin
							incom_particle_data_in <= reg_incoming_particle_data[DATA_WIDTH*8-1:DATA_WIDTH*7];
							incom_particle_data_valid <= 1'b1;
							reg_global_incom_particle_data_valid <= reg_global_incom_particle_data_valid & (~arbiter);
							end
						default: begin
							incom_particle_data_in <= 0;
							incom_particle_data_valid <= 1'b0;
							reg_global_incom_particle_data_valid <= reg_global_incom_particle_data_valid;
							end
					endcase
					end
				// If the particle bin is not ready for incoming data, then keep the original value
				else
					begin
					reg_global_incom_particle_data_valid <= reg_global_incom_particle_data_valid;
					// signals to particle bin
					incom_particle_data_in <= 0;
					incom_particle_data_valid <= 1'b0;
					end
				end
			// The input is not available, and there are no registered particles, which means the system is waiting for the memeory module to get ready for taking particles
			else
				begin
				particle_input_available_to_neighbors <= particle_input_available;
				reg_incoming_particle_data <= 0;
				reg_global_incom_particle_data_valid <= 0;
				// signals to particle bin
				incom_particle_data_in <= 0;
				incom_particle_data_valid <= 1'b0;
				end
			end
		// When not in motion update process
		else
			begin
			particle_input_available_to_neighbors <= 1'b1;
			reg_incoming_particle_data <= 0;
			reg_global_incom_particle_data_valid <= 0;
			// signals to particle bin
			incom_particle_data_in <= 0;
			incom_particle_data_valid <= 1'b0;
			end
		end
	
	// Handles outgoing data, and output available signal
	always@(posedge clk)
		begin
		if(~rst_n)
			begin
			global_outgoing_particle_data_valid <= 0;
			global_particle_data_out <= 0;
			particle_output_available <= 1'b0;
			end
		// Only need to send data to neighboring bins during motion update process
		else if(motion_update_enable)
			begin
			// Only send data if the particle bin has valid output
			if(particle_out_valid)
				begin
				case(outgoing_port)
					0: begin
						global_particle_data_out <= {dummy_all_0_particle_data, dummy_all_0_particle_data, dummy_all_0_particle_data, dummy_all_0_particle_data, dummy_all_0_particle_data, particle_data_out};
						global_outgoing_particle_data_valid <= 1;
						particle_output_available <= particle_output_available_from_neighbors[0];
						end
					1: begin
						global_particle_data_out <= {dummy_all_0_particle_data, dummy_all_0_particle_data, dummy_all_0_particle_data, dummy_all_0_particle_data, particle_data_out, dummy_all_0_particle_data};
						global_outgoing_particle_data_valid <= 2;
						particle_output_available <= particle_output_available_from_neighbors[1];
						end
					2: begin
						global_particle_data_out <= {dummy_all_0_particle_data, dummy_all_0_particle_data, dummy_all_0_particle_data, particle_data_out, dummy_all_0_particle_data, dummy_all_0_particle_data};
						global_outgoing_particle_data_valid <= 4;
						particle_output_available <= particle_output_available_from_neighbors[2];
						end
					3: begin
						global_particle_data_out <= {dummy_all_0_particle_data, dummy_all_0_particle_data, particle_data_out, dummy_all_0_particle_data, dummy_all_0_particle_data, dummy_all_0_particle_data};
						global_outgoing_particle_data_valid <= 8;
						particle_output_available <= particle_output_available_from_neighbors[3];
						end
					4: begin
						global_particle_data_out <= {dummy_all_0_particle_data, particle_data_out, dummy_all_0_particle_data, dummy_all_0_particle_data, dummy_all_0_particle_data, dummy_all_0_particle_data};
						global_outgoing_particle_data_valid <= 16;
						particle_output_available <= particle_output_available_from_neighbors[4];
						end
					5: begin
						global_particle_data_out <= {particle_data_out, dummy_all_0_particle_data, dummy_all_0_particle_data, dummy_all_0_particle_data, dummy_all_0_particle_data, dummy_all_0_particle_data};
						global_outgoing_particle_data_valid <= 32;
						particle_output_available <= particle_output_available_from_neighbors[5];
						end
					default: begin
						global_particle_data_out <= 0;
						global_outgoing_particle_data_valid <= 0;
						particle_output_available <= 1'b1;
						end
				endcase
				end
			// Output nothing when particle bin has nothing to send out
			else
				begin
				global_outgoing_particle_data_valid <= 0;
				global_particle_data_out <= 0;
				particle_output_available <= 1'b0;
				end
			end
		// If not in motion update process, then don't send out data
		else
			begin
			global_outgoing_particle_data_valid <= 0;
			global_particle_data_out <= 0;
			particle_output_available <= 1'b1;
			end
		end
	
	
	// Particle Bin Module
	particle_bins
	#(
		.BIN_ID_X(BIN_ID_X),
		.BIN_ID_Y(BIN_ID_Y),
		.BIN_ID_Z(BIN_ID_Z),
		.BIN_ADDR_WIDTH(BIN_ADDR_WIDTH),
		.DATA_WIDTH(DATA_WIDTH),
		.ADDR_WIDTH(ADDR_WIDTH),
		.BIN_DEPTH(BIN_DEPTH),
		.BIN_OFFSET_WIDTH(BIN_OFFSET_WIDTH)	
	)
	particle_bins
	(
		.clk(clk),
		.rst_n(rst_n),																			// reset on low
		// Force evaluation related signal
		.rd_addr(rd_addr),																	// Input, Particle read address during force evaluation
		.evaluation_particle_read_out(evaluation_particle_read_out),			// Output, Particle data information for force evaluation
		.evaluation_particle_valid(evaluation_particle_valid),					// Output, Valid flag for read out particle data
		// Motion update enable signal
		.motion_update_enable(motion_update_enable),									// Input
		// Incoming particles information
		.incom_particle_data_valid(incom_particle_data_valid),					// Input
		.incom_particle_data_target_bin_x(incom_particle_data_target_bin_x),	// Input, Incoming particle destination bin
		.incom_particle_data_target_bin_y(incom_particle_data_target_bin_y),	// Input
		.incom_particle_data_target_bin_z(incom_particle_data_target_bin_z),	// Input
		.incom_particle_data_in(incom_particle_data_in),							// Input
		.particle_input_available(particle_input_available),						// Output, let the router know if the memory module is ready to take a new particle information
		// Out going particles information
		.particle_output_available(particle_output_available),					// Input, Only send out data if this flag is high
		.particle_data_out(particle_data_out),											// Output, particle information
		.particle_out_valid(particle_out_valid),										// Output, particle valid
		.particle_data_dest_bin_x(particle_data_dest_bin_x),						// Output, particle destination bin
		.particle_data_dest_bin_y(particle_data_dest_bin_y),						// Output
		.particle_data_dest_bin_z(particle_data_dest_bin_z),						// Output
		// Output to controllers
		.local_motion_update_done(local_motion_update_done)						// Output, When the motion update of all the local particles are done, set this flag, when all the bins finished, then the motion update process is finished
	);

endmodule