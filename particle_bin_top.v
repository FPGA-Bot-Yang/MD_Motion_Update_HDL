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
	particle_input_available_to_neighbors,				// Output, to all the neighboring bins, this one is 
	

);

	parameter BIN_ID_X = 0;
	parameter BIN_ID_Y = 0;
	parameter BIN_ID_Z = 0;
	parameter BIN_ADDR_WIDTH = 4;
	parameter DATA_WIDTH = 32*5;
	parameter ADDR_WIDTH = 7;
	parameter BIN_DEPTH = 128;
	parameter BIN_OFFSET_WIDTH = 3;	

	// Input & Output ports
	input clk;
	input rst_n;
	input [ADDR_WIDTH-1:0] rd_addr;
	output reg [DATA_WIDTH-1:0] evaluation_particle_read_out;
	output reg evaluation_particle_valid;
	input motion_update_enable;												// This signal should keep high the entire motion update process
																						// When detect the falling edge, signifies the end of the motion update process
	output local_motion_update_done;											// Flag set as high when all the local particles are done processed for motion update
	output reg particle_input_available_to_neighbors;
	
	
	// Ports connecting to particle_bins
	// Incoming particles information
	reg incom_particle_data_valid;
	reg [BIN_ADDR_WIDTH-1:0] incom_particle_data_target_bin_x;
	reg [BIN_ADDR_WIDTH-1:0] incom_particle_data_target_bin_y;
	reg [BIN_ADDR_WIDTH-1:0] incom_particle_data_target_bin_z;
	reg [DATA_WIDTH-1:0] incom_particle_data_in;
	wire particle_input_available;
	// Outgoing particles information
	reg particle_output_available;
	wire [DATA_WIDTH-1:0] particle_data_out;
	wire particle_out_valid;
	wire [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_x;
	wire [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_y;
	wire [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_z;

	
	// Set the particle_input_available_to_neighbors flag based on the particle_bin module and all the buffered partciles from neighbor bins
	always@(posedge clk)
		if(~rst_n)
			begin
			particle_input_available_to_neighbors <= 1'b1;
			end
		// only need to handle this flag during motion update process
		else if(motion_update_enable)
			begin
			
			end
		// When not in motion update process, set this one as high
		else
			begin
			particle_input_available_to_neighbors <= 1'b1;
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