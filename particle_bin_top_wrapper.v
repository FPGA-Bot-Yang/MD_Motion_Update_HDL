module particle_bin_top_wrapper(
	clk,
	rst_n,
	rd_addr,													// Input
	evaluation_particle_read_out,						// Output, 1-bit
	evaluation_particle_valid,							// Output, 1-bit
	motion_update_enable,								// Input
	local_motion_update_done,							// Output
	particle_input_available_to_neighbors, 		// Output
	particle_output_availalbe_from_neighbors,		// Input
	global_incom_particle_data_valid,				// Input
	global_incom_particle_data_in,					// Input
	global_incom_particle_data_valid,				// Input
	global_incom_particle_data_in,					// Input
	global_outgoing_particle_data_valid,			// Output
	global_particle_data_out							// Output 1-bit
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

	
	input clk;
	input rst_n;
	input [ADDR_WIDTH-1:0] rd_addr;
	output reg evaluation_particle_read_out;
	output evaluation_particle_valid;
	input motion_update_enable;
	output local_motion_update_done;
	output particle_input_available_to_neighbors;
	input [NUM_NEIGHBOR_BIN-1:0] particle_output_availalbe_from_neighbors;
	input [NUM_NEIGHBOR_BIN-1:0] global_incom_particle_data_valid;
	input [NUM_NEIGHBOR_BIN*DATA_WIDTH-1:0] global_incom_particle_data_in;
	output [NUM_NEIGHBOR_BIN-1:0] global_outgoing_particle_data_valid;
	output reg global_particle_data_out;
	
	wire [DATA_WIDTH-1:0] evaluation_particle_read_out_wire;
	wire [NUM_NEIGHBOR_BIN*DATA_WIDTH-1:0] global_particle_data_out_wire;
	

	particle_bin_top
	#(
		.BIN_ID_X(BIN_ID_X),
		.BIN_ID_Y(BIN_ID_Y),
		.BIN_ID_Z(BIN_ID_Z),
		.BIN_ADDR_WIDTH(BIN_ADDR_WIDTH),
		.DATA_WIDTH(DATA_WIDTH),
		.ADDR_WIDTH(ADDR_WIDTH),
		.BIN_DEPTH(BIN_DEPTH),
		.BIN_OFFSET_WIDTH(BIN_OFFSET_WIDTH),
		.NUM_NEIGHBOR_BIN(NUM_NEIGHBOR_BIN),
		.NEIGHBOR_BIN_ADDR_WIDTH(NEIGHBOR_BIN_ADDR_WIDTH)
	)
	particle_bin_top(
		.clk(clk),
		.rst_n(rst_n),
		// Force evaluation related signal
		.rd_addr(rd_addr),														// Input, Particle read address during force evaluation
		.evaluation_particle_read_out(evaluation_particle_read_out_wire),							// Output, Particle data information for force evaluation
		.evaluation_particle_valid(evaluation_particle_valid),								// Output, Valid flag for read out particle data
		// Motion update related signals
		.motion_update_enable(motion_update_enable),									// Input, router only work during motion update process
		.local_motion_update_done(local_motion_update_done),								// Output, When the motion update of all the local particles are done, set this flag, when all the bins finished, then the motion update process is finished
		// Ports connected to all the neighboring bins
		.particle_input_available_to_neighbors(particle_input_available_to_neighbors),				// Output, to all the neighboring bins, let neighbor bins know if the current bin is ready to take particles
		.particle_output_availalbe_from_neighbors(particle_output_availalbe_from_neighbors),			// Input, from all neighboring bins, only send out data to that bin if the flag is high
		.global_incom_particle_data_valid(global_incom_particle_data_valid),					// Input, incoming particle valid signal from all neighbor bins
		.global_incom_particle_data_in(global_incom_particle_data_in),						// Input, incoming particle data from all neighbor bin
		.global_outgoing_particle_data_valid(global_outgoing_particle_data_valid),				// Output, particle valid flag to neighbor bins
		.global_particle_data_out(global_particle_data_out_wire)							// Output, particle data send to neighbor bins
);


endmodule