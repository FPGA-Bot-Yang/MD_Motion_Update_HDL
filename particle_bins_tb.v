`timescale 1ps/1ps

module particle_bins_tb;

	parameter BIN_ID_X = 0;
	parameter BIN_ID_Y = 0;
	parameter BIN_ID_Z = 0;
	parameter BIN_ADDR_WIDTH = 4;
	parameter DATA_WIDTH = 32*5;
	parameter ADDR_WIDTH = 7;
	parameter BIN_DEPTH = 128;
	parameter BIN_OFFSET_WIDTH = 3;						// log(CUTOFF) / log 2

	reg clk;
	reg rst_n;
	reg [ADDR_WIDTH-1:0] rd_addr;
	wire [DATA_WIDTH-1:0] evaluation_particle_read_out;
	wire evaluation_particle_valid;
	reg motion_update_enable;
	reg incom_particle_data_valid;
	reg [BIN_ADDR_WIDTH-1:0] incom_particle_data_target_bin_x;
	reg [BIN_ADDR_WIDTH-1:0] incom_particle_data_target_bin_y;
	reg [BIN_ADDR_WIDTH-1:0] incom_particle_data_target_bin_z;
	reg [DATA_WIDTH-1:0] incom_particle_data_in;
	wire particle_input_available;
	reg particle_output_available;
	wire [DATA_WIDTH-1:0] particle_data_out;
	wire particle_out_valid;
	wire [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_x;
	wire [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_y;
	wire [BIN_ADDR_WIDTH-1:0] particle_data_dest_bin_z;
	wire local_motion_update_done;
	
	reg [3:0] tmp_counter;
	reg [3:0] iteration_counter;
	reg [31:0] tmp_X, tmp_Y, tmp_Z, tmp_Force, tmp_Energy;
	reg prev_motion_update_enable;
	
	
	always #1 clk <= ~clk;
	
	// generating the iteration counter
	always@(posedge clk)
		begin
		if(~rst_n)
			iteration_counter <= 0;
		else if(prev_motion_update_enable && ~motion_update_enable)
			iteration_counter <= iteration_counter + 1'b1;
		else
			iteration_counter <= iteration_counter;
		end
	
	// generating dummy read address
	always@(posedge clk)
		begin
		if(~rst_n)
			rd_addr <= 1'b0;
		else if(rd_addr <= 10)
			rd_addr <= rd_addr + 1'b1;
		else
			rd_addr <= 0;
		end
		
	// generating the incoming particles
	always@(posedge clk)
		begin
		prev_motion_update_enable <= motion_update_enable;
		if(~rst_n)
			begin
			tmp_counter <= 0;
			tmp_X <= 0;
			tmp_Y <= 0;
			tmp_Z <= 0;
			tmp_Force <= 0;
			tmp_Energy <= 0;
			incom_particle_data_valid <= 1'b0;
			incom_particle_data_target_bin_x <= 0;
			incom_particle_data_target_bin_y <= 0;
			incom_particle_data_target_bin_z <= 0;
			incom_particle_data_in <= 0;
			end
		// the 1st itermation of motion update
		else if(iteration_counter == 0 && motion_update_enable && tmp_counter < 6)
			begin
			tmp_counter <= tmp_counter + 1'b1;
			tmp_X <= {25'd0, tmp_counter,3'd1};
			tmp_Y <= {25'd0, tmp_counter,3'd2};
			tmp_Z <= {25'd0, tmp_counter,3'd3};
			tmp_Force <= 0;
			tmp_Energy <= 0;
			particle_output_available <= 1'b1;
			incom_particle_data_valid <= 1'b1;
			incom_particle_data_target_bin_x <= 0;
			incom_particle_data_target_bin_y <= 0;
			incom_particle_data_target_bin_z <= 0;
			incom_particle_data_in <= {tmp_Energy, tmp_Force, tmp_Z, tmp_Y, tmp_X};
			end
		// the 2nd itermation of motion update
		else if(iteration_counter == 1 && motion_update_enable && tmp_counter < 6)
			begin
			tmp_counter <= tmp_counter + 1'b1;
			tmp_X <= {25'd0, tmp_counter,3'd1};
			tmp_Y <= {25'd0, tmp_counter,3'd2};
			tmp_Z <= {25'd0, tmp_counter,3'd3};
			tmp_Force <= 0;
			tmp_Energy <= 0;
			particle_output_available <= 1'b1;
			incom_particle_data_valid <= 1'b1;
			incom_particle_data_target_bin_x <= tmp_counter;
			incom_particle_data_target_bin_y <= tmp_counter;
			incom_particle_data_target_bin_z <= tmp_counter;
			incom_particle_data_in <= {tmp_Energy, tmp_Force, tmp_Z, tmp_Y, tmp_X};
			end
		// reset the tmp_counter
		else if(motion_update_enable && ~prev_motion_update_enable)
			begin
			tmp_counter <= 0;
			tmp_X <= 0;
			tmp_Y <= 0;
			tmp_Z <= 0;
			tmp_Force <= 0;
			tmp_Energy <= 0;
			incom_particle_data_valid <= 1'b0;
			incom_particle_data_target_bin_x <= 2;
			incom_particle_data_target_bin_y <= 3;
			incom_particle_data_target_bin_z <= 4;
			incom_particle_data_in <= 100;
			end
		else
			begin
			tmp_counter <= tmp_counter;
			tmp_X <= 0;
			tmp_Y <= 0;
			tmp_Z <= 0;
			tmp_Force <= 0;
			tmp_Energy <= 0;
			incom_particle_data_valid <= 1'b0;
			incom_particle_data_target_bin_x <= 2;
			incom_particle_data_target_bin_y <= 3;
			incom_particle_data_target_bin_z <= 4;
			incom_particle_data_in <= 100;
			end
		end
		
		
	initial begin
		clk <= 1'b1;
		rst_n <= 1'b0;
		rd_addr <= 0;
		motion_update_enable <= 1'b0;
		incom_particle_data_valid <= 1'b0;
		incom_particle_data_in <= 0;
		incom_particle_data_target_bin_x <= 0;
		incom_particle_data_target_bin_y <= 0;
		incom_particle_data_target_bin_z <= 0;
		
		#4
		rst_n <= 1'b1;
		
		#20
		motion_update_enable <= 1'b1;
		
		#80
		motion_update_enable <= 1'b0;
		#80
		motion_update_enable <= 1'b1;
		#80
		motion_update_enable <= 1'b0;
		
		
	end
	
	particle_bins
	#(
		.BIN_ID_X(0),
		.BIN_ID_Y(0),
		.BIN_ID_Z(0),
		.BIN_ADDR_WIDTH(BIN_ADDR_WIDTH),
		.DATA_WIDTH(DATA_WIDTH),
		.ADDR_WIDTH(ADDR_WIDTH),
		.BIN_DEPTH(BIN_DEPTH),
		.BIN_OFFSET_WIDTH(BIN_OFFSET_WIDTH)
	)
	UUT
	(
		.clk(clk),
		.rst_n(rst_n),														// reset on low
		// Force evaluation related signal
		.rd_addr(rd_addr),														// Particle read address during force evaluation
		.evaluation_particle_read_out(evaluation_particle_read_out),
		.evaluation_particle_valid(evaluation_particle_valid),
		// Motion update enable signal
		.motion_update_enable(motion_update_enable),									// This signal should keep high the entire motion update process
																		// When detect the falling edge, signifies the end of the motion update process
		// Incoming particles information
		.incom_particle_data_valid(incom_particle_data_valid),
		.incom_particle_data_target_bin_x(incom_particle_data_target_bin_x),							// Incoming particle destination bin
		.incom_particle_data_target_bin_y(incom_particle_data_target_bin_y),
		.incom_particle_data_target_bin_z(incom_particle_data_target_bin_z),
		.incom_particle_data_in(incom_particle_data_in),
		.particle_input_available(particle_input_available),
		// Out going particles information
		.particle_output_available(particle_output_available),
		.particle_data_out(particle_data_out),										// Output particle information
		.particle_out_valid(particle_out_valid),										// Output particle valid
		.particle_data_dest_bin_x(particle_data_dest_bin_x),								// Output particle destination bin
		.particle_data_dest_bin_y(particle_data_dest_bin_y),
		.particle_data_dest_bin_z(particle_data_dest_bin_z),
		// Output to controllers
		.local_motion_update_done(local_motion_update_done)
	);


endmodule