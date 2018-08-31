module RAM_2_Port(
	address_a,
	address_b,
	clock,
	data_a,
	data_b,
	wren_a,
	wren_b,
	q_a,
	q_b
);

	parameter WIDTH = 16;
	parameter DEPTH = 256;
	parameter ADDR_WIDTH = 8;

	input  [ADDR_WIDTH-1:0]  address_a;
	input  [ADDR_WIDTH-1:0]  address_b;
	input  clock;
	input  [WIDTH-1:0]  data_a;
	input  [WIDTH-1:0]  data_b;
	input  wren_a;
	input  wren_b;
	output [WIDTH-1:0]  q_a;
	output [WIDTH-1:0]  q_b;

	//wire [WIDTH-1:0] sub_wire0;
	//wire [WIDTH-1:0] sub_wire1;
	//wire [WIDTH-1:0] q_a = sub_wire0[WIDTH-1:0];
	//wire [WIDTH-1:0] q_b = sub_wire1[WIDTH-1:0];

	altera_syncram  altera_syncram_component (
				 .address_a (address_a),
				 .address_b (address_b),
				 .clock0 (clock),
				 .data_a (data_a),
				 .data_b (data_b),
				 .wren_a (wren_a),
				 .wren_b (wren_b),
				 .q_a (q_a),
				 .q_b (q_b),
				 .aclr0 (1'b0),
				 .aclr1 (1'b0),
				 .address2_a (1'b1),
				 .address2_b (1'b1),
				 .addressstall_a (1'b0),
				 .addressstall_b (1'b0),
				 .byteena_a (1'b1),
				 .byteena_b (1'b1),
				 .clock1 (1'b1),
				 .clocken0 (1'b1),
				 .clocken1 (1'b1),
				 .clocken2 (1'b1),
				 .clocken3 (1'b1),
				 .eccencbypass (1'b0),
				 .eccencparity (8'b0),
				 .eccstatus (),
				 .rden_a (1'b1),
				 .rden_b (1'b1),
				 .sclr (1'b0));
	defparam
	  altera_syncram_component.address_reg_b  = "CLOCK0",
	  altera_syncram_component.clock_enable_input_a  = "BYPASS",
	  altera_syncram_component.clock_enable_input_b  = "BYPASS",
	  altera_syncram_component.clock_enable_output_a  = "BYPASS",
	  altera_syncram_component.clock_enable_output_b  = "BYPASS",
	  altera_syncram_component.indata_reg_b  = "CLOCK0",
	  altera_syncram_component.intended_device_family  = "Stratix 10",
	  altera_syncram_component.lpm_type  = "altera_syncram",
	  altera_syncram_component.numwords_a  = DEPTH,
	  altera_syncram_component.numwords_b  = DEPTH,
	  altera_syncram_component.operation_mode  = "BIDIR_DUAL_PORT",
	  altera_syncram_component.outdata_aclr_a  = "NONE",
	  altera_syncram_component.outdata_sclr_a  = "NONE",
	  altera_syncram_component.outdata_aclr_b  = "NONE",
	  altera_syncram_component.outdata_sclr_b  = "NONE",
	  altera_syncram_component.outdata_reg_a  = "CLOCK0",
	  altera_syncram_component.outdata_reg_b  = "CLOCK0",
	  altera_syncram_component.power_up_uninitialized  = "FALSE",
	  altera_syncram_component.read_during_write_mode_mixed_ports  = "DONT_CARE",
	  altera_syncram_component.read_during_write_mode_port_a  = "NEW_DATA_NO_NBE_READ",
	  altera_syncram_component.read_during_write_mode_port_b  = "NEW_DATA_NO_NBE_READ",
	  altera_syncram_component.widthad_a  = ADDR_WIDTH,
	  altera_syncram_component.widthad_b  = ADDR_WIDTH,
	  altera_syncram_component.width_a  = WIDTH,
	  altera_syncram_component.width_b  = WIDTH,
	  altera_syncram_component.width_byteena_a  = 1,
	  altera_syncram_component.width_byteena_b  = 1;
		  
		  
		  
endmodule