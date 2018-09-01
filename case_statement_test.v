module case_statement_test(
	input clk,
	input rst_n,
	input [3:0] data_in_0,
	input [3:0] data_in_1,
	input [3:0] data_in_2,
	input [3:0] data_in_3,
	input [3:0] data_in_4,
	input [3:0] data_in_5,
	input [3:0] data_in_6,
	input [3:0] data_in_7,
	input [3:0] data_in_8,
	output reg [3:0] data_out
);
	
	wire [2:0] sel;
	assign sel = data_in_8[2:0];
	
	always@(posedge clk)
		begin
		if(~rst_n)
			begin
			data_out <= 0;
			end
		else
			begin
			case(sel)
				0: data_out <= data_in_0;
				1: data_out <= data_in_1;
				2: data_out <= data_in_2;
				3: data_out <= data_in_3;
				4: data_out <= data_in_4;
				5: data_out <= data_in_5;
				6: data_out <= data_in_6;
				7: data_out <= data_in_7;
				8: data_out <= data_in_8;
				default: data_out <= 0;
			endcase
			end
		end

endmodule