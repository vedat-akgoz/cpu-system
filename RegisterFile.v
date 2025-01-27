`timescale 1ns / 1ps

module RegisterFile(
	input wire [15:0] I,
	input wire [2:0] OutASel,
	input wire [2:0] OutBSel,
	input wire [2:0] FunSel,
	input wire [3:0] RegSel,
	input wire [3:0] ScrSel,
	
	
	input wire Clock,
	
	output reg [15:0] OutA,
	output reg [15:0] OutB
	
);

wire [15:0] temp1;
wire [15:0] temp2;
wire [15:0] temp3;
wire [15:0] temp4;
wire [15:0] temp5;
wire [15:0] temp6;
wire [15:0] temp7;
wire [15:0] temp8;

Register R1 (.I(I), .FunSel(FunSel), .E(~RegSel[3]), .Clock(Clock), .Q(temp1));
Register R2 (.I(I), .FunSel(FunSel), .E(~RegSel[2]), .Clock(Clock), .Q(temp2));
Register R3 (.I(I), .FunSel(FunSel), .E(~RegSel[1]), .Clock(Clock), .Q(temp3));
Register R4 (.I(I), .FunSel(FunSel), .E(~RegSel[0]), .Clock(Clock), .Q(temp4));

Register S1 (.I(I), .FunSel(FunSel), .E(~ScrSel[3]), .Clock(Clock), .Q(temp5));
Register S2 (.I(I), .FunSel(FunSel), .E(~ScrSel[2]), .Clock(Clock), .Q(temp6));
Register S3 (.I(I), .FunSel(FunSel), .E(~ScrSel[1]), .Clock(Clock), .Q(temp7));
Register S4 (.I(I), .FunSel(FunSel), .E(~ScrSel[0]), .Clock(Clock), .Q(temp8));






always @(*) begin
	
	case(OutASel)
		3'b000: OutA<=temp1;
		3'b001: OutA<=temp2;
		3'b010: OutA<=temp3;
		3'b011: OutA<=temp4;
		3'b100: OutA<=temp5;
		3'b101: OutA<=temp6;
		3'b110: OutA<=temp7;
		3'b111: OutA<=temp8;
	endcase

	case(OutBSel)
		3'b000: OutB<=temp1;
		3'b001: OutB<=temp2;
		3'b010: OutB<=temp3;
		3'b011: OutB<=temp4;
		3'b100: OutB<=temp5;
		3'b101: OutB<=temp6;
		3'b110: OutB<=temp7;
		3'b111: OutB<=temp8;
	endcase
	
end



endmodule