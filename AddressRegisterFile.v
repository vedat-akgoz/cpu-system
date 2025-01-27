	`timescale 1ns / 1ps

module AddressRegisterFile(
	input wire [15:0] I,
	input wire [1:0] OutCSel,
	input wire [1:0] OutDSel,
	input wire [2:0] FunSel,
	input wire [2:0] RegSel,
	input wire Clock,
	
	output reg [15:0]OutC,   
	output reg [15:0]OutD
);

wire [15:0] tempa;
wire [15:0] tempb;
wire [15:0] tempc;

Register PC (.I(I), .FunSel(FunSel), .E(~RegSel[2]), .Clock(Clock), .Q(tempa));
Register AR (.I(I), .FunSel(FunSel), .E(~RegSel[1]), .Clock(Clock), .Q(tempb));
Register SP (.I(I), .FunSel(FunSel), .E(~RegSel[0]), .Clock(Clock), .Q(tempc));

always @(*) begin

	case (OutCSel)
		2'b00: OutC<=tempa;
		2'b01: OutC<=tempa;
		2'b10: OutC<=tempb;
		2'b11: OutC<=tempc;
	endcase
	case (OutDSel)
		2'b00: OutD<=tempa;
		2'b01: OutD<=tempa;
		2'b10: OutD<=tempb;
		2'b11: OutD<=tempc;
	endcase
end
endmodule