`timescale 1ns / 1ps

module InstructionRegister (
    input wire Write,
    input wire LH,
    input wire [7:0] I,
	input wire Clock,
    output reg [15:0] IROut
);

always @(posedge Clock) begin
    if (Write) begin
        if (~LH) begin
            IROut[7:0] <= I;
        end else begin
            IROut[15:8] <= I;
        end
    end else begin
		IROut <= IROut;
	end
end

endmodule