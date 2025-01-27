`timescale 1ns / 1ps

module Register (
    input wire [15:0] I,
    input wire [2:0] FunSel,
    input wire E,
    input wire Clock,
    output reg [15:0] Q
);

always @(posedge Clock) begin
    if (E) begin
        case(FunSel)
            3'b000: Q <= Q - 1;
            3'b001: Q <= Q + 1;
            3'b010: Q <= I;
            3'b011: Q <= 16'd0;
            3'b100: begin
                        Q[15:8] <= 8'd0;
                        Q[7:0] <= I[7:0];
                    end
            3'b101: Q[7:0] <= I[7:0];
            3'b110: Q[15:8] <= I[7:0]; 
            3'b111: begin 
                        if (I[7] == 1) begin
                            Q[15:8] <= 8'd255;
                        end else begin
                            Q[15:8] <= 8'd0;
                        end
                        Q[7:0] <= I[7:0];
                    end
            default: Q <= Q; 
        endcase
    end
end

endmodule