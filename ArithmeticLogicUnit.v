`timescale 1ns / 1ps

module ArithmeticLogicUnit(
	input wire [15:0] A,
	input wire [15:0] B,
	input wire [4:0] FunSel,
	input wire WF,
	input wire Clock,
	
	
	
	output reg [15:0] ALUOut,
	output reg [3:0] FlagsOut
);
reg temp;
always @(*) begin
	case(FunSel)
	5'b00000:   begin
				ALUOut[7:0] <= A[7:0];
				ALUOut[15:8] <= 0;
				end
				
	5'b00001:   begin
				ALUOut[7:0] <= B[7:0];
				ALUOut[15:8] <= 0;
				end
				
	5'b00010:   begin
				ALUOut[7:0] <= ~A[7:0];
				ALUOut[15:8] <= 0;
				end
				
	5'b00011:   begin
				ALUOut[7:0] <= ~B[7:0];
				ALUOut[15:8] <= 0;
				end
				
	5'b00100:   begin
				{temp, ALUOut[7:0]} <= A[7:0] + B[7:0];
				ALUOut[15:8] <= 0;
				end
				
	5'b00101:   begin
				{temp, ALUOut[7:0]} <= A[7:0] + B[7:0] + FlagsOut[2];
				ALUOut[15:8] <= 0;
				end
				
	5'b00110:   begin
				{temp, ALUOut[7:0]} <= A[7:0] - B[7:0];
				ALUOut[15:8] <= 0;
				end
				
	5'b00111:   begin
				ALUOut[7:0] <=A[7:0] & B[7:0];
				ALUOut[15:8] <= 0;
				end
				
	5'b01000:   begin
				ALUOut[7:0] <=A[7:0] | B[7:0];
				ALUOut[15:8] <= 0;
				end
				
	5'b01001:   begin
				ALUOut <=A[7:0] ^ B[7:0];
				ALUOut[15:8] <= 0;
				end
		
	5'b01010:   begin
				ALUOut[7:0] <= ~(A[7:0] & B[7:0]);
				ALUOut[15:8] <= 0;
				end
	
	5'b01011:   begin
				{temp, ALUOut[7:0]} <= {A[7:0], 1'b0};
				ALUOut[15:8] <= 0;
				end
				
	5'b01100:   begin
				{ALUOut[7:0], temp}<={1'b0, A[7:0]};
				ALUOut[15:8] <= 0;
				end
				
	5'b01101:   begin
				{ALUOut[7:0], temp} <= {A[7],A[7:0]};
				ALUOut[15:8] <= 0;
				end
				
	5'b01110:   begin
				ALUOut[7:0] = {A[6:0], FlagsOut[2]};
				temp = A[7];
				ALUOut[15:8] <= 0;
				end
				
	5'b01111:   begin
				ALUOut[7:0] = {FlagsOut[2], A[7:1]};
				temp = A[0];
				ALUOut[15:8] <= 0;
				end
				
	5'b10000:   begin
				ALUOut <= A;
				end
				
	5'b10001:   begin
				ALUOut <= B;
				end
				
	5'b10010:   begin
				ALUOut <= ~A;
				end
				
	5'b10011:   begin
				ALUOut <= ~B;
				end
				
	5'b10100:   begin
				{temp, ALUOut} <= A + B;
				end
				
	5'b10101:   begin
				{temp, ALUOut} <= A + B + FlagsOut[2];
				end
				
	5'b10110:   begin
				{temp, ALUOut} <= A - B;
				end
				
	5'b10111:   begin
				ALUOut <=A & B;
				end
				
	5'b11000:   begin
				ALUOut <=A | B;
				end
				
	5'b11001:   begin
				ALUOut <=A ^ B;
				end
		
	5'b11010:   begin
				ALUOut <= ~(A & B);
				end
	
	5'b11011:   begin
				{temp, ALUOut} <= {A, 1'b0};
				end
				
	5'b11100:   begin
				{ALUOut, temp}<={1'b0, A};
				end
				
	5'b11101:   begin
				{ALUOut, temp} <= {A[15],A[15:0]};
				end
				
	5'b11110:   begin
				ALUOut = {A[14:0], FlagsOut[2]};
				temp = A[15];
				end
				
	5'b11111:   begin
				ALUOut = {FlagsOut[2], A[15:1]};
				temp = A[0];
				end
				


endcase
end


always @(posedge Clock) begin
    case(FunSel)
        5'b00000: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[7];

            end
        end
        5'b00001: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
					
                FlagsOut[1]<= ALUOut[7];

            end
        end
        5'b00010: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[7];

            end
        end
        5'b00011: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[7];

            end
        end
		5'b00100: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
				if (ALUOut[7])
					FlagsOut[1] <= 1;
				else
					FlagsOut[1] <= 0;
				
				if ((A[7] == B[7]) && (ALUOut[7] != A[7]))
					FlagsOut[0] <= 1;
				else
					FlagsOut[0] <= 0;
				
				FlagsOut[2] <= temp;
			end
		end
		5'b00101: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
				if (ALUOut[7])
					FlagsOut[1] <= 1;
				else
					FlagsOut[1] <= 0;
				
				if ((A[7] == B[7]) && (ALUOut[7] != A[7]))
					FlagsOut[0] <= 1;
				else
					FlagsOut[0] <= 0;
				
				FlagsOut[2] <= temp;
			end
		end
		5'b00110: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
				if (ALUOut[7])
					FlagsOut[1] <= 1;
				else
					FlagsOut[1] <= 0;
				
				if ((A[7] == B[7]) && (ALUOut[7] != A[7]))
					FlagsOut[0] <= 1;
				else
					FlagsOut[0] <= 0;
				
				FlagsOut[2] <= temp;
			end
		end
		5'b00111: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[7];

            end
        end
		5'b01000: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[7];

            end
        end
		5'b01001: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[7];

            end
        end
		5'b01010: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[7];

            end
        end
		5'b01011: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[7];
				
				FlagsOut[2] <= temp;
			end
		end
		5'b01100: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[7];
				
				FlagsOut[2] <= temp;
			end
		end
		5'b01101: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
				FlagsOut[2] <= temp;
			end
		end
		5'b01110: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[7];
				
				FlagsOut[2] <= temp;
			end
		end
		5'b01111: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[7];
				
				FlagsOut[2] <= temp;
			end
		end
				
        5'b10000: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
                
                if (ALUOut[15])
                    FlagsOut[1] <= 1;
                else
                    FlagsOut[1] <= 0;
            end
        end
        5'b10001: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[15];

            end
		end
			5'b10010: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[15];

            end
		end
		5'b10011: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[15];

            end
		end
		5'b10100: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[15];
				
				if ((A[15] == B[15]) && (ALUOut[15] != A[15]))
					FlagsOut[0] <= 1;
				else
					FlagsOut[0] <= 0;
				
				FlagsOut[2] <= temp;
			end
		end
		5'b10101: begin
			if (WF) begin
				if (ALUOut == 16'b0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[15];
				
				if ((A[15] == B[15]) && (ALUOut[15] != A[15]))
					FlagsOut[0] <= 1;
				else
					FlagsOut[0] <= 0;
				
				FlagsOut[2] <= temp;
			end
		end
		5'b10110: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[15];
				
				if ((A[15] == B[15]) && (ALUOut[15] != A[15]))
					FlagsOut[0] <= 1;
				else
					FlagsOut[0] <= 0;
				
				FlagsOut[2] <= temp;
			end
		end
		5'b10111: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[15];

            end
		end
		5'b11000: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[15];

            end
		end
		5'b11001: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[15];

            end
		end
		5'b11010: begin
            if (WF) begin
                if (ALUOut == 0)
                    FlagsOut[3] <= 1;
                else
                    FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[15];

            end
		end
		5'b11011: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[15];
				
				FlagsOut[2] <= temp;
			end
		end
		5'b11100: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[15];
				
				FlagsOut[2] <= temp;
			end
		end
		5'b11101: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
				FlagsOut[2] <= temp;
			end
		end
		5'b11110: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[15];
				
				FlagsOut[2] <= temp;
			end
		end
		5'b11111: begin
			if (WF) begin
				if (ALUOut == 0)
					FlagsOut[3] <= 1;
				else
					FlagsOut[3] <= 0;
				
                FlagsOut[1]<= ALUOut[15];
				
				FlagsOut[2] <= temp;
			end
		end
endcase
end


endmodule