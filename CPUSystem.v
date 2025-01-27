`timescale 1ns / 1ps

module SequenceDecoder(
    input wire [2:0] Input,
    output reg [7:0] Output
);
	integer i;
	always @(*) begin				
		for (i = 0; i < 8; i = i + 1) begin if (Input == i)	Output = 8'b00000001 << i;
		end
	end 
endmodule

module OpcodeDecoder(
    input wire [5:0] Input,
    output reg [33:0] Output
);
	integer i;
	always @(*) begin
		for (i = 0; i < 34; i = i + 1) begin if (Input == i)	Output = 34'b00000001 << i;
		end
	end
endmodule

module SequenceCounter(
	input wire Clock,
	input wire Reset,
	input wire Count,
    output reg[2:0] Output
);

    always @(posedge Clock) begin
		if(Reset) Output <= 3'b0;
		else if (Count) Output <= Output + 1'b1;
		else Output <= Output;
    end
endmodule
	 
module CPUSystem(
    input wire Clock,
	input wire Reset,
	input wire[7:0] T 
);
    reg[2:0] RF_OutASel, RF_OutBSel, RF_FunSel;
    reg[3:0] RF_RegSel, RF_ScrSel;
    reg[4:0] ALU_FunSel;
    reg ALU_WF; 
    reg[1:0] ARF_OutCSel, ARF_OutDSel;
    reg[2:0] ARF_FunSel, ARF_RegSel;
    reg IR_LH, IR_Write, Mem_WR, Mem_CS;
    reg[1:0] MuxASel, MuxBSel;
    reg MuxCSel;
    
    ArithmeticLogicUnitSystem _ALUSystem(
        .RF_OutASel(RF_OutASel),   .RF_OutBSel(RF_OutBSel), 
        .RF_FunSel(RF_FunSel),     .RF_RegSel(RF_RegSel),
        .RF_ScrSel(RF_ScrSel),     .ALU_FunSel(ALU_FunSel),
        .ALU_WF(ALU_WF),           .ARF_OutCSel(ARF_OutCSel), 
        .ARF_OutDSel(ARF_OutDSel), .ARF_FunSel(ARF_FunSel),
        .ARF_RegSel(ARF_RegSel),   .IR_LH(IR_LH),
        .IR_Write(IR_Write),       .Mem_WR(Mem_WR),
        .Mem_CS(Mem_CS),           .MuxASel(MuxASel),
        .MuxBSel(MuxBSel),         .MuxCSel(MuxCSel),
        .Clock(Clock)
    );		
	wire[2:0] Sequence_Counter_Output;
	reg Sequence_Counter_Reset;
	wire[33:0] D;
	wire S, Z;
	wire SREG1_File_Select;
	wire SREG2_File_Select;
	wire DREG_File_Select;
	
	wire RxSel_R1, RxSel_R2, RxSel_R3, RxSel_R4;
	wire DREG_PC, DREG_SP, DREG_AR, DREG_R1, DREG_R2, DREG_R3, DREG_R4;
	wire SREG1_PC, SREG1_SP, SREG1_AR, SREG1_R1, SREG1_R2, SREG1_R3, SREG1_R4;
	wire SREG2_PC, SREG2_SP, SREG2_AR, SREG2_R1, SREG2_R2, SREG2_R3, SREG2_R4;
	
	assign RxSel_R1 = ~_ALUSystem.IR.IROut[9] & ~_ALUSystem.IR.IROut[8];
	assign RxSel_R2 = ~_ALUSystem.IR.IROut[9] & _ALUSystem.IR.IROut[8];
	assign RxSel_R3 = _ALUSystem.IR.IROut[9] & ~_ALUSystem.IR.IROut[8];
	assign RxSel_R4 = _ALUSystem.IR.IROut[9] & _ALUSystem.IR.IROut[8];
	
	assign DREG_PC = (~_ALUSystem.IR.IROut[8] & ~_ALUSystem.IR.IROut[7] & ~_ALUSystem.IR.IROut[6]) | (~_ALUSystem.IR.IROut[8] & ~_ALUSystem.IR.IROut[7] & _ALUSystem.IR.IROut[6]);
	assign DREG_SP = ~_ALUSystem.IR.IROut[8] & _ALUSystem.IR.IROut[7] & ~_ALUSystem.IR.IROut[6];
	assign DREG_AR = ~_ALUSystem.IR.IROut[8] & _ALUSystem.IR.IROut[7] & _ALUSystem.IR.IROut[6];
	assign DREG_R1 = _ALUSystem.IR.IROut[8] & ~_ALUSystem.IR.IROut[7] & ~_ALUSystem.IR.IROut[6];
	assign DREG_R2 = _ALUSystem.IR.IROut[8] & ~_ALUSystem.IR.IROut[7] & _ALUSystem.IR.IROut[6];
	assign DREG_R3 = _ALUSystem.IR.IROut[8] & _ALUSystem.IR.IROut[7] & ~_ALUSystem.IR.IROut[6];
	assign DREG_R4 = _ALUSystem.IR.IROut[8] & _ALUSystem.IR.IROut[7] & _ALUSystem.IR.IROut[6];
	
	assign SREG1_PC = (~_ALUSystem.IR.IROut[5] & ~_ALUSystem.IR.IROut[4] & ~_ALUSystem.IR.IROut[3]) | (~_ALUSystem.IR.IROut[5] & ~_ALUSystem.IR.IROut[4] & _ALUSystem.IR.IROut[3]);
	assign SREG1_SP = ~_ALUSystem.IR.IROut[5] & _ALUSystem.IR.IROut[4] & ~_ALUSystem.IR.IROut[3];
	assign SREG1_AR = ~_ALUSystem.IR.IROut[5] & _ALUSystem.IR.IROut[4] & _ALUSystem.IR.IROut[3];
	assign SREG1_R1 = _ALUSystem.IR.IROut[5] & ~_ALUSystem.IR.IROut[4] & ~_ALUSystem.IR.IROut[3];
	assign SREG1_R2 = _ALUSystem.IR.IROut[5] & ~_ALUSystem.IR.IROut[4] & _ALUSystem.IR.IROut[3];
	assign SREG1_R3 = _ALUSystem.IR.IROut[5] & _ALUSystem.IR.IROut[4] & ~_ALUSystem.IR.IROut[3];
	assign SREG1_R4 = _ALUSystem.IR.IROut[5] & _ALUSystem.IR.IROut[4] & _ALUSystem.IR.IROut[3];
	
	assign SREG2_PC = (~_ALUSystem.IR.IROut[2] & ~_ALUSystem.IR.IROut[1] & ~_ALUSystem.IR.IROut[0]) | (~_ALUSystem.IR.IROut[2] & ~_ALUSystem.IR.IROut[1] & _ALUSystem.IR.IROut[0]);
	assign SREG2_SP = ~_ALUSystem.IR.IROut[2] & _ALUSystem.IR.IROut[1] & ~_ALUSystem.IR.IROut[0];
	assign SREG2_AR = ~_ALUSystem.IR.IROut[2] & _ALUSystem.IR.IROut[1] & _ALUSystem.IR.IROut[0];
	assign SREG2_R1 = _ALUSystem.IR.IROut[2] & ~_ALUSystem.IR.IROut[1] & ~_ALUSystem.IR.IROut[0];
	assign SREG2_R2 = _ALUSystem.IR.IROut[2] & ~_ALUSystem.IR.IROut[1] & _ALUSystem.IR.IROut[0];
	assign SREG2_R3 = _ALUSystem.IR.IROut[2] & _ALUSystem.IR.IROut[1] & ~_ALUSystem.IR.IROut[0];
	assign SREG2_R4 = _ALUSystem.IR.IROut[2] & _ALUSystem.IR.IROut[1] & _ALUSystem.IR.IROut[0];
	
	assign Z = _ALUSystem.ALU.FlagsOut[3];																	
	assign S = _ALUSystem.IR.IROut[9];

	assign DREG_File_Select = _ALUSystem.IR.IROut[8]; 
	assign SREG1_File_Select = _ALUSystem.IR.IROut[5];
	assign SREG2_File_Select = _ALUSystem.IR.IROut[2];																

	SequenceCounter SC(.Clock(Clock), .Reset(Sequence_Counter_Reset), .Count(1'b1), .Output(Sequence_Counter_Output));
	SequenceDecoder SD(.Input(Sequence_Counter_Output), .Output(T));
	OpcodeDecoder OD(.Input(_ALUSystem.IR.IROut[15:10]), .Output(D));

	always @(posedge Reset) begin
		Sequence_Counter_Reset <= 1'b1;
		ARF_RegSel <= 3'b011;
		ARF_FunSel <= 3'b011;
		Mem_CS <= 1'b1;
		ALU_WF <= 1'b0;
		IR_Write <= 1'b0;
		RF_FunSel <= 3'b011;
		RF_RegSel <= 4'b0000;
		RF_ScrSel <= 4'b0000;
	end										 		
	always @(T) begin										

		if ((T[2] & ~((D[5] | D[6]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select) & ~((D[5] | D[6]) & T[2] & SREG1_File_Select & ~DREG_File_Select) & ~((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & ~DREG_File_Select)& ~(D[18] & T[2])& ~(D[19] & T[2])& ~(D[31] & T[2])& ~(D[3] & T[2])& ~(D[4] & T[2]))
		)									ARF_RegSel <= 3'b111;												
		else if (T[0] | T[1] | (D[0] & T[4]) | (D[1] & T[4] & ~Z) | (D[2] & T[4] & Z) | ((D[5] | D[6]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select & DREG_PC) | ((D[5] | D[6]) & T[2] & SREG1_File_Select & ~DREG_File_Select & DREG_PC) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & ~DREG_File_Select & DREG_PC)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & ~DREG_File_Select & DREG_PC)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & ~DREG_File_Select & DREG_PC)| (D[31] & T[2])| (D[31] & T[4])| (D[30] & T[5])
		)ARF_RegSel <= 3'b011;
		else if (((D[5] | D[6]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select & DREG_AR)| ((D[5] | D[6]) & T[2] & SREG1_File_Select & ~DREG_File_Select & DREG_AR)  | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & ~DREG_File_Select & DREG_AR)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & ~DREG_File_Select & DREG_AR)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & ~DREG_File_Select & DREG_AR)| (D[18] & T[2])| (D[19] & T[2])| (D[33] & T[4])
		)ARF_RegSel <= 3'b101;
		else if (((D[5] | D[6]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select & DREG_SP)| ((D[5] | D[6]) & T[2] & SREG1_File_Select & ~DREG_File_Select & DREG_SP) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & ~DREG_File_Select & DREG_SP)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & ~DREG_File_Select & DREG_SP)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & ~DREG_File_Select & DREG_SP)| (D[31] & T[3])| (D[30] & T[3])| (D[3] & T[2])| (D[4] & T[2])| (D[31] & T[5])
		)ARF_RegSel <= 3'b110;

		if (T[0] | T[1] | (D[5] & T[3] & ~SREG1_File_Select & ~DREG_File_Select) | (D[5] & T[3] & SREG1_File_Select & ~DREG_File_Select)| (D[18] & T[2])| (D[19] & T[2])| (D[31] & T[3])| (D[30] & T[3])| (D[33] & T[5])| (D[3] & T[2])		| (D[4] & T[2])| (D[31] & T[5])			
		)ARF_FunSel <= 3'b001;
		else if ((D[0] & T[4]) | (D[1] & T[4] & ~Z) | (D[2] & T[4] & Z) | ((D[5] | D[6]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select)| ((D[5] | D[6]) & T[2] & SREG1_File_Select & ~DREG_File_Select) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & ~DREG_File_Select)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & ~DREG_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & ~DREG_File_Select)| (D[30] & T[5])| (D[33] & T[4])			
		)ARF_FunSel <= 3'b010;
		else if ((D[6] & T[3] & ~SREG1_File_Select & ~DREG_File_Select) | (D[6] & T[3] & SREG1_File_Select & ~DREG_File_Select)| (D[4] & T[3])| (D[18] & T[3])| (D[19] & T[3])| (D[33] & T[6])
		)ARF_FunSel <= 3'b000;
		else if (| (D[31] & T[4])
		)ARF_FunSel <= 3'b110;
		else if (| (D[31] & T[2])			
		)ARF_FunSel <= 3'b101;	
		if (T[0] | T[1]
		)ARF_OutDSel <= 2'b00;
		else if (| (D[31] & T[2])| (D[30] & T[3])| (D[3] & T[2])| (D[4] & T[2])
		)ARF_OutDSel <= 2'b11;
		else if ((D[18] & T[2])| (D[19] & T[2])| (D[33] & T[5])
		)ARF_OutDSel <= 2'b10;
		if ((D[0] & T[3]) | (D[1] & T[3] & ~Z) | (D[2] & T[3] & Z) | ((D[5] | D[6]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select & SREG1_PC) | ((D[5] | D[6]) & T[2] & ~SREG1_File_Select & DREG_File_Select & SREG1_PC)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select & SREG1_PC) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & ~SREG1_File_Select & DREG_File_Select & SREG1_PC)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[2] & ~SREG1_File_Select & SREG1_PC)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[3] & ~SREG2_File_Select & SREG2_PC)| (D[30] & T[2])
		)				ARF_OutCSel <= 2'b00;
		else if (((D[5] | D[6]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select & SREG1_SP) | ((D[5] | D[6]) & T[2] & ~SREG1_File_Select & DREG_File_Select & SREG1_SP) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & ~SREG1_File_Select & DREG_File_Select & SREG1_SP)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select & SREG1_SP)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[2] & ~SREG1_File_Select & SREG1_SP)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[3] & ~SREG2_File_Select & SREG2_SP)
		)ARF_OutCSel <= 2'b11;
		
		else if (((D[5] | D[6]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select & SREG1_AR) | ((D[5] | D[6]) & T[2] & ~SREG1_File_Select & DREG_File_Select & SREG1_AR)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select & SREG1_AR) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & ~SREG1_File_Select & DREG_File_Select & SREG1_AR)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[2] & ~SREG1_File_Select & SREG1_AR)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[3] & ~SREG2_File_Select & SREG2_AR)| (D[33] & T[2])
		)ARF_OutCSel <= 2'b10;
		if (T[1]| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & ~SREG1_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & SREG1_File_Select)
		)RF_ScrSel <= 4'b1111;
		else if ((D[0] & T[2]) | (D[1] & T[2] & ~Z) | (D[2] & T[2] & Z) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & ~SREG1_File_Select & DREG_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[2] & ~SREG1_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[2] & SREG1_File_Select)| (D[30] & T[2])| (D[33] & T[2])
		)RF_ScrSel <= 4'b0111;
		else if ((D[0] & T[3]) | (D[1] & T[3] & ~Z) | (D[2] & T[3] & Z)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[3] & ~SREG2_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[3] & SREG2_File_Select)| (D[33] & T[3])
		)RF_ScrSel <= 4'b1011;
		else if ((D[1] & T[4] & ~Z) | (D[2] & T[4] & Z) | T[0]				
		)RF_ScrSel <= 4'b0000;

		if ((D[0] & T[2]) | (D[0] & T[3]) | (D[1] & T[2] & ~Z) | (D[1] & T[3] & ~Z) | (D[2] & T[2] & Z) | (D[2] & T[3] & Z) | ((D[5] | D[6]) & T[2] & ~SREG1_File_Select & DREG_File_Select) | ((D[5] | D[6]) & T[2] & SREG1_File_Select & DREG_File_Select)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & ~SREG1_File_Select & DREG_File_Select) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & DREG_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[2] & ~SREG1_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[3] & ~SREG2_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[2] & SREG1_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[3] & SREG2_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & DREG_File_Select)| (D[32] & T[2])| (D[30] & T[2])	| (D[33] & T[2])| (D[33] & T[3])		
		)RF_FunSel <= 3'b010;	
		else if ((D[1] & T[4] & ~Z) | (D[2] & T[4] & Z) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & ~DREG_File_Select) | T[0]
		)RF_FunSel <= 3'b011;
		else if ((D[5] & T[3] & ~SREG1_File_Select & DREG_File_Select) | (D[5] & T[3] & SREG1_File_Select & DREG_File_Select)
		)RF_FunSel <= 3'b001;
		else if ((D[6] & T[3] & ~SREG1_File_Select & DREG_File_Select) | (D[6] & T[3] & SREG1_File_Select & DREG_File_Select)
		)RF_FunSel <= 3'b000;
		else if ((D[17] & T[2])| (D[18] & T[3])| (D[3] & T[3])
		)RF_FunSel <= 3'b110;
		else if ((D[20] & T[2])| (D[18] & T[2])| (D[3] & T[2])
		)RF_FunSel <= 3'b101;

		if (T[1] | (D[0] & T[4]) | (D[1] & T[4] & ~Z) | (D[2] & T[4] & Z) | ((D[5] | D[6]) & T[3] & SREG1_File_Select & DREG_File_Select) | ((D[5] | D[6]) & T[3] & SREG1_File_Select & ~DREG_File_Select) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & ~DREG_File_Select)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & DREG_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & ~DREG_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & DREG_File_Select)| (D[30] & T[3])| (D[33] & T[4])
		)RF_OutASel <= 3'b100;
		else if (((D[5] | D[6]) & T[2] & SREG1_File_Select & DREG_File_Select & SREG1_R1) | ((D[5] | D[6]) & T[2] & SREG1_File_Select & ~DREG_File_Select & SREG1_R1) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & DREG_File_Select & SREG1_R1)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & ~DREG_File_Select & SREG1_R1)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[2] & SREG1_File_Select & SREG1_R1)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[3] & SREG2_File_Select & SREG2_R1)| (D[19] & T[2] & RxSel_R1)| (D[30] & T[5] & RxSel_R1)| (D[33] & T[5] & RxSel_R1)| (D[4] & T[2] & RxSel_R1)
		)RF_OutASel <= 3'b000;
		else if (((D[5] | D[6]) & T[2] & SREG1_File_Select & DREG_File_Select & SREG1_R2) | ((D[5] | D[6]) & T[2] & SREG1_File_Select & ~DREG_File_Select & SREG1_R2) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & DREG_File_Select & SREG1_R2)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & ~DREG_File_Select & SREG1_R2)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[2] & SREG1_File_Select & SREG1_R2)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[3] & SREG2_File_Select & SREG2_R2)| (D[19] & T[2] & RxSel_R2)| (D[30] & T[5] & RxSel_R2)| (D[33] & T[5] & RxSel_R2)| (D[4] & T[2] & RxSel_R2)

		)RF_OutASel <= 3'b001;
		else if (((D[5] | D[6]) & T[2] & SREG1_File_Select & DREG_File_Select & SREG1_R3) | ((D[5] | D[6]) & T[2] & SREG1_File_Select & ~DREG_File_Select & SREG1_R3) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & DREG_File_Select & SREG1_R3)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & ~DREG_File_Select & SREG1_R3)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[2] & SREG1_File_Select & SREG1_R3)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[3] & SREG2_File_Select & SREG2_R3)| (D[19] & T[2] & RxSel_R3)| (D[30] & T[5] & RxSel_R3)| (D[33] & T[5] & RxSel_R3)| (D[4] & T[2] & RxSel_R3)
		)RF_OutASel <= 3'b010;
		else if (((D[5] | D[6]) & T[2] & SREG1_File_Select & DREG_File_Select & SREG1_R4) | ((D[5] | D[6]) & T[2] & SREG1_File_Select & ~DREG_File_Select & SREG1_R4) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & DREG_File_Select & SREG1_R4)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & ~DREG_File_Select & SREG1_R4)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[2] & SREG1_File_Select & SREG1_R4)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[3] & SREG2_File_Select & SREG2_R4)| (D[19] & T[2] & RxSel_R4)| (D[30] & T[5] & RxSel_R4)| (D[33] & T[5] & RxSel_R4)| (D[4] & T[2] & RxSel_R4)
		)RF_OutASel <= 3'b011;

		if ((D[0] & T[4]) | (D[1] & T[4] & ~Z) | (D[2] & T[4] & Z)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & ~DREG_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & DREG_File_Select)| (D[33] & T[4])
		)RF_OutBSel <= 3'b101;

		if (T[0])RF_RegSel <= 4'b1111;
		if (((D[5] | D[6]) & T[2] & ~SREG1_File_Select & DREG_File_Select & DREG_R1) | ((D[5] | D[6]) & T[2] & SREG1_File_Select & DREG_File_Select & DREG_R1) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & DREG_File_Select & DREG_R1)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & DREG_File_Select & DREG_R1)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & DREG_File_Select & DREG_R1)| (D[32] & T[2] & RxSel_R1)| (D[18] & T[2] & RxSel_R1)| (D[3] & T[2] & RxSel_R1)| ((D[17] | D[20]) & T[2] & RxSel_R1)
		)RF_RegSel <= 4'b0111;
		else if (((D[5] | D[6]) & T[2] & ~SREG1_File_Select & DREG_File_Select & DREG_R2) | ((D[5] | D[6]) & T[2] & SREG1_File_Select & DREG_File_Select & DREG_R2) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & DREG_File_Select & DREG_R2)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & DREG_File_Select & DREG_R2)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & DREG_File_Select & DREG_R2)| (D[32] & T[2] & RxSel_R2)| (D[18] & T[2] & RxSel_R2)| (D[3] & T[2] & RxSel_R2)| ((D[17] | D[20]) & T[2] & RxSel_R2)
		)RF_RegSel <= 4'b1011;
		else if (((D[5] | D[6]) & T[2] & ~SREG1_File_Select & DREG_File_Select & DREG_R3) | ((D[5] | D[6]) & T[2] & SREG1_File_Select & DREG_File_Select & DREG_R3)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & DREG_File_Select & DREG_R3) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & DREG_File_Select & DREG_R3)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & DREG_File_Select & DREG_R3)| (D[32] & T[2] & RxSel_R3)| (D[18] & T[2] & RxSel_R3)| (D[3] & T[2] & RxSel_R3)| ((D[17] | D[20]) & T[2] & RxSel_R3)
		)RF_RegSel <= 4'b1101;
		else if (((D[5] | D[6]) & T[2] & ~SREG1_File_Select & DREG_File_Select & DREG_R4) | ((D[5] | D[6]) & T[2] & SREG1_File_Select & DREG_File_Select & DREG_R4)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & DREG_File_Select & DREG_R4) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & DREG_File_Select & DREG_R4)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & DREG_File_Select & DREG_R4)| (D[32] & T[2] & RxSel_R4)| (D[18] & T[2] & RxSel_R4)| (D[3] & T[2] & RxSel_R4)| ((D[17] | D[20]) & T[2] & RxSel_R4)
		)RF_RegSel <= 4'b1110;

		
		if (T[1] | ((D[5] | D[6]) & T[2] & SREG1_File_Select & DREG_File_Select) | ((D[5] | D[6]) & T[2] & SREG1_File_Select & ~DREG_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[2] & SREG1_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[3] & SREG2_File_Select)| (D[19] & T[2])| (D[30] & T[3])| (D[33] & T[5])| (D[4] & T[2])
		)ALU_FunSel <= 5'b10000; 
		else if ((D[0] & T[4]) | (D[1] & T[4] & ~Z) | (D[2] & T[4] & Z)| (D[33] & T[4])
		)ALU_FunSel <= 5'b10100;
		else if ((D[7] & T[3] & ~SREG1_File_Select & ~DREG_File_Select) | (D[7] & T[3] & ~SREG1_File_Select & DREG_File_Select) | (D[7] & T[2] & SREG1_File_Select & DREG_File_Select) | (D[7] & T[2] & SREG1_File_Select & ~DREG_File_Select)
		)ALU_FunSel <= 5'b11011; 
		else if ((D[8] & T[3] & ~SREG1_File_Select & ~DREG_File_Select) | (D[8] & T[3] & ~SREG1_File_Select & DREG_File_Select) | (D[8] & T[2] & SREG1_File_Select & DREG_File_Select) | (D[8] & T[2] & SREG1_File_Select & ~DREG_File_Select)
		)ALU_FunSel <= 5'b11100; 
		else if ((D[9] & T[3] & ~SREG1_File_Select & ~DREG_File_Select) | (D[9] & T[3] & ~SREG1_File_Select & DREG_File_Select) | (D[9] & T[2] & SREG1_File_Select & DREG_File_Select) | (D[9] & T[2] & SREG1_File_Select & ~DREG_File_Select)
		)ALU_FunSel <= 5'b11101;
		else if ((D[10] & T[3] & ~SREG1_File_Select & ~DREG_File_Select) | (D[10] & T[3] & ~SREG1_File_Select & DREG_File_Select) | (D[10] & T[2] & SREG1_File_Select & DREG_File_Select) | (D[10] & T[2] & SREG1_File_Select & ~DREG_File_Select)
		)ALU_FunSel <= 5'b11110;
		else if ((D[11] & T[3] & ~SREG1_File_Select & ~DREG_File_Select) | (D[11] & T[3] & ~SREG1_File_Select & DREG_File_Select) | (D[11] & T[2] & SREG1_File_Select & DREG_File_Select) | (D[11] & T[2] & SREG1_File_Select & ~DREG_File_Select)
		)ALU_FunSel <= 5'b11111;
		else if ((D[14] & T[3] & ~SREG1_File_Select & ~DREG_File_Select) | (D[14] & T[3] & ~SREG1_File_Select & DREG_File_Select) | (D[14] & T[2] & SREG1_File_Select & DREG_File_Select) | (D[14] & T[2] & SREG1_File_Select & ~DREG_File_Select)
		)ALU_FunSel <= 5'b10010;
		else if ((D[12] & T[4] & ~DREG_File_Select)| (D[12] & T[4] & DREG_File_Select)| (D[27] & T[4] & ~DREG_File_Select)| (D[27] & T[4] & DREG_File_Select)
		)ALU_FunSel <= 5'b10111;
		else if ((D[13] & T[4] & ~DREG_File_Select)| (D[13] & T[4] & DREG_File_Select)| (D[28] & T[4] & ~DREG_File_Select)| (D[28] & T[4] & DREG_File_Select)
		)ALU_FunSel <= 5'b11000;
		else if ((D[15] & T[4] & ~DREG_File_Select)| (D[15] & T[4] & DREG_File_Select)| (D[29] & T[4] & ~DREG_File_Select)| (D[29] & T[4] & DREG_File_Select)
		)ALU_FunSel <= 5'b11001;
		else if ((D[16] & T[4] & ~DREG_File_Select)| (D[16] & T[4] & DREG_File_Select)
		)ALU_FunSel <= 5'b11010;
		else if ((D[21] & T[4] & ~DREG_File_Select)| (D[21] & T[4] & DREG_File_Select)| (D[25] & T[4] & ~DREG_File_Select)| (D[25] & T[4] & DREG_File_Select)
		)ALU_FunSel <= 5'b10100;
		else if ((D[22] & T[4] & ~DREG_File_Select)| (D[22] & T[4] & DREG_File_Select)
		)ALU_FunSel <= 5'b10101; 
		else if ((D[23] & T[4] & ~DREG_File_Select)| (D[23] & T[4] & DREG_File_Select)| (D[26] & T[4] & ~DREG_File_Select)| (D[26] & T[4] & DREG_File_Select)
		)ALU_FunSel <= 5'b10110;

		if (T[0]
		)ALU_WF <= 1'b0;
		else if (((D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & S)| (D[24] & T[3] & ~SREG1_File_Select & S)| (D[24] & T[2] & SREG1_File_Select & S)
		)ALU_WF <= 1'b1;
		
		if ((D[0] & T[2]) | (D[1] & T[2] & ~Z) | (D[2] & T[2] & Z)| (D[32] & T[2])| (D[33] & T[3])| ((D[17] | D[20]) & T[2])
		)MuxASel <= 2'b11;
		else if ((D[0] & T[3]) | (D[1] & T[3] & ~Z) | (D[2] & T[3] & Z) | ((D[5] | D[6]) & T[2] & ~SREG1_File_Select & DREG_File_Select)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & ~SREG1_File_Select & DREG_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[2] & ~SREG1_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[3] & ~SREG2_File_Select)| (D[30] & T[2])| (D[33] & T[2])
		)MuxASel <= 2'b01;
		else if ((D[18] & T[2])| (D[3] & T[2])
		)MuxASel <= 2'b10;
		else if (((D[5] | D[6]) & T[2] & SREG1_File_Select & DREG_File_Select) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & DREG_File_Select) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & DREG_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[2] & SREG1_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[3] & SREG2_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & DREG_File_Select)
		)MuxASel <= 2'b00;					
		if ((D[0] & T[4]) | (D[1] & T[4] & ~Z) | (D[2] & T[4] & Z) | ((D[5] | D[6]) & T[2] & SREG1_File_Select & ~DREG_File_Select) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & ~DREG_File_Select) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & ~DREG_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4] & ~DREG_File_Select)| (D[30] & T[5])| (D[33] & T[4])
		)MuxBSel <= 2'b00;
		else if (((D[5] | D[6]) & T[2] & ~SREG1_File_Select & ~DREG_File_Select)
		)MuxBSel <= 2'b01;
		else if ((D[31] & T[2])
		)MuxBSel <= 2'b10;						
		if ((D[19] & T[2])| (D[30] & T[3])| (D[33] & T[5])| (D[4] & T[2])
		)						MuxCSel <= 1'b0;
		else if ((D[19] & T[3])| (D[30] & T[4])| (D[33] & T[6])| (D[4] & T[3])
		)MuxCSel <= 1'b1;
		
	
		
		if (			T[0] | T[1]| (D[18] & T[2])| (D[19] & T[2])| (D[31] & T[2])| (D[30] & T[3])| (D[33] & T[5])| (D[3] & T[2])| (D[4] & T[2])
		)Mem_CS <= 1'b0;
		else if ((T[2]& ~(D[18] & T[2])& ~(D[19] & T[2])& ~(D[31] & T[2])& ~(D[3] & T[2])& ~(D[4] & T[2]))| (D[30] & T[5])| (D[4] & T[4])
		)Mem_CS <= 1'b1;
		if (				T[0] | T[1]| (D[18] & T[2])| (D[31] & T[2])| (D[3] & T[2])
		)Mem_WR <= 1'b0;
		if ((D[19] & T[2])		| (D[30] & T[3])| (D[33] & T[5])| (D[4] & T[2])			
		)Mem_WR <= 1'b1;
			
		if (T[0] | T[1])IR_Write <= 1'b1;
		else if (T[2])IR_Write <= 1'b0;
		
		if (T[0])IR_LH <= 1'b0;
		else if (T[1])IR_LH <= 1'b1;							 
		if (T[0])									Sequence_Counter_Reset <= 1'b0;
		else if ((D[0] & T[4]) | (D[1] & T[2] & Z) | (D[1] & T[4] & ~Z) | (D[2] & T[2] & ~Z) | (D[2] & T[4] & Z) | ((D[5] | D[6]) & T[3])| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & ~DREG_File_Select) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[3] & ~SREG1_File_Select & DREG_File_Select)| ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & DREG_File_Select) | ((D[7] | D[8] | D[9] | D[10] | D[11] | D[14] | D[24]) & T[2] & SREG1_File_Select & ~DREG_File_Select)| ((D[12] | D[13] | D[15] | D[16] | D[21] | D[22] | D[23] | D[25] | D[26] | D[27] | D[28] | D[29]) & T[4])| (D[32] & T[2])| (D[18] & T[3])| (D[19] & T[3])| (D[31] & T[5])| (D[30] & T[5])| (D[33] & T[6])| (D[3] & T[3])| (D[4] & T[4])| ((D[17] | D[20]) & T[2])
		)Sequence_Counter_Reset <= 1'b1;
		
	end
	
	
endmodule