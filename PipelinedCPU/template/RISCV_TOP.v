module RISCV_TOP (
	//General Signals
	input wire CLK,
	input wire RSTn,

	//I-Memory Signals
	output wire I_MEM_CSN,
	input wire [31:0] I_MEM_DI,//input from IM
	output reg [11:0] I_MEM_ADDR,//in byte address

	//D-Memory Signals
	output wire D_MEM_CSN,
	input wire [31:0] D_MEM_DI,
	output wire [31:0] D_MEM_DOUT,
	output wire [11:0] D_MEM_ADDR,//in word address
	output wire D_MEM_WEN,
	output wire [3:0] D_MEM_BE,

	//RegFile Signals
	output wire RF_WE,
	output wire [4:0] RF_RA1,
	output wire [4:0] RF_RA2,
	output wire [4:0] RF_WA1,
	input wire [31:0] RF_RD1,
	input wire [31:0] RF_RD2,
	output wire [31:0] RF_WD,
	output wire HALT,                   // if set, terminate program
	output reg [31:0] NUM_INST,         // number of instruction completed
	output wire [31:0] OUTPUT_PORT      // equal RF_WD this port is used for test
	);


	reg IFIDwrite;
	reg PCwrite;
	reg [11:0] PC_current_IF;
	reg [11:0] PC_next;

	// TODO: implement pipelined CPU


	initial begin
		NUM_INST <= 0;

		PCwrite <= 1;
		IFIDwrite <= 1;
		PC_current_IF <= 0;
		PC_next <= 0;


	end
	// Todo: something need to add

	assign OUTPUT_PORT = RF_WD;

	always @ (negedge CLK) begin
		if (RSTn) NUM_INST <= NUM_INST + 1;
	end
	// Todo: num_inst - 1 * stall - (others maybe branchmiss)


	// Csn part ----------------------------------------------------------------------------------
	assign I_MEM_CSN = ~RSTn;
	assign D_MEM_CSN = ~RSTn;
	// Csn part end----------------------------------------------------------------------------------







	// HALT part ----------------------------------------------------------------------------------
	reg HALT_signal;
	always @ (posedge CLK) begin
		if((I_MEM_DI == 32'h0x00c00093) || (I_MEM_DI == 32'h0x00008067))
			HALT_signal <= 1'b1;
	end
	assign HALT = HALT_signal;
	// HALT part end ----------------------------------------------------------------------------------











	// pipeline register part --------------------------------------------------------------------------------------------------------------------
	reg[31:0] IR_IF, IR_ID, IR_EX, IR_MEM, IR_WB;
	reg[31:0] rd1_ID, rd1_EX, rd1_MEM, rd1_WB,         rd2_ID, rd2_EX, rd2_MEM;
	reg[31:0] rd1_EX_Forwarded, rd2_EX_Forwarded;

	// ALUsrc1, ALUsrc2, ALUop, RegWrite, MemWrite, Branch, MemtoReg, MemRead (Control part)
	reg [3:0] ALUop_ID, ALUop_EX;
	reg[1:0] ALUsrc1_ID, ALUsrc1_EX;
	reg[2:0] ALUsrc2_ID, ALUsrc2_EX; //2
	reg Branch_ID, Branch_EX, Branch_MEM,        MemRead_ID, MemRead_EX, MemRead_MEM, MemRead_WB,       MemWrite_ID, MemWrite_EX, MemWrite_MEM; //3
	reg RegWrite_ID, RegWrite_EX, RegWrite_MEM, RegWrite_WB;
	reg [1:0] MemtoReg_ID, MemtoReg_EX, MemtoReg_MEM, MemtoReg_WB; // 4

	reg PCjmp, PCjmpr;
	
	// Instruction part
	reg[31:0] InsR, InsR_ID, InsR_EX, InsR_MEM, InsR_WB;
	reg[4:0] RF_Addr_ID, RF_Addr_EX, RF_Addr_MEM, RF_Addr_WB;
	reg[31:0] imm12s_ID, imm12s_EX,               imm20s_ID, imm20s_EX;
	reg[2:0] func3_ID, func3_EX, func3_MEM;
	reg[6:0] func7_ID, func7_EX;
	reg[3:0] BE_ID, BE_EX, BE_MEM, BE_WB;

	// PC part
	
	reg PCsrc;
	
	reg [11:0] PC_current_ID, PC_current_EX;
	reg [11:0] nPCm_EX, nPCm_MEM;


	// Mem? ALU?
	reg [31:0] ALUoutR_EX, ALUoutR_MEM, ALUoutR_WB;
	reg [31:0] DMEM_rd_MEM, DMEM_rd_WB;
	reg Zero_EX, Zero_MEM;

	// For Fowarding
	reg [4:0] rs1_ID, rs1_EX, rs2_ID, rs2_EX;



	// For Hazrard control
	

	always @ (*) begin
		if(I_MEM_DI) begin
			IR_IF = I_MEM_DI;
			InsR = I_MEM_DI;
		end
	end	

	always @(posedge CLK) begin
		if (~RSTn) begin
			//PC?


			//IF ID
			if(IFIDwrite) begin
				PC_current_ID <= PC_current_IF;
				IR_ID <= IR_IF;
				InsR_ID <= InsR;
			end
			


			//ID EX
			PC_current_EX <= PC_current_ID;
			
			IR_EX <= IR_ID;
			rd1_ID <= rd1_EX;
			rd2_ID <= rd2_EX;

			ALUop_EX <= ALUop_ID;
			ALUsrc1_EX <= ALUsrc1_ID;
			ALUsrc2_EX <= ALUsrc2_ID;
			Branch_EX <= Branch_ID;
			MemRead_EX <= MemRead_ID;
			MemWrite_EX <= MemWrite_ID;
			RegWrite_EX <= RegWrite_ID;
			MemtoReg_EX <= MemtoReg_ID;

			InsR_EX <= InsR_ID;
			RF_Addr_EX <= RF_Addr_ID;
			imm12s_EX <= imm12s_ID;
			imm20s_EX <= imm20s_ID;
			func3_EX <= func3_ID;
			func7_EX <= func7_ID;
			BE_EX <= BE_ID;

			rs1_EX <= rs1_ID;
			rs2_EX <= rs2_ID;

			//EX MEM
			nPCm_MEM <= nPCm_EX;

			IR_MEM <= IR_EX;
			rd1_MEM <= rd1_EX;
			rd2_MEM <= rd2_EX;

			Branch_MEM <= Branch_EX;
			MemRead_MEM <= MemRead_EX;
			MemWrite_MEM <= MemWrite_EX;

			RegWrite_MEM <= RegWrite_EX;
			MemtoReg_MEM <= MemtoReg_EX;

			InsR_MEM <= InsR_EX;
			RF_Addr_MEM <= RF_Addr_EX;
			func3_MEM <= func3_EX;
			BE_MEM <= BE_EX;

			ALUoutR_MEM <= ALUoutR_EX;
			Zero_MEM <= Zero_EX;

			//MEM WB
			IR_WB <= IR_MEM;
			rd1_WB <= rd1_MEM;
			
			RegWrite_WB <= RegWrite_MEM;
			MemRead_WB <= MemRead_MEM;

			InsR_WB <= InsR_MEM;
			RF_Addr_WB <= RF_Addr_MEM;
			BE_WB <= BE_MEM;

			ALUoutR_WB <= ALUoutR_MEM;
			DMEM_rd_WB <= DMEM_rd_MEM;

		end

	end
	// pipeline register end-------------------------------------------------------------------------------------------------------











	// PC part1 ------------------------------------------------------------------------------------------------------------------------
	always @ (*) begin
		I_MEM_ADDR = (PC_current_IF) & (12'h0xfff); // add IRWrite
	end

	always @ (posedge CLK) begin //PC counter
		if(~RSTn)
			//PC_current_IF <= 12'b0;
			I_MEM_ADDR <= (PC_current_IF) & (12'h0xfff);
		else begin
			if (PCwrite)  // To be brach prediction
				PC_current_IF <= PC_next;
		end
	end

	always @ (*) begin
		if(PCsrc == 1)
			PC_next= nPCm_MEM;
		else
			PC_next = PC_current_IF + 4;	
	end

	always @ (*) begin
		nPCm_EX = $signed(PC_current_EX + 4) + (imm12s_EX <<< 1); 
	end

	always @ (*) begin
		if(PCjmp == 1) begin
			PC_next=($signed({20'b0, PC_current_EX}) + (imm20s_EX <<< 1));
			if(PCjmpr)
				PC_next = (PC_next) & (12'h0xfffffffe);
		end

	end
	// PC part end ------------------------------------------------------------------------------------------------------------------------













	// Decoding part --------------------------------------------------------------------------------------
	reg [11:0] imm12;
	reg [19:0] imm20;
	reg [6:0] opcode;
	

	always @ (*) begin //decoding
		opcode = InsR_ID[6:0];
		case(opcode)
			7'b0110011 : begin //ADD, SUB, SLI
				RF_Addr_ID = InsR_ID[11:7];
				func3_ID = InsR_ID[14:12];
				rs1_ID = InsR_ID[19:15];
				rs2_ID = InsR_ID[24:20];
				func7_ID = InsR_ID[31:25];		
			end

			7'b0010011 : begin //ADDI, SLTI
				RF_Addr_ID = InsR_ID[11:7];
				func3_ID = InsR_ID[14:12];
				rs1_ID = InsR_ID[19:15];
				if((func3_ID == 3'b001) || (func3_ID == 3'b101)) begin
					imm12 = {7'b0000000, InsR_ID[24:20]};
					func7_ID = InsR_ID[31:25];
				end
				else
					imm12 = InsR_ID[31:20];
			end

			7'b0100011 : begin //SB, SH, SW
				func3_ID = InsR_ID[14:12];
				rs1_ID = InsR_ID[19:15];
				rs2_ID = InsR_ID[24:20];
				imm12 = {InsR_ID[31:25], InsR_ID[11:7]};
			end

			7'b0000011 : begin //LB, LH, LW
				RF_Addr_ID = InsR_ID[11:7];
				func3_ID = InsR_ID[14:12];
				rs1_ID = InsR_ID[19:15];
				imm12 = InsR_ID[31:20];
			end

			7'b1100011 : begin //BEQ, BNE, BLT
				func3_ID = InsR_ID[14:12];
				rs1_ID = InsR_ID[19:15];
				rs2_ID = InsR_ID[24:20];
				imm12 = {InsR_ID[31], InsR_ID[7] ,InsR_ID[30:25],InsR_ID[11:8]};
			end

			7'b1100111 : begin //JALR
				RF_Addr_ID = InsR_ID[11:7];
				func3_ID = InsR_ID[14:12];
				rs1_ID = InsR_ID[19:15];
				imm12 = InsR_ID[31:20];
			end

			7'b1101111 : begin //JAL
				RF_Addr_ID = InsR_ID[11:7];
				imm20 = {InsR_ID[31], InsR_ID[19:12], InsR_ID[20], InsR_ID[30:21]};
			end

			7'b0010111 : begin //AUIPC
				RF_Addr_ID = InsR_ID[11:7];
				imm20 = InsR_ID[31:12];
			end

			7'b0110111 : begin //LUI
				RF_Addr_ID = InsR_ID[11:7];
				imm20 = InsR_ID[31:12];
			end
		endcase
	end
	assign RF_RA1 = rs1_ID;
	assign RF_RA2 = rs2_ID;
	assign RF_WA1 = RF_Addr_WB;
	// Decoding part end-----------------------------------------------------------------------------------








	// sign extend part ----------------------------------------------------------------------------------
	always @ (*) begin // sign extend
		if(imm12[11] == 1)
			imm12s_ID = {20'b11111111111111111111, imm12};
		else
			imm12s_ID = {20'b0, imm12};
		if(imm20[19] == 1)
			imm20s_ID = {12'b111111111111, imm20};
		else
			imm20s_ID = {12'b0, imm20};
	end
	// sign extend part end ----------------------------------------------------------------------------------









	//RF out +  ALU source MUX 1, 2 part -----------------------------------------------------------------------------------
	always @ (posedge CLK) begin
		rd1_ID <= RF_RD1;
		rd2_ID <= RF_RD2;
	end // RF out part

	reg signed [31:0] Aop1;
	reg signed [31:0] Aop2;

	always @(*) begin
		case(ALUsrc1_EX)
			2'b00 : Aop1 = PC_current_EX; //To be revised about PC
			2'b01 : Aop1 = rd1_EX;
			//2'b10 : Aop1 = storePC;  //To be revised about PC
		endcase
	end
	always @ (*) begin
		case(ALUsrc2_EX)
			3'b000 : Aop2 = 4;
			3'b001 : Aop2 = rd2_EX;
			3'b010 : Aop2 = imm12s_EX;
			3'b011 : Aop2 = imm12s_EX <<< 1;
			3'b100 : Aop2 = imm20s_EX;
			3'b101 : Aop2 = imm20s_EX <<< 1;
			3'b110 : Aop2 = {imm20s_EX, 12'b0};
		endcase
	end
	//RF out +  ALU source MUX 1, 2 part source part -----------------------------------------------------------------------------------








	// ALU part ---------------------------------------------------------------------------------------------
	parameter [3:0] ADD=4'b0000, SUB=4'b0001, SLL=4'b0010, SLT=4'b0011, SLTU=4'b0100, XOR=4'b0101, SRL=4'b0110,
	 SRA=4'b0111, OR=4'b1000, AND=4'b1001, BEQ=4'b1010,
	 BNE=4'b1011, BLT=4'b1100, BGE=4'b1101, BLTU=4'b1110, BGEU=4'b1111;

	//reg [31:0] ALUoutR_EX;
	reg [31:0] Aout;
	reg Aoutcon; //Zero
	//reg [31:0] answer;

	always @ (*) begin //ALU
		case(ALUop_EX)
			ADD : begin
				Aout = Aop1 + Aop2;
			end
			SUB : begin
				Aout = Aop1 - Aop2;
			end
			SLL : Aout = Aop1 << Aop2;
			SLT : begin
				if(Aop1 < Aop2)
					Aout = 1;
				else
					Aout = 0;
			end
			SLTU : begin
				if($unsigned(Aop1) < $unsigned(Aop2))
					Aout = 1;
				else
					Aout = 0;
			end
			XOR : Aout = Aop1 ^ Aop2;
			SRL : Aout = Aop1 >> Aop2;
			SRA : Aout = Aop1 >>> Aop2;
			OR : Aout = Aop1 | Aop2;
			AND : Aout = Aop1 & Aop2;
			BEQ : begin
				if(Aop1 == Aop2)
					Aoutcon = 1;
				else
					Aoutcon = 0;
				Aout = Aoutcon;
				Zero_EX = Aoutcon;
			end
			BNE : begin
				if(Aop1 != Aop2)
					Aoutcon = 1;
				else
					Aoutcon = 0;
				Aout = Aoutcon;
				Zero_EX = Aoutcon;
			end
			BLT : begin
				if(Aop1 < Aop2)
					Aoutcon = 1;
				else
					Aoutcon = 0;
				Aout = Aoutcon;
				Zero_EX = Aoutcon;
			end
			BGE : begin
				if(Aop1 >= Aop2)
					Aoutcon = 1;
				else
					Aoutcon = 0;
				Aout = Aoutcon;
				Zero_EX = Aoutcon;
			end
			BLTU : begin
				if($unsigned(Aop1) < $unsigned(Aop2))
					Aoutcon = 1;
				else
					Aoutcon = 0;
				Aout = Aoutcon;
				Zero_EX = Aoutcon;
			end
			BGEU : begin
				if($unsigned(Aop1) >= $unsigned(Aop2))
					Aoutcon = 1;
				else
					Aoutcon = 0;
				Aout = Aoutcon;
				Zero_EX = Aoutcon;
			end
		endcase
	end
	always @ (posedge CLK) begin
		ALUoutR_EX <= Aout;
	end
	// ALU part end-----------------------------------------------------------------------------------------------------------







	// D-Mem part + MemtoReg MUX --------------------------------------------------------------------------------------------------------------
	assign D_MEM_ADDR = (12'h0x3fff)&(ALUoutR_MEM);
	assign D_MEM_DOUT = rd2_MEM;

	
	always @ (posedge CLK) begin // negative?????
		DMEM_rd_MEM <= D_MEM_DI;
	end
	assign D_MEM_BE = BE_MEM;


	reg [31:0] RF_WDD;	
	always @ (*) begin
		case(MemtoReg_WB)
			2'b00 : RF_WDD = ALUoutR_WB;
			2'b01 : begin
				if(BE_WB == 4'b1111)
					RF_WDD = DMEM_rd_WB;
				else if(BE_WB == 4'b0011)
					RF_WDD = DMEM_rd_WB[15:0];
				else if(BE_WB== 4'b0001)
					RF_WDD = DMEM_rd_WB[7:0];
			end
			2'b10 : RF_WDD = ALUoutR_WB;
			//2'b11 : RF_WDD = storePC + 4; // To be revise by PC
		endcase
	end
	assign RF_WD = RF_WDD;	
	// D-Mem part + MemtoReg MUX end --------------------------------------------------------------------------------------------------------------












	// control part ------------------------------------------------------------------------------------------------------------------------
	always @ (*) begin //control unit
		case(opcode)
			7'b0110011 : begin //ADD, SUB
				ALUsrc1_ID = 2'b00;
				ALUsrc2_ID = 2'b00; // To Be revised
				MemtoReg_ID = 2'b00;
				Branch_ID = 1'b0;
				RegWrite_ID = 1'b1;
				MemWrite_ID = 1'b0;
				PCjmp = 1'b0; // To Be revised
				PCjmpr = 1'b0; // To Be revised				
				if((func3_ID == 3'b000) && (func7_ID == 7'b0000000))
					ALUop_ID = ADD;
				else if((func3_ID == 3'b000) && (func7_ID == 7'b0100000))
					ALUop_ID = SUB;
				else if(func3_ID == 3'b001)
					ALUop_ID = SLL;
				else if(func3_ID == 3'b010)
					ALUop_ID = SLT;
				else if(func3_ID == 3'b011)
					ALUop_ID = SLTU;
				else if(func3_ID == 3'b100)
					ALUop_ID = XOR;
				else if((func3_ID == 3'b101) && (func7_ID == 7'b0000000))
					ALUop_ID = SRL;
				else if((func3_ID == 3'b101) && (func7_ID == 7'b0100000))
					ALUop_ID = SRA;
				else if(func3_ID == 3'b110)
					ALUop_ID = OR;
				else if(func3_ID == 3'b111)
					ALUop_ID = AND;	
			end

			7'b0010011 : begin //ADDI, SLTI
				ALUsrc1_ID = 2'b00;
				ALUsrc2_ID = 2'b01; // To Be revised
				MemtoReg_ID = 2'b00;
				Branch_ID = 1'b0;
				RegWrite_ID = 1'b1;
				MemWrite_ID = 1'b0;
				PCjmp = 1'b0; // To Be revised
				PCjmpr = 1'b0; // To Be revised
				if(func3_ID == 3'b000)
					ALUop_ID = ADD;
				else if(func3_ID == 3'b010)
					ALUop_ID = SLT;
				else if(func3_ID == 3'b011)
					ALUop_ID = SLTU;
				else if(func3_ID == 3'b100)
					ALUop_ID = XOR;
				else if(func3_ID == 3'b110)
					ALUop_ID = OR;
				else if(func3_ID == 3'b111)
					ALUop_ID = AND;
				else if(func3_ID == 3'b001)
					ALUop_ID = SLL;
				else if((func3_ID == 3'b101) && (func7_ID == 7'b0000000))
					ALUop_ID = SRL;
				else if((func3_ID == 3'b101) && (func7_ID == 7'b0100000))
					ALUop_ID = SRA;
				
			end
			7'b0100011 : begin //SB, SW, SH
				ALUsrc1_ID = 2'b00;
				ALUsrc2_ID = 2'b01; // To Be revised	
				ALUop_ID = ADD;
				Branch_ID = 1'b0;
				RegWrite_ID = 1'b0;
				MemWrite_ID = 1'b1;
				MemtoReg_ID = 2'b00;
				PCjmp = 1'b0; // To Be revised
				PCjmpr = 1'b0; // To Be revised
				if(func3_ID == 3'b000)
					BE_ID = 4'b0001;
				else if(func3_ID == 3'b001)
					BE_ID = 4'b0011;
				else if(func3_ID == 3'b010)
					BE_ID = 4'b1111;
			end

			7'b0000011 : begin //LB, LH, LW
				ALUsrc1_ID = 2'b00;
				ALUsrc2_ID = 2'b01; // To Be revised
				ALUop_ID = ADD;
				Branch_ID = 1'b0;
				RegWrite_ID = 1'b1;
				MemWrite_ID = 1'b0;
				MemtoReg_ID = 2'b01;
				PCjmpr = 1'b0; // To Be revised
				PCjmp = 1'b0; // To Be revised
				if(func3_ID == 3'b000)
					BE_ID = 4'b0001;
				else if(func3_ID == 3'b001)
					BE_ID = 4'b0011;
				else if(func3_ID == 3'b010)
					BE_ID = 4'b1111;
			end

			7'b1100011 : begin //BEQ, BNE, BLT
				ALUsrc1_ID = 2'b00;
				ALUsrc2_ID = 2'b00; // To Be revised
				Branch_ID = 1'b1;
				PCjmpr = 1'b0;  // To Be revised
				PCjmp = 1'b0;  // To Be revised
				MemWrite_ID = 1'b0;
				RegWrite_ID = 1'b0;
				MemtoReg_ID = 2'b00;
				if(func3_ID == 3'b000)
					ALUop_ID = BEQ;
				else if(func3_ID == 3'b001)
					ALUop_ID = BNE;
				else if(func3_ID == 3'b100)
					ALUop_ID = BLT;
				else if(func3_ID == 3'b101)
					ALUop_ID = BGE;
				else if(func3_ID == 3'b110)
					ALUop_ID = BLTU;
				else if(func3_ID == 3'b111)
					ALUop_ID = BGEU;
			end

			7'b1100111 : begin //JALR
				ALUsrc1_ID = 2'b00;
				ALUsrc2_ID = 2'b01; // To Be revised
				ALUop_ID = ADD;
				Branch_ID = 1'b0;
				PCjmp = 1'b1;  // To Be revised
				PCjmpr = 1'b1;  // To Be revised
				MemWrite_ID = 1'b0;
				RegWrite_ID = 1'b1;
				MemtoReg_ID = 2'b10;
			end

			7'b1101111 : begin //JAL
				ALUsrc1_ID = 2'b10;
				ALUsrc2_ID = 2'b10; // To Be revised
				ALUop_ID = ADD;
				Branch_ID = 1'b0;
				PCjmp = 1'b1;  // To Be revised
				PCjmpr = 1'b0;  // To Be revised
				MemWrite_ID = 1'b0;
				RegWrite_ID = 1'b1;
				MemtoReg_ID = 2'b10;
			end
		endcase
	end

	reg CtrSrc_ID;
	always @ (*) begin 
		if(CtrSrc_ID == 0) begin
			ALUsrc1_ID = 0;
			ALUsrc2_ID = 0;
			ALUop_ID = 0;
			Branch_ID = 0;
			PCjmp = 0;
			PCjmpr = 0;
			MemWrite_ID = 0;
			RegWrite_ID = 0;
			MemtoReg_ID = 0;
		end
	end
// control part end------------------------------------------------------------------------------------------------------------------------










// other control part ----------------------------------------------------
	assign RF_WE = RegWrite_MEM;
	assign D_MEM_WEN = ~(MemWrite_MEM);

	// branch MUX -> To Be revised about PC (Branch_MEM) && (Zero_MEM)
	always @ (*) begin 
		PCsrc = Branch_MEM && ALUoutR_MEM;
	end


// other control part end ----------------------------------------------------







// ForwardA,B MUX ----------------------------------------------------------------
	reg[1:0] ForwardA;
	reg[1:0] ForwardB; 
	always @ (*) begin 
		if(ForwardA == 0)
			rd1_EX_Forwarded = rd1_EX;
		else if(ForwardA == 1)
			rd1_EX_Forwarded = ALUoutR_MEM;
		else
			rd1_EX_Forwarded = RF_WDD;
	end
	always @ (*) begin 
		if(ForwardB == 0)
			rd2_EX_Forwarded = rd2_EX;
		else if(ForwardA == 1)
			rd2_EX_Forwarded = ALUoutR_MEM;
		else
			rd2_EX_Forwarded = RF_WDD;
	end
// ForwardA,B MUX end ----------------------------------------------------------------







// Forward Unit -------------------------------------------------------------------------
	always @ (*) begin 
		if(RegWrite_MEM == 1 && RF_Addr_MEM == rs1_EX)
			ForwardA = 1;
		else if(RegWrite_MEM == 1 && RF_Addr_MEM == rs2_EX)
			ForwardB = 1;
		else if(RegWrite_WB == 1 && RF_Addr_WB == rs1_EX && (RF_Addr_MEM != rs1_EX || RegWrite_MEM == 0))
			ForwardA = 2;
		else if(RegWrite_WB == 1 && RF_Addr_WB == rs2_EX && (RF_Addr_MEM != rs2_EX || RegWrite_MEM == 0))
			ForwardB = 2;
	end	
// Forward Unit end -------------------------------------------------------------------------










// Hazard Unit --------------------------------------------------------------------------------------------
	always @ (*) begin 
		if(MemRead_EX == 1 && (rs1_ID == rs2_EX || rs2_EX == rs2_ID)) begin
			PCwrite = 0;
			IFIDwrite = 0;
			CtrSrc_ID = 0;
		end

	end


// Hazard Unit end --------------------------------------------------------------------------------------------



endmodule //
