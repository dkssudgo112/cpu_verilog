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
	// TODO: implement pipelined CPU
	parameter [3:0] ADD=4'b0000, SUB=4'b0001, SLL=4'b0010, SLT=4'b0011, SLTU=4'b0100, XOR=4'b0101, SRL=4'b0110, SRA=4'b0111, OR=4'b1000, AND=4'b1001, BEQ=4'b1010, BNE=4'b1011, BLT=4'b1100, BGE=4'b1101, BLTU=4'b1110, BGEU=4'b1111;
	parameter [1:0] ST = 2'b00, WT = 2'b01, WN = 2'b10, SN = 2'b11;
	reg [1:0] BS;
	reg BP;
	reg [31:0] Writeback;
	reg [100:0] IF_ID;
	reg [250:0] ID_EX; 
	reg [250:0] EX_MEM; 
	reg [250:0] MEM_WB; 
	reg HALT_signal;
	reg DMcsn;
	reg IMcsn;
	reg [6:0] opcode;
	reg [4:0] rd;
	reg [4:0] rs1;
	reg [4:0] rs2;
	reg [3:0] func3;
	reg [6:0] func7;
	reg [11:0] imm12;
	reg [19:0] imm20;
	reg [4:0] shamt;
	reg [11:0] PC_current;
	reg [11:0] PC_next;
	reg [31:0] outans;
	/////////////////////////////////
	reg [31:0] imm12s;
	reg [31:0] imm20s;
	reg [31:0] Aout;
	reg [3:0] ALUop;
	reg signed [31:0] Aop1;
	reg signed [31:0] Aop2;
	reg Aoutcon;
	reg [11:0] imm12tj;
	reg [11:0] imm12tb;
	reg [19:0] imm20t;
	reg signed [11:0] imm12tjs;
	reg signed [11:0] imm12tbs;
	reg signed [19:0] imm20ts;
	reg [31:0] dmem_in;
	reg [11:0] PC_next_1;
	////////////////////////////Control
	reg HazardA1;
	reg HazardA2;
	reg HazardB1;
	reg HazardB2;
	reg [1:0] PCsrc1;
	reg [1:0] PCsrc2;
	reg [3:0] BE;
	reg ALUsrc2;
	reg MemWrite;
	reg RegWrite;
	reg Branch;
	reg Controlsrc;
	reg Flush;
	reg PCwrite;
	reg IF_IDwrite;
	reg [1:0] MemtoReg;
	reg [1:0] answer;
	reg MemRead;
	reg num_s;
	assign D_MEM_BE = BE;
	assign HALT = MEM_WB[93];
	reg lohazz;
	reg storehaz;
	reg jmp;
	reg [14:0] BTB [1023:0];
	reg [10:0] PC_index;
	reg [12:0] i;
	reg branchdo;





	reg [1023:0] ee_cache [127:0];  
	reg vaild [1023:0];
	integer i;
	reg stall;
	reg[9:0] whatis_index;
	reg[11:0] mem_addr;
	reg[1:0] bit_offset;
	reg read_hit;
	reg read_miss;
	reg write_hit;
	reg write_miss;
	reg mem_output[31:0];
	reg dmemdout[127:0];
	reg DMwen;





	always @ (*) begin
		PC_index = PC_current << 2;
	end



	initial begin
		for(i=0;i<1024;i=i+1) begin
			BTB[i] = 0;
		end

		for(i = 0; i< 1024; i=i+1) begin
			vaild[i] = 1'b0;
		end
		stall = 0;
	end






	always @ (*) begin
		if((ID_EX[146])||(ID_EX[247])) begin
			if(BTB[ID_EX[139:128]<<2][14] == 1'b0) begin
				BTB[ID_EX[139:128]<<2][14] = 1'b1;
				BTB[ID_EX[139:128]<<2][13:12] = WT;
				if(ID_EX[146])
					BTB[ID_EX[139:128]<<2][11:0] = $signed(ID_EX[139:128]) + $signed(ID_EX[127:96]);
				else if(ID_EX[247])
					BTB[ID_EX[139:128]<<2][11:0] = $signed(ID_EX[31:0]) + $signed(ID_EX[127:96]);
			end
		end
	end
	always @ (negedge CLK) begin
		if(EX_MEM[80]) begin
			case(BTB[EX_MEM[44:33]<<2][13:12])
				ST : begin
					if(EX_MEM[32])
						BTB[EX_MEM[44:33]<<2][13:12] <= ST;
					else
						BTB[EX_MEM[44:33]<<2][13:12] <= WT;
				end
				WT : begin
					if(EX_MEM[32])
						BTB[EX_MEM[44:33]<<2][13:12] <= ST;
					else
						BTB[EX_MEM[44:33]<<2][13:12] <= WN;
				end
				WN : begin
					if(EX_MEM[32])
						BTB[EX_MEM[44:33]<<2][13:12] <= WT;
					else
						BTB[EX_MEM[44:33]<<2][13:12] <= SN;
				end
				SN : begin
					if(EX_MEM[32])
						BTB[EX_MEM[44:33]<<2][13:12] <= WN;
					else
						BTB[EX_MEM[44:33]<<2][13:12] <= SN;
				end
			endcase
			// $display("BTB = 0x%0x", BTB[EX_MEM[44:33]<<2]);
		end
	end
	

	always @ (*) begin  //if BTB hi or not
		if(I_MEM_DI[6:0] == 7'b1101111) begin
			PC_next_1 = $signed(PC_current) + (imm20ts << 1);
			branchdo = 1'b0;
		end
		else if(BTB[PC_index][14]) begin
			if((BTB[PC_index][13:12] == ST)||(BTB[PC_index][13:12] == WT)) begin
				PC_next_1 = BTB[PC_index][11:0];
				branchdo = 1'b1;
				// $display("BTB = 0x%0x", BTB[PC_index]);
			end
			else begin
				PC_next_1 = PC_current + 4;
				branchdo = 1'b0;
			end
		end
		else begin
			PC_next_1 = PC_current +4;
			branchdo = 1'b0;
		end
	end
	always @ (*) begin //detect load hazard
		if((ID_EX[160] == 1'b1)&&((rs1 == ID_EX[155:151])||(rs2 == ID_EX[155:151]))) begin
			if((opcode == 7'b0100011)||(opcode == 7'b1101111)) begin
				PCwrite = 1'b1;
				IF_IDwrite = 1'b1;
				Controlsrc = 1'b1;
			end
			else begin
				PCwrite = 1'b0;
				IF_IDwrite = 1'b0;
				Controlsrc = 1'b0;
			end
		end
		else begin
			PCwrite = 1'b1;
			IF_IDwrite = 1'b1;
			Controlsrc = 1'b1;
		end
	end
	always @ (*) begin
		if(((MEM_WB[79] == 1'b1) && (MEM_WB[84:80] == EX_MEM[194:190])) && (EX_MEM[77] == 1'b1)) // store rs2 hazard data input
			storehaz = 1'b1;
		else
			storehaz = 1'b0;
	end

	always @ (*) begin //detect data hazard
		if(ID_EX[169:165] == 5'b00000) begin
			HazardA1 = 1'b0;
			HazardA2 = 1'b0;
		end
		else begin
			if(((EX_MEM[76] == 1'b1) && (EX_MEM[85:81] == ID_EX[169:165]))) begin
				if(ID_EX[207])
					HazardA1 = 1'b1;
				else
					HazardA1 = 1'b0;
			end
			else
				HazardA1 = 1'b0;
		if((MEM_WB[79] == 1'b1) && (MEM_WB[84:80] == ID_EX[169:165]))
			HazardA2 = 1'b1;
		else
			HazardA2 = 1'b0;
		end
		if(ID_EX[174:170] == 5'b00000) begin
			HazardB1 = 1'b0;
			HazardB2 = 1'b0;
		end
		else begin
			if(((EX_MEM[76] == 1'b1) && (EX_MEM[85:81] == ID_EX[174:170]))&&(ID_EX[214:208] == 7'b0110011)) begin
				if(ID_EX[207])
					HazardB1 = 1'b1;
				else
					HazardB1 = 1'b0;
			end
			
			else if(((EX_MEM[76] == 1'b1) && (EX_MEM[85:81] == ID_EX[174:170]))&&(ID_EX[214:208] == 7'b1100011)) begin
				if(ID_EX[207])
					HazardB1 = 1'b1;
				else
					HazardB1 =1'b0;
			end
			else
				HazardB1 = 1'b0;
			if(((MEM_WB[79] == 1'b1) && (MEM_WB[84:80] == ID_EX[174:170]))&&(ID_EX[214:208] == 7'b0110011))
				HazardB2 = 1'b1;
			
			else if(((MEM_WB[79] == 1'b1) && (MEM_WB[84:80] == ID_EX[174:170]))&&(ID_EX[214:208] == 7'b1100011))
				HazardB2 = 1'b1;
			else
				HazardB2 = 1'b0;
		end
	end
	always @ (*) begin // Make control
		if (Controlsrc) begin
			case(opcode)
				7'b0110011 : begin // R-type
					ALUsrc2 = 1'b0;
					if((func3 == 3'b000) && (func7 == 7'b0000000))
						ALUop = ADD;
					else if((func3 == 3'b000) && (func7 == 7'b0100000))
						ALUop = SUB;
					else if(func3 == 3'b001)
						ALUop = SLL;
					else if(func3 == 3'b010)
						ALUop = SLT;
					else if(func3 == 3'b011)
						ALUop = SLTU;
					else if(func3 == 3'b100)
						ALUop = XOR;
					else if((func3 == 3'b101) && (func7 == 7'b0000000))
						ALUop = SRL;
					else if((func3 == 3'b101) && (func7 == 7'b0100000))
						ALUop = SRA;
					else if(func3 == 3'b110)
						ALUop = OR;
					else if(func3 == 3'b111)
						ALUop = AND;
					Branch = 1'b0;
					RegWrite = 1'b1;
					MemtoReg = 2'b01;
					MemWrite = 1'b0;
					MemRead = 1'b0;
					answer = 2'b00;
					num_s = 1'b1;
					lohazz = 1;
					jmp = 1'b0;
				end
				7'b0010011 : begin //ADDI, SLTI
					ALUsrc2 = 1'b1;
					if(func3 == 3'b000)
						ALUop = ADD;
					else if(func3 == 3'b010)
						ALUop = SLT;
					else if(func3 == 3'b011)
						ALUop = SLTU;
					else if(func3 == 3'b100)
						ALUop = XOR;
					else if(func3 == 3'b110)
						ALUop = OR;
					else if(func3 == 3'b111)
						ALUop = AND;
					else if(func3 == 3'b001)
						ALUop = SLL;
					else if((func3 == 3'b101) && (func7 == 7'b0000000))
						ALUop = SRL;
					else if((func3 == 3'b101) && (func7 == 7'b0100000))
						ALUop = SRA;
					Branch = 1'b0;
					RegWrite = 1'b1;
					MemtoReg = 2'b01;
					MemWrite = 1'b0;
					MemRead = 1'b0;
					answer = 2'b00;
					num_s = 1'b1;
					lohazz = 1;
					jmp = 1'b0;
				end
				7'b0100011 : begin //SB, SH, SW
					ALUsrc2 = 1'b1;
					Branch = 1'b0;
					ALUop = ADD;
					RegWrite = 1'b0;
					MemWrite = 1'b1;
					MemRead = 1'b0;
					if(func3 == 3'b000)
						BE = 4'b0001;
					else if(func3 == 3'b001)
						BE = 4'b0011;
					else if(func3 == 3'b010)
						BE = 4'b1111;
					answer = 2'b00;
					num_s = 1'b1;
					lohazz = 1;
					jmp = 1'b0;
				end
				7'b0000011 : begin //LB, LH, LW
					ALUsrc2 = 1'b1;
					Branch = 1'b0;
					ALUop = ADD;
					RegWrite = 1'b1;
					MemWrite = 1'b0;
					MemtoReg = 2'b10;
					MemRead = 1'b1;
					if(func3 == 3'b000)
						BE = 4'b0001;
					else if(func3 == 3'b001)
						BE = 4'b0011;
					else if(func3 == 3'b010)
						BE = 4'b1111;
					answer = 2'b11;
					num_s = 1'b1;
					lohazz = 1;
					jmp = 1'b0;
				end
				7'b1100011 : begin //BEQ, BNE, BLT
					ALUsrc2 = 1'b0;
					Branch = 1'b1;
					MemRead = 1'b0;
					MemWrite = 1'b0;
					if(func3 == 3'b000)
						ALUop = BEQ;
					else if(func3 == 3'b001)
						ALUop = BNE;
					else if(func3 == 3'b100)
						ALUop = BLT;
					else if(func3 == 3'b101)
						ALUop = BGE;
					else if(func3 == 3'b110)
						ALUop = BLTU;
					else if(func3 == 3'b111)
						ALUop = BGEU;
					answer = 2'b10;
					num_s = 1'b1;
					lohazz = 1;
					jmp = 1'b0;
				end
				7'b1100111 : begin //JALR
					Branch = 1'b0;
					ALUsrc2 = 1'b0;
					ALUop = ADD;
					MemtoReg = 2'b00;
					MemRead = 1'b0;
					RegWrite = 1'b1;
					MemWrite = 1'b0;
					answer = 2'b01;
					num_s = 1'b1;
					lohazz = 1;
					jmp = 1'b1;
				end
				7'b1101111 : begin //JAL
					ALUsrc2 = 1'b0;
					ALUop = ADD;
					Branch = 1'b0;
					MemRead = 1'b0;
					MemtoReg = 2'b00;
					RegWrite = 1'b1;
					MemWrite = 1'b0;
					answer = 2'b01;
					num_s = 1'b1;
					lohazz = 1;
					jmp = 1'b0;
				end
			endcase
		end
		else begin
			ALUsrc2 = 0;
			Branch = 0;
			MemtoReg = 0;
			RegWrite = 1'b0;
			MemWrite = 1'b0;
			ALUop = 0;
			MemRead = 0;
			num_s = 0;
			answer = 0;
			lohazz = 0;
			jmp = 1'b0;
		end
	end

	always @ (negedge CLK) begin
		if(~RSTn) begin
			IMcsn <= 1'b1;
			//DMcsn <= 1'b1;
		end
		else begin
			IMcsn <= 1'b0;
			//DMcsn <= 1'b0;
		end
	end


	assign I_MEM_CSN = IMcsn;
	assign D_MEM_CSN = DMcsn;

	always @ (*) begin
		imm20t = {I_MEM_DI[31], I_MEM_DI[19:12], I_MEM_DI[20], I_MEM_DI[30:21]};
		imm12tb = {I_MEM_DI[31], I_MEM_DI[7] ,I_MEM_DI[30:25],I_MEM_DI[11:8]};
	end
	always @ (*) begin // sign extend
		if(imm12tb[11] == 1)
			imm12tbs = {20'b11111111111111111111, imm12tb};
		else
			imm12tbs = {20'b0, imm12tb};
		if(imm20t[19] == 1)
			imm20ts = {12'b111111111111, imm20t};
		else
			imm20ts = {12'b0, imm20t};
	end
	
	always @ (*) begin // change right PC
		case(PCsrc2)
			2'b00 : PC_next = PC_next_1;
			2'b01 : PC_next = ID_EX[139:128] + 4;
			2'b10 : PC_next = $signed(ID_EX[139:128]) + $signed(ID_EX[127:96]);
			2'b11 : PC_next = ($signed(ID_EX[31:0]) + $signed(ID_EX[127:96]))&(12'h0xfffffffe);
		endcase
	end
	always@ (posedge CLK) begin // PC update
		if(~RSTn) begin
			PC_current <= 12'b0;
			NUM_INST <= 0;
		end
		else begin
			if(PCwrite)
				PC_current <= PC_next;
		end
	end
	assign I_MEM_ADDR = (PC_current)&(12'h0xFFF);

	always @ (posedge CLK) begin // IF/ID reg update
		if(stall <= 0) begin
			if(IF_IDwrite) begin
				if(Flush) begin
					IF_ID[31:0] <= I_MEM_DI;
					IF_ID[43:32] <= PC_current;
					IF_ID[44] <= branchdo;
					if((I_MEM_DI == 32'h0x00c00093) || (I_MEM_DI == 32'h0x00008067))
						IF_ID[45] <= 1'b1;
					else
						IF_ID[45] <= 1'b0;
					IF_ID[46] <= 1'b1;
				end
				else
					IF_ID[46] <= 1'b0;
			end
		end
	end
	always @ (*) begin //decoding
		opcode = IF_ID[6:0];
		case(opcode)
			7'b0110011 : begin //ADD, SUB, SLI
				rd = IF_ID[11:7];
				func3 = IF_ID[14:12];
				rs1 = IF_ID[19:15];
				rs2 = IF_ID[24:20];
				func7 = IF_ID[31:25];
			end
			7'b0010011 : begin //ADDI, SLTI
				rd = IF_ID[11:7];
				func3 = IF_ID[14:12];
				rs1 = IF_ID[19:15];
				if((func3 == 3'b001) || (func3 == 3'b101)) begin
					imm12 = {7'b0000000, IF_ID[24:20]};
					func7 = IF_ID[31:25];
				end
				else
					imm12 = IF_ID[31:20];
				rs2 = 0;
			end

			7'b0100011 : begin //SB, SH, SW
				func3 = IF_ID[14:12];
				rs1 = IF_ID[19:15];
				rs2 = IF_ID[24:20];
				imm12 = {IF_ID[31:25], IF_ID[11:7]};
			end

			7'b0000011 : begin //LB, LH, LW
				rd = IF_ID[11:7];
				func3 = IF_ID[14:12];
				rs1 = IF_ID[19:15];
				rs2 = 0;
				imm12 = IF_ID[31:20];
			end

			7'b1100011 : begin //BEQ, BNE, BLT
				func3 = IF_ID[14:12];
				rs1 = IF_ID[19:15];
				rs2 = IF_ID[24:20];
				imm12 = {IF_ID[31], IF_ID[7] ,IF_ID[30:25],IF_ID[11:8]};
			end

			7'b1100111 : begin //JALR
				rd = IF_ID[11:7];
				func3 = IF_ID[14:12];
				rs1 = IF_ID[19:15];
				rs2 = 0;
				imm12 = IF_ID[31:20];
			end

			7'b1101111 : begin //JAL
				rd = IF_ID[11:7];
				rs1 = 0;
				rs2 = 0;
				imm20 = {IF_ID[31], IF_ID[19:12], IF_ID[20], IF_ID[30:21]};
			end
		endcase
	end
	assign RF_RA1 = rs1;
	assign RF_RA2 = rs2;
	always @ (*) begin // sign extend
		if(imm12[11] == 1)
			imm12s = {20'b11111111111111111111, imm12};
		else
			imm12s = {20'b0, imm12};
		if(imm20[19] == 1)
			imm20s = {12'b111111111111, imm20};
		else
			imm20s = {12'b0, imm12};
	end
	always @ (posedge CLK) begin  ////////////////////////ID/EX reg update
		if(stall <= 0) begin
			if(Flush) begin
				ID_EX[31:0] <= RF_RD1;
				ID_EX[63:32] <= RF_RD2;
				ID_EX[95:64] <= imm12s;
				ID_EX[127:96] <= imm12s << 1;
				ID_EX[139:128] <= IF_ID[43:32]; //PC_current
				ID_EX[140] <= IF_ID[44]; //BP
				ID_EX[141] <= ALUsrc2;
				ID_EX[145:142] <= ALUop;
				ID_EX[146] <= Branch;
				ID_EX[147] <= RegWrite;
				ID_EX[148] <= MemWrite;
				ID_EX[150:149] <= MemtoReg;
				ID_EX[155:151] <= rd;
				ID_EX[159:156] <= BE;
				ID_EX[160] <= MemRead;
				ID_EX[162:161] <= answer;
				ID_EX[163] <= num_s;
				ID_EX[164] <= IF_ID[45]; //Halt
				ID_EX[169:165] <= rs1;
				ID_EX[174:170] <= rs2;
				ID_EX[206:175] <= IF_ID[31:0];
				ID_EX[207] <= lohazz;
				ID_EX[214:208] <= opcode;
				ID_EX[246:215] <= imm20s << 1;
				ID_EX[247] <= jmp;
			end
			else
				ID_EX <= 0;
			if(IF_ID[46] == 1'b0)
				ID_EX <= 0;
		end
	end
	
	always @ (*) begin //branch PC
		if(ID_EX[146]) begin
			if(ID_EX[140] == Aoutcon) begin
				PCsrc2 = 2'b00;
				Flush = 1'b1;
			end
			else begin
				if(Aoutcon == 1'b0) begin
					PCsrc2 = 2'b01;
					Flush = 1'b0;
				end
				else begin
					PCsrc2 = 2'b10;
					Flush = 1'b0;
				end
			end
		end
		else if(ID_EX[214:208] == 7'b1100111) begin
			PCsrc2 = 2'b11;
			Flush = 1'b0;
		end
		else begin
			PCsrc2 = 2'b00;
			Flush = 1'b1;
		end
	end

	always @ (*) begin ////////////// ALU mux
		if(HazardA1)
			if((EX_MEM[201:195] == 7'b1101111) || (EX_MEM[201:195] == 7'b1100111))
				Aop1 =  EX_MEM[44:33] + 4;
			else
				Aop1 = EX_MEM[31:0];
		else if(HazardA2) begin
			if((MEM_WB[233:227] == 7'b1101111) || (MEM_WB[233:227] == 7'b1100111))
				Aop1 =  MEM_WB[75:64] + 4;
			else begin
				if(MEM_WB[189])
					Aop1 = MEM_WB[31:0];
				else
					Aop1 = MEM_WB[63:32];
			end
		end
		else
			Aop1 = ID_EX[31:0];
		if(ID_EX[141])
			Aop2 = ID_EX[95:64];
		else begin			
			if(HazardB1)
				if((EX_MEM[201:195] == 7'b1101111) || (EX_MEM[201:195] == 7'b1100111))
					Aop2 = EX_MEM[44:33] + 4;
				else
					Aop2 = EX_MEM[31:0];
			else if(HazardB2) begin
				if((MEM_WB[233:227] == 7'b1101111) || (MEM_WB[233:227] == 7'b1100111))
					Aop2 = MEM_WB[75:64] + 4;
				else begin
					if(MEM_WB[189])
						Aop2 = MEM_WB[31:0];
					else
						Aop2 = MEM_WB[63:32];
				end
			end
			else
				Aop2 = ID_EX[63:32];
		end
	end

	always @ (*) begin //////////ALU
		case(ID_EX[145:142])
			ADD : Aout = Aop1 + Aop2;
			SUB : Aout = Aop1 - Aop2;
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
			end
			BNE : begin
				if(Aop1 != Aop2)
					Aoutcon = 1;
				else
					Aoutcon = 0;
			end
			BLT : begin
				if(Aop1 < Aop2)
					Aoutcon = 1;
				else
					Aoutcon = 0;
			end
			BGE : begin
				if(Aop1 >= Aop2)
					Aoutcon = 1;
				else
					Aoutcon = 0;
			end
			BLTU : begin
				if($unsigned(Aop1) < $unsigned(Aop2))
					Aoutcon = 1;
				else
					Aoutcon = 0;
			end
			BGEU : begin
				if($unsigned(Aop1) >= $unsigned(Aop2))
					Aoutcon = 1;
				else
					Aoutcon = 0;
			end
		endcase
	end


	always @ (posedge CLK) begin // EX/MEM reg update
		if(stall <= 0) begin
			EX_MEM[31:0] <= Aout;
			EX_MEM[32] <= Aoutcon;
			EX_MEM[44:33] <= ID_EX[139:128]; //PC_current
			EX_MEM[75:44] <= ID_EX[63:32]; //RD2
			EX_MEM[76] <= ID_EX[147]; //regwrite
			EX_MEM[77] <= ID_EX[148]; //memwrite
			EX_MEM[79:78] <= ID_EX[150:149]; //memtoreg
			EX_MEM[80] <= ID_EX[146]; //Branch
			EX_MEM[85:81] <= ID_EX[155:151]; // rd
			EX_MEM[89:86] <= ID_EX[159:156]; // BE
			EX_MEM[91:90] <= ID_EX[162:161]; //answer
			EX_MEM[92] <= ID_EX[163]; //num_s
			EX_MEM[93] <= ID_EX[164]; //Halt
			EX_MEM[98:94] <= ID_EX[169:165]; //rs1
			EX_MEM[130:99] <= Aop1;
			EX_MEM[152:131] <= Aop2;
			EX_MEM[156:153] <= ID_EX[145:142];
			EX_MEM[188:157] <= ID_EX[206:175];
			EX_MEM[189] <= ID_EX[160]; //memread
			EX_MEM[194:190] <= ID_EX[174:170]; //rs2
			EX_MEM[201:195] <= ID_EX[214:208]; //opcode
		end
	end

	// ---------------------------------------------------------------------------------------------------------

	always @ (*) begin
		if(storehaz) begin
			if(MEM_WB[189] == 1'b1)
				dmem_in = MEM_WB[31:0];
			else
				dmem_in = MEM_WB[63:32];
		end
		else
			dmem_in = EX_MEM[75:44];
	end
	//assign D_MEM_WEN = ~(EX_MEM[77]);
	//assign D_MEM_BE = EX_MEM[89:86];
	//assign D_MEM_ADDR = (32'h0x3FFF)&(EX_MEM[31:0]);
	//assign D_MEM_DOUT = dmem_in;

	// ---------------------------------------------------------------------------------------------------------

	always @ (posedge CLK) begin
		MEM_WB[31:0] <= mem_output; //D_MEM_DI
		MEM_WB[63:32] <= EX_MEM[31:0]; //Aout
		MEM_WB[75:64] <= EX_MEM[44:33]; //PC_current
		MEM_WB[76] <= EX_MEM[32]; //Aoutcon
		MEM_WB[78:77] <= EX_MEM[79:78]; //memtoreg
		MEM_WB[79] <= EX_MEM[76]; //regwrite
		MEM_WB[84:80] <= EX_MEM[85:81]; //rd
		MEM_WB[88:85] <= EX_MEM[89:86]; //BE
		MEM_WB[89] <= EX_MEM[76]; //regwrite
		MEM_WB[91:90] <= EX_MEM[91:90]; //answer
		MEM_WB[93] <= EX_MEM[93]; //Halt
		MEM_WB[98:94] <= EX_MEM[98:94]; //rs1
		MEM_WB[130:99] <= EX_MEM[130:99];
		MEM_WB[152:131] <= EX_MEM[152:131];
		MEM_WB[156:153] <= EX_MEM[156:153];
		MEM_WB[188:157] <= EX_MEM[188:157];
		MEM_WB[189] <= EX_MEM[189]; // memread
		MEM_WB[194:190] <= EX_MEM[194:190]; //rs2
		MEM_WB[226:195] <= dmem_in;
		MEM_WB[233:227] <= EX_MEM[201:195]; //opcode
		MEM_WB[240] <= storehaz;

		if (stall <= 0) begin
			MEM_WB[92] <= EX_MEM[92]; //num_s
		end
		else if(stall >0) begin
			MEM_WB[92] <= 0;
		end

	end
	assign RF_WE = MEM_WB[79];
	always @ (*) begin //MemtoReg mux
		case(MEM_WB[78:77])
			2'b00 : Writeback = MEM_WB[75:64] + 4;
			2'b01 : Writeback = MEM_WB[63:32];
			2'b10 : begin 
				if(MEM_WB[88:85 == 4'b1111])
					Writeback = MEM_WB[31:0];
				else if(MEM_WB[88:85 == 4'b0011])
					Writeback = MEM_WB[15:0];
				else if(MEM_WB[88:85 == 4'b0001])
					Writeback = MEM_WB[7:0];
			end
		endcase
	end
	always @ (*) begin
		case(MEM_WB[91:90])
			2'b00 : outans = MEM_WB[63:32];
			2'b01 : outans = MEM_WB[75:64] + 4;
			2'b10 : outans = MEM_WB[76];
			2'b11 : outans = MEM_WB[31:0];
		endcase
	end




	// ---------------------------------------------------------------------------------------------------------

	assign D_MEM_ADDR = whatis_index;
	assign D_MEM_DOUT = dmemdout;
	assign D_MEM_WEN = DMwen;

	always @ (*) begin // cache control
		mem_addr = EX_MEM[31:0]; //to be revised - alu out
		whatis_index = mem_addr[11:2];
		bit_offset = mem_addr[1:0];

		if(vaild[whatis_index] == 1 && EX_MEM[189] == 1) begin
			read_hit = 1;
			stall = 0;

			EX_MEM[189] = 0;
		end
		else if(vaild[whatis_index] == 0 && EX_MEM[189] == 1) begin
			read_miss = 1;
			stall = 9;

			EX_MEM[189] = 0;
		end
		else if(vaild[whatis_index] == 1 && EX_MEM[77] == 1) begin
			write_hit = 1;
			stall = 8;

			EX_MEM[77] = 0;
		end
		else if(vaild[whatis_index] == 1 && EX_MEM[77] == 1) begin
			write_miss = 1;
			stall = 17;

			EX_MEM[77] = 0;
		end

	always @(posedge clk) begin 
		
		if(stall >0 ) stall <= stall -1;

		if(read_hit == 1) begin
			if(bit_offset == 2'b00) begin
				mem_output <= ee_cache[whatis_index][31:0];
			end	
			else if(bit_offset == 2'b01) begin
				mem_output <= ee_cache[whatis_index][63:32];
			end
			else if(bit_offset == 2'b10) begin
				mem_output <= ee_cache[whatis_index][95:64];
			end
			else if(bit_offset == 2'b11) begin
				mem_output <= ee_cache[whatis_index][127:96];
			end	
			read_hit <= 0;		
		end

		else if(read_miss ==  1 && stall == 8) begin //memory acsess 
			//inputdmemaddr <= whatis_index;
			DMcsn <= 1'b1;
		end

		else if(read_miss == 1 && stall == 1) begin // cahce update
			ee_cache[whatis_index] <= D_MEM_DI;
			vaild[whatis_index] <= 1;
			if(bit_offset == 2'b00) begin
				mem_output <= D_MEM_DI[31:0];
			end	
			else if(bit_offset == 2'b01) begin
				mem_output <= D_MEM_DI[63:32];
			end
			else if(bit_offset == 2'b10) begin
				mem_output <= D_MEM_DI[95:64];
			end
			else if(bit_offset == 2'b11) begin
				mem_output <= D_MEM_DI[127:96];
			end
		end	
		else if (read_miss == 1 && stall == 1) begin
			DMcsn <= 1'b0;
			read_miss <=0;
		end

		else if(write_hit == 1 && stall == 7) begin // cache update 
			if(bit_offset == 2'b00) begin
				ee_cache[whatis_index][31:0] <= dmem_in;
			end	
			else if(bit_offset == 2'b01) begin
				ee_cache[whatis_index][63:32] <= dmem_in;
			end
			else if(bit_offset == 2'b10) begin
				ee_cache[whatis_index][95:64] <= dmem_in;
			end
			else if(bit_offset == 2'b11) begin
				ee_cache[whatis_index][127:96] <= dmem_in;
			end
		end
		else if(write_hit == 1 && stall == 6) begin //  memory access

			DMcsn <= 1'b1;
			//dmemaddr <= whatis_index;
			DMwen <= 1'b1;

			dmemdout <= ee_cache[whatis_index];

			vaild[whatis_index] <=1;
			write_hit <= 0;		
		end
		else if(write_hit == 1 && stall == 6) begin
			DMcsn <= 1'b0;
			DMwen <= 1'b0;
		end

		else if(write_miss == 1 && stall == 16) begin //memory access
			DMcsn <= 1'b1;
			//dmemaddr <= whatis_index;
		end

		else if(write_miss == 1 && stall == 8) begin //cache change
			ee_cache[whatis_index] <= D_MEM_DI;
			DMcsn <= 1'b0;

		end
		

		else if(write_miss == 1 && stall == 7) begin //cache update
			if(bit_offset == 2'b00) begin
				ee_cache[whatis_index][31:0] <= dmem_in;
			end	
			else if(bit_offset == 2'b01) begin
				ee_cache[whatis_index][63:32] <= dmem_in;
			end
			else if(bit_offset == 2'b10) begin
				ee_cache[whatis_index][95:64] <= dmem_in;
			end
			else if(bit_offset == 2'b11) begin
				ee_cache[whatis_index][127:96] <= dmem_in'
			end
			DMcsn <= 1'b0;
			DMwen <= 1'b0;
		end
		else if(write_miss == 1 && stall == 6) begin //meory access
			DMcsn <= 1'b1;
			DMwen <= 1'b1;
			//dmemdout <= ee_cache[whatis_index];
			vaild[whatis_index] <=1;
			write_miss <= 0;	
		end
		else if(write_miss == 1 && stall == 1) begin
			DMcsn <= 1'b0;
			DMwen <= 1'b0;
		end

	end


	// ---------------------------------------------------------------------------------------------------------



	always @ (negedge CLK) begin
		if(MEM_WB[92]) begin
			NUM_INST <= NUM_INST + 1;
			// $display("NUM_inst = %d", NUM_INST + 1);
			// $display("PC = 0x%0x", MEM_WB[75:64]);
			// $display("Aout = 0x%0x", MEM_WB[63:32]);
			// $display("Aop1 = 0x%0x, Aop2 = 0x%0x", MEM_WB[130:99], MEM_WB[152:131]);
			// $display("ALUop = %b", MEM_WB[156:153]);
			// $display("Rd = %b", MEM_WB[84:80]);
			// $display("Rs1 = %b", MEM_WB[98:94]);
			// $display("Rs2 = %b", MEM_WB[194:190]);
			// $display("answer = %b", MEM_WB[91:90]);
			// $display("Instrucion = 0x%0x", MEM_WB[188:157]);
			// $display("Dread = 0x%0x", MEM_WB[31:0]);
			// $display("D_memory = 0x%0x", MEM_WB[226:195]);
			// $display("sotrehaz = %b", MEM_WB[240]);
		end
	end
	assign OUTPUT_PORT = outans; 
	assign RF_WD = Writeback;
	assign RF_WA1 = MEM_WB[84:80];
endmodule //

