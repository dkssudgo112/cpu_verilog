`timescale 1ns/10ps
`define NUM_TEST 23
`define TESTID_SIZE 5

module TB_RISCV_inst();
	//General Signals
	wire CLK;
	wire RSTn;
	//Memory Signals

	wire I_MEM_CSN;
	wire [31:0] I_MEM_DOUT;
	wire [11:0] I_MEM_ADDR;
	wire D_MEM_CSN;
	wire D_MEM_WEN;
	//wire [3:0] D_MEM_BE; // Cache Lab
	wire [127:0] D_MEM_DOUT; // Cache Lab
	wire [127:0] D_MEM_DI; // Cache Lab
	wire [9:0] D_MEM_ADDR; // Cache Lab
	wire RF_WE;
	wire [4:0] RF_RA1;
	wire [4:0] RF_RA2;
	wire [4:0] RF_WA;
	wire [31:0] RF_RD1;
	wire [31:0] RF_RD2;
	wire [31:0] RF_WD;
	wire HALT;
	wire [31:0] NUM_INST;
	wire [31:0] OUTPUT_PORT;

	//Clock Reset Generator
	RISCV_CLKRST riscv_clkrst1 (
		.CLK           (CLK),
		.RSTn          (RSTn)
	);

	//CPU Core top
	RISCV_TOP riscv_top1 (
		//General Signals
		.CLK          (CLK),
		.RSTn         (RSTn),
		//I-Memory Signals
		.I_MEM_CSN    (I_MEM_CSN),
		.I_MEM_DI     (I_MEM_DOUT),
		.I_MEM_ADDR   (I_MEM_ADDR),
		//D-Memory Signals
		.D_MEM_CSN    (D_MEM_CSN),
		.D_MEM_DI     (D_MEM_DOUT),
		.D_MEM_DOUT   (D_MEM_DI),
		.D_MEM_ADDR   (D_MEM_ADDR),
		.D_MEM_WEN    (D_MEM_WEN),
		//.D_MEM_BE     (D_MEM_BE), // Cache Lab
		//RegFile Signals
		.RF_WE        (RF_WE),
		.RF_RA1       (RF_RA1),
		.RF_RA2       (RF_RA2),
		.RF_WA1       (RF_WA),
		.RF_RD1       (RF_RD1),
		.RF_RD2       (RF_RD2),
		.RF_WD       (RF_WD),
		.HALT		(HALT),
		.NUM_INST (NUM_INST),
		.OUTPUT_PORT (OUTPUT_PORT)
	);

	//I-Memory
	SP_SRAM #(
		.ROMDATA ("testcase\\hex\\inst.hex"), //Initialize I-Memory
		.AWIDTH  (10),
		.SIZE    (1024)
	) i_mem1 (
		.CLK    (CLK),
		.CSN    (I_MEM_CSN),
		.DI		(32'bz),
		.DOUT   (I_MEM_DOUT),
		.ADDR   (I_MEM_ADDR[11:2]),
		.WEN    (1'b1),
		.BE     (4'b0000)
	);

	//D-Memory
	SP_DRAM #(
		.AWIDTH  (10), // Cache Lab
		.SIZE    (1024) // Cache Lab
	) d_mem1 (
		.CLK    (CLK),
		.CSN    (D_MEM_CSN),
		.DI     (D_MEM_DI),
		.DOUT   (D_MEM_DOUT),
		.ADDR   (D_MEM_ADDR),
		.WEN    (D_MEM_WEN)
		//.BE     (D_MEM_BE) // Cache Lab
	);

	//Reg File
	REG_FILE #(
		.DWIDTH (32),
		.MDEPTH (32),
		.AWIDTH (5)
	) reg_file1 (
		.RSTn	(RSTn),
		.CLK    (CLK),
		.WE     (RF_WE),
		.RA1    (RF_RA1),
		.RA2    (RF_RA2),
		.WA     (RF_WA),
		.RD1    (RF_RD1),
		.RD2    (RF_RD2),
		.WD     (RF_WD)
	);

	reg [31:0] cycle;
	reg verify;

	initial begin
		cycle <= 0;
		#1000000 $finish();
	end

	reg [`TESTID_SIZE*8-1:0] TestID[`NUM_TEST-1:0];
	reg [31:0] TestNumInst [`NUM_TEST-1:0];
	reg [31:0] TestAns[`NUM_TEST-1:0];
	reg TestPassed[`NUM_TEST-1:0];
	reg [31:0] i;

	initial begin

		TestID[0] <= "1";		TestNumInst[0] <= 16'h0001;		TestAns[0] <= 16'h0000;		TestPassed[0] <= 1'b0; //SW
		TestID[1] <= "2";		TestNumInst[1] <= 16'h0002;		TestAns[1] <= 16'h0000;		TestPassed[1] <= 1'b0; //LW
		TestID[2] <= "3";		TestNumInst[2] <= 16'h0003;		TestAns[2] <= 16'h0005;		TestPassed[2] <= 1'b0; //ADDI
		TestID[3] <= "4";		TestNumInst[3] <= 16'h0004;		TestAns[3] <= 16'h0000;		TestPassed[3] <= 1'b0; //SLTI
		TestID[4] <= "5";		TestNumInst[4] <= 16'h0005;		TestAns[4] <= 16'h0001;		TestPassed[4] <= 1'b0; //SLTI
		TestID[5] <= "6";		TestNumInst[5] <= 16'h0006;		TestAns[5] <= 16'h0000;		TestPassed[5] <= 1'b0; //SLTI
		TestID[6] <= "7";		TestNumInst[6] <= 16'h0007;		TestAns[6] <= 16'h0001;		TestPassed[6] <= 1'b0; //SLTIU
		TestID[7] <= "8";		TestNumInst[7] <= 16'h0008;		TestAns[7] <= 16'h0005;		TestPassed[7] <= 1'b0; //XORI
		TestID[8] <= "9";		TestNumInst[8] <= 16'h0009;		TestAns[8] <= 16'h0005;		TestPassed[8] <= 1'b0; //ANDI
		TestID[9] <= "10";		TestNumInst[9] <= 16'h000a;		TestAns[9] <= 16'h000f;		TestPassed[9] <= 1'b0; //ORI
		TestID[10] <= "11";		TestNumInst[10] <= 16'h00b;		TestAns[10] <= 16'h001e;		TestPassed[10] <= 1'b0; //SLLI
		TestID[11] <= "12";		TestNumInst[11] <= 16'h00c;	TestAns[11] <= 16'h000f;	TestPassed[11] <= 1'b0; //SRLI
		TestID[12] <= "13";		TestNumInst[12] <= 16'h00d;	TestAns[12] <= 16'h0002;	TestPassed[12] <= 1'b0; //SRAI
		TestID[13] <= "14";		TestNumInst[13] <= 16'h00e;	TestAns[13] <= 16'h0007;	TestPassed[13] <= 1'b0; //ADD
		TestID[14] <= "15";		TestNumInst[14] <= 16'h00f;	TestAns[14] <= 16'h0002;	TestPassed[14] <= 1'b0; //SUB
		TestID[15] <= "16";		TestNumInst[15] <= 16'h0010;	TestAns[15] <= 16'h0014;	TestPassed[15] <= 1'b0; //SLL
		TestID[16] <= "17";		TestNumInst[16] <= 16'h0011;	TestAns[16] <= 16'h0000;	TestPassed[16] <= 1'b0; //SLT
		TestID[17] <= "18";		TestNumInst[17] <= 16'h0012;	TestAns[17] <= 16'h0000;	TestPassed[17] <= 1'b0; //SLTU

		TestID[18] <= "19";		TestNumInst[18] <= 16'h0013;		TestAns[18] <= 16'h0000;		TestPassed[18] <= 1'b0; //XOR
		TestID[19] <= "20";		TestNumInst[19] <= 16'h0014;		TestAns[19] <= 16'h000a;		TestPassed[19] <= 1'b0; //SRL
		TestID[20] <= "21";		TestNumInst[20] <= 16'h0015;		TestAns[20] <= 16'h000a;		TestPassed[20] <= 1'b0; //SRA
		TestID[21] <= "22";		TestNumInst[21] <= 16'h0016;		TestAns[21] <= 16'h001e;		TestPassed[21] <= 1'b0; //OR
		TestID[22] <= "23";		TestNumInst[22] <= 16'h0017;		TestAns[22] <= 16'h0014;		TestPassed[22] <= 1'b0; //AND
	end

	always @ (posedge CLK) begin
		if (RSTn) begin
			cycle <= cycle + 1;

			for(i=0; i<`NUM_TEST; i=i+1) begin
				if ((NUM_INST==TestNumInst[i]) & (TestPassed[i]==0)) begin
					if (OUTPUT_PORT == TestAns[i]) begin
						TestPassed[i] <= 1'b1;
						$display("Test #%s has been passed", TestID[i]);
					end
					else begin
						TestPassed[i] <= 1'b0;
						$display("Test #%s has been failed!", TestID[i]);
						$display("output_port = 0x%0x (Ans : 0x%0x)", OUTPUT_PORT, TestAns[i]);
						$finish();
					end
				end
			end

			if (HALT == 1) begin
				$display("Finish: %d cycle", cycle);
				$display("Success.");
				$finish();
			end
		end
	end

endmodule
