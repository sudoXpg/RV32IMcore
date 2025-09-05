module SOC(
    input CLK,
    input RESET,
    output [4:0] LEDS,
    input RXD,
    output TXD
);

// -- clock-divider 
    wire clk;
    wire resetn;
    Gearbox #(.SLOW(15)) CW (CLK, RESET, clk, resetn);
// --


// -- memory controls
    wire [31:0]     mem_addr;
    wire [31:0]     mem_rdata;
    wire            mem_access;
    wire [31:0]     x1;
// --


// -- module init
    Memory          RAM     (clk, mem_addr, mem_rdata, mem_access);
    Processor       CPU     (clk, resetn, mem_addr, mem_rdata, mem_access, x1);
// --


    assign LEDS = x1[4:0];
    assign TXD = 1'b0;

endmodule






module Memory(
    input                       clk,
    input       [31:0]          mem_addr,
    output reg  [31:0]          mem_rdata,
    input                       mem_access
);

    reg [31:0] MEM [0:255];

    // -- ROM content
    `include "rv_asm.v"   
        integer L0_ = 8;
        initial begin
                ADD(x1,x0,x0);
                ADDI(x2,x0,32);
            Label(L0_); 
                ADDI(x1,x1,1); 
                BNE(x1, x2, LabelRef(L0_));
                EBREAK();
            endASM();
        end
    // -- 

// -- memory address update wrt PC
    always @(posedge clk ) begin
        if(mem_access) begin
            mem_rdata <= MEM[mem_addr[31:2]];
        end
    end
// --


endmodule





module Processor(
    input               clk,
    input               resetn,
    output [31:0]       mem_addr,
    input  [31:0]       mem_rdata,
    output              mem_access,
    output reg [31:0]   x1
);

    reg [31:0] PC = 0;
    reg [31:0] instr;


    // The 10 RISC-V instructions
    wire isALU_reg  =  (instr[6:0] == 7'b0110011);      // rd <- rs1 OP rs2   
    wire isALU_imm  =  (instr[6:0] == 7'b0010011);      // rd <- rs1 OP Iimm
    wire isBranch  =   (instr[6:0] == 7'b1100011);      // if(rs1 OP rs2) PC<-PC+Bimm
    wire isJALR    =   (instr[6:0] == 7'b1100111);      // rd <- PC+4; PC<-rs1+Iimm
    wire isJAL     =   (instr[6:0] == 7'b1101111);      // rd <- PC+4; PC<-PC+Jimm
    wire isAUIPC   =   (instr[6:0] == 7'b0010111);      // rd <- PC + Uimm
    wire isLUI     =   (instr[6:0] == 7'b0110111);      // rd <- Uimm   
    wire isLoad    =   (instr[6:0] == 7'b0000011);      // rd <- mem[rs1+Iimm]
    wire isStore   =   (instr[6:0] == 7'b0100011);      // mem[rs1+Simm] <- rs2
    wire isSYSTEM  =   (instr[6:0] == 7'b1110011);      // special

    // The 5 immediate formats
    wire [31:0] Uimm={    instr[31],   instr[30:12], {12{1'b0}}};
    wire [31:0] Iimm={{21{instr[31]}}, instr[30:20]};
    wire [31:0] Simm={{21{instr[31]}}, instr[30:25],instr[11:7]};
    wire [31:0] Bimm={{20{instr[31]}}, instr[7],instr[30:25],instr[11:8],1'b0};
    wire [31:0] Jimm={{12{instr[31]}}, instr[19:12],instr[20],instr[30:21],1'b0};

    // Source and destination registers
    wire [4:0] rs1Id = instr[19:15];
    wire [4:0] rs2Id = instr[24:20];
    wire [4:0] rdId  = instr[11:7];

    // function codes
    wire [2:0] funct3 = instr[14:12];
    wire [6:0] funct7 = instr[31:25];

    // registers
    reg [31:0]  reg_bank[0:31];
    reg [31:0]  rs1;
    reg [31:0]  rs2;
    wire [31:0] write_back_data;
    wire        write_back_enable;

// -- cleaning registers
    integer i;
    initial begin
        for(i=0;i<32;i++) begin
            reg_bank[i] = 0;
        end
    end
// --


// -- ALU registers/nets
   wire [31:0] aluIn1 = rs1;
   wire [31:0] aluIn2 = isALU_reg ? rs2 : Iimm;
   reg  [31:0] aluOut;
   wire [4:0]  shamt =  isALU_reg ? rs2[4:0] : instr[24:20]; // shift amount
// --




// -- ALU
    always @(*) begin
        case(funct3)
	        3'b000: aluOut = (funct7[5] & instr[5]) ? (aluIn1 - aluIn2) : (aluIn1 + aluIn2);
	        3'b001: aluOut = aluIn1 << shamt;
	        3'b010: aluOut = ($signed(aluIn1) < $signed(aluIn2));
	        3'b011: aluOut = (aluIn1 < aluIn2);
	        3'b100: aluOut = (aluIn1 ^ aluIn2);
	        3'b101: aluOut = funct7[5]? ($signed(aluIn1) >>> shamt) : ($signed(aluIn1) >> shamt); 
	        3'b110: aluOut = (aluIn1 | aluIn2);
	        3'b111: aluOut = (aluIn1 & aluIn2);	
        endcase
    end
// --


// -- Branching
   reg takeBranch;
   
   always @(*) begin
        case(funct3)
	        3'b000: takeBranch = (rs1 == rs2);
	        3'b001: takeBranch = (rs1 != rs2);
	        3'b100: takeBranch = ($signed(rs1) < $signed(rs2));
	        3'b101: takeBranch = ($signed(rs1) >= $signed(rs2));
	        3'b110: takeBranch = (rs1 < rs2);
	        3'b111: takeBranch = (rs1 >= rs2);
	        default: takeBranch = 1'b0;
        endcase
   end
// --


// -- state machine parameters
    localparam FETCHinstr  = 0;
    localparam WAITinstr   = 1;
    localparam FETCHreg    = 2;
    localparam EXECUTE     = 3;

    reg [2:0] state = FETCHinstr;
// --

// -- Instruction result
    assign write_back_data    = (isJAL || isJALR)? (PC+4) : (isLUI)? Uimm : (isAUIPC)? (Uimm + PC) : aluOut;
    assign write_back_enable  = (state == EXECUTE && 
                                (isALU_reg || isALU_imm || isJAL || isJALR || isLUI || isAUIPC));

    wire [31:0] next_PC = (isBranch && takeBranch) ? PC + Bimm : isJAL ? PC+Iimm : isJALR ? rs1 + Iimm : PC + 4;
//--

// -- Main processor loop with FSM
    always @(posedge clk ) begin

        if(!resetn) begin
            PC <= 0;
            state <= FETCHinstr;
            instr <=  32'b0000000_00000_00000_000_00000_0110011; // NOP
        end
        
        else begin
            if(write_back_enable && rdId != 0) begin
                reg_bank[rdId] <= write_back_data;
                // For displaying what happens.
	            if(rdId == 1) begin
	            x1 <= write_back_data;
	    end
            end
        end

        case(state)
            FETCHinstr: begin
                state <= WAITinstr;
            end

            WAITinstr: begin
                instr <= mem_rdata;
                state <= FETCHreg;
            end
            
            FETCHreg: begin
                rs1 <= reg_bank[rs1Id];
                rs2 <= reg_bank[rs2Id];
                state <= EXECUTE;
            end

            EXECUTE: begin
                if(!isSYSTEM) begin
                    PC <= next_PC;
                    state <= FETCHinstr;
                end
            end
        endcase

        if(isSYSTEM)
            $finish();
    end
// --


// -- update RAM address to be read
    assign mem_addr = PC;
    assign mem_access = (state == FETCHinstr);
// --


// -- verbosity
    always @(posedge clk) begin
        $display("x%0d <= %b",rdId,write_back_data);
        if(state == FETCHreg) begin
	        case (1'b1)
	        isALU_reg: $display(
		    	      "ALUreg rd=%d rs1=%d rs2=%d funct3=%b",
		    	      rdId, rs1Id, rs2Id, funct3
		    	      );
	        isALU_imm: $display(
		    	      "ALUimm rd=%d rs1=%d imm=%0d funct3=%b",
		    	      rdId, rs1Id, Iimm, funct3
		    	      );
	        isBranch: $display("BRANCH");
	        isJAL:    $display("JAL");
	        isJALR:   $display("JALR");
	        isAUIPC:  $display("AUIPC");
	        isLUI:    $display("LUI");	
	        isLoad:   $display("LOAD");
	        isStore:  $display("STORE");
	        isSYSTEM: $display("SYSTEM");
	        endcase 
    
            if(isSYSTEM) begin
	            $finish();
	        end
        end 
   end
// --


endmodule


module Gearbox #(parameter SLOW = 21)(
    input CLK,
    input RESET,
    output clk,
    output resetn
);


    reg [SLOW:0] slow_clk = 0;
    always @(posedge CLK) begin
        slow_clk <= slow_clk + 1;
    end
    assign clk = slow_clk[SLOW];
    assign resetn = ~RESET;
endmodule
