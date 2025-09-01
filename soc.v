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
    Clockworks #(.SLOW(15)) CW (CLK, RESET, clk, resetn);
// --

    reg [4:0] leds = 0;

    reg [31:0] PC = 0;
    reg [31:0] MEM [0:255];
    reg [31:0] instr;


    reg [31:0]  reg_bank[0:31];
    reg [31:0]  rs1;
    reg [31:0]  rs2;
    wire [31:0] write_back_data;
    wire        write_back_enable;

    assign write_back_data    = (isJAL || isJALR)? (PC+4) : aluOut;
    assign write_back_enable  = (state == EXECUTE && 
                                (isALUreg || isALUimm || isJAL || isJALR));

    wire [31:0] next_PC = isJAL ? PC+Iimm : isJALR ? rs1 + Iimm : PC + 4;



// -- state machine parameters
    localparam FETCHinstr  = 3'b000;
    localparam FETCHreg    = 3'b001;
    localparam EXECUTE     = 3'b010;

    reg [2:0] state = FETCHinstr;
// --

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
   wire [31:0] aluIn2 = isALUreg ? rs2 : Iimm;
   reg  [31:0] aluOut;
   wire [4:0]  shamt =  isALUreg ? rs2[4:0] : instr[24:20]; // shift amount
// --

// -- ROM content
`include "rv_asm.v"
   
    integer L0_=4;
    initial begin
	    ADD(x1,x0,x0);
        Label(L0_);
	    ADDI(x1,x1,1);
	    JAL(x0,LabelRef(L0_));
	    EBREAK();
	    endASM();
    end
// -- 


    // The 10 RISC-V instructions
    wire isALUreg  =  (instr[6:0] == 7'b0110011); // rd <- rs1 OP rs2   
    wire isALUimm  =  (instr[6:0] == 7'b0010011); // rd <- rs1 OP Iimm
    wire isBranch  =  (instr[6:0] == 7'b1100011); // if(rs1 OP rs2) PC<-PC+Bimm
    wire isJALR    =  (instr[6:0] == 7'b1100111); // rd <- PC+4; PC<-rs1+Iimm
    wire isJAL     =  (instr[6:0] == 7'b1101111); // rd <- PC+4; PC<-PC+Jimm
    wire isAUIPC   =  (instr[6:0] == 7'b0010111); // rd <- PC + Uimm
    wire isLUI     =  (instr[6:0] == 7'b0110111); // rd <- Uimm   
    wire isLoad    =  (instr[6:0] == 7'b0000011); // rd <- mem[rs1+Iimm]
    wire isStore   =  (instr[6:0] == 7'b0100011); // mem[rs1+Simm] <- rs2
    wire isSYSTEM  =  (instr[6:0] == 7'b1110011); // special

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
	            leds <= write_back_data;
	    end
            end
        end

        case(state)
            FETCHinstr: begin
                instr <= MEM[PC[31:2]];
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

    assign LEDS = leds;
    assign TXD = 0;


// -- verbosity
    always @(posedge clk) begin
        $display("x%0d <= %b",rdId,write_back_data);
        if(state == FETCHreg) begin
	        case (1'b1)
	        isALUreg: $display(
		    	      "ALUreg rd=%d rs1=%d rs2=%d funct3=%b",
		    	      rdId, rs1Id, rs2Id, funct3
		    	      );
	        isALUimm: $display(
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





module Clockworks #(parameter SLOW = 21)(
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