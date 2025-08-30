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


    reg [31:0] PC = 0;
    reg [31:0] MEM [0:255];
    reg [31:0] instr;


    reg [31:0]  reg_bank[0:31];
    reg [31:0]  rs1;
    reg [31:0]  rs2;
    wire [31:0] write_back_data;
    wire        write_back_enable;

    assign write_back_data    = 0;
    assign write_back_enable  = 0;



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

// -- ROM content
    initial begin
        PC = 0;
        // add x0, x0, x0
        //                   rs2   rs1  add  rd   ALUREG
        instr = 32'b0000000_00000_00000_000_00000_0110011;
        // add x1, x0, x0
        //                    rs2   rs1  add  rd  ALUREG
        MEM[0] = 32'b0000000_00000_00000_000_00001_0110011;
        // addi x1, x1, 1
        //             imm         rs1  add  rd   ALUIMM
        MEM[1] = 32'b000000000001_00001_000_00001_0010011;
        // addi x1, x1, 1
        //             imm         rs1  add  rd   ALUIMM
        MEM[2] = 32'b000000000001_00001_000_00001_0010011;
        // addi x1, x1, 1
        //             imm         rs1  add  rd   ALUIMM
        MEM[3] = 32'b000000000001_00001_000_00001_0010011;
        // addi x1, x1, 1
        //             imm         rs1  add  rd   ALUIMM
        MEM[4] = 32'b000000000001_00001_000_00001_0010011;
        // lw x2,0(x1)
        //             imm         rs1   w   rd   LOAD
        MEM[5] = 32'b000000000000_00001_010_00010_0000011;
        // sw x2,0(x1)
        //             imm   rs2   rs1   w   imm  STORE
        MEM[6] = 32'b000000_00010_00001_010_00000_0100011;

        // ebreak
        //                                        SYSTEM
        MEM[7] = 32'b000000000001_00000_000_00000_1110011;
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


    always @(posedge clk ) begin

        if(!resetn) begin
            PC <= 0;
            state <= FETCHinstr;
            instr <=  32'b0000000_00000_00000_000_00000_0110011; // NOP
        end
        
        else begin
            if(write_back_enable && rdId != 0) begin
                reg_bank[rdId] <= write_back_data;
            end
        end

        case(state)
            FETCHinstr: begin
                instr <= MEM[PC];
                state <= FETCHreg;
            end

            FETCHreg: begin
                rs1 <= reg_bank[rs1Id];
                rs2 <= reg_bank[rs2Id];
                state <= EXECUTE;
            end

            EXECUTE: begin
                if(!isSYSTEM) begin
                    PC <= PC + 1;
                    state <= FETCHinstr;
                end
            end
        endcase

        if(isSYSTEM)
            $finish();


    end

    assign TXD = 0;
    assign LEDS = isSYSTEM ? 31 : (1 << state);



// -- verbosity
    always @(posedge clk) begin
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