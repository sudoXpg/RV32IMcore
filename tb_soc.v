module bench();
   reg CLK;
   wire RESET = 0; 
   wire [4:0] LEDS;
   reg  RXD = 1'b0;
   wire TXD;

    SOC dut(
        .CLK(CLK),
        .RESET(RESET),
        .LEDS(LEDS),
        .RXD(RXD),
        .TXD(TXD)
    );

    reg[4:0] prev_LEDS = 0;

    always
	    #1 CLK = ~CLK;

    initial begin
        CLK = 0;
        $monitor("LEDS = %x", LEDS);
    end
endmodule