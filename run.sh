rm a.out
iverilog -DBENCH -DBOARD_FREQ=10 tb_soc.v soc.v 
vvp a.out