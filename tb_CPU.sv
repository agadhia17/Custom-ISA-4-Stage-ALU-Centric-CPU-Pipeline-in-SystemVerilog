
//------------------------------------------------------------------------------
// Testbench: tb_CPU_Pipeline
// Purpose  : Self-checking verification for CPU_Pipeline using directed program
//            in instr_mem.txt and register-file probes (Icarus VPI memory limit).
//------------------------------------------------------------------------------
`timescale 1ns/1ns
module tb_CPU_Pipeline;
  reg CLK = 0;
  reg RST;
  
  always #5 CLK = ~CLK; //10 ns clock cycle period
  
  localparam memDepth = 16;
  localparam instrCodeLength = 16;
  localparam instrFile = "instr_mem.txt";
  
  CPU_Pipeline #(.ROMDepth(memDepth), .ROMFile(instrFile)) 
  DUT		(.CLK(CLK), .RST(RST)); //instantion of top module
  
  //register probes for each memory address for waveform dumping(constraint of Icarus)
  wire [15:0] rp00 = DUT.stage2.MEM.RAM[0];
  wire [15:0] rp01 = DUT.stage2.MEM.RAM[1];
  wire [15:0] rp02 = DUT.stage2.MEM.RAM[2];
  wire [15:0] rp03 = DUT.stage2.MEM.RAM[3];
  wire [15:0] rp04 = DUT.stage2.MEM.RAM[4];
  wire [15:0] rp05 = DUT.stage2.MEM.RAM[5];
  wire [15:0] rp06 = DUT.stage2.MEM.RAM[6];
  wire [15:0] rp07 = DUT.stage2.MEM.RAM[7];
  wire [15:0] rp08 = DUT.stage2.MEM.RAM[8];
  wire [15:0] rp09 = DUT.stage2.MEM.RAM[9];
  wire [15:0] rp10 = DUT.stage2.MEM.RAM[10];
  wire [15:0] rp11 = DUT.stage2.MEM.RAM[11];
  wire [15:0] rp12 = DUT.stage2.MEM.RAM[12];
  wire [15:0] rp13 = DUT.stage2.MEM.RAM[13];
  wire [15:0] rp14 = DUT.stage2.MEM.RAM[14];
  wire [15:0] rp15 = DUT.stage2.MEM.RAM[15];

  //task for checking registers for expected values
  task automatic check_reg(input int r, input [15:0] exp);
    reg [15:0] rec;
    begin
      rec = DUT.stage2.MEM.RAM[r]; //register file output based on requested address
      if(rec !== exp) begin
        $display("FAIL; at regFile address R%0d, expected: 0x%04h, recieved: 0x%04h @ t = %0t", r, exp, rec, $time);
        $fatal;
      end else begin
        $display("PASS; regFile[R%0d] is equal to 0x%04h at time = %0t", r, rec, $time);
      end
    end
  endtask
  
  //waits a certain amount of clock cycles to let instructions flow through pipeline
  task automatic wait_cycles(input int n);
    integer k;
    begin
      for(k=0; k<n; k=k+1) @(posedge CLK);
    end
  endtask
  
  initial begin
    $dumpfile("waves.vcd");
    $dumpvars(0, tb_CPU_Pipeline);
    
    //reset
    RST = 1;
    wait_cycles(2);
    RST = 0;
    
    //let instructions run through pipeline
    wait_cycles(memDepth + 8);
    
    //check final register values (expected values based on program)
    check_reg(0, 16'h0000); //expected value r0 after all instr. went thru pipeline
    check_reg(1, 16'h0005); 
    check_reg(2, 16'h000A); 
    check_reg(3, 16'h0003); 
    check_reg(4, 16'h000C); 
    check_reg(5, 16'h000F); 
    check_reg(6, 16'h0005); 
    check_reg(7, 16'h0002); 
    check_reg(8, 16'h0007); 
    check_reg(9, 16'h0003); 
    check_reg(10, 16'h000C); 
    check_reg(11, 16'h0006); 
    check_reg(12, 16'h000F); 
    check_reg(13, 16'h0005); 
    check_reg(14, 16'h000C); 
    check_reg(15, 16'h000D);
    
    $display("ALL CHECKS PASSED.");
    $finish;
  end
endmodule

