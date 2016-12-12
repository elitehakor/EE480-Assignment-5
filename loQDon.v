// basic sizes of things
`define WORD	 [15:0]
`define DOUBLE [31:0]
`define QUAD   [63:0]
`define high_instruction [31:16]
`define low_instruction [15:0]
`define high_instruction_64 [63:48]
`define low_instruction_64  [47:32]
`define Register [31:0]
`define address [3:0]
`define Opcode [15:12]
`define Dest	 [11:8]
`define Arg1	 [7:4]
`define Arg2   [3:0]
`define parallel_val  `Arg1
`define parallel_addr	`Arg2
`define immed 	[7:0]
`define STATE	[5:0]
`define REGSIZE [15:0] 
//`define MEMSIZE [65535:0]
//`define HALF_MEMSIZE [32768:0]
`define MEMSIZE [0:65535]
`define HALF_MEMSIZE [0:32768]
`define BUFFSIZE [0:9]

`define High_Opcode [31:28]
`define High_Dest	  [27:24]
`define High_Arg1   [23:20]
`define High_Arg2	  [19:16]

`define High_parallel_val  `High_Arg1
`define High_parallel_addr `High_Arg2

`define Opcode_64 [47:44]
`define Dest_64	  [43:40]
`define Arg1_64	  [39:36]
`define Arg2_64   [35:32]
`define High_Opcode_64 [63:60]
`define High_Dest_64	 [59:56]
`define High_Arg1_64   [55:52]
`define High_Arg2_64	 [51:48]
`define High_parallel_val_64  `High_Arg1_64
`define High_parallel_addr_64	`High_Arg2_64
`define parallel_val_64  `Arg1
`define parallel_addr_64	`Arg2

// opcode values, also state numbers
`define OPand		  4'b0000
`define OPor  	  4'b0001
`define OPxor 	  4'b0010
`define OPadd		  4'b0100
`define OPaddv 	  4'b0101
`define OPshift	  4'b0111
`define OPpack 	  4'b1000
`define OPunpack  4'b1001
`define OPli		  4'b1010
`define OPmorei 	4'b1011
`define OPldanne	4'b1110
`define OPstjzsys	4'b1111


// state numbers only
`define OPld		6'b010000
`define OPany		6'b010010
`define OPanyv	6'b010011
`define OPneg		6'b010100
`define OPnegv	6'b010101
`define OPst		6'b110000
`define OPjz		6'b110010
`define OPjnz		6'b110011
`define OPsys		6'b111111
`define Start		5'b11111
`define Start1	5'b11110

// Arg2 values for ld, any, anyv, neg, and negv
`define Arg2ld		4'b0000
`define Arg2any		4'b0010
`define Arg2anyv	4'b0011
`define Arg2neg		4'b0100
`define Arg2negv	4'b0101

// Dest values for st, jz, jnz, and sys
`define Destst		4'b0000
`define Destjz		4'b0010
`define Destjnz		4'b0011
`define Destsys		4'b1111

module instruction_block(clk, reset, jump_taken, jump_addr, pc, ir, parallel_store_en, parallel_addr_reg, parallel_val_reg );
input clk, reset, jump_taken;
input `Register jump_addr;
output reg `WORD ir;
output reg `WORD pc;
output reg jump_flag, jump_type;
output reg parallel_store_en;
output reg `address parallel_addr_reg, parallel_val_reg;
integer idx;

reg high_or_low_instruction;
reg parallel_store_en_buf;
reg `WORD oo_instruction_buffer;
reg `address  oo_parallel_val_reg, oo_parallel_addr_reg; 
wire `DOUBLE ir32;
wire `QUAD   ir64;

reg `WORD   mainmem_16 `MEMSIZE;
reg `DOUBLE mainmem `HALF_MEMSIZE;
reg `DOUBLE instruction_buffer `BUFFSIZE;
reg [1:0] oo_execuction;


always @(reset) begin
  pc = 0;
  high_or_low_instruction = 0;
  $readmemh("mainmem_16.vmem",mainmem_16 );
  $readmemh("mainmem.vmem",   mainmem    );

  parallel_store_en = 0;
  parallel_store_en_buf = 0;
  parallel_addr_reg = 4'b0000;
  parallel_val_reg  = 4'b0000;
  oo_execuction     = 2'b00;
  oo_instruction_buffer = 16'h0000;
  oo_parallel_val_reg   = 4'h0;
  oo_parallel_addr_reg  = 4'h0;

  for( idx=0; idx < 65536; idx = idx + 2 ) begin
    mainmem[idx/2] = { mainmem_16[idx], mainmem_16[idx + 1] };
  end

  ir = mainmem[0] `high_instruction;
end

assign ir32 = (jump_taken) ? mainmen[jump_addr/2] : mainmem[pc/2];
assign ir64 = {mainmem[pc/2],mainmem[pc/2+1]};

always @(posedge clk) begin
  $display("ir64=%x 64h=%x 64l=%x 32g=%x 32l=%x", ir64, ir64 `High_Opcode_64, ir64 `Opcode_64, ir64 `High_Opcode, ir64 `Opcode);

  if( oo_execuction == 2 ) begin
    oo_execuction <= 2'h1;
    ir <= oo_instruction_buffer;
    high_or_low_instruction <= 1'b1;
    parallel_val_reg  <= oo_parallel_val_reg;
    parallel_addr_reg <= oo_parallel_addr_reg;
    oo_instruction_buffer <= ir64 `low_instruction_64;
    pc <= pc + 2;
  end else if ( oo_execuction == 0 && !(jump_taken) && pc%2 == 0 &&
    ( 
    ir32 `High_Opcode == `OPand     || 
    ir32 `High_Opcode == `OPor      ||
    ir32 `High_Opcode == `OPxor     ||
    ir32 `High_Opcode == `OPadd     ||
    ir32 `High_Opcode == `OPaddv    ||
    ir32 `High_Opcode == `OPshift
  ) &&
    ir32 `Opcode == `OPstjzsys &&
    ir32 `Dest   == `Destst    &&
    ir32 `parallel_addr != ir32 `High_Dest &&
    ir32 `parallel_addr != ir32 `High_Arg2 &&
    ir32 `parallel_addr != ir32 `High_Arg1 
  ) begin
    parallel_store_en <= 1'b1;
    parallel_val_reg  <= ir32 `parallel_val;
    parallel_addr_reg <= ir32 `parallel_addr;

    // NOTE: We don't change high_or_low instructions here
    ir <= ir32 `high_instruction;
    high_or_low_instruction <= 1'b1;
    pc <= pc + 2;
    //$display("HERE------------------pc=%x",pc);
  end else if ( oo_execuction == 0 && !(jump_taken) && pc%2 == 0 &&
      ( 
        ir64 `High_Opcode_64 == `OPand     || 
        ir64 `High_Opcode_64 == `OPor      ||
        ir64 `High_Opcode_64 == `OPxor     ||
        ir64 `High_Opcode_64 == `OPadd     ||
        ir64 `High_Opcode_64 == `OPaddv    ||
        ir64 `High_Opcode_64 == `OPshift
      ) &&
      ( 
        ir64 `Opcode_64 == `OPand     || 
        ir64 `Opcode_64 == `OPor      ||
        ir64 `Opcode_64 == `OPxor     ||
        ir64 `Opcode_64 == `OPadd     ||
        ir64 `Opcode_64 == `OPaddv    ||
        ir64 `Opcode_64 == `OPshift
      ) &&
      (
        ir64 `High_Opcode == `OPstjzsys &&
        ir64 `High_Dest   == `Destst    &&
        ir64 `High_parallel_addr != ir64 `High_Dest_64 &&
        ir64 `High_parallel_addr != ir64 `High_Arg2_64 &&
        ir64 `High_parallel_addr != ir64 `High_Arg1_64 
      ) &&    
      ( 
        ir64 `Opcode == `OPstjzsys &&
        ir64 `Dest   == `Destst    &&
        ir64 `parallel_addr != ir64 `Dest_64 &&
        ir64 `parallel_addr != ir64 `Arg2_64 &&
        ir64 `parallel_addr != ir64 `Arg1_64 
      )
    ) begin
      oo_execuction <= 2'h2;
      parallel_store_en <= 1'b1;
      ir <= ir64 `high_instruction_64;
      high_or_low_instruction <= 1'b1;
      parallel_val_reg  <= ir64 `High_parallel_val;
      parallel_addr_reg <= ir64 `High_parallel_addr;
      oo_instruction_buffer <= ir64 `low_instruction_64;
      oo_parallel_val_reg   <= ir64 `parallel_val;
      oo_parallel_addr_reg  <= ir64 `parallel_addr;
      pc <= pc + 2;

      $display("------------OUT OF ORDER--------");
    end else begin
    parallel_store_en <= 1'b0;
    ir <= ( (jump_taken) ? ( (jump_addr % 2 != 0)      ? ir32 `low_instruction  : ir32 `high_instruction ) :
                           ( (high_or_low_instruction) ? ir32 `high_instruction : ir32 `low_instruction  ) );
    high_or_low_instruction <= ( (jump_taken) ? ( (jump_addr % 2 == 0) ? 1'b1 : 1'b0 ) : ~high_or_low_instruction );
    parallel_val_reg  <= ir32 `parallel_val;
    parallel_addr_reg <= ir32 `parallel_addr;
    pc <= ( (jump_taken) ? jump_addr : pc + 1 );

  end

  //$display("%x %x %d %d %d %d %d %d", ir32,
  //  ir32 `High_Opcode,
  //  ir32 `High_Opcode == `OPxor,
  //  ir32 `Opcode == `OPstjzsys,
  //  ir32 `Dest   == `Destst,
  //  ir32 `parallel_addr != ir32 `High_Dest,
  //  ir32 `parallel_addr != ir32 `High_Arg2,
  //  ir32 `parallel_addr != ir32 `High_Arg1 );

  //$display("%x %x %x %x %x", jump_addr, jump_taken, high_or_low_instruction, ir, pc);
  //pc <= ( (jump_taken) ? jump_addr : pc + ((
  //                            ( 
  //                              ir32 `High_Opcode == `OPand     || 
  //                              ir32 `High_Opcode == `OPor      ||
  //                              ir32 `High_Opcode == `OPxor     ||
  //                              ir32 `High_Opcode == `OPadd     ||
  //                              ir32 `High_Opcode == `OPaddv    ||
  //                              ir32 `High_Opcode == `OPshift
  //                            ) &&
  //                            ir32 `Opcode == `OPstjzsys &&
  //                            ir32 `Dest   == `Destst    &&
  //                            ir32 `parallel_addr != ir32 `High_Dest &&
  //                            ir32 `parallel_addr != ir32 `High_Arg2 &&
  //                            ir32 `parallel_addr != ir32 `High_Arg1 
  //                          ) ? 2:1));

  //$display("ir32=%x ir=%x hol=%x ir[h]=%x, ir[l]=%x", ir32, ir, high_or_low_instruction, ir32 `high_instruction, ir32 `low_instruction);
  //$display( "IST: pc=%x ir=%x pe=%x pa=%x pv=%x jt=%x hol=%x", pc, ir, parallel_store_en, parallel_addr_reg, parallel_val_reg, jump_taken, high_or_low_instruction );

end

endmodule


module regfile_block(clk, reset, write_en, pc, ir_in, ALU_in, ALUdest, jump_taken, ALU_source1, ALU_source2, ir_in2, D, P,
  u0, u1, u2, u3, u4, u5, u6, u7, u8, u9, parallel_store_mon, parallel_addr_mon, parallel_val_mon, parallel_store_en_in, parallel_addr_reg_in,
  parallel_val_reg_in, parallel_store_en_out, parallel_addr_out, parallel_val_out );

input clk, reset, write_en, jump_taken;
input `WORD pc, ir_in;
input `Register ALU_in;
input `address ALUdest;
input parallel_store_en_in;
input `address parallel_addr_reg_in, parallel_val_reg_in;
output reg parallel_store_mon;
output reg `WORD parallel_addr_mon, parallel_val_mon;
output reg parallel_store_en_out;
output reg `WORD parallel_addr_out, parallel_val_out;
output reg `WORD ir_in2;
output reg `Register ALU_source1, ALU_source2, u0, u1, u2, u3, u4, u5, u6, u7, u8, u9;
output reg `address D, P;

reg `Register regfile `REGSIZE;
reg parallel_store_en_buf;
reg `WORD parallel_addr_reg_buf, parallel_val_buf;


always @(reset) begin
  ALU_source1 = 32'h00000000;
  ALU_source2 = 32'h00000000;
  parallel_store_en_out = 1'b0;
  parallel_addr_out     = 32'h00000000;
  parallel_val_out      = 32'h00000000;
  parallel_store_en_buf = 1'b0;
  parallel_addr_reg_buf     = 32'h00000000;
  parallel_val_buf      = 32'h00000000;
  $readmemh("regfile.vmem",regfile);
end


always @(posedge clk) begin

  /*
  Note code that looks like ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] ) is looking to see 
  if what the ALU just sent back to the register file is one of the arguments the next instruction to be sent to the ALU
    needs. Without this check the register file will send the wrong information to the ALU for the next instruction for WAR.
  */

 u0 <= regfile[6];  //register monitors
 u1 <= regfile[7];
 u2 <= regfile[8];
 u3 <= regfile[9];
 u4 <= regfile[10];
 u5 <= regfile[11];
 u6 <= regfile[12];
 u7 <= regfile[13];
 u8 <= regfile[14];
 u9 <= regfile[15];

 parallel_store_mon <= parallel_store_en_in;
 parallel_val_mon   <= regfile[parallel_val_reg_in ];
 parallel_addr_mon  <= regfile[parallel_addr_reg_in];

 parallel_addr_out <= regfile[parallel_val_reg_in ];
 parallel_val_out  <= regfile[parallel_addr_reg_in];

 //parallel_store_en_buf <= parallel_store_en_out;
 //parallel_addr_reg_buf <= parallel_addr_reg_in;
 //parallel_addr_buf     <= parallel_addr_out;    

 ir_in2 <= ir_in;

 if (write_en)
   regfile[ALUdest] <= ALU_in;

 if (jump_taken == 0) begin
   parallel_store_en_out <= parallel_store_en_in;
   //$display( "ir=%x ALU_source1=%x ALUdest=%x ALU_in=%x reg[]=%x", ir_in, ALU_source1, ALUdest, ALU_in, regfile[ir_in `Arg1]);
   case (ir_in `Opcode)
     `OPldanne:
       case (ir_in `Arg2)	      // use Arg2 as extended opcode
         `Arg2ld: begin  D <= ir_in `Dest; ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] ); end // ld
         `Arg2any: begin D <= ir_in `Dest; ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] ); end // any
         `Arg2anyv: begin D <= ir_in `Dest; ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] ); end // anyv
         `Arg2neg: begin D <= ir_in `Dest; ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] ); end // neg
         default: begin D <= ir_in `Dest; ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] );  end // negv
       endcase
       `OPstjzsys:
         case (ir_in `Dest)	      // use Dest as extended opcode
           `Destst: begin ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] ); 
           ALU_source2 <= ( (write_en && (ALUdest == ir_in `Arg2)) ? ALU_in : regfile[ir_in `Arg2] ); end // st
           `Destjz: begin ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] ); 
           ALU_source2 <= ( (write_en && (ALUdest == ir_in `Arg2)) ? ALU_in : regfile[ir_in `Arg2] ); end // jz
           `Destjnz: begin ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] ); 
           ALU_source2 <= ( (write_en && (ALUdest == ir_in `Arg2)) ? ALU_in : regfile[ir_in `Arg2] ); end // jnz
           //default: begin halt <= 1; end // sys
         endcase

         `OPand: begin D <= ir_in `Dest;   
         ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] );
         ALU_source2 <= ( (write_en && (ALUdest == ir_in `Arg2)) ? ALU_in : regfile[ir_in `Arg2] ); end
         `OPor: begin D <= ir_in `Dest;   
         ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] );
         ALU_source2 <= ( (write_en && (ALUdest == ir_in `Arg2)) ? ALU_in : regfile[ir_in `Arg2] ); end
         `OPxor: begin D <= ir_in `Dest;   
         ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] );
         ALU_source2 <= ( (write_en && (ALUdest == ir_in `Arg2)) ? ALU_in : regfile[ir_in `Arg2] ); end
         `OPadd: begin D <= ir_in `Dest;   
         ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] );
         ALU_source2 <= ( (write_en && (ALUdest == ir_in `Arg2)) ? ALU_in : regfile[ir_in `Arg2] ); end
         `OPaddv: begin D <= ir_in `Dest;   
         ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] );
         ALU_source2 <= ( (write_en && (ALUdest == ir_in `Arg2)) ? ALU_in : regfile[ir_in `Arg2] ); end
         `OPshift: begin D <= ir_in `Dest;  
         ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] );
         ALU_source2 <= ( (write_en && (ALUdest == ir_in `Arg2)) ? ALU_in : regfile[ir_in `Arg2] ); end

         `OPpack: begin D <= ir_in `Dest; P <= ir_in `Arg2; 
         ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] );
         ALU_source2 <= ( (write_en && (ALUdest == ir_in `Dest)) ? ALU_in : regfile[ir_in `Dest] ); end 
         `OPunpack: begin D <= ir_in `Dest; P <= ir_in `Arg2; 
         ALU_source1 <= ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] );
         ALU_source2 <= ( (write_en && (ALUdest == ir_in `Dest)) ? ALU_in : regfile[ir_in `Dest] ); end 

         `OPli: begin D <= ir_in `Dest; ALU_source1 `immed <= ir_in `immed; end
         `OPmorei: begin D <= ir_in `Dest; ALU_source1 `immed <= ir_in `immed; 
         ALU_source2 <= ( (write_en && (ALUdest == ir_in `Dest)) ? ALU_in : regfile[ir_in `Dest] ); end

         // default: halt <= 1;
       endcase	

     end else begin

       ALU_source1 <= regfile[0]; ALU_source2 <= regfile[0]; ir_in2 <= regfile[0][15:0];
       D <= ir_in `Dest;

     end
     //$display( "REG: pc=%x ir_out=%x pe=%x/%x pa=%x/%x pv=%x/%x jt=%x", pc, ir_in, parallel_store_en_in, parallel_store_en_out, parallel_addr_reg_in, parallel_addr_out, parallel_val_reg_in, parallel_val_out, jump_taken );
   end


   endmodule


   module ALU_block(clk, reset, ALU_source1, ALU_source2, ir_in2, D, P, halt, write_en, ALUdest, ALUout, jump_taken, jump_addr, parallel_store_en, parallel_addr, parallel_val );
   input clk, reset;
   input `Register ALU_source1, ALU_source2;
   input `WORD ir_in2;
   input `address  D, P;
   input parallel_store_en;
   input `WORD parallel_addr, parallel_val;
   output reg halt, write_en, jump_taken;
   output reg `Register ALUout, jump_addr;
   output reg `address ALUdest;

   reg `Register datamem `MEMSIZE;
   reg `Register ALUout_buff;
   reg `address ALUdest_buff;
   reg parallel_store_en_buf;
   reg `WORD parallel_addr_buf, parallel_val_buf;

   always @(reset) begin
     halt         = 1'b0;
     write_en     = 1'b0;
     jump_taken   = 1'b0;
     jump_addr    = 32'h00000000;
     ALUout_buff  = 32'h00000000;
     ALUout       = 32'h00000000;
     ALUdest_buff = 4'h0;
     ALUdest      = 4'h0;
     parallel_store_en_buf = 1'b0;
     parallel_addr_buf     = 32'h00000000;
     parallel_val_buf      = 32'h00000000;
     $readmemh("datamem.vmem",datamem);
   end


   always @(posedge clk) begin
     //$display("ir=%x buffered mem[%x]=%x to reg %x en=%x ALU_source1=%x ALUout_buff=%x we=%x", ir_in2, parallel_addr_buf,parallel_val_buf, D, parallel_store_en_buf, ALU_source1, ALUout_buff, write_en );
     /* Note code that looks like ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] ) 
     is checking to see if what the ALU just sent back to the register file is one of the arguments to the instruction
       it is evaluating. If the ALU just sent something to the register file to be stored then it could not possibly have
     sent that information back to the ALU since it hadn't been computed yet. So to get instructions correct the ALU
     must remember the result and register destination of its last instruction.
       */

      if (ir_in2 `Opcode == `OPstjzsys)
        write_en <= 0;
      else
        write_en <= 1;


      ALUdest_buff <= D;
      ALUdest <= D;

      if (jump_taken == 0) begin

        if (parallel_store_en == 1'b1) begin
          //$display("Setting %x to val %x", parallel_addr, parallel_val);
          datamem[parallel_addr] <= parallel_val;
        end
        case (ir_in2 `Opcode)
          `OPldanne:
            case (ir_in2 `Arg2)	      // use Arg2 as extended opcode
              `Arg2ld: begin  //ld
              if( parallel_store_en_buf == 1'b1 && parallel_addr_buf == ALU_source1[3:0] ) begin
                //$display("Loading buffered mem[%x]=%x to reg %x", parallel_addr_buf,parallel_val_buf, D );
                  ALUout <= parallel_val_buf;
                  ALUout_buff <= parallel_val_buf;
                end else begin
                //$display("Loading mem[%x]=%x to reg %x", ALU_source1,datamem[ALU_source1], D );
                  ALUout <= datamem[( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 )];
                  ALUout_buff <= datamem[( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 )]; 
                end
              end 
              `Arg2any: begin   //any
              ALUout <= |( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 ); 
              ALUout_buff <= |( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 );
              end 
            `Arg2anyv: begin   //anyv
              ALUout[7:0] <= |( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] );
              ALUout[15:8] <= |( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[15:8] : ALU_source1[15:8] );
              ALUout[23:16] <= |( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[23:16] : ALU_source1[23:16] );
              ALUout[31:24] <= |( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[31:24] : ALU_source1[31:24] );


              ALUout_buff[7:0] <= |( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] );
              ALUout_buff[15:8] <= |( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[15:8] : ALU_source1[15:8] );
              ALUout_buff[23:16] <= |( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[23:16] : ALU_source1[23:16] );
              ALUout_buff[31:24] <= |( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[31:24] : ALU_source1[31:24] );
            end 
            `Arg2neg: begin   //neg
            ALUout <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? -ALUout_buff : -ALU_source1 ); 
            ALUout_buff <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? -ALUout_buff : -ALU_source1 );
          end 
          default: begin  //negv
          ALUout[7:0] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? -ALUout_buff[7:0] : -ALU_source1[7:0] );
          ALUout[15:8] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? -ALUout_buff[15:8] : -ALU_source1[15:8] );
          ALUout[23:16] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? -ALUout_buff[23:16] : -ALU_source1[23:16] );
          ALUout[31:24] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? -ALUout_buff[31:24] : -ALU_source1[31:24] );

          ALUout_buff[7:0] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? -ALUout_buff[7:0] : -ALU_source1[7:0] );
          ALUout_buff[15:8] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? -ALUout_buff[15:8] : -ALU_source1[15:8] );
          ALUout_buff[23:16] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? -ALUout_buff[23:16] : -ALU_source1[23:16] );
          ALUout_buff[31:24] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? -ALUout_buff[31:24] : -ALU_source1[31:24] );
        end  
      endcase


      `OPstjzsys:
        case (ir_in2 `Dest)	      // use Dest as extended opcode
          `Destst: begin datamem[ALU_source2] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 ); end // st
          `Destjz: begin jump_addr <= ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff : ALU_source2 ); 
          jump_taken <= ( (|( (write_en && (ALUdest == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 )) ? 0 : 1 ); end // jz
          `Destjnz: begin jump_addr <= ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff : ALU_source2 ); 
          jump_taken <= ( (|( (write_en && (ALUdest == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 )) ? 1 : 0 ); end // jnz
          default: begin halt <= 1; end // sys
        endcase



        `OPand: begin ALUout <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 ) & 
          ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff : ALU_source2 );
        ALUout_buff <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 ) & 
          ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff : ALU_source2 ); end
          `OPor: begin ALUout <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 ) | 
            ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff : ALU_source2 );
          ALUout_buff <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 ) | 
            ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff : ALU_source2 ); end
            `OPxor: begin ALUout <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 ) ^ 
              ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff : ALU_source2 );
            ALUout_buff <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 ) ^ 
              ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff : ALU_source2 ); end
              `OPadd: begin ALUout <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 ) + 
                ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff : ALU_source2 );
              ALUout_buff <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 ) + 
                ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff : ALU_source2 ); end
                `OPaddv: begin 
                ALUout[7:0] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] ) + 
                  ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff[7:0] : ALU_source2[7:0] );
                ALUout[15:8] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[15:8] : ALU_source1[15:8] ) + 
                  ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff[15:8] : ALU_source2[15:8] );
                ALUout[23:16] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[23:16] : ALU_source1[23:16] ) + 
                  ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff[23:16] : ALU_source2[23:16] );
                ALUout[31:24] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[31:24] : ALU_source1[31:24] ) + 
                  ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff[31:24] : ALU_source2[31:24] );

                ALUout_buff[7:0] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] ) + 
                  ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff[7:0] : ALU_source2[7:0] );
                ALUout_buff[15:8] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[15:8] : ALU_source1[15:8] ) + 
                  ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff[15:8] : ALU_source2[15:8] );
                ALUout_buff[23:16] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[23:16] : ALU_source1[23:16] ) + 
                  ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff[23:16] : ALU_source2[23:16] );
                ALUout_buff[31:24] <= ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[31:24] : ALU_source1[31:24] ) + 
                  ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff[31:24] : ALU_source2[31:24] );
              end

              `OPshift: begin 
              ALUout <= ( ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff[31] : ALU_source2[31] ) ? 
                (( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[31] : ALU_source1[31] ) ?
                ( -(( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? -ALUout_buff : -ALU_source1 ) >> ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? -ALUout_buff : -ALU_source2 ))) :
                ( (( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 ) >> ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? -ALUout_buff : -ALU_source2 ))) ):
                (( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 ) << ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff : ALU_source2 )) );

              ALUout_buff <= ( ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff[31] : ALU_source2[31] ) ? 
                (( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[31] : ALU_source1[31] ) ?
                ( -(( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? -ALUout_buff : -ALU_source1 ) >> ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? -ALUout_buff : -ALU_source2 ))) :
                ( (( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 ) >> ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? -ALUout_buff : -ALU_source2 ))) ):
                (( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 ) << ( (write_en && (ALUdest_buff == ir_in2 `Arg2)) ? ALUout_buff : ALU_source2 )) ); end
                /*
                This is a ridiculus amount of code for a simple concept but due to the required checking of the output buffer it got very large.
                  Basically there is an extra check. If the source number is negative then multiply by -1, do the right shift, and then multiply
                  by -1 again so that the sign bit gets extended.
                    */


                   `OPpack: begin 
                   ALUout[7:0] <= ( P[0] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] ) : 
                     ( (write_en && (ALUdest_buff == ir_in2 `Dest)) ? ALUout_buff[7:0] : ALU_source2[7:0] ) );
                   ALUout[15:8] <= ( P[1] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] ) : 
                     ( (write_en && (ALUdest_buff == ir_in2 `Dest)) ? ALUout_buff[15:8] : ALU_source2[15:8] ) );
                   ALUout[23:16] <= ( P[2] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] ) : 
                     ( (write_en && (ALUdest_buff == ir_in2 `Dest)) ? ALUout_buff[23:16] : ALU_source2[23:16] ) );
                   ALUout[31:24] <= ( P[3] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] ) : 
                     ( (write_en && (ALUdest_buff == ir_in2 `Dest)) ? ALUout_buff[31:24] : ALU_source2[31:24] ) );

                   ALUout_buff[7:0] <= ( P[0] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] ) : 
                     ( (write_en && (ALUdest_buff == ir_in2 `Dest)) ? ALUout_buff[7:0] : ALU_source2[7:0] ) );
                   ALUout_buff[15:8] <= ( P[1] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] ) : 
                     ( (write_en && (ALUdest_buff == ir_in2 `Dest)) ? ALUout_buff[15:8] : ALU_source2[15:8] ) );
                   ALUout_buff[23:16] <= ( P[2] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] ) : 
                     ( (write_en && (ALUdest_buff == ir_in2 `Dest)) ? ALUout_buff[23:16] : ALU_source2[23:16] ) );
                   ALUout_buff[31:24] <= ( P[3] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] ) : 
                     ( (write_en && (ALUdest_buff == ir_in2 `Dest)) ? ALUout_buff[31:24] : ALU_source2[31:24] ) );
                 end 

                 `OPunpack: begin 
                 ALUout <= ( P[0] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] ) : 0) +
                   ( P[1] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[15:8] : ALU_source1[15:8] ) : 0) +
                   ( P[2] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[23:16] : ALU_source1[23:16] ) : 0) +
                   ( P[3] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[31:24] : ALU_source1[31:24] ) : 0);

                 ALUout_buff <= ( P[0] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[7:0] : ALU_source1[7:0] ) : 0) +
                   ( P[1] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[15:8] : ALU_source1[15:8] ) : 0) +
                   ( P[2] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[23:16] : ALU_source1[23:16] ) : 0) +
                   ( P[3] ? ( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff[31:24] : ALU_source1[31:24] ) : 0);  
               end 

               `OPli: begin ALUout <= { {24{ALU_source1[7]}}, ALU_source1 `immed }; 
               ALUout_buff <= { {24{ALU_source1[7]}}, ALU_source1 `immed }; end
               `OPmorei: begin ALUout <= { ( (write_en && (ALUdest_buff == ir_in2 `Dest)) ? ALUout_buff[23:0] : ALU_source2[23:0] ), (ir_in2 `immed ) };
               ALUout_buff <= { ( (write_en && (ALUdest_buff == ir_in2 `Dest)) ? ALUout_buff[23:0] : ALU_source2[23:0] ), (ir_in2 `immed ) }; end

               //default: halt <= 1;
             endcase	


           end else begin

             write_en <= 0; jump_taken <= 0; 

           end
           parallel_store_en_buf <= parallel_store_en;
           parallel_addr_buf     <= parallel_addr;
           parallel_val_buf      <= parallel_val;    

         end


         endmodule


         module processor(clk, reset, jump, halt, pc_mon, ir, u0, u1, u2, u3, u4, u5, u6, u7, u8, u9, parallel_store_mon, parallel_addr_mon, parallel_val_mon );
         input clk, reset;
         output reg jump;
         output halt;
         output `Register u0, u1, u2, u3, u4, u5, u6, u7, u8, u9;
         output reg `WORD pc_mon, ir;
         output parallel_store_mon;
         output `WORD parallel_addr_mon, parallel_val_mon;

         wire jump_taken, write_en;
         wire `address D, P, ALUdest;
         wire `WORD ir_conn, ir_conn2;
         wire parallel_store_en__inst_to_reg;
         wire `address parallel_addr_reg__inst_to_reg, parallel_val_reg__inst_to_reg;
         wire parallel_store_en__reg_to_alu;
         wire `WORD parallel_addr__reg_to_alu, parallel_val__reg_to_alu;
         wire `WORD pc;
         integer idx;
         wire `Register ALU_out, ALU_source1, ALU_source2, jump_addr;

         always @(posedge clk) begin

           jump <= jump_taken;
           ir <= ir_conn;
           pc_mon <= pc;

         end


         instruction_block ib(clk, reset, jump_taken, jump_addr, pc, ir_conn, parallel_store_en__inst_to_reg, parallel_addr_reg__inst_to_reg, parallel_val_reg__inst_to_reg );
         regfile_block rb(clk, reset, write_en, pc, ir_conn, ALU_out, ALUdest, jump_taken, ALU_source1, ALU_source2, ir_conn2, D, P,
           u0, u1, u2, u3, u4, u5, u6, u7, u8, u9,
           parallel_store_mon, parallel_addr_mon, parallel_val_mon, parallel_store_en__inst_to_reg, parallel_addr_reg__inst_to_reg,
           parallel_val_reg__inst_to_reg, parallel_store_en__reg_to_alu, parallel_addr__reg_to_alu, parallel_val__reg_to_alu );
         ALU_block ab(clk, reset, ALU_source1, ALU_source2, ir_conn2, D, P, halt, write_en, ALUdest, ALU_out, jump_taken, jump_addr,
           parallel_store_en__reg_to_alu, parallel_addr__reg_to_alu, parallel_val__reg_to_alu );



         endmodule


         module testbench();
         reg clk, reset;
         wire halt, jump;
         wire `WORD pc;
         wire `Register u0, u1, u2, u3, u4, u5, u6, u7, u8, u9;
         wire `WORD ir;
         wire  parallel_store_en;
         wire `WORD parallel_addr_reg, parallel_val_reg;

         initial begin
           $dumpfile("dumpfile.vcd");
           $dumpvars(0, p1);
           clk = 0;
           reset = 0;
           #1 reset = 1;
         end

         //always @(posedge clk) $display("%d\t%h\t%h\t%h\t%h\t%h\t%h", pc, ir, jump, halt, parallel_store_en, parallel_addr_reg, parallel_val_reg);
         
         always @(posedge clk) $display("%d\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h", pc, ir, jump, halt, u0, u1, u2, u3, u4, u5, u6, u7, u8, u9, parallel_store_en, parallel_addr_reg, parallel_val_reg);

         always begin
           //$display("%d\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h", pc, ir, jump, halt, u0, u1, u2, u3, u4, u5, u6, u7, u8, u9, parallel_store_en, parallel_addr_reg, parallel_val_reg);
           #10 clk = ~clk;

           if (halt == 1'b1) $finish;
         end

       processor p1(clk, reset, jump, halt, pc, ir, u0, u1, u2, u3, u4, u5, u6, u7, u8, u9, parallel_store_en, parallel_addr_reg, parallel_val_reg);

       endmodule
