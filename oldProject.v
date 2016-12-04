// basic sizes of things
`define WORD	 [15:0]
`define DOUBLE [31:0]
`define high_instruction [31:16]
`define low_instruction [15:0]
`define Register [31:0]
`define address [3:0]
`define Opcode [15:12]
`define Dest	 [11:8]
`define Arg1	[7:4]
`define Arg2    [3:0]
`define immed 	[7:0]
`define STATE	[5:0]
`define REGSIZE [15:0]
`define MEMSIZE [65535:0]
`define HALF_MEMSIZE [32768:0]

// opcode values, also state numbers
`define OPand		4'b0000
`define OPor   		4'b0001
`define OPxor 		4'b0010
`define OPadd		4'b0100
`define OPaddv 		4'b0101
`define OPshift		4'b0111
`define OPpack 		4'b1000
`define OPunpack 	4'b1001
`define OPli		4'b1010
`define OPmorei 	4'b1011
`define OPldanne	4'b1110
`define OPstjzsys	4'b1111


// state numbers only
`define OPld		6'b010000
`define OPany		6'b010010
`define OPanyv		6'b010011
`define OPneg		6'b010100
`define OPnegv		6'b010101
`define OPst		6'b110000
`define OPjz		6'b110010
`define OPjnz		6'b110011
`define OPsys		6'b111111
`define Start		5'b11111
`define Start1		5'b11110

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

module instruction_block(clk, reset, jump_taken, jump_addr, pc_buff, ir);
input clk, reset, jump_taken;
input `Register jump_addr;
output reg `WORD pc_buff, ir;
output reg jump_flag, jump_type;
output reg [1:0] o3_flag;
integer idx;

reg high_or_low_instruction;

reg `WORD pc;
reg `WORD mainmem_16 `MEMSIZE;
reg `DOUBLE mainmem `HALF_MEMSIZE;


always @(reset) begin
  pc = 0;
  high_or_low_instruction = 0;
  $readmemh("mainmem_16.vmem",mainmem_16 );
  $readmemh("mainmem.vmem",mainmem    );

  for( idx=0; idx < 65536; idx = idx + 2 ) begin
    mainmem[idx/2] = { mainmem_16[idx], mainmem_16[idx + 1] };
  end
end


always @(posedge clk) begin

   //if ( (mainmem[pc] != 16'hffff) && (mainmem[jump_addr] != 16'hfff) ) begin
	ir <= ( (jump_taken) ? ( (jump_addr % 2 == 0) ? mainmem[jump_addr/2] `low_instruction : mainmem[jump_addr/2] `high_instruction ) :
                         ( (high_or_low_instruction) ? mainmem[pc/2] `high_instruction : mainmem[pc/2] `high_instruction )       );
	pc <= ( (jump_taken) ? jump_addr : pc+1 );
	pc_buff <= pc;
  high_or_low_instruction <= ( (jump_taken) ? ( (jump_addr % 2 == 0) ? 1'b1 : 1'b0 ) : ~high_or_low_instruction );

   //end
   //else
	//halt <= 1;

end

always @(pc) begin
    // check two instructions

    // if instructions have dependencies, set flag to not allow o-o-o execution

    // else, do not set flag
end

endmodule


module regfile_block(clk, reset, write_en, pc, ir_in, ALU_in, ALUdest, jump_taken, ALU_source1, ALU_source2, ir_in2, D, P,
			u0, u1, u2, u3, u4, u5, u6, u7, u8, u9);

input clk, reset, write_en, jump_taken;
input `WORD pc, ir_in;
input `Register ALU_in;
input `address ALUdest;
output reg `WORD ir_in2;
output reg `Register ALU_source1, ALU_source2, u0, u1, u2, u3, u4, u5, u6, u7, u8, u9;
output reg `address D, P;

reg `Register regfile `REGSIZE;

always @(reset) begin
  ALU_source1 = 32'h00000000;
  ALU_source2 = 32'h00000000;
  $readmemh("regfile.vmem",regfile);

end


always @(posedge clk) begin

/* Note code that looks like ( (write_en && (ALUdest == ir_in `Arg1)) ? ALU_in : regfile[ir_in `Arg1] ) is looking to see
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

    ir_in2 <= ir_in;

    if (write_en)
       regfile[ALUdest] <= ALU_in;

 if (jump_taken == 0) begin
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

	  end
end


endmodule


module ALU_block(clk, reset, ALU_source1, ALU_source2, ir_in2, D, P, halt, write_en, ALUdest, ALUout, jump_taken, jump_addr);
input clk, reset;
input `Register ALU_source1, ALU_source2;
input `WORD ir_in2;
input `address  D, P;
output reg halt, write_en, jump_taken;
output reg `Register ALUout, jump_addr;
output reg `address ALUdest;

reg `Register datamem `MEMSIZE;
reg `Register ALUout_buff;
reg `address ALUdest_buff;

always @(reset) begin
  halt = 1'b0;
  write_en = 1'b0;
  jump_taken = 1'b0;
  jump_addr = 32'h00000000;
  ALUout_buff = 32'h00000000;
  ALUout = 32'h00000000;
  ALUdest_buff = 4'h0;
  ALUdest = 4'h0;
  $readmemh("datamem.vmem",datamem);
end


always @(posedge clk) begin
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
    case (ir_in2 `Opcode)
    `OPldanne:
                case (ir_in2 `Arg2)	      // use Arg2 as extended opcode
                    `Arg2ld: begin  //ld
				   ALUout <= datamem[( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 )];
				   ALUout_buff <= datamem[( (write_en && (ALUdest_buff == ir_in2 `Arg1)) ? ALUout_buff : ALU_source1 )];
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
end


endmodule


module processor(clk, reset, jump, halt, pc, ir, u0, u1, u2, u3, u4, u5, u6, u7, u8, u9);
input clk, reset;
output reg jump;
output halt;
output `Register u0, u1, u2, u3, u4, u5, u6, u7, u8, u9;
output reg `WORD pc, ir;

wire jump_taken, write_en;
wire `address D, P, ALUdest;
wire `WORD pc_buff, ir_conn, ir_conn2;
wire `Register ALU_out, ALU_source1, ALU_source2, jump_addr;

always @(posedge clk) begin

jump <= jump_taken;
ir <= ir_conn;
pc <= pc_buff;

end


instruction_block ib(clk, reset, jump_taken, jump_addr, pc_buff, ir_conn);
regfile_block rb(clk, reset, write_en, pc_buff, ir_conn, ALU_out, ALUdest, jump_taken, ALU_source1, ALU_source2, ir_conn2, D, P,
			u0, u1, u2, u3, u4, u5, u6, u7, u8, u9);
ALU_block ab(clk, reset, ALU_source1, ALU_source2, ir_conn2, D, P, halt, write_en, ALUdest, ALU_out, jump_taken, jump_addr);


endmodule


module testbench();
reg clk, reset;
wire halt, jump;
wire `WORD pc;
wire `Register u0, u1, u2, u3, u4, u5, u6, u7, u8, u9;
wire `WORD ir;

initial begin
  $dumpfile("dumpfile.vcd");
  $dumpvars(0, p1);
	clk = 0;
	reset = 0;
	#1 reset = 1;
end

always begin
	#10 clk = ~clk;
        $display("%d\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h", pc, ir, jump, halt, u0, u1, u2, u3, u4, u5, u6, u7, u8, u9);

        if (halt == 1'b1) $finish;
end

processor p1(clk, reset, jump, halt, pc, ir, u0, u1, u2, u3, u4, u5, u6, u7, u8, u9);

endmodule
