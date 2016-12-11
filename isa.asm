nop			:=	0:16
.dst	$.d, $.s, $.t	:=	.this:4	.d:4	.s:4	.t:4
.alias	.dst 0 and or xor 4 add addv 7 shift
pack	$.d[.p], $.s	:=	0x8:4	.d:4	.s:4	.p:4
unpack	$.d, $.s[.p]	:=	0x9:4	.d:4	.s:4	.p:4
.di	$.d, .i		:=	.this:4	.d:4	.i:8
.alias	.di 0xa li morei
.ds	$.d, $.s	:=	0xe:4	.d:4	.s:4	.this:4
.alias	.ds 0 ld 2 any anyv neg negv
.st	$.s, $.t	:=	0xf:4	.this:4	.s:4	.t:4
.alias	.st 0 st 2 jz jnz
sys			:=	-1:16
.const	zero pc sp fp ra rv u0 u1 u2 u3 u4 u5 u6 u7 u8 u9
.segment .text 16 0x10000 0 .VMEM
.segment .data 32 0x10000 0 .VMEM
.const 0 .lowfirst

li32	$.d, .i	?(((.i&0xffffff80)==0xffffff80)||((.i&0xffffff80)==0)) := {
	li:4	.d:4	.i:8 }
li32	$.d, .i	?(((.i&0xffff8000)==0xffff8000)||((.i&0xffff8000)==0)) := {
	li:4	.d:4	(.i>>8):8
	morei:4	.d:4	.i:8 }
li32	$.d, .i ?(((.i&0xff800000)==0xff800000)||((.i&0xff800000)==0)) := {
	li:4	.d:4	(.i>>16):8
	morei:4	.d:4	(.i>>8):8
	morei:4	.d:4	.i:8 }
li32	$.d, .i	:= {
	li:4	.d:4	(.i>>24):8
	morei:4	.d:4	(.i>>16):8
	morei:4	.d:4	(.i>>8):8
	morei:4	.d:4	.i:8 }
