.data
test:
    .word 0x601
    .text
    nop
    nop
parallel_0:
    xor    $u3, $u0, $u1
    st     $u4, $u9
    ld     $u0, $u9
    li     $u3, 0x9
    li32    $u6, out_of_order_ok
    xor     $u5, $u0, $u3
    jz    $u5, $u6
    sys
out_of_order_ok:
    nop
    li     $u0, 0x2
    li     $u1, 0x1
    li     $u2, 0x2
    li     $u3, 0x3
    li     $u4, 0x4
    nop
    nop
    xor    $u2, $u0, $u1
    xor    $u5, $u2, $u4
    st     $u4, $u3
    st     $u0, $u1
    ld     $u4, $u3
    ld     $u2, $u1
    add    $u3, $u2, $u4
    li     $u2, 6
    li32    $u6, xorok
    xor     $u5, $u2, $u3
    jz    $u5, $u6
    sys
xorok:
    li32    $u5, 0x11111111
    li32    $u0, 0x66666666
    li32    $u1, 0x33333333
    xor    $u3, $u0, $u1
    li32    $u4, 0x55555555
    li32    $u6, jzok
    xor     $u5, $u4, $u3
    jz    $u5, $u6
    sys
jzok:
    li32    $u0, addok
    jz    $zero, $u0
    sys
addok:
    li32    $u8, 0x66666666
    li32    $u9, 0x33333333
    add    $u7, $u8, $u9
    li32    $u0, 0x99999999
    li32    $u1, negliok
    xor    $u4, $u0, $u7
    jz    $u4, $u1
    sys
negliok:
    li32    $u0, -2
    li32    $u1, 2
    li32    $u2, jnzok
    add    $u3, $u0, $u1
    jz    $u3, $u2
    sys
jnzok:
    li32    $u4, negok
    jnz    $u1, $u4
    sys
negok:
    neg    $u3, $u0
    xor    $u4, $u3, $u1
    li32    $u5, andok
    jz    $u4, $u5
    sys
andok:
    li32    $u0, 0x66666666
    li32    $u1, 0x33333333
    li32    $u2, 0x22222222
    and    $u3, $u0, $u1
    xor    $u4, $u3, $u2
    li32    $u5, anyok
    jz    $u4, $u5
    sys
anyok:
    li32    $u0, 0
    li32    $u1, 1
    any    $u2, $u1
    li32    $u3, anyfail
    jz    $u2, $u3     ;never taken
    any    $u2, $u0
    li32    $u3, nopok
    jz    $u2, $u3
anyfail:
    sys


nopok:
    nop ;assume if no crash nop worked
parallel_1:
    xor    $u3, $u0, $u1
    st     $u9, $u4
    and    $u3, $u0, $u2
    st     $u1, $u5
    and    $u3, $u0, $u1
    st     $u1, $u5
orok:
    li32    $u0, 0x66666666
    li32    $u1, 0x33333333
    li32    $u3, 0x77777777
    or    $u4, $u0, $u1
    li32    $u5, ldstok
    xor    $u6, $u4, $u3
    jz    $u6, $u5
    sys
ldstok:
    li32    $u0, test
    li32    $u1, 0x17171717
    st    $u1, $u0
    ld    $u2, $u0
    xor    $u3, $u2, $u1
    li32    $u4, shiftok
    jz    $u3, $u4
    sys
shiftok:
    li32    $u0, 0x11111111
    li32    $u1, 3
    li32    $u2, 0x88888888
    shift    $u3, $u0, $u1
    xor    $u4, $u3, $u2
    li32    $u5, shiftfail
    jnz    $u4, $u5       ;not taken
    li32    $u1, -3
    shift    $u3, $u2, $u1
    li32    $u0, 0xf1111111
    li32    $u6, addvok
    xor    $u4, $u3, $u0
    jnz    $u4, $u5      ;not taken
    li32    $u0, 0x77777777
    li32    $u1, -1
    shift    $u7, $u0, $u1
    li32    $u8, 0x3bbbbbbb
    xor    $u9, $u8, $u7
    jz    $u9, $u6
shiftfail:
    sys


addvok:
    li32    $u0, 0xc0c0c0c0
    li32    $u1, 0x50505050
    li32    $u2, 0x10101010
    addv    $u5, $u0, $u1
    xor    $u3, $u5, $u2
    li32    $u4, anyvok
    jz    $u3, $u4
    sys
anyvok:
    li32    $u0, 0x0001ff07
    li32    $u1, 0x00010101
    anyv    $u2, $u0
    xor    $u3, $u2, $u1
    li32    $u4, negvok
    jz    $u3, $u4
    sys
negvok:
    li32    $u0, 0xf0810700
    li32    $u1, 0x107ff900
    negv    $u2, $u0
    li32    $u3, packok
    xor    $u4, $u1, $u2
    jz    $u4, $u3
    sys
packok:
    li32    $u0, 0x01020387
    li32    $u1, 0x00000000
    li32    $u2, 0x00000000
    li32    $u4, 0x87878787
    pack    $u3[0b1111], $u0
    xor    $u5, $u3, $u4
    li32    $u6, packfail
    jnz    $u5, $u6        ;not taken
    pack    $u2[0b0000], $u0
    li32    $u6, unpackok
    jz    $u2, $u6
packfail:
    sys
unpackok:
    li32    $u0, 0x01010101
    unpack    $u1, $u0[0b1111]
    li32    $u2, 4
    xor    $u3, $u1, $u2
    li32    $u4, unpackfail    
    jnz    $u3, $u4       ;not taken
    unpack    $u5, $u0[0b0000]
    li32    $u4, good
    jz    $u5, $u4
unpackfail:
    sys
good:
    sys ;should end here if everything worked
fail:
    li32    $u0, 0xffffffff ;should not execute if sys works
    sys
