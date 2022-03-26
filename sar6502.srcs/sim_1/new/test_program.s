CARRY    = %00000001
ZERO     = %00000010
INTMASK  = %00000100
DECIMAL  = %00001000
BRK      = %00100000
OVERFLOW = %01000000
NEGATIVE = %10000000

FINISHED_TRIGGER        = $200
NMI_TRIGGER_COUNT       = $2fa
NMI_TRIGGER_DELAY       = $2fb
RESET_TRIGGER_COUNT     = $2fc
RESET_TRIGGER_DELAY     = $2fd
INT_TRIGGER_COUNT       = $2fe
INT_TRIGGER_DELAY       = $2ff

value_dump = $ff00

    .org $0000
                .byte $5e       ; eor zp,x and (zp,x) tests (MSB)

    .org $000a
    .byte $cc, $d2              ; cmp zp,x test

    .org $000e
bit_zp_test:    .byte $f3

    .org $0014
                .byte $01       ; dec zp,x test

    .org $0028
                .byte $7a       ; bit zp,x test
                .byte $6e       ; asl zp,x test

sta_zp_test:    .byte 0, 0, $34

    .org $002e
rmb_zp_test:    .byte $65, $65 ^ $ff
smb_zp_test:    .byte $f0, $f0 ^ $ff

    .org $0034
                .byte $a8       ; ldy zp,x
    .org $003f
ldy_zp_test:    .byte $2c

    .org $004e
rol_zp_test:    .byte $41, $9b, $60

    .org $005f
eor_zp_test:
                .byte $88, $70

    .org $0069
asl_zp_test:    .byte $d3
    .org $0074
dec_zp_test:    .byte $f8

    .org $0078
ldx_zp_test:    .byte $4d

    .org $0092
                .byte $6e       ; lsr zp,x

    .org $0099
branch_bit_test: .byte $50

                .byte $f5       ; ldx zp,y

    .org $009d
lsr_zp_test:    .byte $7e

    .org $00a9
lda_zp_test:    .word lda_indirect_test
    .org $ec
adc_zp_test:
    .byte $88, $d5, $13
inc_zp_test:
    .byte $c2

cmp_zp_test:
    .byte $4f, $0a, $8f

    .org $00ff
                .byte $e6       ; eor zp,x and (zp,x) tests (LSB)

    .org $0100
    .dc $ff,$7a         ; Put stack in known state
    .byte $7a           ; Due to a limitation of our lst parser, need to mark the end point

    .org $0300

start:
    nop
    nop
    nop
    jsr flags_dump

    lda #$03
    jsr flags_dump
    lda lda_zp_test     ; Make sure we don't treat the operand as an opcode
    jsr flags_dump

    lda lda_abs_test
    jsr flags_dump

    ldx #$c0
    ldy #$30
    phx
    phy

    ; Direct flags manipulation
    lda #$ff
    sta $1fe
    stz $1ff

    plp
    jsr flags_dump
    plp
    jsr flags_dump
    sec
    jsr flags_dump
    sed
    jsr flags_dump
    sei
    jsr flags_dump
    clc
    jsr flags_dump
    cld
    jsr flags_dump
    cli
    jsr flags_dump
    clv
    jsr flags_dump

    ; Test addressing modes
    lda lda_abs_test
    lda lda_abs_test,x          ; No page transition
    lda lda_abs_test-$c0,x      ; With page transition
    lda lda_abs_test,y          ; No page transition
    lda lda_abs_test-$30,y      ; With page transition
    lda lda_zp_test
    .if c02
    lda (lda_zp_test+$100-$c0,x)
    .endif
    lda lda_zp_test+$100-$c0,x
    .if c02
    lda (lda_zp_test)
    .endif
    lda (lda_zp_test),y         ; No page transition
    ldy #$f0
    lda (lda_zp_test),y         ; With page transition


    ; ASL test
    lda #1
asl_loop:
    asl asl_abs_test
    jsr flags_dump
    asl asl_abs_test,x
    jsr flags_dump
    asl asl_zp_test
    jsr flags_dump
    asl asl_zp_test,x
    jsr flags_dump
    asl
    jsr flags_dump
    bne asl_loop

    ; ADC tests
    ldx #2
    ldy #1
adc_loop:
    adc adc_abs_test
    pha
    php
    and adc_abs_test
    pha
    php
    adc adc_abs_test,x
    pha
    php
    and adc_abs_test,x
    pha
    php
    adc adc_abs_test,y
    pha
    php
    and adc_abs_test,y
    pha
    php
    adc #$cd
    pha
    php
    and #$a7
    pha
    php
    adc adc_zp_test
    pha
    php
    and adc_zp_test
    pha
    php
    adc (adc_zp_test,x)
    pha
    php
    and (adc_zp_test,x)
    pha
    php
    adc adc_zp_test,x
    pha
    php
    and adc_zp_test,x
    pha
    php
    adc (adc_zp_test)
    pha
    php
    and (adc_zp_test)
    pha
    php
    adc (adc_zp_test),y
    pha
    php
    and (adc_zp_test),y
    pha
    php
    iny
    dex
    bne adc_loop

sbc_loop:
    sbc adc_abs_test
    pha
    php
    ora adc_abs_test
    pha
    php
    sbc adc_abs_test,x
    pha
    php
    ora adc_abs_test,x
    pha
    php
    sbc adc_abs_test,y
    pha
    php
    ora adc_abs_test,y
    pha
    php
    sbc #$cd
    pha
    php
    ora #$cd
    pha
    php
    sbc adc_zp_test
    pha
    php
    ora adc_zp_test
    pha
    php
    sbc (adc_zp_test,x)
    pha
    php
    ora (adc_zp_test,x)
    pha
    php
    sbc adc_zp_test,x
    pha
    php
    ora adc_zp_test,x
    pha
    php
    sbc (adc_zp_test)
    pha
    php
    ora (adc_zp_test)
    pha
    php
    sbc (adc_zp_test),y
    pha
    php
    ora (adc_zp_test),y
    pha
    php
    inx
    dey
    bne sbc_loop

    ; BIT test
    lda #$4f
    php
    ldx #$1a
    php
    ldy #$22
    php

    bit bit_abs_test
    php
    bit bit_abs_test,x
    php
    bit #$b0
    php
    bit bit_zp_test
    php
    bit bit_zp_test,x
    php

    ; BRK test
    sed
    cli
    brk
    .byte $1
    cld
    sei
    brk
    .byte $2

    ; CMP test
    cmp cmp_abs_test
    php
    cmp cmp_abs_test,x
    php
    cmp cmp_abs_test,y
    php
    cmp #$db
    php
    cmp cmp_zp_test
    php
    cmp (cmp_zp_test,x)
    php
    cmp cmp_zp_test,x
    php
    cmp (cmp_zp_test)
    php
    cmp (cmp_zp_test),y
    php
    cmp #$4e
    php
    cmp #$4f
    php
    cmp #$50
    php

    ; CPX test
    cpx cmp_abs_test
    php
    cpx #$db
    php
    cpx cmp_zp_test
    php
    cpx #$19
    php
    cpx #$1a
    php
    cpx #$1b
    php
    ldx #$a0
    cpx #$0
    php

    ; CPY test
    cpy cmp_abs_test
    php
    cpy #$db
    php
    cpy cmp_zp_test
    php
    cpy #$19
    php
    cpy #$1a
    php
    cpy #$1b
    php
    ldx #$a0
    cpy #$0
    php

    ; DEC test
    dec dec_abs_test
    php
    dec dec_abs_test,x
    php
    dec dec_zp_test
    php
    dec dec_zp_test,x
    php

dec_loop:
    dec
    php
    bne dec_loop
    dec
    php

    jsr bb_test
    lda branch_bit_test
    eor #$ff
    sta branch_bit_test
    jsr bb_test

    ; EOR test
    php
    eor eor_abs_test
    pha
    php
    eor eor_abs_test,x
    pha
    php
    eor eor_abs_test,y
    pha
    php
    eor #$f4
    pha
    php
    eor eor_zp_test
    pha
    php
    eor (eor_zp_test,x)
    pha
    php
    eor eor_zp_test,x
    pha
    php
    eor (eor_zp_test)
    pha
    php
    eor (eor_zp_test),y
    pha
    php


    ; inc tests
    dec
    dec
    ldx #$3

inc_loop:
    inc inc_abs_test
    php
    inc inc_abs_test,x
    php
    inc
    php
    inc inc_zp_test
    php
    inc inc_zp_test,x
    php

    dex
    bne inc_loop

    jmp jmp_tests
    brk                 ; Unreachable
jmp_test_done:
    php


    ; LDX test
    ldx ldx_abs_test
    php
    stx value_dump
    ldx ldx_abs_test,y
    php
    stx value_dump
    ldx ldx_abs_test-$22,y
    php
    stx value_dump
    ldx #$04
    php
    stx value_dump
    ldx ldx_zp_test
    php
    stx value_dump
    ldx ldx_zp_test,y
    php
    stx value_dump


    ; LDY test
    ldy ldy_abs_test
    php
    sty value_dump
    ldy ldy_abs_test,x
    php
    sty value_dump
    ldy #$6c
    php
    sty value_dump
    ldy ldy_zp_test
    php
    sty value_dump
    ldy ldy_zp_test,x
    php
    sty value_dump


    ; LSR test
    lsr lsr_abs_test
    php
    lsr lsr_abs_test,x
    php
    lsr
    php
    sta value_dump
    lsr lsr_zp_test
    php
    lsr lsr_zp_test,x
    php


    ; Stack pull tests
    lda #$fc
    sta $1a3    ; A
    stz $1a4    ; Y
    lda #$55
    sta $1a5    ; A
    lda #$dd
    sta $1a6    ; Y
    stz $1a7    ; A
    lda #$03
    sta $1a8    ; Y
    lda #$9b
    sta $1a9    ; X
    lda #$2d
    sta $1aa    ; X
    stz $1ab    ; X

    ldx #$03
pull_test_loop1:
    pla
    sta value_dump
    php
    plp
    ply
    sty value_dump
    php
    plp

    dex
    bne pull_test_loop1

pull_test_loop2:
    plx
    stx value_dump
    php
    plp

    dey
    bne pull_test_loop2


    ; RMB/SMB test
    rmb 0,rmb_zp_test
    rmb 0,rmb_zp_test+1
    rmb 1,rmb_zp_test
    rmb 1,rmb_zp_test+1
    rmb 2,rmb_zp_test
    rmb 2,rmb_zp_test+1
    rmb 3,rmb_zp_test
    rmb 3,rmb_zp_test+1
    rmb 4,rmb_zp_test
    rmb 4,rmb_zp_test+1
    rmb 5,rmb_zp_test
    rmb 5,rmb_zp_test+1
    rmb 6,rmb_zp_test
    rmb 6,rmb_zp_test+1
    rmb 7,rmb_zp_test
    rmb 7,rmb_zp_test+1

    smb 0,smb_zp_test
    smb 0,smb_zp_test+1
    smb 1,smb_zp_test
    smb 1,smb_zp_test+1
    smb 2,smb_zp_test
    smb 2,smb_zp_test+1
    smb 3,smb_zp_test
    smb 3,smb_zp_test+1
    smb 4,smb_zp_test
    smb 4,smb_zp_test+1
    smb 5,smb_zp_test
    smb 5,smb_zp_test+1
    smb 6,smb_zp_test
    smb 6,smb_zp_test+1
    smb 7,smb_zp_test
    smb 7,smb_zp_test+1


    ; ROL/ROR test
    ldx #1
    lda #$af
    rol rol_abs_test
    php
    rol
    php
    sta value_dump
    ror rol_abs_test+1
    php
    rol rol_abs_test,x
    php
    ror rol_abs_test+1,x
    php
    ror
    php
    sta value_dump
    rol rol_zp_test
    php
    ror rol_zp_test+1
    php
    rol rol_zp_test,x
    php
    ror rol_zp_test+1,x
    php


    ; STA test
    ldy #$02
    sta sta_abs_test
    inc
    sta sta_abs_test,x
    inc
    sta sta_abs_test,y
    inc
    sta sta_zp_test
    inc
    sta sta_zp_test,x
    inc
    sta (sta_zp_test,x)
    inc
    sta (sta_zp_test)
    inc
    sta (sta_zp_test),y
    inc
    ldx #$80
    sta sta_abs_test,x
    inc
    ldy #$ff
    sta sta_abs_test,y
    inc
    sta (sta_zp_test),y
    php


    ; STX test
    stx sta_abs_test
    inx
    stx sta_zp_test+1
    inx
    stx sta_zp_test+1,y
    php


    ; STY test
    ldx #$2
    sty sta_abs_test
    iny
    sty sta_zp_test
    iny
    sty sta_zp_test,x
    php


    sta FINISHED_TRIGGER
    .byte 00


reset_handler:
    ldx #$ff
    txs
    lda #start/256
    pha
    lda #start%256
    pha
    lda #INTMASK+OVERFLOW
    pha
    rti
    .byte 00

    .org $0a4f
    .byte $8f           ; cmp (zp) test

    .org $0a71
    .byte $25           ; cmp (zp),y test

    .org $13d5
    .byte $dd           ; adc (zp,x) test

    .org $1690
bit_abs_test:
    .byte $6b

    .org $16aa
    .byte $03           ; bit abs,x

    .org $1a10
ldx_abs_test:
    .byte $6d

    .org $1a32
    .byte $d7           ; ldx abs,y

    .org $26ff
jmp_ind_test2:
    .word jmp_dest2
    .byte 0
    .word jmp_dest4

    .org $2746
    .word jmp_test_done

    .org $274f
    .word jmp_dest5

    .org $27b8
jmp_ind_test1:
    .word jmp_dest1
    .byte 0
    .word jmp_dest3

    .org $27ff
    .word jmp_dest3

    .org $2808
    .word jmp_dest3

    .org $3787
cmp_abs_test:
    .byte $b8

    .org $37a1
    .byte $6f

    .org $37a9
    .byte $0e

    .org $40ef

flags_dump:
    php
    bcs .1
    bcc .1

.2  bvs .3
    bvc .3

.4  bra .5

.3  bne .4
    beq .4

.1  bmi .2
    bpl .2

.5
    plp
    rts
    .byte 00

    .org $41ef
bb_test:
    ; Branch bit tests
    bbr 0, branch_bit_test, .1
    bbs 0, branch_bit_test, .1

.2  bbr 2, branch_bit_test, .3
    bbs 2, branch_bit_test, .3

.4  bbr 4, branch_bit_test, .5
    bbs 4, branch_bit_test, .5

.6  bbr 6, branch_bit_test, .7
    bbs 6, branch_bit_test, .7

.8  rts

.7  bbr 7, branch_bit_test, .8
    bbs 7, branch_bit_test, .8

.5  bbr 5, branch_bit_test, .6
    bbs 5, branch_bit_test, .6

.3  bbr 3, branch_bit_test, .4
    bbs 3, branch_bit_test, .4

.1  bbr 1, branch_bit_test, .2
    bbs 1, branch_bit_test, .2

    brk         ; Unreachable

int_handler:
    php
    plp
    rti

nmi_handler:
    jmp int_handler
    brk         ; Unreachable

    .org $4694
sta_abs_test:   .byte $87, $91, $20

    .org $55aa
jmp_tests:
    ldx #$3

    jmp (jmp_ind_test1)
    brk         ; Unreachable

    .org $5ee6
                .byte $3f                       ; eor (zp,x) test

    .org $61da
dec_abs_test    .byte $7b

    .org $61ff
inc_abs_test    .byte $fe, $30, $22, $48        ; inc abs, int abs,x tests

    .org $627a
                .byte $01 ; dec abs,x test

    .org $6d21
lda_abs_test    .byte $74
    .org $6d51
                .byte $c7       ; lda abs,y
    .org $6de1
                .byte $08       ; lda abs,x

    .org $7088
                .byte $29       ; eor (zp)
    .org $70aa
                .byte $50       ; eor (zp),y

    .org $7ace
adc_abs_test:
    .byte $65, $ca, $26, $6b

    .org $7ea8
lsr_abs_test:
    .byte $9b

    .org $7f9d
    .byte $49                   ; lsr abs,x

    .org $8510
asl_abs_test:   .byte $56
    .org $85d0
                .byte $40       ; asl abs,x test

    .org $a303
jmp_dest2:
                jmp (jmp_ind_test1,x)
                brk             ; Unreachable

jmp_dest3:
                jmp (jmp_ind_test2,x)
                brk

jmp_dest5:
                ldx #$47
                jmp jmp_dest1
                brk             ; Unreachable

jmp_dest4:
                ldx #$50

jmp_dest1:
                jmp (jmp_ind_test2)
                brk             ; Unreachable

    .org $ae38
lda_indirect_test .byte $bf
    .org $ae68
                .byte $20       ; lda (zp),y test
    .org $af28
                .byte $22       ; lda (zp),y test

    .org $c213
    .byte $25                   ; adc (zp,x) test

    .org $d074
eor_abs_test    .byte $17

    .org $d096
                .byte $11        ; eor abs,y

    .org $d114
                .byte $49       ; eor abs,x test

    .org $d2cc
    .byte $38                   ; cmp (zp,x) test

    .org $d588
    .byte $15                   ; adc (zp,x) test
    .byte $8e, $9b, $f5         ; adc (zp),y test

    .org $e308
ldy_abs_test:
    .byte $ff

    .org $e3fd
    .byte $93                   ; ldy abs,x

    .org $f9f8
rol_abs_test:
    .byte $cd, $71, $e4

    .org $fffa
nmi_vector:     .word nmi_handler
reset_vector:   .word reset_handler
irq_vector:     .word int_handler
