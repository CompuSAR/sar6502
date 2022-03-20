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

    .org $000a
    .byte $cc, $d2              ; cmp zp,x test

    .org $000e
bit_zp_test:    .byte $f3

    .org $0014
                .byte $01       ; dec zp,x test

    .org $0028
                .byte $7a       ; bit zp,x test
                .byte $6e       ; asl zp,x test
    .org $0069
asl_zp_test:    .byte $d3
    .org $0074
dec_zp_test:    .byte $f8

    .org $00a9
lda_zp_test:    .word lda_indirect_test
    .org $ec
adc_zp_test:
    .byte $88, $d5, $13, $c2

cmp_zp_test:
    .byte $4f, $0a

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
    sbc adc_abs_test,x
    pha
    php
    sbc adc_abs_test,y
    pha
    php
    sbc #$cd
    pha
    php
    sbc adc_zp_test
    pha
    php
    sbc (adc_zp_test,x)
    pha
    php
    sbc adc_zp_test,x
    pha
    php
    sbc (adc_zp_test)
    pha
    php
    sbc (adc_zp_test),y
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

    .org $3787
cmp_abs_test:
    .byte b8

    .org $37a1
    .byte 6f

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

int_handler:
    php
    plp
    rti

nmi_handler:
    jmp int_handler

    .org $61da
dec_abs_test    .byte $7b

    .org $627a
                .byte $01 ; dec abs,x test

    .org $6d21
lda_abs_test    .byte $74
    .org $6d51
                .byte $c7       ; lda abs,y
    .org $6de1
                .byte $08       ; lda abs,x

    .org $7ace
adc_abs_test:
    .byte $65, $ca, $26, $6b

    .org $8510
asl_abs_test    .byte $56
    .org $85d0
                .byte $40       ; asl abs,x test

    .org $ae38
lda_indirect_test .byte $bf
    .org $ae68
                .byte $20       ; lda (zp),y test
    .org $af28
                .byte $22       ; lda (zp),y test

    .org $c213
    .byte $25                   ; adc (zp,x) test

    .org $d2cc
    .byte $38                   ; cmp (zp,x) test

    .org $d588
    .byte $15                   ; adc (zp,x) test
    .byte $8e, $9b, $f5         ; adc (zp),y test

    .org $fffa
nmi_vector:     .word nmi_handler
reset_vector:   .word reset_handler
irq_vector:     .word int_handler
