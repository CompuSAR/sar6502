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


    .org $0029
                .byte $6e       ; asl zp,x test
    .org $0069
asl_zp_test:    .byte $d3
    .org $00a9
lda_zp_test:    .word lda_indirect_test

    .org $0100
    .dc $ff,$7a         ; Put stack in known state
    .byte $7a           ; Due to a limitation of our lst parser, need to mark the end point

    .org $0300

start:
    nop
    nop
    nop
    lda #$03
    jsr flags_dump
    lda lda_zp_test     ; Make sure we don't treat the operand as an opcode
    jsr flags_dump

    lda lda_abs_test
    jsr flags_dump

    ldx #$c0
    ldy #$30

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
    lda lda_zp_test
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
nmi_handler:
    brk

    .org $6d21
lda_abs_test    .byte $74
    .org $6d51
                .byte $c7       ; lda abs,y
    .org $6de1
                .byte $08       ; lda abs,x

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

    .org $fffa
nmi_vector:     .word nmi_handler
reset_vector:   .word reset_handler
irq_vector:     .word int_handler
