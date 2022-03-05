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


    .org $00a9
lda_zp_test: .byte $07

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

    sta FINISHED_TRIGGER

flags_dump:
    php
    plp
    rts

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

int_handler:
nmi_handler:
    brk



    .org $fffa
nmi_vector:     .word nmi_handler
reset_vector:   .word reset_handler
irq_vector:     .word int_handler