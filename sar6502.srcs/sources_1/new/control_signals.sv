`timescale 1ns / 1ps

package control_signals;

typedef enum logic[31:0] {
    CTL_SIG_INVALID = 'X,

    LOAD_A = 0,
    LOAD_X,
    LOAD_Y,
    LOAD_SP,
    LOAD_DL,
    LOAD_PCL,
    LOAD_PCH,

    LOAD_DataOut,

    AluInverseB,

    StatUpdateC,
    StatUpdateZ,
    StatUpdateI,
    StatUpdateD,
    StatOutputB,
    StatUpdateV,
    StatUpdateN,

    StatUseAlu,
    StatCalcZero,

    CtrlSignals_EndMarker
} ctrl_signals;

localparam ctrl_signals_last = CtrlSignals_EndMarker-1;

typedef enum logic[31:0] {
    AluOp_Invalid = 'X,

    AluOp_pass = 0, // Is it really needed?
    AluOp_add,
    AluOp_and,
    AluOp_or,
    AluOp_xor,
    AluOp_shift_left,
    AluOp_shift_right_logical
} alu_control;

typedef enum {
    FlagsCarry = 0,
    FlagsZero = 1,
    FlagsIrqMask = 2,
    FlagsDecimal = 3,
    FlagsBrk = 4,
    FlagsOverflow = 6,
    FlagsNegative = 7
} Flags;

endpackage // control_signals
