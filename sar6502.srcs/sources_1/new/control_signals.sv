`timescale 1ns / 1ps

package control_signals;

typedef enum logic[31:0] {
    CTL_SIG_INVALID = 'X,

    LOAD_A = 0,
    LOAD_X,
    LOAD_Y,
    LOAD_SP,
    LOAD_DataLow,
    LOAD_DataHigh,
    PC_LOAD,
    PC_ADVANCE,

    UpdateFlagC,
    UpdateFlagZ,
    UpdateFlagI,
    UpdateFlagD,
    OutputFlagB,
    UpdateFlagV,
    UpdateFlagN,

    UseAluFlags,
    CalculateFlagZ,

    // Early signals
    EndMarker
} ctrl_signals;

localparam ctrl_signals_last_latched = EndMarker-1;
localparam ctrl_signals_last = EndMarker-1;

typedef enum logic[31:0] {
    AluOp_INVALID = 'X,

    AluOp_pass = 0,
    AluOp_add,
    AluOp_and,
    AluOp_or,
    AluOp_xor,
    AluOp_shift_left,
    AluOp_shift_right_logical,
    AluOp_shift_right_arithmetic
} alu_control;

endpackage // control_signals
