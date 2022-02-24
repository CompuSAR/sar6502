`timescale 1ns / 1ps

package control_signals;

typedef enum logic[31:0] {
    CTL_SIG_INVALID = 'X,

    LOAD_A = 0,
    LOAD_X,
    LOAD_Y,
    LOAD_SP,
    PC_LOAD,
    PC_ADVANCE
} ctrl_signals;

localparam ctrl_signals_last = PC_ADVANCE;

endpackage // control_signals
