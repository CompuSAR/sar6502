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

    // Early signals
    DummySmdms
} ctrl_signals;

localparam ctrl_signals_last_latched = PC_ADVANCE;
localparam ctrl_signals_last = DummySmdms;

endpackage // control_signals
