`timescale 1ns / 1ps

package load_signals;

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
    UpdateFlagV,
    UpdateFlagN,

    LoadSignals_EndMarker
} ld_signals;

localparam ld_signals_last = LoadSignals_EndMarker-1;

endpackage // load_signals
