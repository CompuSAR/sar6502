`timescale 1ns / 1ps

package bus_sources;
   
typedef enum logic[31:0] {
    DataBusSrc_Invalid = 'X,
    DataBusSrc_Zero = 0,
    DataBusSrc_Ones,
    DataBusSrc_RegA,
    DataBusSrc_Mem,
    DataBusSrc_Alu,
    DataBusSrc_Special,
    DataBusSrc_PcLow,
    DataBusSrc_PcHigh,
    DataBusSrc_Bit0,
    DataBusSrc_Bit1,
    DataBusSrc_Bit2,
    DataBusSrc_Bit3,
    DataBusSrc_Bit4,
    DataBusSrc_Bit5,
    DataBusSrc_Bit6,
    DataBusSrc_Bit7
} DataBusSourceCtl;

typedef enum logic[31:0] {
    AddrBusLowSrc_Invalid = 'X,
    AddrBusLowSrc_PC = 0,
    AddrBusLowSrc_Mem,
    AddrBusLowSrc_DL,
    AddrBusLowSrc_OL,
    AddrBusLowSrc_SP,
    AddrBusLowSrc_ALU,
    AddrBusLowSrc_ALU_Latched,

    AddrBusLowSrc_F8 = 'hF8,
    AddrBusLowSrc_F9,
    AddrBusLowSrc_FA,
    AddrBusLowSrc_FB,
    AddrBusLowSrc_FC,
    AddrBusLowSrc_FD,
    AddrBusLowSrc_FE,
    AddrBusLowSrc_FF
} AddressBusLowSourceCtl;

typedef enum logic[31:0] {
    AddrBusHighSrc_Invalid = 'X,
    AddrBusHighSrc_Zero = 0,
    AddrBusHighSrc_One,
    AddrBusHighSrc_PC,
    AddrBusHighSrc_Mem,
    AddrBusHighSrc_ALU,
    AddrBusHighSrc_OL,
    AddrBusHighSrc_FF
} AddressBusHighSourceCtl;

typedef enum logic[31:0] {
    SpecialBusSrc_Invalid = 'X,
    SpecialBusSrc_RegA = 0,
    SpecialBusSrc_RegX,
    SpecialBusSrc_RegY,
    SpecialBusSrc_RegSP,

    SpecialBusSrc_Mem
} SpecialBusSourceCtl;

typedef enum logic[31:0] {
    AluASrc_Invalid = 'X,

    AluASrc_RegA = 0,
    AluASrc_RegX,
    AluASrc_RegY,
    AluASrc_RegSp,
    AluASrc_PcLow,
    AluASrc_PcHigh,
    AluASrc_Mem,
    AluASrc_ALU
} AluASrcCtl;

typedef enum logic {
    AluBSrc_Invalid = 'X,

    AluBSrc_Zero = 1'b0,
    AluBSrc_DataBus
} AluBSrcCtl;

typedef enum logic[31:0] {
    PcLowSrc_Invalid = 'X,

    PcLowSrc_Mem = 0,
    PcLowSrc_ALU,
    PcLowSrc_Incrementor
} PcLowSourceCtl;

typedef enum logic[31:0] {
    PcHighSrc_Invalid = 'X,

    PcHighSrc_Mem = 0,
    PcHighSrc_Incrementor
} PcHighSourceCtl;

typedef enum logic {
    PcNextSrc_Invalid = 'X,

    PcNextSrc_Pc = 1'b0,
    PcNextSrc_Bus
} PcNextSourceCtl;

typedef enum logic[1:0] {
    DataOutSrc_Invalid = 'X,

    DataOutSrc_DataBus = 2'b0,
    DataOutSrc_Status,
    DataOutSrc_Alu              // Use only for TRB/TSB commands.
} DataOutSourceCtl;

endpackage // bus_sources
