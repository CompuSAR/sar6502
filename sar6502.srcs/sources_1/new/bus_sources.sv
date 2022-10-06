`timescale 1ns / 1ps

package bus_sources;
   
typedef enum logic[31:0] {
    DataBusSrc_Invalid = 'X,
    DataBusSrc_Zero = 0,
    DataBusSrc_One,

    DataBusSrc_End_Marker
} DataBusSourceCtl;

localparam DataBusSourceCtlLast = DataBusSrc_End_Marker - 1;

typedef enum logic[31:0] {
    AddrBusLowSrc_Invalid = 'X,
    AddrBusLowSrc_PC = 0,
    AddrBusLowSrc_DL,

    AddrBusLowSrc_F8 = 'hF8,
    AddrBusLowSrc_F9,
    AddrBusLowSrc_FA,
    AddrBusLowSrc_FB,
    AddrBusLowSrc_FC,
    AddrBusLowSrc_FD,
    AddrBusLowSrc_FE,
    AddrBusLowSrc_FF,

    AddrBusLowSrc_End_Marker
} AddressBusLowSourceCtl;

localparam AddressBusLowSourceCtlLast = AddrBusLowSrc_End_Marker - 1;

typedef enum logic[31:0] {
    AddrBusHighSrc_Invalid = 'X,
    AddrBusHighSrc_PC = 0,
    AddrBusHighSrc_One,
    AddrBusHighSrc_DataIn,
    AddrBusHighSrc_FF,

    AddrBusHighSrc_End_Marker
} AddressBusHighSourceCtl;

localparam AddressBusHighSourceCtlLast = AddrBusHighSrc_End_Marker - 1;

typedef enum logic[31:0] {
    SpecialBusSrc_Invalid = 'X,
    SpecialBusSrc_RegA = 0,
    SpecialBusSrc_RegX,
    SpecialBusSrc_RegY,
    SpecialBusSrc_RegS,

    SpecialBusSrc_Mem,

    SpecialBusSrc_End_Marker
} SpecialBusSourceCtl;

localparam SpecialBusSourceCtlLast = SpecialBusSrc_End_Marker - 1;

typedef enum logic {
    PcLowSrc_Invalid = 'X,

    PcLowSrc_DataLatch = 1'b0,
    PcLowSrc_Incrementor
} PcLowSourceCtl;

typedef enum logic {
    PcHighSrc_Invalid = 'X,

    PcHighSrc_AddressBus = 1'b0,
    PcHighSrc_Incrementor
} PcHighSourceCtl;

endpackage // bus_sources
