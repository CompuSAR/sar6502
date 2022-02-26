`timescale 1ns / 1ps

package bus_sources;
   
typedef enum logic[31:0] {
    DataBusSrc_Invalid = 'X,
    DataBusSrc_Zero = 0,

    DataBusSrc_A,
    DataBusSrc_X,
    DataBusSrc_Y,
    DataBusSrc_SP,

    DataBusSrc_Mem,

    DataBusSrc_End_Marker
} DataBusSourceCtl;

localparam DataBusSourceCtlLast = DataBusSrc_End_Marker - 1;

typedef enum logic[31:0] {
    AddrBusLowSrc_Invalid = 'X,

    AddrBusLowSrc_PC = 0,
    AddrBusLowSrc_SP,

    AddrBusLowSrc_End_Marker
} AddressBusLowSourceCtl;

localparam AddressBusLowSourceCtlLast = AddrBusLowSrc_End_Marker - 1;

typedef enum logic[31:0] {
    AddrBusHighSrc_Invalid = 'X,

    AddrBusHighSrc_Zero = 0,
    AddrBusHighSrc_One,
    AddrBusHighSrc_PC,

    AddrBusHighSrc_End_Marker
} AddressBusHighSourceCtl;

localparam AddressBusHighSourceCtlLast = AddrBusHighSrc_End_Marker - 1;

typedef enum logic[31:0] {
    InternalBusSrc_Invalid = 'X,

    InternalBusSrc_Mem = 0,
    InternalBusSrc_PcLow,
    InternalBusSrc_PcHigh,
    InternalBusSrc_DataLatchLow,
    InternalBusSrc_DataLatchHigh,

    InternalBusSrc_End_Marker
} InternalBusSourceCtl;

localparam InternalBusSourceCtlLast = InternalBusSrc_End_Marker - 1;

endpackage // bus_sources
