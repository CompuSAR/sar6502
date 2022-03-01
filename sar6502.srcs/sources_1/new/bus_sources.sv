`timescale 1ns / 1ps

package bus_sources;
   
typedef enum logic[31:0] {
    DataBusSrc_Invalid = 'X,
    DataBusSrc_Zero = 0,

    DataBusSrc_A,
    DataBusSrc_X,
    DataBusSrc_Y,
    DataBusSrc_SP,
    DataBusSrc_Status,
    DataBusSrc_Alu,

    DataBusSrc_Mem,

    DataBusSrc_End_Marker
} DataBusSourceCtl;

localparam DataBusSourceCtlLast = DataBusSrc_End_Marker - 1;

typedef enum logic[31:0] {
    AddrBusLowSrc_Invalid = 'X,

    AddrBusLowSrc_PC = 0,
    AddrBusLowSrc_SP,
    AddrBusLowSrc_Internal,
    AddrBusLowSrc_DataLatch,

    AddrBusLowSrc_End_Marker
} AddressBusLowSourceCtl;

localparam AddressBusLowSourceCtlLast = AddrBusLowSrc_End_Marker - 1;

typedef enum logic[31:0] {
    AddrBusHighSrc_Invalid = 'X,

    AddrBusHighSrc_Zero = 0,
    AddrBusHighSrc_One,
    AddrBusHighSrc_PC,
    AddrBusHighSrc_Internal,

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

    InternalBusSrc_A,
    InternalBusSrc_Alu,

    InternalBusSrc_End_Marker
} InternalBusSourceCtl;

localparam InternalBusSourceCtlLast = InternalBusSrc_End_Marker - 1;

typedef enum logic[31:0] {
    AluBSourceCtl_Invalid = 'X,

    AluBSourceCtl_DataBus = 0,
    AluBSourceCtl_Mem,

    AluBSourceCtl_End_Marker
} AluBSourceCtl;

localparam AluBSourceCtlLast = AluBSourceCtl_End_Marker - 1;

typedef enum logic[31:0] {
    AluCarrySource_Invalid = 'X,

    AluCarrySource_Zero = 0,
    AluCarrySource_One,
    AluCarrySource_Carry,

    AluCarrySource_End_Marker
} AluCarrySourceCtl;

localparam AluCarrySourceCtlLast = AluCarrySource_End_Marker - 1;

endpackage // bus_sources
