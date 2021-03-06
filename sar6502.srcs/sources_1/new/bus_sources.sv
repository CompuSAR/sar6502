`timescale 1ns / 1ps

package bus_sources;
   
typedef enum logic[31:0] {
    DataBusSrc_Invalid = 'X,
    DataBusSrc_Zero = 0,
    DataBusSrc_Ones,

    DataBusSrc_A,
    DataBusSrc_X,
    DataBusSrc_Y,
    DataBusSrc_SP,
    DataBusSrc_Status,
    DataBusSrc_Alu,
    DataBusSrc_Alu_Latched,

    DataBusSrc_Pc_Low,
    DataBusSrc_Pc_High,

    DataBusSrc_Dl_Low,

    DataBusSrc_Mem,
    DataBusSrc_Mem_Unlatched,

    DataBusSrc_End_Marker
} DataBusSourceCtl;

localparam DataBusSourceCtlLast = DataBusSrc_End_Marker - 1;

typedef enum logic[31:0] {
    AddrBusLowSrc_Invalid = 'X,

    AddrBusLowSrc_Mem = 0,
    AddrBusLowSrc_Alu,
    AddrBusLowSrc_PC,
    AddrBusLowSrc_SP,
    AddrBusLowSrc_DataLatch,
    AddrBusLowSrc_DataLatch_High,

    AddrBusLowSrc_End_Marker
} AddressBusLowSourceCtl;

localparam AddressBusLowSourceCtlLast = AddrBusLowSrc_End_Marker - 1;

typedef enum logic[31:0] {
    AddrBusHighSrc_Invalid = 'X,

    AddrBusHighSrc_Zero = 0,
    AddrBusHighSrc_One,
    AddrBusHighSrc_Mem,
    AddrBusHighSrc_Alu,
    AddrBusHighSrc_Alu_Unlatched,
    AddrBusHighSrc_PC,
    AddrBusHighSrc_DataLatch,

    AddrBusHighSrc_End_Marker
} AddressBusHighSourceCtl;

localparam AddressBusHighSourceCtlLast = AddrBusHighSrc_End_Marker - 1;

typedef enum logic[31:0] {
    PcLowSource_Invalid = 'X,

    PcLowSource_CurrentValue = 0,
    PcLowSource_Mem,
    PcLowSource_Dl,

    PcLowSource_End_Marker
} PcLowSourceCtl;

localparam PcLowSourceCtlLast = PcLowSource_End_Marker - 1;

typedef enum logic[31:0] {
    PcHighSource_Invalid = 'X,

    PcHighSource_CurrentValue = 0,
    PcHighSource_Mem,
    PcHighSource_Dl,
    PcHighSource_Alu_Latched,

    PcHighSource_End_Marker
} PcHighSourceCtl;

localparam PcHighSourceCtlLast = PcHighSource_End_Marker - 1;

typedef enum logic[31:0] {
    DataLatchLowSource_Invalid = 'X,

    DataLatchLowSource_Mem = 0,
    DataLatchLowSource_Alu,
    DataLatchLowSource_Alu_Latched,
    DataLatchLowSource_PC,

    DataLatchLowSource_FA,
    DataLatchLowSource_FC,
    DataLatchLowSource_FE,

    DataLatchLowSource_End_Marker
} DataLatchLowSourceCtl;

localparam DataLatchLowSourceCtlLast = DataLatchLowSource_End_Marker - 1;

typedef enum logic[31:0] {
    DataLatchHighSource_Invalid = 'X,

    DataLatchHighSource_Zero = 0,
    DataLatchHighSource_Mem,
    DataLatchHighSource_Alu,
    DataLatchHighSource_Alu_Latched,
    DataLatchHighSource_PC,

    DataLatchHighSource_FF,

    DataLatchHighSource_End_Marker
} DataLatchHighSourceCtl;

localparam DataLatchHighSourceCtlLast = DataLatchHighSource_End_Marker - 1;

typedef enum logic[31:0] {
    StackPointerSource_Invalid = 'X,

    StackPointerSource_Alu = 0,
    StackPointerSource_DataBus,

    StackPointerSource_End_Marker
} StackPointerSourceCtl;

localparam StackPointerSourceCtlLast = StackPointerSource_End_Marker -1;

typedef enum logic[31:0] {
    AluASourceCtl_Invalid = 'X,

    AluASourceCtl_Zero = 0,
    AluASourceCtl_A,
    AluASourceCtl_X,
    AluASourceCtl_Y,
    AluASourceCtl_DataLatchLow,
    AluASourceCtl_DataLatchHigh,
    AluASourceCtl_SP,
    AluASourceCtl_PC_Low,
    AluASourceCtl_PC_High,
    AluASourceCtl_Mem,
    AluASourceCtl_Alu,

    AluASourceCtl_End_Marker
} AluASourceCtl;

localparam AluASourceCtlLast = AluASourceCtl_End_Marker - 1;

typedef enum logic[31:0] {
    AluBSourceCtl_Invalid = 'X,

    AluBSourceCtl_Zero = 0,
    AluBSourceCtl_Bit0,
    AluBSourceCtl_Bit1,
    AluBSourceCtl_Bit2,
    AluBSourceCtl_Bit3,
    AluBSourceCtl_Bit4,
    AluBSourceCtl_Bit5,
    AluBSourceCtl_Bit6,
    AluBSourceCtl_Bit7,
    AluBSourceCtl_Mem,
    AluBSourceCtl_A,

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
