`timescale 1ns / 1ps

package bus_sources;
   
typedef enum logic[31:0] {
    DataBusSrc_Invalid = 'X,
    DataBusSrc_Zero = 0,

    DataBusSrc_A,
    DataBusSrc_X,
    DataBusSrc_Y,
    DataBusSrc_SP,

    DataBusSrc_Mem
} data_bus_sources_ctl;

localparam data_bus_sources_ctl_last = DataBusSrc_Mem;

typedef enum logic[31:0] {
    AddrBusSrc_Invalid = 'X,

    AddrBusSrc_PC = 0,
    AddrBusSrc_SP
} address_bus_sources_ctl;

localparam address_bus_sources_ctl_last = AddrBusSrc_SP;

endpackage // bus_sources
