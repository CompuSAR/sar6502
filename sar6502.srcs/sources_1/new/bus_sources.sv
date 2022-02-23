package bus_sources;
   
typedef enum {
    DataBusSrc_A,
    DataBusSrc_X,
    DataBusSrc_Y,
    DataBusSrc_SP,

    DataBusSrc_Mem
} data_bus_sources_ctl;

typedef enum {
    AddrBusSrc_PC,
    AddrBusSrc_SP
} address_bus_sources_ctl;

endpackage // bus_sources
