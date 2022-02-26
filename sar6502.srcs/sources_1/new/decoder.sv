`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:  Some Assembly Required
// Engineer: Shachar Shemesh
// 
// Create Date: 02/23/2022 09:43:53 PM
// Design Name: sar6502
// Module Name: decoder
// Project Name: CompuSAR
// Target Devices: 
// Tool Versions: 
// Description: 6502 decoder
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
// License:
//   Copyright (C) 2022.
//   Copyright owners listed in AUTHORS file.
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
//   This program is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU General Public License for more details.
//
//   You should have received a copy of the GNU General Public License
//   along with this program; if not, write to the Free Software
//   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
// 
//////////////////////////////////////////////////////////////////////////////////

module decoder(
        input [7:0]memory_in,
        input clock,
        input RESET,

        output bus_sources::AddressBusLowSourceCtl address_bus_low_source,
        output bus_sources::AddressBusHighSourceCtl address_bus_high_source,
        output bus_sources::DataBusSourceCtl data_bus_source,
        output bus_sources::InternalBusSourceCtl internal_bus_source,
        output logic [control_signals::ctrl_signals_last:0] ctrl_signals,

        output logic rW,
        output logic sync,
        output logic ML,
        output logic VP
    );

bus_sources::AddressBusLowSourceCtl address_bus_low_source_next;
bus_sources::AddressBusHighSourceCtl address_bus_high_source_next;
bus_sources::DataBusSourceCtl data_bus_source_next;
bus_sources::InternalBusSourceCtl internal_bus_source_next;
logic [control_signals::ctrl_signals_last:0] ctrl_signals_next;

logic rW_next, sync_next, ML_next, VP_next;

localparam MAX_OPCODE_CYCLES = 16;

logic [control_signals::ctrl_signals_last_latched:0] latched_ctrl_signals;
assign ctrl_signals[control_signals::ctrl_signals_last:0] = latched_ctrl_signals;
assign ctrl_signals[control_signals::ctrl_signals_last : control_signals::ctrl_signals_last_latched+1] =
    ctrl_signals_next[control_signals::ctrl_signals_last : control_signals::ctrl_signals_last_latched+1];

localparam
    CycleInvalid = 16'bxxxxxxxx_xxxxxxxx,
    CycleFetch   = 16'b00000000_00000000,
    CycleDecode  = 16'b00000000_00000001,
    CycleAddr1   = 16'b00000000_00000010,
    CycleAddr2   = 16'b00000000_00000100,
    CycleAddr3   = 16'b00000000_00001000,
    CycleAddr4   = 16'b00000000_00010000,
    CycleAddr5   = 16'b00000000_00100000,
    CycleAddr6   = 16'b00000000_01000000,
    CycleAddr7   = 16'b00000000_10000000,
    FirstOpCycle = 16'b00000001_00000000,
    CycleOp2     = 16'b00000010_00000000,
    CycleOp3     = 16'b00000100_00000000,
    CycleOp4     = 16'b00001000_00000000,
    CycleOp5     = 16'b00010000_00000000,
    CycleOp6     = 16'b00100000_00000000,
    CycleOp7     = 16'b01000000_00000000,
    CycleOp8     = 16'b10000000_00000000;
logic [MAX_OPCODE_CYCLES-1:0] op_cycle = CycleFetch, op_cycle_next;

enum logic[31:0] { AddrInvalid = 'X, AddrImplicit=0, AddrAbsolute, AddrZeroPage } active_addr_mode, active_addr_mode_next;
typedef enum logic[31:0] {
    OpInvalid = 'X,

    OpLda = 0,
    OpNop
} operations;
operations active_op, active_op_next;


always_ff@(negedge clock) begin
    if( !RESET ) begin
        next_instruction();
    end else begin
        op_cycle <= op_cycle_next;
        active_addr_mode <= active_addr_mode_next;
        active_op <= active_op_next;

        address_bus_low_source <= address_bus_low_source_next;
        address_bus_high_source <= address_bus_high_source_next;
        data_bus_source <= data_bus_source_next;
        internal_bus_source <= internal_bus_source_next;
        latched_ctrl_signals <= ctrl_signals_next[control_signals::ctrl_signals_last_latched:0];

        rW <= rW_next;
        sync <= sync_next;
        ML <= ML_next;
        VP <= VP_next;
    end
end

task set_invalid_state();
begin
    op_cycle_next = CycleInvalid;
    address_bus_low_source_next = bus_sources::AddrBusLowSrc_Invalid;
    address_bus_high_source_next = bus_sources::AddrBusHighSrc_Invalid;
    data_bus_source_next = bus_sources::DataBusSrc_Invalid;
    internal_bus_source_next = bus_sources::InternalBusSrc_Invalid;

    active_op_next = OpInvalid;
    active_addr_mode_next = AddrInvalid;
end
endtask

always_comb begin
    set_invalid_state();

    ctrl_signals_next = 0;
    rW_next = 1;
    sync_next = 0;
    ML_next = 1;
    VP_next = 1;

    active_op_next = active_op;
    active_addr_mode_next = active_addr_mode;
    op_cycle_next = op_cycle << 1;

    if( op_cycle==CycleFetch ) begin
        op_cycle_next = 1;
        address_bus_low_source_next = bus_sources::AddrBusLowSrc_PC;
        address_bus_high_source_next = bus_sources::AddrBusHighSrc_PC;
        ctrl_signals_next[control_signals::PC_ADVANCE] = 1;
    end else if( op_cycle==CycleDecode )
        do_decode();
    else if( op_cycle<FirstOpCycle )
        do_addr_lookup();
    else
        do_operation(active_op);
end

task do_decode();
    case( memory_in )
        8'ha9: set_addr_mode_immediate( OpLda );
        8'ha5: set_addr_mode_zp( OpLda );
        8'hea: set_addr_mode_implicit( OpNop );
    endcase
endtask

task do_addr_lookup();
begin
    case( active_addr_mode )
        AddrAbsolute: do_addr_mode_absolute();
        AddrZeroPage: do_addr_mode_zp();
        default: set_invalid_state();
    endcase
end
endtask

task set_addr_mode_absolute(operations current_op);
    active_addr_mode_next = AddrAbsolute;
    //address_bus_low_source_next = bus_sources::AddrBusLowSrc_PC;
    //address_bus_high_source_next = bus_sources::AddrBusHighSrc_PC;
    internal_bus_source_next = bus_sources::InternalBusSrc_Mem;
    ctrl_signals_next[control_signals::LOAD_DataLow] = 1;
    ctrl_signals_next[control_signals::PC_ADVANCE] = 1;
endtask

task do_addr_mode_absolute();
begin
    case( op_cycle )
        CycleAddr1: begin
            // XXX address_bus_source_next = bus_sources::AddrBusSrc_PC;
            internal_bus_source_next = bus_sources::InternalBusSrc_Mem;
            ctrl_signals_next[control_signals::LOAD_DataHigh] = 1;
            ctrl_signals_next[control_signals::PC_ADVANCE] = 1;
        end
        CycleAddr2: begin
            // XXX address_bus_source_next = bus_sources::AddrBusSrc_DataLatch;
            set_operation( active_op );
        end
        default: set_invalid_state();
    endcase
end
endtask

task set_addr_mode_immediate(operations current_op);
begin
    ctrl_signals_next[control_signals::PC_ADVANCE] = 1;

    set_operation(current_op);
end
endtask

task set_addr_mode_implicit(operations current_op);
begin
    active_addr_mode_next = AddrImplicit;
    set_operation(current_op);
end
endtask

task set_addr_mode_zp(operations current_op);
begin
    active_op_next = current_op;
    active_addr_mode_next = AddrZeroPage;

    internal_bus_source_next = bus_sources::InternalBusSrc_Mem;
    ctrl_signals_next[control_signals::LOAD_DataLow] = 1;
    ctrl_signals_next[control_signals::PC_ADVANCE] = 1;
    address_bus_low_source_next = bus_sources::AddrBusLowSrc_Internal;
    address_bus_high_source_next = bus_sources::AddrBusHighSrc_Zero;
end
endtask

task do_addr_mode_zp();
begin
    set_operation(active_op);
end
endtask

task set_operation(operations current_op);
    active_op_next = current_op;
    op_cycle_next = FirstOpCycle;

    case( current_op )
        OpLda: do_op_lda_first();
        OpNop: do_op_nop_first();
    endcase
endtask

task next_instruction();
begin
    address_bus_low_source_next = bus_sources::AddrBusLowSrc_PC;
    address_bus_high_source_next = bus_sources::AddrBusHighSrc_PC;
    op_cycle_next = CycleFetch;
    sync_next = 1;
end
endtask

task do_operation(operations current_op);
endtask

task do_op_lda_first();
begin
    next_instruction();
    ctrl_signals_next[control_signals::LOAD_A] = 1;
    data_bus_source_next = bus_sources::DataBusSrc_Mem;
end
endtask

task do_op_nop_first();
begin
    next_instruction();
end
endtask

endmodule
