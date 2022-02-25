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

        output bus_sources::address_bus_sources_ctl address_bus_source,
        output bus_sources::data_bus_sources_ctl data_bus_source,
        output logic [control_signals::ctrl_signals_last:0] ctrl_signals,

        output logic rW,
        output logic sync,
        output logic ML,
        output logic VP
    );

bus_sources::address_bus_sources_ctl address_bus_source_next;
bus_sources::data_bus_sources_ctl data_bus_source_next;
logic [control_signals::ctrl_signals_last:0] ctrl_signals_next;

logic rW_next, sync_next, ML_next, VP_next;

localparam FIRST_OPERATION_CYCLE = 8;
localparam MAX_OPCODE_CYCLES = 16;

logic [control_signals::ctrl_signals_last_latched:0] latched_ctrl_signals;
assign ctrl_signals[control_signals::ctrl_signals_last:0] = latched_ctrl_signals;
assign ctrl_signals[control_signals::ctrl_signals_last : control_signals::ctrl_signals_last_latched+1] =
    ctrl_signals_next[control_signals::ctrl_signals_last : control_signals::ctrl_signals_last_latched+1];

logic [MAX_OPCODE_CYCLES-1:0]op_cycle = 0, op_cycle_next;
enum { AddrImplicit } active_addr_mode, active_addr_mode_next;
typedef enum {
    OpLda,
    OpNop
} operations;
operations active_op, active_op_next;


always_ff@(negedge clock) begin
    if( !RESET ) begin
        op_cycle <= 0;
    end else begin
        op_cycle <= op_cycle_next;
        active_addr_mode <= active_addr_mode_next;
        active_op <= active_op_next;

        address_bus_source <= address_bus_source_next;
        data_bus_source <= data_bus_source_next;
        latched_ctrl_signals <= ctrl_signals_next[control_signals::ctrl_signals_last_latched:0];

        rW <= rW_next;
        sync <= sync_next;
        ML <= ML_next;
        VP <= VP_next;
    end
end

task set_invalid_state();
begin
    op_cycle_next = { MAX_OPCODE_CYCLES{1'bX} };
    data_bus_source_next = bus_sources::DataBusSrc_Invalid;
    address_bus_source_next = bus_sources::AddrBusSrc_Invalid;
end
endtask

always_comb begin
    set_invalid_state();

    ctrl_signals_next = 0;
    rW_next = 1;
    sync_next = 0;
    ML_next = 1;
    VP_next = 1;

    op_cycle_next = op_cycle << 1;
    if( op_cycle==0 ) begin
        // Instruction fetch cycle
        op_cycle_next = 1;
        address_bus_source_next = bus_sources::AddrBusSrc_PC;
        ctrl_signals_next[control_signals::PC_ADVANCE] = 1;
    end else if( op_cycle == 1 ) begin
        do_decode();
    end else if( op_cycle < (1<<FIRST_OPERATION_CYCLE) ) begin
        do_addr_lookup();
    end else begin
        do_operation(active_op);
    end
end

task do_decode();
    case( memory_in )
        8'ha9: begin
            set_addr_mode_immediate(OpLda);
        end
        8'hea: begin
            set_addr_mode_implicit( OpNop );
        end
    endcase
endtask

task do_addr_lookup();
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

task set_operation(operations current_op);
    active_op_next = current_op;
    op_cycle_next = FIRST_OPERATION_CYCLE;

    case( current_op )
        OpLda: do_op_lda_first();
        OpNop: do_op_nop_first();
    endcase
endtask

task next_instruction();
begin
    address_bus_source_next = bus_sources::AddrBusSrc_PC;
    op_cycle_next = 0;
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
