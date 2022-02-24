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

localparam FIRST_OPERATION_CYCLE = 8;
localparam MAX_OPCODE_CYCLES = 16;

logic [MAX_OPCODE_CYCLES-1:0]op_cycle = 0, op_cycle_next;
enum { AddrImplicit } active_addr_mode, active_addr_mode_next;
typedef enum { OpNop } operations;
operations active_op, active_op_next;

always_ff@(negedge clock) begin
    if( !RESET ) begin
        op_cycle <= 0;
    end else begin
        op_cycle <= op_cycle_next;
        active_addr_mode <= active_addr_mode_next;
        active_op <= active_op_next;
    end
end

task set_invalid_state();
begin
    op_cycle_next = { MAX_OPCODE_CYCLES{1'bX} };
    data_bus_source = bus_sources::DataBusSrc_Invalid;
    address_bus_source = bus_sources::AddrBusSrc_Invalid;
end
endtask

always_comb begin
    ctrl_signals = 0;
    rW = 1;
    sync = 0;
    ML = 1;
    VP = 1;

    op_cycle_next = op_cycle << 1;
    if( op_cycle==0 ) begin
        // Instruction fetch cycle
        op_cycle_next = 1;
        sync = 1;
        address_bus_source = bus_sources::AddrBusSrc_PC;
        ctrl_signals[control_signals::PC_ADVANCE] = 1;
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
        8'hea: begin
            active_addr_mode_next = AddrImplicit;
            do_operation( OpNop );
        end
    endcase
endtask

task do_addr_lookup();
endtask

task do_operation(operations current_op);
    case( current_op )
        OpNop: do_op_nop();
    endcase
endtask

task next_instruction();
begin
    address_bus_source = bus_sources::AddrBusSrc_PC;
    op_cycle_next = 0;
end
endtask

task do_op_nop();
begin
    next_instruction();
end
endtask

endmodule
