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

module decoder#(parameter CPU_VARIANT = 0)
(
    input clock,
    input reset,
    input ready,

    input [7:0]memory_in,

    output bus_sources::AddressBusLowSourceCtl addr_bus_low_src,
    output bus_sources::AddressBusHighSourceCtl addr_bus_high_src,
    output bus_sources::DataBusSourceCtl data_bus_src,
    output bus_sources::SpecialBusSourceCtl special_bus_src,
    output bus_sources::PcLowSourceCtl pcl_bus_src,
    output bus_sources::PcHighSourceCtl pch_bus_src,

    output control_signals::alu_control alu_op,
    /*
        input [7:0]status,
        input alu_carry,
        input clock,
        input RESET,
        input IRQ,
        input NMI,

        input ready,

        output bus_sources::DataLatchLowSourceCtl data_latch_low_source,
        output bus_sources::DataLatchHighSourceCtl data_latch_high_source,
        output bus_sources::StackPointerSourceCtl stack_pointer_source,
        output bus_sources::AluASourceCtl alu_a_source,
        output bus_sources::AluBSourceCtl alu_b_source,
        output bus_sources::AluCarrySourceCtl alu_carry_source,

    */
    output logic[control_signals::ctrl_signals_last:0] ctrl_signals,

    output logic sync,
    output logic write,
    output logic memory_lock,
    output logic vector_pull,

    output logic incompatible
);

localparam MAX_OPCODE_CYCLES = 16;

localparam
    CycleInvalid = 16'bxxxxxxxx_xxxxxxxx,
    CycleDecode  = 16'b00000000_00000000,
    CycleAddr1  = 16'b00000000_00000001,
    CycleAddr2   = 16'b00000000_00000010,
    CycleAddr3   = 16'b00000000_00000100,
    CycleAddr4   = 16'b00000000_00001000,
    CycleAddr5   = 16'b00000000_00010000,
    CycleAddr6   = 16'b00000000_00100000,
    CycleAddr7   = 16'b00000000_01000000,
    CycleAddr8   = 16'b00000000_10000000,
    FirstOpCycle = 16'b00000001_00000000,
    CycleOp2     = 16'b00000010_00000000,
    CycleOp3     = 16'b00000100_00000000,
    CycleOp4     = 16'b00001000_00000000,
    CycleOp5     = 16'b00010000_00000000,
    CycleOp6     = 16'b00100000_00000000,
    CycleOp7     = 16'b01000000_00000000,
    CycleOp8     = 16'b10000000_00000000;
logic [MAX_OPCODE_CYCLES-1:0] op_cycle = FirstOpCycle, op_cycle_next;
logic [7:0] current_opcode = 8'hdb, next_opcode;

enum { IntStateNone, IntStateReset, IntStateNmi, IntStateIrq } int_state = IntStateReset, int_state_next;

always_ff@(posedge clock) begin
    if( ready ) begin
        op_cycle <= op_cycle_next;
        current_opcode <= next_opcode;
        int_state <= int_state_next;
    end
end

task set_invalid_state();
begin
    op_cycle_next = CycleInvalid;
    next_opcode = 8'hXX;
    incompatible = 1'bX;

    addr_bus_low_src = bus_sources::AddrBusLowSrc_Invalid;
    addr_bus_high_src = bus_sources::AddrBusHighSrc_Invalid;
    data_bus_src = bus_sources::DataBusSrc_Invalid;
    special_bus_src = bus_sources::SpecialBusSrc_Invalid;
    pcl_bus_src = bus_sources::PcLowSrc_Invalid;
    pch_bus_src = bus_sources::PcHighSrc_Invalid;

    ctrl_signals = { control_signals::ctrl_signals_last+1{1'bX} };

    sync = 1'bX;
    write = 1'bX;
    memory_lock = 1'bX;
    vector_pull = 1'bX;
end
endtask

task advance_cycle();
    op_cycle_next = op_cycle<<1;
endtask

task set_default_state();
begin
    set_invalid_state();

    next_opcode = current_opcode;
    int_state_next = int_state;
    incompatible = 1'b0;
    advance_cycle();

    ctrl_signals = { control_signals::ctrl_signals_last+1{1'b0} };

    sync = 1'b0;
    write = 1'b0;
    memory_lock = 1'b0;
    vector_pull = 1'b0;
end
endtask

always_comb begin
    set_default_state();

    if( reset ) begin
        op_cycle_next = CycleDecode;
        next_opcode = 8'h00;    // BRK instruction
        int_state_next = IntStateReset;
    end else if( op_cycle==CycleDecode ) begin
        if( int_state!=IntStateNone ) begin
            // Interrupt pending
            int_state_next = IntStateNone;
            next_opcode = 8'h00;
            op_cycle_next = CycleAddr1;
        end else begin
            do_decode();
        end
    end else
        do_opcode();
end

task do_decode();
    case(memory_in)
        8'h00: op_brk_decode();
        default: set_invalid_state();
    endcase
endtask

task do_opcode();
    case(current_opcode)
        8'h00: op_brk();
        8'hdb: do_stp();
        default: set_invalid_state();
    endcase
endtask

task next_instruction();
    sync = 1'b1;
    addr_bus_low_src = bus_sources::AddrBusLowSrc_PC;
    addr_bus_high_src = bus_sources::AddrBusHighSrc_PC;
    ctrl_signals[control_signals::LOAD_PCL] = 1'b1;
    ctrl_signals[control_signals::LOAD_PCH] = 1'b1;
    pcl_bus_src = bus_sources::PcLowSrc_Incrementor;
    pch_bus_src = bus_sources::PcHighSrc_Incrementor;

    op_cycle_next = CycleDecode;
endtask

task op_brk_decode();
endtask

task op_brk();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_FC;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_FF;
            vector_pull = 1'b1;
        end
        CycleOp2: begin
            ctrl_signals[control_signals::LOAD_DL] = 1'b1;

            addr_bus_low_src = bus_sources::AddrBusLowSrc_FD;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_FF;
            vector_pull = 1'b1;
        end
        CycleOp3: begin
            next_instruction();

            addr_bus_low_src = bus_sources::AddrBusLowSrc_DL;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_DataIn;
        end
    endcase
endtask

task do_stp();
    op_cycle_next = FirstOpCycle;
endtask

endmodule
