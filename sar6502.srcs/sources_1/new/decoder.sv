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
        output bus_sources::PcLowSourceCtl pc_low_source,
        output bus_sources::PcHighSourceCtl pc_high_source,
        output bus_sources::DataLatchLowSourceCtl data_latch_low_source,
        output bus_sources::DataLatchHighSourceCtl data_latch_high_source,
        output bus_sources::StackPointerSourceCtl stack_pointer_source,
        output control_signals::alu_control alu_op,
        output bus_sources::AluASourceCtl alu_a_source,
        output bus_sources::AluBSourceCtl alu_b_source,
        output bus_sources::AluCarrySourceCtl alu_carry_source,
        output logic [control_signals::ctrl_signals_last:0] ctrl_signals,

        output logic rW,
        output logic sync,
        output logic ML,
        output logic VP
    );

bus_sources::AddressBusLowSourceCtl address_bus_low_source_next;
bus_sources::AddressBusHighSourceCtl address_bus_high_source_next;
bus_sources::DataBusSourceCtl data_bus_source_next;
bus_sources::PcLowSourceCtl pc_low_source_next;
bus_sources::PcHighSourceCtl pc_high_source_next;
bus_sources::DataLatchLowSourceCtl data_latch_low_source_next;
bus_sources::DataLatchHighSourceCtl data_latch_high_source_next;
bus_sources::StackPointerSourceCtl stack_pointer_source_next;
bus_sources::AluASourceCtl alu_a_source_next;
bus_sources::AluBSourceCtl alu_b_source_next;
control_signals::alu_control alu_op_next;
bus_sources::AluCarrySourceCtl alu_carry_source_next;
logic [control_signals::ctrl_signals_last_latched : 0] ctrl_signals_next;

logic rW_next, sync_next, ML_next, VP_next;

localparam MAX_OPCODE_CYCLES = 16;

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

    OpNone = 0,
    OpAdc,
    OpBrk,
    OpClc,
    OpLda,
    OpLdx,
    OpNop,
    OpPha,
    OpRti,
    OpTxs
} operations;
operations active_op, active_op_next;

enum { IntStateNone, IntStateReset, IntStateNmi, IntStateIrq } int_state, int_state_next;

always_ff@(negedge clock) begin
    if( !RESET ) begin
        active_op <= OpInvalid;
        active_addr_mode <= AddrInvalid;
        op_cycle <= CycleFetch;
        int_state <= IntStateReset;
    end else begin
        op_cycle <= op_cycle_next;
        active_addr_mode <= active_addr_mode_next;
        active_op <= active_op_next;
        int_state <= int_state_next;

        address_bus_low_source <= address_bus_low_source_next;
        address_bus_high_source <= address_bus_high_source_next;
        data_bus_source <= data_bus_source_next;
        pc_low_source <= pc_low_source_next;
        pc_high_source <= pc_high_source_next;
        data_latch_low_source <= data_latch_low_source_next;
        data_latch_high_source <= data_latch_high_source_next;
        stack_pointer_source <= stack_pointer_source_next;
        alu_a_source <= alu_a_source_next;
        alu_b_source <= alu_b_source_next;
        alu_carry_source <= alu_carry_source_next;
        alu_op <= alu_op_next;
        ctrl_signals[control_signals::ctrl_signals_last_latched : 0] <=
            ctrl_signals_next[control_signals::ctrl_signals_last_latched : 0];

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
    pc_low_source_next = bus_sources::PcLowSource_Invalid;
    pc_high_source_next = bus_sources::PcHighSource_Invalid;
    data_latch_low_source_next = bus_sources::DataLatchLowSource_Invalid;
    data_latch_high_source_next = bus_sources::DataLatchHighSource_Invalid;
    stack_pointer_source_next = bus_sources::StackPointerSource_Invalid;
    alu_a_source_next = bus_sources::AluASourceCtl_Invalid;
    alu_b_source_next = bus_sources::AluBSourceCtl_Invalid;
    alu_op_next = control_signals::AluOp_INVALID;
    alu_carry_source_next = bus_sources::AluCarrySource_Invalid;
    ctrl_signals_next = { control_signals::ctrl_signals_last_latched+1 {1'bx} };
    ctrl_signals[control_signals::ctrl_signals_last : control_signals::ctrl_signals_last_latched] = 'X;

    active_op_next = OpInvalid;
    active_addr_mode_next = AddrInvalid;
end
endtask

always_comb begin
    set_invalid_state();

    ctrl_signals_next = 0;
    ctrl_signals[control_signals::ctrl_signals_last : control_signals::ctrl_signals_last_latched] = 0;
    rW_next = 1;
    sync_next = 0;
    ML_next = 1;
    VP_next = 1;
    int_state_next = int_state;

    active_op_next = active_op;
    active_addr_mode_next = active_addr_mode;
    op_cycle_next = op_cycle << 1;

    if( !RESET ) begin
        int_state_next = IntStateReset;
    end else if( op_cycle==CycleFetch ) begin
        op_cycle_next = 1;
        address_bus_low_source_next = bus_sources::AddrBusLowSrc_PC;
        address_bus_high_source_next = bus_sources::AddrBusHighSrc_PC;

        if( int_state==IntStateNone )
            ctrl_signals[control_signals::PC_ADVANCE] = 1;
        else
            set_operation(OpBrk);
    end else if( op_cycle==CycleDecode )
        do_decode();
    else if( op_cycle<FirstOpCycle )
        do_addr_lookup();
    else
        do_operation();
end

task do_decode();
    case( memory_in )
        8'h18: set_addr_mode_implicit( OpClc );
        8'h40: set_addr_mode_implicit( OpRti );
        8'h48: set_addr_mode_implicit( OpPha );
        8'h6d: set_addr_mode_absolute( OpAdc );
        8'h9a: set_addr_mode_implicit( OpTxs );
        8'ha2: set_addr_mode_immediate( OpLdx );
        8'ha9: set_addr_mode_immediate( OpLda );
        8'ha5: set_addr_mode_zp( OpLda );
        8'had: set_addr_mode_absolute( OpLda );
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
    active_op_next = current_op;
    active_addr_mode_next = AddrAbsolute;

    data_latch_low_source_next = bus_sources::DataLatchLowSource_Mem;
    ctrl_signals_next[control_signals::LOAD_DataLow] = 1;
    ctrl_signals[control_signals::PC_ADVANCE] = 1;

    address_bus_low_source_next = bus_sources::AddrBusLowSrc_PC;
    address_bus_high_source_next = bus_sources::AddrBusHighSrc_PC;
endtask

task do_addr_mode_absolute();
begin
    case( op_cycle )
        CycleAddr1: begin
            data_latch_high_source_next = bus_sources::DataLatchHighSource_Mem;
            ctrl_signals_next[control_signals::LOAD_DataHigh] = 1;
            ctrl_signals[control_signals::PC_ADVANCE] = 1;

            address_bus_low_source_next = bus_sources::AddrBusLowSrc_DataLatch;
            address_bus_high_source_next = bus_sources::AddrBusHighSrc_DataLatch;
        end
        CycleAddr2: begin
            set_operation( active_op );
        end
        default: set_invalid_state();
    endcase
end
endtask

task set_addr_mode_immediate(operations current_op);
begin
    ctrl_signals[control_signals::PC_ADVANCE] = 1;

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

    data_latch_low_source_next = bus_sources::DataLatchLowSource_Mem;
    ctrl_signals_next[control_signals::LOAD_DataLow] = 1;
    data_latch_high_source_next = bus_sources::DataLatchHighSource_Zero;
    ctrl_signals_next[control_signals::LOAD_DataHigh] = 1;

    ctrl_signals[control_signals::PC_ADVANCE] = 1;

    address_bus_low_source_next = bus_sources::AddrBusLowSrc_Mem;
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
        OpAdc: do_op_adc_first();
        OpBrk: do_op_brk_first();
        OpClc: do_op_clc_first();
        OpLda: do_op_lda_first();
        OpLdx: do_op_ldx_first();
        OpNop: do_op_nop_first();
        OpPha: do_op_pha_first();
        OpRti: do_op_rti_first();
        OpTxs: do_op_txs_first();
        default: set_invalid_state();
    endcase
endtask

task do_operation();
begin
    case( active_op )
        OpBrk: do_op_brk();
        OpPha: do_op_pha();
        OpRti: do_op_rti();
        default: set_invalid_state();
    endcase
end
endtask

task next_instruction();
begin
    address_bus_low_source_next = bus_sources::AddrBusLowSrc_PC;
    address_bus_high_source_next = bus_sources::AddrBusHighSrc_PC;
    op_cycle_next = CycleFetch;
    sync_next = 1;
    active_op_next = OpNone;
end
endtask

task do_op_adc_first();
begin
    next_instruction();
    alu_a_source_next = bus_sources::AluASourceCtl_A;
    alu_b_source_next = bus_sources::AluBSourceCtl_Mem;
    alu_carry_source_next = bus_sources::AluCarrySource_Carry;
    alu_op_next = control_signals::AluOp_add;
    data_bus_source_next = bus_sources::DataBusSrc_Alu;

    ctrl_signals_next[control_signals::UpdateFlagN] = 1;
    ctrl_signals_next[control_signals::UpdateFlagV] = 1;
    ctrl_signals_next[control_signals::UpdateFlagZ] = 1;
    ctrl_signals_next[control_signals::UpdateFlagC] = 1;
    ctrl_signals_next[control_signals::UseAluFlags] = 1;
    ctrl_signals_next[control_signals::CalculateFlagZ] = 1;
    ctrl_signals_next[control_signals::LOAD_A] = 1;
end
endtask

task do_op_brk_first();
begin
    data_latch_high_source_next = bus_sources::DataLatchHighSource_FF;
    ctrl_signals_next[control_signals::LOAD_DataHigh] = 1;

    case(int_state)
        IntStateNone: data_latch_low_source_next = bus_sources::DataLatchLowSource_FE;
        IntStateReset: data_latch_low_source_next = bus_sources::DataLatchLowSource_FC;
        IntStateNmi: data_latch_low_source_next = bus_sources::DataLatchLowSource_FA;
        IntStateIrq: data_latch_low_source_next = bus_sources::DataLatchLowSource_FE;
    endcase
    ctrl_signals_next[control_signals::LOAD_DataLow] = 1;

    address_bus_high_source_next = bus_sources::AddrBusHighSrc_PC;
    address_bus_low_source_next = bus_sources::AddrBusLowSrc_PC;
end
endtask

task do_op_brk();
begin
    case(op_cycle)
        FirstOpCycle: begin

            address_bus_high_source_next = bus_sources::AddrBusHighSrc_One;
            address_bus_low_source_next = bus_sources::AddrBusLowSrc_SP;
        end
        CycleOp2: begin
            address_bus_high_source_next = bus_sources::AddrBusHighSrc_One;
            address_bus_low_source_next = bus_sources::AddrBusLowSrc_SP;
        end
        CycleOp3: begin
            address_bus_high_source_next = bus_sources::AddrBusHighSrc_One;
            address_bus_low_source_next = bus_sources::AddrBusLowSrc_SP;
        end
        CycleOp4: begin
            int_state_next = IntStateNone;

            address_bus_low_source_next = bus_sources::AddrBusLowSrc_DataLatch;
            address_bus_high_source_next = bus_sources::AddrBusHighSrc_DataLatch;

            alu_op_next = control_signals::AluOp_add;
            alu_a_source_next = bus_sources::AluASourceCtl_DataLatchLow;
            alu_b_source_next = bus_sources::AluBSourceCtl_Zero;
            alu_carry_source_next = bus_sources::AluCarrySource_One;

            data_latch_low_source_next = bus_sources::DataLatchLowSource_Alu;
            ctrl_signals_next[control_signals::LOAD_DataLow] = 1;

            ctrl_signals_next[control_signals::PC_LOAD] = 1;
            pc_low_source_next = bus_sources::PcLowSource_Mem;

            VP_next = 0;
        end
        CycleOp5: begin
            address_bus_low_source_next = bus_sources::AddrBusLowSrc_DataLatch;
            address_bus_high_source_next = bus_sources::AddrBusHighSrc_DataLatch;

            ctrl_signals_next[control_signals::PC_LOAD] = 1;
            pc_low_source_next = bus_sources::PcLowSource_CurrentValue;
            pc_high_source_next = bus_sources::PcHighSource_Mem;

            VP_next = 0;
        end
        CycleOp6: begin
            next_instruction();
        end
    endcase
end
endtask

task do_op_clc_first();
begin
    next_instruction();

    data_bus_source_next = bus_sources::DataBusSrc_Zero;
    ctrl_signals_next[control_signals::UpdateFlagC] = 1;
    ctrl_signals_next[control_signals::UseAluFlags] = 0;
end
endtask

task do_op_lda_first();
begin
    next_instruction();
    ctrl_signals_next[control_signals::LOAD_A] = 1;
    data_bus_source_next = bus_sources::DataBusSrc_Mem;

    ctrl_signals_next[control_signals::UpdateFlagN] = 1;
    ctrl_signals_next[control_signals::UpdateFlagZ] = 1;
    ctrl_signals_next[control_signals::CalculateFlagZ] = 1;
end
endtask

task do_op_ldx_first();
begin
    next_instruction();
    ctrl_signals_next[control_signals::LOAD_X] = 1;
    data_bus_source_next = bus_sources::DataBusSrc_Mem;

    ctrl_signals_next[control_signals::UpdateFlagN] = 1;
    ctrl_signals_next[control_signals::UpdateFlagZ] = 1;
    ctrl_signals_next[control_signals::CalculateFlagZ] = 1;
end
endtask

task do_op_nop_first();
begin
    next_instruction();
end
endtask

task do_op_pha_first();
begin
    address_bus_low_source_next = bus_sources::AddrBusLowSrc_SP;
    address_bus_high_source_next = bus_sources::AddrBusHighSrc_One;

    data_bus_source_next = bus_sources::DataBusSrc_A;
    rW_next = 0;
end
endtask

task do_op_pha();
begin
    next_instruction();

    alu_op_next = control_signals::AluOp_add;
    alu_a_source_next = bus_sources::AluASourceCtl_SP;
    alu_b_source_next = bus_sources::AluBSourceCtl_Zero;
    alu_carry_source_next = bus_sources::AluCarrySource_Zero;
    ctrl_signals_next[control_signals::AluBInverse] = 1;

    stack_pointer_source_next = bus_sources::StackPointerSource_Alu;
    ctrl_signals_next[control_signals::LOAD_SP] = 1;
end
endtask

task do_op_rti_first();
begin
    address_bus_low_source_next = bus_sources::AddrBusLowSrc_SP;
    address_bus_high_source_next = bus_sources::AddrBusHighSrc_One;

    alu_op_next = control_signals::AluOp_add;
    alu_a_source_next = bus_sources::AluASourceCtl_SP;
    alu_b_source_next = bus_sources::AluBSourceCtl_Zero;
    alu_carry_source_next = bus_sources::AluCarrySource_One;
    ctrl_signals_next[control_signals::AluBInverse] = 0;

    stack_pointer_source_next = bus_sources::StackPointerSource_Alu;
    ctrl_signals_next[control_signals::LOAD_SP] = 1;
end
endtask

task do_op_rti();
begin
    case(op_cycle)
        FirstOpCycle: begin
            address_bus_low_source_next = bus_sources::AddrBusLowSrc_SP;
            address_bus_high_source_next = bus_sources::AddrBusHighSrc_One;

            alu_op_next = control_signals::AluOp_add;
            alu_a_source_next = bus_sources::AluASourceCtl_SP;
            alu_b_source_next = bus_sources::AluBSourceCtl_Zero;
            alu_carry_source_next = bus_sources::AluCarrySource_One;
            ctrl_signals_next[control_signals::AluBInverse] = 0;

            stack_pointer_source_next = bus_sources::StackPointerSource_Alu;
            ctrl_signals_next[control_signals::LOAD_SP] = 1;
        end
        CycleOp2: begin
            // This is the data loaded by the previous address operation
            data_bus_source_next = bus_sources::DataBusSrc_Mem;
            ctrl_signals_next[control_signals::UseAluFlags] = 0;
            ctrl_signals_next[control_signals::UpdateFlagC] = 1;
            ctrl_signals_next[control_signals::UpdateFlagZ] = 1;
            ctrl_signals_next[control_signals::UpdateFlagI] = 1;
            ctrl_signals_next[control_signals::UpdateFlagD] = 1;
            ctrl_signals_next[control_signals::UpdateFlagV] = 1;
            ctrl_signals_next[control_signals::UpdateFlagN] = 1;

            // And this in the current address operation
            address_bus_low_source_next = bus_sources::AddrBusLowSrc_SP;
            address_bus_high_source_next = bus_sources::AddrBusHighSrc_One;

            alu_op_next = control_signals::AluOp_add;
            alu_a_source_next = bus_sources::AluASourceCtl_SP;
            alu_b_source_next = bus_sources::AluBSourceCtl_Zero;
            alu_carry_source_next = bus_sources::AluCarrySource_One;
            ctrl_signals_next[control_signals::AluBInverse] = 0;

            stack_pointer_source_next = bus_sources::StackPointerSource_Alu;
            ctrl_signals_next[control_signals::LOAD_SP] = 1;

            pc_low_source_next = bus_sources::PcLowSource_Mem;
            ctrl_signals_next[control_signals::PC_LOAD] = 1;
        end
        CycleOp3: begin
            address_bus_low_source_next = bus_sources::AddrBusLowSrc_SP;
            address_bus_high_source_next = bus_sources::AddrBusHighSrc_One;

            pc_low_source_next = bus_sources::PcLowSource_CurrentValue;
            pc_high_source_next = bus_sources::PcHighSource_Mem;
            ctrl_signals_next[control_signals::PC_LOAD] = 1;
        end
        CycleOp4: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
end
endtask

task do_op_txs_first();
begin
    next_instruction();

    data_bus_source_next = bus_sources::DataBusSrc_X;
    stack_pointer_source_next = bus_sources::StackPointerSource_DataBus;
    ctrl_signals_next[control_signals::LOAD_SP] = 1;
end
endtask

endmodule
