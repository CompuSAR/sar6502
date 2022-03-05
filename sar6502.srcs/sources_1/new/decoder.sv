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
        input [7:0]status,
        input alu_carry,
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
logic [control_signals::ctrl_signals_last : 0] ctrl_signals_next;

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

enum logic[31:0] {
    AddrInvalid = 'X,
    AddrImplicit=0,
    AddrImmediate,
    AddrAbsolute,
    AddrZeroPage,
    AddrStack
} active_addr_mode, active_addr_mode_next;

logic pc_post_increment, pc_post_increment_next;

typedef enum logic[31:0] {
    OpInvalid = 'X,

    OpNone = 0,
    OpAdc,
    OpBcc,
    OpBcs,
    OpBeq,
    OpBmi,
    OpBne,
    OpBpl,
    OpBra,
    OpBrk,
    OpBvc,
    OpBvs,
    OpClc,
    OpJsr,
    OpLda,
    OpLdx,
    OpNop,
    OpPha,
    OpPhp,
    OpPlp,
    OpRti,
    OpRts,
    OpSta,
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
        pc_post_increment <= 0;
    end else begin
        op_cycle <= op_cycle_next;
        active_addr_mode <= active_addr_mode_next;
        active_op <= active_op_next;
        pc_post_increment <= pc_post_increment_next;
        int_state <= int_state_next;
    end
end

always_comb begin
    address_bus_low_source = address_bus_low_source_next;
    address_bus_high_source = address_bus_high_source_next;
    data_bus_source = data_bus_source_next;
    pc_low_source = pc_low_source_next;
    pc_high_source = pc_high_source_next;
    data_latch_low_source = data_latch_low_source_next;
    data_latch_high_source = data_latch_high_source_next;
    stack_pointer_source = stack_pointer_source_next;
    alu_a_source = alu_a_source_next;
    alu_b_source = alu_b_source_next;
    alu_carry_source = alu_carry_source_next;
    alu_op = alu_op_next;
    ctrl_signals = ctrl_signals_next;

    rW = rW_next;
    sync = sync_next;
    ML = ML_next;
    VP = VP_next;
end

task set_invalid_state();
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
    ctrl_signals_next = { control_signals::ctrl_signals_last {1'bx} };

    active_op_next = OpInvalid;
    active_addr_mode_next = AddrInvalid;
endtask

// Decoder main
always_comb begin
    set_invalid_state();

    ctrl_signals_next = 0;
    rW_next = 1;
    sync_next = 0;
    ML_next = 1;
    VP_next = 1;
    int_state_next = int_state;
    pc_post_increment_next = pc_post_increment;

    active_op_next = active_op;
    active_addr_mode_next = active_addr_mode;
    op_cycle_next = op_cycle << 1;

    if( !RESET ) begin
        int_state_next = IntStateReset;
    end else if( op_cycle==CycleFetch ) begin
        op_cycle_next = 1;
        address_bus_low_source_next = bus_sources::AddrBusLowSrc_PC;
        address_bus_high_source_next = bus_sources::AddrBusHighSrc_PC;
        sync_next = 1;

        do_last_cycle();
    end else if( op_cycle==CycleDecode ) begin
        addr_bus_pc();

        if( int_state==IntStateNone ) begin
            ctrl_signals_next[control_signals::PC_ADVANCE] = 1;
            do_decode();
        end else begin
            set_operation(OpBrk);
        end
    end else if( op_cycle<FirstOpCycle )
        do_addr_lookup();
    else
        do_operation();
end

task do_decode();
    case( memory_in )
        8'h08: set_addr_mode_stack( OpPhp );
        8'h10: set_addr_mode_implicit( OpBpl );
        8'h18: set_addr_mode_implicit( OpClc );
        8'h20: set_addr_mode_stack( OpJsr );
        8'h28: set_addr_mode_stack( OpPlp );
        8'h30: set_addr_mode_implicit( OpBmi );
        8'h40: set_addr_mode_stack( OpRti );
        8'h48: set_addr_mode_stack( OpPha );
        8'h50: set_addr_mode_implicit( OpBvc );
        8'h60: set_addr_mode_stack( OpRts );
        8'h6d: set_addr_mode_absolute( OpAdc );
        8'h70: set_addr_mode_implicit( OpBvs );
        8'h80: set_addr_mode_implicit( OpBra );
        8'h8d: set_addr_mode_absolute( OpSta );
        8'h90: set_addr_mode_implicit( OpBcc );
        8'h9a: set_addr_mode_implicit( OpTxs );
        8'ha2: set_addr_mode_immediate( OpLdx );
        8'ha9: set_addr_mode_immediate( OpLda );
        8'ha5: set_addr_mode_zp( OpLda );
        8'had: set_addr_mode_absolute( OpLda );
        8'hb0: set_addr_mode_implicit( OpBcs );
        8'hd0: set_addr_mode_implicit( OpBne );
        8'hea: set_addr_mode_implicit( OpNop );
        8'hf0: set_addr_mode_implicit( OpBeq );
        default: do_unknown_command();
    endcase
endtask

task do_unknown_command();
    set_invalid_state();
endtask

task do_addr_lookup();
    case( active_addr_mode )
        AddrAbsolute: do_addr_mode_absolute();
        AddrZeroPage: do_addr_mode_zp();
        AddrStack: do_addr_mode_stack();
        default: set_invalid_state();
    endcase
endtask

task addr_bus_pc();
    address_bus_low_source_next = bus_sources::AddrBusLowSrc_PC;
    address_bus_high_source_next = bus_sources::AddrBusHighSrc_PC;
endtask

task addr_bus_sp();
    address_bus_low_source_next = bus_sources::AddrBusLowSrc_SP;
    address_bus_high_source_next = bus_sources::AddrBusHighSrc_One;
endtask

task set_addr_mode_absolute(operations current_op);
    active_op_next = current_op;
    active_addr_mode_next = AddrAbsolute;
endtask

task do_addr_mode_absolute();
    case( op_cycle )
        CycleAddr1: begin
            data_latch_low_source_next = bus_sources::DataLatchLowSource_Mem;
            ctrl_signals_next[control_signals::LOAD_DataLow] = 1;
            ctrl_signals_next[control_signals::PC_ADVANCE] = 1;

            addr_bus_pc();
        end
        CycleAddr2: begin
            data_latch_high_source_next = bus_sources::DataLatchHighSource_Mem;
            ctrl_signals_next[control_signals::LOAD_DataHigh] = 1;
            ctrl_signals_next[control_signals::PC_ADVANCE] = 1;

            address_bus_low_source_next = bus_sources::AddrBusLowSrc_DataLatch;
            address_bus_high_source_next = bus_sources::AddrBusHighSrc_Mem;

            set_operation( active_op );
        end
        default: set_invalid_state();
    endcase
endtask

task set_addr_mode_immediate(operations current_op);
    active_addr_mode_next = AddrImmediate;
    pc_post_increment_next = 1;

    set_operation(current_op);
endtask

task set_addr_mode_implicit(operations current_op);
    active_addr_mode_next = AddrImplicit;
    set_operation(current_op);
endtask

task set_addr_mode_stack(operations current_op);
    active_addr_mode_next = AddrStack;
    active_op_next = current_op;
endtask

task do_addr_mode_stack();
    addr_bus_sp();

    set_operation(active_op);
endtask

task sp_dec();
    // Subtract 1 from stack pointer
    alu_a_source_next = bus_sources::AluASourceCtl_SP;
    alu_b_source_next = bus_sources::AluBSourceCtl_Zero;
    alu_carry_source_next = bus_sources::AluCarrySource_Zero;
    ctrl_signals_next[control_signals::AluBInverse] = 1;
    alu_op_next = control_signals::AluOp_add;

    stack_pointer_source_next = bus_sources::StackPointerSource_Alu;
    ctrl_signals_next[control_signals::LOAD_SP] = 1;
endtask

task sp_inc();
    // Add 1 to stack pointer
    alu_op_next = control_signals::AluOp_add;
    alu_a_source_next = bus_sources::AluASourceCtl_SP;
    alu_b_source_next = bus_sources::AluBSourceCtl_Zero;
    alu_carry_source_next = bus_sources::AluCarrySource_One;
    ctrl_signals_next[control_signals::AluBInverse] = 0;

    stack_pointer_source_next = bus_sources::StackPointerSource_Alu;
    ctrl_signals_next[control_signals::LOAD_SP] = 1;
endtask

task set_addr_mode_zp(operations current_op);
    active_op_next = current_op;
    active_addr_mode_next = AddrZeroPage;
endtask

task do_addr_mode_zp();
    case( op_cycle )
        CycleAddr1: begin
            ctrl_signals_next[control_signals::PC_ADVANCE] = 1;

            address_bus_low_source_next = bus_sources::AddrBusLowSrc_Mem;
            address_bus_high_source_next = bus_sources::AddrBusHighSrc_Zero;

            data_latch_low_source_next = bus_sources::DataLatchLowSource_Mem;
            ctrl_signals_next[control_signals::LOAD_DataLow] = 1;
            data_latch_high_source_next = bus_sources::DataLatchHighSource_Zero;
            ctrl_signals_next[control_signals::LOAD_DataHigh] = 1;

            set_operation(active_op);
        end
        default: set_invalid_state();
    endcase
endtask

task set_operation(operations current_op);
    active_op_next = current_op;
    op_cycle_next = FirstOpCycle;

    case( current_op )
        OpAdc: do_op_adc_first();
        OpBcc: do_op_bcc_first();
        OpBcs: do_op_bcs_first();
        OpBeq: do_op_beq_first();
        OpBmi: do_op_bmi_first();
        OpBne: do_op_bne_first();
        OpBpl: do_op_bpl_first();
        OpBra: do_op_bra_first();
        OpBvc: do_op_bvc_first();
        OpBvs: do_op_bvs_first();
        OpBrk: do_op_brk_first();
        OpClc: do_op_clc_first();
        OpJsr: do_op_jsr_first();
        OpLda: do_op_lda_first();
        OpLdx: do_op_ldx_first();
        OpNop: do_op_nop_first();
        OpPha: do_op_pha_first();
        OpPhp: do_op_php_first();
        OpPlp: do_op_plp_first();
        OpRti: do_op_rti_first();
        OpRts: do_op_rts_first();
        OpSta: do_op_sta_first();
        OpTxs: do_op_txs_first();
        default: set_invalid_state();
    endcase
endtask

task do_operation();
    case( active_op )
        OpBcc: do_branch();
        OpBcs: do_branch();
        OpBeq: do_branch();
        OpBmi: do_branch();
        OpBne: do_branch();
        OpBpl: do_branch();
        OpBra: do_branch();
        OpBvc: do_branch();
        OpBvs: do_branch();
        OpBrk: do_op_brk();
        OpJsr: do_op_jsr();
        OpPlp: do_op_plp();
        OpRti: do_op_rti();
        OpRts: do_op_rts();
        default: set_invalid_state();
    endcase
endtask

task do_last_cycle();
    active_op_next = OpInvalid;

    if( pc_post_increment ) begin
        ctrl_signals_next[control_signals::PC_ADVANCE] = 1;
        pc_post_increment_next = 0;
    end

    case( active_op )
        OpBrk: do_op_brk_last();
        OpJsr: do_op_jsr_last();
        OpLda: do_op_lda_last();
        OpLdx: do_op_ldx_last();
        OpPlp: do_op_plp_last();
        OpRti: do_op_rti_last();
        OpRts: do_op_rts_last();
    endcase
endtask

task next_instruction();
    op_cycle_next = CycleFetch;
endtask

task do_op_adc_first();
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
endtask

task do_op_bcc_first();
    do_branch_first( !status[control_signals::FlagsCarry] );
endtask

task do_op_bcs_first();
    do_branch_first( status[control_signals::FlagsCarry] );
endtask

task do_op_beq_first();
    do_branch_first( status[control_signals::FlagsZero] );
endtask

task do_op_bmi_first();
    do_branch_first( status[control_signals::FlagsNegative] );
endtask

task do_op_bne_first();
    do_branch_first( !status[control_signals::FlagsZero] );
endtask

task do_op_bpl_first();
    do_branch_first( !status[control_signals::FlagsNegative] );
endtask

task do_op_bra_first();
    do_branch_first( 1 );
endtask

task do_op_bvc_first();
    do_branch_first( !status[control_signals::FlagsOverflow] );
endtask

task do_op_bvs_first();
    do_branch_first( status[control_signals::FlagsOverflow] );
endtask

task do_branch_first( input condition );
    if( ! condition ) begin
        next_instruction();
        pc_post_increment_next = 1;
    end
endtask

task do_branch();
    case( op_cycle )
        FirstOpCycle: begin
            addr_bus_pc();

            ctrl_signals_next[control_signals::PC_ADVANCE] = 1;
            alu_a_source_next = bus_sources::AluASourceCtl_PC_Low;
            alu_b_source_next = bus_sources::AluBSourceCtl_Mem;
            alu_op_next = control_signals::AluOp_add;
            alu_carry_source_next = bus_sources::AluCarrySource_Zero;

            if( !alu_carry ) begin
                next_instruction();

                pc_low_source_next = bus_sources::PcLowSource_Alu;
                pc_high_source_next = bus_sources::PcHighSource_CurrentValue;
                ctrl_signals_next[control_signals::PC_LOAD] = 1;
            end else begin
                data_latch_low_source_next = bus_sources::DataLatchLowSource_Alu;
                ctrl_signals_next[control_signals::LOAD_DataLow] = 1;
                data_latch_high_source_next = bus_sources::DataLatchHighSource_PC;
                ctrl_signals_next[control_signals::LOAD_DataHigh] = 1;
            end
        end
        CycleOp2: begin
            next_instruction();

            address_bus_low_source_next = bus_sources::AddrBusLowSrc_DataLatch;
            address_bus_high_source_next = bus_sources::AddrBusHighSrc_DataLatch;
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_brk_first();
    data_latch_high_source_next = bus_sources::DataLatchHighSource_FF;
    ctrl_signals_next[control_signals::LOAD_DataHigh] = 1;

    case(int_state)
        IntStateNone: data_latch_low_source_next = bus_sources::DataLatchLowSource_FE;
        IntStateReset: data_latch_low_source_next = bus_sources::DataLatchLowSource_FC;
        IntStateNmi: data_latch_low_source_next = bus_sources::DataLatchLowSource_FA;
        IntStateIrq: data_latch_low_source_next = bus_sources::DataLatchLowSource_FE;
        default: set_invalid_state();
    endcase
    ctrl_signals_next[control_signals::LOAD_DataLow] = 1;

    address_bus_high_source_next = bus_sources::AddrBusHighSrc_PC;
    address_bus_low_source_next = bus_sources::AddrBusLowSrc_PC;
endtask

task do_op_brk();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_sp();
        end
        CycleOp2: begin
            addr_bus_sp();
        end
        CycleOp3: begin
            addr_bus_sp();
        end
        CycleOp4: begin
            int_state_next = IntStateNone;
        end
        CycleOp5: begin
            address_bus_low_source_next = bus_sources::AddrBusLowSrc_DataLatch;
            address_bus_high_source_next = bus_sources::AddrBusHighSrc_DataLatch;

            alu_op_next = control_signals::AluOp_add;
            alu_a_source_next = bus_sources::AluASourceCtl_DataLatchLow;
            alu_b_source_next = bus_sources::AluBSourceCtl_Zero;
            alu_carry_source_next = bus_sources::AluCarrySource_One;

            data_latch_low_source_next = bus_sources::DataLatchLowSource_Alu;
            ctrl_signals_next[control_signals::LOAD_DataLow] = 1;

            VP_next = 0;
        end
        CycleOp6: begin
            ctrl_signals_next[control_signals::PC_LOAD] = 1;
            pc_low_source_next = bus_sources::PcLowSource_Mem;

            address_bus_low_source_next = bus_sources::AddrBusLowSrc_DataLatch;
            address_bus_high_source_next = bus_sources::AddrBusHighSrc_DataLatch;

            VP_next = 0;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_brk_last();
    ctrl_signals_next[control_signals::PC_LOAD] = 1;
    pc_low_source_next = bus_sources::PcLowSource_CurrentValue;
    pc_high_source_next = bus_sources::PcHighSource_Mem;
endtask

task do_op_clc_first();
    next_instruction();

    data_bus_source_next = bus_sources::DataBusSrc_Zero;
    ctrl_signals_next[control_signals::UpdateFlagC] = 1;
    ctrl_signals_next[control_signals::UseAluFlags] = 0;
endtask

task do_op_jsr_first();
    // Store destination LSB in DL
    ctrl_signals_next[control_signals::LOAD_DataLow] = 1;
    data_latch_low_source_next = bus_sources::DataLatchLowSource_Mem;

    // Dummy stack cycle while we advance the PC
    ctrl_signals_next[control_signals::PC_ADVANCE] = 1;
endtask

task do_op_jsr();
    case(op_cycle)
        FirstOpCycle: begin
            // Write PC MSB to stack
            addr_bus_sp();
            data_bus_source_next = bus_sources::DataBusSrc_Pc_High;
            rW_next = 0;

            sp_dec();
        end
        CycleOp2: begin
            // Write PC LSB to stack
            addr_bus_sp();
            data_bus_source_next = bus_sources::DataBusSrc_Pc_Low;
            rW_next = 0;

            sp_dec();
        end
        CycleOp3: begin
            // Address to read destination MSB
            addr_bus_pc();
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_jsr_last();
    // Load destination MSB into PC
    pc_high_source_next = bus_sources::PcHighSource_Mem;
    pc_low_source_next = bus_sources::PcLowSource_Dl;
    ctrl_signals_next[control_signals::PC_LOAD] = 1;
endtask

task do_op_lda_first();
    next_instruction();
endtask

task do_op_lda_last();
    ctrl_signals_next[control_signals::LOAD_A] = 1;
    data_bus_source_next = bus_sources::DataBusSrc_Mem;

    ctrl_signals_next[control_signals::UpdateFlagN] = 1;
    ctrl_signals_next[control_signals::UpdateFlagZ] = 1;
    ctrl_signals_next[control_signals::CalculateFlagZ] = 1;
endtask

task do_op_ldx_first();
    next_instruction();

    addr_bus_pc();
endtask

task do_op_ldx_last();
    ctrl_signals_next[control_signals::PC_ADVANCE] = 1;

    ctrl_signals_next[control_signals::LOAD_X] = 1;
    data_bus_source_next = bus_sources::DataBusSrc_Mem;

    ctrl_signals_next[control_signals::UpdateFlagN] = 1;
    ctrl_signals_next[control_signals::UpdateFlagZ] = 1;
    ctrl_signals_next[control_signals::CalculateFlagZ] = 1;
endtask

task do_op_nop_first();
    next_instruction();
endtask

task do_op_pha_first();
    data_bus_source_next = bus_sources::DataBusSrc_A;
    rW_next = 0;

    sp_dec();

    next_instruction();
endtask

task do_op_php_first();
    rW_next = 0;

    data_bus_source_next = bus_sources::DataBusSrc_Status;
    ctrl_signals_next[control_signals::OutputFlagB] = 1;
    sp_dec();

    next_instruction();
endtask

task do_op_plp_first();
    // First stack cycle: dummy read
    sp_inc();
endtask

task do_op_plp();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_sp();
            next_instruction();
        end
        default set_invalid_state();
    endcase
endtask

task do_op_plp_last();
    // This is the data loaded by the previous address operation
    data_bus_source_next = bus_sources::DataBusSrc_Mem;
    ctrl_signals_next[control_signals::UseAluFlags] = 0;
    ctrl_signals_next[control_signals::UpdateFlagC] = 1;
    ctrl_signals_next[control_signals::UpdateFlagZ] = 1;
    ctrl_signals_next[control_signals::UpdateFlagI] = 1;
    ctrl_signals_next[control_signals::UpdateFlagD] = 1;
    ctrl_signals_next[control_signals::UpdateFlagV] = 1;
    ctrl_signals_next[control_signals::UpdateFlagN] = 1;
endtask

task do_op_rti_first();
    // First stack cycle: dummy read
    sp_inc();
endtask

task do_op_rti();
    case(op_cycle)
        FirstOpCycle: begin
            // Read status flags
            addr_bus_sp();

            sp_inc();
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

            // Read PC MSB
            addr_bus_sp();

            sp_inc();
        end
        CycleOp3: begin
            // Store PC MSB
            pc_low_source_next = bus_sources::PcLowSource_Mem;
            ctrl_signals_next[control_signals::PC_LOAD] = 1;

            // Read PC LSB
            addr_bus_sp();

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_rti_last();
    pc_low_source_next = bus_sources::PcLowSource_CurrentValue;
    pc_high_source_next = bus_sources::PcHighSource_Mem;
    ctrl_signals_next[control_signals::PC_LOAD] = 1;
endtask

task do_op_rts_first();
    // First stack cycle: dummy read
    sp_inc();
endtask

task do_op_rts();
    case(op_cycle)
        FirstOpCycle: begin
            // Read PC MSB
            addr_bus_sp();

            sp_inc();
        end
        CycleOp2: begin
            // Store PC MSB
            pc_low_source_next = bus_sources::PcLowSource_Mem;
            ctrl_signals_next[control_signals::PC_LOAD] = 1;

            // Read PC LSB
            addr_bus_sp();
        end
        CycleOp3: begin
            pc_low_source_next = bus_sources::PcLowSource_CurrentValue;
            pc_high_source_next = bus_sources::PcHighSource_Mem;
            ctrl_signals_next[control_signals::PC_LOAD] = 1;

            addr_bus_pc();
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_rts_last();
    ctrl_signals_next[control_signals::PC_ADVANCE] = 1;
endtask

task do_op_sta_first();
    rW_next = 0;
    data_bus_source_next = bus_sources::DataBusSrc_A;

    next_instruction();
endtask

task do_op_txs_first();
    next_instruction();

    data_bus_source_next = bus_sources::DataBusSrc_X;
    stack_pointer_source_next = bus_sources::StackPointerSource_DataBus;
    ctrl_signals_next[control_signals::LOAD_SP] = 1;
endtask

endmodule
