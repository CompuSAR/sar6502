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
        input [7:0]memory_in,
        input [7:0]status,
        input alu_carry,
        input clock,
        input RESET,
        input IRQ,
        input NMI,

        input ready,

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
        output logic ctrl_signals[control_signals::ctrl_signals_last:0],

        output logic rW,
        output logic sync,
        output logic ML,
        output logic VP,
        output logic incompatible
    );

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
    AddrAbsoluteInd,
    AddrAbsoluteX,
    AddrAbsoluteY,
    AddrAbsoluteXInd,
    AddrZeroPage,
    AddrZeroPageX,
    AddrZeroPageY,
    AddrZeroPageInd,
    AddrZeroPageXInd,
    AddrZeroPageIndY,
    AddrStack
} active_addr_mode = AddrImplicit, active_addr_mode_next;

typedef enum logic[31:0] {
    OpInvalid = 'X,

    OpBrk = 'h00,

    OpRmb0 = 'h07,
    OpRmb1 = 'h17,
    OpRmb2 = 'h27,
    OpRmb3 = 'h37,
    OpRmb4 = 'h47,
    OpRmb5 = 'h57,
    OpRmb6 = 'h67,
    OpRmb7 = 'h77,
    OpSmb0 = 'h87,
    OpSmb1 = 'h97,
    OpSmb2 = 'ha7,
    OpSmb3 = 'hb7,
    OpSmb4 = 'hc7,
    OpSmb5 = 'hd7,
    OpSmb6 = 'he7,
    OpSmb7 = 'hf7,

    OpBbr0 = 'h0f,
    OpBbr1 = 'h1f,
    OpBbr2 = 'h2f,
    OpBbr3 = 'h3f,
    OpBbr4 = 'h4f,
    OpBbr5 = 'h5f,
    OpBbr6 = 'h6f,
    OpBbr7 = 'h7f,
    OpBbs0 = 'h8f,
    OpBbs1 = 'h9f,
    OpBbs2 = 'haf,
    OpBbs3 = 'hbf,
    OpBbs4 = 'hcf,
    OpBbs5 = 'hdf,
    OpBbs6 = 'hef,
    OpBbs7 = 'hff,

    OpOra = 'h01,
    OpAnd = 'h21,
    OpEor = 'h41,
    OpAdc = 'h61,

    OpAsl = 'h06,
    OpRol = 'h26,
    OpLsr = 'h46,
    OpRor = 'h66,

    OpSta = 'h81,
    OpLda = 'ha1,
    OpCmp = 'hc1,
    OpSbc = 'he1,

    OpStx = 'h86,
    OpLdx = 'ha6,
    OpDec = 'hc6,
    OpInc = 'he6,

    OpBpl = 'h10,
    OpBmi = 'h30,
    OpBvc = 'h50,
    OpBvs = 'h70,
    OpBcc = 'h90,
    OpBcs = 'hb0,
    OpBne = 'hd0,
    OpBeq = 'hf0,
    OpBra = 'h80,

    OpDey = 'h88,
    OpTya = 'h98,
    OpTay = 'ha8,
    OpIny = 'hc8,
    OpInx = 'he8,

    OpTxa = 'h8a,
    OpTxs = 'h9a,
    OpTax = 'haa,
    OpTsx = 'hba,
    OpDex = 'hca,
    OpPhx = 'hda,
    OpNop = 'hea,
    OpPlx = 'hfa,

    OpAslA = 'h0a,
    OpIncA = 'h1a,
    OpRolA = 'h2a,
    OpDecA = 'h3a,
    OpLsrA = 'h4a,
    OpPhy  = 'h5a,
    OpRorA = 'h6a,
    OpPly  = 'h7a,

    OpTsb = 'h04,
    OpTrb = 'h14,
    OpBit = 'h24,
    OpStz = 'h64,
    OpSty = 'h84,
    OpLdy = 'ha4,
    OpCpy = 'hc4,
    OpCpx = 'he4,

    OpJmp = 'h4c,
    OpJsr = 'h20,
    OpRti = 'h40,
    OpRts = 'h60,

    OpPhp = 'h08,
    OpPlp = 'h28,
    OpPha = 'h48,
    OpPla = 'h68,



    OpStp = 'hdb,
    OpWai = 'hcb,

    OpClc = 'h18,
    OpCld = 'hd8,
    OpCli = 'h58,
    OpClv = 'hb8,
    OpSec = 'h38,
    OpSed = 'hf8,
    OpSei = 'h78,

    OpEndMarker = 'h2222
} operations;
operations active_op = OpStp, active_op_next;

enum { IntStateNone, IntStateReset, IntStateNmi, IntStateIrq } int_state = IntStateReset, int_state_next;

logic branch_offset_negative;
logic irq_mask;
logic last_NMI;
logic nmi_pending, nmi_pending_next;

always_ff@(negedge clock) begin
    if( !RESET ) begin
        active_op <= OpBrk;
        active_addr_mode <= AddrInvalid;
        op_cycle <= FirstOpCycle;
        int_state <= IntStateReset;
        nmi_pending <= 0;
    end else if( ready ) begin
        op_cycle <= op_cycle_next;
        active_addr_mode <= active_addr_mode_next;
        active_op <= active_op_next;
        int_state <= int_state_next;
        branch_offset_negative <= memory_in[7];
        if(last_NMI && !NMI)
            nmi_pending <= 1;
        else
            nmi_pending <= nmi_pending_next;
    end else begin
        // If not ready and not reset, everything remains as it was
    end

    irq_mask <= status[control_signals::FlagsIrqMask];
    last_NMI <= NMI;
end

task set_invalid_state();
    int i;

    op_cycle_next = CycleInvalid;
    address_bus_low_source = bus_sources::AddrBusLowSrc_Invalid;
    address_bus_high_source = bus_sources::AddrBusHighSrc_Invalid;
    data_bus_source = bus_sources::DataBusSrc_Invalid;
    pc_low_source = bus_sources::PcLowSource_Invalid;
    pc_high_source = bus_sources::PcHighSource_Invalid;
    data_latch_low_source = bus_sources::DataLatchLowSource_Invalid;
    data_latch_high_source = bus_sources::DataLatchHighSource_Invalid;
    stack_pointer_source = bus_sources::StackPointerSource_Invalid;
    alu_a_source = bus_sources::AluASourceCtl_Invalid;
    alu_b_source = bus_sources::AluBSourceCtl_Invalid;
    alu_op = control_signals::AluOp_INVALID;
    alu_carry_source = bus_sources::AluCarrySource_Invalid;
    for(i=0; i<control_signals::ctrl_signals_last; ++i)
        ctrl_signals[i] = 1'bx;

    active_op_next = OpInvalid;
    active_addr_mode_next = AddrInvalid;
endtask

// Decoder main
always_comb begin
    int i;

    set_invalid_state();

    for(i=0; i<control_signals::LastLoadSignal; ++i)
        ctrl_signals[i] = 1'b0;
    rW = 1;
    sync = 0;
    ML = 1;
    VP = 1;
    incompatible = 0;
    int_state_next = int_state;
    nmi_pending_next = nmi_pending;

    active_op_next = active_op;
    active_addr_mode_next = active_addr_mode;
    op_cycle_next = op_cycle << 1;

    if( !RESET ) begin
        int_state_next = IntStateReset;
        addr_bus_pc();
    end else if( op_cycle==CycleFetch ) begin
        do_fetch_cycle();
    end else if( op_cycle==CycleDecode ) begin
        addr_bus_pc();

        if( int_state==IntStateNone ) begin
            do_decode();
        end else begin
            set_operation(OpBrk);
        end
    end else if( op_cycle<FirstOpCycle )
        do_addr_lookup();
    else
        do_operation();
end

task do_fetch_cycle();
    op_cycle_next = CycleDecode;
    addr_bus_pc();
    sync = 1;

    if( int_state==IntStateNone ) begin
        if(nmi_pending) begin
            int_state_next = IntStateNmi;
            nmi_pending_next = 0;
        end else if( !IRQ && irq_mask==0 )
            int_state_next = IntStateIrq;
        else
            ctrl_signals[control_signals::PC_ADVANCE] = 1;
    end else if( int_state==IntStateReset ) begin
        incompatible = 1;
    end
endtask

task do_decode();
    case( memory_in )
        8'h00: set_addr_mode_implicit( OpBrk );
        8'h01: set_addr_mode_zp_x_ind( OpOra );
        8'h04: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpTsb ); else do_unknown_command();
        8'h05: set_addr_mode_zp( OpOra );
        8'h06: set_addr_mode_zp( OpAsl );
        8'h07: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpRmb0 ); else do_unknown_command();
        8'h08: set_addr_mode_stack( OpPhp );
        8'h09: set_addr_mode_immediate( OpOra );
        8'h0a: set_addr_mode_implicit( OpAslA );
        8'h0c: if( CPU_VARIANT>=1 ) set_addr_mode_absolute( OpTsb ); else do_unknown_command();
        8'h0d: set_addr_mode_absolute( OpOra );
        8'h0e: set_addr_mode_absolute( OpAsl );
        8'h0f: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbr0 ); else do_unknown_command();
        8'h10: set_addr_mode_implicit( OpBpl );
        8'h11: set_addr_mode_zp_ind_y( OpOra );
        8'h12: if( CPU_VARIANT>=1 ) set_addr_mode_zp_ind( OpOra ); else do_unknown_command();
        8'h14: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpTrb ); else do_unknown_command();
        8'h15: set_addr_mode_zp_x( OpOra );
        8'h16: set_addr_mode_zp_x( OpAsl );
        8'h17: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpRmb1 ); else do_unknown_command();
        8'h18: set_addr_mode_implicit( OpClc );
        8'h19: set_addr_mode_abs_y( OpOra );
        8'h1a: if( CPU_VARIANT>=1 ) set_addr_mode_implicit( OpIncA ); else do_unknown_command();
        8'h1c: if( CPU_VARIANT>=1 ) set_addr_mode_absolute( OpTrb ); else do_unknown_command();
        8'h1d: set_addr_mode_abs_x( OpOra );
        8'h1e: set_addr_mode_abs_x( OpAsl );
        8'h1f: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbr1 ); else do_unknown_command();
        8'h20: set_addr_mode_stack( OpJsr );
        8'h21: set_addr_mode_zp_x_ind( OpAnd );
        8'h24: set_addr_mode_zp( OpBit );
        8'h25: set_addr_mode_zp( OpAnd );
        8'h26: set_addr_mode_zp( OpRol );
        8'h27: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpRmb2 ); else do_unknown_command();
        8'h28: set_addr_mode_stack( OpPlp );
        8'h29: set_addr_mode_immediate( OpAnd );
        8'h2a: set_addr_mode_implicit( OpRolA );
        8'h2c: set_addr_mode_absolute( OpBit );
        8'h2d: set_addr_mode_absolute( OpAnd );
        8'h2e: set_addr_mode_absolute( OpRol );
        8'h2f: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbr2 ); else do_unknown_command();
        8'h30: set_addr_mode_implicit( OpBmi );
        8'h31: set_addr_mode_zp_ind_y( OpAnd );
        8'h32: if( CPU_VARIANT>=1 ) set_addr_mode_zp_ind( OpAnd ); else do_unknown_command();
        8'h34: if( CPU_VARIANT>=1 ) set_addr_mode_zp_x( OpBit ); else do_unknown_command();
        8'h35: set_addr_mode_zp_x( OpAnd );
        8'h36: set_addr_mode_zp_x( OpRol );
        8'h37: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpRmb3 ); else do_unknown_command();
        8'h38: set_addr_mode_implicit( OpSec );
        8'h39: set_addr_mode_abs_y( OpAnd );
        8'h3a: if( CPU_VARIANT>=1 ) set_addr_mode_implicit( OpDecA ); else do_unknown_command();
        8'h3c: if( CPU_VARIANT>=1 ) set_addr_mode_abs_x( OpBit ); else do_unknown_command();
        8'h3d: set_addr_mode_abs_x( OpAnd );
        8'h3e: set_addr_mode_abs_x( OpRol );
        8'h3f: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbr3 ); else do_unknown_command();
        8'h40: set_addr_mode_stack( OpRti );
        8'h41: set_addr_mode_zp_x_ind( OpEor );
        8'h45: set_addr_mode_zp( OpEor );
        8'h46: set_addr_mode_zp( OpLsr );
        8'h47: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpRmb4 ); else do_unknown_command();
        8'h48: set_addr_mode_stack( OpPha );
        8'h49: set_addr_mode_immediate( OpEor );
        8'h4a: set_addr_mode_implicit( OpLsrA );
        8'h4c: set_addr_mode_absolute( OpJmp );
        8'h4d: set_addr_mode_absolute( OpEor );
        8'h4e: set_addr_mode_absolute( OpLsr );
        8'h4f: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbr4 ); else do_unknown_command();
        8'h50: set_addr_mode_implicit( OpBvc );
        8'h51: set_addr_mode_zp_ind_y( OpEor );
        8'h52: if( CPU_VARIANT>=1 ) set_addr_mode_zp_ind( OpEor ); else do_unknown_command();
        8'h55: set_addr_mode_zp_x( OpEor );
        8'h56: set_addr_mode_zp_x( OpLsr );
        8'h57: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpRmb5 ); else do_unknown_command();
        8'h58: set_addr_mode_implicit( OpCli );
        8'h59: set_addr_mode_abs_y( OpEor );
        8'h5a: if( CPU_VARIANT>=1 ) set_addr_mode_stack( OpPhy ); else do_unknown_command();
        8'h5d: set_addr_mode_abs_x( OpEor );
        8'h5e: set_addr_mode_abs_x( OpLsr );
        8'h5f: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbr5 ); else do_unknown_command();
        8'h60: set_addr_mode_stack( OpRts );
        8'h61: set_addr_mode_zp_x_ind( OpAdc );
        8'h64: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpStz ); else do_unknown_command();
        8'h65: set_addr_mode_zp( OpAdc );
        8'h66: set_addr_mode_zp( OpRor );
        8'h67: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpRmb6 ); else do_unknown_command();
        8'h68: set_addr_mode_stack( OpPla );
        8'h69: set_addr_mode_immediate( OpAdc );
        8'h6a: set_addr_mode_implicit( OpRorA );
        8'h6c: set_addr_mode_abs_ind( OpJmp );
        8'h6d: set_addr_mode_absolute( OpAdc );
        8'h6e: set_addr_mode_absolute( OpRor );
        8'h6f: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbr6 ); else do_unknown_command();
        8'h70: set_addr_mode_implicit( OpBvs );
        8'h71: set_addr_mode_zp_ind_y( OpAdc );
        8'h72: if( CPU_VARIANT>=1 ) set_addr_mode_zp_ind( OpAdc ); else do_unknown_command();
        8'h74: if( CPU_VARIANT>=1 ) set_addr_mode_zp_x( OpStz ); else do_unknown_command();
        8'h75: set_addr_mode_zp_x( OpAdc );
        8'h76: set_addr_mode_zp_x( OpRor );
        8'h77: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpRmb7 ); else do_unknown_command();
        8'h78: set_addr_mode_implicit( OpSei );
        8'h79: set_addr_mode_abs_y( OpAdc );
        8'h7a: if( CPU_VARIANT>=1 ) set_addr_mode_stack( OpPly ); else do_unknown_command();
        8'h7c: if( CPU_VARIANT>=1 ) set_addr_mode_abs_x_ind( OpJmp ); else do_unknown_command();
        8'h7d: set_addr_mode_abs_x( OpAdc );
        8'h7e: set_addr_mode_abs_x( OpRor );
        8'h7f: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbr7 ); else do_unknown_command();
        8'h80: if( CPU_VARIANT>=1 ) set_addr_mode_implicit( OpBra ); else do_unknown_command();
        8'h81: set_addr_mode_zp_x_ind( OpSta );
        8'h84: set_addr_mode_zp( OpSty );
        8'h85: set_addr_mode_zp( OpSta );
        8'h86: set_addr_mode_zp( OpStx );
        8'h87: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpSmb0 ); else do_unknown_command();
        8'h88: set_addr_mode_implicit( OpDey );
        8'h89: if( CPU_VARIANT>=1 ) set_addr_mode_immediate( OpBit ); else do_unknown_command();
        8'h8a: set_addr_mode_implicit( OpTxa );
        8'h8c: set_addr_mode_absolute( OpSty );
        8'h8d: set_addr_mode_absolute( OpSta );
        8'h8e: set_addr_mode_absolute( OpStx );
        8'h8f: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbs0 ); else do_unknown_command();
        8'h90: set_addr_mode_implicit( OpBcc );
        8'h91: set_addr_mode_zp_ind_y( OpSta );
        8'h92: if( CPU_VARIANT>=1 ) set_addr_mode_zp_ind( OpSta ); else do_unknown_command();
        8'h94: set_addr_mode_zp_x( OpSty );
        8'h95: set_addr_mode_zp_x( OpSta );
        8'h96: set_addr_mode_zp_y( OpStx );
        8'h97: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpSmb1 ); else do_unknown_command();
        8'h98: set_addr_mode_implicit( OpTya );
        8'h99: set_addr_mode_abs_y( OpSta );
        8'h9a: set_addr_mode_implicit( OpTxs );
        8'h9c: if( CPU_VARIANT>=1 ) set_addr_mode_absolute( OpStz ); else do_unknown_command();
        8'h9d: set_addr_mode_abs_x( OpSta );
        8'h9e: if( CPU_VARIANT>=1 ) set_addr_mode_abs_x( OpStz ); else do_unknown_command();
        8'h9f: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbs1 ); else do_unknown_command();
        8'ha0: set_addr_mode_immediate( OpLdy );
        8'ha1: set_addr_mode_zp_x_ind( OpLda );
        8'ha2: set_addr_mode_immediate( OpLdx );
        8'ha4: set_addr_mode_zp( OpLdy );
        8'ha5: set_addr_mode_zp( OpLda );
        8'ha6: set_addr_mode_zp( OpLdx );
        8'ha7: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpSmb2 ); else do_unknown_command();
        8'ha8: set_addr_mode_implicit( OpTay );
        8'ha9: set_addr_mode_immediate( OpLda );
        8'haa: set_addr_mode_implicit( OpTax );
        8'hac: set_addr_mode_absolute( OpLdy );
        8'had: set_addr_mode_absolute( OpLda );
        8'hae: set_addr_mode_absolute( OpLdx );
        8'haf: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbs2 ); else do_unknown_command();
        8'hb0: set_addr_mode_implicit( OpBcs );
        8'hb1: set_addr_mode_zp_ind_y( OpLda );
        8'hb2: if( CPU_VARIANT>=1 ) set_addr_mode_zp_ind( OpLda ); else do_unknown_command();
        8'hb4: set_addr_mode_zp_x( OpLdy );
        8'hb5: set_addr_mode_zp_x( OpLda );
        8'hb6: set_addr_mode_zp_y( OpLdx );
        8'hb7: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpSmb3 ); else do_unknown_command();
        8'hb8: set_addr_mode_implicit( OpClv );
        8'hb9: set_addr_mode_abs_y( OpLda );
        8'hba: set_addr_mode_implicit( OpTsx );
        8'hbc: set_addr_mode_abs_x( OpLdy );
        8'hbd: set_addr_mode_abs_x( OpLda );
        8'hbe: set_addr_mode_abs_y( OpLdx );
        8'hbf: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbs3 ); else do_unknown_command();
        8'hc0: set_addr_mode_immediate( OpCpy );
        8'hc1: set_addr_mode_zp_x_ind( OpCmp );
        8'hc4: set_addr_mode_zp( OpCpy );
        8'hc5: set_addr_mode_zp( OpCmp );
        8'hc6: set_addr_mode_zp( OpDec );
        8'hc7: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpSmb4 ); else do_unknown_command();
        8'hc8: set_addr_mode_implicit( OpIny );
        8'hc9: set_addr_mode_immediate( OpCmp );
        8'hca: set_addr_mode_implicit( OpDex );
        8'hcb: if( CPU_VARIANT>=2 ) set_addr_mode_implicit( OpWai ); else do_unknown_command();
        8'hcc: set_addr_mode_absolute( OpCpy );
        8'hcd: set_addr_mode_absolute( OpCmp );
        8'hce: set_addr_mode_absolute( OpDec );
        8'hcf: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbs4 ); else do_unknown_command();
        8'hd0: set_addr_mode_implicit( OpBne );
        8'hd1: set_addr_mode_zp_ind_y( OpCmp );
        8'hd2: if( CPU_VARIANT>=1 ) set_addr_mode_zp_ind( OpCmp ); else do_unknown_command();
        8'hd5: set_addr_mode_zp_x( OpCmp );
        8'hd6: set_addr_mode_zp_x( OpDec );
        8'hd7: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpSmb5 ); else do_unknown_command();
        8'hd8: set_addr_mode_implicit( OpCld );
        8'hd9: set_addr_mode_abs_y( OpCmp );
        8'hda: if( CPU_VARIANT>=1 ) set_addr_mode_stack( OpPhx ); else do_unknown_command();
        8'hdb: if( CPU_VARIANT>=1 ) set_addr_mode_implicit( OpStp ); else do_unknown_command();
        8'hdd: set_addr_mode_abs_x( OpCmp );
        8'hde: set_addr_mode_abs_x( OpDec );
        8'hdf: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbs5 ); else do_unknown_command();
        8'he0: set_addr_mode_immediate( OpCpx );
        8'he1: set_addr_mode_zp_x_ind( OpSbc );
        8'he4: set_addr_mode_zp( OpCpx );
        8'he5: set_addr_mode_zp( OpSbc );
        8'he6: set_addr_mode_zp( OpInc );
        8'he7: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpSmb6 ); else do_unknown_command();
        8'he8: set_addr_mode_implicit( OpInx );
        8'he9: set_addr_mode_immediate( OpSbc );
        8'hea: set_addr_mode_implicit( OpNop );
        8'hec: set_addr_mode_absolute( OpCpx );
        8'hed: set_addr_mode_absolute( OpSbc );
        8'hee: set_addr_mode_absolute( OpInc );
        8'hef: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbs6 ); else do_unknown_command();
        8'hf0: set_addr_mode_implicit( OpBeq );
        8'hf1: set_addr_mode_zp_ind_y( OpSbc );
        8'hf2: if( CPU_VARIANT>=1 ) set_addr_mode_zp_ind( OpSbc ); else do_unknown_command();
        8'hf5: set_addr_mode_zp_x( OpSbc );
        8'hf6: set_addr_mode_zp_x( OpInc );
        8'hf7: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpSmb7 ); else do_unknown_command();
        8'hf8: set_addr_mode_implicit( OpSed );
        8'hf9: set_addr_mode_abs_y( OpSbc );
        8'hfa: if( CPU_VARIANT>=1 ) set_addr_mode_stack( OpPlx ); else do_unknown_command();
        8'hfd: set_addr_mode_abs_x( OpSbc );
        8'hfe: set_addr_mode_abs_x( OpInc );
        8'hff: if( CPU_VARIANT>=1 ) set_addr_mode_zp( OpBbs7 ); else do_unknown_command();
        default: do_unknown_command();
    endcase
endtask

task do_unknown_command();
    set_invalid_state();
endtask

task do_addr_lookup();
    case( active_addr_mode )
        AddrAbsolute: do_addr_mode_absolute();
        AddrAbsoluteInd: do_addr_mode_abs_x_ind();
        AddrAbsoluteX: do_addr_mode_abs_x();
        AddrAbsoluteY: do_addr_mode_abs_y();
        AddrAbsoluteXInd: do_addr_mode_abs_x_ind();
        AddrZeroPage: do_addr_mode_zp();
        AddrZeroPageX: do_addr_mode_zp_x();
        AddrZeroPageY: do_addr_mode_zp_x();
        AddrZeroPageInd: do_addr_mode_zp_ind();
        AddrZeroPageXInd: do_addr_mode_zp_x_ind();
        AddrZeroPageIndY: do_addr_mode_zp_ind_y();
        AddrStack: do_addr_mode_stack();
        default: set_invalid_state();
    endcase
endtask

task addr_bus_pc();
    address_bus_low_source = bus_sources::AddrBusLowSrc_PC;
    address_bus_high_source = bus_sources::AddrBusHighSrc_PC;
endtask

task addr_bus_sp();
    address_bus_low_source = bus_sources::AddrBusLowSrc_SP;
    address_bus_high_source = bus_sources::AddrBusHighSrc_One;
endtask

task addr_bus_dl();
    address_bus_low_source = bus_sources::AddrBusLowSrc_DataLatch;
    address_bus_high_source = bus_sources::AddrBusHighSrc_DataLatch;
endtask

task addr_bus_dl_mem();
    address_bus_low_source = bus_sources::AddrBusLowSrc_DataLatch;
    address_bus_high_source = bus_sources::AddrBusHighSrc_Mem;
endtask

task set_addr_mode_absolute(operations current_op);
    active_op_next = current_op;
    active_addr_mode_next = AddrAbsolute;
    ctrl_signals[control_signals::PC_ADVANCE] = 1;
endtask

task do_addr_mode_absolute();
    case( op_cycle )
        CycleAddr1: begin
            data_latch_low_source = bus_sources::DataLatchLowSource_Mem;
            ctrl_signals[control_signals::LOAD_DataLow] = 1;
            ctrl_signals[control_signals::PC_ADVANCE] = 1;

            addr_bus_pc();
        end
        CycleAddr2: begin
            data_latch_high_source = bus_sources::DataLatchHighSource_Mem;
            ctrl_signals[control_signals::LOAD_DataHigh] = 1;

            addr_bus_dl_mem();

            set_operation( active_op );
        end
        default: set_invalid_state();
    endcase
endtask

task set_addr_mode_abs_ind(operations current_op);
    active_op_next = current_op;
    active_addr_mode_next = AddrAbsoluteInd;

    ctrl_signals[control_signals::PC_ADVANCE] = 1;
endtask

task set_addr_mode_abs_x_ind(operations current_op);
    active_op_next = current_op;
    active_addr_mode_next = AddrAbsoluteXInd;

    ctrl_signals[control_signals::PC_ADVANCE] = 1;
endtask

task do_addr_mode_abs_x_ind();
    // Load directly to the PC
    case( op_cycle )
        CycleAddr1: begin
            // Vector LSB in data_in_latched
            alu_op = control_signals::AluOp_add;
            alu_a_source = active_addr_mode==AddrAbsoluteXInd ? bus_sources::AluASourceCtl_X : bus_sources::AluASourceCtl_Zero;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_Zero;

            addr_bus_pc();
        end
        CycleAddr2: begin
            // Vector MSB in data_in_latched
            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_b_source = bus_sources::AluBSourceCtl_Zero;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = alu_carry ? bus_sources::AluCarrySource_One : bus_sources::AluCarrySource_Zero;

            ctrl_signals[control_signals::LOAD_DataLow] = 1;
            data_latch_low_source = bus_sources::DataLatchLowSource_Alu_Latched;

            addr_bus_pc();
        end
        CycleAddr3: begin
            pc_low_source = bus_sources::PcLowSource_Dl;
            pc_high_source = bus_sources::PcHighSource_Alu_Latched;
            ctrl_signals[control_signals::PC_LOAD] = 1;
            ctrl_signals[control_signals::PC_ADVANCE] = 1;

            // Address of destination LSB
            addr_bus_pc();
        end
        CycleAddr4: begin
            // Destination LSB in data_in_latched

            addr_bus_pc();

            ctrl_signals[control_signals::LOAD_DataLow] = 1;
            data_latch_low_source = bus_sources::DataLatchLowSource_Mem;
        end
        CycleAddr5: begin
            // Destination MSB in data_in_latched
            do_fetch_cycle();

            pc_low_source = bus_sources::PcLowSource_Dl;
            pc_high_source = bus_sources::PcHighSource_Mem;

            ctrl_signals[control_signals::PC_LOAD] = 1;
        end
        default: set_invalid_state();
    endcase
endtask

task set_addr_mode_abs_x(operations current_op);
    active_op_next = current_op;
    active_addr_mode_next = AddrAbsoluteX;
    ctrl_signals[control_signals::PC_ADVANCE] = 1;
endtask

task do_addr_mode_abs_x();
    case( op_cycle )
        CycleAddr1: begin
            addr_bus_pc();

            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_X;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_Zero;

            data_latch_low_source = bus_sources::DataLatchLowSource_Alu;
            ctrl_signals[control_signals::LOAD_DataLow] = 1;
        end
        CycleAddr2: begin
            if( alu_carry ) begin
                addr_bus_pc();

                alu_op = control_signals::AluOp_add;
                alu_a_source = bus_sources::AluASourceCtl_Mem;
                alu_b_source = bus_sources::AluBSourceCtl_Zero;
                ctrl_signals[control_signals::AluBInverse] = 0;
                alu_carry_source = bus_sources::AluCarrySource_One;

                data_latch_high_source = bus_sources::DataLatchHighSource_Alu;
                ctrl_signals[control_signals::LOAD_DataHigh] = 1;
            end else begin
                data_latch_high_source = bus_sources::DataLatchHighSource_Mem;
                ctrl_signals[control_signals::LOAD_DataHigh] = 1;

                addr_bus_dl_mem();

                ctrl_signals[control_signals::PC_ADVANCE] = 1;
                set_operation( active_op );
            end
        end
        CycleAddr3: begin
            addr_bus_dl();

            ctrl_signals[control_signals::PC_ADVANCE] = 1;
            set_operation( active_op );
        end
        default: set_invalid_state();
    endcase
endtask

task set_addr_mode_abs_y(operations current_op);
    active_op_next = current_op;
    active_addr_mode_next = AddrAbsoluteY;
    ctrl_signals[control_signals::PC_ADVANCE] = 1;
endtask

task do_addr_mode_abs_y();
    case( op_cycle )
        CycleAddr1: begin
            addr_bus_pc();

            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_Y;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_Zero;

            data_latch_low_source = bus_sources::DataLatchLowSource_Alu;
            ctrl_signals[control_signals::LOAD_DataLow] = 1;
        end
        CycleAddr2: begin
            if( alu_carry ) begin
                addr_bus_pc();

                alu_op = control_signals::AluOp_add;
                alu_a_source = bus_sources::AluASourceCtl_Mem;
                alu_b_source = bus_sources::AluBSourceCtl_Zero;
                ctrl_signals[control_signals::AluBInverse] = 0;
                alu_carry_source = bus_sources::AluCarrySource_One;

                data_latch_high_source = bus_sources::DataLatchHighSource_Alu;
                ctrl_signals[control_signals::LOAD_DataHigh] = 1;
            end else begin
                data_latch_high_source = bus_sources::DataLatchHighSource_Mem;
                ctrl_signals[control_signals::LOAD_DataHigh] = 1;

                addr_bus_dl_mem();

                ctrl_signals[control_signals::PC_ADVANCE] = 1;
                set_operation( active_op );
            end
        end
        CycleAddr3: begin
            addr_bus_dl();

            ctrl_signals[control_signals::PC_ADVANCE] = 1;
            set_operation( active_op );
        end
        default: set_invalid_state();
    endcase
endtask

task set_addr_mode_zp_x_ind(operations current_op);
    active_op_next = current_op;
    active_addr_mode_next = AddrZeroPageXInd;
endtask

task do_addr_mode_zp_x_ind();
    case( op_cycle )
        CycleAddr1: begin
            addr_bus_pc();

            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_X;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_Zero;

            data_latch_high_source = bus_sources::DataLatchHighSource_Alu;
            ctrl_signals[control_signals::LOAD_DataHigh] = 1;
        end
        CycleAddr2: begin
            address_bus_low_source = bus_sources::AddrBusLowSrc_DataLatch_High;
            address_bus_high_source = bus_sources::AddrBusHighSrc_Zero;

            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_DataLatchHigh;
            alu_b_source = bus_sources::AluBSourceCtl_Zero;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_One;

            data_latch_high_source = bus_sources::DataLatchHighSource_Alu;
            ctrl_signals[control_signals::LOAD_DataHigh] = 1;

            ctrl_signals[control_signals::PC_ADVANCE] = 1;
        end
        CycleAddr3: begin
            data_latch_low_source = bus_sources::DataLatchLowSource_Mem;
            ctrl_signals[control_signals::LOAD_DataLow] = 1;

            address_bus_low_source = bus_sources::AddrBusLowSrc_DataLatch_High;
            address_bus_high_source = bus_sources::AddrBusHighSrc_Zero;
        end
        CycleAddr4: begin
            data_latch_high_source = bus_sources::DataLatchHighSource_Mem;
            ctrl_signals[control_signals::LOAD_DataHigh] = 1;

            addr_bus_dl_mem();

            set_operation(active_op);
        end
        default: set_invalid_state();
    endcase
endtask

task set_addr_mode_zp_ind_y(operations current_op);
    active_op_next = current_op;
    active_addr_mode_next = AddrZeroPageIndY;

    ctrl_signals[control_signals::PC_ADVANCE] = 1;
endtask

task do_addr_mode_zp_ind_y();
    case(op_cycle)
        CycleAddr1: begin
            address_bus_low_source = bus_sources::AddrBusLowSrc_Mem;
            address_bus_high_source = bus_sources::AddrBusHighSrc_Zero;

            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_b_source = bus_sources::AluBSourceCtl_Zero;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_One;

            data_latch_high_source = bus_sources::DataLatchHighSource_Alu;
            ctrl_signals[control_signals::LOAD_DataHigh] = 1;
        end
        CycleAddr2: begin
            address_bus_low_source = bus_sources::AddrBusLowSrc_DataLatch_High;
            address_bus_high_source = bus_sources::AddrBusHighSrc_Zero;

            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_Y;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_Zero;

            data_latch_low_source = bus_sources::DataLatchLowSource_Alu;
            ctrl_signals[control_signals::LOAD_DataLow] = 1;
        end
        CycleAddr3: begin
            if( !alu_carry ) begin
                addr_bus_dl_mem();

                data_latch_high_source = bus_sources::DataLatchHighSource_Mem;
                ctrl_signals[control_signals::LOAD_DataHigh] = 1;

                set_operation( active_op );
            end else begin
                address_bus_low_source = bus_sources::AddrBusLowSrc_DataLatch_High;
                address_bus_high_source = bus_sources::AddrBusHighSrc_Zero;

                alu_op = control_signals::AluOp_add;
                alu_a_source = bus_sources::AluASourceCtl_Mem;
                alu_b_source = bus_sources::AluBSourceCtl_Zero;
                ctrl_signals[control_signals::AluBInverse] = 0;
                alu_carry_source = bus_sources::AluCarrySource_One;

                data_latch_high_source = bus_sources::DataLatchHighSource_Alu;
                ctrl_signals[control_signals::LOAD_DataHigh] = 1;
            end
        end
        CycleAddr4: begin
            addr_bus_dl();

            set_operation( active_op );
        end
        default: set_invalid_state();
    endcase
endtask

task do_addr_mode_zp_ind_y_sta();
    case(op_cycle)
        CycleAddr1: begin
            address_bus_low_source = bus_sources::AddrBusLowSrc_Mem;
            address_bus_high_source = bus_sources::AddrBusHighSrc_Zero;

            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_b_source = bus_sources::AluBSourceCtl_Zero;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_One;
        end
        CycleAddr2: begin
            data_latch_high_source = bus_sources::DataLatchHighSource_Alu_Latched;
            ctrl_signals[control_signals::LOAD_DataHigh] = 1;

            address_bus_low_source = bus_sources::AddrBusLowSrc_Alu;
            address_bus_high_source = bus_sources::AddrBusHighSrc_Zero;

            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_Y;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_Zero;
        end
        CycleAddr3: begin
            address_bus_low_source = bus_sources::AddrBusLowSrc_DataLatch_High;
            address_bus_high_source = bus_sources::AddrBusHighSrc_Zero;

            data_latch_low_source = bus_sources::DataLatchLowSource_Alu_Latched;
            ctrl_signals[control_signals::LOAD_DataLow] = 1;

            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_b_source = bus_sources::AluBSourceCtl_Zero;
            alu_carry_source = alu_carry ? bus_sources::AluCarrySource_One : bus_sources::AluCarrySource_Zero;
            ctrl_signals[control_signals::AluBInverse] = 0;
        end
        CycleAddr4: begin
            address_bus_low_source = bus_sources::AddrBusLowSrc_DataLatch;
            address_bus_high_source = bus_sources::AddrBusHighSrc_Alu;

            data_latch_high_source = bus_sources::DataLatchHighSource_Alu_Latched;
            ctrl_signals[control_signals::LOAD_DataHigh] = 1;

            set_operation( active_op );
        end
        default: set_invalid_state();
    endcase
endtask

task set_addr_mode_immediate(operations current_op);
    active_addr_mode_next = AddrImmediate;
    ctrl_signals[control_signals::PC_ADVANCE] = 1;

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
    alu_a_source = bus_sources::AluASourceCtl_SP;
    alu_b_source = bus_sources::AluBSourceCtl_Zero;
    ctrl_signals[control_signals::AluBInverse] = 1;
    alu_carry_source = bus_sources::AluCarrySource_Zero;
    alu_op = control_signals::AluOp_add;

    stack_pointer_source = bus_sources::StackPointerSource_Alu;
    ctrl_signals[control_signals::LOAD_SP] = 1;
endtask

task sp_inc();
    // Add 1 to stack pointer
    alu_op = control_signals::AluOp_add;
    alu_a_source = bus_sources::AluASourceCtl_SP;
    alu_b_source = bus_sources::AluBSourceCtl_Zero;
    ctrl_signals[control_signals::AluBInverse] = 0;
    alu_carry_source = bus_sources::AluCarrySource_One;

    stack_pointer_source = bus_sources::StackPointerSource_Alu;
    ctrl_signals[control_signals::LOAD_SP] = 1;
endtask

task set_addr_mode_zp(operations current_op);
    active_op_next = current_op;
    active_addr_mode_next = AddrZeroPage;

    ctrl_signals[control_signals::PC_ADVANCE] = 1;
endtask

task do_addr_mode_zp();
    case( op_cycle )
        CycleAddr1: begin
            address_bus_low_source = bus_sources::AddrBusLowSrc_Mem;
            address_bus_high_source = bus_sources::AddrBusHighSrc_Zero;

            data_latch_low_source = bus_sources::DataLatchLowSource_Mem;
            ctrl_signals[control_signals::LOAD_DataLow] = 1;
            data_latch_high_source = bus_sources::DataLatchHighSource_Zero;
            ctrl_signals[control_signals::LOAD_DataHigh] = 1;

            set_operation(active_op);
        end
        default: set_invalid_state();
    endcase
endtask

task set_addr_mode_zp_x(operations current_op);
    active_op_next = current_op;
    active_addr_mode_next = AddrZeroPageX;
endtask

task set_addr_mode_zp_y(operations current_op);
    active_op_next = current_op;
    active_addr_mode_next = AddrZeroPageY;
endtask

task do_addr_mode_zp_x();
    case( op_cycle )
        CycleAddr1: begin
            addr_bus_pc();

            alu_op = control_signals::AluOp_add;
            if( active_addr_mode==AddrZeroPageY )
                alu_a_source = bus_sources::AluASourceCtl_Y;
            else
                alu_a_source = bus_sources::AluASourceCtl_X;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_Zero;

            data_latch_low_source = bus_sources::DataLatchLowSource_Alu;
            ctrl_signals[control_signals::LOAD_DataLow] = 1;

            data_latch_high_source = bus_sources::DataLatchHighSource_Zero;
            ctrl_signals[control_signals::LOAD_DataHigh] = 1;

            ctrl_signals[control_signals::PC_ADVANCE] = 1;
        end
        CycleAddr2: begin
            addr_bus_dl();

            set_operation( active_op );
        end
        default: set_invalid_state();
    endcase
endtask

task set_addr_mode_zp_ind(operations current_op);
    active_op_next = current_op;
    active_addr_mode_next = AddrZeroPageInd;

    ctrl_signals[control_signals::PC_ADVANCE] = 1;
endtask

task do_addr_mode_zp_ind();
    case( op_cycle )
        CycleAddr1: begin
            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_b_source = bus_sources::AluBSourceCtl_Zero;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_One;

            data_latch_high_source = bus_sources::DataLatchHighSource_Alu;
            ctrl_signals[control_signals::LOAD_DataHigh] = 1;

            address_bus_low_source = bus_sources::AddrBusLowSrc_Mem;
            address_bus_high_source = bus_sources::AddrBusHighSrc_Zero;
        end
        CycleAddr2: begin
            address_bus_low_source = bus_sources::AddrBusLowSrc_DataLatch_High;
            address_bus_high_source = bus_sources::AddrBusHighSrc_Zero;

            data_latch_low_source = bus_sources::DataLatchLowSource_Mem;
            ctrl_signals[control_signals::LOAD_DataLow] = 1;
        end
        CycleAddr3: begin
            addr_bus_dl_mem();

            set_operation( active_op );
        end
        default: set_invalid_state();
    endcase
endtask

task set_operation(operations current_op);
    active_op_next = current_op;
    op_cycle_next = FirstOpCycle;

    case( current_op )
        OpAdc: do_op_adc_first();
        OpAnd: do_op_and_first();
        OpAsl: do_op_asl_first();
        OpAslA: do_op_asl_acc_first();
        OpBbr0: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(0, 0); else set_invalid_state();
        OpBbr1: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(1, 0); else set_invalid_state();
        OpBbr2: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(2, 0); else set_invalid_state();
        OpBbr3: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(3, 0); else set_invalid_state();
        OpBbr4: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(4, 0); else set_invalid_state();
        OpBbr5: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(5, 0); else set_invalid_state();
        OpBbr6: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(6, 0); else set_invalid_state();
        OpBbr7: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(7, 0); else set_invalid_state();
        OpBbs0: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(0, 1); else set_invalid_state();
        OpBbs1: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(1, 1); else set_invalid_state();
        OpBbs2: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(2, 1); else set_invalid_state();
        OpBbs3: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(3, 1); else set_invalid_state();
        OpBbs4: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(4, 1); else set_invalid_state();
        OpBbs5: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(5, 1); else set_invalid_state();
        OpBbs6: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(6, 1); else set_invalid_state();
        OpBbs7: if( CPU_VARIANT>=1 ) do_op_branch_bit_first(7, 1); else set_invalid_state();
        OpBcc: do_op_bcc_first();
        OpBcs: do_op_bcs_first();
        OpBeq: do_op_beq_first();
        OpBit: do_op_bit_first();
        OpBmi: do_op_bmi_first();
        OpBne: do_op_bne_first();
        OpBpl: do_op_bpl_first();
        OpBra: if( CPU_VARIANT>=1 ) do_op_bra_first(); else set_invalid_state();
        OpBrk: do_op_brk_first();
        OpBvc: do_op_bvc_first();
        OpBvs: do_op_bvs_first();
        OpClc: do_op_clc_first();
        OpCld: do_op_cld_first();
        OpCli: do_op_cli_first();
        OpClv: do_op_clv_first();
        OpCmp: do_op_cmp_first();
        OpCpx: do_op_cpx_first();
        OpCpy: do_op_cpy_first();
        OpDec: do_op_dec_first();
        OpDecA: do_op_dec_acc_first();
        OpDex: do_op_dex_first();
        OpDey: do_op_dey_first();
        OpEor: do_op_eor_first();
        OpInc: do_op_inc_first();
        OpIncA: do_op_inc_acc_first();
        OpInx: do_op_inx_first();
        OpIny: do_op_iny_first();
        OpJmp: do_op_jmp_first();
        OpJsr: do_op_jsr_first();
        OpLda: do_op_lda_first();
        OpLdx: do_op_ldx_first();
        OpLdy: do_op_ldy_first();
        OpLsr: do_op_lsr_first();
        OpLsrA: do_op_lsr_acc_first();
        OpNop: do_op_nop_first();
        OpOra: do_op_ora_first();
        OpPha: do_op_pha_first();
        OpPhp: do_op_php_first();
        OpPhx: if( CPU_VARIANT>=1 ) do_op_phx_first(); else set_invalid_state();
        OpPhy: if( CPU_VARIANT>=1 ) do_op_phy_first(); else set_invalid_state();
        OpPla: do_op_pla_first();
        OpPlp: do_op_plp_first();
        OpPlx: if( CPU_VARIANT>=1 ) do_op_plx_first(); else set_invalid_state();
        OpPly: if( CPU_VARIANT>=1 ) do_op_ply_first(); else set_invalid_state();
        OpRmb0: if( CPU_VARIANT>=1 ) do_op_rmb_first(0); else set_invalid_state();
        OpRmb1: if( CPU_VARIANT>=1 ) do_op_rmb_first(1); else set_invalid_state();
        OpRmb2: if( CPU_VARIANT>=1 ) do_op_rmb_first(2); else set_invalid_state();
        OpRmb3: if( CPU_VARIANT>=1 ) do_op_rmb_first(3); else set_invalid_state();
        OpRmb4: if( CPU_VARIANT>=1 ) do_op_rmb_first(4); else set_invalid_state();
        OpRmb5: if( CPU_VARIANT>=1 ) do_op_rmb_first(5); else set_invalid_state();
        OpRmb6: if( CPU_VARIANT>=1 ) do_op_rmb_first(6); else set_invalid_state();
        OpRmb7: if( CPU_VARIANT>=1 ) do_op_rmb_first(7); else set_invalid_state();
        OpRol: do_op_rol_first();
        OpRolA: do_op_rol_acc_first();
        OpRor: do_op_ror_first();
        OpRorA: do_op_ror_acc_first();
        OpRti: do_op_rti_first();
        OpRts: do_op_rts_first();
        OpSbc: do_op_sbc_first();
        OpSec: do_op_sec_first();
        OpSed: do_op_sed_first();
        OpSei: do_op_sei_first();
        OpSmb0: if( CPU_VARIANT>=1 ) do_op_smb_first(0); else set_invalid_state();
        OpSmb1: if( CPU_VARIANT>=1 ) do_op_smb_first(1); else set_invalid_state();
        OpSmb2: if( CPU_VARIANT>=1 ) do_op_smb_first(2); else set_invalid_state();
        OpSmb3: if( CPU_VARIANT>=1 ) do_op_smb_first(3); else set_invalid_state();
        OpSmb4: if( CPU_VARIANT>=1 ) do_op_smb_first(4); else set_invalid_state();
        OpSmb5: if( CPU_VARIANT>=1 ) do_op_smb_first(5); else set_invalid_state();
        OpSmb6: if( CPU_VARIANT>=1 ) do_op_smb_first(6); else set_invalid_state();
        OpSmb7: if( CPU_VARIANT>=1 ) do_op_smb_first(7); else set_invalid_state();
        OpSta: do_op_sta_first();
        OpStp: if( CPU_VARIANT>=1 ) do_op_stop_first(); else set_invalid_state();
        OpStx: do_op_stx_first();
        OpSty: do_op_sty_first();
        OpStz: if( CPU_VARIANT>=1 ) do_op_stz_first(); else set_invalid_state();
        OpTax: do_op_transfer_first(bus_sources::DataBusSrc_A, control_signals::LOAD_X);
        OpTay: do_op_transfer_first(bus_sources::DataBusSrc_A, control_signals::LOAD_Y);
        OpTrb: if( CPU_VARIANT>=1 ) do_op_trb_first(); else set_invalid_state();
        OpTsb: if( CPU_VARIANT>=1 ) do_op_tsb_first(); else set_invalid_state();
        OpTsx: do_op_transfer_first(bus_sources::DataBusSrc_SP, control_signals::LOAD_X);
        OpTxa: do_op_transfer_first(bus_sources::DataBusSrc_X, control_signals::LOAD_A);
        OpTxs: do_op_txs_first();
        OpTya: do_op_transfer_first(bus_sources::DataBusSrc_Y, control_signals::LOAD_A);
        OpWai: if( CPU_VARIANT>=1 ) do_op_wai_first(); else set_invalid_state();
        default: set_invalid_state();
    endcase
endtask

task do_operation();
    case( active_op )
        OpAdc: do_op_adc();
        OpAnd: do_op_and();
        OpAsl: do_op_asl();
        OpAslA: do_op_asl_acc();
        OpBbr0: do_op_branch_bit(0, 0);
        OpBbr1: do_op_branch_bit(1, 0);
        OpBbr2: do_op_branch_bit(2, 0);
        OpBbr3: do_op_branch_bit(3, 0);
        OpBbr4: do_op_branch_bit(4, 0);
        OpBbr5: do_op_branch_bit(5, 0);
        OpBbr6: do_op_branch_bit(6, 0);
        OpBbr7: do_op_branch_bit(7, 0);
        OpBbs0: do_op_branch_bit(0, 1);
        OpBbs1: do_op_branch_bit(1, 1);
        OpBbs2: do_op_branch_bit(2, 1);
        OpBbs3: do_op_branch_bit(3, 1);
        OpBbs4: do_op_branch_bit(4, 1);
        OpBbs5: do_op_branch_bit(5, 1);
        OpBbs6: do_op_branch_bit(6, 1);
        OpBbs7: do_op_branch_bit(7, 1);
        OpBcc: do_branch();
        OpBcs: do_branch();
        OpBeq: do_branch();
        OpBit: do_op_bit();
        OpBmi: do_branch();
        OpBne: do_branch();
        OpBpl: do_branch();
        OpBra: do_branch();
        OpBrk: do_op_brk();
        OpBvc: do_branch();
        OpBvs: do_branch();
        OpCmp: do_op_cmp();
        OpCpx: do_op_cpx();
        OpCpy: do_op_cpy();
        OpDec: do_op_dec();
        OpDecA: do_op_dec_acc();
        OpDex: do_op_dex();
        OpDey: do_op_dey();
        OpEor: do_op_eor();
        OpInc: do_op_inc();
        OpIncA: do_op_inc_acc();
        OpInx: do_op_inx();
        OpIny: do_op_iny();
        OpJmp: do_op_jmp();
        OpJsr: do_op_jsr();
        OpLda: do_op_lda();
        OpLdx: do_op_ldx();
        OpLdy: do_op_ldy();
        OpLsr: do_op_lsr();
        OpLsrA: do_op_lsr_acc();
        OpOra: do_op_ora();
        OpPla: do_op_pla();
        OpPlp: do_op_plp();
        OpPlx: do_op_plx();
        OpPly: do_op_ply();
        OpRmb0: do_op_rmb(0);
        OpRmb1: do_op_rmb(1);
        OpRmb2: do_op_rmb(2);
        OpRmb3: do_op_rmb(3);
        OpRmb4: do_op_rmb(4);
        OpRmb5: do_op_rmb(5);
        OpRmb6: do_op_rmb(6);
        OpRmb7: do_op_rmb(7);
        OpRol: do_op_rol();
        OpRolA: do_op_rol_acc();
        OpRor: do_op_ror();
        OpRorA: do_op_ror_acc();
        OpRti: do_op_rti();
        OpRts: do_op_rts();
        OpSbc: do_op_sbc();
        OpSmb0: do_op_smb(0);
        OpSmb1: do_op_smb(1);
        OpSmb2: do_op_smb(2);
        OpSmb3: do_op_smb(3);
        OpSmb4: do_op_smb(4);
        OpSmb5: do_op_smb(5);
        OpSmb6: do_op_smb(6);
        OpSmb7: do_op_smb(7);
        OpSta: do_op_sta();
        OpStp: do_op_stop();
        OpStz: do_op_stz();
        OpTrb: do_op_trb();
        OpTsb: do_op_tsb();
        OpWai: do_op_wai();
        default: set_invalid_state();
    endcase
endtask

task next_instruction();
    op_cycle_next = CycleFetch;
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
    ctrl_signals[control_signals::PC_ADVANCE] = 1;
    if( ! condition ) begin
        next_instruction();
    end
endtask

task do_branch();
    case( op_cycle )
        FirstOpCycle: begin
            addr_bus_pc();

            data_latch_low_source = bus_sources::DataLatchLowSource_Alu;
            ctrl_signals[control_signals::LOAD_DataLow] = 1;
            data_latch_high_source = bus_sources::DataLatchHighSource_PC;
            ctrl_signals[control_signals::LOAD_DataHigh] = 1;

            alu_a_source = bus_sources::AluASourceCtl_PC_Low;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_op = control_signals::AluOp_add;
            alu_carry_source = bus_sources::AluCarrySource_Zero;
        end
        CycleOp2: begin
            if( branch_offset_negative==0 && !alu_carry || branch_offset_negative==1 && alu_carry ) begin
                do_fetch_cycle();

                pc_low_source = bus_sources::PcLowSource_Dl;
                pc_high_source = bus_sources::PcHighSource_CurrentValue;
                ctrl_signals[control_signals::PC_LOAD] = 1;
            end else begin
                addr_bus_pc();

                alu_a_source = bus_sources::AluASourceCtl_DataLatchHigh;
                alu_b_source = bus_sources::AluBSourceCtl_Zero;
                ctrl_signals[control_signals::AluBInverse] = 0;
                alu_op = control_signals::AluOp_add;

                if( branch_offset_negative ) begin
                    ctrl_signals[control_signals::AluBInverse] = 1;
                    alu_carry_source = bus_sources::AluCarrySource_Zero;
                end else begin
                    alu_carry_source = bus_sources::AluCarrySource_One;
                end

                data_latch_high_source = bus_sources::DataLatchHighSource_Alu;
                ctrl_signals[control_signals::LOAD_DataHigh] = 1;
            end
        end
        CycleOp3: begin
            do_fetch_cycle();

            pc_low_source = bus_sources::PcLowSource_Dl;
            pc_high_source = bus_sources::PcHighSource_Dl;
            ctrl_signals[control_signals::PC_LOAD] = 1;
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_branch_bit_first( input int bitnum, input desired );
endtask

task do_op_branch_bit( input int bitnum, input desired );
    case( op_cycle )
        FirstOpCycle: begin
            addr_bus_dl();
        end
        CycleOp2: begin
            addr_bus_pc();
            op_cycle_next = FirstOpCycle;
            active_op_next = OpBra;
            do_branch_first( memory_in[bitnum] == desired );
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_adc_first();
endtask

task do_op_adc();
    case( op_cycle )
        FirstOpCycle: begin
            alu_a_source = bus_sources::AluASourceCtl_A;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_Carry;
            alu_op = control_signals::AluOp_add;

            data_bus_source = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagV] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagC] = 1;
            ctrl_signals[control_signals::UseAluFlags] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::LOAD_A] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_sbc_first();
endtask

task do_op_sbc();
    case( op_cycle )
        FirstOpCycle: begin
            alu_a_source = bus_sources::AluASourceCtl_A;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 1;
            alu_carry_source = bus_sources::AluCarrySource_Carry;
            alu_op = control_signals::AluOp_add;

            data_bus_source = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagV] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagC] = 1;
            ctrl_signals[control_signals::UseAluFlags] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::LOAD_A] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_and_first();
endtask

task do_op_and();
    case( op_cycle )
        FirstOpCycle: begin
            alu_a_source = bus_sources::AluASourceCtl_A;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_Carry;
            alu_op = control_signals::AluOp_and;

            data_bus_source = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::LOAD_A] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_ora_first();
endtask

task do_op_ora();
    case( op_cycle )
        FirstOpCycle: begin
            alu_a_source = bus_sources::AluASourceCtl_A;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_Carry;
            alu_op = control_signals::AluOp_or;

            data_bus_source = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::LOAD_A] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_eor_first();
endtask

task do_op_eor();
    case( op_cycle )
        FirstOpCycle: begin
            alu_op = control_signals::AluOp_xor;
            alu_a_source = bus_sources::AluASourceCtl_A;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 0;

            data_bus_source = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::LOAD_A] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_asl_first();
    ML = 0;
endtask

task do_op_asl();
    case(op_cycle)
        FirstOpCycle: begin
            ML = 0;

            addr_bus_dl();

            alu_op = control_signals::AluOp_shift_left;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_carry_source = bus_sources::AluCarrySource_Zero;

            ctrl_signals[control_signals::UpdateFlagC] = 1;
            ctrl_signals[control_signals::UseAluFlags] = 1;
        end
        CycleOp2: begin
            ML = 0;

            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;
            addr_bus_dl();
            rW = 0;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_asl_acc_first();
    alu_op = control_signals::AluOp_shift_left;
    alu_a_source = bus_sources::AluASourceCtl_A;
    alu_carry_source = bus_sources::AluCarrySource_Zero;

    ctrl_signals[control_signals::UpdateFlagC] = 1;
    ctrl_signals[control_signals::UseAluFlags] = 1;

    data_latch_low_source = bus_sources::DataLatchLowSource_Alu;
    ctrl_signals[control_signals::LOAD_DataLow] = 1;
endtask

task do_op_asl_acc();
    case( op_cycle )
        FirstOpCycle: begin
            data_bus_source = bus_sources::DataBusSrc_Dl_Low;
            ctrl_signals[control_signals::LOAD_A] = 1;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_lsr_first();
    ML = 0;
endtask

task do_op_lsr();
    case(op_cycle)
        FirstOpCycle: begin
            ML = 0;

            addr_bus_dl();

            alu_op = control_signals::AluOp_shift_right_logical;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_carry_source = bus_sources::AluCarrySource_Zero;

            ctrl_signals[control_signals::UpdateFlagC] = 1;
            ctrl_signals[control_signals::UseAluFlags] = 1;
        end
        CycleOp2: begin
            ML = 0;

            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;
            addr_bus_dl();
            rW = 0;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_lsr_acc_first();
    alu_op = control_signals::AluOp_shift_right_logical;
    alu_a_source = bus_sources::AluASourceCtl_A;
    alu_carry_source = bus_sources::AluCarrySource_Zero;

    ctrl_signals[control_signals::UpdateFlagC] = 1;
    ctrl_signals[control_signals::UseAluFlags] = 1;

    data_latch_low_source = bus_sources::DataLatchLowSource_Alu;
    ctrl_signals[control_signals::LOAD_DataLow] = 1;
endtask

task do_op_lsr_acc();
    case( op_cycle )
        FirstOpCycle: begin
            data_bus_source = bus_sources::DataBusSrc_Dl_Low;
            ctrl_signals[control_signals::LOAD_A] = 1;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_rol_first();
    ML = 0;
endtask

task do_op_rol();
    case(op_cycle)
        FirstOpCycle: begin
            ML = 0;

            addr_bus_dl();

            alu_op = control_signals::AluOp_shift_left;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_carry_source = bus_sources::AluCarrySource_Carry;

            ctrl_signals[control_signals::UpdateFlagC] = 1;
            ctrl_signals[control_signals::UseAluFlags] = 1;
        end
        CycleOp2: begin
            ML = 0;

            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;
            addr_bus_dl();
            rW = 0;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_rol_acc_first();
    alu_op = control_signals::AluOp_shift_left;
    alu_a_source = bus_sources::AluASourceCtl_A;
    alu_carry_source = bus_sources::AluCarrySource_Carry;

    ctrl_signals[control_signals::UpdateFlagC] = 1;
    ctrl_signals[control_signals::UseAluFlags] = 1;

    data_latch_low_source = bus_sources::DataLatchLowSource_Alu;
    ctrl_signals[control_signals::LOAD_DataLow] = 1;
endtask

task do_op_rol_acc();
    case( op_cycle )
        FirstOpCycle: begin
            data_bus_source = bus_sources::DataBusSrc_Dl_Low;
            ctrl_signals[control_signals::LOAD_A] = 1;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_ror_first();
    ML = 0;
endtask

task do_op_ror();
    case(op_cycle)
        FirstOpCycle: begin
            ML = 0;

            addr_bus_dl();

            alu_op = control_signals::AluOp_shift_right_logical;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_carry_source = bus_sources::AluCarrySource_Carry;

            ctrl_signals[control_signals::UpdateFlagC] = 1;
            ctrl_signals[control_signals::UseAluFlags] = 1;
        end
        CycleOp2: begin
            ML = 0;

            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;
            addr_bus_dl();
            rW = 0;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_ror_acc_first();
    alu_op = control_signals::AluOp_shift_right_logical;
    alu_a_source = bus_sources::AluASourceCtl_A;
    alu_carry_source = bus_sources::AluCarrySource_Carry;

    ctrl_signals[control_signals::UpdateFlagC] = 1;
    ctrl_signals[control_signals::UseAluFlags] = 1;

    data_latch_low_source = bus_sources::DataLatchLowSource_Alu;
    ctrl_signals[control_signals::LOAD_DataLow] = 1;
endtask

task do_op_ror_acc();
    case( op_cycle )
        FirstOpCycle: begin
            data_bus_source = bus_sources::DataBusSrc_Dl_Low;
            ctrl_signals[control_signals::LOAD_A] = 1;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_bit_first();
    if( active_addr_mode_next != AddrImmediate ) begin
        data_bus_source = bus_sources::DataBusSrc_Mem_Unlatched;
        ctrl_signals[control_signals::UpdateFlagN] = 1;
        ctrl_signals[control_signals::UpdateFlagV] = 1;
        ctrl_signals[control_signals::UseAluFlags] = 0;
    end
endtask

task do_op_bit();
    case(op_cycle)
        FirstOpCycle: begin
            alu_op = control_signals::AluOp_and;
            alu_a_source = bus_sources::AluASourceCtl_A;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 0;

            data_bus_source = bus_sources::DataBusSrc_Alu;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_brk_first();
    incompatible = 1;

    ctrl_signals[control_signals::PC_ADVANCE] = (int_state==IntStateNone ? 1 : 0);

    address_bus_high_source = bus_sources::AddrBusHighSrc_PC;
    address_bus_low_source = bus_sources::AddrBusLowSrc_PC;
endtask

task do_op_brk();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_sp();
            rW = (int_state == IntStateReset) ? 1 : 0;
            data_bus_source = bus_sources::DataBusSrc_Pc_High;

            sp_dec();
        end
        CycleOp2: begin
            addr_bus_sp();
            rW = (int_state == IntStateReset) ? 1 : 0;
            data_bus_source = bus_sources::DataBusSrc_Pc_Low;

            sp_dec();
        end
        CycleOp3: begin
            data_latch_high_source = bus_sources::DataLatchHighSource_FF;
            ctrl_signals[control_signals::LOAD_DataHigh] = 1;

            case(int_state)
                IntStateNone: data_latch_low_source = bus_sources::DataLatchLowSource_FE;
                IntStateReset: data_latch_low_source = bus_sources::DataLatchLowSource_FC;
                IntStateNmi: data_latch_low_source = bus_sources::DataLatchLowSource_FA;
                IntStateIrq: data_latch_low_source = bus_sources::DataLatchLowSource_FE;
                default: set_invalid_state();
            endcase
            ctrl_signals[control_signals::LOAD_DataLow] = 1;

            addr_bus_sp();
            rW = (int_state == IntStateReset) ? 1 : 0;
            data_bus_source = bus_sources::DataBusSrc_Status;
            ctrl_signals[control_signals::OutputFlagB] = (int_state==IntStateNone ? 1 : 0);

            sp_dec();
        end
        CycleOp4: begin
            addr_bus_dl();
            VP = 0;

            int_state_next = IntStateNone;

            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_DataLatchLow;
            alu_b_source = bus_sources::AluBSourceCtl_Zero;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_One;

            data_latch_low_source = bus_sources::DataLatchLowSource_Alu;
            ctrl_signals[control_signals::LOAD_DataLow] = 1;

            data_bus_source = bus_sources::DataBusSrc_Ones;
            ctrl_signals[control_signals::UpdateFlagI] = 1;
        end
        CycleOp5: begin
            addr_bus_dl();
            VP = 0;

            ctrl_signals[control_signals::PC_LOAD] = 1;
            pc_high_source = bus_sources::PcHighSource_Mem;
            pc_low_source = bus_sources::PcLowSource_Mem;

            if( CPU_VARIANT>0 ) begin
                data_bus_source = bus_sources::DataBusSrc_Zero;
                ctrl_signals[control_signals::UpdateFlagD] = 1;
            end
        end
        CycleOp6: begin
            ctrl_signals[control_signals::PC_LOAD] = 1;
            pc_low_source = bus_sources::PcLowSource_CurrentValue;
            pc_high_source = bus_sources::PcHighSource_Mem;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_clc_first();
    next_instruction();

    data_bus_source = bus_sources::DataBusSrc_Zero;
    ctrl_signals[control_signals::UpdateFlagC] = 1;
    ctrl_signals[control_signals::UseAluFlags] = 0;
endtask

task do_op_cld_first();
    next_instruction();

    data_bus_source = bus_sources::DataBusSrc_Zero;
    ctrl_signals[control_signals::UpdateFlagD] = 1;
endtask

task do_op_cli_first();
    next_instruction();

    data_bus_source = bus_sources::DataBusSrc_Zero;
    ctrl_signals[control_signals::UpdateFlagI] = 1;
endtask

task do_op_clv_first();
    next_instruction();

    data_bus_source = bus_sources::DataBusSrc_Zero;
    ctrl_signals[control_signals::UpdateFlagV] = 1;
    ctrl_signals[control_signals::UseAluFlags] = 0;
endtask

task do_op_cmp_first();
endtask

task do_op_cmp();
    case( op_cycle )
        FirstOpCycle: begin
            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_A;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 1;
            alu_carry_source = bus_sources::AluCarrySource_One;

            data_bus_source = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagC] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::UseAluFlags] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_cpx_first();
endtask

task do_op_cpx();
    case( op_cycle )
        FirstOpCycle: begin
            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_X;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 1;
            alu_carry_source = bus_sources::AluCarrySource_One;

            data_bus_source = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagC] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::UseAluFlags] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_cpy_first();
endtask

task do_op_cpy();
    case( op_cycle )
        FirstOpCycle: begin
            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_Y;
            alu_b_source = bus_sources::AluBSourceCtl_Mem;
            ctrl_signals[control_signals::AluBInverse] = 1;
            alu_carry_source = bus_sources::AluCarrySource_One;

            data_bus_source = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagC] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::UseAluFlags] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_dec_first();
    ML = 0;
endtask

task do_op_dec();
    case( op_cycle )
        FirstOpCycle: begin
            addr_bus_dl();

            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_b_source = bus_sources::AluBSourceCtl_Zero;
            ctrl_signals[control_signals::AluBInverse] = 1;
            alu_carry_source = bus_sources::AluCarrySource_Zero;

            ML = 0;
        end
        CycleOp2: begin
            addr_bus_dl();

            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;
            rW = 0;
            ML = 0;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_dec_acc_first();
    alu_op = control_signals::AluOp_add;
    alu_a_source = bus_sources::AluASourceCtl_A;
    alu_b_source = bus_sources::AluBSourceCtl_Zero;
    ctrl_signals[control_signals::AluBInverse] = 1;
    alu_carry_source = bus_sources::AluCarrySource_Zero;
endtask

task do_op_dec_acc();
    case( op_cycle )
        FirstOpCycle: begin
            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;
            ctrl_signals[control_signals::LOAD_A] = 1;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_dex_first();
    alu_a_source = bus_sources::AluASourceCtl_X;
    alu_b_source = bus_sources::AluBSourceCtl_Zero;
    ctrl_signals[control_signals::AluBInverse] = 1;
    alu_carry_source = bus_sources::AluCarrySource_Zero;
    alu_op = control_signals::AluOp_add;
endtask

task do_op_dex();
    case( op_cycle )
        FirstOpCycle: begin
            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;
            ctrl_signals[control_signals::LOAD_X] = 1;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_dey_first();
    alu_a_source = bus_sources::AluASourceCtl_Y;
    alu_b_source = bus_sources::AluBSourceCtl_Zero;
    ctrl_signals[control_signals::AluBInverse] = 1;
    alu_carry_source = bus_sources::AluCarrySource_Zero;
    alu_op = control_signals::AluOp_add;
endtask

task do_op_dey();
    case( op_cycle )
        FirstOpCycle: begin
            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;
            ctrl_signals[control_signals::LOAD_Y] = 1;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_inx_first();
    alu_a_source = bus_sources::AluASourceCtl_X;
    alu_b_source = bus_sources::AluBSourceCtl_Zero;
    ctrl_signals[control_signals::AluBInverse] = 0;
    alu_carry_source = bus_sources::AluCarrySource_One;
    alu_op = control_signals::AluOp_add;
endtask

task do_op_inc_first();
    ML = 0;
endtask

task do_op_inc();
    case( op_cycle )
        FirstOpCycle: begin
            addr_bus_dl();

            alu_op = control_signals::AluOp_add;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_b_source = bus_sources::AluBSourceCtl_Zero;
            ctrl_signals[control_signals::AluBInverse] = 0;
            alu_carry_source = bus_sources::AluCarrySource_One;

            ML = 0;
        end
        CycleOp2: begin
            addr_bus_dl();

            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;
            rW = 0;
            ML = 0;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_inc_acc_first();
    alu_op = control_signals::AluOp_add;
    alu_a_source = bus_sources::AluASourceCtl_A;
    alu_b_source = bus_sources::AluBSourceCtl_Zero;
    ctrl_signals[control_signals::AluBInverse] = 0;
    alu_carry_source = bus_sources::AluCarrySource_One;
endtask

task do_op_inc_acc();
    case( op_cycle )
        FirstOpCycle: begin
            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;
            ctrl_signals[control_signals::LOAD_A] = 1;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_inx();
    case( op_cycle )
        FirstOpCycle: begin
            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;
            ctrl_signals[control_signals::LOAD_X] = 1;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_iny_first();
    alu_a_source = bus_sources::AluASourceCtl_Y;
    alu_b_source = bus_sources::AluBSourceCtl_Zero;
    ctrl_signals[control_signals::AluBInverse] = 0;
    alu_carry_source = bus_sources::AluCarrySource_One;
    alu_op = control_signals::AluOp_add;
endtask

task do_op_iny();
    case( op_cycle )
        FirstOpCycle: begin
            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;
            ctrl_signals[control_signals::LOAD_Y] = 1;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_jmp_first();
    do_fetch_cycle();

    ctrl_signals[control_signals::PC_LOAD] = 1;
    pc_low_source = bus_sources::PcLowSource_Dl;
    pc_high_source = bus_sources::PcHighSource_Mem;

    address_bus_low_source = bus_sources::AddrBusLowSrc_DataLatch;
    address_bus_high_source = bus_sources::AddrBusHighSrc_Mem;
endtask

task do_op_jmp();
endtask

task do_op_jsr_first();
    // Store destination LSB in DL
    ctrl_signals[control_signals::LOAD_DataLow] = 1;
    data_latch_low_source = bus_sources::DataLatchLowSource_Mem;

    // Dummy stack cycle while we advance the PC
    ctrl_signals[control_signals::PC_ADVANCE] = 1;
endtask

task do_op_jsr();
    case(op_cycle)
        FirstOpCycle: begin
            // Write PC MSB to stack
            addr_bus_sp();
            data_bus_source = bus_sources::DataBusSrc_Pc_High;
            rW = 0;

            sp_dec();
        end
        CycleOp2: begin
            // Write PC LSB to stack
            addr_bus_sp();
            data_bus_source = bus_sources::DataBusSrc_Pc_Low;
            rW = 0;

            sp_dec();
        end
        CycleOp3: begin
            // Address to read destination MSB
            addr_bus_pc();
        end
        CycleOp4: begin
            // Load destination MSB into PC
            pc_high_source = bus_sources::PcHighSource_Mem;
            pc_low_source = bus_sources::PcLowSource_Dl;
            ctrl_signals[control_signals::PC_LOAD] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_lda_first();
endtask

task do_op_lda();
    case( op_cycle )
        FirstOpCycle: begin
            ctrl_signals[control_signals::PC_ADVANCE] = 1;

            ctrl_signals[control_signals::LOAD_A] = 1;
            data_bus_source = bus_sources::DataBusSrc_Mem;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_ldx_first();
endtask

task do_op_ldx();
    case( op_cycle )
        FirstOpCycle: begin
            ctrl_signals[control_signals::PC_ADVANCE] = 1;

            ctrl_signals[control_signals::LOAD_X] = 1;
            data_bus_source = bus_sources::DataBusSrc_Mem;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_ldy_first();
endtask

task do_op_ldy();
    case( op_cycle )
        FirstOpCycle: begin
            ctrl_signals[control_signals::PC_ADVANCE] = 1;

            ctrl_signals[control_signals::LOAD_Y] = 1;
            data_bus_source = bus_sources::DataBusSrc_Mem;

            ctrl_signals[control_signals::UpdateFlagN] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_nop_first();
    next_instruction();
endtask

task do_op_pha_first();
    data_bus_source = bus_sources::DataBusSrc_A;
    rW = 0;

    sp_dec();

    next_instruction();
endtask

task do_op_php_first();
    rW = 0;

    data_bus_source = bus_sources::DataBusSrc_Status;
    ctrl_signals[control_signals::OutputFlagB] = 1;
    sp_dec();

    next_instruction();
endtask

task do_op_phx_first();
    rW = 0;

    data_bus_source = bus_sources::DataBusSrc_X;
    sp_dec();

    next_instruction();
endtask

task do_op_phy_first();
    rW = 0;

    data_bus_source = bus_sources::DataBusSrc_Y;
    sp_dec();

    next_instruction();
endtask

task do_op_pla_first();
    // First stack cycle: dummy read
    sp_inc();
endtask

task do_op_pla();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_sp();
        end
        CycleOp2: begin
            // This is the data loaded by the previous address operation
            data_bus_source = bus_sources::DataBusSrc_Mem;
            ctrl_signals[control_signals::LOAD_A] = 1;

            ctrl_signals[control_signals::UseAluFlags] = 0;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagN] = 1;

            do_fetch_cycle();
        end
        default set_invalid_state();
    endcase
endtask

task do_op_plx_first();
    // First stack cycle: dummy read
    sp_inc();
endtask

task do_op_plx();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_sp();
        end
        CycleOp2: begin
            // This is the data loaded by the previous address operation
            data_bus_source = bus_sources::DataBusSrc_Mem;
            ctrl_signals[control_signals::LOAD_X] = 1;

            ctrl_signals[control_signals::UseAluFlags] = 0;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagN] = 1;

            do_fetch_cycle();
        end
        default set_invalid_state();
    endcase
endtask

task do_op_ply_first();
    // First stack cycle: dummy read
    sp_inc();
endtask

task do_op_ply();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_sp();
        end
        CycleOp2: begin
            // This is the data loaded by the previous address operation
            data_bus_source = bus_sources::DataBusSrc_Mem;
            ctrl_signals[control_signals::LOAD_Y] = 1;

            ctrl_signals[control_signals::UseAluFlags] = 0;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagN] = 1;

            do_fetch_cycle();
        end
        default set_invalid_state();
    endcase
endtask

task do_op_plp_first();
    // First stack cycle: dummy read
    sp_inc();
endtask

task do_op_plp();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_sp();
        end
        CycleOp2: begin
            // This is the data loaded by the previous address operation
            data_bus_source = bus_sources::DataBusSrc_Mem;
            ctrl_signals[control_signals::UseAluFlags] = 0;
            ctrl_signals[control_signals::UpdateFlagC] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 0;
            ctrl_signals[control_signals::UpdateFlagI] = 1;
            ctrl_signals[control_signals::UpdateFlagD] = 1;
            ctrl_signals[control_signals::UpdateFlagV] = 1;
            ctrl_signals[control_signals::UpdateFlagN] = 1;

            do_fetch_cycle();
        end
        default set_invalid_state();
    endcase
endtask

task do_op_rmb_first(input bitnum);
    ML = 0;
endtask

task do_op_rmb(input bitnum);
    case( op_cycle )
        FirstOpCycle: begin
            addr_bus_dl();
            ML = 0;

            alu_op = control_signals::AluOp_and;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            case( active_op )
                OpRmb0: alu_b_source = bus_sources::AluBSourceCtl_Bit0;
                OpRmb1: alu_b_source = bus_sources::AluBSourceCtl_Bit1;
                OpRmb2: alu_b_source = bus_sources::AluBSourceCtl_Bit2;
                OpRmb3: alu_b_source = bus_sources::AluBSourceCtl_Bit3;
                OpRmb4: alu_b_source = bus_sources::AluBSourceCtl_Bit4;
                OpRmb5: alu_b_source = bus_sources::AluBSourceCtl_Bit5;
                OpRmb6: alu_b_source = bus_sources::AluBSourceCtl_Bit6;
                OpRmb7: alu_b_source = bus_sources::AluBSourceCtl_Bit7;
                default: set_invalid_state();
            endcase
            ctrl_signals[control_signals::AluBInverse] = 1;
        end
        CycleOp2: begin
            addr_bus_dl();
            ML = 0;
            rW = 0;

            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_smb_first(input bitnum);
    ML = 0;
endtask

task do_op_smb(input bitnum);
    case( op_cycle )
        FirstOpCycle: begin
            addr_bus_dl();
            ML = 0;

            alu_op = control_signals::AluOp_or;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            case( active_op )
                OpSmb0: alu_b_source = bus_sources::AluBSourceCtl_Bit0;
                OpSmb1: alu_b_source = bus_sources::AluBSourceCtl_Bit1;
                OpSmb2: alu_b_source = bus_sources::AluBSourceCtl_Bit2;
                OpSmb3: alu_b_source = bus_sources::AluBSourceCtl_Bit3;
                OpSmb4: alu_b_source = bus_sources::AluBSourceCtl_Bit4;
                OpSmb5: alu_b_source = bus_sources::AluBSourceCtl_Bit5;
                OpSmb6: alu_b_source = bus_sources::AluBSourceCtl_Bit6;
                OpSmb7: alu_b_source = bus_sources::AluBSourceCtl_Bit7;
                default: set_invalid_state();
            endcase
            ctrl_signals[control_signals::AluBInverse] = 0;
        end
        CycleOp2: begin
            addr_bus_dl();
            ML = 0;
            rW = 0;

            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
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
            data_bus_source = bus_sources::DataBusSrc_Mem;
            ctrl_signals[control_signals::UseAluFlags] = 0;
            ctrl_signals[control_signals::UpdateFlagC] = 1;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::UpdateFlagI] = 1;
            ctrl_signals[control_signals::UpdateFlagD] = 1;
            ctrl_signals[control_signals::UpdateFlagV] = 1;
            ctrl_signals[control_signals::UpdateFlagN] = 1;

            // Read PC MSB
            addr_bus_sp();

            sp_inc();
        end
        CycleOp3: begin
            // Store PC MSB
            pc_low_source = bus_sources::PcLowSource_Mem;
            ctrl_signals[control_signals::PC_LOAD] = 1;

            // Read PC LSB
            addr_bus_sp();
        end
        CycleOp4: begin
            pc_low_source = bus_sources::PcLowSource_CurrentValue;
            pc_high_source = bus_sources::PcHighSource_Mem;
            ctrl_signals[control_signals::PC_LOAD] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
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
            pc_low_source = bus_sources::PcLowSource_Mem;
            ctrl_signals[control_signals::PC_LOAD] = 1;

            // Read PC LSB
            addr_bus_sp();
        end
        CycleOp3: begin
            pc_low_source = bus_sources::PcLowSource_CurrentValue;
            pc_high_source = bus_sources::PcHighSource_Mem;
            ctrl_signals[control_signals::PC_LOAD] = 1;
            ctrl_signals[control_signals::PC_ADVANCE] = 1;

            addr_bus_pc();

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_sec_first();
    next_instruction();

    data_bus_source = bus_sources::DataBusSrc_Ones;
    ctrl_signals[control_signals::UpdateFlagC] = 1;
    ctrl_signals[control_signals::UseAluFlags] = 0;
endtask

task do_op_sed_first();
    next_instruction();

    data_bus_source = bus_sources::DataBusSrc_Ones;
    ctrl_signals[control_signals::UpdateFlagD] = 1;
endtask

task do_op_sei_first();
    next_instruction();

    data_bus_source = bus_sources::DataBusSrc_Ones;
    ctrl_signals[control_signals::UpdateFlagI] = 1;
endtask

task do_op_sta_first();
    // BUG: WDC manual only mentions extra cycle for abs,x operand
    if( op_cycle!=CycleAddr3 && ( active_addr_mode==AddrAbsoluteX || active_addr_mode==AddrAbsoluteY ) ) begin
        addr_bus_dl();
    end else if( op_cycle!=CycleAddr4 && active_addr_mode==AddrZeroPageIndY ) begin
        addr_bus_dl();
        incompatible = 1;
    end else begin
        rW = 0;
        data_bus_source = bus_sources::DataBusSrc_A;

        next_instruction();
    end
endtask

task do_op_sta();
    case( op_cycle )
        FirstOpCycle: begin
            addr_bus_dl();

            rW = 0;
            data_bus_source = bus_sources::DataBusSrc_A;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_stx_first();
    rW = 0;
    data_bus_source = bus_sources::DataBusSrc_X;

    next_instruction();
endtask

task do_op_sty_first();
    rW = 0;
    data_bus_source = bus_sources::DataBusSrc_Y;

    next_instruction();
endtask

task do_op_stz_first();
    // BUG: WDC datasheet never mentions extra cycles for STZ, only for STA
    if( op_cycle!=CycleAddr3 && ( active_addr_mode==AddrAbsoluteX || active_addr_mode==AddrAbsoluteY ) ) begin
        addr_bus_dl();
    end else begin
        rW = 0;
        data_bus_source = bus_sources::DataBusSrc_Zero;

        next_instruction();
    end
endtask

task do_op_stz();
    case( op_cycle )
        FirstOpCycle: begin
            addr_bus_dl();

            rW = 0;
            data_bus_source = bus_sources::DataBusSrc_Zero;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask


task do_op_transfer_first(input bus_sources::DataBusSourceCtl src, input control_signals::ctrl_signals dest);
    next_instruction();

    data_bus_source = src;
    ctrl_signals[dest] = 1;

    ctrl_signals[control_signals::UpdateFlagN] = 1;
    ctrl_signals[control_signals::UpdateFlagZ] = 1;
    ctrl_signals[control_signals::CalculateFlagZ] = 1;
endtask

task do_op_txs_first();
    next_instruction();

    data_bus_source = bus_sources::DataBusSrc_X;
    stack_pointer_source = bus_sources::StackPointerSource_DataBus;
    ctrl_signals[control_signals::LOAD_SP] = 1;
endtask

task do_op_trb_first();
    ML = 0;
endtask

task do_op_trb();
    case( op_cycle )
        FirstOpCycle: begin
            addr_bus_dl();
            ML = 0;

            alu_op = control_signals::AluOp_and;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_b_source = bus_sources::AluBSourceCtl_A;
            ctrl_signals[control_signals::AluBInverse] = 1;
        end
        CycleOp2: begin
            addr_bus_dl();
            ML = 0;
            rW = 0;

            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;

            alu_op = control_signals::AluOp_and;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_b_source = bus_sources::AluBSourceCtl_A;
            ctrl_signals[control_signals::AluBInverse] = 0;
        end
        CycleOp3: begin
            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_tsb_first();
    ML = 0;
endtask

task do_op_tsb();
    case( op_cycle )
        FirstOpCycle: begin
            addr_bus_dl();
            ML = 0;

            alu_op = control_signals::AluOp_or;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_b_source = bus_sources::AluBSourceCtl_A;
            ctrl_signals[control_signals::AluBInverse] = 0;
        end
        CycleOp2: begin
            addr_bus_dl();
            ML = 0;
            rW = 0;

            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;

            alu_op = control_signals::AluOp_and;
            alu_a_source = bus_sources::AluASourceCtl_Mem;
            alu_b_source = bus_sources::AluBSourceCtl_A;
            ctrl_signals[control_signals::AluBInverse] = 0;
        end
        CycleOp3: begin
            data_bus_source = bus_sources::DataBusSrc_Alu_Latched;
            ctrl_signals[control_signals::UpdateFlagZ] = 1;
            ctrl_signals[control_signals::CalculateFlagZ] = 1;

            do_fetch_cycle();
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_stop_first();
endtask

task do_op_stop();
    case( op_cycle )
        FirstOpCycle: begin
            addr_bus_pc();

            op_cycle_next = FirstOpCycle;
        end
        default: set_invalid_state();
    endcase
endtask

task do_op_wai_first();
endtask

task do_op_wai();
    if( op_cycle==CycleOp2 )
        op_cycle_next = FirstOpCycle;

    addr_bus_pc();

    if( !IRQ || nmi_pending )
        next_instruction();
endtask

endmodule
