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

module decoder#(parameter CPU_VARIANT = 2)
(
    input clock,
    input reset,
    input ready,
    input [7:0]status,
    input alu_carry_out,
    input interrupt_request,
    input nonmaskable_interrupt,

    input [7:0]memory_in,

    output bus_sources::AddressBusLowSourceCtl addr_bus_low_src,
    output bus_sources::AddressBusHighSourceCtl addr_bus_high_src,
    output bus_sources::DataBusSourceCtl data_bus_src,
    output bus_sources::SpecialBusSourceCtl special_bus_src,
    output bus_sources::PcLowSourceCtl pcl_bus_src,
    output bus_sources::PcHighSourceCtl pch_bus_src,
    output bus_sources::PcNextSourceCtl pc_next_src,
    output bus_sources::AluASrcCtl alu_a_src,
    output bus_sources::AluBSrcCtl alu_b_src,
    output bus_sources::DataOutSourceCtl data_out_src,

    output control_signals::alu_control alu_op,
    output logic alu_carry_in,

    output logic[control_signals::ctrl_signals_last:0] ctrl_signals,

    output logic sync,
    output logic write,
    output logic memory_lock,
    output logic vector_pull,

    output logic incompatible
);

localparam MAX_OPCODE_CYCLES = 16;

localparam
    CycleInvalid  = 16'bxxxxxxxx_xxxxxxxx,
    CycleAnyAddr  = 16'b00000000_xxxxxxxx,
    CycleDecode   = 16'b00000000_00000001,
    CycleAddr1    = 16'b00000000_00000010,
    CycleAddr2    = 16'b00000000_00000100,
    CycleAddr3    = 16'b00000000_00001000,
    CycleAddr4    = 16'b00000000_00010000,
    CycleAddr5    = 16'b00000000_00100000,
    CycleAddr6    = 16'b00000000_01000000,
    LastAddrCycle = 16'b00000000_10000000,
    CycleAddrMask = 16'b00000000_11111111,

    FirstOpCycle  = 16'b00000001_00000000,
    CycleOp2      = 16'b00000010_00000000,
    CycleOp3      = 16'b00000100_00000000,
    CycleOp4      = 16'b00001000_00000000,
    CycleOp5      = 16'b00010000_00000000,
    CycleOp6      = 16'b00100000_00000000,
    CycleOp7      = 16'b01000000_00000000,
    CycleOp8      = 16'b10000000_00000000;
logic [MAX_OPCODE_CYCLES-1:0] op_cycle = FirstOpCycle, op_cycle_next;
logic [7:0] current_opcode = 8'hdb, next_opcode;
logic jump_negative;
logic bbrs_jump, bbrs_jump_next;

enum { IntStateNone=0, IntStateIrq, IntStateNmi, IntStateReset }
    pending_interrupt = IntStateNone, pending_interrupt_next, current_int, current_int_next;
logic prev_nmi = 1'b0;

always_ff@(posedge clock) begin
    if( ready ) begin
        op_cycle <= op_cycle_next;
        current_opcode <= next_opcode;
        jump_negative <= memory_in[7];
        bbrs_jump <= bbrs_jump_next;
        current_int <= current_int_next;
        pending_interrupt <= pending_interrupt_next;
    end

    // Handle the stateful interrupt lines. The only state change in the CPU unhindered by ready.
    if( reset ) begin
        current_int <= IntStateReset;
        pending_interrupt <= IntStateNone;
        op_cycle <= CycleDecode;
        current_opcode <= 8'h00; // So we don't perform post_op for the previous instruction
        prev_nmi <= 1'b0;
    end else if( nonmaskable_interrupt && !prev_nmi ) begin
        pending_interrupt <= IntStateNmi;
    end

    prev_nmi <= nonmaskable_interrupt;
end

task set_invalid_state();
begin
    op_cycle_next = CycleInvalid;
    next_opcode = 8'hXX;
    incompatible = 1'bX;
    alu_carry_in = 1'bX;
    bbrs_jump_next = 1'bX;

    addr_bus_low_src = bus_sources::AddrBusLowSrc_Invalid;
    addr_bus_high_src = bus_sources::AddrBusHighSrc_Invalid;
    data_bus_src = bus_sources::DataBusSrc_Invalid;
    special_bus_src = bus_sources::SpecialBusSrc_Invalid;
    alu_a_src = bus_sources::AluASrc_Invalid;
    alu_b_src = bus_sources::AluBSrc_Invalid;
    data_out_src = bus_sources::DataOutSrc_Invalid;
    pcl_bus_src = bus_sources::PcLowSrc_Invalid;
    pch_bus_src = bus_sources::PcHighSrc_Invalid;
    pc_next_src = bus_sources::PcNextSrc_Invalid;

    alu_op = control_signals::AluOp_Invalid;

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
    pending_interrupt_next = pending_interrupt;
    current_int_next = current_int;
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

    if( op_cycle==CycleDecode ) begin
        do_post();

        addr_bus_pc();
        if( current_int!=IntStateNone ) begin
            // Interrupt!!!!1!ii
            next_opcode = 8'h00;
            do_address(8'h00);
        end else begin
            next_opcode = memory_in;

            do_address(memory_in);
        end
    end else if( (op_cycle&CycleAddrMask)!=0 )
        do_address(current_opcode);
    else
        do_opcode(current_opcode);
end

task addr_bus_pc();
    addr_bus_low_src = bus_sources::AddrBusLowSrc_PC;
    addr_bus_high_src = bus_sources::AddrBusHighSrc_PC;
endtask

task addr_bus_stack();
    addr_bus_low_src = bus_sources::AddrBusLowSrc_SP;
    addr_bus_high_src = bus_sources::AddrBusHighSrc_One;
endtask

task addr_bus_ol();
    addr_bus_low_src = bus_sources::AddrBusLowSrc_OL;
    addr_bus_high_src = bus_sources::AddrBusHighSrc_OL;
endtask

task advance_pc();
    ctrl_signals[control_signals::LOAD_PCL] = 1'b1;
    ctrl_signals[control_signals::LOAD_PCH] = 1'b1;
    pcl_bus_src = bus_sources::PcLowSrc_Incrementor;
    pch_bus_src = bus_sources::PcHighSrc_Incrementor;
    pc_next_src = bus_sources::PcNextSrc_Pc;
endtask

task stack_pointer_push();
    alu_a_src = bus_sources::AluASrc_RegSp;
    alu_b_src = bus_sources::AluBSrc_Zero;
    alu_op = control_signals::AluOp_add;
    ctrl_signals[control_signals::AluInverseB] = 1'b1;
    alu_carry_in = 1'b0;

    ctrl_signals[control_signals::LOAD_SP] = 1'b1;
endtask

task stack_pointer_pop();
    alu_a_src = bus_sources::AluASrc_RegSp;
    alu_b_src = bus_sources::AluBSrc_Zero;
    alu_op = control_signals::AluOp_add;
    ctrl_signals[control_signals::AluInverseB] = 1'b0;
    alu_carry_in = 1'b1;

    ctrl_signals[control_signals::LOAD_SP] = 1'b1;
endtask

task do_address(input [7:0] opcode);
    case(opcode)
        8'h00: op_brk();                        // BRK
        8'h01: addr_mode_zp_x_ind();            // ORA (zp,x)
        8'h04: addr_mode_zp();                  // TSB zp
        8'h05: addr_mode_zp();                  // ORA zp
        8'h06: addr_mode_zp();                  // ASL zp
        8'h07: addr_mode_zp();                  // RMB
        8'h08: addr_mode_stack(opcode);         // PHP
        8'h09: addr_mode_immediate();           // ORA #
        8'h0a: addr_mode_acc();                 // ASL A
        8'h0c: addr_mode_absolute();            // TRB abs
        8'h0d: addr_mode_absolute();            // ORA abs
        8'h0e: addr_mode_absolute();            // ASL abs
        8'h0f: addr_mode_zp();                  // BBR0 zp
        8'h10: addr_mode_pc_rel();              // BPL
        8'h11: addr_mode_zp_ind_y();            // ORA (zp),y
        8'h12: addr_mode_zp_ind();              // ORA (zp)
        8'h14: addr_mode_zp();                  // TSB zp
        8'h15: addr_mode_zp_x();                // ORA zp,x
        8'h16: addr_mode_zp_x();                // ASL zp,x
        8'h17: addr_mode_zp();                  // RMB
        8'h18: addr_mode_implied();             // CLC
        8'h19: addr_mode_abs_y();               // ORA abs,y
        8'h1a: addr_mode_acc();                 // INC
        8'h1c: addr_mode_absolute();            // TRB abs
        8'h1d: addr_mode_abs_x();               // ORA abs,x
        8'h1e: addr_mode_abs_x();               // ASL abs,x
        8'h1f: addr_mode_zp();                  // BBR1 zp
        8'h20: addr_mode_stack(opcode);         // JSR
        8'h21: addr_mode_zp_x_ind();            // AND (zp,x)
        8'h24: addr_mode_zp();                  // BIT zp
        8'h25: addr_mode_zp();                  // AND zp
        8'h26: addr_mode_zp();                  // ROL zp
        8'h27: addr_mode_zp();                  // RMB
        8'h28: addr_mode_stack(opcode);         // PLP
        8'h29: addr_mode_immediate();           // AND #
        8'h2a: addr_mode_acc();                 // ROL
        8'h2c: addr_mode_absolute();            // BIT abs
        8'h2d: addr_mode_absolute();            // AND abs
        8'h2e: addr_mode_absolute();            // ROL abs
        8'h2f: addr_mode_zp();                  // BBR2 zp
        8'h30: addr_mode_pc_rel();              // BMI
        8'h31: addr_mode_zp_ind_y();            // AND (zp),y
        8'h32: addr_mode_zp_ind();              // AND (zp)
        8'h34: addr_mode_zp_x();                // BIT zp,x
        8'h35: addr_mode_zp_x();                // AND zp,x
        8'h36: addr_mode_zp_x();                // ROL zp,x
        8'h37: addr_mode_zp();                  // RMB
        8'h38: addr_mode_implied();             // SEC
        8'h39: addr_mode_abs_y();               // AND abs,y
        8'h3a: addr_mode_acc();                 // DEC
        8'h3c: addr_mode_abs_x();               // BIT abs,x
        8'h3d: addr_mode_abs_x();               // AND abs,x
        8'h3e: addr_mode_abs_x();               // ROL abs,x
        8'h3f: addr_mode_zp();                  // BBR3 zp
        8'h40: addr_mode_stack(opcode);         // RTI
        8'h41: addr_mode_zp_x_ind();            // EOR (zp,x)
        8'h45: addr_mode_zp();                  // EOR zp
        8'h46: addr_mode_zp();                  // LSR zp
        8'h47: addr_mode_zp();                  // RMB
        8'h48: addr_mode_stack(opcode);         // PHA
        8'h49: addr_mode_immediate();           // EOR #
        8'h4a: addr_mode_acc();                 // LSR A
        8'h4c: addr_mode_absolute();            // JMP abs
        8'h4d: addr_mode_absolute();            // EOR abs
        8'h4e: addr_mode_absolute();            // LSR abs
        8'h4f: addr_mode_zp();                  // BBR4 zp
        8'h50: addr_mode_pc_rel();              // BVC
        8'h51: addr_mode_zp_ind_y();            // EOR (zp),y
        8'h52: addr_mode_zp_ind();              // EOR (zp)
        8'h55: addr_mode_zp_x();                // EOR zp,x
        8'h56: addr_mode_zp_x();                // LSR zp,x
        8'h57: addr_mode_zp();                  // RMB
        8'h58: addr_mode_implied();             // CLI
        8'h59: addr_mode_abs_y();               // EOR abs,y
        8'h5a: addr_mode_stack(opcode);         // PHY
        8'h5d: addr_mode_abs_x();               // EOR abs,x
        8'h5e: addr_mode_abs_x();               // LSR abs,x
        8'h5f: addr_mode_zp();                  // BBR5 zp
        8'h60: addr_mode_stack(opcode);         // RTS
        8'h61: addr_mode_zp_x_ind();            // ADC (zp,x)
        8'h64: addr_mode_zp();                  // STZ zp
        8'h65: addr_mode_zp();                  // ADC zp
        8'h66: addr_mode_zp();                  // ROR zp
        8'h67: addr_mode_zp();                  // RMB
        8'h68: addr_mode_stack(opcode);         // PLA
        8'h69: addr_mode_immediate();           // ADC #
        8'h6a: addr_mode_acc();                 // ROR
        8'h6c: addr_mode_abs_ind();             // JMP (abs)
        8'h6d: addr_mode_absolute();            // ADC abs
        8'h6e: addr_mode_absolute();            // ROR abs
        8'h6f: addr_mode_zp();                  // BBR6 zp
        8'h70: addr_mode_pc_rel();              // BVS
        8'h71: addr_mode_zp_ind_y();            // ADC (zp),y
        8'h72: addr_mode_zp_ind();              // ADC (zp)
        8'h74: addr_mode_zp_x();                // STZ zp,x
        8'h75: addr_mode_zp_x();                // ADC zp,x
        8'h76: addr_mode_zp_x();                // ROR zp,x
        8'h77: addr_mode_zp();                  // RMB
        8'h78: addr_mode_implied();             // SEI
        8'h79: addr_mode_abs_y();               // ADC abs,y
        8'h7a: addr_mode_stack(opcode);         // PLY
        8'h7c: addr_mode_abs_x_ind();           // JMP (abs,x)
        8'h7d: addr_mode_abs_x();               // ADC abs,x
        8'h7e: addr_mode_abs_x();               // ROR abs,x
        8'h7f: addr_mode_zp();                  // BBR7 zp
        8'h80: addr_mode_pc_rel();              // BRA
        8'h81: addr_mode_zp_x_ind();            // STA (zp,x)
        8'h84: addr_mode_zp();                  // STY zp
        8'h85: addr_mode_zp();                  // STA zp
        8'h86: addr_mode_zp();                  // STX zp
        8'h87: addr_mode_zp();                  // SMB
        8'h88: addr_mode_implied();             // DEY
        8'h89: addr_mode_immediate();           // BIT #
        8'h8a: addr_mode_implied();             // TXA
        8'h8c: addr_mode_absolute();            // STY abs
        8'h8d: addr_mode_absolute();            // STA abs
        8'h8e: addr_mode_absolute();            // STX abs
        8'h8f: addr_mode_zp();                  // BBS0 zp
        8'h90: addr_mode_pc_rel();              // BCC
        8'h91: addr_mode_zp_ind_y();            // STA (zp),y
        8'h92: addr_mode_zp_ind();              // STA (zp)
        8'h94: addr_mode_zp_x();                // STY zp,x
        8'h95: addr_mode_zp_x();                // STA zp,x
        8'h96: addr_mode_zp_y();                // STX zp,y
        8'h97: addr_mode_zp();                  // SMB
        8'h98: addr_mode_implied();             // TYA
        8'h99: addr_mode_abs_y();               // STA abs,y
        8'h9a: addr_mode_implied();             // TXS
        8'h9c: addr_mode_absolute();            // STZ abs
        8'h9d: addr_mode_abs_x();               // STA abs,x
        8'h9e: addr_mode_abs_x();               // STZ abs,x
        8'h9f: addr_mode_zp();                  // BBS1 zp
        8'ha0: addr_mode_immediate();           // LDY #
        8'ha1: addr_mode_zp_x_ind();            // LDA (zp,x)
        8'ha2: addr_mode_immediate();           // LDX #
        8'ha4: addr_mode_zp();                  // LDY zp
        8'ha5: addr_mode_zp();                  // LDA zp
        8'ha6: addr_mode_zp();                  // LDX zp
        8'ha7: addr_mode_zp();                  // SMB
        8'ha8: addr_mode_implied();             // TAY
        8'ha9: addr_mode_immediate();           // LDA #
        8'haa: addr_mode_implied();             // TAX
        8'hac: addr_mode_absolute();            // LDY abs
        8'had: addr_mode_absolute();            // LDA abs
        8'hae: addr_mode_absolute();            // LDX abs
        8'haf: addr_mode_zp();                  // BBS2 zp
        8'hb0: addr_mode_pc_rel();              // BCS
        8'hb1: addr_mode_zp_ind_y();            // LDA (zp),y
        8'hb2: addr_mode_zp_ind();              // LDA (zp)
        8'hb4: addr_mode_zp_x();                // LDY zp,x
        8'hb5: addr_mode_zp_x();                // LDA zp,x
        8'hb6: addr_mode_zp_y();                // LDX zp,y
        8'hb7: addr_mode_zp();                  // SMB
        8'hb8: addr_mode_implied();             // CLV
        8'hb9: addr_mode_abs_y();               // LDA abs,y
        8'hba: addr_mode_implied();             // TSX
        8'hbc: addr_mode_abs_x();               // LDY abs,x
        8'hbd: addr_mode_abs_x();               // LDA abs,x
        8'hbe: addr_mode_abs_y();               // LDX abs,y
        8'hbf: addr_mode_zp();                  // BBS3 zp
        8'hc0: addr_mode_immediate();           // CPY #
        8'hc1: addr_mode_zp_x_ind();            // CMP (zp,x)
        8'hc4: addr_mode_zp();                  // CPY zp
        8'hc5: addr_mode_zp();                  // CMP zp
        8'hc6: addr_mode_zp();                  // DEC zp
        8'hc7: addr_mode_zp();                  // SMB
        8'hc8: addr_mode_implied();             // INY
        8'hc9: addr_mode_immediate();           // CMP #
        8'hca: addr_mode_implied();             // DEX
        8'hcb: addr_mode_implied();             // WAI
        8'hcc: addr_mode_absolute();            // CPY abs
        8'hcd: addr_mode_absolute();            // CMP abs
        8'hce: addr_mode_absolute();            // DEC abs
        8'hcf: addr_mode_zp();                  // BBS4 zp
        8'hd0: addr_mode_pc_rel();              // BNE
        8'hd1: addr_mode_zp_ind_y();            // CMP (zp),y
        8'hd2: addr_mode_zp_ind();              // CMP (zp)
        8'hd5: addr_mode_zp_x();                // CMP zp,x
        8'hd6: addr_mode_zp_x();                // DEC zp,x
        8'hd7: addr_mode_zp();                  // SMB
        8'hd8: addr_mode_implied();             // CLD
        8'hd9: addr_mode_abs_y();               // CMP abs,y
        8'hda: addr_mode_stack(opcode);         // PHX
        8'hdb: op_stp();
        8'hdd: addr_mode_abs_x();               // CMP abs,x
        8'hde: addr_mode_abs_x();               // DEC abs,x
        8'hdf: addr_mode_zp();                  // BBS5 zp
        8'he0: addr_mode_immediate();           // CPX #
        8'he1: addr_mode_zp_x_ind();            // SBC (zp,x)
        8'he4: addr_mode_zp();                  // CPX zp
        8'he5: addr_mode_zp();                  // SBC zp
        8'he6: addr_mode_zp();                  // INC zp
        8'he7: addr_mode_zp();                  // SMB
        8'he8: addr_mode_implied();             // INX
        8'he9: addr_mode_immediate();           // SBC #
        8'hea: addr_mode_implied();             // NOP
        8'hec: addr_mode_absolute();            // CPX abs
        8'hed: addr_mode_absolute();            // SBC abs
        8'hee: addr_mode_absolute();            // INC abs
        8'hef: addr_mode_zp();                  // BBS6 zp
        8'hf0: addr_mode_pc_rel();              // BEQ
        8'hf1: addr_mode_zp_ind_y();            // SBC (zp),y
        8'hf2: addr_mode_zp_ind();              // SBC (zp)
        8'hf5: addr_mode_zp_x();                // SBC zp,x
        8'hf6: addr_mode_zp_x();                // INC zp,x
        8'hf7: addr_mode_zp();                  // SMB
        8'hf8: addr_mode_implied();             // SED
        8'hf9: addr_mode_abs_y();               // SBC abs,y
        8'hfa: addr_mode_stack(opcode);         // PLX
        8'hfd: addr_mode_abs_x();               // SBC abs,x
        8'hfe: addr_mode_abs_x();               // INC abs,x
        8'hff: addr_mode_zp();                  // BBS7 zp
        default: set_invalid_state();
    endcase
endtask

task do_opcode(input [7:0]opcode);
    case(opcode)
        8'h00: op_brk();
        8'h01: op_ora();                        // ORA (zp,x)
        8'h04: op_trsb();                       // TSB zp
        8'h05: op_ora();                        // ORA zp
        8'h06: op_asl();                        // ASL zp
        8'h07: op_rsmb();
        8'h08: op_php();
        8'h09: op_ora();                        // ORA #
        8'h0a: op_asl_A();
        8'h0c: op_trsb();                       // TSB abs
        8'h0d: op_ora();                        // ORA abs
        8'h0e: op_asl();                        // ASL abs
        8'h0f: op_bbrs();                       // BBR0 zp
        8'h10: op_bpl();
        8'h11: op_ora();                        // ORA (zp),y
        8'h12: op_ora();                        // ORA (zp)
        8'h14: op_trsb();                       // TRB zp
        8'h15: op_ora();                        // ORA zp,x
        8'h16: op_asl();                        // ASL zp,x
        8'h17: op_rsmb();
        8'h18: op_clc();
        8'h19: op_ora();                        // ORA abs,y
        8'h1a: op_inc_A();                      // INC
        8'h1c: op_trsb();                       // TRB abs
        8'h1d: op_ora();                        // ORA abs,x
        8'h1e: op_asl();                        // ASL abs,x
        8'h1f: op_bbrs();                       // BBR1 zp
        8'h20: op_jsr();                        // JSR abs
        8'h21: op_and();                        // AND (zp),x
        8'h24: op_bit();                        // BIT zp
        8'h25: op_and();                        // AND zp
        8'h26: op_rol();                        // ROL zp
        8'h27: op_rsmb();
        8'h28: op_plp();
        8'h29: op_and();                        // AND #
        8'h2a: op_rol_A();                      // ROL
        8'h2c: op_bit();                        // BIT abs
        8'h2d: op_and();                        // AND abs
        8'h2e: op_rol();                        // ROL abs
        8'h2f: op_bbrs();                       // BBR2 zp
        8'h30: op_bmi();
        8'h31: op_and();                        // AND (zp),y
        8'h32: op_and();                        // AND (zp)
        8'h34: op_bit();                        // BIT zp,x
        8'h35: op_and();                        // AND zp,x
        8'h36: op_rol();                        // ROL zp,x
        8'h37: op_rsmb();
        8'h38: op_sec();
        8'h39: op_and();                        // AND abs,y
        8'h3a: op_dec_A();                      // DEC
        8'h3c: op_bit();                        // BIT abs,x
        8'h3d: op_and();                        // AND abs,x
        8'h3e: op_rol();                        // ROL abs,x
        8'h3f: op_bbrs();                       // BBR3 zp
        8'h40: op_rti();
        8'h41: op_eor();                        // EOR (zp,x)
        8'h45: op_eor();                        // EOR zp
        8'h46: op_lsr();                        // LSR zp
        8'h47: op_rsmb();
        8'h48: op_pha();
        8'h49: op_eor();                        // EOR #
        8'h4a: op_lsr_A();                      // LSR
        8'h4c: op_jmp();                        // JMP abs
        8'h4d: op_eor();                        // EOR abs
        8'h4e: op_lsr();                        // LSR abs
        8'h4f: op_bbrs();                       // BBR4 zp
        8'h50: op_bvc();
        8'h51: op_eor();                        // EOR (zp),y
        8'h52: op_eor();                        // EOR (zp)
        8'h55: op_eor();                        // EOR zp,x
        8'h56: op_lsr();                        // LSR zp,x
        8'h57: op_rsmb();
        8'h58: op_cli();
        8'h59: op_eor();                        // EOR abs,y
        8'h5a: op_phy();
        8'h5d: op_eor();                        // EOR abs,x
        8'h5e: op_lsr();                        // LSR abs,x
        8'h5f: op_bbrs();                       // BBR5 zp
        8'h60: op_rts();
        8'h61: op_adc();                        // ADC (zp,x)
        8'h64: op_stz();                        // STZ zp
        8'h65: op_adc();                        // ADC zp
        8'h66: op_ror();                        // ROR zp
        8'h67: op_rsmb();
        8'h68: op_pla();
        8'h69: op_adc();                        // ADC #
        8'h6a: op_ror_A();                      // ROR
        8'h6c: op_jmp();                        // JMP (abs)
        8'h6d: op_adc();                        // ADC abs
        8'h6e: op_ror();                        // ROR abs
        8'h6f: op_bbrs();                       // BBR6 zp
        8'h70: op_bvs();
        8'h71: op_adc();                        // ADC (zp),y
        8'h72: op_adc();                        // ADC (zp)
        8'h74: op_stz();                        // STZ zp,x
        8'h75: op_adc();                        // ADC zp,x
        8'h76: op_ror();                        // ROR zp,x
        8'h77: op_rsmb();
        8'h78: op_sei();
        8'h79: op_adc();                        // ADC abs,y
        8'h7a: op_ply();
        8'h7c: op_jmp();                        // JMP (abs,x)
        8'h7d: op_adc();                        // ADC abs,x
        8'h7e: op_ror();                        // ROR abs,x
        8'h7f: op_bbrs();                       // BBR7 zp
        8'h80: op_bra();
        8'h81: op_sta();                        // STA (zp,x)
        8'h84: op_sty();                        // STY zp
        8'h85: op_sta();                        // STA zp
        8'h86: op_stx();                        // STX zp
        8'h87: op_rsmb();
        8'h88: op_dey();
        8'h89: op_bit();                        // BIT #
        8'h8a: op_txa();
        8'h8c: op_sty();                        // STY abs
        8'h8d: op_sta();                        // STA abs
        8'h8e: op_stx();                        // STX abs
        8'h8f: op_bbrs();                       // BBS0 zp
        8'h90: op_bcc();
        8'h91: op_sta();                        // STA (zp),y
        8'h92: op_sta();                        // STA (zp)
        8'h94: op_sty();                        // STY zp,x
        8'h95: op_sta();                        // STA zp,x
        8'h96: op_stx();                        // STX zp,y
        8'h97: op_rsmb();
        8'h98: op_tya();
        8'h99: op_sta();                        // STA abs,y
        8'h9a: op_txs();
        8'h9c: op_stz();                        // STZ abs
        8'h9d: op_sta();                        // STA abs,x
        8'h9e: op_stz();                        // STZ abs,x
        8'h9f: op_bbrs();                       // BBS1 zp
        8'ha0: op_ldy();                        // LDY #
        8'ha1: op_lda();                        // LDA (zp,x)
        8'ha2: op_ldx();                        // LDX #
        8'ha4: op_ldy();                        // LDY zp
        8'ha5: op_lda();                        // LDA zp
        8'ha6: op_ldx();                        // LDX zp
        8'ha7: op_rsmb();
        8'ha8: op_tay();
        8'ha9: op_lda();                        // LDA #
        8'haa: op_tax();
        8'hac: op_ldy();                        // LDY abs
        8'had: op_lda();                        // LDA abs
        8'hae: op_ldx();                        // LDX abs
        8'haf: op_bbrs();                       // BBS2 zp
        8'hb0: op_bcs();
        8'hb1: op_lda();                        // LDA (zp),y
        8'hb2: op_lda();                        // LDA (zp)
        8'hb4: op_ldy();                        // LDY zp,x
        8'hb5: op_lda();                        // LDA zp,x
        8'hb6: op_ldx();                        // LDX zp,y
        8'hb7: op_rsmb();
        8'hb8: op_clv();
        8'hb9: op_lda();                        // LDA abs,y
        8'hba: op_tsx();
        8'hbc: op_ldy();                        // LDY abs,x
        8'hbd: op_lda();                        // LDA abs,x
        8'hbe: op_ldx();                        // LDX abs,y
        8'hbf: op_bbrs();                       // BBS3 zp
        8'hc0: op_cpy();                        // CPY #
        8'hc1: op_cmp();                        // CMP (zp,x)
        8'hc4: op_cpy();                        // CPY zp
        8'hc5: op_cmp();                        // CMP zp
        8'hc6: op_dec();                        // DEC zp
        8'hc7: op_rsmb();
        8'hc8: op_iny();
        8'hc9: op_cmp();                        // CMP #
        8'hca: op_dex();
        8'hcb: op_wai();                        // WAI
        8'hcc: op_cpy();                        // CPY abs
        8'hcd: op_cmp();                        // CMP abs
        8'hce: op_dec();                        // DEC abs
        8'hcf: op_bbrs();                       // BBS4 zp
        8'hd0: op_bne();
        8'hd1: op_cmp();                        // CMP (zp),y
        8'hd2: op_cmp();                        // CMP (zp)
        8'hd5: op_cmp();                        // CMP zp,x
        8'hd6: op_dec();                        // DEC zp,x
        8'hd7: op_rsmb();
        8'hd8: op_cld();
        8'hd9: op_cmp();                        // CMP abs,y
        8'hda: op_phx();
        8'hdb: op_stp();
        8'hdd: op_cmp();                        // CMP abs,x
        8'hde: op_dec();                        // DEC abs,x
        8'hdf: op_bbrs();                       // BBS5 zp
        8'he0: op_cpx();                        // CPX #
        8'he1: op_sbc();                        // SBC (zp,x)
        8'he4: op_cpx();                        // CPX zp
        8'he5: op_sbc();                        // SBC zp
        8'he6: op_inc();                        // INC zp
        8'he7: op_rsmb();
        8'he8: op_inx();
        8'he9: op_sbc();                        // SBC #
        8'hea: op_nop();
        8'hec: op_cpx();                        // CPX abs
        8'hed: op_sbc();                        // SBC abs
        8'hee: op_inc();                        // INC abs
        8'hef: op_bbrs();                       // BBS6 zp
        8'hf0: op_beq();
        8'hf1: op_sbc();                        // SBC (zp),y
        8'hf2: op_sbc();                        // SBC (zp)
        8'hf5: op_sbc();                        // SBC zp,x
        8'hf6: op_inc();                        // INC zp,x
        8'hf7: op_rsmb();
        8'hf8: op_sed();
        8'hf9: op_sbc();                        // SBC abs,y
        8'hfa: op_plx();
        8'hfd: op_sbc();                        // SBC abs,x
        8'hfe: op_inc();                        // INC abs,x
        8'hff: op_bbrs();                       // BBS7 zp
        default: set_invalid_state();
    endcase
endtask

task do_post();
    case(current_opcode)
        8'h24: post_bit();                        // BIT zp
        8'h2c: post_bit();                        // BIT abs
        8'h34: post_bit();                        // BIT zp,x
        8'h3c: post_bit();                        // BIT abs,x
        8'h89: post_bit();                        // BIT #
    endcase
endtask

task addr_mode_absolute();
    case(op_cycle)
        CycleDecode: begin
            advance_pc();
        end
        CycleAddr1: begin
            ctrl_signals[control_signals::LOAD_DL] = 1'b1;

            addr_bus_pc();
            advance_pc();
        end
        CycleAddr2: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_DL;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;
            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            op_cycle_next = FirstOpCycle;
            do_opcode(current_opcode);
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_abs_ind();
    if( CPU_VARIANT==0 )
        addr_mode_abs_ind_6502();
    else
        addr_mode_abs_ind_65c02();
endtask

task addr_mode_abs_ind_6502();
    case(op_cycle)
        CycleDecode: begin
            advance_pc();
        end
        CycleAddr1: begin
            addr_bus_pc();
            ctrl_signals[control_signals::LOAD_DL] = 1'b1;

            alu_a_src = bus_sources::AluASrc_Mem;
            alu_b_src = bus_sources::AluBSrc_Zero;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b1;
        end
        CycleAddr2: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_DL;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;

            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            alu_a_src = bus_sources::AluASrc_ALU;
            alu_b_src = bus_sources::AluBSrc_Zero;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b0;
        end
        CycleAddr3: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_OL;

            ctrl_signals[control_signals::LOAD_DL] = 1'b1;
        end
        CycleAddr4: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_DL;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;

            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            op_cycle_next = FirstOpCycle;
            do_opcode(current_opcode);
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_abs_ind_65c02();
    case(op_cycle)
        CycleDecode: begin
            advance_pc();
        end
        CycleAddr1: begin
            addr_bus_pc();

            ctrl_signals[control_signals::LOAD_DL] = 1'b1;
        end
        CycleAddr2: begin
            addr_bus_pc();

            // Dummy cycle?
        end
        CycleAddr3: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_DL;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;

            ctrl_signals[control_signals::LOAD_PCL] = 1'b1;
            ctrl_signals[control_signals::LOAD_PCH] = 1'b1;
            pcl_bus_src = bus_sources::PcLowSrc_Incrementor;
            pch_bus_src = bus_sources::PcHighSrc_Incrementor;
            pc_next_src = bus_sources::PcNextSrc_Bus;
        end
        CycleAddr4: begin
            addr_bus_pc();

            ctrl_signals[control_signals::LOAD_DL] = 1'b1;
        end
        CycleAddr5: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_DL;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;

            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            do_opcode(current_opcode);
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_abs_x();
    case(op_cycle)
        CycleDecode: begin
            advance_pc();
        end
        CycleAddr1: begin
            ctrl_signals[control_signals::LOAD_DL] = 1'b1;

            addr_bus_pc();
            advance_pc();

            alu_a_src = bus_sources::AluASrc_RegX;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b0;
        end
        CycleAddr2: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;
            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            if( alu_carry_out || current_opcode[7:4]==4'h9 )
            begin
                // If the opcode is STA for modes abs,x, we add the extra cycle
                // unconditionally for some unknown reason.

                if( CPU_VARIANT!=0 )
                    incompatible = 1'b1; // TODO implement 65c02 behavior

                alu_a_src = bus_sources::AluASrc_Mem;
                alu_b_src = bus_sources::AluBSrc_Zero;
                alu_carry_in = alu_carry_out;
                alu_op = control_signals::AluOp_add;
            end else begin
                op_cycle_next = FirstOpCycle;
                do_opcode(current_opcode);
            end
        end
        CycleAddr3: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_OL;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_ALU;
            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            op_cycle_next = FirstOpCycle;
            do_opcode(current_opcode);
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_abs_x_ind();
    case(op_cycle)
        CycleDecode: begin
            advance_pc();
        end
        CycleAddr1: begin
            addr_bus_pc();

            alu_a_src = bus_sources::AluASrc_RegX;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b0;
        end
        CycleAddr2: begin
            addr_bus_pc();

            alu_a_src = bus_sources::AluASrc_Mem;
            alu_b_src = bus_sources::AluBSrc_Zero;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = alu_carry_out;
        end
        CycleAddr3: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU_Latched;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_ALU;

            ctrl_signals[control_signals::LOAD_PCL] = 1'b1;
            ctrl_signals[control_signals::LOAD_PCH] = 1'b1;
            pcl_bus_src = bus_sources::PcLowSrc_Incrementor;
            pch_bus_src = bus_sources::PcHighSrc_Incrementor;
            pc_next_src = bus_sources::PcNextSrc_Bus;
        end
        CycleAddr4: begin
            addr_bus_pc();

            ctrl_signals[control_signals::LOAD_DL] = 1'b1;
        end
        CycleAddr5: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_DL;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;

            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            do_opcode(current_opcode);
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_abs_y();
    case(op_cycle)
        CycleDecode: begin
            advance_pc();
        end
        CycleAddr1: begin
            ctrl_signals[control_signals::LOAD_DL] = 1'b1;

            addr_bus_pc();
            advance_pc();

            alu_a_src = bus_sources::AluASrc_RegY;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b0;
        end
        CycleAddr2: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;
            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            if( alu_carry_out || current_opcode[7:4]==4'h9 ) begin
                // If the opcode is STA we add the extra cycle unconditionally for some unknown reason. Unlike STA abs,x,
                // this isn't even documented in the datasheet.

                if( CPU_VARIANT!=0 )
                    incompatible = 1'b1; // TODO implement 65c02 behavior

                alu_a_src = bus_sources::AluASrc_Mem;
                alu_b_src = bus_sources::AluBSrc_Zero;
                alu_carry_in = alu_carry_out;
                alu_op = control_signals::AluOp_add;
            end else begin
                op_cycle_next = FirstOpCycle;
                do_opcode(current_opcode);
            end
        end
        CycleAddr3: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_OL;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_ALU;
            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            op_cycle_next = FirstOpCycle;
            do_opcode(current_opcode);
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_acc();
    case(op_cycle)
        CycleDecode: begin
            op_cycle_next = FirstOpCycle;
            do_opcode(memory_in);
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_implied();
    case(op_cycle)
        CycleDecode: begin
            op_cycle_next = FirstOpCycle;
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_immediate();
    case(op_cycle)
        CycleDecode: begin
            op_cycle_next = FirstOpCycle;
            advance_pc();
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_pc_rel();
    case(op_cycle)
        CycleDecode: begin
            advance_pc();
            op_cycle_next = FirstOpCycle;
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_stack(input [7:0] opcode);
    case(op_cycle)
        CycleDecode: begin
            op_cycle_next = FirstOpCycle;
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_zp();
    case(op_cycle)
        CycleDecode: begin
            advance_pc();
        end
        CycleAddr1: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_Mem;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Zero;
            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            op_cycle_next = FirstOpCycle;
            do_opcode(current_opcode);
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_zp_ind();
    // TODO addressing mode not available on MOS6502
    case(op_cycle)
        CycleDecode: begin
        end
        CycleAddr1: begin
            advance_pc();

            // Fetch base address LSB
            addr_bus_low_src = bus_sources::AddrBusLowSrc_Mem;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Zero;

            // Calc MSB address
            alu_a_src = bus_sources::AluASrc_Mem;
            alu_b_src = bus_sources::AluBSrc_Zero;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b1;
        end
        CycleAddr2: begin
            // Fetch base address MSB
            addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Zero;

            // Store base address LSB
            ctrl_signals[control_signals::LOAD_DL] = 1'b1;
        end
        CycleAddr3: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_DL;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;
            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            op_cycle_next = FirstOpCycle;
            do_opcode(current_opcode);
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_zp_x();
    case(op_cycle)
        CycleDecode: begin
        end
        CycleAddr1: begin
            addr_bus_pc();
            advance_pc();

            alu_a_src = bus_sources::AluASrc_RegX;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b0;
        end
        CycleAddr2: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Zero;

            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            op_cycle_next = FirstOpCycle;
            do_opcode(current_opcode);
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_zp_y();
    case(op_cycle)
        CycleDecode: begin
        end
        CycleAddr1: begin
            addr_bus_pc();
            advance_pc();

            alu_a_src = bus_sources::AluASrc_RegY;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b0;
        end
        CycleAddr2: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Zero;

            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            op_cycle_next = FirstOpCycle;
            do_opcode(current_opcode);
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_zp_x_ind();
    case(op_cycle)
        CycleDecode: begin
        end
        CycleAddr1: begin
            addr_bus_pc();
            advance_pc();

            alu_a_src = bus_sources::AluASrc_RegX;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b0;
        end
        CycleAddr2: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Zero;

            alu_a_src = bus_sources::AluASrc_ALU;
            alu_b_src = bus_sources::AluBSrc_Zero;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b1;
        end
        CycleAddr3: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Zero;

            ctrl_signals[control_signals::LOAD_DL] = 1'b1;
        end
        CycleAddr4: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_DL;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;
            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            op_cycle_next = FirstOpCycle;
            do_opcode(current_opcode);
        end
        default: set_invalid_state();
    endcase
endtask

task addr_mode_zp_ind_y();
    case(op_cycle)
        CycleDecode: begin
            advance_pc();
        end
        CycleAddr1: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_Mem;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Zero;

            alu_a_src = bus_sources::AluASrc_Mem;
            alu_b_src = bus_sources::AluBSrc_Zero;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b1;
        end
        CycleAddr2: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Zero;

            alu_a_src = bus_sources::AluASrc_RegY;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b0;
        end
        CycleAddr3: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;

            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            if( alu_carry_out || current_opcode[7:4]==4'h9 ) begin
                // If the opcode is STA we add the extra cycle unconditionally for some unknown reason. Unlike STA abs,x,
                // this isn't even documented in the datasheet.

                if( CPU_VARIANT!=0 )
                    incompatible = 1; // BUG adapt to 65c02 bus semantics

                alu_a_src = bus_sources::AluASrc_Mem;
                alu_b_src = bus_sources::AluBSrc_Zero;
                alu_op = control_signals::AluOp_add;
                alu_carry_in = alu_carry_out;
            end else begin
                op_cycle_next = FirstOpCycle;
                do_opcode(current_opcode);
            end
        end
        CycleAddr4: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_OL;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_ALU;
            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            op_cycle_next = FirstOpCycle;
            do_opcode(current_opcode);
        end
        default: set_invalid_state();
    endcase
endtask

task next_instruction_no_bus();
    sync = 1'b1;
    if( pending_interrupt!=IntStateNone ) begin
        current_int_next = pending_interrupt;
        pending_interrupt_next = IntStateNone;
    end else if( interrupt_request && !status[control_signals::FlagsIrqMask] ) begin
        current_int_next = IntStateIrq;
        pending_interrupt_next = IntStateNone;
    end else
        advance_pc();

    op_cycle_next = CycleDecode;
endtask

task next_instruction();
    next_instruction_no_bus();
    addr_bus_pc();
endtask

task branch_opcode(input condition);
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_pc();

            if( !condition )
                next_instruction();
            else begin
                alu_a_src = bus_sources::AluASrc_PcLow;
                alu_b_src = bus_sources::AluBSrc_Mem;
                alu_op = control_signals::AluOp_add;
                alu_carry_in = 1'b0;
            end
        end
        CycleOp2: begin
            if( (jump_negative && alu_carry_out) || (!jump_negative && !alu_carry_out) ) begin
                // Didn't cross a page boundary
                next_instruction();

                addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU;
                pc_next_src = bus_sources::PcNextSrc_Bus;
            end else begin
                addr_bus_pc();

                if( CPU_VARIANT==0 ) begin
                    // Bug compatibility with the MOS6502.
                    addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU;
                end

                ctrl_signals[control_signals::LOAD_PCL] = 1'b1;
                pcl_bus_src = bus_sources::PcLowSrc_ALU;

                alu_a_src = bus_sources::AluASrc_PcHigh;
                alu_b_src = bus_sources::AluBSrc_Zero;
                alu_op = control_signals::AluOp_add;
                if( jump_negative ) begin
                    ctrl_signals[control_signals::AluInverseB] = 1'b1;
                    alu_carry_in = 1'b0;
                end else begin
                    alu_carry_in = 1'b1;
                end
            end
        end
        CycleOp3: begin
            next_instruction();

            addr_bus_high_src = bus_sources::AddrBusHighSrc_ALU;
            pc_next_src = bus_sources::PcNextSrc_Bus;
        end
        default: set_invalid_state();
    endcase
endtask

task op_adc();
    casex(op_cycle)
        CycleAnyAddr: begin
        end
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegA;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = status[control_signals::FlagsCarry];

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUpdateV] = 1'b1;
            ctrl_signals[control_signals::StatUseAlu] = 1'b1;

            ctrl_signals[control_signals::LOAD_A] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_eor();
    casex(op_cycle)
        CycleAnyAddr: begin
        end
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegA;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_xor;

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;

            ctrl_signals[control_signals::LOAD_A] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_and();
    casex(op_cycle)
        CycleAnyAddr: begin
        end
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegA;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_and;

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;

            ctrl_signals[control_signals::LOAD_A] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_asl();
    casex(op_cycle)
        CycleAnyAddr: begin
            memory_lock = 1'b1;
        end
        FirstOpCycle: begin
            addr_bus_ol();
            memory_lock = 1'b1;

            if( CPU_VARIANT==0 ) begin
                write = 1'b1;
                data_bus_src = bus_sources::DataBusSrc_Mem;
                data_out_src = bus_sources::DataOutSrc_DataBus;
            end

            alu_a_src = bus_sources::AluASrc_Mem;
            alu_op = control_signals::AluOp_shift_left;
            alu_carry_in = 1'b0;

            data_bus_src = bus_sources::DataBusSrc_Alu;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUseAlu] = 1'b1;
        end
        CycleOp2: begin
            addr_bus_ol();
            memory_lock = 1'b1;
            data_out_src = bus_sources::DataOutSrc_Alu;
            write = 1'b1;
        end
        CycleOp3: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_asl_A();
    casex(op_cycle)
        CycleAnyAddr: begin
        end
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegA;
            alu_op = control_signals::AluOp_shift_left;
            alu_carry_in = 1'b0;

            data_bus_src = bus_sources::DataBusSrc_Alu;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUseAlu] = 1'b1;

            ctrl_signals[control_signals::LOAD_A] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_bbrs();
    casex(op_cycle)
        CycleAnyAddr: begin
        end
        FirstOpCycle: begin
            addr_bus_ol();
        end
        CycleOp2: begin
            addr_bus_pc();
            advance_pc();

            case( current_opcode[6:4] )
                3'h0: bbrs_jump_next = memory_in[0] ^ !current_opcode[7];
                3'h1: bbrs_jump_next = memory_in[1] ^ !current_opcode[7];
                3'h2: bbrs_jump_next = memory_in[2] ^ !current_opcode[7];
                3'h3: bbrs_jump_next = memory_in[3] ^ !current_opcode[7];
                3'h4: bbrs_jump_next = memory_in[4] ^ !current_opcode[7];
                3'h5: bbrs_jump_next = memory_in[5] ^ !current_opcode[7];
                3'h6: bbrs_jump_next = memory_in[6] ^ !current_opcode[7];
                3'h7: bbrs_jump_next = memory_in[7] ^ !current_opcode[7];
            endcase

        end
        CycleOp3: begin
            addr_bus_pc();

            if( !bbrs_jump )
                next_instruction();
            else begin
                alu_a_src = bus_sources::AluASrc_PcLow;
                alu_b_src = bus_sources::AluBSrc_Mem;
                alu_op = control_signals::AluOp_add;
                alu_carry_in = 1'b0;
            end
        end
        CycleOp4: begin
            if( (jump_negative && alu_carry_out) || (!jump_negative && !alu_carry_out) ) begin
                // Didn't cross a page boundary
                next_instruction();

                addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU;
                pc_next_src = bus_sources::PcNextSrc_Bus;
            end else begin
                addr_bus_pc();

                if( CPU_VARIANT==0 ) begin
                    // Bug compatibility with the MOS6502.
                    addr_bus_low_src = bus_sources::AddrBusLowSrc_ALU;
                end

                ctrl_signals[control_signals::LOAD_PCL] = 1'b1;
                pcl_bus_src = bus_sources::PcLowSrc_ALU;

                alu_a_src = bus_sources::AluASrc_PcHigh;
                alu_b_src = bus_sources::AluBSrc_Zero;
                alu_op = control_signals::AluOp_add;
                if( jump_negative ) begin
                    ctrl_signals[control_signals::AluInverseB] = 1'b1;
                    alu_carry_in = 1'b0;
                end else begin
                    alu_carry_in = 1'b1;
                end
            end
        end
        CycleOp5: begin
            next_instruction();

            addr_bus_high_src = bus_sources::AddrBusHighSrc_ALU;
            pc_next_src = bus_sources::PcNextSrc_Bus;
        end
        default: set_invalid_state();
    endcase
endtask

task op_bcc();
    branch_opcode(! status[control_signals::FlagsCarry]);
endtask

task op_bcs();
    branch_opcode(status[control_signals::FlagsCarry]);
endtask

task op_beq();
    branch_opcode(status[control_signals::FlagsZero]);
endtask

task op_bne();
    branch_opcode(!status[control_signals::FlagsZero]);
endtask

task op_bmi();
    branch_opcode(status[control_signals::FlagsNegative]);
endtask

task op_bpl();
    branch_opcode(!status[control_signals::FlagsNegative]);
endtask

task op_bvc();
    branch_opcode(!status[control_signals::FlagsOverflow]);
endtask

task op_bvs();
    branch_opcode(status[control_signals::FlagsOverflow]);
endtask

task op_bra();
    branch_opcode(1'b1);
endtask

task op_bit();
    casex(op_cycle)
        CycleAnyAddr: begin
        end
        FirstOpCycle: begin
            data_bus_src = bus_sources::DataBusSrc_Mem;

            alu_a_src = bus_sources::AluASrc_RegA;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_and;

            if( current_opcode!=8'h89 ) begin
                // N and V flags not updated for the immediate form of the
                // opcode.
                ctrl_signals[control_signals::StatUpdateN] = 1'b1;
                ctrl_signals[control_signals::StatUpdateV] = 1'b1;
            end

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task post_bit();
    data_bus_src = bus_sources::DataBusSrc_AluLast;

    ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
    ctrl_signals[control_signals::StatCalcZero] = 1'b1;
endtask

task op_brk();
    case(op_cycle)
        CycleDecode: begin
            addr_bus_pc();
            if( current_int==IntStateNone )
                advance_pc();

            if( current_int!=IntStateReset )
                op_cycle_next = FirstOpCycle;
            else
                incompatible = 1'b1;
        end
        CycleAddr1: begin
            addr_bus_pc();
            incompatible = 1'b1;
            sync = 1'b1;
        end
        CycleAddr2: begin
            addr_bus_pc();
            incompatible = 1'b1;
            op_cycle_next = FirstOpCycle;
        end
        FirstOpCycle: begin
            stack_pointer_push();
            addr_bus_stack();
            if( current_int!=IntStateReset ) begin
                write = 1;
                data_out_src = bus_sources::DataOutSrc_DataBus;
            end
            data_bus_src = bus_sources::DataBusSrc_PcHigh;
        end
        CycleOp2: begin
            stack_pointer_push();
            addr_bus_stack();
            if( current_int!=IntStateReset ) begin
                write = 1;
                data_out_src = bus_sources::DataOutSrc_DataBus;
            end
            data_bus_src = bus_sources::DataBusSrc_PcLow;
        end
        CycleOp3: begin
            stack_pointer_push();
            addr_bus_stack();
            if( current_int!=IntStateReset ) begin
                write = 1;
                data_out_src = bus_sources::DataOutSrc_Status;
            end
            ctrl_signals[control_signals::StatOutputB] = current_int==IntStateNone ? 1'b1 : 1'b0;
        end
        CycleOp4: begin
            case( current_int )
                IntStateNone: addr_bus_low_src = bus_sources::AddrBusLowSrc_FE;
                IntStateReset: addr_bus_low_src = bus_sources::AddrBusLowSrc_FC;
                IntStateNmi: addr_bus_low_src = bus_sources::AddrBusLowSrc_FA;
                IntStateIrq: addr_bus_low_src = bus_sources::AddrBusLowSrc_FE;
            endcase
            addr_bus_high_src = bus_sources::AddrBusHighSrc_FF;
            vector_pull = 1'b1;

            data_bus_src = bus_sources::DataBusSrc_Ones;
            ctrl_signals[control_signals::StatUpdateI] = 1'b1;
        end
        CycleOp5: begin
            pcl_bus_src = bus_sources::PcLowSrc_Mem;
            ctrl_signals[control_signals::LOAD_PCL] = 1'b1;

            case( current_int )
                IntStateNone: addr_bus_low_src = bus_sources::AddrBusLowSrc_FF;
                IntStateReset: addr_bus_low_src = bus_sources::AddrBusLowSrc_FD;
                IntStateNmi: addr_bus_low_src = bus_sources::AddrBusLowSrc_FB;
                IntStateIrq: addr_bus_low_src = bus_sources::AddrBusLowSrc_FF;
            endcase
            addr_bus_high_src = bus_sources::AddrBusHighSrc_FF;
            vector_pull = 1'b1;

            if( CPU_VARIANT!=0 ) begin
                data_bus_src = bus_sources::DataBusSrc_Zero;
                ctrl_signals[control_signals::StatUpdateD] = 1'b1;
            end
        end
        CycleOp6: begin
            // Must be before the call to next_instruction, as we want it to
            // override us if necessary.
            current_int_next = IntStateNone;

            next_instruction();

            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;
            pc_next_src = bus_sources::PcNextSrc_Bus;
        end
    endcase
endtask

task op_clc();
    case(op_cycle)
        FirstOpCycle: begin
            data_bus_src = bus_sources::DataBusSrc_Zero;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_cld();
    case(op_cycle)
        FirstOpCycle: begin
            data_bus_src = bus_sources::DataBusSrc_Zero;
            ctrl_signals[control_signals::StatUpdateD] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_cli();
    case(op_cycle)
        FirstOpCycle: begin
            data_bus_src = bus_sources::DataBusSrc_Zero;
            ctrl_signals[control_signals::StatUpdateI] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_clv();
    case(op_cycle)
        FirstOpCycle: begin
            data_bus_src = bus_sources::DataBusSrc_Zero;
            ctrl_signals[control_signals::StatUpdateV] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_cmp();
    casex(op_cycle)
        CycleAnyAddr: begin
        end
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegA;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b1;
            ctrl_signals[control_signals::AluInverseB] = 1'b1;

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUseAlu] = 1'b1;

            next_instruction();
        end
    endcase
endtask

task op_cpx();
    casex(op_cycle)
        CycleAnyAddr: begin
        end
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegX;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b1;
            ctrl_signals[control_signals::AluInverseB] = 1'b1;

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUseAlu] = 1'b1;

            next_instruction();
        end
    endcase
endtask

task op_cpy();
    casex(op_cycle)
        CycleAnyAddr: begin
        end
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegY;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b1;
            ctrl_signals[control_signals::AluInverseB] = 1'b1;

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUseAlu] = 1'b1;

            next_instruction();
        end
    endcase
endtask

task op_dec();
    casex(op_cycle)
        CycleAnyAddr: memory_lock = 1'b1;
        FirstOpCycle: begin
            addr_bus_ol();
            memory_lock = 1'b1;

            alu_a_src = bus_sources::AluASrc_Mem;
            alu_b_src = bus_sources::AluBSrc_Zero;
            alu_op = control_signals::AluOp_add;
            ctrl_signals[control_signals::AluInverseB] = 1'b1;
            alu_carry_in = 1'b0;

            if( CPU_VARIANT==0 ) begin
                write = 1'b1;
                data_out_src = bus_sources::DataOutSrc_DataBus;

                // Should have written the value read from memory. Doing so
                // would have required us to add another control option to
                // DataOutSrc, which doesn't make sense just to reproduce
                // a CPU bug.
                // data_bus_src = bus_sources::DataBusSrc_Mem;
                // Instead, we write the *correct* value twice.
                incompatible = 1'b1;
            end

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
        end
        CycleOp2: begin
            addr_bus_ol();
            memory_lock = 1'b1;

            data_out_src = bus_sources::DataOutSrc_Alu;
            write = 1'b1;
        end
        CycleOp3: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_dec_A();
    casex(op_cycle)
        CycleAnyAddr: ;
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegA;
            alu_b_src = bus_sources::AluBSrc_Zero;
            alu_op = control_signals::AluOp_add;
            ctrl_signals[control_signals::AluInverseB] = 1'b1;
            alu_carry_in = 1'b0;

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::LOAD_A] = 1'b1;

            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_dex();
    case(op_cycle)
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegX;
            alu_b_src = bus_sources::AluBSrc_Zero;
            alu_op = control_signals::AluOp_add;
            ctrl_signals[control_signals::AluInverseB] = 1'b1;
            alu_carry_in = 1'b0;

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::LOAD_X] = 1'b1;

            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_dey();
    case(op_cycle)
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegY;
            alu_b_src = bus_sources::AluBSrc_Zero;
            alu_op = control_signals::AluOp_add;
            ctrl_signals[control_signals::AluInverseB] = 1'b1;
            alu_carry_in = 1'b0;

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::LOAD_Y] = 1'b1;

            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_inc();
    casex(op_cycle)
        CycleAnyAddr: memory_lock = 1'b1;
        FirstOpCycle: begin
            addr_bus_ol();
            memory_lock = 1'b1;

            alu_a_src = bus_sources::AluASrc_Mem;
            alu_b_src = bus_sources::AluBSrc_Zero;
            alu_op = control_signals::AluOp_add;
            ctrl_signals[control_signals::AluInverseB] = 1'b0;
            alu_carry_in = 1'b1;

            if( CPU_VARIANT==0 ) begin
                write = 1'b1;
                data_out_src = bus_sources::DataOutSrc_DataBus;

                // Should have written the value read from memory. Doing so
                // would have required us to add another control option to
                // DataOutSrc, which doesn't make sense just to reproduce
                // a CPU bug.
                // data_bus_src = bus_sources::DataBusSrc_Mem;
                // Instead, we write the *correct* value twice.
                incompatible = 1'b1;
            end

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
        end
        CycleOp2: begin
            addr_bus_ol();
            memory_lock = 1'b1;

            data_out_src = bus_sources::DataOutSrc_Alu;
            write = 1'b1;
        end
        CycleOp3: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_inc_A();
    casex(op_cycle)
        CycleAnyAddr: ;
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegA;
            alu_b_src = bus_sources::AluBSrc_Zero;
            alu_op = control_signals::AluOp_add;
            ctrl_signals[control_signals::AluInverseB] = 1'b0;
            alu_carry_in = 1'b1;

            data_bus_src = bus_sources::DataBusSrc_Alu;
            ctrl_signals[control_signals::LOAD_A] = 1'b1;

            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_inx();
    case(op_cycle)
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegX;
            alu_b_src = bus_sources::AluBSrc_Zero;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b1;

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::LOAD_X] = 1'b1;

            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_iny();
    case(op_cycle)
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegY;
            alu_b_src = bus_sources::AluBSrc_Zero;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = 1'b1;

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::LOAD_Y] = 1'b1;

            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_jmp();
    casex(op_cycle)
        CycleAnyAddr: begin
            next_instruction_no_bus();
            pc_next_src = bus_sources::PcNextSrc_Bus;
        end
        default: set_invalid_state();
    endcase
endtask

task op_jsr();
    case(op_cycle)
        FirstOpCycle: begin
            ctrl_signals[control_signals::LOAD_DL] = 1'b1;
            advance_pc();

            addr_bus_stack();
        end
        CycleOp2: begin
            addr_bus_stack();
            data_bus_src = bus_sources::DataBusSrc_PcHigh;
            data_out_src = bus_sources::DataOutSrc_DataBus;
            write = 1;

            stack_pointer_push();
        end
        CycleOp3: begin
            addr_bus_stack();
            data_bus_src = bus_sources::DataBusSrc_PcLow;
            data_out_src = bus_sources::DataOutSrc_DataBus;
            write = 1;

            stack_pointer_push();
        end
        CycleOp4: begin
            addr_bus_pc();
        end
        CycleOp5: begin
            next_instruction();

            addr_bus_low_src = bus_sources::AddrBusLowSrc_DL;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;
            pc_next_src = bus_sources::PcNextSrc_Bus;
        end
        default: set_invalid_state();
    endcase
endtask

task op_lda();
    casex(op_cycle)
        CycleAnyAddr: ;        // Nothing to do here
        FirstOpCycle: begin
            special_bus_src = bus_sources::SpecialBusSrc_Mem;
            ctrl_signals[control_signals::LOAD_A] = 1'b1;

            data_bus_src = bus_sources::DataBusSrc_Special;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_ldx();
    casex(op_cycle)
        CycleAnyAddr: ;        // Nothing to do here
        FirstOpCycle: begin
            data_bus_src = bus_sources::DataBusSrc_Mem;
            ctrl_signals[control_signals::LOAD_X] = 1'b1;

            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_ldy();
    casex(op_cycle)
        CycleAnyAddr: ;
        FirstOpCycle: begin
            data_bus_src = bus_sources::DataBusSrc_Mem;
            ctrl_signals[control_signals::LOAD_Y] = 1'b1;

            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_lsr();
    casex(op_cycle)
        CycleAnyAddr: begin
            memory_lock = 1'b1;
        end
        FirstOpCycle: begin
            addr_bus_ol();
            memory_lock = 1'b1;

            if( CPU_VARIANT==0 ) begin
                write = 1'b1;
                data_out_src = bus_sources::DataOutSrc_DataBus;

                // Should have written the value read from memory. Doing so
                // would have required us to add another control option to
                // DataOutSrc, which doesn't make sense just to reproduce
                // a CPU bug.
                // data_bus_src = bus_sources::DataBusSrc_Mem;
                // Instead, we write the *correct* value twice.
                incompatible = 1'b1;
            end

            alu_a_src = bus_sources::AluASrc_Mem;
            alu_op = control_signals::AluOp_shift_right_logical;
            alu_carry_in = 1'b0;

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUseAlu] = 1'b1;
        end
        CycleOp2: begin
            addr_bus_ol();
            memory_lock = 1'b1;
            data_out_src = bus_sources::DataOutSrc_Alu;
            write = 1'b1;
        end
        CycleOp3: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_lsr_A();
    casex(op_cycle)
        CycleAnyAddr: begin
        end
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegA;
            alu_op = control_signals::AluOp_shift_right_logical;
            alu_carry_in = 1'b0;

            data_bus_src = bus_sources::DataBusSrc_Alu;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUseAlu] = 1'b1;

            ctrl_signals[control_signals::LOAD_A] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_nop();
    case(op_cycle)
        FirstOpCycle: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_ora();
    casex(op_cycle)
        CycleAnyAddr: begin
        end
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegA;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_or;

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;

            ctrl_signals[control_signals::LOAD_A] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_pha();
    case(op_cycle)
        FirstOpCycle: begin
            special_bus_src = bus_sources::SpecialBusSrc_RegA;
            data_bus_src = bus_sources::DataBusSrc_Special;
            data_out_src = bus_sources::DataOutSrc_DataBus;
            write = 1'b1;
            addr_bus_stack();
        end
        CycleOp2: begin
            next_instruction();

            stack_pointer_push();
        end
        default: set_invalid_state();
    endcase
endtask

task op_phx();
    case(op_cycle)
        FirstOpCycle: begin
            special_bus_src = bus_sources::SpecialBusSrc_RegX;
            data_bus_src = bus_sources::DataBusSrc_Special;
            data_out_src = bus_sources::DataOutSrc_DataBus;
            write = 1'b1;
            addr_bus_stack();
        end
        CycleOp2: begin
            next_instruction();

            stack_pointer_push();
        end
        default: set_invalid_state();
    endcase
endtask

task op_phy();
    case(op_cycle)
        FirstOpCycle: begin
            special_bus_src = bus_sources::SpecialBusSrc_RegY;
            data_bus_src = bus_sources::DataBusSrc_Special;
            data_out_src = bus_sources::DataOutSrc_DataBus;
            write = 1'b1;
            addr_bus_stack();
        end
        CycleOp2: begin
            next_instruction();

            stack_pointer_push();
        end
        default: set_invalid_state();
    endcase
endtask

task op_php();
    case(op_cycle)
        FirstOpCycle: begin
            data_out_src = bus_sources::DataOutSrc_Status;
            ctrl_signals[control_signals::StatOutputB] = 1'b1;
            write = 1'b1;
            addr_bus_stack();

            stack_pointer_push();
        end
        CycleOp2: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_pla();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_stack();

            stack_pointer_pop();
        end
        CycleOp2: begin
            addr_bus_stack();
        end
        CycleOp3: begin
            next_instruction();

            data_bus_src = bus_sources::DataBusSrc_Mem;
            ctrl_signals[control_signals::LOAD_A] = 1'b1;

            // Store status flag
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
        end
        default: set_invalid_state();
    endcase
endtask

task op_plx();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_stack();

            stack_pointer_pop();
        end
        CycleOp2: begin
            addr_bus_stack();
        end
        CycleOp3: begin
            next_instruction();

            data_bus_src = bus_sources::DataBusSrc_Mem;
            ctrl_signals[control_signals::LOAD_X] = 1'b1;

            // Store status flag
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
        end
        default: set_invalid_state();
    endcase
endtask

task op_ply();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_stack();

            stack_pointer_pop();
        end
        CycleOp2: begin
            addr_bus_stack();
        end
        CycleOp3: begin
            next_instruction();

            data_bus_src = bus_sources::DataBusSrc_Mem;
            ctrl_signals[control_signals::LOAD_Y] = 1'b1;

            // Store status flag
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
        end
        default: set_invalid_state();
    endcase
endtask

task op_plp();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_stack();

            stack_pointer_pop();
        end
        CycleOp2: begin
            addr_bus_stack();
        end
        CycleOp3: begin
            next_instruction();

            // Store status flag
            data_bus_src = bus_sources::DataBusSrc_Mem;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatUpdateI] = 1'b1;
            ctrl_signals[control_signals::StatUpdateD] = 1'b1;
            ctrl_signals[control_signals::StatUpdateV] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
        end
        default: set_invalid_state();
    endcase
endtask

task op_rol();
    casex(op_cycle)
        CycleAnyAddr: begin
            memory_lock = 1'b1;
        end
        FirstOpCycle: begin
            addr_bus_ol();
            memory_lock = 1'b1;

            if( CPU_VARIANT==0 ) begin
                write = 1'b1;
                data_out_src = bus_sources::DataOutSrc_DataBus;

                // Should have written the value read from memory. Doing so
                // would have required us to add another control option to
                // DataOutSrc, which doesn't make sense just to reproduce
                // a CPU bug.
                // data_bus_src = bus_sources::DataBusSrc_Mem;
                // Instead, we write the *correct* value twice.
                incompatible = 1'b1;
            end

            alu_a_src = bus_sources::AluASrc_Mem;
            alu_op = control_signals::AluOp_shift_left;
            alu_carry_in = status[control_signals::FlagsCarry];

            data_bus_src = bus_sources::DataBusSrc_Alu;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUseAlu] = 1'b1;
        end
        CycleOp2: begin
            addr_bus_ol();
            memory_lock = 1'b1;
            data_out_src = bus_sources::DataOutSrc_Alu;
            write = 1'b1;
        end
        CycleOp3: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_rol_A();
    casex(op_cycle)
        CycleAnyAddr: begin
        end
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegA;
            alu_op = control_signals::AluOp_shift_left;
            alu_carry_in = status[control_signals::FlagsCarry];

            data_bus_src = bus_sources::DataBusSrc_Alu;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUseAlu] = 1'b1;

            ctrl_signals[control_signals::LOAD_A] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_ror();
    casex(op_cycle)
        CycleAnyAddr: begin
            memory_lock = 1'b1;
        end
        FirstOpCycle: begin
            addr_bus_ol();
            memory_lock = 1'b1;

            if( CPU_VARIANT==0 ) begin
                write = 1'b1;
                data_out_src = bus_sources::DataOutSrc_DataBus;

                // Should have written the value read from memory. Doing so
                // would have required us to add another control option to
                // DataOutSrc, which doesn't make sense just to reproduce
                // a CPU bug.
                // data_bus_src = bus_sources::DataBusSrc_Mem;
                // Instead, we write the *correct* value twice.
                incompatible = 1'b1;
            end

            alu_a_src = bus_sources::AluASrc_Mem;
            alu_op = control_signals::AluOp_shift_right_logical;
            alu_carry_in = status[control_signals::FlagsCarry];

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUseAlu] = 1'b1;
        end
        CycleOp2: begin
            addr_bus_ol();
            memory_lock = 1'b1;
            data_out_src = bus_sources::DataOutSrc_Alu;
            write = 1'b1;
        end
        CycleOp3: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_ror_A();
    casex(op_cycle)
        CycleAnyAddr: begin
        end
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegA;
            alu_op = control_signals::AluOp_shift_right_logical;
            alu_carry_in = status[control_signals::FlagsCarry];

            data_bus_src = bus_sources::DataBusSrc_Alu;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUseAlu] = 1'b1;

            ctrl_signals[control_signals::LOAD_A] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_rti();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_stack();

            stack_pointer_pop();
        end
        CycleOp2: begin
            // Read status flag
            addr_bus_stack();

            stack_pointer_pop();
        end
        CycleOp3: begin
            // Store status flag
            data_bus_src = bus_sources::DataBusSrc_Mem;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatUpdateI] = 1'b1;
            ctrl_signals[control_signals::StatUpdateD] = 1'b1;
            ctrl_signals[control_signals::StatUpdateV] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;

            // Read PC LSB
            addr_bus_stack();

            stack_pointer_pop();
        end
        CycleOp4: begin
            ctrl_signals[control_signals::LOAD_PCL] = 1'b1;
            pcl_bus_src = bus_sources::PcLowSrc_Mem;

            // Read PC MSB
            addr_bus_stack();
        end
        CycleOp5: begin
            ctrl_signals[control_signals::LOAD_PCH] = 1'b1;
            pch_bus_src = bus_sources::PcHighSrc_Mem;

            next_instruction();

            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;
            pc_next_src = bus_sources::PcNextSrc_Bus;
        end
        default: set_invalid_state();
    endcase
endtask

task op_rts();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_stack();

            stack_pointer_pop();
        end
        CycleOp2: begin
            // Read PC LSB
            addr_bus_stack();

            stack_pointer_pop();
        end
        CycleOp3: begin
            ctrl_signals[control_signals::LOAD_PCL] = 1'b1;
            pcl_bus_src = bus_sources::PcLowSrc_Mem;

            // Read PC MSB
            addr_bus_stack();
        end
        CycleOp4: begin
            ctrl_signals[control_signals::LOAD_PCH] = 1'b1;
            pch_bus_src = bus_sources::PcHighSrc_Mem;

            addr_bus_pc();
            advance_pc();

            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;
            pc_next_src = bus_sources::PcNextSrc_Bus;
        end
        CycleOp5: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_rsmb();
    casex(op_cycle)
        CycleAnyAddr: begin
            memory_lock = 1'b1;
        end
        FirstOpCycle: begin
            addr_bus_ol();
            memory_lock = 1'b1;

            alu_a_src = bus_sources::AluASrc_Mem;
            case(current_opcode[6:4])
                0: alu_b_src = bus_sources::AluBSrc_Bit0;
                1: alu_b_src = bus_sources::AluBSrc_Bit1;
                2: alu_b_src = bus_sources::AluBSrc_Bit2;
                3: alu_b_src = bus_sources::AluBSrc_Bit3;
                4: alu_b_src = bus_sources::AluBSrc_Bit4;
                5: alu_b_src = bus_sources::AluBSrc_Bit5;
                6: alu_b_src = bus_sources::AluBSrc_Bit6;
                7: alu_b_src = bus_sources::AluBSrc_Bit7;
            endcase

            if( current_opcode[7] ) begin
                // SMB
                ctrl_signals[control_signals::AluInverseB] = 0;
                alu_op = control_signals::AluOp_or;
            end else begin
                // SMB
                ctrl_signals[control_signals::AluInverseB] = 1;
                alu_op = control_signals::AluOp_and;
            end
        end
        CycleOp2: begin
            addr_bus_ol();
            memory_lock = 1'b1;

            data_out_src = bus_sources::DataOutSrc_Alu;
            write = 1'b1;
        end
        CycleOp3: next_instruction();
        default: set_invalid_state();
    endcase
endtask

task op_sbc();
    casex(op_cycle)
        CycleAnyAddr: begin
        end
        FirstOpCycle: begin
            alu_a_src = bus_sources::AluASrc_RegA;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_add;
            alu_carry_in = status[control_signals::FlagsCarry];
            ctrl_signals[control_signals::AluInverseB] = 1'b1;

            data_bus_src = bus_sources::DataBusSrc_Alu;

            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;
            ctrl_signals[control_signals::StatUpdateV] = 1'b1;
            ctrl_signals[control_signals::StatUseAlu] = 1'b1;

            ctrl_signals[control_signals::LOAD_A] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_sec();
    case(op_cycle)
        FirstOpCycle: begin
            data_bus_src = bus_sources::DataBusSrc_Ones;
            ctrl_signals[control_signals::StatUpdateC] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_sed();
    case(op_cycle)
        FirstOpCycle: begin
            data_bus_src = bus_sources::DataBusSrc_Ones;
            ctrl_signals[control_signals::StatUpdateD] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_sei();
    case(op_cycle)
        FirstOpCycle: begin
            data_bus_src = bus_sources::DataBusSrc_Ones;
            ctrl_signals[control_signals::StatUpdateI] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_sta();
    casex(op_cycle)
        CycleAnyAddr: begin
            special_bus_src = bus_sources::SpecialBusSrc_RegA;
            data_bus_src = bus_sources::DataBusSrc_Special;
            data_out_src = bus_sources::DataOutSrc_DataBus;
            write = 1'b1;
        end
        FirstOpCycle: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_stx();
    casex(op_cycle)
        CycleAnyAddr: begin
            special_bus_src = bus_sources::SpecialBusSrc_RegX;
            data_bus_src = bus_sources::DataBusSrc_Special;
            data_out_src = bus_sources::DataOutSrc_DataBus;
            write = 1'b1;
        end
        FirstOpCycle: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_sty();
    casex(op_cycle)
        CycleAnyAddr: begin
            special_bus_src = bus_sources::SpecialBusSrc_RegY;
            data_bus_src = bus_sources::DataBusSrc_Special;
            data_out_src = bus_sources::DataOutSrc_DataBus;
            write = 1'b1;
        end
        FirstOpCycle: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_stz();
    casex(op_cycle)
        CycleAnyAddr: begin
            data_bus_src = bus_sources::DataBusSrc_Zero;
            data_out_src = bus_sources::DataOutSrc_DataBus;
            write = 1'b1;
        end
        FirstOpCycle: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_stp();
    addr_bus_pc();

    op_cycle_next = FirstOpCycle;
endtask

task op_tax();
    data_bus_src = bus_sources::DataBusSrc_Special;
    special_bus_src = bus_sources::SpecialBusSrc_RegA;
    ctrl_signals[control_signals::LOAD_X] = 1'b1;

    ctrl_signals[control_signals::StatUpdateN] = 1'b1;
    ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
    ctrl_signals[control_signals::StatCalcZero] = 1'b1;

    next_instruction();
endtask

task op_tay();
    data_bus_src = bus_sources::DataBusSrc_Special;
    special_bus_src = bus_sources::SpecialBusSrc_RegA;
    ctrl_signals[control_signals::LOAD_Y] = 1'b1;

    ctrl_signals[control_signals::StatUpdateN] = 1'b1;
    ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
    ctrl_signals[control_signals::StatCalcZero] = 1'b1;

    next_instruction();
endtask

task op_tsx();
    special_bus_src = bus_sources::SpecialBusSrc_RegSP;
    data_bus_src = bus_sources::DataBusSrc_Special;
    ctrl_signals[control_signals::LOAD_X] = 1'b1;

    ctrl_signals[control_signals::StatUpdateN] = 1'b1;
    ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
    ctrl_signals[control_signals::StatCalcZero] = 1'b1;

    next_instruction();
endtask

task op_txa();
    data_bus_src = bus_sources::DataBusSrc_Special;
    special_bus_src = bus_sources::SpecialBusSrc_RegX;
    ctrl_signals[control_signals::LOAD_A] = 1'b1;

    ctrl_signals[control_signals::StatUpdateN] = 1'b1;
    ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
    ctrl_signals[control_signals::StatCalcZero] = 1'b1;

    next_instruction();
endtask

task op_txs();
    alu_op = control_signals::AluOp_add;
    alu_a_src = bus_sources::AluASrc_RegX;
    alu_b_src = bus_sources::AluBSrc_Zero;
    alu_carry_in = 1'b0;
    ctrl_signals[control_signals::LOAD_SP] = 1'b1;

    next_instruction();
endtask

task op_tya();
    data_bus_src = bus_sources::DataBusSrc_Special;
    special_bus_src = bus_sources::SpecialBusSrc_RegY;
    ctrl_signals[control_signals::LOAD_A] = 1'b1;

    ctrl_signals[control_signals::StatUpdateN] = 1'b1;
    ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
    ctrl_signals[control_signals::StatCalcZero] = 1'b1;

    next_instruction();
endtask

task op_trsb();
    casex(op_cycle)
        CycleAnyAddr: begin
            memory_lock = 1'b1;
        end
        FirstOpCycle: begin
            addr_bus_ol();
            memory_lock = 1'b1;

            alu_a_src = bus_sources::AluASrc_Mem;
            alu_b_src = bus_sources::AluBSrc_RegA;
            if( current_opcode[4] ) begin
                // TRB
                alu_op = control_signals::AluOp_and;
                ctrl_signals[control_signals::AluInverseB] = 1'b1;
            end else begin
                // TSB
                alu_op = control_signals::AluOp_or;
                ctrl_signals[control_signals::AluInverseB] = 1'b0;
            end
        end
        CycleOp2: begin
            addr_bus_ol();
            memory_lock = 1'b1;
            write = 1'b1;

            data_out_src = bus_sources::DataOutSrc_Alu;

            alu_a_src = bus_sources::AluASrc_RegA;
            alu_b_src = bus_sources::AluBSrc_Mem;
            alu_op = control_signals::AluOp_and;
            ctrl_signals[control_signals::AluInverseB] = 1'b0;

            data_bus_src = bus_sources::DataBusSrc_Alu;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;
        end
        CycleOp3: begin
            next_instruction();
        end
    endcase
endtask

task op_wai();
    addr_bus_pc();

    if( pending_interrupt!=IntStateNone || interrupt_request )
        next_instruction();
    else
        op_cycle_next = FirstOpCycle;
endtask

endmodule
