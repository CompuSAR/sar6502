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
    input [7:0]status,
    input alu_carry_out,
        input IRQ,
        input NMI,

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

enum { IntStateNone, IntStateReset, IntStateNmi, IntStateIrq } int_state = IntStateReset, int_state_next;

always_ff@(posedge clock) begin
    if( ready ) begin
        op_cycle <= op_cycle_next;
        current_opcode <= next_opcode;
        int_state <= int_state_next;
        jump_negative <= memory_in[7];
    end
end

task set_invalid_state();
begin
    op_cycle_next = CycleInvalid;
    next_opcode = 8'hXX;
    incompatible = 1'bX;
    alu_carry_in = 1'bX;

    addr_bus_low_src = bus_sources::AddrBusLowSrc_Invalid;
    addr_bus_high_src = bus_sources::AddrBusHighSrc_Invalid;
    data_bus_src = bus_sources::DataBusSrc_Invalid;
    special_bus_src = bus_sources::SpecialBusSrc_Invalid;
    alu_a_src = bus_sources::AluASrc_Invalid;
    alu_b_src = bus_sources::AluBSrc_Invalid;
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
            op_cycle_next = FirstOpCycle;
        end else begin
            next_opcode = memory_in;

            addr_bus_pc();
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

    special_bus_src = bus_sources::SpecialBusSrc_ALU;
    ctrl_signals[control_signals::LOAD_SP] = 1'b1;
endtask

task stack_pointer_pop();
    alu_a_src = bus_sources::AluASrc_RegSp;
    alu_b_src = bus_sources::AluBSrc_Zero;
    alu_op = control_signals::AluOp_add;
    ctrl_signals[control_signals::AluInverseB] = 1'b0;
    alu_carry_in = 1'b1;

    special_bus_src = bus_sources::SpecialBusSrc_ALU;
    ctrl_signals[control_signals::LOAD_SP] = 1'b1;
endtask

task do_address(input [7:0] opcode);
    case(opcode)
        8'h00: addr_mode_stack(opcode);         // BRK
        8'h08: addr_mode_stack(opcode);         // PHP
        8'h10: addr_mode_pc_rel();              // BPL
        8'h20: addr_mode_stack(opcode);         // JSR
        8'h28: addr_mode_stack(opcode);         // PLP
        8'h30: addr_mode_pc_rel();              // BMI
        8'h40: addr_mode_stack(opcode);         // RTI
        8'h48: addr_mode_stack(opcode);         // PHA
        8'h50: addr_mode_pc_rel();              // BVC
        8'h60: addr_mode_stack(opcode);         // RTS
        8'h70: addr_mode_pc_rel();              // BVS
        8'h80: addr_mode_pc_rel();              // BRA
        8'h8d: addr_mode_adsolute();            // STA abs
        8'h90: addr_mode_pc_rel();              // BCC
        8'h9a: addr_mode_implied();             // TXS
        8'ha0: addr_mode_immediate();           // LDY #
        8'ha2: addr_mode_immediate();           // LDX #
        8'ha5: addr_mode_zp();                  // LDA zp
        8'ha9: addr_mode_immediate();           // LDA #
        8'had: addr_mode_adsolute();            // LDA abs
        8'hb0: addr_mode_pc_rel();              // BCS
        8'hd0: addr_mode_pc_rel();              // BNE
        8'hda: addr_mode_stack(opcode);         // PHX
        8'hea: addr_mode_implied();             // NOP
        8'hf0: addr_mode_pc_rel();              // BEQ
        default: set_invalid_state();
    endcase
endtask

task do_opcode(input [7:0]opcode);
    case(opcode)
        8'h00: op_brk();
        8'h08: op_php();
        8'h10: op_bpl();
        8'h20: op_jsr();
        8'h28: op_plp();
        8'h30: op_bmi();
        8'h40: op_rti();
        8'h48: op_pha();
        8'h50: op_bvc();
        8'h60: op_rts();
        8'h70: op_bvs();
        8'h80: op_bra();
        8'h8d: op_sta();                        // STA abs
        8'h90: op_bcc();
        8'h9a: op_txs();
        8'ha0: op_ldy();                        // LDY #
        8'ha2: op_ldx();                        // LDX #
        8'ha5: op_lda();                        // LDA zp
        8'ha9: op_lda();                        // LDA #
        8'had: op_lda();                        // LDA abs
        8'hb0: op_bcs();
        8'hd0: op_bne();
        8'hdb: op_stp();
        8'hda: op_phx();
        8'hea: op_nop();
        8'hf0: op_beq();
        default: set_invalid_state();
    endcase
endtask

task addr_mode_adsolute();
    case(op_cycle)
        CycleDecode: begin
            advance_pc();
        end
        CycleAddr1: begin
            ctrl_signals[control_signals::LOAD_DL] = 1'b1;

            addr_bus_pc();
            advance_pc();

            op_cycle_next = LastAddrCycle;
        end
        LastAddrCycle: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_DL;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;
            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            do_opcode(current_opcode);
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
            op_cycle_next = LastAddrCycle;
        end
        LastAddrCycle: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_Mem;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_Zero;
            ctrl_signals[control_signals::LOAD_OL] = 1'b1;

            do_opcode(current_opcode);
        end
        default: set_invalid_state();
    endcase
endtask

task next_instruction();
    sync = 1'b1;
    addr_bus_pc();
    advance_pc();

    op_cycle_next = CycleDecode;
endtask

task branch_opcode(input condition);
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_pc();

            if( !condition )
                next_instruction();
            else begin
                alu_a_src = bus_sources::AluASrc_PcLow;
                alu_b_src = bus_sources::AluBSrc_DataBus;
                data_bus_src = bus_sources::DataBusSrc_Mem;
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


task op_brk();
    case(op_cycle)
        FirstOpCycle: begin
            addr_bus_low_src = bus_sources::AddrBusLowSrc_FC;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_FF;
            vector_pull = 1'b1;
        end
        CycleOp2: begin
            pcl_bus_src = bus_sources::PcLowSrc_Mem;
            ctrl_signals[control_signals::LOAD_PCL] = 1'b1;

            addr_bus_low_src = bus_sources::AddrBusLowSrc_FD;
            addr_bus_high_src = bus_sources::AddrBusHighSrc_FF;
            vector_pull = 1'b1;
        end
        CycleOp3: begin
            next_instruction();

            addr_bus_high_src = bus_sources::AddrBusHighSrc_Mem;
            pc_next_src = bus_sources::PcNextSrc_Bus;
        end
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
            write = 1;

            stack_pointer_push();
        end
        CycleOp3: begin
            addr_bus_stack();
            data_bus_src = bus_sources::DataBusSrc_PcLow;
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
    case(op_cycle)
        LastAddrCycle: ;        // Nothing to do here
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
    case(op_cycle)
        FirstOpCycle: begin
            special_bus_src = bus_sources::SpecialBusSrc_Mem;
            ctrl_signals[control_signals::LOAD_X] = 1'b1;

            data_bus_src = bus_sources::DataBusSrc_Special;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;

            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_ldy();
    case(op_cycle)
        FirstOpCycle: begin
            special_bus_src = bus_sources::SpecialBusSrc_Mem;
            ctrl_signals[control_signals::LOAD_Y] = 1'b1;

            data_bus_src = bus_sources::DataBusSrc_Special;
            ctrl_signals[control_signals::StatUpdateN] = 1'b1;
            ctrl_signals[control_signals::StatUpdateZ] = 1'b1;
            ctrl_signals[control_signals::StatCalcZero] = 1'b1;

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

task op_pha();
    case(op_cycle)
        FirstOpCycle: begin
            special_bus_src = bus_sources::SpecialBusSrc_RegA;
            data_bus_src = bus_sources::DataBusSrc_Special;
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
            data_bus_src = bus_sources::DataBusSrc_Status;
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

task op_sta();
    case(op_cycle)
        LastAddrCycle: begin
            special_bus_src = bus_sources::SpecialBusSrc_RegA;
            data_bus_src = bus_sources::DataBusSrc_Special;
            write = 1'b1;
        end
        FirstOpCycle: begin
            next_instruction();
        end
        default: set_invalid_state();
    endcase
endtask

task op_stp();
    op_cycle_next = FirstOpCycle;
endtask

task op_txs();
    special_bus_src = bus_sources::SpecialBusSrc_RegX;
    ctrl_signals[control_signals::LOAD_SP] = 1'b1;

    next_instruction();
endtask

endmodule
