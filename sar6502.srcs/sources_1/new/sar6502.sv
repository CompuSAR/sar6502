`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:  Some Assembly Required
// Engineer: Shachar Shemesh
// 
// Create Date: 02/23/2022 05:40:43 AM
// Design Name: sar6502
// Module Name: sar6502
// Project Name: CompuSAR
// Target Devices: 
// Tool Versions: 
// Description: Customizable implementation of the 6502 CPU
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

module sar6502#(parameter CPU_VARIANT = 2)
(
    input clock,

    output [15:0] address,
    output logic[7:0] data_out,
    output write,

    input [7:0] data_in,
    input ready,

    input reset,
    input interrupt,
    input nmi,

    input set_overflow,
    output memory_lock,

    output vector_pull,
    output sync,

    output incompatible        // Address bus is deliberately incompatible with our base CPU
);

logic [7:0] data_bus;
logic [7:0] address_bus_low, address_bus_high;
logic [7:0] special_bus;
logic [7:0] pcl_in, pch_in;
assign address = {address_bus_high, address_bus_low};
logic [control_signals::ctrl_signals_last:0] control_signals;
logic [15:0] pc_next;
logic [7:0] alu_a_input, alu_b_input;

register        reg_a(.clock(clock), .data_in(data_bus), .latch(decoder.ctrl_signals[control_signals::LOAD_A]), .ready(ready)),
                reg_x(.clock(clock), .data_in(data_bus), .latch(decoder.ctrl_signals[control_signals::LOAD_X]), .ready(ready)),
                reg_y(.clock(clock), .data_in(data_bus), .latch(decoder.ctrl_signals[control_signals::LOAD_Y]), .ready(ready)),
                reg_sp(.clock(clock), .data_in(alu.result), .latch(decoder.ctrl_signals[control_signals::LOAD_SP]), .ready(ready)),
                reg_pcl(.clock(clock), .data_in(pcl_in), .latch(decoder.ctrl_signals[control_signals::LOAD_PCL]), .ready(ready)),
                reg_pch(.clock(clock), .data_in(pch_in), .latch(decoder.ctrl_signals[control_signals::LOAD_PCH]), .ready(ready)),
                // Data latch
                reg_dl(.clock(clock), .data_in(data_in), .latch(decoder.ctrl_signals[control_signals::LOAD_DL]), .ready(ready)),
                // Output latch
                reg_oll(.clock(clock), .data_in(address_bus_low), .latch(decoder.ctrl_signals[control_signals::LOAD_OL]), .ready(ready)),
                reg_olh(.clock(clock), .data_in(address_bus_high), .latch(decoder.ctrl_signals[control_signals::LOAD_OL]), .ready(ready));

alu alu(
    .a(alu_a_input),
    .b(alu_b_input),
    .op(decoder.alu_op),
    .carry_in(decoder.alu_carry_in),
    .inverse_b(decoder.ctrl_signals[control_signals::AluInverseB])
);

logic [7:0] last_alu_result, before_last_alu_result;
logic last_alu_carry, last_alu_overflow;

status_register reg_stat(
    .data_in(data_bus),
    .clock(clock),
    .alu_carry(alu.carry_out),
    .alu_overflow(alu.overflow_out),

    .update_c(decoder.ctrl_signals[control_signals::StatUpdateC]),
    .update_z(decoder.ctrl_signals[control_signals::StatUpdateZ]),
    .update_i(decoder.ctrl_signals[control_signals::StatUpdateI]),
    .update_d(decoder.ctrl_signals[control_signals::StatUpdateD]),
    .output_b(decoder.ctrl_signals[control_signals::StatOutputB]),
    .update_v(decoder.ctrl_signals[control_signals::StatUpdateV]),
    .update_n(decoder.ctrl_signals[control_signals::StatUpdateN]),

    .use_alu_flags(decoder.ctrl_signals[control_signals::StatUseAlu]),
    .calculate_zero(decoder.ctrl_signals[control_signals::StatCalcZero]),

    .ready(ready),
    .set_overflow(set_overflow)
);
logic [7:0] last_status;

decoder#(.CPU_VARIANT(CPU_VARIANT)) decoder(
    .clock(clock),
    .reset(reset),
    .ready(ready),
    .status(last_status),
    .alu_carry_out(last_alu_carry),
    .interrupt_request(interrupt),
    .nonmaskable_interrupt(nmi),
    .memory_in(data_in)
);

assign incompatible = decoder.incompatible;
assign write = decoder.write;
assign memory_lock = decoder.memory_lock;
assign vector_pull = decoder.vector_pull;
assign sync = decoder.sync;

always_comb begin
    case(decoder.addr_bus_low_src)
        bus_sources::AddrBusLowSrc_PC: address_bus_low = reg_pcl.data_out;
        bus_sources::AddrBusLowSrc_Mem: address_bus_low = data_in;
        bus_sources::AddrBusLowSrc_DL: address_bus_low = reg_dl.data_out;
        bus_sources::AddrBusLowSrc_OL: address_bus_low = reg_oll.data_out;
        bus_sources::AddrBusLowSrc_SP: address_bus_low = reg_sp.data_out;
        bus_sources::AddrBusLowSrc_ALU: address_bus_low = last_alu_result;
        bus_sources::AddrBusLowSrc_ALU_Latched: address_bus_low = before_last_alu_result;
        bus_sources::AddrBusLowSrc_F8: address_bus_low = 8'hf8;
        bus_sources::AddrBusLowSrc_F9: address_bus_low = 8'hf9;
        bus_sources::AddrBusLowSrc_FA: address_bus_low = 8'hfa;
        bus_sources::AddrBusLowSrc_FB: address_bus_low = 8'hfb;
        bus_sources::AddrBusLowSrc_FC: address_bus_low = 8'hfc;
        bus_sources::AddrBusLowSrc_FD: address_bus_low = 8'hfd;
        bus_sources::AddrBusLowSrc_FE: address_bus_low = 8'hfe;
        bus_sources::AddrBusLowSrc_FF: address_bus_low = 8'hff;
        default: address_bus_low = 8'hXX;
    endcase
end

always_comb begin
    case(decoder.addr_bus_high_src)
        bus_sources::AddrBusHighSrc_Zero: address_bus_high = 8'h00;
        bus_sources::AddrBusHighSrc_One: address_bus_high = 8'h01;
        bus_sources::AddrBusHighSrc_PC: address_bus_high = reg_pch.data_out;
        bus_sources::AddrBusHighSrc_Mem: address_bus_high = data_in;
        bus_sources::AddrBusHighSrc_ALU: address_bus_high = last_alu_result;
        bus_sources::AddrBusHighSrc_OL: address_bus_high = reg_olh.data_out;
        bus_sources::AddrBusHighSrc_FF: address_bus_high = 8'hff;
        default: address_bus_high = 8'hXX;
    endcase
end

always_comb begin
    case(decoder.special_bus_src)
        bus_sources::SpecialBusSrc_RegA: special_bus = reg_a.data_out;
        bus_sources::SpecialBusSrc_RegX: special_bus = reg_x.data_out;
        bus_sources::SpecialBusSrc_RegY: special_bus = reg_y.data_out;
        bus_sources::SpecialBusSrc_RegSP: special_bus = reg_sp.data_out;
        bus_sources::SpecialBusSrc_Mem: special_bus = data_in;
        default: special_bus = 8'hXX;
    endcase
end

always_comb begin
    case(decoder.alu_a_src)
        bus_sources::AluASrc_RegA: alu_a_input = reg_a.data_out;
        bus_sources::AluASrc_RegX: alu_a_input = reg_x.data_out;
        bus_sources::AluASrc_RegY: alu_a_input = reg_y.data_out;
        bus_sources::AluASrc_RegSp: alu_a_input = reg_sp.data_out;
        bus_sources::AluASrc_PcLow: alu_a_input = reg_pcl.data_out;
        bus_sources::AluASrc_PcHigh: alu_a_input = reg_pch.data_out;
        bus_sources::AluASrc_Mem: alu_a_input = data_in;
        bus_sources::AluASrc_ALU: alu_a_input = last_alu_result;
        default: alu_a_input = 8'hXX;
    endcase

    case(decoder.alu_b_src)
        bus_sources::AluBSrc_Zero: alu_b_input = 8'h00;
        bus_sources::AluBSrc_Mem: alu_b_input = data_in;
        default: alu_b_input = 8'hXX;
    endcase
    if( CPU_VARIANT>=2 ) begin
        case(decoder.alu_b_src)
            bus_sources::AluBSrc_RegA: alu_b_input = reg_a.data_out;
            bus_sources::AluBSrc_Bit0: alu_b_input = 8'b0000_0001;
            bus_sources::AluBSrc_Bit1: alu_b_input = 8'b0000_0010;
            bus_sources::AluBSrc_Bit2: alu_b_input = 8'b0000_0100;
            bus_sources::AluBSrc_Bit3: alu_b_input = 8'b0000_1000;
            bus_sources::AluBSrc_Bit4: alu_b_input = 8'b0001_0000;
            bus_sources::AluBSrc_Bit5: alu_b_input = 8'b0010_0000;
            bus_sources::AluBSrc_Bit6: alu_b_input = 8'b0100_0000;
            bus_sources::AluBSrc_Bit7: alu_b_input = 8'b1000_0000;
        endcase
    end
end

always_comb begin
    case(decoder.data_bus_src)
        bus_sources::DataBusSrc_Zero: data_bus = 8'h00;
        bus_sources::DataBusSrc_Ones: data_bus = 8'hff;
        bus_sources::DataBusSrc_Mem: data_bus = data_in;
        bus_sources::DataBusSrc_Alu: data_bus = alu.result;
        bus_sources::DataBusSrc_AluLast: data_bus = last_alu_result;
        bus_sources::DataBusSrc_Special: data_bus = special_bus;
        bus_sources::DataBusSrc_PcLow: data_bus = reg_pcl.data_out;
        bus_sources::DataBusSrc_PcHigh: data_bus = reg_pch.data_out;

        default: data_bus = 8'hXX;
    endcase
end

always_comb begin
    case(decoder.pc_next_src)
        bus_sources::PcNextSrc_Pc: pc_next = {reg_pch.data_out, reg_pcl.data_out} + 1;
        bus_sources::PcNextSrc_Bus: pc_next = {address_bus_high, address_bus_low} + 1;
        default: pc_next = 16'hXX;
    endcase
end

always_comb begin
    case(decoder.pcl_bus_src)
        bus_sources::PcLowSrc_Mem: pcl_in = data_in;
        bus_sources::PcLowSrc_ALU: pcl_in = last_alu_result;
        bus_sources::PcLowSrc_Incrementor: pcl_in = pc_next[7:0];
        default: pcl_in = 8'hXX;
    endcase

    case(decoder.pch_bus_src)
        bus_sources::PcHighSrc_Mem: pch_in = data_in;
        bus_sources::PcHighSrc_Incrementor: pch_in = pc_next[15:8];
        default: pch_in = 8'hXX;
    endcase
end

always_comb begin
    case(decoder.data_out_src)
        bus_sources::DataOutSrc_Status: data_out = reg_stat.data_out;
        bus_sources::DataOutSrc_DataBus: data_out = data_bus;
        bus_sources::DataOutSrc_Alu: data_out = last_alu_result;
        default: data_out = 8'hXX;
    endcase
end

always_ff@(posedge clock) begin
    if( ready ) begin
        before_last_alu_result <= last_alu_result;
        last_alu_result <= alu.result;
        last_alu_carry <= alu.carry_out;
        last_alu_overflow <= alu.overflow_out;
        last_status <= reg_stat.data_out;
    end
end

endmodule
