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

module sar6502(
    input phi2,
    input [7:0] data_in,
    input RES,
    input rdy,
    input IRQ,
    input NMI,
    input SO,
    output [15:0] address,
    output [7:0] data_out,
    output rW,
    output VP,
    output ML,
    output sync,

    output debug1,
    output debug2,
    output debug3,
    output debug4,
    output debug5,
    output debug6
    );

// Input latches
logic [7:0] data_in_l;
logic RESET_L;
logic ready_l;
logic IRQ_L;
logic NMI_L;
logic SO_L, previous_SO;
logic [15:0]pc_value, prev_pc_value;
logic [7:0]alu_result, alu_result_latched;

always_ff@(negedge phi2) begin
    data_in_l <= data_in;
    RESET_L <= RES;
    ready_l <= rdy;
    IRQ_L <= IRQ;
    NMI_L <= NMI;
    SO_L <= SO;
    previous_SO <= SO_L;
    prev_pc_value <= pc_value;
    alu_result_latched <= alu_result;
end

// Buses
logic [7:0]data_bus;
logic [7:0]data_bus_inputs[bus_sources::DataBusSourceCtlLast:0];
bus_sources::DataBusSourceCtl data_bus_source;
assign data_bus = data_bus_inputs[data_bus_source];
assign data_out = data_bus;

logic [7:0]address_bus_low;
logic [7:0]address_bus_low_inputs[bus_sources::AddressBusLowSourceCtlLast:0];
bus_sources::AddressBusLowSourceCtl address_bus_low_source;

logic [7:0]address_bus_high;
logic [7:0]address_bus_high_inputs[bus_sources::AddressBusHighSourceCtlLast:0];
bus_sources::AddressBusHighSourceCtl address_bus_high_source;

assign address = { address_bus_high_inputs[address_bus_high_source], address_bus_low_inputs[address_bus_low_source] };

logic [7:0]alu_a_inputs[bus_sources::AluASourceCtlLast:0];
bus_sources::AluASourceCtl alu_a_source;
logic [7:0]alu_b_inputs[bus_sources::AluBSourceCtlLast:0];
bus_sources::AluBSourceCtl alu_b_source;

logic alu_carry_inputs[bus_sources::AluCarrySourceCtlLast:0];
bus_sources::AluCarrySourceCtl alu_carry_source;
control_signals::alu_control alu_control;
logic alu_carry, alu_overflow;

alu alu( .a(alu_a_inputs[alu_a_source]), .b(alu_b_inputs[alu_b_source]),
    .carry_in(alu_carry_inputs[alu_carry_source]),
    .inverse_b(ctrl_signals[control_signals::AluBInverse]),
    .control(alu_control), .result(alu_result), .carry_out(alu_carry), .overflow_out(alu_overflow) );

// Registers
register register_a(.data_in(data_bus), .clock(phi2), .latch(ctrl_signals[control_signals::LOAD_A]),
    .data_out(data_bus_inputs[bus_sources::DataBusSrc_A]));
register register_x(.data_in(data_bus), .clock(phi2), .latch(ctrl_signals[control_signals::LOAD_X]),
    .data_out(data_bus_inputs[bus_sources::DataBusSrc_X]));
register register_y(.data_in(data_bus), .clock(phi2), .latch(ctrl_signals[control_signals::LOAD_Y]),
    .data_out(data_bus_inputs[bus_sources::DataBusSrc_Y]));

logic [7:0]stack_pointer_inputs[bus_sources::StackPointerSourceCtlLast : 0];
bus_sources::StackPointerSourceCtl stack_pointer_source;
register register_stack(
    .data_in(stack_pointer_inputs[stack_pointer_source]),
    .clock(phi2),
    .latch(ctrl_signals[control_signals::LOAD_SP]),
    .data_out(data_bus_inputs[bus_sources::DataBusSrc_SP]));

logic [7:0]status_value;
status_register restier_p(.data_in(data_bus), .data_out(status_value), .clock(phi2),
    .alu_carry(alu_carry),
    .alu_overflow(alu_overflow),
    .use_alu_flags(ctrl_signals[control_signals::UseAluFlags]), .calculate_zero(ctrl_signals[control_signals::CalculateFlagZ]),
    .update_c(ctrl_signals[control_signals::UpdateFlagC]),
    .update_z(ctrl_signals[control_signals::UpdateFlagZ]),
    .update_i(ctrl_signals[control_signals::UpdateFlagI]),
    .update_d(ctrl_signals[control_signals::UpdateFlagD]),
    .output_b(ctrl_signals[control_signals::OutputFlagB]),
    .update_v(ctrl_signals[control_signals::UpdateFlagV]),
    .update_n(ctrl_signals[control_signals::UpdateFlagN])
);

logic [15:0]data_latch_value;
bus_sources::DataLatchLowSourceCtl data_latch_low_source;
logic [7:0]data_latch_low_inputs[bus_sources::DataLatchLowSourceCtlLast : 0];
bus_sources::DataLatchHighSourceCtl data_latch_high_source;
logic [7:0]data_latch_high_inputs[bus_sources::DataLatchHighSourceCtlLast : 0];

register data_latch_low( .data_in(data_latch_low_inputs[data_latch_low_source]),
    .clock(phi2), .latch(ctrl_signals[control_signals::LOAD_DataLow]),
    .data_out(data_latch_value[7:0]));
register data_latch_high( .data_in(data_latch_high_inputs[data_latch_high_source]),
    .clock(phi2), .latch(ctrl_signals[control_signals::LOAD_DataHigh]),
    .data_out(data_latch_value[15:8]));

bus_sources::PcLowSourceCtl pc_low_source;
logic [7:0]pc_low_inputs[bus_sources::PcLowSourceCtlLast : 0];

bus_sources::PcHighSourceCtl pc_high_source;
logic [7:0]pc_high_inputs[bus_sources::PcHighSourceCtlLast : 0];

program_counter register_pc(
    .address_in({pc_high_inputs[pc_high_source], pc_low_inputs[pc_low_source]}),
    .ctl_advance(ctrl_signals[control_signals::PC_ADVANCE]),
    .ctl_load(ctrl_signals[control_signals::PC_LOAD]), .clock(phi2),
    .address_out(pc_value));

// Control
logic [control_signals::ctrl_signals_last:0] ctrl_signals;

decoder decoder(
    .memory_in(data_in_l),
    .status( status_value ),
    .alu_carry( alu_carry ),
    .clock(phi2),
    .RESET(RESET_L),

    .address_bus_low_source( address_bus_low_source ),
    .address_bus_high_source( address_bus_high_source ),
    .data_bus_source( data_bus_source ),
    .pc_low_source( pc_low_source ),
    .pc_high_source( pc_high_source ),
    .data_latch_low_source( data_latch_low_source ),
    .data_latch_high_source( data_latch_high_source ),
    .stack_pointer_source( stack_pointer_source ),
    .alu_op(alu_control),
    .alu_a_source(alu_a_source),
    .alu_b_source(alu_b_source),
    .alu_carry_source(alu_carry_source),
    .ctrl_signals( ctrl_signals ),

    .rW( rW ),
    .sync( sync ),
    .ML( ML ),
    .VP( VP )
);

// Assign the rest of the bus inputs
assign data_bus_inputs[bus_sources::DataBusSrc_Zero] = 8'b0;
assign data_bus_inputs[bus_sources::DataBusSrc_Ones] = 8'hff;
assign data_bus_inputs[bus_sources::DataBusSrc_Status] = status_value;
assign data_bus_inputs[bus_sources::DataBusSrc_Alu] = alu_result;
assign data_bus_inputs[bus_sources::DataBusSrc_Alu_Latched] = alu_result_latched;
assign data_bus_inputs[bus_sources::DataBusSrc_Pc_Low] = pc_value[7:0];
assign data_bus_inputs[bus_sources::DataBusSrc_Pc_High] = pc_value[15:8];
assign data_bus_inputs[bus_sources::DataBusSrc_Dl_Low] = data_latch_value[7:0];
assign data_bus_inputs[bus_sources::DataBusSrc_Mem] = data_in_l;
assign data_bus_inputs[bus_sources::DataBusSrc_Mem_Unlatched] = data_in;

assign address_bus_low_inputs[bus_sources::AddrBusLowSrc_Mem] = data_in_l;
assign address_bus_low_inputs[bus_sources::AddrBusLowSrc_Alu] = alu_result_latched;
assign address_bus_low_inputs[bus_sources::AddrBusLowSrc_SP] = data_bus_inputs[bus_sources::DataBusSrc_SP];
assign address_bus_low_inputs[bus_sources::AddrBusLowSrc_PC] = pc_value[7:0];
assign address_bus_low_inputs[bus_sources::AddrBusLowSrc_DataLatch] = data_latch_value[7:0];
assign address_bus_low_inputs[bus_sources::AddrBusLowSrc_DataLatch_High] = data_latch_value[15:8];

assign address_bus_high_inputs[bus_sources::AddrBusHighSrc_Zero] = 8'b0;
assign address_bus_high_inputs[bus_sources::AddrBusHighSrc_One] = 8'b1;
assign address_bus_high_inputs[bus_sources::AddrBusHighSrc_Mem] = data_in_l;
assign address_bus_high_inputs[bus_sources::AddrBusHighSrc_Alu] = alu_result_latched;
assign address_bus_high_inputs[bus_sources::AddrBusHighSrc_Alu_Unlatched] = alu_result;
assign address_bus_high_inputs[bus_sources::AddrBusHighSrc_PC] = pc_value[15:8];
assign address_bus_high_inputs[bus_sources::AddrBusHighSrc_DataLatch] = data_latch_value[15:8];

assign pc_low_inputs[bus_sources::PcLowSource_CurrentValue] = prev_pc_value[7:0];
assign pc_low_inputs[bus_sources::PcLowSource_Mem] = data_in_l;
assign pc_low_inputs[bus_sources::PcLowSource_Dl] = data_latch_value[7:0];

assign pc_high_inputs[bus_sources::PcHighSource_CurrentValue] = prev_pc_value[15:8];
assign pc_high_inputs[bus_sources::PcHighSource_Mem] = data_in_l;
assign pc_high_inputs[bus_sources::PcHighSource_Dl] = data_latch_value[15:8];

assign data_latch_low_inputs[bus_sources::DataLatchLowSource_Mem] = data_in_l;
assign data_latch_low_inputs[bus_sources::DataLatchLowSource_Alu] = alu_result;
assign data_latch_low_inputs[bus_sources::DataLatchLowSource_Alu_Latched] = alu_result_latched;
assign data_latch_low_inputs[bus_sources::DataLatchLowSource_PC] = pc_value[7:0];
assign data_latch_low_inputs[bus_sources::DataLatchLowSource_FA] = 8'hfa;
assign data_latch_low_inputs[bus_sources::DataLatchLowSource_FC] = 8'hfc;
assign data_latch_low_inputs[bus_sources::DataLatchLowSource_FE] = 8'hfe;

assign data_latch_high_inputs[bus_sources::DataLatchHighSource_Zero] = 8'h00;
assign data_latch_high_inputs[bus_sources::DataLatchHighSource_Mem] = data_in_l;
assign data_latch_high_inputs[bus_sources::DataLatchHighSource_Alu] = alu_result;
assign data_latch_high_inputs[bus_sources::DataLatchHighSource_Alu_Latched] = alu_result_latched;
assign data_latch_high_inputs[bus_sources::DataLatchHighSource_PC] = pc_value[15:8];
assign data_latch_high_inputs[bus_sources::DataLatchHighSource_FF] = 8'hff;

assign stack_pointer_inputs[bus_sources::StackPointerSource_Alu] = alu_result;
assign stack_pointer_inputs[bus_sources::StackPointerSource_DataBus] = data_bus;

assign alu_a_inputs[bus_sources::AluASourceCtl_Zero] = 0;
assign alu_a_inputs[bus_sources::AluASourceCtl_A] = data_bus_inputs[bus_sources::DataBusSrc_A];
assign alu_a_inputs[bus_sources::AluASourceCtl_X] = data_bus_inputs[bus_sources::DataBusSrc_X];
assign alu_a_inputs[bus_sources::AluASourceCtl_Y] = data_bus_inputs[bus_sources::DataBusSrc_Y];
assign alu_a_inputs[bus_sources::AluASourceCtl_DataLatchLow] = data_latch_value[7:0];
assign alu_a_inputs[bus_sources::AluASourceCtl_DataLatchHigh] = data_latch_value[15:8];
assign alu_a_inputs[bus_sources::AluASourceCtl_SP] = data_bus_inputs[bus_sources::DataBusSrc_SP];
assign alu_a_inputs[bus_sources::AluASourceCtl_PC_Low] = pc_value[7:0];
assign alu_a_inputs[bus_sources::AluASourceCtl_PC_High] = pc_value[15:8];
assign alu_a_inputs[bus_sources::AluASourceCtl_Mem] = data_in_l;
assign alu_a_inputs[bus_sources::AluASourceCtl_Alu] = alu_result_latched;

assign alu_b_inputs[bus_sources::AluBSourceCtl_Zero] = 8'b0;
assign alu_b_inputs[bus_sources::AluBSourceCtl_Mem] = data_in_l;

assign alu_carry_inputs[bus_sources::AluCarrySource_Zero] = 0;
assign alu_carry_inputs[bus_sources::AluCarrySource_One] = 1;
assign alu_carry_inputs[bus_sources::AluCarrySource_Carry] =
    data_bus_inputs[bus_sources::DataBusSrc_Status][control_signals::FlagsCarry];

endmodule
