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
    output sync
    );

// Input latches
logic [7:0] data_in_l;
logic RESET_L;
logic ready_l;
logic IRQ_L;
logic NMI_L;
logic SO_L, previous_SO;

always_ff@(negedge phi2) begin
    data_in_l <= data_in;
    RESET_L <= RES;
    ready_l <= rdy;
    IRQ_L <= IRQ;
    NMI_L <= NMI;
    SO_L <= SO;
    previous_SO <= SO_L;
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

logic [7:0]internal_bus;
logic [7:0]internal_bus_inputs[bus_sources::InternalBusSourceCtlLast:0];
bus_sources::InternalBusSourceCtl internal_bus_source;

assign internal_bus = internal_bus_inputs[internal_bus_source];

// Registers
register register_a(.data_in(data_bus), .clock(phi2), .latch(ctrl_signals[control_signals::LOAD_A]),
    .data_out(data_bus_inputs[bus_sources::DataBusSrc_A]));
register register_x(.data_in(data_bus), .clock(phi2), .latch(ctrl_signals[control_signals::LOAD_X]),
    .data_out(data_bus_inputs[bus_sources::DataBusSrc_X]));
register register_y(.data_in(data_bus), .clock(phi2), .latch(ctrl_signals[control_signals::LOAD_Y]),
    .data_out(data_bus_inputs[bus_sources::DataBusSrc_Y]));
register register_stack(.data_in(data_bus), .clock(phi2), .latch(ctrl_signals[control_signals::LOAD_SP]),
    .data_out(data_bus_inputs[bus_sources::DataBusSrc_SP]));

register data_latch_low( .data_in(internal_bus), .clock(phi2), .latch(ctrl_signals[control_signals::LOAD_DataLow]),
    .data_out(internal_bus_inputs[bus_sources::InternalBusSrc_DataLatchLow]));
register data_latch_high( .data_in(internal_bus), .clock(phi2), .latch(ctrl_signals[control_signals::LOAD_DataHigh]),
    .data_out(internal_bus_inputs[bus_sources::InternalBusSrc_DataLatchHigh]));

wire [15:0]pc_value;
program_counter register_pc(
    .address_in(address), .ctl_advance(ctrl_signals[control_signals::PC_ADVANCE]),
    .ctl_load(ctrl_signals[control_signals::PC_LOAD]), .clock(phi2), .RESET(RESET_L),
    .address_out(pc_value));

// Control
logic [control_signals::ctrl_signals_last:0] ctrl_signals;

decoder decoder(
    .memory_in(data_in_l),
    .clock(phi2),
    .RESET(RESET_L),

    .address_bus_low_source( address_bus_low_source ),
    .address_bus_high_source( address_bus_high_source ),
    .data_bus_source( data_bus_source ),
    .internal_bus_source( internal_bus_source ),
    .ctrl_signals( ctrl_signals ),

    .rW( rW ),
    .sync( sync ),
    .ML( ML ),
    .VP( VP )
);

assign data_bus_inputs[bus_sources::DataBusSrc_Zero] = 8'b0;
assign data_bus_inputs[bus_sources::DataBusSrc_Mem] = data_in_l;

assign internal_bus_inputs[bus_sources::InternalBusSrc_Mem] = data_in_l;
assign internal_bus_inputs[bus_sources::InternalBusSrc_PcLow] = pc_value[7:0];
assign internal_bus_inputs[bus_sources::InternalBusSrc_PcHigh] = pc_value[15:8];

assign address_bus_low_inputs[bus_sources::AddrBusLowSrc_SP] = data_bus_inputs[bus_sources::DataBusSrc_SP];
assign address_bus_low_inputs[bus_sources::AddrBusLowSrc_PC] = pc_value[7:0];

assign address_bus_high_inputs[bus_sources::AddrBusHighSrc_Zero] = 8'b0;
assign address_bus_high_inputs[bus_sources::AddrBusHighSrc_One] = 8'b1;
assign address_bus_high_inputs[bus_sources::AddrBusHighSrc_PC] = pc_value[15:8];

endmodule
