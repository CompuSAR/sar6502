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
    output VP,
    output ML,
    output SYNC
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
logic [7:0]data_bus_sources[bus_sources::data_bus_sources_ctl.num()];
bus_sources::data_bus_sources_ctl data_bus_source;
assign data_bus = data_bus_sources[data_bus_source];
assign data_out = data_bus;

logic [15:0]address_bus;
logic [15:0]address_bus_sources[bus_sources::address_bus_sources_ctl.num()];
bus_sources::address_bus_sources_ctl address_bus_source;

assign address_bus = address_bus_sources[address_bus_source];
assign address = address_bus;

// Registers
register register_a(.data_in(data_bus), .clock(phi2), .latch(ctrl_signals[LOAD_A]), .data_out(data_bus_sources[bus_sources::DataBusSrc_A]));
register register_x(.data_in(data_bus), .clock(phi2), .latch(ctrl_signals[LOAD_X]), .data_out(data_bus_sources[bus_sources::DataBusSrc_X]));
register register_y(.data_in(data_bus), .clock(phi2), .latch(ctrl_signals[LOAD_Y]), .data_out(data_bus_sources[bus_sources::DataBusSrc_Y]));
register register_stack(.data_in(data_bus), .clock(phi2), .latch(ctrl_signals[LOAD_SP]), .data_out(data_bus_sources[bus_sources::DataBusSrc_SP]));

always_comb begin
    address_bus_sources[bus_sources::AddrBusSrc_SP] = {8'h01, data_bus_sources[bus_sources::DataBusSrc_SP]};
end

program_counter register_pc(
    .address_in(address_bus), .ctl_advance(ctrl_signals[PC_ADVANCE]), .ctl_load(ctrl_signals[PC_LOAD]), .clock(phi2),
    .address_out(address_bus_sources[bus_sources::AddrBusSrc_PC]));

// Control
logic [control_signals::ctrl_signal_names.num()-1:0] ctrl_signals;

decoder decoder(
    .memory_in(data_in_l),
    .clock(phi2),
    .RESET(RESET_L),

    .address_bus_source( address_bus_source ),
    .data_bus_source( data_bus_source ),
    .ctrl_signals( ctrl_signals )
);

assign data_bus_sources[bus_sources::DataBusSrc_Mem] = data_in_l;

endmodule
