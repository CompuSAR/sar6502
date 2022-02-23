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

`include "bus_sources.vh"
`include "control_signals.vh"

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

logic [ctrl_signal_names.num()-1:0] ctrl_signals;

logic [7:0]data_bus;
logic [15:0]address_bus;

register register_a(.latch(ctrl_signals[LOAD_A]));
register register_x(.latch(ctrl_signals[LOAD_X]));
register register_y(.latch(ctrl_signals[LOAD_Y]));
register register_stack();

endmodule
