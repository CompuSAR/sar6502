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

module sar6502#(parameter CPU_VARIANT = 0)
(
    input clock,

    output [15:0] address,
    output [7:0] data_out,
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
assign address = {address_bus_high, address_bus_low};
logic [control_signals::ctrl_signals_last:0] control_signals;

register        reg_a(.clock(clock), .data_in(data_bus), .ready(ready)),
                reg_x(.clock(clock), .data_in(data_bus), .ready(ready)),
                reg_y(.clock(clock), .data_in(data_bus), .ready(ready)),
                reg_s(.clock(clock), .data_in(data_bus), .ready(ready));
program_counter reg_pc(.clock(clock), .ready(ready));

decoder decoder(
    .clock(clock),
    .reset(reset),
    .memory_in(data_in)
);

assign incompatible = decoder.incompatible;
assign write = decoder.write;
assign memory_lock = decoder.memory_lock;
assign vector_pull = decoder.vector_pull;
assign sync = decoder.sync;

always_comb begin
    case(decoder.addr_bus_low_src)
        bus_sources::AddrBusLowSrc_PC: address_bus_low = reg_pc.address_out[7:0];
        bus_sources::AddrBusLowSrc_FC: address_bus_low = 8'hfc;
        default: address_bus_low = 8'hXX;
    endcase

    case(decoder.addr_bus_high_src)
        bus_sources::AddrBusHighSrc_PC: address_bus_high = reg_pc.address_out[15:8];
        bus_sources::AddrBusHighSrc_FF: address_bus_high = 8'hff;
        default: address_bus_high = 8'hXX;
    endcase
end

endmodule
