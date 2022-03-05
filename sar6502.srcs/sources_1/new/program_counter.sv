`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:  Some Assembly Required
// Engineer: Shachar Shemesh
// 
// Create Date: 02/23/2022 09:12:28 PM
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

module program_counter(
        input [15:0] address_in,
        input ctl_advance,
        input ctl_load,
        input clock,

        output logic [15:0] address_out
    );

logic [15:0] address_stored;

logic [15:0] intermediate1;

always_comb begin
    if( ctl_load )
        intermediate1 = address_in;
    else
        intermediate1 = address_stored;

    if( ctl_advance )
        address_out = intermediate1 + 1;
    else
        address_out = intermediate1;
end

always_ff@(negedge clock) begin
    address_stored <= address_out;
end

endmodule
