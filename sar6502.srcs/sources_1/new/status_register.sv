`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:  Some Assembly Required
// Engineer: Shachar Shemesh
// 
// Create Date: 02/26/2022 07:38:22 PM
// Design Name: sar6502
// Module Name: status_register
// Project Name: CompuSAR
// Target Devices: 
// Tool Versions: 
// Description: 6502 flags register
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


module status_register(
        input [7:0] data_in,
        output logic[7:0]data_out,
        input clock,
        input alu_carry,
        input alu_overflow,

        input update_c,
        input update_z,
        input update_i,
        input update_d,
        input output_b,
        input update_v,
        input update_n,

        input use_alu_flags,
        input calculate_zero,

        input ready,
        input set_overflow
    );

logic negative, overflow, decimal, irq_mask, zero, carry;
logic prev_so = 0;

assign data_out = { negative, overflow, 1'b1, output_b, decimal, irq_mask, zero, carry };
logic[7:0] stored_flags;

always_comb begin
    negative =
        update_n ?
        data_in[control_signals::FlagsNegative] :
        stored_flags[control_signals::FlagsNegative];

    overflow =
        update_v ?
        (use_alu_flags ? alu_overflow : data_in[control_signals::FlagsOverflow]) :
        stored_flags[control_signals::FlagsOverflow];
    if( prev_so==0 && set_overflow==1 )
        overflow = 1'b1;

    decimal =
        update_d ?
        data_in[control_signals::FlagsDecimal] :
        stored_flags[control_signals::FlagsDecimal];

    irq_mask =
        update_i ?
        data_in[control_signals::FlagsIrqMask] :
        stored_flags[control_signals::FlagsIrqMask];

    zero =
        update_z ?
        (calculate_zero ? (data_in==0 ? 1'b1 : 1'b0) : data_in[control_signals::FlagsZero]) :
        stored_flags[control_signals::FlagsZero];

    carry =
        update_c ?
        ( use_alu_flags ? alu_carry : data_in[control_signals::FlagsCarry] ) :
        stored_flags[control_signals::FlagsCarry];
end

always_ff@(posedge clock)
begin
    if( ready ) begin
        stored_flags <= data_out;
        prev_so <= set_overflow;
    end
end

endmodule
