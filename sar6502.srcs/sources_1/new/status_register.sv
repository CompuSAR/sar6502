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
        output [7:0]data_out,
        input clock,
        input alu_carry,

        input update_c,
        input update_z,
        input update_i,
        input update_d,
        input output_b,
        input update_v,
        input update_n,

        input use_alu_carry,
        input calculate_zero
    );

logic negative, overflow, decimal, irq_mask, zero, carry;

assign data_out = { negative, overflow, 1'b1, output_b, decimal, irq_mask, zero, carry };

always_ff@(negedge clock)
begin
    if( update_n )
        negative <= data_in[7];
    if( update_v )
        overflow <= data_in[6];
    if( update_d )
        decimal <= data_in[3];
    if( update_i )
        irq_mask <= data_in[2];
    if( update_z ) begin
        if( calculate_zero )
            zero <= data_in==0 ? 1 : 0;
        else
            zero <= data_in[1];
    end
    if( update_c )
        carry <= use_alu_carry ? alu_carry : data_in[0];
end

endmodule
