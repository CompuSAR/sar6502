`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Some Assembly Required Youtube channel https://www.youtube.com/channel/UCp5Z7utSI2IHQUsnkPH41bw
// Engineer: Shachar Shemesh
//
// Create Date: 09/19/2021 05:42:25 PM
// Design Name: WD65C02S core almost compatible design
// Module Name: alu
// Project Name: CompuSAR
// Target Devices: Xilinx Spartan-7
// Tool Versions: Vivado 2021.1
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
// License:
//   Copyright (C) 2021.
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

`include "control_signals.vh"

module alu(
        input [7:0]a,
        input [7:0]b,
        input carry_in,
        input [7:0]control,

        output [7:0]result,
        output carry_out,
        output overflow_out
    );

always_comb
begin
    status_out = 0;

    result = 8'bX;
    carry_out = 1'bX;

    case(control)
        AluOp_pass:                     result = a;
        AluOp_add:                      do_plus();
        AluOp_and:                      result = a & b;
        AluOp_or:                       result = a | b;
        AluOp_xor:                      result = a ^ b;
        AluOp_shift_left:               { carry_out, result } = { b, carry_in };
        AluOp_shift_right_logical:      { result, carry_out } = { carry_in, b };
        AluOp_shift_right_arithmetic:   { result, carry_out } = { b[7], b };
    endcase

    overflow_out = result[6];
end

logic [8:0]intermediate_result;

task do_plus();
begin
    intermediate_result = a+b+carry_in;
    result = intermediate_result[7:0];

    carry_out = intermediate_result[8];

    if( a[7]==b[7] && a[7]!=result[7] )
        // Adding same sign integer resulted in opposite sign integer: must be an overflow
        overflow_out = 1;
end
endtask

endmodule
