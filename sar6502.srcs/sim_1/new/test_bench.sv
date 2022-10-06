`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Design Name:
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////
// Company:  Some Assembly Required
// Engineer: Shachar Shemesh
//
// Create Date: 02/24/2022 05:26:21 AM
// Design Name: sar6502
// Module Name: test_bench
// Project Name: CompuSAR
// Target Devices:
// Tool Versions:
// Description: CPU test bench
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

module test_bench(
    );

// Timing parameters for 14Mhz operation
localparam tACC = 30;       // Access time (min)
localparam tAH = 10;        // Address hold time (min)
localparam tADS = 30;       // Address setup time (max)
localparam tBVD = 25;       // BE to valid data (max)
localparam tPWH = 35;       // Clock pulse width high (min)
localparam tPWL = 35;       // Clock pulse width low (min)
localparam tCYC = tPWH+tPWL;       // Cycle time (min)
localparam tF = 5;          // Fall time (max)
localparam tR = 5;          // Rise time (max)
localparam tPCH = 10;       // Processor control hold time (min)
localparam tPCS = 10;       // Processor control setup time (min)
localparam tDHR = 10;       // Read data hold time (min)
localparam tDSR = 10;       // Read data setup time (min)
localparam tMDS = 25;       // Write data delay time (max)
localparam tDHW = 10;       // Write data hold time (min)

logic clock;
logic [7:0] data_in;
logic [15:0] address_latched;

typedef enum { SigReset, SigIrq, SigNmi, SigSo, SigReady, Sig_NumElements } signal_types;
logic signals[Sig_NumElements-1:0];

sar6502#(.CPU_VARIANT(2)) cpu(
    .clock(clock),
    .data_in(data_in),
    .ready( !signals[SigReady] ),

    .reset( signals[SigReset] ),
    .interrupt( signals[SigIrq] ),
    .nmi( signals[SigNmi] ),
    .set_overflow( signals[SigSo] )
);

logic [7:0]memory[65536];
logic [35:0]test_plan[30000];

logic [7:0]prev_data_out;
logic [15:0]prev_address;
logic prev_write, prev_memory_lock, prev_vector_pull, prev_sync, prev_incompatible;

struct {
    int delay;
    int count;
} pending_signals[Sig_NumElements-1:0];

always_ff@(posedge clock) begin
    if( cpu.write==1 ) begin
        memory[cpu.address] <= cpu.data_out;
        data_in <= 8'bX;
    end else begin
        data_in <= memory[cpu.address];
    end

    prev_data_out <= cpu.data_out;
    prev_address <= cpu.address;
    prev_write <= cpu.write;
    prev_memory_lock <= cpu.memory_lock;
    prev_vector_pull <= cpu.vector_pull;
    prev_sync <= cpu.sync;
    prev_incompatible <= cpu.incompatible;
end

initial begin
    // Clock and memory read handling
    clock = 0;
    $readmemh("test_program.mem", memory);
    $readmemh("test_plan.mem", test_plan);

    foreach( signals[i] ) begin
        pending_signals[i].delay = 0;
        pending_signals[i].count = 0;
        signals[i] = 0;
    end

    pending_signals[SigReset].delay = 2;
    pending_signals[SigReset].count = 5;

    forever begin
        #tPWL clock = 1;
        clock_high();
        #tPWH clock = 0;
    end
end

int cycle_num = 0;

task clock_high();
    // Signal control
    foreach( signals[i] ) begin
        if( signals[i]==1 || pending_signals[i].delay>0 ) begin
            if( pending_signals[i].delay>0 ) begin
                pending_signals[i].delay--;

                if( pending_signals[i].delay==0 )
                    signals[i] = 1;
            end else begin
                pending_signals[i].count--;

                if( pending_signals[i].count==0 )
                    signals[i] = 0;
            end
        end
    end
endtask

always_ff@(posedge clock)
    verify();

task verify();
    automatic logic [35:0]plan_line;
begin
    // Verification
    if( cycle_num==0 ) begin
        if( prev_address !== 16'hfffc )
            // Waiting to begin
            return;
        else
            cycle_num=1;
    end

    plan_line = test_plan[cycle_num];
    casex( plan_line[35:32] )
        4'h1: verify_cycle(plan_line);
        default: begin
            $display("Unknown instruction type at cycle %d", cycle_num);
            $finish();
        end
    endcase

    if( prev_address[15:8]===8'h02 && prev_write===1'b1 )
        perform_io();

    cycle_num++;
end
endtask

task verify_cycle( input logic [35:0]plan_line );
begin
    if( !prev_incompatible || prev_write==1 || plan_line[0]==0 ) begin
        assert_state( prev_address, plan_line[31:16], "Address bus" );
    end else
        $display("Known incompatibility cycle %d. Not comparing address %x to desired %x", cycle_num, prev_address, plan_line[31:16]);
    assert_state( prev_write, !plan_line[0], "Read/write" );
    assert_state( prev_sync, plan_line[1], "Sync" );
    if( plan_line[2]==1 ) // Due to bug in wd65c02, allow our ML to be active while theirs isn't.
        assert_state( prev_memory_lock, plan_line[2], "Memory lock" );
    assert_state( prev_vector_pull, plan_line[3], "Vector pull" );

    if( !prev_write ) begin
        // Read
        if( !prev_incompatible )
            assert_state( data_in, plan_line[15:8], "Data in" );
    end else begin
        // Write
        memory[prev_address] = prev_data_out;
        assert_state( prev_data_out, plan_line[15:8], "Data out" );
    end
end
endtask

task assert_state( input logic [15:0]actual, input logic [15:0]expected, input string name );
    if( actual === expected )
        return;

    $display("Verification failed on cycle %d time %t pin %s: expected %x, received %x on address %04x",
        cycle_num, $time, name, expected, actual, prev_address);
    $finish();
endtask

task perform_io();
    $display("Cycle %d: IO writing %x to %x", cycle_num, prev_data_out, prev_address);

    casex( prev_address[7:0] )
        8'h00: begin
            $display("Test finished successfully");
            $finish();
        end
        8'h80: pending_signals[SigReady].count = prev_data_out;
        8'h81: pending_signals[SigReady].delay = prev_data_out;
        8'h82: pending_signals[SigSo].count = prev_data_out;
        8'h83: pending_signals[SigSo].delay = prev_data_out;
        8'hfa: pending_signals[SigNmi].count = prev_data_out;
        8'hfb: pending_signals[SigNmi].delay = prev_data_out;
        8'hfc: pending_signals[SigReset].count = prev_data_out;
        8'hfd: pending_signals[SigReset].delay = prev_data_out;
        8'hfe: pending_signals[SigIrq].count = prev_data_out;
        8'hff: pending_signals[SigIrq].delay = prev_data_out;
    endcase
endtask

endmodule
