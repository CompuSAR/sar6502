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
logic [15:0] address_bus;
logic [7:0] data_in, data_out;

logic read_Write, vector_pull, memory_lock, sync, incompatible;

typedef enum { SigReset, SigIrq, SigNmi, SigSo, SigReady, Sig_NumElements } signal_types;
logic signals[Sig_NumElements-1:0];

sar6502#(.CPU_VARIANT(2)) cpu(
    .phi2(clock),
    .data_in(data_in),
    .RES( !signals[SigReset] ),
    .rdy( !signals[SigReady] ),
    .IRQ( !signals[SigIrq] ),
    .NMI( !signals[SigNmi] ),
    .SO( !signals[SigSo] ),

    .address(address_bus),
    .data_out(data_out),
    .rW(read_Write),
    .VP(vector_pull),
    .ML(memory_lock),
    .sync(sync),
    .incompatible(incompatible)
);

logic [7:0]memory[65536];
logic [35:0]test_plan[30000];

struct {
    int delay;
    int count;
} pending_signals[Sig_NumElements-1:0];

logic [15:0]address_latched;

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
        #tDHR data_in = 8'bX;
        #(tPWL-tDHR) clock = 1;
        address_latched = address_bus;
        clock_high();
        #(tPWH-tDSR) data_in=memory[address_latched];
        #tDSR
        clock_low();
        clock = 0;
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

task clock_low();
    automatic logic [35:0]plan_line;
begin
    // Verification
    if( cycle_num==0 ) begin
        if( address_bus !== 16'hfffc )
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

    if( address_bus[15:8]===8'h02 && read_Write===1'b0 )
        perform_io();

    cycle_num++;
end
endtask

task verify_cycle( input logic [35:0]plan_line );
begin
    if( !incompatible || read_Write==0 || plan_line[0]==0 ) begin
        assert_state( address_bus, plan_line[31:16], "Address bus" );
    end else
        $display("Known incompatibility cycle %d. Not comparing address %x to desired %x", cycle_num, address_bus, plan_line[31:16]);
    assert_state( read_Write, plan_line[0], "Read/write" );
    assert_state( sync, plan_line[1], "Sync" );
    if( plan_line[2]==1 ) // Due to bug in wd65c02, allow our ML to be active while theirs isn't.
        assert_state( memory_lock, !plan_line[2], "Memory lock" );
    assert_state( vector_pull, !plan_line[3], "Vector pull" );

    if( read_Write ) begin
        // Read
        if( !incompatible )
            assert_state( data_in, plan_line[15:8], "Data in" );
    end else begin
        // Write
        memory[address_bus] = data_out;
        assert_state( data_out, plan_line[15:8], "Data out" );
    end
end
endtask

task assert_state( input logic [15:0]actual, input logic [15:0]expected, input string name );
    if( actual === expected )
        return;

    $display("Verification failed on cycle %d time %t pin %s: expected %x, received %x on address %04x",
        cycle_num, $time, name, expected, actual, address_bus);
    $finish();
endtask

task perform_io();
    $display("Cycle %d: IO writing %x to %x", cycle_num, data_out, address_bus);

    casex( address_bus[7:0] )
        8'h00: begin
            $display("Test finished successfully");
            $finish();
        end
        8'h80: pending_signals[SigReady].count = data_out;
        8'h81: pending_signals[SigReady].delay = data_out;
        8'h82: pending_signals[SigSo].count = data_out;
        8'h83: pending_signals[SigSo].delay = data_out;
        8'hfa: pending_signals[SigNmi].count = data_out;
        8'hfb: pending_signals[SigNmi].delay = data_out;
        8'hfc: pending_signals[SigReset].count = data_out;
        8'hfd: pending_signals[SigReset].delay = data_out;
        8'hfe: pending_signals[SigIrq].count = data_out;
        8'hff: pending_signals[SigIrq].delay = data_out;
    endcase
endtask

endmodule
