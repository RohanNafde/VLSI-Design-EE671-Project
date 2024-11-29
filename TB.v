// Released under the 3-Clause BSD License:
//
// Copyright 2010-2019 Matthew Hagerty (matthew <at> dnotq <dot> io)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

`timescale 1ns / 1ps

module tb_sdram_simple;

    // Inputs
    reg clk_100m0_i = 0;
    reg reset_i = 0;
    reg refresh_i = 0;
    reg rw_i = 0;
    reg we_i = 0;
    reg [23:0] addr_i = 24'b0;
    reg [15:0] data_i = 16'b0;
    reg ub_i = 0;
    reg lb_i = 0;

    // Bi-directional
    wire [15:0] sdData_io;

    // Outputs
    wire ready_o;
    wire done_o;
    wire [15:0] data_o;
    wire sdCke_o;
    wire sdCe_bo;
    wire sdRas_bo;
    wire sdCas_bo;
    wire sdWe_bo;
    wire [1:0] sdBs_o;
    wire [12:0] sdAddr_o;
    wire sdDqmh_o;
    wire sdDqml_o;

    // Clock period definitions
    parameter clk_100m0_i_period = 10;

    // State type for testbench FSM
    parameter ST_WAIT    = 3'b000,
			  ST_IDLE    = 3'b001,
			  ST_READ    = 3'b010,
			  ST_WRITE   = 3'b011,
			  ST_REFRESH = 3'b100;

    reg [2:0] state_r = ST_WAIT;
    reg [2:0] state_x = ST_WAIT;

    // Instantiate the Device Under Test (DUT)
    sdram_simple dut (
        .clk_100m0_i(clk_100m0_i),
        .reset_i(reset_i),
        .refresh_i(refresh_i),
        .rw_i(rw_i),
        .we_i(we_i),
        .addr_i(addr_i),
        .data_i(data_i),
        .ub_i(ub_i),
        .lb_i(lb_i),
        .ready_o(ready_o),
        .done_o(done_o),
        .data_o(data_o),
        .sdCke_o(sdCke_o),
        .sdCe_bo(sdCe_bo),
        .sdRas_bo(sdRas_bo),
        .sdCas_bo(sdCas_bo),
        .sdWe_bo(sdWe_bo),
        .sdBs_o(sdBs_o),
        .sdAddr_o(sdAddr_o),
        .sdData_io(sdData_io),
        .sdDqmh_o(sdDqmh_o),
        .sdDqml_o(sdDqml_o)
    );

    // Clock generation
    always begin
        # (clk_100m0_i_period / 2) clk_100m0_i = ~clk_100m0_i;
    end

    // FSM process
    always @(posedge clk_100m0_i) begin
        state_r <= state_x;
    end

    always @(state_r, ready_o, done_o) begin
        // Default values
        state_x = state_r;
        rw_i = 0;
        we_i = 1;
        ub_i = 0;
        lb_i = 0;

        case (state_r)
            ST_WAIT: begin
                if (ready_o) state_x = ST_READ;
            end

            ST_IDLE: begin
                state_x = ST_IDLE;
            end

            ST_READ: begin
                if (!done_o) begin
                    rw_i = 1;
                    addr_i = 24'b000000000000011000000001;
                end else begin
                    state_x = ST_WRITE;
                end
            end

            ST_WRITE: begin
                if (!done_o) begin
                    rw_i = 1;
                    we_i = 0;
                    addr_i = 24'b000000000000011000000001;
                    data_i = 16'hADCD;
                    ub_i = 1;
                    lb_i = 0;
                end else begin
                    state_x = ST_REFRESH;
                end
            end

            ST_REFRESH: begin
                if (!done_o) begin
                    refresh_i = 1;
                end else begin
                    state_x = ST_IDLE;
                end
            end

            // default: begin
            //     state_x = ST_IDLE;
            // end
        endcase
    end

    // Reset stimulus process
    initial begin
        // Hold reset state for 20 ns
        reset_i = 1;
        #20;
        reset_i = 0;
        #1000; // Run simulation for 1000 ns
        $finish; // Stop the simulation
    end

	initial
	begin
	$dumpfile("waveforms.vcd");
	$dumpvars(0,dut);
	end

endmodule
