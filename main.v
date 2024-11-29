// Released under the 3-Clause BSD License:

// Copyright 2010-2019 Matthew Hagerty (matthew <at> dnotq <dot> io)

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.

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

module sdram_simple (
	// Host Side
	input wire clk_100m0_i,    // Master clock
	input wire reset_i,        // Reset, active high
	input wire refresh_i,      // Initiate a refresh cycle, active high
	input wire rw_i,           // Initiate a read or write operation, active high
	input wire we_i,           // Write enable, active low
	input wire [23:0] addr_i,  // Address from host to SDRAM
	input wire [15:0] data_i,  // Data from host to SDRAM
	input wire ub_i,           // Data upper byte enable, active low
	input wire lb_i,           // Data lower byte enable, active low
	output reg ready_o,       // Set to '1' when the memory is ready
	output reg done_o,        // Read, write, or refresh, operation is done
	output reg [15:0] data_o, // Data from SDRAM to host

	// SDRAM Side
	output reg sdCke_o,         // Clock-enable to SDRAM
	output reg sdCe_bo,	        // Chip-select to SDRAM
	output reg sdRas_bo,	    // SDRAM row address strobe
	output reg sdCas_bo,	    // SDRAM column address strobe
	output reg sdWe_bo,	        // SDRAM write enable
	output reg [1:0] sdBs_bo,   // SDRAM bank address
	output reg [12:0] sdAddr_o, // SDRAM row/column address
	inout wire [15:0] sdData_io, // Data to/from SDRAM
	output reg sdDqmh_o,	    // Enable upper-byte of SDRAM databus if true
	output reg sdDqml_o	    // Enable lower-byte of SDRAM databus if true
);

	// SDRAM controller states
    parameter ST_INIT_WAIT      = 4'b0000,
			  ST_INIT_PRECHARGE = 4'b0001,
			  ST_INIT_REFRESH1  = 4'b0010,
			  ST_INIT_MODE      = 4'b0011,
			  ST_INIT_REFRESH2  = 4'b0100,
			  ST_IDLE           = 4'b0101,
			  ST_REFRESH        = 4'b0110,
			  ST_ACTIVATE       = 4'b0111,
			  ST_RCD            = 4'b1000,
			  ST_RW             = 4'b1001,
			  ST_RAS1           = 4'b1010,
			  ST_RAS2           = 4'b1011,
			  ST_PRECHARGE      = 4'b1100;

    reg [3:0] state_r = ST_INIT_WAIT, state_x = ST_INIT_WAIT;

    // SDRAM mode register data
    localparam [12:0] MODE_REG = 13'b000_0_00_010_0_000; // Stays constant

    // SDRAM commands
	parameter CMD_ACTIVATE  = 4'b0011,
			  CMD_PRECHARGE = 4'b0010,
			  CMD_WRITE     = 4'b0100,
			  CMD_READ      = 4'b0101,
			  CMD_MODE      = 4'b0000,
		 	  CMD_NOP       = 4'b0111,
		 	  CMD_REFRESH   = 4'b0001;

    reg [3:0] cmd_r, cmd_x;

    // Internal signals
	reg [1:0] bank_s;
	reg [12:0] row_s;
	reg [8:0] col_s;
	reg [12:0] addr_r;
	reg [12:0] addr_x;
	reg [15:0] sd_dout_r;
	reg [15:0] sd_dout_x;
	reg sd_busdir_r;
	reg sd_busdir_x;

	reg [15:0] timer_r, timer_x; // 0 to 20000
	reg [3:0] refcnt_r, refcnt_x; // 0 to 8

	reg [1:0] bank_r, bank_x;
	reg cke_r, cke_x;
	reg sd_dqmu_r, sd_dqmu_x;
	reg sd_dqml_r, sd_dqml_x;
	reg ready_r, ready_x;

	// Data buffer for SDRAM to Host
	reg [15:0] buf_dout_r, buf_dout_x;

	// All signals to SDRAM buffered

	assign {sdCe_bo, sdRas_bo, sdCas_bo, sdWe_bo} = cmd_r;   // SDRAM operation control bits
	assign sdCke_o     = cke_r;      // SDRAM clock enable
	assign sdBs_o      = bank_r;     // SDRAM bank address
	assign sdAddr_o    = addr_r;     // SDRAM address
	assign sdData_io   = sd_dout_r when sd_busdir_r = '1' else (others => 'Z');   // SDRAM data bus.
	assign sdDqmh_o    = sd_dqmu_r;  // SDRAM high data byte enable, active low
	assign sdDqml_o    = sd_dqml_r;  // SDRAM low date byte enable, active low

	// Signals back to host.
	assign ready_o = ready_r;
	assign data_o = buf_dout_r;

	// 23  22  | 21 20 19 18 17 16 15 14 13 12 11 10 09 | 08 07 06 05 04 03 02 01 00 |
	// BS0 BS1 |        ROW (A12-A0)  8192 rows         |   COL (A8-A0)  512 cols    |
	assign bank_s = addr_i[23:22];
	assign row_s = addr_i[21:9];
	assign col_s = addr_i[8:0];

always @ (state_r, timer_r, refcnt_r, cke_r, addr_r, sd_dout_r, sd_busdir_r, sd_dqmu_r, sd_dqml_r, ready_r,
    bank_s, row_s, col_s,
    rw_i, refresh_i, addr_i, data_i, we_i, ub_i, lb_i,
    buf_dout_r, sdData_io)
begin
	state_x     = state_r;       // Stay in the same state unless changed.
	timer_x     = timer_r;       // Hold the cycle timer by default.
	refcnt_x    = refcnt_r;      // Hold the refresh timer by default.
	cke_x       = cke_r;         // Stay in the same clock mode unless changed.
	cmd_x       = CMD_NOP;       // Default to NOP unless changed.
	bank_x      = bank_r;        // Register the SDRAM bank.
	addr_x      = addr_r;        // Register the SDRAM address.
	sd_dout_x   = sd_dout_r;     // Register the SDRAM write data.
	sd_busdir_x = sd_busdir_r;   // Register the SDRAM bus tristate control.
	sd_dqmu_x   = sd_dqmu_r;
	sd_dqml_x   = sd_dqml_r;
	buf_dout_x  = buf_dout_r;    // SDRAM to host data buffer.

	ready_x     = ready_r;       // Always ready unless performing initialization.
	done_o      = 1'b0;           // Done tick, single cycle.

	if (timer_r != 0) begin
			timer_x = timer_r - 1;  // Decrement the cycle timer.
	end else begin
		cke_x       = 1'b1;
		bank_x      = bank_s;
		// A10 low for rd/wr commands to suppress auto-precharge.
		addr_x      = {4'b0000, col_s};
		sd_dqmu_x   = 1'b0;
		sd_dqml_x   = 1'b0;

		case (state_r)
			ST_INIT_WAIT: begin
				// 1. Wait 200us with DQM signals high, cmd NOP.
				// 2. Precharge all banks.
				// 3. Eight refresh cycles.
				// 4. Set mode register.
				// 5. Eight refresh cycles.

				state_x = ST_INIT_PRECHARGE;
				timer_x = 20000;      // 200us wait
				// timer_x = 2;          // for simulation
				sd_dqmu_x = 1'b1;
				sd_dqml_x = 1'b1;
			end
			ST_INIT_PRECHARGE: begin
				state_x = ST_INIT_REFRESH1;
				refcnt_x = 8;         // 8 refresh cycles
				// refcnt_r = 2;		 // for simulation
				cmd_x = CMD_PRECHARGE;
				timer_x = 2;          // Trp = 20ns
				bank_x = 2'b00;
				addr_x[10] = 1'b1;    // Precharge all banks
			end
			ST_INIT_REFRESH1: begin
				if (refcnt_r == 0) begin
					state_x = ST_INIT_MODE;
				end else begin
					refcnt_x = refcnt_r - 1;
					cmd_x = CMD_REFRESH;
					timer_x = 7;      // 70ns refresh
				end
			end
			ST_INIT_MODE: begin
                state_x = ST_INIT_REFRESH2;
                refcnt_x = 8;         // 8 refresh cycles
                bank_x = 2'b00;
                addr_x = MODE_REG;
                cmd_x = CMD_MODE;
                timer_x = 2;          // Trsc = 2 cycles
            end
            ST_INIT_REFRESH2: begin
                if (refcnt_r == 0) begin
                    state_x = ST_IDLE;
                    ready_x = 1'b1;
                end else begin
                    refcnt_x = refcnt_r - 1;
                    cmd_x = CMD_REFRESH;
                    timer_x = 7;      // 70ns refresh
                end
            end
			ST_IDLE: begin
				// Handle idle state behavior
				if (rw_i == 1'b1) begin
					state_x = ST_ACTIVATE;  // Transition to ACTIVATE state
					cmd_x = CMD_ACTIVATE;   // Send ACTIVATE command
					addr_x = row_s;         // Set bank select and row address
				end else if (refresh_i == 1'b1) begin
					state_x = ST_REFRESH;   // Transition to REFRESH state
					cmd_x = CMD_REFRESH;    // Send REFRESH command
					timer_x = 7;            // Wait 7 cycles for refresh
				end
			end

			ST_REFRESH: begin
				state_x = ST_IDLE;          // Transition back to IDLE state
				done_o = 1'b1;              // Indicate operation completion
			end

			ST_ACTIVATE: begin
				// Prepare SDRAM for subsequent operations
				state_x = ST_RCD;           // Transition to RCD (Row to Column delay) state
				sd_dout_x = data_i;         // Register any input data for writing
			end

			ST_RCD: begin
				// Handle Row to Column delay
				state_x = ST_RW;            // Transition to READ/WRITE state
				if (we_i == 1'b0) begin
					cmd_x = CMD_WRITE;      // Issue WRITE command
					sd_busdir_x = 1'b1;     // Enable data bus for writing
					sd_dqmu_x = ub_i;       // Set upper byte mask
					sd_dqml_x = lb_i;       // Set lower byte mask
				end else begin
					cmd_x = CMD_READ;       // Issue READ command
				end
			end

			ST_RW: begin
				// Execute read or write operation
				state_x = ST_RAS1;          // Transition to RAS1 (Row Access Strobe) state
				sd_busdir_x = 1'b0;         // Disable data bus output
			end

			ST_RAS1: begin
				// Register data from SDRAM
				state_x = ST_RAS2;          // Transition to RAS2 state
				buf_dout_x = sdData_io;     // Capture data from SDRAM into output buffer
			end

			ST_RAS2: begin
				// Handle precharge preparation
				state_x = ST_PRECHARGE;     // Transition to PRECHARGE state
				cmd_x = CMD_PRECHARGE;      // Issue PRECHARGE command
				addr_x[10] = 1'b1;          // Set precharge all banks flag
			end

			ST_PRECHARGE: begin
				// Complete precharge operation
				state_x = ST_IDLE;          // Transition back to IDLE state
				done_o = 1'b1;              // Indicate operation completion
				timer_x = 1;                // Buffer to ensure host readiness before IDLE
			end
		endcase
	end
end

always @(posedge clk_100m0_i) begin
    if (reset_i) begin
        state_r     <= ST_INIT_WAIT;
        timer_r     <= 0;
        cmd_r       <= CMD_NOP;
        cke_r       <= 1'b0;
        ready_r     <= 1'b0;
    end else begin
        state_r     <= state_x;
        timer_r     <= timer_x;
        refcnt_r    <= refcnt_x;
        cke_r       <= cke_x;         // CKE to SDRAM.
        cmd_r       <= cmd_x;         // Command to SDRAM.
        bank_r      <= bank_x;        // Bank to SDRAM.
        addr_r      <= addr_x;        // Address to SDRAM.
        sd_dout_r   <= sd_dout_x;     // Data to SDRAM.
        sd_busdir_r <= sd_busdir_x;   // SDRAM bus direction.
        sd_dqmu_r   <= sd_dqmu_x;     // Upper byte enable to SDRAM.
        sd_dqml_r   <= sd_dqml_x;     // Lower byte enable to SDRAM.
        ready_r     <= ready_x;
        buf_dout_r  <= buf_dout_x;
    end
end

endmodule