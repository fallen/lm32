/*
 * LatticeMico32
 * Data Translation Lookaside Buffer
 *
 * Copyright (c) 2011-2012 Yann Sionneau <yann.sionneau@gmail.com>
 * Copyright (c) 2012 Michael Walle <michael@walle.cc>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

`include "lm32_include.v"

`ifdef CFG_MMU_ENABLED

`ifdef CFG_MMU_WITH_ASID
`define LM32_DTLB_STATE_WIDTH               4
`else
`define LM32_DTLB_STATE_WIDTH               2
`endif
`define LM32_DTLB_STATE_RNG                 (`LM32_DTLB_STATE_WIDTH-1):0
`define LM32_DTLB_STATE_CHECK               `LM32_DTLB_STATE_WIDTH'h1
`define LM32_DTLB_STATE_FLUSH               `LM32_DTLB_STATE_WIDTH'h2
`ifdef CFG_MMU_WITH_ASID
`define LM32_DTLB_STATE_ASID_FLUSH_READ     `LM32_DTLB_STATE_WIDTH'h3
`define LM32_DTLB_STATE_ASID_FLUSH_WRITE    `LM32_DTLB_STATE_WIDTH'h4
`endif

`define LM32_DTLB_OFFSET_RNG                offset_msb:offset_lsb
`define LM32_DTLB_IDX_RNG                   index_msb:index_lsb
`define LM32_DTLB_VPFN_RNG                  vpfn_msb:vpfn_lsb
`define LM32_DTLB_TAG_RNG                   tag_msb:tag_lsb
`define LM32_DTLB_ADDR_RNG                  (index_width-1):0
// CFG_MMU_ASID_WIDTH == 0 if CFG_MMU_WITH_ASID == `FALSE
`define LM32_DTLB_DATA_WIDTH                (`CFG_MMU_ASID_WIDTH + vpfn_width + tag_width + 3)  // +3 for valid, ci, ro
`define LM32_DTLB_DATA_RNG                  (`LM32_DTLB_DATA_WIDTH-1):0

`ifdef CFG_MMU_WITH_ASID
`define LM32_DTLB_ASID_RNG                  asid_msb:asid_lsb
`endif

/////////////////////////////////////////////////////
// Module interface
/////////////////////////////////////////////////////

module lm32_dtlb (
    // ----- Inputs -------
    clk_i,
    rst_i,
    enable,
    stall_x,
    stall_m,
    address_x,
    address_m,
    load_d,
    store_d,
    load_q_x,
    store_q_x,
    load_q_m,
    store_q_m,
    tlbpaddr,
    tlbvaddr,
    update,
    flush,
`ifdef CFG_MMU_WITH_ASID
    asid_flush,
    current_asid,
`endif
    invalidate,
    // ----- Outputs -----
    physical_load_store_address_m,
    stall_request,
    miss_vfn,
    miss_x,
    fault_x,
    cache_inhibit_x
    );

/////////////////////////////////////////////////////
// Parameters
/////////////////////////////////////////////////////

parameter entries = 1024;               // Number of entries in DTLB
parameter page_size = 4096;             // DTLB page size

localparam offset_width = `CLOG2(page_size);
localparam index_width = `CLOG2(entries);
localparam offset_lsb = 0;
localparam offset_msb = (offset_lsb+offset_width-1);
`ifdef CFG_MMU_WITH_ASID
localparam asid_lsb = 7;
localparam asid_msb = asid_lsb + `CFG_MMU_ASID_WIDTH - 1;
`endif
localparam index_lsb = (offset_msb+1);
localparam index_msb = (index_lsb+index_width-1);
localparam tag_lsb = (index_msb+1);
localparam tag_msb = (`LM32_WORD_WIDTH-1);
localparam tag_width = (tag_msb-tag_lsb+1);
localparam vpfn_lsb = (offset_msb+1);
localparam vpfn_msb = (`LM32_WORD_WIDTH-1);
localparam vpfn_width = (vpfn_msb-vpfn_lsb+1);

/////////////////////////////////////////////////////
// Inputs
/////////////////////////////////////////////////////

input clk_i;                            // Clock
input rst_i;                            // Reset

input enable;                           // Data TLB enable

input stall_x;                          // Stall X stage
input stall_m;                          // Stall M stage

input [`LM32_WORD_RNG] address_x;       // X stage load/store address
input [`LM32_WORD_RNG] address_m;       // M stage load/store address
input load_d;                           // Load instruction in D stage
input store_d;                          // Store instruction in D stage
input load_q_x;                         // Load instruction in X stage
input store_q_x;                        // Store instruction in X stage
input load_q_m;                         // Load instruction in M stage
input store_q_m;                        // Store instruction in M stage

input [`LM32_WORD_RNG] tlbpaddr;
input [`LM32_WORD_RNG] tlbvaddr;
input update;
input flush;
`ifdef CFG_MMU_WITH_ASID
input asid_flush;
input [(`CFG_MMU_ASID_WIDTH-1):0] current_asid;
`endif
input invalidate;

/////////////////////////////////////////////////////
// Outputs
/////////////////////////////////////////////////////

output [`LM32_WORD_RNG] physical_load_store_address_m;
wire   [`LM32_WORD_RNG] physical_load_store_address_m;
output stall_request;
wire   stall_request;
output [`LM32_WORD_RNG] miss_vfn;
wire   [`LM32_WORD_RNG] miss_vfn;
output miss_x;
wire   miss_x;
output fault_x;
wire   fault_x;
output cache_inhibit_x;
wire   cache_inhibit_x;

/////////////////////////////////////////////////////
// Internal nets and registers
/////////////////////////////////////////////////////

wire [`LM32_DTLB_ADDR_RNG] read_address;
wire [`LM32_DTLB_ADDR_RNG] write_address;
wire [`LM32_DTLB_DATA_RNG] write_data;
wire [`LM32_DTLB_DATA_RNG] tlbe;
wire [`LM32_DTLB_DATA_RNG] tlbe_inval;
wire [`LM32_DTLB_TAG_RNG] tlbe_tag_x;
wire [`LM32_DTLB_VPFN_RNG] tlbe_pfn_x;
`ifdef CFG_MMU_WITH_ASID
wire [`LM32_DTLB_ASID_RNG] tlbe_asid_x;
wire [`LM32_DTLB_ASID_RNG] asid_to_flush;
`endif
wire tlbe_valid_x;
wire tlbe_ro_x;
wire tlbe_ci_x;
wire checking;
wire entire_tlb_flushing;
wire write_port_enable;
wire asid_flushing;

reg [`LM32_DTLB_STATE_RNG] state;                         // Current state of FSM
reg [`LM32_DTLB_ADDR_RNG] flush_set;
reg [`LM32_DTLB_VPFN_RNG] tlbe_pfn_m;
reg lookup;
`ifdef CFG_MMU_WITH_ASID
reg asid_flush_we;
`endif

/////////////////////////////////////////////////////
// Functions
/////////////////////////////////////////////////////

////////////////////////////////////////////////////
// Instantiations
/////////////////////////////////////////////////////

lm32_ram
  #(
    // ----- Parameters -------
    .data_width (`LM32_DTLB_DATA_WIDTH),
    .address_width (index_width)
// Modified for Milkymist: removed non-portable RAM parameters
    ) data_ram
    (
     // ----- Inputs -------
     .read_clk (clk_i),
     .write_clk (clk_i),
     .reset (rst_i),
     .read_address (read_address),
     .enable_read (lookup),
     .write_address (write_address),
     .enable_write (`TRUE),
     .write_enable (write_port_enable),
     .write_data (write_data),
     // ----- Outputs -------
     .read_data (
        {
`ifdef CFG_MMU_WITH_ASID
          tlbe_asid_x,
`endif
          tlbe_pfn_x, tlbe_tag_x, tlbe_ci_x, tlbe_ro_x, tlbe_valid_x
        }
     )
     );

/////////////////////////////////////////////////////
// Combinational logic
/////////////////////////////////////////////////////

// Use this trick to prevent from
// putting ifdef all over the place

assign asid_flushing =
`ifdef CFG_MMU_WITH_ASID
(state == `LM32_DTLB_STATE_ASID_FLUSH_READ) || (state == `LM32_DTLB_STATE_ASID_FLUSH_WRITE);
`else
0;
`endif

// Compute address to use to index into the DTLB data memory
assign read_address = address_x[`LM32_DTLB_IDX_RNG];

// tlb_update_address will receive data from a CSR register
assign write_address = ((entire_tlb_flushing == `TRUE) || (asid_flushing))
                            ? flush_set
                            : tlbvaddr[`LM32_DTLB_IDX_RNG];

assign write_port_enable = (update == `TRUE) || (invalidate == `TRUE) || (entire_tlb_flushing == `TRUE)
`ifdef CFG_MMU_WITH_ASID
                           || ((asid_flushing == `TRUE) && (asid_flush_we == `TRUE))
`endif
                          ;

assign physical_load_store_address_m = (enable == `FALSE)
                ? address_m
                : {tlbe_pfn_m, address_m[`LM32_DTLB_OFFSET_RNG]};

assign tlbe = {
`ifdef CFG_MMU_WITH_ASID
        tlbvaddr[`LM32_DTLB_ASID_RNG],     // ASID
`endif
        tlbpaddr[`LM32_DTLB_VPFN_RNG],     // pfn
        tlbvaddr[`LM32_DTLB_TAG_RNG],      // tag
        tlbpaddr[2],                       // cache inhibit
        tlbpaddr[1],                       // read only
        `TRUE};                            // valid
assign tlbe_inval = {{`LM32_DTLB_DATA_WIDTH-1{1'b0}}, `FALSE};
assign write_data = (invalidate == `TRUE) || (entire_tlb_flushing == `TRUE) || (asid_flushing == `TRUE) ? tlbe_inval : tlbe;


`ifdef CFG_MMU_WITH_ASID
assign asid_to_flush = tlbvaddr[`LM32_DTLB_ASID_RNG];
assign tlbe_match = ({tlbe_asid_x, tlbe_tag_x, tlbe_valid_x} == {current_asid, address_x[`LM32_DTLB_TAG_RNG], `TRUE});
`else
assign tlbe_match = ({tlbe_tag_x, tlbe_valid_x} == {address_x[`LM32_DTLB_TAG_RNG], `TRUE});
`endif

assign miss_vfn = {address_x[`LM32_DTLB_VPFN_RNG], {offset_width{1'b0}}};
assign miss_x = ((enable == `TRUE) && ((load_q_x == `TRUE) || (store_q_x == `TRUE)) && (tlbe_match == `FALSE) && (lookup == `FALSE));
assign cache_inhibit_x = ((enable == `TRUE) && (tlbe_ci_x == `TRUE));
assign fault_x = ((enable == `TRUE) && (store_q_x == `TRUE) && (tlbe_match == `TRUE) && (tlbe_ro_x == `TRUE));

assign checking = state[0];
assign entire_tlb_flushing = state[1];
assign stall_request = (entire_tlb_flushing == `TRUE) || (lookup == `TRUE) || (asid_flushing == `TRUE);

/////////////////////////////////////////////////////
// Sequential logic
/////////////////////////////////////////////////////

// Lookup logic
always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
    if (rst_i == `TRUE)
        lookup <= `FALSE;
    else
    begin
        if ((enable == `TRUE) && (stall_x == `FALSE) && ((load_d == `TRUE) || (store_d == `TRUE)))
            lookup <= `TRUE;
        else
            lookup <= `FALSE;
    end
end

// X/M stage registers
always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
    if (rst_i == `TRUE)
        tlbe_pfn_m <= {vpfn_width{1'bx}};
    else if (stall_m == `FALSE)
        tlbe_pfn_m <= tlbe_pfn_x;
end

always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
    if (rst_i == `TRUE)
    begin
        flush_set <= {index_width{1'b1}};
        state <= `LM32_DTLB_STATE_FLUSH;
`ifdef CFG_MMU_WITH_ASID
        asid_flush_we <= `FALSE;
`endif
    end
    else
    begin
        case (state)

        `LM32_DTLB_STATE_CHECK:
        begin
            if (flush == `TRUE) begin
                flush_set <= {index_width{1'b1}};
                state <= `LM32_DTLB_STATE_FLUSH;
            end
`ifdef CFG_MMU_WITH_ASID
            else if (asid_flush == `TRUE)
            begin
                flush_set <= {index_width{1'b1}};
                state <= `LM32_DTLB_STATE_ASID_FLUSH_READ;
            end
`endif
        end

        `LM32_DTLB_STATE_FLUSH:
        begin
            if (flush_set == {index_width{1'b0}})
                state <= `LM32_DTLB_STATE_CHECK;
            flush_set <= flush_set - 1'b1;
        end

`ifdef CFG_MMU_WITH_ASID
        `LM32_DTLB_STATE_ASID_FLUSH_READ:
        begin
            if (tlbe_asid_x == asid_to_flush)
                asid_flush_we <= 1;
            state <= `LM32_DTLB_STATE_ASID_FLUSH_WRITE;
        end

        `LM32_DTLB_STATE_ASID_FLUSH_WRITE:
        begin
            if (flush_set == {index_width{1'b0}})
            begin
                state <= `LM32_DTLB_STATE_CHECK;
            end
            else
            begin
                flush_set <= flush_set - 1'b1;
                state <= `LM32_DTLB_STATE_ASID_FLUSH_READ;
            end
            asid_flush_we <= 0;
        end
`endif

        endcase
    end
end

endmodule

`endif

