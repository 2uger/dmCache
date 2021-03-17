/*
 * Direct mapped cache with 128b block
 * Controller use write back policy, that's why
 * it use dirty bits when processor want to 
 * write data(we dont write it back immediately)
 */

module cache(input logic clk,
             input logic reset,
             // Cpu
             input logic [31:0] i_cpu_addr,
             input logic [31:0] i_cpu_data,
             input logic i_cpu_read,
             input logic i_cpu_write,

             output logic [31:0] o_cpu_readdata,
             output logic o_cpu_ready,
             
             // Memory
             input logic [31:0] i_mem_data,
             input logic i_mem_ready,

             output logic o_mem_read,
             output logic o_mem_write,
             output logic [31:0] o_mem_addr,
             output logic [127:0] o_mem_writedata);
    
    // FSM states
    localparam IDLE = 0;
    localparam COMPARE = 1;
    localparam ALLOCATE = 2;
    localparam WRITE_BACK = 3;

    logic [2:0] state;

    // Tag storage signals
    logic i_valid;
    logic i_dirty;
    logic tag_writeenable;
    logic hit;
    logic miss;
    logic valid;
    logic modify;

    // Data storage signals
    logic [127:0] writedata_buf;
    logic [31:0] write_addr_buf;
    logic [31:0] readdata_buf;

    logic data_writeenable;

    cache_tag_store ts(.clk(clk),
                       .reset(reset),
                       .index(i_cpu_addr[13:4]),
                       .i_valid(i_valid),
                       .i_dirty(i_dirty),
                       .i_tag(i_cpu_addr[31:14]),
                       .writeenable(tag_writeenable),
                       .o_hit(hit),
                       .o_miss(miss),
                       .o_valid(valid),
                       .o_modify(modify));

    cache_data_store ds(.clk(clk),
                        .reset(reset),
                        .index(i_cpu_addr[13:4]),
                        .writedata(writedata_buf),
                        .writeenable(data_writeenable),
                        .readdata(readdata_buf));

    always_comb begin
        // Fill writedata_buf by mux


        case (i_cpu_addr[3:2])
            2'b00: o_cpu_readdata <= readdata_buf[31:0];
            2'b01: o_cpu_readdata <= readdata_buf[63:32];
            2'b10: o_cpu_readdata <= readdata_buf[95:64];
            2'b11: o_cpu_readdata <= readdata_buf[127:96];
        endcase
    end

    always_ff @(posedge clk) begin
        if(reset) begin
            o_cpu_ready <= 0;
            o_mem_read <= 0;
            o_mem_write <= 0;
            o_mem_addr <= 0;
            tag_writeenable <= 0;
            data_writeenable <= 0;
            state <= IDLE;
        end else begin
                case (state)
                    IDLE: begin
                        if (i_cpu_read || i_cpu_write) begin
                            state <= COMPARE
                        end 
                    end
                    COMPARE: begin
                        if (hit) begin
                            o_cpu_ready <= 1'b1;
                            if (i_cpu_write) begin
                                i_valid <= 1'b1;
                                i_dirty <= 1'b1;
                                tag_writeenable <= 1'b1;
                                data_writeenable <= 1'b1;
                            end
                            state <= IDLE;
                        end else begin
                                // Not hit => 
                                // If not valid and not dirty 
                                // just read new info to that tag line
                                // else because of writeback policy 
                                // we should writeback old information
                                tag_writeenable <= 1'b1;
                                i_tag <= 1;
                                if (!valid && !modify) begin
                                    o_mem_read <= 1;
                                    state <= ALLOCATE;
                                end else begin
                                        o_mem_write <= 1;
                                        state <= WRITEBACK;
                                    end
                            end
                    end
                    ALLOCATE: begin

                    end
                    WRITEBACK: begin
                    end
            end
    end
endmodule

module cache_data_store(input logic clk,
                        input logic reset,
                        input logic [9:0] index,
                        input logic [127:0] writedata
                        input logic writeenable,
                        output logic [127:0] readata);
    
    logic [127:0] data_store [1023:0];

    initial begin
        for (int i = 0; i < 1024; i++)
            data_store[i] = 0;
    end

    assign readdata = data_store[index];

    always_ff @(posedge clk) begin
        if (writeenable) begin
            data_store[index] <= writedata;
        end
    end
endmodule
