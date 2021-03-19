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

             output logic [31:0] o_cpu_data,
             output logic o_cpu_ready,
             
             // Memory
             input logic [127:0] i_mem_data,
             input logic i_mem_r_ready,
             input logic i_mem_w_ready,

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
    logic [2:0] next_state;

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
    logic [31:0] readdata_buf;

    logic data_writeenable;

    // Signal for choosing right data for data storage
    logic [0:2] wd_m1;
    logic [0:2] wd_m2;
    logic [0:2] wd_m3;
    logic [0:2] wd_m4;



    assign state = next_state;

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
        // Describe all combinational logic
        writedata_buf[127:96] = (wd_m1 == 2'b00) ? i_cpu_data : 
                                (wd_m1 == 2'b01) ? i_mem_data[127:96] : readdata_buf[127:96]; 

        writedata_buf[95:64] = (wd_m2 == 2'b00) ? i_cpu_data : 
                               (wd_m2 == 2'b01) ? i_mem_data[95:64] : readdata_buf[95:64]; 

        writedata_buf[63:32] = (wd_m3 == 2'b00) ? i_cpu_data : 
                               (wd_m1 == 2'b01) ? i_mem_data[63:32] : readdata_buf[63:32]; 

        writedata_buf[31:0] = (wd_m1 == 2'b00) ? i_cpu_data : 
                              (wd_m1 == 2'b01) ? i_mem_data[31:0] : readdata_buf[31:0]; 


        case (i_cpu_addr[3:2])
            2'b00: o_cpu_data <= readdata_buf[31:0];
            2'b01: o_cpu_data <= readdata_buf[63:32];
            2'b10: o_cpu_data <= readdata_buf[95:64];
            2'b11: o_cpu_data <= readdata_buf[127:96];
        endcase

        o_mem_addr = i_cpu_addr;

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
                            next_state <= COMPARE
                        end 
                    end
                    COMPARE: begin
                        if (hit) begin
                            o_cpu_ready <= 1'b1;
                            if (i_cpu_write) begin
                                i_valid <= 1'b1;
                                // When we write we always set dirty bit
                                i_dirty <= 1'b1;
                                data_writeenable <= 1'b1;
                            end
                            next_state <= IDLE;
                        end else begin
                                // Not hit => 
                                // If not valid and not dirty 
                                // just read new info to that tag line
                                // else because of writeback policy 
                                // we should writeback old information
                                if (!valid && !modify) begin
                                    o_mem_read <= 1;
                                    state <= FETCH1;
                                end else if (valid && modify) begin
                                        o_mem_write <= 1;
                                        state <= WRITEBACK;
                                    end
                            end
                    end
                    FETCH: begin
                        // Waiting for data from memory
                        if (i_mem_ready) begin
                            if (i_cpu_write) begin
                                // Check
                                case (i_cpu_addr[3:2])
                                    2'b00: wd_m1 <= 2'b00;
                                    2'b01: wd_m2 <= 2'b00;
                                    2'b10: wd_m3 <= 2'b00;
                                    2'b11: wd_m4 <= 2'b00;
                                endcase
                                i_dirty <= 1;
                            end else begin
                                    wd_m1 <= 2'b01;
                                    wd_m2 <= 2'b01;
                                    wd_m3 <= 2'b01;
                                    wd_m4 <= 2'b01;
                                end
                            i_valid <= 1;
                            i_dirty <= 0;
                            tag_writeenable <= 1;
                            data_writeenable <= 1;
                            o_mem_read <= 0;
                            next_state <= IDLE;
                        end
                    end
                    WRITEBACK: begin
                        if (i_mem_w_ready) begin
                            o_mem_read <= 1;
                            o_mem_write <= 0;
                            next_state <= FETCH;
                        end
                    end
            end
    end
endmodule
