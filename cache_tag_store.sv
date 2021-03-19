module cache_tag_store(input logic clk,
                       input logic reset,
                       input logic [9:0] index,

                       input logic i_valid,
                       input logic i_dirty,
                       input logic [17:0] i_tag,
                       input logic writeenable,

                       output logic hit,
                       output logic miss
                       output logic valid,
                       output logic modify);
                   
    logic [19:0] tag_store [1023:0];

    initial begin
        for (int i = 0; i < 1024; i++)
            tag_store[i] = 0;
    end

    logic [17:0] o_tag;
    logic o_valid;
    logic o_dirty;

    assign o_tag = tag_store[index][17:0];
    assign o_valid = tag_store[index][18:17];
    assign o_dirty = tag_store[index][19:18];

    assign hit = (o_tag == i_tag) && o_valid == 1'b1;
    assign modify = o_valid && o_dirty;
    assign miss = !o_valid || ((o_tag != i_tag && !o_dirty));

    always_ff @(posedge clk) begin
        if (!reset && writeenable) begin
            tag_store[index][17:0] <= i_tag;
            tag_store[index][18:17] <= i_valid;
            tag_store[index][19:18] <= i_dirty;
        end
    end
endmodule
