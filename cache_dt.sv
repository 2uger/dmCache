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
