package cache_dt;
    typedef struct {
        logic valid;
        logic dirty;
        logic [31:14] tag;
    } cache_tag_entry;
endpackage
