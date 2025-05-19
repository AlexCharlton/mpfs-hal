// Assertion helper macro
`define ASSERT_EQ(actual, expected) \
    assert (actual === expected) \
    else begin \
        $error("%s = %h, expected %h", `"actual`", actual, expected); \
        $stop; \
    end
